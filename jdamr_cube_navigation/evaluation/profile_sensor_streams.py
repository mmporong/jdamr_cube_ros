#!/usr/bin/env python3
"""Profile sensor values and timing from one recorded SLAM dataset."""

from __future__ import annotations

import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402,I100
import numpy as np  # noqa: E402,I201
import yaml  # noqa: E402,I201

from compare_slam_runs import path_length, yaw_of  # noqa: E402,I100,I201
from corridor_run_media import parse_route_log  # noqa: E402,I100,I201


SENSOR_TOPICS = ('/scan', '/odom', '/imu/data_raw', '/tf')


def _sha256(path: Path) -> str:
    """Return the SHA-256 digest of a file."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _home_relative(path: Path) -> str:
    """Render a path relative to the current home directory when possible."""
    try:
        return f'$HOME/{path.resolve().relative_to(Path.home())}'
    except ValueError:
        return str(path.resolve())


def _stamp_ns(header: Any) -> int:
    """Return a ROS header stamp in nanoseconds."""
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def distribution(samples: Any, digits: int = 6) -> dict[str, Any]:
    """Return finite-sample descriptive statistics without a unit claim."""
    values = np.asarray(samples, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {'samples': 0}
    percentiles = np.percentile(values, [1, 5, 50, 95, 99])
    return {
        'samples': int(values.size),
        'min': round(float(np.min(values)), digits),
        'p01': round(float(percentiles[0]), digits),
        'p05': round(float(percentiles[1]), digits),
        'p50': round(float(percentiles[2]), digits),
        'p95': round(float(percentiles[3]), digits),
        'p99': round(float(percentiles[4]), digits),
        'max': round(float(np.max(values)), digits),
        'mean': round(float(np.mean(values)), digits),
        'stddev': round(float(np.std(values)), digits),
    }


def robust_sigma(samples: Any) -> float | None:
    """Return a normal-equivalent sigma estimated from median deviation."""
    values = np.asarray(samples, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return None
    median = np.median(values)
    return float(np.median(np.abs(values - median)) * 1.482602218505602)


def _plain_data(value: Any) -> Any:
    """Convert NumPy scalars into JSON/YAML-safe Python values."""
    if isinstance(value, dict):
        return {key: _plain_data(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain_data(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def timing_profile(recorder_stamps_ns: list[int],
                   header_stamps_ns: list[int]) -> tuple[
                       dict[str, Any], dict[str, Any]]:
    """Describe message periods, jitter, and recorder latency."""
    if len(recorder_stamps_ns) != len(header_stamps_ns):
        raise ValueError('recorder/header stamp counts differ')
    if not recorder_stamps_ns:
        return {'messages': 0}, {
            'recorder_interval_s': [],
            'header_interval_s': [],
            'header_jitter_s': [],
            'recorder_latency_s': [],
        }

    recorder_ns = np.asarray(recorder_stamps_ns, dtype=np.int64)
    header_ns = np.asarray(header_stamps_ns, dtype=np.int64)
    recorder_interval_s = np.diff(recorder_ns) / 1e9
    header_interval_s = np.diff(header_ns) / 1e9
    positive_header_interval_s = header_interval_s[header_interval_s > 0.0]
    if positive_header_interval_s.size:
        median_period_s = float(np.median(positive_header_interval_s))
        header_jitter_s = np.abs(
            positive_header_interval_s - median_period_s)
    else:
        median_period_s = None
        header_jitter_s = np.asarray([], dtype=np.float64)
    recorder_latency_s = (recorder_ns - header_ns) / 1e9
    elapsed_s = (recorder_ns[-1] - recorder_ns[0]) / 1e9
    rate_hz = ((len(recorder_ns) - 1) / elapsed_s
               if elapsed_s > 0.0 and len(recorder_ns) > 1 else None)

    result = {
        'messages': len(recorder_stamps_ns),
        'observed_rate_hz': (
            round(float(rate_hz), 3) if rate_hz is not None else None),
        'header_non_increasing_samples': int(
            np.count_nonzero(header_interval_s <= 0.0)),
        'recorder_before_header_samples': int(
            np.count_nonzero(recorder_latency_s < 0.0)),
        'recorder_interval_s': distribution(recorder_interval_s),
        'header_interval_s': distribution(positive_header_interval_s),
        'header_abs_jitter_from_median_s': distribution(header_jitter_s),
        'recorder_minus_header_s': distribution(recorder_latency_s),
    }
    series = {
        'recorder_interval_s': recorder_interval_s,
        'header_interval_s': positive_header_interval_s,
        'header_jitter_s': header_jitter_s,
        'recorder_latency_s': recorder_latency_s,
    }
    return result, series


def nearest_offsets_s(reference_stamps_ns: list[int],
                      target_stamps_ns: list[int]) -> np.ndarray:
    """Return signed nearest-neighbour header offsets, target minus source."""
    targets_ns = sorted(target_stamps_ns)
    offsets_s = []
    for reference_ns in reference_stamps_ns:
        index = bisect_left(targets_ns, reference_ns)
        candidates_ns = targets_ns[max(0, index - 1):index + 1]
        if not candidates_ns:
            continue
        target_ns = min(candidates_ns, key=lambda value: abs(
            value - reference_ns))
        offsets_s.append((target_ns - reference_ns) / 1e9)
    return np.asarray(offsets_s, dtype=np.float64)


def summarize_scan(scan_frames: list[dict[str, Any]]) -> tuple[
        dict[str, Any], dict[str, Any]]:
    """Summarize LaserScan validity, range, and scan timing values."""
    valid_ranges_m = []
    valid_fraction = []
    point_counts = []
    scan_times_s = []
    time_increments_s = []
    valid_points = 0
    nan_points = 0
    positive_infinity_points = 0
    negative_infinity_points = 0
    below_min_points = 0
    above_max_points = 0
    for frame in scan_frames:
        ranges_m = np.asarray(frame['ranges_m'], dtype=np.float64)
        range_min_m = float(frame['range_min_m'])
        range_max_m = float(frame['range_max_m'])
        finite = np.isfinite(ranges_m)
        below = finite & (ranges_m < range_min_m)
        above = finite & (ranges_m > range_max_m)
        valid = finite & ~below & ~above
        count = int(ranges_m.size)
        point_counts.append(count)
        valid_count = int(np.count_nonzero(valid))
        valid_points += valid_count
        nan_points += int(np.count_nonzero(np.isnan(ranges_m)))
        positive_infinity_points += int(np.count_nonzero(np.isposinf(ranges_m)))
        negative_infinity_points += int(np.count_nonzero(np.isneginf(ranges_m)))
        below_min_points += int(np.count_nonzero(below))
        above_max_points += int(np.count_nonzero(above))
        valid_fraction.append(valid_count / count if count else 0.0)
        valid_ranges_m.extend(ranges_m[valid].tolist())
        scan_times_s.append(float(frame['scan_time_s']))
        time_increments_s.append(float(frame['time_increment_s']))
    total_points = sum(point_counts)
    invalid_points = total_points - valid_points
    nonfinite_points = (
        nan_points + positive_infinity_points + negative_infinity_points)
    result = {
        'frames': len(scan_frames),
        'points_total': total_points,
        'points_valid': valid_points,
        'points_invalid': invalid_points,
        'invalid_reason_counts': {
            'nan': nan_points,
            'positive_infinity': positive_infinity_points,
            'negative_infinity': negative_infinity_points,
            'below_range_min': below_min_points,
            'above_range_max': above_max_points,
        },
        'valid_fraction': round(
            valid_points / total_points, 6) if total_points else None,
        'nonfinite_fraction': round(
            nonfinite_points / total_points, 6) if total_points else None,
        'points_per_frame': distribution(point_counts, digits=3),
        'valid_fraction_per_frame': distribution(valid_fraction),
        'valid_range_m': distribution(valid_ranges_m, digits=4),
        'declared_scan_time_s': distribution(scan_times_s),
        'declared_time_increment_s': distribution(time_increments_s, digits=9),
    }
    series = {
        'valid_range_m': np.asarray(valid_ranges_m, dtype=np.float64),
        'valid_fraction': np.asarray(valid_fraction, dtype=np.float64),
    }
    return result, series


def _wrapped_delta_rad(current_rad: np.ndarray,
                       previous_rad: np.ndarray) -> np.ndarray:
    """Return signed planar angle differences in the closed principal range."""
    return np.arctan2(np.sin(current_rad - previous_rad),
                      np.cos(current_rad - previous_rad))


def summarize_odometry(samples: list[dict[str, Any]], *,
                       stationary_linear_mps: float,
                       stationary_angular_radps: float) -> tuple[
                           dict[str, Any], dict[str, Any]]:
    """Summarize wheel-odometry motion and classify stationary samples."""
    ordered = sorted(samples, key=lambda sample: sample['header_stamp_ns'])
    if not ordered:
        return {'samples': 0}, {
            'header_stamps_ns': [], 'stationary': [],
            'translation_increment_m': np.asarray([]),
            'abs_yaw_increment_rad': np.asarray([]),
        }
    x_m = np.asarray([sample['x_m'] for sample in ordered])
    y_m = np.asarray([sample['y_m'] for sample in ordered])
    yaw_rad = np.asarray([sample['yaw_rad'] for sample in ordered])
    linear_mps = np.asarray([sample['linear_mps'] for sample in ordered])
    angular_radps = np.asarray([
        sample['angular_radps'] for sample in ordered])
    translation_increment_m = np.hypot(np.diff(x_m), np.diff(y_m))
    yaw_increment_rad = _wrapped_delta_rad(yaw_rad[1:], yaw_rad[:-1])
    stationary = ((np.abs(linear_mps) <= stationary_linear_mps)
                  & (np.abs(angular_radps) <= stationary_angular_radps))
    increment_stationary = stationary[1:] & stationary[:-1]
    trajectory = [
        (sample['header_stamp_ns'] / 1e9, sample['x_m'], sample['y_m'],
         sample['yaw_rad'])
        for sample in ordered
    ]
    start_to_end_m = math.hypot(x_m[-1] - x_m[0], y_m[-1] - y_m[0])
    result = {
        'samples': len(ordered),
        'stationary_classification': {
            'linear_speed_threshold_mps': stationary_linear_mps,
            'angular_speed_threshold_radps': stationary_angular_radps,
            'stationary_samples': int(np.count_nonzero(stationary)),
            'moving_samples': int(np.count_nonzero(~stationary)),
        },
        'translation_increment_m': distribution(
            translation_increment_m, digits=6),
        'abs_yaw_increment_rad': distribution(
            np.abs(yaw_increment_rad), digits=7),
        'linear_velocity_mps': distribution(linear_mps, digits=5),
        'angular_velocity_radps': distribution(angular_radps, digits=5),
        'stationary_translation_increment_m': distribution(
            translation_increment_m[increment_stationary], digits=7),
        'stationary_abs_yaw_increment_rad': distribution(
            np.abs(yaw_increment_rad[increment_stationary]), digits=8),
        'decimated_path_length_m': round(path_length(trajectory), 3),
        'start_to_end_m': round(float(start_to_end_m), 4),
    }
    series = {
        'header_stamps_ns': [sample['header_stamp_ns'] for sample in ordered],
        'stationary': stationary.tolist(),
        'translation_increment_m': translation_increment_m,
        'abs_yaw_increment_rad': np.abs(yaw_increment_rad),
    }
    return result, series


def summarize_imu(samples: list[dict[str, Any]],
                  odom_series: dict[str, Any], *,
                  max_alignment_s: float = 0.05) -> tuple[
                      dict[str, Any], dict[str, Any]]:
    """Summarize IMU angular velocity in stationary and moving segments."""
    ordered = sorted(samples, key=lambda sample: sample['header_stamp_ns'])
    x_radps = np.asarray([sample['x_radps'] for sample in ordered])
    y_radps = np.asarray([sample['y_radps'] for sample in ordered])
    z_radps = np.asarray([sample['z_radps'] for sample in ordered])
    norm_radps = np.sqrt(x_radps ** 2 + y_radps ** 2 + z_radps ** 2)
    odom_stamps_ns = odom_series['header_stamps_ns']
    odom_stationary = odom_series['stationary']
    motion_classes: list[bool | None] = []
    alignment_offsets_s = []
    for sample in ordered:
        if not odom_stamps_ns:
            motion_classes.append(None)
            continue
        index = bisect_left(odom_stamps_ns, sample['header_stamp_ns'])
        candidates = range(
            max(0, index - 1), min(len(odom_stamps_ns), index + 1))
        nearest_index = min(
            candidates,
            key=lambda candidate: abs(
                odom_stamps_ns[candidate] - sample['header_stamp_ns']))
        offset_s = abs(
            odom_stamps_ns[nearest_index] - sample['header_stamp_ns']) / 1e9
        if offset_s > max_alignment_s:
            motion_classes.append(None)
            continue
        alignment_offsets_s.append(offset_s)
        motion_classes.append(bool(odom_stationary[nearest_index]))
    stationary_mask = np.asarray(
        [value is True for value in motion_classes], dtype=bool)
    moving_mask = np.asarray(
        [value is False for value in motion_classes], dtype=bool)
    classified_mask = stationary_mask | moving_mask
    result = {
        'samples': len(ordered),
        'odom_motion_alignment': {
            'max_allowed_offset_s': max_alignment_s,
            'classified_samples': int(np.count_nonzero(classified_mask)),
            'unclassified_samples': int(np.count_nonzero(~classified_mask)),
            'absolute_offset_s': distribution(alignment_offsets_s),
        },
        'angular_velocity_x_radps': distribution(x_radps, digits=6),
        'angular_velocity_y_radps': distribution(y_radps, digits=6),
        'angular_velocity_z_radps': distribution(z_radps, digits=6),
        'angular_velocity_norm_radps': distribution(norm_radps, digits=6),
        'stationary': {
            'samples': int(np.count_nonzero(stationary_mask)),
            'angular_velocity_z_radps': distribution(
                z_radps[stationary_mask], digits=7),
            'angular_velocity_norm_radps': distribution(
                norm_radps[stationary_mask], digits=7),
        },
        'moving': {
            'samples': int(np.count_nonzero(moving_mask)),
            'angular_velocity_z_radps': distribution(
                z_radps[moving_mask], digits=6),
            'angular_velocity_norm_radps': distribution(
                norm_radps[moving_mask], digits=6),
        },
    }
    series = {
        'stationary_z_radps': z_radps[stationary_mask],
        'moving_z_radps': z_radps[moving_mask],
    }
    return result, series


def _resolve_mcap(source: Path) -> Path:
    """Resolve exactly one MCAP file from a bag directory or file."""
    source = source.expanduser().resolve()
    if source.is_file() and source.suffix == '.mcap':
        return source
    bag_files = sorted(source.glob('*.mcap')) if source.is_dir() else []
    if len(bag_files) != 1:
        raise ValueError(f'expected one MCAP under {source}, got {len(bag_files)}')
    return bag_files[0]


def _analysis_window(bag: Path, route_log: Path | None) -> dict[str, Any]:
    """Return the successful route window when its log is available."""
    if route_log is None:
        candidate = bag.parent.parent / f'{bag.parent.name}.route.log'
        route_log = candidate if candidate.is_file() else None
    if route_log is None:
        return {'kind': 'full_bag', 'start_unix_ns': None,
                'end_unix_ns': None, 'route_log': None}
    route_log = route_log.expanduser().resolve()
    route = parse_route_log(route_log.read_text(encoding='utf-8'))
    if not route['success']:
        raise ValueError(f'route log has no success event: {route_log}')
    return {
        'kind': 'successful_route',
        'start_unix_ns': route['start_stamp_ns'],
        'end_unix_ns': route['end_stamp_ns'],
        'route_log': _home_relative(route_log),
        'route_log_sha256': _sha256(route_log),
    }


def _odometry_sample(message: Any) -> dict[str, Any]:
    """Extract one odometry sample using explicit SI-unit field names."""
    pose = message.ros_msg.pose.pose
    twist = message.ros_msg.twist.twist
    return {
        'header_stamp_ns': _stamp_ns(message.ros_msg.header),
        'x_m': pose.position.x,
        'y_m': pose.position.y,
        'yaw_rad': yaw_of(pose.orientation),
        'linear_mps': twist.linear.x,
        'angular_radps': twist.angular.z,
    }


def _imu_sample(message: Any) -> dict[str, Any]:
    """Extract one IMU angular-velocity sample in radians per second."""
    angular = message.ros_msg.angular_velocity
    return {
        'header_stamp_ns': _stamp_ns(message.ros_msg.header),
        'x_radps': angular.x,
        'y_radps': angular.y,
        'z_radps': angular.z,
    }


def analyse_bag(source: Path, *, route_log: Path | None = None,
                stationary_linear_mps: float = 0.01,
                stationary_angular_radps: float = 0.02,
                stationary_prelude_s: float = 60.0,
                imu_odom_max_alignment_s: float = 0.05) -> tuple[
                    dict[str, Any], dict[str, Any]]:
    """Read one MCAP and return the measured profile and plotting series."""
    from mcap_ros2.reader import read_ros2_messages

    bag = _resolve_mcap(source)
    analysis_window = _analysis_window(bag, route_log)
    timing_stamps: dict[str, dict[str, list[int]]] = {
        topic: {'recorder': [], 'header': []}
        for topic in ('scan', 'odom', 'imu', 'tf_odom_base', 'tf_map_odom')
    }
    scan_frames = []
    odom_samples = []
    imu_samples = []
    prelude_odom_samples = []
    prelude_imu_samples = []
    capture_start_ns = None
    capture_end_ns = None
    for message in read_ros2_messages(str(bag), topics=list(SENSOR_TOPICS)):
        recorder_stamp_ns = int(message.log_time_ns)
        capture_start_ns = (recorder_stamp_ns if capture_start_ns is None
                            else min(capture_start_ns, recorder_stamp_ns))
        capture_end_ns = (recorder_stamp_ns if capture_end_ns is None
                          else max(capture_end_ns, recorder_stamp_ns))
        window_start_ns = analysis_window['start_unix_ns']
        window_end_ns = analysis_window['end_unix_ns']
        prelude_start_ns = (
            window_start_ns - round(stationary_prelude_s * 1e9)
            if analysis_window['kind'] == 'successful_route' else None)
        in_prelude = (prelude_start_ns is not None
                      and prelude_start_ns <= recorder_stamp_ns
                      < window_start_ns)
        topic = message.channel.topic
        if in_prelude and topic == '/odom':
            prelude_odom_samples.append(_odometry_sample(message))
        elif in_prelude and topic == '/imu/data_raw':
            prelude_imu_samples.append(_imu_sample(message))
        if (window_start_ns is not None
                and not window_start_ns <= recorder_stamp_ns <= window_end_ns):
            continue
        if topic == '/scan':
            header_stamp_ns = _stamp_ns(message.ros_msg.header)
            timing_stamps['scan']['recorder'].append(recorder_stamp_ns)
            timing_stamps['scan']['header'].append(header_stamp_ns)
            scan_frames.append({
                'ranges_m': message.ros_msg.ranges,
                'range_min_m': message.ros_msg.range_min,
                'range_max_m': message.ros_msg.range_max,
                'scan_time_s': message.ros_msg.scan_time,
                'time_increment_s': message.ros_msg.time_increment,
            })
        elif topic == '/odom':
            header_stamp_ns = _stamp_ns(message.ros_msg.header)
            timing_stamps['odom']['recorder'].append(recorder_stamp_ns)
            timing_stamps['odom']['header'].append(header_stamp_ns)
            odom_samples.append(_odometry_sample(message))
        elif topic == '/imu/data_raw':
            header_stamp_ns = _stamp_ns(message.ros_msg.header)
            timing_stamps['imu']['recorder'].append(recorder_stamp_ns)
            timing_stamps['imu']['header'].append(header_stamp_ns)
            imu_samples.append(_imu_sample(message))
        elif topic == '/tf':
            for transform in message.ros_msg.transforms:
                parent = transform.header.frame_id.lstrip('/')
                child = transform.child_frame_id.lstrip('/')
                timing_key = None
                if parent == 'odom' and child == 'base_footprint':
                    timing_key = 'tf_odom_base'
                elif parent == 'map' and child == 'odom':
                    timing_key = 'tf_map_odom'
                if timing_key is not None:
                    timing_stamps[timing_key]['recorder'].append(
                        recorder_stamp_ns)
                    timing_stamps[timing_key]['header'].append(
                        _stamp_ns(transform.header))

    timing = {}
    timing_series = {}
    for name, stamps in timing_stamps.items():
        timing[name], timing_series[name] = timing_profile(
            stamps['recorder'], stamps['header'])
    cross_pairs = {
        'scan_to_odom': ('scan', 'odom'),
        'scan_to_imu': ('scan', 'imu'),
        'scan_to_odom_base_tf': ('scan', 'tf_odom_base'),
        'scan_to_map_odom_tf': ('scan', 'tf_map_odom'),
    }
    cross_stream = {}
    cross_series = {}
    for name, (reference, target) in cross_pairs.items():
        offsets_s = nearest_offsets_s(
            timing_stamps[reference]['header'],
            timing_stamps[target]['header'])
        cross_stream[name] = {
            'signed_nearest_header_offset_s': distribution(offsets_s),
            'absolute_nearest_header_offset_s': distribution(
                np.abs(offsets_s)),
        }
        cross_series[name] = offsets_s

    scan, scan_series = summarize_scan(scan_frames)
    odom, odom_series = summarize_odometry(
        odom_samples,
        stationary_linear_mps=stationary_linear_mps,
        stationary_angular_radps=stationary_angular_radps)
    imu, imu_series = summarize_imu(
        imu_samples, odom_series,
        max_alignment_s=imu_odom_max_alignment_s)
    if prelude_odom_samples and prelude_imu_samples:
        noise_odom, noise_odom_series = summarize_odometry(
            prelude_odom_samples,
            stationary_linear_mps=stationary_linear_mps,
            stationary_angular_radps=stationary_angular_radps)
        noise_imu, noise_imu_series = summarize_imu(
            prelude_imu_samples, noise_odom_series,
            max_alignment_s=imu_odom_max_alignment_s)
        stationary_window = {
            'kind': 'pre_route_prelude',
            'requested_duration_s': stationary_prelude_s,
            'start_unix_ns': analysis_window['start_unix_ns']
            - round(stationary_prelude_s * 1e9),
            'end_unix_ns': analysis_window['start_unix_ns'],
        }
    else:
        noise_odom = odom
        noise_imu = imu
        noise_imu_series = imu_series
        stationary_window = {
            'kind': 'analysis_window_fallback',
            'requested_duration_s': None,
            'start_unix_ns': analysis_window['start_unix_ns'],
            'end_unix_ns': analysis_window['end_unix_ns'],
        }
    capture_duration_s = ((capture_end_ns - capture_start_ns) / 1e9
                          if capture_start_ns is not None else 0.0)
    if analysis_window['start_unix_ns'] is None:
        analysis_window['start_unix_ns'] = capture_start_ns
        analysis_window['end_unix_ns'] = capture_end_ns
    analysis_window['duration_s'] = round(
        (analysis_window['end_unix_ns']
         - analysis_window['start_unix_ns']) / 1e9, 3)
    stationary_imu = noise_imu.get('stationary', {})
    stationary_imu_z = stationary_imu.get(
        'angular_velocity_z_radps', {})
    stationary_imu_sigma_radps = robust_sigma(
        noise_imu_series['stationary_z_radps'])
    report = {
        'schema_version': 1,
        'run_id': bag.parent.name,
        'source': {
            'mcap': _home_relative(bag),
            'mcap_sha256': _sha256(bag),
        },
        'capture': {
            'start_unix_ns': capture_start_ns,
            'end_unix_ns': capture_end_ns,
            'duration_s': round(capture_duration_s, 3),
        },
        'analysis_window': analysis_window,
        'timing': timing,
        'cross_stream_alignment': cross_stream,
        'scan': scan,
        'wheel_odometry': odom,
        'imu': imu,
        'stationary_noise_profile': {
            'window': stationary_window,
            'wheel_odometry': noise_odom,
            'imu': noise_imu,
        },
        'simulation_initial_candidates': {
            'measured_not_tuned': True,
            'scan_nonfinite_ray_fraction': scan['nonfinite_fraction'],
            'scan_period_s': timing['scan'].get(
                'header_interval_s', {}).get('p50'),
            'scan_period_abs_jitter_p99_s': timing['scan'].get(
                'header_abs_jitter_from_median_s', {}).get('p99'),
            'scan_recorder_minus_header_p95_s': timing['scan'].get(
                'recorder_minus_header_s', {}).get('p95'),
            'imu_stationary_z_bias_radps': stationary_imu_z.get('p50'),
            'imu_stationary_z_robust_sigma_radps': (
                round(stationary_imu_sigma_radps, 7)
                if stationary_imu_sigma_radps is not None else None),
            'odom_stationary_translation_increment_p99_m': noise_odom.get(
                'stationary_translation_increment_m', {}).get('p99'),
            'odom_stationary_abs_yaw_increment_p99_rad': noise_odom.get(
                'stationary_abs_yaw_increment_rad', {}).get('p99'),
        },
        'limitations': [
            'The real-robot bag has no external ground truth, so additive '
            'LiDAR range noise and odometry accuracy are not identifiable.',
            'Recorder latency includes driver, scheduling, DDS, and recorder '
            'effects; it is not a sensor-only latency measurement.',
            'Non-finite LiDAR rays are no-return observations, not evidence '
            'of transport-level message dropout.',
            'The map-to-odom TF header is future-dated by localization '
            'tolerance, so recorder-minus-header is not latency for that TF.',
            'Stationary segments are inferred from wheel-odometry twist '
            'thresholds and are not an independently labelled experiment.',
            'Simulation candidates are observed initial envelopes, not tuned '
            'or validated sim-to-real parameters.',
        ],
    }
    series = {
        'timing': timing_series,
        'cross_stream': cross_series,
        'scan': scan_series,
        'odom': odom_series,
        'imu': imu_series,
        'stationary_noise': noise_imu_series,
    }
    return _plain_data(report), series


def _configure_plot_font() -> None:
    """Use an installed Korean font when matplotlib can resolve it."""
    from matplotlib import font_manager
    font_path = Path(
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font_path.is_file():
        family = font_manager.FontProperties(fname=str(font_path)).get_name()
        plt.rcParams['font.family'] = family
    plt.rcParams['axes.unicode_minus'] = False


def render_sensor_profile(report: dict[str, Any], series: dict[str, Any],
                          output: Path) -> None:
    """Render sensor-value distributions for portfolio and review."""
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), dpi=160)
    scan_ranges_m = series['scan']['valid_range_m']
    axes[0, 0].hist(scan_ranges_m, bins=80, color='#2563eb', alpha=0.85)
    axes[0, 0].set_title('LiDAR valid range distribution')
    axes[0, 0].set_xlabel('range [m]')
    axes[0, 0].set_ylabel('ray count')

    valid_pct = series['scan']['valid_fraction'] * 100.0
    axes[0, 1].hist(valid_pct, bins=50, color='#0f766e', alpha=0.85)
    axes[0, 1].set_title('LiDAR valid rays per scan')
    axes[0, 1].set_xlabel('valid rays [%]')
    axes[0, 1].set_ylabel('scan count')

    translation_mm = series['odom']['translation_increment_m'] * 1000.0
    axes[1, 0].hist(translation_mm, bins=80, color='#7c3aed', alpha=0.85)
    axes[1, 0].set_title('Wheel-odometry translation increments')
    axes[1, 0].set_xlabel('increment [mm]')
    axes[1, 0].set_ylabel('sample count')

    stationary_z = series['stationary_noise']['stationary_z_radps']
    moving_z = series['imu']['moving_z_radps']
    combined_z = np.concatenate((stationary_z, moving_z))
    lower_radps, upper_radps = np.percentile(combined_z, [1, 99])
    if stationary_z.size:
        visible = stationary_z[(stationary_z >= lower_radps)
                               & (stationary_z <= upper_radps)]
        axes[1, 1].hist(visible, bins=70, alpha=0.75,
                        label='stationary', color='#16a34a')
    if moving_z.size:
        visible = moving_z[(moving_z >= lower_radps)
                           & (moving_z <= upper_radps)]
        axes[1, 1].hist(visible, bins=70, alpha=0.50,
                        label='moving', color='#f59e0b')
    axes[1, 1].set_title('IMU yaw-rate distribution (p01–p99 view)')
    axes[1, 1].set_xlabel('angular velocity z [rad/s]')
    axes[1, 1].set_ylabel('sample count')
    axes[1, 1].legend()
    for axis in axes.flat:
        axis.grid(alpha=0.2)
    figure.suptitle(f"JD-AMR sensor profile — {report['run_id']}")
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, bbox_inches='tight')
    plt.close(figure)


def render_timing_profile(report: dict[str, Any], series: dict[str, Any],
                          output: Path) -> None:
    """Render stream jitter, latency, and nearest timestamp alignment."""
    timing = series['timing']
    names = ['scan', 'odom', 'imu', 'tf_odom_base', 'tf_map_odom']
    labels = ['scan', 'odom', 'IMU', 'odom→base TF', 'map→odom TF']
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), dpi=160)
    jitter_ms = [timing[name]['header_jitter_s'] * 1000.0
                 for name in names]
    axes[0, 0].boxplot(jitter_ms, labels=labels, showfliers=False)
    axes[0, 0].set_title('Header-period absolute jitter')
    axes[0, 0].set_ylabel('jitter [ms]')
    axes[0, 0].tick_params(axis='x', rotation=20)

    latency_ms = [timing[name]['recorder_latency_s'] * 1000.0
                  for name in names]
    axes[0, 1].boxplot(latency_ms, labels=labels, showfliers=False)
    axes[0, 1].set_title('Recorder timestamp minus header timestamp')
    axes[0, 1].set_ylabel('signed offset [ms]')
    axes[0, 1].set_yscale('symlog', linthresh=5.0)
    axes[0, 1].tick_params(axis='x', rotation=20)

    cross_names = list(series['cross_stream'])
    cross_labels = [name.replace('_to_', '→').replace('_', ' ')
                    for name in cross_names]
    offsets_ms = [np.abs(series['cross_stream'][name]) * 1000.0
                  for name in cross_names]
    axes[1, 0].boxplot(offsets_ms, labels=cross_labels, showfliers=False)
    axes[1, 0].set_title('Nearest header timestamp alignment')
    axes[1, 0].set_ylabel('absolute offset [ms]')
    axes[1, 0].tick_params(axis='x', rotation=20)

    p99_gap_ms = [report['timing'][name]['recorder_interval_s'].get(
        'p99', 0.0) * 1000.0 for name in names]
    max_gap_ms = [report['timing'][name]['recorder_interval_s'].get(
        'max', 0.0) * 1000.0 for name in names]
    positions = np.arange(len(names))
    width = 0.36
    axes[1, 1].bar(positions - width / 2, p99_gap_ms, width,
                   label='p99', color='#0f766e')
    axes[1, 1].bar(positions + width / 2, max_gap_ms, width,
                   label='max', color='#dc2626')
    axes[1, 1].set_xticks(positions, labels, rotation=20)
    axes[1, 1].set_title('Recorder-time message gaps')
    axes[1, 1].set_ylabel('gap [ms]')
    axes[1, 1].legend()
    for axis in axes.flat:
        axis.grid(alpha=0.2)
    figure.suptitle(f"JD-AMR timing profile — {report['run_id']}")
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, bbox_inches='tight')
    plt.close(figure)


def render_markdown(report: dict[str, Any]) -> str:
    """Return a concise evidence report suitable for repository review."""
    timing = report['timing']
    scan = report['scan']
    odom = report['wheel_odometry']
    noise_profile = report['stationary_noise_profile']
    noise_imu_stationary = noise_profile['imu']['stationary'][
        'angular_velocity_z_radps']
    lines = [
        f"# 센서·시간 프로파일 — `{report['run_id']}`",
        '',
        '## 입력과 범위',
        '',
        f"- MCAP: `{report['source']['mcap']}`",
        f"- SHA-256: `{report['source']['mcap_sha256']}`",
        f"- 전체 캡처: {report['capture']['duration_s']:.3f}초",
        f"- 분석 구간: {report['analysis_window']['kind']} / "
        f"{report['analysis_window']['duration_s']:.3f}초",
        f"- 정적 노이즈 구간: {noise_profile['window']['kind']} / "
        f"{noise_profile['window']['requested_duration_s']}초",
        '- 아래 수치는 실물 주행 MCAP의 관측값이며 외부 ground truth가 아니다.',
        '',
        '## 스트림 시간 특성',
        '',
        '| 스트림 | 메시지 | 관측 Hz | header 주기 p50 | header jitter p99 | recorder−header p95 | 최대 gap |',
        '|---|---:|---:|---:|---:|---:|---:|',
    ]
    labels = {
        'scan': 'LiDAR', 'odom': 'wheel odom', 'imu': 'IMU',
        'tf_odom_base': 'odom→base TF', 'tf_map_odom': 'map→odom TF',
    }
    for key, label in labels.items():
        item = timing[key]
        lines.append(
            f"| {label} | {item['messages']:,} | "
            f"{item['observed_rate_hz']} | "
            f"{item['header_interval_s'].get('p50')}초 | "
            f"{item['header_abs_jitter_from_median_s'].get('p99')}초 | "
            f"{item['recorder_minus_header_s'].get('p95')}초 | "
            f"{item['recorder_interval_s'].get('max')}초 |")
    lines.extend([
        '',
        '## 센서 관측',
        '',
        f"- LiDAR: {scan['frames']:,} frames, "
        f"{scan['points_total']:,} rays 중 "
        f"{scan['valid_fraction'] * 100.0:.2f}%가 선언 범위 안의 유효값이었다.",
        f"- LiDAR 유효 거리: p05 {scan['valid_range_m'].get('p05')}m, "
        f"p50 {scan['valid_range_m'].get('p50')}m, "
        f"p95 {scan['valid_range_m'].get('p95')}m.",
        f"- wheel odom: decimated path {odom['decimated_path_length_m']}m, "
        f"start-to-end {odom['start_to_end_m']}m.",
        f'- 주행 구간에서 odom이 정지로 분류한 IMU 표본: '
        f"{report['imu']['stationary']['samples']:,}개.",
        f'- 주행 직전 정적 구간 IMU z축: median '
        f"{noise_imu_stationary.get('p50')}rad/s, robust sigma "
        f"{report['simulation_initial_candidates']['imu_stationary_z_robust_sigma_radps']}rad/s.",
        '',
        '## 시뮬레이션 초기값 후보',
        '',
        '이 값들은 실측 분포의 재현 시작점이다. 튜닝 완료값이나 센서 고유 사양으로 취급하지 않는다.',
        '',
        '```yaml',
        yaml.safe_dump(report['simulation_initial_candidates'],
                       allow_unicode=True, sort_keys=False).rstrip(),
        '```',
        '',
        '## 해석 한계',
        '',
    ])
    lines.extend(f'- {limitation}' for limitation in report['limitations'])
    lines.append('')
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    """Generate reusable profile data, plots, and an evidence report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--route-log', type=Path, default=None)
    parser.add_argument('--stationary-linear-mps', type=float, default=0.01)
    parser.add_argument('--stationary-angular-radps', type=float, default=0.02)
    parser.add_argument('--stationary-prelude-s', type=float, default=60.0)
    parser.add_argument('--imu-odom-max-alignment-s', type=float, default=0.05)
    args = parser.parse_args(argv)
    if args.stationary_linear_mps < 0.0:
        parser.error('--stationary-linear-mps must be non-negative')
    if args.stationary_angular_radps < 0.0:
        parser.error('--stationary-angular-radps must be non-negative')
    if args.stationary_prelude_s <= 0.0:
        parser.error('--stationary-prelude-s must be positive')
    if args.imu_odom_max_alignment_s <= 0.0:
        parser.error('--imu-odom-max-alignment-s must be positive')

    _configure_plot_font()
    report, series = analyse_bag(
        args.bag,
        route_log=args.route_log,
        stationary_linear_mps=args.stationary_linear_mps,
        stationary_angular_radps=args.stationary_angular_radps,
        stationary_prelude_s=args.stationary_prelude_s,
        imu_odom_max_alignment_s=args.imu_odom_max_alignment_s)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    yaml_path = args.output_dir / 'sensor_profile.yaml'
    json_path = args.output_dir / 'sensor_profile.json'
    markdown_path = args.output_dir / 'sensor_profile.md'
    sensor_plot_path = args.output_dir / 'sensor_profile.png'
    timing_plot_path = args.output_dir / 'timing_profile.png'
    yaml_path.write_text(
        yaml.safe_dump(report, allow_unicode=True, sort_keys=False),
        encoding='utf-8')
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    markdown_path.write_text(render_markdown(report), encoding='utf-8')
    render_sensor_profile(report, series, sensor_plot_path)
    render_timing_profile(report, series, timing_plot_path)
    outputs = [yaml_path, json_path, markdown_path, sensor_plot_path,
               timing_plot_path]
    print(json.dumps({
        'run_id': report['run_id'],
        'source_mcap_sha256': report['source']['mcap_sha256'],
        'outputs': [str(path.resolve()) for path in outputs],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
