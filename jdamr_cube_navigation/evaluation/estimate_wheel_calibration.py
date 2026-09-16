#!/usr/bin/env python3
"""
Estimate wheel-odometry calibration candidates from an AMCL run.

AMCL is a saved-map localization reference, not external ground truth. The
result is therefore a candidate for a short measured drive, not a parameter
that may be promoted directly to the physical robot.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class PoseSample:
    """One timestamped planar pose."""

    stamp_ns: int
    x_m: float
    y_m: float
    yaw_rad: float


def wrapped_delta(start_rad: float, end_rad: float) -> float:
    """Return the shortest signed angular displacement."""
    return math.atan2(
        math.sin(end_rad - start_rad), math.cos(end_rad - start_rad))


def quaternion_yaw(quaternion: Any) -> float:
    """Return planar yaw from a ROS quaternion-like object."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z))


def pair_nearest(reference: list[PoseSample], odometry: list[PoseSample],
                 maximum_gap_s: float) -> list[tuple[PoseSample, PoseSample]]:
    """Pair reference samples to the closest odometry sample in time."""
    if not reference or not odometry:
        return []
    stamps = [sample.stamp_ns for sample in odometry]
    maximum_gap_ns = int(maximum_gap_s * 1e9)
    pairs = []
    for sample in reference:
        index = bisect_left(stamps, sample.stamp_ns)
        candidates = range(max(0, index - 1), min(len(stamps), index + 1))
        nearest = min(
            candidates,
            key=lambda candidate: abs(stamps[candidate] - sample.stamp_ns),
            default=None)
        if nearest is None:
            continue
        if abs(stamps[nearest] - sample.stamp_ns) <= maximum_gap_ns:
            pairs.append((sample, odometry[nearest]))
    return pairs


def estimate_candidates(
        paired: list[tuple[PoseSample, PoseSample]], current_radius_m: float,
        current_separation_m: float, current_radius_ratio: float = 1.0,
        minimum_translation_m: float = 0.08,
        maximum_straight_yaw_rad: float = 0.04,
        minimum_turn_yaw_rad: float = 0.08) -> dict[str, Any]:
    """Estimate radius, left/right ratio and a turn-based separation check."""
    positive_inputs = (
        current_radius_m, current_separation_m, current_radius_ratio,
        minimum_translation_m, maximum_straight_yaw_rad,
        minimum_turn_yaw_rad)
    if not all(math.isfinite(value) and value > 0.0
               for value in positive_inputs):
        raise ValueError('calibration dimensions and thresholds must be finite and positive')
    straight_reference_m = 0.0
    straight_odometry_m = 0.0
    straight_yaw_error_rad = 0.0
    straight_intervals = 0
    turn_reference_sq = 0.0
    turn_cross = 0.0
    turn_intervals = 0

    for (reference_a, odometry_a), (reference_b, odometry_b) in zip(
            paired, paired[1:]):
        reference_m = math.hypot(
            reference_b.x_m - reference_a.x_m,
            reference_b.y_m - reference_a.y_m)
        odometry_m = math.hypot(
            odometry_b.x_m - odometry_a.x_m,
            odometry_b.y_m - odometry_a.y_m)
        reference_yaw = wrapped_delta(
            reference_a.yaw_rad, reference_b.yaw_rad)
        odometry_yaw = wrapped_delta(
            odometry_a.yaw_rad, odometry_b.yaw_rad)

        if (reference_m >= minimum_translation_m
                and odometry_m >= minimum_translation_m
                and abs(reference_yaw) <= maximum_straight_yaw_rad):
            straight_reference_m += reference_m
            straight_odometry_m += odometry_m
            straight_yaw_error_rad += odometry_yaw - reference_yaw
            straight_intervals += 1

        if (abs(reference_yaw) >= minimum_turn_yaw_rad
                and reference_yaw * odometry_yaw > 0.0):
            turn_reference_sq += reference_yaw * reference_yaw
            turn_cross += reference_yaw * odometry_yaw
            turn_intervals += 1

    if straight_intervals < 3 or straight_reference_m <= 0.0:
        raise ValueError('insufficient straight AMCL/odometry intervals')
    if turn_intervals < 3 or turn_reference_sq <= 0.0:
        raise ValueError('insufficient turning AMCL/odometry intervals')

    radius_scale = straight_reference_m / straight_odometry_m
    heading_drift_rad_per_m = (
        straight_yaw_error_rad / straight_reference_m)
    radius_ratio = (
        current_radius_ratio
        - heading_drift_rad_per_m * current_separation_m)
    turn_yaw_scale = turn_cross / turn_reference_sq
    turn_separation_m = (
        current_separation_m * radius_scale * turn_yaw_scale)

    return {
        'paired_samples': len(paired),
        'straight_intervals': straight_intervals,
        'turn_intervals': turn_intervals,
        'straight_reference_distance_m': round(straight_reference_m, 6),
        'straight_odometry_distance_m': round(straight_odometry_m, 6),
        'radius_scale': round(radius_scale, 8),
        'heading_drift_rad_per_m': round(heading_drift_rad_per_m, 8),
        'turn_yaw_scale': round(turn_yaw_scale, 8),
        'candidate': {
            'wheel_radius_m': round(current_radius_m * radius_scale, 6),
            'wheel_radius_ratio_right_over_left': round(radius_ratio, 6),
            'turn_based_wheel_separation_m': round(turn_separation_m, 6),
        },
    }


def read_mcap(path: Path) -> tuple[list[PoseSample], list[PoseSample]]:
    """Read odometry and AMCL poses from one ROS 2 MCAP."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    odometry = []
    reference = []
    with path.open('rb') as stream:
        reader = make_reader(
            stream, decoder_factories=[DecoderFactory()])
        for _, channel, _, decoded in reader.iter_decoded_messages(
                topics=['/odom', '/amcl_pose']):
            pose = decoded.pose.pose
            stamp = decoded.header.stamp
            sample = PoseSample(
                stamp_ns=(
                    int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)),
                x_m=float(pose.position.x),
                y_m=float(pose.position.y),
                yaw_rad=quaternion_yaw(pose.orientation),
            )
            if channel.topic == '/odom':
                odometry.append(sample)
            else:
                reference.append(sample)
    odometry.sort(key=lambda sample: sample.stamp_ns)
    reference.sort(key=lambda sample: sample.stamp_ns)
    return odometry, reference


def sha256(path: Path) -> str:
    """Return a file SHA-256 digest."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def portable_path(path: Path) -> str:
    """Render artifact paths through the portable home-level symlink."""
    artifact_root = (Path.home() / 'jdamr_artifacts').resolve()
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(artifact_root)
    except ValueError:
        return str(resolved)
    return f'$HOME/jdamr_artifacts/{relative}'


def main(argv: list[str] | None = None) -> int:
    """Run the offline candidate estimator."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--current-radius-m', type=float, required=True)
    parser.add_argument('--current-separation-m', type=float, required=True)
    parser.add_argument('--current-radius-ratio', type=float, default=1.0)
    parser.add_argument('--maximum-pair-gap-s', type=float, default=0.1)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    dimensions = (
        args.current_radius_m, args.current_separation_m,
        args.current_radius_ratio)
    if not all(math.isfinite(value) and value > 0.0
               for value in dimensions):
        parser.error('wheel dimensions must be finite and positive')
    if (not math.isfinite(args.maximum_pair_gap_s)
            or args.maximum_pair_gap_s <= 0.0):
        parser.error('--maximum-pair-gap-s must be finite and positive')

    odometry, reference = read_mcap(args.bag)
    paired = pair_nearest(reference, odometry, args.maximum_pair_gap_s)
    estimate = estimate_candidates(
        paired, args.current_radius_m, args.current_separation_m,
        args.current_radius_ratio)
    report = {
        'schema_version': 1,
        'source': {
            'mcap': portable_path(args.bag),
            'sha256': sha256(args.bag),
            'odometry_samples': len(odometry),
            'amcl_samples': len(reference),
        },
        'current': {
            'wheel_radius_m': args.current_radius_m,
            'wheel_separation_m': args.current_separation_m,
            'wheel_radius_ratio_right_over_left': args.current_radius_ratio,
        },
        'method': {
            'time_basis': 'ROS message header stamp',
            'maximum_pair_gap_s': args.maximum_pair_gap_s,
            'minimum_translation_m': 0.08,
            'maximum_straight_yaw_rad': 0.04,
            'minimum_turn_yaw_rad': 0.08,
        },
        'estimate': estimate,
        'status': 'CANDIDATE_REQUIRES_MEASURED_DRIVE',
        'limitations': [
            'AMCL is a saved-map localization reference, not external ground truth.',
            'Sparse AMCL samples can under-estimate travelled distance.',
            'Do not deploy without a measured straight and rotation check.',
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(report, allow_unicode=True, sort_keys=False),
        encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
