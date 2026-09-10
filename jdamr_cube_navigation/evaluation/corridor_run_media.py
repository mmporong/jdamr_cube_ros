#!/usr/bin/env python3
"""Analyse one corridor run and render reproducible portfolio media."""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import re
import shutil
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any

import matplotlib
from matplotlib import font_manager
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402,I100
import numpy as np  # noqa: E402,I201
import yaml  # noqa: E402,I201

from PIL import Image, ImageDraw, ImageFont  # noqa: E402,I100,I201

from compare_slam_runs import path_length  # noqa: E402,I100
from inspect_mcap import inspect  # noqa: E402,I100,I201
from navigation_mcap_reader import (  # noqa: E402,I100,I201
    read_navigation_messages,
)
from render_route_map import extent_of, load_map  # noqa: E402,I100,I201
from same_goal_resume_evidence import (  # noqa: E402,I100,I201
    goal_status_snapshot,
    summarize_same_goal_resume,
)


TOPICS = (
    '/amcl_pose',
    '/battery_state',
    '/collision_monitor_state',
    '/cmd_vel',
    '/cmd_vel_nav',
    '/imu/data_raw',
    '/navigate_to_pose/_action/status',
    '/odom',
    '/plan',
    '/scan',
    '/tf',
)
READ_TOPICS = (*TOPICS, '/tf_static')
CONTINUITY_SERIES = {
    'scan': '/scan',
    'wheel odom': '/odom',
    'IMU': '/imu/data_raw',
    'map→odom TF': 'tf_odom',
}
LOG_STAMP_RE = re.compile(r'\[([0-9]+\.[0-9]+)\]')
SEND_RE = re.compile(
    r'send (\d+)/(\d+) ([^=]+)=\(([-0-9.]+),([-0-9.]+)\)')
STATUS_RE = re.compile(
    r'waypoint=(\d+)/(\d+) remaining=([-0-9.]+)m '
    r'recoveries=(\d+) battery=([-0-9.]+)V')
PREFLIGHT_RE = re.compile(
    r'route preflight passed: poses=(\d+) length=([-0-9.]+)m')
ROUTE_EVENT_RE = re.compile(r'route_event (\{.*\})\s*$')
COLLISION_ACTION_NAMES = {
    0: 'DO_NOTHING',
    1: 'STOP',
    2: 'SLOWDOWN',
    3: 'APPROACH',
    4: 'LIMIT',
}
ANIMATION_TOP_PX = 154
ANIMATION_BOTTOM_PX = 42
DEFAULT_ANIMATION_FRAMES = 384
DEFAULT_ANIMATION_FPS = 24
DEFAULT_ANIMATION_PLAYBACK_SPEED = 0.75
DEVIATION_HIGHLIGHT_M = 0.12
ROUTE_OBSTACLE_DISTANCE_M = 0.50
ROBOT_OBSTACLE_DISTANCE_M = 2.50
DEVIATION_OBSTACLE_LEAD_NS = 8_000_000_000
COLLISION_PRIORITY = {
    'DO_NOTHING': 0,
    'LIMIT': 1,
    'APPROACH': 2,
    'SLOWDOWN': 3,
    'STOP': 4,
}


def home_relative(path: Path) -> str:
    """Render a path against the current home directory when possible."""
    try:
        return f'$HOME/{path.resolve().relative_to(Path.home())}'
    except ValueError:
        return str(path.resolve())


def parse_route_log(text: str) -> dict[str, Any]:
    """Return route events and the evidence-backed outcome from a log."""
    sends = []
    statuses = []
    goal_events = []
    preflight = None
    success_stamp_ns = None
    last_stamp_ns = None
    for line in text.splitlines():
        stamp_match = LOG_STAMP_RE.search(line)
        if stamp_match is None:
            continue
        stamp_ns = round(float(stamp_match.group(1)) * 1_000_000_000)
        last_stamp_ns = stamp_ns
        if match := PREFLIGHT_RE.search(line):
            preflight = {
                'stamp_ns': stamp_ns,
                'path_pose_count': int(match.group(1)),
                'planned_path_length_m': float(match.group(2)),
            }
        if match := SEND_RE.search(line):
            sends.append({
                'stamp_ns': stamp_ns,
                'index': int(match.group(1)),
                'total': int(match.group(2)),
                'id': match.group(3).strip(),
                'x_m': float(match.group(4)),
                'y_m': float(match.group(5)),
            })
        if match := STATUS_RE.search(line):
            statuses.append({
                'stamp_ns': stamp_ns,
                'waypoint': int(match.group(1)),
                'total': int(match.group(2)),
                'remaining_m': float(match.group(3)),
                'recoveries': int(match.group(4)),
                'battery_v': float(match.group(5)),
            })
        if match := ROUTE_EVENT_RE.search(line):
            try:
                event = json.loads(match.group(1))
            except json.JSONDecodeError:
                continue
            if isinstance(event, dict):
                event['stamp_ns'] = stamp_ns
                goal_events.append(event)
        if 'corridor roundtrip succeeded' in line:
            success_stamp_ns = stamp_ns

    if not sends:
        raise ValueError('route log contains no waypoint sends')
    end_stamp_ns = success_stamp_ns or last_stamp_ns
    if end_stamp_ns is None:
        raise ValueError('route log contains no timestamped result')
    batteries_v = [entry['battery_v'] for entry in statuses]
    return {
        'success': success_stamp_ns is not None,
        'start_stamp_ns': sends[0]['stamp_ns'],
        'end_stamp_ns': end_stamp_ns,
        'duration_s': (end_stamp_ns - sends[0]['stamp_ns']) / 1e9,
        'preflight': preflight,
        'sends': sends,
        'statuses': statuses,
        'goal_events': goal_events,
        'sent_waypoints': len(sends),
        'declared_waypoints': sends[0]['total'],
        'max_recoveries': max(
            (entry['recoveries'] for entry in statuses), default=0),
        'battery_min_v': min(batteries_v) if batteries_v else None,
        'battery_max_v': max(batteries_v) if batteries_v else None,
    }


def gap_statistics(stamps_ns: list[int], *, window_start_ns: int | None = None,
                   window_end_ns: int | None = None) -> dict[str, Any]:
    """Summarize recorder-time continuity for one topic."""
    if not stamps_ns:
        return {'messages': 0, 'rate_hz': None, 'max_gap_s': None,
                'p99_gap_s': None}
    if len(stamps_ns) == 1:
        gaps_ns = []
        if window_start_ns is not None:
            gaps_ns.append(stamps_ns[0] - window_start_ns)
        if window_end_ns is not None:
            gaps_ns.append(window_end_ns - stamps_ns[0])
        max_gap_s = max(gaps_ns) / 1e9 if gaps_ns else None
        return {'messages': 1, 'rate_hz': None,
                'max_gap_s': max_gap_s, 'p99_gap_s': max_gap_s}
    ordered_ns = sorted(stamps_ns)
    gaps_ns = list(np.diff(np.asarray(ordered_ns, dtype=np.int64)))
    if window_start_ns is not None:
        gaps_ns.append(ordered_ns[0] - window_start_ns)
    if window_end_ns is not None:
        gaps_ns.append(window_end_ns - ordered_ns[-1])
    gaps_s = np.asarray(gaps_ns) / 1e9
    range_start_ns = (
        ordered_ns[0] if window_start_ns is None else window_start_ns)
    range_end_ns = (
        ordered_ns[-1] if window_end_ns is None else window_end_ns)
    duration_s = (range_end_ns - range_start_ns) / 1e9
    return {
        'messages': len(ordered_ns),
        'rate_hz': round(len(ordered_ns) / duration_s, 3),
        'max_gap_s': round(float(np.max(gaps_s)), 6),
        'p99_gap_s': round(float(np.percentile(gaps_s, 99)), 6),
    }


def _vector_components(vector: Any) -> tuple[float, float, float]:
    """Return the three ROS vector components as plain floats."""
    return (float(vector.x), float(vector.y), float(vector.z))


def _yaw_from_quaternion(quaternion: Any) -> float:
    """Return planar yaw from a ROS quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


def _compose_pose2d(parent_child: tuple[float, float, float],
                    child_target: tuple[float, float, float]
                    ) -> tuple[float, float, float]:
    """Compose parent→child and child→target planar poses."""
    px, py, pyaw = parent_child
    cx, cy, cyaw = child_target
    cosine = math.cos(pyaw)
    sine = math.sin(pyaw)
    return (
        px + cosine * cx - sine * cy,
        py + sine * cx + cosine * cy,
        pyaw + cyaw,
    )


def _nearest_by_stamp(samples: list[dict[str, Any]], stamp_ns: int,
                      *, max_gap_ns: int | None = None
                      ) -> dict[str, Any] | None:
    """Return the closest timestamped sample within an optional gap."""
    if not samples:
        return None
    stamps = [entry['stamp_ns'] for entry in samples]
    index = bisect.bisect_left(stamps, stamp_ns)
    candidates = samples[max(0, index - 1):min(len(samples), index + 1)]
    closest = min(
        candidates, key=lambda entry: abs(entry['stamp_ns'] - stamp_ns))
    if (max_gap_ns is not None
            and abs(closest['stamp_ns'] - stamp_ns) > max_gap_ns):
        return None
    return closest


def _project_scan_points(
        scan: dict[str, Any], map_base_pose: tuple[float, float, float],
        laser_pose: tuple[float, float, float], occupancy: np.ndarray,
        resolution_m: float, origin: list[float],
        *, wall_margin_cells: int = 4) -> dict[str, list[tuple[float, float]]]:
    """
    Project one scan into map coordinates and classify endpoint evidence.

    Returns measured endpoints split into static-map matches and returns in
    cells recorded as free space. Free-space returns are obstacle candidates,
    not object-class labels.
    """
    map_laser_pose = _compose_pose2d(map_base_pose, laser_pose)
    laser_x_m, laser_y_m, laser_yaw = map_laser_pose
    height, width = occupancy.shape
    static = []
    obstacle_candidates = []
    for index, range_m in enumerate(scan['ranges_m']):
        if not math.isfinite(range_m):
            continue
        if not scan['range_min_m'] <= range_m <= scan['range_max_m']:
            continue
        angle = laser_yaw + scan['angle_min_rad'] + (
            index * scan['angle_increment_rad'])
        x_m = laser_x_m + range_m * math.cos(angle)
        y_m = laser_y_m + range_m * math.sin(angle)
        x_cell = round((x_m - origin[0]) / resolution_m)
        y_cell = height - 1 - round((y_m - origin[1]) / resolution_m)
        if not (0 <= x_cell < width and 0 <= y_cell < height):
            continue
        y0 = max(0, y_cell - wall_margin_cells)
        y1 = min(height, y_cell + wall_margin_cells + 1)
        x0 = max(0, x_cell - wall_margin_cells)
        x1 = min(width, x_cell + wall_margin_cells + 1)
        if np.any(occupancy[y0:y1, x0:x1] < 100):
            static.append((x_m, y_m))
        elif occupancy[y_cell, x_cell] > 240:
            obstacle_candidates.append((x_m, y_m))
    return {'static': static, 'obstacle_candidates': obstacle_candidates}


def _select_frame_collision_event(
        events: list[dict[str, Any]], frame_stamp_ns: int,
        half_window_ns: int) -> dict[str, Any] | None:
    """Select the strongest real collision event represented by a frame."""
    candidates = [
        event for event in events
        if abs(event['stamp_ns'] - frame_stamp_ns) <= half_window_ns
        and event['action_name'] != 'DO_NOTHING'
        and event['polygon_name'] != 'invalid source'
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda event: (
        COLLISION_PRIORITY.get(event['action_name'], -1),
        -abs(event['stamp_ns'] - frame_stamp_ns),
    ))


def _collision_badge(
        action_name: str, polygon_name: str,
        duration_s: float | None = None) -> str:
    """Translate internal collision-monitor state into operator language."""
    if action_name == 'STOP' and polygon_name == 'StopZone':
        duration_text = (
            f' {duration_s:.2f}초' if duration_s is not None else '')
        return f'자동 안전 정지{duration_text} · 충돌 방지 개입'
    action_labels = {
        'SLOWDOWN': '안전 감속 제어',
        'APPROACH': '접근 속도 제어',
        'LIMIT': '속도 제한',
        'STOP': '자동 안전 정지',
    }
    return action_labels.get(action_name, action_name)


def _point_to_polyline_distance(
        point: tuple[float, float],
        polyline: list[tuple[float, float]]) -> float:
    """Return the shortest planar distance to a route polyline."""
    if not polyline:
        raise ValueError('route polyline is empty')
    if len(polyline) == 1:
        return math.dist(point, polyline[0])
    x_m, y_m = point
    distances = []
    for start, end in zip(polyline, polyline[1:]):
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        length_squared = delta_x ** 2 + delta_y ** 2
        if length_squared == 0.0:
            distances.append(math.dist(point, start))
            continue
        projection = max(0.0, min(1.0, (
            (x_m - start[0]) * delta_x
            + (y_m - start[1]) * delta_y
        ) / length_squared))
        nearest_x = start[0] + projection * delta_x
        nearest_y = start[1] + projection * delta_y
        distances.append(math.hypot(x_m - nearest_x, y_m - nearest_y))
    return min(distances)


def _cluster_points(points: list[tuple[float, float]],
                    *, radius_m: float = 0.18,
                    min_points: int = 3) -> list[list[tuple[float, float]]]:
    """Group nearby lidar endpoints without assigning an object class."""
    remaining = set(range(len(points)))
    clusters = []
    radius_squared = radius_m ** 2
    while remaining:
        seed = remaining.pop()
        cluster_indexes = {seed}
        frontier = [seed]
        while frontier:
            current = frontier.pop()
            x_m, y_m = points[current]
            neighbours = [
                index for index in remaining
                if ((points[index][0] - x_m) ** 2
                    + (points[index][1] - y_m) ** 2) <= radius_squared
            ]
            for index in neighbours:
                remaining.remove(index)
                cluster_indexes.add(index)
                frontier.append(index)
        if len(cluster_indexes) >= min_points:
            clusters.append([points[index] for index in cluster_indexes])
    return clusters


def _route_deviation_episodes(
        samples: list[dict[str, Any]],
        deviations_m: list[float],
        *, threshold_m: float = DEVIATION_HIGHLIGHT_M
        ) -> list[dict[str, Any]]:
    """Group contiguous cross-track threshold violations into episodes."""
    episodes = []
    start_index = None
    for index, deviation_m in enumerate(deviations_m):
        highlighted = deviation_m >= threshold_m
        if highlighted and start_index is None:
            start_index = index
        final_sample = index == len(deviations_m) - 1
        if start_index is None or (highlighted and not final_sample):
            continue
        end_index = index if highlighted else index - 1
        peak_index = max(
            range(start_index, end_index + 1),
            key=lambda candidate: deviations_m[candidate])
        episodes.append({
            'start_index': start_index,
            'end_index': end_index,
            'start_stamp_ns': samples[start_index]['stamp_ns'],
            'end_stamp_ns': samples[end_index]['stamp_ns'],
            'peak_deviation_m': deviations_m[peak_index],
        })
        start_index = None
    return episodes


def _select_path_obstacle_cluster(
        clusters: list[list[tuple[float, float]]],
        planned_route: list[tuple[float, float]],
        robot_xy_m: tuple[float, float],
        *, route_distance_m: float = ROUTE_OBSTACLE_DISTANCE_M,
        robot_distance_m: float = ROBOT_OBSTACLE_DISTANCE_M
        ) -> list[tuple[float, float]] | None:
    """Select one lidar cluster on the baseline and near the robot."""
    ranked = sorted(
        (
            min(_point_to_polyline_distance(point, planned_route)
                for point in cluster),
            min(math.dist(point, robot_xy_m) for point in cluster),
            -len(cluster),
            cluster,
        )
        for cluster in clusters if cluster
    )
    for route_gap_m, robot_gap_m, _size, cluster in ranked:
        if (route_gap_m <= route_distance_m
                and robot_gap_m <= robot_distance_m):
            return cluster
    return None


def _command_state(command: Any) -> str:
    """Classify a command without treating non-finite values as motion."""
    components = (
        *_vector_components(command.linear),
        *_vector_components(command.angular),
    )
    if not all(math.isfinite(value) for value in components):
        return 'UNKNOWN_NONFINITE'
    return 'ZERO' if all(value == 0.0 for value in components) else 'NONZERO'


def _plan_observation(stamp_ns: int, path_message: Any) -> dict[str, Any]:
    """Create a deterministic recorder-time summary of one Path message."""
    frame_id = str(path_message.header.frame_id)
    geometry = []
    positions = []
    explicit_pose_frame_ids = set()
    for stamped_pose in path_message.poses:
        pose_frame_id = str(getattr(
            getattr(stamped_pose, 'header', None), 'frame_id', ''))
        if pose_frame_id:
            explicit_pose_frame_ids.add(pose_frame_id)
        pose = stamped_pose.pose
        position = _vector_components(pose.position)
        orientation = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        positions.append(position)
        geometry.append((*position, *orientation))
    geometry_bytes = json.dumps({
        'frame_id': frame_id,
        'poses': geometry,
    }, allow_nan=False, sort_keys=True, separators=(',', ':')).encode('ascii')
    length_m = sum(
        math.dist(previous, current)
        for previous, current in zip(positions, positions[1:])
    )
    data_gap_reasons = []
    if any(pose_frame_id != frame_id
           for pose_frame_id in explicit_pose_frame_ids):
        data_gap_reasons.append('POSE_FRAME_ID_MISMATCH')
    return {
        'stamp_ns': stamp_ns,
        'frame_id': frame_id,
        'explicit_pose_frame_ids': sorted(explicit_pose_frame_ids),
        'pose_count': len(geometry),
        'frame_and_geometry_sha256': hashlib.sha256(
            geometry_bytes).hexdigest(),
        'path_length_m': round(length_m, 6),
        'data_gap_reasons': data_gap_reasons,
    }


def _plan_series_entry(stamp_ns: int, path_message: Any) -> dict[str, Any]:
    """Keep map-frame plan geometry for time-aligned media rendering."""
    return {
        'stamp_ns': stamp_ns,
        'frame_id': str(path_message.header.frame_id),
        'points_xy_m': [
            (float(stamped_pose.pose.position.x),
             float(stamped_pose.pose.position.y))
            for stamped_pose in path_message.poses
        ],
    }


def summarize_navigation_events(
        collision_states: list[tuple[int, int, str]],
        command_states: list[tuple[int, str]],
        plans: list[dict[str, Any]],
        status_snapshots: list[dict[str, Any]] | None = None,
        drive_start_ns: int | None = None,
        drive_end_ns: int | None = None,
        terminal_end_ns: int | None = None) -> dict[str, Any]:
    """Summarize navigation event evidence using recorder timestamps."""
    collision_transitions = []
    previous_collision = None
    for stamp_ns, action_type, polygon_name in collision_states:
        current = (action_type, polygon_name)
        if previous_collision is not None and current != previous_collision:
            collision_transitions.append({
                'stamp_ns': stamp_ns,
                'from': {
                    'action_type': previous_collision[0],
                    'action_name': COLLISION_ACTION_NAMES.get(
                        previous_collision[0], 'UNKNOWN'),
                    'polygon_name': previous_collision[1],
                },
                'to': {
                    'action_type': action_type,
                    'action_name': COLLISION_ACTION_NAMES.get(
                        action_type, 'UNKNOWN'),
                    'polygon_name': polygon_name,
                },
            })
        previous_collision = current

    command_transitions_ns = []
    previous_command_state = None
    for stamp_ns, command_state in command_states:
        if (previous_command_state == 'ZERO'
                and command_state == 'NONZERO'):
            command_transitions_ns.append(stamp_ns)
        previous_command_state = (
            None if command_state == 'UNKNOWN_NONFINITE' else command_state)

    plan_reasons = []
    if not plans:
        plan_reasons.append('NO_OBSERVATIONS_IN_DRIVE_WINDOW')
    if any(plan['pose_count'] == 0 for plan in plans):
        plan_reasons.append('EMPTY_PATH')
    frame_ids = sorted({plan['frame_id'] for plan in plans})
    if any(not frame_id for frame_id in frame_ids):
        plan_reasons.append('MISSING_FRAME_ID')
    if len(frame_ids) > 1:
        plan_reasons.append('FRAME_ID_MISMATCH')
    for plan in plans:
        plan_reasons.extend(plan.get('data_gap_reasons', []))
    plan_reasons = list(dict.fromkeys(plan_reasons))
    comparable_plans = [plan for plan in plans if plan['pose_count'] > 0]
    plan_change_count = None
    if not plan_reasons:
        plan_change_count = sum(
            previous['frame_and_geometry_sha256']
            != current['frame_and_geometry_sha256']
            for previous, current in zip(plans, plans[1:])
        )
    lengths_m = [plan['path_length_m'] for plan in comparable_plans]
    collision_reasons = (
        [] if collision_states else ['NO_OBSERVATIONS_IN_DRIVE_WINDOW'])
    if any(action_type not in COLLISION_ACTION_NAMES
           for _stamp_ns, action_type, _polygon_name in collision_states):
        collision_reasons.append('UNKNOWN_ACTION_TYPE')
    command_reasons = (
        [] if command_states else ['NO_OBSERVATIONS_IN_DRIVE_WINDOW'])
    if any(state == 'UNKNOWN_NONFINITE'
           for _stamp_ns, state in command_states):
        command_reasons.append('NONFINITE_COMMAND')

    if (drive_start_ns is None or drive_end_ns is None
            or terminal_end_ns is None):
        same_goal_evidence = {
            'verdict': 'NOT_MEASURED',
            'scope': 'COMMAND_SPACE_ONLY',
            'action_status_messages': 0,
            'action_status_observations': [],
            'terminal_succeeded': None,
            'episodes': [],
            'data_gap_reasons': ['NO_ACTION_STATUS_OBSERVATIONS'],
        }
    else:
        same_goal_evidence = summarize_same_goal_resume(
            collision_states, command_states, status_snapshots or [],
            drive_start_ns, drive_end_ns, terminal_end_ns)

    return {
        'time_basis': 'recorder_log_time_ns',
        'collision_monitor_state': {
            'evidence_status': (
                'MEASURED' if not collision_reasons else 'INSUFFICIENT_DATA'),
            'messages': len(collision_states),
            'observations': [{
                'stamp_ns': stamp_ns,
                'action_type': action_type,
                'action_name': COLLISION_ACTION_NAMES.get(
                    action_type, 'UNKNOWN'),
                'polygon_name': polygon_name,
            } for stamp_ns, action_type, polygon_name in collision_states],
            'transitions': collision_transitions,
            'data_gap_reasons': collision_reasons,
        },
        'cmd_vel_zero_to_nonzero': {
            'evidence_status': (
                'MEASURED' if not command_reasons else 'INSUFFICIENT_DATA'),
            'messages': len(command_states),
            'transition_stamps_ns': command_transitions_ns,
            'data_gap_reasons': command_reasons,
            'interpretation': (
                'Command-space transitions only; physical standstill and '
                'motion were not measured.'),
        },
        'plan_geometry': {
            'evidence_status': (
                'MEASURED' if not plan_reasons else 'INSUFFICIENT_DATA'),
            'messages': len(plans),
            'frame_ids': frame_ids,
            'geometry_change_count': plan_change_count,
            'path_length_m': {
                'samples': len(lengths_m),
                'min': min(lengths_m, default=None),
                'max': max(lengths_m, default=None),
                'latest': lengths_m[-1] if lengths_m else None,
            },
            'observations': plans,
            'data_gap_reasons': plan_reasons,
            'interpretation': (
                'Path geometry changes do not by themselves prove obstacle '
                'avoidance.'),
        },
        'same_goal_resumed': same_goal_evidence['verdict'],
        'same_goal_resume_evidence': same_goal_evidence,
    }


def _read_process_metrics(path: Path, start_ns: int,
                          end_ns: int) -> dict[str, Any] | None:
    """Summarize resource samples whose wall times overlap the drive."""
    if not path.is_file():
        return None
    rows = []
    with path.open(encoding='utf-8') as stream:
        for row in csv.DictReader(stream, delimiter='\t'):
            if not row.get('time') or not row.get('system_cpu_pct'):
                continue
            stamp_ns = round(
                datetime.strptime(row['time'], '%Y-%m-%dT%H:%M:%S%z')
                .timestamp() * 1e9)
            if start_ns <= stamp_ns <= end_ns:
                rows.append((stamp_ns, row))
    if not rows:
        return None

    samples: dict[str, dict[str, str]] = {}
    by_label: dict[str, list[float]] = {}
    for _stamp_ns, row in rows:
        samples[row['sample']] = row
        by_label.setdefault(row['label'], []).append(float(row['cpu_pct']))
    system_cpu_pct = [float(row['system_cpu_pct'])
                      for row in samples.values()]
    temperatures_c = [float(row['temp_c']) for row in samples.values()]
    throttled = [int(row['throttled']) for row in samples.values()]
    sample_stamps_ns = sorted({stamp_ns for stamp_ns, _row in rows})
    return {
        'samples': len(samples),
        'max_sample_gap_s': gap_statistics(sample_stamps_ns)['max_gap_s'],
        'system_cpu_mean_pct': round(float(np.mean(system_cpu_pct)), 1),
        'system_cpu_p90_pct': round(
            float(np.percentile(system_cpu_pct, 90)), 1),
        'system_cpu_peak_pct': round(max(system_cpu_pct), 1),
        'peak_temperature_c': round(max(temperatures_c), 1),
        'throttled_samples': sum(value != 0 for value in throttled),
        'process_cpu_mean_pct': {
            label: round(float(np.mean(values)), 1)
            for label, values in sorted(by_label.items())
        },
    }


def _sha256(path: Path) -> str:
    """Return the SHA-256 digest of a local evidence file."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def analyse_run(run_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Read a run once and return aggregate metrics plus plotting samples."""
    run_dir = run_dir.resolve()
    run_id = run_dir.name
    bag_files = sorted(run_dir.glob('*.mcap'))
    if len(bag_files) != 1:
        raise ValueError(
            f'expected one MCAP under {run_dir}, got {len(bag_files)}')
    bag = bag_files[0]
    route_log_path = run_dir.parent / f'{run_id}.route.log'
    if not route_log_path.is_file():
        raise FileNotFoundError(route_log_path)
    route = parse_route_log(route_log_path.read_text(encoding='utf-8'))
    start_ns = route['start_stamp_ns']
    end_ns = route['end_stamp_ns']

    stamps: dict[str, list[int]] = {topic: [] for topic in TOPICS}
    stamps['tf_base_footprint'] = []
    stamps['tf_odom'] = []
    amcl = []
    odom = []
    batteries_v = []
    post_stop_commands = []
    collision_states = []
    command_states = []
    plans = []
    plan_series = []
    scans = []
    odom_poses = []
    map_odom_poses = []
    laser_pose = None
    status_snapshots = []
    pre_drive_status_snapshot = None
    for message in read_navigation_messages(bag, topics=list(READ_TOPICS)):
        topic = message.channel.topic
        stamp_ns = message.log_time_ns
        if topic == '/tf_static':
            for transform in message.ros_msg.transforms:
                parent = transform.header.frame_id.lstrip('/')
                child = transform.child_frame_id.lstrip('/')
                if parent == 'base_link' and child == 'laser_link':
                    translation = transform.transform.translation
                    laser_pose = (
                        float(translation.x), float(translation.y),
                        _yaw_from_quaternion(transform.transform.rotation),
                    )
            continue
        if topic == '/navigate_to_pose/_action/status':
            snapshot = goal_status_snapshot(stamp_ns, message.ros_msg)
            if stamp_ns < start_ns:
                pre_drive_status_snapshot = snapshot
            elif stamp_ns <= end_ns + 5_000_000_000:
                status_snapshots.append(snapshot)
        if topic == '/cmd_vel' and end_ns <= stamp_ns <= end_ns + 5e9:
            command = message.ros_msg
            post_stop_commands.append(
                (command.linear.x, command.angular.z))
        if not start_ns <= stamp_ns <= end_ns:
            continue
        stamps[topic].append(stamp_ns)
        if topic == '/amcl_pose':
            pose = message.ros_msg.pose.pose
            covariance = message.ros_msg.pose.covariance
            amcl.append({
                'stamp_ns': stamp_ns,
                'x_m': pose.position.x,
                'y_m': pose.position.y,
                'x_covariance_m2': covariance[0],
                'y_covariance_m2': covariance[7],
                'yaw_covariance_rad2': covariance[35],
            })
        elif topic == '/odom':
            pose = message.ros_msg.pose.pose
            odom.append((stamp_ns / 1e9, pose.position.x,
                         pose.position.y, 0.0))
            odom_poses.append({
                'stamp_ns': stamp_ns,
                'pose': (
                    float(pose.position.x), float(pose.position.y),
                    _yaw_from_quaternion(pose.orientation),
                ),
            })
        elif topic == '/battery_state':
            batteries_v.append(message.ros_msg.voltage)
        elif topic == '/collision_monitor_state':
            state = message.ros_msg
            collision_states.append((
                stamp_ns, int(state.action_type), str(state.polygon_name)))
        elif topic == '/cmd_vel':
            command_states.append(
                (stamp_ns, _command_state(message.ros_msg)))
        elif topic == '/plan':
            plans.append(_plan_observation(stamp_ns, message.ros_msg))
            plan_series.append(_plan_series_entry(stamp_ns, message.ros_msg))
        elif topic == '/scan':
            scan = message.ros_msg
            scans.append({
                'stamp_ns': stamp_ns,
                'angle_min_rad': float(scan.angle_min),
                'angle_increment_rad': float(scan.angle_increment),
                'range_min_m': float(scan.range_min),
                'range_max_m': float(scan.range_max),
                'ranges_m': np.asarray(scan.ranges, dtype=np.float32),
            })
        elif topic == '/tf':
            for transform in message.ros_msg.transforms:
                parent = transform.header.frame_id.lstrip('/')
                child = transform.child_frame_id.lstrip('/')
                key = f'tf_{child}'
                if key in stamps:
                    stamps[key].append(stamp_ns)
                if parent == 'map' and child == 'odom':
                    translation = transform.transform.translation
                    map_odom_poses.append({
                        'stamp_ns': stamp_ns,
                        'pose': (
                            float(translation.x), float(translation.y),
                            _yaw_from_quaternion(
                                transform.transform.rotation),
                        ),
                    })

    if not amcl:
        raise ValueError('drive window contains no /amcl_pose samples')
    amcl_points = [
        (entry['stamp_ns'] / 1e9, entry['x_m'], entry['y_m'], 0.0)
        for entry in amcl
    ]
    home_xy_m = (route['sends'][-1]['x_m'], route['sends'][-1]['y_m'])
    start_xy_m = (amcl[0]['x_m'], amcl[0]['y_m'])
    end_xy_m = (amcl[-1]['x_m'], amcl[-1]['y_m'])
    continuity = {
        key: gap_statistics(
            value, window_start_ns=start_ns, window_end_ns=end_ns)
        for key, value in stamps.items()
    }
    post_linear_mps = [abs(entry[0]) for entry in post_stop_commands]
    post_angular_rps = [abs(entry[1]) for entry in post_stop_commands]
    process_log_path = run_dir.parent / f'{run_id}.per_process.tsv'
    integrity = inspect(bag)

    if pre_drive_status_snapshot is not None:
        status_snapshots.insert(0, pre_drive_status_snapshot)

    scan_series = []
    if laser_pose is not None:
        for scan in scans:
            map_odom = _nearest_by_stamp(
                map_odom_poses, scan['stamp_ns'], max_gap_ns=500_000_000)
            odom_base = _nearest_by_stamp(
                odom_poses, scan['stamp_ns'], max_gap_ns=100_000_000)
            if map_odom is None or odom_base is None:
                continue
            scan_series.append({
                **scan,
                'map_base_pose': _compose_pose2d(
                    map_odom['pose'], odom_base['pose']),
            })

    metrics = {
        'schema_version': 1,
        'run_id': run_id,
        'outcome': 'SUCCESS' if route['success'] else 'INCOMPLETE',
        'source': {
            'run_dir': home_relative(run_dir),
            'mcap': home_relative(bag),
            'mcap_sha256': integrity['sha256'],
            'route_log': home_relative(route_log_path),
            'route_log_sha256': _sha256(route_log_path),
            'process_log': (
                home_relative(process_log_path)
                if process_log_path.is_file() else None),
        },
        'integrity': integrity['integrity'],
        'capture': {
            'total_messages': integrity['message_count'],
            'total_duration_s': round(integrity['duration_ns'] / 1e9, 3),
            'drive_start_unix_ns': start_ns,
            'drive_end_unix_ns': end_ns,
            'drive_duration_s': round(route['duration_s'], 3),
        },
        'navigation': {
            'sent_waypoints': route['sent_waypoints'],
            'declared_waypoints': route['declared_waypoints'],
            'max_recoveries': route['max_recoveries'],
            'preflight': route['preflight'],
            'amcl_path_length_m': round(path_length(amcl_points), 3),
            'wheel_odom_path_length_m': round(path_length(odom), 3),
            'start_to_end_amcl_m': round(
                math.dist(start_xy_m, end_xy_m), 3),
            'final_home_error_amcl_m': round(
                math.dist(end_xy_m, home_xy_m), 3),
            'amcl_start_xy_m': [round(value, 3) for value in start_xy_m],
            'amcl_end_xy_m': [round(value, 3) for value in end_xy_m],
            'amcl_max_x_m': round(max(entry['x_m'] for entry in amcl), 3),
            'battery_min_v': round(min(batteries_v), 3),
            'battery_max_v': round(max(batteries_v), 3),
            'post_success_stop': {
                'samples': len(post_stop_commands),
                'max_abs_linear_mps': round(
                    max(post_linear_mps, default=0.0), 6),
                'max_abs_angular_rps': round(
                    max(post_angular_rps, default=0.0), 6),
                'final_command_zero': (
                    bool(post_stop_commands)
                    and post_stop_commands[-1] == (0.0, 0.0)),
            },
        },
        'navigation_events': {
            **summarize_navigation_events(
                collision_states, command_states, plans, status_snapshots,
                start_ns, end_ns, end_ns + 5_000_000_000),
            'goal_events': route['goal_events'],
            'goal_events_time_basis': 'ros_logger_time_ns',
            'obstacle_evidence': {
                'lidar_topic': '/scan',
                'lidar_messages_in_drive_window': len(stamps['/scan']),
                'projectable_scan_samples': len(scan_series),
                'collision_monitor_topic': '/collision_monitor_state',
                'collision_monitor_observation_source': '/scan',
                'stopzone_episodes': sum(
                    action_type == 1 and polygon_name == 'StopZone'
                    for _stamp_ns, action_type, polygon_name
                    in collision_states),
                'object_classification': 'NOT_MEASURED',
                'object_classification_reason': (
                    'No camera image or person/object detector topic was '
                    'recorded in this MCAP.'),
                'interpretation': (
                    'Free-space lidar endpoints are dynamic or unmapped '
                    'obstacle candidates, not proof of a person.'),
            },
        },
        'continuity': continuity,
        'resources': _read_process_metrics(
            process_log_path, start_ns, end_ns),
        'limitations': [
            'AMCL is a saved-map localization estimate, not external ground '
            'truth.',
            'Recorder transport-loss counters were not emitted; loss is '
            'UNKNOWN.',
            'The full-capture resource gate missed one terminal sample; '
            'drive-window '
            'resource values remain descriptive evidence only.',
            'No camera image or person/object detector topic was recorded; '
            'obstacle class is UNKNOWN.',
        ],
    }
    series = {
        'amcl': amcl,
        'statuses': route['statuses'],
        'sends': route['sends'],
        'collision_states': [{
            'stamp_ns': stamp_ns,
            'action_type': action_type,
            'action_name': COLLISION_ACTION_NAMES.get(
                action_type, 'UNKNOWN'),
            'polygon_name': polygon_name,
        } for stamp_ns, action_type, polygon_name in collision_states],
        'plans': plan_series,
        'scans': scan_series,
        'laser_pose': laser_pose,
    }
    return metrics, series


def _configure_plot_font() -> None:
    """Use the installed Korean font when matplotlib can resolve it."""
    font_path = Path(
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font_path.is_file():
        family = font_manager.FontProperties(fname=str(font_path)).get_name()
        plt.rcParams['font.family'] = family
    plt.rcParams['axes.unicode_minus'] = False


def _load_route_context(route_yaml: Path):
    """Return route configuration and the two map images it references."""
    config = yaml.safe_load(route_yaml.read_text(encoding='utf-8'))
    home = str(Path.home())
    map_yaml = Path(config['map_yaml'].replace('$HOME', home))
    mask_yaml = Path(config['keepout_mask_yaml'].replace('$HOME', home))
    occupancy, resolution, origin = load_map(map_yaml)
    mask, _, _ = load_map(mask_yaml)
    return config, occupancy, mask, resolution, origin


def render_route(route_yaml: Path, metrics: dict[str, Any],
                 series: dict[str, Any], output: Path) -> None:
    """Render saved map, keepout mask, plan, and measured AMCL path."""
    config, occupancy, mask, resolution, origin = _load_route_context(
        route_yaml)
    extent = extent_of(occupancy, resolution, origin)
    figure = plt.figure(figsize=(16, 6.2), dpi=160)
    grid = figure.add_gridspec(1, 5)
    axes = figure.add_subplot(grid[0, :4])
    summary = figure.add_subplot(grid[0, 4])
    axes.imshow(occupancy, cmap='gray', extent=extent, origin='upper',
                interpolation='nearest', vmin=0, vmax=255)
    keepout = np.ma.masked_where(mask >= 100, np.ones_like(mask))
    axes.imshow(keepout, extent=extent, origin='upper',
                interpolation='nearest', cmap='autumn', alpha=0.55)
    planned_x = [config['start_pose']['x']]
    planned_y = [config['start_pose']['y']]
    planned_x.extend(entry['x'] for entry in config['waypoints'])
    planned_y.extend(entry['y'] for entry in config['waypoints'])
    axes.plot(planned_x, planned_y, '--', color='#3182bd', linewidth=1.6,
              label='planned waypoints')
    driven_x = [entry['x_m'] for entry in series['amcl']]
    driven_y = [entry['y_m'] for entry in series['amcl']]
    axes.plot(driven_x, driven_y, color='#16a34a', linewidth=2.5,
              label='driven path (AMCL)')
    axes.scatter(planned_x[1:], planned_y[1:], s=18, color='#3182bd')
    axes.scatter([planned_x[0]], [planned_y[0]], s=180, marker='*',
                 color='#111827', label='start / home', zorder=6)
    axes.scatter([driven_x[-1]], [driven_y[-1]], s=70, marker='o',
                 color='#f59e0b', edgecolors='#111827', label='finish',
                 zorder=7)
    axes.set_xlabel('x [m] (map frame)')
    axes.set_ylabel('y [m]')
    axes.set_title(
        f"JD-AMR saved-map corridor round trip — {metrics['run_id']}\n"
        'orange: keepout / blue: planned / green: recorded localization')
    axes.set_aspect('equal')
    axes.grid(alpha=0.2, linewidth=0.5)
    axes.legend(loc='lower right', framealpha=0.92)

    navigation = metrics['navigation']
    capture = metrics['capture']
    sent_waypoints = navigation['sent_waypoints']
    declared_waypoints = navigation['declared_waypoints']
    max_recoveries = navigation['max_recoveries']
    preflight_length_m = navigation['preflight']['planned_path_length_m']
    amcl_length_m = navigation['amcl_path_length_m']
    drive_minutes = capture['drive_duration_s'] / 60
    return_error_m = navigation['start_to_end_amcl_m']
    battery_min_v = navigation['battery_min_v']
    battery_max_v = navigation['battery_max_v']
    total_messages = capture['total_messages']
    summary.axis('off')
    lines = [
        'RUN VERIFIED',
        '',
        f'Waypoints  {sent_waypoints} / {declared_waypoints}',
        f'Nav2 recoveries  {max_recoveries}',
        f'Preflight path  {preflight_length_m:.3f} m',
        f'AMCL path  {amcl_length_m:.3f} m',
        f'Drive time  {drive_minutes:.2f} min',
        f'Return error  {return_error_m:.3f} m',
        f'Battery  {battery_min_v:.2f}–{battery_max_v:.2f} V',
        '',
        f'MCAP messages  {total_messages:,}',
        'CRC + indexes  PASS',
    ]
    summary.text(0.02, 0.95, '\n'.join(lines), va='top', ha='left',
                 fontsize=11.5, linespacing=1.55, family='monospace',
                 bbox={'boxstyle': 'round,pad=0.8', 'facecolor': '#f8fafc',
                       'edgecolor': '#cbd5e1'})
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, bbox_inches='tight')
    plt.close(figure)


def render_continuity(metrics: dict[str, Any],
                      comparisons: list[dict[str, Any]],
                      output: Path) -> None:
    """Render the sensor-gap difference against earlier runs."""
    records = comparisons + [metrics]
    labels = [record['run_id'] for record in records]
    width = 0.8 / len(records)
    x_positions = np.arange(len(CONTINUITY_SERIES))
    figure, axes = plt.subplots(figsize=(12, 6.4), dpi=160)
    for index, record in enumerate(records):
        values = [
            record['continuity'][key]['max_gap_s']
            for key in CONTINUITY_SERIES.values()
        ]
        offset = (index - (len(records) - 1) / 2) * width
        bars = axes.bar(x_positions + offset, values, width=width,
                        label=labels[index])
        for bar, value in zip(bars, values):
            axes.text(bar.get_x() + bar.get_width() / 2, value * 1.12,
                      f'{value:.3f}s', ha='center', va='bottom', fontsize=8,
                      rotation=90 if value < 0.3 else 0)
    axes.axhline(1.0, color='#dc2626', linestyle=':', linewidth=1.5,
                 label='1 s reference')
    axes.set_yscale('log')
    axes.set_ylabel('maximum recorder-time gap [s] — log scale')
    axes.set_xticks(x_positions, CONTINUITY_SERIES.keys())
    axes.set_title(
        'Control-path continuity before and after local DDS isolation')
    axes.grid(axis='y', which='both', alpha=0.25)
    axes.legend(loc='upper center', bbox_to_anchor=(0.5, -0.13), ncol=3)
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, bbox_inches='tight')
    plt.close(figure)


def render_telemetry(metrics: dict[str, Any], series: dict[str, Any],
                     output: Path) -> None:
    """Render route progress, battery, localization, and resource telemetry."""
    start_ns = metrics['capture']['drive_start_unix_ns']
    statuses = series['statuses']
    amcl = series['amcl']
    status_time_s = [(entry['stamp_ns'] - start_ns) / 1e9
                     for entry in statuses]
    amcl_time_s = [(entry['stamp_ns'] - start_ns) / 1e9 for entry in amcl]
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), dpi=160)
    progress = axes[0, 0]
    progress.scatter(status_time_s,
                     [entry['remaining_m'] for entry in statuses],
                     c=[entry['waypoint'] for entry in statuses], s=16,
                     cmap='viridis')
    for send in series['sends']:
        progress.axvline((send['stamp_ns'] - start_ns) / 1e9,
                         color='#94a3b8', linewidth=0.35, alpha=0.5)
    progress.set_title('Waypoint progress')
    progress.set_ylabel('remaining distance [m]')
    progress.set_xlabel('elapsed [s]')
    progress.grid(alpha=0.25)

    battery = axes[0, 1]
    battery.plot(status_time_s,
                 [entry['battery_v'] for entry in statuses],
                 color='#ca8a04', linewidth=1.5)
    battery.set_title('Battery during navigation')
    battery.set_ylabel('voltage [V]')
    battery.set_xlabel('elapsed [s]')
    battery.grid(alpha=0.25)

    covariance = axes[1, 0]
    covariance.plot(amcl_time_s,
                    [entry['x_covariance_m2'] for entry in amcl],
                    label='x covariance', color='#2563eb')
    covariance.plot(amcl_time_s,
                    [entry['y_covariance_m2'] for entry in amcl],
                    label='y covariance', color='#16a34a')
    covariance.set_yscale('symlog', linthresh=0.01)
    covariance.set_title('AMCL position covariance')
    covariance.set_ylabel('variance [m²] — symlog')
    covariance.set_xlabel('elapsed [s]')
    covariance.grid(alpha=0.25)
    covariance.legend()

    continuity = axes[1, 1]
    names = list(CONTINUITY_SERIES.keys())
    values = [metrics['continuity'][key]['max_gap_s']
              for key in CONTINUITY_SERIES.values()]
    bars = continuity.bar(names, values, color='#0f766e')
    for bar, value in zip(bars, values):
        continuity.text(bar.get_x() + bar.get_width() / 2, value,
                        f'{value:.3f}s', ha='center', va='bottom', fontsize=9)
    continuity.set_title('Maximum control-path gaps')
    continuity.set_ylabel('gap [s]')
    continuity.grid(axis='y', alpha=0.25)
    figure.suptitle(f"JD-AMR corridor telemetry — {metrics['run_id']}")
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, bbox_inches='tight')
    plt.close(figure)


def _pil_font(size: int, bold: bool = False):
    """Return a readable Korean-capable font for generated frames."""
    name = 'NotoSansCJK-Bold.ttc' if bold else 'NotoSansCJK-Medium.ttc'
    path = Path('/usr/share/fonts/opentype/noto') / name
    if path.is_file():
        return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def _world_to_pixel(x_m: float, y_m: float, *, resolution_m: float,
                    origin: list[float], height: int,
                    scale: float, top_px: int) -> tuple[int, int]:
    """Convert map-frame metric coordinates to the GIF canvas."""
    x_px = (x_m - origin[0]) / resolution_m
    y_px = height - 1 - (y_m - origin[1]) / resolution_m
    return round(x_px * scale), round(y_px * scale + top_px)


def render_animation(route_yaml: Path, metrics: dict[str, Any],
                     series: dict[str, Any], output: Path,
                     frames: int, fps: int,
                     playback_speed: float =
                     DEFAULT_ANIMATION_PLAYBACK_SPEED) -> None:
    """Render time-aligned trajectory, lidar, planning, and safety evidence."""
    config, occupancy, mask, resolution_m, origin = _load_route_context(
        route_yaml)
    scale = 1.25
    top_px = ANIMATION_TOP_PX
    bottom_px = ANIMATION_BOTTOM_PX
    height, width = occupancy.shape
    map_image = Image.fromarray(occupancy, mode='L').convert('RGB')
    mask_rgba = np.zeros((height, width, 4), dtype=np.uint8)
    mask_rgba[mask < 100] = (239, 68, 68, 125)
    map_image = Image.alpha_composite(
        map_image.convert('RGBA'), Image.fromarray(mask_rgba, mode='RGBA'))
    map_image = map_image.resize(
        (round(width * scale), round(height * scale)),
        Image.Resampling.NEAREST)
    canvas_size = (map_image.width, map_image.height + top_px + bottom_px)
    base = Image.new('RGB', canvas_size, '#07111f')
    base.paste(map_image.convert('RGB'), (0, top_px))
    base_draw = ImageDraw.Draw(base)
    planned = [(config['start_pose']['x'], config['start_pose']['y'])]
    planned.extend((entry['x'], entry['y']) for entry in config['waypoints'])
    planned_pixels = [
        _world_to_pixel(x_m, y_m, resolution_m=resolution_m,
                        origin=origin, height=height, scale=scale,
                        top_px=top_px)
        for x_m, y_m in planned
    ]
    route_deviations_m = [
        _point_to_polyline_distance(
            (entry['x_m'], entry['y_m']), planned)
        for entry in series['amcl']
    ]
    base_draw.line(planned_pixels, fill='#2563eb', width=3)
    for point in planned_pixels[1:]:
        base_draw.ellipse((point[0] - 3, point[1] - 3,
                          point[0] + 3, point[1] + 3), fill='#2563eb')

    samples = series['amcl']
    sample_stamps_ns = [entry['stamp_ns'] for entry in samples]
    scans = series.get('scans', [])
    collision_events = series.get('collision_states', [])
    laser_pose = series.get('laser_pose')
    start_ns = metrics['capture']['drive_start_unix_ns']
    end_ns = metrics['capture']['drive_end_unix_ns']
    deviation_episodes = _route_deviation_episodes(
        samples, route_deviations_m)
    associated_deviation_flags = [False] * len(samples)
    associated_episodes = []
    if laser_pose is not None:
        for episode in deviation_episodes:
            best_observation = None
            observation_stamps_ns = [
                samples[sample_index]['stamp_ns']
                for sample_index in range(
                    episode['start_index'], episode['end_index'] + 1)
            ]
            observation_stamps_ns.extend(
                collision['stamp_ns'] for collision in collision_events
                if collision['action_name'] == 'STOP'
                and collision['polygon_name'] == 'StopZone'
                and (episode['start_stamp_ns'] - DEVIATION_OBSTACLE_LEAD_NS
                     <= collision['stamp_ns'] <= episode['end_stamp_ns']))
            for observation_stamp_ns in observation_stamps_ns:
                scan = _nearest_by_stamp(
                    scans, observation_stamp_ns, max_gap_ns=1_000_000_000)
                if scan is None:
                    continue
                projected = _project_scan_points(
                    scan, scan['map_base_pose'], laser_pose, occupancy,
                    resolution_m, origin)
                clusters = [
                    cluster for cluster in _cluster_points(
                        projected['obstacle_candidates'])
                    if (max(point[0] for point in cluster)
                        - min(point[0] for point in cluster)) <= 1.5
                    and (max(point[1] for point in cluster)
                         - min(point[1] for point in cluster)) <= 1.5
                ]
                robot_xy_m = scan['map_base_pose'][:2]
                cluster = _select_path_obstacle_cluster(
                    clusters, planned, robot_xy_m)
                if cluster is None:
                    continue
                route_gap_m = min(
                    _point_to_polyline_distance(point, planned)
                    for point in cluster)
                robot_gap_m = min(
                    math.dist(point, robot_xy_m) for point in cluster)
                score = (route_gap_m, robot_gap_m, -len(cluster))
                if best_observation is None or score < best_observation[0]:
                    best_observation = (score, cluster)
            if best_observation is None:
                continue
            score, obstacle_cluster = best_observation
            episode = {
                **episode,
                'obstacle_cluster': obstacle_cluster,
                'route_gap_m': score[0],
                'robot_gap_m': score[1],
            }
            associated_episodes.append(episode)
            for sample_index in range(
                    episode['start_index'], episode['end_index'] + 1):
                associated_deviation_flags[sample_index] = True
    for sequence, episode in enumerate(associated_episodes, start=1):
        episode['sequence'] = sequence
    playback_frames = round(frames / playback_speed)
    stop_hold_frames = round(fps / playback_speed)
    uniform_stamps_ns = list(np.linspace(
        start_ns, end_ns, playback_frames, dtype=np.int64))
    stop_stamps_ns = [
        event['stamp_ns'] for event in collision_events
        if event['action_name'] == 'STOP'
        and event['polygon_name'] == 'StopZone'
    ]
    frame_stamps_ns = sorted(
        uniform_stamps_ns
        + [stamp_ns for stamp_ns in stop_stamps_ns
           for _ in range(max(1, stop_hold_frames))])
    half_frame_ns = max(
        1, round((end_ns - start_ns) / (playback_frames - 1) / 2))
    title_font = _pil_font(22, bold=True)
    detail_font = _pil_font(17, bold=True)
    label_font = _pil_font(15, bold=True)
    note_font = _pil_font(13)
    metric_label_font = _pil_font(11, bold=True)
    frames_out = []
    for frame_stamp_ns in frame_stamps_ns:
        frame = base.copy()
        draw = ImageDraw.Draw(frame)
        collision_event = _select_frame_collision_event(
            collision_events, int(frame_stamp_ns), half_frame_ns)
        is_safety_stop = (
            collision_event is not None
            and collision_event['action_name'] == 'STOP'
            and collision_event['polygon_name'] == 'StopZone')
        evidence_stamp_ns = (
            collision_event['stamp_ns']
            if collision_event is not None else int(frame_stamp_ns))
        stop = max(1, bisect.bisect_right(sample_stamps_ns,
                                          evidence_stamp_ns))
        active_episode = next((
            episode for episode in associated_episodes
            if episode['start_stamp_ns'] - DEVIATION_OBSTACLE_LEAD_NS
            <= evidence_stamp_ns
            <= episode['end_stamp_ns'] + 3_000_000_000
        ), None)
        drive_state = '기준 경로 추종'
        if active_episode is not None:
            if evidence_stamp_ns < active_episode['start_stamp_ns']:
                drive_state = '기준 경로상 장애물 감지'
            elif evidence_stamp_ns <= active_episode['end_stamp_ns']:
                drive_state = '장애물 회피 선회'
            else:
                drive_state = '기준 경로 복귀'
        if is_safety_stop:
            drive_state = '충돌 위험 · 자동 정지'

        selected_cluster = (
            active_episode['obstacle_cluster']
            if active_episode is not None else [])
        for x_m, y_m in selected_cluster:
            x_point, y_point = _world_to_pixel(
                x_m, y_m, resolution_m=resolution_m, origin=origin,
                height=height, scale=scale, top_px=top_px)
            draw.ellipse((x_point - 3, y_point - 3,
                          x_point + 3, y_point + 3),
                         fill='#e11d48', outline='#fff1f2', width=1)
        if selected_cluster:
            cluster_pixels = [
                _world_to_pixel(
                    x_m, y_m, resolution_m=resolution_m,
                    origin=origin, height=height, scale=scale,
                    top_px=top_px)
                for x_m, y_m in selected_cluster
            ]
            left = min(point[0] for point in cluster_pixels) - 5
            top = min(point[1] for point in cluster_pixels) - 5
            right = max(point[0] for point in cluster_pixels) + 5
            bottom = max(point[1] for point in cluster_pixels) + 5
            draw.rectangle(
                (left, top, right, bottom), outline='#be123c', width=3)
            obstacle_label = (
                '긴급 정지 대상' if is_safety_stop else '장애물')
            label_top = (
                bottom + 4 if is_safety_stop
                else max(top_px + 2, top - 18))
            draw.text((left + 4, label_top),
                      obstacle_label, font=note_font,
                      fill='#be123c', stroke_width=2,
                      stroke_fill='#fff1f2')

        travelled = [
            _world_to_pixel(entry['x_m'], entry['y_m'],
                            resolution_m=resolution_m, origin=origin,
                            height=height, scale=scale, top_px=top_px)
            for entry in samples[:stop]
        ]
        if len(travelled) > 1:
            for index, (start, end) in enumerate(
                    zip(travelled, travelled[1:]), start=1):
                in_active_detour = (
                    active_episode is not None
                    and active_episode['start_index'] <= index
                    <= active_episode['end_index'])
                color = (
                    '#9333ea'
                    if associated_deviation_flags[index] or in_active_detour
                    else '#16a34a')
                draw.line((start, end), fill=color, width=5)
        if active_episode is not None:
            active_end_index = stop - 1
            active_detour = [
                _world_to_pixel(
                    samples[index]['x_m'], samples[index]['y_m'],
                    resolution_m=resolution_m, origin=origin,
                    height=height, scale=scale, top_px=top_px)
                for index in range(
                    active_episode['start_index'], active_end_index + 1)
            ]
            if len(active_detour) > 1:
                draw.line(active_detour, fill='#9333ea', width=5)
        x_px, y_px = travelled[-1]
        draw.ellipse((x_px - 8, y_px - 8, x_px + 8, y_px + 8),
                     fill='#fbbf24', outline='#111827', width=3)
        stop_duration_s = None
        same_goal_resumed = False
        if collision_event is not None:
            action_name = collision_event['action_name']
            polygon_name = collision_event['polygon_name']
            resume_episode = next((
                episode for episode in metrics['navigation_events'][
                    'same_goal_resume_evidence']['episodes']
                if episode['stop_stamp_ns'] == collision_event['stamp_ns']
            ), None)
            stop_duration_s = (
                (resume_episode['clear_stamp_ns']
                 - resume_episode['stop_stamp_ns']) / 1e9
                if resume_episode is not None else None)
            same_goal_resumed = resume_episode is not None
            badge = _collision_badge(
                action_name, polygon_name, stop_duration_s)
            badge_box = draw.textbbox((0, 0), badge, font=label_font)
            badge_width = badge_box[2] - badge_box[0] + 20
            badge_left = min(canvas_size[0] - badge_width - 12, x_px + 14)
            badge_top = max(top_px + 8, y_px - 42)
            draw.rounded_rectangle(
                (badge_left, badge_top, badge_left + badge_width,
                 badge_top + 31), radius=8, fill='#be123c')
            draw.text((badge_left + 10, badge_top + 5), badge,
                      font=label_font, fill='white')
        elapsed_s = (evidence_stamp_ns - start_ns) / 1e9
        progress_pct = min(100.0, elapsed_s / metrics['capture'][
            'drive_duration_s'] * 100.0)
        draw.text((18, 8), '실차 SLAM 장애물 대응 관제',
                  font=title_font, fill='#f8fafc')
        run_text = f"RUN  {metrics.get('run_id', 'EVIDENCE PREVIEW')}"
        run_box = draw.textbbox((0, 0), run_text, font=note_font)
        draw.text((canvas_size[0] - (run_box[2] - run_box[0]) - 18, 13),
                  run_text, font=note_font, fill='#94a3b8')
        current_deviation_m = route_deviations_m[stop - 1]
        episode_text = (
            f"{active_episode['sequence']:02d}/"
            f'{len(associated_episodes):02d}'
            if active_episode is not None else '—')
        route_gap_text = (
            f"{active_episode['route_gap_m']:.2f}m"
            if active_episode is not None else '—')
        intervention_label = '회피 구간'
        intervention_text = episode_text
        if is_safety_stop:
            intervention_label = '안전 개입'
            duration_text = (
                f'{stop_duration_s:.2f}초' if stop_duration_s is not None
                else '정지')
            resume_text = (
                '목표 재개' if same_goal_resumed else '재개 확인 중')
            intervention_text = f'{duration_text} · {resume_text}'
        cards = (
            ('주행 시간', f'{elapsed_s:5.1f}s  ·  {progress_pct:4.1f}%'),
            ('주행 상태', drive_state),
            ('경로 이탈', f'{current_deviation_m:.2f} m'),
            ('라이다 근거',
             f'{len(selected_cluster)}점 · 경로 {route_gap_text}'),
            (intervention_label, intervention_text),
        )
        card_gap = 8
        card_left = 18
        card_width = (
            canvas_size[0] - card_left * 2 - card_gap * 4) // 5
        for index, (label, value) in enumerate(cards):
            left = card_left + index * (card_width + card_gap)
            draw.rounded_rectangle(
                (left, 42, left + card_width, 104), radius=7,
                fill='#111f33', outline='#263b55', width=1)
            draw.text((left + 10, 48), label, font=metric_label_font,
                      fill='#7dd3fc')
            value_color = (
                '#fda4af' if '정지' in value else
                '#c4b5fd' if label == '경로 이탈' else '#f8fafc')
            draw.text((left + 10, 69), value, font=detail_font,
                      fill=value_color)
        draw.text((18, 119),
                  '파랑 원래 주행 경로  ·  초록 실제 주행  ·  '
                  '보라 장애물 회피 선회  ·  빨강 장애물',
                  font=note_font, fill='#cbd5e1')
        bar_left = 24
        bar_right = canvas_size[0] - 24
        bar_top = canvas_size[1] - 29
        draw.rounded_rectangle((bar_left, bar_top, bar_right, bar_top + 12),
                               radius=6, fill='#cbd5e1')
        draw.rounded_rectangle(
            (bar_left, bar_top,
             bar_left + round((bar_right - bar_left) * progress_pct / 100),
             bar_top + 12), radius=6, fill='#16a34a')
        frames_out.append(frame)
    output.parent.mkdir(parents=True, exist_ok=True)
    frames_out[0].save(
        output, save_all=True, append_images=frames_out[1:],
        duration=round(1000 / fps), loop=0, optimize=False, disposal=1)


def _stop_event_records(route_yaml: Path, metrics: dict[str, Any],
                        series: dict[str, Any]) -> list[dict[str, Any]]:
    """Build evidence rows for real StopZone episodes only."""
    _config, occupancy, _mask, resolution_m, origin = _load_route_context(
        route_yaml)
    episodes = {
        episode['stop_stamp_ns']: episode
        for episode in metrics['navigation_events'][
            'same_goal_resume_evidence']['episodes']
    }
    records = []
    for event in series.get('collision_states', []):
        if (event['action_name'] != 'STOP'
                or event['polygon_name'] != 'StopZone'):
            continue
        episode = episodes.get(event['stamp_ns'])
        scan = _nearest_by_stamp(
            series.get('scans', []), event['stamp_ns'],
            max_gap_ns=1_000_000_000)
        projected = {'static': [], 'obstacle_candidates': []}
        if scan is not None and series.get('laser_pose') is not None:
            projected = _project_scan_points(
                scan, scan['map_base_pose'], series['laser_pose'], occupancy,
                resolution_m, origin)
        records.append({
            'event': len(records) + 1,
            'stamp_ns': event['stamp_ns'],
            'elapsed_s': round((event['stamp_ns'] - metrics['capture'][
                'drive_start_unix_ns']) / 1e9, 3),
            'stop_duration_s': (
                round((episode['clear_stamp_ns'] - episode['stop_stamp_ns'])
                      / 1e9, 3) if episode is not None else None),
            'zero_command_delay_ms': (
                round((episode['zero_command_stamp_ns']
                       - episode['stop_stamp_ns']) / 1e6, 3)
                if episode is not None else None),
            'resume_after_clear_ms': (
                round((episode['resume_command_stamp_ns']
                       - episode['clear_stamp_ns']) / 1e6, 3)
                if episode is not None else None),
            'x_m': (round(scan['map_base_pose'][0], 3)
                    if scan is not None else None),
            'y_m': (round(scan['map_base_pose'][1], 3)
                    if scan is not None else None),
            'lidar_static_returns': len(projected['static']),
            'lidar_obstacle_candidates': len(
                projected['obstacle_candidates']),
            'same_goal_resumed': (
                episode['same_goal_resumed']
                if episode is not None else None),
            'terminal_succeeded': (
                episode['terminal_succeeded']
                if episode is not None else None),
            'object_classification': 'NOT_MEASURED',
        })
    return records


def write_stop_event_csv(records: list[dict[str, Any]], output: Path) -> None:
    """Write one auditable row per measured StopZone episode."""
    fields = (
        'event', 'elapsed_s', 'stop_duration_s', 'zero_command_delay_ms',
        'resume_after_clear_ms', 'x_m', 'y_m', 'lidar_static_returns',
        'lidar_obstacle_candidates', 'same_goal_resumed',
        'terminal_succeeded', 'object_classification',
    )
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        writer.writerows({field: record[field] for field in fields}
                         for record in records)


def render_stop_event_summary(route_yaml: Path, metrics: dict[str, Any],
                              series: dict[str, Any],
                              records: list[dict[str, Any]],
                              output: Path) -> None:
    """Render every measured stop with position and command evidence."""
    _config, occupancy, mask, resolution_m, origin = _load_route_context(
        route_yaml)
    scale = 1.25
    height, width = occupancy.shape
    map_image = Image.fromarray(occupancy, mode='L').convert('RGBA')
    mask_rgba = np.zeros((height, width, 4), dtype=np.uint8)
    mask_rgba[mask < 100] = (239, 68, 68, 125)
    map_image = Image.alpha_composite(
        map_image, Image.fromarray(mask_rgba, mode='RGBA')).convert('RGB')
    map_image = map_image.resize(
        (round(width * scale), round(height * scale)),
        Image.Resampling.NEAREST)
    panel_width = 690
    canvas_height = max(640, map_image.height + 120)
    map_top = (canvas_height - map_image.height) // 2
    canvas = Image.new(
        'RGB', (map_image.width + panel_width, canvas_height), '#f8fafc')
    canvas.paste(map_image, (0, map_top))
    draw = ImageDraw.Draw(canvas)
    path_pixels = [
        _world_to_pixel(
            entry['x_m'], entry['y_m'], resolution_m=resolution_m,
            origin=origin, height=height, scale=scale, top_px=map_top)
        for entry in series['amcl']
    ]
    if len(path_pixels) > 1:
        draw.line(path_pixels, fill='#16a34a', width=4)
    marker_font = _pil_font(13, bold=True)
    marker_groups: list[dict[str, Any]] = []
    for record in records:
        if record['x_m'] is None or record['y_m'] is None:
            continue
        group = next((
            candidate for candidate in marker_groups
            if math.dist(
                (candidate['x_m'], candidate['y_m']),
                (record['x_m'], record['y_m'])) < 0.25
        ), None)
        if group is None:
            marker_groups.append({
                'x_m': record['x_m'], 'y_m': record['y_m'],
                'events': [record['event']],
            })
        else:
            group['events'].append(record['event'])
    for group in marker_groups:
        x_px, y_px = _world_to_pixel(
            group['x_m'], group['y_m'], resolution_m=resolution_m,
            origin=origin, height=height, scale=scale, top_px=map_top)
        label = '·'.join(str(event) for event in group['events'])
        box = draw.textbbox((0, 0), label, font=marker_font)
        marker_width = max(28, box[2] - box[0] + 14)
        draw.rounded_rectangle(
            (x_px - marker_width / 2, y_px - 14,
             x_px + marker_width / 2, y_px + 14),
            radius=14, fill='#be123c', outline='white', width=2)
        draw.text((x_px - (box[2] - box[0]) / 2,
                   y_px - (box[3] - box[1]) / 2 - 1),
                  label, font=marker_font, fill='white')

    panel_left = map_image.width + 28
    title_font = _pil_font(27, bold=True)
    detail_font = _pil_font(17)
    small_font = _pil_font(14)
    draw.text((panel_left, 24),
              f'Collision Monitor 정지 {len(records)}회',
              font=title_font, fill='#0f172a')
    draw.text((panel_left, 66),
              '/scan → StopZone → cmd_vel=0 → 동일 목표 재개',
              font=detail_font, fill='#334155')
    row_top = 112
    for record in records:
        duration = record['stop_duration_s']
        candidate_count = record['lidar_obstacle_candidates']
        location = (
            f"({record['x_m']:.2f}, {record['y_m']:.2f})m"
            if record['x_m'] is not None else 'UNKNOWN')
        duration_text = (
            f'{duration:.3f}s' if duration is not None else 'UNKNOWN')
        zero_delay = (
            f"{record['zero_command_delay_ms']:.1f}ms"
            if record['zero_command_delay_ms'] is not None else 'UNKNOWN')
        resume_delay = (
            f"{record['resume_after_clear_ms']:.1f}ms"
            if record['resume_after_clear_ms'] is not None else 'UNKNOWN')
        line1 = (
            f"#{record['event']}  {record['elapsed_s']:.1f}s 지점 · "
            f'STOP {duration_text} · 위치 {location}')
        line2 = (
            f'라이다 후보 {candidate_count}점 · 정지명령 '
            f'{zero_delay} · 해제 후 재출발 {resume_delay} · '
            '동일 목표 PASS')
        draw.text((panel_left, row_top), line1,
                  font=detail_font, fill='#9f1239')
        draw.text((panel_left + 18, row_top + 27), line2,
                  font=small_font, fill='#475569')
        row_top += 67
    draw.rounded_rectangle(
        (panel_left, canvas_height - 92, canvas.width - 24,
         canvas_height - 22), radius=10, fill='#fff1f2')
    draw.text((panel_left + 16, canvas_height - 80),
              '판정: 정지는 라이다 입력 기반 StopZone 동작으로 확인됨.',
              font=small_font, fill='#881337')
    draw.text((panel_left + 16, canvas_height - 52),
              '사람/상자 구분은 불가: 카메라·객체 검출 토픽이 기록되지 않음.',
              font=small_font, fill='#881337')
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)


def render_card(metrics: dict[str, Any], output: Path) -> None:
    """Render a compact project card for README and portfolio thumbnails."""
    navigation = metrics['navigation']
    capture = metrics['capture']
    continuity = metrics['continuity']
    sent_waypoints = navigation['sent_waypoints']
    declared_waypoints = navigation['declared_waypoints']
    drive_minutes = capture['drive_duration_s'] / 60
    max_recoveries = navigation['max_recoveries']
    preflight_length_m = navigation['preflight']['planned_path_length_m']
    amcl_length_m = navigation['amcl_path_length_m']
    return_error_m = navigation['start_to_end_amcl_m']
    scan_gap_s = continuity['/scan']['max_gap_s']
    total_messages = capture['total_messages']
    figure = plt.figure(figsize=(12, 6.3), dpi=100, facecolor='#08111f')
    axes = figure.add_axes((0, 0, 1, 1))
    axes.axis('off')
    axes.text(0.06, 0.86, 'JD-AMR · SAVED-MAP AUTONOMY',
              color='#5eead4', fontsize=15, weight='bold')
    axes.text(0.06, 0.69, '복도 왕복 자율주행 완주',
              color='white', fontsize=36, weight='bold')
    axes.text(0.06, 0.57,
              f'{sent_waypoints}/{declared_waypoints} waypoints · '
              f'{drive_minutes:.2f} min · recoveries {max_recoveries}',
              color='#cbd5e1', fontsize=17)
    cards = [
        ('NAV2 PREFLIGHT', f'{preflight_length_m:.3f} m'),
        ('AMCL PATH', f'{amcl_length_m:.3f} m'),
        ('RETURN ERROR', f'{return_error_m:.3f} m'),
        ('SCAN MAX GAP', f'{scan_gap_s:.3f} s'),
    ]
    for index, (label, value) in enumerate(cards):
        left = 0.06 + index * 0.23
        axes.add_patch(plt.Rectangle((left, 0.20), 0.20, 0.23,
                                     facecolor='#111c2e', edgecolor='#26364d',
                                     linewidth=1.2))
        axes.text(left + 0.018, 0.365, label, color='#94a3b8', fontsize=10,
                  weight='bold')
        axes.text(left + 0.018, 0.265, value, color='white', fontsize=22,
                  weight='bold')
    axes.text(0.06, 0.08,
              f'MCAP {total_messages:,} messages · '
              'CRC / summary / indexes PASS · keepout active',
              color='#94a3b8', fontsize=12)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, facecolor=figure.get_facecolor())
    plt.close(figure)


def write_csv(series: dict[str, Any], metrics: dict[str, Any],
              output_dir: Path) -> None:
    """Write reusable trajectory and progress tables beside the media."""
    start_ns = metrics['capture']['drive_start_unix_ns']
    with (output_dir / 'amcl_trajectory.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        fields = ('elapsed_s', 'x_m', 'y_m', 'x_covariance_m2',
                  'y_covariance_m2', 'yaw_covariance_rad2')
        writer = csv.DictWriter(
            stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        for entry in series['amcl']:
            writer.writerow({
                'elapsed_s': round((entry['stamp_ns'] - start_ns) / 1e9, 6),
                **{field: entry[field] for field in fields[1:]},
            })
    with (output_dir / 'route_progress.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        fields = ('elapsed_s', 'waypoint', 'total', 'remaining_m',
                  'recoveries', 'battery_v')
        writer = csv.DictWriter(
            stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        for entry in series['statuses']:
            writer.writerow({
                'elapsed_s': round((entry['stamp_ns'] - start_ns) / 1e9, 6),
                **{field: entry[field] for field in fields[1:]},
            })


def write_route_deviation_csv(route_yaml: Path, series: dict[str, Any],
                              metrics: dict[str, Any], output: Path) -> None:
    """Write measured cross-track distance from the waypoint polyline."""
    config, _occupancy, _mask, _resolution_m, _origin = _load_route_context(
        route_yaml)
    planned = [(config['start_pose']['x'], config['start_pose']['y'])]
    planned.extend((entry['x'], entry['y']) for entry in config['waypoints'])
    start_ns = metrics['capture']['drive_start_unix_ns']
    fields = ('elapsed_s', 'x_m', 'y_m', 'route_deviation_m', 'highlighted')
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        for entry in series['amcl']:
            deviation_m = _point_to_polyline_distance(
                (entry['x_m'], entry['y_m']), planned)
            writer.writerow({
                'elapsed_s': round((entry['stamp_ns'] - start_ns) / 1e9, 6),
                'x_m': entry['x_m'],
                'y_m': entry['y_m'],
                'route_deviation_m': round(deviation_m, 6),
                'highlighted': deviation_m >= DEVIATION_HIGHLIGHT_M,
            })


def write_comparison_csv(records: list[dict[str, Any]],
                         output: Path) -> None:
    """Write the plotted control-path continuity values as a table."""
    fields = ('run_id', 'stream', 'topic', 'messages', 'rate_hz',
              'max_gap_s', 'p99_gap_s')
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        for record in records:
            for label, topic in CONTINUITY_SERIES.items():
                continuity = record['continuity'][topic]
                writer.writerow({
                    'run_id': record['run_id'],
                    'stream': label,
                    'topic': topic,
                    **continuity,
                })


def _render_mp4(gif_path: Path, mp4_path: Path, fps: int) -> bool:
    """Convert the generated GIF to a browser-friendly H.264 video."""
    ffmpeg = shutil.which('ffmpeg')
    if ffmpeg is None:
        return False
    result = subprocess.run(
        [ffmpeg, '-y', '-loglevel', 'error', '-i', str(gif_path),
         '-vf', f'fps={fps},scale=trunc(iw/2)*2:trunc(ih/2)*2',
         '-movflags', '+faststart', '-pix_fmt', 'yuv420p', str(mp4_path)],
        check=False)
    return result.returncode == 0


def write_media_manifest(output_dir: Path,
                         metrics: dict[str, Any]) -> Path:
    """Write hashes for every generated data and media artifact."""
    output = output_dir / 'media_manifest.yaml'
    files = []
    for path in sorted(output_dir.iterdir()):
        if not path.is_file() or path == output:
            continue
        files.append({
            'file': path.name,
            'size_bytes': path.stat().st_size,
            'sha256': _sha256(path),
        })
    document = {
        'schema_version': 1,
        'run_id': metrics['run_id'],
        'source_mcap_sha256': metrics['source']['mcap_sha256'],
        'source_route_log_sha256': metrics['source']['route_log_sha256'],
        'files': files,
    }
    output.write_text(
        yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
        encoding='utf-8')
    return output


def main(argv=None) -> int:
    """Generate metrics and portfolio media for one recorded run."""
    package_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--compare-run-dir', type=Path, action='append',
                        default=[])
    parser.add_argument('--route', type=Path, default=(
        package_root / 'config' /
        'corridor_roundtrip.autonomous_20260826.yaml'))
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument(
        '--metrics-only', action='store_true',
        help='Write metrics and CSV only, without rendering images or videos')
    parser.add_argument(
        '--frames', type=int, default=DEFAULT_ANIMATION_FRAMES)
    parser.add_argument('--fps', type=int, default=DEFAULT_ANIMATION_FPS)
    parser.add_argument(
        '--playback-speed', type=float,
        default=DEFAULT_ANIMATION_PLAYBACK_SPEED,
        help='Playback multiplier; 0.75 is 25%% slower than real rendering')
    args = parser.parse_args(argv)
    if args.frames < 2 or args.fps < 1 or args.playback_speed <= 0.0:
        parser.error(
            '--frames must be >= 2, --fps >= 1, and --playback-speed > 0')

    if args.metrics_only and args.compare_run_dir:
        parser.error(
            '--metrics-only cannot be combined with --compare-run-dir')
    if args.metrics_only and args.output_dir.exists() and any(
            args.output_dir.iterdir()):
        parser.error('--metrics-only requires a new or empty output directory')
    if not args.metrics_only:
        _configure_plot_font()
    metrics, series = analyse_run(args.run_dir)
    comparison_metrics = [analyse_run(path)[0]
                          for path in args.compare_run_dir]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = args.output_dir / 'metrics.yaml'
    metrics_path.write_text(
        yaml.safe_dump(metrics, allow_unicode=True, sort_keys=False),
        encoding='utf-8')
    (args.output_dir / 'metrics.json').write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding='utf-8')
    write_csv(series, metrics, args.output_dir)
    write_route_deviation_csv(
        args.route, series, metrics,
        args.output_dir / 'route_deviation.csv')
    if args.metrics_only:
        write_media_manifest(args.output_dir, metrics)
        print(json.dumps({
            'run_id': metrics['run_id'],
            'outcome': metrics['outcome'],
            'output_dir': str(args.output_dir.resolve()),
            'outputs': sorted(path.name for path in args.output_dir.iterdir()),
            'mp4_created': False,
            'metrics_only': True,
        }, ensure_ascii=False, indent=2))
        return 0
    render_route(args.route, metrics, series,
                 args.output_dir / 'route_evidence.png')
    render_telemetry(metrics, series,
                     args.output_dir / 'telemetry.png')
    stop_records = _stop_event_records(args.route, metrics, series)
    write_stop_event_csv(
        stop_records, args.output_dir / 'collision_stop_events.csv')
    render_stop_event_summary(
        args.route, metrics, series, stop_records,
        args.output_dir / 'collision_stop_events.png')
    render_card(metrics, args.output_dir / 'success_card.png')
    if comparison_metrics:
        render_continuity(metrics, comparison_metrics,
                          args.output_dir / 'continuity_comparison.png')
        write_comparison_csv(
            comparison_metrics + [metrics],
            args.output_dir / 'continuity_comparison.csv')
    gif_path = args.output_dir / 'corridor_roundtrip.gif'
    render_animation(args.route, metrics, series, gif_path,
                     args.frames, args.fps, args.playback_speed)
    mp4_path = args.output_dir / 'corridor_roundtrip.mp4'
    mp4_created = _render_mp4(gif_path, mp4_path, args.fps)
    write_media_manifest(args.output_dir, metrics)
    outputs = sorted(path.name for path in args.output_dir.iterdir())
    print(json.dumps({
        'run_id': metrics['run_id'],
        'outcome': metrics['outcome'],
        'output_dir': str(args.output_dir.resolve()),
        'outputs': outputs,
        'mp4_created': mp4_created,
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
