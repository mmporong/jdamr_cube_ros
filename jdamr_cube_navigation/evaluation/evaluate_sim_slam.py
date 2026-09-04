#!/usr/bin/env python3
"""Measure a simulated 2D SLAM trajectory against Gazebo model pose."""

from __future__ import annotations

import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Sequence

from compare_slam_runs import (
    apply_rigid,
    compose,
    estimated_trajectory,
    path_length,
    read_map_to_odom,
    read_odometry,
    rigid_align,
    yaw_of,
)
from mcap_ros2.reader import read_ros2_messages


Pose2 = tuple[float, float, float, float]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def wrap_yaw(yaw_rad: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (yaw_rad + math.pi) % math.tau - math.pi


def inverse(pose: Sequence[float]) -> tuple[float, float, float]:
    """Invert a planar (x, y, yaw) transform."""
    x_m, y_m, yaw_rad = pose
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        -cos_yaw * x_m - sin_yaw * y_m,
        sin_yaw * x_m - cos_yaw * y_m,
        -yaw_rad,
    )


def relative(first: Sequence[float], second: Sequence[float]):
    """Return the planar motion from *first* to *second*."""
    return compose(inverse(first), second)


def read_ground_truth(bag: Path) -> list[Pose2]:
    """Read independent Gazebo model poses as (t, x, y, yaw)."""
    samples = []
    for message in read_ros2_messages(
            str(bag), topics=['/ground_truth_pose']):
        pose_stamped = message.ros_msg
        stamp = pose_stamped.header.stamp
        samples.append((
            stamp.sec + stamp.nanosec * 1e-9,
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            yaw_of(pose_stamped.pose.orientation),
        ))
    samples.sort()
    return samples


def time_match(
        estimated: Sequence[Pose2],
        truth: Sequence[Pose2],
        *,
        max_offset_s: float = 0.075) -> list[tuple[Pose2, Pose2]]:
    """Pair each estimate with the closest ground-truth timestamp."""
    if not truth:
        return []
    truth_times = [sample[0] for sample in truth]
    pairs = []
    for estimate in estimated:
        index = bisect_left(truth_times, estimate[0])
        candidates = truth[max(0, index - 1):index + 1]
        closest = min(
            candidates,
            key=lambda sample: abs(sample[0] - estimate[0]))
        if abs(closest[0] - estimate[0]) <= max_offset_s:
            pairs.append((estimate, closest))
    return pairs


def _root_mean_square(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def absolute_trajectory_error(
        pairs: Sequence[tuple[Pose2, Pose2]]) -> dict[str, Any]:
    """Return SE(2)-aligned translational and yaw ATE."""
    if len(pairs) < 2:
        raise ValueError('ATE requires at least two timestamp pairs')
    estimated_xy = [(estimate[1], estimate[2])
                    for estimate, _ in pairs]
    truth_xy = [(truth[1], truth[2]) for _, truth in pairs]
    alignment = rigid_align(estimated_xy, truth_xy)
    if alignment is None:
        raise ValueError('ATE alignment is degenerate')
    translation_errors_m = [
        math.dist(apply_rigid(alignment, estimate[1], estimate[2]),
                  (truth[1], truth[2]))
        for estimate, truth in pairs
    ]
    yaw_errors_rad = [
        abs(wrap_yaw(estimate[3] + alignment[0] - truth[3]))
        for estimate, truth in pairs
    ]
    return {
        'samples': len(pairs),
        'alignment_yaw_rad': alignment[0],
        'translation_rms_m': _root_mean_square(translation_errors_m),
        'translation_max_m': max(translation_errors_m),
        'yaw_rms_rad': _root_mean_square(yaw_errors_rad),
        'yaw_max_rad': max(yaw_errors_rad),
    }


def relative_pose_error(
        pairs: Sequence[tuple[Pose2, Pose2]],
        *,
        delta_s: float = 1.0,
        tolerance_s: float = 0.075) -> dict[str, Any]:
    """Return fixed-time-horizon translational and yaw RPE."""
    if len(pairs) < 2:
        raise ValueError('RPE requires at least two timestamp pairs')
    pair_times = [pair[0][0] for pair in pairs]
    translation_errors_m = []
    yaw_errors_rad = []
    for index, (estimate_a, truth_a) in enumerate(pairs):
        target_time = estimate_a[0] + delta_s
        candidate_index = bisect_left(pair_times, target_time)
        candidates = [candidate for candidate in (
            candidate_index - 1, candidate_index)
            if index < candidate < len(pairs)]
        if not candidates:
            continue
        end_index = min(
            candidates,
            key=lambda candidate: abs(pair_times[candidate] - target_time))
        if abs(pair_times[end_index] - target_time) > tolerance_s:
            continue
        estimate_b, truth_b = pairs[end_index]
        estimated_motion = relative(estimate_a[1:], estimate_b[1:])
        truth_motion = relative(truth_a[1:], truth_b[1:])
        error = relative(truth_motion, estimated_motion)
        translation_errors_m.append(math.hypot(error[0], error[1]))
        yaw_errors_rad.append(abs(wrap_yaw(error[2])))
    if not translation_errors_m:
        raise ValueError('RPE has no pairs at the requested time horizon')
    return {
        'samples': len(translation_errors_m),
        'delta_s': delta_s,
        'translation_rms_m': _root_mean_square(translation_errors_m),
        'translation_max_m': max(translation_errors_m),
        'yaw_rms_rad': _root_mean_square(yaw_errors_rad),
        'yaw_max_rad': max(yaw_errors_rad),
    }


def analyse(bag: Path, backend: str) -> dict[str, Any]:
    """Analyse one finalized simulation result bag."""
    map_to_odom = read_map_to_odom(bag)
    odometry = read_odometry(bag)
    truth = read_ground_truth(bag)
    estimated = estimated_trajectory(map_to_odom, odometry)
    pairs = time_match(estimated, truth)
    odometry_pairs = time_match(odometry, truth)
    if not map_to_odom:
        raise ValueError('bag has no map->odom SLAM transforms')
    if not odometry:
        raise ValueError('bag has no wheel odometry')
    if not truth:
        raise ValueError('bag has no Gazebo ground-truth pose')
    if len(pairs) < 2:
        raise ValueError('not enough synchronized estimate/truth pairs')
    if len(odometry_pairs) < 2:
        raise ValueError('not enough synchronized odometry/truth pairs')
    slam_ate = absolute_trajectory_error(pairs)
    odometry_ate = absolute_trajectory_error(odometry_pairs)
    return {
        'schema_version': 1,
        'backend': backend,
        'source': {
            'bag': str(bag.resolve()),
            'sha256': _sha256(bag),
        },
        'ground_truth': {
            'source': 'gazebo_model_pose',
            'topic': '/ground_truth_pose',
            'independent_of_wheel_odometry': True,
            'samples': len(truth),
            'path_length_m': path_length(truth),
        },
        'slam': {
            'map_to_odom_samples': len(map_to_odom),
            'estimated_pose_samples': len(estimated),
            'path_length_m': path_length(estimated),
        },
        'synchronization': {
            'max_offset_s': 0.075,
            'matched_samples': len(pairs),
        },
        'ate': slam_ate,
        'rpe': relative_pose_error(pairs),
        'wheel_odometry_reference': {
            'path_length_m': path_length(odometry),
            'ate': odometry_ate,
            'rpe': relative_pose_error(odometry_pairs),
        },
        'comparison': {
            'slam_to_wheel_ate_rms_ratio': (
                slam_ate['translation_rms_m']
                / odometry_ate['translation_rms_m']
                if odometry_ate['translation_rms_m'] > 0.0 else None),
            'slam_improves_wheel_translation_ate': (
                slam_ate['translation_rms_m']
                < odometry_ate['translation_rms_m']),
        },
        'method': {
            'alignment': 'SE(2) rigid transform without scale',
            'rpe_delta_s': 1.0,
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--backend', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if not args.bag.is_file():
        parser.error('--bag must be one finalized MCAP file')
    if args.output.exists():
        parser.error('--output must not already exist')
    result = analyse(args.bag, args.backend)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
