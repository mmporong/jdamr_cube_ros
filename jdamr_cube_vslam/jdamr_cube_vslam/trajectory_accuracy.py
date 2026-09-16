"""Evaluate planar distance and yaw accuracy without hiding scale error."""

import argparse
from bisect import bisect_left
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from statistics import mean

import yaml


@dataclass(frozen=True)
class Pose:
    """Timestamped planar pose in SI units."""

    timestamp_s: float
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class Alignment:
    """Rigid SE(2) transform from visual coordinates to reference coordinates."""

    yaw_rad: float
    translation_x_m: float
    translation_y_m: float


def wrap_angle_rad(angle_rad: float) -> float:
    """Wrap radians to [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw_rad(qx: float, qy: float, qz: float,
                          qw: float) -> float:
    """Return planar yaw from a quaternion."""
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(sin_yaw, cos_yaw)


def read_trajectory(path: Path) -> list[Pose]:
    """Read the trajectory CSV contract emitted by the recorder."""
    poses = []
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            poses.append(Pose(
                timestamp_s=float(row['timestamp_s']),
                x_m=float(row['x_m']),
                y_m=float(row['y_m']),
                yaw_rad=quaternion_to_yaw_rad(
                    float(row['qx']), float(row['qy']),
                    float(row['qz']), float(row['qw'])),
            ))
    if len(poses) < 2:
        raise ValueError(f'trajectory requires at least two poses: {path}')
    return sorted(poses, key=lambda pose: pose.timestamp_s)


def associate_trajectories(
        visual: list[Pose], reference: list[Pose],
        max_time_delta_s: float) -> list[tuple[Pose, Pose]]:
    """Pair each visual pose with its nearest reference timestamp."""
    reference_times_s = [pose.timestamp_s for pose in reference]
    pairs = []
    for visual_pose in visual:
        insertion = bisect_left(reference_times_s, visual_pose.timestamp_s)
        candidates = []
        if insertion < len(reference):
            candidates.append(reference[insertion])
        if insertion > 0:
            candidates.append(reference[insertion - 1])
        if not candidates:
            continue
        reference_pose = min(
            candidates,
            key=lambda pose: abs(pose.timestamp_s - visual_pose.timestamp_s))
        if abs(reference_pose.timestamp_s - visual_pose.timestamp_s) \
                <= max_time_delta_s:
            pairs.append((visual_pose, reference_pose))
    return pairs


def rigid_align_se2(pairs: list[tuple[Pose, Pose]]) -> Alignment:
    """Align visual XY to reference XY using rotation and translation only."""
    if len(pairs) < 2:
        raise ValueError('at least two associated poses are required')
    visual_x_mean_m = mean(pair[0].x_m for pair in pairs)
    visual_y_mean_m = mean(pair[0].y_m for pair in pairs)
    reference_x_mean_m = mean(pair[1].x_m for pair in pairs)
    reference_y_mean_m = mean(pair[1].y_m for pair in pairs)

    dot_sum_m2 = 0.0
    cross_sum_m2 = 0.0
    for visual_pose, reference_pose in pairs:
        visual_x_m = visual_pose.x_m - visual_x_mean_m
        visual_y_m = visual_pose.y_m - visual_y_mean_m
        reference_x_m = reference_pose.x_m - reference_x_mean_m
        reference_y_m = reference_pose.y_m - reference_y_mean_m
        dot_sum_m2 += (
            visual_x_m * reference_x_m + visual_y_m * reference_y_m)
        cross_sum_m2 += (
            visual_x_m * reference_y_m - visual_y_m * reference_x_m)

    alignment_yaw_rad = math.atan2(cross_sum_m2, dot_sum_m2)
    cosine = math.cos(alignment_yaw_rad)
    sine = math.sin(alignment_yaw_rad)
    translation_x_m = (
        reference_x_mean_m
        - (cosine * visual_x_mean_m - sine * visual_y_mean_m))
    translation_y_m = (
        reference_y_mean_m
        - (sine * visual_x_mean_m + cosine * visual_y_mean_m))
    return Alignment(
        yaw_rad=alignment_yaw_rad,
        translation_x_m=translation_x_m,
        translation_y_m=translation_y_m,
    )


def apply_alignment(pose: Pose, alignment: Alignment) -> Pose:
    """Apply a rigid alignment without changing trajectory scale."""
    cosine = math.cos(alignment.yaw_rad)
    sine = math.sin(alignment.yaw_rad)
    return Pose(
        timestamp_s=pose.timestamp_s,
        x_m=(cosine * pose.x_m - sine * pose.y_m
             + alignment.translation_x_m),
        y_m=(sine * pose.x_m + cosine * pose.y_m
             + alignment.translation_y_m),
        yaw_rad=wrap_angle_rad(pose.yaw_rad + alignment.yaw_rad),
    )


def path_length_m(poses: list[Pose]) -> float:
    """Return accumulated planar path length."""
    return sum(
        math.hypot(current.x_m - previous.x_m,
                   current.y_m - previous.y_m)
        for previous, current in zip(poses, poses[1:])
    )


def _percentile(values: list[float], percentile: float) -> float:
    ordered = sorted(values)
    index = (len(ordered) - 1) * percentile
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def evaluate_pairs(pairs: list[tuple[Pose, Pose]]) -> dict:
    """Compute metric trajectory errors after scale-preserving alignment."""
    alignment = rigid_align_se2(pairs)
    visual_aligned = [apply_alignment(pair[0], alignment) for pair in pairs]
    reference = [pair[1] for pair in pairs]
    translation_errors_m = [
        math.hypot(visual_pose.x_m - reference_pose.x_m,
                   visual_pose.y_m - reference_pose.y_m)
        for visual_pose, reference_pose in zip(visual_aligned, reference)
    ]
    yaw_errors_rad = [
        abs(wrap_angle_rad(visual_pose.yaw_rad - reference_pose.yaw_rad))
        for visual_pose, reference_pose in zip(visual_aligned, reference)
    ]
    visual_distance_m = path_length_m(visual_aligned)
    reference_distance_m = path_length_m(reference)
    if reference_distance_m <= 0.0:
        raise ValueError('reference path length must be positive')

    translation_rpe_m = []
    yaw_rpe_rad = []
    for index in range(1, len(pairs)):
        visual_delta_x_m = (
            visual_aligned[index].x_m - visual_aligned[index - 1].x_m)
        visual_delta_y_m = (
            visual_aligned[index].y_m - visual_aligned[index - 1].y_m)
        reference_delta_x_m = reference[index].x_m - reference[index - 1].x_m
        reference_delta_y_m = reference[index].y_m - reference[index - 1].y_m
        translation_rpe_m.append(math.hypot(
            visual_delta_x_m - reference_delta_x_m,
            visual_delta_y_m - reference_delta_y_m,
        ))
        visual_delta_yaw_rad = wrap_angle_rad(
            visual_aligned[index].yaw_rad
            - visual_aligned[index - 1].yaw_rad)
        reference_delta_yaw_rad = wrap_angle_rad(
            reference[index].yaw_rad - reference[index - 1].yaw_rad)
        yaw_rpe_rad.append(abs(wrap_angle_rad(
            visual_delta_yaw_rad - reference_delta_yaw_rad)))

    visual_loop_error_m = math.hypot(
        visual_aligned[-1].x_m - visual_aligned[0].x_m,
        visual_aligned[-1].y_m - visual_aligned[0].y_m,
    )
    reference_loop_error_m = math.hypot(
        reference[-1].x_m - reference[0].x_m,
        reference[-1].y_m - reference[0].y_m,
    )
    rad_to_deg = 180.0 / math.pi
    return {
        'pair_count': len(pairs),
        'alignment': asdict(alignment),
        'translation_ate_rmse_m': math.sqrt(mean(
            error_m * error_m for error_m in translation_errors_m)),
        'translation_ate_mean_m': mean(translation_errors_m),
        'translation_ate_p95_m': _percentile(translation_errors_m, 0.95),
        'translation_rpe_rmse_m': math.sqrt(mean(
            error_m * error_m for error_m in translation_rpe_m)),
        'yaw_rmse_deg': math.sqrt(mean(
            error_rad * error_rad for error_rad in yaw_errors_rad))
        * rad_to_deg,
        'yaw_mean_abs_deg': mean(yaw_errors_rad) * rad_to_deg,
        'yaw_p95_deg': _percentile(yaw_errors_rad, 0.95) * rad_to_deg,
        'yaw_rpe_rmse_deg': math.sqrt(mean(
            error_rad * error_rad for error_rad in yaw_rpe_rad))
        * rad_to_deg,
        'visual_path_length_m': visual_distance_m,
        'reference_path_length_m': reference_distance_m,
        'distance_scale_error_percent': (
            visual_distance_m / reference_distance_m - 1.0) * 100.0,
        'visual_start_end_distance_m': visual_loop_error_m,
        'reference_start_end_distance_m': reference_loop_error_m,
        'loop_distance_difference_m': abs(
            visual_loop_error_m - reference_loop_error_m),
    }


def _nearest_pose(poses: list[Pose], timestamp_s: float) -> Pose:
    return min(poses, key=lambda pose: abs(pose.timestamp_s - timestamp_s))


def evaluate_controlled_segments(
        visual: list[Pose], measurements_path: Path) -> list[dict]:
    """Evaluate tape- or fixture-measured segments from a YAML protocol."""
    data = yaml.safe_load(measurements_path.read_text(encoding='utf-8'))
    segments = data.get('controlled_measurements', {}).get('segments', [])
    results = []
    rad_to_deg = 180.0 / math.pi
    for segment in segments:
        start_s = float(segment['start_sec'])
        end_s = float(segment['end_sec'])
        subset = [
            pose for pose in visual
            if start_s <= pose.timestamp_s <= end_s
        ]
        if len(subset) < 2:
            subset = [
                _nearest_pose(visual, start_s),
                _nearest_pose(visual, end_s),
            ]
        result = {'name': segment['name']}
        if 'expected_distance_m' in segment:
            measured_distance_m = path_length_m(subset)
            expected_distance_m = float(segment['expected_distance_m'])
            result.update({
                'measured_distance_m': measured_distance_m,
                'expected_distance_m': expected_distance_m,
                'distance_error_m': measured_distance_m - expected_distance_m,
                'distance_error_percent': (
                    measured_distance_m / expected_distance_m - 1.0) * 100.0,
            })
        if 'expected_yaw_deg' in segment:
            measured_yaw_deg = wrap_angle_rad(
                subset[-1].yaw_rad - subset[0].yaw_rad) * rad_to_deg
            expected_yaw_deg = float(segment['expected_yaw_deg'])
            result.update({
                'measured_yaw_deg': measured_yaw_deg,
                'expected_yaw_deg': expected_yaw_deg,
                'yaw_error_deg': measured_yaw_deg - expected_yaw_deg,
            })
        results.append(result)
    return results


def render_markdown(report: dict) -> str:
    """Render the core accuracy evidence as a compact Markdown report."""
    metrics = report['trajectory_metrics']
    lines = [
        '# RGB-D SLAM 거리·각도 정확도',
        '',
        '| 지표 | 결과 |',
        '|---|---:|',
        f"| 시간 정렬 pose | {metrics['pair_count']} |",
        f"| 이동 ATE RMSE | {metrics['translation_ate_rmse_m']:.4f} m |",
        f"| 이동 ATE P95 | {metrics['translation_ate_p95_m']:.4f} m |",
        f"| yaw RMSE | {metrics['yaw_rmse_deg']:.3f} deg |",
        f"| yaw P95 | {metrics['yaw_p95_deg']:.3f} deg |",
        f"| 거리 스케일 오차 | {metrics['distance_scale_error_percent']:.3f} % |",
        f"| 폐루프 거리 차이 | {metrics['loop_distance_difference_m']:.4f} m |",
        '',
        'SE(2) 회전·이동만 정렬했으며 스케일 보정은 적용하지 않았다.',
        'Reference가 wheel odometry 또는 AMCL이면 절대 ground truth가 아니다.',
    ]
    controlled = report.get('controlled_segments', [])
    if controlled:
        lines.extend(['', '## 실측 구간', ''])
        for segment in controlled:
            lines.append(f"- {segment['name']}: `{json.dumps(segment, ensure_ascii=False)}`")
    return '\n'.join(lines) + '\n'


def _parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--visual', required=True, type=Path)
    parser.add_argument('--reference', required=True, type=Path)
    parser.add_argument('--max-time-delta', type=float, default=0.08)
    parser.add_argument('--minimum-pairs', type=int, default=30)
    parser.add_argument('--measurements', type=Path)
    parser.add_argument('--output-json', required=True, type=Path)
    parser.add_argument('--output-md', required=True, type=Path)
    return parser.parse_args(argv)


def main(args=None):
    """Evaluate trajectories and write machine- and human-readable evidence."""
    parsed = _parse_args(args)
    visual = read_trajectory(parsed.visual)
    reference = read_trajectory(parsed.reference)
    pairs = associate_trajectories(
        visual, reference, parsed.max_time_delta)
    if len(pairs) < parsed.minimum_pairs:
        raise SystemExit(
            f'associated pose count {len(pairs)} is below '
            f'minimum {parsed.minimum_pairs}')
    report = {
        'inputs': {
            'visual': str(parsed.visual),
            'reference': str(parsed.reference),
            'max_time_delta_s': parsed.max_time_delta,
            'scale_alignment_applied': False,
        },
        'trajectory_metrics': evaluate_pairs(pairs),
    }
    if parsed.measurements:
        report['controlled_segments'] = evaluate_controlled_segments(
            visual, parsed.measurements)
    parsed.output_json.parent.mkdir(parents=True, exist_ok=True)
    parsed.output_md.parent.mkdir(parents=True, exist_ok=True)
    parsed.output_json.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    parsed.output_md.write_text(render_markdown(report), encoding='utf-8')


if __name__ == '__main__':
    main()
