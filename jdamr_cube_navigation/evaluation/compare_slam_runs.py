#!/usr/bin/env python3
"""Compare offline 2D SLAM backends replayed from the same recorded bag."""

# The robot has no external ground truth, so this never calls its numbers ATE
# or RPE.  It reports what the recordings actually support: how far each
# backend says the robot travelled, how far its estimate drifts from wheel
# odometry and from the AMCL pose recorded against the published map, and how
# much of the corridor each backend mapped.

from __future__ import annotations

import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from mcap_ros2.reader import read_ros2_messages


def _sha256(path: Path) -> str:
    """Return the SHA-256 digest of one comparison input."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def yaw_of(orientation):
    """Return the planar yaw of a quaternion."""
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))


def compose(parent, child):
    """Compose two planar (x, y, yaw) transforms."""
    px, py, pyaw = parent
    cx, cy, cyaw = child
    return (px + cx * math.cos(pyaw) - cy * math.sin(pyaw),
            py + cx * math.sin(pyaw) + cy * math.cos(pyaw),
            pyaw + cyaw)


# High-rate wheel odometry contains small alternating position changes.
# Summing every consecutive pair inflates path length, so the trajectory is
# decimated to a minimum spatial step before measuring it.
MIN_STEP_M = 0.05


def decimate(points, min_step=MIN_STEP_M):
    """Return points spaced at least *min_step* apart, keeping the ends."""
    if not points:
        return []
    kept = [points[0]]
    for point in points[1:]:
        if math.dist(kept[-1][1:3], point[1:3]) >= min_step:
            kept.append(point)
    if kept[-1] is not points[-1]:
        kept.append(points[-1])
    return kept


def path_length(points):
    """Return the polyline length of a trajectory, noise removed."""
    kept = decimate(points)
    return sum(math.dist(a[1:3], b[1:3]) for a, b in zip(kept, kept[1:]))


def rigid_align(source, target):
    """Return the rotation and translation putting *source* onto *target*."""
    # A SLAM backend starts its map frame at the robot's first pose, while the
    # recorded AMCL pose lives in the pre-built map frame.  Comparing them
    # without alignment measures the frame offset, not the estimate.  Planar
    # Kabsch uses rotation and translation only, never scale.
    if len(source) < 2:
        return None
    sx = sum(p[0] for p in source) / len(source)
    sy = sum(p[1] for p in source) / len(source)
    tx = sum(p[0] for p in target) / len(target)
    ty = sum(p[1] for p in target) / len(target)
    num = sum((p[0] - sx) * (q[1] - ty) - (p[1] - sy) * (q[0] - tx)
              for p, q in zip(source, target))
    den = sum((p[0] - sx) * (q[0] - tx) + (p[1] - sy) * (q[1] - ty)
              for p, q in zip(source, target))
    if num == 0.0 and den == 0.0:
        return None
    theta = math.atan2(num, den)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return (theta,
            tx - (cos_t * sx - sin_t * sy),
            ty - (sin_t * sx + cos_t * sy))


def apply_rigid(transform, x, y):
    """Apply a (theta, dx, dy) planar transform to a point."""
    theta, dx, dy = transform
    return (math.cos(theta) * x - math.sin(theta) * y + dx,
            math.sin(theta) * x + math.cos(theta) * y + dy)


def read_map_to_odom(result_bag: Path):
    """Return the backend's map->odom estimates as (t, x, y, yaw)."""
    samples = []
    for message in read_ros2_messages(str(result_bag), topics=['/tf']):
        for transform in message.ros_msg.transforms:
            if (transform.header.frame_id.lstrip('/') == 'map'
                    and transform.child_frame_id.lstrip('/') == 'odom'):
                stamp = transform.header.stamp
                samples.append((
                    stamp.sec + stamp.nanosec * 1e-9,
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    yaw_of(transform.transform.rotation)))
    samples.sort()
    return samples


def read_odometry(source_bag: Path):
    """Return recorded wheel odometry as (t, x, y, yaw)."""
    samples = []
    for message in read_ros2_messages(str(source_bag), topics=['/odom']):
        pose = message.ros_msg.pose.pose
        stamp = message.ros_msg.header.stamp
        samples.append((stamp.sec + stamp.nanosec * 1e-9,
                        pose.position.x, pose.position.y,
                        yaw_of(pose.orientation)))
    samples.sort()
    return samples


def read_amcl(source_bag: Path):
    """Return the recorded AMCL pose as (t, x, y, yaw)."""
    samples = []
    for message in read_ros2_messages(str(source_bag), topics=['/amcl_pose']):
        pose = message.ros_msg.pose.pose
        stamp = message.ros_msg.header.stamp
        samples.append((stamp.sec + stamp.nanosec * 1e-9,
                        pose.position.x, pose.position.y,
                        yaw_of(pose.orientation)))
    samples.sort()
    return samples


def nearest(samples, timestamp):
    """Return the sample closest in time, or None when the list is empty."""
    if not samples:
        return None
    index = bisect_left(samples, timestamp, key=lambda sample: sample[0])
    candidates = samples[max(0, index - 1):index + 1]
    best = min(candidates, key=lambda sample: abs(sample[0] - timestamp))
    return best if abs(best[0] - timestamp) < 1.0 else None


def estimated_trajectory(map_to_odom, odometry):
    """Compose the backend correction with wheel odometry."""
    trajectory = []
    for stamp, ox, oy, oyaw in odometry:
        correction = nearest(map_to_odom, stamp)
        if correction is None:
            continue
        x, y, yaw = compose(correction[1:], (ox, oy, oyaw))
        trajectory.append((stamp, x, y, yaw))
    return trajectory


def map_statistics(map_yaml: Path):
    """Return occupied/free cell counts and metric extent of a saved map."""
    import yaml
    document = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    with (map_yaml.parent / document['image']).open('rb') as stream:
        if stream.readline().strip() != b'P5':
            raise ValueError('saved map is not a binary PGM')
        header_tokens = []
        while len(header_tokens) < 3:
            line = stream.readline()
            if not line:
                raise ValueError('saved map has a truncated PGM header')
            if line.lstrip().startswith(b'#'):
                continue
            header_tokens.extend(line.split())
        width, height, max_value = map(int, header_tokens[:3])
        if max_value != 255:
            raise ValueError('saved map PGM maximum must be 255')
        data = stream.read()
    if len(data) != width * height:
        raise ValueError('saved map PGM dimensions do not match its payload')
    resolution = float(document['resolution'])
    occupied_threshold = float(document['occupied_thresh'])
    free_threshold = float(document['free_thresh'])
    negate = bool(document.get('negate', 0))
    occupancy = [
        (value if negate else 255 - value) / 255.0
        for value in data
    ]
    occupied = sum(1 for value in occupancy
                   if value > occupied_threshold)
    free = sum(1 for value in occupancy if value < free_threshold)
    unknown = width * height - occupied - free
    cell_area_m2 = resolution ** 2
    return {
        'width_cells': width,
        'height_cells': height,
        'extent_m': [round(width * resolution, 2),
                     round(height * resolution, 2)],
        'occupied_cells': occupied,
        'free_cells': free,
        'unknown_cells': unknown,
        'occupied_area_m2': round(occupied * cell_area_m2, 4),
        'free_area_m2': round(free * cell_area_m2, 4),
        'unknown_area_m2': round(unknown * cell_area_m2, 4),
    }


def _timestamp_sha256(samples):
    """Hash an ordered trajectory timestamp set without decimal rounding."""
    digest = hashlib.sha256()
    for sample in samples:
        digest.update(f'{sample[0]:.9f}\n'.encode())
    return digest.hexdigest()


def _analysis_window(samples, start_unix_ns=None, end_unix_ns=None):
    """Return samples inside an inclusive absolute timestamp window."""
    if start_unix_ns is None and end_unix_ns is None:
        return samples
    start_s = float('-inf') if start_unix_ns is None else start_unix_ns * 1e-9
    end_s = float('inf') if end_unix_ns is None else end_unix_ns * 1e-9
    return [sample for sample in samples if start_s <= sample[0] <= end_s]


def deviation(trajectory, reference):
    """Return residuals after aligning the estimate onto the reference."""
    pairs = []
    paired_timestamps = []
    for stamp, x, y, _ in decimate(trajectory, 0.10):
        match = nearest(reference, stamp)
        if match is not None:
            pairs.append(((x, y), (match[1], match[2])))
            paired_timestamps.append((stamp,))
    if len(pairs) < 2:
        return None
    transform = rigid_align([p[0] for p in pairs], [p[1] for p in pairs])
    if transform is None:
        return None
    residuals = [math.dist(apply_rigid(transform, *source), target)
                 for source, target in pairs]
    rms_m_raw = math.sqrt(sum(r * r for r in residuals) / len(residuals))
    return {
        'samples': len(residuals),
        'timestamp_sha256': _timestamp_sha256(paired_timestamps),
        'aligned_yaw_deg': round(math.degrees(transform[0]), 2),
        'rms_m_raw': rms_m_raw,
        'rms_m': round(rms_m_raw, 3),
        'max_m_raw': max(residuals),
        'max_m': round(max(residuals), 3),
    }


def common_reference_deviations(trajectories, reference):
    """Evaluate every trajectory on one shared reference timestamp set."""
    common = []
    for reference_sample in decimate(reference, 0.10):
        matches = {
            label: nearest(trajectory, reference_sample[0])
            for label, trajectory in trajectories.items()
        }
        if all(match is not None for match in matches.values()):
            common.append((reference_sample, matches))
    if len(common) < 2:
        raise ValueError('fewer than two common AMCL reference samples')
    timestamps = [(reference_sample[0],) for reference_sample, _ in common]
    timestamp_sha256 = _timestamp_sha256(timestamps)
    variants = {}
    for label in trajectories:
        source_points = [
            (matches[label][1], matches[label][2]) for _, matches in common]
        target_points = [
            (reference_sample[1], reference_sample[2])
            for reference_sample, _ in common]
        transform = rigid_align(source_points, target_points)
        if transform is None:
            raise ValueError(f'cannot align common reference samples: {label}')
        residuals = [
            math.dist(apply_rigid(transform, *source), target)
            for source, target in zip(source_points, target_points)
        ]
        rms_m_raw = math.sqrt(
            sum(residual * residual for residual in residuals) /
            len(residuals))
        variants[label] = {
            'samples': len(residuals),
            'timestamp_sha256': timestamp_sha256,
            'aligned_yaw_deg': round(math.degrees(transform[0]), 2),
            'rms_m_raw': rms_m_raw,
            'rms_m': round(rms_m_raw, 3),
            'max_m_raw': max(residuals),
            'max_m': round(max(residuals), 3),
        }
    return {
        'samples': len(common),
        'timestamp_sha256': timestamp_sha256,
        'variants': variants,
    }


def analyse(
        result_bag: Path, source_bag: Path, map_yaml: Path | None,
        *, start_unix_ns=None, end_unix_ns=None):
    """Return one comparison record for a replayed backend run."""
    map_to_odom = read_map_to_odom(result_bag)
    odometry = read_odometry(source_bag)
    amcl = read_amcl(source_bag)
    trajectory = _analysis_window(
        estimated_trajectory(map_to_odom, odometry),
        start_unix_ns, end_unix_ns)
    odometry = _analysis_window(odometry, start_unix_ns, end_unix_ns)
    amcl = _analysis_window(amcl, start_unix_ns, end_unix_ns)
    map_to_odom_window = _analysis_window(
        map_to_odom, start_unix_ns, end_unix_ns)

    record = {
        'result_bag': result_bag.name,
        'result_bag_sha256': _sha256(result_bag),
        'source_bag': source_bag.name,
        'source_bag_sha256': _sha256(source_bag),
        'map_to_odom_updates': len(map_to_odom_window),
        'odometry_samples': len(odometry),
        'trajectory_samples': len(trajectory),
        'trajectory_timestamp_sha256': _timestamp_sha256(trajectory),
        'analysis_window': {
            'start_unix_ns': start_unix_ns,
            'end_unix_ns': end_unix_ns,
        },
        'map_to_odom_first_unix_ns': (
            round(map_to_odom[0][0] * 1e9) if map_to_odom else None),
        'map_to_odom_last_unix_ns': (
            round(map_to_odom[-1][0] * 1e9) if map_to_odom else None),
        'map_to_odom_window_covered': (
            bool(map_to_odom)
            and (start_unix_ns is None
                 or map_to_odom[0][0] * 1e9 <= start_unix_ns)
            and (end_unix_ns is None
                 or map_to_odom[-1][0] * 1e9 >= end_unix_ns)),
    }
    if trajectory:
        record['estimated_length_m'] = round(path_length(trajectory), 3)
        record['start_xy'] = [round(trajectory[0][1], 3),
                              round(trajectory[0][2], 3)]
        record['end_xy'] = [round(trajectory[-1][1], 3),
                            round(trajectory[-1][2], 3)]
        start_to_end_m_raw = math.dist(
            trajectory[0][1:3], trajectory[-1][1:3])
        record['start_to_end_m_raw'] = start_to_end_m_raw
        record['start_to_end_m'] = round(start_to_end_m_raw, 3)
    if odometry:
        record['odometry_length_m'] = round(path_length(odometry), 3)
    if amcl:
        record['amcl_length_m'] = round(path_length(amcl), 3)
        record['deviation_from_amcl'] = deviation(trajectory, amcl)
    record['deviation_from_odometry'] = deviation(trajectory, odometry)
    if map_yaml and map_yaml.is_file():
        import yaml
        map_document = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
        map_image = map_yaml.parent / map_document['image']
        record['map_yaml'] = map_yaml.name
        record['map_yaml_sha256'] = _sha256(map_yaml)
        record['map_image_sha256'] = _sha256(map_image)
        record['map'] = map_statistics(map_yaml)
    return record


def comparison_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    """Select a backend only when both observable criteria agree."""
    complete = [
        record for record in records
        if record.get('start_to_end_m') is not None
        and record.get('deviation_from_amcl', {}).get('rms_m') is not None
    ]
    if len({record.get('backend') for record in complete}) < 2:
        return {'selected_backend': None, 'reason': 'insufficient_data'}
    closure_winner = min(complete, key=lambda record: record['start_to_end_m'])
    reference_winner = min(
        complete, key=lambda record: record['deviation_from_amcl']['rms_m'])
    selected_backend = (
        closure_winner['backend']
        if closure_winner['backend'] == reference_winner['backend'] else None)
    result = {
        'selected_backend': selected_backend,
        'reason': ('criteria_agree' if selected_backend is not None
                   else 'criteria_disagree'),
        'closed_loop_consistency_winner': closure_winner['backend'],
        'saved_map_reference_consistency_winner': reference_winner['backend'],
        'ground_truth_available': False,
    }
    if selected_backend is not None and len(complete) > 1:
        selected = next(record for record in complete
                        if record['backend'] == selected_backend)
        alternatives = [record for record in complete
                        if record['backend'] != selected_backend]
        closest_alternative = min(
            alternatives, key=lambda record: record['start_to_end_m'])
        closure_denominator = selected['start_to_end_m']
        reference_denominator = selected['deviation_from_amcl']['rms_m']
        result['relative_to_closest_alternative'] = {
            'backend': closest_alternative['backend'],
            'selected_start_to_end_m': selected['start_to_end_m'],
            'alternative_start_to_end_m': (
                closest_alternative['start_to_end_m']),
            'start_to_end_difference_m': round(
                closest_alternative['start_to_end_m']
                - selected['start_to_end_m'], 3),
            'start_to_end_ratio': (
                round(closest_alternative['start_to_end_m']
                      / closure_denominator, 2)
                if closure_denominator > 0.0 else None),
            'amcl_aligned_rms_ratio': (
                round(closest_alternative['deviation_from_amcl']['rms_m']
                      / reference_denominator, 2)
                if reference_denominator > 0.0 else None),
        }
    return result


def comparison_precondition_errors(
        records: list[dict[str, Any]]) -> list[str]:
    """Return reasons that make a same-source backend comparison invalid."""
    errors = []
    backends = [record.get('backend') for record in records]
    if len(records) < 2:
        errors.append('at least two completed backend results are required')
    if any(not backend for backend in backends):
        errors.append('every result must identify its backend')
    if len(set(backends)) != len(backends):
        errors.append('backend names must be unique')
    source_hashes = {record.get('source_bag_sha256') for record in records}
    source_names = {record.get('source_bag') for record in records}
    if None in source_hashes or len(source_hashes) != 1:
        errors.append('all results must use one identical source bag hash')
    if None in source_names or len(source_names) != 1:
        errors.append('all results must use one identical source bag name')
    for record in records:
        backend = record.get('backend') or '<unknown>'
        required = {
            'trajectory': record.get('start_to_end_m'),
            'AMCL reference RMS': record.get(
                'deviation_from_amcl', {}).get('rms_m'),
            'saved map': record.get('map'),
            'saved map metadata': record.get('map_yaml'),
        }
        missing = [name for name, value in required.items() if not value
                   and value != 0.0]
        if missing:
            errors.append(f"{backend}: missing {', '.join(missing)}")
    return errors


def _configure_plot_font() -> None:
    """Use an installed Korean font when matplotlib can resolve it."""
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    font_path = Path(
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font_path.is_file():
        family = font_manager.FontProperties(fname=str(font_path)).get_name()
        plt.rcParams['font.family'] = family
    plt.rcParams['axes.unicode_minus'] = False


def render_comparison(records: list[dict[str, Any]], results: Path,
                      output: Path) -> None:
    """Render maps and observable same-bag consistency metrics."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from PIL import Image

    _configure_plot_font()
    ordered = sorted(records, key=lambda record: record['backend'])
    map_columns = min(3, len(ordered))
    map_rows = math.ceil(len(ordered) / map_columns)
    figure_width = 14 if len(ordered) <= 2 else 18
    figure_height = 4.0 * map_rows + 5.5
    figure = plt.figure(
        figsize=(figure_width, figure_height), dpi=160)
    grid = figure.add_gridspec(
        map_rows + 2,
        map_columns,
        height_ratios=([1.0] * map_rows) + [0.8, 0.65])
    for index, record in enumerate(ordered):
        row, column = divmod(index, map_columns)
        axes = figure.add_subplot(grid[row, column])
        map_yaml = results / record['map_yaml']
        document = __import__('yaml').safe_load(
            map_yaml.read_text(encoding='utf-8'))
        image = np.asarray(Image.open(map_yaml.parent / document['image']))
        axes.imshow(image, cmap='gray', vmin=0, vmax=255)
        axes.set_title(
            f"{record['backend']}\n"
            f"map {record['map']['extent_m'][0]}×"
            f"{record['map']['extent_m'][1]} m")
        axes.axis('off')
    for index in range(len(ordered), map_rows * map_columns):
        row, column = divmod(index, map_columns)
        axes = figure.add_subplot(grid[row, column])
        axes.axis('off')

    metrics = figure.add_subplot(grid[map_rows, :])
    backends = [record['backend'] for record in ordered]
    positions = np.arange(len(ordered))
    width = 0.36
    closure_m = [record['start_to_end_m'] for record in ordered]
    reference_rms_m = [
        record['deviation_from_amcl']['rms_m'] for record in ordered]
    metrics.bar(positions - width / 2, closure_m, width,
                label='start-to-end', color='#2563eb')
    metrics.bar(positions + width / 2, reference_rms_m, width,
                label='aligned AMCL RMS', color='#16a34a')
    metrics.set_xticks(positions, backends, rotation=12, ha='right')
    metrics.set_ylabel('distance [m] — lower is more consistent')
    metrics.set_title('Same-bag consistency metrics')
    metrics.grid(axis='y', alpha=0.25)
    metrics.legend()

    summary = figure.add_subplot(grid[map_rows + 1, :])
    summary.axis('off')
    selection = comparison_summary(records)
    selection_label = (selection['selected_backend']
                       if selection['selected_backend'] is not None
                       else 'DEFERRED')
    lines = [
        'BACKEND SELECTION',
        '',
        f'Selected  {selection_label}',
        '',
        f"{'variant':<43} {'path [m]':>9} {'closure [m]':>12} "
        f"{'AMCL RMS [m]':>12}",
    ]
    for record in ordered:
        lines.append(
            f"{record['backend']:<43} "
            f"{record['estimated_length_m']:>9.3f} "
            f"{record['start_to_end_m']:>12.3f} "
            f"{record['deviation_from_amcl']['rms_m']:>12.3f}")
    lines.extend([
        '',
        'AMCL is a saved-map localization reference,',
        'not external ground truth or ATE.',
    ])
    summary.text(
        0.03, 0.97, '\n'.join(lines), va='top', ha='left', fontsize=10.5,
        family='monospace', linespacing=1.25,
        bbox={'boxstyle': 'round,pad=0.8', 'facecolor': '#f8fafc',
              'edgecolor': '#cbd5e1'})
    figure.suptitle(
        'JD-AMR offline 2D SLAM comparison — identical recorded input',
        y=0.995)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.975))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, bbox_inches='tight')
    plt.close(figure)


def render_report(records: list[dict[str, Any]]) -> str:
    """Return a concise Korean evidence report for portfolio synthesis."""
    ordered = sorted(records, key=lambda record: record['backend'])
    selection = comparison_summary(records)
    lines = [
        '# 동일 MCAP 2D SLAM 백엔드 비교',
        '',
        '## 실험 조건',
        '',
        '- 동일한 실물 복도 주행 MCAP을 격리된 ROS domain에서 재생했다.',
        '- 저장 지도와 이동 명령은 재생하지 않았고, 기록된 AMCL `map→odom`은 제거했다.',
        '- 각 실행에서는 mapping backend 하나만 `map→odom` 권한을 가졌다.',
        '- 외부 ground truth가 없으므로 아래 값은 ATE/RPE가 아니다.',
        '',
        '## 결과',
        '',
        '| 백엔드/조건 | 추정 경로 | 시작–종료 | AMCL 기준 정렬 RMS | AMCL 기준 최대 편차 | 지도 범위 |',
        '|---|---:|---:|---:|---:|---:|',
    ]
    for record in ordered:
        extent_m = record['map']['extent_m']
        lines.append(
            f"| {record['backend']} | {record['estimated_length_m']:.3f}m | "
            f"{record['start_to_end_m']:.3f}m | "
            f"{record['deviation_from_amcl']['rms_m']:.3f}m | "
            f"{record['deviation_from_amcl']['max_m']:.3f}m | "
            f'{extent_m[0]}×{extent_m[1]}m |')
    lines.extend(['', '## 선택', ''])
    if selection['selected_backend'] is None:
        lines.append(
            '두 일관성 기준의 우승 백엔드가 달라 선택을 보류한다. 추가 기준과 '
            'ground truth 실험 없이 기본 백엔드를 정하지 않는다.')
    else:
        ratios = selection['relative_to_closest_alternative']
        if ratios['start_to_end_ratio'] <= 1.05:
            lines.append(
                f'`{selection["selected_backend"]}`를 현재 복도 데이터의 '
                f'기본 백엔드로 선택한다. 시작–종료 불일치는 '
                f'{ratios["selected_start_to_end_m"]:.3f}m와 '
                f'{ratios["alternative_start_to_end_m"]:.3f}m로 수치상 '
                f'유사했고, 저장 지도 AMCL 기준 정렬 RMS는 '
                f'{ratios["amcl_aligned_rms_ratio"]}배 작았다.')
        else:
            lines.append(
                f'`{selection["selected_backend"]}`를 현재 복도 데이터의 '
                f'기본 백엔드로 선택한다. 동일 입력에서 시작–종료 '
                f'불일치가 {ratios["start_to_end_ratio"]}배 작고, 저장 '
                f'지도 AMCL 기준 정렬 RMS가 '
                f'{ratios["amcl_aligned_rms_ratio"]}배 작았다.')
    lines.extend([
        '',
        '이 비교는 백엔드의 절대 정확도를 증명하지 않는다. 같은 센서 '
        '입력에서 폐루프 구조와 저장 지도 기준 일관성을 얼마나 '
        '유지했는지를 비교한 결과다.',
        '',
    ])
    return '\n'.join(lines)


def main(argv=None):
    """Analyse every replayed run in a directory."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--results', type=Path, required=True,
                        help='directory holding *_result bags and maps')
    parser.add_argument('--source-root', type=Path,
                        default=Path.home() / 'jdamr_artifacts')
    parser.add_argument('--output', type=Path, default=None)
    parser.add_argument('--plot', type=Path, default=None)
    parser.add_argument('--report', type=Path, default=None)
    args = parser.parse_args(argv)

    records = []
    input_errors = []
    result_dirs = sorted(args.results.glob('*_result'))
    if not result_dirs:
        input_errors.append(f'no *_result directories under {args.results}')
    for result_dir in result_dirs:
        mcaps = sorted(result_dir.glob('*.mcap'))
        if len(mcaps) != 1:
            input_errors.append(
                f'{result_dir.name}: expected one result MCAP, got '
                f'{len(mcaps)}')
            continue
        mcap = mcaps[0]
        # A recorder still writing leaves no finalized metadata, and reading
        # the unfinished MCAP raises instead of returning partial data.
        metadata = result_dir / 'metadata.yaml'
        if not metadata.is_file() or metadata.stat().st_size == 0:
            input_errors.append(
                f'{result_dir.name}: recording is not finalized')
            continue
        stem = result_dir.name[:-len('_result')]
        source_name, separator, backend = stem.rpartition('__')
        if not separator or not source_name or not backend:
            input_errors.append(
                f'{result_dir.name}: expected <source>__<backend>_result')
            continue
        source_dir = args.source_root / source_name
        source_mcaps = sorted(source_dir.glob('*.mcap'))
        if len(source_mcaps) != 1:
            input_errors.append(
                f'{source_dir}: expected one source MCAP, got '
                f'{len(source_mcaps)}')
            continue
        source_mcap = source_mcaps[0]
        map_yaml = args.results / f'{stem}_map.yaml'
        if not map_yaml.is_file():
            input_errors.append(f'{stem}: saved map YAML is missing')
            continue
        record = analyse(mcap, source_mcap, map_yaml)
        record['backend'] = backend
        records.append(record)
    input_errors.extend(comparison_precondition_errors(records))
    if input_errors:
        parser.error('comparison preconditions failed:\n- '
                     + '\n- '.join(input_errors))
    for record in records:
        print(json.dumps(record, ensure_ascii=False, indent=2))
    if args.output:
        args.output.write_text(
            json.dumps(records, ensure_ascii=False, indent=2),
            encoding='utf-8')
        print(f'\n저장: {args.output}')
    if args.plot:
        render_comparison(records, args.results, args.plot)
        print(f'비교 그림 저장: {args.plot}')
    if args.report:
        args.report.write_text(render_report(records), encoding='utf-8')
        print(f'비교 보고서 저장: {args.report}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
