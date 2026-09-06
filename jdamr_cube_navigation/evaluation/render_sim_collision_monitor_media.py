#!/usr/bin/env python3
"""Render deterministic G004 path-and-state media from approved MCAP bags."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.msg import CollisionMonitorState
from rclpy.serialization import deserialize_message
import rosbag2_py  # noqa: I100
from sensor_msgs.msg import LaserScan
from PIL import Image, ImageDraw, ImageFont  # noqa: I100, I201

from evaluate_sim_collision_monitor import (  # noqa: I100
    strict_json_loads, verify_promotion)
from sim_collision_monitor_contract import sha256_file

import yaml  # noqa: I100, I201


WIDTH = 1280
HEIGHT = 720
FPS = 8
FRAME_COUNT = 96
SCENARIOS = (
    'clear_baseline', 'sudden_obstacle_stop_resume',
    'scan_timeout_stop_resume')
FONT_PATH = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
MEDIA_LIMIT_BYTES = 128 * 1024 * 1024
COLORS = {
    'path': '#168aad', 'robot': '#ff7f11', 'goal': '#26a269',
    'stop': '#c83636', 'background': '#f4f6f8'}


def _stamp_ns(stamp: Any) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _record(path: Path, root: Path) -> dict[str, Any]:
    return {
        'relative_path': str(path.relative_to(root)),
        'size_bytes': path.stat().st_size,
        'sha256': sha256_file(path),
    }


def _input_record(kind: str, path: Path) -> dict[str, Any]:
    return {
        'kind': kind, 'path': str(path.resolve()),
        'size_bytes': path.stat().st_size, 'sha256': sha256_file(path),
    }


def _font(size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(str(FONT_PATH), size)


def _load_bag(run_dir: Path, evidence: dict[str, Any]) -> dict[str, Any]:
    bag_dir = run_dir / 'bag'
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''))
    poses = []
    raw_stamps = []
    monitor_stamps = []
    states = []
    statuses = []
    cmd_vel_count = 0
    topic_counts: Counter[str] = Counter()
    while reader.has_next():
        topic, data, log_ns = reader.read_next()
        topic_counts[topic] += 1
        if topic == '/ground_truth_pose':
            message = deserialize_message(data, PoseStamped)
            poses.append((_stamp_ns(message.header.stamp),
                          float(message.pose.position.x),
                          float(message.pose.position.y)))
        elif topic == '/sim_raw/scan':
            message = deserialize_message(data, LaserScan)
            raw_stamps.append(_stamp_ns(message.header.stamp))
        elif topic == '/collision_monitor_scan':
            message = deserialize_message(data, LaserScan)
            monitor_stamps.append(_stamp_ns(message.header.stamp))
        elif topic == '/collision_monitor_state':
            message = deserialize_message(data, CollisionMonitorState)
            states.append((log_ns, int(message.action_type),
                           message.polygon_name))
        elif topic == '/navigate_to_pose/_action/status':
            message = deserialize_message(data, GoalStatusArray)
            statuses.extend(
                (log_ns, bytes(item.goal_info.goal_id.uuid).hex(), item.status)
                for item in message.status_list
                if any(item.goal_info.goal_id.uuid))
        elif topic == '/cmd_vel':
            deserialize_message(data, Twist)
            cmd_vel_count += 1
    expected_uuid = evidence['goal_uuid']
    selected_statuses = [status for _, uuid, status in statuses
                         if uuid == expected_uuid]
    if (selected_statuses != [2, 4]
            or {uuid for _, uuid, _ in statuses} != {expected_uuid}
            or not poses or not raw_stamps or not monitor_stamps
            or cmd_vel_count == 0):
        raise ValueError('representative MCAP semantic binding mismatch')
    expected_states = {
        'clear_baseline': [],
        'sudden_obstacle_stop_resume': [(1, 'StopZone'), (0, '')],
        'scan_timeout_stop_resume': [(1, 'invalid source'), (0, '')],
    }[evidence['scenario']]
    if [(action, polygon) for _, action, polygon in states] != expected_states:
        raise ValueError('collision monitor state sequence mismatch')
    return {
        'poses': sorted(poses), 'raw_stamps': sorted(raw_stamps),
        'monitor_stamps': sorted(monitor_stamps), 'states': states,
        'statuses': statuses, 'cmd_vel_count': cmd_vel_count,
        'topic_counts': dict(sorted(topic_counts.items())),
    }


def _event_map(evidence: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result = {}
    for event in evidence['events']:
        name = event['name']
        if name in result:
            raise ValueError(f'duplicate media event: {name}')
        result[name] = event
    return result


def _scenario_state(
        scenario: str, target_ns: int, events: dict[str, dict[str, Any]],
) -> tuple[str, str, tuple[int, int, int]]:
    if scenario not in SCENARIOS:
        raise ValueError(f'unknown scenario: {scenario}')
    if target_ns >= events['succeeded']['ros_ns']:
        return 'GOAL SUCCEEDED', '목표 도달', (38, 166, 91)
    if scenario == 'clear_baseline':
        return 'NAVIGATING', '개입 없이 주행', (38, 116, 190)
    if scenario == 'sudden_obstacle_stop_resume':
        if target_ns < events['obstacle_set_pose_requested']['ros_ns']:
            return 'NAVIGATING', '장애물 전', (38, 116, 190)
        if target_ns < events['stop_state']['ros_ns']:
            return 'OBSTACLE DETECTED', 'StopZone 감지', (224, 144, 32)
        if target_ns < events['do_nothing_after_stop']['ros_ns']:
            return 'STOP · StopZone', '정지 유지', (200, 54, 54)
        return 'RESUMED', '장애물 제거 후 재개', (38, 166, 91)
    if target_ns < events['monitor_scan_frozen']['ros_ns']:
        return 'NAVIGATING', '정상 scan 입력', (38, 116, 190)
    if target_ns < events['stop_state']['ros_ns']:
        return 'SCAN TIMEOUT', 'monitor scan 동결', (224, 144, 32)
    if target_ns < events['do_nothing_after_stop']['ros_ns']:
        return 'STOP · invalid source', '입력 timeout 정지', (200, 54, 54)
    return 'RESUMED', 'scan 복구 후 재개', (38, 166, 91)


def _scenario_state_for_progress(
        scenario: str, progress: float,
) -> tuple[str, str, tuple[int, int, int]]:
    """Show every causal phase even when simulation ROS stamps coincide."""
    if progress >= 1.0:
        return 'GOAL SUCCEEDED', '목표 도달', (38, 166, 91)
    if scenario == 'clear_baseline':
        return 'NAVIGATING', '개입 없이 주행', (38, 116, 190)
    if scenario == 'sudden_obstacle_stop_resume':
        if progress < 0.18:
            return 'NAVIGATING', '장애물 전', (38, 116, 190)
        if progress < 0.32:
            return 'OBSTACLE DETECTED', 'StopZone 감지', (224, 144, 32)
        if progress < 0.58:
            return 'STOP · StopZone', '정지 유지', (200, 54, 54)
        return 'RESUMED', '장애물 제거 후 재개', (38, 166, 91)
    if scenario == 'scan_timeout_stop_resume':
        if progress < 0.18:
            return 'NAVIGATING', '정상 scan 입력', (38, 116, 190)
        if progress < 0.32:
            return 'SCAN TIMEOUT', 'monitor scan 동결', (224, 144, 32)
        if progress < 0.58:
            return 'STOP · invalid source', '입력 timeout 정지', (200, 54, 54)
        return 'RESUMED', 'scan 복구 후 재개', (38, 166, 91)
    raise ValueError(f'unknown scenario: {scenario}')


def _timeline_anchors(
        scenario: str, events: dict[str, dict[str, Any]],
) -> list[tuple[float, int]]:
    """Allocate visible screen time to short stop and recovery transitions."""
    if scenario == 'clear_baseline':
        names = ((0.0, 'goal_accepted'), (1.0, 'succeeded'))
    elif scenario == 'sudden_obstacle_stop_resume':
        names = (
            (0.0, 'goal_accepted'), (0.18, 'obstacle_set_pose_requested'),
            (0.32, 'stop_state'), (0.58, 'do_nothing_after_stop'),
            (1.0, 'succeeded'))
    elif scenario == 'scan_timeout_stop_resume':
        names = (
            (0.0, 'goal_accepted'), (0.18, 'monitor_scan_frozen'),
            (0.32, 'stop_state'), (0.58, 'do_nothing_after_stop'),
            (1.0, 'succeeded'))
    else:
        raise ValueError(f'unknown scenario: {scenario}')
    return [(fraction, events[name]['ros_ns']) for fraction, name in names]


def _target_time_ns(anchors: list[tuple[float, int]], progress: float) -> int:
    for (left_fraction, left_ns), (right_fraction, right_ns) in zip(
            anchors, anchors[1:]):
        if progress <= right_fraction:
            ratio = ((progress - left_fraction)
                     / (right_fraction - left_fraction))
            return round(left_ns + ratio * (right_ns - left_ns))
    return anchors[-1][1]


def _media_fraction(anchors: list[tuple[float, int]], target_ns: int) -> float:
    for (left_fraction, left_ns), (right_fraction, right_ns) in zip(
            anchors, anchors[1:]):
        if target_ns <= right_ns:
            if right_ns == left_ns:
                return right_fraction
            ratio = (target_ns - left_ns) / (right_ns - left_ns)
            return left_fraction + ratio * (right_fraction - left_fraction)
    return 1.0


def _marker_passed(progress: float, marker_fraction: float) -> bool:
    """Use the displayed event phase when source ROS stamps coincide."""
    return progress >= marker_fraction


def _world_to_panel(
        x_m: float, y_m: float, map_size: tuple[int, int],
        origin: tuple[float, float], resolution: float,
        panel: tuple[int, int, int, int],
) -> tuple[float, float]:
    width, height = map_size
    pixel_x = (x_m - origin[0]) / resolution
    pixel_y = height - 1 - (y_m - origin[1]) / resolution
    left, top, panel_width, panel_height = panel
    return (left + pixel_x / width * panel_width,
            top + pixel_y / height * panel_height)


def _draw_frame(
        scenario: str, evidence: dict[str, Any], bag: dict[str, Any],
        map_image: Image.Image, map_yaml: dict[str, Any], frame_index: int,
) -> Image.Image:
    events = _event_map(evidence)
    start_ns = events['goal_accepted']['ros_ns']
    progress = frame_index / (FRAME_COUNT - 1)
    anchors = _timeline_anchors(scenario, events)
    target_ns = _target_time_ns(anchors, progress)
    poses = [pose for pose in bag['poses'] if start_ns <= pose[0] <= target_ns]
    if not poses:
        poses = [min(bag['poses'], key=lambda item: abs(item[0] - target_ns))]
    canvas = Image.new('RGB', (WIDTH, HEIGHT), COLORS['background'])
    draw = ImageDraw.Draw(canvas)
    draw.text((44, 25), {
        'clear_baseline': 'Baseline · 중단 없는 자율주행',
        'sudden_obstacle_stop_resume': 'Sudden obstacle · StopZone 정지와 재개',
        'scan_timeout_stop_resume': 'Scan timeout · invalid source 정지와 복구',
    }[scenario], font=_font(32), fill='#162331')
    notice = 'Gazebo 시뮬레이션 · 승인 MCAP 재생'
    if scenario != 'clear_baseline':
        notice += ' · 이벤트 구간 확대(실제 시간 비율 아님)'
    draw.text((46, 76), notice, font=_font(16), fill='#65727e')
    panel = (45, 115, 820, 164)
    resized = map_image.resize((panel[2], panel[3]), Image.Resampling.NEAREST)
    canvas.paste(resized.convert('RGB'), (panel[0], panel[1]))
    draw.rectangle((panel[0], panel[1], panel[0] + panel[2],
                    panel[1] + panel[3]), outline='#506070', width=2)
    origin = tuple(map_yaml['origin'][:2])
    resolution = float(map_yaml['resolution'])
    points = [_world_to_panel(x, y, map_image.size, origin, resolution, panel)
              for _, x, y in poses]
    if len(points) >= 2:
        draw.line(points, fill=COLORS['path'], width=6)
    current = points[-1]
    draw.ellipse((current[0] - 10, current[1] - 10,
                  current[0] + 10, current[1] + 10),
                 fill=COLORS['robot'], outline='white', width=3)
    goal = _world_to_panel(6.0, 0.0, map_image.size, origin, resolution, panel)
    draw.ellipse((goal[0] - 7, goal[1] - 7, goal[0] + 7, goal[1] + 7),
                 fill=COLORS['goal'])
    if scenario == 'sudden_obstacle_stop_resume':
        active = (events['obstacle_set_pose_requested']['ros_ns']
                  <= target_ns < events['clear_set_pose_requested']['ros_ns'])
        if active:
            state = evidence['entity_states'][0]
            ox, oy = _world_to_panel(
                state['pose_m'][0], state['pose_m'][1], map_image.size,
                origin, resolution, panel)
            draw.rectangle((ox - 18, oy - 14, ox + 18, oy + 14),
                           fill=COLORS['stop'], outline='white', width=2)
    status, korean, color = _scenario_state_for_progress(scenario, progress)
    draw.rounded_rectangle((910, 115, 1235, 279), 18, fill='white',
                           outline='#d2d8de', width=2)
    draw.text((938, 137), 'Collision Monitor', font=_font(20), fill='#596773')
    draw.text((938, 177), status, font=_font(27), fill=color)
    draw.text((938, 224), korean, font=_font(20), fill='#253442')
    draw.text((45, 326), '실제 MCAP ground-truth 경로', font=_font(21),
              fill='#253442')
    if scenario == 'clear_baseline':
        facts = [
            'Collision Monitor intervention: 0',
            'Goal: same UUID · SUCCEEDED',
            'Contact / cancel: 0 / 0']
    elif scenario == 'sudden_obstacle_stop_resume':
        facts = [
            'STOP source: StopZone',
            f"Stop distance: {evidence['stop_distance_m']:.3f} m",
            'Clearance: '
            f"{evidence['footprint_to_obstacle_clearance_m']:.3f} m"]
    else:
        source_age_s = (
            evidence['stop_state_ros_ns']
            - evidence['last_monitor_scan_stamp_ns_at_freeze']) / 1e9
        facts = [
            'STOP source: invalid source',
            f'Source age: {source_age_s:.3f} s',
            'Raw/nav scan continued · monitor scan frozen']
    for index, text in enumerate(facts):
        draw.text((55, 374 + index * 42), text, font=_font(21),
                  fill='#263746')
    line_y = 560
    draw.line((55, line_y, 1225, line_y), fill='#aab4be', width=4)
    markers = ['goal_accepted', 'succeeded']
    if scenario == 'sudden_obstacle_stop_resume':
        markers = ['goal_accepted', 'obstacle_set_pose_requested',
                   'stop_state', 'do_nothing_after_stop', 'succeeded']
    elif scenario == 'scan_timeout_stop_resume':
        markers = ['goal_accepted', 'monitor_scan_frozen', 'stop_state',
                   'do_nothing_after_stop', 'succeeded']
    marker_fractions = [item[0] for item in anchors]
    short_labels = {
        'goal_accepted': 'START',
        'obstacle_set_pose_requested': 'OBSTACLE SET',
        'monitor_scan_frozen': 'SCAN FROZEN',
        'stop_state': 'STOP',
        'do_nothing_after_stop': 'RESUMED',
        'succeeded': 'SUCCESS',
    }
    for index, (name, marker_fraction) in enumerate(zip(
            markers, marker_fractions)):
        x = 55 + marker_fraction * 1170
        passed = _marker_passed(progress, marker_fraction)
        draw.ellipse((x - 7, line_y - 7, x + 7, line_y + 7),
                     fill='#168aad' if passed else '#c9d0d6')
        label_y = line_y + 28 + (index % 3) * 34
        draw.line((x, line_y + 8, x, label_y - 4), fill='#8c98a3', width=1)
        label = short_labels[name]
        draw.text((max(40, min(1135, x - 38)), label_y), label,
                  font=_font(14), fill='#4b5965')
    timeline_x = 55 + progress * 1170
    draw.line((timeline_x, line_y - 18, timeline_x, line_y + 18),
              fill=COLORS['robot'], width=4)
    draw.text((1068, 674), f'{progress * 100:5.1f}%', font=_font(17),
              fill='#52616e')
    return canvas


def _write_mp4(frames: list[Image.Image], output: Path) -> None:
    command = [
        'ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo',
        '-pix_fmt', 'rgb24', '-s', f'{WIDTH}x{HEIGHT}', '-r', str(FPS),
        '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
        '-pix_fmt', 'yuv420p', '-threads', '1', '-map_metadata', '-1',
        '-fflags', '+bitexact', '-flags:v', '+bitexact', str(output)]
    environment = dict(os.environ, LC_ALL='C', LANG='C', TZ='UTC')
    process = subprocess.Popen(
        command, stdin=subprocess.PIPE, env=environment)
    assert process.stdin is not None
    try:
        for frame in frames:
            process.stdin.write(frame.tobytes())
    finally:
        process.stdin.close()
    if process.wait() != 0:
        raise RuntimeError('ffmpeg failed')


def _probe(path: Path) -> dict[str, Any]:
    result = subprocess.run([
        'ffprobe', '-v', 'error', '-select_streams', 'v:0',
        '-show_entries', 'stream=codec_name,width,height,pix_fmt,avg_frame_rate,'
        'nb_frames:format=duration', '-of', 'json', str(path),
    ], capture_output=True, check=True, text=True)
    document = json.loads(result.stdout)
    stream = document['streams'][0]
    if (stream['codec_name'] != 'h264' or stream['width'] != WIDTH
            or stream['height'] != HEIGHT or stream['pix_fmt'] != 'yuv420p'
            or stream['avg_frame_rate'] != f'{FPS}/1'
            or int(stream['nb_frames']) != FRAME_COUNT):
        raise ValueError('rendered MP4 stream contract mismatch')
    return {
        'codec': stream['codec_name'], 'width': stream['width'],
        'height': stream['height'], 'pixel_format': stream['pix_fmt'],
        'fps': FPS, 'frame_count': int(stream['nb_frames']),
        'duration_s': float(document['format']['duration']),
    }


def _probe_gif(path: Path) -> dict[str, Any]:
    result = subprocess.run([
        'ffprobe', '-v', 'error', '-select_streams', 'v:0',
        '-show_entries', 'stream=codec_name,width,height,pix_fmt,nb_frames:'
        'format=duration', '-of', 'json', str(path),
    ], capture_output=True, check=True, text=True)
    document = json.loads(result.stdout)
    stream = document['streams'][0]
    if (stream['codec_name'] != 'gif' or stream['width'] != WIDTH
            or stream['height'] != HEIGHT or int(stream['nb_frames']) != 3):
        raise ValueError('summary GIF stream contract mismatch')
    return {
        'codec': 'gif', 'width': stream['width'], 'height': stream['height'],
        'pixel_format': stream['pix_fmt'],
        'frame_count': int(stream['nb_frames']),
        'duration_s': float(document['format']['duration']),
    }


def _render_policy() -> dict[str, Any]:
    return {
        'width': WIDTH, 'height': HEIGHT, 'fps': FPS,
        'frame_count': FRAME_COUNT, 'duration_s': FRAME_COUNT / FPS,
        'selection': 'predeclared_seed_11_three_scenarios',
        'visual_type': 'actual_map_path_state_transition_overlay',
        'claim_scope': 'same-host simulation evidence; not physical safety',
        'font': _input_record('font', FONT_PATH),
        'colors': COLORS,
        'locale': 'C', 'timezone': 'UTC',
        'ffmpeg_video': {
            'codec': 'libx264', 'preset': 'medium', 'crf': 20,
            'pixel_format': 'yuv420p', 'threads': 1,
            'metadata_stripped': True, 'bitexact': True,
        },
        'time_warp': 'piecewise_event_anchors_for_dynamic_scenarios',
    }


def render(matrix_root: Path, representative_root: Path, output: Path) -> None:
    if verify_promotion(representative_root)['status'] != 'PASS':
        raise ValueError('representative promotion must pass before rendering')
    if output.exists():
        raise FileExistsError(f'output already exists: {output}')
    output.mkdir(parents=True)
    map_path = matrix_root / 'assets/slam_corridor_eval.pgm'
    map_yaml_path = matrix_root / 'assets/slam_corridor_eval.yaml'
    map_image = Image.open(map_path).convert('L')
    map_yaml = yaml.safe_load(map_yaml_path.read_text())
    input_pairs = [
        ('matrix_contract', matrix_root / 'contract.json'),
        ('matrix_aggregate', matrix_root / 'aggregate.json'),
        ('matrix_final_summary', matrix_root / 'final_summary.json'),
        ('matrix_runtime_manifest', matrix_root / 'runtime_manifest.json'),
        ('promotion_manifest', representative_root / 'promotion_manifest.json'),
        ('occupancy_map', map_path), ('occupancy_map_yaml', map_yaml_path),
        ('renderer_source', Path(__file__).resolve()), ('font', FONT_PATH)]
    posters = []
    output_records = []
    scenario_bindings = []
    for scenario in SCENARIOS:
        run_id = f'{scenario}__seed_11'
        run_dir = representative_root / run_id
        evidence_path = run_dir / 'evidence.json'
        evaluation_path = run_dir / 'evaluation.json'
        metadata_path = run_dir / 'bag/metadata.yaml'
        bag_path = run_dir / 'bag/bag_0.mcap'
        input_pairs.extend((
            (f'{scenario}_evidence', evidence_path),
            (f'{scenario}_evaluation', evaluation_path),
            (f'{scenario}_metadata', metadata_path),
            (f'{scenario}_bag', bag_path)))
        evidence = strict_json_loads(evidence_path.read_text())
        bag = _load_bag(run_dir, evidence)
        event_map = _event_map(evidence)
        scenario_bindings.append({
            'scenario': scenario, 'run_id': run_id,
            'goal_uuid': evidence['goal_uuid'],
            'topic_counts': bag['topic_counts'],
            'time_warp_anchors': [
                {'media_fraction': fraction, 'source_ros_ns': ros_ns}
                for fraction, ros_ns in _timeline_anchors(
                    scenario, event_map)],
        })
        frames = [_draw_frame(
            scenario, evidence, bag, map_image, map_yaml, index)
            for index in range(FRAME_COUNT)]
        media_path = output / f'{scenario}__seed_11.mp4'
        _write_mp4(frames, media_path)
        output_records.append({**_record(media_path, output),
                               'probe': _probe(media_path)})
        poster_index = FRAME_COUNT // 2 if scenario != 'clear_baseline' else 48
        posters.append(frames[poster_index])
    gif_path = output / 'collision_monitor_scenarios_summary.gif'
    posters[0].save(
        gif_path, save_all=True, append_images=posters[1:], duration=1800,
        loop=0, optimize=False, disposal=2)
    with Image.open(gif_path) as gif:
        if gif.size != (WIDTH, HEIGHT) or getattr(gif, 'n_frames', 1) != 3:
            raise ValueError('summary GIF contract mismatch')
    output_records.append({**_record(gif_path, output),
                           'probe': _probe_gif(gif_path)})
    if verify_promotion(representative_root)['status'] != 'PASS':
        raise ValueError('representative promotion changed during rendering')
    manifest = {
        'schema_version': 1,
        'render_policy': _render_policy(),
        'inputs': [_input_record(kind, path) for kind, path in input_pairs],
        'outputs': output_records,
        'scenario_bindings': scenario_bindings,
        'verification': {
            'source_matrix_fresh_pre_post': True,
            'promotion_fresh_pre_post': True,
            'clean_rerender_byte_equal': True,
            'root_exact_set_no_symlinks': True,
            'media_size_cap_bytes': MEDIA_LIMIT_BYTES,
        },
    }
    (output / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n')


def _output_hashes(output: Path) -> dict[str, str]:
    return {path.name: sha256_file(path) for path in sorted(output.iterdir())
            if path.name != 'manifest.json'}


def _expected_input_pairs(
        matrix_root: Path, representative_root: Path,
) -> list[tuple[str, Path]]:
    pairs = [
        ('matrix_contract', matrix_root / 'contract.json'),
        ('matrix_aggregate', matrix_root / 'aggregate.json'),
        ('matrix_final_summary', matrix_root / 'final_summary.json'),
        ('matrix_runtime_manifest', matrix_root / 'runtime_manifest.json'),
        ('promotion_manifest', representative_root / 'promotion_manifest.json'),
        ('occupancy_map', matrix_root / 'assets/slam_corridor_eval.pgm'),
        ('occupancy_map_yaml', matrix_root / 'assets/slam_corridor_eval.yaml'),
        ('renderer_source', Path(__file__).resolve()), ('font', FONT_PATH)]
    for scenario in SCENARIOS:
        run = representative_root / f'{scenario}__seed_11'
        pairs.extend((
            (f'{scenario}_evidence', run / 'evidence.json'),
            (f'{scenario}_evaluation', run / 'evaluation.json'),
            (f'{scenario}_metadata', run / 'bag/metadata.yaml'),
            (f'{scenario}_bag', run / 'bag/bag_0.mcap')))
    return pairs


def _validate_stored(
        matrix_root: Path, representative_root: Path, output: Path,
) -> dict[str, Any]:
    if (output.is_symlink() or not output.is_dir()
            or output.resolve() != output.absolute()):
        raise ValueError('media output root must be a canonical directory')
    if verify_promotion(representative_root)['status'] != 'PASS':
        raise ValueError('representative promotion changed after rendering')
    manifest = strict_json_loads((output / 'manifest.json').read_text())
    if set(manifest) != {
            'schema_version', 'render_policy', 'inputs', 'outputs',
            'scenario_bindings', 'verification'}:
        raise ValueError('media manifest schema mismatch')
    if manifest['schema_version'] != 1:
        raise ValueError('media manifest version mismatch')
    if manifest['render_policy'] != _render_policy():
        raise ValueError('media render policy mismatch')
    expected_inputs = [_input_record(kind, path) for kind, path in (
        _expected_input_pairs(matrix_root, representative_root))]
    if manifest['inputs'] != expected_inputs:
        raise ValueError('media input identity mismatch')
    expected_names = {
        f'{scenario}__seed_11.mp4' for scenario in SCENARIOS}
    expected_names |= {'collision_monitor_scenarios_summary.gif'}
    output_names = {record['relative_path'] for record in manifest['outputs']}
    if output_names != expected_names or len(manifest['outputs']) != 4:
        raise ValueError('media output manifest mismatch')
    actual_entries = set(output.iterdir())
    expected_entries = {output / 'manifest.json'} | {
        output / name for name in expected_names}
    if (actual_entries != expected_entries
            or any(path.is_symlink() or not path.is_file()
                   for path in actual_entries)):
        raise ValueError('media root exact-set mismatch')
    for record in manifest['outputs']:
        path = output / record['relative_path']
        if (_record(path, output)['size_bytes'] != record['size_bytes']
                or sha256_file(path) != record['sha256']):
            raise ValueError('media output identity mismatch')
        if path.suffix == '.mp4':
            probe = _probe(path)
        else:
            probe = _probe_gif(path)
        if probe != record['probe']:
            raise ValueError('media ffprobe identity mismatch')
    total = sum(path.stat().st_size for path in actual_entries)
    if total > MEDIA_LIMIT_BYTES:
        raise ValueError('media root exceeds 128 MiB')
    expected_verification = {
        'source_matrix_fresh_pre_post': True,
        'promotion_fresh_pre_post': True,
        'clean_rerender_byte_equal': True,
        'root_exact_set_no_symlinks': True,
        'media_size_cap_bytes': MEDIA_LIMIT_BYTES,
    }
    if manifest['verification'] != expected_verification:
        raise ValueError('media verification flags mismatch')
    expected_bindings = []
    for scenario in SCENARIOS:
        run_id = f'{scenario}__seed_11'
        run_dir = representative_root / run_id
        evidence = strict_json_loads((run_dir / 'evidence.json').read_text())
        bag = _load_bag(run_dir, evidence)
        expected_bindings.append({
            'scenario': scenario, 'run_id': run_id,
            'goal_uuid': evidence['goal_uuid'],
            'topic_counts': bag['topic_counts'],
            'time_warp_anchors': [
                {'media_fraction': fraction, 'source_ros_ns': ros_ns}
                for fraction, ros_ns in _timeline_anchors(
                    scenario, _event_map(evidence))],
        })
    if manifest['scenario_bindings'] != expected_bindings:
        raise ValueError('media scenario binding set mismatch')
    return manifest


def verify_deterministic(
        matrix_root: Path, representative_root: Path, output: Path,
) -> None:
    manifest = _validate_stored(matrix_root, representative_root, output)
    expected = {record['relative_path']: record['sha256']
                for record in manifest['outputs']}
    if _output_hashes(output) != expected:
        raise ValueError('stored media hash mismatch')
    with tempfile.TemporaryDirectory(prefix='g004-media-verify-') as temp:
        replay = Path(temp) / 'media'
        render(matrix_root, representative_root, replay)
        _validate_stored(matrix_root, representative_root, replay)
        if _output_hashes(replay) != expected:
            raise ValueError('media is not deterministically reproducible')


def render_atomic(
        matrix_root: Path, representative_root: Path, output: Path,
) -> None:
    if output.exists():
        raise FileExistsError(f'output already exists: {output}')
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(
        prefix=f'.{output.name}.', dir=output.parent)) / 'media'
    try:
        render(matrix_root, representative_root, stage)
        verify_deterministic(matrix_root, representative_root, stage)
        stage.rename(output)
    finally:
        if stage.parent.exists():
            shutil.rmtree(stage.parent)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--matrix-root', type=Path, required=True)
    parser.add_argument('--representative-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--verify-existing', action='store_true')
    args = parser.parse_args()
    if args.verify_existing:
        verify_deterministic(
            args.matrix_root, args.representative_root, args.output_dir)
    else:
        render_atomic(
            args.matrix_root, args.representative_root, args.output_dir)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
