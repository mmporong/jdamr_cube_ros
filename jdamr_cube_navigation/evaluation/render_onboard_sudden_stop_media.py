#!/usr/bin/env python3
"""Render a mobile-manipulator sudden-stop story from one PASS smoke run."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
from typing import Any

import matplotlib
from matplotlib import font_manager  # noqa: I100
FONT_PATH = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
font_manager.fontManager.addfont(str(FONT_PATH))
matplotlib.rcParams['font.family'] = font_manager.FontProperties(
    fname=str(FONT_PATH)).get_name()
matplotlib.use('Agg')
import matplotlib.animation as animation  # noqa: E402,I100
import matplotlib.patches as patches  # noqa: E402,I100
import matplotlib.pyplot as plt  # noqa: E402,I100
import numpy as np  # noqa: E402,I201
from PIL import Image  # noqa: E402,I100,I201

from navigation_mcap_reader import read_navigation_messages  # noqa: E402,I100

import yaml  # noqa: E402,I100,I201


FRAME_COUNT = 72
FPS = 8


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _stamp_ns(stamp: Any) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _yaw(rotation: Any) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))


def _read_story(mcap: Path) -> dict[str, list]:
    story = {'gt': [], 'plans': []}
    for item in read_navigation_messages(
            mcap, topics=['/ground_truth_pose', '/plan']):
        message = item.ros_msg
        if item.channel.topic == '/ground_truth_pose':
            pose = message.pose
            story['gt'].append((
                _stamp_ns(message.header.stamp), float(pose.position.x),
                float(pose.position.y), _yaw(pose.orientation)))
        else:
            story['plans'].append((
                _stamp_ns(message.header.stamp),
                [(float(pose.pose.position.x), float(pose.pose.position.y))
                 for pose in message.poses]))
    if not story['gt'] or not story['plans']:
        raise ValueError('MCAP lacks ground truth or Nav2 plan')
    story['gt'].sort()
    story['plans'].sort()
    return story


def _latest(samples: list, timestamp_ns: int):
    candidates = [sample for sample in samples if sample[0] <= timestamp_ns]
    return candidates[-1] if candidates else samples[0]


def _event_map(document: dict) -> dict[str, dict]:
    result = {event['name']: event for event in document['events']}
    required = {
        'goal_accepted', 'obstacle_set_pose_requested', 'stop_state',
        'clear_set_pose_requested', 'succeeded'}
    if not required <= set(result):
        missing = sorted(required - set(result))
        raise ValueError(f'missing media events: {missing}')
    return result


def _anchors(events: dict[str, dict]) -> list[tuple[float, int]]:
    return [
        (0.0, events['goal_accepted']['ros_ns']),
        (0.20, events['obstacle_set_pose_requested']['ros_ns']),
        (0.34, events['stop_state']['ros_ns']),
        (0.60, events['clear_set_pose_requested']['ros_ns']),
        (1.0, events['succeeded']['ros_ns']),
    ]


def _target_ns(anchors: list[tuple[float, int]], progress: float) -> int:
    for (left_fraction, left_ns), (right_fraction, right_ns) in zip(
            anchors, anchors[1:]):
        if progress <= right_fraction:
            ratio = ((progress - left_fraction)
                     / (right_fraction - left_fraction))
            return round(left_ns + ratio * (right_ns - left_ns))
    return anchors[-1][1]


def _state(progress: float) -> tuple[str, str, str]:
    if progress >= 1.0:
        return 'GOAL SUCCEEDED', '동일 목표로 최종 도착', '#22c55e'
    if progress < 0.20:
        return 'NAVIGATING', '장애물 출현 전', '#38bdf8'
    if progress < 0.34:
        return 'OBSTACLE DETECTED', '팔 외곽 앞에서 정지 명령', '#f59e0b'
    if progress < 0.60:
        return 'STOP · StopZone', '접촉 없이 정지 유지', '#ef4444'
    return 'RESUMED', '장애물 제거 후 목표 유지', '#22c55e'


def _map_extent(metadata: dict, image: np.ndarray) -> tuple[float, ...]:
    resolution_m = float(metadata['resolution'])
    origin_x_m, origin_y_m = metadata['origin'][:2]
    return (origin_x_m, origin_x_m + image.shape[1] * resolution_m,
            origin_y_m, origin_y_m + image.shape[0] * resolution_m)


def _rectangle(axis, zone: dict, color: str, alpha: float, label: str):
    rear_m, front_m = zone['rear_m'], zone['front_m']
    half_width_m = zone['half_width_m']
    axis.add_patch(patches.Rectangle(
        (rear_m, -half_width_m), front_m - rear_m, 2.0 * half_width_m,
        linewidth=2, edgecolor=color, facecolor=color, alpha=alpha,
        label=label))


def _source_record(path: Path) -> dict:
    return {'path': str(path.resolve()), 'size_bytes': path.stat().st_size,
            'sha256': _sha256(path)}


def render(run_root: Path, output: Path) -> dict:
    """Create a spatial story, compact GIF, metrics, and hash manifest."""
    output.mkdir(parents=True, exist_ok=False)
    summary_path = run_root / 'summary.json'
    summary = json.loads(summary_path.read_text())
    if summary.get('status') != 'PASS' or len(summary.get('results', [])) != 1:
        raise ValueError('media input must be one PASS smoke run')
    result = summary['results'][0]
    if result.get('case') != 'sudden_stop_resume':
        raise ValueError('media input is not the sudden-stop case')
    case_root = run_root / 'sudden_stop_resume'
    scenario_path = case_root / 'scenario.json'
    scenario = json.loads(scenario_path.read_text())
    contract = scenario['contract']
    mcap = Path(result['recording']['mcap']['path'])
    if _sha256(mcap) != result['recording']['mcap']['sha256']:
        raise ValueError('source MCAP hash mismatch')
    story = _read_story(mcap)
    events = _event_map(scenario)
    anchors = _anchors(events)

    zones_path = run_root / 'assets/sim_keepout_zones.yaml'
    map_yaml_path = Path(yaml.safe_load(
        zones_path.read_text())['map_yaml'])
    map_metadata = yaml.safe_load(map_yaml_path.read_text())
    map_image_path = (map_yaml_path.parent / map_metadata['image']).resolve()
    map_image = np.asarray(Image.open(map_image_path))
    extent = _map_extent(map_metadata, map_image)
    goal = contract['goal_pose']
    obstacle = scenario['entity_states'][0]['pose_m']
    obstacle_length_m = contract['sudden_obstacle']['length_m']
    obstacle_width_m = contract['sudden_obstacle']['width_m']
    base_inputs = contract['stop_zone']['inputs']
    base_footprint = {
        'front_m': base_inputs['footprint_front_m'],
        'rear_m': base_inputs['footprint_rear_m'],
        'half_width_m': base_inputs['footprint_half_width_m'],
    }
    arm = contract['travel_pose_envelope']['bounds_m']
    timestamps_ns = [
        _target_ns(anchors, frame / (FRAME_COUNT - 1))
        for frame in range(FRAME_COUNT)]
    figure, (corridor, geometry) = plt.subplots(
        1, 2, figsize=(12.8, 7.2), dpi=100,
        gridspec_kw={'width_ratios': [1.45, 1.0]})
    figure.patch.set_facecolor('#0f172a')

    def draw(frame: int) -> None:
        progress = frame / (FRAME_COUNT - 1)
        timestamp_ns = timestamps_ns[frame]
        corridor.clear()
        geometry.clear()
        visible_gt = [sample for sample in story['gt']
                      if sample[0] <= timestamp_ns]
        if not visible_gt:
            visible_gt = [story['gt'][0]]
        robot = visible_gt[-1]
        plan = _latest(story['plans'], timestamp_ns)[1]
        active = 0.20 <= progress < 0.60
        title, subtitle, color = _state(progress)
        protected_clearance_cm = (
            scenario['protected_envelope_to_obstacle_clearance_m'] * 100.0)

        corridor.imshow(
            map_image, cmap='gray', origin='lower', extent=extent, alpha=0.78)
        corridor.plot([sample[1] for sample in visible_gt],
                      [sample[2] for sample in visible_gt],
                      color='#38bdf8', linewidth=3,
                      label='Gazebo ground truth')
        corridor.plot([point[0] for point in plan],
                      [point[1] for point in plan], color='#facc15',
                      linewidth=1.4, alpha=0.85, label='Nav2 plan')
        corridor.scatter([goal['x_m']], [goal['y_m']], marker='*', s=180,
                         color='#22c55e', label='Goal')
        corridor.arrow(robot[1], robot[2], 0.32 * math.cos(robot[3]),
                       0.32 * math.sin(robot[3]), color='#f97316',
                       width=0.025, length_includes_head=True)
        corridor.scatter([robot[1]], [robot[2]], s=85, color='#f97316')
        if active:
            corridor.add_patch(patches.Rectangle(
                (obstacle[0] - obstacle_length_m / 2.0,
                 obstacle[1] - obstacle_width_m / 2.0),
                obstacle_length_m, obstacle_width_m,
                facecolor='#ef4444', edgecolor='white', linewidth=1.5,
                label='Sudden obstacle'))
        corridor.set(xlim=(-9.0, 7.0), ylim=(-2.1, 2.1))
        corridor.set_aspect('equal')
        corridor.set_title('Actual corridor path and event', color='#e2e8f0')
        corridor.legend(loc='lower left', fontsize=8)

        _rectangle(geometry, contract['slowdown_zone'], '#eab308', 0.10,
                   'SlowdownZone 0.50 m')
        _rectangle(geometry, contract['stop_zone'], '#ef4444', 0.12,
                   'StopZone 0.40 m')
        _rectangle(geometry, base_footprint, '#38bdf8', 0.35,
                   'Base footprint 0.23 m')
        _rectangle(geometry, {
            'front_m': arm['front_m'], 'rear_m': arm['rear_m'],
            'half_width_m': arm['half_width_m'],
        }, '#a855f7', 0.55, 'Stowed arm 0.303 m')
        relative_obstacle_x = obstacle[0] - robot[1]
        if active:
            geometry.add_patch(patches.Rectangle(
                (relative_obstacle_x - obstacle_length_m / 2.0,
                 obstacle[1] - robot[2] - obstacle_width_m / 2.0),
                obstacle_length_m, obstacle_width_m,
                facecolor='#ef4444', edgecolor='white', linewidth=2,
                label='Obstacle'))
        else:
            geometry.axvline(
                relative_obstacle_x - obstacle_length_m / 2.0,
                color='#ef4444', linestyle=':', alpha=0.45)
        geometry.axvline(0.0, color='#e2e8f0', linewidth=1)
        geometry.arrow(0.0, 0.0, 0.12, 0.0, width=0.008,
                       color='#f97316', length_includes_head=True)
        geometry.set(xlim=(-0.35, 0.80), ylim=(-0.48, 0.48),
                     xlabel='base_footprint 기준 전방 거리 [m]',
                     ylabel='좌우 거리 [m]')
        geometry.set_aspect('equal')
        geometry.set_title('Arm-aware protective geometry', color='#e2e8f0')
        geometry.legend(loc='upper right', fontsize=7)
        geometry.text(
            0.03, 0.03,
            f'Protected clearance: {protected_clearance_cm:.1f} cm\n'
            f'Contact: {scenario["contact_count"]} · goal UUID: same\n'
            f'Scan→zero command: '
            f'{result["scenario"]["observer_scan_to_zero_command_s"]:.3f} s',
            transform=geometry.transAxes, fontsize=9,
            bbox={'facecolor': 'white', 'alpha': 0.88, 'edgecolor': 'none'})
        figure.suptitle(
            f'{title} — {subtitle}\n'
            'Gazebo simulation replay · event intervals are time-expanded',
            color=color, fontsize=16)
        figure.tight_layout()

    movie = animation.FuncAnimation(
        figure, draw, frames=FRAME_COUNT, interval=1000 / FPS)
    mp4 = output / 'sudden_stop_same_goal_story.mp4'
    movie.save(mp4, writer=animation.FFMpegWriter(
        fps=FPS, bitrate=1600, codec='libx264',
        extra_args=['-pix_fmt', 'yuv420p']))
    draw(round((FRAME_COUNT - 1) * 0.45))
    poster = output / 'sudden_stop_protected_clearance.png'
    figure.savefig(poster, dpi=100)
    plt.close(figure)
    gif = output / 'sudden_stop_same_goal_story.gif'
    subprocess.run([
        'ffmpeg', '-y', '-loglevel', 'error', '-i', str(mp4),
        '-vf', 'fps=6,scale=768:-1:flags=lanczos', '-loop', '0', str(gif),
    ], check=True)

    metrics = output / 'metrics.json'
    metrics.write_text(json.dumps({
        'schema_version': 1,
        'claim_scope': 'GAZEBO_SIM_INTEGRATION_ONLY',
        'physical_robot_safety': 'NOT_CLAIMED',
        'candidate_zones_m': {
            'stop_front': contract['stop_zone']['front_m'],
            'slowdown_front': contract['slowdown_zone']['front_m'],
        },
        'production_stop_zone_deficit_m': contract[
            'travel_pose_envelope']['production_stop_zone_deficit_m'],
        'stowed_arm_front_m': arm['front_m'],
        'base_clearance_m': scenario[
            'footprint_to_obstacle_clearance_m'],
        'protected_envelope_clearance_m': scenario[
            'protected_envelope_to_obstacle_clearance_m'],
        'observer_scan_to_zero_command_s': result['scenario'][
            'observer_scan_to_zero_command_s'],
        'robot_contact_count': scenario['contact_count'],
        'goal_send_count': scenario['goal_send_count'],
        'goal_cancel_count': scenario['goal_cancel_count'],
        'action_terminal': scenario['action_terminal'],
        'same_goal_command_verdict': result['scenario'][
            'same_goal_command_verdict'],
        'travel_pose_maximum_error_rad': result['travel_pose'][
            'maximum_error_rad'],
        'source_run_summary_sha256': _sha256(summary_path),
        'source_scenario_sha256': _sha256(scenario_path),
        'source_mcap_sha256': _sha256(mcap),
    }, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    generated = []
    for path in sorted(output.iterdir()):
        if path.name != 'manifest.json':
            generated.append({
                'name': path.name, 'size_bytes': path.stat().st_size,
                'sha256': _sha256(path)})
    manifest = output / 'manifest.json'
    manifest.write_text(json.dumps({
        'schema_version': 1,
        'generator': _source_record(Path(__file__)),
        'sources': [
            _source_record(summary_path), _source_record(scenario_path),
            _source_record(mcap), _source_record(map_yaml_path),
            _source_record(map_image_path),
        ],
        'generated': generated,
        'render_policy': {
            'frame_count': FRAME_COUNT, 'fps': FPS,
            'duration_s': FRAME_COUNT / FPS,
            'event_time_expansion': True,
            'visual': 'actual_path_plus_arm_aware_protective_geometry',
        },
    }, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return json.loads(manifest.read_text())


def main() -> int:
    """Parse input paths and render the verified story."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(render(args.run_root, args.output), sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
