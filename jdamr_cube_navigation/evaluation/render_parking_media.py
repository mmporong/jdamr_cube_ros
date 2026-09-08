#!/usr/bin/env python3
"""Render a spatial and temporal parking story from one verified MCAP."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
from typing import Any

import matplotlib
matplotlib.use('Agg')
import matplotlib.animation as animation  # noqa: E402,I100
import matplotlib.pyplot as plt  # noqa: E402,I100
import numpy as np  # noqa: E402,I201
from PIL import Image  # noqa: E402,I100,I201

from navigation_mcap_reader import read_navigation_messages  # noqa: E402,I100

import yaml  # noqa: E402,I100,I201


FRAME_COUNT = 80
FPS = 8
GOAL_COLOR = '#22c55e'
GROUND_TRUTH_COLOR = '#38bdf8'
ESTIMATE_COLOR = '#f59e0b'


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _yaw(rotation: Any) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))


def _yaw_error_deg(actual_rad: float, target_rad: float) -> float:
    difference_rad = actual_rad - target_rad
    return math.degrees(abs(math.atan2(
        math.sin(difference_rad), math.cos(difference_rad))))


def _read_story(mcap: Path, goal: tuple[float, float, float]) -> dict:
    story = {'gt': [], 'amcl': [], 'cmd': [], 'plans': []}
    topics = ['/ground_truth_pose', '/amcl_pose', '/cmd_vel', '/plan']
    first_ns = None
    for item in read_navigation_messages(mcap, topics=topics):
        first_ns = item.log_time_ns if first_ns is None else first_ns
        elapsed_s = (item.log_time_ns - first_ns) * 1e-9
        message = item.ros_msg
        if item.channel.topic == '/ground_truth_pose':
            pose = message.pose
            story['gt'].append((elapsed_s, float(pose.position.x),
                                float(pose.position.y),
                                _yaw(pose.orientation)))
        elif item.channel.topic == '/amcl_pose':
            pose = message.pose.pose
            story['amcl'].append((elapsed_s, float(pose.position.x),
                                  float(pose.position.y),
                                  _yaw(pose.orientation)))
        elif item.channel.topic == '/cmd_vel':
            story['cmd'].append((elapsed_s, float(message.linear.x),
                                 float(message.angular.z)))
        else:
            story['plans'].append((elapsed_s, [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in message.poses]))
    if any(not story[name] for name in story):
        raise ValueError('parking MCAP lacks a required media topic')
    moving = [item[0] for item in story['cmd']
              if abs(item[1]) > 0.005 or abs(item[2]) > 0.005]
    if not moving:
        raise ValueError('parking MCAP contains no movement')
    story['start_s'] = max(0.0, min(moving) - 1.0)
    story['end_s'] = min(story['gt'][-1][0], max(moving) + 2.0)
    goal_x_m, goal_y_m, goal_yaw_rad = goal
    story['gt_errors'] = [
        (item[0], math.hypot(item[1] - goal_x_m,
                             item[2] - goal_y_m),
         _yaw_error_deg(item[3], goal_yaw_rad))
        for item in story['gt']
    ]
    story['amcl_errors'] = [
        (item[0], math.hypot(item[1] - goal_x_m,
                             item[2] - goal_y_m),
         _yaw_error_deg(item[3], goal_yaw_rad))
        for item in story['amcl']
    ]
    return story


def _latest(samples: list, timestamp_s: float):
    candidates = [sample for sample in samples if sample[0] <= timestamp_s]
    return candidates[-1] if candidates else samples[0]


def _map_extent(metadata: dict, image: np.ndarray) -> tuple[float, ...]:
    resolution_m = float(metadata['resolution'])
    origin_x_m, origin_y_m = metadata['origin'][:2]
    return (origin_x_m, origin_x_m + image.shape[1] * resolution_m,
            origin_y_m, origin_y_m + image.shape[0] * resolution_m)


def _read_parking_confirmation(route_log: Path) -> dict:
    marker = 'route_event '
    for line in reversed(route_log.read_text(errors='replace').splitlines()):
        if marker not in line:
            continue
        try:
            event = json.loads(line.split(marker, 1)[1])
        except json.JSONDecodeError:
            continue
        if event.get('event') == 'parking_estimate_confirmed':
            return event
    raise ValueError('route log lacks parking_estimate_confirmed evidence')


def render(run_root: Path, output: Path) -> dict:
    """Create MP4, GIF, final overlay, metrics and a hashed manifest."""
    output.mkdir(parents=True, exist_ok=False)
    summary = json.loads((run_root / 'summary.json').read_text())
    if summary.get('status') != 'PASS':
        raise ValueError('media input must be a PASS parking run')
    mcap = Path(summary['recording']['mcap']['path'])
    route = yaml.safe_load(
        (run_root / 'assets/parking_route.yaml').read_text())
    target = route['waypoints'][-1]
    goal = (float(target['x']), float(target['y']), float(target['yaw']))
    story = _read_story(mcap, goal)
    confirmed_tf = _read_parking_confirmation(run_root / 'route.log')
    map_yaml_path = Path(route['map_yaml'])
    map_metadata = yaml.safe_load(map_yaml_path.read_text())
    map_image_path = (map_yaml_path.parent / map_metadata['image']).resolve()
    map_image = np.asarray(Image.open(map_image_path))
    extent = _map_extent(map_metadata, map_image)
    timestamps_s = np.linspace(story['start_s'], story['end_s'], FRAME_COUNT)
    figure, (spatial, errors) = plt.subplots(
        1, 2, figsize=(12.8, 7.2), dpi=100,
        gridspec_kw={'width_ratios': [1.45, 1.0]})
    figure.patch.set_facecolor('#0f172a')

    def draw(frame: int) -> None:
        timestamp_s = float(timestamps_s[frame])
        spatial.clear()
        errors.clear()
        spatial.imshow(map_image, cmap='gray', origin='lower', extent=extent,
                       alpha=0.78)
        gt = [item for item in story['gt'] if item[0] <= timestamp_s]
        amcl = [item for item in story['amcl'] if item[0] <= timestamp_s]
        spatial.plot([item[1] for item in gt], [item[2] for item in gt],
                     color=GROUND_TRUTH_COLOR, linewidth=3,
                     label='Gazebo ground truth')
        spatial.plot([item[1] for item in amcl], [item[2] for item in amcl],
                     color=ESTIMATE_COLOR, linewidth=2, linestyle='--',
                     label='AMCL estimate')
        plan = _latest(story['plans'], timestamp_s)[1]
        spatial.plot([point[0] for point in plan],
                     [point[1] for point in plan], color='#facc15',
                     linewidth=1.5, alpha=0.8, label='Nav2 plan')
        spatial.add_patch(plt.Circle(
            goal[:2], 0.05, fill=False, color=GOAL_COLOR, linewidth=3,
            label='5 cm target'))
        spatial.arrow(goal[0], goal[1], 0.30 * math.cos(goal[2]),
                      0.30 * math.sin(goal[2]), color=GOAL_COLOR,
                      width=0.012, length_includes_head=True)
        robot = _latest(story['gt'], timestamp_s)
        spatial.arrow(robot[1], robot[2], 0.23 * math.cos(robot[3]),
                      0.23 * math.sin(robot[3]), color='#f97316',
                      width=0.015, length_includes_head=True)
        spatial.scatter([robot[1]], [robot[2]], s=80, color='#f97316')
        spatial.set(xlim=(-6.7, -5.2), ylim=(-1.35, 1.35))
        spatial.set_title(
            'Precision parking: path and final heading', color='#e2e8f0')
        spatial.set_aspect('equal')
        spatial.legend(loc='lower left', fontsize=8)

        for samples, color, label in (
                (story['gt_errors'], GROUND_TRUTH_COLOR, 'GT'),
                (story['amcl_errors'], ESTIMATE_COLOR, 'AMCL')):
            visible = [item for item in samples if item[0] <= timestamp_s]
            errors.plot([item[0] for item in visible],
                        [item[1] * 100.0 for item in visible], color=color,
                        linewidth=2, label=f'{label} position [cm]')
            errors.plot([item[0] for item in visible],
                        [item[2] for item in visible], color=color,
                        linewidth=1.5, linestyle=':',
                        label=f'{label} yaw [deg]')
        errors.axhline(5.0, color='#ef4444', linewidth=1,
                       label='position limit 5 cm')
        errors.axhline(3.0, color='#a855f7', linewidth=1,
                       label='yaw limit 3 deg')
        errors.axvline(timestamp_s, color='#94a3b8', linewidth=1)
        errors.set(xlim=(story['start_s'], story['end_s']), ylim=(0, 100),
                   xlabel='MCAP elapsed wall time [s]')
        errors.set_title('Target error over time', color='#e2e8f0')
        errors.text(
            0.03, 0.04,
            'Final GT: '
            f'{story["gt_errors"][-1][1] * 100.0:.2f} cm / '
            f'{story["gt_errors"][-1][2]:.2f} deg\n'
            'Confirmed map→base TF: '
            f'{confirmed_tf["position_error_m"] * 100.0:.2f} cm / '
            f'{math.degrees(confirmed_tf["yaw_error_rad"]):.2f} deg',
            transform=errors.transAxes, fontsize=8,
            bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': 'none'})
        errors.legend(loc='upper right', fontsize=7)
        figure.suptitle(
            'Gazebo simulation replay — not physical-robot validation',
            color='#e2e8f0', fontsize=14)
        figure.tight_layout()

    movie = animation.FuncAnimation(
        figure, draw, frames=FRAME_COUNT, interval=1000 / FPS)
    mp4 = output / 'parking_seed11_story.mp4'
    movie.save(mp4, writer=animation.FFMpegWriter(
        fps=FPS, bitrate=1600, codec='libx264'))
    draw(FRAME_COUNT - 1)
    overlay = output / 'parking_seed11_final.png'
    figure.savefig(overlay, dpi=100)
    plt.close(figure)
    gif = output / 'parking_seed11_story.gif'
    subprocess.run([
        'ffmpeg', '-y', '-loglevel', 'error', '-i', str(mp4),
        '-vf', 'fps=6,scale=768:-1:flags=lanczos', '-loop', '0', str(gif),
    ], check=True)
    final_gt = story['gt_errors'][-1]
    final_amcl = story['amcl_errors'][-1]
    metrics = output / 'metrics.json'
    metrics.write_text(json.dumps({
        'schema_version': 2,
        'claim_scope': 'GAZEBO_SIMULATION_ONLY',
        'physical_accuracy': 'NOT_MEASURED',
        'target': {'x_m': goal[0], 'y_m': goal[1],
                   'yaw_rad': goal[2], 'position_limit_m': 0.05,
                   'yaw_limit_deg': 3.0},
        'final_ground_truth': {
            'position_error_m': final_gt[1], 'yaw_error_deg': final_gt[2]},
        'confirmed_map_base_tf': {
            'position_error_m': confirmed_tf['position_error_m'],
            'yaw_error_deg': math.degrees(confirmed_tf['yaw_error_rad']),
            'stationary_hold_s': confirmed_tf['hold_s']},
        'last_recorded_amcl_pose': {
            'position_error_m': final_amcl[1],
            'yaw_error_deg': final_amcl[2]},
        'source_run_summary_sha256': _sha256(run_root / 'summary.json'),
        'source_mcap_sha256': _sha256(mcap),
    }, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    generated = []
    for path in sorted(output.iterdir()):
        if path.name == 'manifest.json':
            continue
        generated.append({'name': path.name, 'size_bytes': path.stat().st_size,
                          'sha256': _sha256(path)})
    manifest = output / 'manifest.json'
    manifest.write_text(json.dumps({
        'schema_version': 1,
        'generator': _source_record(Path(__file__)),
        'source_run': _source_record(run_root / 'summary.json'),
        'source_mcap': _source_record(mcap),
        'generated': generated,
    }, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return json.loads(manifest.read_text())


def _source_record(path: Path) -> dict:
    return {'path': str(path.resolve()), 'size_bytes': path.stat().st_size,
            'sha256': _sha256(path)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(render(args.run_root, args.output), sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
