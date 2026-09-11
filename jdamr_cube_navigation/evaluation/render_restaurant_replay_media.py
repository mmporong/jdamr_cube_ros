#!/usr/bin/env python3
"""Render a 2x control-room replay from one verified restaurant MCAP."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
from bisect import bisect_right
from pathlib import Path
from typing import Any

import cv2
from navigation_mcap_reader import read_navigation_messages  # noqa: I100,I201
import numpy as np  # noqa: I100,I201
import yaml  # noqa: I100,I201


WIDTH = 1920
HEIGHT = 720
FPS = 15
PLAYBACK_RATE = 2.0
MAP_RECT = (40, 118, 1320, 264)
BLUE = (235, 160, 45)
GREEN = (105, 220, 95)
PURPLE = (220, 80, 205)
RED = (65, 65, 240)
CYAN = (235, 210, 80)
WHITE = (238, 242, 248)
MUTED = (150, 160, 175)


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


def _latest(samples: list, timestamp_s: float):
    if not samples:
        return None
    index = bisect_right([item[0] for item in samples], timestamp_s) - 1
    return samples[index] if index >= 0 else None


def _scenario_offset(summary: dict, collision: list) -> float:
    """Align monotonic scenario evidence to the MCAP recorder clock."""
    interventions = summary['scenario']['interventions']
    if not interventions or not collision:
        return 0.0
    return collision[0][0] - interventions[0]['trigger_elapsed_s']


def _guard_segments(summary: dict, scenario_offset_s: float) -> list[dict]:
    """Map guard-clock transitions onto the aligned scenario clock."""
    events = summary['traction_guard']['events']
    scenario_events = summary['scenario']['events']
    protective = next(
        item for item in events if item['to'] == 'PROTECTIVE_STOP')
    clear = next(item for item in scenario_events
                 if item['name'] == (
                     'traction_fault_cleared_on_protective_stop'))
    guard_to_scenario_s = protective['at_s'] - clear['elapsed_s']
    return [{**item, 'media_s': (
        item['at_s'] - guard_to_scenario_s + scenario_offset_s)}
            for item in events]


def _state_at(segments: list[dict], timestamp_s: float) -> str:
    state = 'NAVIGATING'
    for event in segments:
        if event['media_s'] > timestamp_s:
            break
        state = event['to']
    return state


def _read_story(mcap: Path) -> dict:
    topics = [
        '/ground_truth_pose', '/amcl_pose', '/plan', '/scan', '/cmd_vel',
        '/guarded_cmd_vel', '/collision_monitor_state',
    ]
    story = {name: [] for name in (
        'gt', 'amcl', 'plans', 'scans', 'cmd', 'guarded', 'collision')}
    first_ns = None
    latest_gt = None
    for item in read_navigation_messages(mcap, topics=topics):
        first_ns = item.log_time_ns if first_ns is None else first_ns
        timestamp_s = (item.log_time_ns - first_ns) * 1e-9
        message = item.ros_msg
        topic = item.channel.topic
        if topic == '/ground_truth_pose':
            pose = message.pose
            latest_gt = (timestamp_s, float(pose.position.x),
                         float(pose.position.y), _yaw(pose.orientation))
            story['gt'].append(latest_gt)
        elif topic == '/amcl_pose':
            pose = message.pose.pose
            story['amcl'].append((
                timestamp_s, float(pose.position.x),
                float(pose.position.y), _yaw(pose.orientation)))
        elif topic == '/plan':
            story['plans'].append((timestamp_s, [
                (float(pose.pose.position.x), float(pose.pose.position.y))
                for pose in message.poses]))
        elif topic == '/scan' and latest_gt is not None:
            points = []
            for index, distance_m in enumerate(message.ranges):
                if not math.isfinite(distance_m) or distance_m > 6.0:
                    continue
                angle = (latest_gt[3] + math.pi + message.angle_min
                         + index * message.angle_increment)
                points.append((
                    latest_gt[1] + distance_m * math.cos(angle),
                    latest_gt[2] + distance_m * math.sin(angle)))
            story['scans'].append((timestamp_s, points))
        elif topic in ('/cmd_vel', '/guarded_cmd_vel'):
            key = 'guarded' if topic == '/guarded_cmd_vel' else 'cmd'
            story[key].append((timestamp_s, float(message.linear.x),
                               float(message.angular.z)))
        else:
            story['collision'].append((
                timestamp_s, int(message.action_type),
                str(message.polygon_name)))
    required = ('gt', 'amcl', 'plans', 'scans', 'cmd', 'guarded', 'collision')
    if any(not story[key] for key in required):
        raise ValueError('MCAP lacks a required restaurant media topic')
    story['duration_s'] = story['gt'][-1][0]
    return story


def _world_to_pixel(x_m: float, y_m: float, metadata: dict,
                    image_shape: tuple[int, ...]) -> tuple[int, int]:
    left, top, width, height = MAP_RECT
    resolution_m = float(metadata['resolution'])
    origin_x_m, origin_y_m = metadata['origin'][:2]
    map_height, map_width = image_shape[:2]
    x_fraction = (x_m - origin_x_m) / (map_width * resolution_m)
    y_fraction = (y_m - origin_y_m) / (map_height * resolution_m)
    return (round(left + x_fraction * width),
            round(top + height - y_fraction * height))


def _line(frame: np.ndarray, points: list, metadata: dict,
          image_shape: tuple[int, ...], color: tuple[int, int, int],
          thickness: int) -> None:
    pixels = [_world_to_pixel(point[0], point[1], metadata, image_shape)
              for point in points]
    if len(pixels) >= 2:
        cv2.polylines(frame, [np.asarray(pixels, dtype=np.int32)], False,
                      color, thickness, cv2.LINE_AA)


def render(run_root: Path, output_dir: Path) -> dict:
    """Create the MP4, thumbnail, and integrity manifest."""
    summary_path = run_root / 'summary.json'
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    if summary.get('status') != 'PASS':
        raise ValueError('media input must be a PASS integrated run')
    mcap = Path(summary['bag']['path'])
    if _sha256(mcap) != summary['bag']['sha256']:
        raise ValueError('MCAP hash differs from the run summary')
    map_yaml_path = (Path(__file__).resolve().parent / 'assets'
                     / 'nav_obstacle' / 'slam_corridor_eval.yaml')
    metadata = yaml.safe_load(map_yaml_path.read_text(encoding='utf-8'))
    map_path = map_yaml_path.parent / metadata['image']
    map_gray = cv2.imread(str(map_path), cv2.IMREAD_GRAYSCALE)
    if map_gray is None:
        raise ValueError('evaluation map image is unreadable')
    story = _read_story(mcap)
    scenario_offset_s = _scenario_offset(summary, story['collision'])
    guard_segments = _guard_segments(summary, scenario_offset_s)
    interventions = [{**item,
                      'start_s': item['trigger_elapsed_s'] + scenario_offset_s,
                      'end_s': item['resume_elapsed_s'] + scenario_offset_s}
                     for item in summary['scenario']['interventions']]
    goals = [{**item,
              'start_s': item['accepted_elapsed_s'] + scenario_offset_s,
              'end_s': item['result_elapsed_s'] + scenario_offset_s}
             for item in summary['scenario']['goals']]
    traction_start_s = next(
        item['elapsed_s'] + scenario_offset_s
        for item in summary['scenario']['events']
        if item['name'] == 'traction_fault_enabled')
    output_dir.mkdir(parents=True, exist_ok=False)
    raw_mp4 = output_dir / '.restaurant_replay_control_raw.mp4'
    mp4 = output_dir / 'restaurant_replay_control_2x.mp4'
    writer = cv2.VideoWriter(
        str(raw_mp4), cv2.VideoWriter_fourcc(*'mp4v'), FPS, (WIDTH, HEIGHT))
    if not writer.isOpened():
        raise RuntimeError('video writer could not open the MP4 output')
    map_view = cv2.resize(cv2.flip(map_gray, 0),
                          (MAP_RECT[2], MAP_RECT[3]),
                          interpolation=cv2.INTER_NEAREST)
    map_view = cv2.cvtColor(map_view, cv2.COLOR_GRAY2BGR)
    frame_count = math.ceil(story['duration_s'] / PLAYBACK_RATE * FPS)
    thumbnail_frame = None
    gt_times = [item[0] for item in story['gt']]
    for frame_index in range(frame_count):
        timestamp_s = frame_index / FPS * PLAYBACK_RATE
        frame = np.full((HEIGHT, WIDTH, 3), (22, 27, 36), dtype=np.uint8)
        left, top, width, height = MAP_RECT
        frame[top:top + height, left:left + width] = map_view
        plan = _latest(story['plans'], timestamp_s)
        if plan is not None:
            _line(frame, plan[1], metadata, map_gray.shape, BLUE, 3)
        gt_index = max(0, bisect_right(gt_times, timestamp_s) - 1)
        trail = [(item[1], item[2])
                 for item in story['gt'][:gt_index + 1]]
        _line(frame, trail,
              metadata, map_gray.shape, GREEN, 4)
        scan = _latest(story['scans'], timestamp_s)
        if scan is not None:
            for point in scan[1][::2]:
                cv2.circle(frame, _world_to_pixel(
                    point[0], point[1], metadata, map_gray.shape),
                    1, CYAN, -1, cv2.LINE_AA)
        active_obstacles = [item for item in interventions
                            if item['start_s'] <= timestamp_s <= item['end_s']]
        for event in active_obstacles:
            center = _world_to_pixel(
                event['obstacle_pose_m'][0], event['obstacle_pose_m'][1],
                metadata, map_gray.shape)
            cv2.rectangle(frame, (center[0] - 18, center[1] - 15),
                          (center[0] + 18, center[1] + 15), PURPLE, -1)
        robot = story['gt'][gt_index]
        center = _world_to_pixel(robot[1], robot[2], metadata, map_gray.shape)
        tip = _world_to_pixel(
            robot[1] + 0.42 * math.cos(robot[3]),
            robot[2] + 0.42 * math.sin(robot[3]), metadata, map_gray.shape)
        cv2.circle(frame, center, 9, WHITE, -1, cv2.LINE_AA)
        cv2.arrowedLine(
            frame, center, tip, WHITE, 3, cv2.LINE_AA, tipLength=0.35)

        state = _state_at(guard_segments, timestamp_s)
        goal_index = next((index for index, goal in enumerate(goals, start=1)
                           if goal['start_s'] <= timestamp_s <= goal['end_s']),
                          min(20, sum(timestamp_s > goal['end_s']
                                      for goal in goals) + 1))
        collision = _latest(story['collision'], timestamp_s)
        collision_stop = collision is not None and collision[1] != 0
        commanded = _latest(story['cmd'], timestamp_s)
        guarded = _latest(story['guarded'], timestamp_s)
        traction_active = (
            traction_start_s <= timestamp_s and state != 'RECOVERED')
        incident = ('ROUTE OBSTACLE / PROTECTIVE STOP' if collision_stop
                    else 'TRACTION MISMATCH / RECOVERY' if traction_active
                    else 'ROUTE TRACKING')
        incident_color = (
            PURPLE if collision_stop else RED if traction_active else GREEN)
        cv2.putText(frame, 'RESTAURANT AMR / DATA-BASED SCENARIO REPLAY',
                    (40, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.05, WHITE, 2,
                    cv2.LINE_AA)
        cv2.putText(frame, incident, (40, 94), cv2.FONT_HERSHEY_SIMPLEX,
                    0.82, incident_color, 2, cv2.LINE_AA)
        panel_x = 1400
        labels = [
            ('REPLAY', f'{timestamp_s:6.1f} / {story["duration_s"]:5.1f} s'),
            ('GOAL', f'{goal_index:02d} / 20'),
            ('OBSTACLE STOP',
             f'{sum(timestamp_s > item["end_s"] for item in interventions)}'
             ' / 6'),
            ('TRACTION STATE', state),
            ('CMD LINEAR', f'{commanded[1] if commanded else 0.0:+.3f} m/s'),
            ('GUARDED LINEAR', f'{guarded[1] if guarded else 0.0:+.3f} m/s'),
        ]
        for row, (label, value) in enumerate(labels):
            y = 150 + row * 75
            cv2.putText(frame, label, (panel_x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, MUTED, 1, cv2.LINE_AA)
            value_color = (
                incident_color if label == 'TRACTION STATE' else WHITE)
            cv2.putText(frame, value, (panel_x, y + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.78, value_color, 2,
                        cv2.LINE_AA)
        timeline_y = 600
        cv2.line(frame, (40, timeline_y), (1360, timeline_y), MUTED, 2)
        progress_x = round(40 + 1320 * timestamp_s / story['duration_s'])
        cv2.line(frame, (progress_x, timeline_y - 14),
                 (progress_x, timeline_y + 14), WHITE, 3)
        for event in interventions:
            x = round(40 + 1320 * event['start_s'] / story['duration_s'])
            cv2.circle(frame, (x, timeline_y), 7, PURPLE, -1)
        traction_x = round(40 + 1320 * traction_start_s / story['duration_s'])
        cv2.circle(frame, (traction_x, timeline_y), 8, RED, -1)
        cv2.putText(frame, 'PLAN', (40, 670), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, BLUE, 2)
        cv2.putText(frame, 'AMCL / ACTUAL TRACK', (150, 670),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, GREEN, 2)
        cv2.putText(frame, 'LIDAR', (400, 670), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, CYAN, 2)
        cv2.putText(frame, 'OBSTACLE', (510, 670),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, PURPLE, 2)
        cv2.putText(frame, 'TRACTION FAULT', (670, 670),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, RED, 2)
        writer.write(frame)
        if thumbnail_frame is None and state == 'PROTECTIVE_STOP':
            thumbnail_frame = frame.copy()
    writer.release()
    subprocess.run([
        'ffmpeg', '-y', '-loglevel', 'error', '-i', str(raw_mp4),
        '-an', '-c:v', 'libx264', '-crf', '20', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', str(mp4),
    ], check=True)
    raw_mp4.unlink()
    thumbnail = output_dir / 'restaurant_replay_protective_stop.png'
    cv2.imwrite(str(thumbnail), thumbnail_frame if thumbnail_frame is not None
                else frame)
    manifest = {
        'schema_version': 1,
        'claim': 'functional_scenario_replay_not_digital_twin',
        'playback_rate': PLAYBACK_RATE,
        'fps': FPS,
        'frame_count': frame_count,
        'source_summary': {'path': str(summary_path.resolve()),
                           'sha256': _sha256(summary_path)},
        'source_mcap': {'path': str(mcap.resolve()), 'sha256': _sha256(mcap)},
        'generated': [
            {'path': str(mp4.resolve()), 'bytes': mp4.stat().st_size,
             'sha256': _sha256(mp4)},
            {'path': str(thumbnail.resolve()),
             'bytes': thumbnail.stat().st_size,
             'sha256': _sha256(thumbnail)},
        ],
    }
    manifest_path = output_dir / 'manifest.json'
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8')
    return manifest


def main() -> int:
    """Render media from command-line paths."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(render(args.run_root, args.output), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
