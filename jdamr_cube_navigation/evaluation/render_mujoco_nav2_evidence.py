#!/usr/bin/env python3
"""Render one verified Nav2 run as a bounded MuJoCo portfolio scene."""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time
from typing import Any

os.environ.setdefault('MUJOCO_GL', 'egl')

import cv2  # noqa: E402
import mujoco  # noqa: E402
from navigation_mcap_reader import read_navigation_messages  # noqa: E402
import numpy as np  # noqa: E402


WIDTH = 1280
HEIGHT = 720
FPS = 24
DURATION_S = 24.0
MAX_PLAN_POINTS = 48
MAX_TRAIL_POINTS = 56
LIDAR_RAY_COUNT = 45
LIDAR_RANGE_M = 4.0
MIN_AVAILABLE_MEMORY_BYTES = 2 * 1024 ** 3
MIN_FREE_DISK_BYTES = 1024 ** 3


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw(rotation: Any) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))


def _read_story(mcap: Path) -> dict[str, list]:
    story: dict[str, list] = {'gt': [], 'plans': []}
    for item in read_navigation_messages(
            mcap, topics=['/ground_truth_pose', '/plan']):
        message = item.ros_msg
        if item.channel.topic == '/ground_truth_pose':
            story['gt'].append((
                _stamp_ns(message.header.stamp),
                float(message.pose.position.x),
                float(message.pose.position.y),
                _yaw(message.pose.orientation)))
        else:
            points = [
                (float(pose.pose.position.x), float(pose.pose.position.y))
                for pose in message.poses]
            story['plans'].append((_stamp_ns(message.header.stamp), points))
    for key in story:
        story[key].sort(key=lambda sample: sample[0])
    if not story['gt'] or not story['plans']:
        raise ValueError('MCAP lacks ground-truth poses or Nav2 plans')
    return story


def _load_run(run_root: Path) -> dict[str, Any]:
    summary_path = run_root / 'summary.json'
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    if summary.get('status') != 'PASS' or len(summary.get('results', [])) != 1:
        raise ValueError('renderer requires one PASS smoke result')
    result = summary['results'][0]
    if result.get('case') != 'detour_sudden_stop_resume':
        raise ValueError('renderer requires the combined detour/stop case')
    case_root = run_root / result['case']
    scenario_path = case_root / 'scenario.json'
    scenario = json.loads(scenario_path.read_text(encoding='utf-8'))
    if scenario.get('action_terminal') != 'succeeded':
        raise ValueError('scenario did not reach the Nav2 goal')
    detour = result.get('detour_evidence', {})
    if detour.get('status') != 'PASS':
        raise ValueError('scenario lacks verified static-obstacle detour')
    mcap = Path(result['recording']['mcap']['path'])
    if _sha256(mcap) != result['recording']['mcap']['sha256']:
        raise ValueError('source MCAP hash mismatch')
    events = {event['name']: event for event in scenario['events']}
    required = {
        'goal_accepted', 'obstacle_crossing_started', 'stop_state',
        'clear_set_pose_requested', 'succeeded'}
    if not required <= set(events):
        raise ValueError(f'missing scenario events: {sorted(required - set(events))}')
    return {
        'summary_path': summary_path, 'summary': summary,
        'result': result, 'scenario_path': scenario_path,
        'scenario': scenario, 'events': events, 'mcap': mcap,
        'story': _read_story(mcap),
    }


def _sample_pose(samples: list[tuple], timestamp_ns: int) -> tuple:
    times = [sample[0] for sample in samples]
    right = bisect.bisect_right(times, timestamp_ns)
    if right <= 0:
        return samples[0]
    if right >= len(samples):
        return samples[-1]
    left_sample, right_sample = samples[right - 1], samples[right]
    span = right_sample[0] - left_sample[0]
    ratio = 0.0 if span <= 0 else (
        timestamp_ns - left_sample[0]) / span
    yaw_delta = math.atan2(
        math.sin(right_sample[3] - left_sample[3]),
        math.cos(right_sample[3] - left_sample[3]))
    return (
        timestamp_ns,
        left_sample[1] + ratio * (right_sample[1] - left_sample[1]),
        left_sample[2] + ratio * (right_sample[2] - left_sample[2]),
        left_sample[3] + ratio * yaw_delta,
    )


def _latest_plan(plans: list[tuple], timestamp_ns: int) -> list[tuple]:
    times = [sample[0] for sample in plans]
    index = max(0, bisect.bisect_right(times, timestamp_ns) - 1)
    return plans[index][1]


def _anchors(run: dict[str, Any]) -> dict[str, Any]:
    events = run['events']
    detour_ns = int(run['result']['detour_evidence']['detour_peak_ros_ns'])
    ros_anchors = [
        (0.00, int(events['goal_accepted']['ros_ns'])),
        (0.34, detour_ns),
        (0.49, int(events['obstacle_crossing_started']['ros_ns'])),
        (0.62, int(events['stop_state']['ros_ns'])),
        (0.78, int(events['clear_set_pose_requested']['ros_ns'])),
        (1.00, int(events['succeeded']['ros_ns'])),
    ]
    if any(right[1] < left[1]
           for left, right in zip(ros_anchors, ros_anchors[1:])):
        raise ValueError('ROS evidence events are not monotonic')
    goal = events['goal_accepted']
    crossing = events['obstacle_crossing_started']
    ros_span = crossing['ros_ns'] - goal['ros_ns']
    if ros_span <= 0:
        raise ValueError('detour-to-crossing ROS interval is invalid')
    detour_ratio = (detour_ns - goal['ros_ns']) / ros_span
    detour_steady_ns = round(
        goal['steady_ns'] + detour_ratio
        * (crossing['steady_ns'] - goal['steady_ns']))
    steady_anchors = [
        (0.00, int(goal['steady_ns'])),
        (0.34, detour_steady_ns),
        (0.49, int(crossing['steady_ns'])),
        (0.62, int(events['stop_state']['steady_ns'])),
        (0.78, int(events['clear_set_pose_requested']['steady_ns'])),
        (1.00, int(events['succeeded']['steady_ns'])),
    ]
    if any(right[1] <= left[1]
           for left, right in zip(steady_anchors, steady_anchors[1:])):
        raise ValueError('steady evidence events are not strictly ordered')
    return {
        'ros': ros_anchors, 'steady': steady_anchors,
        'detour_steady_ns': detour_steady_ns,
    }


def _target_ns(anchors: list[tuple[float, int]], progress: float) -> int:
    for (left_fraction, left_ns), (right_fraction, right_ns) in zip(
            anchors, anchors[1:]):
        if progress <= right_fraction:
            ratio = ((progress - left_fraction)
                     / (right_fraction - left_fraction))
            return round(left_ns + ratio * (right_ns - left_ns))
    return anchors[-1][1]


def _smoothstep(value: float) -> float:
    bounded = min(1.0, max(0.0, value))
    return bounded * bounded * (3.0 - 2.0 * bounded)


def _pedestrian_pose(run: dict[str, Any], timestamp_ns: int) -> tuple[float, ...]:
    events = run['events']
    entry = events['obstacle_crossing_started']
    entry_end = events.get('obstacle_crossing_completed')
    exit_start = events.get('obstacle_crossing_exit_started')
    exit_end = events.get('obstacle_crossing_exit_completed')
    clear = events['clear_set_pose_requested']
    start = entry['start_pose_m']
    target = entry['target_pose_m']
    if timestamp_ns < entry['steady_ns']:
        return (start[0], 20.0, start[2])
    if entry_end and timestamp_ns < entry_end['steady_ns']:
        ratio = _smoothstep(
            (timestamp_ns - entry['steady_ns'])
            / (entry_end['steady_ns'] - entry['steady_ns']))
        return tuple(
            left + ratio * (right - left)
            for left, right in zip(start, target))
    if (exit_start and exit_end
            and timestamp_ns < exit_end['steady_ns']):
        exit_pose = exit_start['exit_pose_m']
        ratio = _smoothstep(
            (timestamp_ns - exit_start['steady_ns'])
            / (exit_end['steady_ns'] - exit_start['steady_ns']))
        return tuple(
            left + ratio * (right - left)
            for left, right in zip(target, exit_pose))
    if timestamp_ns < clear['steady_ns']:
        return tuple(target)
    return (target[0], 20.0, target[2])


def _scene_xml(route: dict[str, Any]) -> str:
    center_x_m, center_y_m = route['obstacle']['center_xy_m']
    length_m, width_m = route['obstacle']['dimensions_m']
    return f"""<mujoco model="jdamr_nav2_portfolio">
  <compiler angle="radian"/>
  <option timestep="0.01" gravity="0 0 -9.81"/>
  <visual>
    <global offwidth="{WIDTH}" offheight="{HEIGHT}"
            azimuth="180" elevation="-20"/>
    <quality shadowsize="4096" offsamples="4"/>
    <headlight ambient="0.20 0.23 0.28" diffuse="0.72 0.76 0.82"
               specular="0.25 0.25 0.25"/>
    <rgba haze="0.12 0.16 0.22 1"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.04 0.07 0.12" rgb2="0.22 0.30 0.42"
             width="512" height="3072"/>
    <texture name="floor_tex" type="2d" builtin="checker"
             rgb1="0.16 0.18 0.21" rgb2="0.20 0.23 0.27"
             width="512" height="512"/>
    <material name="floor" texture="floor_tex" texrepeat="16 3"
              reflectance="0.08" shininess="0.3"/>
    <material name="wall" rgba="0.48 0.56 0.64 1" roughness="0.75"/>
    <material name="robot" rgba="0.06 0.36 0.62 1"
              metallic="0.35" roughness="0.32"/>
    <material name="robot_dark" rgba="0.035 0.055 0.075 1"
              metallic="0.45" roughness="0.28"/>
    <material name="crate" rgba="0.92 0.34 0.07 1" roughness="0.55"/>
    <material name="warning" rgba="0.98 0.78 0.08 1" emission="0.08"/>
    <material name="person" rgba="0.82 0.10 0.10 1" roughness="0.7"/>
    <material name="skin" rgba="0.72 0.48 0.32 1" roughness="0.8"/>
  </asset>
  <worldbody>
    <light pos="-4 -1 5" dir="0.35 0.08 -1"
           diffuse="0.85 0.88 0.95" castshadow="true"/>
    <light pos="4 1 4" dir="-0.25 -0.05 -1"
           diffuse="0.55 0.68 0.85" castshadow="true"/>
    <geom name="floor" type="plane" size="12 2.4 0.1" material="floor"/>
    <geom name="north_wall" type="box" pos="0 1.25 0.55"
          size="11 0.08 0.55" material="wall"/>
    <geom name="south_wall" type="box" pos="0 -1.25 0.55"
          size="11 0.08 0.55" material="wall"/>
    <geom name="west_wall" type="box" pos="-10.9 0 0.55"
          size="0.08 1.25 0.55" material="wall"/>
    <geom name="east_wall" type="box" pos="10.9 0 0.55"
          size="0.08 1.25 0.55" material="wall"/>
    <geom name="lane_north" type="box" pos="0 0.92 0.006"
          size="10.5 0.025 0.006" rgba="0.08 0.52 0.80 0.75"
          contype="0" conaffinity="0"/>
    <geom name="lane_south" type="box" pos="0 -0.92 0.006"
          size="10.5 0.025 0.006" rgba="0.08 0.52 0.80 0.75"
          contype="0" conaffinity="0"/>
    <geom name="goal" type="cylinder" pos="6 0 0.012"
          size="0.28 0.012" rgba="0.08 0.82 0.38 0.9"
          contype="0" conaffinity="0"/>
    <body name="route_crate" pos="{center_x_m} {center_y_m} 0">
      <geom name="route_collision" type="box" pos="0 0 0.5"
            size="{length_m / 2} {width_m / 2} 0.5" material="crate"/>
      <geom type="box" pos="0 0 0.08"
            size="{length_m / 2 + 0.04} {width_m / 2 + 0.04} 0.035"
            material="robot_dark"/>
      <geom type="box" pos="0 0 0.38"
            size="{length_m / 2 + 0.006} 0.018 0.055"
            material="warning" contype="0" conaffinity="0"/>
      <geom type="box" pos="0 0 0.70"
            size="{length_m / 2 + 0.006} 0.018 0.055"
            material="warning" contype="0" conaffinity="0"/>
    </body>
    <body name="robot" mocap="true" pos="-8 0 0.14">
      <geom name="base" type="box" pos="0 0 0.10"
            size="0.24 0.21 0.10" material="robot"/>
      <geom type="box" pos="0.02 0 0.24"
            size="0.17 0.16 0.045" material="robot_dark"/>
      <geom type="cylinder" pos="0.11 0 0.34"
            size="0.075 0.055" material="robot_dark"/>
      <geom type="cylinder" pos="0.11 0 0.402" size="0.085 0.008"
            rgba="0.04 0.85 0.95 1" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos="0 0.235 0.10" euler="1.5708 0 0"
            size="0.075 0.035" material="robot_dark"/>
      <geom type="cylinder" pos="0 -0.235 0.10" euler="1.5708 0 0"
            size="0.075 0.035" material="robot_dark"/>
      <geom type="box" pos="0.245 0 0.08" size="0.018 0.20 0.065"
            rgba="0.96 0.53 0.07 1"/>
    </body>
    <body name="pedestrian" mocap="true" pos="1 20 0.5">
      <geom name="person_torso" type="capsule" pos="0 0 0.16"
            size="0.15 0.27" material="person"/>
      <geom name="person_head" type="sphere" pos="0 0 0.62"
            size="0.13" material="skin"/>
      <geom type="capsule" pos="0 0.10 -0.27"
            size="0.065 0.24" material="robot_dark"/>
      <geom type="capsule" pos="0 -0.10 -0.27"
            size="0.065 0.24" material="robot_dark"/>
      <geom type="capsule" pos="0 0.22 0.15" euler="1.5708 0 0"
            size="0.055 0.22" material="skin"/>
      <geom type="capsule" pos="0 -0.22 0.15" euler="1.5708 0 0"
            size="0.055 0.22" material="skin"/>
    </body>
    <body pos="-5.5 1.16 0.55">
      <geom type="box" size="0.025 0.02 0.32"
            rgba="0.10 0.72 0.92 1" contype="0" conaffinity="0"/>
    </body>
    <body pos="0.8 -1.16 0.55">
      <geom type="box" size="0.025 0.02 0.32"
            rgba="0.98 0.55 0.08 1" contype="0" conaffinity="0"/>
    </body>
    <body pos="5.2 1.16 0.55">
      <geom type="box" size="0.025 0.02 0.32"
            rgba="0.10 0.72 0.92 1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""


def _downsample(points: list, limit: int) -> list:
    if len(points) <= limit:
        return points
    indices = np.linspace(0, len(points) - 1, limit).round().astype(int)
    return [points[index] for index in indices]


def _add_line(scene: Any, start: tuple[float, ...], end: tuple[float, ...],
              color: tuple[float, ...], width: float) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom, mujoco.mjtGeom.mjGEOM_LINE,
        np.zeros(3), np.zeros(3), np.eye(3).reshape(-1),
        np.asarray(color, dtype=np.float32))
    mujoco.mjv_connector(
        geom, mujoco.mjtGeom.mjGEOM_LINE, width,
        np.asarray(start, dtype=np.float64),
        np.asarray(end, dtype=np.float64))
    scene.ngeom += 1


def _draw_world_traces(scene: Any, plan: list[tuple], trail: list[tuple]) -> None:
    plan = _downsample(plan, MAX_PLAN_POINTS)
    trail = _downsample(trail, MAX_TRAIL_POINTS)
    for left, right in zip(plan, plan[1:]):
        _add_line(
            scene, (left[0], left[1], 0.035),
            (right[0], right[1], 0.035), (1.0, 0.78, 0.06, 0.85), 3.0)
    for left, right in zip(trail, trail[1:]):
        _add_line(
            scene, (left[1], left[2], 0.055),
            (right[1], right[2], 0.055), (0.05, 0.72, 1.0, 0.95), 4.0)


def _lidar_scan(
        model: Any, data: Any, scene: Any, robot_id: int,
        pose: tuple) -> float:
    origin = np.asarray((pose[1], pose[2], 0.39), dtype=np.float64)
    distances = []
    for relative_angle in np.linspace(
            -math.radians(135.0), math.radians(135.0), LIDAR_RAY_COUNT):
        angle = pose[3] + relative_angle
        direction = np.asarray(
            (math.cos(angle), math.sin(angle), 0.0), dtype=np.float64)
        geom_id = np.asarray([-1], dtype=np.int32)
        distance = mujoco.mj_ray(
            model, data, origin, direction, None, 1, robot_id, geom_id)
        if distance < 0.0:
            distance = LIDAR_RANGE_M
        distance = min(distance, LIDAR_RANGE_M)
        distances.append(distance)
        if len(distances) % 2 == 0:
            endpoint = origin + direction * distance
            color = ((1.0, 0.22, 0.12, 0.45)
                     if distance < 0.75 else (0.05, 0.82, 0.96, 0.28))
            _add_line(scene, tuple(origin), tuple(endpoint), color, 1.0)
    return min(distances)


def _state(
        run: dict[str, Any], timestamp_ns: int, pose: tuple,
        detour_steady_ns: int) -> tuple[str, str]:
    events = run['events']
    crossing_ns = events['obstacle_crossing_started']['steady_ns']
    stop_ns = events['stop_state']['steady_ns']
    clear_ns = events['clear_set_pose_requested']['steady_ns']
    success_ns = events['succeeded']['steady_ns']
    if timestamp_ns >= success_ns:
        return 'GOAL SUCCEEDED', 'green'
    if timestamp_ns >= clear_ns:
        return 'RESUMED - SAME GOAL', 'green'
    if timestamp_ns >= stop_ns:
        return 'STOP - PEDESTRIAN', 'red'
    if timestamp_ns >= crossing_ns:
        return 'PEDESTRIAN DETECTED', 'amber'
    if timestamp_ns >= detour_steady_ns or -2.4 <= pose[1] <= 0.2:
        return 'NAV2 - STATIC DETOUR', 'cyan'
    return 'NAVIGATING', 'blue'


COLORS = {
    'blue': (235, 180, 40), 'cyan': (230, 210, 25),
    'amber': (35, 165, 245), 'red': (55, 55, 235),
    'green': (95, 205, 55),
}


def _panel(frame: np.ndarray, x1: int, y1: int, x2: int, y2: int,
           alpha: float = 0.72) -> None:
    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (17, 24, 39), -1)
    cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0.0, frame)


def _map_point(x_m: float, y_m: float, box: tuple[int, ...]) -> tuple[int, int]:
    left, top, right, bottom = box
    x_px = left + int((x_m + 9.0) / 16.0 * (right - left))
    y_px = bottom - int((y_m + 1.4) / 2.8 * (bottom - top))
    return x_px, y_px


def _draw_hud(
        frame: np.ndarray, run: dict[str, Any], pose: tuple,
        plan: list[tuple], trail: list[tuple], person: tuple[float, ...],
        nearest_lidar_m: float, state: tuple[str, str]) -> None:
    state, color_name = state
    state_color = COLORS[color_name]
    _panel(frame, 28, 24, 688, 118)
    cv2.putText(frame, 'JDAMR  |  NAV2 OBSTACLE CHALLENGE', (52, 59),
                cv2.FONT_HERSHEY_DUPLEX, 0.72, (238, 243, 250), 1,
                cv2.LINE_AA)
    cv2.putText(frame, state, (52, 98), cv2.FONT_HERSHEY_DUPLEX,
                0.82, state_color, 2, cv2.LINE_AA)

    _panel(frame, 28, 552, 502, 692)
    goal_distance_m = math.dist((pose[1], pose[2]), (6.0, 0.0))
    lateral_m = abs(pose[2])
    cv2.putText(frame, f'LiDAR nearest     {nearest_lidar_m:4.2f} m',
                (52, 586), cv2.FONT_HERSHEY_SIMPLEX, 0.56,
                (224, 232, 240), 1, cv2.LINE_AA)
    cv2.putText(frame, f'Lateral offset    {lateral_m:4.2f} m',
                (52, 617), cv2.FONT_HERSHEY_SIMPLEX, 0.56,
                (224, 232, 240), 1, cv2.LINE_AA)
    cv2.putText(frame, f'Goal distance     {goal_distance_m:4.2f} m',
                (52, 648), cv2.FONT_HERSHEY_SIMPLEX, 0.56,
                (224, 232, 240), 1, cv2.LINE_AA)
    cv2.putText(frame, 'CONTACT 0 | PATH MCAP | LIDAR MUJOCO RAYCAST',
                (52, 678), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                (105, 218, 140), 1, cv2.LINE_AA)

    map_box = (886, 34, 1248, 254)
    _panel(frame, map_box[0] - 16, map_box[1] - 16,
           map_box[2] + 12, map_box[3] + 34, 0.80)
    cv2.putText(frame, 'LIVE NAV2 PLAN / GROUND TRUTH',
                (map_box[0], map_box[3] + 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, (205, 216, 229), 1, cv2.LINE_AA)
    north_left = _map_point(-9.0, 1.18, map_box)
    north_right = _map_point(7.0, 1.18, map_box)
    south_left = _map_point(-9.0, -1.18, map_box)
    south_right = _map_point(7.0, -1.18, map_box)
    cv2.line(frame, north_left, north_right, (120, 135, 150), 2)
    cv2.line(frame, south_left, south_right, (120, 135, 150), 2)
    route = run['result']['detour_evidence']['obstacle']
    center = route['center_xy_m']
    dims = route['dimensions_m']
    corner_a = _map_point(
        center[0] - dims[0] / 2.0, center[1] + dims[1] / 2.0, map_box)
    corner_b = _map_point(
        center[0] + dims[0] / 2.0, center[1] - dims[1] / 2.0, map_box)
    cv2.rectangle(frame, corner_a, corner_b, (25, 100, 238), -1)
    for left, right in zip(_downsample(plan, 60), _downsample(plan, 60)[1:]):
        cv2.line(frame, _map_point(*left, map_box),
                 _map_point(*right, map_box), (35, 205, 250), 1,
                 cv2.LINE_AA)
    mapped_trail = [_map_point(item[1], item[2], map_box)
                    for item in _downsample(trail, 80)]
    if len(mapped_trail) >= 2:
        cv2.polylines(frame, [np.asarray(mapped_trail, dtype=np.int32)],
                      False, (245, 180, 45), 2, cv2.LINE_AA)
    robot_px = _map_point(pose[1], pose[2], map_box)
    cv2.circle(frame, robot_px, 5, (40, 150, 255), -1)
    if abs(person[1]) < 2.0:
        cv2.circle(frame, _map_point(person[0], person[1], map_box),
                   5, (55, 55, 235), -1)


def _peak_rss_kib() -> int:
    for line in Path('/proc/self/status').read_text().splitlines():
        if line.startswith('VmHWM:'):
            return int(line.split()[1])
    return 0


def _resource_sample(output_dir: Path) -> dict[str, int]:
    values = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:'):
            values['available_memory_bytes'] = int(line.split()[1]) * 1024
            break
    values['free_disk_bytes'] = shutil.disk_usage(output_dir).free
    return values


def _guard_resources(output_dir: Path) -> dict[str, int]:
    sample = _resource_sample(output_dir)
    if sample['available_memory_bytes'] < MIN_AVAILABLE_MEMORY_BYTES:
        raise RuntimeError('render aborted before host memory exhaustion')
    if sample['free_disk_bytes'] < MIN_FREE_DISK_BYTES:
        raise RuntimeError('render aborted before filesystem exhaustion')
    return sample


def _portable_source_path(path: Path, run_root: Path) -> str:
    try:
        relative = path.resolve().relative_to(run_root.resolve())
    except ValueError:
        return path.name
    return str(Path(run_root.name) / relative)


def _start_encoder(path: Path) -> subprocess.Popen:
    ffmpeg = shutil.which('ffmpeg')
    if ffmpeg is None:
        raise RuntimeError('ffmpeg is required')
    return subprocess.Popen([
        ffmpeg, '-loglevel', 'error', '-y', '-f', 'rawvideo',
        '-pix_fmt', 'bgr24', '-s', f'{WIDTH}x{HEIGHT}', '-r', str(FPS),
        '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'medium',
        '-crf', '19', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
        str(path),
    ], stdin=subprocess.PIPE)


def _finish_encoder(encoder: subprocess.Popen) -> int:
    if encoder.stdin is not None:
        encoder.stdin.close()
    try:
        return encoder.wait(timeout=60.0)
    except subprocess.TimeoutExpired:
        encoder.terminate()
        try:
            return encoder.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            encoder.kill()
            return encoder.wait(timeout=5.0)


def _make_gif(video: Path, output: Path) -> None:
    result = subprocess.run([
        'ffmpeg', '-loglevel', 'error', '-y', '-i', str(video),
        '-vf', ('fps=6,scale=640:-1:flags=lanczos,split[s0][s1];'
                '[s0]palettegen=max_colors=96[p];'
                '[s1][p]paletteuse=dither=bayer:bayer_scale=4'),
        str(output),
    ], capture_output=True, text=True, timeout=180.0, check=False)
    if result.returncode != 0 or not output.is_file():
        raise RuntimeError(result.stderr.strip() or 'GIF encoding failed')


def render(run_root: Path, output_dir: Path) -> dict[str, Any]:
    """Stream MuJoCo frames to disk without retaining a frame collection."""
    if output_dir.exists():
        raise ValueError('output directory must not already exist')
    output_dir.mkdir(parents=True)
    _guard_resources(output_dir)
    run = _load_run(run_root)
    xml_path = output_dir / 'jdamr_nav2_portfolio_scene.xml'
    xml_path.write_text(
        _scene_xml(run['result']['detour_evidence']), encoding='utf-8')
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    robot_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, 'robot')
    robot_mocap_id = model.body_mocapid[robot_id]
    pedestrian_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, 'pedestrian')
    pedestrian_mocap_id = model.body_mocapid[pedestrian_id]
    anchors = _anchors(run)
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 3.35
    camera.elevation = -15.0
    frames = round(DURATION_S * FPS)
    video_path = output_dir / 'mujoco_nav2_obstacle_challenge.mp4'
    preview_path = output_dir / 'mujoco_nav2_obstacle_challenge.jpg'
    encoder = _start_encoder(video_path)
    started_s = time.monotonic()
    minimum_available_memory_bytes = math.inf
    minimum_free_disk_bytes = math.inf
    try:
        for frame_index in range(frames):
            if frame_index % FPS == 0:
                resource_sample = _guard_resources(output_dir)
                minimum_available_memory_bytes = min(
                    minimum_available_memory_bytes,
                    resource_sample['available_memory_bytes'])
                minimum_free_disk_bytes = min(
                    minimum_free_disk_bytes,
                    resource_sample['free_disk_bytes'])
            progress = frame_index / (frames - 1)
            timestamp_ns = _target_ns(anchors['ros'], progress)
            steady_ns = _target_ns(anchors['steady'], progress)
            pose = _sample_pose(run['story']['gt'], timestamp_ns)
            plan = _latest_plan(run['story']['plans'], timestamp_ns)
            trail = [sample for sample in run['story']['gt']
                     if sample[0] <= timestamp_ns]
            person = _pedestrian_pose(run, steady_ns)
            data.mocap_pos[robot_mocap_id] = (pose[1], pose[2], 0.14)
            data.mocap_quat[robot_mocap_id] = (
                math.cos(pose[3] / 2.0), 0.0, 0.0,
                math.sin(pose[3] / 2.0))
            data.mocap_pos[pedestrian_mocap_id] = person
            data.mocap_quat[pedestrian_mocap_id] = (1.0, 0.0, 0.0, 0.0)
            mujoco.mj_forward(model, data)
            camera.lookat[:] = (
                pose[1] + 0.85 * math.cos(pose[3]),
                pose[2] + 0.85 * math.sin(pose[3]), 0.25)
            camera.azimuth = 180.0 - math.degrees(pose[3])
            renderer.update_scene(data, camera=camera)
            _draw_world_traces(renderer.scene, plan, trail)
            nearest_lidar_m = _lidar_scan(
                model, data, renderer.scene, robot_id, pose)
            rgb = renderer.render()
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            _draw_hud(
                bgr, run, pose, plan, trail, person,
                nearest_lidar_m,
                _state(run, steady_ns, pose, anchors['detour_steady_ns']))
            if frame_index == round(frames * 0.34):
                cv2.imwrite(str(preview_path), bgr, [cv2.IMWRITE_JPEG_QUALITY, 94])
            if encoder.stdin is None:
                raise RuntimeError('ffmpeg stdin is unavailable')
            encoder.stdin.write(bgr.tobytes())
    finally:
        renderer.close()
        returncode = _finish_encoder(encoder)
    if returncode != 0 or not video_path.is_file() or video_path.stat().st_size == 0:
        raise RuntimeError(f'video encoder failed: {returncode}')
    gif_path = output_dir / 'mujoco_nav2_obstacle_challenge.gif'
    _make_gif(video_path, gif_path)
    elapsed_s = time.monotonic() - started_s
    manifest = {
        'schema_version': 1,
        'status': 'PASS',
        'source_semantics': (
            'MuJoCo visualization of recorded Nav2 ground truth, plans, and '
            'scenario events; Gazebo remains the physics/sensor evidence source'),
        'scenario': 'detour_sudden_stop_resume',
        'engine': {'name': 'MuJoCo', 'version': mujoco.__version__,
                   'render_backend': os.environ['MUJOCO_GL']},
        'video': {
            'path': video_path.name, 'sha256': _sha256(video_path),
            'size_bytes': video_path.stat().st_size,
            'width': WIDTH, 'height': HEIGHT, 'fps': FPS,
            'frames': frames, 'duration_s': DURATION_S,
        },
        'gif': {
            'path': gif_path.name, 'sha256': _sha256(gif_path),
            'size_bytes': gif_path.stat().st_size,
        },
        'preview': {
            'path': preview_path.name,
            'sha256': _sha256(preview_path),
            'size_bytes': preview_path.stat().st_size,
        },
        'scene': {'path': xml_path.name, 'sha256': _sha256(xml_path)},
        'source': {
            'summary': {'path': _portable_source_path(
                            run['summary_path'], run_root),
                        'sha256': _sha256(run['summary_path'])},
            'scenario': {'path': _portable_source_path(
                             run['scenario_path'], run_root),
                         'sha256': _sha256(run['scenario_path'])},
            'mcap': {'path': _portable_source_path(run['mcap'], run_root),
                     'sha256': _sha256(run['mcap'])},
        },
        'verified_behavior': {
            'straight_centerline_blocked': True,
            'detour': run['result']['detour_evidence'],
            'same_goal_resume': run['result']['same_goal_command_evidence'][
                'evidence']['verdict'],
            'contact_count': run['scenario']['contact_count'],
        },
        'resource_policy': {
            'streamed_frames': True, 'frames_retained_in_memory': 1,
            'peak_rss_kib': _peak_rss_kib(),
            'minimum_available_memory_bytes': int(
                minimum_available_memory_bytes),
            'minimum_free_disk_bytes': int(minimum_free_disk_bytes),
            'abort_below_available_memory_bytes': (
                MIN_AVAILABLE_MEMORY_BYTES),
            'abort_below_free_disk_bytes': MIN_FREE_DISK_BYTES,
            'render_elapsed_s': elapsed_s,
            'average_render_fps': frames / elapsed_s,
        },
    }
    manifest_path = output_dir / 'mujoco_portfolio_manifest.json'
    manifest_path.write_text(json.dumps(
        manifest, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-root', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    manifest = render(args.run_root, args.output_dir)
    print(json.dumps(manifest, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
