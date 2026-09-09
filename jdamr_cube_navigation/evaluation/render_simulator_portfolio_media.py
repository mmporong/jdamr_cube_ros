#!/usr/bin/env python3
"""Render synchronized Gazebo, LiDAR, localization, and Nav2 evidence."""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import cv2
import numpy as np  # noqa: I201
from PIL import Image, ImageDraw, ImageFont  # noqa: I100,I201
import yaml  # noqa: I201

from navigation_mcap_reader import read_navigation_messages  # noqa: I100


REGULAR_FONT = Path(
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
BOLD_FONT = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc')
CANVAS_SIZE = (1920, 1080)
CAMERA_RECT = (32, 104, 1296, 815)
MAP_RECT = (1320, 104, 1888, 447)
SCAN_RECT = (1320, 471, 1888, 815)
FOOTER_RECT = (32, 839, 1888, 1048)
LASER_TO_BASE_YAW_RAD = math.pi
MAP_RESOLUTION_M = 0.04
ROUTE_CRATE_CENTER_M = (-1.0, 0.0)
ROUTE_CRATE_DIMENSIONS_M = (0.5, 0.4)
WHITE = (255, 255, 255, 255)
SURFACE = (243, 246, 250, 255)
INK = (24, 43, 66, 255)
BLUE = (35, 95, 209, 255)
TEAL = (0, 126, 135, 255)
MAGENTA = (165, 51, 129, 255)


def _validate_capture_geometry(result: dict) -> None:
    """Fail closed when the recorded scene differs from the planar overlay."""
    world = result['simulator_video'].get('capture_world', {})
    doorway = world.get('pedestrian_doorway', {})
    if (world.get('camera', {}).get('view') != 'fixed_world_oblique_full_route'
            or world.get('collision_change_scope') != 'corridor_edge_pedestrian_doorways'
            or doorway.get('camera_and_lidar_geometry_aligned') is not True
            or doorway.get('crossing_edge_y_m') != 1.5
            or result.get('scenario', {}).get('obstacle_crossing_edge_y_m') != 1.5):
        raise RuntimeError(
            'fixed-camera doorway capture required; historical video is not a substitute')
    world_asset = world['output']
    if _sha256(Path(world_asset['path'])) != world_asset['sha256']:
        raise RuntimeError('capture world hash mismatch')
    obstacle = result['static_route_obstacle']
    if (tuple(obstacle['expected_pose_m'][:2]) != ROUTE_CRATE_CENTER_M
            or tuple(obstacle['dimensions_m'][:2])
            != ROUTE_CRATE_DIMENSIONS_M):
        raise RuntimeError(
            'recorded route obstacle differs from overlay geometry')
    asset = result['simulator_video']['capture_urdf']['output']
    path = Path(asset['path'])
    if _sha256(path) != asset['sha256']:
        raise RuntimeError('capture URDF hash mismatch')
    joint = ET.parse(path).getroot().find(
        "./joint[@name='laser_joint']/origin")
    xyz = [float(value) for value in joint.attrib['xyz'].split()]
    rpy = [float(value) for value in joint.attrib['rpy'].split()]
    if (xyz[:2] != [0.0, 0.0] or rpy[:2] != [0.0, 0.0]
            or not math.isclose(rpy[2], LASER_TO_BASE_YAW_RAD)):
        raise RuntimeError(
            'capture laser extrinsic differs from planar overlay')


def _saved_map_points(path: Path) -> list[tuple[float, float]]:
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    with Image.open(path.parent / config['image']) as source:
        pixels = np.asarray(source.convert('L'))
    occupancy = pixels / 255.0 if config['negate'] else 1.0 - pixels / 255.0
    rows, columns = np.where(occupancy > config['occupied_thresh'])
    origin_x_m, origin_y_m, yaw_rad = config['origin']
    resolution_m = config['resolution']
    x_m = (columns + 0.5) * resolution_m
    y_m = (pixels.shape[0] - rows - 0.5) * resolution_m
    return list(zip(
        origin_x_m + x_m * math.cos(yaw_rad) - y_m * math.sin(yaw_rad),
        origin_y_m + x_m * math.sin(yaw_rad) + y_m * math.cos(yaw_rad)))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _font(path: Path, size: int) -> ImageFont.FreeTypeFont:
    if not path.is_file():
        raise RuntimeError(f'Korean font unavailable: {path}')
    return ImageFont.truetype(str(path), size=size)


def _ffmpeg_writer(path: Path, width: int, height: int, fps: float):
    command = [
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-f', 'rawvideo', '-pix_fmt', 'bgr24',
        '-s', f'{width}x{height}', '-r', f'{fps:g}', '-i', '-',
        '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '20',
        '-threads', '2',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path),
    ]
    return subprocess.Popen(command, stdin=subprocess.PIPE)


def _event_frame_interval(
        evidence: dict, metadata: dict, total_frames: int) -> tuple[int, int]:
    """Map verified steady-clock events onto retained camera frames."""
    frames = _event_frames(evidence, metadata, total_frames)
    start = frames.get('obstacle_crossing_started')
    end = frames.get('obstacle_crossing_exit_completed')
    if start is None or end is None or end - start + 1 < 3:
        raise RuntimeError('verified pedestrian event timing is incomplete')
    return start, end


def _event_frames(
        evidence: dict, metadata: dict, total_frames: int) -> dict[str, int]:
    timestamps = metadata.get('frame_timestamps')
    if timestamps:
        times = [item['steady_ns'] for item in timestamps]
        if len(times) != total_frames or times != sorted(times):
            raise RuntimeError('invalid per-frame camera timestamps')
        return {
            event['name']: min(total_frames - 1, bisect.bisect_left(
                times, event['steady_ns']))
            for event in evidence.get('events', [])
            if isinstance(event.get('steady_ns'), int)}
    first_ns = metadata.get('first_frame_steady_ns')
    last_ns = metadata.get('last_frame_steady_ns')
    if (not isinstance(first_ns, int) or not isinstance(last_ns, int)
            or total_frames < 20 or first_ns >= last_ns):
        raise RuntimeError('camera steady-clock metadata is incomplete')
    scale = (total_frames - 1) / (last_ns - first_ns)
    output = {}
    for event in evidence.get('events', []):
        steady_ns = event.get('steady_ns')
        name = event.get('name')
        if not isinstance(name, str) or not isinstance(steady_ns, int):
            continue
        output[name] = min(total_frames - 1, max(
            0, round((steady_ns - first_ns) * scale)))
    return output


def _frame_wall_ns(
        metadata: dict, frame_index: int, total_frames: int) -> int:
    """Use the camera recorder's wall clock to address MCAP log time."""
    timestamps = metadata.get('frame_timestamps')
    if timestamps:
        if len(timestamps) != total_frames:
            raise RuntimeError('camera timestamp count mismatch')
        return timestamps[frame_index]['wall_ns']
    first_ns = metadata.get('first_frame_wall_ns')
    last_ns = metadata.get('last_frame_wall_ns')
    if (not isinstance(first_ns, int) or not isinstance(last_ns, int)
            or first_ns >= last_ns or total_frames < 2):
        raise RuntimeError('camera wall-clock metadata is incomplete')
    fraction = frame_index / (total_frames - 1)
    return round(first_ns + fraction * (last_ns - first_ns))


def _yaw(rotation: Any) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))


def _read_telemetry(mcap: Path) -> dict[str, list]:
    telemetry: dict[str, list] = {
        'scans': [], 'poses': [], 'ground_truth': [], 'plans': [], 'cmd': [],
        'odom': []}
    topics = ['/scan', '/amcl_pose', '/ground_truth_pose', '/plan', '/cmd_vel',
              '/odom', '/tf']
    map_to_odom = None
    transform_poses = []
    for item in read_navigation_messages(mcap, topics=topics):
        message = item.ros_msg
        topic = item.channel.topic
        if topic == '/scan':
            telemetry['scans'].append((
                item.log_time_ns, float(message.angle_min),
                float(message.angle_increment), float(message.range_min),
                float(message.range_max),
                np.asarray(message.ranges, dtype=np.float32)))
        elif topic == '/amcl_pose':
            pose = message.pose.pose
            telemetry['poses'].append((
                item.log_time_ns, float(pose.position.x),
                float(pose.position.y), _yaw(pose.orientation)))
        elif topic == '/ground_truth_pose':
            pose = message.pose
            telemetry['ground_truth'].append((
                item.log_time_ns, float(pose.position.x),
                float(pose.position.y), _yaw(pose.orientation)))
        elif topic == '/plan':
            points = [(float(pose.pose.position.x),
                       float(pose.pose.position.y))
                      for pose in message.poses]
            telemetry['plans'].append((item.log_time_ns, points))
        elif topic == '/cmd_vel':
            telemetry['cmd'].append((
                item.log_time_ns, float(message.linear.x),
                float(message.angular.z)))
        elif topic == '/odom':
            telemetry['odom'].append((
                item.log_time_ns, float(message.twist.twist.linear.x),
                float(message.twist.twist.angular.z)))
            if map_to_odom is not None and message.header.frame_id == 'odom':
                tx_m, ty_m, yaw_rad = map_to_odom
                odom_pose = message.pose.pose
                x_m, y_m = odom_pose.position.x, odom_pose.position.y
                transform_poses.append((
                    item.log_time_ns,
                    tx_m + math.cos(yaw_rad) * x_m - math.sin(yaw_rad) * y_m,
                    ty_m + math.sin(yaw_rad) * x_m + math.cos(yaw_rad) * y_m,
                    yaw_rad + _yaw(odom_pose.orientation)))
        elif topic == '/tf':
            for transform in message.transforms:
                if (transform.header.frame_id == 'map'
                        and transform.child_frame_id == 'odom'):
                    map_to_odom = (
                        float(transform.transform.translation.x),
                        float(transform.transform.translation.y),
                        _yaw(transform.transform.rotation))
    if transform_poses:
        telemetry['poses'] = transform_poses
    for values in telemetry.values():
        values.sort(key=lambda sample: sample[0])
    missing = [name for name in ('scans', 'poses', 'plans', 'cmd', 'odom')
               if not telemetry[name]]
    if missing:
        raise RuntimeError(f'MCAP dashboard topics are missing: {missing}')
    return telemetry


def _sample_pose(samples: list[tuple], timestamp_ns: int) -> tuple:
    times = [sample[0] for sample in samples]
    right = bisect.bisect_right(times, timestamp_ns)
    if right <= 0:
        return None
    return samples[right - 1]


def _latest(samples: list[tuple], timestamp_ns: int, default=None) -> tuple:
    times = [sample[0] for sample in samples]
    index = bisect.bisect_right(times, timestamp_ns) - 1
    return samples[index] if index >= 0 else default


def _scan_world_points(scan: tuple, pose: tuple) -> np.ndarray:
    """Transform one real LaserScan into the map frame using AMCL pose."""
    _, angle_min, angle_increment, range_min, range_max, ranges = scan
    indices = np.arange(ranges.size, dtype=np.float32)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    if not np.any(valid):
        return np.empty((0, 2), dtype=np.float32)
    angles = (angle_min + indices[valid] * angle_increment
              + pose[3] + LASER_TO_BASE_YAW_RAD)
    selected = ranges[valid]
    return np.column_stack((
        pose[1] + selected * np.cos(angles),
        pose[2] + selected * np.sin(angles))).astype(np.float32)


def _scan_local_points(scan: tuple) -> np.ndarray:
    _, angle_min, angle_increment, range_min, range_max, ranges = scan
    indices = np.arange(ranges.size, dtype=np.float32)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    if not np.any(valid):
        return np.empty((0, 2), dtype=np.float32)
    angles = angle_min + indices[valid] * angle_increment
    selected = ranges[valid]
    return np.column_stack(
        (selected * np.cos(angles), selected * np.sin(angles))).astype(
            np.float32)


def _crate_detection_ns(telemetry: dict[str, list]) -> int:
    """Return the first scan whose endpoint lands on the route crate."""
    center_x_m, center_y_m = ROUTE_CRATE_CENTER_M
    length_m, width_m = ROUTE_CRATE_DIMENSIONS_M
    tolerance_m = 0.10
    for scan in telemetry['scans']:
        pose = _sample_pose(telemetry['poses'], scan[0])
        if pose is None:
            continue
        points = _scan_world_points(scan, pose)
        detected = (
            (np.abs(points[:, 0] - center_x_m)
             <= length_m / 2.0 + tolerance_m)
            & (np.abs(points[:, 1] - center_y_m)
               <= width_m / 2.0 + tolerance_m))
        if np.any(detected):
            return int(scan[0])
    raise RuntimeError('route crate was never detected by recorded /scan')


def _pedestrian_bounds(frame: np.ndarray):
    """Find the largest dark-red person torso above the robot body."""
    height = frame.shape[0]
    region = frame[:round(height * 0.68)]
    blue, green, red = cv2.split(region)
    red16 = red.astype(np.int16)
    mask = ((red > 65) & (red16 > green.astype(np.int16) + 20)
            & (red16 > blue.astype(np.int16) + 20)).astype(np.uint8)
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    candidates = [stats[index] for index in range(1, count)
                  if stats[index, cv2.CC_STAT_AREA] >= 160]
    if not candidates:
        return None
    left, top, width, box_height, _ = max(
        candidates, key=lambda item: item[cv2.CC_STAT_AREA])
    return int(left), int(top), int(left + width), int(top + box_height)


def _box(draw: ImageDraw.ImageDraw, xy, fill, radius=16, outline=None,
         width=1) -> None:
    draw.rounded_rectangle(
        xy, radius=radius, fill=fill, outline=outline, width=width)


def _panel(draw: ImageDraw.ImageDraw, rect: tuple[int, int, int, int]) -> None:
    _box(draw, rect, WHITE, radius=18, outline=(*INK[:3], 38), width=2)


def _observation(
        index: int, frames: dict[str, int], crate_visible: bool,
        speed_mps: float, angular_rps: float = 0.0) -> tuple[
            str, tuple[int, int, int, int], str]:
    """Describe only states evidenced by recorded events and telemetry."""
    if index < frames.get('goal_accepted', 0):
        return '센서 수신 · 출발 대기', INK, '/scan 수신 후 목표 승인 대기'
    succeeded = frames.get('succeeded', 10 ** 9)
    crossing = frames.get('obstacle_crossing_started', 10 ** 9)
    stopped = frames.get('stop_state', 10 ** 9)
    cleared = frames.get('clear_set_pose_requested', 10 ** 9)
    exit_complete = frames.get('obstacle_crossing_exit_completed', 10 ** 9)
    if index >= succeeded:
        return '목표 도착', TEAL, 'NavigateToPose 성공 이벤트 수신'
    command_stopped = abs(speed_mps) < 0.01 and abs(angular_rps) < 0.01
    if stopped <= index < cleared and command_stopped:
        return ('보행자 대응 · 정지 명령', MAGENTA,
                '/collision_monitor_state STOP · /cmd_vel 0')
    if crossing <= index < stopped:
        return ('보행자 횡단 중', MAGENTA,
                'Gazebo 보행자 횡단 이벤트 진행 중')
    if cleared <= index <= exit_complete:
        if not command_stopped:
            return ('주행 재개', TEAL,
                    '장애물 제거 이후 /cmd_vel 재개 관측')
        return '정지 유지', MAGENTA, '장애물 제거 이후 /cmd_vel 0 유지'
    if crate_visible:
        return ('주행 · 장애물 관측', BLUE,
                '/scan 끝점이 고정 장애물 영역에서 관측됨')
    return '자율주행', BLUE, '목표 승인 상태 · /cmd_vel 명령 관측'


def _state(index: int, frames: dict[str, int], crate_visible: bool,
           speed_mps: float) -> tuple[str, tuple[int, int, int, int]]:
    text, color, _ = _observation(
        index, frames, crate_visible, speed_mps)
    return text, color


def _map_transform(rect: tuple[int, int, int, int], center_x_m: float):
    left, top, right, bottom = rect
    padding_x, padding_y = 24, 48
    x_min_m, x_max_m = center_x_m - 4.2, center_x_m + 4.2
    y_min_m, y_max_m = -1.35, 1.35
    scale = min((right - left - 2 * padding_x) / (x_max_m - x_min_m),
                (bottom - top - 2 * padding_y) / (y_max_m - y_min_m))

    def transform(x_m: float, y_m: float) -> tuple[int, int]:
        x = round((left + right) / 2 + (x_m - center_x_m) * scale)
        y = round((top + bottom) / 2 - y_m * scale)
        return x, y

    return transform, (x_min_m, x_max_m, y_min_m, y_max_m)


def _draw_local_map(
        draw: ImageDraw.ImageDraw, rect: tuple[int, int, int, int],
        pose: tuple, scan: tuple, plan: list[tuple],
        occupied_cells: set[tuple[int, int]], crate_visible: bool,
        fonts: dict[str, ImageFont.FreeTypeFont], keepout_points=(),
        scan_pose=None) -> None:
    left, top, right, bottom = rect
    transform, bounds = _map_transform(rect, pose[1])
    x_min_m, x_max_m, y_min_m, y_max_m = bounds
    draw.text((left + 22, top + 17), '저장 지도 + 라이다 관측',
              font=fonts['panel'], fill=INK)
    draw.text((right - 130, top + 21), '경로 · 위치',
              font=fonts['small'], fill=TEAL)
    for y_m in (-1.2, 0.0, 1.2):
        x1, y1 = transform(x_min_m, y_m)
        x2, y2 = transform(x_max_m, y_m)
        draw.line((x1, y1, x2, y2), fill=(*INK[:3], 32), width=1)
    for x_m in range(math.floor(x_min_m), math.ceil(x_max_m) + 1):
        x1, y1 = transform(x_m, y_min_m)
        x2, y2 = transform(x_m, y_max_m)
        draw.line((x1, y1, x2, y2), fill=(*INK[:3], 24), width=1)

    map_pixels = []
    for cell_x, cell_y in occupied_cells:
        x_m = cell_x * MAP_RESOLUTION_M
        y_m = cell_y * MAP_RESOLUTION_M
        if x_min_m <= x_m <= x_max_m and y_min_m <= y_m <= y_max_m:
            map_pixels.append(transform(x_m, y_m))
    if map_pixels:
        draw.point(map_pixels, fill=(*INK[:3], 155))
    forbidden = [transform(float(x_m), float(y_m))
                 for x_m, y_m in keepout_points
                 if x_min_m <= x_m <= x_max_m and y_min_m <= y_m <= y_max_m]
    if forbidden:
        for x, y in forbidden:
            draw.rectangle((x - 1, y - 1, x + 1, y + 1),
                           fill=MAGENTA)
        draw.text((left + 22, top + 45), '분홍: 진입 금지 · 여유 포함',
                  font=fonts['small'], fill=MAGENTA)

    if plan:
        visible_plan = [transform(x_m, y_m) for x_m, y_m in plan
                        if x_min_m <= x_m <= x_max_m
                        and y_min_m <= y_m <= y_max_m]
        if len(visible_plan) >= 2:
            draw.line(visible_plan, fill=BLUE, width=3)

    current_points = (_scan_world_points(scan, scan_pose)
                      if scan_pose is not None else np.empty((0, 2)))
    points = [transform(float(x_m), float(y_m))
              for x_m, y_m in current_points
              if x_min_m <= x_m <= x_max_m
              and y_min_m <= y_m <= y_max_m]
    if points:
        draw.point(points, fill=TEAL)

    if crate_visible:
        half_l = ROUTE_CRATE_DIMENSIONS_M[0] / 2.0
        half_w = ROUTE_CRATE_DIMENSIONS_M[1] / 2.0
        for x_m, y_m in current_points:
            if (abs(x_m - ROUTE_CRATE_CENTER_M[0]) <= half_l + 0.1
                    and abs(y_m - ROUTE_CRATE_CENTER_M[1]) <= half_w + 0.1
                    and x_min_m <= x_m <= x_max_m
                    and y_min_m <= y_m <= y_max_m):
                x, y = transform(float(x_m), float(y_m))
                draw.ellipse((x - 2, y - 2, x + 2, y + 2),
                             fill=BLUE)

    robot_x, robot_y = transform(pose[1], pose[2])
    heading = -pose[3]
    triangle = []
    for angle, radius in ((0.0, 14), (2.52, 10), (-2.52, 10)):
        triangle.append((
            robot_x + round(math.cos(heading + angle) * radius),
            robot_y + round(math.sin(heading + angle) * radius)))
    draw.polygon(triangle, fill=WHITE, outline=BLUE)
    draw.text((left + 22, bottom - 31),
              f'pose  x {pose[1]:+.2f} m  y {pose[2]:+.2f} m',
              font=fonts['small'], fill=(*INK[:3], 155))
    draw.text((right - 174, bottom - 31),
              '저장 지도 기준', font=fonts['small'],
              fill=(*INK[:3], 155))


def _draw_lidar(
        draw: ImageDraw.ImageDraw, rect: tuple[int, int, int, int],
        scan: tuple, frame_index: int, fps: float,
        fonts: dict[str, ImageFont.FreeTypeFont]) -> int:
    left, top, right, bottom = rect
    draw.text((left + 22, top + 17), '360° LiDAR · /scan',
              font=fonts['panel'], fill=INK)
    center_x = (left + right) // 2
    center_y = top + 191
    radius = 116
    for ring, distance_m in ((1, 1), (2, 2), (3, 3)):
        ring_radius = round(radius * ring / 3)
        draw.ellipse((center_x - ring_radius, center_y - ring_radius,
                      center_x + ring_radius, center_y + ring_radius),
                     outline=(*INK[:3], 42), width=1)
        draw.text((center_x + 4, center_y - ring_radius + 2),
                  f'{distance_m}m', font=fonts['tiny'],
                  fill=(*INK[:3], 140))
    draw.line((center_x - radius, center_y, center_x + radius, center_y),
              fill=(*INK[:3], 34), width=1)
    draw.line((center_x, center_y - radius, center_x, center_y + radius),
              fill=(*INK[:3], 34), width=1)
    local = _scan_local_points(scan)
    valid_count = len(local)
    if valid_count:
        plot_points = []
        for x_m, y_m in local:
            if math.hypot(x_m, y_m) > 3.0:
                continue
            scale = radius / 3.0
            plot_points.append((center_x + round(float(x_m) * scale),
                                center_y - round(float(y_m) * scale)))
        draw.point(plot_points, fill=TEAL)
        for x, y in plot_points[::12]:
            draw.line((center_x, center_y, x, y),
                      fill=(*TEAL[:3], 65), width=1)
    draw.ellipse((center_x - 7, center_y - 7, center_x + 7, center_y + 7),
                 fill=INK)
    draw.text((left + 22, bottom - 32),
              f'유효 리턴 {valid_count} / {len(scan[5])}',
              font=fonts['small'], fill=(*INK[:3], 155))
    draw.text((right - 142, bottom - 32), '표시 반경 3 m',
              font=fonts['small'], fill=TEAL)
    return valid_count


def _fonts() -> dict[str, ImageFont.FreeTypeFont]:
    return {
        'title': _font(BOLD_FONT, 31),
        'subtitle': _font(REGULAR_FONT, 18),
        'state': _font(BOLD_FONT, 20),
        'panel': _font(BOLD_FONT, 20),
        'small': _font(REGULAR_FONT, 15),
        'tiny': _font(REGULAR_FONT, 11),
        'value': _font(BOLD_FONT, 25),
    }


def _render_dashboard(
        frame: np.ndarray, index: int, total_frames: int, fps: float,
        event_frames: dict[str, int], timestamp_ns: int,
        telemetry: dict[str, list], occupied_cells: set[tuple[int, int]],
        crate_detection_ns: int, metrics: dict, compressed: bool = False,
        fonts: dict[str, ImageFont.FreeTypeFont] | None = None) -> np.ndarray:
    if fonts is None:
        fonts = _fonts()
    canvas = Image.new('RGBA', CANVAS_SIZE, SURFACE)
    draw = ImageDraw.Draw(canvas, 'RGBA')
    camera_image = Image.fromarray(
        cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).resize(
            (CAMERA_RECT[2] - CAMERA_RECT[0],
             CAMERA_RECT[3] - CAMERA_RECT[1]), Image.Resampling.LANCZOS)
    canvas.paste(camera_image, (CAMERA_RECT[0], CAMERA_RECT[1]))
    draw.rounded_rectangle(CAMERA_RECT, radius=18,
                           outline=(*INK[:3], 52), width=2)
    _panel(draw, MAP_RECT)
    _panel(draw, SCAN_RECT)
    _panel(draw, FOOTER_RECT)

    pose = _sample_pose(telemetry['poses'], timestamp_ns)
    scan = _latest(telemetry['scans'], timestamp_ns,
                   (0, 0., 0., 0., 0., np.array([], dtype=np.float32)))
    plan = _latest(telemetry['plans'], timestamp_ns, (0, []))[1]
    cmd = _latest(telemetry['cmd'], timestamp_ns, (0, 0., 0.))
    odom = _latest(telemetry['odom'], timestamp_ns, (0, 0., 0.))
    crate_visible = timestamp_ns >= crate_detection_ns
    state_text, state_color, state_reason = _observation(
        index, event_frames, crate_visible, cmd[1], cmd[2])

    draw.text((36, 29), 'JDAMR 자율주행 관제', font=fonts['title'],
              fill=INK)
    draw.text((548, 40),
              'Gazebo 3D · 경로 계획 · 위치 추정 · LiDAR · 진입 금지',
              font=fonts['subtitle'], fill=(*INK[:3], 155))
    badge_width = round(draw.textlength(state_text, font=fonts['state'])) + 36
    _box(draw, (1888 - badge_width, 24, 1888, 78), state_color, radius=16)
    draw.text((1888 - badge_width + 18, 36), state_text,
              font=fonts['state'], fill=WHITE)

    if compressed:
        _box(draw, (1158, 124, 1274, 157), MAGENTA, radius=9)
        draw.text((1176, 128), '주행 8×', font=fonts['small'],
                  fill=WHITE)
    source_elapsed_s = (
        timestamp_ns - metrics.get('capture_start_ns', timestamp_ns)) / 1e9
    _box(draw, (52, 124, 288, 160), (*WHITE[:3], 232), radius=8,
         outline=(*INK[:3], 32))
    draw.text((65, 129), f'기록 시각 +{source_elapsed_s:06.2f} s',
              font=fonts['small'], fill=INK)
    crossing_start = event_frames.get('obstacle_crossing_started', 10 ** 9)
    crossing_end = event_frames.get(
        'obstacle_crossing_exit_completed', -1)
    if crossing_start <= index <= crossing_end:
        _box(draw, (52, 171, 288, 207), MAGENTA, radius=8)
        draw.text((65, 176), '보행자 횡단 시나리오', font=fonts['small'],
                  fill=WHITE)

    if pose is not None:
        _draw_local_map(
            draw, MAP_RECT, pose, scan, plan, occupied_cells,
            crate_visible, fonts, metrics.get('keepout_points', ()),
            _sample_pose(telemetry['poses'], scan[0]))
    else:
        draw.text((MAP_RECT[0] + 24, MAP_RECT[1] + 24),
                  '위치 데이터 수신 대기', font=fonts['panel'],
                  fill=(*INK[:3], 155))
    _draw_lidar(draw, SCAN_RECT, scan, index, fps, fonts)

    left, top, right, bottom = FOOTER_RECT
    draw.text((left + 22, top + 16), '현재 주행 텔레메트리',
              font=fonts['panel'], fill=INK)
    live_cells = [
        ('명령 선속도', f'{cmd[1]:+.3f} m/s' if cmd[0] else '수신 대기'),
        ('명령 회전속도', f'{cmd[2]:+.3f} rad/s' if cmd[0] else '수신 대기'),
        ('실측 선속도', f'{odom[1]:+.3f} m/s' if odom[0] else '수신 대기'),
        ('실측 회전속도', f'{odom[2]:+.3f} rad/s' if odom[0] else '수신 대기'),
    ]
    cell_width = 210
    for cell, (label, value) in enumerate(live_cells):
        x = left + 22 + cell * cell_width
        draw.text((x, top + 57), label, font=fonts['small'],
                  fill=(*INK[:3], 145))
        draw.text((x, top + 84), value, font=fonts['value'], fill=TEAL)
        if cell:
            draw.line((x - 14, top + 57, x - 14, top + 126),
                      fill=(*INK[:3], 30), width=1)
    state_left = left + 22 + len(live_cells) * cell_width + 12
    draw.line((state_left - 18, top + 57, state_left - 18, top + 130),
              fill=(*INK[:3], 30), width=1)
    draw.text((state_left, top + 57), '현재 관측 상태', font=fonts['small'],
              fill=(*INK[:3], 145))
    draw.text((state_left, top + 82), state_text, font=fonts['value'],
              fill=state_color)
    draw.text((state_left, top + 118), f'근거  {state_reason}',
              font=fonts['small'], fill=INK)
    succeeded = index >= event_frames.get('succeeded', 10 ** 9)
    if succeeded:
        clearance = metrics.get('keepout_clearance_m')
        clearance_text = (
            f'금지구역 이격 {clearance:.3f} m'
            if clearance is not None else '금지구역 이격 해당 없음')
        summary_text = (
            f'완주 검증  접촉 {metrics["contact_count"]}회 · '
            f'목표 취소 {metrics["goal_cancel_count"]}회 · '
            f'{clearance_text}')
        summary_width = round(
            draw.textlength(summary_text, font=fonts['small'])) + 30
        _box(draw, (right - summary_width - 20, top + 12,
                    right - 20, top + 45), SURFACE, radius=9,
             outline=(*TEAL[:3], 80))
        draw.text((right - summary_width - 5, top + 17), summary_text,
                  font=fonts['small'], fill=TEAL)
    progress_left = left + 22
    progress_right = right - 22
    progress_y = bottom - 23
    draw.line((progress_left, progress_y, progress_right, progress_y),
              fill=(*INK[:3], 36), width=4)
    current_x = progress_left + round(
        (progress_right - progress_left) * index / max(1, total_frames - 1))
    draw.line((progress_left, progress_y, current_x, progress_y),
              fill=BLUE, width=4)
    draw.ellipse((current_x - 6, progress_y - 6,
                  current_x + 6, progress_y + 6),
                 fill=WHITE, outline=BLUE, width=2)
    return cv2.cvtColor(np.asarray(canvas.convert('RGB')), cv2.COLOR_RGB2BGR)


def _write_gif(source: Path, output: Path, start_s: float = 0.0) -> None:
    subprocess.run([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-ss', str(start_s), '-t', '12', '-i', str(source),
        '-filter_complex_threads', '1', '-filter_complex',
        ('fps=8,scale=960:-1:flags=lanczos,split[s0][s1];'
         '[s0]palettegen=max_colors=128[p];'
         '[s1][p]paletteuse=dither=bayer:bayer_scale=3'),
        str(output),
    ], check=True)


def main() -> int:
    """Validate one passing run and render its public media bundle."""
    from contextlib import ExitStack

    def cleanup_writer(writer) -> None:
        stdin = writer.stdin
        if stdin is not None and not stdin.closed:
            try:
                stdin.close()
            except OSError:
                pass
        if writer.poll() is None:
            writer.terminate()
        try:
            writer.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            writer.kill()
            writer.wait()

    parser = argparse.ArgumentParser()
    parser.add_argument('--run-root', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error('--output-dir must be new or empty')
    args.output_dir.mkdir(parents=True, exist_ok=True)

    summary_path = args.run_root / 'summary.json'
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    if summary.get('status') != 'PASS' or len(summary.get('results', [])) != 1:
        raise RuntimeError('portfolio media requires one passing run')
    result = summary['results'][0]
    if result.get('status') != 'PASS' or 'simulator_video' not in result:
        raise RuntimeError('passing Gazebo sensor video evidence is missing')
    raw_video = Path(result['simulator_video']['raw_video']['path'])
    if _sha256(raw_video) != result['simulator_video']['raw_video']['sha256']:
        raise RuntimeError('raw Gazebo video hash mismatch')
    _validate_capture_geometry(result)
    evidence_path = args.run_root / result['case'] / 'scenario.json'
    evidence = json.loads(evidence_path.read_text(encoding='utf-8'))
    scenario = result['scenario']
    metrics = {
        'goal_send_count': scenario['goal_send_count'],
        'goal_cancel_count': scenario['goal_cancel_count'],
        'contact_count': scenario['contact_count'],
        'protected_clearance_m':
            scenario['protected_envelope_minimum_clearance_m'],
        'command_latency_s': scenario['observer_scan_to_zero_command_s'],
        'entry_side': evidence.get('obstacle_entry_side'),
        'keepout_clearance_m': result.get('keepout_route_evidence', {}).get(
            'minimum_footprint_clearance_m'),
    }
    mcap = Path(result['recording']['mcap']['path'])
    if _sha256(mcap) != result['recording']['mcap']['sha256']:
        raise RuntimeError('source MCAP hash mismatch')
    telemetry = _read_telemetry(mcap)
    crate_detection_ns = _crate_detection_ns(telemetry)

    with ExitStack() as resources:
        capture = cv2.VideoCapture(str(raw_video))
        resources.callback(capture.release)
        source_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        source_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        if (source_width, source_height) != (1280, 720) or fps <= 0:
            raise RuntimeError('unexpected Gazebo source video geometry')
        total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        metadata_path = Path(
            result['simulator_video']['capture_metadata']['path'])
        metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        if metadata.get('frames') != total_frames:
            raise RuntimeError('camera metadata frame count mismatch')
        if not metadata.get('frame_timestamps'):
            raise RuntimeError(
                'dashboard requires per-frame capture timestamps')
        metrics['capture_start_ns'] = (
            metadata['frame_timestamps'][0]['wall_ns'])
        wall_times = [
            item['wall_ns'] for item in metadata['frame_timestamps']]
        if (len(wall_times) != total_frames or any(
                right <= left
                for left, right in zip(wall_times, wall_times[1:]))):
            raise RuntimeError('camera wall timestamps must strictly increase')
        event_frames = _event_frames(evidence, metadata, total_frames)
        obstacle_start, obstacle_end = _event_frame_interval(
            evidence, metadata, total_frames)
        arrival_start = event_frames.get(
            'succeeded',
            max(obstacle_end + 1, total_frames - round(2.0 * fps)))
        full_path = args.output_dir / 'gazebo_sensor_dashboard_full.mp4'
        highlight_path = (
            args.output_dir / 'gazebo_sensor_dashboard_highlight.mp4')
        full_writer = _ffmpeg_writer(full_path, *CANVAS_SIZE, fps)
        resources.callback(cleanup_writer, full_writer)
        highlight_writer = _ffmpeg_writer(
            highlight_path, *CANVAS_SIZE, fps)
        resources.callback(cleanup_writer, highlight_writer)
        if full_writer.stdin is None or highlight_writer.stdin is None:
            raise RuntimeError('ffmpeg stdin unavailable')
        highlight_start = 0
        normal_until = min(
            arrival_start - 1, obstacle_end + round(4.0 * fps))
        compressed_stride = 8
        posters = {
            event_frames.get('physical_stop', obstacle_start):
                args.output_dir / 'gazebo_lidar_pedestrian_stop.png',
            total_frames - 1:
                args.output_dir / 'gazebo_sensor_goal_arrival.png',
        }
        fonts = _fonts()
        map_path = (
            Path(__file__).parent
            / 'assets/nav_obstacle/slam_corridor_eval.yaml')
        keepout_map_path = args.run_root / 'assets/sim_keepout_mask.yaml'
        occupied_cells = {(round(float(x_m) / MAP_RESOLUTION_M),
                           round(float(y_m) / MAP_RESOLUTION_M))
                          for x_m, y_m in _saved_map_points(map_path)}
        metrics['keepout_points'] = _saved_map_points(keepout_map_path)
        if result.get('keepout_route_evidence', {}).get('status') == 'PASS':
            target = result['keepout_route_evidence'][
                'minimum_clearance_pose_xy_yaw']
            sample = min(telemetry['ground_truth'],
                         key=lambda point: math.hypot(
                             point[1] - target[0], point[2] - target[1]))
            poster_index = min(
                total_frames - 1, bisect.bisect_left(wall_times, sample[0]))
            posters[poster_index] = (
                args.output_dir / 'gazebo_keepout_detour.png')
        highlight_frames = 0
        full_frames = 0
        pre_detection_encoded_frames = 0
        index = 0
        crate_first_frame = None
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            timestamp_ns = _frame_wall_ns(metadata, index, total_frames)
            if (crate_first_frame is None
                    and timestamp_ns >= crate_detection_ns):
                crate_first_frame = index
            compressed = (
                normal_until < index < arrival_start
                and (index - normal_until) % compressed_stride == 0)
            annotated = _render_dashboard(
                frame, index, total_frames, fps, event_frames, timestamp_ns,
                telemetry, occupied_cells, crate_detection_ns, metrics,
                compressed=False, fonts=fonts)
            next_ns = (
                _frame_wall_ns(metadata, index + 1, total_frames)
                if index + 1 < total_frames
                else timestamp_ns + round(1e9 / fps))
            origin_ns = metadata['frame_timestamps'][0]['wall_ns']
            copies = max(
                0, round((next_ns - origin_ns) * fps / 1e9)
                - round((timestamp_ns - origin_ns) * fps / 1e9))
            pixels = annotated.tobytes()
            for _ in range(copies):
                full_writer.stdin.write(pixels)
            full_frames += copies
            if timestamp_ns < crate_detection_ns:
                pre_detection_encoded_frames += copies
            selected = (
                highlight_start <= index <= normal_until
                or compressed or index >= arrival_start)
            if selected:
                if compressed:
                    highlight_frame = _render_dashboard(
                        frame, index, total_frames, fps, event_frames,
                        timestamp_ns, telemetry, occupied_cells,
                        crate_detection_ns, metrics, compressed=True,
                        fonts=fonts)
                    pixels = highlight_frame.tobytes()
                for _ in range(copies):
                    highlight_writer.stdin.write(pixels)
                highlight_frames += copies
            if index in posters:
                if not cv2.imwrite(str(posters[index]), annotated):
                    raise RuntimeError(
                        f'could not write poster: {posters[index]}')
            index += 1
        full_writer.stdin.close()
        highlight_writer.stdin.close()
        returncodes = (full_writer.wait(), highlight_writer.wait())
        if any(returncode != 0 for returncode in returncodes):
            raise RuntimeError('ffmpeg video encoding failed')
    if (index != total_frames or crate_first_frame is None
            or crate_first_frame <= 0 or pre_detection_encoded_frames <= 0):
        raise RuntimeError('source frames or LiDAR detection are incomplete')

    capture_elapsed_s = (
        metadata['frame_timestamps'][-1]['wall_ns']
        - metadata['frame_timestamps'][0]['wall_ns']) / 1e9
    encoded_duration_s = full_frames / fps
    expected_duration_s = capture_elapsed_s + 1.0 / fps
    if abs(encoded_duration_s - expected_duration_s) > 0.5 / fps + 1e-8:
        raise RuntimeError(
            'encoded duration differs from recorded elapsed time')

    gif_path = args.output_dir / 'gazebo_sensor_dashboard_highlight.gif'
    crossing_s = (
        metadata['frame_timestamps'][obstacle_start]['wall_ns']
        - metadata['frame_timestamps'][0]['wall_ns']) / 1e9
    _write_gif(highlight_path, gif_path, max(0.0, crossing_s - 2.0))
    outputs = [full_path, highlight_path, gif_path, *posters.values()]
    manifest = {
        'schema_version': 2,
        'status': 'PASS',
        'claim_scope': result['claim_scope'],
        'source': {
            'type': 'gazebo_camera_plus_synchronized_mcap',
            'video_path': str(raw_video.resolve()),
            'video_sha256': _sha256(raw_video),
            'mcap_path': str(mcap.resolve()),
            'mcap_sha256': _sha256(mcap),
            'summary': str(summary_path.resolve()),
            'summary_sha256': _sha256(summary_path),
            'time_alignment': (
                'per_frame_callback_wall_time_to_mcap_receipt_time'),
            'timing_limit': 'receipt timestamps include transport latency',
            'full_frames': full_frames,
            'capture_elapsed_s': capture_elapsed_s,
            'encoded_duration_s': encoded_duration_s,
            'expected_duration_with_last_frame_s': expected_duration_s,
            'native_capture_fps': (total_frames - 1) / capture_elapsed_s,
            'pre_detection_encoded_frames': pre_detection_encoded_frames,
        },
        'dashboard': {
            'canvas': {'width': CANVAS_SIZE[0], 'height': CANVAS_SIZE[1]},
            'topics': [
                '/scan', '/amcl_pose', '/tf', '/odom', '/plan', '/cmd_vel'],
            'scan_messages': len(telemetry['scans']),
            'saved_map_occupied_cells': len(occupied_cells),
            'saved_map_yaml_sha256': _sha256(map_path),
            'saved_map_image_sha256': _sha256(map_path.with_suffix('.pgm')),
            'route_crate_first_detection_log_ns': crate_detection_ns,
            'route_crate_first_visible_frame': crate_first_frame,
            'route_crate_hidden_before_scan_detection': True,
            'map_semantic': (
                'saved_occupancy_map_plus_current_scan_using_amcl_pose; '
                'localization_navigation_not_live_slam'),
        },
        'detected_visual_events': {
            'pedestrian_first_frame': obstacle_start,
            'pedestrian_last_frame': obstacle_end,
            'full_corridor_crossing_completed': True,
            'arrival_overlay_first_frame': arrival_start,
        },
        'verified_metrics': metrics,
        'highlight': {
            'frames': highlight_frames,
            'fps': fps,
            'middle_cruise_stride': compressed_stride,
            'middle_cruise_label': '8x',
        },
        'outputs': [{
            'path': str(path.resolve()),
            'size_bytes': path.stat().st_size,
            'sha256': _sha256(path),
        } for path in outputs],
    }
    manifest_path = args.output_dir / 'portfolio_media_manifest.json'
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2,
                   sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(manifest, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
