"""Fuse synchronized RGB-D frames using wheel odometry as a pose seed."""

import argparse
from bisect import bisect_left
from collections import Counter
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path

from jdamr_cube_vslam.rgbd_snapshot_ply import (
    project_registered_depth,
    write_binary_ply,
)
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CameraInfo, Image


COLOR_TOPIC = '/camera/color/image_raw'
DEPTH_TOPIC = '/camera/depth/image_raw'
CAMERA_INFO_TOPIC = '/camera/color/camera_info'
ODOM_TOPIC = '/odom'


@dataclass(frozen=True)
class ImageFrame:
    """Identify an image frame without retaining its pixel payload."""

    ordinal: int
    stamp_ns: int


@dataclass(frozen=True)
class OdomPose:
    """Represent one planar wheel-odometry sample."""

    stamp_ns: int
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class FrameSelection:
    """Describe one RGB-D pair selected for fusion."""

    target_yaw_deg: float
    color: ImageFrame
    depth: ImageFrame
    pose: OdomPose


@dataclass(frozen=True)
class CameraExtrinsic:
    """Describe the camera-link pose relative to the base frame."""

    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def interpolate_pose(poses: list[OdomPose], stamp_ns: int) -> OdomPose:
    """Linearly interpolate position and unwrapped yaw at a timestamp."""
    if not poses:
        raise ValueError('at least one odometry pose is required')
    stamps_ns = [pose.stamp_ns for pose in poses]
    index = bisect_left(stamps_ns, stamp_ns)
    if index == 0:
        return OdomPose(stamp_ns, poses[0].x_m, poses[0].y_m,
                        poses[0].yaw_rad)
    if index == len(poses):
        last = poses[-1]
        return OdomPose(stamp_ns, last.x_m, last.y_m, last.yaw_rad)
    before = poses[index - 1]
    after = poses[index]
    fraction = ((stamp_ns - before.stamp_ns)
                / (after.stamp_ns - before.stamp_ns))
    return OdomPose(
        stamp_ns=stamp_ns,
        x_m=before.x_m + fraction * (after.x_m - before.x_m),
        y_m=before.y_m + fraction * (after.y_m - before.y_m),
        yaw_rad=before.yaw_rad
        + fraction * (after.yaw_rad - before.yaw_rad),
    )


def unwrap_odom_yaw(poses: list[OdomPose]) -> list[OdomPose]:
    """Return poses with continuous yaw across the plus/minus-pi boundary."""
    if not poses:
        return []
    yaws = np.unwrap([pose.yaw_rad for pose in poses])
    return [
        OdomPose(pose.stamp_ns, pose.x_m, pose.y_m, float(yaw))
        for pose, yaw in zip(poses, yaws)
    ]


def pair_rgbd_frames(
        color_frames: list[ImageFrame], depth_frames: list[ImageFrame],
        max_pair_delta_ms: float) -> tuple[list[tuple[ImageFrame, ImageFrame]],
                                           list[int]]:
    """Pair an already synchronized processing bag by topic ordinal."""
    if max_pair_delta_ms <= 0.0:
        raise ValueError('max_pair_delta_ms must be positive')
    if len(color_frames) != len(depth_frames):
        raise ValueError(
            'processing bag must contain equal color and depth counts')
    maximum_delta_ns = round(max_pair_delta_ms * 1e6)
    pairs = []
    deltas_ns = []
    for color, depth in zip(color_frames, depth_frames):
        delta_ns = abs(color.stamp_ns - depth.stamp_ns)
        if delta_ns > maximum_delta_ns:
            raise ValueError(
                f'RGB-D pair exceeds sync limit: {delta_ns / 1e6:.3f} ms')
        pairs.append((color, depth))
        deltas_ns.append(delta_ns)
    return pairs, deltas_ns


def select_frames_by_yaw(
        pairs: list[tuple[ImageFrame, ImageFrame]],
        poses: list[OdomPose], yaw_step_deg: float) -> list[FrameSelection]:
    """Choose pairs nearest to regular wheel-odometry yaw increments."""
    if yaw_step_deg <= 0.0:
        raise ValueError('yaw_step_deg must be positive')
    if not pairs:
        raise ValueError('at least one RGB-D pair is required')
    poses = unwrap_odom_yaw(sorted(poses, key=lambda pose: pose.stamp_ns))
    paired_poses = [
        interpolate_pose(poses, color.stamp_ns) for color, _ in pairs
    ]
    initial_yaw = paired_poses[0].yaw_rad
    relative_yaws = np.array([
        pose.yaw_rad - initial_yaw for pose in paired_poses
    ])
    final_yaw = float(relative_yaws[-1])
    direction = 1.0 if final_yaw >= 0.0 else -1.0
    extent_deg = abs(math.degrees(final_yaw))
    targets_deg = list(np.arange(0.0, extent_deg, yaw_step_deg))
    targets_deg.append(extent_deg)
    selections = []
    used_indices = set()
    for magnitude_deg in targets_deg:
        target_rad = direction * math.radians(magnitude_deg)
        index = int(np.argmin(np.abs(relative_yaws - target_rad)))
        if index in used_indices:
            continue
        used_indices.add(index)
        color, depth = pairs[index]
        selections.append(FrameSelection(
            target_yaw_deg=direction * magnitude_deg,
            color=color,
            depth=depth,
            pose=paired_poses[index],
        ))
    return selections


def rotation_matrix(roll_rad: float, pitch_rad: float,
                    yaw_rad: float) -> np.ndarray:
    """Build a Z-Y-X rotation matrix."""
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def transform_optical_points(
        points_optical_m: np.ndarray, pose: OdomPose,
        initial_pose: OdomPose,
        camera_extrinsic: CameraExtrinsic) -> np.ndarray:
    """Transform optical-frame points into the initial base frame."""
    points_camera_link = np.column_stack((
        points_optical_m[:, 2],
        -points_optical_m[:, 0],
        -points_optical_m[:, 1],
    ))
    camera_rotation = rotation_matrix(
        camera_extrinsic.roll_rad,
        camera_extrinsic.pitch_rad,
        camera_extrinsic.yaw_rad,
    )
    camera_translation = np.array([
        camera_extrinsic.x_m,
        camera_extrinsic.y_m,
        camera_extrinsic.z_m,
    ])
    points_base = points_camera_link @ camera_rotation.T + camera_translation

    initial_cosine = math.cos(initial_pose.yaw_rad)
    initial_sine = math.sin(initial_pose.yaw_rad)
    odom_delta = np.array([
        pose.x_m - initial_pose.x_m,
        pose.y_m - initial_pose.y_m,
    ])
    relative_translation = np.array([
        initial_cosine * odom_delta[0] + initial_sine * odom_delta[1],
        -initial_sine * odom_delta[0] + initial_cosine * odom_delta[1],
    ])
    relative_yaw = pose.yaw_rad - initial_pose.yaw_rad
    cosine = math.cos(relative_yaw)
    sine = math.sin(relative_yaw)
    transformed = points_base.copy()
    transformed[:, 0] = (
        cosine * points_base[:, 0] - sine * points_base[:, 1]
        + relative_translation[0])
    transformed[:, 1] = (
        sine * points_base[:, 0] + cosine * points_base[:, 1]
        + relative_translation[1])
    return transformed


def voxel_average(points_m: np.ndarray, colors_rgb: np.ndarray,
                  voxel_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Average coordinates and colors for each occupied voxel."""
    if voxel_m <= 0.0:
        raise ValueError('voxel_m must be positive')
    if len(points_m) != len(colors_rgb):
        raise ValueError('point and color counts must match')
    if len(points_m) == 0:
        return points_m.astype(np.float32), colors_rgb.astype(np.uint8)
    keys = np.floor(points_m / voxel_m).astype(np.int64)
    _, inverse, counts = np.unique(
        keys, axis=0, return_inverse=True, return_counts=True)
    point_sums = np.column_stack([
        np.bincount(inverse, weights=points_m[:, axis])
        for axis in range(3)
    ])
    color_sums = np.column_stack([
        np.bincount(inverse, weights=colors_rgb[:, axis])
        for axis in range(3)
    ])
    averaged_points = point_sums / counts[:, None]
    averaged_colors = np.rint(color_sums / counts[:, None])
    return averaged_points.astype(np.float32), averaged_colors.astype(np.uint8)


def evaluate_yaw_coverage(actual_yaw_deg: float,
                          expected_yaw_deg: float | None) -> dict:
    """Classify whether the recorded yaw covers the requested rotation."""
    if expected_yaw_deg is None:
        return {
            'expected_yaw_deg': None,
            'yaw_coverage_percent': None,
            'capture_complete': None,
            'result_status': 'not_evaluated',
        }
    if expected_yaw_deg <= 0.0:
        raise ValueError('expected_yaw_deg must be positive')
    coverage_percent = abs(actual_yaw_deg) / expected_yaw_deg * 100.0
    complete = coverage_percent >= 90.0
    return {
        'expected_yaw_deg': expected_yaw_deg,
        'yaw_coverage_percent': coverage_percent,
        'capture_complete': complete,
        'result_status': 'complete' if complete else 'partial_capture',
    }


def _open_reader(input_bag: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_bag), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def _message_stamp_ns(message) -> int:
    return (message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec)


def _scan_bag(input_bag: Path):
    reader = _open_reader(input_bag)
    ordinals = Counter()
    color_frames = []
    depth_frames = []
    poses = []
    camera_info = None
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        ordinal = ordinals[topic]
        ordinals[topic] += 1
        if topic == COLOR_TOPIC:
            message = deserialize_message(serialized, Image)
            color_frames.append(ImageFrame(
                ordinal, _message_stamp_ns(message)))
        elif topic == DEPTH_TOPIC:
            message = deserialize_message(serialized, Image)
            depth_frames.append(ImageFrame(
                ordinal, _message_stamp_ns(message)))
        elif topic == CAMERA_INFO_TOPIC and camera_info is None:
            camera_info = deserialize_message(serialized, CameraInfo)
        elif topic == ODOM_TOPIC:
            message = deserialize_message(serialized, Odometry)
            orientation = message.pose.pose.orientation
            position = message.pose.pose.position
            poses.append(OdomPose(
                stamp_ns=_message_stamp_ns(message),
                x_m=position.x,
                y_m=position.y,
                yaw_rad=quaternion_yaw(
                    orientation.x, orientation.y,
                    orientation.z, orientation.w),
            ))
    if camera_info is None:
        raise RuntimeError('color camera info was not recorded')
    if not poses:
        raise RuntimeError('wheel odometry was not recorded')
    return color_frames, depth_frames, poses, camera_info


def _load_selected_images(input_bag: Path,
                          selections: list[FrameSelection]):
    wanted = {
        COLOR_TOPIC: {selection.color.ordinal for selection in selections},
        DEPTH_TOPIC: {selection.depth.ordinal for selection in selections},
    }
    images = {COLOR_TOPIC: {}, DEPTH_TOPIC: {}}
    ordinals = Counter()
    reader = _open_reader(input_bag)
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        ordinal = ordinals[topic]
        ordinals[topic] += 1
        if topic in wanted and ordinal in wanted[topic]:
            images[topic][ordinal] = deserialize_message(serialized, Image)
    return images


def _image_arrays(color_message: Image,
                  depth_message: Image) -> tuple[np.ndarray, np.ndarray]:
    if color_message.encoding != 'rgb8':
        raise ValueError(
            f'expected rgb8 color, got {color_message.encoding}')
    if depth_message.encoding != '16UC1':
        raise ValueError(
            f'expected 16UC1 depth, got {depth_message.encoding}')
    color = np.frombuffer(color_message.data, dtype=np.uint8).reshape(
        color_message.height, color_message.width, 3)
    depth = np.frombuffer(depth_message.data, dtype=np.uint16).reshape(
        depth_message.height, depth_message.width)
    return color, depth


def _percentile_95(values: list[int]) -> float:
    ordered = sorted(values)
    index = round(0.95 * (len(ordered) - 1))
    return ordered[index] / 1e6


def fuse_bag(
        input_bag: Path, output_directory: Path,
        yaw_step_deg: float = 5.0, max_pair_delta_ms: float = 30.0,
        pixel_step: int = 2, minimum_depth_m: float = 0.25,
        maximum_depth_m: float = 4.0, voxel_m: float = 0.02,
        camera_extrinsic: CameraExtrinsic = CameraExtrinsic(),
        expected_yaw_deg: float | None = None) -> dict:
    """Create an odometry-seeded colored point cloud from a paired bag."""
    if not input_bag.is_dir():
        raise FileNotFoundError(f'input bag not found: {input_bag}')
    if pixel_step <= 0:
        raise ValueError('pixel_step must be positive')
    output_directory.mkdir(parents=True, exist_ok=True)
    color_frames, depth_frames, poses, camera_info = _scan_bag(input_bag)
    pairs, pair_deltas_ns = pair_rgbd_frames(
        color_frames, depth_frames, max_pair_delta_ms)
    selections = select_frames_by_yaw(pairs, poses, yaw_step_deg)
    images = _load_selected_images(input_bag, selections)
    unwrapped_poses = unwrap_odom_yaw(
        sorted(poses, key=lambda pose: pose.stamp_ns))
    initial_pose = interpolate_pose(
        unwrapped_poses, selections[0].color.stamp_ns)

    transformed_clouds = []
    color_clouds = []
    selected_rows = []
    for selection in selections:
        color_message = images[COLOR_TOPIC][selection.color.ordinal]
        depth_message = images[DEPTH_TOPIC][selection.depth.ordinal]
        color, depth = _image_arrays(color_message, depth_message)
        points_optical, colors = project_registered_depth(
            depth, color, camera_info,
            pixel_step=pixel_step,
            minimum_depth_m=minimum_depth_m,
            maximum_depth_m=maximum_depth_m,
        )
        transformed = transform_optical_points(
            points_optical, selection.pose, initial_pose,
            camera_extrinsic)
        transformed_clouds.append(transformed)
        color_clouds.append(colors)
        selected_rows.append({
            'target_yaw_deg': selection.target_yaw_deg,
            'actual_yaw_deg': math.degrees(
                selection.pose.yaw_rad - initial_pose.yaw_rad),
            'timestamp_s': selection.color.stamp_ns / 1e9,
            'color_ordinal': selection.color.ordinal,
            'depth_ordinal': selection.depth.ordinal,
            'sync_delta_ms': abs(
                selection.color.stamp_ns - selection.depth.stamp_ns) / 1e6,
            'projected_point_count': len(points_optical),
        })

    input_points = np.concatenate(transformed_clouds)
    input_colors = np.concatenate(color_clouds)
    fused_points, fused_colors = voxel_average(
        input_points, input_colors, voxel_m)
    output_ply = output_directory / 'fused_cloud.ply'
    write_binary_ply(output_ply, fused_points, fused_colors)

    selected_csv = output_directory / 'selected_frames.csv'
    with selected_csv.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=selected_rows[0].keys())
        writer.writeheader()
        writer.writerows(selected_rows)

    selected_yaws = [row['actual_yaw_deg'] for row in selected_rows]
    bounds = {
        axis: {
            'minimum_m': float(np.min(fused_points[:, index])),
            'maximum_m': float(np.max(fused_points[:, index])),
        }
        for index, axis in enumerate(('x', 'y', 'z'))
    }
    yaw_coverage = evaluate_yaw_coverage(
        selected_yaws[-1] - selected_yaws[0], expected_yaw_deg)
    summary = {
        'fusion_type': 'odometry_seeded_point_cloud',
        'not_slam': True,
        'input_bag': str(input_bag),
        'output_ply': str(output_ply),
        'selected_frames_csv': str(selected_csv),
        'rgbd_pair_count': len(pairs),
        'selected_frame_count': len(selections),
        'yaw_step_deg': yaw_step_deg,
        'fused_yaw_extent_deg': selected_yaws[-1] - selected_yaws[0],
        'wheel_odom_yaw_extent_deg': math.degrees(
            unwrapped_poses[-1].yaw_rad - unwrapped_poses[0].yaw_rad),
        **yaw_coverage,
        'sync_delta_mean_ms': sum(pair_deltas_ns) / len(pair_deltas_ns) / 1e6,
        'sync_delta_p95_ms': _percentile_95(pair_deltas_ns),
        'sync_delta_max_ms': max(pair_deltas_ns) / 1e6,
        'projected_point_count': int(len(input_points)),
        'voxelized_point_count': int(len(fused_points)),
        'pixel_step': pixel_step,
        'minimum_depth_m': minimum_depth_m,
        'maximum_depth_m': maximum_depth_m,
        'voxel_m': voxel_m,
        'bounds': bounds,
        'camera_extrinsic': {
            'x_m': camera_extrinsic.x_m,
            'y_m': camera_extrinsic.y_m,
            'z_m': camera_extrinsic.z_m,
            'roll_deg': math.degrees(camera_extrinsic.roll_rad),
            'pitch_deg': math.degrees(camera_extrinsic.pitch_rad),
            'yaw_deg': math.degrees(camera_extrinsic.yaw_rad),
            'measurement_status': 'cli_value_or_zero_assumption',
        },
        'limitations': [
            'Wheel odometry seeds every frame pose; this output is not SLAM.',
            'Camera-to-base extrinsics must be measured for metric alignment.',
            'No loop closure or scan-matching optimization is applied.',
        ],
    }
    summary_path = output_directory / 'fusion_summary.json'
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    return summary


def _parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--yaw-step-deg', type=float, default=5.0)
    parser.add_argument('--max-pair-delta-ms', type=float, default=30.0)
    parser.add_argument('--pixel-step', type=int, default=2)
    parser.add_argument('--minimum-depth-m', type=float, default=0.25)
    parser.add_argument('--maximum-depth-m', type=float, default=4.0)
    parser.add_argument('--voxel-m', type=float, default=0.02)
    parser.add_argument('--expected-yaw-deg', type=float)
    parser.add_argument('--camera-x-m', type=float, default=0.0)
    parser.add_argument('--camera-y-m', type=float, default=0.0)
    parser.add_argument('--camera-z-m', type=float, default=0.0)
    parser.add_argument('--camera-roll-deg', type=float, default=0.0)
    parser.add_argument('--camera-pitch-deg', type=float, default=0.0)
    parser.add_argument('--camera-yaw-deg', type=float, default=0.0)
    return parser.parse_args(argv)


def main(args=None):
    """Run wheel-odometry-seeded RGB-D point-cloud fusion."""
    parsed = _parse_args(args)
    extrinsic = CameraExtrinsic(
        x_m=parsed.camera_x_m,
        y_m=parsed.camera_y_m,
        z_m=parsed.camera_z_m,
        roll_rad=math.radians(parsed.camera_roll_deg),
        pitch_rad=math.radians(parsed.camera_pitch_deg),
        yaw_rad=math.radians(parsed.camera_yaw_deg),
    )
    summary = fuse_bag(
        input_bag=parsed.input,
        output_directory=parsed.output_dir,
        yaw_step_deg=parsed.yaw_step_deg,
        max_pair_delta_ms=parsed.max_pair_delta_ms,
        pixel_step=parsed.pixel_step,
        minimum_depth_m=parsed.minimum_depth_m,
        maximum_depth_m=parsed.maximum_depth_m,
        voxel_m=parsed.voxel_m,
        camera_extrinsic=extrinsic,
        expected_yaw_deg=parsed.expected_yaw_deg,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
