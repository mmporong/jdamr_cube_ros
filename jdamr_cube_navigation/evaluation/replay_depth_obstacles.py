#!/usr/bin/env python3
"""Replay recorded depth frames through the obstacle projection core."""

import argparse
from collections import Counter, defaultdict
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

from jdamr_cube_navigation.depth_obstacle_core import (
    DepthObstacleConfig,
    project_depth,
)

import numpy as np

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BAG = Path.home() / 'jdamr_data/vslam/rgbd_20260917T172715/bag'
DEFAULT_OUTPUT = (
    Path.home() / 'jdamr_artifacts/depth_obstacles_20260918/replay_summary.json')
DEFAULT_GEOMETRY = (
    REPOSITORY_ROOT
    / 'jdamr_cube_description/config/new_base_geometry.yaml'
)
DEFAULT_MOUNT = (
    REPOSITORY_ROOT / 'jdamr_cube_vslam/config/camera_mount.yaml'
)
DEPTH_TOPIC = '/camera/depth/image_raw'
INFO_TOPIC = '/camera/depth/camera_info'
OPTICAL_FRAME = 'camera_color_optical_frame'
CALIBRATION_MODEL = 'rectified_projection'


def _runtime_dependencies():
    """Load ROS-only dependencies lazily so pure helpers remain testable."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    from jdamr_cube_navigation.depth_obstacle_filter import (
        body_bounds_from_geometry,
        camera_intrinsics,
        transform_matrix,
    )
    return SimpleNamespace(
        rosbag2_py=rosbag2_py,
        deserialize_message=deserialize_message,
        get_message=get_message,
        body_bounds_from_geometry=body_bounds_from_geometry,
        camera_intrinsics=camera_intrinsics,
        transform_matrix=transform_matrix,
    )


def _iter_messages(bag_path, runtime):
    """Yield deserialized messages from one fresh sequential bag reader."""
    reader = runtime.rosbag2_py.SequentialReader()
    reader.open(
        runtime.rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id='mcap'),
        runtime.rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        yield (
            topic,
            runtime.deserialize_message(
                serialized, runtime.get_message(topic_types[topic])),
            timestamp_ns,
        )


def _collect_context(bag_path, runtime):
    """Collect calibration and candidate invariant transforms."""
    camera_info = None
    info_count = 0
    depth_count = 0
    edges = defaultdict(list)
    for topic, message, _ in _iter_messages(bag_path, runtime):
        if topic == INFO_TOPIC:
            info_count += 1
            if camera_info is None:
                camera_info = message
        elif topic == DEPTH_TOPIC:
            depth_count += 1
        elif topic in {'/tf', '/tf_static'}:
            for stamped in message.transforms:
                matrix = runtime.transform_matrix(stamped.transform)
                edges[stamped.child_frame_id].append((
                    stamped.header.frame_id, matrix, topic))
    if camera_info is None:
        raise ValueError(f'missing required camera info topic: {INFO_TOPIC}')
    if depth_count == 0:
        raise ValueError(f'missing required depth topic: {DEPTH_TOPIC}')
    return camera_info, info_count, depth_count, edges


def _resolve_invariant_chain(edges, source_frame, target_frame):
    """Resolve equivalent upward paths whose recorded values never change."""
    grouped_edges = {}
    for child, observations in edges.items():
        grouped = defaultdict(list)
        for parent, matrix, topic in observations:
            grouped[parent].append((matrix, topic))
        grouped_edges[child] = grouped

    def invariant_parents(child):
        result = []
        for parent, parent_observations in grouped_edges.get(
                child, {}).items():
            reference = parent_observations[0][0]
            if any(not np.allclose(
                    reference, matrix, rtol=0.0, atol=1e-8)
                   for matrix, _ in parent_observations[1:]):
                raise ValueError(
                    f'recorded transform is not static for {child}')
            result.append((
                parent,
                reference,
                {topic for _, topic in parent_observations},
            ))
        return result

    paths = []

    def visit(current, transform, frames, topics):
        if current == target_frame:
            paths.append((transform, frames, topics))
            return
        if current in frames[:-1]:
            return
        for parent, matrix, edge_topics in invariant_parents(current):
            visit(
                parent,
                matrix @ transform,
                frames + [parent],
                topics | edge_topics,
            )

    visit(source_frame, np.eye(4), [source_frame], set())
    if not paths:
        raise ValueError(
            f'missing recorded transform from {source_frame} toward '
            f'{target_frame}')
    reference = paths[0][0]
    if any(not np.allclose(reference, path[0], rtol=0.0, atol=1e-8)
           for path in paths[1:]):
        raise ValueError(
            f'ambiguous recorded transform paths from {source_frame} '
            f'to {target_frame}')
    _, frames, topics = min(paths, key=lambda item: (len(item[1]), item[1]))
    return reference, frames, sorted(topics)


def _quaternion_from_rpy(roll, pitch, yaw):
    """Convert fixed-axis roll, pitch, yaw to an XYZW quaternion."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return SimpleNamespace(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def _load_measured_mount(path, runtime):
    """Load the measured base-to-camera mount through the node TF helper."""
    document = yaml.safe_load(Path(path).expanduser().read_text())
    mount = document.get('camera_mount', {})
    if mount.get('status') != 'measured':
        raise ValueError('camera mount status must be measured')
    if (mount.get('parent_frame') != 'base_link'
            or mount.get('child_frame') != 'camera_link'):
        raise ValueError(
            'camera mount must describe base_link to camera_link')
    values = mount.get('transform', {})
    names = ('x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad')
    try:
        x, y, z, roll, pitch, yaw = [float(values[name]) for name in names]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('camera mount transform is incomplete') from error
    if not np.all(np.isfinite((x, y, z, roll, pitch, yaw))):
        raise ValueError('camera mount transform must be finite')
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y, z=z),
        rotation=_quaternion_from_rpy(roll, pitch, yaw),
    )
    return runtime.transform_matrix(transform), document


def _selected_depth_indices(total_frames, frame_stride, max_frames):
    """Span the recording after fixed-stride sampling and a uniform cap."""
    candidates = np.arange(0, total_frames, frame_stride, dtype=np.int64)
    if len(candidates) <= max_frames:
        return set(candidates.tolist())
    positions = np.linspace(0, len(candidates) - 1, max_frames)
    selected = candidates[np.rint(positions).astype(np.int64)]
    return set(selected.tolist())


def _statistics(values):
    """Return JSON-safe aggregate statistics for one numeric series."""
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0:
        return {'mean': None, 'p95': None, 'max': None}
    return {
        'mean': float(np.mean(array)),
        'p95': float(np.percentile(array, 95)),
        'max': float(np.max(array)),
    }


def evaluate_bag(
        bag_path, output_path, max_frames=100, frame_stride=15,
        geometry_path=DEFAULT_GEOMETRY, mount_path=DEFAULT_MOUNT,
        config=DepthObstacleConfig()):
    """Run a bounded, non-publishing replay and write its JSON summary."""
    if max_frames <= 0 or frame_stride <= 0:
        raise ValueError('max_frames and frame_stride must be positive')
    bag_path = Path(bag_path).expanduser().resolve()
    output_path = Path(output_path).expanduser().resolve()
    geometry_path = Path(geometry_path).expanduser().resolve()
    mount_path = Path(mount_path).expanduser().resolve()
    runtime = _runtime_dependencies()
    camera_info, info_count, depth_count, edges = _collect_context(
        bag_path, runtime)
    camera_to_optical, recorded_chain, tf_topics = (
        _resolve_invariant_chain(
            edges, OPTICAL_FRAME, 'camera_link'))
    base_to_camera, mount_document = _load_measured_mount(
        mount_path, runtime)
    base_to_optical = base_to_camera @ camera_to_optical
    body_bounds = runtime.body_bounds_from_geometry(geometry_path)
    selected_indices = _selected_depth_indices(
        depth_count, frame_stride, max_frames)

    statuses = Counter()
    processing_ms = []
    valid_fractions = []
    ray_points = []
    obstacle_points = []
    selected_timestamps_ns = []
    depth_index = -1
    for topic, image, timestamp_ns in _iter_messages(bag_path, runtime):
        if topic != DEPTH_TOPIC:
            continue
        depth_index += 1
        if depth_index not in selected_indices:
            continue
        started = time.perf_counter_ns()
        try:
            intrinsics = runtime.camera_intrinsics(
                camera_info, image, OPTICAL_FRAME, CALIBRATION_MODEL)
            result = project_depth(
                image.data, image.width, image.height, image.step,
                image.encoding, image.is_bigendian, intrinsics,
                base_to_optical, body_bounds, config)
        except (TypeError, ValueError) as error:
            statuses[f'input_error:{error}'] += 1
            processing_ms.append(
                (time.perf_counter_ns() - started) / 1e6)
            selected_timestamps_ns.append(int(timestamp_ns))
            continue
        processing_ms.append((time.perf_counter_ns() - started) / 1e6)
        selected_timestamps_ns.append(int(timestamp_ns))
        statuses[result.status] += 1
        valid_fractions.append(result.valid_fraction)
        ray_points.append(len(result.rays_xyz))
        obstacle_points.append(len(result.obstacles_xyz))

    selected_count = len(processing_ms)
    healthy_count = statuses.get('healthy', 0)
    summary = {
        'schema_version': 1,
        'evaluation': 'offline_depth_obstacle_replay',
        'sources': {
            'bag': str(bag_path),
            'geometry': str(geometry_path),
            'camera_mount': str(mount_path),
            'depth_topic': DEPTH_TOPIC,
            'camera_info_topic': INFO_TOPIC,
            'recorded_tf_topics': tf_topics,
        },
        'mount_method': (
            'measured_config_mount_plus_recorded_static_tf'),
        'recorded_sensor_chain': recorded_chain,
        'mount_status': mount_document['camera_mount']['status'],
        'limitations': [
            'not_validated_sensor_alignment',
            'not_driving_or_slam_proof',
            ('camera_driver_fixed_transforms_were_recorded_on_tf_topic_'
             'rather_than_tf_static'),
        ],
        'selection': {
            'method': 'fixed_stride_then_uniform_cap_across_recording',
            'max_frames': int(max_frames),
            'frame_stride': int(frame_stride),
            'depth_messages': int(depth_count),
            'camera_info_messages': int(info_count),
            'selected_frames': selected_count,
            'first_selected_timestamp_ns': (
                selected_timestamps_ns[0]
                if selected_timestamps_ns else None),
            'last_selected_timestamp_ns': (
                selected_timestamps_ns[-1]
                if selected_timestamps_ns else None),
        },
        'config': asdict(config),
        'counts': {
            'valid_cloud_frames': int(healthy_count),
            'invalid_cloud_frames': int(selected_count - healthy_count),
            'status': dict(sorted(statuses.items())),
        },
        'frame_stats': {
            'valid_fraction': _statistics(valid_fractions),
            'ray_points': _statistics(ray_points),
            'obstacle_points': _statistics(obstacle_points),
        },
        'processing_ms': _statistics(processing_ms),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    return summary


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=DEFAULT_BAG)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--max-frames', type=int, default=100)
    parser.add_argument('--frame-stride', type=int, default=15)
    parser.add_argument('--geometry', type=Path, default=DEFAULT_GEOMETRY)
    parser.add_argument('--mount', type=Path, default=DEFAULT_MOUNT)
    return parser


def main(argv=None):
    """Run the command-line offline evaluator."""
    arguments = _parser().parse_args(argv)
    summary = evaluate_bag(
        arguments.bag, arguments.output,
        max_frames=arguments.max_frames,
        frame_stride=arguments.frame_stride,
        geometry_path=arguments.geometry,
        mount_path=arguments.mount,
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
