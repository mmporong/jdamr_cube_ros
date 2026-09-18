#!/usr/bin/env python3
"""Isolated runtime smoke test for the installed Nav2 depth voxel layer."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix

from geometry_msgs.msg import TransformStamped

from jdamr_cube_navigation.depth_navigation_config import (
    build_depth_navigation_params,
)

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState

from nav2_msgs.srv import GetCostmap

import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from sensor_msgs.msg import LaserScan, PointCloud2, PointField

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import yaml


DOMAIN_ID = '197'
TOPIC_PREFIX = '/depth_voxel_probe'
NODE_NAME = 'voxel_probe_costmap'
ARTIFACT_DIR = (
    Path.home() / 'jdamr_artifacts/depth_obstacles_20260918'
    / 'voxel_smoke_reviewed4')
DEFAULT_OUTPUT = ARTIFACT_DIR / 'summary.json'
BASE_PARAMS = (
    Path(__file__).resolve().parents[1]
    / 'config' / 'new_base_nav2_params.yaml'
)
NAV2_EXECUTABLE = (
    Path(get_package_prefix('nav2_costmap_2d'))
    / 'lib/nav2_costmap_2d/nav2_costmap_2d')
TARGET_X_M = 1.025
TARGET_Y_M = 0.0
TARGET_Z_M = 0.75
RAY_ENDPOINT_X_M = 1.425
RAY_ENDPOINT_Z_M = TARGET_Z_M * RAY_ENDPOINT_X_M / TARGET_X_M
LETHAL_COST = 254
PROGRESS = {'stages': []}


def _effective_parameters():
    """Build the real overlay, then isolate its local costmap consumers."""
    base = yaml.safe_load(BASE_PARAMS.read_text(encoding='utf-8'))
    overlay = build_depth_navigation_params(base)
    params = overlay['local_costmap']['local_costmap']['ros__parameters']
    params['plugins'] = ['obstacle_layer', 'depth_obstacle_layer']
    # An empty YAML sequence has no ROS parameter type. Omitting the optional
    # key uses Costmap2DROS's empty-filter default without loading a filter.
    params.pop('filters', None)
    params['update_frequency'] = 10.0
    params['publish_frequency'] = 5.0
    params['always_send_full_costmap'] = True
    params['obstacle_layer']['scan']['topic'] = f'{TOPIC_PREFIX}/scan'
    params['obstacle_layer']['scan']['inf_is_valid'] = True
    depth = params['depth_obstacle_layer']
    depth['depth_marks']['topic'] = f'{TOPIC_PREFIX}/depth_marks'
    depth['depth_rays']['topic'] = f'{TOPIC_PREFIX}/depth_rays'
    return {NODE_NAME: {'ros__parameters': params}}


def _point_cloud(node, points):
    """Create a little-endian XYZ cloud in the synthetic camera frame."""
    message = PointCloud2()
    message.header.frame_id = 'camera_color_optical_frame'
    message.header.stamp = node.get_clock().now().to_msg()
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(
            name=name, offset=index * 4,
            datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(('x', 'y', 'z'))
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = b''.join(struct.pack('<fff', *point) for point in points)
    return message


def _free_scan(node):
    """Create one low planar LiDAR clearing ray along positive X."""
    message = LaserScan()
    message.header.frame_id = 'base_footprint'
    message.header.stamp = node.get_clock().now().to_msg()
    message.angle_min = 0.0
    message.angle_max = 0.0
    message.angle_increment = 0.01
    message.scan_time = 0.1
    message.range_min = 0.05
    message.range_max = 3.0
    message.ranges = [math.inf]
    return message


def _publish_static_frames(node):
    """Publish identity odom/base/camera transforms for the isolated probe."""
    broadcaster = StaticTransformBroadcaster(node)
    stamp = node.get_clock().now().to_msg()
    transforms = []
    for parent, child in (
            ('odom', 'base_footprint'),
            ('base_footprint', 'camera_color_optical_frame')):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    broadcaster.sendTransform(transforms)
    return broadcaster


def _wait_for_service(node, client, process, timeout_s, name):
    """Wait boundedly while also detecting an early child exit."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                f'Nav2 exited before {name}; code={process.returncode}')
        if client.wait_for_service(timeout_sec=0.2):
            return
        rclpy.spin_once(node, timeout_sec=0.0)
    raise TimeoutError(f'timed out waiting for {name}')


def _discover_costmap_service(node, process, timeout_s):
    """Discover the standalone binary's GetCostmap name in our domain."""
    deadline = time.monotonic() + timeout_s
    service_type = 'nav2_msgs/srv/GetCostmap'
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                'Nav2 exited before GetCostmap discovery; '
                f'code={process.returncode}')
        matches = [
            name for name, types in node.get_service_names_and_types()
            if service_type in types
        ]
        master_matches = [
            name for name in matches
            if name.rsplit('/', 1)[-1] == 'get_costmap'
        ]
        if len(master_matches) == 1:
            client = node.create_client(GetCostmap, master_matches[0])
            if client.wait_for_service(timeout_sec=0.5):
                return client, master_matches[0]
        elif len(master_matches) > 1:
            raise RuntimeError(
                'ambiguous master GetCostmap services in isolated domain: '
                f'{master_matches}')
        rclpy.spin_once(node, timeout_sec=0.1)
    services = [name for name, _ in node.get_service_names_and_types()]
    raise TimeoutError(
        f'timed out discovering GetCostmap; visible services={services}')


def _call(node, client, request, timeout_s, name):
    """Call one ROS service with a strict wall-clock timeout."""
    future = client.call_async(request)
    rclpy.spin_until_future_complete(
        node, future, timeout_sec=timeout_s)
    if not future.done():
        raise TimeoutError(f'timed out calling {name}')
    error = future.exception()
    if error is not None:
        raise RuntimeError(f'{name} failed: {error}')
    return future.result()


def _transition(node, client, transition_id, name):
    """Apply and verify one lifecycle transition."""
    request = ChangeState.Request()
    request.transition.id = transition_id
    response = _call(node, client, request, 8.0, name)
    if not response.success:
        raise RuntimeError(f'lifecycle transition failed: {name}')


def _burst(node, publishers, messages, duration_s=0.8):
    """Publish a short bounded burst so each layer receives an observation."""
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        for publisher, factory in zip(publishers, messages):
            publisher.publish(factory())
        rclpy.spin_once(node, timeout_sec=0.03)
        time.sleep(0.04)


def _wait_for_subscribers(node, publishers, timeout_s):
    """Prove DDS connections exist before publishing one-shot evidence."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        counts = [item.get_subscription_count() for item in publishers]
        if all(count > 0 for count in counts):
            return counts
        rclpy.spin_once(node, timeout_sec=0.1)
    raise TimeoutError(f'timed out waiting for subscribers; counts={counts}')


def _cost_at(costmap, x_m, y_m):
    """Read the target neighborhood despite float32 metadata rounding."""
    metadata = costmap.metadata
    column = math.floor((x_m - metadata.origin.position.x)
                        / metadata.resolution)
    row = math.floor((y_m - metadata.origin.position.y)
                     / metadata.resolution)
    if not 0 <= column < metadata.size_x or not 0 <= row < metadata.size_y:
        raise RuntimeError(
            f'target outside costmap: cell=({column}, {row}), '
            f'size=({metadata.size_x}, {metadata.size_y})')
    costs = []
    for candidate_row in range(max(0, row - 1),
                               min(metadata.size_y, row + 2)):
        for candidate_column in range(max(0, column - 1),
                                      min(metadata.size_x, column + 2)):
            costs.append(int(costmap.data[
                candidate_row * metadata.size_x + candidate_column]))
    return max(costs)


def _wait_for_cost(node, client, predicate, timeout_s, label):
    """Poll the real GetCostmap service until the target cell matches."""
    deadline = time.monotonic() + timeout_s
    observed = []
    while time.monotonic() < deadline:
        response = _call(
            node, client, GetCostmap.Request(), 2.0, 'get_costmap')
        cost = _cost_at(response.map, TARGET_X_M, TARGET_Y_M)
        observed.append(cost)
        if predicate(cost):
            return cost, observed
        time.sleep(0.1)
    raise AssertionError(f'{label} failed; observed costs={observed}')


def _installed_version():
    """Read the Debian package version for the executed Nav2 plugin."""
    completed = subprocess.run(
        ['dpkg-query', '-W', '-f=${Version}',
         'ros-jazzy-nav2-costmap-2d'],
        check=False, capture_output=True, text=True, timeout=3.0)
    return completed.stdout.strip() or 'unknown'


def _stop_child(process):
    """Stop only the exact Nav2 child created by this probe."""
    if process.poll() is not None:
        return process.returncode
    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.wait(timeout=2.0)


def run_smoke(log_stream):
    """Start one isolated lifecycle costmap and execute four assertions."""
    parameters = _effective_parameters()
    descriptor, parameter_name = tempfile.mkstemp(
        prefix='depth_voxel_probe_', suffix='.yaml')
    os.close(descriptor)
    parameter_path = Path(parameter_name)
    parameter_path.write_text(
        yaml.safe_dump(parameters, sort_keys=False), encoding='utf-8')
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = DOMAIN_ID
    environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    environment['ROS_STATIC_PEERS'] = ''
    process = subprocess.Popen(
        [
            str(NAV2_EXECUTABLE),
            '--ros-args',
            '--remap', f'__node:={NODE_NAME}',
            '--params-file', str(parameter_path),
        ],
        env=environment,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    node = None
    lifecycle = None
    lifecycle_state = 'unconfigured'
    child_exit_code = None
    try:
        rclpy.init()
        node = rclpy.create_node('depth_voxel_probe_driver')
        broadcaster = _publish_static_frames(node)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        mark_publisher = node.create_publisher(
            PointCloud2, f'{TOPIC_PREFIX}/depth_marks', sensor_qos)
        ray_publisher = node.create_publisher(
            PointCloud2, f'{TOPIC_PREFIX}/depth_rays', sensor_qos)
        scan_publisher = node.create_publisher(
            LaserScan, f'{TOPIC_PREFIX}/scan', sensor_qos)
        lifecycle = node.create_client(
            ChangeState, f'/{NODE_NAME}/change_state')
        _wait_for_service(
            node, lifecycle, process, 8.0, 'lifecycle service')
        PROGRESS['stages'].append('lifecycle_service_ready')
        _transition(
            node, lifecycle, Transition.TRANSITION_CONFIGURE, 'configure')
        lifecycle_state = 'inactive'
        PROGRESS['stages'].append('configured_plugins')
        _transition(
            node, lifecycle, Transition.TRANSITION_ACTIVATE, 'activate')
        lifecycle_state = 'active'
        PROGRESS['stages'].append('activated_costmap')
        get_costmap, get_costmap_name = _discover_costmap_service(
            node, process, 5.0)
        PROGRESS['stages'].append('get_costmap_service_ready')
        subscription_counts = _wait_for_subscribers(
            node, [mark_publisher, ray_publisher, scan_publisher], 4.0)
        PROGRESS['stages'].append('all_source_subscribers_ready')

        def empty_cloud():
            return _point_cloud(node, [])

        def elevated_mark():
            return _point_cloud(
                node, [(TARGET_X_M, TARGET_Y_M, TARGET_Z_M)])

        def floor_ray():
            return _point_cloud(node, [(RAY_ENDPOINT_X_M, 0.0, 0.0)])

        def high_ray():
            return _point_cloud(
                node, [(RAY_ENDPOINT_X_M, 0.0, RAY_ENDPOINT_Z_M)])

        def free_scan():
            return _free_scan(node)

        _burst(
            node, [mark_publisher, ray_publisher],
            [elevated_mark, empty_cloud])
        marked, marked_observed = _wait_for_cost(
            node, get_costmap, lambda value: value == LETHAL_COST,
            4.0, 'elevated depth marking')
        PROGRESS['stages'].append('elevated_mark_confirmed')

        _burst(
            node, [mark_publisher, scan_publisher],
            [empty_cloud, free_scan])
        after_lidar, lidar_observed = _wait_for_cost(
            node, get_costmap, lambda value: value == LETHAL_COST,
            3.0, 'low LiDAR preservation')

        _burst(
            node, [mark_publisher, ray_publisher],
            [empty_cloud, floor_ray])
        after_floor, floor_observed = _wait_for_cost(
            node, get_costmap, lambda value: value == LETHAL_COST,
            3.0, 'floor ray preservation')

        _burst(
            node, [mark_publisher, ray_publisher],
            [empty_cloud, high_ray])
        after_high, high_observed = _wait_for_cost(
            node, get_costmap, lambda value: value == 0,
            4.0, '3D ray clearing')
        del broadcaster
        layer = parameters[NODE_NAME]['ros__parameters'][
            'depth_obstacle_layer']
        return {
            'status': 'pass',
            'nav2_costmap_2d_version': _installed_version(),
            'ros_domain_id': int(DOMAIN_ID),
            'discovery_range': 'LOCALHOST',
            'topic_prefix': TOPIC_PREFIX,
            'get_costmap_service': get_costmap_name,
            'source_subscription_counts': subscription_counts,
            'real_sensor_or_motion_commands_used': False,
            'limitations': [
                'synthetic_identity_camera_frame_not_real_optical_alignment',
                'isolated_costmap_layer_smoke_not_driving_proof',
            ],
            'effective_geometry': {
                key: parameters[NODE_NAME]['ros__parameters'][key]
                for key in ('footprint', 'width', 'height', 'resolution')
            },
            'plugins': parameters[NODE_NAME]['ros__parameters']['plugins'],
            'observation_sources': {
                'lidar': parameters[NODE_NAME]['ros__parameters'][
                    'obstacle_layer']['observation_sources'],
                'depth': layer['observation_sources'],
                'depth_marks': layer['depth_marks'],
                'depth_rays': layer['depth_rays'],
            },
            'cases': {
                'elevated_depth_mark': {
                    'pass': marked == LETHAL_COST,
                    'cost': marked,
                    'observed': marked_observed,
                },
                'low_lidar_free_ray_preserves_high_voxel': {
                    'pass': after_lidar == LETHAL_COST,
                    'cost': after_lidar,
                    'observed': lidar_observed,
                },
                'floor_depth_ray_preserves_high_voxel': {
                    'pass': after_floor == LETHAL_COST,
                    'cost': after_floor,
                    'observed': floor_observed,
                },
                'three_dimensional_ray_clears_high_voxel': {
                    'pass': after_high == 0,
                    'cost': after_high,
                    'observed': high_observed,
                },
            },
        }
    finally:
        if (node is not None and lifecycle is not None
                and process.poll() is None):
            try:
                if lifecycle_state == 'active':
                    _transition(
                        node, lifecycle, Transition.TRANSITION_DEACTIVATE,
                        'deactivate')
                    lifecycle_state = 'inactive'
                    PROGRESS['stages'].append('deactivated_costmap')
                if lifecycle_state == 'inactive':
                    _transition(
                        node, lifecycle, Transition.TRANSITION_CLEANUP,
                        'cleanup')
                    PROGRESS['stages'].append('cleaned_costmap')
            except Exception as error:
                PROGRESS['lifecycle_cleanup_error'] = str(error)
        child_exit_code = _stop_child(process)
        PROGRESS['child_cleanup_exit_code'] = child_exit_code
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        parameter_path.unlink(missing_ok=True)
        log_stream.flush()
        if child_exit_code not in (0, -signal.SIGINT):
            log_stream.write(
                f'\nprobe cleanup child exit code: {child_exit_code}\n')


def main(argv=None):
    """Run once, preserve evidence, and return a shell-friendly verdict."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    summary_path = arguments.output.expanduser().resolve()
    log_path = summary_path.parent / 'nav2_costmap.log'
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    if summary_path.exists() or log_path.exists():
        raise FileExistsError(
            'refusing to overwrite smoke evidence under '
            f'{summary_path.parent}')
    started = time.time()
    environment_before = {
        'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID'),
        'ROS_AUTOMATIC_DISCOVERY_RANGE': os.environ.get(
            'ROS_AUTOMATIC_DISCOVERY_RANGE'),
        'ROS_STATIC_PEERS': os.environ.get('ROS_STATIC_PEERS'),
    }
    os.environ['ROS_DOMAIN_ID'] = DOMAIN_ID
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_STATIC_PEERS'] = ''
    PROGRESS['stages'] = []
    PROGRESS.pop('child_cleanup_exit_code', None)
    try:
        with log_path.open('x', encoding='utf-8') as log_stream:
            report = run_smoke(log_stream)
    except Exception as error:
        report = {
            'status': 'blocked_or_failed',
            'error_type': type(error).__name__,
            'error': str(error),
            'nav2_costmap_2d_version': _installed_version(),
            'real_sensor_or_motion_commands_used': False,
        }
    finally:
        for name, value in environment_before.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value
    report['duration_s'] = time.time() - started
    report['log'] = str(log_path)
    report['progress'] = PROGRESS.copy()
    cleanup_code = PROGRESS.get('child_cleanup_exit_code')
    report['cleanup'] = {
        'lifecycle_deactivate_and_cleanup_completed': all(
            stage in PROGRESS['stages']
            for stage in ('deactivated_costmap', 'cleaned_costmap')),
        'child_exit_code_after_sigint': cleanup_code,
        'anomaly': cleanup_code not in (0, -signal.SIGINT),
    }
    if report['cleanup']['anomaly']:
        report.setdefault('limitations', []).append(
            'standalone_nav2_process_exited_minus_11_after_clean_lifecycle')
    summary_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report['status'] == 'pass' else 1


if __name__ == '__main__':
    raise SystemExit(main())
