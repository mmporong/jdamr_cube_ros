#!/usr/bin/env python3
"""
Exercise installed Nav2 against synthetic scans in an isolated local domain.

No driver, planner, or physical command topic is started. This is a monitor
integration check, not a real-robot stopping-distance or avoidance trial.
"""

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, Twist
from jdamr_cube_navigation.depth_navigation_config import build_depth_navigation_params
from jdamr_cube_navigation.depth_obstacle_filter import make_cloud
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.msg import CollisionMonitorState
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage
import yaml


DOMAIN_ID = 199
PREFIX = '/collision_probe'


def run(params_path, geometry_path, output, *, with_depth=False):
    """Start one monitor child and stop that exact process in all exit paths."""
    output.mkdir(parents=True, exist_ok=False)
    document = yaml.safe_load(params_path.read_text())
    if with_depth:
        document = build_depth_navigation_params(document)
    geometry = yaml.safe_load(geometry_path.read_text())
    monitor = copy.deepcopy(document['collision_monitor']['ros__parameters'])
    monitor.update(cmd_vel_in_topic=PREFIX + '/input',
                   cmd_vel_out_topic=PREFIX + '/output',
                   state_topic=PREFIX + '/state', use_sim_time=False)
    monitor['scan']['topic'] = PREFIX + '/scan'
    if with_depth:
        monitor['depth_obstacles']['topic'] = PREFIX + '/depth_obstacles'
    monitor['FootprintApproach']['footprint_topic'] = PREFIX + '/footprint'
    monitor['StopZone']['polygon_pub_topic'] = PREFIX + '/stop_zone'
    monitor['SlowdownZone']['polygon_pub_topic'] = PREFIX + '/slowdown_zone'
    footprint = yaml.safe_load(document['local_costmap']['local_costmap'][
        'ros__parameters']['footprint'])
    laser_x, laser_y, laser_z = geometry['laser_translation']['value']
    laser_yaw = geometry['laser_yaw']['value']
    environment = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN_ID),
                       ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                       ROS_STATIC_PEERS='', FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    # The harness and the only child share these explicit discovery limits.
    os.environ.update({key: environment[key] for key in (
        'ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_STATIC_PEERS',
        'FASTDDS_BUILTIN_TRANSPORTS')})
    executable = str(Path(get_package_prefix('nav2_collision_monitor')) /
                     'lib/nav2_collision_monitor/collision_monitor')
    result = {'scope': 'synthetic_installed_monitor_only_no_physical_motion',
              'domain_id': DOMAIN_ID, 'topic_prefix': PREFIX,
              'depth_source_enabled': with_depth,
              'params_sha256': hashlib.sha256(params_path.read_bytes()).hexdigest(),
              'geometry_sha256': hashlib.sha256(geometry_path.read_bytes()).hexdigest(),
              'cases': []}
    process, node = None, None
    rclpy.init(domain_id=DOMAIN_ID)
    try:
        with tempfile.TemporaryDirectory(prefix='jdamr_collision_probe_') as temp:
            config = Path(temp) / 'params.yaml'
            config.write_text(yaml.safe_dump({PREFIX + '/collision_monitor': {
                'ros__parameters': monitor}}))
            command = [executable, '--ros-args', '--params-file', str(config),
                       '-r', '__ns:=' + PREFIX,
                       '-r', '/tf:=' + PREFIX + '/tf',
                       '-r', '/tf_static:=' + PREFIX + '/tf_static']
            with (output / 'monitor.log').open('w') as log:
                process = subprocess.Popen(command, env=environment, stdout=log,
                                           stderr=subprocess.STDOUT)
                node = rclpy.create_node('synthetic_scan_harness', namespace=PREFIX)
                scan_pub = node.create_publisher(
                    LaserScan, PREFIX + '/scan', qos_profile_sensor_data)
                depth_pub = node.create_publisher(
                    PointCloud2, PREFIX + '/depth_obstacles', qos_profile_sensor_data)
                cmd_pub = node.create_publisher(Twist, PREFIX + '/input', 10)
                tf_pub = node.create_publisher(TFMessage, PREFIX + '/tf', 10)
                static_pub = node.create_publisher(
                    TFMessage, PREFIX + '/tf_static',
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                footprint_pub = node.create_publisher(
                    PolygonStamped, PREFIX + '/footprint', 10)
                received, states, polygons = [], [], []
                subscriptions = [
                    node.create_subscription(Twist, PREFIX + '/output',
                                             lambda msg: received.append((
                                                 time.monotonic(), msg)), 10),
                    node.create_subscription(CollisionMonitorState, PREFIX + '/state',
                                             lambda msg: states.append(msg), 10),
                    node.create_subscription(PolygonStamped, PREFIX + '/stop_zone',
                                             lambda msg: polygons.append(msg), 10)]
                static_tf = TransformStamped()
                static_tf.header.frame_id, static_tf.child_frame_id = (
                    'base_footprint', 'laser_link')
                static_tf.transform.translation.x = float(laser_x)
                static_tf.transform.translation.y = float(laser_y)
                static_tf.transform.translation.z = float(laser_z)
                static_tf.transform.rotation.z = math.sin(laser_yaw / 2)
                static_tf.transform.rotation.w = math.cos(laser_yaw / 2)
                static_pub.publish(TFMessage(transforms=[static_tf]))

                lifecycle = node.create_client(
                    ChangeState, PREFIX + '/collision_monitor/change_state')
                if not lifecycle.wait_for_service(timeout_sec=10):
                    raise RuntimeError('monitor lifecycle unavailable')
                for transition in (1, 3):
                    request = ChangeState.Request()
                    request.transition.id = transition
                    future = lifecycle.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
                    if not future.done() or not future.result().success:
                        raise RuntimeError(f'lifecycle transition failed: {transition}')

                def exercise(name, velocity, points, expected, *, stale=False,
                             depth_points=(), stale_depth=False):
                    start = time.monotonic()
                    until = start + (1.6 if stale or stale_depth else 0.8)
                    while time.monotonic() < until:
                        stamp = node.get_clock().now().to_msg()
                        odom_tf = TransformStamped()
                        odom_tf.header.stamp, odom_tf.header.frame_id = stamp, 'odom'
                        odom_tf.child_frame_id = 'base_footprint'
                        odom_tf.transform.rotation.w = 1.0
                        tf_pub.publish(TFMessage(transforms=[odom_tf]))
                        shape = PolygonStamped()
                        shape.header.frame_id, shape.header.stamp = 'base_footprint', stamp
                        shape.polygon.points = [Point32(x=x, y=y) for x, y in footprint]
                        footprint_pub.publish(shape)
                        if not stale:
                            scan = LaserScan()
                            scan.header.frame_id, scan.header.stamp = 'laser_link', stamp
                            scan.angle_min, scan.angle_increment = -math.pi, math.tau / 1440
                            scan.angle_max = scan.angle_min + 1439 * scan.angle_increment
                            scan.range_min, scan.range_max = 0.28, 12.0
                            scan.ranges = [float('inf')] * 1440
                            for x, y in points:
                                dx, dy = x - laser_x, y - laser_y
                                angle = (math.atan2(dy, dx) - laser_yaw + math.pi) % math.tau
                                index = round(angle / scan.angle_increment) % 1440
                                scan.ranges[index] = math.hypot(dx, dy)
                            scan_pub.publish(scan)
                        if with_depth and not stale_depth:
                            depth_pub.publish(make_cloud(
                                list(depth_points) if depth_points else
                                np.empty((0, 3)),
                                'base_footprint', stamp))
                        command_message = Twist()
                        command_message.linear.x, command_message.angular.z = velocity
                        cmd_pub.publish(command_message)
                        rclpy.spin_once(node, timeout_sec=0.02)
                        time.sleep(0.03)
                    for _ in range(5):
                        rclpy.spin_once(node, timeout_sec=0.02)
                    fresh = [msg for stamp_s, msg in received if stamp_s >= until - 0.25]
                    state = states[-1].polygon_name if states else None
                    selected = (len(polygons[-1].polygon.points) if polygons else None)
                    passed = bool(fresh) and all(expected(msg, state, selected) for msg in fresh)
                    case = {'name': name, 'pass': passed, 'state': state,
                            'stop_polygon_vertices': selected,
                            'settled_output_count': len(fresh),
                            'last_output': ([fresh[-1].linear.x, fresh[-1].angular.z]
                                            if fresh else None)}
                    result['cases'].append(case)
                    if not passed:
                        raise RuntimeError(f'monitor case failed: {case}')

                chair = [(0.29 + offset * 0.004, -0.30) for offset in range(-2, 3)]
                exercise('clear_forward', (0.08, 0.0), [],
                         lambda cmd, _, n: abs(cmd.linear.x - 0.08) < 1e-6 and n == 4)
                exercise('chair_forward_not_stop', (0.08, 0.0), chair,
                         lambda cmd, state, n: abs(cmd.linear.x - (
                             0.08 * monitor['SlowdownZone']['slowdown_ratio'])) < 1e-6
                         and cmd.angular.z == 0 and state == 'SlowdownZone' and n == 4)
                for yaw in (0.2, -0.2):
                    exercise('chair_rotation_' + str(yaw), (0.0, yaw), chair,
                             lambda cmd, state, n: cmd.linear.x == 0
                             and cmd.angular.z == 0 and state == 'StopZone'
                             and n == 16)
                exercise('chair_stopped_not_rotation', (0.0, 0.0), chair,
                         lambda cmd, state, n: cmd.linear.x == 0 and cmd.angular.z == 0
                         and state != 'StopZone' and n == 4)
                for side in (-1, 1):
                    intrusion = [(0.12 + offset * 0.004, side * 0.265)
                                 for offset in range(-2, 3)]
                    exercise('front_intrusion_' + str(side), (0.08, 0.0), intrusion,
                             lambda cmd, state, n: cmd.linear.x == 0 and cmd.angular.z == 0
                             and state == 'StopZone' and n == 4)
                exercise('clear_before_stale', (0.08, 0.0), [],
                         lambda cmd, _, n: cmd.linear.x > 0 and n == 4)
                exercise('stale_scan_stops', (0.08, 0.0), [],
                         lambda cmd, state, _: cmd.linear.x == 0 and cmd.angular.z == 0
                         and state == 'invalid source',
                         stale=True)
                exercise('fresh_scan_resumes', (0.08, 0.0), [],
                         lambda cmd, state, n: cmd.linear.x > 0 and state != 'invalid source'
                         and n == 4)
                if with_depth:
                    depth_intrusion = [(0.12, y, 0.7) for y in (-0.08, 0.0, 0.08)]
                    exercise('depth_only_high_obstacle_stops', (0.08, 0.0), [],
                             lambda cmd, state, _: cmd.linear.x == 0
                             and cmd.angular.z == 0 and state == 'StopZone',
                             depth_points=depth_intrusion)
                    exercise('fresh_clear_depth_resumes', (0.08, 0.0), [],
                             lambda cmd, state, _: cmd.linear.x > 0
                             and state != 'invalid source')
                    exercise('stale_depth_with_fresh_lidar_stops', (0.08, 0.0), [],
                             lambda cmd, state, _: cmd.linear.x == 0
                             and cmd.angular.z == 0 and state == 'invalid source',
                             stale_depth=True)
                    exercise('fresh_depth_after_dropout_resumes', (0.08, 0.0), [],
                             lambda cmd, state, _: cmd.linear.x > 0
                             and state != 'invalid source')
                result['pass'] = True
                assert len(subscriptions) == 3
    except Exception as error:
        result.update({'pass': False, 'error': str(error)})
        raise
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    """Require an explicit artifact directory, never connect to a robot."""
    parser = argparse.ArgumentParser(description=__doc__)
    package = Path(__file__).resolve().parents[1]
    parser.add_argument('--params', type=Path,
                        default=package / 'config/new_base_nav2_params.yaml')
    parser.add_argument('--geometry', type=Path, default=package.parent /
                        'jdamr_cube_description/config/new_base_geometry.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--with-depth', action='store_true',
                        help='Also exercise the opt-in pointcloud source')
    args = parser.parse_args()
    print(json.dumps(run(args.params, args.geometry, args.output,
                         with_depth=args.with_depth), indent=2))


if __name__ == '__main__':
    main()
