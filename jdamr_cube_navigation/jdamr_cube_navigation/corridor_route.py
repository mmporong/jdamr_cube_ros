"""Preflight and execute a fail-closed saved-map corridor route."""

import argparse
import hashlib
import math
import os
from pathlib import Path
import signal
import sys
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathThroughPoses, NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import BatteryState, LaserScan
import yaml


AMCL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _expanded_path(value, parent=None):
    path = Path(os.path.expandvars(os.path.expanduser(str(value))))
    if not path.is_absolute() and parent is not None:
        path = parent / path
    return path.resolve()


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_route(route_yaml):
    """Load a route and verify the immutable map/mask artifacts it names."""
    route_yaml = route_yaml.resolve()
    with route_yaml.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or config.get('schema_version') != 1:
        raise ValueError('route schema_version must be 1')
    waypoints = config.get('waypoints')
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError('route requires at least two waypoints')
    identifiers = set()
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            raise ValueError('each waypoint must be a mapping')
        waypoint_id = str(waypoint.get('id', '')).strip()
        if not waypoint_id or waypoint_id in identifiers:
            raise ValueError('waypoint ids must be non-empty and unique')
        identifiers.add(waypoint_id)
        for field in ('x', 'y'):
            value = float(waypoint[field])
            if not math.isfinite(value):
                raise ValueError(f'{waypoint_id} {field} must be finite')
            waypoint[field] = value
        if 'yaw' in waypoint:
            waypoint['yaw'] = float(waypoint['yaw'])

    map_yaml = _expanded_path(config['map_yaml'], route_yaml.parent)
    mask_yaml = _expanded_path(
        config['keepout_mask_yaml'], route_yaml.parent)
    for label, path in (('map', map_yaml), ('keepout mask', mask_yaml)):
        if not path.is_file():
            raise FileNotFoundError(f'{label} YAML not found: {path}')
    with mask_yaml.open(encoding='utf-8') as stream:
        mask_metadata = yaml.safe_load(stream)
    mask_image = _expanded_path(mask_metadata['image'], mask_yaml.parent)
    actual_hash = _sha256(mask_image)
    expected_hash = str(config.get('expected_mask_sha256', ''))
    if not expected_hash or actual_hash != expected_hash:
        raise ValueError(
            f'keepout mask hash mismatch: expected={expected_hash} '
            f'actual={actual_hash}')
    config['route_yaml'] = route_yaml
    config['map_yaml'] = map_yaml
    config['keepout_mask_yaml'] = mask_yaml
    config['keepout_mask_image'] = mask_image
    return config


class CorridorRoute(Node):
    """Validate every leg, then send one explicit-BT goal at a time."""

    def __init__(self, config, start_index=0):
        super().__init__('jdamr_corridor_route')
        self.config = config
        self.waypoints = config['waypoints'][start_index:]
        if not self.waypoints:
            raise ValueError('start_index is past the final waypoint')
        self.minimum_battery_v = float(config.get('minimum_battery_v', 10.5))
        self.freshness_s = float(config.get('sensor_freshness_s', 2.5))
        self.stop_requested = False
        self.samples = {'battery': None, 'odom': None, 'scan': None}
        self.battery_voltage = None
        self.amcl_seen = None
        self.amcl_covariance = None
        self.navigate = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.compute = ActionClient(
            self, ComputePathThroughPoses, 'compute_path_through_poses')
        self.create_subscription(
            BatteryState, '/battery_state', self._battery_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_callback,
            AMCL_QOS)
        package_share = get_package_share_directory('jdamr_cube_navigation')
        self.behavior_tree = os.path.join(
            package_share, 'behavior_trees',
            'navigate_to_pose_corridor_fail_fast.xml')
        self._last_feedback = 0.0

    def request_stop(self):
        """Ask the current action to cancel without invalidating ROS context."""
        self.stop_requested = True

    def _battery_callback(self, message):
        self.battery_voltage = float(message.voltage)
        self.samples['battery'] = time.monotonic()

    def _odom_callback(self, _message):
        self.samples['odom'] = time.monotonic()

    def _scan_callback(self, _message):
        self.samples['scan'] = time.monotonic()

    def _amcl_callback(self, message):
        self.amcl_seen = time.monotonic()
        covariance = message.pose.covariance
        self.amcl_covariance = (float(covariance[0]), float(covariance[7]))

    def _sensors_ready(self):
        now = time.monotonic()
        fresh = all(
            timestamp is not None and now - timestamp <= self.freshness_s
            for timestamp in self.samples.values())
        return (
            fresh and self.battery_voltage is not None and
            self.battery_voltage >= self.minimum_battery_v)

    def _localization_ready(self):
        if self.amcl_seen is None or self.amcl_covariance is None:
            return False
        return (
            time.monotonic() - self.amcl_seen <= 15.0 and
            max(self.amcl_covariance) <= 0.5)

    def _navigation_ready(self):
        return self._sensors_ready() and self._localization_ready()

    def wait_until_ready(self, timeout=15.0):
        """Require current sensors, battery, and a bounded AMCL estimate."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self.stop_requested:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._navigation_ready():
                return True
        return False

    def _pose(self, index, waypoint):
        if 'yaw' in waypoint:
            yaw = waypoint['yaw']
        elif index + 1 < len(self.waypoints):
            following = self.waypoints[index + 1]
            yaw = math.atan2(
                following['y'] - waypoint['y'],
                following['x'] - waypoint['x'])
        else:
            yaw = 0.0
        pose = PoseStamped()
        pose.header.frame_id = str(self.config.get('frame_id', 'map'))
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(waypoint['x'])
        pose.pose.position.y = float(waypoint['y'])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def preflight(self):
        """Require Nav2 to plan the full remaining route before movement."""
        if not self.compute.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('compute_path_through_poses unavailable')
            return False
        goal = ComputePathThroughPoses.Goal()
        goal.goals = [
            self._pose(index, waypoint)
            for index, waypoint in enumerate(self.waypoints)
        ]
        goal.planner_id = 'GridBased'
        goal.use_start = False
        future = self.compute.send_goal_async(goal)
        while not future.done() and not self.stop_requested:
            rclpy.spin_once(self, timeout_sec=0.1)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            self.get_logger().error('route preflight goal rejected')
            return False
        result_future = handle.get_result_async()
        while not result_future.done() and not self.stop_requested:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            self._cancel(handle, 'operator interrupt during preflight')
            return False
        result = result_future.result().result
        if result.error_code != 0 or not result.path.poses:
            self.get_logger().error(
                f'route preflight failed: {result.error_code} '
                f'{result.error_msg}')
            return False
        length = sum(
            math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y)
            for previous, current in zip(
                result.path.poses, result.path.poses[1:]))
        self.get_logger().info(
            f'route preflight passed: poses={len(result.path.poses)} '
            f'length={length:.3f}m')
        return True

    def _feedback(self, route_index, message):
        now = time.monotonic()
        if now - self._last_feedback < 5.0:
            return
        self._last_feedback = now
        feedback = message.feedback
        self.get_logger().info(
            f'waypoint={route_index + 1}/{len(self.waypoints)} '
            f'remaining={feedback.distance_remaining:.2f}m '
            f'recoveries={feedback.number_of_recoveries} '
            f'battery={self.battery_voltage:.2f}V')

    def _cancel(self, handle, reason):
        self.get_logger().warning(f'cancel requested: {reason}')
        future = handle.cancel_goal_async()
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def execute(self):
        """Execute each waypoint once and stop on the first fault."""
        if not self.navigate.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose unavailable')
            return False
        for index, waypoint in enumerate(self.waypoints):
            if self.stop_requested or not self._navigation_ready():
                self.get_logger().error(
                    'sensor/battery/localization guard blocked next goal')
                return False
            goal = NavigateToPose.Goal()
            goal.pose = self._pose(index, waypoint)
            goal.behavior_tree = self.behavior_tree
            self.get_logger().info(
                f'send {index + 1}/{len(self.waypoints)} '
                f'{waypoint["id"]}=({waypoint["x"]:.2f},'
                f'{waypoint["y"]:.2f})')
            future = self.navigate.send_goal_async(
                goal,
                feedback_callback=lambda message, route_index=index:
                self._feedback(route_index, message),
            )
            while not future.done() and not self.stop_requested:
                rclpy.spin_once(self, timeout_sec=0.1)
            handle = future.result() if future.done() else None
            if handle is None or not handle.accepted:
                self.get_logger().error('navigate goal rejected')
                return False
            result_future = handle.get_result_async()
            while not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.stop_requested:
                    self._cancel(handle, 'operator interrupt')
                    return False
                if not self._navigation_ready():
                    self._cancel(
                        handle,
                        'sensor, battery, or localization guard failure')
                    return False
            wrapped = result_future.result()
            if (
                    wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                    wrapped.result.error_code != 0):
                self.get_logger().error(
                    f'navigation failed: status={wrapped.status} '
                    f'error={wrapped.result.error_code} '
                    f'{wrapped.result.error_msg}')
                return False
        self.get_logger().info('corridor roundtrip succeeded')
        return True


def parse_args(argv):
    """Parse non-ROS route arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', required=True, type=Path)
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument(
        '--execute', action='store_true',
        help='move after preflight; omitted means planning-only')
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main(args=None):
    """Verify route artifacts, preflight Nav2, and optionally drive."""
    argv = args if args is not None else sys.argv
    parsed = parse_args(argv)
    if parsed.start_index < 0:
        raise SystemExit('--start-index must be non-negative')
    try:
        config = load_route(parsed.route)
    except (KeyError, OSError, TypeError, ValueError) as error:
        print(f'route configuration error: {error}', file=sys.stderr)
        raise SystemExit(2) from error

    rclpy.init(
        args=argv,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = CorridorRoute(config, parsed.start_index)

    def request_stop(_signal_number, _frame):
        node.request_stop()

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        ready = node.wait_until_ready()
        preflighted = ready and node.preflight()
        succeeded = preflighted and (
            not parsed.execute or node.execute())
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if succeeded else 1)


if __name__ == '__main__':
    main()
