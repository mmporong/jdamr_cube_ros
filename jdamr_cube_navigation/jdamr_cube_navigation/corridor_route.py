"""Preflight and execute a fail-closed saved-map corridor route."""

import argparse
import hashlib
import json
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
NAVIGATION_BEHAVIOR_TREES = {
    'corridor': 'navigate_to_pose_corridor_fail_fast.xml',
    'obstacle_candidate': 'navigate_to_pose_dynamic_obstacle_eval.xml',
}
GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'STATUS_UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'STATUS_ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'STATUS_EXECUTING',
    GoalStatus.STATUS_CANCELING: 'STATUS_CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'STATUS_SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'STATUS_CANCELED',
    GoalStatus.STATUS_ABORTED: 'STATUS_ABORTED',
}


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
    start_pose = config.get('start_pose')
    if start_pose is not None:
        if not isinstance(start_pose, dict):
            raise ValueError('start_pose must be a mapping')
        for field in ('x', 'y'):
            value = float(start_pose[field])
            if not math.isfinite(value):
                raise ValueError(f'start_pose {field} must be finite')
            start_pose[field] = value

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


def _goal_uuid(handle):
    """Render the action goal identifier without changing its byte order."""
    return bytes(handle.goal_id.uuid).hex()


class CorridorRoute(Node):
    """Validate every leg, then send one explicit-BT goal at a time."""

    def __init__(self, config, start_index=0, navigation_profile='corridor'):
        super().__init__('jdamr_corridor_route')
        self.config = config
        self.start_index = start_index
        self.waypoints = config['waypoints'][start_index:]
        if not self.waypoints:
            raise ValueError('start_index is past the final waypoint')
        self.minimum_battery_v = float(config.get('minimum_battery_v', 10.5))
        self.freshness_s = float(config.get('sensor_freshness_s', 2.5))
        # Battery freshness and sensor freshness are separate contracts: a
        # delayed battery sample is not equivalent to driving blind.  The
        # measured rate and threshold history live in the evaluation report.
        self.battery_freshness_s = float(
            config.get('battery_freshness_s', 30.0))
        self.max_amcl_covariance = (
            float(config.get('max_amcl_x_covariance', 0.5)),
            float(config.get('max_amcl_y_covariance', 0.5)),
        )
        self.amcl_freshness_s = float(
            config.get('amcl_freshness_s', 15.0))
        self.max_resume_start_distance_m = float(
            config.get('max_resume_start_distance_m', 6.0))
        if start_index > 0:
            self.start_reference = self.waypoints[0]
            self.start_distance_limit_m = self.max_resume_start_distance_m
            self.start_check_kind = 'resume'
        else:
            self.start_reference = config.get('start_pose', self.waypoints[0])
            self.start_distance_limit_m = float(
                config.get(
                    'max_route_start_distance_m',
                    self.max_resume_start_distance_m))
            self.start_check_kind = 'route'
        self.start_check_pending = True
        self.stop_requested = False
        self.samples = {'battery': None, 'odom': None, 'scan': None}
        self.battery_voltage = None
        self.amcl_seen = None
        self.amcl_covariance = None
        self.amcl_position = None
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
            NAVIGATION_BEHAVIOR_TREES[navigation_profile])
        self.navigation_profile = navigation_profile
        self.get_logger().info(
            'navigation behavior tree selected: '
            f'profile={navigation_profile} source=navigation_profile '
            f'path={self.behavior_tree}')
        self._last_feedback = 0.0

    def _route_event(self, event, route_index, handle, **fields):
        """Emit one stable JSON line for goal lifecycle correlation."""
        record = {
            'event': event,
            'goal_uuid': _goal_uuid(handle),
            'waypoint_index': self.start_index + route_index + 1,
            'waypoint_total': len(self.config['waypoints']),
            'waypoint_id': self.waypoints[route_index]['id'],
            'navigation_profile': self.navigation_profile,
            'terminal_status': None,
            'terminal_status_code': None,
        }
        record.update(fields)
        self.get_logger().info(
            'route_event ' + json.dumps(
                record, sort_keys=True, separators=(',', ':')))

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
        self.amcl_position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def _guard_failure(self, require_fresh_amcl=True):
        """Describe the exact fail-closed input instead of a generic stop."""
        now = time.monotonic()
        for name, timestamp in self.samples.items():
            if timestamp is None:
                return f'{name} missing'
            age = now - timestamp
            # Scan and odometry going quiet means the robot is driving blind.
            # Battery only needs to be recent enough to trust the voltage.
            limit = (self.battery_freshness_s if name == 'battery'
                     else self.freshness_s)
            if age > limit:
                return (
                    f'{name} stale: age={age:.3f}s '
                    f'limit={limit:.3f}s')
        if self.battery_voltage is None:
            return 'battery voltage missing'
        if not math.isfinite(self.battery_voltage):
            return (
                'battery voltage non-finite: '
                f'value={self.battery_voltage}')
        if self.battery_voltage < self.minimum_battery_v:
            return (
                f'battery low: voltage={self.battery_voltage:.3f}V '
                f'limit={self.minimum_battery_v:.3f}V')
        if self.amcl_seen is None or self.amcl_covariance is None:
            return 'AMCL pose missing'
        amcl_age_s = now - self.amcl_seen
        if require_fresh_amcl and amcl_age_s > self.amcl_freshness_s:
            return (
                f'AMCL pose stale: age={amcl_age_s:.3f}s '
                f'limit={self.amcl_freshness_s:.3f}s')
        for axis, covariance, limit in zip(
                ('x', 'y'), self.amcl_covariance,
                self.max_amcl_covariance):
            if not math.isfinite(covariance):
                return (
                    f'AMCL {axis} covariance non-finite: '
                    f'value={covariance}')
            if covariance > limit:
                return (
                    f'AMCL {axis} covariance high: value={covariance:.3f} '
                    f'limit={limit:.3f}')
        if self.start_check_pending:
            if self.amcl_position is None:
                return f'AMCL position missing for {self.start_check_kind} start'
            for axis, position in zip(('x', 'y'), self.amcl_position):
                if not math.isfinite(position):
                    return (
                        f'AMCL {self.start_check_kind} start {axis} '
                        f'non-finite: value={position}')
            distance_m = math.hypot(
                self.amcl_position[0] - self.start_reference['x'],
                self.amcl_position[1] - self.start_reference['y'],
            )
            if distance_m > self.start_distance_limit_m:
                return (
                    f'AMCL {self.start_check_kind} start too far: '
                    f'distance={distance_m:.3f}m '
                    f'limit={self.start_distance_limit_m:.3f}m')
        return None

    def _navigation_ready(self, require_fresh_amcl=True):
        return self._guard_failure(require_fresh_amcl) is None

    def wait_until_ready(self, timeout=15.0):
        """Require current sensors, battery, and a bounded AMCL estimate."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self.stop_requested:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._navigation_ready():
                self.start_check_pending = False
                return True
        self.get_logger().error(
            'navigation readiness timeout: '
            f'{self._guard_failure() or "operator stop"}')
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

    def _cancel(self, handle, reason, route_index=None):
        self.get_logger().warning(f'cancel requested: {reason}')
        if route_index is not None:
            self._route_event(
                'cancel_requested', route_index, handle, reason=reason)
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
        # wait_until_ready() already proved that localization produced a
        # current pose.  AMCL's pose topic may remain quiet while a correctly
        # localized robot is stationary.  Runtime
        # safety continues to require fresh scan/odom, valid battery data, and
        # bounded covariance, but silence on /amcl_pose alone is not a fault.
        for index, waypoint in enumerate(self.waypoints):
            if (
                    self.stop_requested or
                    not self._navigation_ready(require_fresh_amcl=False)):
                self.get_logger().error(
                    'navigation guard blocked next goal: '
                    f'{self._guard_failure(False) or "operator stop"}')
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
            self._route_event('accepted', index, handle)
            result_future = handle.get_result_async()
            while not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.stop_requested:
                    self._cancel(handle, 'operator interrupt', index)
                    return False
                if not self._navigation_ready(require_fresh_amcl=False):
                    self._cancel(
                        handle,
                        self._guard_failure(False) or
                        'navigation guard failure',
                        index)
                    return False
            wrapped = result_future.result()
            self._route_event(
                'result', index, handle,
                terminal_status=GOAL_STATUS_NAMES.get(
                    wrapped.status, 'STATUS_UNKNOWN'),
                terminal_status_code=int(wrapped.status),
                nav2_error_code=int(wrapped.result.error_code))
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
        '--navigation-profile',
        choices=tuple(NAVIGATION_BEHAVIOR_TREES),
        default='corridor')
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
    node = CorridorRoute(
        config, parsed.start_index, parsed.navigation_profile)

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
