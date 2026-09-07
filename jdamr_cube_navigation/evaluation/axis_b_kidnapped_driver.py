#!/usr/bin/env python3
"""Create an evaluation-only kidnapped-robot trace in Gazebo."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import subprocess
import tempfile
import time

from amcl_fault_contract import canonical_json_bytes, sha256_file
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


START_POSE = [-8.0, 0.0, 0.0]
TELEPORT_POSE = [0.0, 0.0, math.pi]
MAP_RESOLUTION_M_PER_CELL = 0.05
LIDAR_BEAM_RAD = math.radians(1.0)
GT_TRANSLATION_TOLERANCE_M = 2.0 * MAP_RESOLUTION_M_PER_CELL
GT_YAW_TOLERANCE_RAD = LIDAR_BEAM_RAD
ODOM_CONTINUITY_TOLERANCE_M = 0.10
STATIONARY_SPEED_MPS = 0.01
ROTATION_TARGET_RAD = 2.0 * math.pi
INITIAL_OBSERVATION_RAD = 0.8
POST_ZERO_OBSERVATION_RAD = 1.2
ROTATION_CRUISE_RADPS = 0.5
ROTATION_SLOW_ZONE_RAD = 0.20
ROTATION_MIN_RADPS = 0.05
ZERO_HOLD_S = 1.0


def _yaw(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z +
               orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))


def _angle_delta(left: float, right: float) -> float:
    return math.remainder(left - right, 2.0 * math.pi)


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def rotation_command_radps(accumulated_yaw_rad: float) -> float:
    """Reduce angular speed near one full observed rotation."""
    remaining_yaw_rad = ROTATION_TARGET_RAD - accumulated_yaw_rad
    if remaining_yaw_rad <= 0.0:
        return 0.0
    if remaining_yaw_rad >= ROTATION_SLOW_ZONE_RAD:
        return ROTATION_CRUISE_RADPS
    return max(ROTATION_MIN_RADPS, remaining_yaw_rad)


def _finite_number(value) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _pose3(value) -> bool:
    return (type(value) is list and len(value) == 3 and
            all(_finite_number(item) for item in value))


def _source_identity() -> dict:
    source_path = Path(__file__).resolve()
    return {'path': str(source_path), 'size_bytes': source_path.stat().st_size,
            'sha256': sha256_file(source_path)}


def validate_kidnapped_trace(
        value: dict, expected_driver_source: dict | None = None) -> dict:
    """Fail closed on every causal and physical trace condition."""
    expected = {
        'schema_version', 'events', 'pre_teleport_gt', 'post_teleport_gt',
        'pre_teleport_odom', 'post_teleport_odom', 't0_scan_stamp_ns',
        'teleport_gt_verified_header_stamp_ns',
        'unwrapped_observation_yaw_rad', 'final_zero',
        'reverse_observation_yaw_rad',
        'initial_observation_yaw_rad', 'post_zero_observation_yaw_rad',
        'initial_reverse_yaw_rad', 'post_zero_reverse_yaw_rad',
        'zero_hold_s', 'stationary_sample_count',
        'post_teleport_initialpose_count', 'failure', 'driver_source'}
    if type(value) is not dict or set(value) != expected:
        raise ValueError('kidnapped trace schema drift')
    if value['schema_version'] != 1 or value['failure'] is not None:
        raise ValueError('kidnapped trace failed')
    expected_source = expected_driver_source or _source_identity()
    if value['driver_source'] != expected_source:
        raise ValueError('kidnapped driver source identity drift')
    events = value['events']
    if type(events) is not list:
        raise ValueError('kidnapped event list drift')
    required = ['initial_observation_complete', 'stationary_confirmed',
                'teleport_requested',
                'teleport_ack', 'teleport_gt_verified', 't0_scan',
                'rotation_complete', 'zero_hold_complete',
                'post_zero_observation_complete',
                'final_zero_hold_complete']
    if any(
            type(event) is not dict or
            set(event) != ({'name', 'steady_ns', 'ros_ns',
                            'scan_header_stamp_ns'}
                           if event.get('name') == 't0_scan' else
                           {'name', 'steady_ns', 'ros_ns'}) or
            type(event['steady_ns']) is not int or
            type(event['ros_ns']) is not int or
            event['steady_ns'] <= 0 or event['ros_ns'] < 0
            for event in events):
        raise ValueError('kidnapped causal event drift')
    names = [event['name'] for event in events]
    if names != required:
        raise ValueError('kidnapped causal event drift')
    if any(left['steady_ns'] >= right['steady_ns']
           for left, right in zip(events, events[1:])):
        raise ValueError('kidnapped event order drift')
    if any(left['ros_ns'] > right['ros_ns']
           for left, right in zip(events, events[1:])):
        raise ValueError('kidnapped ROS sim-time order drift')
    motion_events = events[names.index('t0_scan'):]
    if any(left['ros_ns'] >= right['ros_ns']
           for left, right in zip(motion_events, motion_events[1:])):
        raise ValueError('kidnapped motion event sim-time drift')
    pre_gt = value['pre_teleport_gt']
    post_gt = value['post_teleport_gt']
    if not all(_pose3(pose) for pose in (
            pre_gt, post_gt, value['pre_teleport_odom'],
            value['post_teleport_odom'])):
        raise ValueError('kidnapped pose schema drift')
    if (math.dist(pre_gt[:2], START_POSE[:2]) > GT_TRANSLATION_TOLERANCE_M or
            math.dist(post_gt[:2], TELEPORT_POSE[:2]) >
            GT_TRANSLATION_TOLERANCE_M or
            abs(_angle_delta(post_gt[2], TELEPORT_POSE[2])) >
            GT_YAW_TOLERANCE_RAD):
        raise ValueError('GT teleport magnitude drift')
    if (math.dist(value['pre_teleport_odom'][:2],
                  value['post_teleport_odom'][:2]) >
            ODOM_CONTINUITY_TOLERANCE_M or
            abs(_angle_delta(value['pre_teleport_odom'][2],
                             value['post_teleport_odom'][2])) >
            GT_YAW_TOLERANCE_RAD):
        raise ValueError('odom discontinuity across teleport')
    t0 = next(event for event in events if event['name'] == 't0_scan')
    verified = next(event for event in events
                    if event['name'] == 'teleport_gt_verified')
    header_boundary = value['teleport_gt_verified_header_stamp_ns']
    if (type(header_boundary) is not int or header_boundary <= 0 or
            type(value['t0_scan_stamp_ns']) is not int or
            value['t0_scan_stamp_ns'] <= 0 or
            t0['scan_header_stamp_ns'] != value['t0_scan_stamp_ns'] or
            t0['steady_ns'] <= verified['steady_ns'] or
            value['t0_scan_stamp_ns'] < header_boundary or
            not _finite_number(value['unwrapped_observation_yaw_rad']) or
            value['unwrapped_observation_yaw_rad'] < ROTATION_TARGET_RAD or
            value['unwrapped_observation_yaw_rad'] >
            ROTATION_TARGET_RAD + GT_YAW_TOLERANCE_RAD or
            not _finite_number(value['reverse_observation_yaw_rad']) or
            value['reverse_observation_yaw_rad'] < 0.0 or
            value['reverse_observation_yaw_rad'] > GT_YAW_TOLERANCE_RAD or
            not _finite_number(value['initial_observation_yaw_rad']) or
            value['initial_observation_yaw_rad'] < INITIAL_OBSERVATION_RAD or
            not _finite_number(value['initial_reverse_yaw_rad']) or
            value['initial_reverse_yaw_rad'] < 0.0 or
            value['initial_reverse_yaw_rad'] > GT_YAW_TOLERANCE_RAD or
            not _finite_number(value['post_zero_observation_yaw_rad']) or
            value['post_zero_observation_yaw_rad'] <
            POST_ZERO_OBSERVATION_RAD or
            not _finite_number(value['post_zero_reverse_yaw_rad']) or
            value['post_zero_reverse_yaw_rad'] < 0.0 or
            value['post_zero_reverse_yaw_rad'] > GT_YAW_TOLERANCE_RAD or
            value['final_zero'] is not True or
            not _finite_number(value['zero_hold_s']) or
            value['zero_hold_s'] < ZERO_HOLD_S or
            type(value['stationary_sample_count']) is not int or
            value['stationary_sample_count'] < 10 or
            type(value['post_teleport_initialpose_count']) is not int or
            value['post_teleport_initialpose_count'] != 0):
        raise ValueError('kidnapped rotation or stop gate failed')
    return value


class KidnappedDriver(Node):
    """Drive the exact teleport and observation rotation state machine."""

    def __init__(self, args):
        """Create subscriptions and state for one kidnapped trace."""
        super().__init__(
            'g002_axis_b_kidnapped_driver', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.args = args
        self.events = []
        self.gt = None
        self.odom = None
        self.pre_gt = None
        self.post_gt = None
        self.pre_odom = None
        self.post_odom = None
        self.stationary_count = 0
        self.t0_scan_stamp_ns = None
        self.teleport_gt_verified_header_stamp_ns = None
        self.post_teleport_initialpose_count = 0
        self.accumulated_yaw_rad = 0.0
        self.reverse_observation_yaw_rad = 0.0
        self.initial_observation_yaw_rad = 0.0
        self.initial_reverse_yaw_rad = 0.0
        self.post_zero_observation_yaw_rad = 0.0
        self.post_zero_reverse_yaw_rad = 0.0
        self.last_yaw = None
        self.phase = 'INITIAL_ROTATE'
        self.zero_started = None
        self.done = False
        self.failure = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/ground_truth_pose', self._gt, 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._initialpose, 10)
        self.create_subscription(
            LaserScan, '/sim_raw/scan', self._scan, qos_profile_sensor_data)
        self.create_timer(0.02, self._tick)

    def _event(self, name, scan_header_stamp_ns=None):
        event = {'name': name, 'steady_ns': time.monotonic_ns(),
                 'ros_ns': self.get_clock().now().nanoseconds}
        if name == 't0_scan':
            if type(scan_header_stamp_ns) is not int:
                raise ValueError('t0 event requires scan header stamp')
            event['scan_header_stamp_ns'] = scan_header_stamp_ns
        elif scan_header_stamp_ns is not None:
            raise ValueError('scan header stamp is only valid for t0')
        self.events.append(event)

    def _gt(self, message):
        pose = message.pose
        current = [pose.position.x, pose.position.y, _yaw(pose.orientation)]
        self.gt = current
        if self.phase == 'WAIT_GT_TELEPORT' and math.dist(
                current[:2], TELEPORT_POSE[:2]) <= GT_TRANSLATION_TOLERANCE_M:
            self.post_gt = list(current)
            self.post_odom = list(self.odom)
            self.teleport_gt_verified_header_stamp_ns = _stamp_ns(
                message.header.stamp)
            self._event('teleport_gt_verified')
            self.phase = 'WAIT_T0'
        elif self.phase in ('INITIAL_ROTATE', 'ROTATE', 'POST_ZERO_ROTATE'):
            if self.last_yaw is not None:
                delta = _angle_delta(current[2], self.last_yaw)
                if self.phase == 'INITIAL_ROTATE':
                    self.initial_observation_yaw_rad += delta
                    if delta < 0.0:
                        self.initial_reverse_yaw_rad += abs(delta)
                elif self.phase == 'ROTATE':
                    self.accumulated_yaw_rad += delta
                    if delta < 0.0:
                        self.reverse_observation_yaw_rad += abs(delta)
                else:
                    self.post_zero_observation_yaw_rad += delta
                    if delta < 0.0:
                        self.post_zero_reverse_yaw_rad += abs(delta)
            self.last_yaw = current[2]

    def _odom(self, message):
        pose = message.pose.pose
        self.odom = [pose.position.x, pose.position.y, _yaw(pose.orientation)]
        speed = math.hypot(message.twist.twist.linear.x,
                           message.twist.twist.linear.y)
        angular = abs(message.twist.twist.angular.z)
        if self.phase == 'WAIT_STATIONARY':
            self.stationary_count = self.stationary_count + 1 \
                if speed <= STATIONARY_SPEED_MPS and angular <= 0.01 else 0

    def _scan(self, message):
        if self.phase == 'WAIT_T0':
            self.t0_scan_stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000 +
                int(message.header.stamp.nanosec))
            self._event('t0_scan', self.t0_scan_stamp_ns)
            self.last_yaw = self.gt[2]
            self.phase = 'ROTATE'

    def _initialpose(self, _message):
        if self.phase in (
                'WAIT_GT_TELEPORT', 'WAIT_T0', 'ROTATE', 'ZERO_HOLD',
                'POST_ZERO_ROTATE', 'FINAL_ZERO_HOLD'):
            self.post_teleport_initialpose_count += 1

    def _teleport(self):
        self._event('teleport_requested')
        request = ('name: "jdamr_cube", position {x: 0.0 y: 0.0 z: 0.01} '
                   'orientation {z: 1.0 w: 0.0}')
        result = subprocess.run([
            'gz', 'service', '-s', '/world/slam_corridor/set_pose',
            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000', '--req', request],
            capture_output=True, text=True, timeout=10.0, check=False)
        if result.returncode != 0 or 'data: true' not in result.stdout.lower():
            raise RuntimeError('Gazebo robot teleport failed')
        self._event('teleport_ack')

    def _tick(self):
        try:
            if self.phase == 'INITIAL_ROTATE':
                command = Twist()
                command.angular.z = rotation_command_radps(
                    self.initial_observation_yaw_rad)
                self.cmd_pub.publish(command)
                if self.initial_observation_yaw_rad >= INITIAL_OBSERVATION_RAD:
                    self._event('initial_observation_complete')
                    self.zero_started = time.monotonic()
                    self.phase = 'INITIAL_ZERO_HOLD'
            elif self.phase == 'INITIAL_ZERO_HOLD':
                self.cmd_pub.publish(Twist())
                if time.monotonic() - self.zero_started >= ZERO_HOLD_S:
                    self.stationary_count = 0
                    self.phase = 'WAIT_STATIONARY'
            elif (self.phase == 'WAIT_STATIONARY' and self.stationary_count >= 10
                    and self.gt is not None and self.odom is not None):
                self.pre_gt = list(self.gt)
                self.pre_odom = list(self.odom)
                self._event('stationary_confirmed')
                self._teleport()
                self.phase = 'WAIT_GT_TELEPORT'
            elif self.phase == 'ROTATE':
                command = Twist()
                if self.accumulated_yaw_rad < ROTATION_TARGET_RAD:
                    command.angular.z = rotation_command_radps(
                        self.accumulated_yaw_rad)
                else:
                    self._event('rotation_complete')
                    self.zero_started = time.monotonic()
                    self.phase = 'ZERO_HOLD'
                self.cmd_pub.publish(command)
            elif self.phase == 'ZERO_HOLD':
                self.cmd_pub.publish(Twist())
                if time.monotonic() - self.zero_started >= ZERO_HOLD_S:
                    self._event('zero_hold_complete')
                    self.last_yaw = self.gt[2]
                    self.phase = 'POST_ZERO_ROTATE'
            elif self.phase == 'POST_ZERO_ROTATE':
                command = Twist()
                command.angular.z = rotation_command_radps(
                    self.post_zero_observation_yaw_rad)
                self.cmd_pub.publish(command)
                if self.post_zero_observation_yaw_rad >= \
                        POST_ZERO_OBSERVATION_RAD:
                    self._event('post_zero_observation_complete')
                    self.zero_started = time.monotonic()
                    self.phase = 'FINAL_ZERO_HOLD'
            elif self.phase == 'FINAL_ZERO_HOLD':
                self.cmd_pub.publish(Twist())
                if time.monotonic() - self.zero_started >= ZERO_HOLD_S:
                    self._event('final_zero_hold_complete')
                    self.done = True
        except Exception as exc:
            self.failure = f'{type(exc).__name__}: {exc}'
            self.done = True

    def evidence(self):
        """Return the bounded trace document for independent validation."""
        return {'schema_version': 1, 'driver_source': _source_identity(),
                'events': self.events,
                'pre_teleport_gt': self.pre_gt,
                'post_teleport_gt': self.post_gt,
                'pre_teleport_odom': self.pre_odom,
                'post_teleport_odom': self.post_odom,
                't0_scan_stamp_ns': self.t0_scan_stamp_ns,
                'teleport_gt_verified_header_stamp_ns':
                    self.teleport_gt_verified_header_stamp_ns,
                'unwrapped_observation_yaw_rad': self.accumulated_yaw_rad,
                'reverse_observation_yaw_rad':
                    self.reverse_observation_yaw_rad,
                'initial_observation_yaw_rad':
                    self.initial_observation_yaw_rad,
                'initial_reverse_yaw_rad': self.initial_reverse_yaw_rad,
                'post_zero_observation_yaw_rad':
                    self.post_zero_observation_yaw_rad,
                'post_zero_reverse_yaw_rad': self.post_zero_reverse_yaw_rad,
                'final_zero': self.phase == 'FINAL_ZERO_HOLD' or self.done,
                'zero_hold_s': (time.monotonic() - self.zero_started
                                if self.zero_started else 0.0),
                'stationary_sample_count': self.stationary_count,
                'post_teleport_initialpose_count':
                    self.post_teleport_initialpose_count,
                'failure': self.failure}


def main() -> int:
    """Run until the exact kidnapped trace is captured or rejected."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--evidence', required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = KidnappedDriver(args)
    deadline = time.monotonic() + 300.0
    try:
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.done:
            node.failure = 'kidnapped driver timeout'
        value = node.evidence()
        with tempfile.NamedTemporaryFile(
                dir=args.evidence.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(canonical_json_bytes(value))
        temporary.replace(args.evidence)
        if node.failure is None:
            validate_kidnapped_trace(value)
        return 0 if node.failure is None else 1
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
