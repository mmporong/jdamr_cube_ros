#!/usr/bin/env python3
"""Collect evidence for one G004 Collision Monitor simulation."""

from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
from typing import Any

from geometry_msgs.msg import PoseStamped, Twist

from jdamr_cube_navigation.sim_scan_gate import _stop_zone_points

from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState

from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.utilities import remove_ros_args

from sensor_msgs.msg import LaserScan

from std_srvs.srv import SetBool

from tf2_msgs.msg import TFMessage
import tf2_py

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:
    Contacts = None


SET_POSE_SERVICE = '/world/slam_corridor/set_pose'
ACTION_TERMINALS = {4: 'succeeded', 5: 'cancelled', 6: 'aborted'}
MAX_GOAL_TF_EDGES = 32


def _point_segment_distance(point, start, end) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return math.dist(point, start)
    ratio = max(0.0, min(1.0, (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / denominator))
    return math.dist(point, (start[0] + ratio * dx, start[1] + ratio * dy))


def rectangle_clearance(
        robot_pose_xyyaw, footprint, obstacle_center, obstacle_dimensions,
) -> float:
    """Return minimum distance between oriented robot and obstacle boxes."""
    x_m, y_m, yaw_rad = robot_pose_xyyaw
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    local = [
        (footprint['front_m'], footprint['half_width_m']),
        (footprint['front_m'], -footprint['half_width_m']),
        (footprint['rear_m'], -footprint['half_width_m']),
        (footprint['rear_m'], footprint['half_width_m']),
    ]
    robot = [
        (x_m + cosine * px - sine * py,
         y_m + sine * px + cosine * py) for px, py in local]
    half_x = obstacle_dimensions[0] / 2.0
    half_y = obstacle_dimensions[1] / 2.0
    ox, oy = obstacle_center
    obstacle = [
        (ox + half_x, oy + half_y), (ox + half_x, oy - half_y),
        (ox - half_x, oy - half_y), (ox - half_x, oy + half_y)]

    def projections(polygon, axis):
        values = [point[0] * axis[0] + point[1] * axis[1]
                  for point in polygon]
        return min(values), max(values)

    for polygon in (robot, obstacle):
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            axis = (-(end[1] - start[1]), end[0] - start[0])
            robot_min, robot_max = projections(robot, axis)
            obstacle_min, obstacle_max = projections(obstacle, axis)
            if robot_max < obstacle_min or obstacle_max < robot_min:
                break
        else:
            continue
        break
    else:
        return 0.0
    edges = list(zip(robot, robot[1:] + robot[:1]))
    obstacle_edges = list(zip(obstacle, obstacle[1:] + obstacle[:1]))
    return min(
        [_point_segment_distance(point, *edge)
         for point in robot for edge in obstacle_edges]
        + [_point_segment_distance(point, *edge)
           for point in obstacle for edge in edges])


def parse_entity_pose_info(output: str, entity_name: str) -> dict[str, Any]:
    """Read the last finite pose while requiring a stable entity ID."""
    decoder = json.JSONDecoder()
    offset = 0
    matches = []
    while offset < len(output):
        while offset < len(output) and output[offset].isspace():
            offset += 1
        if offset == len(output):
            break
        document, offset = decoder.raw_decode(output, offset)
        if not isinstance(document, dict):
            raise ValueError('Gazebo pose/info document must be an object')
        matches.extend(
            pose for pose in document.get('pose', [])
            if pose.get('name') == entity_name)
    if not matches:
        raise ValueError(f'entity not found: {entity_name}')
    entity_ids = {pose.get('id') for pose in matches}
    if (len(entity_ids) != 1 or type(next(iter(entity_ids))) is not int
            or next(iter(entity_ids)) <= 0):
        raise ValueError(f'entity ID changed: {entity_name}')
    position = matches[-1].get('position', {})
    if not isinstance(position, dict):
        raise ValueError(f'incomplete entity pose: {entity_name}')
    axes = ('x', 'y', 'z')
    defaulted_axes = [axis for axis in axes if axis not in position]
    provided = [position[axis] for axis in axes if axis in position]
    if not all(
            not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) for value in provided):
        raise ValueError(f'non-finite entity pose: {entity_name}')
    pose_m = [float(position.get(axis, 0.0)) for axis in axes]
    orientation = matches[-1].get('orientation')
    orientation_defaulted = orientation is None
    if orientation_defaulted:
        orientation_yaw_rad = 0.0
    else:
        if not isinstance(orientation, dict):
            raise ValueError(f'invalid entity orientation: {entity_name}')
        quaternion = [orientation.get(axis, 0.0)
                      for axis in ('x', 'y', 'z', 'w')]
        if not all(
                not isinstance(value, bool)
                and isinstance(value, (int, float))
                and math.isfinite(value) for value in quaternion):
            raise ValueError(f'invalid entity orientation: {entity_name}')
        norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
        if not math.isclose(norm, 1.0, abs_tol=1e-6):
            raise ValueError(f'unnormalized entity orientation: {entity_name}')
        x_q, y_q, z_q, w_q = map(float, quaternion)
        orientation_yaw_rad = math.atan2(
            2.0 * (w_q * z_q + x_q * y_q),
            1.0 - 2.0 * (y_q * y_q + z_q * z_q))
    return {
        'entity_id': entity_ids.pop(), 'pose_m': pose_m,
        'defaulted_axes': defaulted_axes,
        'orientation_yaw_rad': orientation_yaw_rad,
        'orientation_defaulted': orientation_defaulted}


def read_entity_pose(entity_name: str) -> dict[str, Any]:
    """Read one Gazebo pose/info document for a preloaded entity."""
    result = subprocess.run([
        'gz', 'topic', '-e', '-t', '/world/slam_corridor/pose/info',
        '-n', '1', '--json-output',
    ], capture_output=True, text=True, timeout=5.0, check=False)
    if result.returncode != 0 or not result.stdout.strip():
        raise RuntimeError(
            result.stderr.strip() or 'empty Gazebo pose/info sample')
    return parse_entity_pose_info(result.stdout, entity_name)


def retained_size_bytes(paths: list[Path]) -> int:
    """Return retained artifact bytes without following directories."""
    return sum(path.stat().st_size for path in paths if path.is_file())


def robot_entity_contact_pair(
        contact: Any, entity_name: str) -> tuple[str, str] | None:
    """Return a stable pair only for contact between the probe and robot."""
    try:
        names = (str(contact.collision1.name), str(contact.collision2.name))
    except AttributeError:
        return None
    if (any(entity_name in name for name in names)
            and any('jdamr_cube' in name for name in names)):
        return tuple(sorted(names))
    return None


def write_quarantine_manifest(
        path: Path, incomplete_bags: list[Path]) -> None:
    """Record incomplete bags for explicit later cleanup; delete nothing."""
    path.write_text(json.dumps({
        'cleanup_performed': False,
        'incomplete_bags': [str(item.resolve()) for item in incomplete_bags],
    }, indent=2, sort_keys=True) + '\n')


def stamp_ns(message: Any) -> int:
    """Return a ROS header stamp in nanoseconds."""
    return message.header.stamp.sec * 1_000_000_000 + (
        message.header.stamp.nanosec)


class CollisionMonitorScenario(Node):
    """Send one goal and inject one bounded scan or obstacle event."""

    def __init__(self, args: argparse.Namespace, contract: dict) -> None:
        """Create all observation and control endpoints."""
        super().__init__('sim_collision_monitor_scenario')
        self.args = args
        self.contract = contract
        self.direct_scan = getattr(args, 'direct_scan', False)
        if self.direct_scan and (
                args.scenario != 'sudden_obstacle_stop_resume'
                or contract.get('integration_kind')
                != 'onboard_candidate_direct_scan'):
            raise ValueError('direct scan requires the onboard sudden contract')
        self.direct_scan_armed = False
        self.direct_scan_capture = None
        self.started_steady_ns = time.monotonic_ns()
        self.started_ros_ns = self.get_clock().now().nanoseconds
        self.events: list[dict[str, Any]] = []
        self.raw_scan_count = 0
        self.navigation_scan_count = 0
        self.monitor_scan_count = 0
        self.raw_count_at_trigger = 0
        self.navigation_count_at_trigger = 0
        self.reference_scan_stamp_ns: int | None = None
        self.clear_scan_stamp_ns: int | None = None
        self.clear_reference_scan_stamp_ns: int | None = None
        self.last_monitor_scan_stamp_ns: int | None = None
        self.last_monitor_scan_steady_ns: int | None = None
        self.monitor_scan_count_at_freeze: int | None = None
        self.last_monitor_scan_stamp_ns_at_freeze: int | None = None
        self.last_monitor_scan_steady_ns_at_freeze: int | None = None
        self.monitor_scan_count_at_stop: int | None = None
        self.raw_scan_count_at_stop: int | None = None
        self.navigation_scan_count_at_stop: int | None = None
        self.minimum_scan_range_m = math.inf
        self.last_pose: tuple[float, float] | None = None
        self.world_pose: tuple[float, float] | None = None
        self.world_yaw_rad: float | None = None
        self.terminal_ground_truth_pose: tuple[float, float] | None = None
        self.final_estimated_pose: tuple[float, float] | None = None
        self.final_estimated_pose_stamp_ns: int | None = None
        self.final_estimated_pose_observed_ros_ns: int | None = None
        self.goal_tolerance_entry: dict[str, Any] | None = None
        self.goal_tf_buffer = tf2_py.BufferCore()
        self.goal_tf_lock = threading.Lock()
        self.goal_tf_records: dict[tuple[str, str], deque] = {}
        self.initial_world_pose: tuple[float, float] | None = None
        self.trigger_pose: tuple[float, float] | None = None
        self.stop_pose: tuple[float, float] | None = None
        self.last_linear_speed_mps = 0.0
        self.moving_observed = False
        self.physical_stop_observed = False
        self.physical_stop_steady_ns: int | None = None
        self.stop_state_count = 0
        self.pre_stop_action_types: set[int] = set()
        self.last_action_type = 0
        self.stop_action_type: int | None = None
        self.stop_polygon_name: str | None = None
        self.resume_action_type: int | None = None
        self.trigger_steady_ns: int | None = None
        self.stop_steady_ns: int | None = None
        self.stop_ros_ns: int | None = None
        self.zero_steady_ns: int | None = None
        self.zero_ros_ns: int | None = None
        self.zero_started_ns: int | None = None
        self.contact_count = 0
        self.raw_contact_count = 0
        self.robot_contact_pairs: set[tuple[str, str]] = set()
        self.contact_matched_publisher_count_max = 0
        self.minimum_clearance_m = math.inf
        self.clearance_sample_count = 0
        self.minimum_clearance_witness: dict[str, Any] | None = None
        self.clearance_samples: list[list[float | int]] = []
        self.obstacle_center_m: tuple[float, float] | None = None
        self.obstacle_active = False
        self.obstacle_activation_steady_ns: int | None = None
        self.obstacle_activation_ros_ns: int | None = None
        self.obstacle_request_steady_ns: int | None = None
        self.obstacle_request_ros_ns: int | None = None
        self.obstacle_verified_steady_ns: int | None = None
        self.obstacle_verified_ros_ns: int | None = None
        self.gate_arm_request_steady_ns: int | None = None
        self.gate_arm_request_ros_ns: int | None = None
        self.gate_arm_ack_steady_ns: int | None = None
        self.gate_arm_ack_ros_ns: int | None = None
        self.clear_request_steady_ns: int | None = None
        self.clear_request_ros_ns: int | None = None
        self.clear_ack_steady_ns: int | None = None
        self.clear_ack_ros_ns: int | None = None
        self.clear_verified_steady_ns: int | None = None
        self.clear_verified_ros_ns: int | None = None
        self.entity_states: list[dict[str, Any]] = []
        self.goal_send_count = 0
        self.goal_cancel_count = 0
        self.goal_uuid: str | None = None
        self.terminal_goal_uuid: str | None = None
        self.action_terminal = 'unknown'
        self.finished = False
        self.activation_error: str | None = None
        self.harness_error: str | None = None
        self.phase = 'WAITING_FOR_GOAL'
        self.pending_freeze_value: bool | None = None
        self.freeze_ros_ns: int | None = None
        self.freeze_steady_ns: int | None = None
        self.freeze_request_ros_ns: int | None = None
        self.freeze_request_steady_ns: int | None = None
        self.unfreeze_request_ros_ns: int | None = None
        self.unfreeze_request_steady_ns: int | None = None
        self.unfreeze_ack_ros_ns: int | None = None
        self.unfreeze_ack_steady_ns: int | None = None
        self.observation_group = MutuallyExclusiveCallbackGroup()
        self.action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(
            TFMessage, '/tf', self._goal_tf_dynamic, 100)
        self.create_subscription(
            TFMessage, '/tf_static', self._goal_tf_static,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.freeze_client = (None if self.direct_scan else self.create_client(
            SetBool, '/sim_scan_gate/freeze_monitor'))
        self.create_subscription(
            LaserScan, '/scan' if self.direct_scan else '/sim_raw/scan',
            self._raw_scan,
            qos_profile_sensor_data,
            callback_group=self.observation_group)
        self.create_subscription(
            LaserScan, '/scan', self._navigation_scan,
            qos_profile_sensor_data,
            callback_group=self.observation_group)
        self.create_subscription(
            LaserScan, '/scan' if self.direct_scan else '/collision_monitor_scan',
            self._monitor_scan,
            qos_profile_sensor_data,
            callback_group=self.observation_group)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state',
            self._monitor_state, 10, callback_group=self.observation_group)
        self.create_subscription(
            Odometry, '/odom', self._odom, 10,
            callback_group=self.observation_group)
        self.create_subscription(
            PoseStamped, '/ground_truth_pose', self._ground_truth, 10,
            callback_group=self.observation_group)
        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel, 10,
            callback_group=self.observation_group)
        self.contact_subscription_created = False
        if Contacts is None:
            self.activation_error = 'ros_gz_interfaces/Contacts unavailable'
        elif not args.contact_topic:
            self.activation_error = 'contact topic is required'
        else:
            self.create_subscription(
                Contacts, args.contact_topic, self._contacts, 10,
                callback_group=self.observation_group)
            self.contact_subscription_created = True
        self.create_timer(0.02, self._tick)

    def _event(self, name: str, **details: Any) -> dict[str, Any]:
        existing = [event for event in self.events if event['name'] == name]
        if existing:
            return existing[0]
        event = {
            'name': name,
            'steady_ns': time.monotonic_ns(),
            'ros_ns': self.get_clock().now().nanoseconds,
            **details,
        }
        self.events.append(event)
        return event

    def _raw_scan(self, message: LaserScan) -> None:
        self.raw_scan_count += 1
        finite = [value for value in message.ranges if math.isfinite(value)]
        if finite:
            self.minimum_scan_range_m = min(
                self.minimum_scan_range_m, min(finite))

    def _navigation_scan(self, message: LaserScan) -> None:
        self.navigation_scan_count += 1

    def _monitor_scan(self, message: LaserScan) -> None:
        self.monitor_scan_count += 1
        current_stamp_ns = stamp_ns(message)
        self.last_monitor_scan_stamp_ns = current_stamp_ns
        self.last_monitor_scan_steady_ns = time.monotonic_ns()
        if (self.direct_scan and self.direct_scan_armed
                and self.direct_scan_capture is None
                and self.obstacle_request_steady_ns is not None
                and self.reference_scan_stamp_ns is not None
                and current_stamp_ns > self.reference_scan_stamp_ns):
            points = _stop_zone_points(message, self.contract)
            if len(points) >= self.contract['stop_zone']['min_points']:
                self.direct_scan_capture = {
                    'scan_receive_steady_ns': self.last_monitor_scan_steady_ns,
                    'scan_header_stamp_ns': current_stamp_ns,
                    'stop_zone_points': points,
                    'semantic': self.contract['measurement_semantic'],
                }
        if self.reference_scan_stamp_ns is None:
            self.reference_scan_stamp_ns = current_stamp_ns
        if (self.stop_steady_ns is not None
                and self.clear_reference_scan_stamp_ns is not None
                and current_stamp_ns > self.clear_reference_scan_stamp_ns
                and self.clear_scan_stamp_ns is None):
            self.clear_scan_stamp_ns = current_stamp_ns

    def _monitor_state(self, message: CollisionMonitorState) -> None:
        self.last_action_type = int(message.action_type)
        if self.direct_scan and self.trigger_steady_ns is None:
            return
        if self.direct_scan and self.stop_steady_ns is None:
            self.pre_stop_action_types.add(int(message.action_type))
        if message.action_type == CollisionMonitorState.STOP:
            self.stop_state_count += 1
            if self.stop_steady_ns is None:
                event = self._event(
                    'stop_state', polygon=message.polygon_name)
                self.stop_steady_ns = event['steady_ns']
                self.stop_ros_ns = event['ros_ns']
                self.monitor_scan_count_at_stop = self.monitor_scan_count
                self.raw_scan_count_at_stop = self.raw_scan_count
                self.navigation_scan_count_at_stop = self.navigation_scan_count
                self.stop_action_type = int(message.action_type)
                self.stop_polygon_name = message.polygon_name
        elif self.stop_steady_ns is not None:
            self.resume_action_type = int(message.action_type)
            self._event('do_nothing_after_stop')

    def _odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.last_pose = (position.x, position.y)
        twist = message.twist.twist
        self.last_linear_speed_mps = math.hypot(
            twist.linear.x, twist.linear.y)
        if self.last_linear_speed_mps > 0.03:
            self.moving_observed = True
        if (self.stop_steady_ns is not None and self.moving_observed
                and self.zero_steady_ns is not None
                and self.last_linear_speed_mps <= 0.01):
            self.physical_stop_observed = True
            if self.stop_pose is None:
                self.stop_pose = self.world_pose
                event = self._event('physical_stop')
                self.physical_stop_steady_ns = event['steady_ns']

    def _ground_truth(self, message: PoseStamped) -> None:
        position = message.pose.position
        orientation = message.pose.orientation
        self.world_pose = (position.x, position.y)
        self.world_yaw_rad = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z))
        if self.initial_world_pose is None:
            self.initial_world_pose = self.world_pose
        if (self.obstacle_active and self.obstacle_center_m is not None
                and self.world_yaw_rad is not None):
            inputs = self.contract['stop_zone']['inputs']
            footprint = {
                'front_m': inputs['footprint_front_m'],
                'rear_m': inputs['footprint_rear_m'],
                'half_width_m': inputs['footprint_half_width_m'],
            }
            dimensions = [
                self.contract['sudden_obstacle']['length_m'],
                self.contract['sudden_obstacle']['width_m']]
            robot_pose = [position.x, position.y, self.world_yaw_rad]
            sample = [stamp_ns(message), *robot_pose]
            self.clearance_samples.append(sample)
            clearance_m = rectangle_clearance(
                robot_pose, footprint, self.obstacle_center_m, dimensions)
            if clearance_m < self.minimum_clearance_m:
                self.minimum_clearance_m = clearance_m
                self.minimum_clearance_witness = {
                    'robot_pose_xyyaw': robot_pose,
                    'obstacle_center_xy': list(self.obstacle_center_m),
                    'obstacle_dimensions_m': dimensions,
                    'clearance_m': clearance_m,
                }
            self.clearance_sample_count += 1

    def _cmd_vel(self, message: Twist) -> None:
        is_zero = (
            abs(message.linear.x) <= 1e-4
            and abs(message.linear.y) <= 1e-4
            and abs(message.angular.z) <= 1e-4)
        if is_zero:
            if (self.direct_scan_capture is not None
                    and 'zero_receive_steady_ns' not in self.direct_scan_capture):
                self.direct_scan_capture['zero_receive_steady_ns'] = (
                    time.monotonic_ns())
            if self.zero_started_ns is None:
                self.zero_started_ns = time.monotonic_ns()
            if self.stop_steady_ns is not None and self.zero_steady_ns is None:
                event = self._event('final_zero')
                self.zero_steady_ns = event['steady_ns']
                self.zero_ros_ns = event['ros_ns']
        else:
            self.zero_started_ns = None

    def _contacts(self, message: Any) -> None:
        contacts = list(getattr(message, 'contacts', []))
        self.raw_contact_count += len(contacts)
        for contact in contacts:
            pair = robot_entity_contact_pair(contact, self.args.entity_name)
            if pair is not None:
                self.contact_count += 1
                self.robot_contact_pairs.add(pair)

    def _set_entity_pose(
            self, pose_m: tuple[float, float, float],
            transition: str | None = None) -> bool:
        request = (
            f'name: "{self.args.entity_name}", position {{x: {pose_m[0]}, '
            f'y: {pose_m[1]}, z: {pose_m[2]}}}')
        result = subprocess.run([
            'gz', 'service', '-s', SET_POSE_SERVICE,
            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000', '--req', request,
        ], capture_output=True, text=True, timeout=10.0, check=False)
        if result.returncode != 0 or 'data: true' not in result.stdout.lower():
            self.activation_error = (
                result.stderr.strip() or result.stdout.strip())
            return False
        if transition is not None:
            event = self._event(f'{transition}_set_pose_ack')
            if transition == 'obstacle':
                self.obstacle_activation_steady_ns = event['steady_ns']
                self.obstacle_activation_ros_ns = event['ros_ns']
            else:
                self.clear_ack_steady_ns = event['steady_ns']
                self.clear_ack_ros_ns = event['ros_ns']
        deadline_s = time.monotonic() + 5.0
        while time.monotonic() < deadline_s:
            try:
                state = read_entity_pose(self.args.entity_name)
            except (RuntimeError, ValueError, subprocess.TimeoutExpired):
                continue
            error_m = math.dist(state['pose_m'], pose_m)
            if error_m <= 0.02:
                state.update({
                    'expected_pose_m': list(pose_m),
                    'position_error_m': error_m,
                })
                self.entity_states.append(state)
                if transition is not None:
                    event = self._event(f'{transition}_pose_verified')
                    if transition == 'obstacle':
                        self.obstacle_verified_steady_ns = event['steady_ns']
                        self.obstacle_verified_ros_ns = event['ros_ns']
                    else:
                        self.clear_verified_steady_ns = event['steady_ns']
                        self.clear_verified_ros_ns = event['ros_ns']
                entity_ids = {
                    item['entity_id'] for item in self.entity_states}
                if len(entity_ids) != 1:
                    self.activation_error = 'preloaded entity ID changed'
                    return False
                return True
        self.activation_error = 'preloaded entity pose verification timeout'
        return False

    def _freeze(self, frozen: bool) -> bool:
        if not self.freeze_client.wait_for_service(timeout_sec=5.0):
            self.activation_error = 'freeze service unavailable'
            return False
        request = SetBool.Request()
        request.data = frozen
        future = self.freeze_client.call_async(request)
        self.pending_freeze_value = frozen
        future.add_done_callback(self._freeze_result)
        return True

    def _arm_reaction_capture(self) -> bool:
        if getattr(self, 'direct_scan', False):
            self.direct_scan_armed = True
            self._event('direct_scan_observer_armed')
            return True
        request_event = self._event('scan_gate_arm_requested')
        self.gate_arm_request_steady_ns = request_event['steady_ns']
        self.gate_arm_request_ros_ns = request_event['ros_ns']
        result = subprocess.run([
            'ros2', 'service', 'call', '/sim_scan_gate/arm_reaction',
            'std_srvs/srv/SetBool', '{data: true}',
        ], capture_output=True, text=True, timeout=10.0, check=False)
        response_text = result.stdout.lower().replace(' ', '')
        if (result.returncode != 0
                or ('success=true' not in response_text
                    and 'success:true' not in response_text)):
            self.activation_error = (
                result.stderr.strip() or result.stdout.strip()
                or 'scan gate arm failed')
            return False
        ack_event = self._event('scan_gate_arm_ack')
        self.gate_arm_ack_steady_ns = ack_event['steady_ns']
        self.gate_arm_ack_ros_ns = ack_event['ros_ns']
        return True

    def _freeze_result(self, future: Any) -> None:
        response = future.result()
        frozen = self.pending_freeze_value
        self.pending_freeze_value = None
        if response is None or not response.success:
            self.activation_error = 'freeze service request failed'
            self.phase = 'FAILED'
            return
        if frozen:
            event = self._event('monitor_scan_frozen')
            self.freeze_ros_ns = event['ros_ns']
            self.freeze_steady_ns = event['steady_ns']
            self.monitor_scan_count_at_freeze = self.monitor_scan_count
            self.last_monitor_scan_stamp_ns_at_freeze = (
                self.last_monitor_scan_stamp_ns)
            self.last_monitor_scan_steady_ns_at_freeze = (
                self.last_monitor_scan_steady_ns)
            self.phase = 'WAITING_FOR_STOP'
        else:
            event = self._event('monitor_scan_unfrozen')
            self.unfreeze_ack_ros_ns = event['ros_ns']
            self.unfreeze_ack_steady_ns = event['steady_ns']
            self.phase = 'WAITING_FOR_RESUME'

    def _send_goal(self) -> bool:
        if not self.action_client.wait_for_server(timeout_sec=30.0):
            self.activation_error = 'navigate_to_pose unavailable'
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.args.goal_x_m
        goal.pose.pose.orientation.w = 1.0
        future = self.action_client.send_goal_async(goal)
        self.goal_send_count += 1
        future.add_done_callback(self._goal_response)
        return True

    def _goal_response(self, future: Any) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.action_terminal = 'rejected'
            self.phase = 'FAILED'
            return
        with self.goal_tf_lock:
            self.goal_uuid = bytes(handle.goal_id.uuid).hex()
            self._event('goal_accepted', goal_uuid=self.goal_uuid)
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._action_result)
        self.phase = 'RUNNING'

    def _action_result(self, future: Any) -> None:
        result = future.result()
        with self.goal_tf_lock:
            self.action_terminal = ACTION_TERMINALS.get(
                result.status, f'status_{result.status}')
            self.terminal_goal_uuid = self.goal_uuid
            event = self._event(self.action_terminal)
            self.final_estimated_pose_observed_ros_ns = event['ros_ns']
            self.terminal_ground_truth_pose = self.world_pose
            try:
                transform = self.goal_tf_buffer.lookup_transform_core(
                    'map', 'base_link', rclpy.time.Time().to_msg())
                translation = transform.transform.translation
                self.final_estimated_pose = (translation.x, translation.y)
                self.final_estimated_pose_stamp_ns = stamp_ns(transform)
            except (RuntimeError, tf2_py.TransformException) as error:
                self.harness_error = f'final map-to-base transform: {error}'
        self.phase = 'FINAL_HOLD'

    @staticmethod
    def _tf_record(transform, is_static: bool) -> dict[str, Any]:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            'parent': transform.header.frame_id,
            'child': transform.child_frame_id,
            'stamp_ns': stamp_ns(transform),
            'translation': [translation.x, translation.y, translation.z],
            'rotation': [rotation.x, rotation.y, rotation.z, rotation.w],
            'is_static': is_static,
        }

    def _goal_tf_dynamic(self, message: TFMessage) -> None:
        self._goal_tf_update(message, False)

    def _goal_tf_static(self, message: TFMessage) -> None:
        self._goal_tf_update(message, True)

    def _goal_tf_update(self, message: TFMessage, is_static: bool) -> None:
        """Latch the first exact TF update observed inside XY tolerance."""
        with self.goal_tf_lock:
            candidate_stamps = []
            for transform in message.transforms:
                if not transform.header.frame_id or not transform.child_frame_id:
                    continue
                if is_static:
                    self.goal_tf_buffer.set_transform_static(
                        transform, 'g004_goal_witness')
                else:
                    self.goal_tf_buffer.set_transform(
                        transform, 'g004_goal_witness')
                key = (transform.header.frame_id, transform.child_frame_id)
                if (key not in self.goal_tf_records
                        and len(self.goal_tf_records) >= MAX_GOAL_TF_EDGES):
                    continue
                records = self.goal_tf_records.setdefault(key, deque(maxlen=4))
                records.append(self._tf_record(transform, is_static))
                if not is_static and stamp_ns(transform) > 0:
                    candidate_stamps.append(stamp_ns(transform))
            if (self.goal_uuid is None or self.goal_tolerance_entry is not None
                    or self.action_terminal != 'unknown'):
                return
            for candidate_stamp_ns in sorted(set(candidate_stamps)):
                self._observe_goal_tolerance_entry(candidate_stamp_ns)
                if self.goal_tolerance_entry is not None:
                    return

    def _canonical_goal_tf_inputs(
            self, query_stamp_ns: int) -> list[dict[str, Any]] | None:
        """Return the unique minimal map-to-base transform witness."""
        graph: dict[str, list[str]] = {}
        for parent, child in self.goal_tf_records:
            graph.setdefault(parent, []).append(child)
        paths = []

        def visit(frame: str, path: list[tuple[str, str]], seen: set) -> None:
            if frame == 'base_link':
                paths.append(path)
                return
            for child in graph.get(frame, []):
                if child not in seen:
                    visit(child, path + [(frame, child)], seen | {child})

        visit('map', [], {'map'})
        if len(paths) != 1:
            return None
        selected = []
        for edge in paths[0]:
            unique = {json.dumps(record, sort_keys=True): record
                      for record in self.goal_tf_records[edge]}
            records = sorted(unique.values(), key=lambda item: item['stamp_ns'])
            static = [record for record in records if record['is_static']]
            dynamic = [record for record in records if not record['is_static']]
            if static:
                if len(static) != 1 or dynamic:
                    return None
                chosen = static
            else:
                exact = [record for record in dynamic
                         if record['stamp_ns'] == query_stamp_ns]
                if len(exact) == 1:
                    chosen = exact
                else:
                    before = [record for record in dynamic
                              if record['stamp_ns'] < query_stamp_ns]
                    after = [record for record in dynamic
                             if record['stamp_ns'] > query_stamp_ns]
                    if exact or not before or not after:
                        return None
                    chosen = [before[-1], after[0]]
            selected.extend(chosen)
        return selected

    def _observe_goal_tolerance_entry(self, candidate_stamp_ns: int) -> None:
        """Query one received TF timestamp without a latest-TF fallback."""
        try:
            transform = self.goal_tf_buffer.lookup_transform_core(
                'map', 'base_link',
                rclpy.time.Time(nanoseconds=candidate_stamp_ns).to_msg())
        except (RuntimeError, tf2_py.TransformException):
            return
        stamp_ns = (
            transform.header.stamp.sec * 1_000_000_000
            + transform.header.stamp.nanosec)
        observer_ros_ns = self.get_clock().now().nanoseconds
        stamp_offset_s = (stamp_ns - observer_ros_ns) / 1e9
        position = [
            transform.transform.translation.x,
            transform.transform.translation.y]
        goal = [self.contract['goal_pose']['x_m'],
                self.contract['goal_pose']['y_m']]
        error_m = math.dist(position, goal)
        verification = self.contract['goal_verification']
        if (not all(math.isfinite(value) for value in position)
                or stamp_ns <= 0 or stamp_ns > observer_ros_ns
                or abs(stamp_offset_s)
                > verification['estimate_stamp_tolerance_s']
                or (error_m > verification['position_tolerance_m']
                    and not math.isclose(
                        error_m, verification['position_tolerance_m'],
                        rel_tol=0.0, abs_tol=1e-12))):
            return
        tf_inputs = self._canonical_goal_tf_inputs(stamp_ns)
        if tf_inputs is None:
            return
        event = self._event(
            'goal_position_tolerance_entered', goal_uuid=self.goal_uuid,
            frame_id=transform.header.frame_id,
            child_frame_id=transform.child_frame_id,
            tf_inputs=tf_inputs,
            pose_m=position, goal_pose_m=goal,
            transform_stamp_ns=stamp_ns,
            transform_observer_ros_ns=observer_ros_ns,
            transform_stamp_offset_s=stamp_offset_s,
            position_error_m=error_m)
        self.goal_tolerance_entry = event

    def _trigger(self) -> bool:
        if self.args.scenario == 'scan_timeout_stop_resume':
            self.trigger_steady_ns = time.monotonic_ns()
            self.raw_count_at_trigger = self.raw_scan_count
            self.navigation_count_at_trigger = self.navigation_scan_count
            self.trigger_pose = self.world_pose
            self.reference_scan_stamp_ns = self.last_monitor_scan_stamp_ns
            event = self._event('monitor_scan_freeze_requested')
            self.freeze_request_ros_ns = event['ros_ns']
            self.freeze_request_steady_ns = event['steady_ns']
            triggered = self._freeze(True)
            return triggered
        if self.args.scenario == 'sudden_obstacle_stop_resume':
            if self.world_pose is None:
                return False
            if not self._arm_reaction_capture():
                return False
            pose_snapshot = self.world_pose
            if pose_snapshot is None:
                return False
            self.trigger_steady_ns = time.monotonic_ns()
            self.raw_count_at_trigger = self.raw_scan_count
            self.navigation_count_at_trigger = self.navigation_scan_count
            self.trigger_pose = pose_snapshot
            self.reference_scan_stamp_ns = self.last_monitor_scan_stamp_ns
            center_x_m = pose_snapshot[0] + self.contract[
                'sudden_obstacle']['activation_center_offset_x_m']
            center_y_m = pose_snapshot[1]
            request_event = self._event('obstacle_set_pose_requested')
            self.obstacle_request_steady_ns = request_event['steady_ns']
            self.obstacle_request_ros_ns = request_event['ros_ns']
            triggered = self._set_entity_pose(
                (center_x_m, center_y_m, 0.5), transition='obstacle')
            if triggered:
                self.obstacle_center_m = (center_x_m, center_y_m)
                self.obstacle_active = True
            return triggered
        return True

    def _clear_trigger(self) -> bool:
        self.clear_reference_scan_stamp_ns = self.last_monitor_scan_stamp_ns
        if self.args.scenario == 'scan_timeout_stop_resume':
            event = self._event('monitor_scan_unfreeze_requested')
            self.unfreeze_request_ros_ns = event['ros_ns']
            self.unfreeze_request_steady_ns = event['steady_ns']
            return self._freeze(False)
        event = self._event('clear_set_pose_requested')
        self.clear_request_ros_ns = event['ros_ns']
        self.clear_request_steady_ns = event['steady_ns']
        cleared = self._set_entity_pose(
            tuple(self.contract['sudden_obstacle']['removal_pose_m']),
            transition='clear')
        if cleared:
            self.obstacle_active = False
            self._event('obstacle_deactivated')
        return cleared

    def _tick(self) -> None:
        elapsed_s = (time.monotonic_ns() - self.started_steady_ns) / 1e9
        self.contact_matched_publisher_count_max = max(
            self.contact_matched_publisher_count_max,
            len(self.get_publishers_info_by_topic(self.args.contact_topic))
            if self.args.contact_topic else 0)
        if elapsed_s > self.args.run_timeout_s:
            self.harness_error = 'scenario_timeout'
            self.finished = True
            return
        if self.phase == 'WAITING_FOR_GOAL':
            if self.reference_scan_stamp_ns is None:
                return
            self.phase = 'GOAL_PENDING' if self._send_goal() else 'FAILED'
        elif (self.phase == 'RUNNING'
              and self.args.scenario != 'clear_baseline'
              and self.moving_observed and self.last_pose is not None):
            if (self.direct_scan and self.last_linear_speed_mps
                    < self.contract['sudden_obstacle']['trigger_min_speed_mps']):
                return
            if self._trigger():
                self.phase = (
                    'FREEZE_PENDING'
                    if self.args.scenario == 'scan_timeout_stop_resume'
                    else 'WAITING_FOR_STOP')
            else:
                self.phase = 'FAILED'
        elif (self.phase == 'WAITING_FOR_STOP'
              and self.physical_stop_observed
              and self.zero_steady_ns is not None):
            if self._clear_trigger():
                self.phase = (
                    'UNFREEZE_PENDING'
                    if self.args.scenario == 'scan_timeout_stop_resume'
                    else 'WAITING_FOR_RESUME')
            else:
                self.phase = 'FAILED'
        elif (self.phase == 'WAITING_FOR_RESUME'
              and self.resume_action_type == CollisionMonitorState.DO_NOTHING
              and self.clear_scan_stamp_ns is not None):
            self.phase = 'RESUMED'
        elif self.phase == 'FINAL_HOLD':
            required_hold_s = self.contract['final_zero_hold_s']
            if (self.zero_started_ns is not None
                    and (time.monotonic_ns() - self.zero_started_ns) / 1e9
                    >= required_hold_s):
                self.finished = True
        if self.phase == 'FAILED':
            self.finished = True

    def evidence(self) -> dict[str, Any]:
        """Build durable evidence from observed ROS and steady clocks."""
        ended_steady_ns = time.monotonic_ns()
        ended_ros_ns = self.get_clock().now().nanoseconds
        stop_latency_steady_s = None
        stop_latency_ros_s = None
        latency_reference_ns = self.freeze_steady_ns
        if (self.args.scenario == 'scan_timeout_stop_resume'
                and latency_reference_ns is not None
                and self.zero_steady_ns is not None):
            stop_latency_steady_s = (
                self.zero_steady_ns - latency_reference_ns) / 1e9
        ros_latency_reference_ns = self.freeze_ros_ns
        if (self.args.scenario == 'scan_timeout_stop_resume'
                and ros_latency_reference_ns is not None
                and self.zero_ros_ns is not None):
            stop_latency_ros_s = (
                self.zero_ros_ns - ros_latency_reference_ns) / 1e9
        stop_distance_m = None
        if self.trigger_pose is not None and self.stop_pose is not None:
            stop_distance_m = math.dist(self.trigger_pose, self.stop_pose)
        zero_hold_s = 0.0
        if self.zero_started_ns is not None:
            zero_hold_s = (ended_steady_ns - self.zero_started_ns) / 1e9
        terminal_ground_truth_goal_error_m = None
        terminal_localization_error_m = None
        if self.terminal_ground_truth_pose is not None:
            terminal_ground_truth_goal_error_m = math.dist(
                self.terminal_ground_truth_pose,
                (self.contract['goal_pose']['x_m'],
                 self.contract['goal_pose']['y_m']))
        if (self.terminal_ground_truth_pose is not None
                and self.final_estimated_pose is not None):
            terminal_localization_error_m = math.dist(
                self.terminal_ground_truth_pose, self.final_estimated_pose)
        return {
            'schema_version': 1,
            'direct_scan_capture': self.direct_scan_capture,
            'scan_capture_mode': 'direct_observer' if self.direct_scan else 'gate',
            'scenario_source': {
                'path': str(Path(__file__).resolve()),
                'size_bytes': Path(__file__).resolve().stat().st_size,
                'sha256': hashlib.sha256(
                    Path(__file__).resolve().read_bytes()).hexdigest(),
            },
            'run_id': f'{self.args.scenario}__seed_{self.args.seed}',
            'scenario': self.args.scenario,
            'seed': self.args.seed,
            'contract': self.contract,
            'events': self.events,
            'entity_states': self.entity_states,
            'initial_world_pose_m': self.initial_world_pose,
            'final_world_pose_m': self.world_pose,
            'final_estimated_pose_m': self.final_estimated_pose,
            'final_estimated_pose_frame_id': 'map',
            'final_estimated_pose_stamp_ns': (
                self.final_estimated_pose_stamp_ns),
            'final_estimated_pose_observed_ros_ns': (
                self.final_estimated_pose_observed_ros_ns),
            'estimated_pose_stamp_offset_at_terminal_s': (
                (self.final_estimated_pose_stamp_ns
                 - self.final_estimated_pose_observed_ros_ns) / 1e9
                if self.final_estimated_pose_stamp_ns is not None
                and self.final_estimated_pose_observed_ros_ns is not None
                else None),
            'terminal_ground_truth_pose_m': self.terminal_ground_truth_pose,
            'terminal_ground_truth_observed_ros_ns': (
                self.final_estimated_pose_observed_ros_ns),
            'terminal_ground_truth_goal_error_m': (
                terminal_ground_truth_goal_error_m),
            'terminal_estimated_to_gt_error_m': (
                terminal_localization_error_m),
            'goal_position_tolerance_entry': self.goal_tolerance_entry,
            'goal_uuid': self.goal_uuid,
            'terminal_goal_uuid': self.terminal_goal_uuid,
            'goal_send_count': self.goal_send_count,
            'goal_cancel_count': self.goal_cancel_count,
            'action_terminal': self.action_terminal,
            'raw_scan_continued': (
                self.raw_scan_count > self.raw_count_at_trigger),
            'navigation_scan_continued': (
                self.navigation_scan_count > self.navigation_count_at_trigger),
            'collision_monitor_scan_frozen': (
                self.monitor_scan_count_at_freeze is not None
                and self.monitor_scan_count_at_stop
                == self.monitor_scan_count_at_freeze),
            'monitor_scan_count_at_freeze': (
                self.monitor_scan_count_at_freeze),
            'monitor_scan_count_at_stop': self.monitor_scan_count_at_stop,
            'monitor_scan_count_final': self.monitor_scan_count,
            'last_monitor_scan_steady_ns_at_freeze': (
                self.last_monitor_scan_steady_ns_at_freeze),
            'last_monitor_scan_stamp_ns_at_freeze': (
                self.last_monitor_scan_stamp_ns_at_freeze),
            'raw_scan_count_at_trigger': self.raw_count_at_trigger,
            'raw_scan_count_at_stop': self.raw_scan_count_at_stop,
            'navigation_scan_count_at_trigger': (
                self.navigation_count_at_trigger),
            'navigation_scan_count_at_stop': (
                self.navigation_scan_count_at_stop),
            'reference_scan_stamp_ns': self.reference_scan_stamp_ns,
            'scan_gate_arm_request_steady_ns': (
                self.gate_arm_request_steady_ns),
            'scan_gate_arm_request_ros_ns': self.gate_arm_request_ros_ns,
            'scan_gate_arm_ack_steady_ns': self.gate_arm_ack_steady_ns,
            'scan_gate_arm_ack_ros_ns': self.gate_arm_ack_ros_ns,
            'obstacle_activation_steady_ns': (
                self.obstacle_activation_steady_ns),
            'obstacle_activation_ros_ns': self.obstacle_activation_ros_ns,
            'obstacle_request_steady_ns': self.obstacle_request_steady_ns,
            'obstacle_request_ros_ns': self.obstacle_request_ros_ns,
            'obstacle_verified_steady_ns': self.obstacle_verified_steady_ns,
            'obstacle_verified_ros_ns': self.obstacle_verified_ros_ns,
            'obstacle_request_to_zero_steady_s': (
                (self.zero_steady_ns - self.obstacle_request_steady_ns) / 1e9
                if self.zero_steady_ns is not None
                and self.obstacle_request_steady_ns is not None else None),
            'obstacle_ack_to_zero_steady_s': (
                (self.zero_steady_ns
                 - self.obstacle_activation_steady_ns) / 1e9
                if self.zero_steady_ns is not None
                and self.obstacle_activation_steady_ns is not None else None),
            'clear_request_steady_ns': self.clear_request_steady_ns,
            'clear_request_ros_ns': self.clear_request_ros_ns,
            'clear_ack_steady_ns': self.clear_ack_steady_ns,
            'clear_ack_ros_ns': self.clear_ack_ros_ns,
            'clear_verified_steady_ns': self.clear_verified_steady_ns,
            'clear_verified_ros_ns': self.clear_verified_ros_ns,
            'unfreeze_request_steady_ns': self.unfreeze_request_steady_ns,
            'unfreeze_request_ros_ns': self.unfreeze_request_ros_ns,
            'unfreeze_ack_steady_ns': self.unfreeze_ack_steady_ns,
            'unfreeze_ack_ros_ns': self.unfreeze_ack_ros_ns,
            'trigger_steady_ns': self.trigger_steady_ns,
            'trigger_world_pose_m': self.trigger_pose,
            'clear_scan_stamp_ns': self.clear_scan_stamp_ns,
            'clear_reference_scan_stamp_ns': (
                self.clear_reference_scan_stamp_ns),
            'stop_state_count': self.stop_state_count,
            'pre_stop_action_types': sorted(self.pre_stop_action_types),
            'stop_action_type': self.stop_action_type,
            'stop_polygon_name': self.stop_polygon_name,
            'final_action_type': self.last_action_type,
            'resume_action_type': self.resume_action_type,
            'freeze_ack_to_zero_ros_s': (
                stop_latency_ros_s
                if self.args.scenario == 'scan_timeout_stop_resume'
                else None),
            'freeze_ack_to_zero_steady_s': (
                stop_latency_steady_s
                if self.args.scenario == 'scan_timeout_stop_resume'
                else None),
            'freeze_ros_ns': self.freeze_ros_ns,
            'freeze_steady_ns': self.freeze_steady_ns,
            'freeze_request_ros_ns': self.freeze_request_ros_ns,
            'freeze_request_steady_ns': self.freeze_request_steady_ns,
            'stop_state_steady_ns': self.stop_steady_ns,
            'stop_state_ros_ns': self.stop_ros_ns,
            'zero_steady_ns': self.zero_steady_ns,
            'zero_ros_ns': self.zero_ros_ns,
            'stop_distance_m': stop_distance_m,
            'footprint_to_obstacle_clearance_m': (
                self.minimum_clearance_m
                if self.args.scenario == 'sudden_obstacle_stop_resume'
                and math.isfinite(self.minimum_clearance_m) else None),
            'clearance_sample_count': self.clearance_sample_count,
            'minimum_clearance_witness': self.minimum_clearance_witness,
            'clearance_samples': self.clearance_samples,
            'minimum_observed_scan_range_m': (
                self.minimum_scan_range_m
                if math.isfinite(self.minimum_scan_range_m) else None),
            'physical_stop_observed': self.physical_stop_observed,
            'physical_stop_steady_ns': self.physical_stop_steady_ns,
            'stop_world_pose_m': self.stop_pose,
            'contact_count': self.contact_count,
            'raw_contact_count': self.raw_contact_count,
            'robot_contact_pairs': [
                list(pair) for pair in sorted(self.robot_contact_pairs)],
            'contact_matched_publisher_count_max': (
                self.contact_matched_publisher_count_max),
            'contact_subscription_created': (
                self.contact_subscription_created),
            'final_cmd_vel_zero': self.zero_started_ns is not None,
            'final_zero_hold_s': zero_hold_s,
            'ros_start_ns': self.started_ros_ns,
            'ros_end_ns': ended_ros_ns,
            'steady_start_ns': self.started_steady_ns,
            'steady_end_ns': ended_steady_ns,
            'activation_error': self.activation_error,
            'harness_error': self.harness_error,
        }


def parse_args() -> argparse.Namespace:
    """Parse one scenario invocation."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--contract', type=Path, required=True)
    parser.add_argument(
        '--entity-name', default='g003_preloaded_front_observation_probe')
    parser.add_argument('--contact-topic', default='')
    parser.add_argument('--goal-x-m', type=float, default=6.0)
    parser.add_argument('--run-timeout-s', type=float, default=180.0)
    parser.add_argument('--direct-scan', action='store_true')
    args = parser.parse_args(remove_ros_args()[1:])
    if args.direct_scan and (
            args.scenario != 'sudden_obstacle_stop_resume'
            or os.environ.get('ROS_DOMAIN_ID') not in {'186', '187'}):
        parser.error('--direct-scan is limited to isolated sudden-obstacle runs')
    return args


def main() -> int:
    """Run one bounded scenario and persist evidence on every exit."""
    args = parse_args()
    contract = json.loads(args.contract.read_text())
    rclpy.init()
    node = CollisionMonitorScenario(args, contract)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
    except Exception as error:
        node.harness_error = f'{type(error).__name__}: {error}'
    finally:
        evidence = node.evidence()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + '\n')
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if evidence['harness_error'] is None else 2


if __name__ == '__main__':
    raise SystemExit(main())
