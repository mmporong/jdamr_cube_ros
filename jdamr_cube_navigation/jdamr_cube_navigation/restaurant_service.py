"""Teach and visit named table service poses with existing Nav2 parking."""

import argparse
import ast
import json
import math
from pathlib import Path
import signal
import sys
import time
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.corridor_route import _quaternion_yaw, AMCL_QOS, CorridorRoute
from jdamr_cube_navigation.parking import load_parking_contract, ParkingHold, pose_errors
from jdamr_cube_navigation.reverse_parking import reverse_waypoints, static_corridor_clear
from jdamr_cube_navigation.service_destinations import (
    add_pose, candidates, front_gap_evidence, grid_signature, home_pose, load_registry,
    map_grid_signature, new_registry, route_config, save_registry, set_home_pose, taught_pose,
    validate_gap_measurement,
    validate_registry, verify_identity,
)
from nav2_msgs.action import ComputePathThroughPoses, FollowPath, NavigateToPose
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import OccupancyGrid, Path as RosPath
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from tf2_ros import TransformException
import yaml


BLOCKED_PLAN_CODES = {
    ComputePathThroughPoses.Result.GOAL_OCCUPIED,
    ComputePathThroughPoses.Result.NO_VALID_PATH,
}


def load_service_contract(path):
    """Load candidate localization bounds with distinct SI units."""
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('service confidence contract schema must be 1')
    for key in ('max_x_covariance_m2', 'max_y_covariance_m2', 'max_yaw_covariance_rad2',
                'minimum_start_battery_v', 'minimum_running_battery_v'):
        value = document.get(key)
        if (isinstance(value, bool) or not isinstance(value, (float, int))
                or not math.isfinite(value) or value <= 0.0):
            raise ValueError(f'{key} must be finite and positive')
    if document['minimum_start_battery_v'] < document['minimum_running_battery_v']:
        raise ValueError('departure battery threshold must not be below running cutoff')
    return document


def select_destination(poses, plan):
    """Try one alternate only for an occupied goal or a missing path."""
    attempts = []
    for pose in poses[:2]:
        result = plan(pose)
        attempts.append({'pose_id': pose['id'], **result})
        if result['ok']:
            return pose, attempts
        if result.get('error_code') not in BLOCKED_PLAN_CODES:
            break
    return None, attempts


class BoxDwell:
    """Confirm one continuously fresh, stable box observation window."""

    def __init__(self, dwell_s, maximum_age_s=0.75):
        if (isinstance(dwell_s, bool) or not isinstance(dwell_s, (int, float))
                or not math.isfinite(dwell_s) or dwell_s <= 0.0):
            raise ValueError('box dwell must be finite and positive')
        if (isinstance(maximum_age_s, bool)
                or not isinstance(maximum_age_s, (int, float))
                or not math.isfinite(maximum_age_s) or maximum_age_s <= 0.0):
            raise ValueError('box observation age must be finite and positive')
        self.dwell_s = float(dwell_s)
        self.maximum_age_s = float(maximum_age_s)
        self.started_s = None

    def observe(self, now_s, received_s, status):
        """Return a fail-closed hold state for one parsed observer message."""
        values = (now_s, received_s)
        if (not isinstance(status, dict)
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in values)
                or now_s < received_s or now_s - received_s > self.maximum_age_s
                or status.get('detected') is not True
                or status.get('stable') is not True
                or status.get('surface_kind') != 'front'):
            self.started_s = None
            return {'confirmed': False, 'hold_s': 0.0, 'reason': 'box_not_stable'}
        for key in ('front_distance_m', 'lateral_error_m', 'edge_angle_deg'):
            value = status.get(key)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                self.started_s = None
                return {'confirmed': False, 'hold_s': 0.0,
                        'reason': 'box_observation_invalid'}
        if self.started_s is None:
            self.started_s = float(now_s)
        hold_s = max(0.0, float(now_s) - self.started_s)
        return {
            'confirmed': hold_s >= self.dwell_s,
            'hold_s': hold_s,
            'reason': 'box_dwell_complete' if hold_s >= self.dwell_s else 'box_dwell_active',
        }


class ServiceRoute(CorridorRoute):
    """Reuse route protection, low-speed parking and post-goal pose checking."""

    def __init__(self, registry, contract, result_stream):
        initial = {'id': 'capture', 'priority': 1,
                   'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0,
                   'approach_offset_m': 0.5}
        super().__init__(route_config(registry, initial),
                         navigation_profile='obstacle_base_candidate',
                         parking_contract=contract)
        # A named destination may be approached from anywhere on this map.
        self.start_check_pending = False
        self.registry = registry
        self.result_stream = result_stream
        self.table_id = None
        self.pose_id = None
        self.confirmation = None
        self.selected_pose = None
        self.active_handle = None
        self.pending_goal = None
        self.navigation_result = None
        self.navigation_uuid = None
        self.cancel_navigation = self.create_client(
            CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.query_navigation = self.create_client(
            NavigateToPose.Impl.GetResultService, 'navigate_to_pose/_action/get_result')
        self.follow_reverse = ActionClient(self, FollowPath, 'follow_path')
        self.cancel_reverse = self.create_client(
            CancelGoal, 'follow_path/_action/cancel_goal')
        self.query_reverse = self.create_client(
            FollowPath.Impl.GetResultService, 'follow_path/_action/get_result')
        self.validate_reverse_path = self.create_client(IsPathValid, 'is_path_valid')
        self.active_action_type = NavigateToPose
        self.amcl_yaw_covariance_rad2 = None
        package = Path(get_package_share_directory('jdamr_cube_navigation'))
        self.service_contract = load_service_contract(
            package / 'config/restaurant_service_contract.yaml')
        self.minimum_battery_v = self.service_contract['minimum_running_battery_v']
        self.max_amcl_covariance = (
            self.service_contract['max_x_covariance_m2'],
            self.service_contract['max_y_covariance_m2'])
        self.live_grids = {}
        self.expected_grids = {}
        self.map_mismatch = None
        self.box_status = None
        self.box_status_received_s = None
        self.create_subscription(
            String, '/box_parking/perception_status', self._box_callback, 10)
        for key, topic in (('map', '/map'), ('keepout', '/keepout_filter_mask')):
            self.create_subscription(
                OccupancyGrid, topic,
                lambda message, name=key: self._map_callback(name, message), AMCL_QOS)
        self.run_deadline_s = None
        self.create_timer(0.1, self._deadline_tick)

    def _box_callback(self, message):
        try:
            document = json.loads(message.data)
        except (AttributeError, TypeError, json.JSONDecodeError):
            self.box_status = None
            self.box_status_received_s = None
            return
        if not isinstance(document, dict):
            self.box_status = None
            self.box_status_received_s = None
            return
        self.box_status = document
        self.box_status_received_s = time.monotonic()

    def _deadline_tick(self):
        if self.run_deadline_s is not None and time.monotonic() >= self.run_deadline_s:
            self.request_stop()

    def _amcl_callback(self, message):
        super()._amcl_callback(message)
        self.amcl_yaw_covariance_rad2 = float(message.pose.covariance[35])

    def _map_callback(self, name, message):
        try:
            origin = message.info.origin
            if abs(origin.position.z) > 1e-9:
                raise ValueError('map origin must be planar')
            signature = grid_signature(
                message.info.width, message.info.height, message.info.resolution,
                (origin.position.x, origin.position.y, _quaternion_yaw(origin.orientation)),
                message.data, message.header.frame_id)
            self.live_grids[name] = signature
            if name in self.expected_grids and signature != self.expected_grids[name]:
                self.map_mismatch = f'live {name} changed or does not match registered data'
                self.request_stop()
        except ValueError as error:
            self.map_mismatch = str(error)
            self.request_stop()

    def _guard_failure(self, require_fresh_amcl=True):
        if self.map_mismatch:
            return self.map_mismatch
        failure = super()._guard_failure(require_fresh_amcl)
        if failure:
            return failure
        yaw_covariance_rad2 = self.amcl_yaw_covariance_rad2
        if (yaw_covariance_rad2 is None or not math.isfinite(yaw_covariance_rad2)
                or yaw_covariance_rad2 < 0.0):
            return 'AMCL yaw covariance invalid'
        if yaw_covariance_rad2 > self.service_contract['max_yaw_covariance_rad2']:
            return 'AMCL yaw covariance high'
        if any(value < 0.0 for value in self.amcl_covariance):
            return 'AMCL position covariance invalid'
        moved = (self.amcl_motion_distance_m >= self.revisit_motion_min_distance_m
                 or self.amcl_motion_rotation_rad >= self.revisit_motion_min_rotation_rad)
        if moved and time.monotonic() - self.amcl_seen > self.revisit_motion_amcl_freshness_s:
            return 'AMCL stale after motion'
        return None

    def emit(self, event, **fields):
        """Flush structured evidence on each transition, including failures."""
        record = {'event': event, 'table_id': self.table_id,
                  'pose_id': self.pose_id, 'monotonic_s': time.monotonic(),
                  'physical_accuracy': 'NOT_MEASURED', **fields}
        record['front_gap'] = front_gap_evidence(getattr(self, 'selected_pose', None) or {})
        covariance = getattr(self, 'amcl_covariance', None)
        yaw_covariance = getattr(self, 'amcl_yaw_covariance_rad2', None)
        record['localization_covariance'] = {
            'xy_m2': ([value if math.isfinite(value) else None for value in covariance]
                      if covariance is not None else None),
            'yaw_rad2': (
                yaw_covariance if yaw_covariance is not None
                and math.isfinite(yaw_covariance) else None),
        }
        payload = json.dumps(record, ensure_ascii=False, allow_nan=False)
        self.result_stream.write(payload + '\n')
        self.result_stream.flush()
        self.get_logger().info('service_event ' + payload)

    def _route_event(self, event, route_index, handle, **fields):
        if handle is None:
            if event not in ('parking_estimate_confirmed', 'parking_not_confirmed'):
                raise ValueError('goal lifecycle events require a goal handle')
            # Dwell is an observation window, not another Nav2 action goal.
            self.confirmation = fields
            self.emit('parked_dwell_observation', observation_event=event,
                      goal_uuid=None, waypoint_id=self.waypoints[route_index]['id'],
                      **fields)
            return
        super()._route_event(event, route_index, handle, **fields)
        if event == 'accepted':
            self.active_handle = handle
        elif event == 'result':
            self.active_handle = None
        if event in ('parking_estimate_confirmed', 'parking_not_confirmed'):
            self.confirmation = fields
        yaw_covariance = getattr(self, 'amcl_yaw_covariance_rad2', None)
        self.emit(event, waypoint_id=self.waypoints[route_index]['id'],
                  amcl_yaw_covariance_rad2=(
                      yaw_covariance if yaw_covariance is not None
                      and math.isfinite(yaw_covariance) else None), **fields)

    def _wait(self, future, timeout_s):
        deadline_s = time.monotonic() + timeout_s
        while not future.done() and not self.stop_requested and time.monotonic() < deadline_s:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.exception() is not None:
            raise RuntimeError('request interrupted, failed or timed out')
        return future.result()

    @staticmethod
    def _publisher_names(publishers):
        names = []
        for publisher in publishers:
            namespace = getattr(publisher, 'node_namespace', '') or '/'
            name = getattr(publisher, 'node_name', '<unknown>')
            names.append(
                f'/{name}' if namespace == '/'
                else f'{namespace.rstrip("/")}/{name}')
        return names

    def _startup_protection_ready(self, discovery_timeout_s=2.5):
        """Allow only an empty DDS graph to settle before the first motion."""
        expected = {
            '/cmd_vel': 'collision_monitor',
            '/keepout_filter_mask': 'keepout_filter_mask_server',
            '/keepout_costmap_filter_info':
                'keepout_costmap_filter_info_server',
        }
        deadline_s = time.monotonic() + discovery_timeout_s
        while True:
            missing = None
            for topic, node_name in expected.items():
                publishers = self.get_publishers_info_by_topic(topic)
                actual = self._publisher_names(publishers)
                if not publishers:
                    missing = (topic, node_name, actual)
                    continue
                if len(publishers) != 1 or publishers[0].node_name != node_name:
                    return (
                        f'{topic} publisher must be {node_name} only; '
                        f'actual={actual}')
            if missing is None:
                return None
            if self.stop_requested or time.monotonic() >= deadline_s:
                topic, node_name, actual = missing
                return (
                    f'{topic} publisher must be {node_name} only; '
                    f'actual={actual}')
            rclpy.spin_once(self, timeout_sec=0.05)

    def verify_live_maps(self):
        """Require the running map servers to name the registered assets."""
        validate_registry(self.registry)
        self.expected_grids = {
            name: map_grid_signature(self.registry[name]['yaml_path'])
            for name in ('map', 'keepout')}
        deadline_s = time.monotonic() + 3.0
        while (len(self.live_grids) != 2 and not self.stop_requested
               and time.monotonic() < deadline_s):
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.map_mismatch or self.live_grids != self.expected_grids:
            raise RuntimeError(
                self.map_mismatch or 'live map/keepout does not match registered data')
        for node_name, identity in (
                ('map_server', self.registry['map']),
                ('keepout_filter_mask_server', self.registry['keepout'])):
            client = AsyncParameterClient(self, node_name)
            if not client.wait_for_services(timeout_sec=2.0):
                raise RuntimeError(f'{node_name} parameters unavailable')
            response = self._wait(client.get_parameters(['yaml_filename']), 2.0)
            if len(response.values) != 1:
                raise RuntimeError(f'{node_name} yaml_filename unavailable')
            path = parameter_value_to_python(response.values[0])
            if not isinstance(path, str) or not path:
                raise RuntimeError(f'{node_name} yaml_filename is not a file')
            verify_identity(identity, path)
        protection_error = self._startup_protection_ready()
        if protection_error:
            raise RuntimeError(protection_error)

    def plan_pose(self, pose, single=False):
        """Validate the entire approach before dispatching any motion goal."""
        self.pose_id = pose['id']
        self.config = route_config(self.registry, pose)
        if single:
            self.config['waypoints'] = self.config['waypoints'][-1:]
        self.waypoints = self.config['waypoints']
        if self.stop_requested or not self._navigation_ready(require_fresh_amcl=False):
            return {'ok': False, 'reason': 'navigation_not_ready'}
        if not self.compute.wait_for_server(timeout_sec=2.0):
            return {'ok': False, 'reason': 'planner_unavailable'}
        goal = ComputePathThroughPoses.Goal()
        goal.goals = [self._pose(index, item) for index, item in enumerate(self.waypoints)]
        goal.planner_id = 'GridBased'
        goal.use_start = False
        handle = None
        try:
            handle = self._wait(self.compute.send_goal_async(goal), 3.0)
            if not handle.accepted:
                return {'ok': False, 'reason': 'planner_rejected'}
            wrapped = self._wait(handle.get_result_async(), 10.0)
        except RuntimeError as error:
            if handle is not None and handle.accepted:
                self._cancel(handle, 'service planning interrupted')
            return {'ok': False, 'reason': str(error)}
        result = wrapped.result
        ok = (wrapped.status == GoalStatus.STATUS_SUCCEEDED
              and result.error_code == 0 and bool(result.path.poses))
        return {'ok': ok, 'error_code': int(result.error_code),
                'reason': result.error_msg or ('planned' if ok else 'planning_failed')}

    def finish_navigation(self):
        """Resolve late acceptance and wait for terminal cancellation evidence."""
        if self.pending_goal is not None:
            deadline_s = time.monotonic() + 5.0
            while not self.pending_goal.done() and time.monotonic() < deadline_s:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not self.pending_goal.done() or self.pending_goal.exception():
                return self._cancel_navigation_uuid()
            handle = self.pending_goal.result()
            self.pending_goal = None
            if handle.accepted:
                self.active_handle = handle
        if self.active_handle is None:
            return True
        handle = self.active_handle
        result_future = self.navigation_result or handle.get_result_async()
        if not result_future.done():
            self._cancel(handle, 'service command interrupted')
            deadline_s = time.monotonic() + 5.0
            while not result_future.done() and time.monotonic() < deadline_s:
                rclpy.spin_once(self, timeout_sec=0.05)
        terminal = (result_future.done() and result_future.exception() is None
                    and result_future.result().status in (
                        GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                        GoalStatus.STATUS_ABORTED))
        self.emit('navigation_terminal' if terminal else 'cancel_unconfirmed')
        if terminal:
            self.active_handle = None
            self.navigation_result = None
        return terminal

    def _cancel_navigation_uuid(self):
        """Cancel our goal even when its acceptance response was lost."""
        if self.navigation_uuid is None:
            self.emit('cancel_unconfirmed', reason='navigation_goal_id_unavailable')
            return False
        deadline_s = time.monotonic() + 5.0
        cancel_future = None
        query_future = None
        action_type = getattr(self, 'active_action_type', NavigateToPose)
        cancel_client = (self.cancel_reverse if action_type is FollowPath
                         else self.cancel_navigation)
        query_client = (self.query_reverse if action_type is FollowPath
                        else self.query_navigation)
        while time.monotonic() < deadline_s:
            if (cancel_client.service_is_ready()
                    and (cancel_future is None or cancel_future.done())):
                request = CancelGoal.Request()
                request.goal_info.goal_id = self.navigation_uuid
                cancel_future = cancel_client.call_async(request)
            if (query_client.service_is_ready()
                    and (query_future is None or query_future.done())):
                request = action_type.Impl.GetResultService.Request()
                request.goal_id = self.navigation_uuid
                query_future = query_client.call_async(request)
            rclpy.spin_once(self, timeout_sec=0.1)
            if (query_future is not None and query_future.done()
                    and query_future.exception() is None):
                status = query_future.result().status
                if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                              GoalStatus.STATUS_ABORTED):
                    self.pending_goal = None
                    self.active_handle = None
                    self.navigation_result = None
                    self.emit('navigation_terminal', terminal_status_code=int(status))
                    return True
        self.emit('cancel_unconfirmed', reason='navigation_terminal_unavailable',
                  goal_uuid=bytes(self.navigation_uuid.uuid).hex())
        return False

    def _departure_battery_ready(self):
        """Require reserve before a new action, without raising the in-motion cutoff."""
        voltage_v = self.battery_voltage
        threshold_v = self.service_contract['minimum_start_battery_v']
        if (voltage_v is None or not math.isfinite(voltage_v)
                or voltage_v < threshold_v):
            self.emit('departure_blocked', reason='battery_departure_reserve',
                      voltage_v=voltage_v if voltage_v is not None
                      and math.isfinite(voltage_v) else None,
                      minimum_start_battery_v=threshold_v,
                      minimum_running_battery_v=self.minimum_battery_v)
            return False
        return True

    def wait_until_ready(self, timeout=15.0):
        """Check a departure reserve after fresh battery and sensor samples arrive."""
        return (super().wait_until_ready(timeout=timeout)
                and self._departure_battery_ready())

    def execute(self):
        """Bound transit and parking actions and retain their terminal result."""
        if not self.navigate.wait_for_server(timeout_sec=2.0):
            return False
        self.active_action_type = NavigateToPose
        for index, waypoint in enumerate(self.waypoints):
            if self.stop_requested or not self._navigation_ready(require_fresh_amcl=False):
                return False
            if not self._departure_battery_ready():
                return False
            goal = NavigateToPose.Goal()
            goal.pose = self._pose(index, waypoint)
            final = index == len(self.waypoints) - 1
            goal.behavior_tree = self.parking_behavior_tree if final else self.behavior_tree
            self.navigation_uuid = NavigateToPose.Impl.SendGoalService.Request().goal_id
            self.navigation_uuid.uuid = list(uuid.uuid4().bytes)
            self.pending_goal = self.navigate.send_goal_async(goal, goal_uuid=self.navigation_uuid)
            try:
                handle = self._wait(self.pending_goal, 5.0)
                self.pending_goal = None
                if not handle.accepted:
                    self.emit('failed', reason='navigation_rejected')
                    return False
                self._route_event('accepted', index, handle)
                self.navigation_result = handle.get_result_async()
                while not self.navigation_result.done():
                    rclpy.spin_once(self, timeout_sec=0.05)
                    if (self.stop_requested
                            or not self._navigation_ready(require_fresh_amcl=False)):
                        self.emit('interrupted', reason=(
                            self._guard_failure(False) or 'operator_or_timeout'))
                        return False
                wrapped = self.navigation_result.result()
                self._route_event('result', index, handle,
                                  terminal_status_code=int(wrapped.status),
                                  nav2_error_code=int(wrapped.result.error_code))
                self.navigation_result = None
                if (self.stop_requested or wrapped.status != GoalStatus.STATUS_SUCCEEDED
                        or wrapped.result.error_code):
                    return False
                if final and not self._verify_parking_stop(index, waypoint, handle):
                    return False
            finally:
                if not self.finish_navigation():
                    raise RuntimeError('navigation cancellation unconfirmed')
        return True

    def capture_stationary_pose(self, timeout_s=10.0):
        """Capture fresh TF while odometry and consecutive poses remain still."""
        gate = ParkingHold(self.parking_contract)
        target = None
        last_stamp = None
        revision = self.parking_motion_revision
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s and not self.stop_requested:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.parking_odom is None or self.amcl_covariance is None:
                continue
            if self._guard_failure(require_fresh_amcl=True):
                target = None
                continue
            received_s, stamp, linear_mps, angular_radps = self.parking_odom
            stamp_key = (stamp.sec, stamp.nanosec)
            if stamp_key == last_stamp:
                continue
            if revision != self.parking_motion_revision or (last_stamp and stamp_key < last_stamp):
                target = None
            last_stamp = stamp_key
            revision = self.parking_motion_revision
            try:
                transform = self.parking_tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                position = transform.transform.translation
                actual = (position.x, position.y, _quaternion_yaw(transform.transform.rotation))
                ros_now_s = self.get_clock().now().nanoseconds * 1e-9
                ages_s = [time.monotonic() - received_s]
                for source_stamp in (stamp, transform.header.stamp):
                    ages_s.append(ros_now_s - source_stamp.sec - source_stamp.nanosec * 1e-9)
                if min(ages_s) < 0.0:
                    raise ValueError('observation ahead of ROS time')
                if target is None:
                    target = actual
                    gate = ParkingHold(self.parking_contract)
                command = self.parking_command
                # Teaching measures a stationary robot; a silent command topic
                # is recorded as unobserved rather than as measured zero.
                result = gate.observe(
                    time.monotonic(), target, actual, linear_mps, angular_radps,
                    command[1] if command else 0.0,
                    command[2] if command else 0.0, max(ages_s))
                if result['reason'] in ('position_out_of_tolerance', 'yaw_out_of_tolerance'):
                    target = None
                if result['confirmed']:
                    return actual, {
                        **result, 'source_ages_s': ages_s,
                        'amcl_xy_covariance_m2': list(self.amcl_covariance),
                        'amcl_yaw_covariance_rad2': self.amcl_yaw_covariance_rad2,
                        'command_observed': command is not None,
                    }
            except (TransformException, ValueError):
                target = None
        raise RuntimeError(
            'stationary teaching unavailable: need fresh TF, odometry and localization')

    def visit(self, table_id, execute=False, timeout_s=180.0):
        """Resolve, plan, navigate, park and confirm one selected table pose."""
        self.table_id = table_id
        self.selected_pose = None
        self.confirmation = None
        poses = candidates(self.registry, table_id)
        self.run_deadline_s = time.monotonic() + timeout_s
        self.verify_live_maps()
        if not self.wait_until_ready(timeout=10.0):
            raise RuntimeError('navigation data unavailable')
        if not self._parking_parameters_ready():
            raise RuntimeError('load the generated Parking controller parameters into Nav2')
        pose, attempts = select_destination(poses, self.plan_pose)
        self.emit('planning', attempts=attempts)
        if pose is None:
            self.emit('failed', reason='no_service_pose_planned')
            return False
        self.selected_pose = pose
        self.emit('selected', target_pose=[pose[key] for key in ('x_m', 'y_m', 'yaw_rad')],
                  waypoints=route_config(self.registry, pose)['waypoints'],
                  xy_tolerance_m=self.parking_contract['xy_tolerance_m'],
                  yaw_tolerance_rad=self.parking_contract['yaw_tolerance_rad'])
        if not execute:
            self.emit('planned_only')
            return True
        # A second asset check catches changes made while planning.
        self.verify_live_maps()
        success = self.execute()
        self.emit('arrived' if success else 'failed', confirmation=self.confirmation)
        return success

    def wait_for_box(self, dwell_s, timeout_s):
        """Hold at the destination while the front box remains stably observed."""
        gate = BoxDwell(dwell_s)
        if (isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float))
                or not math.isfinite(timeout_s) or timeout_s <= dwell_s):
            raise ValueError('box timeout must be finite and exceed dwell')
        self.emit('box_wait_started', dwell_s=float(dwell_s), timeout_s=float(timeout_s))
        deadline_s = time.monotonic() + float(timeout_s)
        while not self.stop_requested and time.monotonic() < deadline_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            result = gate.observe(
                time.monotonic(), self.box_status_received_s, self.box_status)
            if result['confirmed']:
                self.emit('box_wait_complete', **result, observation=self.box_status)
                return True
        self.emit('box_wait_failed', reason=(
            self._guard_failure(False) or 'box_detection_timeout'))
        return False

    def go_home(self, execute=False, timeout_s=180.0):
        """Plan and optionally execute the taught home pose with yaw verification."""
        pose = home_pose(self.registry)
        self.table_id = 'home_dock'
        self.pose_id = pose['id']
        self.selected_pose = pose
        self.confirmation = None
        self.run_deadline_s = time.monotonic() + timeout_s
        self.verify_live_maps()
        if not self.wait_until_ready(timeout=10.0):
            raise RuntimeError('navigation data unavailable')
        reverse = pose.get('parking_direction', 'forward') == 'reverse'
        if not (self._parking_parameters_ready(reverse=True) if reverse
                else self._parking_parameters_ready()):
            raise RuntimeError('load the generated Parking controller parameters into Nav2')
        if reverse:
            if not self._reverse_smoother_ready():
                raise RuntimeError('reverse velocity is blocked or unbounded in smoother')
            return self._go_home_reverse(pose, execute)
        planned = self.plan_pose(pose)
        self.emit('home_planning', attempt=planned)
        if not planned['ok']:
            self.emit('failed', reason='home_not_planned')
            return False
        self.emit('home_selected',
                  target_pose=[pose[key] for key in ('x_m', 'y_m', 'yaw_rad')],
                  waypoints=route_config(self.registry, pose)['waypoints'],
                  xy_tolerance_m=self.parking_contract['xy_tolerance_m'],
                  yaw_tolerance_rad=self.parking_contract['yaw_tolerance_rad'])
        if not execute:
            self.emit('home_planned_only')
            return True
        self.verify_live_maps()
        success = self.execute()
        self.emit('home_arrived' if success else 'failed', confirmation=self.confirmation)
        return success

    def _make_reverse_path(self, start, target):
        points = reverse_waypoints(
            start, target, xy_tolerance_m=self.parking_contract['xy_tolerance_m'],
            yaw_tolerance_rad=self.parking_contract['yaw_tolerance_rad'])
        path = RosPath()
        path.poses = [self._pose(i, {'x': x, 'y': y, 'yaw': yaw})
                      for i, (x, y, yaw) in enumerate(points)]
        path.header = path.poses[0].header
        return path

    def _reverse_smoother_ready(self):
        client = AsyncParameterClient(self, 'velocity_smoother')
        if not client.wait_for_services(timeout_sec=2.0):
            return False
        response = self._wait(client.get_parameters(['min_velocity', 'max_velocity']), 2.0)
        values = [parameter_value_to_python(v) for v in response.values]
        if (len(values) != 2 or any(not isinstance(v, list) or len(v) != 3 for v in values)
                or any(isinstance(x, bool) or not isinstance(x, (int, float))
                       or not math.isfinite(x) for v in values for x in v)):
            return False
        limit = self.parking_contract['desired_linear_mps']
        return (math.isclose(values[0][0], -limit, abs_tol=1e-9)
                and 0.0 < values[1][0] <= limit)

    def _reverse_path_valid(self, path):
        points = [(item.pose.position.x, item.pose.position.y,
                   _quaternion_yaw(item.pose.orientation)) for item in path.poses]
        if not static_corridor_clear(
                Path(self.registry['map']['yaml_path']),
                Path(self.registry['keepout']['yaml_path']), points,
                self._reverse_footprint()):
            self.emit('reverse_path_checked', valid=False,
                      reason='static_obstacle_unknown_keepout_or_map_boundary')
            return False
        if not self.validate_reverse_path.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('reverse path collision validation unavailable')
        request = IsPathValid.Request()
        request.path = path
        result = self._wait(self.validate_reverse_path.call_async(request), 3.0)
        self.emit('reverse_path_checked', valid=result.is_valid,
                  invalid_pose_indices=list(result.invalid_pose_indices))
        return result.is_valid and not result.invalid_pose_indices

    def _reverse_footprint(self):
        """Use matching runtime footprints, including their configured padding."""
        footprints = []
        for name in ('global_costmap/global_costmap', 'local_costmap/local_costmap'):
            client = AsyncParameterClient(self, name)
            if not client.wait_for_services(timeout_sec=2.0):
                raise RuntimeError(f'{name} footprint unavailable')
            response = self._wait(client.get_parameters(
                ['footprint', 'footprint_padding', 'robot_base_frame']), 2.0)
            values = [parameter_value_to_python(v) for v in response.values]
            if len(values) != 3 or values[2] != 'base_footprint':
                raise RuntimeError('reverse footprint frame mismatch')
            polygon = ast.literal_eval(values[0]) if isinstance(values[0], str) else None
            padding = values[1]
            if (not isinstance(polygon, (tuple, list)) or len(polygon) < 3
                    or isinstance(padding, bool) or not isinstance(padding, (int, float))
                    or not math.isfinite(padding) or padding < 0.0):
                raise RuntimeError('reverse footprint geometry unavailable')
            if any(not isinstance(point, (tuple, list)) or len(point) != 2
                   or any(isinstance(v, bool) or not isinstance(v, (int, float))
                          or not math.isfinite(v) for v in point) for point in polygon):
                raise RuntimeError('reverse footprint geometry invalid')
            footprints.append((polygon, padding))
        if footprints[0] != footprints[1]:
            raise RuntimeError('global and local footprints disagree')
        polygon, padding = footprints[0]
        # A bounding rectangle is conservative for the configured base polygon.
        xmin = min(p[0] for p in polygon) - padding
        xmax = max(p[0] for p in polygon) + padding
        ymin = min(p[1] for p in polygon) - padding
        ymax = max(p[1] for p in polygon) + padding
        return [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]

    def _go_home_reverse(self, pose, execute):
        """Align outside the dock, then follow a bounded reverse path via Nav2."""
        full_config = route_config(self.registry, pose)
        self.config, self.waypoints = full_config, full_config['waypoints']
        target = tuple(pose[key] for key in ('x_m', 'y_m', 'yaw_rad'))
        actual = None
        if (execute and getattr(self, 'parking_command', None) is not None
                and getattr(self, 'parking_odom', None) is not None):
            try:
                actual = self._parking_observation()['actual_pose']
            except (TransformException, ValueError) as error:
                self.emit('home_stationary_shortcut_unavailable', reason=str(error))
        if actual is not None:
            distance, angle = pose_errors(target, actual)
            if (distance <= self.parking_contract['xy_tolerance_m']
                    and angle <= self.parking_contract['yaw_tolerance_rad']):
                success = self._verify_parking_stop(1, self.waypoints[-1], None)
                self.emit('home_arrived' if success else 'failed',
                          already_at_home=True, confirmation=self.confirmation)
                return success
        stage = full_config['waypoints'][0]
        stage_pose = {**pose, 'id': stage['id'], 'x_m': stage['x'], 'y_m': stage['y'],
                      'parking_direction': 'forward'}
        nominal_path = self._make_reverse_path((stage['x'], stage['y'], stage['yaw']), target)
        if not self._reverse_path_valid(nominal_path):
            return False
        try:
            planned = self.plan_pose(stage_pose, single=True)
            self.emit('reverse_staging_planned', attempt=planned)
            if not planned['ok']:
                return False
            if not execute:
                self.emit('home_planned_only', parking_direction='reverse',
                          final_path_validation='RECHECK_ACTUAL_PATH_AT_STAGING')
                return True
            if not self.execute():
                return False
        finally:
            self.config, self.waypoints = full_config, full_config['waypoints']
            self.pose_id = pose['id']
        # The actual stationary staging pose, not an assumed waypoint, seeds
        # the path. Reject misalignment before any reverse action is sent.
        self.verify_live_maps()
        actual, _evidence = self.capture_stationary_pose()
        path = self._make_reverse_path(actual, target)
        if not self._reverse_path_valid(path):
            return False
        success = self._execute_reverse_path(path)
        self.emit('home_arrived' if success else 'failed',
                  parking_direction='reverse', confirmation=self.confirmation)
        return success

    def _execute_reverse_path(self, path):
        """Keep reverse motion in controller → smoother → monitor → base."""
        if (self.stop_requested or not self._navigation_ready(require_fresh_amcl=False)
                or not self._departure_battery_ready()
                or not self.follow_reverse.wait_for_server(timeout_sec=2.0)):
            return False
        self.active_action_type = FollowPath
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'ParkingReverse'
        goal.goal_checker_id = 'parking_goal_checker'
        goal.progress_checker_id = 'progress_checker'
        self.navigation_uuid = FollowPath.Impl.SendGoalService.Request().goal_id
        self.navigation_uuid.uuid = list(uuid.uuid4().bytes)
        self.pending_goal = self.follow_reverse.send_goal_async(
            goal, goal_uuid=self.navigation_uuid)
        try:
            handle = self._wait(self.pending_goal, 5.0)
            self.pending_goal = None
            if not handle.accepted:
                return False
            self._route_event('accepted', 1, handle, motion='reverse')
            self.navigation_result = handle.get_result_async()
            while not self.navigation_result.done():
                rclpy.spin_once(self, timeout_sec=0.05)
                if (self.stop_requested
                        or not self._navigation_ready(require_fresh_amcl=False)):
                    return False
            wrapped = self.navigation_result.result()
            self._route_event('result', 1, handle, motion='reverse',
                              terminal_status_code=int(wrapped.status),
                              nav2_error_code=int(wrapped.result.error_code))
            self.navigation_result = None
            if (self.stop_requested or wrapped.status != GoalStatus.STATUS_SUCCEEDED
                    or wrapped.result.error_code):
                return False
            return self._verify_parking_stop(1, self.waypoints[-1], handle)
        finally:
            if not self.finish_navigation():
                raise RuntimeError('reverse action cancellation unconfirmed')

    def wait_parked(self, dwell_s=5.0):
        """Count dwell only while the selected pose remains stopped and aligned."""
        if self.selected_pose is None:
            raise ValueError('parking dwell requires a selected destination')
        index = len(self.waypoints) - 1
        pose = self.selected_pose
        waypoint = {'x': pose['x_m'], 'y': pose['y_m'], 'yaw': pose['yaw_rad']}
        # A navigation deadline must not expire during the subsequent dwell.
        self.run_deadline_s = None
        self.emit('parked_dwell_started', dwell_s=dwell_s)
        success = self._verify_parking_stop(index, waypoint, None, hold_s=dwell_s)
        self.emit('parked_dwell_complete' if success else 'parked_dwell_failed',
                  dwell_s=dwell_s, confirmation=self.confirmation)
        return success

    def serve(self, table_id, execute=False):
        """Dock at home, hold five seconds, serve one table, then dock home."""
        # Resolve every required destination before dispatching the first goal.
        home_pose(self.registry)
        candidates(self.registry, table_id)
        self.emit('serving_started', selected_table=table_id, dwell_s=5.0,
                  pour_stage=False, execute=execute)
        steps = [
            ('initial_home', lambda: self.go_home(execute=execute)),
            ('home_dwell', lambda: self.wait_parked(5.0) if execute else True),
            ('table', lambda: self.visit(table_id, execute=execute)),
            ('table_dwell', lambda: self.wait_parked(5.0) if execute else True),
            ('return_home', lambda: self.go_home(execute=execute)),
        ]
        for phase, action in steps:
            if self.stop_requested:
                self.emit('serving_failed', phase=phase, reason='operator_or_timeout')
                return False
            self.emit('serving_phase_started', phase=phase)
            if not action():
                self.emit('serving_failed', phase=phase)
                return False
        self.run_deadline_s = None
        self.emit('serving_complete' if execute else 'serving_plan_checks_complete',
                  selected_table=table_id, physical_accuracy='NOT_MEASURED')
        return True

    def roundtrip(self, table_ids, execute=False, dwell_s=20.0,
                  box_timeout_s=45.0):
        """Visit selected box stations in order, then return to taught home."""
        destinations = [table_ids] if isinstance(table_ids, str) else list(table_ids)
        if (not destinations or any(
                not isinstance(item, str) or not item for item in destinations)):
            raise ValueError('roundtrip requires at least one destination ID')
        if len(destinations) != len(set(destinations)):
            raise ValueError('roundtrip destination IDs must be unique')
        home = home_pose(self.registry)
        self.emit('roundtrip_started', destination_ids=destinations,
                  home_pose=[home[key] for key in ('x_m', 'y_m', 'yaw_rad')],
                  dwell_s=float(dwell_s))
        for index, table_id in enumerate(destinations):
            self.emit('station_started', station_id=table_id, station_index=index)
            if not self.visit(table_id, execute=execute):
                self.emit('roundtrip_failed', phase='destination',
                          station_id=table_id, station_index=index)
                return False
            if execute and not self.wait_for_box(dwell_s, box_timeout_s):
                self.emit('roundtrip_failed', phase='box_wait',
                          station_id=table_id, station_index=index)
                return False
            self.emit('station_complete', station_id=table_id, station_index=index)
        if not self.go_home(execute=execute):
            self.emit('roundtrip_failed', phase='home')
            return False
        self.emit('roundtrip_complete' if execute else 'roundtrip_planned')
        return True


def parse_args(argv):
    """Keep registry maintenance, read-only planning and execution explicit."""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    initialize = commands.add_parser('init')
    initialize.add_argument('--map', type=Path, required=True)
    initialize.add_argument('--keepout', type=Path, required=True)
    initialize.add_argument('--registry', type=Path, required=True)
    listing = commands.add_parser('list')
    listing.add_argument('--registry', type=Path, required=True)
    for command in ('teach', 'go', 'teach-home', 'home', 'roundtrip', 'serve'):
        subparser = commands.add_parser(command)
        subparser.add_argument('--registry', type=Path, required=True)
        subparser.add_argument('--log', type=Path, required=True)
        if command == 'roundtrip':
            subparser.add_argument(
                '--table-id', dest='table_ids', action='append', required=True,
                help='Ordered destination ID; repeat this option for each station')
        elif command not in ('teach-home', 'home'):
            subparser.add_argument('--table-id', required=True)
        if command == 'teach':
            subparser.add_argument('--pose-id', required=True)
            subparser.add_argument('--priority', type=int, choices=(1, 2), default=1)
            subparser.add_argument('--approach-offset-m', type=float, default=0.5)
            subparser.add_argument('--replace', action='store_true')
            subparser.add_argument('--target-front-gap-m', type=float,
                                   help='Requested chassis-front gap; does not shift taught pose')
            subparser.add_argument('--measured-front-gap-m', type=float,
                                   help='Gap measured at this stationary teaching pose')
            subparser.add_argument('--gap-measurement-note',
                                   help='Measurement method and physical reference')
        elif command == 'teach-home':
            subparser.add_argument('--pose-id', default='home_dock')
            subparser.add_argument('--approach-offset-m', type=float, default=0.7)
            subparser.add_argument('--replace', action='store_true')
            subparser.add_argument('--parking-direction', choices=('forward', 'reverse'),
                                   default='forward')
        else:
            subparser.add_argument('--execute', action='store_true')
            if command == 'roundtrip':
                subparser.add_argument('--dwell-s', type=float, default=20.0)
                subparser.add_argument('--box-timeout-s', type=float, default=45.0)
    parsed = parser.parse_args(remove_ros_args(args=argv)[1:])
    if parsed.command == 'teach':
        try:
            validate_gap_measurement(parsed.measured_front_gap_m, parsed.gap_measurement_note)
            if parsed.target_front_gap_m is not None and (
                    not math.isfinite(parsed.target_front_gap_m)
                    or parsed.target_front_gap_m <= 0):
                raise ValueError('target_front_gap_m must be finite and positive')
        except ValueError as error:
            parser.error(str(error))
    if parsed.command == 'roundtrip':
        if (not math.isfinite(parsed.dwell_s) or parsed.dwell_s <= 0.0
                or not math.isfinite(parsed.box_timeout_s)
                or parsed.box_timeout_s <= parsed.dwell_s):
            parser.error('roundtrip box timeout must exceed a positive dwell')
        if len(parsed.table_ids) != len(set(parsed.table_ids)):
            parser.error('roundtrip destination IDs must be unique')
    return parsed


def main(args=None):
    """Operate on the local Nav2 graph only when a ROS command was requested."""
    argv = args if args is not None else sys.argv
    parsed = parse_args(argv)
    node = None
    ros_started = False
    handlers = {}
    try:
        if parsed.command == 'init':
            save_registry(parsed.registry, new_registry(parsed.map, parsed.keepout))
            print(f'created: {parsed.registry}')
            return
        registry = load_registry(parsed.registry)
        if parsed.command == 'list':
            print(json.dumps(registry, ensure_ascii=False, indent=2, allow_nan=False))
            return
        contract_path = (Path(get_package_share_directory('jdamr_cube_navigation'))
                         / 'config/parking_contract.yaml')
        contract = load_parking_contract(contract_path)
        with parsed.log.open('x', encoding='utf-8') as stream:
            rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
            ros_started = True
            node = ServiceRoute(registry, contract, stream)
            for signum in (signal.SIGINT, signal.SIGTERM):
                handlers[signum] = signal.signal(signum, lambda *_: node.request_stop())
            try:
                if parsed.command in ('teach', 'teach-home'):
                    table_id = parsed.table_id if parsed.command == 'teach' else 'home_dock'
                    node.table_id, node.pose_id = table_id, parsed.pose_id
                    node.verify_live_maps()
                    actual_pose, observation = node.capture_stationary_pose()
                    if parsed.command == 'teach':
                        pose = taught_pose(parsed.pose_id, actual_pose, observation,
                                           parsed.priority, parsed.approach_offset_m,
                                           parsed.measured_front_gap_m,
                                           parsed.gap_measurement_note,
                                           parsed.target_front_gap_m)
                        updated = add_pose(
                            registry, parsed.table_id, pose, parsed.replace)
                    else:
                        pose = taught_pose(
                            parsed.pose_id, actual_pose, observation,
                            approach_offset_m=parsed.approach_offset_m)
                        pose['parking_direction'] = parsed.parking_direction
                        updated = set_home_pose(registry, pose, parsed.replace)
                    # Detect an intervening edit before replacing the registry.
                    if load_registry(parsed.registry) != registry:
                        raise RuntimeError('registry changed during teaching; capture not saved')
                    save_registry(parsed.registry, updated, replace=True)
                    node.selected_pose = pose
                    node.emit('taught', pose=pose)
                    ok = True
                elif parsed.command == 'go':
                    ok = node.visit(parsed.table_id, parsed.execute)
                elif parsed.command == 'serve':
                    ok = node.serve(parsed.table_id, parsed.execute)
                elif parsed.command == 'home':
                    ok = node.go_home(parsed.execute)
                else:
                    ok = node.roundtrip(
                        parsed.table_ids, parsed.execute,
                        parsed.dwell_s, parsed.box_timeout_s)
            except Exception as error:
                node.emit('failed', reason=f'{type(error).__name__}: {error}')
                raise
            finally:
                if not node.finish_navigation():
                    raise RuntimeError(
                        'navigation cancellation unconfirmed; inspect the robot stop state')
            if not ok:
                raise RuntimeError('service task did not complete; see JSONL log')
    except (OSError, KeyError, TypeError, ValueError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1) from error
    finally:
        for signum, handler in handlers.items():
            signal.signal(signum, handler)
        if node is not None:
            node.destroy_node()
        if ros_started:
            rclpy.shutdown()


if __name__ == '__main__':
    main()
