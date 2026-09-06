"""Drive one simulation-only Nav2 fixed-obstacle evidence scenario."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import time
from collections import deque  # noqa: I100
from pathlib import Path  # noqa: I100
from typing import Any

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, Twist

from jdamr_cube_navigation.costmap_evidence import (
    blocking_probe_from_endpoints, canonical_grid_key,
    excess_blocking_cells, obstacle_angular_window_samples,
    obstacle_surface_endpoints, roi_cell_samples, roi_statistics,
    roi_statistics_from_samples, transform_point_2d,
    transformed_roi_cell_samples)

from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import ClearEntireCostmap

from nav_msgs.msg import Odometry, Path as NavPath

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # pragma: no cover - reported as an environment blocker
    Contacts = None


SET_POSE_SERVICE = '/world/slam_corridor/set_pose'
ACTION_TERMINALS = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'cancelled',
}
SCENARIO_OBSTACLES = {
    'baseline': None,
    'detour': {'x_m': -1.0, 'y_m': 0.0,
               'length_m': 0.50, 'width_m': 0.40},
    'event_driven_removal': {'x_m': -1.0, 'y_m': 0.0,
                             'length_m': 0.50, 'width_m': 0.40},
    'full_block': {'x_m': -1.0, 'y_m': 0.0,
                   'length_m': 0.50, 'width_m': 2.30},
    'goal_occupied': {'x_m': 6.0, 'y_m': 0.0,
                      'length_m': 1.70, 'width_m': 1.70},
}
PREFLIGHT_SCENARIOS = {'sub_minimum_probe', 'front_observation_probe'}
MAX_CLEAR_OBSERVATION_TRACE = 64
PRELOADED_ROLE_BY_SCENARIO = {
    'detour': 'route',
    'event_driven_removal': 'route',
    'full_block': 'full_block',
    'goal_occupied': 'goal_occupied',
    'sub_minimum_probe': 'sub_minimum_probe',
    'front_observation_probe': 'front_observation_probe',
}


def parse_entity_pose_info(output: str, entity_name: str) -> dict[str, Any]:
    """Extract one model's stable entity ID and position from Pose_V JSON."""
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
        for pose in document.get('pose', []):
            if pose.get('name') == entity_name:
                matches.append(pose)
    if matches:
        entity_ids = {int(pose['id']) for pose in matches}
        if len(entity_ids) != 1:
            raise ValueError(
                f'entity ID changed across pose/info samples: {entity_name}')
        position = matches[-1].get('position', {})
        pose_m = [
            float(position.get('x', 0.0)),
            float(position.get('y', 0.0)),
            float(position.get('z', 0.0)),
        ]
        if not all(math.isfinite(value) for value in pose_m):
            raise ValueError(f'non-finite pose/info position: {entity_name}')
        return {'entity_id': entity_ids.pop(), 'pose_m': pose_m}
    raise ValueError(f'entity not found in pose/info: {entity_name}')


def read_entity_pose(
        entity_name: str, timeout_s: float = 5.0,
        environment: dict[str, str] | None = None) -> dict[str, Any]:
    """Read one fresh Gazebo Pose_V sample for a preloaded model."""
    result = subprocess.run([
        'gz', 'topic', '-e', '-t', '/world/slam_corridor/pose/info',
        '-n', '1', '--json-output',
    ], env=environment, capture_output=True, text=True,
        timeout=timeout_s, check=False)
    if result.returncode != 0 or not result.stdout.strip():
        raise RuntimeError(
            result.stderr.strip() or 'empty Gazebo pose/info sample')
    return parse_entity_pose_info(result.stdout, entity_name)


def _is_zero(message: Twist, tolerance: float = 1e-4) -> bool:
    return all(abs(value) <= tolerance for value in (
        message.linear.x, message.linear.y, message.linear.z,
        message.angular.x, message.angular.y, message.angular.z))


class ScenarioObserver(Node):
    """Observe Nav2 evidence and manipulate only Gazebo scenario entities."""

    def __init__(self, args: argparse.Namespace, contract: dict[str, Any],
                 scenario_spec: dict[str, Any]):
        """Bind the observer to one generated run contract."""
        super().__init__('sim_nav_obstacle_scenario')
        self.args = args
        self.contract = contract
        self.scenario_spec = scenario_spec
        self.obstacle = scenario_spec['obstacle']
        reference_margin_m = (
            contract['robot_circumscribed_radius_m']
            + contract['costmap_resolution_m'] / math.sqrt(2.0))
        self.reference_costmap_probe = ({
            **self.obstacle,
            'length_m': self.obstacle['length_m'] + 2.0 * reference_margin_m,
            'width_m': self.obstacle['width_m'] + 2.0 * reference_margin_m,
        } if self.obstacle is not None else None)
        influence_margin_m = contract['costmap_resolution_m'] / math.sqrt(2.0)
        self.obstacle_influence_probe = ({
            **self.obstacle,
            'length_m': self.obstacle['length_m'] + 2.0 * influence_margin_m,
            'width_m': self.obstacle['width_m'] + 2.0 * influence_margin_m,
        } if self.obstacle is not None else None)
        self.laser_yaw_in_base_rad = float(contract[
            'simulation_support_plane_correction']['scan_preflight_contract'][
                'laser_yaw_in_base_rad'])
        role = PRELOADED_ROLE_BY_SCENARIO.get(args.scenario)
        self.preloaded_spec = (
            contract['preloaded_obstacles']['models'][role]
            if role is not None else None)
        self.entity_name = (
            self.preloaded_spec['name']
            if self.preloaded_spec is not None else None)
        self.contact_topic = (
            f'/world/slam_corridor/model/{self.entity_name}/link/body/'
            'sensor/contact_sensor/contact' if self.entity_name else '')
        self.events: list[dict[str, Any]] = []
        self.global_blocking = False
        self.local_blocking = False
        self.global_cleared = False
        self.local_cleared = False
        self.clear_mode: str | None = None
        self.clear_observation_counts = {'global': 0, 'local': 0}
        self.clear_service_request_counts = {'global': 0, 'local': 0}
        self.clear_service_response_counts = {'global': 0, 'local': 0}
        self.clear_service_futures: dict[str, Any] = {}
        self.global_baseline_snapshots: list[dict[str, Any]] = []
        self.global_baseline_ranks: dict[tuple[int, int], int] = {}
        self.global_baseline_geometry: dict[str, Any] | None = None
        self.baseline_ready = args.scenario != 'event_driven_removal'
        self.mark_excess_snapshots: dict[str, dict[str, Any]] = {}
        self.pending_local_costmaps = deque()
        self.pending_local_costmap_failures: list[dict[str, Any]] = []
        self.new_scan_after_deactivate_s: float | None = None
        self.recovery_started_s: float | None = None
        self.recovery_responses_complete_s: float | None = None
        self.clear_epoch = 0
        self.final_clear_completion_s: float | None = None
        self.final_post_clear_plan_s: float | None = None
        self.clear_completion_history: list[dict[str, Any]] = []
        self.post_clear_plan_history: list[dict[str, Any]] = []
        self.final_clear_samples: dict[str, dict[str, Any]] = {}
        self.removal_passage: dict[str, Any] | None = None
        self.ground_truth_message_count = 0
        self.activated = False
        self.deactivated = False
        self.new_scan_after_deactivate = False
        self.mark_complete_s: float | None = None
        self.first_zero_s: float | None = None
        self.action_terminal = 'unknown'
        self.action_terminal_s: float | None = None
        self.zero_since_s: float | None = None
        self.contact_count = 0
        self.contact_matched_publisher_count_max = 0
        self.costmap_message_counts = {'global': 0, 'local': 0}
        self.costmap_counts_at_deactivate: dict[str, int] | None = None
        self.costmap_max_roi_cost = {'global': -1, 'local': -1}
        self.costmap_roi_statistics: dict[str, dict[str, Any]] = {}
        self.costmap_surface_roi_statistics: dict[str, dict[str, Any]] = {}
        self.clear_observation_trace: list[dict[str, Any]] = []
        self.clear_observation_sequence_counts = {'global': 0, 'local': 0}
        self.clear_roi_was_observable = {'global': False, 'local': False}
        self.plan_message_count = 0
        self.scan_message_count = 0
        self.last_scan_stamp_ns: int | None = None
        self.deactivate_scan_stamp_ns: int | None = None
        self.obstacle_surface_beam_count = 0
        self.activation_reference_scan_stamp_ns: int | None = None
        self.activation_surface_scan: dict[str, Any] | None = None
        self.mark_eligible_scan: dict[str, Any] | None = None
        self.mark_eligible_costmap_probe: dict[str, float] | None = None
        self.clear_costmap_probe: dict[str, float] | None = None
        self.post_deactivate_scan_window: dict[str, Any] | None = None
        self.robot_pose_xy_yaw: tuple[float, float, float] | None = None
        self.odom_pose_xy_yaw: tuple[float, float, float] | None = None
        self.minimum_robot_obstacle_center_distance_m: float | None = None
        self.minimum_robot_obstacle_aabb_distance_m: float | None = None
        self.minimum_pre_deactivate_aabb_distance_m: float | None = None
        self.plans_after_mark: list[list[list[float]]] = []
        self.plans_before_mark: list[list[list[float]]] = []
        self.start_s = time.monotonic()
        self.finished = False
        self.activation_error: str | None = None
        self.entity_states: list[dict[str, Any]] = []
        self.action_client = ActionClient(self, NavigateToPose,
                                          'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(
            contract['local_update_period_s'],
            self._poll_pending_local_costmaps)
        self.clear_clients = {
            'global': self.create_client(
                ClearEntireCostmap,
                '/global_costmap/clear_entirely_global_costmap'),
            'local': self.create_client(
                ClearEntireCostmap,
                '/local_costmap/clear_entirely_local_costmap'),
        }
        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Costmap, '/global_costmap/costmap_raw', self._global_costmap,
            costmap_qos)
        self.create_subscription(
            Costmap, '/local_costmap/costmap_raw', self._local_costmap,
            costmap_qos)
        self.create_subscription(NavPath, '/plan', self._plan, 10)
        self.create_subscription(LaserScan, '/scan', self._scan, 10)
        self.create_subscription(
            PoseStamped, '/ground_truth_pose', self._ground_truth, 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel, 10)
        if Contacts is not None and self.contact_topic:
            self.create_subscription(
                Contacts, self.contact_topic, self._contacts, 10)

    def _event(self, name: str, **details: Any) -> None:
        if any(item['name'] == name for item in self.events):
            return
        self.events.append({
            'name': name,
            'elapsed_s': time.monotonic() - self.start_s,
            **details,
        })

    def _set_entity_pose(self, pose_m: list[float]) -> bool:
        request = (
            f'name: "{self.entity_name}", position {{x: {pose_m[0]}, '
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
        return True

    def _capture_entity_state(
            self, label: str, expected_pose_m: list[float]) -> bool:
        deadline_s = time.monotonic() + 5.0
        last_error = ''
        while time.monotonic() < deadline_s:
            try:
                state = read_entity_pose(self.entity_name, timeout_s=2.0)
            except (RuntimeError, ValueError, json.JSONDecodeError,
                    subprocess.TimeoutExpired) as error:
                last_error = str(error)
                continue
            error_m = math.dist(state['pose_m'], expected_pose_m)
            if error_m <= 0.02:
                state.update({
                    'label': label,
                    'expected_pose_m': expected_pose_m,
                    'position_error_m': error_m,
                })
                self.entity_states.append(state)
                entity_ids = {
                    item['entity_id'] for item in self.entity_states}
                if len(entity_ids) != 1:
                    self.activation_error = 'preloaded entity ID changed'
                    return False
                return True
            last_error = f'pose error {error_m:.6f}m'
        self.activation_error = (
            f'entity pose verification failed: {last_error}')
        return False

    def activate(self) -> bool:
        """Move the preloaded model from its park pose to the active pose."""
        if self.obstacle is None:
            return True
        if not self._capture_entity_state(
                'park_before_activate', self.preloaded_spec['park_pose_m']):
            return False
        self.activation_reference_scan_stamp_ns = self.last_scan_stamp_ns
        activated = self._set_entity_pose(self.preloaded_spec['active_pose_m'])
        activated = activated and self._capture_entity_state(
            'active', self.preloaded_spec['active_pose_m'])
        if activated:
            self.activated = True
            self._event('activate_ack')
        return activated

    def deactivate(self) -> bool:
        """Move the active model back to its verified park pose."""
        deactivated = self._set_entity_pose(self.preloaded_spec['park_pose_m'])
        deactivated = deactivated and self._capture_entity_state(
            'park_after_deactivate', self.preloaded_spec['park_pose_m'])
        if deactivated:
            self.deactivate_scan_stamp_ns = self.last_scan_stamp_ns
            self.costmap_counts_at_deactivate = dict(
                self.costmap_message_counts)
            self.deactivated = True
            self._event('deactivate_ack')
        return deactivated

    def send_goal(self) -> bool:
        """Send the fixed goal with the evaluation-only behavior tree."""
        if not self.action_client.wait_for_server(timeout_sec=30.0):
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.args.goal_x_m
        goal.pose.pose.position.y = self.args.goal_y_m
        goal.pose.pose.orientation.z = math.sin(self.args.goal_yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.args.goal_yaw_rad / 2.0)
        goal.behavior_tree = str(self.args.behavior_tree.resolve())
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.action_terminal = 'rejected'
            self.action_terminal_s = time.monotonic()
            return False
        self._event('goal_accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._action_result)
        return True

    def _action_result(self, future) -> None:
        result = future.result()
        self.action_terminal = ACTION_TERMINALS.get(
            result.status, f'status_{result.status}')
        self.action_terminal_s = time.monotonic()
        self._event(self.action_terminal)
        if self.args.scenario == 'event_driven_removal':
            if not (self.global_cleared and self.local_cleared
                    and self.final_clear_completion_s is not None
                    and self.final_post_clear_plan_s is not None
                    and self.removal_passage is not None
                    and self.final_clear_completion_s
                    < self.final_post_clear_plan_s
                    < self.action_terminal_s):
                self.activation_error = 'action_terminal_before_clear_complete'

    @staticmethod
    def _stamp_ns(message: Costmap) -> int:
        stamp = message.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _capture_global_baseline(self, message: Costmap) -> None:
        statistics = roi_statistics(message, self.reference_costmap_probe)
        if (statistics['sampled_cell_count'] == 0
                or statistics['unknown_count'] > 0):
            self.activation_error = 'baseline_empty_or_unknown'
            return
        metadata = statistics['metadata']
        cells = roi_cell_samples(
            message, self.reference_costmap_probe)['cells']
        ranks = {
            canonical_grid_key(
                cell['center_x_m'], cell['center_y_m'],
                metadata['origin_x_m'], metadata['origin_y_m'],
                metadata['resolution_m']):
            (1 if cell['cost'] == 253 else 2 if cell['cost'] == 254 else 0)
            for cell in cells
        }
        geometry = {
            **metadata,
            'index_bounds': statistics['index_bounds'],
            'reference_probe': self.reference_costmap_probe,
        }
        canonical = sorted(
            (key[0], key[1], rank) for key, rank in ranks.items())
        digest = hashlib.sha256(json.dumps(
            {'geometry': geometry, 'classes': canonical},
            sort_keys=True).encode()).hexdigest()
        snapshot = {
            'stamp_ns': self._stamp_ns(message),
            'geometry': geometry,
            'canonical_class_sha256': digest,
            'sampled_cell_count': statistics['sampled_cell_count'],
            'unknown_count': statistics['unknown_count'],
        }
        if self.global_baseline_snapshots:
            previous = self.global_baseline_snapshots[-1]
            if (snapshot['stamp_ns'] <= previous['stamp_ns']
                    or snapshot['geometry'] != previous['geometry']
                    or snapshot['canonical_class_sha256']
                    != previous['canonical_class_sha256']):
                self.activation_error = 'baseline_unstable'
                return
        self.global_baseline_snapshots.append(snapshot)
        if len(self.global_baseline_snapshots) == 2:
            self.global_baseline_ranks = ranks
            self.global_baseline_geometry = geometry
            self.baseline_ready = True

    def _map_from_costmap_transform(self, message: Costmap):
        frame_id = message.header.frame_id
        if frame_id == 'map':
            return None, self._stamp_ns(message)
        stamp = Time.from_msg(message.header.stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', frame_id, stamp, timeout=Duration(seconds=0.0))
        except TransformException:
            return False, None
        transform_stamp_ns = self._stamp_ns(transform)
        if transform_stamp_ns != self._stamp_ns(message):
            if self.args.scenario == 'event_driven_removal':
                self.activation_error = 'costmap_tf_stamp_mismatch'
            return False, transform_stamp_ns
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw_rad = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))
        return (lambda x_m, y_m: transform_point_2d(
            x_m, y_m, translation.x, translation.y, yaw_rad)), (
                transform_stamp_ns)

    def _costmap(self, source: str, message: Costmap) -> None:
        if self.obstacle is None:
            return
        self.costmap_message_counts[source] += 1
        if (self.args.scenario == 'event_driven_removal'
                and source == 'global' and not self.activated
                and self.last_scan_stamp_ns is not None
                and self.robot_pose_xy_yaw is not None
                and len(self.global_baseline_snapshots) < 2):
            self._capture_global_baseline(message)
            return
        if source == 'local':
            if not self.activated:
                return
            transform, transform_stamp_ns = self._map_from_costmap_transform(
                message)
            if transform is False:
                if len(self.pending_local_costmaps) >= self.contract[
                        'pending_local_costmap_limit']:
                    dropped_s, dropped = self.pending_local_costmaps.popleft()
                    self.pending_local_costmap_failures.append({
                        'stamp_ns': self._stamp_ns(dropped),
                        'elapsed_s': time.monotonic() - self.start_s,
                        'age_s': time.monotonic() - dropped_s,
                        'reason': 'exact_stamp_tf_queue_overflow_drop',
                    })
                    if self.args.scenario == 'event_driven_removal':
                        self.activation_error = (
                            'pending_local_costmap_overflow')
                        return
                self.pending_local_costmaps.append((time.monotonic(), message))
                return
        else:
            transform, transform_stamp_ns = None, self._stamp_ns(message)
        self._process_costmap(
            source, message, transform, transform_stamp_ns)

    def _process_costmap(
            self, source: str, message: Costmap, transform,
            transform_stamp_ns: int | None) -> None:
        padded_obstacle = self.reference_costmap_probe
        surface_obstacle = self.clear_costmap_probe or padded_obstacle
        if transform is None:
            padded_samples = roi_cell_samples(message, padded_obstacle)
            surface_samples = roi_cell_samples(message, surface_obstacle)
            influence_samples = roi_cell_samples(
                message, self.obstacle_influence_probe)
        else:
            padded_samples = transformed_roi_cell_samples(
                message, padded_obstacle, transform)
            surface_samples = transformed_roi_cell_samples(
                message, surface_obstacle, transform)
            influence_samples = transformed_roi_cell_samples(
                message, self.obstacle_influence_probe, transform)
        padded_statistics = roi_statistics_from_samples(
            message, padded_samples)
        surface_statistics = roi_statistics_from_samples(
            message, surface_samples)
        influence_statistics = roi_statistics_from_samples(
            message, influence_samples)
        blocking_cells = [
            cell for cell in padded_samples['cells']
            if cell['cost'] in {253, 254}]
        geometry = self.global_baseline_geometry
        excess_cells = (excess_blocking_cells(
            blocking_cells, self.global_baseline_ranks,
            geometry['origin_x_m'], geometry['origin_y_m'],
            geometry['resolution_m'])
            if self.baseline_ready and geometry is not None else [])
        influence_half_length_m = self.obstacle_influence_probe[
            'length_m'] / 2.0
        influence_half_width_m = self.obstacle_influence_probe[
            'width_m'] / 2.0
        influence_excess_cells = [
            cell for cell in excess_cells
            if (abs(cell['map_center_x_m'] - self.obstacle['x_m'])
                <= influence_half_length_m
                and abs(cell['map_center_y_m'] - self.obstacle['y_m'])
                <= influence_half_width_m)]
        padded_statistics['baseline_excess_count'] = len(excess_cells)
        padded_statistics['baseline_excess_cells'] = excess_cells[:16]
        padded_statistics['influence_excess_count'] = len(
            influence_excess_cells)
        padded_statistics['influence_excess_cells'] = (
            influence_excess_cells[:16])
        padded_statistics['influence_probe'] = influence_statistics
        padded_statistics['tf_stamp_ns'] = transform_stamp_ns
        self.costmap_roi_statistics[source] = padded_statistics
        self.costmap_surface_roi_statistics[source] = surface_statistics
        self.costmap_max_roi_cost[source] = max(
            self.costmap_max_roi_cost[source],
            padded_statistics['max_cost'])
        blocking = (bool(excess_cells) and bool(influence_excess_cells)
                    if self.args.scenario == 'event_driven_removal'
                    else padded_statistics['blocking'])
        blocking_field = f'{source}_blocking'
        marking_observation_ready = (
            self.activation_surface_scan is not None
            and self.mark_eligible_scan is not None)
        if (marking_observation_ready and blocking
                and not getattr(self, blocking_field)):
            setattr(self, blocking_field, True)
            self.clear_roi_was_observable[source] = (
                padded_statistics['sampled_cell_count'] > 0)
            self.mark_excess_snapshots[source] = {
                'stamp_ns': self._stamp_ns(message),
                'baseline_excess_count': len(excess_cells),
                'baseline_excess_cells': excess_cells[:16],
                'influence_excess_count': len(influence_excess_cells),
                'influence_excess_cells': influence_excess_cells[:16],
                'influence_roi_intersects': bool(influence_excess_cells),
            }
            self._event(blocking_field)
        if (self.args.scenario == 'event_driven_removal'
                and self.deactivated and self.new_scan_after_deactivate
                and getattr(self, blocking_field)):
            self._record_clear_observation(
                source, message, surface_statistics, padded_statistics)
        elif (self.args.scenario in PREFLIGHT_SCENARIOS
              and self.deactivated and self.new_scan_after_deactivate
              and getattr(self, blocking_field) and not blocking):
            cleared_field = f'{source}_cleared'
            if not getattr(self, cleared_field):
                setattr(self, cleared_field, True)
                self._event(cleared_field)
        required_marked = (
            self.global_blocking
            and (self.args.scenario in {'full_block', 'goal_occupied'}
                 or self.local_blocking))
        if required_marked and self.mark_complete_s is None:
            self.mark_complete_s = time.monotonic()

    def _global_costmap(self, message: Costmap) -> None:
        self._costmap('global', message)

    def _local_costmap(self, message: Costmap) -> None:
        self._costmap('local', message)

    def _poll_pending_local_costmaps(self) -> None:
        now_s = time.monotonic()
        while self.pending_local_costmaps:
            received_s, message = self.pending_local_costmaps[0]
            if now_s - received_s > self.contract['costmap_tf_timeout_s']:
                self.pending_local_costmaps.popleft()
                self.pending_local_costmap_failures.append({
                    'stamp_ns': self._stamp_ns(message),
                    'elapsed_s': now_s - self.start_s,
                    'age_s': now_s - received_s,
                    'reason': 'exact_stamp_tf_snapshot_expired_drop',
                })
                if self.args.scenario == 'event_driven_removal':
                    self.activation_error = 'costmap_exact_tf_timeout'
                    return
                continue
            transform, transform_stamp_ns = self._map_from_costmap_transform(
                message)
            if transform is False:
                return
            self.pending_local_costmaps.popleft()
            self._process_costmap(
                'local', message, transform, transform_stamp_ns)

    def _record_clear_observation(
            self, source: str, message: Costmap,
            surface_statistics: dict[str, Any],
            padded_statistics: dict[str, Any]) -> None:
        """Require two consecutive clear updates in the active clear phase."""
        if (self.clear_mode == 'bounded_recovery'
                and self.recovery_responses_complete_s is None):
            return
        previous_count = self.clear_observation_counts[source]
        influence_statistics = padded_statistics['influence_probe']
        observable = (
            surface_statistics['sampled_cell_count'] > 0
            and influence_statistics['sampled_cell_count'] > 0
            and surface_statistics['unknown_count'] == 0
            and influence_statistics['unknown_count'] == 0)
        blocking = (
            surface_statistics['blocking']
            or padded_statistics['influence_excess_count'] > 0)
        background_excess = (
            padded_statistics['baseline_excess_count'] > 0 and not blocking)
        if observable:
            self.clear_roi_was_observable[source] = True
        if self.removal_passage is not None:
            decision = ('post_passage_diagnostic_background_excess'
                        if background_excess
                        else 'post_passage_diagnostic')
        elif (source == 'local' and not observable
                and self.clear_roi_was_observable[source]):
            self.clear_observation_counts[source] = 0
            self.local_cleared = False
            self.final_clear_completion_s = None
            self.final_post_clear_plan_s = None
            self.final_clear_samples.pop(source, None)
            self.activation_error = 'local_roi_evicted_before_clear'
            decision = 'evicted'
        elif not observable:
            self.clear_observation_counts[source] = 0
            if previous_count > 0:
                setattr(self, f'{source}_cleared', False)
                self.final_clear_completion_s = None
                self.final_post_clear_plan_s = None
                self.final_clear_samples.pop(source, None)
            decision = 'ignored'
        elif blocking:
            self.clear_observation_counts[source] = 0
            decision = 'reset'
            setattr(self, f'{source}_cleared', False)
            self.final_clear_completion_s = None
            self.final_post_clear_plan_s = None
            self.final_clear_samples.pop(source, None)
            if previous_count > 0:
                self._event(f'{source}_clear_reset')
        elif background_excess:
            self.clear_observation_counts[source] = min(
                previous_count + 1,
                self.contract['clear_observation_count'])
            decision = 'diagnostic_background_excess'
        else:
            self.clear_observation_counts[source] = min(
                previous_count + 1,
                self.contract['clear_observation_count'])
            decision = 'increment'
        required_count = self.contract['clear_observation_count']
        current_count = self.clear_observation_counts[source]
        self.clear_observation_sequence_counts[source] += 1
        if len(self.clear_observation_trace) >= MAX_CLEAR_OBSERVATION_TRACE:
            self.clear_observation_trace.pop(0)
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        receive_elapsed_s = time.monotonic() - self.start_s
        self.clear_observation_trace.append({
            'source': source,
            'sequence': self.clear_observation_sequence_counts[source],
            'stamp_ns': stamp_ns,
            'elapsed_s': receive_elapsed_s,
            'surface_probe': surface_statistics,
            'padded_planning_probe': padded_statistics,
            'observable': observable,
            'counter_before': previous_count,
            'counter_after': current_count,
            'decision': decision,
            'robot_pose_xy_yaw': self.robot_pose_xy_yaw,
            'odom_pose_xy_yaw': self.odom_pose_xy_yaw,
        })
        if (self.clear_mode == 'bounded_recovery'
                and self.clear_observation_counts[source] > previous_count):
            self._event(
                f'post_recovery_{source}_clear_'
                f'{self.clear_observation_counts[source]}')
        source_cleared_field = f'{source}_cleared'
        if (current_count >= required_count
                and not getattr(self, source_cleared_field)):
            setattr(self, source_cleared_field, True)
            self.final_clear_samples[source] = {
                'epoch_candidate': self.clear_epoch + 1,
                'sequence': self.clear_observation_sequence_counts[source],
                'stamp_ns': stamp_ns,
                'receive_elapsed_s': receive_elapsed_s,
            }
            self._event(source_cleared_field)
        if (all(value >= required_count
                for value in self.clear_observation_counts.values())
                and self.final_clear_completion_s is None):
            if self.clear_mode is None:
                self.clear_mode = 'passive'
            self.clear_epoch += 1
            self.final_clear_completion_s = time.monotonic()
            self.clear_completion_history.append({
                'epoch': self.clear_epoch,
                'elapsed_s': self.final_clear_completion_s - self.start_s,
                'source_sequences': {
                    name: self.clear_observation_sequence_counts[name]
                    for name in ('global', 'local')},
                'source_samples': dict(self.final_clear_samples),
            })

    def _request_bounded_clear_recovery(self) -> None:
        """Request each standard Nav2 clear service at most once."""
        if self.clear_mode is not None:
            return
        if self.action_terminal_s is not None:
            self.activation_error = 'action_terminal_before_clear_recovery'
            return
        self.clear_mode = 'bounded_recovery'
        self.recovery_started_s = time.monotonic()
        self.clear_observation_counts = {'global': 0, 'local': 0}
        self.global_cleared = False
        self.local_cleared = False
        self.final_clear_completion_s = None
        self.final_post_clear_plan_s = None
        self.final_clear_samples = {}
        self.removal_passage = None
        revoked_event_names = {
            'global_cleared', 'local_cleared', 'post_clear_new_plan',
            'far_side_passage'}
        self.events = [
            event for event in self.events
            if event['name'] not in revoked_event_names]
        self._event('passive_clear_window_expired')
        timeout_s = self.contract['clear_service_timeout_s']
        for source, client in self.clear_clients.items():
            if not client.wait_for_service(timeout_sec=timeout_s):
                self.activation_error = f'{source}_clear_service_unavailable'
                return
            self.clear_service_request_counts[source] += 1
            self._event(f'{source}_clear_requested')
            self.clear_service_futures[source] = client.call_async(
                ClearEntireCostmap.Request())

    def poll_clear_recovery(self) -> None:
        """Advance the bounded recovery without retrying any service."""
        if self.args.scenario != 'event_driven_removal':
            return
        now_s = time.monotonic()
        if (self.new_scan_after_deactivate_s is not None
                and self.clear_mode is None
                and now_s - self.new_scan_after_deactivate_s
                >= self.contract['passive_clear_window_s']):
            self._request_bounded_clear_recovery()
        if self.clear_mode != 'bounded_recovery':
            return
        for source, future in self.clear_service_futures.items():
            if (self.clear_service_response_counts[source] == 0
                    and future.done()):
                try:
                    response = future.result()
                except Exception as error:
                    self.activation_error = (
                        f'{source}_clear_service_error:{type(error).__name__}')
                    return
                if response is None:
                    self.activation_error = f'{source}_clear_service_false'
                    return
                self.clear_service_response_counts[source] = 1
                self._event(f'{source}_clear_responded')
        if (self.recovery_responses_complete_s is None
                and all(value == 1 for value in
                        self.clear_service_response_counts.values())):
            self.recovery_responses_complete_s = now_s
            self.clear_observation_counts = {'global': 0, 'local': 0}
        if (self.recovery_started_s is not None
                and now_s - self.recovery_started_s
                > self.contract['clear_service_timeout_s']
                and self.recovery_responses_complete_s is None):
            self.activation_error = 'clear_service_response_timeout'
        if (self.recovery_responses_complete_s is not None
                and now_s - self.recovery_responses_complete_s
                > self.contract['post_recovery_clear_window_s']
                and not (self.global_cleared and self.local_cleared)):
            self.activation_error = 'post_recovery_clear_timeout'

    def _ground_truth(self, message: PoseStamped) -> None:
        self.ground_truth_message_count += 1
        orientation = message.pose.orientation
        yaw_rad = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))
        self.robot_pose_xy_yaw = (
            message.pose.position.x, message.pose.position.y, yaw_rad)
        if (self.args.scenario == 'event_driven_removal'
                and self.removal_passage is None
                and self.final_post_clear_plan_s is not None
                and self.global_cleared and self.local_cleared):
            passage_contract = self.contract['removal_passage']
            direction_sign = passage_contract['direction_sign']
            crossed = (
                direction_sign * (
                    message.pose.position.x
                    - passage_contract['far_edge_x_m'])
                > passage_contract['robot_radius_m'])
            if crossed:
                stamp_ns = (
                    message.header.stamp.sec * 1_000_000_000
                    + message.header.stamp.nanosec)
                receive_elapsed_s = time.monotonic() - self.start_s
                final_plan = self.post_clear_plan_history[-1]
                final_samples = self.clear_completion_history[-1][
                    'source_samples']
                if not (max(item['stamp_ns'] for item in
                            final_samples.values())
                        < final_plan['stamp_ns'] < stamp_ns
                        and max(item['receive_elapsed_s'] for item in
                                final_samples.values())
                        < final_plan['receive_elapsed_s']
                        < receive_elapsed_s):
                    self.activation_error = 'passage_evidence_out_of_order'
                else:
                    self.removal_passage = {
                        'epoch': self.clear_epoch,
                        'ground_truth_sequence': (
                            self.ground_truth_message_count),
                        'stamp_ns': stamp_ns,
                        'receive_elapsed_s': receive_elapsed_s,
                        'robot_x_m': message.pose.position.x,
                        'strict_threshold_x_m': passage_contract[
                            'strict_robot_center_threshold_x_m'],
                    }
                    self._event('far_side_passage')
        if self.obstacle is not None:
            distance_m = math.hypot(
                message.pose.position.x - self.obstacle['x_m'],
                message.pose.position.y - self.obstacle['y_m'])
            if (self.minimum_robot_obstacle_center_distance_m is None
                    or distance_m
                    < self.minimum_robot_obstacle_center_distance_m):
                self.minimum_robot_obstacle_center_distance_m = distance_m
            aabb_distance_m = math.hypot(
                max(0.0, abs(message.pose.position.x - self.obstacle['x_m'])
                    - self.obstacle['length_m'] / 2.0),
                max(0.0, abs(message.pose.position.y - self.obstacle['y_m'])
                    - self.obstacle['width_m'] / 2.0))
            if (self.minimum_robot_obstacle_aabb_distance_m is None
                    or aabb_distance_m
                    < self.minimum_robot_obstacle_aabb_distance_m):
                self.minimum_robot_obstacle_aabb_distance_m = aabb_distance_m
            before_deactivate_m = self.minimum_pre_deactivate_aabb_distance_m
            if (self.activated and not self.deactivated
                    and (before_deactivate_m is None
                         or aabb_distance_m < before_deactivate_m)):
                self.minimum_pre_deactivate_aabb_distance_m = aabb_distance_m

    def _odom(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        yaw_rad = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))
        self.odom_pose_xy_yaw = (
            message.pose.pose.position.x, message.pose.pose.position.y,
            yaw_rad)

    def _scan(self, message: LaserScan) -> None:
        self.scan_message_count += 1
        stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec)
        self.last_scan_stamp_ns = stamp_ns
        if self.obstacle is not None and self.robot_pose_xy_yaw is not None:
            surface_endpoints = obstacle_surface_endpoints(
                list(message.ranges), message.angle_min,
                message.angle_increment, self.robot_pose_xy_yaw,
                self.laser_yaw_in_base_rad, self.obstacle,
                self.contract['costmap_resolution_m'])
            surface_ranges = [item[0] for item in surface_endpoints]
            surface_beams = len(surface_ranges)
            self.obstacle_surface_beam_count += surface_beams
            if surface_beams:
                self._event('scan_obstacle_surface')
                if (self.activation_surface_scan is None
                        and self.activation_reference_scan_stamp_ns
                        is not None
                        and stamp_ns
                        > self.activation_reference_scan_stamp_ns):
                    self.activation_surface_scan = {
                        'stamp_ns': stamp_ns,
                        'surface_beam_count': surface_beams,
                        'minimum_surface_range_m': min(surface_ranges),
                    }
                    self._event('activation_surface_scan')
                if (self.mark_eligible_scan is None
                        and min(surface_ranges) <= self.contract[
                            'obstacle_mark_max_range_m']):
                    mark_eligible_s = time.monotonic()
                    self.mark_eligible_scan = {
                        'stamp_ns': stamp_ns,
                        'surface_beam_count': surface_beams,
                        'minimum_surface_range_m': min(surface_ranges),
                    }
                    core_half_width_m = (
                        SCENARIO_OBSTACLES['detour']['width_m'] / 2.0)
                    self.mark_eligible_costmap_probe = (
                        blocking_probe_from_endpoints(
                            surface_endpoints, self.obstacle['y_m'],
                            core_half_width_m,
                            self.contract['robot_circumscribed_radius_m']))
                    self.clear_costmap_probe = (
                        blocking_probe_from_endpoints(
                            surface_endpoints, self.obstacle['y_m'],
                            core_half_width_m,
                            self.contract['costmap_resolution_m']))
                    self._event('mark_eligible_scan')
                    if self.args.scenario in {
                            'full_block', 'goal_occupied'}:
                        self.mark_complete_s = mark_eligible_s
        if (self.deactivated and not self.new_scan_after_deactivate
                and self.deactivate_scan_stamp_ns is not None
                and stamp_ns > self.deactivate_scan_stamp_ns):
            self.new_scan_after_deactivate = True
            self.new_scan_after_deactivate_s = time.monotonic()
            if (self.obstacle is not None
                    and self.robot_pose_xy_yaw is not None):
                self.post_deactivate_scan_window = (
                    obstacle_angular_window_samples(
                        list(message.ranges), message.angle_min,
                        message.angle_increment, self.robot_pose_xy_yaw,
                        self.laser_yaw_in_base_rad, self.obstacle))
                self.post_deactivate_scan_window['stamp_ns'] = stamp_ns
            self._event('new_scan')

    def _plan(self, message: NavPath) -> None:
        self.plan_message_count += 1
        points = [[pose.pose.position.x, pose.pose.position.y]
                  for pose in message.poses]
        finite_nonempty = bool(points) and all(
            all(math.isfinite(value) for value in point) for point in points)
        if self.mark_complete_s is None:
            self.plans_before_mark.append(points)
            self._event('pre_mark_plan')
        else:
            self.plans_after_mark.append(points)
            if (self.args.scenario == 'event_driven_removal'
                    and not self.deactivated):
                self._event('post_mark_plan')
                self.deactivate()
            elif (self.args.scenario != 'event_driven_removal'
                  or (self.global_cleared and self.local_cleared)):
                if self.args.scenario == 'event_driven_removal':
                    if (self.final_clear_completion_s is None
                            or self.final_post_clear_plan_s is not None):
                        return
                    if not finite_nonempty:
                        self.activation_error = 'invalid_post_clear_plan'
                        return
                    receive_elapsed_s = time.monotonic() - self.start_s
                    stamp_ns = (
                        message.header.stamp.sec * 1_000_000_000
                        + message.header.stamp.nanosec)
                    clear_samples = self.clear_completion_history[-1][
                        'source_samples']
                    if not (stamp_ns > max(
                            item['stamp_ns'] for item in
                            clear_samples.values())
                            and receive_elapsed_s > max(
                                item['receive_elapsed_s'] for item in
                                clear_samples.values())):
                        self.activation_error = 'post_clear_plan_out_of_order'
                        return
                    self.final_post_clear_plan_s = time.monotonic()
                    self.post_clear_plan_history.append({
                        'epoch': self.clear_epoch,
                        'elapsed_s': (
                            self.final_post_clear_plan_s - self.start_s),
                        'receive_elapsed_s': receive_elapsed_s,
                        'stamp_ns': stamp_ns,
                    })
                    self._event('post_clear_new_plan')
                self._event('new_plan')

    def _cmd_vel(self, message: Twist) -> None:
        now_s = time.monotonic()
        if _is_zero(message):
            if self.zero_since_s is None:
                self.zero_since_s = now_s
            if self.mark_complete_s is not None and self.first_zero_s is None:
                self.first_zero_s = now_s
                self._event('final_zero')
        else:
            self.zero_since_s = None

    def _contacts(self, message) -> None:
        for contact in getattr(message, 'contacts', []):
            names = (contact.collision1.name, contact.collision2.name)
            if (any(self.entity_name in name for name in names)
                    and any('jdamr_cube' in name for name in names)):
                self.contact_count += 1

    def ready_to_finish(self) -> bool:
        """Return true after a terminal and the configured final zero hold."""
        if self.args.scenario == 'event_driven_removal':
            if not (self.deactivated and self.new_scan_after_deactivate
                    and self.global_cleared and self.local_cleared
                    and self.final_clear_completion_s is not None
                    and self.final_post_clear_plan_s is not None
                    and self.removal_passage is not None
                    and self.final_clear_completion_s
                    < self.final_post_clear_plan_s):
                return False
        if self.action_terminal_s is None or self.zero_since_s is None:
            return False
        return (time.monotonic() - self.zero_since_s
                >= self.contract['final_zero_hold_s'])

    def probe_ready(self) -> bool:
        """Return true after the selected sensor probe has been removed."""
        updates_after_deactivate = {
            source: (
                self.costmap_message_counts[source]
                - self.costmap_counts_at_deactivate[source])
            for source in ('global', 'local')
        } if self.costmap_counts_at_deactivate is not None else {
            'global': 0, 'local': 0}
        costmaps_updated = all(
            count >= 2 for count in updates_after_deactivate.values())
        if self.args.scenario == 'front_observation_probe':
            return (self.deactivated and self.new_scan_after_deactivate
                    and self.global_cleared and self.local_cleared
                    and costmaps_updated)
        return (self.deactivated and self.new_scan_after_deactivate
                and costmaps_updated)

    def probe_passed(self) -> bool:
        """Apply the fail-closed sensor-probe contract."""
        common = (
            self.deactivated and self.new_scan_after_deactivate
            and self.contact_count == 0
            and self.contact_matched_publisher_count_max > 0)
        if self.args.scenario == 'front_observation_probe':
            return (common and self.obstacle_surface_beam_count > 0
                    and self.global_blocking and self.local_blocking
                    and self.global_cleared and self.local_cleared)
        return (common and self.obstacle_surface_beam_count == 0
                and not self.global_blocking and not self.local_blocking)

    def evidence(self, runner_timed_out: bool) -> dict[str, Any]:
        """Return raw evidence; the runner appends teardown evidence."""
        now_s = time.monotonic()
        return {
            'schema_version': 1,
            'run_id': f'{self.args.scenario}__seed_{self.args.seed}',
            'scenario': self.args.scenario,
            'seed': self.args.seed,
            'contract': self.contract,
            'observation_persistence_s': self.args.observation_persistence_s,
            'events': self.events,
            'global_blocking': self.global_blocking,
            'local_blocking': self.local_blocking,
            'global_cleared': self.global_cleared,
            'local_cleared': self.local_cleared,
            'new_scan_after_deactivate': self.new_scan_after_deactivate,
            'clear_mode': self.clear_mode,
            'clear_observation_counts': self.clear_observation_counts,
            'clear_service_request_counts': self.clear_service_request_counts,
            'clear_service_response_counts': (
                self.clear_service_response_counts),
            'clear_recovery_scope': self.contract['clear_recovery_scope'],
            'global_baseline_snapshots': self.global_baseline_snapshots,
            'baseline_ready': self.baseline_ready,
            'mark_excess_snapshots': self.mark_excess_snapshots,
            'pending_local_costmap_failures': (
                self.pending_local_costmap_failures),
            'clear_epoch': self.clear_epoch,
            'clear_completion_history': self.clear_completion_history,
            'post_clear_plan_history': self.post_clear_plan_history,
            'removal_passage': self.removal_passage,
            'final_clear_completion_elapsed_s': (
                self.final_clear_completion_s - self.start_s
                if self.final_clear_completion_s is not None else None),
            'final_post_clear_plan_elapsed_s': (
                self.final_post_clear_plan_s - self.start_s
                if self.final_post_clear_plan_s is not None else None),
            'action_terminal_elapsed_s': (
                self.action_terminal_s - self.start_s
                if self.action_terminal_s is not None else None),
            'plans_after_mark': self.plans_after_mark,
            'plans_before_mark': self.plans_before_mark,
            'plan_message_count': self.plan_message_count,
            'scan_message_count': self.scan_message_count,
            'last_scan_stamp_ns': self.last_scan_stamp_ns,
            'deactivate_scan_stamp_ns': self.deactivate_scan_stamp_ns,
            'post_deactivate_scan_window': self.post_deactivate_scan_window,
            'obstacle_surface_beam_count': self.obstacle_surface_beam_count,
            'activation_reference_scan_stamp_ns': (
                self.activation_reference_scan_stamp_ns),
            'activation_surface_scan': self.activation_surface_scan,
            'mark_eligible_scan': self.mark_eligible_scan,
            'mark_eligible_costmap_probe': (
                self.mark_eligible_costmap_probe),
            'clear_costmap_probe': self.clear_costmap_probe,
            'costmap_message_counts': self.costmap_message_counts,
            'costmap_counts_at_deactivate': (
                self.costmap_counts_at_deactivate),
            'costmap_updates_after_deactivate': ({
                source: (
                    self.costmap_message_counts[source]
                    - self.costmap_counts_at_deactivate[source])
                for source in ('global', 'local')
            } if self.costmap_counts_at_deactivate is not None else None),
            'costmap_max_roi_cost': self.costmap_max_roi_cost,
            'costmap_matched_publisher_counts': {
                'global': len(self.get_publishers_info_by_topic(
                    '/global_costmap/costmap_raw')),
                'local': len(self.get_publishers_info_by_topic(
                    '/local_costmap/costmap_raw')),
            },
            'costmap_roi_statistics': self.costmap_roi_statistics,
            'costmap_surface_roi_statistics': (
                self.costmap_surface_roi_statistics),
            'clear_observation_trace': self.clear_observation_trace,
            'final_robot_pose_xy_yaw': self.robot_pose_xy_yaw,
            'minimum_robot_obstacle_center_distance_m': (
                self.minimum_robot_obstacle_center_distance_m),
            'minimum_robot_obstacle_aabb_distance_m': (
                self.minimum_robot_obstacle_aabb_distance_m),
            'minimum_robot_obstacle_aabb_distance_before_deactivate_m': (
                self.minimum_pre_deactivate_aabb_distance_m),
            'action_terminal': self.action_terminal,
            'runner_cancelled': False,
            'runner_timed_out': runner_timed_out,
            'terminal_elapsed_from_mark_s': (
                self.action_terminal_s - self.mark_complete_s
                if self.action_terminal_s is not None
                and self.mark_complete_s is not None else None),
            'first_zero_from_mark_s': (
                self.first_zero_s - self.mark_complete_s
                if self.first_zero_s is not None
                and self.mark_complete_s is not None else None),
            'contact_count': self.contact_count,
            'contact_matched_publisher_count_max': (
                self.contact_matched_publisher_count_max),
            'contact_scope': self.contract['contact_scope'],
            'final_cmd_vel_zero': self.zero_since_s is not None,
            'final_zero_hold_s': (
                now_s - self.zero_since_s if self.zero_since_s is not None
                else 0.0),
            'activation_error': self.activation_error,
            'entity_states': self.entity_states,
            'entity_identity_stable': (
                bool(self.entity_states)
                and len({item['entity_id'] for item in self.entity_states})
                == 1),
            'claim_scope': (
                'Gazebo의 고정 장애물 재계획 증거이며 이동 물체·사람 안전·'
                '실차 충돌 안전을 입증하지 않는다.'),
        }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse scenario arguments without consuming ROS remapping arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--contract', type=Path, required=True)
    parser.add_argument('--behavior-tree', type=Path, required=True)
    parser.add_argument('--goal-x-m', type=float, default=6.0)
    parser.add_argument('--goal-y-m', type=float, default=0.0)
    parser.add_argument('--goal-yaw-rad', type=float, default=0.0)
    parser.add_argument('--run-timeout-s', type=float, default=180.0)
    parser.add_argument(
        '--observation-persistence-s', type=float, default=0.0)
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv: list[str] | None = None) -> int:
    """Run one scenario and persist evidence even when a gate fails."""
    args = parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') == '12':
        raise RuntimeError('physical robot ROS domain is forbidden')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError(
            'simulation evaluation requires LOCALHOST discovery')
    if (args.scenario not in SCENARIO_OBSTACLES
            and args.scenario not in PREFLIGHT_SCENARIOS):
        raise ValueError(f'unknown scenario: {args.scenario}')
    contract = json.loads(args.contract.read_text(encoding='utf-8'))
    rclpy.init(args=[])
    if args.scenario in PREFLIGHT_SCENARIOS:
        probe_key = ('front_observation_probe'
                     if args.scenario == 'front_observation_probe'
                     else 'sub_minimum_range_probe')
        probe = contract['simulation_support_plane_correction'][
            'scan_preflight_contract'][probe_key]
        obstacle = {
            'x_m': probe['center_x_m'], 'y_m': probe['center_y_m'],
            'length_m': probe['length_m'], 'width_m': probe['width_m']}
    else:
        obstacle = SCENARIO_OBSTACLES[args.scenario]
    observer = ScenarioObserver(args, contract, {'obstacle': obstacle})
    runner_timed_out = False
    try:
        if args.scenario in PREFLIGHT_SCENARIOS:
            readiness_deadline_s = time.monotonic() + args.run_timeout_s
            while rclpy.ok() and observer.last_scan_stamp_ns is None:
                rclpy.spin_once(observer, timeout_sec=0.05)
                if time.monotonic() >= readiness_deadline_s:
                    runner_timed_out = True
                    break
            if not runner_timed_out and observer.activate():
                deadline_s = time.monotonic() + args.run_timeout_s
                scan_contract = contract[
                    'simulation_support_plane_correction'][
                        'scan_preflight_contract']
                minimum_scan_messages = math.ceil(
                    2.0 * contract['global_update_period_s']
                    * scan_contract['simulation_update_rate_hz'])
                while rclpy.ok() and not observer.probe_ready():
                    rclpy.spin_once(observer, timeout_sec=0.05)
                    observer.contact_matched_publisher_count_max = max(
                        observer.contact_matched_publisher_count_max,
                        len(observer.get_publishers_info_by_topic(
                            observer.contact_topic)))
                    if (not observer.deactivated
                            and args.scenario == 'front_observation_probe'
                            and observer.obstacle_surface_beam_count > 0
                            and observer.global_blocking
                            and observer.local_blocking):
                        observer.deactivate()
                    if (not observer.deactivated
                            and args.scenario == 'sub_minimum_probe'
                            and observer.scan_message_count
                            >= minimum_scan_messages):
                        observer.deactivate()
                    if time.monotonic() >= deadline_s:
                        runner_timed_out = True
                        break
        else:
            readiness_deadline_s = time.monotonic() + args.run_timeout_s
            while rclpy.ok() and (
                    observer.last_scan_stamp_ns is None
                    or observer.robot_pose_xy_yaw is None
                    or not observer.baseline_ready):
                rclpy.spin_once(observer, timeout_sec=0.05)
                if observer.activation_error is not None:
                    break
                if time.monotonic() >= readiness_deadline_s:
                    runner_timed_out = True
                    break
            activated = (not runner_timed_out
                         and observer.activation_error is None
                         and observer.activate())
            if activated and args.scenario in {
                    'detour', 'event_driven_removal', 'full_block'}:
                surface_deadline_s = time.monotonic() + args.run_timeout_s
                while (rclpy.ok()
                       and observer.activation_surface_scan is None):
                    rclpy.spin_once(observer, timeout_sec=0.05)
                    if time.monotonic() >= surface_deadline_s:
                        runner_timed_out = True
                        break
            if activated and not runner_timed_out and observer.send_goal():
                deadline_s = time.monotonic() + args.run_timeout_s
                while rclpy.ok() and not observer.ready_to_finish():
                    rclpy.spin_once(observer, timeout_sec=0.05)
                    observer.poll_clear_recovery()
                    if observer.contact_topic:
                        observer.contact_matched_publisher_count_max = max(
                            observer.contact_matched_publisher_count_max,
                            len(observer.get_publishers_info_by_topic(
                                observer.contact_topic)))
                    if observer.activation_error is not None:
                        break
                    if time.monotonic() >= deadline_s:
                        runner_timed_out = True
                        break
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        document = observer.evidence(runner_timed_out)
        if args.scenario in PREFLIGHT_SCENARIOS:
            document['preflight'] = True
            document['status'] = (
                'PASS' if observer.probe_passed() and not runner_timed_out
                else 'FAIL')
        args.output.write_text(json.dumps(
            document, ensure_ascii=False,
            indent=2, sort_keys=True) + '\n', encoding='utf-8')
        observer.destroy_node()
        rclpy.shutdown()
    if args.scenario in PREFLIGHT_SCENARIOS:
        return 0 if document['status'] == 'PASS' else 2
    return 0 if not runner_timed_out and document.get(
        'activation_error') is None else 2


if __name__ == '__main__':
    raise SystemExit(main())
