"""Execute the data-derived 20-goal restaurant replay in Gazebo."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path
from typing import Any, Callable

from geometry_msgs.msg import PoseStamped, Twist

from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool

try:
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
except ImportError:
    Entity = None
    SetEntityPose = None


SUCCESS = 4
SET_POSE_SERVICE = '/world/slam_corridor/set_pose'


def scene_schedule(contract: dict[str, Any]) -> dict[int, list[dict]]:
    """Group the three video-grounded scenes by active waypoint."""
    events = contract['obstacle_interventions']['events']
    scenes = contract['obstacle_interventions']['scenes']
    if (contract['route']['simulation_waypoint_count'] != 20
            or len(events) != 6 or len(scenes) != 3):
        raise ValueError(
            'restaurant replay requires 20 goals, six records, three scenes')
    if [item['event'] for item in events] != list(range(1, 7)):
        raise ValueError('source stop records must retain order')
    schedule: dict[int, list[dict]] = {}
    covered_events = []
    for scene in scenes:
        index = int(scene['trigger_waypoint_index'])
        if not 1 <= index <= 20:
            raise ValueError('scene waypoint index is outside the route')
        covered_events.extend(scene['source_event_ids'])
        schedule.setdefault(index, []).append(scene)
    if covered_events != list(range(1, 7)):
        raise ValueError('scene grouping must cover source records once')
    return schedule


def waypoint_yaw(waypoint: dict[str, Any]) -> float:
    """Return the route direction expected at one transformed waypoint."""
    identifier = waypoint['id']
    return math.pi if (
        identifier.startswith('return_')
        or identifier in {'turnaround', 'home'}) else 0.0


def wheel_slip_commands(
        links: list[str], lateral: float,
        longitudinal: float) -> list[list[str]]:
    """Build explicit Gazebo service commands for both drive-wheel links."""
    return [[
        'gz', 'service', '-s', '/world/slam_corridor/wheel_slip',
        '--reqtype', 'gz.msgs.WheelSlipParametersCmd',
        '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req',
        ('entity: {name: "jdamr_cube::' + link + '", type: LINK}, '
         f'slip_compliance_lateral: {lateral}, '
         f'slip_compliance_longitudinal: {longitudinal}'),
    ] for link in links]


class RestaurantReplayScenario(Node):
    """Sequence goals through two box detours and one person stop."""

    def __init__(self, args: argparse.Namespace, contract: dict[str, Any]):
        """Bind action, obstacle control, and evidence subscriptions."""
        super().__init__('sim_restaurant_replay_scenario')
        if SetEntityPose is None or Entity is None:
            raise RuntimeError(
                'ros_gz_interfaces SetEntityPose is unavailable')
        self.args = args
        self.contract = contract
        self.schedule = scene_schedule(contract)
        self.action = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pose_client = self.create_client(
            SetEntityPose, SET_POSE_SERVICE)
        self.traction_fault_publisher = self.create_publisher(
            Bool, '/sim/traction_fault_active', 10)
        self.world_pose: tuple[float, float, float] | None = None
        self.latest_command = Twist()
        self.latest_guarded_command = Twist()
        self.monitor_action = CollisionMonitorState.DO_NOTHING
        self.monitor_polygon = ''
        self.started_s = time.monotonic()
        self.events: list[dict[str, Any]] = []
        self.goals: list[dict[str, Any]] = []
        self.interventions: list[dict[str, Any]] = []
        self.error: str | None = None
        self.slip_active = False
        self.slip_enabled_s: float | None = None
        self.guarded_zero_since_s: float | None = None
        self.create_subscription(
            PoseStamped, '/ground_truth_pose', self._ground_truth,
            qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self._command, 10)
        self.create_subscription(
            Twist, '/guarded_cmd_vel', self._guarded_command, 10)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state',
            self._monitor, 10)

    def _elapsed_s(self) -> float:
        return time.monotonic() - self.started_s

    def _event(self, name: str, **fields: Any) -> dict[str, Any]:
        event = {
            'name': name, 'elapsed_s': self._elapsed_s(),
            'sim_s': self.get_clock().now().nanoseconds / 1e9,
            **fields}
        self.events.append(event)
        return event

    def _ground_truth(self, message: PoseStamped) -> None:
        pose = message.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z
                   + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2
                         + pose.orientation.z ** 2))
        self.world_pose = (
            float(pose.position.x), float(pose.position.y), yaw)

    def _command(self, message: Twist) -> None:
        self.latest_command = message

    def _guarded_command(self, message: Twist) -> None:
        self.latest_guarded_command = message

    def _monitor(self, message: CollisionMonitorState) -> None:
        self.monitor_action = int(message.action_type)
        self.monitor_polygon = str(message.polygon_name)

    def _spin_until(
            self, predicate: Callable[[], bool], timeout_s: float) -> bool:
        deadline_s = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline_s:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def ready(self) -> bool:
        """Wait for ground truth, action server, and entity service."""
        return (
            self._spin_until(lambda: self.world_pose is not None, 30.0)
            and self.action.wait_for_server(timeout_sec=30.0)
            and self.pose_client.wait_for_service(timeout_sec=10.0))

    def _set_entity(
            self, entity_name: str, x_m: float, y_m: float,
            z_m: float) -> bool:
        request = SetEntityPose.Request()
        request.entity.name = entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = x_m
        request.pose.position.y = y_m
        request.pose.position.z = z_m
        request.pose.orientation.w = 1.0
        future = self.pose_client.call_async(request)
        return self._spin_until(
            lambda: future.done(), 3.0
        ) and future.result() is not None and future.result().success

    def _set_wheel_slip(self, longitudinal: float) -> bool:
        fault = self.contract['traction_fault']['fault_injection']
        commands = wheel_slip_commands(
            fault['wheel_links'], fault['slip_compliance_lateral'],
            longitudinal)
        for command in commands:
            result = subprocess.run(
                command, capture_output=True, text=True, timeout=5.0,
                check=False)
            if result.returncode != 0 or 'data: true' not in result.stdout:
                return False
        self.slip_active = longitudinal > 0.0
        self.slip_enabled_s = (
            time.monotonic() if self.slip_active else None)
        state = Bool()
        state.data = self.slip_active
        self.traction_fault_publisher.publish(state)
        return True

    @staticmethod
    def _goal_uuid(handle: Any) -> str:
        return bytes(handle.goal_id.uuid).hex()

    def _cross_person(self, scene: dict[str, Any], goal_uuid: str) -> bool:
        if self.world_pose is None:
            self.error = 'ground_truth_missing_at_person_trigger'
            return False
        x_m, y_m, yaw = self.world_pose
        center_x_m = x_m + self.args.obstacle_ahead_m * math.cos(yaw)
        center_y_m = y_m + self.args.obstacle_ahead_m * math.sin(yaw)
        lateral_x = -math.sin(yaw)
        lateral_y = math.cos(yaw)
        entity = scene['object_entity']
        record = {
            'scene_id': scene['scene_id'], 'kind': scene['kind'],
            'source_event_ids': scene['source_event_ids'],
            'waypoint_index': scene['trigger_waypoint_index'],
            'goal_uuid': goal_uuid,
            'crossing_center_m': [center_x_m, center_y_m],
            'trigger_elapsed_s': self._elapsed_s(),
            'trigger_sim_s': self.get_clock().now().nanoseconds / 1e9,
            'person_track_m': [],
        }
        support_pose = scene['supporting_box_pose_m']
        if not self._set_entity(
                scene['supporting_box_entity'], float(support_pose[0]),
                float(support_pose[1]), float(support_pose[2])):
            self.error = 'supporting_box_activation_failed'
            return False
        record['supporting_box_pose_m'] = support_pose
        for step in range(9):
            lateral = 0.90 - step * 0.1125
            pose = (
                center_x_m + lateral_x * lateral,
                center_y_m + lateral_y * lateral)
            if not self._set_entity(entity, pose[0], pose[1], 0.0):
                self.error = 'person_crossing_activation_failed'
                return False
            record['person_track_m'].append([*pose, self._elapsed_s()])
            rclpy.spin_once(self, timeout_sec=0.06)
        stopped = self._spin_until(lambda: (
            self.monitor_action == CollisionMonitorState.STOP
            and self.monitor_polygon == 'StopZone'
            and abs(float(self.latest_command.linear.x)) <= 1e-4
            and abs(float(self.latest_command.angular.z)) <= 1e-4
        ), self.args.stop_timeout_s)
        record['stop_observed'] = stopped
        record['stop_elapsed_s'] = self._elapsed_s() if stopped else None
        record['stop_sim_s'] = (
            self.get_clock().now().nanoseconds / 1e9 if stopped else None)
        if stopped:
            hold_deadline_s = time.monotonic() + max(
                1.2, self.args.obstacle_hold_s)
            while rclpy.ok() and time.monotonic() < hold_deadline_s:
                rclpy.spin_once(self, timeout_sec=0.02)
        for step in range(1, 9):
            lateral = -step * 0.1125
            pose = (
                center_x_m + lateral_x * lateral,
                center_y_m + lateral_y * lateral)
            if not self._set_entity(entity, pose[0], pose[1], 0.0):
                self.error = 'person_crossing_clear_failed'
                return False
            record['person_track_m'].append([*pose, self._elapsed_s()])
            rclpy.spin_once(self, timeout_sec=0.06)
        cleared = self._set_entity(
            entity, self.args.park_x_m, self.args.park_y_m, 0.0)
        record['clear_acknowledged'] = cleared
        record['clear_sim_s'] = self.get_clock().now().nanoseconds / 1e9
        resumed = cleared and self._spin_until(lambda: (
            self.monitor_action == CollisionMonitorState.DO_NOTHING
            and (abs(float(self.latest_command.linear.x)) > 1e-3
                 or abs(float(self.latest_command.angular.z)) > 1e-3)
        ), self.args.resume_timeout_s)
        record['same_goal_resumed'] = resumed
        record['resume_elapsed_s'] = self._elapsed_s() if resumed else None
        record['resume_sim_s'] = (
            self.get_clock().now().nanoseconds / 1e9 if resumed else None)
        record['success'] = stopped and cleared and resumed
        self.interventions.append(record)
        if not (stopped and cleared and resumed):
            self.error = 'obstacle_stop_or_resume_gate_failed'
            return False
        return True

    def _run_goal(self, index: int, waypoint: dict[str, Any]) -> bool:
        fault = self.contract['traction_fault']['fault_injection']
        traction_goal = index == fault['trigger_waypoint_index']
        if traction_goal:
            compliance = fault['slip_compliance_longitudinal']
            if not self._set_wheel_slip(compliance):
                self.error = 'traction_fault_activation_failed'
                return False
            self._event(
                'traction_fault_enabled', waypoint_index=index,
                slip_compliance_longitudinal=compliance)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(waypoint['x_m'])
        goal.pose.pose.position.y = float(waypoint['y_m'])
        yaw = waypoint_yaw(waypoint)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.behavior_tree = str(self.args.behavior_tree.resolve())
        send_future = self.action.send_goal_async(goal)
        if not self._spin_until(lambda: send_future.done(), 10.0):
            self.error = f'goal_{index}_accept_timeout'
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.error = f'goal_{index}_rejected'
            return False
        goal_uuid = self._goal_uuid(handle)
        goal_record = {
            'waypoint_index': index, 'waypoint_id': waypoint['id'],
            'goal_uuid': goal_uuid, 'accepted_elapsed_s': self._elapsed_s(),
        }
        self.goals.append(goal_record)
        result_future = handle.get_result_async()
        pending_scenes = list(self.schedule.get(index, ()))
        avoidance_records: list[dict[str, Any]] = []
        deadline_s = time.monotonic() + self.args.goal_timeout_s
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
            moving = (
                abs(float(self.latest_command.linear.x))
                >= self.args.minimum_trigger_speed_mps)
            guarded_stopped = (
                abs(float(self.latest_guarded_command.linear.x)) <= 1e-4
                and abs(float(self.latest_guarded_command.angular.z)) <= 1e-4)
            traction_stop = (
                traction_goal and self.slip_active and moving
                and self.slip_enabled_s is not None
                and time.monotonic() - self.slip_enabled_s >= 1.5
                and guarded_stopped
                and self.monitor_action == CollisionMonitorState.DO_NOTHING)
            if traction_stop:
                if self.guarded_zero_since_s is None:
                    self.guarded_zero_since_s = time.monotonic()
                elif time.monotonic() - self.guarded_zero_since_s >= 0.2:
                    if not self._set_wheel_slip(0.0):
                        self.error = 'traction_fault_clear_failed'
                        return False
                    self._event(
                        'traction_fault_cleared_on_protective_stop',
                        waypoint_index=index)
            else:
                self.guarded_zero_since_s = None
            if pending_scenes and moving:
                scene = pending_scenes[0]
                if scene['kind'] == 'person_crossing_emergency_stop':
                    pending_scenes.pop(0)
                    if not self._cross_person(scene, goal_uuid):
                        return False
                elif scene['kind'] == 'static_avoidance':
                    object_x_m, object_y_m = scene['object_pose_m']
                    distance_m = math.hypot(
                        self.world_pose[0] - object_x_m,
                        self.world_pose[1] - object_y_m)
                    if distance_m <= 3.0:
                        pending_scenes.pop(0)
                        avoidance_records.append({
                            'scene_id': scene['scene_id'],
                            'kind': scene['kind'],
                            'source_event_ids': scene['source_event_ids'],
                            'waypoint_index': index,
                            'goal_uuid': goal_uuid,
                            'object_pose_m': scene['object_pose_m'],
                            'trigger_elapsed_s': self._elapsed_s(),
                            'trigger_sim_s': (
                                self.get_clock().now().nanoseconds / 1e9),
                            'minimum_distance_m': distance_m,
                            'maximum_abs_angular_z': abs(float(
                                self.latest_command.angular.z)),
                        })
            for record in avoidance_records:
                object_x_m, object_y_m = record['object_pose_m']
                record['minimum_distance_m'] = min(
                    record['minimum_distance_m'], math.hypot(
                        self.world_pose[0] - object_x_m,
                        self.world_pose[1] - object_y_m))
                record['maximum_abs_angular_z'] = max(
                    record['maximum_abs_angular_z'],
                    abs(float(self.latest_command.angular.z)))
            if time.monotonic() >= deadline_s:
                self.error = f'goal_{index}_result_timeout'
                return False
        result = result_future.result()
        status = int(result.status) if result is not None else -1
        if traction_goal and self.slip_active:
            if not self._set_wheel_slip(0.0):
                self.error = 'traction_fault_clear_failed'
                return False
            self._event('traction_fault_disabled', waypoint_index=index)
        goal_record.update({
            'status': status,
            'result_elapsed_s': self._elapsed_s(),
            'pending_scene_count': len(pending_scenes),
        })
        for record in avoidance_records:
            record['result_sim_s'] = self.get_clock().now().nanoseconds / 1e9
            record['same_goal_continued'] = status == SUCCESS
            record['steering_observed'] = (
                record['maximum_abs_angular_z'] >= 0.03)
            record['success'] = (
                record['same_goal_continued']
                and record['steering_observed'])
            self.interventions.append(record)
        if (status != SUCCESS or pending_scenes
                or any(not item['success'] for item in avoidance_records)):
            self.error = f'goal_{index}_failed_or_scenes_missing'
            return False
        return True

    def run(self) -> bool:
        """Execute every transformed goal without cancelling active goals."""
        if not self.ready():
            self.error = 'runtime_not_ready'
            return False
        self._event('runtime_ready')
        waypoints = self.contract['route']['waypoints']
        try:
            for index, waypoint in enumerate(waypoints, start=1):
                if not self._run_goal(index, waypoint):
                    return False
        finally:
            if self.slip_active:
                self._set_wheel_slip(0.0)
        self._event('route_complete')
        return True

    def evidence(self, success: bool) -> dict[str, Any]:
        """Return bounded scenario evidence without a digital-twin claim."""
        return {
            'schema_version': 1,
            'scenario_id': self.contract['scenario_id'],
            'status': 'PASS' if success else 'FAIL',
            'claim': 'functional_scenario_replay_not_digital_twin',
            'error': self.error,
            'goal_count': len(self.goals),
            'successful_goal_count': sum(
                item.get('status') == SUCCESS for item in self.goals),
            'intervention_count': len(self.interventions),
            'successful_intervention_count': sum(
                item.get('success')
                for item in self.interventions),
            'goals': self.goals,
            'interventions': self.interventions,
            'events': self.events,
        }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse replay arguments while leaving ROS arguments untouched."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract', type=Path, required=True)
    parser.add_argument('--behavior-tree', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--obstacle-entity', required=True)
    parser.add_argument('--park-x-m', type=float, default=0.0)
    parser.add_argument('--park-y-m', type=float, default=50.0)
    parser.add_argument('--park-z-m', type=float, default=0.5)
    parser.add_argument('--obstacle-ahead-m', type=float, default=0.55)
    parser.add_argument('--obstacle-hold-s', type=float, default=0.40)
    parser.add_argument('--stop-timeout-s', type=float, default=5.0)
    parser.add_argument('--resume-timeout-s', type=float, default=8.0)
    parser.add_argument('--goal-timeout-s', type=float, default=90.0)
    parser.add_argument(
        '--minimum-trigger-speed-mps', type=float, default=0.03)
    args, ros_args = parser.parse_known_args(argv)
    args.ros_args = ros_args
    return args


def main(argv: list[str] | None = None) -> int:
    """Run the replay and always persist its terminal evidence."""
    args = parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') == '12':
        raise RuntimeError('physical robot ROS domain is forbidden')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('simulation replay requires LOCALHOST discovery')
    contract = json.loads(args.contract.read_text(encoding='utf-8'))
    rclpy.init(args=args.ros_args)
    node = RestaurantReplayScenario(args, contract)
    success = False
    try:
        success = node.run()
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(
            node.evidence(success), ensure_ascii=False, indent=2
        ) + '\n', encoding='utf-8')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 2


if __name__ == '__main__':
    raise SystemExit(main())
