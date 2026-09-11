"""Simulation-only velocity guard for a bounded traction recovery trial."""

from __future__ import annotations

import argparse
import json
import math
import os
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from geometry_msgs.msg import PoseStamped, Twist

from nav2_msgs.msg import CollisionMonitorState

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


NAVIGATING = 'NAVIGATING'
PROTECTIVE_STOP = 'PROTECTIVE_STOP'
RELOCALIZE = 'RELOCALIZE'
LOW_SPEED_RESUME = 'LOW_SPEED_RESUME'
RECOVERED = 'RECOVERED'
FAULT_LATCHED = 'FAULT_LATCHED'


@dataclass(frozen=True)
class GuardConfig:
    """Synthetic simulation thresholds retained in the output evidence."""

    window_s: float = 1.0
    minimum_command_mps: float = 0.10
    minimum_odom_progress_m: float = 0.08
    minimum_truth_to_odom_ratio: float = 0.85
    anomaly_dwell_s: float = 0.40
    stop_hold_s: float = 0.75
    relocalize_hold_s: float = 0.75
    relocalize_timeout_s: float = 3.0
    low_speed_scale: float = 0.80
    low_speed_resume_s: float = 5.0
    recovery_grace_s: float = 5.0
    collision_clear_grace_s: float = 1.0
    standstill_mps: float = 0.02
    input_timeout_s: float = 0.50

    def validate(self) -> None:
        """Reject non-finite or unsafe state-machine thresholds."""
        values = tuple(self.__dict__.values())
        if any(not math.isfinite(value) for value in values):
            raise ValueError('guard thresholds must be finite')
        positive = (
            self.window_s, self.minimum_command_mps,
            self.minimum_odom_progress_m, self.anomaly_dwell_s,
            self.stop_hold_s, self.relocalize_hold_s,
            self.relocalize_timeout_s, self.low_speed_resume_s,
            self.recovery_grace_s,
            self.collision_clear_grace_s,
            self.standstill_mps, self.input_timeout_s,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError(
                'guard time, speed, and distance values must be positive')
        if not 0.0 < self.minimum_truth_to_odom_ratio < 1.0:
            raise ValueError(
                'motion ratio threshold must be between zero and one')
        if not 0.0 < self.low_speed_scale < 1.0:
            raise ValueError('low-speed scale must be between zero and one')
        if self.relocalize_timeout_s < self.relocalize_hold_s:
            raise ValueError('relocalize timeout must cover its hold interval')


class TractionRecoveryState:
    """Pure deterministic transition policy for one recovery then latch."""

    def __init__(self, config: GuardConfig):
        """Create the policy in its navigation state."""
        config.validate()
        self.config = config
        self.state = NAVIGATING
        self.entered_s = 0.0
        self.anomaly_since_s: float | None = None
        self.stable_since_s: float | None = None
        self.recovery_count = 0
        self.events: list[dict[str, Any]] = []

    def _transition(self, state: str, now_s: float, reason: str) -> None:
        self.events.append({
            'from': self.state, 'to': state, 'at_s': now_s,
            'reason': reason,
        })
        self.state = state
        self.entered_s = now_s
        self.anomaly_since_s = None
        self.stable_since_s = None

    def update(self, now_s: float, *, anomaly: bool,
               localization_stable: bool) -> str:
        """Advance one state using a monotonic time sample."""
        if not math.isfinite(now_s) or now_s < self.entered_s:
            raise ValueError('state time must be finite and monotonic')
        if self.state in (NAVIGATING, RECOVERED):
            if (self.state == RECOVERED
                    and now_s - self.entered_s < self.config.recovery_grace_s):
                self.anomaly_since_s = None
                return self.state
            if not anomaly:
                self.anomaly_since_s = None
                return self.state
            if self.anomaly_since_s is None:
                self.anomaly_since_s = now_s
                return self.state
            if now_s - self.anomaly_since_s < self.config.anomaly_dwell_s:
                return self.state
            target = (
                PROTECTIVE_STOP if self.state == NAVIGATING
                else FAULT_LATCHED)
            self._transition(target, now_s, 'sustained_traction_mismatch')
            return self.state
        if self.state == PROTECTIVE_STOP:
            if now_s - self.entered_s >= self.config.stop_hold_s:
                self._transition(
                    RELOCALIZE, now_s, 'zero_velocity_hold_complete')
            return self.state
        if self.state == RELOCALIZE:
            if localization_stable:
                if self.stable_since_s is None:
                    self.stable_since_s = now_s
                elif (now_s - self.stable_since_s
                      >= self.config.relocalize_hold_s):
                    self.recovery_count += 1
                    self._transition(
                        LOW_SPEED_RESUME, now_s,
                        'localization_stable_for_required_hold')
            else:
                self.stable_since_s = None
            if (self.state == RELOCALIZE and now_s - self.entered_s
                    >= self.config.relocalize_timeout_s):
                self._transition(
                    FAULT_LATCHED, now_s, 'relocalization_timeout')
            return self.state
        if self.state == LOW_SPEED_RESUME:
            if now_s - self.entered_s >= self.config.low_speed_resume_s:
                self._transition(RECOVERED, now_s, 'bounded_resume_complete')
            return self.state
        return self.state

    def velocity_scale(self) -> float:
        """Return the only command scale allowed in the current state."""
        if self.state in (PROTECTIVE_STOP, RELOCALIZE, FAULT_LATCHED):
            return 0.0
        if self.state == LOW_SPEED_RESUME:
            return self.config.low_speed_scale
        return 1.0


def displacement(samples: deque[tuple[float, float, float]]) -> float:
    """Return endpoint displacement for a timestamped planar sample window."""
    if len(samples) < 2:
        return 0.0
    return math.dist(samples[0][1:], samples[-1][1:])


def motion_ratio(odom: deque[tuple[float, float, float]],
                 truth: deque[tuple[float, float, float]]
                 ) -> tuple[float | None, float, float]:
    """Return truth/odom progress without treating zero odom as a fault."""
    odom_m = displacement(odom)
    truth_m = displacement(truth)
    return ((truth_m / odom_m) if odom_m > 0.0 else None, odom_m, truth_m)


def traction_observation_allowed(
        monitor_action: int, now_s: float, last_stop_s: float | None,
        clear_grace_s: float) -> bool:
    """Keep obstacle stops out of the independent traction classifier."""
    return (
        monitor_action == CollisionMonitorState.DO_NOTHING
        and (last_stop_s is None or now_s - last_stop_s >= clear_grace_s))


class TractionVelocityGuard(Node):
    """Gate Gazebo velocity using odom versus independent truth progress."""

    def __init__(self, config: GuardConfig, output: Path):
        """Bind observation topics and the guarded simulator command."""
        super().__init__('traction_velocity_guard')
        self.config = config
        self.output = output
        self.policy = TractionRecoveryState(config)
        self.started_ns = self.get_clock().now().nanoseconds
        self.latest_command = Twist()
        self.command_stamp_ns: int | None = None
        self.odom = deque()
        self.truth = deque()
        self.last_truth_speed_mps = math.inf
        self.last_truth: tuple[int, float, float] | None = None
        self.monitor_action = CollisionMonitorState.DO_NOTHING
        self.last_collision_stop_s: float | None = None
        self.maximum_odom_progress_m = 0.0
        self.minimum_motion_ratio: float | None = None
        self.output_messages = 0
        self.state_samples: list[dict[str, Any]] = []
        self.publisher = self.create_publisher(Twist, '/guarded_cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self._command, 10)
        self.create_subscription(
            Odometry, '/odom', self._odom, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/ground_truth_pose', self._truth,
            qos_profile_sensor_data)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state',
            self._collision_monitor, 10)
        self.create_timer(0.02, self._tick)

    def _now_s(self) -> float:
        return (self.get_clock().now().nanoseconds - self.started_ns) / 1e9

    def _trim(self, samples: deque[tuple[float, float, float]]) -> None:
        cutoff_s = self._now_s() - self.config.window_s
        while len(samples) > 2 and samples[1][0] < cutoff_s:
            samples.popleft()

    def _command(self, message: Twist) -> None:
        self.latest_command = message
        self.command_stamp_ns = self.get_clock().now().nanoseconds

    def _odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.odom.append((
            self._now_s(), float(position.x), float(position.y)))
        self._trim(self.odom)

    def _truth(self, message: PoseStamped) -> None:
        now_ns = self.get_clock().now().nanoseconds
        position = message.pose.position
        current = (now_ns, float(position.x), float(position.y))
        if self.last_truth is not None and now_ns > self.last_truth[0]:
            self.last_truth_speed_mps = math.dist(
                current[1:], self.last_truth[1:]) / (
                    (now_ns - self.last_truth[0]) / 1e9)
        self.last_truth = current
        self.truth.append((self._now_s(), current[1], current[2]))
        self._trim(self.truth)

    def _collision_monitor(self, message: CollisionMonitorState) -> None:
        self.monitor_action = int(message.action_type)
        if self.monitor_action == CollisionMonitorState.STOP:
            self.last_collision_stop_s = self._now_s()
            self.odom.clear()
            self.truth.clear()

    @staticmethod
    def _scaled(message: Twist, scale: float) -> Twist:
        output = Twist()
        output.linear.x = message.linear.x * scale
        output.linear.y = message.linear.y * scale
        output.linear.z = message.linear.z * scale
        output.angular.x = message.angular.x * scale
        output.angular.y = message.angular.y * scale
        output.angular.z = message.angular.z * scale
        return output

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        now_s = self._now_s()
        ratio, odom_m, truth_m = motion_ratio(self.odom, self.truth)
        self.maximum_odom_progress_m = max(
            self.maximum_odom_progress_m, odom_m)
        if ratio is not None:
            self.minimum_motion_ratio = (
                ratio if self.minimum_motion_ratio is None
                else min(self.minimum_motion_ratio, ratio))
        command_fresh = (
            self.command_stamp_ns is not None
            and (now_ns - self.command_stamp_ns) / 1e9
            <= self.config.input_timeout_s)
        commanded_mps = (
            abs(float(self.latest_command.linear.x)) if command_fresh else 0.0)
        anomaly = (
            traction_observation_allowed(
                self.monitor_action, now_s, self.last_collision_stop_s,
                self.config.collision_clear_grace_s)
            and
            commanded_mps >= self.config.minimum_command_mps
            and odom_m >= self.config.minimum_odom_progress_m
            and ratio is not None
            and ratio < self.config.minimum_truth_to_odom_ratio)
        localization_stable = (
            command_fresh
            and self.last_truth is not None
            and self.last_truth_speed_mps <= self.config.standstill_mps)
        before = self.policy.state
        state = self.policy.update(
            now_s, anomaly=anomaly,
            localization_stable=localization_stable)
        if before == LOW_SPEED_RESUME and state == RECOVERED:
            self.odom.clear()
            self.truth.clear()
        scale = self.policy.velocity_scale()
        self.publisher.publish(self._scaled(self.latest_command, scale))
        self.output_messages += 1
        if not self.state_samples or self.state_samples[-1]['state'] != state:
            self.state_samples.append({
                'at_s': now_s, 'state': state, 'scale': scale,
                'motion_ratio': ratio, 'odom_progress_m': odom_m,
                'truth_progress_m': truth_m,
            })

    def write_evidence(self) -> None:
        """Persist the state transitions and simulator-only claim boundary."""
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps({
            'schema_version': 1,
            'detector': 'gazebo_truth_to_wheel_odom_progress_ratio',
            'deployment_scope': 'simulation_only',
            'production_analogue': 'AMCL_or_scan_localization_to_wheel_odom',
            'config': self.config.__dict__,
            'final_state': self.policy.state,
            'recovery_count': self.policy.recovery_count,
            'events': self.policy.events,
            'state_samples': self.state_samples,
            'minimum_motion_ratio': self.minimum_motion_ratio,
            'maximum_odom_progress_m': self.maximum_odom_progress_m,
            'output_messages': self.output_messages,
            'limitations': [
                'Gazebo ground truth is an evaluation oracle unavailable '
                'on the real robot.',
                'Thresholds are synthetic stress values, not measured '
                'floor parameters.',
            ],
        }, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def main(argv: list[str] | None = None) -> int:
    """Run the simulation-only velocity guard."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument(
        '--recovery-grace-s', type=float,
        default=GuardConfig.recovery_grace_s)
    args, ros_args = parser.parse_known_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') == '12':
        parser.error('physical robot domain 12 is forbidden')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        parser.error('simulation guard requires LOCALHOST discovery')
    config = GuardConfig(recovery_grace_s=args.recovery_grace_s)
    config.validate()
    rclpy.init(args=ros_args)
    node = TractionVelocityGuard(config, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_evidence()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
