#!/usr/bin/env python3
"""Inject deterministic simulation-only sensor and odometry faults."""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Any


@dataclass(frozen=True)
class FaultProfile:
    """Fault magnitudes applied by the simulation-only topic proxy."""

    scan_range_noise_stddev_m: float = 0.0
    scan_message_dropout_probability: float = 0.0
    scan_stamp_jitter_max_s: float = 0.0
    scan_transport_delay_s: float = 0.0
    odom_transport_delay_s: float = 0.0
    tf_transport_delay_s: float = 0.0
    wheel_translation_scale: float = 1.0
    wheel_slip_stddev_fraction: float = 0.0

    @classmethod
    def from_mapping(cls, values: dict[str, Any]) -> 'FaultProfile':
        """Validate and construct a profile from JSON-compatible values."""
        unknown = set(values) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f'unsupported fault fields: {sorted(unknown)}')
        profile = cls(**{name: float(value)
                         for name, value in values.items()})
        if any(not math.isfinite(value)
               for value in profile.__dict__.values()):
            raise ValueError('fault magnitudes must be finite')
        nonnegative = (
            profile.scan_range_noise_stddev_m,
            profile.scan_stamp_jitter_max_s,
            profile.scan_transport_delay_s,
            profile.odom_transport_delay_s,
            profile.tf_transport_delay_s,
            profile.wheel_slip_stddev_fraction,
        )
        if any(value < 0.0 for value in nonnegative):
            raise ValueError('fault magnitudes must be non-negative')
        if not 0.0 <= profile.scan_message_dropout_probability < 1.0:
            raise ValueError(
                'scan_message_dropout_probability must be in [0, 1)')
        if profile.wheel_translation_scale <= 0.0:
            raise ValueError('wheel_translation_scale must be positive')
        if profile.scan_range_noise_stddev_m > 1.0:
            raise ValueError('scan range noise must not exceed 1 m')
        if profile.scan_stamp_jitter_max_s > 1.0:
            raise ValueError('scan timestamp jitter must not exceed 1 s')
        if max(profile.scan_transport_delay_s,
               profile.odom_transport_delay_s,
               profile.tf_transport_delay_s) > 5.0:
            raise ValueError('transport delay must not exceed 5 s')
        if profile.wheel_slip_stddev_fraction > 1.0:
            raise ValueError('wheel slip standard deviation must not exceed 1')
        return profile


class FaultModel:
    """Pure deterministic fault model shared by the ROS proxy and tests."""

    def __init__(self, profile: FaultProfile, seed: int):
        """Create independent deterministic generators for one run seed."""
        self.profile = profile
        self._scan_rng = random.Random(seed * 17 + 1)
        self._range_rng = random.Random(seed * 17 + 2)
        self._wheel_rng = random.Random(seed * 17 + 3)

    def drop_scan(self) -> bool:
        """Return whether the next complete scan message is dropped."""
        return (self._scan_rng.random()
                < self.profile.scan_message_dropout_probability)

    def scan_stamp_jitter_s(self) -> float:
        """Return bounded zero-mean timestamp jitter in seconds."""
        limit = self.profile.scan_stamp_jitter_max_s
        return self._scan_rng.uniform(-limit, limit)

    def noisy_range_m(
            self, value_m: float, range_min_m: float,
            range_max_m: float) -> float:
        """Perturb one finite range while retaining LaserScan bounds."""
        if not math.isfinite(value_m):
            return value_m
        noisy = value_m + self._range_rng.gauss(
            0.0, self.profile.scan_range_noise_stddev_m)
        return min(max(noisy, range_min_m), range_max_m)

    def wheel_increment_scale(self) -> float:
        """Return deterministic scale plus independent incremental slip."""
        slip = self._wheel_rng.gauss(
            0.0, self.profile.wheel_slip_stddev_fraction)
        return max(0.0, self.profile.wheel_translation_scale * (1.0 + slip))


def main(argv: list[str] | None = None) -> int:
    """Run the ROS 2 proxy; physical robot domain 12 is rejected."""
    import argparse
    import copy
    import heapq
    import json
    import os
    from pathlib import Path

    from builtin_interfaces.msg import Time
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True,
                        help='Fault profile JSON object')
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--stats', type=Path, required=True)
    args, ros_args = parser.parse_known_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') == '12':
        parser.error('physical robot domain 12 is forbidden')
    profile = FaultProfile.from_mapping(json.loads(args.profile))
    model = FaultModel(profile, args.seed)

    def stamp_ns(stamp: Time) -> int:
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def set_stamp(stamp: Time, value_ns: int) -> None:
        value_ns = max(0, value_ns)
        stamp.sec, stamp.nanosec = divmod(value_ns, 1_000_000_000)

    class FaultInjector(Node):
        """Proxy raw Gazebo topics to the canonical SLAM topic names."""

        def __init__(self) -> None:
            super().__init__('sim_fault_injector')
            self._queue: list[tuple[int, int, Any, Any, str, int]] = []
            self._sequence = 0
            self._previous_raw_odom: Odometry | None = None
            self._previous_fault_odom: Odometry | None = None
            self._fault_odom_positions: dict[int, tuple[float, float]] = {}
            self._pending_tf: dict[int, TFMessage] = {}
            self._stats = {
                'schema_version': 1,
                'seed': args.seed,
                'profile': profile.__dict__,
                'scan_received': 0,
                'scan_first_stamp_ns': None,
                'scan_last_stamp_ns': None,
                'scan_published': 0,
                'scan_dropped': 0,
                'scan_noise_squared_sum_m2': 0.0,
                'scan_noise_samples': 0,
                'scan_abs_stamp_jitter_max_s': 0.0,
                'wheel_scale_sum': 0.0,
                'wheel_scale_squared_sum': 0.0,
                'wheel_scale_samples': 0,
                'wheel_tf_received': 0,
                'tf_wheel_corrections': 0,
                'delay_observations': {},
            }
            self._scan_pub = self.create_publisher(
                LaserScan, '/scan', qos_profile_sensor_data)
            self._odom_pub = self.create_publisher(Odometry, '/odom', 50)
            self._tf_pub = self.create_publisher(TFMessage, '/tf', 50)
            self.create_subscription(
                LaserScan, '/sim_raw/scan', self._scan,
                qos_profile_sensor_data)
            self.create_subscription(
                Odometry, '/sim_raw/odom', self._odom, 50)
            self.create_subscription(
                TFMessage, '/sim_raw/tf', self._tf, 50)
            self.create_timer(0.001, self._publish_due)

        def _enqueue(self, delay_s: float, publisher: Any, message: Any,
                     label: str) -> None:
            enqueued_ns = self.get_clock().now().nanoseconds
            due_ns = enqueued_ns + int(delay_s * 1e9)
            heapq.heappush(
                self._queue, (due_ns, self._sequence, publisher, message,
                              label, enqueued_ns))
            self._sequence += 1

        def _publish_due(self) -> None:
            now_ns = self.get_clock().now().nanoseconds
            while self._queue and self._queue[0][0] <= now_ns:
                _, _, publisher, message, label, enqueued_ns = heapq.heappop(
                    self._queue)
                publisher.publish(message)
                delay = (now_ns - enqueued_ns) * 1e-9
                observations = self._stats['delay_observations'].setdefault(
                    label, {'messages': 0, 'sum_s': 0.0, 'max_s': 0.0})
                observations['messages'] += 1
                observations['sum_s'] += delay
                observations['max_s'] = max(observations['max_s'], delay)
                if label == 'scan':
                    self._stats['scan_published'] += 1

        def _scan(self, raw: LaserScan) -> None:
            self._stats['scan_received'] += 1
            raw_stamp_ns = stamp_ns(raw.header.stamp)
            if self._stats['scan_first_stamp_ns'] is None:
                self._stats['scan_first_stamp_ns'] = raw_stamp_ns
            self._stats['scan_last_stamp_ns'] = raw_stamp_ns
            if model.drop_scan():
                self._stats['scan_dropped'] += 1
                return
            message = copy.deepcopy(raw)
            jitter_s = model.scan_stamp_jitter_s()
            jitter_ns = int(jitter_s * 1e9)
            self._stats['scan_abs_stamp_jitter_max_s'] = max(
                self._stats['scan_abs_stamp_jitter_max_s'], abs(jitter_s))
            set_stamp(message.header.stamp,
                      stamp_ns(message.header.stamp) + jitter_ns)
            noisy_ranges = []
            for value in message.ranges:
                noisy = model.noisy_range_m(
                    value, message.range_min, message.range_max)
                noisy_ranges.append(noisy)
                if math.isfinite(value):
                    self._stats['scan_noise_squared_sum_m2'] += (
                        noisy - value) ** 2
                    self._stats['scan_noise_samples'] += 1
            message.ranges = noisy_ranges
            self._enqueue(
                profile.scan_transport_delay_s, self._scan_pub, message,
                'scan')

        def _odom(self, raw: Odometry) -> None:
            message = copy.deepcopy(raw)
            if (self._previous_raw_odom is not None
                    and self._previous_fault_odom is not None):
                scale = model.wheel_increment_scale()
                self._stats['wheel_scale_sum'] += scale
                self._stats['wheel_scale_squared_sum'] += scale * scale
                self._stats['wheel_scale_samples'] += 1
                raw_pose = raw.pose.pose.position
                old_raw_pose = self._previous_raw_odom.pose.pose.position
                old_fault_pose = self._previous_fault_odom.pose.pose.position
                message.pose.pose.position.x = (
                    old_fault_pose.x + (raw_pose.x - old_raw_pose.x) * scale)
                message.pose.pose.position.y = (
                    old_fault_pose.y + (raw_pose.y - old_raw_pose.y) * scale)
                message.twist.twist.linear.x *= scale
                message.twist.twist.linear.y *= scale
            self._previous_raw_odom = copy.deepcopy(raw)
            self._previous_fault_odom = copy.deepcopy(message)
            message_stamp_ns = stamp_ns(message.header.stamp)
            self._fault_odom_positions[message_stamp_ns] = (
                message.pose.pose.position.x, message.pose.pose.position.y)
            while len(self._fault_odom_positions) > 200:
                self._fault_odom_positions.pop(next(iter(
                    self._fault_odom_positions)))
            pending_tf = self._pending_tf.pop(message_stamp_ns, None)
            if pending_tf is not None:
                self._correct_and_enqueue_tf(pending_tf)
            self._enqueue(
                profile.odom_transport_delay_s, self._odom_pub, message,
                'odom')

        def _correct_and_enqueue_tf(self, message: TFMessage) -> None:
            corrected = 0
            for transform in message.transforms:
                key = stamp_ns(transform.header.stamp)
                position = self._fault_odom_positions.get(key)
                if (position is not None
                        and transform.header.frame_id == 'odom'
                        and transform.child_frame_id == 'base_footprint'):
                    transform.transform.translation.x = position[0]
                    transform.transform.translation.y = position[1]
                    corrected += 1
            self._stats['tf_wheel_corrections'] += corrected
            self._enqueue(
                profile.tf_transport_delay_s, self._tf_pub,
                message, 'tf')

        def _tf(self, raw: TFMessage) -> None:
            message = copy.deepcopy(raw)
            wheel_fault_active = (
                profile.wheel_translation_scale != 1.0
                or profile.wheel_slip_stddev_fraction != 0.0)
            odom_stamps = [
                stamp_ns(transform.header.stamp)
                for transform in message.transforms
                if (transform.header.frame_id == 'odom'
                    and transform.child_frame_id == 'base_footprint')]
            self._stats['wheel_tf_received'] += len(odom_stamps)
            if (wheel_fault_active and odom_stamps
                    and odom_stamps[0] not in self._fault_odom_positions):
                self._pending_tf[odom_stamps[0]] = message
                return
            self._correct_and_enqueue_tf(message)

        def write_stats(self) -> None:
            """Persist observed injection counts and magnitudes on shutdown."""
            noise_samples = self._stats['scan_noise_samples']
            self._stats['scan_noise_rms_m'] = (
                math.sqrt(self._stats['scan_noise_squared_sum_m2']
                          / noise_samples)
                if noise_samples else None)
            wheel_samples = self._stats['wheel_scale_samples']
            self._stats['wheel_scale_mean'] = (
                self._stats['wheel_scale_sum'] / wheel_samples
                if wheel_samples else None)
            self._stats['wheel_scale_stddev'] = (
                math.sqrt(max(
                    0.0,
                    self._stats['wheel_scale_squared_sum'] / wheel_samples
                    - self._stats['wheel_scale_mean'] ** 2))
                if wheel_samples else None)
            first_stamp_ns = self._stats['scan_first_stamp_ns']
            last_stamp_ns = self._stats['scan_last_stamp_ns']
            self._stats['scan_observed_rate_hz'] = (
                (self._stats['scan_received'] - 1) * 1e9
                / (last_stamp_ns - first_stamp_ns)
                if (self._stats['scan_received'] > 1
                    and last_stamp_ns > first_stamp_ns) else None)
            self._stats['wheel_tf_pending_at_shutdown'] = len(self._pending_tf)
            for values in self._stats['delay_observations'].values():
                values['mean_s'] = values['sum_s'] / values['messages']
            args.stats.parent.mkdir(parents=True, exist_ok=True)
            args.stats.write_text(
                json.dumps(self._stats, ensure_ascii=False, indent=2),
                encoding='utf-8')

    rclpy.init(args=ros_args)
    node = FaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_stats()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
