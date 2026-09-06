#!/usr/bin/env python3
"""Evaluation-only LaserScan fan-out with a freezeable monitor branch."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args

from sensor_msgs.msg import LaserScan

from std_srvs.srv import SetBool


def _stop_zone_points(message: LaserScan, contract: dict) -> list[list[float]]:
    zone = contract['stop_zone']
    transform = contract['scan_to_base_transform']
    cosine = math.cos(transform['yaw_rad'])
    sine = math.sin(transform['yaw_rad'])
    points = []
    for index, range_m in enumerate(message.ranges):
        if not math.isfinite(range_m):
            continue
        angle = message.angle_min + index * message.angle_increment
        scan_x = range_m * math.cos(angle)
        scan_y = range_m * math.sin(angle)
        x_m = transform['x_m'] + cosine * scan_x - sine * scan_y
        y_m = transform['y_m'] + sine * scan_x + cosine * scan_y
        if (zone['rear_m'] <= x_m <= zone['front_m']
                and abs(y_m) <= zone['half_width_m']):
            points.append([x_m, y_m])
    return points


class SimScanGate(Node):
    """Always forward navigation scans and selectively freeze monitor scans."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Create scan fan-out publishers and the freeze control service."""
        super().__init__('sim_scan_gate')
        self.args = args
        self.contract = json.loads(args.contract.read_text())
        self.contract_sha256 = hashlib.sha256(
            args.contract.read_bytes()).hexdigest()
        self.moving_observed = False
        self.pending_trigger = None
        self.reaction_armed = False
        self.arm_receive_steady_ns = None
        self.monitor_frozen = False
        self.navigation_publisher = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.monitor_publisher = self.create_publisher(
            LaserScan, '/collision_monitor_scan', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, '/sim_raw/scan', self.scan_callback,
            qos_profile_sensor_data)
        self.cmd_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        self.freeze_service = self.create_service(
            SetBool, '~/freeze_monitor', self.freeze_callback)
        self.arm_service = self.create_service(
            SetBool, '~/arm_reaction', self.arm_callback)

    def scan_callback(self, message: LaserScan) -> None:
        """Forward every scan to Nav2 and only fresh scans to the monitor."""
        self.navigation_publisher.publish(message)
        if not self.monitor_frozen:
            if (self.reaction_armed
                    and self.moving_observed
                    and self.pending_trigger is None
                    and not self.args.evidence.exists()):
                points = _stop_zone_points(message, self.contract)
                if len(points) >= 3:
                    source_path = Path(__file__).resolve()
                    stamp_ns = (
                        message.header.stamp.sec * 1_000_000_000
                        + message.header.stamp.nanosec)
                    self.pending_trigger = {
                        'schema_version': 1,
                        'run_id': self.args.run_id,
                        'scenario': self.args.scenario,
                        'seed': self.args.seed,
                        'contract_sha256': self.contract_sha256,
                        'source_path': str(source_path),
                        'source_size_bytes': (
                            source_path.stat().st_size),
                        'source_sha256': hashlib.sha256(
                            source_path.read_bytes()).hexdigest(),
                        'arm_receive_steady_ns': self.arm_receive_steady_ns,
                        'stamp_ns': stamp_ns,
                        'frame_id': message.header.frame_id,
                        'angle_min_rad': float(message.angle_min),
                        'angle_increment_rad': float(message.angle_increment),
                        'ranges_m': [
                            float(value) if math.isfinite(value) else None
                            for value in message.ranges],
                        'stop_zone_points': points,
                    }
                    self.pending_trigger['publish_steady_ns'] = (
                        time.monotonic_ns())
            self.monitor_publisher.publish(message)

    def cmd_callback(self, message: Twist) -> None:
        """Latch first post-trigger zero in the same process clock domain."""
        is_zero = (
            abs(message.linear.x) <= 1e-4
            and abs(message.linear.y) <= 1e-4
            and abs(message.angular.z) <= 1e-4)
        if not is_zero:
            self.moving_observed = True
            return
        if self.pending_trigger is None or self.args.evidence.exists():
            return
        zero_steady_ns = time.monotonic_ns()
        self.pending_trigger['zero_receive_steady_ns'] = zero_steady_ns
        self.pending_trigger['scan_publish_to_zero_receive_steady_s'] = (
            (zero_steady_ns - self.pending_trigger['publish_steady_ns']) / 1e9)
        temporary = self.args.evidence.with_suffix('.tmp')
        temporary.write_text(json.dumps(
            self.pending_trigger, indent=2, sort_keys=True) + '\n')
        temporary.replace(self.args.evidence)

    def arm_callback(
            self, request: SetBool.Request,
            response: SetBool.Response) -> SetBool.Response:
        """Arm one sudden-obstacle reaction capture."""
        if (request.data
                and self.args.scenario != 'sudden_obstacle_stop_resume'):
            response.success = False
            response.message = 'arming is valid only for sudden obstacle runs'
            return response
        self.reaction_armed = request.data
        if request.data:
            self.arm_receive_steady_ns = time.monotonic_ns()
        response.success = True
        response.message = 'armed' if request.data else 'disarmed'
        return response

    def freeze_callback(
            self, request: SetBool.Request,
            response: SetBool.Response) -> SetBool.Response:
        """Set the evaluation-only Collision Monitor branch state."""
        self.monitor_frozen = request.data
        response.success = True
        response.message = 'frozen' if request.data else 'forwarding'
        return response


def main(args=None) -> None:
    """Run the evaluation scan gate."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--contract', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--seed', type=int, required=True)
    parsed = parser.parse_args(remove_ros_args(args=args)[1:])
    rclpy.init(args=args)
    node = SimScanGate(parsed)
    try:
        spin_until_shutdown(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def spin_until_shutdown(node: Node) -> None:
    """Spin until a normal signal shutdown without hiding live-context errors."""
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except RCLError:
        if rclpy.ok():
            raise
