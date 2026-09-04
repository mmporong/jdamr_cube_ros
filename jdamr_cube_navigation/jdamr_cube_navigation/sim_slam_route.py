"""Publish a deterministic square route only when Gazebo truth is present."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
import time

from geometry_msgs.msg import PoseStamped, Twist
import rclpy


@dataclass(frozen=True)
class Segment:
    """One constant-velocity route segment."""

    name: str
    duration_s: float
    linear_mps: float
    angular_radps: float


def square_segments(
        *, side_m: float = 0.8,
        linear_mps: float = -0.2,
        angular_radps: float = 0.65) -> list[Segment]:
    """Return a four-side route with settle and stop intervals."""
    side_duration_s = side_m / abs(linear_mps)
    turn_duration_s = (math.pi / 2.0) / abs(angular_radps)
    segments = [Segment('settle', 2.0, 0.0, 0.0)]
    for index in range(4):
        segments.append(Segment(
            f'side_{index + 1}', side_duration_s, linear_mps, 0.0))
        segments.append(Segment(
            f'turn_{index + 1}', turn_duration_s, 0.0,
            angular_radps))
    segments.append(Segment('stop', 2.0, 0.0, 0.0))
    return segments


def corridor_segments(
        *, distance_m: float = 14.0,
        linear_mps: float = 0.35,
        angular_radps: float = 0.5) -> list[Segment]:
    """Return one long out-and-back route with a 180-degree turn."""
    travel_duration_s = distance_m / abs(linear_mps)
    turn_duration_s = math.pi / abs(angular_radps)
    return [
        Segment('settle', 2.0, 0.0, 0.0),
        Segment('outbound', travel_duration_s, abs(linear_mps), 0.0),
        Segment('turnaround', turn_duration_s, 0.0, angular_radps),
        Segment('return', travel_duration_s, abs(linear_mps), 0.0),
        Segment('stop', 2.0, 0.0, 0.0),
    ]


def segment_at(segments: list[Segment], elapsed_s: float) -> Segment | None:
    """Return the active route segment at an elapsed simulation time."""
    if elapsed_s < 0.0:
        return None
    cursor_s = 0.0
    for segment in segments:
        cursor_s += segment.duration_s
        if elapsed_s < cursor_s:
            return segment
    return None


def _twist(segment: Segment | None) -> Twist:
    message = Twist()
    if segment is not None:
        message.linear.x = segment.linear_mps
        message.angular.z = segment.angular_radps
    return message


def main(argv: list[str] | None = None) -> int:
    """Run the simulation-only route and stop with repeated zero commands."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', choices=('square', 'corridor'),
                        default='square')
    parser.add_argument('--side-m', type=float, default=0.8)
    parser.add_argument('--distance-m', type=float, default=14.0)
    parser.add_argument('--linear-mps', type=float, default=-0.2)
    parser.add_argument('--angular-radps', type=float, default=0.65)
    args, ros_args = parser.parse_known_args(argv)
    if args.side_m <= 0.0:
        parser.error('--side-m must be positive')
    if args.distance_m <= 0.0:
        parser.error('--distance-m must be positive')
    if args.linear_mps == 0.0 or args.angular_radps == 0.0:
        parser.error('route speeds must be non-zero')
    if os.environ.get('ROS_DOMAIN_ID') == '12':
        parser.error('physical robot domain 12 is forbidden')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        parser.error('ROS_AUTOMATIC_DISCOVERY_RANGE must be LOCALHOST')

    rclpy.init(args=ros_args)
    node = rclpy.create_node('sim_slam_route')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    truth_received = False

    def truth_callback(_message: PoseStamped) -> None:
        nonlocal truth_received
        truth_received = True

    node.create_subscription(
        PoseStamped, '/ground_truth_pose', truth_callback, 10)
    wait_deadline = time.monotonic() + 15.0
    while not truth_received and time.monotonic() < wait_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not truth_received:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Gazebo ground-truth pose was not received')

    if args.route == 'corridor':
        segments = corridor_segments(
            distance_m=args.distance_m,
            linear_mps=args.linear_mps,
            angular_radps=args.angular_radps)
        commanded_path_m = 2.0 * args.distance_m
    else:
        segments = square_segments(
            side_m=args.side_m,
            linear_mps=args.linear_mps,
            angular_radps=args.angular_radps)
        commanded_path_m = 4.0 * args.side_m
    start_ns = node.get_clock().now().nanoseconds
    total_duration_s = sum(segment.duration_s for segment in segments)
    while rclpy.ok():
        elapsed_s = (
            node.get_clock().now().nanoseconds - start_ns) * 1e-9
        segment = segment_at(segments, elapsed_s)
        publisher.publish(_twist(segment))
        rclpy.spin_once(node, timeout_sec=0.02)
        if segment is None:
            break
        time.sleep(0.03)
    for _ in range(10):
        publisher.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.02)
    print(json.dumps({
        'completed': True,
        'route': args.route,
        'side_m': args.side_m,
        'distance_m': args.distance_m,
        'commanded_path_m': commanded_path_m,
        'total_duration_s': total_duration_s,
        'segments': [segment.name for segment in segments],
    }, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
