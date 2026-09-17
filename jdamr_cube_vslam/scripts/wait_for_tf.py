#!/usr/bin/env python3
"""Wait until one TF chain is available without moving the robot."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    """Wait for a transform and return a process-friendly status."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--from-frame', required=True)
    parser.add_argument('--to-frame', required=True)
    parser.add_argument('--timeout', type=float, default=10.0)
    parser.add_argument('--use-sim-time', action='store_true')
    args = parser.parse_args()

    if args.timeout <= 0.0:
        parser.error('--timeout must be positive')

    rclpy.init()
    overrides = []
    if args.use_sim_time:
        overrides.append(Parameter('use_sim_time', value=True))
    node = Node('jdamr_vslam_tf_gate', parameter_overrides=overrides)
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node, spin_thread=False)

    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if buffer.can_transform(args.from_frame, args.to_frame, Time()):
                transform = buffer.lookup_transform(
                    args.from_frame, args.to_frame, Time())
                translation = transform.transform.translation
                print(
                    f'TF ready: {args.from_frame} -> {args.to_frame} '
                    f'xyz=({translation.x:.6f}, {translation.y:.6f}, '
                    f'{translation.z:.6f})')
                return 0
        print(
            f'TF unavailable after {args.timeout:.1f}s: '
            f'{args.from_frame} -> {args.to_frame}')
        return 1
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
