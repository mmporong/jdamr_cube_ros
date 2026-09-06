#!/usr/bin/env python3
"""Finalize an offline Cartographer replay through persistent ROS clients."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from cartographer_ros_msgs.srv import FinishTrajectory
import rclpy
from rclpy.node import Node


class ReplayFinalizer(Node):
    """Keep service discovery alive before the bag finishes."""

    def __init__(self):
        super().__init__('cartographer_replay_finalizer')
        self.finish_client = self.create_client(
            FinishTrajectory, '/finish_trajectory')

    def wait_services(self, timeout_s: float) -> bool:
        """Wait for the installed finish service using wall-clock bounds."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.finish_client.service_is_ready():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def call(self, client, request, timeout_s: float):
        """Call one discovered service with a wall-clock timeout."""
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done() or future.exception() is not None:
            raise RuntimeError('Cartographer finalization service timed out')
        return future.result()

    def finalize(self, timeout_s: float) -> list[str]:
        """Finish the trajectory before the node's shutdown optimization."""
        stages = []
        finish = self.call(
            self.finish_client, FinishTrajectory.Request(trajectory_id=0),
            timeout_s)
        if finish.status.code != 0:
            raise RuntimeError(f'finish trajectory failed: {finish.status}')
        stages.append('finish_trajectory_ok')
        return stages


def main() -> int:
    """Wait for a trigger and persist finalization response evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trigger', type=Path, required=True)
    parser.add_argument('--sidecar', type=Path, required=True)
    parser.add_argument('--timeout-s', type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = ReplayFinalizer()
    document = {'schema_version': 1, 'stages': [], 'status': 'INVALID'}
    try:
        if not node.wait_services(args.timeout_s):
            raise RuntimeError('Cartographer finish service unavailable')
        while not args.trigger.is_file():
            rclpy.spin_once(node, timeout_sec=0.1)
        document['stages'] = node.finalize(args.timeout_s)
        document['status'] = 'PASS'
        for stage in document['stages']:
            print(f'evidence_stage={stage}', flush=True)
    except Exception as error:
        document['error'] = f'{type(error).__name__}: {error}'
    finally:
        args.sidecar.write_text(
            json.dumps(document, indent=2), encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if document['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
