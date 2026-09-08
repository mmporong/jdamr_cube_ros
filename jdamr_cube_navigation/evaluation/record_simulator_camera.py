#!/usr/bin/env python3
"""Record a Gazebo camera's ROS image stream as portfolio evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import threading
import time

import cv2
from cv_bridge import CvBridge  # noqa: I201
import rclpy  # noqa: I201
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image  # noqa: I201


class CameraRecorder(Node):
    """Write source pixels without overlays and retain clock alignment."""

    def __init__(self, topic: str, output: Path, metadata: Path,
                 ready_file: Path, fps: float) -> None:
        """Subscribe to one image topic and initialize output state."""
        super().__init__('jdamr_simulator_camera_recorder')
        self.topic = topic
        self.output = output
        self.metadata = metadata
        self.ready_file = ready_file
        self.fps = fps
        self.bridge = CvBridge()
        self.writer: cv2.VideoWriter | None = None
        self.frames = 0
        self.width: int | None = None
        self.height: int | None = None
        self.source_encoding: str | None = None
        self.first_frame_steady_ns: int | None = None
        self.last_frame_steady_ns: int | None = None
        self.first_frame_wall_ns: int | None = None
        self.last_frame_wall_ns: int | None = None
        self.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        if self.writer is None:
            self.height, self.width = frame.shape[:2]
            self.source_encoding = message.encoding
            self.output.parent.mkdir(parents=True, exist_ok=True)
            self.writer = cv2.VideoWriter(
                str(self.output), cv2.VideoWriter_fourcc(*'mp4v'), self.fps,
                (self.width, self.height))
            if not self.writer.isOpened():
                raise RuntimeError(f'video writer unavailable: {self.output}')
            self.first_frame_steady_ns = time.monotonic_ns()
            self.first_frame_wall_ns = time.time_ns()
            self.ready_file.write_text(
                f'{self.width}x{self.height}@{self.fps:g}\n', encoding='utf-8')
        self.writer.write(frame)
        self.frames += 1
        self.last_frame_steady_ns = time.monotonic_ns()
        self.last_frame_wall_ns = time.time_ns()

    def finalize(self) -> None:
        """Close the codec and persist capture provenance."""
        if self.writer is not None:
            self.writer.release()
        duration_s = None
        if (self.first_frame_steady_ns is not None
                and self.last_frame_steady_ns is not None):
            duration_s = (
                self.last_frame_steady_ns - self.first_frame_steady_ns) / 1e9
        document = {
            'schema_version': 1,
            'source': 'gazebo_camera_sensor',
            'topic': self.topic,
            'output': str(self.output.resolve()),
            'codec': 'mp4v',
            'requested_fps': self.fps,
            'source_encoding': self.source_encoding,
            'width': self.width,
            'height': self.height,
            'frames': self.frames,
            'first_frame_steady_ns': self.first_frame_steady_ns,
            'last_frame_steady_ns': self.last_frame_steady_ns,
            'first_frame_wall_ns': self.first_frame_wall_ns,
            'last_frame_wall_ns': self.last_frame_wall_ns,
            'capture_duration_s': duration_s,
        }
        self.metadata.write_text(
            json.dumps(document, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')


def main() -> int:
    """Record until SIGINT/SIGTERM and finalize the video atomically."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--metadata', required=True, type=Path)
    parser.add_argument('--ready-file', required=True, type=Path)
    parser.add_argument('--fps', type=float, default=10.0)
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error('--fps must be positive')
    for path in (args.output, args.metadata, args.ready_file):
        if path.exists():
            parser.error(f'output already exists: {path}')

    rclpy.init()
    recorder = CameraRecorder(
        args.topic, args.output, args.metadata, args.ready_file, args.fps)
    interrupted = False
    stop_requested = threading.Event()

    def request_stop(_signum, _frame) -> None:
        nonlocal interrupted
        interrupted = True
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(recorder, timeout_sec=0.1)
    finally:
        recorder.finalize()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if interrupted and recorder.frames > 0 else 2


if __name__ == '__main__':
    raise SystemExit(main())
