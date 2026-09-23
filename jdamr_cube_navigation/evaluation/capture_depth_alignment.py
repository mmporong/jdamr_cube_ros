#!/usr/bin/env python3
"""Capture one read-only depth/scan alignment sample from live ROS topics."""

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import sys
import time

from jdamr_cube_navigation.depth_obstacle_filter import (
    camera_intrinsics, transform_matrix,
)
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


TOPICS = {
    'depth': '/camera/depth/image_raw',
    'camera_info': '/camera/depth/camera_info',
    'rgb': '/camera/color/image_raw',
    'scan': '/scan',
    'status': '/depth_navigation/status',
}
BASE_FRAME = 'base_footprint'
LASER_FRAME = 'laser_link'
MAX_PAIR_DELTA_S = 0.1
MAX_PAIR_AGE_S = 0.5


def stamp_ns(message):
    """Return a ROS message header stamp as integer nanoseconds."""
    seconds_ns = int(message.header.stamp.sec) * 1_000_000_000
    return seconds_ns + int(message.header.stamp.nanosec)


def select_depth_scan_pair(depths, scans, now_ns):
    """Select the newest recent depth having a scan within the tolerance."""
    max_age_ns = int(MAX_PAIR_AGE_S * 1e9)
    max_delta_ns = int(MAX_PAIR_DELTA_S * 1e9)
    valid_depths = [
        item for item in depths
        if 0 < stamp_ns(item) <= now_ns
        and now_ns - stamp_ns(item) <= max_age_ns]
    valid_scans = [
        item for item in scans
        if 0 < stamp_ns(item) <= now_ns
        and now_ns - stamp_ns(item) <= max_age_ns]
    valid_depths.sort(key=stamp_ns, reverse=True)
    for depth in valid_depths:
        depth_ns = stamp_ns(depth)
        if not valid_scans:
            break
        scan = min(
            valid_scans, key=lambda item: abs(stamp_ns(item) - depth_ns))
        if abs(stamp_ns(scan) - depth_ns) <= max_delta_ns:
            return depth, scan
    raise ValueError('no_recent_depth_scan_pair_within_0.1s')


def image_array(message, channels=1):
    """Copy an Image while honoring row padding, endianness, and channels."""
    if channels == 1:
        if message.encoding.upper() != '16UC1':
            raise ValueError('depth_encoding_must_be_16UC1')
        dtype = np.dtype(('>' if message.is_bigendian else '<') + 'u2')
    else:
        if message.encoding.lower() not in {'rgb8', 'bgr8', 'rgba8', 'bgra8'}:
            raise ValueError('unsupported_rgb_encoding')
        dtype = np.dtype('u1')
        channels = 4 if 'a8' in message.encoding.lower() else 3
    packed = int(message.width) * channels * dtype.itemsize
    invalid_dimensions = message.width <= 0 or message.height <= 0
    undersized_step = message.step < packed
    undersized_data = len(message.data) < message.step * message.height
    if invalid_dimensions or undersized_step or undersized_data:
        raise ValueError('invalid_image_buffer_geometry')
    shape = ((message.height, message.width) if channels == 1 else
             (message.height, message.width, channels))
    strides = ((message.step, dtype.itemsize) if channels == 1 else
               (message.step, channels, 1))
    view = np.ndarray(shape, dtype=dtype, buffer=message.data,
                      strides=strides)
    result = np.array(view, dtype=np.uint16 if channels == 1 else np.uint8)
    if channels > 1:
        if message.encoding.lower().startswith('bgr'):
            result = result[..., [2, 1, 0, 3] if channels == 4 else [2, 1, 0]]
        if channels == 4:
            result = result[..., :3]
    return np.ascontiguousarray(result)


class AlignmentCapture(Node):
    """Subscribe only; retain bounded samples and write no ROS output."""

    def __init__(self):
        """Create read-only sensor subscriptions and a TF receive buffer."""
        super().__init__('capture_depth_alignment')
        self.frames = Counter()
        self.depths = deque(maxlen=256)
        self.scans = deque(maxlen=256)
        self.info = None
        self.rgb = None
        self.status_states = Counter()
        self.status_healthy = 0
        self.status_valid = 0
        self.processing_ms = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image, TOPICS['depth'], self._on_depth, qos)
        self.create_subscription(
            CameraInfo, TOPICS['camera_info'], self._on_info, qos)
        self.create_subscription(Image, TOPICS['rgb'], self._on_rgb, qos)
        self.create_subscription(
            LaserScan, TOPICS['scan'], self._on_scan, qos)
        self.create_subscription(
            String, TOPICS['status'], self._on_status, qos)

    def _on_depth(self, message):
        self.frames['depth'] += 1
        self.depths.append(message)

    def _on_info(self, message):
        self.frames['camera_info'] += 1
        self.info = message

    def _on_rgb(self, message):
        self.frames['rgb'] += 1
        self.rgb = message

    def _on_scan(self, message):
        self.frames['scan'] += 1
        self.scans.append(message)

    def _on_status(self, message):
        self.frames['status'] += 1
        try:
            document = json.loads(message.data)
            if not isinstance(document, dict):
                raise ValueError('status_json_must_be_an_object')
            state = str(document.get('state', 'missing_state'))
            healthy = document.get('healthy') is True
            elapsed = document.get('processing_ms')
            self.status_states[state] += 1
            self.status_valid += 1
            self.status_healthy += int(healthy)
            if isinstance(elapsed, (int, float)) and math.isfinite(elapsed):
                self.processing_ms.append(float(elapsed))
        except (TypeError, ValueError, json.JSONDecodeError):
            self.status_states['invalid_json'] += 1

    def capture(self):
        """Validate newest depth and nearest received scan, then snapshot."""
        missing = []
        if not self.depths:
            missing.append('depth')
        if not self.scans:
            missing.append('scan')
        if self.info is None:
            missing.append('camera_info')
        if missing:
            raise ValueError('required_data_missing:' + ','.join(missing))
        capture_ns = self.get_clock().now().nanoseconds
        depth, scan = select_depth_scan_pair(
            self.depths, self.scans, capture_ns)
        depth_ns = stamp_ns(depth)
        scan_ns = stamp_ns(scan)
        pair_delta_s = abs(scan_ns - depth_ns) / 1e9
        if scan.header.frame_id != LASER_FRAME:
            raise ValueError(
                f'laser_frame_mismatch:{scan.header.frame_id!r}')
        intrinsics = camera_intrinsics(
            self.info, depth, depth.header.frame_id, 'rectified_projection')
        depth_image = image_array(depth)
        timeout = Duration(seconds=0.05)
        try:
            depth_tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, depth.header.frame_id,
                Time.from_msg(depth.header.stamp), timeout=timeout)
            scan_tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, LASER_FRAME,
                Time.from_msg(scan.header.stamp), timeout=timeout)
        except TransformException as error:
            raise ValueError(f'transform_lookup_failed:{error}') from error
        scan_indices = np.arange(len(scan.ranges), dtype=np.float64)
        scan_angles = scan.angle_min + scan_indices * scan.angle_increment
        arrays = {
            'depth_raw_uint16': depth_image,
            'scan_ranges': np.asarray(scan.ranges, dtype=np.float32),
            'scan_angles': scan_angles,
            'scan_range_min_m': np.asarray(scan.range_min),
            'scan_range_max_m': np.asarray(scan.range_max),
            'intrinsics_fx_fy_cx_cy': np.asarray(intrinsics),
            'tf_optical_to_base': transform_matrix(depth_tf.transform),
            'tf_laser_to_base': transform_matrix(scan_tf.transform),
            'pair_delta_s': np.asarray(pair_delta_s),
            'depth_stamp_ns': np.asarray(depth_ns, dtype=np.int64),
            'scan_stamp_ns': np.asarray(scan_ns, dtype=np.int64),
        }
        rgb_details = {'available': False, 'depth_delta_s': None}
        if self.rgb is not None:
            rgb_ns = stamp_ns(self.rgb)
            try:
                arrays['rgb'] = image_array(self.rgb, channels=3)
                arrays['rgb_stamp_ns'] = np.asarray(rgb_ns, dtype=np.int64)
                rgb_details = {
                    'available': True,
                    'depth_delta_s': abs(rgb_ns - depth_ns) / 1e9,
                    'dimensions': [self.rgb.height, self.rgb.width],
                    'encoding': self.rgb.encoding,
                }
            except ValueError as error:
                rgb_details = {'available': False, 'error': str(error),
                               'depth_delta_s': abs(rgb_ns - depth_ns) / 1e9}
        details = {
            'pair_delta_s': pair_delta_s,
            'pairing_guarantee': (
                'nearest scan among received buffered messages; '
                'not exact sync'),
            'stamps_ns': {'capture': capture_ns,
                          'depth': depth_ns, 'scan': scan_ns},
            'frames': {'depth': depth.header.frame_id,
                       'scan': scan.header.frame_id,
                       'base': BASE_FRAME},
            'input': {
                'depth': {'dimensions': [depth.height, depth.width],
                          'encoding': depth.encoding, 'step': depth.step},
                'scan': {'sample_count': len(scan.ranges),
                         'angle_min': scan.angle_min,
                         'angle_increment': scan.angle_increment,
                         'range_min_m': scan.range_min,
                         'range_max_m': scan.range_max},
                'rgb': rgb_details,
            },
        }
        arrays['capture_stamp_ns'] = np.asarray(capture_ns, dtype=np.int64)
        details['age_at_capture_s'] = {
            'depth': (capture_ns - depth_ns) / 1e9,
            'scan': (capture_ns - scan_ns) / 1e9,
        }
        return arrays, details

    def diagnostics(self):
        """Return frame and status statistics collected during the run."""
        values = np.asarray(self.processing_ms, dtype=np.float64)
        return {
            'frame_counts': dict(self.frames),
            'status': {
                'state_counts': dict(self.status_states),
                'valid_count': self.status_valid,
                'healthy_ratio': (self.status_healthy / self.status_valid
                                  if self.status_valid else None),
                'healthy_ratio_basis': (
                    'healthy status messages / valid status JSON messages; '
                    'not frame success rate'),
                'processing_ms_p50': (float(np.percentile(values, 50))
                                      if values.size else None),
                'processing_ms_p95': (float(np.percentile(values, 95))
                                      if values.size else None),
            },
        }


def parse_args(argv):
    """Parse tool-only arguments after ROS arguments are removed."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=12.0)
    parsed = parser.parse_args(argv)
    if not math.isfinite(parsed.duration) or parsed.duration <= 0.0:
        parser.error('--duration must be finite and positive')
    return parsed


def main(args=None):
    """Run a finite capture and always leave a JSON result after spinning."""
    argv = sys.argv if args is None else [sys.argv[0], *args]
    parsed = parse_args(rclpy.utilities.remove_ros_args(args=argv)[1:])
    parsed.output_dir.mkdir(parents=True, exist_ok=False)
    json_path = parsed.output_dir / 'capture.json'
    npz_path = parsed.output_dir / 'capture.npz'
    started = time.monotonic()
    rclpy.init(args=args)
    node = AlignmentCapture()
    exit_code = 0
    summary = {'success': False, 'duration_requested_s': parsed.duration,
               'capture_attempts': 0, 'tf_lookup_retry_errors': [],
               'success_scope': 'snapshot_only_not_sensor_alignment_approval'}
    try:
        while rclpy.ok() and time.monotonic() - started < parsed.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        for attempt in range(4):
            summary['capture_attempts'] += 1
            try:
                arrays, details = node.capture()
                break
            except ValueError as error:
                lookup_failed = str(error).startswith(
                    'transform_lookup_failed:')
                if not lookup_failed or attempt == 3:
                    raise
                summary['tf_lookup_retry_errors'].append(str(error))
                # Let a delayed dynamic TF arrive, but keep exact stamp lookup.
                rclpy.spin_once(node, timeout_sec=0.25)
        np.savez_compressed(npz_path, **arrays)
        summary.update({'success': True, 'capture': details,
                        'npz': npz_path.name})
    except (KeyboardInterrupt, ValueError, OSError) as error:
        exit_code = 1
        message = str(error)
        if message.startswith('transform_lookup_failed:'):
            summary['failure_kind'] = 'transform_lookup_failed'
        elif message.startswith('required_data_missing:'):
            summary['failure_kind'] = 'required_data_missing'
        else:
            summary['failure_kind'] = 'capture_failed'
        summary['error'] = message
    finally:
        summary['duration_actual_s'] = time.monotonic() - started
        summary.update(node.diagnostics())
        json_path.write_text(json.dumps(
            summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
