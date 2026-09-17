"""Publish perception-only box-parking observations from Astra depth."""

import json
import math
import time

from jdamr_cube_navigation.box_top_detection import (
    BoxTopConfig,
    CameraIntrinsics,
    detect_box_front,
    detect_box_top,
    DetectionStability,
)
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class DepthBoxParkingNode(Node):
    """Observe a box top without owning or publishing velocity commands."""

    def __init__(self):
        super().__init__('jdamr_depth_box_parking')
        defaults = BoxTopConfig()
        for name, value in (
            ('minimum_depth_m', defaults.minimum_depth_m),
            ('maximum_depth_m', defaults.maximum_depth_m),
            ('desired_standoff_m', defaults.desired_standoff_m),
            ('pixel_step', defaults.pixel_step),
            ('plane_distance_m', defaults.plane_distance_m),
            ('minimum_normal_y', defaults.minimum_normal_y),
            ('minimum_front_normal_z', defaults.minimum_front_normal_z),
            ('maximum_top_y_m', defaults.maximum_top_y_m),
            ('minimum_width_m', defaults.minimum_width_m),
            ('maximum_width_m', defaults.maximum_width_m),
            ('minimum_height_m', defaults.minimum_height_m),
            ('maximum_height_m', defaults.maximum_height_m),
            ('minimum_depth_extent_m', defaults.minimum_depth_extent_m),
            ('minimum_inliers', defaults.minimum_inliers),
            ('ransac_iterations', defaults.ransac_iterations),
            ('maximum_points', defaults.maximum_points),
            ('random_seed', defaults.random_seed),
            ('processing_hz', 5.0),
            ('stable_frames', 8),
            ('stable_distance_m', 0.03),
            ('stable_lateral_m', 0.03),
            ('stable_angle_deg', 3.0),
            ('surface_mode', 'front'),
        ):
            self.declare_parameter(name, value)
        self._config = BoxTopConfig(**{
            field: self.get_parameter(field).value
            for field in BoxTopConfig.__dataclass_fields__
        })
        self._stability = DetectionStability(
            required_frames=int(self.get_parameter('stable_frames').value),
            maximum_distance_spread_m=float(
                self.get_parameter('stable_distance_m').value),
            maximum_lateral_spread_m=float(
                self.get_parameter('stable_lateral_m').value),
            maximum_angle_spread_deg=float(
                self.get_parameter('stable_angle_deg').value),
        )
        self._surface_mode = str(
            self.get_parameter('surface_mode').value)
        if self._surface_mode not in {'front', 'top'}:
            raise ValueError('surface_mode must be front or top')
        processing_hz = float(self.get_parameter('processing_hz').value)
        if processing_hz <= 0.0:
            raise ValueError('processing_hz must be positive')
        self._minimum_period_s = 1.0 / processing_hz
        self._last_processed_s = -math.inf
        self._intrinsics = None
        self._status = self.create_publisher(
            String, '/box_parking/perception_status', 10)
        self.create_subscription(
            CameraInfo, '/camera/depth/camera_info',
            self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/depth/image_raw',
            self._on_depth, qos_profile_sensor_data)
        self.get_logger().info(
            'depth box parking perception started; velocity output is disabled')

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._intrinsics = CameraIntrinsics(
            width=message.width,
            height=message.height,
            focal_x_px=message.k[0],
            focal_y_px=message.k[4],
            center_x_px=message.k[2],
            center_y_px=message.k[5],
        )

    def _publish(self, document: dict) -> None:
        message = String()
        message.data = json.dumps(
            document, ensure_ascii=False, separators=(',', ':'))
        try:
            self._status.publish(message)
        except RCLError:
            if rclpy.ok():
                raise

    def _on_depth(self, message: Image) -> None:
        now_s = time.monotonic()
        if now_s - self._last_processed_s < self._minimum_period_s:
            return
        self._last_processed_s = now_s
        if self._intrinsics is None:
            self._publish({
                'detected': False,
                'stable': False,
                'reason': 'camera_info_missing',
                'control_ready': False,
            })
            return
        if message.encoding != '16UC1':
            self._publish({
                'detected': False,
                'stable': False,
                'reason': f'unsupported_depth_encoding:{message.encoding}',
                'control_ready': False,
            })
            return
        depth_mm = np.frombuffer(message.data, dtype=np.uint16).reshape(
            message.height, message.width)
        detector = (
            detect_box_front
            if self._surface_mode == 'front'
            else detect_box_top
        )
        detection = detector(depth_mm, self._intrinsics, self._config)
        stable = self._stability.update(detection)
        stamp_s = (message.header.stamp.sec
                   + message.header.stamp.nanosec * 1e-9)
        if detection is None:
            self._publish({
                'stamp_s': stamp_s,
                'frame_id': message.header.frame_id,
                'detected': False,
                'stable': False,
                'stable_frame_count': self._stability.frame_count,
                'reason': 'no_box_surface_candidate',
                'control_ready': False,
            })
            return
        self._publish({
            'stamp_s': stamp_s,
            'frame_id': message.header.frame_id,
            'detected': True,
            'stable': stable,
            'stable_frame_count': self._stability.frame_count,
            'surface_kind': detection.surface_kind,
            'front_distance_m': detection.front_distance_m,
            'desired_standoff_m': self._config.desired_standoff_m,
            'standoff_error_m': (
                detection.front_distance_m
                - self._config.desired_standoff_m),
            'lateral_error_m': detection.center_x_m,
            'edge_angle_deg': math.degrees(detection.edge_angle_rad),
            'width_m': detection.width_m,
            'height_m': detection.height_m,
            'depth_extent_m': detection.depth_extent_m,
            'plane_normal': list(detection.plane_normal),
            'plane_rms_m': detection.plane_rms_m,
            'inlier_count': detection.inlier_count,
            'candidate_count': detection.candidate_count,
            'confidence': detection.confidence,
            'reason': 'stable_detection' if stable else 'collecting_stability',
            'control_ready': False,
            'control_blocker': 'camera_extrinsic_and_physical_validation',
        })


def main(args=None):
    """Run the perception-only depth box-parking observer."""
    rclpy.init(args=args)
    node = DepthBoxParkingNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
