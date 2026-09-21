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
from jdamr_cube_navigation.depth_obstacle_core import _depth_view
from jdamr_cube_navigation.depth_obstacle_filter import camera_intrinsics
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


LATEST_SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def decode_depth_observation(
        info, image, optical_frame, calibration_model,
        now_s, max_image_age_s):
    """Validate synchronized geometry, freshness and the ROS depth buffer."""
    stamp_s = (image.header.stamp.sec
               + image.header.stamp.nanosec * 1e-9)
    age_s = now_s - stamp_s
    if (not math.isfinite(stamp_s) or stamp_s <= 0.0
            or not math.isfinite(age_s)
            or not 0.0 <= age_s <= max_image_age_s):
        raise ValueError('stale_or_future_depth')
    if str(image.encoding).upper() != '16UC1':
        raise ValueError(f'unsupported_depth_encoding:{image.encoding}')
    values = camera_intrinsics(
        info, image, optical_frame, calibration_model)
    intrinsics = CameraIntrinsics(
        width=image.width,
        height=image.height,
        focal_x_px=values[0],
        focal_y_px=values[1],
        center_x_px=values[2],
        center_y_px=values[3],
    )
    depth_mm = _depth_view(
        image.data, image.width, image.height, image.step,
        image.encoding, image.is_bigendian)
    return depth_mm, intrinsics, stamp_s, age_s


class DepthBoxParkingNode(Node):
    """Observe a box top without owning or publishing velocity commands."""

    def __init__(self):
        """Configure a read-only observer with bounded input freshness."""
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
            ('max_image_age_s', 0.5),
            ('optical_frame', 'camera_color_optical_frame'),
            ('calibration_model', 'rectified_projection'),
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
        self._max_image_age_s = float(
            self.get_parameter('max_image_age_s').value)
        if not math.isfinite(processing_hz) or processing_hz <= 0.0:
            raise ValueError('processing_hz must be finite and positive')
        if (not math.isfinite(self._max_image_age_s)
                or self._max_image_age_s <= 0.0):
            raise ValueError('max_image_age_s must be finite and positive')
        self._optical_frame = str(
            self.get_parameter('optical_frame').value)
        self._calibration_model = str(
            self.get_parameter('calibration_model').value)
        self._minimum_period_s = 1.0 / processing_hz
        self._last_processed_s = -math.inf
        self._camera_info = None
        self._status = self.create_publisher(
            String, '/box_parking/perception_status', 10)
        self.create_subscription(
            CameraInfo, '/camera/depth/camera_info',
            self._on_camera_info, LATEST_SENSOR_QOS)
        self.create_subscription(
            Image, '/camera/depth/image_raw',
            self._on_depth, LATEST_SENSOR_QOS)
        self.get_logger().info(
            'depth box parking perception started; '
            'velocity output is disabled')

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._camera_info = message

    def _publish(self, document: dict) -> None:
        document = {**document, 'control_ready': False}
        message = String()
        message.data = json.dumps(
            document, ensure_ascii=False, separators=(',', ':'))
        try:
            self._status.publish(message)
        except RCLError:
            if rclpy.ok():
                raise

    def _reject(self, reason: str, **details) -> None:
        """Clear consecutive stability after any unusable observation."""
        self._stability.update(None)
        self._publish({
            'detected': False,
            'stable': False,
            'stable_frame_count': 0,
            'reason': reason,
            **details,
        })

    def _on_depth(self, message: Image) -> None:
        now_s = time.monotonic()
        if now_s - self._last_processed_s < self._minimum_period_s:
            return
        self._last_processed_s = now_s
        if self._camera_info is None:
            self._reject('camera_info_missing')
            return
        try:
            ros_now_s = self.get_clock().now().nanoseconds * 1e-9
            depth_mm, intrinsics, stamp_s, age_s = decode_depth_observation(
                self._camera_info, message, self._optical_frame,
                self._calibration_model, ros_now_s,
                self._max_image_age_s)
            detector = (
                detect_box_front
                if self._surface_mode == 'front'
                else detect_box_top
            )
            detection = detector(depth_mm, intrinsics, self._config)
        except (AttributeError, ValueError, TypeError,
                OverflowError, IndexError) as error:
            self._reject(str(error) or type(error).__name__)
            return
        except np.linalg.LinAlgError as error:
            self._reject(f'numerical_detection_failure:{error}')
            return
        input_age_s = age_s
        age_s = self.get_clock().now().nanoseconds * 1e-9 - stamp_s
        timing = {
            'input_age_s': input_age_s,
            'processing_duration_s': time.monotonic() - now_s,
        }
        stable = self._stability.update(detection)
        if detection is None:
            self._publish({
                'stamp_s': stamp_s,
                'age_s': age_s,
                **timing,
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
            'age_s': age_s,
            **timing,
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
