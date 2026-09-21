"""Bounded RGB-D obstacle observer; never publishes motion commands."""

from dataclasses import fields
import json
import math
from pathlib import Path
import signal
import time

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.depth_obstacle_core import (
    DepthObstacleConfig, project_depth,
)
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def camera_intrinsics(info, image, optical_frame, calibration_model):
    """Require matching geometry, without silently resizing calibration."""
    if (info.header.frame_id != optical_frame
            or image.header.frame_id != optical_frame):
        raise ValueError('camera_frame_mismatch')
    if info.width != image.width or info.height != image.height:
        raise ValueError('camera_dimensions_mismatch')
    if (info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
            or any((info.roi.x_offset, info.roi.y_offset,
                    info.roi.width, info.roi.height, info.roi.do_rectify))):
        raise ValueError('camera_roi_or_binning_unsupported')
    if not np.all(np.isfinite(info.d)) or np.any(np.abs(info.d) > 1e-12):
        raise ValueError('unrectified_distortion_unsupported')
    if calibration_model == 'rectified_projection':
        projection = np.asarray(info.p).reshape(3, 4)
        if (not np.all(np.isfinite(projection))
                or not np.allclose(projection[:, 3], 0.0)
                or not np.allclose(projection[2], [0., 0., 1., 0.])
                or abs(projection[0, 1]) > 1e-12
                or abs(projection[1, 0]) > 1e-12
                or not np.allclose(np.asarray(info.r).reshape(3, 3),
                                   np.eye(3))):
            raise ValueError('invalid_rectified_projection')
        values = (projection[0, 0], projection[1, 1],
                  projection[0, 2], projection[1, 2])
    elif calibration_model == 'pinhole_k':
        matrix = np.asarray(info.k).reshape(3, 3)
        if (not np.all(np.isfinite(matrix))
                or not np.allclose(matrix[2], [0., 0., 1.])
                or abs(matrix[0, 1]) > 1e-12
                or abs(matrix[1, 0]) > 1e-12):
            raise ValueError('invalid_pinhole_intrinsics')
        values = matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2]
    else:
        raise ValueError('unknown_calibration_model')
    if values[0] <= 0.0 or values[1] <= 0.0:
        raise ValueError('invalid_camera_focal_length')
    return values


def transform_matrix(transform):
    """Convert a validated ROS rigid transform to a homogeneous matrix."""
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        raise ValueError('invalid_tf_quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    t = transform.translation
    result[:3, 3] = [t.x, t.y, t.z]
    if not np.all(np.isfinite(result)):
        raise ValueError('invalid_tf_translation')
    return result


def make_cloud(points, frame_id, stamp):
    """Pack little-endian XYZ float32, preserving the measurement stamp."""
    points = np.asarray(points, dtype='<f4')
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('cloud must be Nx3')
    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = stamp
    cloud.height, cloud.width = 1, len(points)
    cloud.fields = [PointField(name=name, offset=index*4,
                               datatype=PointField.FLOAT32, count=1)
                    for index, name in enumerate(('x', 'y', 'z'))]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.width * cloud.point_step
    cloud.is_dense = True
    cloud.data = points.tobytes()
    return cloud


def body_bounds_from_geometry(path):
    """Read measured chassis dimensions, not duplicate physical constants."""
    geometry = yaml.safe_load(Path(path).expanduser().read_text())
    names = ('front_to_wheel_axis', 'frame_length', 'wheel_outer_width')
    if any(geometry[name]['unit'] != 'm' for name in names):
        raise ValueError('geometry units must be metres')
    front, length, width = [geometry[name]['value'] for name in names]
    if (not all(math.isfinite(x) for x in (front, length, width))
            or not 0.0 < front < length or width <= 0.0):
        raise ValueError('invalid chassis geometry')
    return front-length, front, -width/2, width/2


class DepthObstacleFilter(Node):
    """Observe only; an opt-in Nav2 profile consumes these clouds."""

    def __init__(self, **kwargs):
        """Load the observer policy without starting a driver or navigator."""
        super().__init__('depth_obstacle_filter', **kwargs)
        defaults = {
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'obstacle_topic': '/depth_navigation/obstacles',
            'ray_topic': '/depth_navigation/rays',
            'status_topic': '/depth_navigation/status',
            'base_frame': 'base_footprint',
            'optical_frame': 'camera_color_optical_frame',
            'calibration_model': 'rectified_projection',
            'max_processing_hz': 5.0,
            'max_image_age_s': 0.5,
            'transform_timeout_s': 0.05,
            'geometry_file': str(
                Path(get_package_share_directory('jdamr_cube_description'))
                / 'config' / 'new_base_geometry.yaml'),
        }
        defaults.update({field.name: getattr(DepthObstacleConfig(), field.name)
                         for field in fields(DepthObstacleConfig)})
        self.declare_parameters('', [
            (name, value, ParameterDescriptor(read_only=True))
            for name, value in defaults.items()])
        self.values = {
            name: self.get_parameter(name).value for name in defaults}
        self.config = DepthObstacleConfig(**{
            field.name: self.values[field.name]
            for field in fields(DepthObstacleConfig)})
        for name in ('max_processing_hz', 'max_image_age_s'):
            if (not math.isfinite(self.values[name])
                    or self.values[name] <= 0.0):
                raise ValueError(f'{name} must be finite and positive')
        if not 0.0 <= self.values['transform_timeout_s'] <= 0.1:
            raise ValueError('transform_timeout_s must be within [0, 0.1]')
        self.body_bounds = body_bounds_from_geometry(
            self.values['geometry_file'])
        self.info = None
        self.last_attempt = -math.inf
        self.last_published_stamp = -1
        self.last_received_at = None
        self.processed_frames = 0
        self.status = {'state': 'waiting_for_depth', 'healthy': False}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.obstacle_pub = self.create_publisher(
            PointCloud2, self.values['obstacle_topic'], sensor_qos)
        self.ray_pub = self.create_publisher(
            PointCloud2, self.values['ray_topic'], sensor_qos)
        self.status_pub = self.create_publisher(
            String, self.values['status_topic'], 1)
        self.create_subscription(CameraInfo, self.values['camera_info_topic'],
                                 self.on_camera_info, sensor_qos)
        self.create_subscription(Image, self.values['depth_topic'],
                                 self.on_image, sensor_qos)
        self.create_timer(0.5, self.publish_status)

    def on_camera_info(self, info):
        """Retain calibration; static CameraInfo stamps need not be live."""
        self.info = info

    def set_status(self, state, healthy=False, **details):
        """Keep the most recent reason and bounded processing counters."""
        self.status = {'state': state, 'healthy': healthy,
                       'processed_frames': self.processed_frames, **details}

    def publish_status(self):
        """Publish diagnostics without manufacturing fresh observations."""
        if (self.last_received_at is not None
                and time.monotonic() - self.last_received_at >
                self.values['max_image_age_s']):
            self.set_status('depth_stream_timeout')
        self.status_pub.publish(String(data=json.dumps(self.status)))

    def on_image(self, image):
        """Process at a bounded rate and reject stale or malformed frames."""
        self.last_received_at = time.monotonic()
        if (self.last_received_at - self.last_attempt <
                1/self.values['max_processing_hz']):
            return
        self.last_attempt = self.last_received_at
        try:
            stamp = Time.from_msg(image.header.stamp)
        except ValueError:
            self.set_status('invalid_depth_stamp')
            return
        age_s = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if (stamp.nanoseconds <= 0
                or not 0.0 <= age_s <= self.values['max_image_age_s']):
            self.set_status('stale_or_future_depth', age_s=age_s)
            return
        if stamp.nanoseconds <= self.last_published_stamp:
            self.set_status('non_increasing_depth_stamp')
            return
        if self.info is None:
            self.set_status('waiting_for_camera_info')
            return
        try:
            intrinsics = camera_intrinsics(
                self.info, image, self.values['optical_frame'],
                self.values['calibration_model'])
            transform = self.tf_buffer.lookup_transform(
                self.values['base_frame'], image.header.frame_id, stamp,
                timeout=Duration(seconds=self.values['transform_timeout_s']))
            result = project_depth(
                image.data, image.width, image.height, image.step,
                image.encoding, image.is_bigendian, intrinsics,
                transform_matrix(transform.transform), self.body_bounds,
                self.config)
        except TransformException:
            self.set_status('missing_transform_at_measurement_time')
            return
        except (ValueError, TypeError) as error:
            self.set_status('invalid_depth_input', reason=str(error))
            return
        self.processed_frames += 1
        elapsed_ms = (time.monotonic() - self.last_attempt) * 1000
        age_s = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if not 0.0 <= age_s <= self.values['max_image_age_s']:
            self.set_status('expired_during_processing', age_s=age_s)
            return
        self.set_status(result.status, result.healthy,
                        valid_fraction=result.valid_fraction,
                        sampled_pixels=result.sampled_pixel_count,
                        valid_depth_pixels=result.valid_depth_count,
                        obstacle_points=len(result.obstacles_xyz),
                        ray_points=len(result.rays_xyz),
                        processing_ms=elapsed_ms, age_s=age_s)
        if not result.healthy:
            # No empty-cloud heartbeat: enabled consumers must time out.
            return
        self.obstacle_pub.publish(make_cloud(
            result.obstacles_xyz, self.values['base_frame'],
            image.header.stamp))
        self.ray_pub.publish(make_cloud(
            result.rays_xyz, self.values['base_frame'], image.header.stamp))
        self.last_published_stamp = stamp.nanoseconds


def spin_with_worker_yield(executor):
    """Yield the Python dispatcher so queued worker callbacks can progress."""
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.05)
        # Ready-but-busy callback groups can keep the dispatch thread runnable.
        # Yield its GIL without changing sensor timestamps or freshness limits.
        time.sleep(0.001)


def main(args=None):
    """Run TF reception separately from bounded image processing."""
    rclpy.init(args=args)
    node = DepthObstacleFilter()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        spin_with_worker_yield(executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch may forward a second SIGINT after the process-group signal.
        # Finish this node's cleanup before restoring the normal handler.
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous_handler)
