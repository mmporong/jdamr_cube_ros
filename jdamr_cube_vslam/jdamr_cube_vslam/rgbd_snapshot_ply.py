"""Write one synchronized Astra S RGB-D frame as a metric colored PLY."""

import argparse
import json
from pathlib import Path
import struct

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def image_stamp_s(message: Image) -> float:
    """Convert a ROS image timestamp to seconds."""
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def project_registered_depth(
        depth_mm: np.ndarray, color_rgb: np.ndarray,
        camera_info: CameraInfo, pixel_step: int = 2,
        minimum_depth_m: float = 0.20,
        maximum_depth_m: float = 5.0) -> tuple[np.ndarray, np.ndarray]:
    """Project registered depth into the color optical frame."""
    depth_height_px, depth_width_px = depth_mm.shape
    color_resized = cv2.resize(
        color_rgb, (depth_width_px, depth_height_px),
        interpolation=cv2.INTER_AREA)
    scale_x = depth_width_px / camera_info.width
    scale_y = depth_height_px / camera_info.height
    focal_x_px = camera_info.k[0] * scale_x
    focal_y_px = camera_info.k[4] * scale_y
    center_x_px = camera_info.k[2] * scale_x
    center_y_px = camera_info.k[5] * scale_y
    if focal_x_px <= 0.0 or focal_y_px <= 0.0:
        raise ValueError('camera_info focal length must be positive')

    rows_px, columns_px = np.mgrid[
        0:depth_height_px:pixel_step,
        0:depth_width_px:pixel_step,
    ]
    depth_m = depth_mm[::pixel_step, ::pixel_step].astype(np.float32) * 0.001
    valid = (depth_m >= minimum_depth_m) & (depth_m <= maximum_depth_m)
    z_m = depth_m[valid]
    x_m = (columns_px[valid] - center_x_px) * z_m / focal_x_px
    y_m = (rows_px[valid] - center_y_px) * z_m / focal_y_px
    points_m = np.column_stack((x_m, y_m, z_m))
    colors_rgb = color_resized[::pixel_step, ::pixel_step][valid]
    return points_m, colors_rgb


def write_binary_ply(path: Path, points_m: np.ndarray,
                     colors_rgb: np.ndarray) -> None:
    """Write an efficient little-endian colored point cloud."""
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        'ply\n'
        'format binary_little_endian 1.0\n'
        f'element vertex {len(points_m)}\n'
        'property float x\n'
        'property float y\n'
        'property float z\n'
        'property uchar red\n'
        'property uchar green\n'
        'property uchar blue\n'
        'end_header\n'
    ).encode('ascii')
    with path.open('wb') as stream:
        stream.write(header)
        for point_m, color_rgb in zip(points_m, colors_rgb):
            stream.write(struct.pack(
                '<fffBBB',
                float(point_m[0]), float(point_m[1]), float(point_m[2]),
                int(color_rgb[0]), int(color_rgb[1]), int(color_rgb[2]),
            ))


class RgbdSnapshotNode(Node):
    """Synchronize one RGB-D pair and persist a metric point cloud."""

    def __init__(self, output_ply: Path, output_json: Path):
        super().__init__('jdamr_rgbd_snapshot_ply')
        from cv_bridge import CvBridge

        self._output_ply = output_ply
        self._output_json = output_json
        self._bridge = CvBridge()
        self._color = None
        self._depth = None
        self._camera_info = None
        self.done = False
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_color, sensor_qos)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._on_depth, sensor_qos)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self._on_camera_info, sensor_qos)

    def _on_color(self, message: Image) -> None:
        self._color = message
        self._try_write()

    def _on_depth(self, message: Image) -> None:
        self._depth = message
        self._try_write()

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._camera_info = message
        self._try_write()

    def _try_write(self) -> None:
        if self.done or self._color is None or self._depth is None \
                or self._camera_info is None:
            return
        sync_delta_s = abs(
            image_stamp_s(self._color) - image_stamp_s(self._depth))
        if sync_delta_s > 0.03:
            return
        color_rgb = self._bridge.imgmsg_to_cv2(
            self._color, desired_encoding='rgb8')
        depth_mm = self._bridge.imgmsg_to_cv2(
            self._depth, desired_encoding='16UC1')
        points_m, colors_rgb = project_registered_depth(
            depth_mm, color_rgb, self._camera_info)
        if len(points_m) == 0:
            return
        write_binary_ply(self._output_ply, points_m, colors_rgb)
        valid_depth_m = points_m[:, 2]
        summary = {
            'frame_id': self._depth.header.frame_id,
            'point_count': int(len(points_m)),
            'sync_delta_s': sync_delta_s,
            'minimum_depth_m': float(np.min(valid_depth_m)),
            'median_depth_m': float(np.median(valid_depth_m)),
            'maximum_depth_m': float(np.max(valid_depth_m)),
            'depth_registered_to_color': True,
        }
        self._output_json.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8')
        self.done = True


def _parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-ply', required=True, type=Path)
    parser.add_argument('--output-json', required=True, type=Path)
    return parser.parse_args(argv)


def main(args=None):
    """Wait for one valid RGB-D pair, then write PLY and exit."""
    parsed = _parse_args(args)
    rclpy.init()
    node = RgbdSnapshotNode(parsed.output_ply, parsed.output_json)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
