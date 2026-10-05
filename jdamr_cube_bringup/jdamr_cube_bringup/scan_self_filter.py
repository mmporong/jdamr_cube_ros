"""
Drop LiDAR returns that land on the robot itself (2026-10-05).

The bimanual build stands four frame columns from the lower frame to the 720 mm
tabletop, through the LiDAR plane. Their returns sit inside the footprint, so the
Collision Monitor would hold every motion and AMCL would match them to the map.
Returns inside the configured self boxes (base frame, plus a margin) become +inf:
"no return", which the costmaps do not clear with (inf_is_valid false), AMCL skips
as a max-range beam and the Collision Monitor ignores. The sectors behind the
columns stay unseen; they are reported at start so the blind angles are known.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def load_boxes(text):
    """Parse [{name, center_xy_m: [x, y], size_xy_m: [sx, sy]}, ...] into checked tuples."""
    boxes = []
    for entry in json.loads(text) if text else []:
        cx, cy = (float(v) for v in entry['center_xy_m'])
        sx, sy = (float(v) for v in entry['size_xy_m'])
        if not all(math.isfinite(v) for v in (cx, cy, sx, sy)) or sx <= 0.0 or sy <= 0.0:
            raise ValueError(f'invalid self box {entry!r}')
        boxes.append((str(entry.get('name', len(boxes))), cx, cy, sx, sy))
    return boxes


def filter_ranges(ranges, angle_min, angle_increment, laser_pose, boxes, margin_m):
    """
    Return (ranges with self returns set to +inf, count dropped).

    Vectorised: the filter sits in front of every scan consumer, the Collision
    Monitor included (pure Python took 0.53 ms per 1000 beams on a Ryzen 7 260,
    several times that on a Pi 4).
    """
    lx, ly, lyaw = laser_pose
    distances = np.asarray(ranges, dtype=float)
    angles = lyaw + angle_min + np.arange(distances.size) * angle_increment
    finite = np.isfinite(distances)
    safe = np.where(finite, distances, 0.0)
    x_m, y_m = lx + safe * np.cos(angles), ly + safe * np.sin(angles)
    hit = np.zeros(distances.size, dtype=bool)
    for _name, cx, cy, sx, sy in boxes:
        hit |= ((np.abs(x_m - cx) <= sx / 2 + margin_m)
                & (np.abs(y_m - cy) <= sy / 2 + margin_m))
    hit &= finite
    out = distances.copy()
    out[hit] = np.inf
    return out.tolist(), int(hit.sum())


def blind_sectors(laser_pose, boxes, margin_m):
    """Bearing span (deg, base frame) and nearest distance of each box seen from the laser."""
    lx, ly, _lyaw = laser_pose
    sectors = []
    for name, cx, cy, sx, sy in boxes:
        hx, hy = sx / 2 + margin_m, sy / 2 + margin_m
        corners = [(cx + dx, cy + dy) for dx in (-hx, hx) for dy in (-hy, hy)]
        centre = math.atan2(cy - ly, cx - lx)
        offsets = [math.atan2(math.sin(math.atan2(y - ly, x - lx) - centre),
                              math.cos(math.atan2(y - ly, x - lx) - centre))
                   for x, y in corners]
        sectors.append((name, math.degrees(centre + min(offsets)),
                        math.degrees(centre + max(offsets)),
                        min(math.hypot(x - lx, y - ly) for x, y in corners)))
    return sectors


# Same as the YDLIDAR driver publisher and the onboard bag's offered profile for /scan.
SCAN_OUT_QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                          reliability=QoSReliabilityPolicy.RELIABLE)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScanSelfFilter(Node):
    """Republish scan_raw as scan without the robot's own returns."""

    def __init__(self):
        super().__init__('scan_self_filter')
        self.declare_parameter('input_topic', 'scan_raw')
        self.declare_parameter('output_topic', 'scan')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('self_boxes_json', '')
        self.declare_parameter('margin_m', 0.01)
        self.boxes = load_boxes(self.get_parameter('self_boxes_json').value)
        self.margin_m = float(self.get_parameter('margin_m').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.laser_pose = None
        self.publisher = self.create_publisher(
            LaserScan, self.get_parameter('output_topic').value, SCAN_OUT_QOS)
        self.create_subscription(LaserScan, self.get_parameter('input_topic').value,
                                 self._scan, qos_profile_sensor_data)

    def _scan(self, scan):
        if self.laser_pose is None:
            try:
                transform = self.tf.lookup_transform(
                    self.base_frame, scan.header.frame_id, rclpy.time.Time()).transform
            except TransformException:
                # Fail closed: nothing goes out unfiltered, but say why scans are missing.
                self.get_logger().warning(
                    f'no transform {self.base_frame} <- {scan.header.frame_id}: '
                    'scans are dropped until it is available', throttle_duration_sec=5.0)
                return
            self.laser_pose = (transform.translation.x, transform.translation.y,
                               _yaw(transform.rotation))
            # The laser joint is fixed: stop receiving /tf (100 Hz costs CPU and delays scans).
            unregister = getattr(self.tf_listener, 'unregister', None)
            if unregister is not None:
                unregister()
            for name, start, end, near in blind_sectors(self.laser_pose, self.boxes,
                                                        self.margin_m):
                self.get_logger().info(
                    f'self box {name}: blind {start:.1f}..{end:.1f} deg, {near:.3f} m')
        ranges, _dropped = filter_ranges(scan.ranges, scan.angle_min, scan.angle_increment,
                                         self.laser_pose, self.boxes, self.margin_m)
        scan.ranges = ranges
        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
