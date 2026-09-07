#!/usr/bin/env python3
"""Publish evaluation-only map-to-odom TF from Gazebo ground truth."""

from __future__ import annotations

from collections import deque
import math

from geometry_msgs.msg import PoseStamped, TransformStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from tf2_ros import TransformBroadcaster


def normalize_yaw(yaw_rad: float) -> float:
    """Normalize one planar heading to the closed-open principal interval."""
    return math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Project a finite quaternion onto planar yaw."""
    values = (x, y, z, w)
    if any(not math.isfinite(value) for value in values):
        raise ValueError('quaternion must be finite')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        raise ValueError('quaternion norm must be positive')
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def planar_map_to_odom(
        map_to_base: tuple[float, float, float],
        odom_to_base: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return map-to-odom so its composition with odom-to-base is GT."""
    if any(not math.isfinite(value)
           for value in (*map_to_base, *odom_to_base)):
        raise ValueError('planar poses must be finite')
    map_x, map_y, map_yaw = map_to_base
    odom_x, odom_y, odom_yaw = odom_to_base
    yaw = normalize_yaw(map_yaw - odom_yaw)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        map_x - (cosine * odom_x - sine * odom_y),
        map_y - (sine * odom_x + cosine * odom_y),
        yaw,
    )


def stamp_ns(message) -> int:
    """Convert a ROS message header stamp to integer nanoseconds."""
    return (int(message.header.stamp.sec) * 1_000_000_000 +
            int(message.header.stamp.nanosec))


class GroundTruthLocalization(Node):
    """Own the sole evaluation map-to-odom transform authority."""

    def __init__(self) -> None:
        super().__init__('ground_truth_localization')
        self.declare_parameter('ground_truth_topic', '/ground_truth_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('ground_truth_frame', 'g005_frontier')
        self.declare_parameter('max_pair_age_s', 0.05)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._ground_truth_frame = str(
            self.get_parameter('ground_truth_frame').value)
        max_pair_age_s = float(self.get_parameter('max_pair_age_s').value)
        if (not self._map_frame or not self._odom_frame or
                not self._base_frame or not self._ground_truth_frame or
                not math.isfinite(max_pair_age_s) or
                max_pair_age_s <= 0.0):
            raise ValueError('invalid ground-truth localization parameters')
        self._max_pair_age_ns = round(max_pair_age_s * 1_000_000_000)
        self._odom_samples = deque(maxlen=32)
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('ground_truth_topic').value),
            self._on_ground_truth, qos_profile_sensor_data)

    def _on_odom(self, message: Odometry) -> None:
        if (message.header.frame_id.lstrip('/') !=
                self._odom_frame.lstrip('/') or
                message.child_frame_id.lstrip('/') !=
                self._base_frame.lstrip('/')):
            self.get_logger().error('rejected odometry frame mismatch')
            return
        stamp = stamp_ns(message)
        if stamp <= 0:
            return
        pose = message.pose.pose
        try:
            yaw = quaternion_yaw(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        sample = (stamp, float(pose.position.x), float(pose.position.y), yaw)
        if any(not math.isfinite(value) for value in sample[1:]):
            self.get_logger().error('rejected non-finite odometry pose')
            return
        self._odom_samples.append(sample)

    def _on_ground_truth(self, message: PoseStamped) -> None:
        gt_stamp = stamp_ns(message)
        if (message.header.frame_id.lstrip('/') !=
                self._ground_truth_frame.lstrip('/')):
            self.get_logger().error('rejected ground-truth frame mismatch')
            return
        if gt_stamp <= 0 or not self._odom_samples:
            return
        odom = min(
            self._odom_samples, key=lambda item: abs(item[0] - gt_stamp))
        if abs(odom[0] - gt_stamp) > self._max_pair_age_ns:
            return
        pose = message.pose
        try:
            gt_yaw = quaternion_yaw(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
            x_m, y_m, yaw_rad = planar_map_to_odom(
                (float(pose.position.x), float(pose.position.y), gt_yaw),
                (odom[1], odom[2], odom[3]),
            )
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.translation.x = x_m
        transform.transform.translation.y = y_m
        transform.transform.rotation.z = math.sin(yaw_rad / 2.0)
        transform.transform.rotation.w = math.cos(yaw_rad / 2.0)
        self._broadcaster.sendTransform(transform)


def main(args=None) -> None:
    """Run the evaluation-only localization authority."""
    rclpy.init(args=args)
    node = None
    try:
        node = GroundTruthLocalization()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
