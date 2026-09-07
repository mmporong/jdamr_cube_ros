"""Pure transform tests for G005 ground-truth localization."""

import math

from geometry_msgs.msg import PoseStamped

import jdamr_cube_navigation.g005_ground_truth_localization as localization
from jdamr_cube_navigation.g005_ground_truth_localization import (
    GroundTruthLocalization,
    normalize_yaw,
    planar_map_to_odom,
    quaternion_yaw,
)

from nav_msgs.msg import Odometry

import pytest

import rclpy


class BroadcasterProbe:
    def __init__(self):
        self.messages = []

    def sendTransform(self, message):
        self.messages.append(message)


@pytest.fixture
def localization_node():
    rclpy.init(args=[])
    node = GroundTruthLocalization()
    node._broadcaster = BroadcasterProbe()
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _odometry(stamp_ns, x_m=0.0, y_m=0.0, yaw_rad=0.0):
    message = Odometry()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_footprint'
    message.pose.pose.position.x = x_m
    message.pose.pose.position.y = y_m
    message.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return message


def _ground_truth(stamp_ns, x_m=0.0, y_m=0.0, yaw_rad=0.0):
    message = PoseStamped()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = 'g005_frontier'
    message.pose.position.x = x_m
    message.pose.position.y = y_m
    message.pose.orientation.z = math.sin(yaw_rad / 2.0)
    message.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return message


def _compose(left, right):
    cosine = math.cos(left[2])
    sine = math.sin(left[2])
    return (
        left[0] + cosine * right[0] - sine * right[1],
        left[1] + sine * right[0] + cosine * right[1],
        normalize_yaw(left[2] + right[2]),
    )


@pytest.mark.parametrize(
    'ground_truth,odometry',
    [
        ((4.0, -2.0, 0.0), (1.0, 3.0, 0.0)),
        ((2.0, 5.0, math.pi / 2.0), (3.0, 0.0, 0.0)),
        ((-1.0, 0.5, -2.8), (0.2, -0.7, 2.9)),
    ],
)
def test_planar_map_to_odom_recomposes_ground_truth(ground_truth, odometry):
    transform = planar_map_to_odom(ground_truth, odometry)
    recomposed = _compose(transform, odometry)
    assert recomposed == pytest.approx(ground_truth)


def test_quaternion_yaw_normalizes_scaled_input_and_rejects_zero():
    yaw = 1.2
    assert quaternion_yaw(
        0.0, 0.0, 2.0 * math.sin(yaw / 2.0),
        2.0 * math.cos(yaw / 2.0)) == pytest.approx(yaw)
    with pytest.raises(ValueError, match='norm'):
        quaternion_yaw(0.0, 0.0, 0.0, 0.0)


def test_normalize_yaw_wraps_without_changing_equivalent_heading():
    assert normalize_yaw(3.0 * math.pi) == pytest.approx(math.pi)
    assert normalize_yaw(-3.0 * math.pi) == pytest.approx(-math.pi)
    assert normalize_yaw(0.25) == pytest.approx(0.25)


def test_ground_truth_frame_mismatch_never_publishes_transform(
        localization_node):
    localization_node._on_odom(_odometry(1_000_000_000))
    message = _ground_truth(1_000_000_000)
    message.header.frame_id = 'unexpected_world'
    localization_node._on_ground_truth(message)
    assert localization_node._broadcaster.messages == []


def test_callbacks_publish_exact_stamped_recomposable_transform(
        localization_node):
    odometry = (1.0, -0.5, 0.4)
    ground_truth = (4.0, 2.0, -0.8)
    localization_node._on_odom(
        _odometry(1_000_000_000, *odometry))
    localization_node._on_ground_truth(
        _ground_truth(1_000_000_000, *ground_truth))
    assert len(localization_node._broadcaster.messages) == 1
    transform = localization_node._broadcaster.messages[0]
    assert transform.header.stamp.sec == 1
    assert transform.header.frame_id == 'map'
    assert transform.child_frame_id == 'odom'
    map_to_odom = (
        transform.transform.translation.x,
        transform.transform.translation.y,
        quaternion_yaw(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w),
    )
    assert _compose(map_to_odom, odometry) == pytest.approx(ground_truth)


def test_nearest_pair_is_used_and_stale_pair_is_dropped(localization_node):
    localization_node._on_odom(_odometry(940_000_000, 1.0))
    localization_node._on_odom(_odometry(1_040_000_000, 2.0))
    localization_node._on_ground_truth(_ground_truth(1_000_000_000, 5.0))
    transform = localization_node._broadcaster.messages[0]
    assert transform.transform.translation.x == pytest.approx(3.0)
    localization_node._broadcaster.messages.clear()
    localization_node._on_ground_truth(_ground_truth(1_100_000_000, 5.0))
    assert localization_node._broadcaster.messages == []


def test_invalid_odom_and_nonfinite_ground_truth_fail_closed(
        localization_node):
    odom = _odometry(1_000_000_000)
    odom.header.frame_id = 'wrong_odom'
    localization_node._on_odom(odom)
    assert list(localization_node._odom_samples) == []
    localization_node._on_odom(_odometry(1_000_000_000))
    ground_truth = _ground_truth(1_000_000_000)
    ground_truth.pose.position.x = math.nan
    localization_node._on_ground_truth(ground_truth)
    assert localization_node._broadcaster.messages == []


def test_main_shuts_down_context_when_constructor_fails(monkeypatch):
    def fail_constructor():
        raise ValueError('constructor failure')

    monkeypatch.setattr(
        localization, 'GroundTruthLocalization', fail_constructor)
    with pytest.raises(ValueError, match='constructor failure'):
        localization.main(args=[])
    assert not rclpy.ok()
