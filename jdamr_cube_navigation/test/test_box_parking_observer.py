"""ROS message boundary tests for the perception-only parking observer."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

from jdamr_cube_navigation.box_top_detection import BoxTopConfig
from jdamr_cube_navigation.depth_box_parking import (
    decode_depth_observation,
    DepthBoxParkingNode,
    LATEST_SENSOR_QOS,
)
import numpy as np
import pytest
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image


OPTICAL_FRAME = 'camera_color_optical_frame'


def test_depth_inputs_keep_only_latest_best_effort_sample():
    assert LATEST_SENSOR_QOS.history == HistoryPolicy.KEEP_LAST
    assert LATEST_SENSOR_QOS.depth == 1
    assert LATEST_SENSOR_QOS.reliability == ReliabilityPolicy.BEST_EFFORT
    assert LATEST_SENSOR_QOS.durability == DurabilityPolicy.VOLATILE
    source = Path(__file__).resolve().parents[1].joinpath(
        'jdamr_cube_navigation', 'depth_box_parking.py').read_text()
    assert source.count('self._on_camera_info, LATEST_SENSOR_QOS') == 1
    assert source.count('self._on_depth, LATEST_SENSOR_QOS') == 1


def _messages(depth, *, stamp_s=10.0, step=None, bigendian=False):
    depth = np.asarray(depth)
    height, width = depth.shape
    info = CameraInfo()
    info.header.frame_id = OPTICAL_FRAME
    info.width, info.height = width, height
    info.k = [float('nan')] * 9
    info.d = [0.0] * 5
    info.r = [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
    info.p = [5.0, 0.0, 1.0, 0.0,
              0.0, 5.0, 1.0, 0.0,
              0.0, 0.0, 1.0, 0.0]
    image = Image()
    image.header.frame_id = OPTICAL_FRAME
    image.header.stamp.sec = int(stamp_s)
    image.header.stamp.nanosec = int((stamp_s % 1.0) * 1e9)
    image.width, image.height = width, height
    image.encoding = '16UC1'
    image.is_bigendian = bigendian
    image.step = width * 2 if step is None else step
    image.data = depth.tobytes()
    return info, image


def test_uses_rectified_projection_when_k_contains_nan():
    """Astra's invalid K must not replace its valid projection matrix."""
    depth = np.array([[1000, 2000], [3000, 4000]], dtype='<u2')
    info, image = _messages(depth)
    decoded, intrinsics, stamp_s, age_s = decode_depth_observation(
        info, image, OPTICAL_FRAME, 'rectified_projection', 10.2, 0.5)
    np.testing.assert_array_equal(decoded, depth)
    assert intrinsics.focal_x_px == 5.0
    assert intrinsics.center_x_px == 1.0
    assert stamp_s == pytest.approx(10.0)
    assert age_s == pytest.approx(0.2)


def test_camera_info_callback_preserves_projection_message():
    """The callback retains P instead of prematurely extracting invalid K."""
    info, _ = _messages(np.ones((2, 2), dtype='<u2'))
    node = object.__new__(DepthBoxParkingNode)
    node._on_camera_info(info)
    assert node._camera_info is info


def test_honors_big_endian_rows_with_padding():
    """The shared decoder preserves row stride and byte order."""
    rows = np.array([[1000, 2000, 9999], [3000, 4000, 9999]], dtype='>u2')
    info, image = _messages(rows[:, :2], step=6, bigendian=True)
    image.data = rows.tobytes()
    decoded, _, _, _ = decode_depth_observation(
        info, image, OPTICAL_FRAME, 'rectified_projection', 10.1, 0.5)
    np.testing.assert_array_equal(decoded, [[1000, 2000], [3000, 4000]])


@pytest.mark.parametrize(
    ('mutate', 'reason'),
    [
        (lambda info, image: setattr(info, 'p', [float('nan')] * 12),
         'invalid_rectified_projection'),
        (lambda info, image: setattr(image, 'width', 3),
         'camera_dimensions_mismatch'),
        (lambda info, image: setattr(image, 'data', b'\x00'),
         'data buffer is smaller than image dimensions'),
        (lambda info, image: setattr(image, 'encoding', 'rgb8'),
         'unsupported_depth_encoding:rgb8'),
    ],
)
def test_rejects_invalid_ros_inputs_without_numpy_reshape_errors(
        mutate, reason):
    """Malformed ROS fields fail with a bounded input reason."""
    info, image = _messages(np.ones((2, 2), dtype='<u2'))
    mutate(info, image)
    with pytest.raises(ValueError, match=reason):
        decode_depth_observation(
            info, image, OPTICAL_FRAME, 'rectified_projection', 10.1, 0.5)


@pytest.mark.parametrize(('stamp_s', 'now_s'), [(9.0, 10.0), (11.0, 10.0)])
def test_rejects_stale_and_future_images(stamp_s, now_s):
    """Only measurements inside the configured age window are accepted."""
    info, image = _messages(
        np.ones((2, 2), dtype='<u2'), stamp_s=stamp_s)
    with pytest.raises(ValueError, match='stale_or_future_depth'):
        decode_depth_observation(
            info, image, OPTICAL_FRAME, 'rectified_projection', now_s, 0.5)


def test_rejection_callback_resets_stability_and_publishes_safe_state():
    """Every rejection clears history and keeps motion control disabled."""
    class StabilityRecorder:
        def __init__(self):
            self.updates = []

        def update(self, detection):
            self.updates.append(detection)
            return False

    node = object.__new__(DepthBoxParkingNode)
    node._stability = StabilityRecorder()

    class PublisherRecorder:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    node._status = PublisherRecorder()
    node._reject('invalid_rectified_projection')
    assert node._stability.updates == [None]
    assert json.loads(node._status.messages[0].data) == {
        'detected': False,
        'stable': False,
        'stable_frame_count': 0,
        'reason': 'invalid_rectified_projection',
        'control_ready': False,
    }


def test_invalid_camera_info_is_rejected_by_depth_callback_without_raising():
    """Malformed calibration reaches a safe callback rejection document."""
    info, image = _messages(np.ones((2, 2), dtype='<u2'))
    info.p = [float('nan')] * 12
    node = object.__new__(DepthBoxParkingNode)
    node._minimum_period_s = 0.0
    node._last_processed_s = -math.inf
    node._camera_info = info
    node._optical_frame = OPTICAL_FRAME
    node._calibration_model = 'rectified_projection'
    node._max_image_age_s = 0.5
    node._surface_mode = 'front'
    node._config = BoxTopConfig()
    node._stability = SimpleNamespace(update=lambda detection: False)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_100_000_000))
    messages = []
    node._status = SimpleNamespace(publish=messages.append)
    node._on_depth(image)
    document = json.loads(messages[0].data)
    assert document['reason'] == 'invalid_rectified_projection'
    assert document['detected'] is False
    assert document['stable'] is False
    assert document['control_ready'] is False


def test_observer_source_never_publishes_velocity_or_control_ready_true():
    """The observer source cannot acquire a velocity output path."""
    source = Path(__file__).resolve().parents[1].joinpath(
            'jdamr_cube_navigation', 'depth_box_parking.py').read_text()
    assert 'create_publisher(Twist' not in source
    assert "'/cmd_vel'" not in source
    assert "'control_ready': True" not in source


def test_age_includes_detection_processing_time(monkeypatch):
    """A fast input must not hide a slow detector in published diagnostics."""
    info, image = _messages(np.ones((2, 2), dtype='<u2') * 1000)
    node = object.__new__(DepthBoxParkingNode)
    node._minimum_period_s, node._last_processed_s = 0.0, -math.inf
    node._camera_info, node._optical_frame = info, OPTICAL_FRAME
    node._calibration_model, node._max_image_age_s = 'rectified_projection', .5
    node._surface_mode, node._config = 'front', BoxTopConfig()
    node._stability = SimpleNamespace(update=lambda detection: False, frame_count=0)
    clock_ticks = iter((10_100_000_000, 10_700_000_000))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=next(clock_ticks)))
    monkeypatch.setattr(
        'jdamr_cube_navigation.depth_box_parking.detect_box_front', lambda *_: None)
    messages = []
    node._status = SimpleNamespace(publish=messages.append)
    node._on_depth(image)
    document = json.loads(messages[0].data)
    assert document['input_age_s'] == pytest.approx(.1)
    assert document['age_s'] == pytest.approx(.7)
    assert document['processing_duration_s'] >= 0.
    assert document['control_ready'] is False
