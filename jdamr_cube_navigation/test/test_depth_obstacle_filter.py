"""Exercise ROS message and callback contracts without a robot driver."""

import copy
from pathlib import Path
import time

from geometry_msgs.msg import TransformStamped
import jdamr_cube_navigation.depth_obstacle_filter as filter_module
from jdamr_cube_navigation.depth_obstacle_filter import (
    body_bounds_from_geometry, camera_intrinsics, DepthObstacleFilter,
    make_cloud, transform_matrix,
)
import numpy as np
import pytest
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer


ROOT = Path(__file__).resolve().parents[2]
GEOMETRY = ROOT / 'jdamr_cube_description/config/new_base_geometry.yaml'
OPTICAL = 'camera_color_optical_frame'


def test_dispatcher_yields_between_worker_submissions(monkeypatch):
    """Retain blocking idle waits while preventing a busy dispatch loop."""
    states = iter([True, True, False])
    events = []

    class Executor:
        def spin_once(self, timeout_sec):
            events.append(('spin', timeout_sec))

    monkeypatch.setattr(filter_module.rclpy, 'ok', lambda: next(states))
    monkeypatch.setattr(filter_module.time, 'sleep',
                        lambda delay: events.append(('yield', delay)))
    filter_module.spin_with_worker_yield(Executor())
    assert events == [('spin', 0.05), ('yield', 0.001),
                      ('spin', 0.05), ('yield', 0.001)]


def test_dispatcher_does_not_hide_callback_errors(monkeypatch):
    """Let main perform normal cleanup when the executor reports failure."""
    class Executor:
        def spin_once(self, timeout_sec):
            raise RuntimeError('callback failed')

    monkeypatch.setattr(filter_module.rclpy, 'ok', lambda: True)
    with pytest.raises(RuntimeError, match='callback failed'):
        filter_module.spin_with_worker_yield(Executor())


def messages():
    """Use synthetic pinhole geometry, not asserted physical calibration."""
    info = CameraInfo()
    info.header.frame_id = OPTICAL
    info.width, info.height = 3, 3
    info.k = [float('nan'), 0., float('nan'),
              0., float('nan'), float('nan'), 0., 0., 1.]
    info.p = [5., 0., 1., 0., 0., 5., 1., 0., 0., 0., 1., 0.]
    info.r = np.eye(3).ravel().tolist()
    info.d = [0.] * 5
    image = Image()
    image.header.frame_id = OPTICAL
    image.width, image.height, image.step = 3, 3, 6
    image.encoding = '16UC1'
    image.data = np.full((3, 3), 1000, dtype='<u2').tobytes()
    return info, image


def optical_transform():
    """Create a synthetic base-to-optical transform for message tests."""
    transform = TransformStamped()
    transform.header.frame_id = 'base_footprint'
    transform.child_frame_id = OPTICAL
    transform.transform.translation.x = 0.065
    transform.transform.translation.z = 0.215
    q = transform.transform.rotation
    q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
    return transform


class Collector:
    """Capture callback output without emitting observations on DDS."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def observer():
    """Use a dedicated context/domain and never start a command publisher."""
    context = Context()
    context.init(domain_id=198)
    node = DepthObstacleFilter(context=context, parameter_overrides=[
        Parameter('geometry_file', value=str(GEOMETRY)),
        Parameter('pixel_stride', value=1),
        Parameter('transform_timeout_s', value=0.0),
    ])
    node.obstacle_pub = Collector()
    node.ray_pub = Collector()
    node.status_pub = Collector()
    node.tf_buffer.set_transform_static(optical_transform(), 'synthetic_test')
    try:
        yield node
    finally:
        node.destroy_node()
        context.shutdown()


def feed(node, image):
    """Stamp a new synthetic image while bypassing only rate throttling."""
    image.header.stamp = (
        node.get_clock().now() - Duration(seconds=0.01)).to_msg()
    node.last_attempt = -float('inf')
    node.on_image(image)


def test_registered_projection_does_not_use_nan_k():
    info, image = messages()
    assert camera_intrinsics(info, image, OPTICAL,
                             'rectified_projection') == (5., 5., 1., 1.)
    with pytest.raises(ValueError, match='invalid_pinhole_intrinsics'):
        camera_intrinsics(info, image, OPTICAL, 'pinhole_k')


@pytest.mark.parametrize('change', [
    lambda info: setattr(info.header, 'frame_id', 'different_camera'),
    lambda info: setattr(info, 'width', 640),
    lambda info: setattr(info, 'binning_x', 2),
    lambda info: setattr(info.roi, 'x_offset', 1),
    lambda info: setattr(info, 'd', [0.2, 0., 0., 0., 0.]),
    lambda info: info.p.__setitem__(0, float('nan')),
    lambda info: info.p.__setitem__(3, 0.2),
    lambda info: info.r.__setitem__(0, 0.),
])
def test_incompatible_calibration_rejected(change):
    info, image = messages()
    change(info)
    with pytest.raises(ValueError):
        camera_intrinsics(info, image, OPTICAL, 'rectified_projection')


def test_measured_body_geometry():
    assert body_bounds_from_geometry(GEOMETRY) == pytest.approx(
        (-0.275, 0.065, -0.270, 0.270))


def test_transform_and_cloud_wire_format():
    transform = optical_transform().transform
    np.testing.assert_allclose(transform_matrix(transform)[:3, :3],
                               [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    _, image = messages()
    cloud = make_cloud([[1., 2., 3.]], 'base_footprint', image.header.stamp)
    assert cloud.width == 1 and cloud.point_step == cloud.row_step == 12
    assert [field.offset for field in cloud.fields] == [0, 4, 8]
    np.testing.assert_equal(np.frombuffer(cloud.data, dtype='<f4'), [1., 2., 3.])
    transform.rotation.w = 0.0
    with pytest.raises(ValueError, match='quaternion'):
        transform_matrix(transform)


def test_valid_frame_and_stamp_preserved(observer):
    info, image = messages()
    observer.on_camera_info(info)
    feed(observer, image)
    assert observer.status['healthy']
    cloud = observer.obstacle_pub.messages[-1]
    assert cloud.width > 0
    assert cloud.header.frame_id == 'base_footprint'
    assert cloud.header.stamp == image.header.stamp
    xyz = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 3)
    assert np.all(xyz[:, 2] >= 0.05)
    assert observer.ray_pub.messages[-1].width > cloud.width


def test_floor_only_is_healthy_not_sensor_dropout(observer):
    info, image = messages()
    observer.on_camera_info(info)
    depth = np.zeros((3, 3), dtype='<u2')
    depth[2, :] = 1075
    image.data = depth.tobytes()
    feed(observer, image)
    assert observer.status['healthy']
    assert observer.obstacle_pub.messages[-1].width == 0
    assert observer.ray_pub.messages[-1].width == 3


@pytest.mark.parametrize('kind', ['zero', 'nan', 'truncated', 'encoding'])
def test_invalid_depth_never_publishes_empty_heartbeat(observer, kind):
    info, image = messages()
    observer.on_camera_info(info)
    if kind == 'zero':
        image.data = np.zeros((3, 3), dtype='<u2').tobytes()
    elif kind == 'nan':
        image.encoding, image.step = '32FC1', 12
        image.data = np.full((3, 3), np.nan, dtype='<f4').tobytes()
    elif kind == 'truncated':
        image.data = b'\x00'
    else:
        image.encoding = 'rgb8'
    feed(observer, image)
    assert not observer.status['healthy']
    assert not observer.obstacle_pub.messages
    assert not observer.ray_pub.messages


def test_wait_for_info_and_missing_tf(observer):
    info, image = messages()
    feed(observer, image)
    assert observer.status['state'] == 'waiting_for_camera_info'
    observer.on_camera_info(info)
    observer.tf_buffer = Buffer()
    feed(observer, image)
    assert observer.status['state'] == 'missing_transform_at_measurement_time'
    assert not observer.obstacle_pub.messages


@pytest.mark.parametrize('offset', [-2.0, 1.0])
def test_stale_and_future_rejected(observer, offset):
    info, image = messages()
    observer.on_camera_info(info)
    image.header.stamp = (observer.get_clock().now()
                          + Duration(seconds=offset)).to_msg()
    observer.on_image(image)
    assert observer.status['state'] == 'stale_or_future_depth'
    assert not observer.obstacle_pub.messages


def test_no_republishing_old_frame_as_new(observer):
    info, image = messages()
    observer.on_camera_info(info)
    feed(observer, image)
    observer.last_attempt = -float('inf')
    observer.on_image(copy.deepcopy(image))
    assert observer.status['state'] == 'non_increasing_depth_stamp'
    assert len(observer.obstacle_pub.messages) == 1


def test_dropout_status_does_not_refresh_cloud(observer):
    info, image = messages()
    observer.on_camera_info(info)
    feed(observer, image)
    observer.last_received_at = time.monotonic() - 2
    observer.publish_status()
    assert observer.status['state'] == 'depth_stream_timeout'
    assert len(observer.obstacle_pub.messages) == 1


def test_rate_limiter_discards_without_queuing(observer):
    info, image = messages()
    observer.on_camera_info(info)
    feed(observer, image)
    for _ in range(10):
        observer.on_image(image)
    assert len(observer.obstacle_pub.messages) == 1


def test_valid_depth_outside_roi_does_not_publish(observer):
    info, image = messages()
    observer.on_camera_info(info)
    transform = optical_transform()
    transform.transform.translation.x = -10.0
    observer.tf_buffer.set_transform_static(transform, 'wrong_mount_test')
    feed(observer, image)
    assert observer.status['state'] == 'no_roi_observations'
    assert not observer.obstacle_pub.messages
    assert not observer.ray_pub.messages


def test_filter_contract_parameters_cannot_change_mid_run(observer):
    result = observer.set_parameters([
        Parameter('optical_frame', value='different_camera')])[0]
    assert not result.successful
    assert observer.get_parameter('optical_frame').value == OPTICAL
