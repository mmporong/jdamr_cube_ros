"""Tests for wheel-odometry-seeded RGB-D point-cloud fusion."""

import math

from jdamr_cube_vslam.fuse_rgbd_odom import (
    CameraExtrinsic,
    evaluate_yaw_coverage,
    ImageFrame,
    interpolate_pose,
    OdomPose,
    pair_rgbd_frames,
    select_frames_by_yaw,
    transform_optical_points,
    unwrap_odom_yaw,
    voxel_average,
)
import numpy as np
import pytest


def test_unwrap_and_interpolate_yaw_across_pi_boundary():
    """Yaw remains continuous while interpolating across plus/minus pi."""
    poses = unwrap_odom_yaw([
        OdomPose(0, 0.0, 0.0, math.radians(170.0)),
        OdomPose(10, 1.0, 0.0, math.radians(-170.0)),
    ])
    middle = interpolate_pose(poses, 5)
    assert math.degrees(middle.yaw_rad) == pytest.approx(180.0)
    assert middle.x_m == pytest.approx(0.5)


def test_pairing_rejects_processing_bag_sync_violation():
    """Pairs outside the configured synchronization window are rejected."""
    color = [ImageFrame(0, 0)]
    depth = [ImageFrame(0, 31_000_000)]
    with pytest.raises(ValueError, match='sync limit'):
        pair_rgbd_frames(color, depth, max_pair_delta_ms=30.0)


def test_selects_regular_clockwise_yaw_increments():
    """Clockwise motion selects frames at regular negative-yaw intervals."""
    pairs = [
        (ImageFrame(index, index * 10), ImageFrame(index, index * 10 + 1))
        for index in range(10)
    ]
    poses = [
        OdomPose(index * 10, 0.0, 0.0, math.radians(-10.0 * index))
        for index in range(10)
    ]
    selected = select_frames_by_yaw(pairs, poses, yaw_step_deg=30.0)
    assert [selection.color.ordinal for selection in selected] == [0, 3, 6, 9]
    assert selected[-1].target_yaw_deg == pytest.approx(-90.0)


def test_optical_axes_and_base_rotation_are_applied():
    """Optical coordinates are converted before applying the base pose."""
    optical = np.array([[0.0, 0.0, 1.0]])
    initial = OdomPose(0, 0.0, 0.0, 0.0)
    turned = OdomPose(1, 0.0, 0.0, math.pi / 2.0)
    transformed = transform_optical_points(
        optical, turned, initial, CameraExtrinsic())
    np.testing.assert_allclose(transformed, [[0.0, 1.0, 0.0]], atol=1e-9)


def test_camera_translation_and_voxel_average():
    """Extrinsic translation and per-voxel means preserve metric values."""
    optical = np.array([[0.0, 0.0, 1.0]])
    pose = OdomPose(0, 0.0, 0.0, 0.0)
    transformed = transform_optical_points(
        optical, pose, pose, CameraExtrinsic(x_m=0.1, z_m=0.2))
    np.testing.assert_allclose(transformed, [[1.1, 0.0, 0.2]])

    points = np.array([[0.001, 0.0, 0.0], [0.009, 0.0, 0.0]])
    colors = np.array([[0, 10, 20], [10, 20, 30]], dtype=np.uint8)
    fused_points, fused_colors = voxel_average(points, colors, voxel_m=0.01)
    np.testing.assert_allclose(fused_points, [[0.005, 0.0, 0.0]])
    np.testing.assert_array_equal(fused_colors, [[5, 15, 25]])


def test_expected_yaw_marks_partial_capture():
    """A recording below 90 percent of the requested turn is partial."""
    result = evaluate_yaw_coverage(-49.0, expected_yaw_deg=90.0)
    assert result['capture_complete'] is False
    assert result['result_status'] == 'partial_capture'
    assert result['yaw_coverage_percent'] == pytest.approx(54.444444)
