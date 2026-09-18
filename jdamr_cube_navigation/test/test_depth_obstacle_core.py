"""Tests for ROS-independent depth obstacle projection."""

from dataclasses import replace

from jdamr_cube_navigation.depth_obstacle_core import (
    DepthObstacleConfig,
    project_depth,
)

import numpy as np

import pytest


INTRINSICS = (1.0, 1.0, 0.0, 0.0)
IDENTITY = np.eye(4)
NO_BODY = (10.0, 11.0, 10.0, 11.0)
CONFIG = DepthObstacleConfig(
    min_depth_m=0.4,
    max_depth_m=3.0,
    min_height_m=0.05,
    max_height_m=1.5,
    max_forward_m=4.0,
    max_lateral_m=4.0,
    pixel_stride=1,
    voxel_size_m=0.01,
    min_valid_fraction=0.05,
    max_points=20000,
)


def _project(depth, *, config=CONFIG, intrinsics=INTRINSICS,
             transform=IDENTITY, body_bounds=NO_BODY,
             encoding='32FC1', is_bigendian=False,
             step=None, data=None):
    dtype = ('>f4' if is_bigendian else '<f4')
    image = np.asarray(depth, dtype=dtype)
    height, width = image.shape
    if data is None:
        data = image.tobytes()
    if step is None:
        step = width * image.dtype.itemsize
    return project_depth(
        data=data,
        width=width,
        height=height,
        step=step,
        encoding=encoding,
        is_bigendian=is_bigendian,
        intrinsics=intrinsics,
        transform4x4=transform,
        body_bounds=body_bounds,
        config=config,
    )


def test_projects_distance_and_keeps_floor_in_ray_cloud():
    """Ray output keeps floor while obstacle output applies height bounds."""
    depth = np.array([[1.0, 2.0]], dtype=np.float32)
    result = _project(depth)
    assert result.healthy
    np.testing.assert_allclose(
        result.rays_xyz,
        [[0.0, 0.0, 1.0], [2.0, 0.0, 2.0]],
    )
    np.testing.assert_allclose(result.obstacles_xyz, [[0.0, 0.0, 1.0]])


def test_applies_rotation_and_translation_before_spatial_filtering():
    """Projection uses the full optical-to-base rigid transform."""
    transform = np.array([
        [0.0, 0.0, 1.0, 0.5],
        [-1.0, 0.0, 0.0, 0.25],
        [0.0, -1.0, 0.0, 0.1],
        [0.0, 0.0, 0.0, 1.0],
    ])
    result = _project(
        [[1.0, 1.0]], transform=transform,
        config=replace(CONFIG, min_height_m=-1.0),
    )
    np.testing.assert_allclose(
        result.rays_xyz,
        [[1.5, 0.25, 0.1], [1.5, -0.75, 0.1]],
    )


def test_removes_points_inside_measured_body_xy_bounds():
    """The supplied measured body rectangle masks self observations."""
    result = _project(
        [[1.0, 1.0]],
        body_bounds=(-0.1, 0.1, -0.1, 0.1),
    )
    np.testing.assert_allclose(result.rays_xyz, [[1.0, 0.0, 1.0]])
    assert result.roi_point_count == 1


def test_floor_only_frame_is_healthy_with_empty_obstacles():
    """A valid floor-only frame remains usable for clearing rays."""
    transform = np.eye(4)
    transform[2, 3] = -1.0
    result = _project([[1.0, 1.0]], transform=transform)
    assert result.status == 'healthy'
    assert result.healthy
    assert result.rays_xyz.shape == (2, 3)
    assert result.obstacles_xyz.shape == (0, 3)


@pytest.mark.parametrize('case', ['behind', 'lateral', 'body'])
def test_valid_depth_outside_observed_roi_is_not_free_space(case):
    """A rigid but wrong transform cannot refresh an empty clear cloud."""
    transform = np.eye(4)
    body = NO_BODY
    if case == 'behind':
        transform[0, 3] = -10.0
    elif case == 'lateral':
        transform[1, 3] = 10.0
    else:
        body = (-1.0, 2.0, -1.0, 1.0)
    result = _project([[1.0, 1.0]], transform=transform, body_bounds=body)
    assert result.valid_fraction == 1.0
    assert result.status == 'no_roi_observations'
    assert not result.healthy
    assert len(result.rays_xyz) == len(result.obstacles_xyz) == 0


def test_reads_16uc1_big_endian_with_padded_rows():
    """Millimeter input honors both byte order and ROS row padding."""
    rows = []
    for values in ((1000, 2000), (1500, 0)):
        rows.append(np.asarray(values, dtype='>u2').tobytes() + b'pad!')
    result = project_depth(
        data=b''.join(rows), width=2, height=2, step=8,
        encoding='16UC1', is_bigendian=True,
        intrinsics=INTRINSICS, transform4x4=IDENTITY,
        body_bounds=NO_BODY, config=CONFIG,
    )
    assert result.valid_depth_count == 3
    np.testing.assert_allclose(
        result.rays_xyz,
        [[0.0, 0.0, 1.0], [2.0, 0.0, 2.0], [0.0, 1.5, 1.5]],
    )


def test_rejects_nan_inf_zero_and_out_of_range_depth():
    """Only finite, positive, configured-range depths are projected."""
    result = _project(
        [[np.nan, np.inf, 0.0, 0.3, 3.1, 1.0]],
        config=replace(CONFIG, max_forward_m=6.0),
    )
    assert result.valid_depth_count == 1
    assert result.sampled_pixel_count == 6
    np.testing.assert_allclose(result.rays_xyz, [[5.0, 0.0, 1.0]])


def test_no_depth_and_sparse_depth_are_unhealthy():
    """Missing and insufficient observations report unhealthy statuses."""
    no_depth = _project([[0.0, np.nan]])
    assert no_depth.status == 'no_valid_depth'
    assert not no_depth.healthy
    sparse = _project(
        [[1.0] + [0.0] * 9],
        config=replace(CONFIG, min_valid_fraction=0.2),
    )
    assert sparse.status == 'insufficient_valid_fraction'
    assert not sparse.healthy
    assert sparse.rays_xyz.shape == (0, 3)


@pytest.mark.parametrize('transform', [
    np.diag([2.0, 1.0, 1.0, 1.0]),
    np.diag([-1.0, 1.0, 1.0, 1.0]),
    np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, np.nan],
    ]),
])
def test_rejects_invalid_transform(transform):
    """Non-rigid, reflected, and non-finite transforms are rejected."""
    with pytest.raises(ValueError, match='transform'):
        _project([[1.0]], transform=transform)


def test_voxel_outputs_use_observed_points_not_synthetic_centroids():
    """Both voxel clouds retain a real first observation."""
    result = _project(
        [[1.0, 1.01]],
        intrinsics=(1000.0, 1.0, 0.0, 0.0),
        config=replace(CONFIG, voxel_size_m=0.1),
    )
    assert result.rays_xyz.shape == (1, 3)
    assert result.obstacles_xyz.shape == (1, 3)
    np.testing.assert_array_equal(result.rays_xyz[0], [0.0, 0.0, 1.0])
    np.testing.assert_array_equal(
        result.obstacles_xyz[0], [0.0, 0.0, 1.0])


def test_floor_ray_endpoint_remains_an_observed_point_after_voxelization():
    """A floor voxel endpoint retains exact source-point identity."""
    transform = np.eye(4)
    transform[2, 3] = -1.0
    result = _project(
        [[1.0, 1.01]], transform=transform,
        intrinsics=(1000.0, 1.0, 0.0, 0.0),
        config=replace(CONFIG, voxel_size_m=0.1),
    )
    np.testing.assert_array_equal(result.rays_xyz, [[0.0, 0.0, 0.0]])
    assert result.obstacles_xyz.shape == (0, 3)


def test_point_overflow_is_unhealthy_instead_of_truncated():
    """Overflow prevents a misleading partial cloud from being returned."""
    result = _project(
        [[1.0, 1.0]],
        config=replace(CONFIG, voxel_size_m=0.01, max_points=1),
    )
    assert result.status == 'point_overflow'
    assert not result.healthy
    assert result.roi_point_count == 2
    assert result.rays_xyz.shape == (0, 3)


@pytest.mark.parametrize(
    ('kwargs', 'message'),
    [
        ({'encoding': 'mono16'}, 'encoding'),
        ({'step': 3}, 'step'),
        ({'data': b'\x00' * 4}, 'buffer'),
        ({'intrinsics': (0.0, 1.0, 0.0, 0.0)}, 'focal'),
    ],
)
def test_rejects_invalid_image_or_intrinsics(kwargs, message):
    """Malformed buffers and camera contracts fail before projection."""
    with pytest.raises(ValueError, match=message):
        _project([[1.0, 1.0]], **kwargs)
