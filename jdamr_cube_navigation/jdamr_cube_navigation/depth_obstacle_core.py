"""Project metric depth images into base-frame obstacle point clouds."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DepthObstacleConfig:
    """Depth, spatial, sampling, and frame-quality bounds."""

    min_depth_m: float = 0.4
    max_depth_m: float = 2.5
    min_height_m: float = 0.05
    max_height_m: float = 1.5
    max_forward_m: float = 2.5
    max_lateral_m: float = 1.5
    pixel_stride: int = 4
    voxel_size_m: float = 0.04
    min_valid_fraction: float = 0.05
    max_points: int = 20000


@dataclass(frozen=True)
class DepthObstacleResult:
    """
    Projected outputs and pre-voxel frame-quality counters.

    ``roi_point_count`` and ``obstacle_point_count`` count observations before
    voxel filtering; output array lengths give the corresponding voxel counts.
    """

    rays_xyz: np.ndarray
    obstacles_xyz: np.ndarray
    status: str
    healthy: bool
    sampled_pixel_count: int
    valid_depth_count: int
    roi_point_count: int
    obstacle_point_count: int
    valid_fraction: float


def _empty_points() -> np.ndarray:
    return np.empty((0, 3), dtype=np.float32)


def _result(
        status: str, healthy: bool, sampled_count: int, valid_count: int,
        valid_fraction: float, roi_count: int = 0,
        obstacle_count: int = 0, rays: np.ndarray | None = None,
        obstacles: np.ndarray | None = None) -> DepthObstacleResult:
    return DepthObstacleResult(
        rays_xyz=_empty_points() if rays is None else rays,
        obstacles_xyz=_empty_points() if obstacles is None else obstacles,
        status=status,
        healthy=healthy,
        sampled_pixel_count=sampled_count,
        valid_depth_count=valid_count,
        roi_point_count=roi_count,
        obstacle_point_count=obstacle_count,
        valid_fraction=valid_fraction,
    )


def _validate_config(config: DepthObstacleConfig) -> None:
    numeric = (
        config.min_depth_m, config.max_depth_m,
        config.min_height_m, config.max_height_m,
        config.max_forward_m, config.max_lateral_m,
        config.voxel_size_m, config.min_valid_fraction,
    )
    if not np.all(np.isfinite(numeric)):
        raise ValueError('config values must be finite')
    if not 0.0 <= config.min_depth_m < config.max_depth_m:
        raise ValueError('depth bounds are invalid')
    if config.min_height_m > config.max_height_m:
        raise ValueError('height bounds are invalid')
    if config.max_forward_m <= 0.0 or config.max_lateral_m <= 0.0:
        raise ValueError('spatial bounds must be positive')
    if config.voxel_size_m <= 0.0:
        raise ValueError('voxel_size_m must be positive')
    if not 0.0 <= config.min_valid_fraction <= 1.0:
        raise ValueError('min_valid_fraction must be in [0, 1]')
    if (not isinstance(config.pixel_stride, int)
            or isinstance(config.pixel_stride, bool)
            or config.pixel_stride <= 0):
        raise ValueError('pixel_stride must be a positive integer')
    if (not isinstance(config.max_points, int)
            or isinstance(config.max_points, bool)
            or config.max_points <= 0):
        raise ValueError('max_points must be a positive integer')


def _validate_geometry(
        intrinsics, transform4x4, body_bounds
) -> tuple[tuple[float, float, float, float], np.ndarray,
           tuple[float, float, float, float]]:
    try:
        camera = np.asarray(intrinsics, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError('intrinsics must be numeric') from error
    if camera.shape != (4,) or not np.all(np.isfinite(camera)):
        raise ValueError('intrinsics must contain four finite values')
    fx, fy, cx, cy = camera.tolist()
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('focal lengths must be positive')

    try:
        transform = np.asarray(transform4x4, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError('transform must be numeric') from error
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError('transform must be a finite 4x4 matrix')
    rotation = transform[:3, :3]
    if not np.allclose(
            rotation.T @ rotation, np.eye(3), rtol=0.0, atol=1e-5):
        raise ValueError('transform rotation must be orthonormal')
    if not np.isclose(
            np.linalg.det(rotation), 1.0, rtol=0.0, atol=1e-5):
        raise ValueError('transform rotation determinant must be +1')
    if not np.allclose(
            transform[3], (0.0, 0.0, 0.0, 1.0),
            rtol=0.0, atol=1e-8):
        raise ValueError('transform must be homogeneous')

    try:
        bounds_array = np.asarray(body_bounds, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError('body_bounds must be numeric') from error
    if bounds_array.shape != (4,) or not np.all(np.isfinite(bounds_array)):
        raise ValueError('body_bounds must contain four finite values')
    xmin, xmax, ymin, ymax = bounds_array.tolist()
    if xmin >= xmax or ymin >= ymax:
        raise ValueError('body_bounds are invalid')
    return (fx, fy, cx, cy), transform, (xmin, xmax, ymin, ymax)


def _depth_view(
        data, width: int, height: int, step: int,
        encoding: str, is_bigendian: bool) -> np.ndarray:
    if (not isinstance(width, int) or isinstance(width, bool)
            or not isinstance(height, int) or isinstance(height, bool)
            or width <= 0 or height <= 0):
        raise ValueError('image dimensions must be positive integers')
    if not isinstance(step, int) or isinstance(step, bool) or step <= 0:
        raise ValueError('step must be a positive integer')
    normalized_encoding = str(encoding).upper()
    if normalized_encoding == '16UC1':
        scalar_type = 'u2'
    elif normalized_encoding == '32FC1':
        scalar_type = 'f4'
    else:
        raise ValueError('encoding must be 16UC1 or 32FC1')
    dtype = np.dtype(('>' if bool(is_bigendian) else '<') + scalar_type)
    packed_row_size = width * dtype.itemsize
    if step < packed_row_size:
        raise ValueError('step is smaller than a packed image row')
    try:
        buffer = memoryview(data)
    except TypeError as error:
        raise ValueError('data must expose a byte buffer') from error
    if not buffer.contiguous:
        raise ValueError('data buffer must be contiguous')
    if buffer.nbytes < step * height:
        raise ValueError('data buffer is smaller than image dimensions')
    try:
        return np.ndarray(
            shape=(height, width), dtype=dtype, buffer=buffer,
            strides=(step, dtype.itemsize),
        )
    except (TypeError, ValueError) as error:
        raise ValueError(
            'data buffer cannot represent the depth image') from error


def _voxel_representatives(points: np.ndarray, size_m: float) -> np.ndarray:
    """Keep the first observed point in each base-frame fixed-grid voxel."""
    if len(points) == 0:
        return _empty_points()
    voxel_indices = np.floor(points / size_m).astype(np.int64)
    _, first_indices = np.unique(
        voxel_indices, axis=0, return_index=True)
    first_indices.sort()
    return np.ascontiguousarray(points[first_indices], dtype=np.float32)


def project_depth(
        data, width: int, height: int, step: int, encoding: str,
        is_bigendian: bool, intrinsics, transform4x4, body_bounds,
        config: DepthObstacleConfig = DepthObstacleConfig(),
) -> DepthObstacleResult:
    """
    Project a depth buffer to base-frame ray and obstacle point clouds.

    The camera frame follows the ROS optical convention: X right, Y down,
    and Z forward. ``transform4x4`` maps those points into the base frame.
    """
    if not isinstance(config, DepthObstacleConfig):
        raise ValueError('config must be a DepthObstacleConfig')
    _validate_config(config)
    camera, transform, bounds = _validate_geometry(
        intrinsics, transform4x4, body_bounds)
    depth_image = _depth_view(
        data, width, height, step, encoding, is_bigendian)

    rows = np.arange(0, height, config.pixel_stride, dtype=np.float64)
    columns = np.arange(0, width, config.pixel_stride, dtype=np.float64)
    sampled = depth_image[::config.pixel_stride, ::config.pixel_stride]
    depths_m = sampled.astype(np.float64)
    if str(encoding).upper() == '16UC1':
        depths_m *= 0.001
    sampled_count = int(depths_m.size)
    valid_mask = np.isfinite(depths_m)
    valid_mask &= depths_m > 0.0
    valid_mask &= depths_m >= config.min_depth_m
    valid_mask &= depths_m <= config.max_depth_m
    valid_count = int(np.count_nonzero(valid_mask))
    valid_fraction = valid_count / sampled_count
    if valid_count == 0:
        return _result(
            'no_valid_depth', False, sampled_count, 0, valid_fraction)
    if valid_fraction < config.min_valid_fraction:
        return _result(
            'insufficient_valid_fraction', False,
            sampled_count, valid_count, valid_fraction)

    column_grid, row_grid = np.meshgrid(columns, rows)
    z_camera = depths_m[valid_mask]
    fx, fy, cx, cy = camera
    points_camera = np.column_stack((
        (column_grid[valid_mask] - cx) * z_camera / fx,
        (row_grid[valid_mask] - cy) * z_camera / fy,
        z_camera,
    ))
    points_base = points_camera @ transform[:3, :3].T + transform[:3, 3]

    spatial_mask = points_base[:, 0] >= 0.0
    spatial_mask &= points_base[:, 0] <= config.max_forward_m
    spatial_mask &= np.abs(points_base[:, 1]) <= config.max_lateral_m
    xmin, xmax, ymin, ymax = bounds
    inside_body = points_base[:, 0] >= xmin
    inside_body &= points_base[:, 0] <= xmax
    inside_body &= points_base[:, 1] >= ymin
    inside_body &= points_base[:, 1] <= ymax
    spatial_mask &= ~inside_body
    roi_points = points_base[spatial_mask]
    roi_count = int(len(roi_points))
    if roi_count == 0:
        return _result(
            'no_roi_observations', False, sampled_count, valid_count,
            valid_fraction)

    height_mask = roi_points[:, 2] >= config.min_height_m
    height_mask &= roi_points[:, 2] <= config.max_height_m
    obstacle_points = roi_points[height_mask]
    obstacle_count = int(len(obstacle_points))
    rays = _voxel_representatives(roi_points, config.voxel_size_m)
    obstacles = _voxel_representatives(
        obstacle_points, config.voxel_size_m)
    if len(rays) > config.max_points or len(obstacles) > config.max_points:
        return _result(
            'point_overflow', False, sampled_count, valid_count,
            valid_fraction, roi_count, obstacle_count)
    return _result(
        'healthy', True, sampled_count, valid_count, valid_fraction,
        roi_count, obstacle_count, rays, obstacles)
