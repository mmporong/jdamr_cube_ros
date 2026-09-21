"""Detect a nearby box top in a metric depth image."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics for a depth image."""

    width: int
    height: int
    focal_x_px: float
    focal_y_px: float
    center_x_px: float
    center_y_px: float


@dataclass(frozen=True)
class BoxTopConfig:
    """Geometry and quality bounds for a box-top candidate."""

    minimum_depth_m: float = 0.35
    maximum_depth_m: float = 2.0
    desired_standoff_m: float = 0.45
    pixel_step: int = 2
    plane_distance_m: float = 0.025
    minimum_normal_y: float = 0.65
    minimum_front_normal_z: float = 0.80
    maximum_top_y_m: float = 0.05
    minimum_width_m: float = 0.18
    maximum_width_m: float = 1.20
    minimum_height_m: float = 0.08
    maximum_height_m: float = 0.80
    minimum_depth_extent_m: float = 0.08
    minimum_inliers: int = 120
    ransac_iterations: int = 80
    maximum_points: int = 6000
    random_seed: int = 17


@dataclass(frozen=True)
class BoxTopDetection:
    """Metric box-top observation in the camera optical frame."""

    center_x_m: float
    center_y_m: float
    front_distance_m: float
    width_m: float
    depth_extent_m: float
    edge_angle_rad: float
    plane_normal: tuple[float, float, float]
    plane_rms_m: float
    inlier_count: int
    candidate_count: int
    confidence: float
    surface_kind: str = 'top'
    height_m: float = 0.0


def _validate(intrinsics: CameraIntrinsics, config: BoxTopConfig) -> None:
    if intrinsics.width <= 0 or intrinsics.height <= 0:
        raise ValueError('image dimensions must be positive')
    if intrinsics.focal_x_px <= 0.0 or intrinsics.focal_y_px <= 0.0:
        raise ValueError('focal lengths must be positive')
    if config.pixel_step <= 0:
        raise ValueError('pixel_step must be positive')
    if not 0.0 < config.minimum_depth_m < config.maximum_depth_m:
        raise ValueError('depth bounds are invalid')
    if config.plane_distance_m <= 0.0:
        raise ValueError('plane_distance_m must be positive')
    if not 0.0 <= config.minimum_normal_y <= 1.0:
        raise ValueError('minimum_normal_y must be in [0, 1]')
    if not 0.0 <= config.minimum_front_normal_z <= 1.0:
        raise ValueError('minimum_front_normal_z must be in [0, 1]')
    if not 0.0 < config.minimum_width_m < config.maximum_width_m:
        raise ValueError('box width bounds are invalid')
    if not 0.0 < config.minimum_height_m < config.maximum_height_m:
        raise ValueError('box height bounds are invalid')
    if config.minimum_inliers < 3 or config.ransac_iterations <= 0:
        raise ValueError('RANSAC bounds are invalid')


def depth_points(
        depth_mm: np.ndarray, intrinsics: CameraIntrinsics,
        config: BoxTopConfig) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Back-project the configured valid depth range into optical coordinates."""
    _validate(intrinsics, config)
    if depth_mm.shape != (intrinsics.height, intrinsics.width):
        raise ValueError('depth image dimensions do not match camera info')
    rows, columns = np.mgrid[
        0:intrinsics.height:config.pixel_step,
        0:intrinsics.width:config.pixel_step,
    ]
    depths_m = depth_mm[::config.pixel_step, ::config.pixel_step].astype(
        np.float64) * 0.001
    valid = np.isfinite(depths_m)
    valid &= depths_m >= config.minimum_depth_m
    valid &= depths_m <= config.maximum_depth_m
    z_m = depths_m[valid]
    x_m = ((columns[valid] - intrinsics.center_x_px) * z_m
           / intrinsics.focal_x_px)
    y_m = ((rows[valid] - intrinsics.center_y_px) * z_m
           / intrinsics.focal_y_px)
    points = np.column_stack((x_m, y_m, z_m))
    return points, columns[valid], rows[valid]


def _plane_from_sample(sample: np.ndarray):
    first, second, third = sample
    normal = np.cross(second - first, third - first)
    norm = float(np.linalg.norm(normal))
    if norm < 1e-9:
        return None
    normal /= norm
    offset = -float(np.dot(normal, first))
    return normal, offset


def _fit_plane(points: np.ndarray) -> tuple[np.ndarray, float]:
    center = np.mean(points, axis=0)
    _, _, right_vectors = np.linalg.svd(points - center, full_matrices=False)
    normal = right_vectors[-1]
    normal /= np.linalg.norm(normal)
    return normal, -float(np.dot(normal, center))


def _front_edge(points: np.ndarray) -> float:
    """Estimate the box front-edge angle in the optical X-Z plane."""
    x_values = points[:, 0]
    if np.ptp(x_values) < 0.05:
        return 0.0
    boundaries = np.linspace(
        np.percentile(x_values, 5.0), np.percentile(x_values, 95.0), 9)
    edge_x = []
    edge_z = []
    for lower, upper in zip(boundaries[:-1], boundaries[1:]):
        selected = points[(x_values >= lower) & (x_values < upper)]
        if len(selected) < 5:
            continue
        edge_x.append(float(np.median(selected[:, 0])))
        edge_z.append(float(np.percentile(selected[:, 2], 10.0)))
    if len(edge_x) < 5:
        return 0.0
    slope, _ = np.polyfit(edge_x, edge_z, 1)
    return math.atan(float(slope))


def _plane_shape(points: np.ndarray) -> tuple[float, float, float]:
    bounds = np.percentile(points, [5.0, 95.0], axis=0)
    extents = bounds[1] - bounds[0]
    return tuple(float(value) for value in extents)


def _normal_is_valid(
        normal: np.ndarray, config: BoxTopConfig,
        surface_kind: str) -> bool:
    if surface_kind == 'front':
        return abs(normal[2]) >= config.minimum_front_normal_z
    return abs(normal[1]) >= config.minimum_normal_y


def _shape_is_valid(
        points: np.ndarray, normal: np.ndarray,
        config: BoxTopConfig, surface_kind: str) -> bool:
    if not _normal_is_valid(normal, config, surface_kind):
        return False
    width_m, height_m, depth_extent_m = _plane_shape(points)
    if not config.minimum_width_m <= width_m <= config.maximum_width_m:
        return False
    if surface_kind == 'front':
        return (
            config.minimum_height_m
            <= height_m <= config.maximum_height_m
        )
    center = np.median(points, axis=0)
    return (
        center[1] <= config.maximum_top_y_m
        and depth_extent_m >= config.minimum_depth_extent_m
    )


def _detect_box_plane(
        depth_mm: np.ndarray, intrinsics: CameraIntrinsics,
        config: BoxTopConfig, surface_kind: str) -> BoxTopDetection | None:
    if surface_kind not in {'front', 'top'}:
        raise ValueError('surface_kind must be front or top')
    points, _, _ = depth_points(depth_mm, intrinsics, config)
    if len(points) < config.minimum_inliers:
        return None
    if len(points) > config.maximum_points:
        stride = math.ceil(len(points) / config.maximum_points)
        points = points[::stride]

    random = np.random.default_rng(config.random_seed)
    best_mask = None
    best_score = -math.inf
    for _ in range(config.ransac_iterations):
        indices = random.choice(len(points), size=3, replace=False)
        plane = _plane_from_sample(points[indices])
        if plane is None:
            continue
        normal, offset = plane
        if not _normal_is_valid(normal, config, surface_kind):
            continue
        distances = np.abs(points @ normal + offset)
        mask = distances <= config.plane_distance_m
        count = int(np.count_nonzero(mask))
        if count < config.minimum_inliers:
            continue
        residual = float(np.sqrt(np.mean(distances[mask] ** 2)))
        score = count / max(residual, 0.001)
        if score <= best_score:
            continue
        candidate = points[mask]
        if not _shape_is_valid(candidate, normal, config, surface_kind):
            continue
        best_mask = mask
        best_score = score
    if best_mask is None:
        return None

    candidate = points[best_mask]
    normal, offset = _fit_plane(candidate)
    if surface_kind == 'front' and normal[2] > 0.0:
        normal = -normal
        offset = -offset
    elif surface_kind == 'top' and normal[1] < 0.0:
        normal = -normal
        offset = -offset
    distances = np.abs(points @ normal + offset)
    refined_mask = distances <= config.plane_distance_m
    refined = points[refined_mask]
    if len(refined) < config.minimum_inliers:
        return None
    if not _shape_is_valid(refined, normal, config, surface_kind):
        return None
    center = np.median(refined, axis=0)
    width_m, height_m, depth_extent_m = _plane_shape(refined)
    plane_rms_m = float(np.sqrt(np.mean(distances[refined_mask] ** 2)))
    if surface_kind == 'front':
        front_distance_m = float(np.median(refined[:, 2]))
        edge_angle_rad = math.atan2(float(normal[0]), float(-normal[2]))
    else:
        front_distance_m = float(np.percentile(refined[:, 2], 10.0))
        edge_angle_rad = _front_edge(refined)
    inlier_ratio = len(refined) / len(points)
    residual_quality = max(
        0.0, 1.0 - plane_rms_m / config.plane_distance_m)
    support_quality = min(1.0, inlier_ratio / 0.20)
    confidence = 0.55 * residual_quality + 0.45 * support_quality
    return BoxTopDetection(
        center_x_m=float(center[0]),
        center_y_m=float(center[1]),
        front_distance_m=front_distance_m,
        width_m=width_m,
        depth_extent_m=depth_extent_m,
        edge_angle_rad=edge_angle_rad,
        plane_normal=tuple(float(value) for value in normal),
        plane_rms_m=plane_rms_m,
        inlier_count=int(len(refined)),
        candidate_count=int(len(points)),
        confidence=float(confidence),
        surface_kind=surface_kind,
        height_m=height_m,
    )


def detect_box_top(
        depth_mm: np.ndarray, intrinsics: CameraIntrinsics,
        config: BoxTopConfig = BoxTopConfig()) -> BoxTopDetection | None:
    """Return the strongest nearby horizontal top-plane candidate."""
    return _detect_box_plane(depth_mm, intrinsics, config, 'top')


def detect_box_front(
        depth_mm: np.ndarray, intrinsics: CameraIntrinsics,
        config: BoxTopConfig = BoxTopConfig()) -> BoxTopDetection | None:
    """Return the strongest bounded vertical front-plane candidate."""
    return _detect_box_plane(depth_mm, intrinsics, config, 'front')


class DetectionStability:
    """Require consecutive, mutually consistent box observations."""

    def __init__(self, required_frames: int = 8,
                 maximum_distance_spread_m: float = 0.03,
                 maximum_lateral_spread_m: float = 0.03,
                 maximum_angle_spread_deg: float = 3.0):
        if required_frames <= 0:
            raise ValueError('required_frames must be positive')
        self.required_frames = required_frames
        self.maximum_distance_spread_m = maximum_distance_spread_m
        self.maximum_lateral_spread_m = maximum_lateral_spread_m
        self.maximum_angle_spread_rad = math.radians(maximum_angle_spread_deg)
        self._history: list[BoxTopDetection] = []

    def update(self, detection: BoxTopDetection | None) -> bool:
        """Return whether the latest consecutive observation window is stable."""
        if detection is None:
            self._history.clear()
            return False
        self._history.append(detection)
        self._history = self._history[-self.required_frames:]
        if len(self._history) < self.required_frames:
            return False
        distances = [item.front_distance_m for item in self._history]
        laterals = [item.center_x_m for item in self._history]
        angles = [item.edge_angle_rad for item in self._history]
        stable = (
            max(distances) - min(distances)
            <= self.maximum_distance_spread_m
            and max(laterals) - min(laterals)
            <= self.maximum_lateral_spread_m
            and max(angles) - min(angles)
            <= self.maximum_angle_spread_rad
        )
        if not stable:
            self._history = [detection]
        return stable

    @property
    def frame_count(self) -> int:
        """Return the current consecutive observation count."""
        return len(self._history)
