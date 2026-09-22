"""Pure LiDAR witness for one depth-estimated box face at a stopped pose."""

import math

import numpy as np


PROVENANCE = 'LIDAR_DEPTH_FACE_AGREEMENT_NOT_EXTERNAL_ACCURACY'
MAX_TANGENT_HALF_WIDTH_M = 0.20
MAX_CANDIDATE_PLANE_DISTANCE_M = 0.04
MINIMUM_SUPPORT = 5
MINIMUM_TANGENT_SPREAD_M = 0.08
MAXIMUM_RESIDUAL_RMS_M = 0.015
MAXIMUM_NORMAL_YAW_DIFFERENCE_RAD = math.radians(5.0)
MAXIMUM_MEDIAN_NORMAL_OFFSET_M = 0.02


def _finite(value, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise ValueError(f'{label} must be finite')
    return float(value)


def _mapping_values(document, keys, label):
    if not isinstance(document, dict):
        raise ValueError(f'{label} must be a mapping')
    return tuple(_finite(document.get(key), f'{label}.{key}') for key in keys)


def _unit_xy(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f'{label} must contain two components')
    x_m, y_m = (_finite(item, label) for item in value)
    norm = math.hypot(x_m, y_m)
    if not math.isclose(norm, 1.0, abs_tol=1e-3):
        raise ValueError(f'{label} must be a unit vector')
    return np.array((x_m / norm, y_m / norm), dtype=np.float64)


def _laser_mount(geometry):
    if not isinstance(geometry, dict):
        raise ValueError('geometry must be a mapping')
    try:
        translation = geometry['laser_translation']
        yaw = geometry['laser_yaw']
    except KeyError as error:
        raise ValueError('geometry is missing laser mounting data') from error
    if (not isinstance(translation, dict) or translation.get('unit') != 'm'
            or not isinstance(yaw, dict) or yaw.get('unit') != 'rad'):
        raise ValueError('laser mounting data must use meters and radians')
    xyz = translation.get('value')
    if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
        raise ValueError('laser translation must contain XYZ')
    x_m, y_m, z_m = (
        _finite(value, f'laser_translation[{index}]')
        for index, value in enumerate(xyz))
    return x_m, y_m, z_m, _finite(yaw.get('value'), 'laser_yaw')


def _rotate(points, yaw_rad):
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    rotation = np.array(((cosine, -sine), (sine, cosine)))
    return np.asarray(points) @ rotation.T


def _angle_difference(first, second):
    return abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)


def witness_box_face_with_lidar(
        ranges, *, angle_min, angle_increment, range_min, range_max,
        geometry, depth_target, robot_pose):
    """Confirm local LiDAR/depth face agreement without claiming accuracy."""
    angle_min = _finite(angle_min, 'angle_min')
    angle_increment = _finite(angle_increment, 'angle_increment')
    range_min = _finite(range_min, 'range_min')
    range_max = _finite(range_max, 'range_max')
    if angle_increment == 0.0:
        raise ValueError('angle_increment must be nonzero')
    if range_min < 0.0 or range_max <= range_min:
        raise ValueError('laser range bounds are invalid')
    try:
        range_values = list(ranges)
    except TypeError as error:
        raise ValueError('ranges must be iterable') from error
    if not range_values:
        raise ValueError('ranges must not be empty')

    if not isinstance(depth_target, dict):
        raise ValueError('depth_target must be a mapping')
    face = np.array(depth_target.get('face_center_map_xy_m'), dtype=object)
    # Validate compound target fields separately to retain precise failures.
    if face.shape != (2,):
        raise ValueError('face_center_map_xy_m must contain two components')
    face = np.array([
        _finite(face[0], 'face_center_map_xy_m[0]'),
        _finite(face[1], 'face_center_map_xy_m[1]'),
    ])
    depth_normal = _unit_xy(
        depth_target.get('outward_normal_map_xy'),
        'outward_normal_map_xy')
    tangent = np.array((-depth_normal[1], depth_normal[0]))
    robot_x, robot_y, robot_yaw = _mapping_values(
        robot_pose, ('x_m', 'y_m', 'yaw_rad'), 'robot_pose')
    laser_x, laser_y, _laser_z, laser_yaw = _laser_mount(geometry)

    scan_points = []
    for index, raw_range in enumerate(range_values):
        if (isinstance(raw_range, bool)
                or not isinstance(raw_range, (int, float))
                or not math.isfinite(raw_range)
                or raw_range <= range_min or raw_range > range_max):
            continue
        angle = angle_min + index * angle_increment
        scan_points.append((raw_range * math.cos(angle),
                            raw_range * math.sin(angle)))
    if not scan_points:
        raise ValueError('scan has no valid ranges')
    base_points = _rotate(scan_points, laser_yaw)
    base_points += np.array((laser_x, laser_y))
    map_points = _rotate(base_points, robot_yaw)
    map_points += np.array((robot_x, robot_y))

    relative = map_points - face
    normal_offsets = relative @ depth_normal
    tangent_offsets = relative @ tangent
    selected = map_points[
        (np.abs(normal_offsets) <= MAX_CANDIDATE_PLANE_DISTANCE_M)
        & (np.abs(tangent_offsets) <= MAX_TANGENT_HALF_WIDTH_M)]
    if len(selected) < MINIMUM_SUPPORT:
        raise ValueError('LiDAR face support is below five points')

    center = np.mean(selected, axis=0)
    centered = selected - center
    _left, _singular, vectors = np.linalg.svd(centered, full_matrices=False)
    line_direction = vectors[0]
    fitted_normal = np.array((-line_direction[1], line_direction[0]))
    if float(fitted_normal @ depth_normal) < 0.0:
        fitted_normal = -fitted_normal
    fitted_tangent = np.array((-fitted_normal[1], fitted_normal[0]))
    fitted_offsets = centered @ fitted_normal
    residual_rms_m = float(np.sqrt(np.mean(fitted_offsets ** 2)))
    tangent_positions = selected @ fitted_tangent
    tangent_spread_m = float(np.ptp(tangent_positions))
    median_normal_offset_m = float(np.median(
        (selected - face) @ depth_normal))
    fitted_yaw = math.atan2(fitted_normal[1], fitted_normal[0])
    depth_yaw = math.atan2(depth_normal[1], depth_normal[0])
    yaw_difference_rad = _angle_difference(fitted_yaw, depth_yaw)

    if tangent_spread_m < MINIMUM_TANGENT_SPREAD_M:
        raise ValueError('LiDAR face tangent spread is too narrow')
    if residual_rms_m > MAXIMUM_RESIDUAL_RMS_M:
        raise ValueError('LiDAR face line residual is too large')
    if yaw_difference_rad > MAXIMUM_NORMAL_YAW_DIFFERENCE_RAD:
        raise ValueError('LiDAR and depth face normals disagree')
    if abs(median_normal_offset_m) > MAXIMUM_MEDIAN_NORMAL_OFFSET_M:
        raise ValueError('LiDAR and depth face distances disagree')

    face_to_plane_m = float(fitted_normal @ (center - face))
    fused_face = face + fitted_normal * face_to_plane_m
    return {
        'fused_face_center_map_xy_m': tuple(float(value) for value in fused_face),
        'outward_normal_map_xy': tuple(
            float(value) for value in fitted_normal),
        'residual_rms_m': residual_rms_m,
        'distance_difference_m': median_normal_offset_m,
        'normal_yaw_difference_rad': yaw_difference_rad,
        'support_count': int(len(selected)),
        'tangent_spread_m': tangent_spread_m,
        'provenance': PROVENANCE,
    }
