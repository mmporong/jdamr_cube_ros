"""Pure geometry for a table-edge docking target."""

import math

from .box_docking_target import (
    _finite_fields,
    _finite_number,
    _normalize_angle,
    _rotate_rpy,
    _rotate_xy,
    _validated_mount,
    MAXIMUM_OBSERVATION_AGE_S,
)


PROVENANCE = 'TABLE_EDGE_CAMERA_MOUNT_ESTIMATE'
MINIMUM_EDGE_LENGTH_M = 0.10
MINIMUM_CONFIDENCE = 0.50
MINIMUM_UP_DOT = 0.80
MAXIMUM_EDGE_RMS_M = 0.01
MAXIMUM_EDGE_VERTICAL_DELTA_M = 0.05
MAXIMUM_HEIGHT_DISAGREEMENT_M = 0.05


def _finite_vector(value, length, label):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f'{label} must contain {length} finite values')
    return tuple(
        _finite_number(item, f'{label}[{index}]')
        for index, item in enumerate(value))


def _optical_point_to_base(point, mount):
    optical_x, optical_y, optical_z = point
    mount_x, mount_y, mount_z, roll, pitch, yaw = mount
    base_x, base_y, base_z = _rotate_rpy(
        optical_z, -optical_x, -optical_y, roll, pitch, yaw)
    return base_x + mount_x, base_y + mount_y, base_z + mount_z


def _optical_vector_to_base(vector, mount):
    optical_x, optical_y, optical_z = vector
    _mount_x, _mount_y, _mount_z, roll, pitch, yaw = mount
    return _rotate_rpy(
        optical_z, -optical_x, -optical_y, roll, pitch, yaw)


def _validated_table_observation(document, mount, now_s):
    if (not isinstance(document, dict)
            or document.get('detected') is not True
            or document.get('stable') is not True
            or document.get('surface_kind') != 'table_edge'):
        raise ValueError('stable detected table_edge surface is required')
    stamp_s, confidence, table_height_m, edge_rms_m = _finite_fields(
        document,
        ('stamp_s', 'confidence', 'table_height_m', 'edge_rms_m'),
        'observation')
    now_s = _finite_number(now_s, 'now_s')
    age_s = now_s - stamp_s
    if age_s < 0.0 or age_s > MAXIMUM_OBSERVATION_AGE_S:
        raise ValueError('table-edge observation is stale or future-dated')
    if not MINIMUM_CONFIDENCE <= confidence <= 1.0:
        raise ValueError('table-edge confidence must be in [0.5, 1]')
    if table_height_m <= 0.0:
        raise ValueError('table_height_m must be positive')
    if not 0.0 <= edge_rms_m <= MAXIMUM_EDGE_RMS_M:
        raise ValueError('edge_rms_m exceeds the docking quality bound')

    endpoints = document.get('edge_endpoints_optical_m')
    if not isinstance(endpoints, (list, tuple)) or len(endpoints) != 2:
        raise ValueError('edge_endpoints_optical_m must contain two points')
    endpoints = tuple(
        _finite_vector(point, 3, f'edge_endpoints_optical_m[{index}]')
        for index, point in enumerate(endpoints))
    normal = _finite_vector(document.get('plane_normal'), 3, 'plane_normal')
    normal_magnitude = math.sqrt(sum(value * value for value in normal))
    if not math.isclose(normal_magnitude, 1.0, abs_tol=1e-3):
        raise ValueError('plane_normal must be a unit optical vector')

    endpoints_base = tuple(
        _optical_point_to_base(point, mount) for point in endpoints)
    edge_dx = endpoints_base[1][0] - endpoints_base[0][0]
    edge_dy = endpoints_base[1][1] - endpoints_base[0][1]
    edge_length_m = math.hypot(edge_dx, edge_dy)
    if edge_length_m < MINIMUM_EDGE_LENGTH_M:
        raise ValueError('table edge is too short for docking geometry')
    if abs(endpoints_base[1][2] - endpoints_base[0][2]) > (
            MAXIMUM_EDGE_VERTICAL_DELTA_M):
        raise ValueError('table edge is not level in the base frame')
    observed_height_m = 0.5 * (
        endpoints_base[0][2] + endpoints_base[1][2])
    if abs(observed_height_m - table_height_m) > (
            MAXIMUM_HEIGHT_DISAGREEMENT_M):
        raise ValueError('table height disagrees with transformed edge')
    normal_base = _optical_vector_to_base(normal, mount)
    if normal_base[2] < MINIMUM_UP_DOT:
        raise ValueError('table plane normal must point upward in base frame')
    return endpoints_base, table_height_m, edge_rms_m


def compute_table_edge_docking_target(
        observation, camera_mount, robot_pose, *, requested_gap_m,
        front_extent_m, now_s):
    """Estimate a map-frame base pose facing a visible table front edge."""
    gap_m = _finite_number(requested_gap_m, 'requested_gap_m')
    extent_m = _finite_number(front_extent_m, 'front_extent_m')
    if gap_m <= 0.0:
        raise ValueError('requested_gap_m must be positive')
    if extent_m <= 0.0:
        raise ValueError('front_extent_m must be positive')
    robot_x, robot_y, robot_yaw = _finite_fields(
        robot_pose, ('x_m', 'y_m', 'yaw_rad'), 'robot_pose')
    mount = _validated_mount(camera_mount)
    endpoints_base, table_height_m, edge_rms_m = (
        _validated_table_observation(observation, mount, now_s))

    center_x = 0.5 * (endpoints_base[0][0] + endpoints_base[1][0])
    center_y = 0.5 * (endpoints_base[0][1] + endpoints_base[1][1])
    edge_dx = endpoints_base[1][0] - endpoints_base[0][0]
    edge_dy = endpoints_base[1][1] - endpoints_base[0][1]
    edge_length = math.hypot(edge_dx, edge_dy)
    basis_x = edge_dx / edge_length
    basis_y = edge_dy / edge_length
    normal_x, normal_y = -basis_y, basis_x
    camera_x, camera_y = mount[0], mount[1]
    if ((camera_x - center_x) * normal_x
            + (camera_y - center_y) * normal_y) < 0.0:
        normal_x = -normal_x
        normal_y = -normal_y

    center_offset_m = extent_m + gap_m
    target_base_x = center_x + normal_x * center_offset_m
    target_base_y = center_y + normal_y * center_offset_m
    target_map_dx, target_map_dy = _rotate_xy(
        target_base_x, target_base_y, robot_yaw)
    center_map_dx, center_map_dy = _rotate_xy(
        center_x, center_y, robot_yaw)
    normal_map_x, normal_map_y = _rotate_xy(
        normal_x, normal_y, robot_yaw)
    basis_map_x, basis_map_y = _rotate_xy(
        basis_x, basis_y, robot_yaw)
    estimate_gap_m = math.hypot(
        center_x - target_base_x,
        center_y - target_base_y) - extent_m
    heading_base = math.atan2(-normal_y, -normal_x)
    return {
        'x_m': robot_x + target_map_dx,
        'y_m': robot_y + target_map_dy,
        'yaw_rad': _normalize_angle(robot_yaw + heading_base),
        'estimate_gap_m': estimate_gap_m,
        'provenance': PROVENANCE,
        'face_center_map_xy_m': (
            robot_x + center_map_dx, robot_y + center_map_dy),
        'outward_normal_map_xy': (normal_map_x, normal_map_y),
        'table_height_m': table_height_m,
        'edge_basis_map_xy': (basis_map_x, basis_map_y),
        'edge_rms_m': edge_rms_m,
        'surface_kind': 'table_edge',
    }
