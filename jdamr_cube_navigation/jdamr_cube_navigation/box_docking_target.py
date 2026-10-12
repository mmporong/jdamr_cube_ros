"""Pure geometry for an estimated box-face docking target."""

import math


PROVENANCE = 'NOMINAL_CAMERA_MOUNT_ESTIMATE'
MINIMUM_DISTANCE_M = 0.35
MAXIMUM_DISTANCE_M = 2.0
MAXIMUM_OBSERVATION_AGE_S = 0.5
MINIMUM_NORMAL_POINT_DOT = 0.8


def _finite_number(value, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise ValueError(f'{label} must be finite')
    return float(value)


def _finite_fields(document, keys, label):
    if not isinstance(document, dict):
        raise ValueError(f'{label} must be a mapping')
    return tuple(
        _finite_number(document.get(key), f'{label}.{key}') for key in keys)


def _normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _rotate_xy(x_m, y_m, yaw_rad):
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return cosine * x_m - sine * y_m, sine * x_m + cosine * y_m


def _rotate_rpy(x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad):
    """Apply the URDF fixed-axis Rz(yaw) Ry(pitch) Rx(roll) rotation."""
    roll_cosine = math.cos(roll_rad)
    roll_sine = math.sin(roll_rad)
    pitch_cosine = math.cos(pitch_rad)
    pitch_sine = math.sin(pitch_rad)
    yaw_cosine = math.cos(yaw_rad)
    yaw_sine = math.sin(yaw_rad)

    roll_y_m = roll_cosine * y_m - roll_sine * z_m
    roll_z_m = roll_sine * y_m + roll_cosine * z_m
    pitch_x_m = pitch_cosine * x_m + pitch_sine * roll_z_m
    pitch_z_m = -pitch_sine * x_m + pitch_cosine * roll_z_m
    return (
        yaw_cosine * pitch_x_m - yaw_sine * roll_y_m,
        yaw_sine * pitch_x_m + yaw_cosine * roll_y_m,
        pitch_z_m,
    )


def _validated_mount(document):
    try:
        mount = document['camera_mount']
        transform = mount['transform']
    except (KeyError, TypeError) as error:
        raise ValueError('camera mount document is incomplete') from error
    if (mount.get('status') not in {'measured', 'derived'}
            or mount.get('parent_frame') != 'base_link'
            or mount.get('child_frame') != 'camera_link'):
        raise ValueError(
            'camera mount must be measured or derived '
            'base_link to camera_link')
    values = _finite_fields(
        transform,
        ('x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad'),
        'camera_mount.transform')
    return values


def _validated_observation(document, now_s, *, require_vertical_position):
    if (not isinstance(document, dict)
            or document.get('detected') is not True
            or document.get('stable') is not True
            or document.get('surface_kind') != 'front'):
        raise ValueError('stable detected front surface is required')
    stamp_s, distance_m, lateral_m, confidence = _finite_fields(
        document,
        ('stamp_s', 'front_distance_m', 'lateral_error_m', 'confidence'),
        'observation')
    now_s = _finite_number(now_s, 'now_s')
    age_s = now_s - stamp_s
    if age_s < 0.0 or age_s > MAXIMUM_OBSERVATION_AGE_S:
        raise ValueError('box observation is stale or future-dated')
    if not MINIMUM_DISTANCE_M <= distance_m <= MAXIMUM_DISTANCE_M:
        raise ValueError('box front distance is outside the observed range')
    if not 0.0 <= confidence <= 1.0:
        raise ValueError('box confidence must be in [0, 1]')
    vertical_position = document.get('vertical_position_m')
    if require_vertical_position and vertical_position is None:
        raise ValueError(
            'nonzero camera roll/pitch requires observed '
            'vertical face position')
    if vertical_position is not None:
        vertical_position = _finite_number(
            vertical_position, 'observation.vertical_position_m')
    else:
        vertical_position = 0.0
    normal = document.get('plane_normal')
    if not isinstance(normal, (list, tuple)) or len(normal) != 3:
        raise ValueError('plane_normal must contain three optical components')
    normal = tuple(
        _finite_number(value, f'plane_normal[{index}]')
        for index, value in enumerate(normal))
    magnitude = math.sqrt(sum(value * value for value in normal))
    if not math.isclose(magnitude, 1.0, abs_tol=1e-3):
        raise ValueError('plane_normal must be a unit optical vector')
    if normal[2] >= 0.0:
        raise ValueError('front plane normal must point toward the camera')
    consistency_vertical_m = (
        vertical_position if require_vertical_position else 0.0)
    point_magnitude = math.sqrt(
        lateral_m * lateral_m
        + consistency_vertical_m * consistency_vertical_m
        + distance_m * distance_m)
    toward_camera = (-lateral_m / point_magnitude,
                     -consistency_vertical_m / point_magnitude,
                     -distance_m / point_magnitude)
    consistency = sum(a * b for a, b in zip(normal, toward_camera))
    if consistency < MINIMUM_NORMAL_POINT_DOT:
        raise ValueError(
            'plane normal and observed face point are inconsistent')
    return distance_m, lateral_m, vertical_position, normal


def compute_box_docking_target(
        observation, camera_mount, robot_pose, *, requested_gap_m,
        front_extent_m, now_s):
    """Estimate a map-frame base pose facing a stable observed box front."""
    gap_m = _finite_number(requested_gap_m, 'requested_gap_m')
    extent_m = _finite_number(front_extent_m, 'front_extent_m')
    if gap_m <= 0.0:
        raise ValueError('requested_gap_m must be positive')
    if extent_m <= 0.0:
        raise ValueError('front_extent_m must be positive')
    robot_x, robot_y, robot_yaw = _finite_fields(
        robot_pose, ('x_m', 'y_m', 'yaw_rad'), 'robot_pose')
    mount_x, mount_y, mount_z, roll_rad, pitch_rad, mount_yaw = (
        _validated_mount(camera_mount))
    tilted_mount = (
        not math.isclose(roll_rad, 0.0, abs_tol=1e-9)
        or not math.isclose(pitch_rad, 0.0, abs_tol=1e-9)
    )
    if not tilted_mount:
        roll_rad = 0.0
        pitch_rad = 0.0
    distance_m, lateral_m, vertical_m, optical_normal = (
        _validated_observation(
            observation, now_s,
            require_vertical_position=tilted_mount))

    # Optical (right, down, forward) -> mount (forward, left, up).
    face_x, face_y, _face_z = _rotate_rpy(
        distance_m, -lateral_m, -vertical_m,
        roll_rad, pitch_rad, mount_yaw)
    face_x += mount_x
    face_y += mount_y
    _face_z += mount_z
    normal_x, normal_y, _normal_z = _rotate_rpy(
        optical_normal[2], -optical_normal[0], -optical_normal[1],
        roll_rad, pitch_rad, mount_yaw)
    planar_norm = math.hypot(normal_x, normal_y)
    if planar_norm <= 1e-9:
        raise ValueError('plane normal has no usable base-plane direction')
    normal_x /= planar_norm
    normal_y /= planar_norm

    center_offset_m = extent_m + gap_m
    target_base_x = face_x + normal_x * center_offset_m
    target_base_y = face_y + normal_y * center_offset_m
    target_map_dx, target_map_dy = _rotate_xy(
        target_base_x, target_base_y, robot_yaw)
    face_map_dx, face_map_dy = _rotate_xy(face_x, face_y, robot_yaw)
    normal_map_x, normal_map_y = _rotate_xy(
        normal_x, normal_y, robot_yaw)
    heading_base = math.atan2(-normal_y, -normal_x)
    estimate_gap_m = math.hypot(
        face_x - target_base_x, face_y - target_base_y) - extent_m
    return {
        'x_m': robot_x + target_map_dx,
        'y_m': robot_y + target_map_dy,
        'yaw_rad': _normalize_angle(robot_yaw + heading_base),
        'estimate_gap_m': estimate_gap_m,
        'provenance': PROVENANCE,
        'face_center_map_xy_m': (
            robot_x + face_map_dx, robot_y + face_map_dy),
        'outward_normal_map_xy': (normal_map_x, normal_map_y),
    }
