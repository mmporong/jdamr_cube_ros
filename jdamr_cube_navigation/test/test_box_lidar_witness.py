"""Pure tests for one stopped LiDAR/depth box-face witness."""

import math
from pathlib import Path

from jdamr_cube_navigation.box_lidar_witness import (
    PROVENANCE,
    witness_box_face_with_lidar,
)
import numpy as np
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
GEOMETRY = yaml.safe_load((
    ROOT / 'jdamr_cube_description/config/new_base_geometry.yaml'
).read_text(encoding='utf-8'))
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2.0 * math.pi / 720.0
RANGE_MIN = 0.05
RANGE_MAX = 12.0


def target(face=(0.5, 0.0), normal=(-1.0, 0.0)):
    return {
        'face_center_map_xy_m': face,
        'outward_normal_map_xy': normal,
    }


def scan_for_face(face, normal, robot, *, tangent_limit=0.20, offset=None):
    ranges = [math.inf] * 721
    robot_xy = np.array((robot['x_m'], robot['y_m']))
    face = np.array(face, dtype=np.float64)
    normal = np.array(normal, dtype=np.float64)
    tangent = np.array((-normal[1], normal[0]))
    laser_translation = np.array(
        GEOMETRY['laser_translation']['value'][:2], dtype=np.float64)
    laser_origin = robot_xy + rotate(laser_translation, robot['yaw_rad'])
    laser_yaw_map = robot['yaw_rad'] + GEOMETRY['laser_yaw']['value']
    for index in range(len(ranges)):
        angle = ANGLE_MIN + index * ANGLE_INCREMENT
        direction = np.array((math.cos(angle + laser_yaw_map),
                              math.sin(angle + laser_yaw_map)))
        denominator = float(normal @ direction)
        if abs(denominator) < 1e-9:
            continue
        distance = float(normal @ (face - laser_origin) / denominator)
        if distance <= RANGE_MIN or distance > RANGE_MAX:
            continue
        hit = laser_origin + direction * distance
        if abs(float((hit - face) @ tangent)) <= tangent_limit:
            if offset is not None:
                distance += offset(index) / denominator
            ranges[index] = distance
    return ranges


def rotate(vector, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    x_m, y_m = vector
    return np.array((cosine * x_m - sine * y_m,
                     sine * x_m + cosine * y_m))


def witness(ranges, depth_target=None, robot=None, geometry=None):
    return witness_box_face_with_lidar(
        ranges,
        angle_min=ANGLE_MIN,
        angle_increment=ANGLE_INCREMENT,
        range_min=RANGE_MIN,
        range_max=RANGE_MAX,
        geometry=GEOMETRY if geometry is None else geometry,
        depth_target=target() if depth_target is None else depth_target,
        robot_pose=(
            {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
            if robot is None else robot),
    )


def test_aligned_face_at_predock_distance_has_bounded_witness():
    assert GEOMETRY['laser_translation']['value'][0] == -0.01
    assert GEOMETRY['laser_yaw']['value'] == math.pi
    robot = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    ranges = scan_for_face((0.5, 0.0), (-1.0, 0.0), robot)
    result = witness(ranges, robot=robot)
    assert result['fused_face_center_map_xy_m'] == pytest.approx((0.5, 0.0))
    assert result['outward_normal_map_xy'] == pytest.approx((-1.0, 0.0))
    assert result['residual_rms_m'] < 1e-9
    assert abs(result['distance_difference_m']) < 1e-9
    assert result['normal_yaw_difference_rad'] < 1e-9
    assert result['support_count'] >= 5
    assert result['tangent_spread_m'] >= 0.08
    assert result['provenance'] == PROVENANCE


def test_map_face_and_normal_at_45_degrees_are_recovered():
    angle = math.radians(45.0)
    face = (0.5 * math.cos(angle), 0.5 * math.sin(angle))
    normal = (-math.cos(angle), -math.sin(angle))
    robot = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    ranges = scan_for_face(face, normal, robot)
    result = witness(ranges, target(face, normal), robot)
    assert result['fused_face_center_map_xy_m'] == pytest.approx(face, abs=1e-6)
    assert result['outward_normal_map_xy'] == pytest.approx(normal, abs=1e-6)


def test_robot_rotated_180_and_laser_rotated_pi_are_applied():
    robot = {'x_m': 1.0, 'y_m': 0.3, 'yaw_rad': math.pi}
    face = (0.5, 0.3)
    normal = (1.0, 0.0)
    ranges = scan_for_face(face, normal, robot)
    result = witness(ranges, target(face, normal), robot)
    assert result['fused_face_center_map_xy_m'] == pytest.approx(face)
    assert result['outward_normal_map_xy'] == pytest.approx(normal)


def test_invalid_ranges_and_nan_mount_are_rejected():
    with pytest.raises(ValueError, match='range bounds'):
        witness_box_face_with_lidar(
            [1.0], angle_min=0.0, angle_increment=0.1,
            range_min=2.0, range_max=1.0, geometry=GEOMETRY,
            depth_target=target(),
            robot_pose={'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0})
    geometry = yaml.safe_load(yaml.safe_dump(GEOMETRY))
    geometry['laser_translation']['value'][0] = math.nan
    ranges = scan_for_face(
        (0.5, 0.0), (-1.0, 0.0),
        {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0})
    with pytest.raises(ValueError, match='finite'):
        witness(ranges, geometry=geometry)
    with pytest.raises(ValueError, match='no valid ranges'):
        witness([math.nan] * 721)


def test_fewer_than_five_points_is_rejected():
    ranges = [math.inf] * 721
    full = scan_for_face(
        (0.5, 0.0), (-1.0, 0.0),
        {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0})
    valid = [index for index, value in enumerate(full) if math.isfinite(value)]
    for index in valid[:4]:
        ranges[index] = full[index]
    with pytest.raises(ValueError, match='below five'):
        witness(ranges)


def test_narrow_tangent_support_is_rejected():
    robot = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    ranges = scan_for_face(
        (0.5, 0.0), (-1.0, 0.0), robot, tangent_limit=0.025)
    with pytest.raises(ValueError, match='spread is too narrow'):
        witness(ranges, robot=robot)


def test_in_window_alternating_outliers_fail_line_residual():
    robot = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    ranges = scan_for_face(
        (0.5, 0.0), (-1.0, 0.0), robot,
        offset=lambda index: 0.03 if index % 2 else -0.03)
    with pytest.raises(ValueError, match='residual is too large'):
        witness(ranges, robot=robot)


def test_uniform_small_offset_projects_depth_face_to_lidar_plane():
    robot = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    ranges = scan_for_face(
        (0.5, 0.0), (-1.0, 0.0), robot, offset=lambda _index: -0.01)
    result = witness(ranges, robot=robot)
    assert result['fused_face_center_map_xy_m'] == pytest.approx((0.51, 0.0))
    assert result['distance_difference_m'] == pytest.approx(-0.01)


def test_uniform_offset_beyond_distance_agreement_is_rejected():
    robot = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    ranges = scan_for_face(
        (0.5, 0.0), (-1.0, 0.0), robot, offset=lambda _index: -0.03)
    with pytest.raises(ValueError, match='distances disagree'):
        witness(ranges, robot=robot)


def test_line_normal_more_than_five_degrees_from_depth_is_rejected():
    robot = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    angle = math.radians(7.0)
    lidar_normal = (-math.cos(angle), -math.sin(angle))
    ranges = scan_for_face((0.5, 0.0), lidar_normal, robot)
    with pytest.raises(ValueError, match='normals disagree'):
        witness(ranges, robot=robot)
