"""Pure geometry tests for nominal-camera box docking targets."""

from copy import deepcopy
import math
from pathlib import Path

from jdamr_cube_navigation.box_docking_target import (
    compute_box_docking_target,
    PROVENANCE,
)
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]


def mount():
    return {
        'camera_mount': {
            'status': 'measured',
            'parent_frame': 'base_link',
            'child_frame': 'camera_link',
            'transform': {
                'x_m': 0.065, 'y_m': 0.0, 'z_m': 0.215,
                'roll_rad': 0.0, 'pitch_rad': 0.0, 'yaw_rad': 0.0,
            },
        },
    }


def observation():
    return {
        'stamp_s': 10.0,
        'detected': True,
        'stable': True,
        'surface_kind': 'front',
        'front_distance_m': 0.8,
        'lateral_error_m': 0.0,
        'plane_normal': [0.0, 0.0, -1.0],
        'confidence': 0.9,
    }


def target(observed=None, mounted=None, robot=None, gap=0.05, extent=0.25):
    return compute_box_docking_target(
        observation() if observed is None else observed,
        mount() if mounted is None else mounted,
        ({'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
         if robot is None else robot),
        requested_gap_m=gap, front_extent_m=extent, now_s=10.2)


def test_centered_face_produces_five_centimeter_estimated_gap():
    result = target()
    face_x_m = 0.065 + 0.8
    assert result['x_m'] == pytest.approx(face_x_m - 0.25 - 0.05)
    assert result['y_m'] == pytest.approx(0.0)
    assert result['yaw_rad'] == pytest.approx(0.0)
    assert result['estimate_gap_m'] == pytest.approx(0.05)
    assert result['provenance'] == PROVENANCE
    assert result['face_center_map_xy_m'] == pytest.approx((face_x_m, 0.0))
    assert result['outward_normal_map_xy'] == pytest.approx((-1.0, 0.0))


def test_requested_predock_gap_uses_same_face_geometry():
    result = target(gap=0.45)
    assert result['x_m'] == pytest.approx(0.065 + 0.8 - 0.25 - 0.45)
    assert result['estimate_gap_m'] == pytest.approx(0.45)


@pytest.mark.parametrize(
    ('normal_x', 'lateral_m', 'expected_sign'),
    [(math.sin(0.2), 0.08, 1.0), (-math.sin(0.2), -0.08, -1.0)],
)
def test_optical_normal_x_controls_heading_sign(
        normal_x, lateral_m, expected_sign):
    observed = observation()
    observed['lateral_error_m'] = lateral_m
    observed['plane_normal'] = [normal_x, 0.0, -math.cos(0.2)]
    result = target(observed=observed)
    assert math.copysign(1.0, result['yaw_rad']) == expected_sign
    assert abs(result['yaw_rad']) == pytest.approx(0.2)


@pytest.mark.parametrize(('lateral_m', 'expected_y'), [(0.1, -0.1), (-0.1, 0.1)])
def test_optical_right_maps_to_base_right_negative_left_axis(
        lateral_m, expected_y):
    observed = observation()
    observed['lateral_error_m'] = lateral_m
    result = target(observed=observed)
    assert result['y_m'] == pytest.approx(expected_y)


def test_robot_map_yaw_rotates_target_and_heading():
    result = target(robot={'x_m': 1.0, 'y_m': 2.0, 'yaw_rad': math.pi / 2.0})
    local_x = 0.065 + 0.8 - 0.25 - 0.05
    assert result['x_m'] == pytest.approx(1.0)
    assert result['y_m'] == pytest.approx(2.0 + local_x)
    assert result['yaw_rad'] == pytest.approx(math.pi / 2.0)
    assert result['face_center_map_xy_m'] == pytest.approx((1.0, 2.865))
    assert result['outward_normal_map_xy'] == pytest.approx((0.0, -1.0))


def test_camera_forward_translation_shifts_goal_by_same_amount():
    translated = target()
    centered_mount = mount()
    centered_mount['camera_mount']['transform']['x_m'] = 0.0
    centered = target(mounted=centered_mount)
    assert translated['x_m'] - centered['x_m'] == pytest.approx(0.065)


def test_repository_nominal_camera_mount_is_consumed_as_configuration():
    configured_mount = yaml.safe_load((
        ROOT / 'jdamr_cube_vslam/config/camera_mount.yaml'
    ).read_text(encoding='utf-8'))
    result = target(mounted=configured_mount)
    assert result['x_m'] == pytest.approx(0.065 + 0.8 - 0.25 - 0.05)
    assert result['provenance'] == 'NOMINAL_CAMERA_MOUNT_ESTIMATE'


@pytest.mark.parametrize(
    'mutate, message', [
        (lambda item: item.update(front_distance_m=math.nan), 'finite'),
        (lambda item: item.update(stamp_s=math.nan), 'finite'),
        (lambda item: item.update(stamp_s=9.0), 'stale'),
        (lambda item: item.update(front_distance_m=-0.2), 'observed range'),
        (lambda item: item.update(confidence=math.nan), 'finite'),
        (lambda item: item.update(surface_kind='top'), 'front surface'),
        (lambda item: item.update(plane_normal=[0.0, 0.0, 1.0]),
         'toward the camera'),
        (lambda item: item.update(plane_normal=[0.0, 0.0, -2.0]),
         'unit optical vector'),
    ],
)
def test_invalid_or_unsafe_observations_are_rejected(mutate, message):
    observed = observation()
    mutate(observed)
    with pytest.raises(ValueError, match=message):
        target(observed=observed)


def test_unit_but_nonfront_normal_is_rejected_by_point_consistency():
    observed = observation()
    observed['plane_normal'] = [math.sqrt(0.5), 0.0, -math.sqrt(0.5)]
    with pytest.raises(ValueError, match='inconsistent'):
        target(observed=observed)


@pytest.mark.parametrize('axis', ['roll_rad', 'pitch_rad'])
def test_nonzero_nominal_tilt_is_rejected_without_vertical_face_position(axis):
    mounted = deepcopy(mount())
    mounted['camera_mount']['transform'][axis] = 0.01
    with pytest.raises(ValueError, match='vertical face position'):
        target(mounted=mounted)


@pytest.mark.parametrize(
    'robot, gap, extent', [
        ({'x_m': math.nan, 'y_m': 0.0, 'yaw_rad': 0.0}, 0.05, 0.25),
        ({'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}, math.nan, 0.25),
        ({'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}, 0.05, 0.0),
    ],
)
def test_nonfinite_pose_gap_or_nonpositive_extent_is_rejected(
        robot, gap, extent):
    with pytest.raises(ValueError):
        target(robot=robot, gap=gap, extent=extent)
