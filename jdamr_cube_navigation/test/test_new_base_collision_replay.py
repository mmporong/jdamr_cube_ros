"""Unit coverage for the offline new-base collision replay."""

import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = (
    ROOT / 'jdamr_cube_navigation/evaluation/replay_new_base_collision.py')
PARAMS = ROOT / 'jdamr_cube_navigation/config/new_base_nav2_params.yaml'
GEOMETRY = ROOT / 'jdamr_cube_description/config/new_base_geometry.yaml'


def _load():
    spec = importlib.util.spec_from_file_location(
        'new_base_collision_replay', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


REPLAY = _load()


def _stop_zone():
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    return document['collision_monitor']['ros__parameters']['StopZone']


@pytest.mark.parametrize(('linear_x', 'angular_z', 'expected'), [
    (0.0, 0.0, 'stopped'),
    (0.0, 0.005, 'rotation'),
    (0.0, 1.0, 'rotation'),
    (0.0, -0.005, 'rotation_clockwise'),
    (0.0, -1.0, 'rotation_clockwise'),
    (0.005, 0.0, 'translation_forward'),
    (0.2, 1.0, 'translation_forward'),
    (-0.005, 0.0, 'translation_backward'),
    (-0.2, -1.0, 'translation_backward'),
])
def test_current_velocity_polygon_order_and_inclusive_boundaries(
        linear_x, angular_z, expected):
    """Select zero, turn, forward, and reverse boundaries in order."""
    assert REPLAY.select_velocity_polygon(
        _stop_zone(), linear_x, angular_z) == expected


def test_uncovered_velocity_is_unknown_instead_of_assumed_fallback():
    """Report an uncovered callback rather than inventing a fallback."""
    assert REPLAY.select_velocity_polygon(_stop_zone(), 1.1, 0.0) is None
    assert REPLAY.select_velocity_polygon(_stop_zone(), 0.0, 1.1) is None


def test_old_rotation_matching_is_explicitly_counterfactual():
    """The old hypothesis gives zero and clockwise edge to rotation."""
    old = REPLAY.old_rotation_matching(_stop_zone())
    assert REPLAY.select_velocity_polygon(old, 0.0, 0.0) == 'rotation'
    assert REPLAY.select_velocity_polygon(old, 0.0, -1.0) == 'rotation'


@pytest.mark.parametrize(('point', 'expected'), [
    ((0.5, 0.5), True),
    ((0.0, 0.0), False),
    ((1.0, 0.0), False),
    ((0.0, 1.0), True),
    ((-0.01, 0.5), False),
])
def test_polygon_membership_matches_jazzy_boundary_asymmetry(point, expected):
    """Preserve upstream strict and inclusive boundary asymmetry."""
    polygon = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    assert REPLAY.point_in_polygon(point, polygon) is expected


def test_scan_filter_rejects_nan_inf_and_out_of_range():
    """Keep only finite readings inside the declared sensor range."""
    scan = SimpleNamespace(
        ranges=[math.nan, math.inf, 0.27, 0.28, 1.0, 16.0, 16.01],
        range_min=0.28, range_max=16.0,
        angle_min=0.0, angle_increment=0.0,
    )
    identity = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    points, rejected = REPLAY.scan_points(scan, identity)
    assert [point[0] for point in points] == pytest.approx([0.28, 1.0, 16.0])
    assert rejected == {'non_finite': 2, 'outside_valid_range': 2}


def test_recorded_static_tf_chain_converts_laser_to_base():
    """Compose the recorded laser-to-base static chain."""
    def transform(parent, child, x, yaw):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=parent), child_frame_id=child,
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=x, y=0.0, z=0.0),
                rotation=SimpleNamespace(
                    x=0.0, y=0.0, z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0))))

    transforms = [
        transform('base_footprint', 'base_link', 0.0, 0.0),
        transform('base_link', 'laser_link', -0.01, math.pi),
    ]
    base_from_scan, chain = REPLAY.resolve_transform(
        transforms, 'laser_link', 'base_footprint')
    assert chain == ['laser_link', 'base_link', 'base_footprint']
    assert REPLAY.apply_transform(base_from_scan, (1.0, 0.0, 0.0)) == (
        pytest.approx(-1.01), pytest.approx(0.0, abs=1e-12),
        pytest.approx(0.0))


@pytest.mark.parametrize(('translation', 'quaternion'), [
    ((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, math.inf, 1.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
])
def test_static_tf_fails_closed_on_invalid_numbers(translation, quaternion):
    """Reject non-finite and zero-length static transforms."""
    item = SimpleNamespace(
        header=SimpleNamespace(frame_id='base_footprint'),
        child_frame_id='laser_link',
        transform=SimpleNamespace(
            translation=SimpleNamespace(
                x=translation[0], y=translation[1], z=translation[2]),
            rotation=SimpleNamespace(
                x=quaternion[0], y=quaternion[1], z=quaternion[2],
                w=quaternion[3])))
    with pytest.raises(ValueError):
        REPLAY.resolve_transform([item], 'laser_link', 'base_footprint')


def test_static_tf_fails_closed_on_conflicting_parent():
    """Reject a static child assigned to two parents."""
    def item(parent):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=parent),
            child_frame_id='laser_link',
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))

    with pytest.raises(ValueError, match='conflicting static TF parents'):
        REPLAY.resolve_transform(
            [item('base_link'), item('base_footprint')],
            'laser_link', 'base_footprint')


def test_actual_body_and_padded_clearance_keep_chair_point_outside():
    """Do not call a forward chair point a lateral body collision."""
    geometry = yaml.safe_load(GEOMETRY.read_text(encoding='utf-8'))
    actual = REPLAY.actual_body_bounds(geometry)
    padded = (-0.295, 0.085, -0.29, 0.29)
    chair = (0.29, -0.30)
    assert actual == pytest.approx((-0.275, 0.065, -0.27, 0.27))
    assert REPLAY.rectangle_clearance(chair, actual) > 0.225
    assert REPLAY.rectangle_clearance(chair, padded) == pytest.approx(
        math.hypot(0.205, 0.01))


def test_rectangle_clearance_is_signed_for_body_hits():
    """Distinguish interior hits, boundary contact, and free clearance."""
    bounds = (-1.0, 1.0, -0.5, 0.5)
    assert REPLAY.rectangle_clearance((0.0, 0.0), bounds) == -0.5
    assert REPLAY.rectangle_clearance((1.0, 0.0), bounds) == 0.0
    assert REPLAY.rectangle_clearance((2.0, 0.0), bounds) == 1.0
