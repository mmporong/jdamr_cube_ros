"""Tests for pure reverse-parking path and controller helpers."""

import math
from copy import deepcopy  # noqa: I100
from pathlib import Path

from jdamr_cube_navigation.reverse_parking import (  # noqa: I101
    RPP_PLUGIN, reverse_controller_overrides, reverse_waypoints,
    static_corridor_clear,
)

import pytest

import yaml


def _controller(desired_linear_vel=0.12):
    return {
        'controller_plugins': ['FollowPath', 'Parking'],
        'FollowPath': {'plugin': RPP_PLUGIN},
        'Parking': {
            'plugin': RPP_PLUGIN,
            'desired_linear_vel': desired_linear_vel,
            'use_rotate_to_heading': True,
            'allow_reversing': False,
            'use_collision_detection': True,
            'lookahead_dist': 0.4,
        },
    }


def _write_grid(
        root: Path, name: str, pixels: list[int], *, width: int = 7,
        height: int = 5, resolution: float = 0.1,
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
        negate: int = 0) -> Path:
    """Write a small binary PGM map fixture and its YAML metadata."""
    image = root / f'{name}.pgm'
    image.write_bytes(
        f'P5\n{width} {height}\n255\n'.encode('ascii') + bytes(pixels))
    metadata = root / f'{name}.yaml'
    metadata.write_text(yaml.safe_dump({
        'image': image.name,
        'resolution': resolution,
        'origin': list(origin),
        'negate': negate,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }), encoding='utf-8')
    return metadata


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2.0, math.pi])
def test_waypoints_reverse_along_cardinal_headings(yaw):
    """Generate dense rearward segments and retain both exact endpoints."""
    target = (1.0, -2.0, yaw)
    start = (
        target[0] + 0.2 * math.cos(yaw),
        target[1] + 0.2 * math.sin(yaw),
        yaw,
    )

    path = reverse_waypoints(start, target)

    assert path[0] == start
    assert path[-1] == target
    for current, following in zip(path, path[1:]):
        dx = following[0] - current[0]
        dy = following[1] - current[1]
        assert math.hypot(dx, dy) <= 0.025 + 1e-12
        assert dx * math.cos(current[2]) + dy * math.sin(current[2]) < 0.0


def test_waypoints_interpolate_shortest_yaw_across_pi_boundary():
    """Interpolate measured yaw without taking the long rotation."""
    start_yaw = math.radians(179.0)
    target_yaw = math.radians(-179.0)
    target = (0.0, 0.0, target_yaw)
    start = (
        0.2 * math.cos(target_yaw),
        0.2 * math.sin(target_yaw),
        start_yaw,
    )

    path = reverse_waypoints(start, target)

    assert path[0][2] == start_yaw
    assert path[-1][2] == target_yaw
    yaw_steps = [
        math.atan2(
            math.sin(following[2] - current[2]),
            math.cos(following[2] - current[2]))
        for current, following in zip(path, path[1:])
    ]
    assert all(0.0 < step < math.radians(1.0) for step in yaw_steps)


@pytest.mark.parametrize('start,target,kwargs,match', [
    ((-0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {}, 'in front'),
    ((0.2, 0.051, 0.0), (0.0, 0.0, 0.0), {}, 'lateral'),
    ((0.2, 0.0, math.radians(3.1)), (0.0, 0.0, 0.0), {}, 'yaw'),
    ((0.05, 0.0, 0.0), (0.0, 0.0, 0.0), {}, 'already at goal'),
    ((2.01, 0.0, 0.0), (0.0, 0.0, 0.0), {}, 'max_distance'),
    ((0.001, 0.05, math.radians(-3.0)), (0.0, 0.0, 0.0), {},
     'not behind'),
])
def test_waypoints_reject_unsafe_geometry(start, target, kwargs, match):
    """Reject paths that are not short, aligned, and reverse-only."""
    with pytest.raises(ValueError, match=match):
        reverse_waypoints(start, target, **kwargs)


@pytest.mark.parametrize('start,target,kwargs', [
    ((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0), {}),
    ((True, 0.0, 0.0), (0.0, 0.0, 0.0), {}),
    ((0.2, 0.0, 0.0), (0.0, math.inf, 0.0), {}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {'spacing_m': False}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {'spacing_m': 0.0}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {'spacing_m': 0.026}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {'xy_tolerance_m': 0.051}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {'max_distance_m': 2.01}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0),
     {'yaw_tolerance_rad': math.inf}),
])
def test_waypoints_reject_nonfinite_boolean_and_invalid_bounds(
        start, target, kwargs):
    """Reject malformed poses and attempts to relax accepted bounds."""
    with pytest.raises(ValueError):
        reverse_waypoints(start, target, **kwargs)


def test_controller_override_is_isolated_and_safety_limited():
    """Clone Parking without mutating or aliasing the caller's mappings."""
    original = _controller()
    before = deepcopy(original)

    configured = reverse_controller_overrides(original)

    assert original == before
    assert configured['controller_plugins'] == [
        'FollowPath', 'Parking', 'ParkingReverse']
    assert configured['Parking'] == original['Parking']
    reverse = configured['ParkingReverse']
    assert reverse['plugin'] == RPP_PLUGIN
    assert reverse['desired_linear_vel'] == 0.08
    assert reverse['use_rotate_to_heading'] is False
    assert reverse['allow_reversing'] is True
    assert reverse['use_collision_detection'] is True
    assert reverse['lookahead_dist'] == 0.4

    reverse['lookahead_dist'] = 99.0
    assert configured['Parking']['lookahead_dist'] == 0.4
    assert original['Parking']['lookahead_dist'] == 0.4


def test_controller_override_keeps_stricter_parking_velocity():
    """Avoid raising a source controller velocity already below the cap."""
    configured = reverse_controller_overrides(_controller(0.05))
    assert configured['ParkingReverse']['desired_linear_vel'] == 0.05


@pytest.mark.parametrize('mutate', [
    lambda value: value['controller_plugins'].remove('Parking'),
    lambda value: value['controller_plugins'].append('Parking'),
    lambda value: value.pop('Parking'),
    lambda value: value['Parking'].__setitem__(
        'use_collision_detection', False),
    lambda value: value['Parking'].__setitem__('plugin', 'not-rpp'),
    lambda value: value['Parking'].__setitem__('desired_linear_vel', True),
    lambda value: value['controller_plugins'].append('ParkingReverse'),
])
def test_controller_override_rejects_missing_duplicate_or_unsafe_source(
        mutate):
    """Require one collision-enabled RPP Parking source and a free ID."""
    controller = _controller()
    mutate(controller)
    with pytest.raises(ValueError):
        reverse_controller_overrides(controller)


def test_static_corridor_accepts_only_clear_map_and_keepout_cells(tmp_path):
    """Accept a footprint corridor when both aligned grids are free."""
    free = [254] * (9 * 7)
    source = _write_grid(tmp_path, 'map', free, width=9, height=7)
    keepout = _write_grid(tmp_path, 'keepout', free, width=9, height=7)

    assert static_corridor_clear(
        source, keepout, [(0.45, 0.35, math.pi / 2.0)],
        [(-0.04, -0.02), (0.04, -0.02), (0.04, 0.02),
         (-0.04, 0.02)])


@pytest.mark.parametrize('grid_name,pixel', [
    ('map', 205),
    ('map', 0),
    ('keepout', 205),
    ('keepout', 0),
])
def test_static_corridor_rejects_unknown_occupied_and_keepout_pixels(
        tmp_path, grid_name, pixel):
    """Treat every pixel that is not clearly free as blocked."""
    grids = {
        'map': [254] * (9 * 7),
        'keepout': [254] * (9 * 7),
    }
    center_index = (7 - 1 - 3) * 9 + 4
    grids[grid_name][center_index] = pixel
    source = _write_grid(tmp_path, 'map', grids['map'], width=9, height=7)
    keepout = _write_grid(
        tmp_path, 'keepout', grids['keepout'], width=9, height=7)

    assert not static_corridor_clear(
        source, keepout, [(0.45, 0.35, 0.0)],
        [(-0.03, -0.03), (0.03, -0.03), (0.03, 0.03),
         (-0.03, 0.03)])


def test_static_corridor_applies_negate_and_free_threshold(tmp_path):
    """Honor map-server occupancy conversion for negated images."""
    free_when_negated = [0] * (9 * 7)
    source = _write_grid(
        tmp_path, 'map', free_when_negated, width=9, height=7, negate=1)
    keepout = _write_grid(
        tmp_path, 'keepout', free_when_negated,
        width=9, height=7, negate=1)
    footprint = [
        (-0.03, -0.03), (0.03, -0.03), (0.03, 0.03), (-0.03, 0.03)]

    assert static_corridor_clear(
        source, keepout, [(0.45, 0.35, 0.0)], footprint)

    occupied = list(free_when_negated)
    occupied[(7 - 1 - 3) * 9 + 4] = 255
    blocked_source = _write_grid(
        tmp_path, 'blocked', occupied, width=9, height=7, negate=1)
    assert not static_corridor_clear(
        blocked_source, keepout, [(0.45, 0.35, 0.0)], footprint)


def test_static_corridor_expands_between_pose_samples(tmp_path):
    """Cover translation gaps between poses with a conservative margin."""
    source_pixels = [254] * (15 * 7)
    source_pixels[(7 - 1 - 3) * 15 + 6] = 0
    source = _write_grid(
        tmp_path, 'map', source_pixels, width=15, height=7)
    keepout = _write_grid(
        tmp_path, 'keepout', [254] * (15 * 7), width=15, height=7)

    assert not static_corridor_clear(
        source, keepout, [(0.45, 0.35, 0.0), (0.75, 0.35, 0.0)],
        [(-0.02, -0.02), (0.02, -0.02), (0.02, 0.02),
         (-0.02, 0.02)])


def test_static_corridor_does_not_expand_into_neighboring_cells(tmp_path):
    """Avoid rejecting a free swept polygon for a nearby blocked cell."""
    source_pixels = [254] * (15 * 7)
    source_pixels[(7 - 1 - 2) * 15 + 6] = 0
    source = _write_grid(
        tmp_path, 'map', source_pixels, width=15, height=7)
    keepout = _write_grid(
        tmp_path, 'keepout', [254] * (15 * 7), width=15, height=7)

    assert static_corridor_clear(
        source, keepout, [(0.45, 0.35, 0.0), (0.75, 0.35, 0.0)],
        [(-0.02, -0.02), (0.02, -0.02), (0.02, 0.02),
         (-0.02, 0.02)])


def test_static_corridor_rejects_map_boundary_and_grid_mismatch(tmp_path):
    """Fail closed outside either grid and reject differing geometry."""
    free = [254] * (9 * 7)
    source = _write_grid(tmp_path, 'map', free, width=9, height=7)
    keepout = _write_grid(tmp_path, 'keepout', free, width=9, height=7)
    footprint = [
        (-0.04, -0.04), (0.04, -0.04), (0.04, 0.04), (-0.04, 0.04)]

    assert not static_corridor_clear(
        source, keepout, [(0.04, 0.35, 0.0)], footprint)

    mismatch = _write_grid(
        tmp_path, 'mismatch', free, width=9, height=7, resolution=0.11)
    with pytest.raises(ValueError, match='identical geometry'):
        static_corridor_clear(
            source, mismatch, [(0.45, 0.35, 0.0)], footprint)


def test_static_corridor_rejects_rotated_grid_origins(tmp_path):
    """Fail closed even when both grids share one unsupported rotation."""
    free = [254] * (9 * 7)
    rotated_origin = (0.0, 0.0, 0.1)
    source = _write_grid(
        tmp_path, 'map', free, width=9, height=7, origin=rotated_origin)
    keepout = _write_grid(
        tmp_path, 'keepout', free, width=9, height=7,
        origin=rotated_origin)

    with pytest.raises(ValueError, match='rotated map origins'):
        static_corridor_clear(
            source, keepout, [(0.45, 0.35, 0.0)],
            [(-0.04, -0.04), (0.04, -0.04), (0.04, 0.04),
             (-0.04, 0.04)])
