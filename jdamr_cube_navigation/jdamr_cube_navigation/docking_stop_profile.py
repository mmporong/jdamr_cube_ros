"""Pure bounded StopZone adjustment for the user-approved docking target."""

from copy import deepcopy
import json
import math


# This is a user-approved target clearance, not an externally measured result.
USER_APPROVED_CLEARANCE_M = 0.05
MINIMUM_PADDED_STOP_MARGIN_M = 0.02


def _finite_positive(value, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0.0):
        raise ValueError(f'{label} must be finite and positive')
    return float(value)


def _polygon(value, label):
    try:
        points = json.loads(value) if isinstance(value, str) else deepcopy(value)
    except (TypeError, json.JSONDecodeError) as error:
        raise ValueError(f'{label} must be a polygon') from error
    if (not isinstance(points, list) or len(points) != 4
            or any(not isinstance(point, (list, tuple)) or len(point) != 2
                   or any(isinstance(coordinate, bool)
                          or not isinstance(coordinate, (int, float))
                          or not math.isfinite(coordinate)
                          for coordinate in point)
                   for point in points)):
        raise ValueError(f'{label} must contain four finite XY points')
    return [list(point) for point in points]


def _rectangle_front(points, label):
    xs = sorted({float(point[0]) for point in points})
    ys = sorted({float(point[1]) for point in points})
    if (len(xs) != 2 or len(ys) != 2 or not math.isclose(
            ys[0], -ys[1], abs_tol=1e-9)
            or {tuple(point) for point in points}
            != {(x_m, y_m) for x_m in xs for y_m in ys}):
        raise ValueError(f'{label} must be an axis-aligned symmetric rectangle')
    return xs[1]


def _geometry_front(geometry):
    if not isinstance(geometry, dict):
        raise ValueError('geometry must be a mapping')
    try:
        front = geometry['front_to_wheel_axis']
        length = geometry['frame_length']
        width = geometry['wheel_outer_width']
    except KeyError as error:
        raise ValueError('geometry is missing measured base dimensions') from error
    for label, entry in (
            ('front_to_wheel_axis', front),
            ('frame_length', length),
            ('wheel_outer_width', width)):
        if not isinstance(entry, dict) or entry.get('unit') != 'm':
            raise ValueError(f'geometry {label} must use meters')
    front_m = _finite_positive(front.get('value'), 'front_to_wheel_axis')
    length_m = _finite_positive(length.get('value'), 'frame_length')
    _finite_positive(width.get('value'), 'wheel_outer_width')
    if front_m >= length_m:
        raise ValueError('front_to_wheel_axis must be smaller than frame_length')
    return front_m


def _costmap_front(document, physical_front_m):
    fronts = []
    for name in ('local_costmap', 'global_costmap'):
        try:
            value = document[name][name]['ros__parameters']['footprint']
        except (KeyError, TypeError) as error:
            raise ValueError(f'{name} footprint is missing') from error
        points = _polygon(value, f'{name} footprint')
        fronts.append(_rectangle_front(points, f'{name} footprint'))
    if not math.isclose(fronts[0], fronts[1], abs_tol=1e-9):
        raise ValueError('local and global costmap footprint fronts differ')
    padding_m = fronts[0] - physical_front_m
    if padding_m < 0.0:
        raise ValueError('costmap footprint front is behind the physical base')
    if USER_APPROVED_CLEARANCE_M + 1e-9 < padding_m:
        raise ValueError('approved clearance is smaller than footprint padding')
    return fronts[0]


def _set_front(points, front_m):
    old_front_m = max(float(point[0]) for point in points)
    if old_front_m + 1e-9 < front_m:
        raise ValueError('precision profile must not expand a smaller StopZone')
    for point in points:
        if math.isclose(float(point[0]), old_front_m, abs_tol=1e-9):
            point[0] = front_m


def apply_docking_stop_profile(nav2_document, geometry):
    """Return a copy with only forward/stopped StopZone front X adjusted."""
    if not isinstance(nav2_document, dict):
        raise ValueError('Nav2 document must be a mapping')
    physical_front_m = _geometry_front(geometry)
    padded_front_m = _costmap_front(nav2_document, physical_front_m)
    target_front_m = physical_front_m + USER_APPROVED_CLEARANCE_M
    if target_front_m + 1e-9 < (
            padded_front_m + MINIMUM_PADDED_STOP_MARGIN_M):
        raise ValueError('docking StopZone front lacks padded footprint margin')

    result = deepcopy(nav2_document)
    try:
        stop_zone = result['collision_monitor']['ros__parameters']['StopZone']
    except (KeyError, TypeError) as error:
        raise ValueError('collision_monitor StopZone is missing') from error
    for name in ('translation_forward', 'stopped'):
        try:
            original = stop_zone[name]['points']
        except (KeyError, TypeError) as error:
            raise ValueError(f'StopZone {name} polygon is missing') from error
        points = _polygon(original, f'StopZone {name}')
        _rectangle_front(points, f'StopZone {name}')
        _set_front(points, target_front_m)
        stop_zone[name]['points'] = (
            json.dumps(points) if isinstance(original, str) else points)
    return result
