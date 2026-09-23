"""
Pure helpers for a short Nav2 reverse-parking path.

Path generation validates geometry, and the static corridor check rejects
occupied, unknown and keepout cells. Live obstacle detection remains with
Nav2 costmaps and the collision monitor.
"""

from __future__ import annotations

import math
from copy import deepcopy  # noqa: I100
from pathlib import Path
from typing import Any, Sequence

from jdamr_cube_navigation.keepout_mask import _map_metadata


ALREADY_AT_GOAL_DISTANCE_M = 0.05
MAX_XY_TOLERANCE_M = 0.05
MAX_YAW_TOLERANCE_RAD = math.radians(3.0)
MAX_REVERSE_DISTANCE_M = 2.0
MAX_WAYPOINT_SPACING_M = 0.025
MAX_REVERSE_LINEAR_VELOCITY_MPS = 0.08
RPP_PLUGIN = (
    'nav2_regulated_pure_pursuit_controller::'
    'RegulatedPurePursuitController')


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f'{name} must be a finite number')
    return number


def _pose(value: Sequence[float], name: str) -> tuple[float, float, float]:
    try:
        if len(value) != 3:
            raise ValueError(f'{name} must contain x, y, and yaw')
    except TypeError as error:
        raise ValueError(f'{name} must contain x, y, and yaw') from error
    return tuple(_finite_number(item, name) for item in value)


def _shortest_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def reverse_waypoints(
        start_pose: Sequence[float], target_pose: Sequence[float],
        xy_tolerance_m: float = 0.05,
        yaw_tolerance_rad: float = math.radians(3.0),
        max_distance_m: float = 2.0,
        spacing_m: float = 0.025,
) -> list[tuple[float, float, float]]:
    """
    Interpolate an obstacle-unchecked, straight reverse-only path.

    The start must be in front of the target along the target heading, with
    lateral and yaw errors inside the supplied tolerances.  Both the measured
    start pose and the exact target pose are included in the returned path.
    """
    start = _pose(start_pose, 'start_pose')
    target = _pose(target_pose, 'target_pose')
    xy_tolerance = _finite_number(xy_tolerance_m, 'xy_tolerance_m')
    yaw_tolerance = _finite_number(
        yaw_tolerance_rad, 'yaw_tolerance_rad')
    max_distance = _finite_number(max_distance_m, 'max_distance_m')
    spacing = _finite_number(spacing_m, 'spacing_m')
    if not 0.0 <= xy_tolerance <= MAX_XY_TOLERANCE_M:
        raise ValueError('xy_tolerance_m exceeds the accepted bound')
    if not 0.0 <= yaw_tolerance <= MAX_YAW_TOLERANCE_RAD:
        raise ValueError('yaw_tolerance_rad exceeds the accepted bound')
    if not 0.0 < max_distance <= MAX_REVERSE_DISTANCE_M:
        raise ValueError('max_distance_m exceeds the accepted bound')
    if not 0.0 < spacing <= MAX_WAYPOINT_SPACING_M:
        raise ValueError('spacing_m exceeds the accepted bound')

    target_heading = (math.cos(target[2]), math.sin(target[2]))
    target_left = (-target_heading[1], target_heading[0])
    target_to_start = (start[0] - target[0], start[1] - target[1])
    forward_distance = sum(
        component * heading
        for component, heading in zip(target_to_start, target_heading))
    lateral_error = abs(sum(
        component * left
        for component, left in zip(target_to_start, target_left)))
    distance = math.hypot(*target_to_start)
    yaw_delta = _shortest_angle(target[2] - start[2])

    if distance <= ALREADY_AT_GOAL_DISTANCE_M:
        raise ValueError('already at goal')
    if distance > max_distance:
        raise ValueError('reverse path exceeds max_distance_m')
    if forward_distance <= 0.0:
        raise ValueError('start_pose must be in front of target_pose')
    if lateral_error > xy_tolerance:
        raise ValueError('start_pose lateral error exceeds xy_tolerance_m')
    if abs(yaw_delta) > yaw_tolerance:
        raise ValueError('start_pose yaw error exceeds yaw_tolerance_rad')

    segment_count = math.ceil(distance / spacing)
    waypoints = []
    for index in range(segment_count + 1):
        fraction = index / segment_count
        if index == segment_count:
            waypoints.append(target)
            continue
        x = start[0] + (target[0] - start[0]) * fraction
        y = start[1] + (target[1] - start[1]) * fraction
        yaw = start[2] if index == 0 else _shortest_angle(
            start[2] + yaw_delta * fraction)
        waypoints.append((x, y, yaw))

    for current, following in zip(waypoints, waypoints[1:]):
        displacement = (
            following[0] - current[0], following[1] - current[1])
        for yaw in (current[2], following[2]):
            rearward_dot = (
                displacement[0] * math.cos(yaw)
                + displacement[1] * math.sin(yaw))
            if rearward_dot >= 0.0:
                raise ValueError('path segment is not behind robot heading')
    return waypoints


def reverse_controller_overrides(controller_dict: dict) -> dict:
    """Append an isolated, collision-enabled reverse Parking controller."""
    if not isinstance(controller_dict, dict):
        raise ValueError('controller parameters must be a mapping')
    controllers = controller_dict.get('controller_plugins')
    if not isinstance(controllers, list):
        raise ValueError('controller_plugins must be a list')
    if controllers.count('Parking') != 1:
        raise ValueError('controller_plugins must contain Parking once')
    if ('ParkingReverse' in controllers
            or 'ParkingReverse' in controller_dict):
        raise ValueError('ParkingReverse controller already exists')

    parking = controller_dict.get('Parking')
    if not isinstance(parking, dict):
        raise ValueError('Parking configuration must be a mapping')
    if (parking.get('plugin') != RPP_PLUGIN
            or parking.get('use_collision_detection') is not True):
        raise ValueError(
            'reverse parking requires collision-enabled '
            'Regulated Pure Pursuit')
    desired_velocity = _finite_number(
        parking.get('desired_linear_vel'), 'Parking.desired_linear_vel')
    if desired_velocity <= 0.0:
        raise ValueError('Parking.desired_linear_vel must be positive')

    configured = deepcopy(controller_dict)
    configured['controller_plugins'].append('ParkingReverse')
    reverse = deepcopy(configured['Parking'])
    reverse.update({
        'use_rotate_to_heading': False,
        'allow_reversing': True,
        'use_collision_detection': True,
        'desired_linear_vel': min(
            desired_velocity, MAX_REVERSE_LINEAR_VELOCITY_MPS),
    })
    configured['ParkingReverse'] = reverse
    return configured


def _clearly_free(metadata: dict, row: int, column: int) -> bool:
    pixel = metadata['pixels'][row * metadata['width'] + column]
    if pixel == 205:
        return False
    occupancy = (
        pixel / 255.0 if metadata['negate'] else (255 - pixel) / 255.0)
    return occupancy < metadata['free_thresh']


def _convex_hull(
        points: Sequence[Sequence[float]],
) -> list[tuple[float, float]]:
    ordered = sorted({
        (float(point[0]), float(point[1])) for point in points
    })
    if len(ordered) < 3:
        raise ValueError('footprint must enclose an area')

    def cross(origin, first, second):
        return ((first[0] - origin[0]) * (second[1] - origin[1])
                - (first[1] - origin[1]) * (second[0] - origin[0]))

    lower = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        raise ValueError('footprint must enclose an area')
    return hull


def _world_footprint(
        pose: Sequence[float], footprint: Sequence[Sequence[float]],
) -> list[tuple[float, float]]:
    x_pose, y_pose, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        (x_pose + x_local * cosine - y_local * sine,
         y_pose + x_local * sine + y_local * cosine)
        for x_local, y_local in footprint
    ]


def _polygon_intersects_cell(
        polygon: Sequence[Sequence[float]], padding: float,
        cell_left: float, cell_bottom: float, resolution: float,
) -> bool:
    cell = (
        (cell_left, cell_bottom),
        (cell_left + resolution, cell_bottom),
        (cell_left + resolution, cell_bottom + resolution),
        (cell_left, cell_bottom + resolution),
    )
    axes = [(1.0, 0.0), (0.0, 1.0)]
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        length = math.hypot(delta_x, delta_y)
        if length > 0.0:
            axes.append((-delta_y / length, delta_x / length))
    for axis_x, axis_y in axes:
        polygon_projection = [
            x_value * axis_x + y_value * axis_y
            for x_value, y_value in polygon
        ]
        cell_projection = [
            x_value * axis_x + y_value * axis_y
            for x_value, y_value in cell
        ]
        if (max(polygon_projection) + padding < min(cell_projection)
                or max(cell_projection) < min(polygon_projection) - padding):
            return False
    return True


def _corridor_blockage(
        source: dict, keepout: dict,
        pieces: Sequence[tuple[Sequence[Sequence[float]], float]],
) -> dict | None:
    resolution = source['resolution']
    origin_x, origin_y, _origin_yaw = source['origin']
    width = source['width']
    height = source['height']
    for polygon, padding in pieces:
        minimum_column = math.floor(
            (min(point[0] for point in polygon) - padding - origin_x)
            / resolution) - 1
        maximum_column = math.floor(
            (max(point[0] for point in polygon) + padding - origin_x)
            / resolution) + 1
        minimum_map_row = math.floor(
            (min(point[1] for point in polygon) - padding - origin_y)
            / resolution) - 1
        maximum_map_row = math.floor(
            (max(point[1] for point in polygon) + padding - origin_y)
            / resolution) + 1
        for map_row in range(minimum_map_row, maximum_map_row + 1):
            cell_bottom = origin_y + map_row * resolution
            for column in range(minimum_column, maximum_column + 1):
                cell_left = origin_x + column * resolution
                if not _polygon_intersects_cell(
                        polygon, padding, cell_left, cell_bottom, resolution):
                    continue
                if not (0 <= map_row < height and 0 <= column < width):
                    return {
                        'reason': 'outside_grid',
                        'column': column,
                        'map_row': map_row,
                    }
                row = height - 1 - map_row
                for grid_name, metadata in (
                        ('map', source), ('keepout', keepout)):
                    if not _clearly_free(metadata, row, column):
                        index = row * width + column
                        return {
                            'reason': f'{grid_name}_not_free',
                            'grid': grid_name,
                            'column': column,
                            'map_row': map_row,
                            'image_row': row,
                            'pixel': metadata['pixels'][index],
                            'cell_bounds': (
                                cell_left, cell_bottom,
                                cell_left + resolution,
                                cell_bottom + resolution),
                        }
    return None


def static_corridor_clear(
        map_yaml: Path, keepout_yaml: Path,
        poses: Sequence[Sequence[float]],
        footprint: Sequence[Sequence[float]],
) -> bool:
    """
    Check a sampled footprint corridor against static map pixels.

    This complements Nav2 path validation by treating unknown pixels as
    blocked.  It checks only the supplied static map and keepout mask, not
    live obstacles or the robot's runtime costmaps.
    """
    source = _map_metadata(Path(map_yaml).resolve())
    keepout = _map_metadata(Path(keepout_yaml).resolve())
    if (not math.isclose(
            source['resolution'], keepout['resolution'], abs_tol=1e-12)
            or source['origin'] != keepout['origin']
            or source['width'] != keepout['width']
            or source['height'] != keepout['height']):
        raise ValueError('map and keepout grids must have identical geometry')
    resolution = source['resolution']
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('map resolution must be positive and finite')
    for metadata in (source, keepout):
        if (metadata['negate'] not in (0, 1)
                or not math.isfinite(metadata['free_thresh'])
                or not 0.0 <= metadata['free_thresh'] <= 1.0):
            raise ValueError('map occupancy metadata is invalid')

    try:
        checked_poses = tuple(_pose(pose, 'pose') for pose in poses)
    except TypeError as error:
        raise ValueError('poses must be a non-empty sequence') from error
    if not checked_poses:
        raise ValueError('poses must be a non-empty sequence')
    try:
        if len(footprint) < 3:
            raise ValueError('footprint must contain at least three points')
    except TypeError as error:
        raise ValueError(
            'footprint must contain at least three points') from error
    footprint_points = []
    for point in footprint:
        try:
            if len(point) != 2:
                raise ValueError('footprint points must contain x and y')
        except TypeError as error:
            raise ValueError(
                'footprint points must contain x and y') from error
        footprint_points.append((
            _finite_number(point[0], 'footprint'),
            _finite_number(point[1], 'footprint')))

    footprint_points = _convex_hull(footprint_points)
    footprint_radius = max(
        math.hypot(x_value, y_value)
        for x_value, y_value in footprint_points)
    world_footprints = [
        _world_footprint(pose, footprint_points) for pose in checked_poses]
    pieces = [(polygon, 0.0) for polygon in world_footprints]
    for index, (current, following) in enumerate(zip(
            checked_poses, checked_poses[1:])):
        yaw_change = abs(_shortest_angle(following[2] - current[2]))
        sagitta = footprint_radius * (1.0 - math.cos(yaw_change / 2.0))
        swept_hull = _convex_hull(
            world_footprints[index] + world_footprints[index + 1])
        pieces.append((swept_hull, sagitta))
    return _corridor_blockage(source, keepout, pieces) is None
