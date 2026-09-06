"""Pure costmap evidence calculations for simulation evaluation."""

import math
from typing import Any


MAX_RECORDED_BLOCKING_CELLS = 16


def cost_rank(value: int) -> int:
    """Return the Nav2 blocking rank used for baseline-delta evidence."""
    return {253: 1, 254: 2}.get(value, 0)


def canonical_grid_key(
        x_m: float, y_m: float, origin_x_m: float, origin_y_m: float,
        resolution_m: float) -> tuple[int, int]:
    """Quantize a map-frame cell center onto the reference costmap grid."""
    return (
        math.floor((x_m - origin_x_m) / resolution_m),
        math.floor((y_m - origin_y_m) / resolution_m),
    )


def transform_point_2d(
        x_m: float, y_m: float, translation_x_m: float,
        translation_y_m: float, yaw_rad: float) -> tuple[float, float]:
    """Apply a planar map-from-source transform to a cell center."""
    return (
        translation_x_m + math.cos(yaw_rad) * x_m
        - math.sin(yaw_rad) * y_m,
        translation_y_m + math.sin(yaw_rad) * x_m
        + math.cos(yaw_rad) * y_m,
    )


def excess_blocking_cells(
        observed_cells: list[dict[str, Any]],
        baseline_ranks: dict[tuple[int, int], int],
        origin_x_m: float, origin_y_m: float, resolution_m: float,
        transform=None) -> list[dict[str, Any]]:
    """Return blocking cells stronger than the fixed-map baseline class."""
    excess = []
    for cell in observed_cells:
        x_m = cell.get('map_center_x_m', cell['center_x_m'])
        y_m = cell.get('map_center_y_m', cell['center_y_m'])
        if transform is not None:
            x_m, y_m = transform(x_m, y_m)
        key = canonical_grid_key(
            x_m, y_m, origin_x_m, origin_y_m, resolution_m)
        baseline_rank = baseline_ranks.get(key)
        observed_rank = cost_rank(cell['cost'])
        if baseline_rank is None or observed_rank > baseline_rank:
            excess.append({
                **cell,
                'map_center_x_m': x_m,
                'map_center_y_m': y_m,
                'canonical_key': list(key),
                'baseline_rank': baseline_rank,
                'observed_rank': observed_rank,
            })
    return excess


def count_obstacle_surface_beams(
        ranges: list[float], angle_min_rad: float,
        angle_increment_rad: float,
        robot_pose_xy_yaw: tuple[float, float, float],
        laser_yaw_in_base_rad: float,
        obstacle: dict[str, float], padding_m: float) -> int:
    """Count finite scan endpoints inside an axis-aligned obstacle AABB."""
    return len(obstacle_surface_ranges(
        ranges, angle_min_rad, angle_increment_rad, robot_pose_xy_yaw,
        laser_yaw_in_base_rad, obstacle, padding_m))


def obstacle_surface_ranges(
        ranges: list[float], angle_min_rad: float,
        angle_increment_rad: float,
        robot_pose_xy_yaw: tuple[float, float, float],
        laser_yaw_in_base_rad: float,
        obstacle: dict[str, float], padding_m: float) -> list[float]:
    """Return finite scan ranges whose endpoints fall in the obstacle AABB."""
    return [item[0] for item in obstacle_surface_endpoints(
        ranges, angle_min_rad, angle_increment_rad, robot_pose_xy_yaw,
        laser_yaw_in_base_rad, obstacle, padding_m)]


def obstacle_surface_endpoints(
        ranges: list[float], angle_min_rad: float,
        angle_increment_rad: float,
        robot_pose_xy_yaw: tuple[float, float, float],
        laser_yaw_in_base_rad: float,
        obstacle: dict[str, float], padding_m: float
        ) -> list[tuple[float, float, float]]:
    """Return range and world endpoint for beams on an obstacle surface."""
    robot_x_m, robot_y_m, robot_yaw_rad = robot_pose_xy_yaw
    half_length_m = obstacle['length_m'] / 2.0 + padding_m
    half_width_m = obstacle['width_m'] / 2.0 + padding_m
    matches = []
    for index, range_m in enumerate(ranges):
        if not math.isfinite(range_m):
            continue
        angle_rad = (
            robot_yaw_rad + laser_yaw_in_base_rad + angle_min_rad
            + index * angle_increment_rad)
        endpoint_x_m = robot_x_m + range_m * math.cos(angle_rad)
        endpoint_y_m = robot_y_m + range_m * math.sin(angle_rad)
        if (abs(endpoint_x_m - obstacle['x_m']) <= half_length_m
                and abs(endpoint_y_m - obstacle['y_m']) <= half_width_m):
            matches.append((range_m, endpoint_x_m, endpoint_y_m))
    return matches


def blocking_probe_from_endpoints(
        endpoints: list[tuple[float, float, float]],
        obstacle_center_y_m: float, core_half_width_m: float,
        robot_circumscribed_radius_m: float) -> dict[str, float] | None:
    """Cover scan endpoints and the costmap's robot-radius blocking band."""
    core = [
        item for item in endpoints
        if abs(item[2] - obstacle_center_y_m) <= core_half_width_m]
    if not core:
        return None
    min_x_m = min(item[1] for item in core)
    max_x_m = max(item[1] for item in core)
    min_y_m = min(item[2] for item in core)
    max_y_m = max(item[2] for item in core)
    diameter_m = 2.0 * robot_circumscribed_radius_m
    return {
        'x_m': (min_x_m + max_x_m) / 2.0,
        'y_m': (min_y_m + max_y_m) / 2.0,
        'length_m': max_x_m - min_x_m + diameter_m,
        'width_m': max_y_m - min_y_m + diameter_m,
    }


def obstacle_angular_window_samples(
        ranges: list[float], angle_min_rad: float,
        angle_increment_rad: float,
        robot_pose_xy_yaw: tuple[float, float, float],
        laser_yaw_in_base_rad: float,
        obstacle: dict[str, float]) -> dict[str, Any]:
    """Return raw beams spanning the obstacle's four projected corners."""
    robot_x_m, robot_y_m, robot_yaw_rad = robot_pose_xy_yaw
    center_angle_rad = math.atan2(
        obstacle['y_m'] - robot_y_m, obstacle['x_m'] - robot_x_m)
    corner_angles_rad = [
        math.atan2(
            obstacle['y_m'] + y_sign * obstacle['width_m'] / 2.0
            - robot_y_m,
            obstacle['x_m'] + x_sign * obstacle['length_m'] / 2.0
            - robot_x_m)
        for x_sign in (-1.0, 1.0)
        for y_sign in (-1.0, 1.0)]

    def circular_difference(left_rad: float, right_rad: float) -> float:
        return math.atan2(
            math.sin(left_rad - right_rad),
            math.cos(left_rad - right_rad))

    half_span_rad = max(
        abs(circular_difference(angle_rad, center_angle_rad))
        for angle_rad in corner_angles_rad)
    values = []
    for index, range_m in enumerate(ranges):
        world_angle_rad = (
            robot_yaw_rad + laser_yaw_in_base_rad + angle_min_rad
            + index * angle_increment_rad)
        if abs(circular_difference(
                world_angle_rad, center_angle_rad)) <= half_span_rad:
            values.append(range_m)
    return {
        'beam_count': len(values),
        'finite_count': sum(math.isfinite(value) for value in values),
        'positive_infinity_count': sum(
            math.isinf(value) and value > 0.0 for value in values),
        'negative_infinity_count': sum(
            math.isinf(value) and value < 0.0 for value in values),
        'nan_count': sum(math.isnan(value) for value in values),
        'ranges': values,
    }


def transform_world_obstacle_to_odom(
        obstacle: dict[str, float],
        world_robot_pose: tuple[float, float, float],
        odom_robot_pose: tuple[float, float, float]) -> dict[str, float]:
    """Express a world-axis-aligned obstacle in the current odom frame."""
    world_x_m, world_y_m, world_yaw_rad = world_robot_pose
    odom_x_m, odom_y_m, odom_yaw_rad = odom_robot_pose
    delta_x_m = obstacle['x_m'] - world_x_m
    delta_y_m = obstacle['y_m'] - world_y_m
    relative_x_m = (
        math.cos(world_yaw_rad) * delta_x_m
        + math.sin(world_yaw_rad) * delta_y_m)
    relative_y_m = (
        -math.sin(world_yaw_rad) * delta_x_m
        + math.cos(world_yaw_rad) * delta_y_m)
    transformed = dict(obstacle)
    transformed['x_m'] = (
        odom_x_m + math.cos(odom_yaw_rad) * relative_x_m
        - math.sin(odom_yaw_rad) * relative_y_m)
    transformed['y_m'] = (
        odom_y_m + math.sin(odom_yaw_rad) * relative_x_m
        + math.cos(odom_yaw_rad) * relative_y_m)
    frame_yaw_rad = odom_yaw_rad - world_yaw_rad
    transformed['length_m'] = (
        abs(math.cos(frame_yaw_rad)) * obstacle['length_m']
        + abs(math.sin(frame_yaw_rad)) * obstacle['width_m'])
    transformed['width_m'] = (
        abs(math.sin(frame_yaw_rad)) * obstacle['length_m']
        + abs(math.cos(frame_yaw_rad)) * obstacle['width_m'])
    return transformed


def roi_cell_samples(
        costmap: Any, obstacle: dict[str, float]) -> dict[str, Any]:
    """Return every sampled cell and bounded indices for an obstacle ROI."""
    metadata = costmap.metadata
    origin = metadata.origin.position
    half_length_m = obstacle['length_m'] / 2.0
    half_width_m = obstacle['width_m'] / 2.0
    min_x = max(0, min(metadata.size_x, math.floor(
        (obstacle['x_m'] - half_length_m - origin.x)
        / metadata.resolution)))
    max_x = max(0, min(metadata.size_x, math.ceil(
        (obstacle['x_m'] + half_length_m - origin.x)
        / metadata.resolution)))
    min_y = max(0, min(metadata.size_y, math.floor(
        (obstacle['y_m'] - half_width_m - origin.y)
        / metadata.resolution)))
    max_y = max(0, min(metadata.size_y, math.ceil(
        (obstacle['y_m'] + half_width_m - origin.y)
        / metadata.resolution)))
    cells = [
        {
            'index_x': x,
            'index_y': y,
            'center_x_m': origin.x + (x + 0.5) * metadata.resolution,
            'center_y_m': origin.y + (y + 0.5) * metadata.resolution,
            'cost': costmap.data[y * metadata.size_x + x],
        }
        for y in range(min_y, max_y)
        for x in range(min_x, max_x)]
    return {'cells': cells, 'index_bounds': [min_x, max_x, min_y, max_y]}


def transformed_roi_cell_samples(
        costmap: Any, obstacle: dict[str, float], transform) -> dict[str, Any]:
    """Select source-grid cells whose transformed centers enter a map ROI."""
    metadata = costmap.metadata
    origin = metadata.origin.position
    half_length_m = obstacle['length_m'] / 2.0
    half_width_m = obstacle['width_m'] / 2.0
    cells = []
    indices_x = []
    indices_y = []
    for y in range(metadata.size_y):
        for x in range(metadata.size_x):
            source_x_m = origin.x + (x + 0.5) * metadata.resolution
            source_y_m = origin.y + (y + 0.5) * metadata.resolution
            map_x_m, map_y_m = transform(source_x_m, source_y_m)
            if (abs(map_x_m - obstacle['x_m']) <= half_length_m
                    and abs(map_y_m - obstacle['y_m']) <= half_width_m):
                cells.append({
                    'index_x': x, 'index_y': y,
                    'center_x_m': source_x_m, 'center_y_m': source_y_m,
                    'map_center_x_m': map_x_m, 'map_center_y_m': map_y_m,
                    'cost': costmap.data[y * metadata.size_x + x],
                })
                indices_x.append(x)
                indices_y.append(y)
    bounds = ([min(indices_x), max(indices_x) + 1,
               min(indices_y), max(indices_y) + 1]
              if cells else [0, 0, 0, 0])
    return {'cells': cells, 'index_bounds': bounds}


def roi_statistics_from_samples(
        costmap: Any, samples: dict[str, Any]) -> dict[str, Any]:
    """Summarize already selected ROI cells without changing their frame."""
    metadata = costmap.metadata
    origin = metadata.origin.position
    cells = samples['cells']
    values = [item['cost'] for item in cells]
    blocking_cells = [
        item for item in cells if item['cost'] in {253, 254}
    ][:MAX_RECORDED_BLOCKING_CELLS]
    lethal_count = sum(value == 254 for value in values)
    inscribed_count = sum(value == 253 for value in values)
    unknown_count = sum(value == 255 for value in values)
    return {
        'blocking': lethal_count + inscribed_count > 0,
        'blocking_count': lethal_count + inscribed_count,
        'lethal': lethal_count > 0,
        'lethal_count': lethal_count,
        'inscribed_count': inscribed_count,
        'unknown_count': unknown_count,
        'blocking_cells': blocking_cells,
        'sampled_cell_count': len(values),
        'max_cost': max(values, default=-1),
        'index_bounds': samples['index_bounds'],
        'metadata': {
            'origin_x_m': origin.x,
            'origin_y_m': origin.y,
            'resolution_m': metadata.resolution,
            'size_x': metadata.size_x,
            'size_y': metadata.size_y,
        },
    }


def roi_statistics(costmap: Any,
                   obstacle: dict[str, float]) -> dict[str, Any]:
    """Return bounded obstacle-ROI indices and costs for a Nav2 costmap."""
    samples = roi_cell_samples(costmap, obstacle)
    return roi_statistics_from_samples(costmap, samples)
