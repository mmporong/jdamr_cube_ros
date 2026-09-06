#!/usr/bin/env python3
"""Pure evaluation contract for deterministic fixed-obstacle Nav2 runs."""

from __future__ import annotations

import math
from typing import Iterable, Mapping, Sequence


SEEDS = (11, 23, 42, 67, 89)
START_POSE = {'x_m': -8.0, 'y_m': 0.0, 'yaw_rad': 0.0}
GOAL_POSE = {'x_m': 6.0, 'y_m': 0.0, 'yaw_rad': 0.0}
SCENARIOS = {
    'baseline': {'obstacle': None, 'expected_terminal': 'succeeded'},
    'detour': {
        'obstacle': {'x_m': -1.0, 'y_m': 0.0,
                     'length_m': 0.50, 'width_m': 0.40},
        'expected_terminal': 'succeeded',
    },
    'event_driven_removal': {
        'obstacle': {'x_m': -1.0, 'y_m': 0.0,
                     'length_m': 0.50, 'width_m': 0.40},
        'expected_terminal': 'succeeded',
    },
    'full_block': {
        'obstacle': {'x_m': -1.0, 'y_m': 0.0,
                     'length_m': 0.50, 'width_m': 2.30},
        'expected_terminal': 'blocked',
    },
    'goal_occupied': {
        'obstacle': {'x_m': 6.0, 'y_m': 0.0,
                     'length_m': 1.70, 'width_m': 1.70},
        'expected_terminal': 'blocked',
    },
}


def derive_contract(params: Mapping) -> dict:
    """Derive spatial and timing limits from the active production values."""
    controller = params['controller_server']['ros__parameters']
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_map = params['global_costmap']['global_costmap']['ros__parameters']
    navigator = params['bt_navigator']['ros__parameters']
    monitor = params['collision_monitor']['ros__parameters']

    footprint = _parse_footprint(local['footprint'])
    radius_m = max(math.hypot(x_m, y_m) for x_m, y_m in footprint)
    inflation_radius_m = max(
        local['inflation_layer']['inflation_radius'],
        global_map['inflation_layer']['inflation_radius'])
    resolution_m = max(local['resolution'], global_map['resolution'])
    global_period_s = 1.0 / global_map['update_frequency']
    global_publish_period_s = 1.0 / global_map['publish_frequency']
    bt_period_s = navigator['bt_loop_duration'] / 1000.0
    server_timeout_s = navigator['default_server_timeout'] / 1000.0
    local_period_s = 1.0 / local['update_frequency']
    local_publish_period_s = 1.0 / local['publish_frequency']
    clear_observation_count = 2
    slowest_costmap_publish_period_s = max(
        global_publish_period_s, local_publish_period_s)
    passive_clear_window_s = (
        (clear_observation_count + 1)
        * slowest_costmap_publish_period_s)
    clear_service_timeout_s = (
        clear_observation_count * global_period_s + server_timeout_s)
    costmap_tf_timeout_s = (
        clear_observation_count * local_publish_period_s)

    removal_passage = derive_removal_passage_contract(
        START_POSE, GOAL_POSE,
        SCENARIOS['event_driven_removal']['obstacle'], resolution_m,
        radius_m)
    return {
        'nav2_process_topology': 'component_container_isolated',
        'robot_circumscribed_radius_m': radius_m,
        'path_clearance_m': (
            max(radius_m, inflation_radius_m)
            + resolution_m / math.sqrt(2.0)),
        'global_update_period_s': global_period_s,
        'global_publish_period_s': global_publish_period_s,
        'costmap_resolution_m': resolution_m,
        'final_zero_hold_s': monitor['stop_pub_timeout'],
        'blocked_terminal_limit_s': (
            server_timeout_s + controller['costmap_update_timeout']
            + global_period_s + 2.0 * bt_period_s),
        'controller_period_s': 1.0 / controller['controller_frequency'],
        'local_update_period_s': local_period_s,
        'local_publish_period_s': local_publish_period_s,
        'clear_observation_count': clear_observation_count,
        'passive_clear_window_s': passive_clear_window_s,
        'clear_service_timeout_s': clear_service_timeout_s,
        'post_recovery_clear_window_s': passive_clear_window_s,
        'costmap_tf_timeout_s': costmap_tf_timeout_s,
        'pending_local_costmap_limit': (
            math.ceil(costmap_tf_timeout_s / local_publish_period_s) + 1),
        'clear_recovery_scope': (
            'evaluation-only bounded Nav2 ClearEntireCostmap recovery; '
            'not production automatic recovery'),
        'planner_update_period_s': 1.0,
        'obstacle_mark_max_range_m': min(
            local['obstacle_layer']['scan']['obstacle_max_range'],
            global_map['obstacle_layer']['scan']['obstacle_max_range']),
        'removal_passage': removal_passage,
    }


def derive_removal_passage_contract(
        start: Mapping, goal: Mapping, obstacle: Mapping,
        resolution_m: float, robot_radius_m: float) -> dict:
    """Derive the strict far-side passage threshold for a straight route."""
    delta_x_m = float(goal['x_m']) - float(start['x_m'])
    if delta_x_m == 0.0:
        raise ValueError('passage route must have a nonzero x direction')
    if not (float(start['y_m']) == float(obstacle['y_m'])
            == float(goal['y_m'])):
        raise ValueError('passage route and obstacle must be y-aligned')
    direction_sign = 1 if delta_x_m > 0.0 else -1
    if not (direction_sign * float(start['x_m'])
            < direction_sign * float(obstacle['x_m'])
            < direction_sign * float(goal['x_m'])):
        raise ValueError('obstacle must lie strictly between start and goal')
    far_edge_x_m = (
        float(obstacle['x_m'])
        + direction_sign * (
            float(obstacle['length_m']) / 2.0
            + resolution_m / math.sqrt(2.0)))
    return {
        'direction_sign': direction_sign,
        'far_edge_x_m': far_edge_x_m,
        'robot_radius_m': robot_radius_m,
        'strict_robot_center_threshold_x_m': (
            far_edge_x_m + direction_sign * robot_radius_m),
    }


def removal_passage_crossed(robot_x_m: float, contract: Mapping) -> bool:
    """Return true only beyond the strict far-edge footprint boundary."""
    direction_sign = contract['direction_sign']
    return (direction_sign * (robot_x_m - contract['far_edge_x_m'])
            > contract['robot_radius_m'])


def _parse_footprint(value: str) -> list[tuple[float, float]]:
    import ast

    points = ast.literal_eval(value)
    return [(float(point[0]), float(point[1])) for point in points]


def scenario_matrix() -> list[dict]:
    """Return the complete ordered 5-by-5 experiment matrix."""
    return [
        {'scenario': scenario, 'seed': seed,
         'run_id': f'{scenario}__seed_{seed}'}
        for scenario in SCENARIOS
        for seed in SEEDS
    ]


def path_clears_obstacle(
        points_xy_m: Iterable[Sequence[float]], obstacle: Mapping,
        clearance_m: float) -> bool:
    """Return true when every path point clears the inflated obstacle AABB."""
    half_length_m = obstacle['length_m'] / 2.0 + clearance_m
    half_width_m = obstacle['width_m'] / 2.0 + clearance_m
    points = [(float(point[0]), float(point[1]))
              for point in points_xy_m]
    if not points or any(not all(math.isfinite(value) for value in point)
                         for point in points):
        return False

    def segment_intersects(left, right) -> bool:
        min_x = obstacle['x_m'] - half_length_m
        max_x = obstacle['x_m'] + half_length_m
        min_y = obstacle['y_m'] - half_width_m
        max_y = obstacle['y_m'] + half_width_m
        delta_x = right[0] - left[0]
        delta_y = right[1] - left[1]
        lower = 0.0
        upper = 1.0
        for start, delta, minimum, maximum in (
                (left[0], delta_x, min_x, max_x),
                (left[1], delta_y, min_y, max_y)):
            if delta == 0.0:
                if minimum <= start <= maximum:
                    continue
                return False
            first = (minimum - start) / delta
            second = (maximum - start) / delta
            lower = max(lower, min(first, second))
            upper = min(upper, max(first, second))
            if lower > upper:
                return False
        return True

    if len(points) == 1:
        return not segment_intersects(points[0], points[0])
    return not any(segment_intersects(left, right)
                   for left, right in zip(points, points[1:]))


def validate_removal_sequence(
        events: Sequence[str], clear_mode: str = 'passive') -> list[str]:
    """Validate the causal order required by the removal scenario."""
    recovery = (
        'passive_clear_window_expired',
        'global_clear_requested', 'local_clear_requested',
        'global_clear_responded', 'local_clear_responded',
        'post_recovery_global_clear_1', 'post_recovery_global_clear_2',
        'post_recovery_local_clear_1', 'post_recovery_local_clear_2',
    ) if clear_mode == 'bounded_recovery' else ()
    required = (
        'global_blocking', 'local_blocking', 'post_mark_plan',
        'deactivate_ack', 'new_scan', *recovery,
        'global_cleared', 'local_cleared', 'post_clear_new_plan',
        'succeeded')
    errors = []
    positions = {}
    for event in required:
        try:
            positions[event] = events.index(event)
        except ValueError:
            errors.append(f'missing_event:{event}')
    if not errors:
        phases = [
            ('global_blocking', 'local_blocking'),
            ('post_mark_plan',),
            ('deactivate_ack',),
            ('new_scan',),
        ]
        if clear_mode == 'bounded_recovery':
            phases.extend([
                ('passive_clear_window_expired',),
                ('global_clear_requested', 'local_clear_requested'),
                ('global_clear_responded', 'local_clear_responded'),
            ])
        phases.extend([
            ('global_cleared', 'local_cleared'),
            ('post_clear_new_plan',),
            ('succeeded',),
        ])
        for before, after in zip(phases, phases[1:]):
            if max(positions[event] for event in before) >= min(
                    positions[event] for event in after):
                errors.append(
                    f'event_phase_order:{"+".join(before)}>='
                    f'{"+".join(after)}')
        if clear_mode == 'bounded_recovery':
            for source in ('global', 'local'):
                source_events = (
                    f'{source}_clear_responded',
                    f'post_recovery_{source}_clear_1',
                    f'post_recovery_{source}_clear_2',
                )
                if not (positions[source_events[0]]
                        < positions[source_events[1]]
                        < positions[source_events[2]]
                        < positions[f'{source}_cleared']):
                    errors.append(
                        f'event_source_order:{">=".join(source_events)}')
    return errors


def classify_terminal(
        scenario: str, action_terminal: str, elapsed_s: float,
        runner_cancelled: bool,
        blocked_terminal_limit_s: float = 5.02) -> str:
    """Classify a run without turning a harness cancellation into success."""
    if runner_cancelled:
        return 'INVALID'
    expected = SCENARIOS[scenario]['expected_terminal']
    if expected == 'blocked':
        if action_terminal in {'aborted', 'rejected'}:
            if 0.0 <= elapsed_s <= blocked_terminal_limit_s:
                return 'BLOCKED'
            return 'FAIL'
        return 'FAIL'
    return 'PASS' if action_terminal == 'succeeded' else 'FAIL'
