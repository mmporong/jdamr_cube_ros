#!/usr/bin/env python3
"""Pure G004 Collision Monitor evaluation contract derivation."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Iterable


SCENARIOS = (
    'clear_baseline',
    'sudden_obstacle_stop_resume',
    'scan_timeout_stop_resume',
)
SEEDS = (11, 23, 42, 67, 89)
SOURCE_TIMEOUT_CANDIDATES_S = (0.25, 0.30, 0.40)
SYNTHETIC_VALID_SCAN_GAPS_S = (
    (0.10, 0.20, 0.26),
    (0.10, 0.22, 0.28),
    (0.10, 0.24, 0.29),
)
LIDAR_RATE_HZ = 10.0
FOOTPRINT_FRONT_M = 0.23
FOOTPRINT_REAR_M = -0.23
FOOTPRINT_HALF_WIDTH_M = 0.20
MAX_FORWARD_SPEED_MPS = 0.18
MAX_SCAN_GAP_S = 0.107525
MAX_DECEL_MPS2 = 0.50
COSTMAP_RESOLUTION_M = 0.05
SMOOTHING_FREQUENCY_HZ = 20.0
PHYSICS_STEP_S = 0.001
FINAL_ZERO_HOLD_S = 1.95
GOAL_POSITION_TOLERANCE_M = 0.15
TOTAL_RETENTION_LIMIT_BYTES = 512 * 1024 * 1024
REPRESENTATIVE_BAG_LIMIT_BYTES = 128 * 1024 * 1024
NAV2_JAZZY_REFERENCE_COMMIT = 'f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501'
REPRESENTATIVE_TOPICS = (
    '/tf', '/tf_static', '/ground_truth_pose', '/sim_raw/scan',
    '/collision_monitor_scan', '/collision_monitor_state', '/cmd_vel_nav',
    '/cmd_vel_smoothed', '/cmd_vel', '/odom',
    '/navigate_to_pose/_action/status',
    '/world/slam_corridor/model/g003_preloaded_front_observation_probe/'
    'link/body/sensor/contact_sensor/contact', '/clock')


def sha256_file(path: Path) -> str:
    """Return one file SHA-256 digest."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def false_stop_count(
        source_timeout_s: float,
        valid_scan_gap_traces_s: Iterable[Iterable[float]]) -> int:
    """Count valid synthetic traces containing a source-timeout gap."""
    return sum(
        any(gap_s > source_timeout_s for gap_s in trace_s)
        for trace_s in valid_scan_gap_traces_s)


def select_source_timeout_s(
        candidates_s: Iterable[float] = SOURCE_TIMEOUT_CANDIDATES_S,
        traces_s: Iterable[Iterable[float]] = SYNTHETIC_VALID_SCAN_GAPS_S,
) -> float:
    """Select the shortest candidate producing no false synthetic STOP."""
    eligible_s = [
        candidate_s for candidate_s in candidates_s
        if false_stop_count(candidate_s, traces_s) == 0]
    if not eligible_s:
        raise ValueError('no source_timeout candidate passes synthetic traces')
    return min(eligible_s)


def derive_stop_deadline_s(
        source_timeout_s: float,
        smoothing_frequency_hz: float,
        physics_step_s: float,
) -> float:
    """Derive timeout-stop deadline from configured processing periods."""
    if min(source_timeout_s, smoothing_frequency_hz, physics_step_s) <= 0.0:
        raise ValueError('deadline inputs must be positive')
    smoother_period_s = 1.0 / smoothing_frequency_hz
    return source_timeout_s + smoother_period_s + physics_step_s


def derive_stop_zone(
        smoothing_frequency_hz: float = SMOOTHING_FREQUENCY_HZ,
        max_scan_gap_s: float = MAX_SCAN_GAP_S,
        footprint_front_m: float = FOOTPRINT_FRONT_M,
        footprint_rear_m: float = FOOTPRINT_REAR_M,
        footprint_half_width_m: float = FOOTPRINT_HALF_WIDTH_M,
        max_forward_speed_mps: float = MAX_FORWARD_SPEED_MPS,
        max_decel_mps2: float = MAX_DECEL_MPS2,
        costmap_resolution_m: float = COSTMAP_RESOLUTION_M) -> dict:
    """Derive the stop polygon from footprint and bounded stopping motion."""
    if smoothing_frequency_hz <= 0.0:
        raise ValueError('smoothing_frequency_hz must be positive')
    cell_half_diagonal_m = costmap_resolution_m / math.sqrt(2.0)
    reaction_time_s = max_scan_gap_s + 1.0 / smoothing_frequency_hz
    reaction_distance_m = max_forward_speed_mps * reaction_time_s
    braking_distance_m = max_forward_speed_mps ** 2 / (2.0 * max_decel_mps2)
    return {
        'front_m': (
            footprint_front_m + cell_half_diagonal_m
            + reaction_distance_m + braking_distance_m),
        'rear_m': footprint_rear_m - cell_half_diagonal_m,
        'half_width_m': footprint_half_width_m + cell_half_diagonal_m,
        'inputs': {
            'footprint_front_m': footprint_front_m,
            'footprint_rear_m': footprint_rear_m,
            'footprint_half_width_m': footprint_half_width_m,
            'max_forward_speed_mps': max_forward_speed_mps,
            'max_scan_gap_s': max_scan_gap_s,
            'max_decel_mps2': max_decel_mps2,
            'costmap_resolution_m': costmap_resolution_m,
            'smoothing_frequency_hz': smoothing_frequency_hz,
        },
        'derived': {
            'cell_half_diagonal_m': cell_half_diagonal_m,
            'reaction_time_s': reaction_time_s,
            'reaction_distance_m': reaction_distance_m,
            'braking_distance_m': braking_distance_m,
        },
    }


def scenario_matrix() -> list[dict]:
    """Return the exact 15-run G004 matrix."""
    return [
        {
            'scenario': scenario,
            'seed': seed,
            'run_id': f'{scenario}__seed_{seed}',
        }
        for scenario in SCENARIOS for seed in SEEDS
    ]


def derive_contract(
        smoothing_frequency_hz: float,
        physics_step_s: float,
        installed_versions: dict | None = None,
        max_scan_gap_s: float = MAX_SCAN_GAP_S,
        footprint_front_m: float = FOOTPRINT_FRONT_M,
        footprint_rear_m: float = FOOTPRINT_REAR_M,
        footprint_half_width_m: float = FOOTPRINT_HALF_WIDTH_M,
        max_forward_speed_mps: float = MAX_FORWARD_SPEED_MPS,
        max_decel_mps2: float = MAX_DECEL_MPS2,
        costmap_resolution_m: float = COSTMAP_RESOLUTION_M,
        goal_position_tolerance_m: float = GOAL_POSITION_TOLERANCE_M,
        estimate_stamp_tolerance_s: float = 1.0,
        goal_checker_name: str = 'general_goal_checker',
        goal_checker_plugin: str = 'nav2_controller::SimpleGoalChecker',
        goal_checker_stateful: bool = True,
) -> dict:
    """Build the machine-readable evaluation contract."""
    source_timeout_s = select_source_timeout_s()
    timeout_stop_deadline_s = derive_stop_deadline_s(
        source_timeout_s, smoothing_frequency_hz, physics_step_s)
    nominal_scan_reaction_budget_s = (
        max_scan_gap_s + 1.0 / smoothing_frequency_hz + physics_step_s)
    stop_zone = derive_stop_zone(
        smoothing_frequency_hz, max_scan_gap_s, footprint_front_m,
        footprint_rear_m, footprint_half_width_m, max_forward_speed_mps,
        max_decel_mps2, costmap_resolution_m)
    obstacle_half_length_m = 0.25
    obstacle_half_width_m = 0.20
    activation_surface_x_m = (
        stop_zone['front_m'] + costmap_resolution_m)
    return {
        'schema_version': 1,
        'claim_scope': (
            'Gazebo simulation with idealized 2D LiDAR; no human-detection, '
            'safety-certification, or production-qualification claim.'),
        'start_pose': {'x_m': -8.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        'goal_pose': {'x_m': 6.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        'goal_verification': {
            'position_tolerance_m': goal_position_tolerance_m,
            'estimate_stamp_tolerance_s': estimate_stamp_tolerance_s,
            'orientation_verified': False,
            'selected_goal_checker': goal_checker_name,
            'selected_goal_checker_plugin': goal_checker_plugin,
            'selected_goal_checker_stateful': goal_checker_stateful,
            'scope': (
                'Completion is one uncancelled goal with the same UUID and '
                'one final SUCCEEDED result under the source-selected '
                'stateful SimpleGoalChecker. A first observed goal-active '
                'map-to-base x/y tolerance witness is optional; when present '
                'its canonical TF inputs are verified. Terminal pose, ground '
                'truth, and localization error are diagnostics; yaw is not '
                'claimed'),
        },
        'collision_monitor_state_interface': {
            'package_version': (installed_versions or {}).get(
                'ros-jazzy-nav2-msgs', 'unrecorded'),
            'actions': {
                'DO_NOTHING': 0, 'STOP': 1, 'SLOWDOWN': 2,
                'APPROACH': 3, 'LIMIT': 4,
            },
            'fields': ['action_type', 'polygon_name'],
        },
        'collision_monitor_package_version': (installed_versions or {}).get(
            'ros-jazzy-nav2-collision-monitor', 'unrecorded'),
        'topic_chain': [
            'controller_server:/cmd_vel_nav',
            'velocity_smoother:/cmd_vel_smoothed',
            'collision_monitor:/cmd_vel',
        ],
        'scan_gate': {
            'input_topic': '/sim_raw/scan',
            'navigation_output_topic': '/scan',
            'collision_monitor_output_topic': '/collision_monitor_scan',
            'navigation_output_freezable': False,
            'collision_monitor_output_freezable': True,
        },
        'scan_to_base_transform': {
            'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': math.pi,
            'source_joint': 'laser_joint', 'source_frame': 'laser_link'},
        'lidar_rate_hz': LIDAR_RATE_HZ,
        'stop_zone': stop_zone,
        'sudden_obstacle': {
            'length_m': 2.0 * obstacle_half_length_m,
            'width_m': 2.0 * obstacle_half_width_m,
            'activation_surface_x_m': activation_surface_x_m,
            'activation_center_offset_x_m': (
                activation_surface_x_m + obstacle_half_length_m),
            'lead_margin_m': costmap_resolution_m,
            'removal_pose_m': [0.0, 40.0, 0.5],
            'pose_json_default_semantic': (
                'ProtoJSON omits fields holding implicit defaults; omitted '
                'numeric axes decode as 0.0. '
                'https://protobuf.dev/programming-guides/json/'),
        },
        'source_timeout_candidates_s': list(SOURCE_TIMEOUT_CANDIDATES_S),
        'synthetic_valid_scan_gap_traces_s': [
            list(trace_s) for trace_s in SYNTHETIC_VALID_SCAN_GAPS_S],
        'selected_source_timeout_s': source_timeout_s,
        'timeout_clock_provenance': {
            'repository': 'https://github.com/ros-navigation/navigation2',
            'branch': 'jazzy',
            'commit': NAV2_JAZZY_REFERENCE_COMMIT,
            'scan_source': {
                'url': (
                    'https://raw.githubusercontent.com/ros-navigation/'
                    f'navigation2/{NAV2_JAZZY_REFERENCE_COMMIT}/'
                    'nav2_collision_monitor/src/scan.cpp'),
                'sha256': (
                    'd3fb3132ab1b949473e78aff82d6c188becc19decd3d3ab9e'
                    '7563364d9ed8fae'),
                'semantic': (
                    'Scan::getData passes LaserScan.header.stamp to '
                    'Source::sourceValid.'),
            },
            'source_base': {
                'url': (
                    'https://raw.githubusercontent.com/ros-navigation/'
                    f'navigation2/{NAV2_JAZZY_REFERENCE_COMMIT}/'
                    'nav2_collision_monitor/src/source.cpp'),
                'sha256': (
                    '582f11a2e50885cc777825aeaa0bf9222af88de1b9ec1e8a8'
                    'e4368559aa5382f'),
                'semantic': (
                    'Source::sourceValid compares current ROS time minus '
                    'source stamp against source_timeout.'),
            },
        },
        'smoothing_frequency_hz': smoothing_frequency_hz,
        'physics_step_s': physics_step_s,
        'stop_deadline_s': timeout_stop_deadline_s,
        'sudden_stop_deadline_s': nominal_scan_reaction_budget_s,
        'timeout_stop_deadline_s': timeout_stop_deadline_s,
        'nominal_scan_reaction_budget_s': nominal_scan_reaction_budget_s,
        'sudden_latency_semantic': (
            'sim_scan_gate obstacle-bearing Collision Monitor input publish '
            'to first post-trigger zero /cmd_vel receive in the same system '
            'monotonic clock; sensor-detected simulated reaction only, not '
            'physical appearance-to-stop'),
        'maximum_stop_distance_m': (
            max_forward_speed_mps * timeout_stop_deadline_s
            + max_forward_speed_mps ** 2 / (2.0 * max_decel_mps2)),
        'final_zero_hold_s': FINAL_ZERO_HOLD_S,
        'matrix': scenario_matrix(),
        'retention': {
            'full_matrix_bags_enabled': False,
            'representative_opt_in_after_pass': True,
            'representative_scenarios': list(SCENARIOS),
            'representative_topics': list(REPRESENTATIVE_TOPICS),
            'total_limit_bytes': TOTAL_RETENTION_LIMIT_BYTES,
            'individual_bag_limit_bytes': REPRESENTATIVE_BAG_LIMIT_BYTES,
        },
    }


def write_contract(path: Path, contract: dict) -> None:
    """Write one deterministic contract JSON."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(contract, indent=2, sort_keys=True) + '\n')
