"""Define a direct-scan probe for a stowed mobile-manipulator envelope."""

import json
import math
from pathlib import Path

import yaml

from prepare_sim_collision_monitor_run import (  # noqa: I100,I201
    _production_motion_inputs, installed_nav2_versions,
    PRODUCTION_FILES, SCAN_PROFILE,
)
from robot_stow_envelope import collision_envelope  # noqa: I201
from sim_collision_monitor_contract import (  # noqa: I201
    derive_contract, sha256_file,
)


ROOT = Path(__file__).resolve().parents[2]
MESH_ROOT = ROOT / 'jdamr_cube_description/meshes'


def _ceil_to_resolution(value: float, resolution: float) -> float:
    return math.ceil((value - 1e-12) / resolution) * resolution


def _zone_points(zone: dict) -> str:
    return json.dumps([
        [zone['front_m'], zone['half_width_m']],
        [zone['front_m'], -zone['half_width_m']],
        [zone['rear_m'], -zone['half_width_m']],
        [zone['rear_m'], zone['half_width_m']],
    ])


def _rectangle_zone(monitor: dict, name: str, action_type: str) -> dict:
    zone = monitor[name]
    if zone['action_type'] != action_type or zone['enabled'] is not True:
        raise ValueError(f'probe requires the production {name}')
    points = json.loads(zone['points'])
    if (len(points) != 4 or any(len(point) != 2 for point in points)
            or not all(math.isfinite(value)
                       for point in points for value in point)):
        raise ValueError(f'probe requires a finite rectangular {name}')
    front_m = max(point[0] for point in points)
    rear_m = min(point[0] for point in points)
    half_width_m = max(abs(point[1]) for point in points)
    if {tuple(point) for point in points} != {
            (front_m, half_width_m), (front_m, -half_width_m),
            (rear_m, half_width_m), (rear_m, -half_width_m)}:
        raise ValueError(f'probe requires a symmetric rectangular {name}')
    return {
        'front_m': front_m, 'rear_m': rear_m,
        'half_width_m': half_width_m,
        'min_points': int(zone['min_points']),
        'action_type': action_type,
    }


def build_contract(params_path: Path, urdf_path: Path | None = None) -> dict:
    """Derive candidate zones from the fixed travel-pose collision envelope."""
    urdf_path = urdf_path or PRODUCTION_FILES['production_urdf']
    params = yaml.safe_load(params_path.read_text(encoding='utf-8'))
    motion = _production_motion_inputs(
        params, urdf_path)
    arm_envelope = collision_envelope(
        urdf_path, MESH_ROOT, root_frame='base_footprint',
        link_prefix='arm_')
    profile = json.loads(SCAN_PROFILE.read_text(encoding='utf-8'))
    gap_s = profile['timing']['scan']['header_interval_s']['max']
    smoother = params['velocity_smoother']['ros__parameters']
    contract = derive_contract(
        float(smoother['smoothing_frequency']), 0.001,
        installed_nav2_versions(), gap_s, **motion)
    monitor = params['collision_monitor']['ros__parameters']
    production_stop_zone = _rectangle_zone(monitor, 'StopZone', 'stop')
    production_slowdown_zone = _rectangle_zone(
        monitor, 'SlowdownZone', 'slowdown')
    if production_slowdown_zone['front_m'] <= production_stop_zone['front_m']:
        raise ValueError('SlowdownZone must extend beyond StopZone')
    slowdown_ratio = float(monitor['SlowdownZone']['slowdown_ratio'])
    reaction_time_s = gap_s + 1.0 / float(smoother['smoothing_frequency'])
    speed_mps = motion['max_forward_speed_mps']
    reaction_distance_m = speed_mps * reaction_time_s
    braking_distance_m = speed_mps ** 2 / (
        2.0 * motion['max_decel_mps2'])
    bounded_stop_distance_m = reaction_distance_m + braking_distance_m
    cell_half_diagonal_m = motion['costmap_resolution_m'] / math.sqrt(2.0)
    arm_front_m = arm_envelope['bounds_m']['front_m']
    protected_envelope = {
        'front_m': max(arm_front_m, motion['footprint_front_m']),
        'rear_m': min(
            arm_envelope['bounds_m']['rear_m'], motion['footprint_rear_m']),
        'half_width_m': max(
            arm_envelope['bounds_m']['half_width_m'],
            motion['footprint_half_width_m']),
        'composition': 'union_AABB_of_base_footprint_and_stowed_arm',
    }
    required_front_m = (
        protected_envelope['front_m'] + bounded_stop_distance_m
        + cell_half_diagonal_m)
    stop_zone = {
        **production_stop_zone,
        'front_m': _ceil_to_resolution(
            max(production_stop_zone['front_m'], required_front_m),
            motion['costmap_resolution_m']),
    }
    minimum_slowdown_lead_m = 2.0 * motion['costmap_resolution_m']
    slowdown_zone = {
        **production_slowdown_zone,
        'front_m': _ceil_to_resolution(max(
            production_slowdown_zone['front_m'],
            stop_zone['front_m'] + minimum_slowdown_lead_m,
        ), motion['costmap_resolution_m']),
    }
    available_stop_distance_m = (
        stop_zone['front_m'] - protected_envelope['front_m'])
    if bounded_stop_distance_m >= available_stop_distance_m:
        raise ValueError(
            'candidate stop zone lacks stowed-arm stopping margin')
    surface_m = (stop_zone['front_m'] + slowdown_zone['front_m']) / 2.0
    contract['stop_zone'] = {
        **stop_zone,
        'inputs': contract['stop_zone']['inputs'],
        'points': _zone_points(stop_zone),
        'source': 'derived_stowed_mobile_manipulator_StopZone_candidate',
    }
    contract['slowdown_zone'] = {
        **slowdown_zone,
        'slowdown_ratio': slowdown_ratio,
        'points': _zone_points(slowdown_zone),
        'source': 'derived_stowed_mobile_manipulator_SlowdownZone_candidate',
    }
    contract['production_zones'] = {
        'stop_zone': {
            **production_stop_zone,
            'points': monitor['StopZone']['points'],
        },
        'slowdown_zone': {
            **production_slowdown_zone,
            'slowdown_ratio': slowdown_ratio,
            'points': monitor['SlowdownZone']['points'],
        },
    }
    contract['non_contact_margin'] = {
        'protected_front_m': protected_envelope['front_m'],
        'maximum_approach_speed_mps': speed_mps,
        'reaction_time_s': reaction_time_s,
        'reaction_distance_m': reaction_distance_m,
        'braking_distance_m': braking_distance_m,
        'bounded_stop_distance_m': bounded_stop_distance_m,
        'cell_half_diagonal_m': cell_half_diagonal_m,
        'available_stop_distance_m': available_stop_distance_m,
        'remaining_margin_m': (
            available_stop_distance_m - bounded_stop_distance_m),
        'scope': (
            'configuration-derived full-speed bound for a fixed stow pose; '
            'not measured physical stopping distance or safety certification'),
    }
    contract['travel_pose_envelope'] = arm_envelope
    contract['travel_pose_envelope']['production_stop_zone_deficit_m'] = max(
        0.0, arm_front_m - production_stop_zone['front_m'])
    contract['travel_pose_envelope']['valid_only_at_joint_positions'] = True
    contract['protected_envelope'] = protected_envelope
    contract['travel_pose_gate'] = {
        'required_before_navigation': True,
        'position_tolerance_rad': 0.03,
        'source_topic': '/joint_states',
        'failure_action': 'do_not_send_navigation_goal',
    }
    contract['sudden_obstacle'].update({
        'activation_surface_x_m': surface_m,
        'activation_center_offset_x_m': (
            surface_m + contract['sudden_obstacle']['length_m'] / 2.0),
        'placement': (
            'synthetic_midpoint_between_candidate_StopZone_and_SlowdownZone'),
        'trigger_min_speed_mps': motion['max_forward_speed_mps'] * 0.9,
        'lead_margin_m': None,
    })
    contract['integration_kind'] = 'onboard_candidate_direct_scan'
    contract['claim_scope'] = (
        'SIM_INTEGRATION_ONLY: synthetic intrusion into a candidate field '
        'derived for the configured arm stow pose; not human tracking, '
        'real-world stopping distance, latency qualification, dynamic arm '
        'envelope coverage, safety certification, or a G004 full matrix')
    contract['scan_gate'] = {'used': False, 'monitor_input_topic': '/scan'}
    contract['selected_source_timeout_s'] = monitor['source_timeout']
    contract['production_inputs'] = {
        'params_path': str(params_path.resolve()),
        'params_sha256': sha256_file(params_path),
        'collision_monitor': monitor,
    }
    # Old G004 deadlines concern its scan gate and changed monitor profile;
    # retaining those thresholds would mislabel this operational-profile run.
    for key in ('stop_deadline_s', 'sudden_stop_deadline_s',
                'timeout_stop_deadline_s', 'maximum_stop_distance_m',
                'source_timeout_candidates_s',
                'synthetic_valid_scan_gap_traces_s', 'sudden_latency_semantic',
                'matrix', 'retention'):
        contract.pop(key, None)
    contract['measurement_semantic'] = (
        'observer scan-receive to zero-command-receive interval only; '
        'not sensor-publish, obstacle-appearance, or physical-stop latency')
    return contract


def scenario_passed(document: dict, returncode: int) -> bool:
    """Require one uncancelled goal, STOP, observed standstill, and arrival."""
    goal_uuid = document.get('goal_uuid')
    pose = document.get('final_world_pose_m')
    contract = document.get('contract', {})
    goal = contract.get('goal_pose', {})
    arrived = (
        isinstance(pose, (list, tuple)) and len(pose) == 2
        and all(math.isfinite(value) for value in pose)
        and 'x_m' in goal and 'y_m' in goal
        and math.dist(pose, (goal['x_m'], goal['y_m'])) <= 0.5)
    capture = document.get('direct_scan_capture') or {}
    return (
        returncode == 0 and arrived and bool(goal_uuid)
        and document.get('terminal_goal_uuid') == goal_uuid
        and document.get('goal_send_count') == 1
        and document.get('goal_cancel_count') == 0
        and document.get('action_terminal') == 'succeeded'
        and document.get('stop_action_type') == 1
        and document.get('stop_polygon_name') == 'StopZone'
        and document.get('resume_action_type') == 0
        and document.get('physical_stop_observed') is True
        and document.get('clear_scan_stamp_ns') is not None
        and document.get('contact_matched_publisher_count_max', 0) > 0
        and document.get('contact_count') == 0
        and document.get('footprint_to_obstacle_clearance_m', 0) > 0
        and document.get('protected_envelope_to_obstacle_clearance_m', 0) > 0
        and document.get('final_cmd_vel_zero') is True
        and document.get('final_zero_hold_s', 0) >= contract.get(
            'final_zero_hold_s', math.inf)
        and capture.get('zero_receive_steady_ns', 0) > capture.get(
            'scan_receive_steady_ns', math.inf)
        and document.get('activation_error') is None
        and document.get('harness_error') is None
    )
