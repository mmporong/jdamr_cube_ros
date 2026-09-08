"""Define a direct-scan integration probe without changing the Nav2 controller."""

import json
import math
from pathlib import Path

from prepare_sim_collision_monitor_run import (
    _production_motion_inputs, installed_nav2_versions, PRODUCTION_FILES,
    SCAN_PROFILE,
)
from sim_collision_monitor_contract import derive_contract, sha256_file

import yaml


def build_contract(params_path: Path) -> dict:
    """Derive observation geometry from the unchanged production footprint."""
    params = yaml.safe_load(params_path.read_text(encoding='utf-8'))
    motion = _production_motion_inputs(
        params, PRODUCTION_FILES['production_urdf'])
    profile = json.loads(SCAN_PROFILE.read_text(encoding='utf-8'))
    gap_s = profile['timing']['scan']['header_interval_s']['max']
    smoother = params['velocity_smoother']['ros__parameters']
    contract = derive_contract(
        float(smoother['smoothing_frequency']), 0.001,
        installed_nav2_versions(), gap_s, **motion)
    monitor = params['collision_monitor']['ros__parameters']
    zone = monitor['StopZone']
    if zone['action_type'] != 'stop' or zone['enabled'] is not True:
        raise ValueError('probe requires the production StopZone')
    points = json.loads(zone['points'])
    if (len(points) != 4 or any(len(point) != 2 for point in points)
            or not all(math.isfinite(value) for point in points for value in point)):
        raise ValueError('probe requires a finite rectangular stop polygon')
    front_m = max(point[0] for point in points)
    rear_m = min(point[0] for point in points)
    half_width_m = max(abs(point[1]) for point in points)
    if {tuple(point) for point in points} != {
            (front_m, half_width_m), (front_m, -half_width_m),
            (rear_m, half_width_m), (rear_m, -half_width_m)}:
        raise ValueError('probe requires an axis-aligned symmetric rectangle')
    footprint_front_m = motion['footprint_front_m']
    if front_m <= footprint_front_m:
        raise ValueError('no non-contact test region ahead of the footprint')
    # Midpoint is a synthetic injection placement, not a calibrated safe
    # distance.  It exercises the existing STOP zone without changing it.
    surface_m = (front_m + footprint_front_m) / 2.0
    contract['stop_zone'] = {
        'front_m': front_m, 'rear_m': rear_m,
        'half_width_m': half_width_m,
        'min_points': int(zone['min_points']),
        'inputs': contract['stop_zone']['inputs'],
        'source': 'unchanged_production_StopZone',
    }
    contract['sudden_obstacle'].update({
        'activation_surface_x_m': surface_m,
        'activation_center_offset_x_m': (
            surface_m + contract['sudden_obstacle']['length_m'] / 2.0),
        'placement': 'synthetic_midpoint_between_footprint_and_StopZone_front',
        'trigger_min_speed_mps': motion['max_forward_speed_mps'] * 0.9,
        'lead_margin_m': None,
    })
    contract['integration_kind'] = 'onboard_candidate_direct_scan'
    contract['claim_scope'] = (
        'SIM_INTEGRATION_ONLY: synthetic intrusion into the unchanged STOP '
        'zone; not human tracking, real-world stopping distance, latency '
        'qualification, or a G004 full-matrix result')
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
        and document.get('final_cmd_vel_zero') is True
        and document.get('final_zero_hold_s', 0) >= contract.get(
            'final_zero_hold_s', math.inf)
        and capture.get('zero_receive_steady_ns', 0) > capture.get(
            'scan_receive_steady_ns', math.inf)
        and document.get('activation_error') is None
        and document.get('harness_error') is None
    )
