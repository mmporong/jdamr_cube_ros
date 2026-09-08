#!/usr/bin/env python3
"""Run a bounded simulation-only smoke of the real onboard Nav2 candidate."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path  # noqa: I100
from typing import Any

from evaluate_same_goal_bag import evaluate as evaluate_same_goal_bag

from jdamr_cube_navigation.keepout_mask import build_mask, validate_mask

from navigation_mcap_reader import read_navigation_messages

from onboard_stop_contract import (
    build_contract as build_stop_contract,
    scenario_passed as stop_scenario_passed,
)

from prepare_sim_nav_obstacle_run import prepare

from run_sim_nav_obstacle_eval import (  # noqa: I101
    _cleanup_identity, _environment, _group_members, _sha256, _start, _stop,
    _wait_lifecycle_active, _wait_tf_available, _wait_topics, ASSETS, BT,
)

import yaml


ROOT = Path(__file__).resolve().parents[2]
PRODUCTION_PARAMS = ROOT / 'jdamr_cube_navigation/config/nav2_params.yaml'
PROCESS_MARKER = 'JDAMR_NAV_EVAL_RUN_ID'
DOMAIN_IDS = {186, 187}
SEED = 11
CASES = ('detour', 'event_driven_removal', 'sudden_stop_resume')
DEFAULT_CASES = ('detour', 'event_driven_removal')
ALLOWED_PARAM_DELTAS = {
    ('amcl', 'ros__parameters', 'initial_pose', 'x'),
    ('local_costmap', 'local_costmap', 'ros__parameters',
     'always_send_full_costmap'),
    ('global_costmap', 'global_costmap', 'ros__parameters',
     'always_send_full_costmap'),
}
CAP_BYTES = 64 * 1024 * 1024
BAG_LIVE_CAP_BYTES = 56 * 1024 * 1024
RECORDED_TOPICS = (
    '/cmd_vel', '/collision_monitor_state',
    '/navigate_to_pose/_action/status', '/odom', '/ground_truth_pose',
    '/plan', '/amcl_pose', '/tf', '/tf_static')
NAV_SCENARIO_SOURCE = (
    ROOT / 'jdamr_cube_navigation/jdamr_cube_navigation/'
    'sim_nav_obstacle_scenario.py')
STOP_SCENARIO_SOURCE = (
    ROOT / 'jdamr_cube_navigation/jdamr_cube_navigation/'
    'sim_collision_monitor_scenario.py')


def _leaf_differences(
        left: Any, right: Any, prefix=()) -> set[tuple[str, ...]]:
    """Return exact leaf paths whose values or presence differ."""
    if isinstance(left, dict) and isinstance(right, dict):
        differences = set()
        for key in left.keys() | right.keys():
            if key not in left or key not in right:
                differences.add((*prefix, str(key)))
            else:
                differences |= _leaf_differences(
                    left[key], right[key], (*prefix, str(key)))
        return differences
    return set() if left == right else {prefix}


def prepare_candidate_assets(output_root: Path) -> dict[str, Any]:
    """Create production-faithful params and a nonempty route-away mask."""
    assets = output_root / 'assets'
    generated = prepare(assets)
    production = yaml.safe_load(
        PRODUCTION_PARAMS.read_text(encoding='utf-8'))
    candidate = yaml.safe_load(PRODUCTION_PARAMS.read_text(encoding='utf-8'))
    candidate['amcl']['ros__parameters']['initial_pose'].update({
        'x': -8.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0})
    for name in ('local_costmap', 'global_costmap'):
        candidate[name][name]['ros__parameters'][
            'always_send_full_costmap'] = True
    differences = _leaf_differences(production, candidate)
    if differences != ALLOWED_PARAM_DELTAS:
        raise RuntimeError(f'candidate parameter delta drift: {differences}')
    generated['params'].write_text(
        yaml.safe_dump(candidate, sort_keys=False), encoding='utf-8')

    contract = json.loads(generated['contract'].read_text(encoding='utf-8'))
    contract['observation_persistence_s'] = 0.0
    contract['candidate_params'] = {
        'path': str(generated['params'].resolve()),
        'sha256': _sha256(generated['params']),
        'delta_paths': sorted('.'.join(path) for path in differences),
        'production_scan_overrides_added': [],
    }
    contract['claim_scope'] = (
        'SIM_INTEGRATION only: isolated Gazebo smoke of the real onboard '
        'obstacle_candidate launch; no physical-robot or safety claim.')
    generated['contract'].write_text(json.dumps(
        contract, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')

    direct_contract = build_stop_contract(PRODUCTION_PARAMS)
    probe = contract['preloaded_obstacles']['models'][
        'front_observation_probe']
    expected_dimensions_m = (
        direct_contract['sudden_obstacle']['length_m'],
        direct_contract['sudden_obstacle']['width_m'])
    if not all(math.isclose(actual, expected, abs_tol=1e-12)
               for actual, expected in zip(
                   (probe['length_m'], probe['width_m']),
                   expected_dimensions_m)):
        raise RuntimeError('direct-scan obstacle dimensions drifted')
    if probe['park_pose_m'] != direct_contract[
            'sudden_obstacle']['removal_pose_m']:
        raise RuntimeError('direct-scan obstacle parking pose drifted')
    direct_contract['preloaded_obstacle'] = {
        'role': probe['role'], 'name': probe['name'],
        'length_m': probe['length_m'], 'width_m': probe['width_m'],
        'park_pose_m': probe['park_pose_m'],
    }
    direct_contract_path = assets / 'onboard_stop_contract.json'
    direct_contract_path.write_text(json.dumps(
        direct_contract, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')

    zones = assets / 'sim_keepout_zones.yaml'
    zones.write_text(yaml.safe_dump({
        'schema_version': 1,
        'map_yaml': str((ASSETS / 'slam_corridor_eval.yaml').resolve()),
        'safety_margin_m': 0.35,
        'zones': [{
            'id': 'sim_route_away_northeast_corner',
            'enabled': True,
            'polygon': [[9.7, 1.55], [10.25, 1.55],
                        [10.25, 1.85], [9.7, 1.85]],
        }],
        'connectivity_checks': [{
            'id': 'candidate_route', 'start': [-8.0, 0.0],
            'goal': [6.0, 0.0], 'clearance_m': 0.0,
        }],
    }, sort_keys=False), encoding='utf-8')
    mask_report = build_mask(zones, assets / 'sim_keepout_mask')
    validate_mask(
        assets / 'sim_keepout_mask.yaml', ASSETS / 'slam_corridor_eval.yaml')
    return {
        'params': generated['params'], 'contract': generated['contract'],
        'stop_contract': direct_contract_path,
        'mask': assets / 'sim_keepout_mask.yaml',
        'mask_report': mask_report, 'param_deltas': differences,
    }


def _run(command: list[str], environment: dict[str, str], timeout_s=10.0):
    return subprocess.run(command, env=environment, capture_output=True,
                          text=True, timeout=timeout_s, check=False)


def _parameter(node: str, name: str, environment: dict[str, str]) -> str:
    result = _run(['ros2', 'param', 'get', node, name], environment)
    if result.returncode != 0:
        raise RuntimeError(f'parameter unavailable: {node}.{name}: '
                           f'{result.stderr.strip()}')
    return result.stdout.strip()


def _lifecycle(node: str, environment: dict[str, str]) -> str:
    result = _run(['ros2', 'lifecycle', 'get', node], environment)
    return result.stdout.strip() if result.returncode == 0 else 'unavailable'


def _wait_log_markers(path: Path, markers: tuple[str, ...],
                      timeout_s: float) -> None:
    """Wait for required in-process startup facts without DDS CLI delay."""
    deadline_s = time.monotonic() + timeout_s
    missing = list(markers)
    while time.monotonic() < deadline_s:
        text = path.read_text(errors='replace') if path.is_file() else ''
        missing = [marker for marker in markers if marker not in text]
        if not missing:
            return
        time.sleep(0.1)
    raise RuntimeError(f'launch log readiness timeout: {missing}')


def _write_recording_qos(path: Path) -> None:
    """Write compatible QoS while preserving the hidden status snapshot."""
    profile = {
        topic: {
            'history': 'keep_last', 'depth': 10,
            'reliability': 'best_effort', 'durability': 'volatile',
        } for topic in RECORDED_TOPICS
    }
    profile['/navigate_to_pose/_action/status'].update({
        'depth': 1, 'reliability': 'reliable',
        'durability': 'transient_local'})
    profile['/tf_static'].update({
        'depth': 1, 'durability': 'transient_local'})
    path.write_text(yaml.safe_dump(profile, sort_keys=True), encoding='utf-8')


def _start_compact_recorder(case_dir: Path, environment: dict[str, str]):
    """Start one wall-log-time MCAP recorder for compact runtime evidence."""
    qos_path = case_dir / 'recording_qos.yaml'
    _write_recording_qos(qos_path)
    bag_dir = case_dir / 'bag'
    command = [
        'ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_dir),
        '--storage-preset-profile', 'zstd_fast',
        '--include-hidden-topics', '--qos-profile-overrides-path',
        str(qos_path), '--topics', *RECORDED_TOPICS,
    ]
    if '--use-sim-time' in command:
        raise RuntimeError('recorder log time must remain wall time')
    return _start(command, case_dir / 'recorder.log', environment), bag_dir


def _wait_bag_ready(process: subprocess.Popen, bag_dir: Path,
                    timeout_s: float = 10.0) -> None:
    """Wait until the single MCAP writer creates its data file."""
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if process.poll() is not None:
            raise RuntimeError('compact recorder exited before ready')
        if any(bag_dir.glob('*.mcap')):
            return
        time.sleep(0.1)
    raise RuntimeError('compact recorder readiness timeout')


def _tree_size(root: Path) -> int:
    return sum(path.stat().st_size for path in root.rglob('*')
               if path.is_file())


def _one_finalized_mcap(bag_dir: Path) -> Path:
    matches = sorted(bag_dir.glob('*.mcap'))
    metadata = bag_dir / 'metadata.yaml'
    if len(matches) != 1 or not metadata.is_file():
        raise RuntimeError('compact recorder did not finalize one MCAP')
    mcap = matches[0]
    with mcap.open('rb') as stream:
        if stream.read(8) != b'\x89MCAP0\r\n':
            raise RuntimeError('invalid MCAP header')
        stream.seek(-8, 2)
        if stream.read(8) != b'\x89MCAP0\r\n':
            raise RuntimeError('invalid MCAP footer')
    if _tree_size(bag_dir) > CAP_BYTES:
        raise RuntimeError('finalized compact bag exceeds 64 MiB')
    return mcap


def _status_summary(mcap: Path) -> dict[str, Any]:
    uuids, statuses, message_count = [], [], 0
    for message in read_navigation_messages(
            mcap, topics=['/navigate_to_pose/_action/status']):
        message_count += 1
        for status in message.ros_msg.status_list:
            values = bytes(status.goal_info.goal_id.uuid)
            uuids.append(values.hex())
            statuses.append(int(status.status))
    return {
        'message_count': message_count,
        'goal_uuids': sorted(set(uuids)),
        'statuses': statuses,
        'final_status': statuses[-1] if statuses else None,
        'source_mcap_sha256': _sha256(mcap),
    }


def _source_identity(path: Path) -> dict[str, Any]:
    return {'path': str(path.resolve()), 'sha256': _sha256(path)}


def _cap_output(root: Path) -> int:
    """Enforce the evidence cap without deleting or rewriting raw data."""
    total = sum(
        path.stat().st_size for path in root.rglob('*') if path.is_file())
    if total > CAP_BYTES:
        raise RuntimeError(f'evidence exceeds 64 MiB cap: {total}')
    return total


def scenario_passed(document: dict[str, Any], case: str,
                    returncode: int) -> bool:
    """Reject action success unless ground truth proves the intended route."""
    pose = document.get('final_robot_pose_xy_yaw')
    goal_reached = (
        isinstance(pose, list) and len(pose) >= 2
        and abs(float(pose[0]) - 6.0) <= 0.5
        and abs(float(pose[1])) <= 0.5)
    marked = document.get('mark_eligible_scan') is not None
    replanned = bool(document.get('plans_after_mark'))
    if case == 'event_driven_removal':
        replanned = (
            replanned and document.get('removal_passage') is not None)
    return (
        returncode == 0
        and document.get('action_terminal') == 'succeeded'
        and document.get('global_blocking') is True
        and document.get('local_blocking') is True
        and document.get('contact_matched_publisher_count_max', 0) > 0
        and document.get('contact_count') == 0
        and document.get('final_cmd_vel_zero') is True
        and goal_reached and marked and replanned)


def _scenario_command(
        case: str, prepared: dict[str, Any], evidence: Path,
        entity: str, contact_topic: str) -> list[str]:
    if case == 'sudden_stop_resume':
        return [
            'ros2', 'run', 'jdamr_cube_navigation',
            'sim_collision_monitor_scenario',
            '--scenario', 'sudden_obstacle_stop_resume',
            '--seed', str(SEED), '--output', str(evidence),
            '--contract', str(prepared['stop_contract']),
            '--entity-name', entity, '--contact-topic', contact_topic,
            '--goal-x-m', '6.0', '--run-timeout-s', '180',
            '--direct-scan', '--ros-args', '-p', 'use_sim_time:=true',
        ]
    return [
        'ros2', 'run', 'jdamr_cube_navigation',
        'sim_nav_obstacle_scenario', '--scenario', case,
        '--seed', str(SEED), '--output', str(evidence),
        '--contract', str(prepared['contract']),
        '--behavior-tree', str(BT), '--run-timeout-s', '180',
        '--observation-persistence-s', '0.0',
        '--ros-args', '-p', 'use_sim_time:=true',
    ]


def _case_passed(
        document: dict[str, Any], case: str, returncode: int,
        same_goal_command_evidence: dict[str, Any]) -> bool:
    if case != 'sudden_stop_resume':
        return scenario_passed(document, case, returncode)
    command_evidence = same_goal_command_evidence.get('evidence', {})
    return (
        stop_scenario_passed(document, returncode)
        and command_evidence.get('verdict') == 'CONFIRMED'
        and command_evidence.get('terminal_succeeded') is True)


def run_case(case: str, output_root: Path, domain_id: int,
             prepared: dict[str, Any], startup_only: bool = False) -> dict:
    """Launch isolated Gazebo plus the real onboard core and run one case."""
    direct_stop = case == 'sudden_stop_resume'
    scenario_contract = (
        prepared['stop_contract'] if direct_stop else prepared['contract'])
    scenario_source = STOP_SCENARIO_SOURCE if direct_stop else NAV_SCENARIO_SOURCE
    run_id = f'onboard_candidate_{case}_seed_{SEED}'
    case_dir = output_root / case
    case_dir.mkdir(parents=True, exist_ok=True)
    environment = _environment(run_id, domain_id)
    environment.update({
        'ROS_LOCALHOST_ONLY': '1',
        'GZ_PARTITION': f'jdamr_onboard_{os.getpid()}_{case}',
    })
    launched = []
    bag_dir: Path | None = None
    mcap: Path | None = None
    result: dict[str, Any] = {
        'case': case, 'seed': SEED, 'domain_id': domain_id,
        'run_id': run_id, 'status': 'FAIL', 'scenario_started': False,
        'claim_scope': 'SIM_INTEGRATION',
        'case_claim': {
            'detour': 'fixed_obstacle_detour',
            'event_driven_removal': (
                'obstacle_removal_and_replan_not_sudden_stop_resume'),
            'sudden_stop_resume': (
                'synthetic_sudden_obstacle_stop_same_goal_resume'),
        }[case],
        'runtime_profile': {
            'name': 'obstacle_candidate',
            'launch': 'onboard_nav2_core.launch.py',
        },
        'source_identity': {
            'runner': _source_identity(Path(__file__)),
            'onboard_core': _source_identity(
                ROOT / 'jdamr_cube_navigation/launch/'
                'onboard_nav2_core.launch.py'),
            'production_params': _source_identity(PRODUCTION_PARAMS),
            'candidate_params': _source_identity(prepared['params']),
            'candidate_bt': _source_identity(BT),
            'scenario_contract': _source_identity(scenario_contract),
            'scenario_source': _source_identity(scenario_source),
        },
    }
    try:
        launched.append(_start([
            'ros2', 'launch', 'jdamr_cube_gazebo', 'gazebo.launch.py',
            f'world:={ASSETS / "slam_corridor_contact.world"}',
            f'urdf_file:={ASSETS / "jdamr_cube_nav_eval.urdf"}',
            'gui:=false', 'enable_image_bridges:=false', f'seed:={SEED}',
            'x_pose:=-8.0', 'y_pose:=0.0', 'z_pose:=0.01',
        ], case_dir / 'gazebo.log', environment))
        _wait_topics({'/scan', '/odom', '/ground_truth_pose'},
                     environment, 60.0)
        launched.append(_start([
            'ros2', 'launch', 'jdamr_cube_navigation',
            'onboard_nav2_core.launch.py',
            f'map:={ASSETS / "slam_corridor_eval.yaml"}',
            f'keepout_mask:={prepared["mask"]}',
            f'params_file:={prepared["params"]}',
            'use_sim_time:=true', 'autostart:=true',
            'navigation_profile:=obstacle_candidate',
        ], case_dir / 'onboard_nav2_core.log', environment))
        _wait_topics({
            '/global_costmap/costmap_raw', '/local_costmap/costmap_raw',
            '/clock', '/cmd_vel', '/plan', '/keepout_filter_mask',
            '/keepout_costmap_filter_info',
        }, environment, 75.0, case_dir / 'onboard_nav2_core.log')
        lifecycle_nodes = (
            '/keepout_filter_mask_server',
            '/keepout_costmap_filter_info_server', '/map_server', '/amcl',
            '/controller_server', '/planner_server', '/velocity_smoother',
            '/collision_monitor', '/behavior_server', '/bt_navigator')
        # bt_navigator is activated last by the navigation lifecycle manager.
        # Checking it plus both independently-managed keepout servers proves
        # readiness without ten serial ROS CLI discovery delays.  The complete
        # lifecycle inventory is captured after the time-sensitive scenario.
        _wait_lifecycle_active((
            '/keepout_filter_mask_server',
            '/keepout_costmap_filter_info_server', '/bt_navigator'),
            environment, 75.0)
        _wait_tf_available('map', 'base_footprint', environment, 60.0,
                           case_dir / 'tf_readiness.log')
        _wait_log_markers(case_dir / 'onboard_nav2_core.log', (
            '[behavior_server]: Activating wait',
            ('[local_costmap.local_costmap]: KeepoutFilter: '
             'Received filter mask'),
            ('[global_costmap.global_costmap]: KeepoutFilter: '
             'Received filter mask'),
        ), 15.0)
        pre_goal_clocks = {
            node: _parameter(node, 'use_sim_time', environment)
            for node in ('/controller_server', '/collision_monitor')}
        if not all(value.lower().endswith('true')
                   for value in pre_goal_clocks.values()):
            raise RuntimeError(
                f'pre-goal ROS time mismatch: {pre_goal_clocks}')
        if startup_only:
            result['startup'] = {
                'status': 'PASS', 'pre_goal_use_sim_time': pre_goal_clocks}
            result['status'] = 'PASS'
            return result

        asset_contract = json.loads(
            prepared['contract'].read_text(encoding='utf-8'))
        role = 'front_observation_probe' if direct_stop else 'route'
        entity = asset_contract[
            'preloaded_obstacles']['models'][role]['name']
        contact_topic = (f'/world/slam_corridor/model/{entity}/link/body/'
                         'sensor/contact_sensor/contact')
        launched.append(_start([
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'{contact_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ], case_dir / 'contact_bridge.log', environment))
        recorder, bag_dir = _start_compact_recorder(case_dir, environment)
        launched.append(recorder)
        _wait_bag_ready(recorder[0], bag_dir)
        evidence = case_dir / 'scenario.json'
        scenario = _start(_scenario_command(
            case, prepared, evidence, entity, contact_topic),
            case_dir / 'scenario.log', environment)
        launched.append(scenario)
        result['scenario_started'] = True
        deadline_s = time.monotonic() + 210.0
        while scenario[0].poll() is None:
            if recorder[0].poll() is not None:
                raise RuntimeError('compact recorder exited during scenario')
            if _tree_size(output_root) > BAG_LIVE_CAP_BYTES:
                raise RuntimeError('smoke evidence exceeded 56 MiB live cap')
            if time.monotonic() >= deadline_s:
                raise subprocess.TimeoutExpired('scenario', 210.0)
            time.sleep(0.25)
        returncode = scenario[0].wait(timeout=3.0)
        if recorder[0].poll() is not None:
            raise RuntimeError('compact recorder exited before scenario end')
        _stop(recorder[0])
        mcap = _one_finalized_mcap(bag_dir)
        result['recording'] = {
            'mcap': _source_identity(mcap),
            'size_bytes': mcap.stat().st_size,
            'topics': list(RECORDED_TOPICS),
            'log_time_basis': 'wall_time',
            'message_header_time_basis': 'publisher_defined_sim_time',
            'time_basis_mixing_forbidden': True,
        }
        result['raw_action_status'] = _status_summary(mcap)
        same_goal_evidence = evaluate_same_goal_bag(mcap)
        result['same_goal_command_evidence'] = same_goal_evidence
        if not evidence.is_file():
            raise RuntimeError('scenario produced no evidence')
        scenario_document = json.loads(evidence.read_text(encoding='utf-8'))
        bt_parameter = _parameter(
            '/bt_navigator', 'default_nav_to_pose_bt_xml', environment)
        keepout_parameters = {
            name: _parameter(
                f'/{name}/{name}', 'keepout_filter.enabled', environment)
            for name in ('local_costmap', 'global_costmap')}
        clock_parameters = {
            node: _parameter(node, 'use_sim_time', environment)
            for node in ('/amcl', '/controller_server',
                         '/collision_monitor', '/bt_navigator')}
        result['startup'] = {
            'status': 'PASS', 'candidate_bt': bt_parameter,
            'keepout_enabled': keepout_parameters,
            'use_sim_time': clock_parameters,
            'pre_goal_use_sim_time': pre_goal_clocks,
            'timing_note': (
                'moving serial diagnostics after the scenario reduces delay; '
                'explicit component use_sim_time is the clock-domain fix'),
            'lifecycle': {node: _lifecycle(node, environment)
                          for node in lifecycle_nodes},
            'mask_keepout_cells': prepared['mask_report']['keepout_cells'],
            'gz_partition': environment['GZ_PARTITION'],
            'ros_localhost_only': environment['ROS_LOCALHOST_ONLY'],
        }
        if BT.name not in bt_parameter or not all(
                value.lower().endswith('true')
                for value in (*keepout_parameters.values(),
                              *clock_parameters.values())):
            raise RuntimeError('post-run candidate configuration drift')
        if direct_stop:
            result['scenario'] = {
                'returncode': returncode,
                'action_terminal': scenario_document.get('action_terminal'),
                'events': [event['name'] for event in
                           scenario_document.get('events', [])],
                'goal_uuid': scenario_document.get('goal_uuid'),
                'terminal_goal_uuid': scenario_document.get(
                    'terminal_goal_uuid'),
                'goal_send_count': scenario_document.get('goal_send_count'),
                'goal_cancel_count': scenario_document.get(
                    'goal_cancel_count'),
                'pre_stop_action_types': scenario_document.get(
                    'pre_stop_action_types'),
                'stop_action_type': scenario_document.get('stop_action_type'),
                'stop_polygon_name': scenario_document.get(
                    'stop_polygon_name'),
                'resume_action_type': scenario_document.get(
                    'resume_action_type'),
                'physical_stop_observed': scenario_document.get(
                    'physical_stop_observed'),
                'direct_scan_capture': scenario_document.get(
                    'direct_scan_capture'),
                'observer_scan_to_zero_command_s': (
                    (
                        scenario_document['direct_scan_capture'][
                            'zero_receive_steady_ns']
                        - scenario_document['direct_scan_capture'][
                            'scan_receive_steady_ns']) / 1e9
                    if (scenario_document.get('direct_scan_capture')
                        and 'zero_receive_steady_ns'
                        in scenario_document['direct_scan_capture'])
                    else None),
                'contact_count': scenario_document.get('contact_count'),
                'contact_matched_publisher_count_max': scenario_document.get(
                    'contact_matched_publisher_count_max'),
                'minimum_clearance_m': scenario_document.get(
                    'footprint_to_obstacle_clearance_m'),
                'final_cmd_vel_zero': scenario_document.get(
                    'final_cmd_vel_zero'),
                'final_zero_hold_s': scenario_document.get(
                    'final_zero_hold_s'),
                'final_world_pose_m': scenario_document.get(
                    'final_world_pose_m'),
                'activation_error': scenario_document.get('activation_error'),
                'harness_error': scenario_document.get('harness_error'),
                'same_goal_command_verdict': same_goal_evidence[
                    'evidence']['verdict'],
            }
        else:
            result['scenario'] = {
                'returncode': returncode,
                'action_terminal': scenario_document.get('action_terminal'),
                'events': [event['name'] for event in
                           scenario_document.get('events', [])],
                'costmap_message_counts': scenario_document.get(
                    'costmap_message_counts'),
                'contact_count': scenario_document.get('contact_count'),
                'contact_matched_publisher_count_max': scenario_document.get(
                    'contact_matched_publisher_count_max'),
                'global_blocking': scenario_document.get('global_blocking'),
                'local_blocking': scenario_document.get('local_blocking'),
                'final_cmd_vel_zero': scenario_document.get(
                    'final_cmd_vel_zero'),
                'final_zero_hold_s': scenario_document.get(
                    'final_zero_hold_s'),
                'minimum_obstacle_aabb_distance_m': scenario_document.get(
                    'minimum_robot_obstacle_aabb_distance_m'),
                'final_robot_pose_xy_yaw': scenario_document.get(
                    'final_robot_pose_xy_yaw'),
                'mark_eligible_scan': scenario_document.get(
                    'mark_eligible_scan'),
                'plans_after_mark_count': len(
                    scenario_document.get('plans_after_mark', [])),
                'activation_error': scenario_document.get('activation_error'),
            }
        passed = _case_passed(
            scenario_document, case, returncode, same_goal_evidence)
        result['status'] = 'PASS' if passed else 'FAIL'
    except Exception as error:
        result['harness_error'] = f'{type(error).__name__}: {error}'
    finally:
        groups = [process.pid for process, _stream in launched]
        for process, _stream in reversed(launched):
            _stop(process)
        for _process, stream in launched:
            stream.close()
        survivors = _cleanup_identity(run_id, domain_id)
        if 'raw_action_status' not in result and mcap is not None:
            try:
                result['raw_action_status'] = _status_summary(mcap)
            except Exception as error:
                result['recording_error'] = (
                    f'{type(error).__name__}: {error}')
                result['status'] = 'FAIL'
        result['teardown'] = {
            'launched_process_groups': groups,
            'remaining_process_groups': [group for group in groups
                                         if _group_members(group)],
            'identity_survivors': survivors,
        }
        if (result['teardown']['remaining_process_groups'] or survivors):
            result['status'] = 'FAIL'
            result['teardown_error'] = 'owned_processes_remain'
        try:
            result['output_bytes'] = _cap_output(case_dir)
        except RuntimeError as error:
            result['output_bytes'] = _tree_size(case_dir)
            result['recording_error'] = str(error)
            result['status'] = 'FAIL'
        (case_dir / 'summary.json').write_text(json.dumps(
            result, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
    return result


def main() -> int:
    """Parse the bounded smoke request, execute it, and persist a summary."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--domain-id', type=int, default=186)
    parser.add_argument('--case', choices=CASES, action='append')
    parser.add_argument('--startup-only', action='store_true')
    args = parser.parse_args()
    if args.domain_id == 12 or args.domain_id not in DOMAIN_IDS:
        parser.error('only isolated ROS domain 186 or 187 is allowed')
    selected = args.case or list(DEFAULT_CASES)
    assigned_domains = [args.domain_id + index
                        for index in range(len(selected))]
    if any(domain not in DOMAIN_IDS for domain in assigned_domains):
        parser.error('selected cases require only domains 186 and 187')
    if args.output_root.exists() and any(args.output_root.iterdir()):
        parser.error('output root must be new or empty')
    args.output_root.mkdir(parents=True, exist_ok=True)
    prepared = prepare_candidate_assets(args.output_root)
    results = []
    for index, case in enumerate(selected):
        results.append(run_case(
            case, args.output_root, assigned_domains[index], prepared,
            startup_only=args.startup_only))
    summary = {
        'schema_version': 1, 'seed': SEED, 'cases_requested': selected,
        'claim_scope': 'SIM_INTEGRATION', 'results': results,
        'status': ('PASS' if all(item['status'] == 'PASS' for item in results)
                   else 'FAIL'),
    }
    (args.output_root / 'summary.json').write_text(json.dumps(
        summary, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    try:
        summary['output_bytes'] = _cap_output(args.output_root)
    except RuntimeError as error:
        summary['status'] = 'FAIL'
        summary['output_error'] = str(error)
    (args.output_root / 'summary.json').write_text(json.dumps(
        summary, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    return 0 if summary['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
