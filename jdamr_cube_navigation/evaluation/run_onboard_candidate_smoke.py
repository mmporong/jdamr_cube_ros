#!/usr/bin/env python3
"""Run a bounded simulation-only smoke of the real onboard Nav2 candidate."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re  # noqa: I100
import subprocess
import time
from typing import Any

from jdamr_cube_navigation.keepout_mask import build_mask, validate_mask

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
CASES = ('detour', 'event_driven_removal')
ALLOWED_PARAM_DELTAS = {
    ('amcl', 'ros__parameters', 'initial_pose', 'x'),
    ('local_costmap', 'local_costmap', 'ros__parameters',
     'always_send_full_costmap'),
    ('global_costmap', 'global_costmap', 'ros__parameters',
     'always_send_full_costmap'),
}
CAP_BYTES = 64 * 1024 * 1024


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


def _raw_observers(case_dir: Path, environment: dict[str, str]):
    launched = []
    specs = (
        ('action_status.log', '/navigate_to_pose/_action/status', None),
        ('collision_monitor_state.log', '/collision_monitor_state', None),
        ('cmd_vel.log', '/cmd_vel', None),
        ('odom.log', '/odom', None),
        ('global_costmap_once.log', '/global_costmap/costmap_raw', '--once'),
        ('local_costmap_once.log', '/local_costmap/costmap_raw', '--once'),
    )
    for filename, topic, once in specs:
        command = ['ros2', 'topic', 'echo', topic, '--full-length']
        if topic == '/navigate_to_pose/_action/status':
            command.extend([
                '--qos-reliability', 'reliable',
                '--qos-durability', 'transient_local'])
        elif 'costmap' in topic:
            command.extend([
                '--qos-reliability', 'reliable',
                '--qos-durability', 'transient_local'])
        if once:
            command.append(once)
        launched.append(_start(command, case_dir / filename, environment))
    return launched


def _status_summary(path: Path) -> dict[str, Any]:
    text = path.read_text(errors='replace') if path.is_file() else ''
    uuids = []
    for match in re.finditer(
            r'goal_info:\s*\n\s*goal_id:\s*\n\s*uuid:\s*\n'
            r'((?:\s*-\s*\d+\s*\n){16})', text):
        values = [int(value) for value in re.findall(r'-\s*(\d+)', match[1])]
        uuids.append(''.join(f'{value:02x}' for value in values))
    statuses = [int(value) for value in re.findall(
        r'(?m)^\s*status:\s*(\d+)\s*$', text)]
    return {
        'message_count': text.count('status_list:'),
        'goal_uuids': sorted(set(uuids)),
        'statuses': statuses,
        'final_status': statuses[-1] if statuses else None,
        'sha256': _sha256(path) if path.is_file() else None,
    }


def _cap_output(root: Path) -> int:
    """Bound text evidence while preserving large-log heads and tails."""
    for path in root.rglob('*.log'):
        if path.stat().st_size <= 4 * 1024 * 1024:
            continue
        data = path.read_bytes()
        marker = b'\nONBOARD_SMOKE_LOG_TRUNCATED\n'
        path.write_bytes(data[:2 * 1024 * 1024] + marker
                         + data[-2 * 1024 * 1024:])
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


def run_case(case: str, output_root: Path, domain_id: int,
             prepared: dict[str, Any], startup_only: bool = False) -> dict:
    """Launch isolated Gazebo plus the real onboard core and run one case."""
    run_id = f'onboard_candidate_{case}_seed_{SEED}'
    case_dir = output_root / case
    case_dir.mkdir(parents=True, exist_ok=True)
    environment = _environment(run_id, domain_id)
    environment.update({
        'ROS_LOCALHOST_ONLY': '1',
        'GZ_PARTITION': f'jdamr_onboard_{os.getpid()}_{case}',
    })
    launched = []
    result: dict[str, Any] = {
        'case': case, 'seed': SEED, 'domain_id': domain_id,
        'run_id': run_id, 'status': 'FAIL', 'scenario_started': False,
        'claim_scope': 'SIM_INTEGRATION',
        'case_claim': (
            'fixed_obstacle_detour' if case == 'detour'
            else 'obstacle_removal_and_replan_not_sudden_stop_resume'),
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

        contract = json.loads(prepared['contract'].read_text(encoding='utf-8'))
        role = 'route'
        entity = contract['preloaded_obstacles']['models'][role]['name']
        contact_topic = (f'/world/slam_corridor/model/{entity}/link/body/'
                         'sensor/contact_sensor/contact')
        launched.append(_start([
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'{contact_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ], case_dir / 'contact_bridge.log', environment))
        launched.extend(_raw_observers(case_dir, environment))
        evidence = case_dir / 'scenario.json'
        scenario = _start([
            'ros2', 'run', 'jdamr_cube_navigation',
            'sim_nav_obstacle_scenario', '--scenario', case,
            '--seed', str(SEED), '--output', str(evidence),
            '--contract', str(prepared['contract']),
            '--behavior-tree', str(BT), '--run-timeout-s', '180',
            '--observation-persistence-s', '0.0',
            '--ros-args', '-p', 'use_sim_time:=true',
        ], case_dir / 'scenario.log', environment)
        launched.append(scenario)
        result['scenario_started'] = True
        returncode = scenario[0].wait(timeout=210.0)
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
            'final_cmd_vel_zero': scenario_document.get('final_cmd_vel_zero'),
            'final_zero_hold_s': scenario_document.get('final_zero_hold_s'),
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
        passed = scenario_passed(scenario_document, case, returncode)
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
        result['raw_action_status'] = _status_summary(
            case_dir / 'action_status.log')
        result['teardown'] = {
            'launched_process_groups': groups,
            'remaining_process_groups': [group for group in groups
                                         if _group_members(group)],
            'identity_survivors': survivors,
        }
        if (result['teardown']['remaining_process_groups'] or survivors):
            result['status'] = 'FAIL'
            result['teardown_error'] = 'owned_processes_remain'
        result['output_bytes'] = _cap_output(case_dir)
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
    selected = args.case or list(CASES)
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
    _cap_output(args.output_root)
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    return 0 if summary['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
