#!/usr/bin/env python3
"""Run one integrated restaurant obstacle and traction replay in Gazebo."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any

from prepare_sim_nav_obstacle_run import prepare as prepare_navigation

from run_sim_nav_obstacle_eval import (
    _cleanup_identity,
    _environment,
    _group_members,
    _sha256,
    _start,
    _stop,
    _wait_lifecycle_active,
    _wait_tf_available,
    _wait_topics,
)


ROOT = Path(__file__).resolve().parents[2]
ASSETS = (ROOT / 'jdamr_cube_navigation' / 'evaluation' / 'assets'
          / 'nav_obstacle')
BT = (ROOT / 'jdamr_cube_navigation' / 'behavior_trees'
      / 'navigate_to_pose_dynamic_obstacle_eval.xml')
RECORDED_TOPICS = (
    '/tf', '/tf_static', '/scan', '/odom', '/ground_truth_pose',
    '/amcl_pose', '/plan', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel',
    '/guarded_cmd_vel', '/collision_monitor_state',
    '/navigate_to_pose/_action/status', '/clock',
)


def one_mcap(bag_dir: Path) -> Path | None:
    """Return the only finalized MCAP, or None for incomplete evidence."""
    files = sorted(bag_dir.glob('*.mcap')) if bag_dir.is_dir() else []
    metadata = bag_dir / 'metadata.yaml'
    return files[0] if len(files) == 1 and metadata.is_file() else None


def classify_result(
        scenario: dict[str, Any] | None, guard: dict[str, Any] | None,
        mcap: Path | None,
        survivors: list[dict[str, Any]]) -> tuple[str, list[str]]:
    """Apply every terminal gate without laundering partial execution."""
    failures = []
    if scenario is None:
        failures.append('scenario_evidence_missing')
    else:
        if scenario.get('status') != 'PASS':
            failures.append('scenario_failed')
        if scenario.get('successful_goal_count') != 20:
            failures.append('waypoint_gate_failed')
        if scenario.get('successful_intervention_count') != 6:
            failures.append('obstacle_intervention_gate_failed')
    if guard is None:
        failures.append('guard_evidence_missing')
    else:
        if guard.get('recovery_count') != 1:
            failures.append('traction_recovery_count_failed')
        if guard.get('final_state') == 'FAULT_LATCHED':
            failures.append('traction_guard_latched')
    if mcap is None:
        failures.append('mcap_missing_or_unfinalized')
    if survivors:
        failures.append('process_survivors_present')
    return ('PASS' if not failures else 'FAIL', failures)


def load_if_present(path: Path) -> dict[str, Any] | None:
    """Load one JSON document only when the producer finalized it."""
    if not path.is_file() or path.stat().st_size == 0:
        return None
    return json.loads(path.read_text(encoding='utf-8'))


def main(argv: list[str] | None = None) -> int:
    """Launch the isolated stack, run the replay, and seal its evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario-contract', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--domain-id', type=int, default=198)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--timeout-s', type=float, default=900.0)
    args = parser.parse_args(argv)
    if args.domain_id == 12:
        parser.error('physical robot domain 12 is forbidden')
    if args.output_dir.exists():
        parser.error('--output-dir must not already exist')
    if not args.scenario_contract.is_file():
        parser.error('--scenario-contract must exist')
    contract = json.loads(
        args.scenario_contract.read_text(encoding='utf-8'))
    world = Path(contract['traction_fault']['world']['path'])
    robot_urdf = Path(contract['traction_fault']['robot_urdf']['path'])
    bridge_record = contract['traction_fault'].get('guarded_bridge')
    bridge = Path(bridge_record['path']) if bridge_record else None
    for label, path in (
            ('traction world', world), ('guarded bridge', bridge),
            ('evaluation URDF', robot_urdf),
            ('evaluation map', ASSETS / 'slam_corridor_eval.yaml'),
            ('behavior tree', BT)):
        if path is None or not path.is_file():
            parser.error(f'{label} does not exist: {path}')

    run_dir = args.output_dir
    run_dir.mkdir(parents=True)
    nav_inputs = prepare_navigation(
        run_dir / 'navigation_inputs', movement_time_allowance_s=25.0)
    scenario_path = run_dir / 'scenario_evidence.json'
    guard_path = run_dir / 'traction_guard_evidence.json'
    bag_dir = run_dir / 'bag'
    environment = _environment(args.run_id, args.domain_id)
    processes: list[tuple[str, subprocess.Popen, Any]] = []
    failure: str | None = None
    scenario_returncode: int | None = None

    def launch(name: str, command: list[str]) -> subprocess.Popen:
        process, stream = _start(
            command, run_dir / f'{name}.log', environment)
        processes.append((name, process, stream))
        return process

    try:
        launch('gazebo', [
            'ros2', 'launch', 'jdamr_cube_gazebo', 'gazebo.launch.py',
            f'world:={world}',
            f'urdf_file:={robot_urdf}',
            f'bridge_config:={bridge}', 'gui:=false',
            'enable_image_bridges:=false', f'seed:={args.seed}',
            'x_pose:=-8.0', 'y_pose:=0.0', 'z_pose:=0.01',
        ])
        _wait_topics(
            {'/scan', '/odom', '/ground_truth_pose'}, environment, 60.0,
            run_dir / 'gazebo.log')
        launch('navigation', [
            'ros2', 'launch', 'jdamr_cube_navigation',
            'navigation.launch.py',
            f'map:={ASSETS / "slam_corridor_eval.yaml"}',
            f'params_file:={nav_inputs["params"]}',
            'use_keepout:=false', 'use_sim_time:=true',
            'use_composition:=True',
        ])
        _wait_topics({
            '/cmd_vel', '/plan', '/collision_monitor_state', '/amcl_pose',
        }, environment, 90.0, run_dir / 'navigation.log')
        _wait_lifecycle_active((
            '/amcl', '/controller_server', '/planner_server',
            '/bt_navigator', '/velocity_smoother', '/collision_monitor',
        ), environment, 90.0)
        _wait_tf_available(
            'map', 'base_footprint', environment, 60.0,
            run_dir / 'tf_readiness.log')
        launch('traction_guard', [
            'ros2', 'run', 'jdamr_cube_navigation',
            'traction_velocity_guard', '--output', str(guard_path),
            '--ros-args', '-p', 'use_sim_time:=true',
        ])
        _wait_topics({'/guarded_cmd_vel'}, environment, 30.0)
        launch('recorder', [
            'ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_dir),
            '--topics', *RECORDED_TOPICS,
        ])
        scenario = launch('scenario', [
            'ros2', 'run', 'jdamr_cube_navigation',
            'sim_restaurant_replay_scenario',
            '--contract', str(args.scenario_contract.resolve()),
            '--behavior-tree', str(BT), '--output', str(scenario_path),
            '--obstacle-entity', 'g003_preloaded_route_obstacle',
            '--goal-timeout-s', '120.0',
            '--ros-args', '-p', 'use_sim_time:=true',
        ])
        scenario_returncode = scenario.wait(timeout=args.timeout_s)
        if scenario_returncode != 0:
            failure = f'scenario_returncode:{scenario_returncode}'
    except Exception as error:
        failure = f'{type(error).__name__}: {error}'
    finally:
        launched_groups = [process.pid for _, process, _ in processes]
        for _, process, _ in reversed(processes):
            _stop(process)
        for _, _, stream in processes:
            stream.close()
        survivors = _cleanup_identity(args.run_id, args.domain_id)
        remaining_groups = [
            group for group in launched_groups if _group_members(group)]

    scenario_evidence = load_if_present(scenario_path)
    guard_evidence = load_if_present(guard_path)
    mcap = one_mcap(bag_dir)
    outcome, gate_failures = classify_result(
        scenario_evidence, guard_evidence, mcap, survivors)
    if remaining_groups:
        outcome = 'FAIL'
        gate_failures.append('launched_process_groups_not_empty')
    if failure is not None:
        outcome = 'FAIL'
        gate_failures.append('runner_failure')
    summary = {
        'schema_version': 1,
        'run_id': args.run_id,
        'status': outcome,
        'failure': failure,
        'gate_failures': gate_failures,
        'scenario_returncode': scenario_returncode,
        'source': {
            'scenario_contract': {
                'path': str(args.scenario_contract.resolve()),
                'sha256': _sha256(args.scenario_contract),
            },
            'world': {'path': str(world), 'sha256': _sha256(world)},
            'robot_urdf': {
                'path': str(robot_urdf), 'sha256': _sha256(robot_urdf)},
            'bridge': {'path': str(bridge), 'sha256': _sha256(bridge)},
        },
        'scenario': scenario_evidence,
        'traction_guard': guard_evidence,
        'bag': (
            {'path': str(mcap.resolve()), 'sha256': _sha256(mcap),
             'bytes': mcap.stat().st_size}
            if mcap is not None else None),
        'teardown': {
            'launched_process_groups': launched_groups,
            'remaining_process_groups': remaining_groups,
            'identity_survivors': survivors,
        },
    }
    summary_path = run_dir / 'summary.json'
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(json.dumps({
        'status': outcome, 'run_dir': str(run_dir.resolve()),
        'gate_failures': gate_failures,
    }, ensure_ascii=False))
    return 0 if outcome == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
