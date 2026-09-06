#!/usr/bin/env python3
"""Compare composed Nav2 startup readiness across evaluation-only RMWs."""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
import time
from pathlib import Path  # noqa: I100
from typing import Any

from prepare_sim_nav_obstacle_run import prepare

from run_sim_nav_obstacle_eval import (  # noqa: I101
    ASSETS, InfrastructureInvalid, _cleanup_identity, _environment, _start,
    _stop, _wait_lifecycle_active, _wait_scan, _wait_tf_available,
    _wait_topics, configure_evaluation_rmw, evaluation_rmw_evidence)


RESOURCE_SAMPLER = Path(__file__).with_name(
    'sample_process_group_resources.py')
NAVIGATION_NODES = (
    '/amcl', '/controller_server', '/planner_server', '/bt_navigator',
    '/velocity_smoother', '/collision_monitor')
REQUIRED_TOPICS = {
    '/global_costmap/costmap_raw', '/local_costmap/costmap_raw',
    '/cmd_vel', '/plan'}
RMW_IMPLEMENTATIONS = ('rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp')


def startup_matrix(
        base_domain_id: int, attempts_per_rmw: int,
        cyclone_overlay_prefix: Path) -> list[dict[str, Any]]:
    """Build an alternating, unique-domain startup comparison schedule."""
    if attempts_per_rmw <= 0:
        raise ValueError('attempts_per_rmw must be positive')
    attempts = []
    for attempt_index in range(attempts_per_rmw):
        for implementation in RMW_IMPLEMENTATIONS:
            domain_id = base_domain_id + len(attempts)
            if domain_id == 12 or not 0 <= domain_id <= 232:
                raise ValueError('startup probe domain is outside 0..232')
            attempts.append({
                'attempt_index': attempt_index + 1,
                'domain_id': domain_id,
                'implementation': implementation,
                'overlay_prefix': (
                    cyclone_overlay_prefix
                    if implementation == 'rmw_cyclonedds_cpp' else None),
            })
    return attempts


def _resource_summary(path: Path) -> dict[str, Any]:
    rows = [json.loads(line) for line in path.read_text().splitlines()
            if line.strip()]
    cpu_values = [row['cpu_pct_one_core'] for row in rows
                  if row['cpu_pct_one_core'] is not None]
    rss_values = [row['rss_mb'] for row in rows]
    return {
        'sample_count': len(rows),
        'peak_cpu_pct_one_core': max(cpu_values, default=None),
        'median_cpu_pct_one_core': (
            statistics.median(cpu_values) if cpu_values else None),
        'peak_rss_mb': max(rss_values, default=0.0),
    }


def _run_startup(
        output_root: Path, implementation: str, overlay_prefix: Path | None,
        attempt_index: int, domain_id: int,
        timeout_s: float) -> dict[str, Any]:
    configure_evaluation_rmw(implementation, overlay_prefix)
    run_id = f'startup_{implementation}__attempt_{attempt_index:02d}'
    run_dir = output_root / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    environment = _environment(run_id, domain_id)
    rmw_evidence = evaluation_rmw_evidence(environment)
    launched = []
    sampler = None
    started_s = time.monotonic()
    failure = None
    infrastructure_classification = None
    active_node_count = 0
    ready_topic_count = 0
    try:
        gazebo = _start([
            'ros2', 'launch', 'jdamr_cube_gazebo', 'gazebo.launch.py',
            f'world:={ASSETS / "slam_corridor_contact.world"}',
            f'urdf_file:={ASSETS / "jdamr_cube_nav_eval.urdf"}',
            'gui:=false', 'enable_image_bridges:=false',
            f'seed:={attempt_index}', 'x_pose:=-8.0', 'y_pose:=0.0',
            'z_pose:=0.01',
        ], run_dir / 'gazebo.log', environment)
        launched.append(gazebo)
        _wait_topics(
            {'/scan', '/odom', '/ground_truth_pose'}, environment, timeout_s)
        _wait_scan(environment, timeout_s)
        navigation = _start([
            'ros2', 'launch', 'jdamr_cube_navigation',
            'navigation.launch.py',
            f'map:={ASSETS / "slam_corridor_eval.yaml"}',
            f'params_file:={output_root / "nav2_obstacle_eval.params.yaml"}',
            'use_keepout:=false', 'use_sim_time:=true',
            'use_composition:=True',
        ], run_dir / 'navigation.log', environment)
        launched.append(navigation)
        resource_path = run_dir / 'navigation_resources.jsonl'
        sampler = subprocess.Popen([
            sys.executable, str(RESOURCE_SAMPLER),
            '--process-group', str(navigation[0].pid),
            '--output', str(resource_path), '--interval-s', '0.5',
        ], env=environment)
        _wait_topics(
            REQUIRED_TOPICS, environment, timeout_s,
            run_dir / 'navigation.log')
        ready_topic_count = len(REQUIRED_TOPICS)
        _wait_lifecycle_active(NAVIGATION_NODES, environment, timeout_s)
        active_node_count = len(NAVIGATION_NODES)
        _wait_tf_available(
            'map', 'base_footprint', environment, timeout_s,
            run_dir / 'tf_readiness.log')
    except InfrastructureInvalid as error:
        infrastructure_classification = str(error)
        failure = f'{type(error).__name__}: {error}'
    except Exception as error:
        failure = f'{type(error).__name__}: {error}'
    finally:
        for process, _ in reversed(launched):
            _stop(process)
        for _, stream in launched:
            stream.close()
        if sampler is not None:
            try:
                sampler.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                sampler.terminate()
                sampler.wait(timeout=5.0)
        survivors = _cleanup_identity(run_id, domain_id)
    resource_path = run_dir / 'navigation_resources.jsonl'
    result = {
        'schema_version': 2,
        'run_id': run_id,
        'process_topology': 'component_container_isolated',
        'attempt_index': attempt_index,
        'domain_id': domain_id,
        'evaluation_rmw': rmw_evidence,
        'status': 'PASS' if failure is None and not survivors else 'FAIL',
        'startup_elapsed_s': time.monotonic() - started_s,
        'failure': failure,
        'infrastructure_classification': infrastructure_classification,
        'active_node_count': active_node_count,
        'ready_topic_count': ready_topic_count,
        'timeout_count': int(
            failure is not None and 'timeout' in failure.lower()),
        'identity_survivors': survivors,
        'resource_summary': (
            _resource_summary(resource_path)
            if resource_path.is_file() else None),
    }
    (run_dir / 'evidence.json').write_text(
        json.dumps(result, indent=2, sort_keys=True) + '\n')
    return result


def summarize(results: list[dict[str, Any]], attempts_per_rmw: int) -> dict:
    """Require both preregistered samples to meet one readiness contract."""
    implementations = {}
    for implementation in RMW_IMPLEMENTATIONS:
        selected = [item for item in results
                    if item['evaluation_rmw']['requested_identifier']
                    == implementation]
        implementations[implementation] = {
            'attempt_count': len(selected),
            'pass_count': sum(item['status'] == 'PASS' for item in selected),
            'timeout_count': sum(item['timeout_count'] for item in selected),
            'survivor_count': sum(
                len(item['identity_survivors']) for item in selected),
            'infrastructure_invalid_count': sum(
                item['infrastructure_classification'] is not None
                for item in selected),
        }
    complete = all(
        item['attempt_count'] == attempts_per_rmw
        for item in implementations.values())
    all_ready = complete and all(
        item['pass_count'] == attempts_per_rmw
        and item['timeout_count'] == 0
        and item['survivor_count'] == 0
        for item in implementations.values())
    return {
        'schema_version': 2,
        'status': 'PASS' if all_ready else 'FAIL',
        'process_topology': 'component_container_isolated',
        'attempts_per_rmw': attempts_per_rmw,
        'required_active_node_count': len(NAVIGATION_NODES),
        'required_ready_topic_count': len(REQUIRED_TOPICS),
        'implementations': implementations,
        'results': results,
    }


def main() -> int:
    """Run an alternating FastDDS/CycloneDDS composed-startup comparison."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--base-domain-id', type=int, required=True)
    parser.add_argument('--attempts-per-rmw', type=int, default=20)
    parser.add_argument('--cyclone-overlay-prefix', type=Path, required=True)
    parser.add_argument('--timeout-s', type=float, default=60.0)
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=False)
    prepare(args.output_root)
    attempts = startup_matrix(
        args.base_domain_id, args.attempts_per_rmw,
        args.cyclone_overlay_prefix)
    results = [_run_startup(
        args.output_root, item['implementation'], item['overlay_prefix'],
        item['attempt_index'], item['domain_id'], args.timeout_s)
        for item in attempts]
    summary = summarize(results, args.attempts_per_rmw)
    (args.output_root / 'summary.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n')
    print(json.dumps(summary['implementations'], sort_keys=True))
    return 0 if summary['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
