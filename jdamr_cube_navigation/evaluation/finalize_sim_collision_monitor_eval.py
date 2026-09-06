#!/usr/bin/env python3
"""Create an exact-set, hash-bound G004 matrix summary."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import statistics
from typing import Any

from evaluate_sim_collision_monitor import aggregate, strict_json_loads

from sim_collision_monitor_contract import scenario_matrix, sha256_file


def metric(values: list[float]) -> dict[str, float]:
    """Return compact min, median, and max statistics."""
    return {
        'min': min(values),
        'median': statistics.median(values),
        'max': max(values),
    }


def finalize(root: Path) -> dict[str, Any]:
    """Re-evaluate exact inputs and bind all retained evidence by hash."""
    expected = Counter(item['run_id'] for item in scenario_matrix())
    paths = sorted(root.glob('*/evidence.json'))
    actual = Counter(path.parent.name for path in paths)
    if actual != expected or len(paths) != 15:
        raise ValueError('evidence path set is not the exact 15-run matrix')
    result = aggregate(paths)
    if result['status'] != 'PASS':
        raise ValueError('matrix must pass before finalization')
    documents = [strict_json_loads(path.read_text()) for path in paths]
    runtime_path = root / 'runtime_manifest.json'
    runtime = strict_json_loads(runtime_path.read_text())
    harness_sources = runtime['harness_sources']
    for record in harness_sources.values():
        source = Path(record['path'])
        if (not source.is_file()
                or source.stat().st_size != record['size_bytes']
                or sha256_file(source) != record['sha256']):
            raise ValueError('harness source changed after matrix execution')
    scenarios = {}
    for scenario in sorted({item['scenario'] for item in documents}):
        selected = [item for item in documents
                    if item['scenario'] == scenario]
        record: dict[str, Any] = {
            'run_count': len(selected),
            'contact_count_total': sum(
                item['contact_count'] for item in selected),
            'goal_cancel_count_total': sum(
                item['goal_cancel_count'] for item in selected),
            'survivor_count_total': sum(
                len(item['identity_survivors']) for item in selected),
            'minimum_observed_scan_range_m': metric([
                item['minimum_observed_scan_range_m']
                for item in selected]),
        }
        if scenario == 'sudden_obstacle_stop_resume':
            record['footprint_to_obstacle_clearance_m'] = metric([
                item['footprint_to_obstacle_clearance_m']
                for item in selected])
            key = 'scan_publish_to_zero_receive_steady_s'
            record[key] = metric([
                strict_json_loads(Path(
                    item['scan_gate_evidence']['path']).read_text())[key]
                for item in selected])
        if scenario == 'scan_timeout_stop_resume':
            record['freeze_ack_to_zero_ros_s'] = metric([
                item['freeze_ack_to_zero_ros_s'] for item in selected])
            record['freeze_ack_to_zero_steady_s'] = metric([
                item['freeze_ack_to_zero_steady_s'] for item in selected])
        if scenario != 'clear_baseline':
            record['stop_distance_m'] = metric([
                item['stop_distance_m'] for item in selected])
        if scenario == 'scan_timeout_stop_resume':
            record['source_age_s'] = metric([
                (item['stop_state_ros_ns']
                 - item['last_monitor_scan_stamp_ns_at_freeze']) / 1e9
                for item in selected])
        scenarios[scenario] = record
    resources = {}
    for group in ('gazebo', 'nav2'):
        records = [item['resource_evidence'][group] for item in documents]
        resources[group] = {
            'max_rss_mb': max(item['max_rss_mb'] for item in records),
            'median_cpu_pct_one_core': metric([
                item['median_cpu_pct_one_core'] for item in records]),
            'p95_cpu_pct_one_core': metric([
                item['p95_cpu_pct_one_core'] for item in records]),
            'sample_count_total': sum(
                item['sample_count'] for item in records),
            'trace_size_bytes_total': sum(
                item['size_bytes'] for item in records),
        }
    records = []
    for path in paths:
        evaluation_path = path.parent / 'evaluation.json'
        for kind, source in (
                ('evidence', path), ('evaluation', evaluation_path)):
            records.append({
                'kind': kind,
                'run_id': path.parent.name,
                'relative_path': str(source.relative_to(root)),
                'size_bytes': source.stat().st_size,
                'sha256': sha256_file(source),
            })
    return {
        'schema_version': 1,
        'matrix': {
            'status': result['status'], 'run_count': result['run_count'],
            'passed': result['passed'], 'failed': result['failed'],
        },
        'scenarios': scenarios,
        'resources': resources,
        'representative_policy': {
            'seed': 11,
            'selection': 'predeclared_first_seed_not_post_hoc_cherry_pick',
        },
        'records': records,
        'harness_sources': harness_sources,
        'runtime_manifest_input_sha256': sha256_file(runtime_path),
        'aggregate_input_sha256': sha256_file(root / 'aggregate.json'),
        'contract_input_sha256': sha256_file(root / 'contract.json'),
    }


def main() -> int:
    """Write one finalized matrix summary without changing run evidence."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    document = finalize(args.root)
    args.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
