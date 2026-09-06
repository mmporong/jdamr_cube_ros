#!/usr/bin/env python3
"""Evaluate one or more fixed-obstacle Nav2 JSON evidence documents."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from typing import Any

from sim_nav_obstacle_contract import (  # noqa: I101
    classify_terminal, path_clears_obstacle, scenario_matrix, SCENARIOS,
    validate_removal_sequence)


def sha256_file(path: Path) -> str:
    """Return a file SHA-256 digest."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _clear_trace_has_final_pair(trace: list[dict[str, Any]],
                                source: str) -> bool:
    """Require two adjacent authoritative clear samples for one source."""
    samples = [item for item in trace if item.get('source') == source]
    accepted_decisions = {'increment', 'diagnostic_background_excess'}
    for first, second in zip(samples, samples[1:]):
        if (first.get('decision') in accepted_decisions
                and first.get('counter_before') == 0
                and first.get('counter_after') == 1
                and second.get('decision') in accepted_decisions
                and second.get('counter_before') == 1
                and second.get('counter_after') == 2
                and first.get('observable') is True
                and second.get('observable') is True
                and not first.get('surface_probe', {}).get('blocking', True)
                and first.get('padded_planning_probe', {}).get(
                    'influence_excess_count') == 0
                and first.get('padded_planning_probe', {}).get(
                    'baseline_excess_count') is not None
                and first.get('padded_planning_probe', {}).get(
                    'influence_probe', {}).get('sampled_cell_count', 0) > 0
                and first.get('padded_planning_probe', {}).get(
                    'influence_probe', {}).get('unknown_count') == 0
                and not second.get('surface_probe', {}).get('blocking', True)
                and second.get('padded_planning_probe', {}).get(
                    'influence_excess_count') == 0
                and second.get('padded_planning_probe', {}).get(
                    'baseline_excess_count') is not None
                and second.get('padded_planning_probe', {}).get(
                    'influence_probe', {}).get('sampled_cell_count', 0) > 0
                and second.get('padded_planning_probe', {}).get(
                    'influence_probe', {}).get('unknown_count') == 0):
            return True
    return False


def evaluate_run(document: dict[str, Any]) -> dict[str, Any]:
    """Apply the fail-closed G003 gates to one run document."""
    scenario = document['scenario']
    contract = document['contract']
    events = [item['name'] for item in document.get('events', [])]
    final_zero_hold_s = document.get('final_zero_hold_s')
    checks = {
        'global_and_local_blocking': True,
        'global_and_local_cleared': True,
        'path_clearance': True,
        'contact_zero': document.get('contact_count', -1) == 0,
        'contact_scope_matches_contract': (
            document.get('contact_scope') == contract.get('contact_scope')
            and contract.get('contact_scope', {}).get(
                'collision_geometry_preserved') is True
            and contract.get('contact_scope', {}).get(
                'excluded_collision_elements') == []),
        'final_cmd_vel_zero': document.get('final_cmd_vel_zero') is True,
        'final_zero_hold': (
            isinstance(final_zero_hold_s, (float, int))
            and final_zero_hold_s
            >= contract['final_zero_hold_s']),
        'teardown_survivor_zero': (
            document.get('teardown', {}).get('identity_survivors') == []
            and document.get('teardown', {}).get(
                'remaining_process_groups') == []),
        'runner_did_not_cancel': not document.get('runner_cancelled', True),
        'runner_did_not_timeout': not document.get('runner_timed_out', True),
        'contact_smoke_passed': (
            document.get('contact_smoke_status') == 'PASS'),
        'observation_persistence_explicit': (
            document.get('observation_persistence_s')
            == contract['observation_persistence_s']),
        'harness_error_absent': 'harness_error' not in document,
        'activation_error_absent': document.get('activation_error') is None,
    }
    obstacle = SCENARIOS[scenario]['obstacle']
    if obstacle is not None:
        if scenario in {'detour', 'event_driven_removal'}:
            checks['global_and_local_blocking'] = (
                document.get('global_blocking') is True
                and document.get('local_blocking') is True)
        else:
            checks['global_blocking_after_eligible_scan'] = (
                document.get('global_blocking') is True)
        checks['preloaded_entity_identity_stable'] = (
            document.get('entity_identity_stable') is True)
        checks['contact_publisher_present'] = (
            document.get('contact_matched_publisher_count_max', 0) >= 1)
        distance_field = (
            'minimum_robot_obstacle_aabb_distance_before_deactivate_m'
            if scenario == 'event_driven_removal'
            else 'minimum_robot_obstacle_aabb_distance_m')
        aabb_distance_m = document.get(distance_field)
        checks['ground_truth_footprint_clearance'] = (
            isinstance(aabb_distance_m, (float, int))
            and aabb_distance_m
            >= contract['robot_circumscribed_radius_m'])
    if scenario in {
            'detour', 'event_driven_removal', 'full_block',
            'goal_occupied'}:
        activation_scan = document.get('activation_surface_scan') or {}
        activation_reference_ns = document.get(
            'activation_reference_scan_stamp_ns')
        checks['activation_scan_strictly_newer'] = (
            isinstance(activation_reference_ns, int)
            and activation_scan.get('stamp_ns', -1) > activation_reference_ns
            and activation_scan.get('surface_beam_count', 0) > 0)
        mark_scan = document.get('mark_eligible_scan') or {}
        checks['mark_eligible_scan'] = (
            mark_scan.get('surface_beam_count', 0) > 0
            and mark_scan.get('minimum_surface_range_m', float('inf'))
            <= contract['obstacle_mark_max_range_m'])
        pre_mark_paths = document.get('plans_before_mark', [])
        checks['pre_mark_plan_intersects'] = (
            bool(pre_mark_paths)
            and any(not path_clears_obstacle(
                path, obstacle, contract['path_clearance_m'])
                for path in pre_mark_paths))
        required_blocking_events = ['global_blocking']
        if scenario in {'detour', 'event_driven_removal'}:
            required_blocking_events.append('local_blocking')
        event_positions = {
            name: events.index(name) for name in (
                'activate_ack', 'activation_surface_scan', 'goal_accepted',
                'pre_mark_plan', 'mark_eligible_scan',
                *required_blocking_events)
            if name in events}
        blocking_positions = [event_positions[name]
                              for name in required_blocking_events
                              if name in event_positions]
        checks['blocking_causal_order'] = (
            len(event_positions) == 5 + len(required_blocking_events)
            and event_positions['activate_ack']
            < event_positions['activation_surface_scan']
            and event_positions['activate_ack']
            < event_positions['goal_accepted']
            <= event_positions['pre_mark_plan']
            < event_positions['mark_eligible_scan']
            < min(blocking_positions))
        if scenario in {'detour', 'event_driven_removal', 'full_block'}:
            checks['blocking_causal_order'] = (
                checks['blocking_causal_order']
                and 'activation_surface_scan' in event_positions
                and 'goal_accepted' in event_positions
                and event_positions['activation_surface_scan']
                < event_positions['goal_accepted'])
    if scenario == 'event_driven_removal':
        clear_mode = document.get('clear_mode')
        service_requests = document.get('clear_service_request_counts', {})
        service_responses = document.get(
            'clear_service_response_counts', {})
        clear_trace = document.get('clear_observation_trace', [])
        baseline = document.get('global_baseline_snapshots', [])
        mark_excess = document.get('mark_excess_snapshots', {})
        final_clear_s = document.get('final_clear_completion_elapsed_s')
        final_plan_s = document.get('final_post_clear_plan_elapsed_s')
        terminal_s = document.get('action_terminal_elapsed_s')
        passage = document.get('removal_passage') or {}
        clear_history = document.get('clear_completion_history', [])
        plan_history = document.get('post_clear_plan_history', [])
        checks['global_and_local_cleared'] = (
            document.get('global_cleared') is True
            and document.get('local_cleared') is True
            and document.get('new_scan_after_deactivate') is True
            and not validate_removal_sequence(events, clear_mode))
        checks['bounded_clear_policy'] = (
            (clear_mode == 'passive'
             and service_requests == {'global': 0, 'local': 0}
             and service_responses == {'global': 0, 'local': 0})
            or (clear_mode == 'bounded_recovery'
                and service_requests == {'global': 1, 'local': 1}
                and service_responses == {'global': 1, 'local': 1}))
        checks['clear_observation_trace'] = (
            {item.get('source') for item in clear_trace}
            == {'global', 'local'}
            and all({
                'stamp_ns', 'surface_probe', 'padded_planning_probe',
                'observable', 'counter_before', 'counter_after', 'decision',
            }.issubset(item) for item in clear_trace)
            and all(_clear_trace_has_final_pair(clear_trace, source)
                    for source in ('global', 'local')))
        checks['stable_global_baseline'] = (
            document.get('baseline_ready') is True
            and len(baseline) == 2
            and baseline[0].get('stamp_ns', 0) < baseline[1].get('stamp_ns', 0)
            and baseline[0].get('geometry') == baseline[1].get('geometry')
            and baseline[0].get('canonical_class_sha256')
            == baseline[1].get('canonical_class_sha256')
            and all(item.get('sampled_cell_count', 0) > 0
                    and item.get('unknown_count') == 0 for item in baseline))
        checks['mark_excess_over_baseline'] = all(
            mark_excess.get(source, {}).get('baseline_excess_count', 0) > 0
            and mark_excess.get(source, {}).get(
                'influence_roi_intersects') is True
            for source in ('global', 'local'))
        checks['exact_stamp_local_costmap_tf'] = (
            not document.get('pending_local_costmap_failures')
            and all(item.get('stamp_ns')
                    == item.get('padded_planning_probe', {}).get('tf_stamp_ns')
                    for item in clear_trace if item.get('source') == 'local'))
        passage_contract = contract.get('removal_passage', {})
        direction_sign = passage_contract.get('direction_sign', 0)
        robot_x_m = passage.get('robot_x_m')
        checks['far_side_passage'] = (
            direction_sign in {-1, 1}
            and isinstance(robot_x_m, (float, int))
            and direction_sign * (
                robot_x_m - passage_contract.get('far_edge_x_m', robot_x_m))
            > passage_contract.get('robot_radius_m', float('inf'))
            and 'far_side_passage' in events)
        checks['final_clear_plan_terminal_order'] = (
            all(isinstance(value, (float, int))
                for value in (final_clear_s, final_plan_s, terminal_s))
            and final_clear_s < final_plan_s < terminal_s
            and bool(clear_history) and bool(plan_history)
            and passage.get('epoch') == clear_history[-1].get('epoch')
            == plan_history[-1].get('epoch')
            and max((item.get('stamp_ns', -1) for item in
                     clear_history[-1].get('source_samples', {}).values()),
                    default=-1)
            < plan_history[-1].get('stamp_ns', -1)
            < passage.get('stamp_ns', -1)
            and max((item.get('receive_elapsed_s', float('inf'))
                     for item in clear_history[-1].get(
                         'source_samples', {}).values()),
                    default=float('inf'))
            < plan_history[-1].get('receive_elapsed_s', -1)
            < passage.get('receive_elapsed_s', -1)
            < terminal_s)
    paths = document.get('plans_after_mark', [])
    if scenario in {'full_block', 'goal_occupied'}:
        checks['path_clearance'] = True
    elif obstacle is not None and paths:
        checks['path_clearance'] = any(
            path_clears_obstacle(
                path, obstacle, contract['path_clearance_m'])
            for path in paths)
    elif scenario in {'detour', 'event_driven_removal'}:
        checks['path_clearance'] = False

    terminal_elapsed_s = document.get('terminal_elapsed_from_mark_s')
    terminal_class = classify_terminal(
        scenario,
        document.get('action_terminal', 'unknown'),
        (terminal_elapsed_s if isinstance(terminal_elapsed_s, (float, int))
         else float('inf')),
        document.get('runner_cancelled', True),
        contract['blocked_terminal_limit_s'])
    expected_class = ('BLOCKED' if SCENARIOS[scenario]['expected_terminal']
                      == 'blocked' else 'PASS')
    checks['expected_terminal'] = terminal_class == expected_class
    failures = [name for name, passed in checks.items() if not passed]
    return {
        'schema_version': 1,
        'run_id': document['run_id'],
        'scenario': scenario,
        'seed': document['seed'],
        'terminal_classification': terminal_class,
        'status': 'PASS' if not failures else 'FAIL',
        'checks': checks,
        'failures': failures,
        'cache_provenance': document.get('cache_provenance', {}),
    }


def aggregate(paths: list[Path]) -> dict[str, Any]:
    """Evaluate a complete matrix and record every source hash."""
    runs = []
    sources = []
    identities = []
    identity_errors = []
    for path in sorted(paths):
        document = json.loads(path.read_text(encoding='utf-8'))
        required = {'run_id', 'scenario', 'seed'}
        if not required <= document.keys():
            if (path.name == 'evidence.json'
                    and path.parent.name == 'scan_ab_preflight'):
                continue
            identity_errors.append(
                {'path': str(path.resolve()), 'reason': 'missing_identity'})
            continue
        identity = (
            document['run_id'], document['scenario'], document['seed'])
        identities.append(identity)
        canonical_copy = path.name == f"{document['run_id']}.json"
        raw_run_evidence = (
            path.name == 'evidence.json'
            and path.parent.name == document['run_id'])
        if not (canonical_copy or raw_run_evidence):
            identity_errors.append({
                'path': str(path.resolve()),
                'reason': 'filename_run_id_mismatch',
            })
        runs.append(evaluate_run(document))
        sources.append({
            'path': str(path.resolve()),
            'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path),
        })
    passed = sum(run['status'] == 'PASS' for run in runs)
    expected = [
        (item['run_id'], item['scenario'], item['seed'])
        for item in scenario_matrix()]
    expected_counts = Counter(expected)
    actual_counts = Counter(identities)
    missing = list((expected_counts - actual_counts).elements())
    unexpected = list((actual_counts - expected_counts).elements())
    duplicate = sorted(
        identity for identity, count in actual_counts.items() if count > 1)
    matrix_complete = (
        len(expected) == len(identities)
        and actual_counts == expected_counts
        and not identity_errors)
    all_runs_pass = passed == len(expected) == len(runs)
    return {
        'schema_version': 1,
        'status': 'PASS' if matrix_complete and all_runs_pass else 'FAIL',
        'matrix_complete': matrix_complete,
        'partial_smoke': not matrix_complete,
        'missing_run_ids': sorted(identity[0] for identity in missing),
        'unexpected_run_ids': sorted(identity[0] for identity in unexpected),
        'missing_identities': sorted(missing),
        'unexpected_identities': sorted(unexpected),
        'duplicate_identities': duplicate,
        'identity_errors': identity_errors,
        'run_count': len(runs),
        'passed': passed,
        'failed': len(runs) - passed,
        'sources': sources,
        'runs': runs,
        'claim_scope': (
            'Gazebo의 고정 장애물 재계획 평가이며 이동 물체·사람 안전·'
            '실차 충돌 안전을 입증하지 않는다.'),
    }


def main() -> int:
    """Evaluate JSON evidence files and write one aggregate document."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = aggregate(list(args.input_root.glob('*/evidence.json')))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(
            result, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    print(json.dumps({key: result[key] for key in (
        'status', 'run_count', 'passed', 'failed')}, sort_keys=True))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
