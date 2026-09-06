#!/usr/bin/env python3
"""Run and seal the G005 15-case frontier-policy runtime matrix."""

from __future__ import annotations

import argparse
import hashlib
import math
import os
from pathlib import Path
import re
import tempfile

from evaluate_frontier_policy import coverage_metrics
from frontier_policy_contract import (
    canonical_json_bytes,
    decision_token,
    file_identity,
    FULL_PLAN,
    neutral_sample,
    policy_rank,
    unreachable_outcome as classify_unreachable_outcome,
    validate_planner_batch,
)
from generate_frontier_policy_assets import validate_assets


CLAIM_SCOPE = 'ACTUAL_GAZEBO_NAV2_FROZEN_PLANNER_FRONTIER_EVALUATION'
FULL_RUNTIME_PATH_ENABLED = False
FULL_RUNTIME_BLOCKER = 'BUILTIN_ACTUAL_ROS_RUNTIME_ADAPTER_NOT_IMPLEMENTED'
SIMULATION_HORIZON_S = 900.0
FINAL_ZERO_HOLD_S = 1.0
RUNTIME_PROOF_KEYS = {
    'schema_version', 'claim_scope', 'run_id', 'policy', 'layout_seed',
    'status', 'request_sha256', 'asset_identity', 'shared_initial_sha256',
    'shared_reveal_sha256', 'shared_runtime_sha256', 'simulation_horizon_s',
    'runtime_identity', 'decisions', 'coverage_samples', 'gt_pose_samples',
    'planner_batch_timeline',
    'contact_count', 'command_authority', 'tf_authority', 'lifecycle',
    'navigation_outcomes', 'resources', 'runner_cancellation_count',
    'production_inputs_before', 'production_inputs_after', 'teardown',
}


def _finite(value: object) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _lower_sha256(value: object) -> bool:
    return type(value) is str and re.fullmatch(r'[0-9a-f]{64}', value) is not None


def _sha256_json(value: object) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def _regular_identity_valid(value: object) -> bool:
    return (type(value) is dict and set(value) == {
        'path', 'size_bytes', 'sha256'} and
        type(value['path']) is str and bool(value['path']) and
        type(value['size_bytes']) is int and value['size_bytes'] >= 0 and
        _lower_sha256(value['sha256']))


def _validate_decision(value: object, run_id: str, policy: str,
                       layout_seed: int) -> list[dict]:
    keys = {
        'planner_batch_attempt_index', 'map_sequence',
        'map_payload_sha256', 'start_record',
        'blacklist_ids', 'candidate_sampling', 'decision_token',
        'costmap_sha256_before', 'costmap_sha256_after', 'planner_action',
        'planner_results', 'eligible_candidates', 'selected_goal',
        'stale',
    }
    if (type(value) is not dict or set(value) != keys or
            type(value['planner_batch_attempt_index']) is not int or
            value['planner_batch_attempt_index'] <= 0 or
            type(value['map_sequence']) is not int or
            value['map_sequence'] <= 0 or value['stale'] is not False or
            value['planner_action'] != {
                'type': 'nav2_msgs/action/ComputePathToPose',
                'planner_id': 'GridBased', 'use_start': True,
                'timeout_s': 2.0, 'execution': 'ACTUAL_NAV2_ACTION'}):
        raise ValueError('G005 runtime decision schema drift')
    start = value['start_record']
    if (type(start) is not dict or set(start) != {
            'frame_id', 'stamp_ns', 'pose_xy_yaw'} or
            start['frame_id'] != 'map' or type(start['stamp_ns']) is not int or
            start['stamp_ns'] <= 0 or type(start['pose_xy_yaw']) is not list or
            len(start['pose_xy_yaw']) != 3 or
            any(not _finite(item) for item in start['pose_xy_yaw']) or
            not _lower_sha256(value['map_payload_sha256']) or
            not _lower_sha256(value['decision_token']) or
            not _lower_sha256(value['costmap_sha256_before']) or
            not _lower_sha256(value['costmap_sha256_after']) or
            value['costmap_sha256_before'] !=
            value['costmap_sha256_after']):
        raise ValueError('G005 runtime decision identity schema drift')
    sampling = value['candidate_sampling']
    if (type(sampling) is not dict or set(sampling) != {
            'raw_candidate_ids', 'excluded_candidate_ids',
            'selected_candidate_ids', 'omitted_candidate_ids'}):
        raise ValueError('G005 runtime candidate sampling drift')
    if any(type(items) is not list or any(
            type(item) is not int for item in items)
           for items in sampling.values()):
        raise ValueError('G005 runtime candidate identifiers drift')
    raw = sampling['raw_candidate_ids']
    excluded = sampling['excluded_candidate_ids']
    selected = sampling['selected_candidate_ids']
    if (raw != sorted(set(raw)) or excluded != sorted(set(excluded)) or
            not set(excluded).issubset(raw) or set(excluded) & set(selected)):
        raise ValueError('G005 runtime blacklist evidence drift')
    if value['blacklist_ids'] != excluded:
        raise ValueError('G005 runtime blacklist exclusion is not bound')
    expected_sample = neutral_sample(
        layout_seed, sorted(set(raw) - set(excluded)))
    if (selected != expected_sample['selected_candidate_ids'] or
            sampling['omitted_candidate_ids'] !=
            expected_sample['omitted_candidate_ids']):
        raise ValueError('G005 runtime neutral sample drift')
    token = decision_token(
        run_id, policy, value['map_sequence'], value['map_payload_sha256'],
        value['start_record'], excluded, selected)
    if token != value['decision_token']:
        raise ValueError('G005 runtime decision token drift')
    eligible = validate_planner_batch(
        value['planner_results'], token, value['costmap_sha256_before'],
        value['costmap_sha256_after'])
    if eligible != value['eligible_candidates'] or not eligible:
        raise ValueError('G005 runtime planner batch drift')
    start_xy = value['start_record'].get('pose_xy_yaw', [])[:2]
    if (len(start_xy) != 2 or
            [item['candidate']['cell_index']
             for item in value['planner_results']] != selected or
            any(item['poses'][0] != start_xy
                for item in value['planner_results'])):
        raise ValueError('G005 runtime frozen planner start drift')
    ranked = policy_rank(policy, [dict(item) for item in eligible])
    if value['selected_goal'] != ranked[0]:
        raise ValueError('G005 runtime selected frontier drift')
    return ranked


def _validate_coverage(value: object) -> dict:
    if type(value) is not dict or set(value) != {
            'reachable_denominator_cells', 'samples'}:
        raise ValueError('G005 runtime coverage schema drift')
    metrics = coverage_metrics(
        value['samples'], value['reachable_denominator_cells'])
    if metrics['elapsed_s'] > SIMULATION_HORIZON_S or (
            metrics['final_coverage_ratio'] < 0.85):
        raise ValueError('G005 runtime coverage gate failed')
    return metrics


def _gt_metrics(value: object, coverage_t85_s: float) -> dict:
    keys = {'steady_elapsed_s', 'sim_elapsed_s', 'x_m', 'y_m',
            'clearance_m'}
    if (type(value) is not list or len(value) < 2 or
            any(type(item) is not dict or set(item) != keys or
                any(not _finite(item[key]) for key in keys) or
                item['steady_elapsed_s'] < 0.0 or
                not 0.0 <= item['sim_elapsed_s'] <= SIMULATION_HORIZON_S or
                item['clearance_m'] < 0.0
                for item in value)):
        raise ValueError('G005 runtime GT pose trace drift')
    if any(right['steady_elapsed_s'] <= left['steady_elapsed_s'] or
           right['sim_elapsed_s'] <= left['sim_elapsed_s']
           for left, right in zip(value, value[1:])):
        raise ValueError('G005 runtime GT pose time is not monotonic')
    if not value[0]['sim_elapsed_s'] <= coverage_t85_s <= value[-1][
            'sim_elapsed_s']:
        raise ValueError('G005 GT trace does not bracket 85 percent coverage')
    distance = 0.0
    if coverage_t85_s > value[0]['sim_elapsed_s']:
        for left, right in zip(value, value[1:]):
            if coverage_t85_s >= right['sim_elapsed_s']:
                distance += math.hypot(right['x_m'] - left['x_m'],
                                       right['y_m'] - left['y_m'])
                if coverage_t85_s == right['sim_elapsed_s']:
                    break
                continue
            ratio = ((coverage_t85_s - left['sim_elapsed_s']) /
                     (right['sim_elapsed_s'] - left['sim_elapsed_s']))
            interpolated_x = left['x_m'] + ratio * (
                right['x_m'] - left['x_m'])
            interpolated_y = left['y_m'] + ratio * (
                right['y_m'] - left['y_m'])
            distance += math.hypot(interpolated_x - left['x_m'],
                                   interpolated_y - left['y_m'])
            break
    return {'gt_path_length_to_85_m': distance,
            'minimum_clearance_m': min(item['clearance_m'] for item in value)}


def _validate_planner_batch_timeline(value: object,
                                     decisions: list[dict]) -> dict:
    keys = {'attempt_index', 'map_sequence', 'map_payload_sha256',
            'start_record_sha256', 'candidate_sha256', 'fresh',
            'reachable_count'}
    if (type(value) is not list or not value or
            any(type(item) is not dict or set(item) != keys
                for item in value)):
        raise ValueError('G005 planner batch timeline schema drift')
    if [item['attempt_index'] for item in value] != list(
            range(1, len(value) + 1)):
        raise ValueError('G005 planner batch timeline index drift')
    if any(type(item['map_sequence']) is not int or
           item['map_sequence'] <= 0 or item['fresh'] is not True or
           type(item['reachable_count']) is not int or
           item['reachable_count'] < 0 or
           any(not _lower_sha256(item[key]) for key in (
               'map_payload_sha256', 'start_record_sha256',
               'candidate_sha256')) for item in value):
        raise ValueError('G005 planner batch timeline identity drift')
    if any(right['map_sequence'] <= left['map_sequence']
           for left, right in zip(value, value[1:])):
        raise ValueError('G005 planner batch map sequence is not fresh')
    decision_indices = [item['planner_batch_attempt_index']
                        for item in decisions]
    reachable_indices = [item['attempt_index'] for item in value
                         if item['reachable_count'] > 0]
    if decision_indices != reachable_indices:
        raise ValueError('G005 planner batch decision ordered binding drift')
    by_attempt = {item['planner_batch_attempt_index']: item
                  for item in decisions}
    for item in value:
        decision = by_attempt.get(item['attempt_index'])
        if item['reachable_count'] == 0:
            if decision is not None:
                raise ValueError('G005 unreachable planner batch selected a goal')
            continue
        if decision is None:
            raise ValueError('G005 reachable planner batch was omitted')
        sampling = decision['candidate_sampling']['selected_candidate_ids']
        if (item['reachable_count'] != len(decision['eligible_candidates']) or
                item['map_sequence'] != decision['map_sequence'] or
                item['map_payload_sha256'] !=
                decision['map_payload_sha256'] or
                item['start_record_sha256'] !=
                _sha256_json(decision['start_record']) or
                item['candidate_sha256'] != _sha256_json(sampling)):
            raise ValueError('G005 planner batch timeline binding drift')
    unresolved = False
    for offset in range(len(value) - 2):
        window = value[offset:offset + 3]
        if not all(item['reachable_count'] == 0 for item in window):
            continue
        converted = [{
            'map_sha256': item['map_payload_sha256'],
            'start_sha256': item['start_record_sha256'],
            'candidate_sha256': item['candidate_sha256'],
            'fresh': item['fresh'],
            'reachable_count': item['reachable_count']} for item in window]
        if classify_unreachable_outcome(converted)['status'] != (
                'UNRESOLVED_FRONTIERS'):
            raise ValueError('G005 all-unreachable retry identity drift')
        if offset + 3 != len(value):
            raise ValueError('G005 planner timeline continued after terminal')
        unresolved = True
    trailing_unreachable = 0
    for item in reversed(value):
        if item['reachable_count'] != 0:
            break
        trailing_unreachable += 1
    status = ('UNRESOLVED_FRONTIERS' if unresolved else
              ('PENDING' if trailing_unreachable else 'NOT_OBSERVED'))
    return {'status': status,
            'result': ('FAIL' if unresolved else 'NOT_EVALUATED'),
            'exploration_complete': False, 'blacklist_exhausted': False}


def _validate_runtime_proof_shape(value: object, policy: str,
                                  layout_seed: int) -> dict:
    """Validate the dormant runtime proof schema without enabling it."""
    run_id = f'{policy}__seed_{layout_seed}'
    if (type(value) is not dict or set(value) != RUNTIME_PROOF_KEYS or
            value['schema_version'] != 1 or value['claim_scope'] !=
            CLAIM_SCOPE or value['run_id'] != run_id or
            value['policy'] != policy or value['layout_seed'] != layout_seed or
            value['status'] not in ('PASS', 'FAIL') or
            value['simulation_horizon_s'] != SIMULATION_HORIZON_S or
            not _regular_identity_valid(value['asset_identity'])):
        raise ValueError('G005 runtime proof schema drift')
    for key in ('request_sha256', 'shared_initial_sha256',
                'shared_reveal_sha256', 'shared_runtime_sha256'):
        if not _lower_sha256(value[key]):
            raise ValueError('G005 runtime proof hash drift')
    runtime = value['runtime_identity']
    if (type(runtime) is not dict or set(runtime) != {
            'gazebo_actual', 'nav2_actual', 'compute_path_action_actual',
            'oracle_reveal_map', 'ground_truth_localization',
            'not_slam_evaluation', 'synthetic_fixture'} or
            runtime != {
                'gazebo_actual': True, 'nav2_actual': True,
                'compute_path_action_actual': True,
                'oracle_reveal_map': True,
                'ground_truth_localization': True,
                'not_slam_evaluation': True,
                'synthetic_fixture': False}):
        raise ValueError('G005 runtime identity is not actual Gazebo/Nav2')
    if type(value['decisions']) is not list:
        raise ValueError('G005 runtime decision list schema drift')
    ranked = [_validate_decision(item, run_id, policy, layout_seed)
              for item in value['decisions']]
    unresolved = _validate_planner_batch_timeline(
        value['planner_batch_timeline'], value['decisions'])
    if unresolved['status'] == 'UNRESOLVED_FRONTIERS' and (
            value['status'] != 'FAIL'):
        raise ValueError('G005 all-unreachable history was laundered')
    if value['status'] == 'PASS' and unresolved['status'] != 'NOT_OBSERVED':
        raise ValueError('G005 PASS contains unresolved frontiers')
    if value['status'] == 'PASS' and not ranked:
        raise ValueError('G005 PASS has no reachable planner decision')
    coverage = _validate_coverage(value['coverage_samples'])
    gt = _gt_metrics(value['gt_pose_samples'], coverage['coverage_t85_s'])
    command = value['command_authority']
    if (type(command) is not dict or set(command) != {
            'topic', 'publisher_gids', 'publisher_nodes',
            'final_zero_hold_s', 'final_linear_x', 'final_angular_z'} or
            command['topic'] != '/cmd_vel' or
            len(command['publisher_gids']) != 1 or
            any(type(gid) is not str or not gid
                for gid in command['publisher_gids']) or
            command['publisher_nodes'] != ['/collision_monitor'] or
            not _finite(command['final_zero_hold_s']) or
            command['final_zero_hold_s'] < FINAL_ZERO_HOLD_S or
            command['final_linear_x'] != 0.0 or
            command['final_angular_z'] != 0.0):
        raise ValueError('G005 runtime final command authority drift')
    authority = value['tf_authority']
    if (type(authority) is not dict or set(authority) != {
            'map_to_odom_publisher_gids', 'map_to_odom_publisher_nodes'} or
            len(authority['map_to_odom_publisher_gids']) != 1 or
            any(type(gid) is not str or not gid
                for gid in authority['map_to_odom_publisher_gids']) or
            authority['map_to_odom_publisher_nodes'] !=
            ['/ground_truth_localization']):
        raise ValueError('G005 runtime TF authority drift')
    lifecycle = value['lifecycle']
    if (type(lifecycle) is not dict or not lifecycle or
            any(type(name) is not str or state != 'active'
                for name, state in lifecycle.items())):
        raise ValueError('G005 runtime lifecycle drift')
    navigation = value['navigation_outcomes']
    if (type(navigation) is not dict or set(navigation) != {
            'recovery_count', 'failure_count'} or
            any(type(navigation[key]) is not int or navigation[key] < 0
                for key in navigation)):
        raise ValueError('G005 runtime navigation outcome drift')
    resources = value['resources']
    if (type(resources) is not dict or set(resources) != {
            'cpu_seconds', 'peak_rss_bytes', 'samples'} or
            not _finite(resources['cpu_seconds']) or
            resources['cpu_seconds'] <= 0.0 or
            type(resources['peak_rss_bytes']) is not int or
            resources['peak_rss_bytes'] <= 0 or
            type(resources['samples']) is not int or
            resources['samples'] < 2):
        raise ValueError('G005 runtime resource evidence drift')
    before = value['production_inputs_before']
    after = value['production_inputs_after']
    if (type(before) is not dict or not before or before != after or
            any(not _regular_identity_valid(item) for item in before.values())):
        raise ValueError('G005 runtime production hash parity drift')
    teardown = value['teardown']
    if (type(teardown) is not dict or teardown != {
            'survivor_count': 0, 'finalized_before_teardown': True} or
            type(value['contact_count']) is not int or
            value['contact_count'] != 0 or
            type(value['runner_cancellation_count']) is not int or
            value['runner_cancellation_count'] != 0):
        raise ValueError('G005 runtime safety or teardown gate failed')
    return {'ranked': ranked, 'coverage': coverage, 'gt': gt,
            'unreachable_outcome': unresolved}


def validate_runtime_proof(value: object, policy: str,
                           layout_seed: int) -> dict:
    """Reject runtime evidence until the repository owns an actual adapter."""
    del value, policy, layout_seed
    if not FULL_RUNTIME_PATH_ENABLED:
        raise ValueError(FULL_RUNTIME_BLOCKER)
    raise ValueError(FULL_RUNTIME_BLOCKER)


def validate_paired_first_decisions(proofs: list[dict]) -> None:
    """Require identical first-decision inputs for three policies per seed."""
    if type(proofs) is not list or len(proofs) != 15:
        raise ValueError('G005 paired proof matrix is incomplete')
    for seed in sorted({seed for _, seed in FULL_PLAN}):
        if any(type(item) is not dict for item in proofs):
            raise ValueError('G005 paired proof schema drift')
        paired = [item for item in proofs if item.get('layout_seed') == seed]
        if {item.get('policy') for item in paired} != {
                'current', 'nearest', 'gain_nav'} or len(paired) != 3:
            raise ValueError('G005 paired policy set drift')
        if any(type(item.get('decisions')) is not list or
               not item['decisions'] for item in paired):
            raise ValueError('G005 paired first decision missing')
        decisions = [item['decisions'][0] for item in paired]
        fields = ('map_sequence', 'map_payload_sha256', 'start_record',
                  'blacklist_ids', 'candidate_sampling',
                  'costmap_sha256_before', 'costmap_sha256_after')
        if any(type(decision) is not dict or
               not set(fields).issubset(decision) for decision in decisions):
            raise ValueError('G005 paired first decision schema drift')
        if any(
                len({_sha256_json(decision[field]) for decision in decisions})
                != 1 for field in fields):
            raise ValueError('G005 paired first decision parity drift')


def compact_runtime_proof(value: dict) -> dict:
    """Derive the exact compact full-run record from sealed runtime proof."""
    if not FULL_RUNTIME_PATH_ENABLED:
        raise ValueError(FULL_RUNTIME_BLOCKER)
    derived = _validate_runtime_proof_shape(
        value, value['policy'], value['layout_seed'])
    coverage = derived['coverage']
    gt = derived['gt']
    decisions = value['decisions']
    planning_rejects = sum(
        len(item['candidate_sampling']['selected_candidate_ids']) -
        len(item['eligible_candidates']) for item in decisions)
    score_records = []
    for item in derived['ranked'][0]:
        gains = [row['gain_cells'] for row in derived['ranked'][0]]
        lengths = [row['nav_length_m'] for row in derived['ranked'][0]]
        gain_span = max(gains) - min(gains)
        length_span = max(lengths) - min(lengths)
        score_records.append({
            **item,
            'gain_norm': ((item['gain_cells'] - min(gains)) / gain_span
                          if gain_span else 0.0),
            'length_norm': ((item['nav_length_m'] - min(lengths)) /
                            length_span if length_span else 0.0),
        })
    runtime_status = value['status']
    hard_pass = (runtime_status == 'PASS' and
                 gt['minimum_clearance_m'] >= 0.05)
    return {
        'schema_version': 1, 'run_id': value['run_id'], 'mode': 'full',
        'policy': value['policy'], 'layout_seed': value['layout_seed'],
        'status': 'PASS' if hard_pass else 'FAIL',
        'first_goal_cell_index': decisions[0]['selected_goal']['cell_index'],
        **coverage, **gt, 'planning_reject_count': planning_rejects,
        'recovery_count': value['navigation_outcomes']['recovery_count'],
        'failure_count': value['navigation_outcomes']['failure_count'],
        'score_decompositions': score_records,
        'contact_count': value['contact_count'],
        'cpu_seconds': value['resources']['cpu_seconds'],
        'peak_rss_bytes': value['resources']['peak_rss_bytes'],
        'final_zero': True, 'map_to_odom_authority_count': 1,
        'cmd_vel_publisher_count': 1,
        'runner_cancellation_count': value['runner_cancellation_count'],
        'lifecycle_active': True, 'initial_parity': True,
        'input_parity': True, 'hash_parity': True,
        'production_hashes_unchanged': True,
        'first_decision_reachable_count':
        len(decisions[0]['eligible_candidates']),
        'survivor_count': value['teardown']['survivor_count'],
    }


def build_execution_plan(asset_root: Path, base_domain_id: int) -> dict:
    """Create the immutable 15-run request plan without starting ROS."""
    assets = validate_assets(asset_root, 'full')
    if (type(base_domain_id) is not int or base_domain_id < 20 or
            base_domain_id + len(FULL_PLAN) - 1 > 232):
        raise ValueError('G005 domain block must fit 20..232')
    source_identities = {
        name: file_identity(Path(path).resolve()) for name, path in {
            'contract': __import__('frontier_policy_contract').__file__,
            'evaluator': __import__('evaluate_frontier_policy').__file__,
            'runner': __file__,
        }.items()}
    shared_runtime_sha = _sha256_json(source_identities)
    requests = []
    for offset, (policy, seed) in enumerate(FULL_PLAN):
        layout_record = assets['layouts'][str(seed)]
        shared_initial = _sha256_json({
            'layout_seed': seed,
            'start_cell_index': layout_record['start_cell_index'],
            'gt_occupancy_sha256': layout_record['gt_occupancy_sha256'],
        })
        shared_reveal = _sha256_json({
            'layout_seed': seed,
            'reveal': assets['layout_contract']['reveal'],
        })
        request = {
            'schema_version': 1, 'run_id': f'{policy}__seed_{seed}',
            'policy': policy, 'layout_seed': seed,
            'ros_domain_id': base_domain_id + offset,
            'simulation_horizon_s': SIMULATION_HORIZON_S,
            'asset_root': str(asset_root.resolve()),
            'asset_identity': file_identity(
                asset_root / f'layout_{seed}_gt.json'),
            'shared_initial_sha256': shared_initial,
            'shared_reveal_sha256': shared_reveal,
            'shared_runtime_sha256': shared_runtime_sha,
            'required_runtime_claim': CLAIM_SCOPE,
        }
        request['request_sha256'] = _sha256_json(request)
        requests.append(request)
    return {'schema_version': 1, 'mode': 'full',
            'claim_scope': 'G005_EXECUTION_PLAN_NO_RUNTIME_CLAIM',
            'runtime_path': {
                'enabled': FULL_RUNTIME_PATH_ENABLED,
                'status': 'PENDING', 'blocker': FULL_RUNTIME_BLOCKER,
                'external_adapter_results_accepted': False},
            'asset_manifest': file_identity(
                asset_root / 'asset_manifest.json'),
            'requests': requests}


def _write_atomic_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
            prefix=f'.{path.name}.', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(canonical_json_bytes(value))
        stream.flush()
        os.fsync(stream.fileno())
    try:
        temporary.replace(path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def run_full(asset_root: Path, output_root: Path, command: list[str],
             base_domain_id: int, timeout_s: float) -> dict:
    """Reject every external adapter until an owned ROS path exists."""
    del asset_root, output_root, command, base_domain_id, timeout_s
    raise RuntimeError(FULL_RUNTIME_BLOCKER)


def main() -> int:
    """Plan or execute the exact G005 full matrix."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--asset-root', required=True, type=Path)
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--base-domain-id', type=int, default=100)
    parser.add_argument('--timeout-s', type=float, default=1200.0)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('runtime_command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.timeout_s <= SIMULATION_HORIZON_S:
        raise ValueError('G005 wall timeout must exceed the simulation horizon')
    command = args.runtime_command
    if command[:1] == ['--']:
        command = command[1:]
    if args.dry_run:
        if command:
            raise ValueError('G005 dry-run does not accept a runtime command')
        if args.output_root.exists() or args.output_root.is_symlink():
            raise ValueError('G005 dry-run output must be absent')
        _write_atomic_json(
            args.output_root,
            build_execution_plan(args.asset_root, args.base_domain_id))
        return 0
    parser.error(FULL_RUNTIME_BLOCKER)


if __name__ == '__main__':
    raise SystemExit(main())
