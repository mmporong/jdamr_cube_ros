#!/usr/bin/env python3
"""Strict evaluator for G005 paired frontier-policy artifacts."""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import re
import statistics

from frontier_policy_contract import (
    ARTIFACT_LIMIT_BYTES,
    canonical_json_bytes,
    decision_token,
    file_identity,
    FULL_PLAN,
    neutral_sample,
    policy_rank,
    RUN_OUTPUT_LIMIT_BYTES,
    strict_json_load,
    validate_planner_batch,
)

from generate_frontier_policy_assets import validate_assets


FULL_METRIC_FIELDS = (
    'final_coverage_ratio', 'coverage_t50_s', 'coverage_t70_s',
    'coverage_t85_s', 'elapsed_s', 'coverage_auc',
    'gt_path_length_to_85_m', 'planning_reject_count', 'recovery_count',
    'failure_count', 'score_decompositions', 'contact_count',
    'minimum_clearance_m', 'cpu_seconds', 'peak_rss_bytes', 'final_zero',
    'map_to_odom_authority_count', 'cmd_vel_publisher_count',
    'runner_cancellation_count', 'lifecycle_active', 'initial_parity',
    'input_parity', 'hash_parity', 'production_hashes_unchanged',
    'first_decision_reachable_count', 'survivor_count')
PROMOTION_RULES = {
    'all_15_runs_pass': True,
    'final_coverage_ratio_each_min': 0.85,
    'first_decision_divergence_seed_count_min': 3,
    'first_decision_reachable_count_each_min': 2,
    'gain_nav_path_ratio_le_1_seed_count_min': 4,
    'gain_nav_path_paired_median_ratio_max': 0.90,
    'paired_median_upper_ratio_max': 1.05,
    'gain_nav_auc_paired_median_ratio_min': 0.95,
    'gain_nav_clearance_paired_median_ratio_min': 0.95,
    'failure_and_recovery_no_regression': True,
    'nearest_policy_reference_only': True,
    'contact_count_each_max': 0,
    'minimum_clearance_m_each_min': 0.05,
    'survivor_count_each_max': 0,
}
FULL_RUN_KEYS = {'schema_version', 'run_id', 'mode', 'policy', 'layout_seed',
                 'validity', 'outcome', 'invalid_reasons',
                 'first_goal_cell_index', *FULL_METRIC_FIELDS}


def _finite_number(value) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _lower_sha256(value) -> bool:
    return (type(value) is str and
            re.fullmatch(r'[0-9a-f]{64}', value) is not None)


def _identity_matches_disk(value: object) -> bool:
    if type(value) is not dict or set(value) != {
            'path', 'size_bytes', 'sha256'}:
        return False
    path = Path(value['path']) if type(value['path']) is str else Path('')
    if not path.is_absolute():
        return False
    try:
        return value == file_identity(path)
    except (OSError, ValueError):
        return False


def _validate_execution_request(value: object) -> None:
    if type(value) is not dict or not _lower_sha256(
            value.get('request_sha256')):
        raise ValueError('G005 execution request schema drift')
    payload = dict(value)
    request_sha256 = payload.pop('request_sha256')
    if hashlib.sha256(canonical_json_bytes(
            payload)).hexdigest() != request_sha256:
        raise ValueError('G005 execution request hash drift')
    components = value.get('runtime_components')
    if (type(components) is not dict or not components or
            any(not _identity_matches_disk(item)
                for item in components.values())):
        raise ValueError('G005 runtime component disk drift')


def _ratio(numerator: float, denominator: float) -> float:
    if (not _finite_number(numerator) or not _finite_number(denominator) or
            denominator <= 0.0):
        raise ValueError(
            'paired ratio denominator must be finite and positive')
    return numerator / denominator


def coverage_metrics(samples: list[dict], denominator_cells: int,
                     horizon_s: float | None = None) -> dict:
    """Compute monotonic reachable coverage thresholds and normalized AUC."""
    if (type(denominator_cells) is not int or denominator_cells <= 0 or
            type(samples) is not list or len(samples) < 2):
        raise ValueError('coverage input schema drift')
    previous_time_s = -1.0
    previous_count = -1
    ratios = []
    for item in samples:
        if (type(item) is not dict or set(item) != {
                'elapsed_s', 'revealed_reachable_cells'} or
                not _finite_number(item['elapsed_s']) or
                item['elapsed_s'] < 0.0 or
                type(item['revealed_reachable_cells']) is not int or
                not previous_time_s < item['elapsed_s'] or
                not previous_count <= item['revealed_reachable_cells'] <=
                denominator_cells):
            raise ValueError('coverage samples are not monotonic')
        previous_time_s = item['elapsed_s']
        previous_count = item['revealed_reachable_cells']
        ratios.append(item['revealed_reachable_cells'] / denominator_cells)
    if (horizon_s is not None and
            (not _finite_number(horizon_s) or horizon_s <= 0.0 or
             samples[0]['elapsed_s'] != 0.0 or
             samples[-1]['elapsed_s'] != horizon_s)):
        raise ValueError('coverage samples do not span the exact horizon')
    thresholds = {}
    for target in (0.50, 0.70, 0.85):
        matches = [item['elapsed_s'] for item, ratio in zip(samples, ratios)
                   if ratio >= target]
        if not matches and horizon_s is None:
            raise ValueError('coverage threshold was not reached')
        thresholds[f'coverage_t{round(target * 100)}_s'] = (
            matches[0] if matches else None)
    elapsed_s = samples[-1]['elapsed_s']
    if elapsed_s <= 0.0:
        raise ValueError('coverage elapsed time must be positive')
    area = sum((right['elapsed_s'] - left['elapsed_s']) *
               (left_ratio + right_ratio) * 0.5
               for left, right, left_ratio, right_ratio in zip(
                   samples, samples[1:], ratios, ratios[1:]))
    return {**thresholds, 'elapsed_s': elapsed_s,
            'final_coverage_ratio': ratios[-1],
            'coverage_auc': area / elapsed_s}


def promotion_decision(runs: list[dict]) -> dict:
    """Apply the preregistered paired promotion rule or remain unevaluated."""
    expected = {(policy, seed) for policy, seed in FULL_PLAN}
    if ({(item.get('policy'), item.get('layout_seed')) for item in runs} !=
            expected or len(runs) != 15):
        return {'status': 'NOT_EVALUATED', 'promote_gain_nav': False,
                'rules': PROMOTION_RULES}
    if any(item.get('validity') != 'VALID' or
           item.get('outcome') != 'PASS' for item in runs):
        return {'status': 'FAIL', 'promote_gain_nav': False,
                'rules': PROMOTION_RULES}
    by_key = {(item['policy'], item['layout_seed']): item for item in runs}
    divergent = 0
    path_non_worse = 0
    ratios = {name: [] for name in (
        'gt_path_length_to_85_m', 'coverage_t50_s', 'coverage_t70_s',
        'coverage_t85_s', 'elapsed_s', 'cpu_seconds', 'peak_rss_bytes',
        'coverage_auc', 'minimum_clearance_m')}
    valid = all(_full_hard_gates(item) for item in runs)
    for seed in sorted({seed for _, seed in FULL_PLAN}):
        rows = [by_key[(policy, seed)] for policy in
                ('current', 'nearest', 'gain_nav')]
        if len({row['first_goal_cell_index'] for row in rows}) >= 2:
            divergent += 1
        current = rows[0]
        gain_nav = rows[2]
        for name in ratios:
            ratios[name].append(_ratio(gain_nav[name], current[name]))
        if ratios['gt_path_length_to_85_m'][-1] <= 1.0:
            path_non_worse += 1
        if (gain_nav['failure_count'] > current['failure_count'] or
                gain_nav['recovery_count'] > current['recovery_count']):
            valid = False
    upper_names = ('coverage_t50_s', 'coverage_t70_s', 'coverage_t85_s',
                   'elapsed_s', 'cpu_seconds', 'peak_rss_bytes')
    passed = (
        valid and divergent >= 3 and path_non_worse >= 4 and
        statistics.median(ratios['gt_path_length_to_85_m']) <= 0.90 and
        all(statistics.median(ratios[name]) <= 1.05
            for name in upper_names) and
        statistics.median(ratios['coverage_auc']) >= 0.95 and
        statistics.median(ratios['minimum_clearance_m']) >= 0.95)
    return {'status': 'PASS' if passed else 'FAIL',
            'promote_gain_nav': passed, 'rules': PROMOTION_RULES}


def _full_hard_gates(value: dict) -> bool:
    return (
        value['validity'] == 'VALID' and value['outcome'] == 'PASS' and
        value['invalid_reasons'] == [] and
        value['final_coverage_ratio'] >= 0.85 and
        value['first_decision_reachable_count'] >= 2 and
        value['contact_count'] == 0 and
        value['minimum_clearance_m'] >= 0.05 and
        value['final_zero'] is True and
        value['map_to_odom_authority_count'] == 1 and
        value['cmd_vel_publisher_count'] == 1 and
        value['runner_cancellation_count'] == 0 and
        value['lifecycle_active'] is True and
        value['initial_parity'] is True and
        value['input_parity'] is True and value['hash_parity'] is True and
        value['production_hashes_unchanged'] is True and
        value['survivor_count'] == 0)


def _validate_smoke_evidence(value: dict) -> None:
    keys = {'schema_version', 'run_id', 'mode', 'policy', 'layout_seed',
            'map_sequence', 'map_payload_sha256', 'start_record',
            'blacklist_ids', 'candidate_sampling', 'raw_candidates',
            'planner_contract', 'planner_results', 'costmap_sha256_before',
            'costmap_sha256_after', 'decision_token', 'eligible_candidates',
            'selected_goal', 'unreachable_observations', 'status',
            'full_metric_schema', 'runtime_metrics', 'promotion'}
    if (type(value) is not dict or set(value) != keys or
            value['schema_version'] != 1 or value['mode'] != 'smoke' or
            value['run_id'] != 'current__seed_11' or
            value['policy'] != 'current' or value['layout_seed'] != 11 or
            value['map_sequence'] != 1 or value['status'] != 'PASS' or
            value['full_metric_schema'] != list(FULL_METRIC_FIELDS) or
            value['promotion'] != promotion_decision([])):
        raise ValueError('G005 smoke evidence schema drift')
    token = decision_token(
        value['run_id'], value['policy'], value['map_sequence'],
        value['map_payload_sha256'], value['start_record'],
        value['blacklist_ids'],
        value['candidate_sampling']['selected_candidate_ids'])
    if token != value['decision_token']:
        raise ValueError('G005 decision token drift')
    if value['costmap_sha256_before'] != value['costmap_sha256_after']:
        raise ValueError('G005 costmap changed during planner batch')
    sampling = value['candidate_sampling']
    if (type(sampling) is not dict or set(sampling) != {
            'raw_candidate_ids', 'excluded_candidate_ids',
            'selected_candidate_ids', 'omitted_candidate_ids'} or
            sampling['excluded_candidate_ids'] != value['blacklist_ids']):
        raise ValueError('G005 candidate sampling schema drift')
    raw_ids = sampling['raw_candidate_ids']
    excluded_ids = sampling['excluded_candidate_ids']
    if (any(type(values) is not list or
            any(type(item) is not int for item in values)
            for values in sampling.values()) or
            raw_ids != sorted(set(raw_ids)) or
            not set(excluded_ids).issubset(raw_ids)):
        raise ValueError('G005 candidate ID schema drift')
    allowed_ids = sorted(set(raw_ids) - set(excluded_ids))
    expected_sample = neutral_sample(value['layout_seed'], allowed_ids)
    if (sampling['selected_candidate_ids'] !=
            expected_sample['selected_candidate_ids'] or
            sampling['omitted_candidate_ids'] !=
            expected_sample['omitted_candidate_ids'] or
            set(excluded_ids) & set(sampling['selected_candidate_ids'])):
        raise ValueError('G005 blacklist or neutral sample drift')
    recomputed = validate_planner_batch(
        value['planner_results'], value['decision_token'],
        value['costmap_sha256_before'], value['costmap_sha256_after'])
    if recomputed != value['eligible_candidates']:
        raise ValueError('G005 eligible planner result drift')
    start_xy = value['start_record']['pose_xy_yaw'][:2]
    if any(record['poses'][0] != start_xy
           for record in value['planner_results']):
        raise ValueError('G005 planner start pose drift')
    eligible = value['eligible_candidates']
    if len(eligible) < 2:
        raise ValueError('G005 first decision lacks paired reachable choices')
    ranked = policy_rank(value['policy'], [dict(item) for item in eligible])
    if value['selected_goal'] != ranked[0]:
        raise ValueError('G005 selected goal policy drift')
    if value['planner_contract'] != {
            'action': 'ComputePathToPose', 'use_start': True,
            'planner_id': 'GridBased', 'timeout_s': 2.0,
            'same_frozen_start_for_all': True,
            'execution_status':
            'SYNTHETIC_FIXTURE_NOT_RUNTIME_COMPUTE_PATH'}:
        raise ValueError('G005 planner contract drift')
    if (value['unreachable_observations'] != [] or
            [item['cell_index'] for item in value['raw_candidates']] !=
            sampling['selected_candidate_ids'] or
            [item['candidate'] for item in value['planner_results']] !=
            value['raw_candidates']):
        raise ValueError('G005 candidate evidence binding drift')
    if value['runtime_metrics'] != {
            field: 'NOT_MEASURED_SMOKE' for field in FULL_METRIC_FIELDS}:
        raise ValueError('G005 smoke fabricated a full-run metric')


def _validate_full_evidence(value: dict, policy: str, seed: int) -> None:
    if (type(value) is not dict or set(value) != FULL_RUN_KEYS or
            value['schema_version'] != 1 or value['mode'] != 'full' or
            value['policy'] != policy or value['layout_seed'] != seed or
            value['run_id'] != f'{policy}__seed_{seed}' or
            value['validity'] not in ('VALID', 'INVALID') or
            value['outcome'] not in ('PASS', 'FAIL') or
            type(value['invalid_reasons']) is not list or
            any(type(item) is not str or not item
                for item in value['invalid_reasons']) or
            ((value['validity'] == 'VALID') !=
             (value['invalid_reasons'] == [])) or
            (value['validity'] == 'INVALID' and
             value['outcome'] != 'FAIL') or
            type(value['first_goal_cell_index']) is not int):
        raise ValueError('G005 full run schema drift')
    numeric = ('final_coverage_ratio', 'elapsed_s', 'coverage_auc',
               'minimum_clearance_m', 'cpu_seconds')
    threshold_fields = ('coverage_t50_s', 'coverage_t70_s',
                        'coverage_t85_s', 'gt_path_length_to_85_m')
    count_fields = ('planning_reject_count', 'recovery_count',
                    'failure_count', 'contact_count', 'peak_rss_bytes',
                    'map_to_odom_authority_count',
                    'cmd_vel_publisher_count', 'runner_cancellation_count',
                    'first_decision_reachable_count', 'survivor_count')
    bool_fields = ('final_zero', 'lifecycle_active', 'initial_parity',
                   'input_parity', 'hash_parity',
                   'production_hashes_unchanged')
    reached = [value[field] for field in threshold_fields[:3]]
    if (any(not _finite_number(value[field]) or value[field] < 0.0
            for field in numeric) or
            any(item is not None and (
                not _finite_number(item) or item < 0.0)
                for item in reached) or
            reached != sorted(reached, key=lambda item: (
                item is None, item if item is not None else 0.0)) or
            any(item is not None and item > value['elapsed_s']
                for item in reached) or
            value['elapsed_s'] != 900.0 or
            ((value['coverage_t85_s'] is None) !=
             (value['gt_path_length_to_85_m'] is None)) or
            (value['gt_path_length_to_85_m'] is not None and (
                not _finite_number(value['gt_path_length_to_85_m']) or
                value['gt_path_length_to_85_m'] < 0.0)) or
            not 0.0 <= value['final_coverage_ratio'] <= 1.0 or
            not 0.0 <= value['coverage_auc'] <= 1.0 or
            any(type(value[field]) is not int or value[field] < 0
                for field in count_fields) or
            any(type(value[field]) is not bool for field in bool_fields) or
            not (value['outcome'] == 'FAIL' and
                 value['score_decompositions'] == []) and
            not _score_decompositions_valid(
                value['score_decompositions'], policy)):
        raise ValueError('G005 full metric type or range drift')


def _score_decompositions_valid(batches: object, policy: str) -> bool:
    if type(batches) is not list or not batches:
        return False
    if [item.get('decision_index') for item in batches
            if type(item) is dict] != list(range(1, len(batches) + 1)):
        return False
    if any(type(batch) is not dict or set(batch) != {
            'decision_index', 'decision_token', 'records'} or
            type(batch['decision_index']) is not int or
            not _lower_sha256(batch['decision_token']) or
            type(batch['records']) is not list or len(batch['records']) < 1
            for batch in batches):
        return False
    expected = {'cell_index', 'gain_cells', 'bfs_distance_m', 'heading_rad',
                'nav_length_m', 'gain_norm', 'length_norm', 'utility'}
    for batch in batches:
        records = batch['records']
        if any(type(item) is not dict or set(item) != expected or
               type(item['cell_index']) is not int or any(
                   not _finite_number(item[name]) for name in expected
                   if name != 'cell_index') for item in records):
            return False
        if len({item['cell_index'] for item in records}) != len(records):
            return False
        for item in records:
            if policy == 'current':
                expected_utility = (item['gain_cells'] -
                                    0.20 * item['bfs_distance_m'] -
                                    0.10 * item['heading_rad'])
            elif policy == 'nearest':
                expected_utility = -item['nav_length_m']
            else:
                if not (0.0 <= item['gain_norm'] <= 1.0 and
                        0.0 <= item['length_norm'] <= 1.0):
                    return False
                expected_utility = (0.5 * item['gain_norm'] -
                                    0.5 * item['length_norm'])
            if not math.isclose(item['utility'], expected_utility,
                                rel_tol=0.0, abs_tol=1e-12):
                return False
    return True


def validate_artifact(root: Path, expected_mode: str) -> dict:
    """Validate an exact smoke or full G005 result tree."""
    if (not root.is_absolute() or not root.is_dir() or root.is_symlink() or
            expected_mode not in ('smoke', 'full')):
        raise ValueError('non-canonical G005 artifact root')
    if expected_mode == 'full':
        from run_frontier_policy_full import (
            FULL_RUNTIME_BLOCKER, FULL_RUNTIME_PATH_ENABLED)
        if not FULL_RUNTIME_PATH_ENABLED:
            raise ValueError(FULL_RUNTIME_BLOCKER)
    manifest = strict_json_load(root / 'manifest.json')
    manifest_keys = {'schema_version', 'mode', 'claim_scope', 'asset_root',
                     'asset_manifest', 'full_plan', 'executed_plan', 'runs',
                     'promotion', 'evaluator_source', 'runner_source',
                     'runtime_executor', 'runtime_proofs', 'tree_files',
                     'tree_sha256', 'storage_limit_bytes',
                     'contract_identity', 'execution_plan_sha256',
                     'frontier_policy_handoff'}
    if (type(manifest) is not dict or set(manifest) != manifest_keys or
            manifest['schema_version'] != 1 or
            manifest['mode'] != expected_mode or
            manifest['claim_scope'] !=
            ('OFFLINE_FRONTIER_POLICY_CONTRACT_SMOKE_NO_NAV2_NO_MOTION'
             if expected_mode == 'smoke' else
             'SIMULATION_ONLY_FRONTIER_POLICY_PAIRED_EVALUATION') or
            manifest['full_plan'] != [{'policy': policy, 'layout_seed': seed}
                                      for policy, seed in FULL_PLAN] or
            manifest['storage_limit_bytes'] != ARTIFACT_LIMIT_BYTES):
        raise ValueError('G005 artifact manifest schema drift')
    asset_root = Path(manifest['asset_root'])
    asset_manifest = validate_assets(
        asset_root, 'smoke' if expected_mode == 'smoke' else 'full')
    if manifest['asset_manifest'] != file_identity(
            asset_root / 'asset_manifest.json'):
        raise ValueError('G005 asset identity drift')
    if manifest['evaluator_source'] != file_identity(Path(__file__).resolve()):
        raise ValueError('G005 evaluator source drift')
    if expected_mode == 'smoke':
        from run_frontier_policy_smoke import __file__ as runner_file
    else:
        from run_frontier_policy_full import __file__ as runner_file
    if manifest['runner_source'] != file_identity(Path(runner_file).resolve()):
        raise ValueError('G005 runner source drift')
    run_paths = [root / item['path'] for item in manifest['runs']]
    expected_plan = ([{'policy': 'current', 'layout_seed': 11}]
                     if expected_mode == 'smoke' else
                     [{'policy': policy, 'layout_seed': seed}
                      for policy, seed in FULL_PLAN])
    expected_names = (['current__seed_11.json']
                      if expected_mode == 'smoke' else
                      [f'{policy}__seed_{seed}.json'
                       for policy, seed in FULL_PLAN])
    if (manifest['executed_plan'] != expected_plan or
            manifest['runs'] != [{'path': name} for name in expected_names] or
            len(run_paths) != len(expected_plan)):
        raise ValueError('G005 executed plan drift')
    if expected_mode == 'smoke':
        if (manifest['runtime_executor'] != 'NOT_APPLICABLE_SMOKE' or
                manifest['runtime_proofs'] != [] or
                manifest['execution_plan_sha256'] != 'NOT_APPLICABLE_SMOKE' or
                manifest['frontier_policy_handoff'] != {
                    'decision': 'NOT_EVALUATED',
                    'selected_policy': 'current',
                    'production_change_authorized': False}):
            raise ValueError('G005 smoke runtime proof drift')
    else:
        from run_frontier_policy_full import (
            validate_paired_first_decisions, validate_runtime_proof)
        proof_names = [f'{policy}__seed_{seed}.runtime.json'
                       for policy, seed in FULL_PLAN]
        if (type(manifest['runtime_executor']) is not dict or
                set(manifest['runtime_executor']) != {
                    'argv', 'executable'} or
                type(manifest['runtime_executor']['argv']) is not list or
                not manifest['runtime_executor']['argv'] or
                type(manifest['runtime_executor']['executable']) is not dict or
                type(manifest['execution_plan_sha256']) is not str or
                len(manifest['execution_plan_sha256']) != 64 or
                manifest['runtime_proofs'] != [
                    {'path': name} for name in proof_names]):
            raise ValueError('G005 full runtime executor drift')
        executable_record = manifest['runtime_executor']['executable']
        executable_path = Path(executable_record.get('path', ''))
        if (not executable_path.is_absolute() or
                executable_record != file_identity(executable_path)):
            raise ValueError('G005 full runtime executable identity drift')
        execution_plan_path = root / 'execution_plan.json'
        if (not execution_plan_path.is_file() or
                execution_plan_path.is_symlink() or
                file_identity(execution_plan_path)['sha256'] !=
                manifest['execution_plan_sha256']):
            raise ValueError('G005 execution plan identity drift')
        execution_plan = strict_json_load(execution_plan_path)
        if (type(execution_plan) is not dict or
                execution_plan.get('schema_version') != 1 or
                execution_plan.get('mode') != 'full' or
                execution_plan.get('claim_scope') !=
                'G005_EXECUTION_PLAN_NO_RUNTIME_CLAIM' or
                execution_plan.get('asset_manifest') !=
                manifest['asset_manifest'] or
                type(execution_plan.get('requests')) is not list or
                len(execution_plan['requests']) != 15):
            raise ValueError('G005 execution plan schema drift')
        runtime_proofs = []
        for proof_name, plan in zip(proof_names, expected_plan):
            proof_path = root / proof_name
            if (proof_path.parent != root or not proof_path.is_file() or
                    proof_path.is_symlink() or
                    proof_path.stat().st_size > RUN_OUTPUT_LIMIT_BYTES):
                raise ValueError('G005 runtime proof path or cap drift')
            proof = strict_json_load(proof_path)
            validate_runtime_proof(
                proof, plan['policy'], plan['layout_seed'], asset_manifest)
            if proof['asset_identity'] != file_identity(
                    asset_root / f"layout_{plan['layout_seed']}_gt.json"):
                raise ValueError('G005 runtime proof asset binding drift')
            runtime_proofs.append(proof)
        validate_paired_first_decisions(runtime_proofs)
        for request, proof in zip(execution_plan['requests'], runtime_proofs):
            _validate_execution_request(request)
            if (request.get('asset_root') != str(asset_root.resolve()) or
                    request.get('asset_manifest_identity') !=
                    manifest['asset_manifest']):
                raise ValueError('G005 execution request asset drift')
            for key in ('run_id', 'policy', 'layout_seed', 'request_sha256',
                        'asset_identity', 'asset_manifest_identity',
                        'production_inputs_sha256', 'shared_initial_sha256',
                        'shared_reveal_sha256', 'shared_runtime_sha256',
                        'simulation_horizon_s'):
                if request.get(key) != proof[key]:
                    raise ValueError('G005 execution plan proof binding drift')
            if request.get('runtime_components') != proof[
                    'runtime_identity']['components']:
                raise ValueError('G005 runtime component binding drift')
        for seed in sorted({seed for _, seed in FULL_PLAN}):
            paired = [item for item in runtime_proofs
                      if item['layout_seed'] == seed]
            for key in ('asset_identity', 'shared_initial_sha256',
                        'shared_reveal_sha256', 'shared_runtime_sha256'):
                if len({hashlib.sha256(
                        canonical_json_bytes(item[key])).hexdigest()
                        for item in paired}) != 1:
                    raise ValueError('G005 paired runtime parity drift')
    runs = []
    for path, plan in zip(run_paths, expected_plan):
        if (path.parent != root or not path.is_file() or path.is_symlink() or
                path.stat().st_size > RUN_OUTPUT_LIMIT_BYTES):
            raise ValueError('G005 run path or cap drift')
        value = strict_json_load(path)
        if expected_mode == 'smoke':
            _validate_smoke_evidence(value)
        else:
            _validate_full_evidence(
                value, plan['policy'], plan['layout_seed'])
            from run_frontier_policy_full import compact_runtime_proof
            proof = strict_json_load(
                root / f"{plan['policy']}__seed_{plan['layout_seed']}"
                '.runtime.json')
            if value != compact_runtime_proof(proof, asset_manifest):
                raise ValueError('G005 compact metrics are not proof-derived')
        runs.append(value)
    expected_promotion = promotion_decision(
        runs if expected_mode == 'full' else [])
    if manifest['promotion'] != expected_promotion:
        raise ValueError('G005 promotion result drift')
    from frontier_policy_contract import __file__ as contract_file
    if manifest['contract_identity'] != file_identity(
            Path(contract_file).resolve()):
        raise ValueError('G005 contract identity drift')
    expected_handoff = {
        'decision': ('GAIN_NAV_EVAL_CANDIDATE'
                     if expected_promotion['promote_gain_nav'] else
                     ('NOT_EVALUATED' if expected_mode == 'smoke' else
                      'RETAIN_CURRENT')),
        'selected_policy': ('gain_nav'
                            if expected_promotion['promote_gain_nav'] else
                            'current'),
        'production_change_authorized': False,
    }
    if manifest['frontier_policy_handoff'] != expected_handoff:
        raise ValueError('G005 frontier policy handoff drift')
    file_names = sorted(path.name for path in root.iterdir()
                        if path.name != 'manifest.json')
    if any(path.is_symlink() or not path.is_file() for path in root.iterdir()):
        raise ValueError('G005 artifact contains non-regular file')
    records = [file_identity(root / name, relative_to=root)
               for name in file_names]
    if (manifest['tree_files'] != records or
            manifest['tree_sha256'] != hashlib.sha256(
                canonical_json_bytes(records)).hexdigest() or
            sum(path.stat().st_size for path in root.iterdir()) >
            ARTIFACT_LIMIT_BYTES):
        raise ValueError('G005 artifact tree or cap drift')
    return manifest


def main() -> int:
    """Validate one stored G005 artifact."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--mode', choices=('smoke', 'full'), required=True)
    args = parser.parse_args()
    validate_artifact(args.root, args.mode)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
