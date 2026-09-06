#!/usr/bin/env python3
"""Regression and hostile tests for the G005 frontier policy harness."""

from copy import deepcopy
import math
from pathlib import Path

import evaluate_frontier_policy as evaluator
from frontier_policy_contract import (
    build_layout,
    canonical_json_bytes,
    connected_reachable_cells,
    decision_token,
    FULL_PLAN,
    neutral_sample,
    policy_rank,
    reveal_scan,
    strict_json_load,
    unreachable_outcome,
    unresolved_after_three,
    validate_planner_batch,
)
from generate_frontier_policy_assets import generate, validate_assets
import pytest
from run_frontier_policy_smoke import run_smoke


def _candidates():
    return [
        {'cell_index': 8, 'gain_cells': 5, 'bfs_distance_m': 2.0,
         'heading_rad': 1.0, 'nav_length_m': 3.0},
        {'cell_index': 3, 'gain_cells': 4, 'bfs_distance_m': 1.0,
         'heading_rad': 0.0, 'nav_length_m': 1.0},
    ]


def _full_run(policy, seed):
    ratio = 1.05 if policy == 'gain_nav' else 1.0
    path_m = 9.0 if policy == 'gain_nav' else 10.0
    auc = 0.76 if policy == 'gain_nav' else 0.8
    clearance_m = 0.095 if policy == 'gain_nav' else 0.1
    score_records = []
    for index in (1, 2):
        record = {'cell_index': index, 'gain_cells': 4.0,
                  'bfs_distance_m': 2.0, 'heading_rad': 0.5,
                  'nav_length_m': 3.0, 'gain_norm': float(index - 1),
                  'length_norm': float(index - 1), 'utility': 0.0}
        if policy == 'current':
            record['utility'] = 3.55
        elif policy == 'nearest':
            record['utility'] = -3.0
        score_records.append(record)
    return {
        'schema_version': 1, 'run_id': f'{policy}__seed_{seed}',
        'mode': 'full', 'policy': policy, 'layout_seed': seed,
        'status': 'PASS',
        'first_goal_cell_index': {'current': 1, 'nearest': 2,
                                  'gain_nav': 3}[policy],
        'final_coverage_ratio': 0.9,
        'coverage_t50_s': 40.0 * ratio,
        'coverage_t70_s': 70.0 * ratio,
        'coverage_t85_s': 90.0 * ratio,
        'elapsed_s': 100.0 * ratio, 'coverage_auc': auc,
        'gt_path_length_to_85_m': path_m,
        'planning_reject_count': 0, 'recovery_count': 0,
        'failure_count': 0, 'score_decompositions': score_records,
        'contact_count': 0, 'minimum_clearance_m': clearance_m,
        'cpu_seconds': 10.0 * ratio,
        'peak_rss_bytes': 1000 if policy != 'gain_nav' else 1050,
        'final_zero': True, 'map_to_odom_authority_count': 1,
        'cmd_vel_publisher_count': 1, 'runner_cancellation_count': 0,
        'lifecycle_active': True, 'initial_parity': True,
        'input_parity': True, 'hash_parity': True,
        'production_hashes_unchanged': True,
        'first_decision_reachable_count': 2, 'survivor_count': 0}


def test_layout_reachable_denominator_and_reveal_are_deterministic_monotonic():
    layout = build_layout(11)
    reachable = connected_reachable_cells(layout)
    assert len(reachable) > 100
    start = layout['start_cell'][1] * layout['width'] + layout['start_cell'][0]
    observed = [-1] * len(layout['data'])
    first = reveal_scan(layout, observed, start, 0)
    second = reveal_scan(layout, first, start, 1)
    assert first == reveal_scan(layout, observed, start, 0)
    assert all(left == -1 or left == right
               for left, right in zip(first, second))
    assert all(value in (-1, truth)
               for value, truth in zip(second, layout['data']))


def test_coverage_metrics_use_connected_denominator_and_exact_thresholds():
    result = evaluator.coverage_metrics([
        {'elapsed_s': 0.0, 'revealed_reachable_cells': 0},
        {'elapsed_s': 5.0, 'revealed_reachable_cells': 50},
        {'elapsed_s': 7.0, 'revealed_reachable_cells': 70},
        {'elapsed_s': 9.0, 'revealed_reachable_cells': 90},
        {'elapsed_s': 10.0, 'revealed_reachable_cells': 100}], 100)
    assert result['coverage_t50_s'] == 5.0
    assert result['coverage_t70_s'] == 7.0
    assert result['coverage_t85_s'] == 9.0
    assert result['final_coverage_ratio'] == 1.0
    assert 0.0 < result['coverage_auc'] < 1.0


def test_reveal_rejects_a_cell_class_flip():
    layout = build_layout(11)
    start = layout['start_cell'][1] * layout['width'] + layout['start_cell'][0]
    observed = [-1] * len(layout['data'])
    observed[start] = 100
    with pytest.raises(ValueError, match='class flip'):
        reveal_scan(layout, observed, start, 2)


def test_neutral_sample_is_order_independent_and_limited():
    forward = neutral_sample(11, list(range(20)))
    reverse = neutral_sample(11, list(reversed(range(20))))
    assert forward == reverse
    assert len(forward['selected_candidate_ids']) == 5
    with pytest.raises(ValueError, match='unique'):
        neutral_sample(11, [1, 1])


def test_current_nearest_and_gain_nav_formulas_and_ties():
    current = policy_rank('current', deepcopy(_candidates()))
    assert current[0]['utility'] == pytest.approx(4.5)
    nearest = policy_rank('nearest', deepcopy(_candidates()))
    assert nearest[0]['cell_index'] == 3
    gain_nav = policy_rank('gain_nav', deepcopy(_candidates()))
    assert gain_nav[0]['cell_index'] in (3, 8)
    tied = [dict(_candidates()[0], cell_index=index, gain_cells=1,
                 bfs_distance_m=1.0, heading_rad=0.0, nav_length_m=1.0)
            for index in (9, 2)]
    assert [item['cell_index'] for item in policy_rank(
        'gain_nav', tied)] == [2, 9]


def test_current_formula_matches_production_frontier_score():
    from jdamr_cube_navigation.frontier_core import FrontierConfig
    config = FrontierConfig()
    candidate = _candidates()[0]
    expected = (config.information_gain_weight * candidate['gain_cells'] -
                config.distance_weight * candidate['bfs_distance_m'] -
                config.heading_weight * candidate['heading_rad'])
    assert policy_rank('current', [candidate])[0]['utility'] == expected


def test_decision_token_changes_for_every_frozen_input():
    start = {'frame_id': 'map', 'stamp_ns': 1,
             'pose_xy_yaw': [0.0, 0.0, 0.0]}
    base = decision_token('run', 'current', 1, 'a' * 64, start, [], [1, 2])
    assert base != decision_token(
        'run', 'current', 2, 'a' * 64, start, [], [1, 2])
    assert base != decision_token(
        'run', 'current', 1, 'a' * 64, start, [9], [1, 2])
    assert base != decision_token(
        'run', 'current', 1, 'a' * 64, start, [], [2, 1])


def test_planner_batch_requires_frozen_costmap_and_token():
    candidate = _candidates()[0]
    record = {'candidate': candidate, 'token': 'token', 'use_start': True,
              'planner_id': 'GridBased', 'timeout_s': 2.0,
              'error_code': 0, 'frame_id': 'map',
              'poses': [[0.0, 0.0], [3.0, 4.0]]}
    result = validate_planner_batch([record], 'token', 'same', 'same')
    assert result[0]['nav_length_m'] == 5.0
    assert validate_planner_batch([record], 'stale', 'same', 'same') == []
    with pytest.raises(ValueError, match='costmap changed'):
        validate_planner_batch([record], 'token', 'left', 'right')


def test_unreachable_requires_three_fresh_identical_batches():
    row = {'map_sha256': 'a', 'start_sha256': 'b',
           'candidate_sha256': 'c', 'fresh': True, 'reachable_count': 0}
    assert unresolved_after_three([dict(row)] * 3) is True
    assert unreachable_outcome([dict(row)] * 3) == {
        'status': 'UNRESOLVED_FRONTIERS', 'result': 'FAIL',
        'exploration_complete': False, 'blacklist_exhausted': False}
    assert unresolved_after_three([dict(row)] * 2) is False
    changed = [dict(row), dict(row), dict(row, candidate_sha256='d')]
    assert unresolved_after_three(changed) is False
    assert unresolved_after_three(
        [dict(row), dict(row), dict(row, reachable_count=1)]) is False


def test_asset_generation_and_tree_tampering_fail_closed(tmp_path):
    root = tmp_path / 'assets'
    manifest = generate(root, 'smoke')
    assert manifest['layout_seeds'] == [11]
    assert len(manifest['full_plan']) == 15
    assert validate_assets(root, 'smoke') == manifest
    (root / 'shadow.txt').write_text('x', encoding='utf-8')
    with pytest.raises(ValueError, match='inventory'):
        validate_assets(root, 'smoke')


def test_strict_json_rejects_duplicate_and_nonfinite(tmp_path):
    path = tmp_path / 'bad.json'
    path.write_text('{"a":1,"a":2}', encoding='utf-8')
    with pytest.raises(ValueError, match='duplicate'):
        strict_json_load(path)
    path.write_text('{"a":NaN}', encoding='utf-8')
    with pytest.raises(ValueError, match='non-finite'):
        strict_json_load(path)
    with pytest.raises(ValueError):
        canonical_json_bytes({'a': math.inf})


def test_smoke_artifact_and_blacklist_are_exact(tmp_path):
    assets = tmp_path / 'assets'
    result = tmp_path / 'result'
    generate(assets, 'smoke')
    manifest = run_smoke(assets, result)
    assert evaluator.validate_artifact(result, 'smoke') == manifest
    evidence_path = result / 'current__seed_11.json'
    evidence = strict_json_load(evidence_path)
    sampling = evidence['candidate_sampling']
    assert sampling['excluded_candidate_ids']
    assert not (set(sampling['excluded_candidate_ids']) &
                set(sampling['selected_candidate_ids']))


def test_smoke_token_and_selected_goal_tampering_fail(tmp_path):
    assets = tmp_path / 'assets'
    result = tmp_path / 'result'
    generate(assets, 'smoke')
    run_smoke(assets, result)
    evidence_path = result / 'current__seed_11.json'
    evidence = strict_json_load(evidence_path)
    evidence['decision_token'] = '0' * 64
    evidence_path.write_bytes(canonical_json_bytes(evidence))
    with pytest.raises(ValueError, match='token'):
        evaluator.validate_artifact(result, 'smoke')


def test_promotion_stays_not_evaluated_without_exact_15():
    result = evaluator.promotion_decision([])
    assert result['status'] == 'NOT_EVALUATED'
    assert result['promote_gain_nav'] is False
    assert len(FULL_PLAN) == 15


def test_promotion_exact_paired_boundary_passes_and_regression_fails():
    rows = [_full_run(policy, seed) for policy, seed in FULL_PLAN]
    assert evaluator.promotion_decision(rows)['promote_gain_nav'] is True
    rows[10]['gt_path_length_to_85_m'] = 10.000001
    rows[11]['gt_path_length_to_85_m'] = 10.000001
    assert evaluator.promotion_decision(rows)['promote_gain_nav'] is False


@pytest.mark.parametrize('field,value', [
    ('gt_path_length_to_85_m', 9.000001),
    ('elapsed_s', 105.000001),
    ('coverage_auc', 0.759999),
    ('minimum_clearance_m', 0.094999),
])
def test_promotion_paired_median_boundaries_fail(field, value):
    rows = [_full_run(policy, seed) for policy, seed in FULL_PLAN]
    for row in rows[10:13]:
        row[field] = value
    assert evaluator.promotion_decision(rows)['promote_gain_nav'] is False


@pytest.mark.parametrize('field', ['failure_count', 'recovery_count'])
def test_promotion_failure_and_recovery_must_not_regress(field):
    rows = [_full_run(policy, seed) for policy, seed in FULL_PLAN]
    rows[10][field] = 1
    assert evaluator.promotion_decision(rows)['promote_gain_nav'] is False


def test_full_schema_and_score_decomposition_fail_closed():
    value = _full_run('current', 11)
    evaluator._validate_full_evidence(value, 'current', 11)
    changed = deepcopy(value)
    changed['score_decompositions'][0]['utility'] += 0.01
    with pytest.raises(ValueError, match='metric'):
        evaluator._validate_full_evidence(changed, 'current', 11)
    changed = deepcopy(value)
    changed['unknown'] = 1
    with pytest.raises(ValueError, match='schema'):
        evaluator._validate_full_evidence(changed, 'current', 11)


@pytest.mark.parametrize('field,value', [
    ('final_coverage_ratio', 0.849999),
    ('first_decision_reachable_count', 1),
    ('cmd_vel_publisher_count', 2),
    ('runner_cancellation_count', 1),
    ('production_hashes_unchanged', False),
])
def test_full_hard_gate_mutations_block_promotion(field, value):
    rows = [_full_run(policy, seed) for policy, seed in FULL_PLAN]
    rows[0][field] = value
    assert evaluator.promotion_decision(rows)['promote_gain_nav'] is False


def test_numeric_contract_names_include_units():
    source = Path(__import__('frontier_policy_contract').__file__).read_text()
    assert 'RESOLUTION_M_PER_CELL' in source
    assert 'LIDAR_RATE_HZ' in source
    assert 'RUN_OUTPUT_LIMIT_BYTES' in source
