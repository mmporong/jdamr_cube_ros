"""Fail-closed tests for the G005 full runtime matrix runner."""

from copy import deepcopy
import hashlib
from pathlib import Path
import subprocess
import sys

import evaluate_frontier_policy as evaluator
from frontier_policy_contract import (
    canonical_json_bytes,
    decision_token,
    neutral_sample,
    policy_rank,
)
from generate_frontier_policy_assets import generate
import pytest
from run_frontier_policy_full import (
    _gt_metrics,
    _validate_runtime_proof_shape,
    build_execution_plan,
    CLAIM_SCOPE,
    FULL_RUNTIME_BLOCKER,
    FULL_RUNTIME_PATH_ENABLED,
    run_full,
    SIMULATION_HORIZON_S,
    validate_paired_first_decisions,
    validate_runtime_proof,
)


def _identity(name='input'):
    return {'path': f'/tmp/{name}', 'size_bytes': 10, 'sha256': 'a' * 64}


def _proof(policy='current', seed=11):
    run_id = f'{policy}__seed_{seed}'
    raw = [10, 20, 30]
    sampling = neutral_sample(seed, raw)
    sampling['excluded_candidate_ids'] = []
    candidates = {
        10: {'cell_index': 10, 'gain_cells': 5.0,
             'bfs_distance_m': 2.0, 'heading_rad': 0.1,
             'nav_length_m': 0.0},
        20: {'cell_index': 20, 'gain_cells': 4.0,
             'bfs_distance_m': 1.0, 'heading_rad': 0.2,
             'nav_length_m': 0.0},
        30: {'cell_index': 30, 'gain_cells': 6.0,
             'bfs_distance_m': 3.0, 'heading_rad': 0.3,
             'nav_length_m': 0.0},
    }
    start = {'frame_id': 'map', 'stamp_ns': 100,
             'pose_xy_yaw': [0.0, 0.0, 0.0]}
    map_sha = 'b' * 64
    token = decision_token(
        run_id, policy, 1, map_sha, start, [],
        sampling['selected_candidate_ids'])
    planner = []
    for index in sampling['selected_candidate_ids']:
        candidate = candidates[index]
        planner.append({
            'candidate': candidate, 'token': token, 'use_start': True,
            'planner_id': 'GridBased', 'timeout_s': 2.0,
            'error_code': 0, 'frame_id': 'map',
            'poses': [[0.0, 0.0], [index / 10.0, 0.0]],
        })
    eligible = []
    for record in planner:
        item = dict(record['candidate'])
        item['nav_length_m'] = record['poses'][-1][0]
        eligible.append(item)
    selected = policy_rank(policy, deepcopy(eligible))[0]
    decision = {
        'planner_batch_attempt_index': 1,
        'map_sequence': 1, 'map_payload_sha256': map_sha,
        'start_record': start, 'blacklist_ids': [],
        'candidate_sampling': sampling, 'decision_token': token,
        'costmap_sha256_before': 'c' * 64,
        'costmap_sha256_after': 'c' * 64,
        'planner_action': {
            'type': 'nav2_msgs/action/ComputePathToPose',
            'planner_id': 'GridBased', 'use_start': True,
            'timeout_s': 2.0, 'execution': 'ACTUAL_NAV2_ACTION'},
        'planner_results': planner, 'eligible_candidates': eligible,
        'selected_goal': selected, 'stale': False,
    }
    before = {'frontier_core': _identity('frontier_core')}
    return {
        'schema_version': 1, 'claim_scope': CLAIM_SCOPE, 'run_id': run_id,
        'policy': policy, 'layout_seed': seed, 'status': 'PASS',
        'request_sha256': 'd' * 64, 'asset_identity': _identity('asset'),
        'shared_initial_sha256': 'e' * 64,
        'shared_reveal_sha256': 'f' * 64,
        'shared_runtime_sha256': '1' * 64,
        'simulation_horizon_s': SIMULATION_HORIZON_S,
        'runtime_identity': {
            'gazebo_actual': True, 'nav2_actual': True,
            'compute_path_action_actual': True,
            'oracle_reveal_map': True,
            'ground_truth_localization': True,
            'not_slam_evaluation': True, 'synthetic_fixture': False},
        'decisions': [decision],
        'planner_batch_timeline': [{
            'attempt_index': 1, 'map_sequence': 1,
            'map_payload_sha256': map_sha,
            'start_record_sha256': hashlib.sha256(
                canonical_json_bytes(start)).hexdigest(),
            'candidate_sha256': hashlib.sha256(canonical_json_bytes(
                sampling['selected_candidate_ids'])).hexdigest(),
            'fresh': True, 'reachable_count': len(eligible)}],
        'coverage_samples': {
            'reachable_denominator_cells': 100,
            'samples': [
                {'elapsed_s': 0.0, 'revealed_reachable_cells': 10},
                {'elapsed_s': 50.0, 'revealed_reachable_cells': 50},
                {'elapsed_s': 70.0, 'revealed_reachable_cells': 70},
                {'elapsed_s': 90.0, 'revealed_reachable_cells': 85},
                {'elapsed_s': 100.0, 'revealed_reachable_cells': 90}]},
        'gt_pose_samples': [
            {'steady_elapsed_s': 0.0, 'sim_elapsed_s': 0.0,
             'x_m': 0.0, 'y_m': 0.0,
             'clearance_m': 0.2},
            {'steady_elapsed_s': 25.0, 'sim_elapsed_s': 50.0,
             'x_m': 1.0, 'y_m': 0.0,
             'clearance_m': 0.1},
            {'steady_elapsed_s': 45.0, 'sim_elapsed_s': 90.0,
             'x_m': 2.0, 'y_m': 0.0,
             'clearance_m': 0.1},
            {'steady_elapsed_s': 50.0, 'sim_elapsed_s': 100.0,
             'x_m': 2.1, 'y_m': 0.0,
             'clearance_m': 0.1}],
        'contact_count': 0,
        'command_authority': {
            'topic': '/cmd_vel', 'publisher_gids': ['01'],
            'publisher_nodes': ['/collision_monitor'],
            'final_zero_hold_s': 1.0, 'final_linear_x': 0.0,
            'final_angular_z': 0.0},
        'tf_authority': {
            'map_to_odom_publisher_gids': ['02'],
            'map_to_odom_publisher_nodes': ['/ground_truth_localization']},
        'lifecycle': {'planner_server': 'active',
                      'bt_navigator': 'active'},
        'navigation_outcomes': {'recovery_count': 0, 'failure_count': 0},
        'resources': {'cpu_seconds': 4.0, 'peak_rss_bytes': 1000,
                      'samples': 2},
        'runner_cancellation_count': 0,
        'production_inputs_before': before,
        'production_inputs_after': deepcopy(before),
        'teardown': {'survivor_count': 0,
                     'finalized_before_teardown': True},
    }


def _proof_with_two_reachable_batches():
    proof = _proof()
    second = deepcopy(proof['decisions'][0])
    second['planner_batch_attempt_index'] = 2
    second['map_sequence'] = 2
    second['map_payload_sha256'] = '6' * 64
    second['start_record']['stamp_ns'] = 200
    sampling = second['candidate_sampling']['selected_candidate_ids']
    token = decision_token(
        proof['run_id'], proof['policy'], second['map_sequence'],
        second['map_payload_sha256'], second['start_record'],
        second['blacklist_ids'], sampling)
    second['decision_token'] = token
    for result in second['planner_results']:
        result['token'] = token
    proof['decisions'].append(second)
    proof['planner_batch_timeline'].append({
        'attempt_index': 2, 'map_sequence': 2,
        'map_payload_sha256': second['map_payload_sha256'],
        'start_record_sha256': hashlib.sha256(canonical_json_bytes(
            second['start_record'])).hexdigest(),
        'candidate_sha256': hashlib.sha256(canonical_json_bytes(
            sampling)).hexdigest(), 'fresh': True,
        'reachable_count': len(second['eligible_candidates'])})
    return proof


def test_execution_plan_is_exact_15_and_paired_hashes_match(tmp_path):
    assets = tmp_path / 'assets'
    generate(assets, 'full')
    plan = build_execution_plan(assets, 100)
    assert len(plan['requests']) == 15
    assert plan['runtime_path'] == {
        'enabled': False, 'status': 'PENDING',
        'blocker': FULL_RUNTIME_BLOCKER,
        'external_adapter_results_accepted': False}
    assert [item['ros_domain_id'] for item in plan['requests']] == list(
        range(100, 115))
    seed_11 = [item for item in plan['requests']
               if item['layout_seed'] == 11]
    for key in ('asset_identity', 'shared_initial_sha256',
                'shared_reveal_sha256', 'shared_runtime_sha256'):
        assert len({hashlib.sha256(canonical_json_bytes(item[key])).hexdigest()
                    for item in seed_11}) == 1


def test_dormant_runtime_proof_shape_derives_metrics_from_traces():
    proof = _proof()
    derived = _validate_runtime_proof_shape(proof, 'current', 11)
    assert derived['coverage']['coverage_t85_s'] == 90.0
    assert derived['gt']['gt_path_length_to_85_m'] == pytest.approx(2.0)
    assert derived['gt']['minimum_clearance_m'] == pytest.approx(0.1)


def test_gt_path_to_85_interpolates_by_sim_time_inside_segment():
    result = _gt_metrics([
        {'steady_elapsed_s': 0.0, 'sim_elapsed_s': 0.0,
         'x_m': 0.0, 'y_m': 0.0, 'clearance_m': 1.0},
        {'steady_elapsed_s': 50.0, 'sim_elapsed_s': 100.0,
         'x_m': 100.0, 'y_m': 0.0, 'clearance_m': 1.0}], 90.0)
    assert result['gt_path_length_to_85_m'] == pytest.approx(90.0)


@pytest.mark.parametrize('mutation,match', [
    (lambda rows: rows.append(dict(rows[-1])), 'not monotonic'),
    (lambda rows: rows[1].update(sim_elapsed_s=901.0), 'trace drift'),
    (lambda rows: rows[1].update(x_m=float('nan')), 'trace drift'),
])
def test_gt_path_timestamps_and_values_fail_closed(mutation, match):
    rows = [
        {'steady_elapsed_s': 0.0, 'sim_elapsed_s': 0.0,
         'x_m': 0.0, 'y_m': 0.0, 'clearance_m': 1.0},
        {'steady_elapsed_s': 50.0, 'sim_elapsed_s': 100.0,
         'x_m': 100.0, 'y_m': 0.0, 'clearance_m': 1.0}]
    mutation(rows)
    with pytest.raises(ValueError, match=match):
        _gt_metrics(rows, 90.0)


def test_gt_path_rejects_t85_outside_pose_trace():
    rows = [
        {'steady_elapsed_s': 0.0, 'sim_elapsed_s': 10.0,
         'x_m': 0.0, 'y_m': 0.0, 'clearance_m': 1.0},
        {'steady_elapsed_s': 10.0, 'sim_elapsed_s': 20.0,
         'x_m': 10.0, 'y_m': 0.0, 'clearance_m': 1.0}]
    with pytest.raises(ValueError, match='does not bracket'):
        _gt_metrics(rows, 9.0)


def test_runtime_path_is_disabled_and_manual_result_injection_fails():
    assert FULL_RUNTIME_PATH_ENABLED is False
    with pytest.raises(ValueError, match=FULL_RUNTIME_BLOCKER):
        validate_runtime_proof(_proof(), 'current', 11)


def test_full_artifact_validator_rejects_injected_tree_before_reading_it(
        tmp_path):
    root = tmp_path / 'injected'
    root.mkdir()
    with pytest.raises(ValueError, match=FULL_RUNTIME_BLOCKER):
        evaluator.validate_artifact(root, 'full')


@pytest.mark.parametrize('mutate,match', [
    (lambda value: value['runtime_identity'].update(
        synthetic_fixture=True), 'actual Gazebo/Nav2'),
    (lambda value: value['decisions'][0].update(stale=True),
     'decision schema'),
    (lambda value: value['decisions'][0].update(
        costmap_sha256_after='9' * 64), 'identity schema'),
    (lambda value: value['command_authority'].update(
        publisher_gids=['01', '02']), 'command authority'),
    (lambda value: value['teardown'].update(survivor_count=1),
     'safety or teardown'),
])
def test_runtime_proof_hostile_mutations_fail_closed(mutate, match):
    proof = _proof()
    mutate(proof)
    with pytest.raises(ValueError, match=match):
        _validate_runtime_proof_shape(proof, 'current', 11)


def test_blacklist_must_equal_hard_exclusion_set():
    proof = _proof()
    proof['decisions'][0]['blacklist_ids'] = [99]
    with pytest.raises(ValueError, match='blacklist exclusion'):
        _validate_runtime_proof_shape(proof, 'current', 11)


@pytest.mark.parametrize('mutate', [
    lambda decision: decision.update(map_payload_sha256='A' * 64),
    lambda decision: decision['start_record'].update(stamp_ns=0),
    lambda decision: decision['start_record'].update(extra=True),
    lambda decision: decision['start_record']['pose_xy_yaw'].__setitem__(
        2, float('inf')),
    lambda decision: decision.update(costmap_sha256_before='short'),
])
def test_decision_identity_schema_is_exact_and_finite(mutate):
    proof = _proof()
    mutate(proof['decisions'][0])
    with pytest.raises(ValueError, match='identity schema'):
        _validate_runtime_proof_shape(proof, 'current', 11)


def test_blacklisted_candidate_cannot_be_selected_or_planned():
    proof = _proof()
    selected = proof['decisions'][0][
        'candidate_sampling']['selected_candidate_ids'][0]
    proof['decisions'][0]['blacklist_ids'] = [selected]
    proof['decisions'][0][
        'candidate_sampling']['excluded_candidate_ids'] = [selected]
    with pytest.raises(ValueError, match='blacklist'):
        _validate_runtime_proof_shape(proof, 'current', 11)


def test_three_policy_first_decision_parity_drift_fails():
    proofs = [_proof(policy, seed) for policy, seed in (
        (policy, seed) for policy in ('current', 'nearest', 'gain_nav')
        for seed in (11, 23, 42, 67, 89))]
    # Decision tokens differ by policy, but every decision input must match.
    for seed in (11, 23, 42, 67, 89):
        seed_proofs = [item for item in proofs if item['layout_seed'] == seed]
        baseline = seed_proofs[0]['decisions'][0]
        for proof in seed_proofs[1:]:
            decision = proof['decisions'][0]
            for key in ('map_sequence', 'map_payload_sha256', 'start_record',
                        'blacklist_ids', 'candidate_sampling',
                        'costmap_sha256_before', 'costmap_sha256_after'):
                decision[key] = deepcopy(baseline[key])
    validate_paired_first_decisions(proofs)
    proofs[-1]['decisions'][0]['map_payload_sha256'] = '9' * 64
    with pytest.raises(ValueError, match='first decision parity'):
        validate_paired_first_decisions(proofs)


def test_all_unreachable_requires_three_identical_fresh_attempts():
    proof = _proof()
    proof['status'] = 'FAIL'
    row = {'map_payload_sha256': '2' * 64,
           'start_record_sha256': '3' * 64,
           'candidate_sha256': '4' * 64, 'fresh': True,
           'reachable_count': 0}
    proof['planner_batch_timeline'].extend([
        dict(row, attempt_index=index, map_sequence=index)
        for index in (2, 3, 4)])
    derived = _validate_runtime_proof_shape(proof, 'current', 11)
    assert derived['unreachable_outcome']['status'] == 'UNRESOLVED_FRONTIERS'
    proof['planner_batch_timeline'][-1]['candidate_sha256'] = '5' * 64
    with pytest.raises(ValueError, match='retry identity drift'):
        _validate_runtime_proof_shape(proof, 'current', 11)


def test_planner_batch_timeline_omission_and_dropped_index_fail():
    proof = _proof()
    proof['planner_batch_timeline'][0]['attempt_index'] = 2
    with pytest.raises(ValueError, match='index drift'):
        _validate_runtime_proof_shape(proof, 'current', 11)
    proof = _proof()
    proof['planner_batch_timeline'][0]['reachable_count'] = 0
    with pytest.raises(ValueError, match='ordered binding drift'):
        _validate_runtime_proof_shape(proof, 'current', 11)


@pytest.mark.parametrize('mutate', [
    lambda proof: proof['decisions'][1].update(
        planner_batch_attempt_index=1),
    lambda proof: proof['decisions'].append(deepcopy(proof['decisions'][1])),
    lambda proof: proof['decisions'].pop(),
    lambda proof: proof['decisions'].reverse(),
])
def test_decision_indices_exactly_match_reachable_timeline_order(mutate):
    proof = _proof_with_two_reachable_batches()
    _validate_runtime_proof_shape(proof, 'current', 11)
    mutate(proof)
    with pytest.raises(ValueError, match='ordered binding drift'):
        _validate_runtime_proof_shape(proof, 'current', 11)


def test_pending_unreachable_batch_cannot_be_laundered_as_pass():
    proof = _proof()
    proof['planner_batch_timeline'].append({
        'attempt_index': 2, 'map_sequence': 2,
        'map_payload_sha256': '2' * 64,
        'start_record_sha256': '3' * 64,
        'candidate_sha256': '4' * 64,
        'fresh': True, 'reachable_count': 0})
    with pytest.raises(ValueError, match='PASS contains unresolved'):
        _validate_runtime_proof_shape(proof, 'current', 11)


def test_full_runner_rejects_synthetic_adapter_and_publishes_nothing(
        tmp_path):
    assets = tmp_path / 'assets'
    output = tmp_path / 'result'
    generate(assets, 'full')
    adapter = tmp_path / 'adapter.py'
    adapter.write_text(
        '#!/usr/bin/env python3\n'
        'import json, os\n'
        'request=json.load(open(os.environ["G005_REQUEST_JSON"]))\n'
        'json.dump({"synthetic": True}, '
        'open(os.environ["G005_RESULT_JSON"], "w"))\n',
        encoding='utf-8')
    adapter.chmod(0o755)
    with pytest.raises(RuntimeError, match=FULL_RUNTIME_BLOCKER):
        run_full(assets, output, [str(adapter)], 100, 901.0)
    assert not output.exists()
    assert not [path for path in tmp_path.iterdir()
                if path.name.startswith('.g005-frontier-full-')]


def test_domain_block_rejects_physical_or_overflow_ranges(tmp_path):
    assets = tmp_path / 'assets'
    generate(assets, 'full')
    with pytest.raises(ValueError, match='domain block'):
        build_execution_plan(assets, 12)
    with pytest.raises(ValueError, match='domain block'):
        build_execution_plan(assets, 220)


def test_cli_non_dry_run_reports_exact_builtin_adapter_blocker(tmp_path):
    script = Path(__import__('run_frontier_policy_full').__file__)
    completed = subprocess.run([
        sys.executable, str(script), '--asset-root', str(tmp_path / 'assets'),
        '--output-root', str(tmp_path / 'result'), '--', '/bin/true'],
        capture_output=True, text=True, check=False)
    assert completed.returncode == 2
    assert FULL_RUNTIME_BLOCKER in completed.stderr
    assert not (tmp_path / 'result').exists()
