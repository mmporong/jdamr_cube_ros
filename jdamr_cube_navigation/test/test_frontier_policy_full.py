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

import run_frontier_policy_full as full_runner
from run_frontier_policy_full import (
    _assemble_runtime_proof,
    _gt_metrics,
    _keep_log_tail,
    _runtime_commands,
    _sample_process_groups,
    _stop_process_group,
    _validate_runtime_proof_shape,
    build_execution_plan,
    CLAIM_SCOPE,
    FULL_RUNTIME_BLOCKER,
    FULL_RUNTIME_PATH_ENABLED,
    MEASUREMENT_FIELDS,
    run_full,
    RUNTIME_LOG_TAIL_BYTES,
    SIMULATION_HORIZON_S,
    validate_paired_first_decisions,
    validate_runtime_proof,
)


def _identity(name='input'):
    return {'path': f'/tmp/{name}', 'size_bytes': 10, 'sha256': 'a' * 64}


def _asset_manifest():
    return {'production_inputs': {
        'frontier_core': _identity('frontier_core')}}


def _seal(proof):
    payload = {field: hashlib.sha256(canonical_json_bytes(
        proof[field])).hexdigest() for field in MEASUREMENT_FIELDS}
    payload_sha = hashlib.sha256(canonical_json_bytes(payload)).hexdigest()
    proof['observer_measurement'] = {
        'finalized_before_teardown': True,
        'payload': payload,
        'payload_sha256': payload_sha,
    }
    teardown_payload = {
        'survivor_count': 0,
        'finalized_after_teardown': True,
        'observer_payload_sha256': payload_sha,
    }
    proof['teardown'] = {
        **teardown_payload,
        'final_sha256': hashlib.sha256(canonical_json_bytes(
            teardown_payload)).hexdigest(),
    }
    return proof


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
    return _seal({
        'schema_version': 1, 'claim_scope': CLAIM_SCOPE, 'run_id': run_id,
        'policy': policy, 'layout_seed': seed, 'validity': 'VALID',
        'outcome': 'PASS', 'invalid_reasons': [],
        'request_sha256': 'd' * 64, 'asset_identity': _identity('asset'),
        'asset_manifest_identity': _identity('asset_manifest'),
        'production_inputs_sha256': hashlib.sha256(canonical_json_bytes(
            before)).hexdigest(),
        'shared_initial_sha256': 'e' * 64,
        'shared_reveal_sha256': 'f' * 64,
        'shared_runtime_sha256': '1' * 64,
        'simulation_horizon_s': SIMULATION_HORIZON_S,
        'runtime_identity': {
            'gazebo_actual': True, 'nav2_actual': True,
            'compute_path_action_actual': True,
            'oracle_reveal_map': True,
            'ground_truth_localization': True,
            'not_slam_evaluation': True, 'synthetic_fixture': False,
            'components': {name: _identity(name) for name in (
                'runtime_launch', 'nav2_launch', 'bridge_config',
                'ground_truth_localization', 'observer', 'coordinator',
                'resource_sampler',
                'evaluation_params', 'behavior_tree', 'evaluation_robot')},
            'upstream_versions': {
                'gazebo': '1', 'nav2': '1', 'ros': 'jazzy'}},
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
                {'elapsed_s': 100.0, 'revealed_reachable_cells': 90},
                {'elapsed_s': 900.0, 'revealed_reachable_cells': 90}]},
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
             'clearance_m': 0.1},
            {'steady_elapsed_s': 450.0, 'sim_elapsed_s': 900.0,
             'x_m': 2.1, 'y_m': 0.0,
             'clearance_m': 0.1}],
        'observer_health': {
            'accepted_scan_count': 9000,
            'actual_mean_scan_rate_hz': 10.0,
            'allowed_jitter_ns': 30_000_000,
            'expected_scan_period_ns': 100_000_000,
            'health': 'VALID',
            'last_accepted_stamp_ns': 900_000_000_000,
            'map_payload_sha256': map_sha,
            'map_sequence': 20,
            'observed_max_gap_ns': 103_000_000,
            'observed_scan_count': 9000,
            'rejected_scan_count': 0,
            'rejection_counts': {
                'non_monotonic_stamp': 0,
                'oracle_contract': 0,
                'pose_stale': 0,
                'pose_unavailable': 0,
                'scan_contract': 0,
                'scan_period_jitter': 0,
            },
            'schema_version': 1,
        },
        'contact_count': 0,
        'command_authority': {
            'topic': '/cmd_vel', 'publisher_gids': ['01'],
            'publisher_nodes': ['/collision_monitor'],
            'final_zero_hold_s': 1.0, 'final_linear_x': 0.0,
            'final_angular_z': 0.0},
        'tf_authority': {
            'map_to_odom_publisher_gids': ['02'],
            'map_to_odom_publisher_nodes': ['/g005_frontier_coordinator']},
        'lifecycle': {
            'behavior_server': 'active', 'bt_navigator': 'active',
            'collision_monitor': 'active', 'controller_server': 'active',
            'planner_server': 'active', 'velocity_smoother': 'active'},
        'navigation_outcomes': {'recovery_count': 0, 'failure_count': 0},
        'resources': {'cpu_seconds': 4.0, 'peak_rss_bytes': 1000,
                      'samples': 2},
        'runner_cancellation_count': 0,
        'production_inputs_before': before,
        'production_inputs_after': deepcopy(before),
        'observer_measurement': {}, 'teardown': {},
    })


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
    return _seal(proof)


def test_execution_plan_is_exact_15_and_paired_hashes_match(tmp_path):
    assets = tmp_path / 'assets'
    generate(assets, 'full')
    plan = build_execution_plan(assets, 100)
    assert len(plan['requests']) == 15
    assert plan['runtime_path'] == {
        'enabled': True, 'status': 'READY',
        'blocker': FULL_RUNTIME_BLOCKER,
        'external_adapter_results_accepted': False}
    assert [item['ros_domain_id'] for item in plan['requests']] == list(
        range(100, 115))
    assert set(plan['requests'][0]['runtime_components']) == {
        'behavior_tree', 'bridge_config', 'evaluation_params',
        'evaluation_robot', 'ground_truth_localization', 'nav2_launch',
        'observer', 'coordinator', 'resource_sampler', 'runtime_launch'}
    versions = plan['requests'][0]['upstream_versions_required']
    assert versions['gazebo'].startswith('ros_gz_sim=')
    assert versions['nav2'].startswith('nav2_bringup=')
    assert versions['ros'] == 'jazzy'
    for request in plan['requests']:
        evaluator._validate_execution_request(request)
    seed_11 = [item for item in plan['requests']
               if item['layout_seed'] == 11]
    for key in ('asset_identity', 'shared_initial_sha256',
                'shared_reveal_sha256', 'shared_runtime_sha256'):
        assert len({hashlib.sha256(canonical_json_bytes(item[key])).hexdigest()
                    for item in seed_11}) == 1


def test_execution_request_hash_and_component_bytes_are_sealed(tmp_path):
    assets = tmp_path / 'assets'
    generate(assets, 'full')
    request = build_execution_plan(assets, 100)['requests'][0]
    changed = deepcopy(request)
    changed['ros_domain_id'] += 1
    with pytest.raises(ValueError, match='request hash drift'):
        evaluator._validate_execution_request(changed)
    changed = deepcopy(request)
    changed['runtime_components']['runtime_launch']['sha256'] = '9' * 64
    payload = dict(changed)
    payload.pop('request_sha256')
    changed['request_sha256'] = hashlib.sha256(
        canonical_json_bytes(payload)).hexdigest()
    with pytest.raises(ValueError, match='component disk drift'):
        evaluator._validate_execution_request(changed)


def test_dormant_runtime_proof_shape_derives_metrics_from_traces():
    proof = _proof()
    derived = _validate_runtime_proof_shape(
        proof, 'current', 11, _asset_manifest())
    assert derived['coverage']['coverage_t85_s'] == 90.0
    assert derived['gt']['gt_path_length_to_85_m'] == pytest.approx(2.0)
    assert derived['gt']['minimum_clearance_m'] == pytest.approx(0.1)


def test_gt_path_to_85_interpolates_by_sim_time_inside_segment():
    result = _gt_metrics([
        {'steady_elapsed_s': 0.0, 'sim_elapsed_s': 0.0,
         'x_m': 0.0, 'y_m': 0.0, 'clearance_m': 1.0},
        {'steady_elapsed_s': 50.0, 'sim_elapsed_s': 100.0,
         'x_m': 100.0, 'y_m': 0.0, 'clearance_m': 1.0},
        {'steady_elapsed_s': 450.0, 'sim_elapsed_s': 900.0,
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
         'x_m': 100.0, 'y_m': 0.0, 'clearance_m': 1.0},
        {'steady_elapsed_s': 450.0, 'sim_elapsed_s': 900.0,
         'x_m': 100.0, 'y_m': 0.0, 'clearance_m': 1.0}]
    mutation(rows)
    with pytest.raises(ValueError, match=match):
        _gt_metrics(rows, 90.0)


def test_gt_path_rejects_t85_outside_pose_trace():
    rows = [
        {'steady_elapsed_s': 0.0, 'sim_elapsed_s': 0.0,
         'x_m': 0.0, 'y_m': 0.0, 'clearance_m': 1.0},
        {'steady_elapsed_s': 450.0, 'sim_elapsed_s': 900.0,
         'x_m': 10.0, 'y_m': 0.0, 'clearance_m': 1.0}]
    with pytest.raises(ValueError, match='does not bracket'):
        _gt_metrics(rows, 901.0)


def test_runtime_path_is_enabled_and_validates_owned_proof():
    assert FULL_RUNTIME_PATH_ENABLED is True
    validate_runtime_proof(
        _proof(), 'current', 11, _asset_manifest())


def test_owned_runner_assembles_pre_and_post_teardown_seals():
    expected = _proof()
    request = {
        key: expected[key] for key in (
            'run_id', 'policy', 'layout_seed', 'request_sha256',
            'asset_identity', 'asset_manifest_identity',
            'production_inputs_sha256', 'shared_initial_sha256',
            'shared_reveal_sha256', 'shared_runtime_sha256',
            'simulation_horizon_s')}
    request['runtime_components'] = expected[
        'runtime_identity']['components']
    request['upstream_versions_required'] = expected[
        'runtime_identity']['upstream_versions']
    measurement = {
        key: deepcopy(expected[key]) for key in {
            'validity', 'outcome', 'invalid_reasons',
            *MEASUREMENT_FIELDS}}
    actual = _assemble_runtime_proof(
        request, measurement, expected['production_inputs_before'],
        expected['production_inputs_after'], 0)
    validate_runtime_proof(actual, 'current', 11, _asset_manifest())
    assert actual['observer_measurement']['finalized_before_teardown'] is True
    assert actual['teardown']['finalized_after_teardown'] is True


def test_runtime_log_is_bounded_to_diagnostic_tail(tmp_path):
    path = tmp_path / 'runtime.log'
    path.write_bytes(b'x' * (RUNTIME_LOG_TAIL_BYTES + 100))
    _keep_log_tail(path)
    content = path.read_bytes()
    assert content.startswith(b'G005_RUNTIME_LOG_TRUNCATED_TO_TAIL\n')
    assert len(content) <= RUNTIME_LOG_TAIL_BYTES + 64


def test_owned_runner_terminates_its_process_group():
    process = subprocess.Popen(
        [sys.executable, '-c', 'import time; time.sleep(60)'],
        start_new_session=True)
    assert _stop_process_group(process, grace_s=0.2) == 0
    assert process.poll() is not None


def test_owned_runner_forces_sim_time_for_the_coordinator(tmp_path):
    request = {'asset_root': str(tmp_path), 'layout_seed': 11,
               '_request_path': str(tmp_path / 'request.json')}
    _, coordinator = _runtime_commands(
        request, tmp_path / 'measurement.json')
    assert coordinator[coordinator.index('-p') + 1] == 'use_sim_time:=true'


def test_owned_runner_accumulates_both_process_group_resources(monkeypatch):
    samples = {
        10: iter([(1.0, 100, 2), (1.4, 120, 2)]),
        20: iter([(2.0, 200, 3), (2.7, 180, 2)]),
    }
    monkeypatch.setattr(
        full_runner, 'process_group_totals',
        lambda process_group: next(samples[process_group]))
    state = {'cpu_seconds': 0.0, 'peak_rss_bytes': 0, 'samples': 0,
             'last_cpu_seconds': {}}
    first = _sample_process_groups((10, 20), state)
    second = _sample_process_groups((10, 20), state)
    assert first == {'cpu_seconds': 3.0, 'peak_rss_bytes': 300,
                     'samples': 1}
    assert second['cpu_seconds'] == pytest.approx(4.1)
    assert second['peak_rss_bytes'] == 300
    assert second['samples'] == 2


def test_full_artifact_validator_rejects_incomplete_tree(
        tmp_path):
    root = tmp_path / 'injected'
    root.mkdir()
    with pytest.raises(FileNotFoundError):
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
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


def test_blacklist_must_equal_hard_exclusion_set():
    proof = _proof()
    proof['decisions'][0]['blacklist_ids'] = [99]
    with pytest.raises(ValueError, match='blacklist exclusion'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


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
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


def test_blacklisted_candidate_cannot_be_selected_or_planned():
    proof = _proof()
    selected = proof['decisions'][0][
        'candidate_sampling']['selected_candidate_ids'][0]
    proof['decisions'][0]['blacklist_ids'] = [selected]
    proof['decisions'][0][
        'candidate_sampling']['excluded_candidate_ids'] = [selected]
    with pytest.raises(ValueError, match='blacklist'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


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
            for key in ('map_sequence', 'map_payload_sha256',
                        'blacklist_ids', 'candidate_sampling',
                        'costmap_sha256_before', 'costmap_sha256_after'):
                decision[key] = deepcopy(baseline[key])
            decision['start_record']['frame_id'] = baseline[
                'start_record']['frame_id']
            decision['start_record']['pose_xy_yaw'] = deepcopy(
                baseline['start_record']['pose_xy_yaw'])
            decision['start_record']['stamp_ns'] += 1000
    validate_paired_first_decisions(proofs)
    proofs[-1]['decisions'][0]['map_payload_sha256'] = '9' * 64
    with pytest.raises(ValueError, match='first decision parity'):
        validate_paired_first_decisions(proofs)


def test_all_unreachable_requires_three_identical_fresh_attempts():
    proof = _proof()
    proof['outcome'] = 'FAIL'
    row = {'map_payload_sha256': '2' * 64,
           'start_record_sha256': '3' * 64,
           'candidate_sha256': '4' * 64, 'fresh': True,
           'reachable_count': 0}
    proof['planner_batch_timeline'].extend([
        dict(row, attempt_index=index, map_sequence=2)
        for index in (2, 3, 4)])
    _seal(proof)
    derived = _validate_runtime_proof_shape(
        proof, 'current', 11, _asset_manifest())
    assert derived['unreachable_outcome']['status'] == 'UNRESOLVED_FRONTIERS'
    proof['planner_batch_timeline'][-1]['candidate_sha256'] = '5' * 64
    _seal(proof)
    changed = _validate_runtime_proof_shape(
        proof, 'current', 11, _asset_manifest())
    assert changed['unreachable_outcome']['status'] == 'PENDING'


def test_same_map_sequence_cannot_claim_a_different_payload():
    proof = _proof_with_two_reachable_batches()
    proof['planner_batch_timeline'][1]['map_sequence'] = 1
    _seal(proof)
    with pytest.raises(ValueError, match='map identity drift'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


def test_planner_batch_timeline_omission_and_dropped_index_fail():
    proof = _proof()
    proof['planner_batch_timeline'][0]['attempt_index'] = 2
    with pytest.raises(ValueError, match='index drift'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())
    proof = _proof()
    proof['planner_batch_timeline'][0]['reachable_count'] = 0
    with pytest.raises(ValueError, match='ordered binding drift'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


@pytest.mark.parametrize('mutate', [
    lambda proof: proof['decisions'][1].update(
        planner_batch_attempt_index=1),
    lambda proof: proof['decisions'].append(deepcopy(proof['decisions'][1])),
    lambda proof: proof['decisions'].pop(),
    lambda proof: proof['decisions'].reverse(),
])
def test_decision_indices_exactly_match_reachable_timeline_order(mutate):
    proof = _proof_with_two_reachable_batches()
    _validate_runtime_proof_shape(
        proof, 'current', 11, _asset_manifest())
    mutate(proof)
    with pytest.raises(ValueError, match='ordered binding drift'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


def test_pending_unreachable_batch_cannot_be_laundered_as_pass():
    proof = _proof()
    proof['planner_batch_timeline'].append({
        'attempt_index': 2, 'map_sequence': 2,
        'map_payload_sha256': '2' * 64,
        'start_record_sha256': '3' * 64,
        'candidate_sha256': '4' * 64,
        'fresh': True, 'reachable_count': 0})
    with pytest.raises(ValueError, match='PASS contains unresolved'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


def test_valid_early_failure_preserves_horizon_and_integrity():
    proof = _proof()
    proof['coverage_samples']['samples'][-1][
        'revealed_reachable_cells'] = 80
    proof['coverage_samples']['samples'][-2][
        'revealed_reachable_cells'] = 80
    proof['coverage_samples']['samples'][-3][
        'revealed_reachable_cells'] = 75
    proof['outcome'] = 'FAIL'
    _seal(proof)
    derived = _validate_runtime_proof_shape(
        proof, 'current', 11, _asset_manifest())
    assert derived['coverage']['coverage_t85_s'] is None
    assert derived['coverage']['elapsed_s'] == SIMULATION_HORIZON_S
    assert derived['gt']['gt_path_length_to_85_m'] is None


def test_valid_contact_failure_is_not_rejected_as_invalid_proof():
    proof = _proof()
    proof['contact_count'] = 1
    proof['outcome'] = 'FAIL'
    _seal(proof)
    derived = _validate_runtime_proof_shape(
        proof, 'current', 11, _asset_manifest())
    assert derived['coverage']['final_coverage_ratio'] == 0.9


def test_invalid_runtime_is_preserved_with_reason_and_cannot_pass(
        monkeypatch):
    proof = _proof()
    proof['validity'] = 'INVALID'
    proof['outcome'] = 'FAIL'
    proof['invalid_reasons'] = ['OBSERVER_SCAN_RATE_DRIFT']
    proof['observer_health']['health'] = 'INVALID'
    proof['observer_health']['rejected_scan_count'] = 1
    proof['observer_health']['rejection_counts'][
        'scan_period_jitter'] = 1
    _seal(proof)
    derived = _validate_runtime_proof_shape(
        proof, 'current', 11, _asset_manifest())
    assert derived['coverage']['elapsed_s'] == SIMULATION_HORIZON_S
    monkeypatch.setattr(full_runner, 'FULL_RUNTIME_PATH_ENABLED', True)
    compact = full_runner.compact_runtime_proof(proof, _asset_manifest())
    assert compact['validity'] == 'INVALID'
    assert compact['outcome'] == 'FAIL'
    assert compact['invalid_reasons'] == ['OBSERVER_SCAN_RATE_DRIFT']
    evaluator._validate_full_evidence(compact, 'current', 11)


def test_invalid_runtime_requires_reason_and_fail_outcome():
    proof = _proof()
    proof['validity'] = 'INVALID'
    _seal(proof)
    with pytest.raises(ValueError, match='schema drift'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


def test_lifecycle_requires_exact_nav2_node_set():
    proof = _proof()
    proof['lifecycle'] = {'not_a_nav2_node': 'active'}
    _seal(proof)
    with pytest.raises(ValueError, match='lifecycle drift'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


@pytest.mark.parametrize('field,value', [
    ('actual_mean_scan_rate_hz', 4.0),
    ('actual_mean_scan_rate_hz', 20.0),
    ('observed_max_gap_ns', 250_000_000),
])
def test_valid_observer_health_rejects_scan_rate_drift(field, value):
    proof = _proof()
    proof['observer_health'][field] = value
    _seal(proof)
    with pytest.raises(ValueError, match='observer health evidence drift'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


def test_compact_proof_preserves_every_decision_score_batch(monkeypatch):
    proof = _proof_with_two_reachable_batches()
    monkeypatch.setattr(full_runner, 'FULL_RUNTIME_PATH_ENABLED', True)
    compact = full_runner.compact_runtime_proof(proof, _asset_manifest())
    assert [item['decision_index'] for item in
            compact['score_decompositions']] == [1, 2]
    assert [item['decision_token'] for item in
            compact['score_decompositions']] == [
                item['decision_token'] for item in proof['decisions']]
    evaluator._validate_full_evidence(compact, 'current', 11)


def test_valid_no_reachable_decision_failure_compacts_without_fabrication(
        monkeypatch):
    proof = _proof()
    proof['decisions'] = []
    proof['planner_batch_timeline'] = [{
        'attempt_index': index, 'map_sequence': 1,
        'map_payload_sha256': '2' * 64,
        'start_record_sha256': '3' * 64,
        'candidate_sha256': '4' * 64,
        'fresh': True, 'reachable_count': 0,
    } for index in (1, 2, 3)]
    proof['outcome'] = 'FAIL'
    _seal(proof)
    monkeypatch.setattr(full_runner, 'FULL_RUNTIME_PATH_ENABLED', True)
    compact = full_runner.compact_runtime_proof(proof, _asset_manifest())
    assert compact['first_goal_cell_index'] == -1
    assert compact['first_decision_reachable_count'] == 0
    assert compact['score_decompositions'] == []
    evaluator._validate_full_evidence(compact, 'current', 11)


def test_runtime_proof_rejects_short_horizon_and_seal_tampering():
    proof = _proof()
    proof['coverage_samples']['samples'][-1]['elapsed_s'] = 899.0
    _seal(proof)
    with pytest.raises(ValueError, match='exact horizon'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())
    proof = _proof()
    proof['coverage_samples']['samples'][0]['elapsed_s'] = 1.0
    _seal(proof)
    with pytest.raises(ValueError, match='exact horizon'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())
    proof = _proof()
    proof['observer_measurement']['payload_sha256'] = '9' * 64
    with pytest.raises(ValueError, match='measurement seal'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, _asset_manifest())


def test_production_inputs_are_bound_to_asset_manifest():
    proof = _proof()
    changed_manifest = _asset_manifest()
    changed_manifest['production_inputs']['frontier_core']['sha256'] = '9' * 64
    with pytest.raises(ValueError, match='manifest binding'):
        _validate_runtime_proof_shape(
            proof, 'current', 11, changed_manifest)


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
    with pytest.raises(ValueError, match='external runtime adapters'):
        run_full(assets, output, [str(adapter)], 100, 901.0)
    assert not output.exists()
    assert not [path for path in tmp_path.iterdir()
                if path.name.startswith('.g005-frontier-full-')]


def test_full_runner_atomically_seals_owned_exact_matrix(
        tmp_path, monkeypatch):
    assets = tmp_path / 'assets'
    output = tmp_path / 'result'
    generate(assets, 'full')

    def fake_execute(request, stage, timeout_s, production_inputs):
        del stage, timeout_s
        proof = _proof(request['policy'], request['layout_seed'])
        for key in (
                'request_sha256', 'asset_identity',
                'asset_manifest_identity', 'production_inputs_sha256',
                'shared_initial_sha256', 'shared_reveal_sha256',
                'shared_runtime_sha256', 'simulation_horizon_s'):
            proof[key] = deepcopy(request[key])
        proof['runtime_identity']['components'] = deepcopy(
            request['runtime_components'])
        proof['runtime_identity']['upstream_versions'] = deepcopy(
            request['upstream_versions_required'])
        proof['production_inputs_before'] = deepcopy(production_inputs)
        proof['production_inputs_after'] = deepcopy(production_inputs)
        return _seal(proof)

    monkeypatch.setattr(full_runner, '_execute_one_request', fake_execute)
    manifest = run_full(assets, output, [], 100, 901.0)
    assert manifest['executed_plan'] == [
        {'policy': policy, 'layout_seed': seed}
        for policy, seed in full_runner.FULL_PLAN]
    assert len(manifest['runtime_proofs']) == 15
    assert output.is_dir()
    evaluator.validate_artifact(output, 'full')


def test_domain_block_rejects_physical_or_overflow_ranges(tmp_path):
    assets = tmp_path / 'assets'
    generate(assets, 'full')
    with pytest.raises(ValueError, match='domain block'):
        build_execution_plan(assets, 12)
    with pytest.raises(ValueError, match='domain block'):
        build_execution_plan(assets, 220)


def test_cli_rejects_external_runtime_adapter(tmp_path):
    script = Path(__import__('run_frontier_policy_full').__file__)
    completed = subprocess.run([
        sys.executable, str(script), '--asset-root', str(tmp_path / 'assets'),
        '--output-root', str(tmp_path / 'result'), '--', '/bin/true'],
        capture_output=True, text=True, check=False)
    assert completed.returncode == 2
    assert 'external runtime adapters are not accepted' in completed.stderr
    assert not (tmp_path / 'result').exists()
