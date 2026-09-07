#!/usr/bin/env python3
"""Boundary and hostile tests for G002 Axis B contracts."""

import copy
import math
from pathlib import Path
import signal
from types import SimpleNamespace

from amcl_fault_contract import canonical_json_bytes, sha256_file
import axis_b_kidnapped_driver as kidnapped_driver
from axis_b_kidnapped_driver import (
    rotation_command_radps,
    ROTATION_TARGET_RAD,
    validate_kidnapped_trace,
)
import evaluate_amcl_axis_b as evaluator
import generate_amcl_axis_b_input as input_generator
from generate_amcl_axis_b_input import SCENARIOS, SOURCE_TOPICS
import pytest
import run_amcl_axis_b as axis_b_runner
from run_amcl_axis_b import (
    _execution_contract,
    _pair_gt,
    initial_estimate,
    parse_input_roots,
)
from run_amcl_axis_b import readiness
import run_amcl_determinism_preflight as preflight


def _cloud(x, yaw=0.0, variance=1.0):
    import math
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = covariance[35] = variance
    return {'pose': [x, 0.0, 0.0, 0.0, 0.0,
                     math.sin(yaw / 2.0), math.cos(yaw / 2.0)],
            'covariance': covariance,
            'fifo_associated_pose_scan_header_stamp_ns': 100}


def _pair(index, x=0.0):
    return {'scan_index': index, 'scan_header_stamp_ns': 100,
            'gt_before_stamp_ns': 100, 'gt_after_stamp_ns': 100,
            'gt_max_delta_ns': 0, 'gt_interpolation_fraction': 0.0,
            'gt_frame_id': 'map',
            'gt_pose': [x, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]}


def test_recovery_requires_three_consecutive_and_accepts_boundary():
    clouds = [_cloud(0.15, 0.25)] * 3
    result = evaluator.recovery_metrics(
        clouds, [_pair(5), _pair(6), _pair(7)], 5)
    assert result['recovered'] is True
    assert result['first_recovery']['scans_after_t0'] == 2
    result = evaluator.recovery_metrics(
        [_cloud(0.150000001)] * 3, [_pair(5), _pair(6), _pair(7)], 5)
    assert result['recovered'] is False


def test_false_convergence_requires_three_confident_wrong_poses():
    variance = (0.15 / 3.0) ** 2
    result = evaluator.recovery_metrics(
        [_cloud(1.0, variance=variance)] * 3,
        [_pair(0), _pair(1), _pair(2)], 0)
    assert result['false_convergence'] is True


def test_promotion_is_not_evaluated_without_exact_full_inputs():
    assert evaluator.promotion_decision([], None) == {
        'status': 'NOT_EVALUATED', 'promote_p2': False,
        'selected_profile': 'P0'}


def test_promotion_exact_thresholds_pass():
    rows = []
    for scenario, profile, seed in evaluator.FULL_PLAN:
        recovery = 10
        if scenario == 'kidnapped' and profile == 'P0':
            recovery = 12
        if scenario == 'kidnapped' and profile == 'P2':
            recovery = 11 if seed in (11, 23, 42) else 12
        rows.append({'scenario': scenario, 'profile': profile, 'seed': seed,
                     'recovered': True, 'false_convergence': False,
                     'recovery_scan': recovery})
    result = evaluator.promotion_decision(rows, {
        'cpu_ratios_by_seed': [1.05] * 5,
        'latency_p95_ratios_by_seed': [1.05] * 5,
        'cpu_median': 1.05, 'latency_p95_median': 1.05,
        'individual_max': 1.05})
    assert result['promote_p2'] is True
    rows[-1]['false_convergence'] = True
    assert evaluator.promotion_decision(rows, {
        'cpu_ratios_by_seed': [1.0] * 5,
        'latency_p95_ratios_by_seed': [1.0] * 5,
        'cpu_median': 1.0, 'latency_p95_median': 1.0,
        'individual_max': 1.0})['promote_p2'] is False


def test_obsolete_authority_fallback_is_removed():
    assert not hasattr(evaluator, 'authority_limited_promotion')


def test_axis_b_runner_rejects_map_identity_before_execution(
        monkeypatch, tmp_path):
    contract = {
        'production_inputs': {},
        'axis_b': {'map_profile': {
            'yaml': {'identity': 'expected-map'}}},
    }
    monkeypatch.setattr(axis_b_runner, 'readiness',
                        lambda *_args: {'status': 'PASS'})
    monkeypatch.setattr(axis_b_runner, '_execution_contract',
                        lambda _state: ([], {}))
    monkeypatch.setattr(axis_b_runner, 'current_free_bytes',
                        lambda _path: 7 * 1024 ** 3)
    monkeypatch.setattr(axis_b_runner, 'strict_json_load',
                        lambda _path: contract)
    monkeypatch.setattr(preflight, '_validate_source_records', lambda _: None)
    monkeypatch.setattr(preflight, '_identity',
                        lambda _path: {'identity': 'wrong-map'})
    args = SimpleNamespace(
        output_root=tmp_path / 'out', input=[], axis_a_root=None,
        mode='smoke', domain_base=180, prepared_root=tmp_path,
        map_yaml=tmp_path / 'map.yaml')
    with pytest.raises(ValueError, match='map snapshot identity'):
        axis_b_runner.run(args)


def test_axis_b_claims_disclose_fixed_input_forced_rotation_limits():
    assert evaluator.EVALUATION_DESIGN == 'FIXED_INPUT_ESTIMATOR_REPLAY'
    assert evaluator.CLOSED_LOOP_RECOVERY_BEHAVIOR_EVALUATED is False
    assert evaluator.OBSERVATION_SCHEDULE_SOURCE == \
        'PRERECORDED_FORCED_ROTATION'
    assert evaluator.SEED_SEMANTICS == 'ESTIMATOR_STOCHASTICITY_ONLY'
    assert evaluator.INPUT_DIVERSITY_ACROSS_ESTIMATOR_SEEDS is False
    assert evaluator.GT_USAGE == 'EVALUATION_ONLY_NOT_ESTIMATOR_INPUT'
    for claim in (evaluator.CLAIM_SMOKE, evaluator.CLAIM_FULL):
        assert 'FIXED_INPUT' in claim
        assert 'FORCED_ROTATION' in claim
        assert 'NON_CLOSED_LOOP' in claim


def test_promotion_rejects_duplicate_and_unrecovered_rows():
    rows = [{'scenario': scenario, 'profile': profile, 'seed': seed,
             'recovered': True, 'false_convergence': False,
             'recovery_scan': 10}
            for scenario, profile, seed in evaluator.FULL_PLAN]
    ratios = {'cpu_ratios_by_seed': [1.0] * 5,
              'latency_p95_ratios_by_seed': [1.0] * 5,
              'cpu_median': 1.0, 'latency_p95_median': 1.0,
              'individual_max': 1.0}
    rows[-1] = dict(rows[0])
    with pytest.raises(ValueError, match='result set'):
        evaluator.promotion_decision(rows, ratios)
    rows = [{'scenario': scenario, 'profile': profile, 'seed': seed,
             'recovered': True, 'false_convergence': False,
             'recovery_scan': 10}
            for scenario, profile, seed in evaluator.FULL_PLAN]
    target = next(row for row in rows if row['scenario'] == 'kidnapped' and
                  row['profile'] == 'P2')
    target['recovered'] = False
    target['recovery_scan'] = None
    assert evaluator.promotion_decision(rows, ratios)['promote_p2'] is False


def test_promotion_recomputes_axis_a_ratio_summary():
    rows = [{'scenario': scenario, 'profile': profile, 'seed': seed,
             'recovered': True, 'false_convergence': False,
             'recovery_scan': 10}
            for scenario, profile, seed in evaluator.FULL_PLAN]
    ratios = {'cpu_ratios_by_seed': [1.0] * 5,
              'latency_p95_ratios_by_seed': [1.0] * 5,
              'cpu_median': 1.01, 'latency_p95_median': 1.0,
              'individual_max': 1.0}
    with pytest.raises(ValueError, match='recomputation'):
        evaluator.promotion_decision(rows, ratios)


def test_axis_a_ratio_load_rejects_post_validation_manifest_swap(
        monkeypatch, tmp_path):
    manifest = tmp_path / 'axis_a_manifest.json'
    manifest.write_text('{}', encoding='utf-8')
    monkeypatch.setattr(evaluator, 'validate_axis_a_full',
                        lambda root: {'runs': []})
    original_identity = preflight._identity(manifest)
    calls = iter((original_identity,
                  dict(original_identity, sha256='0' * 64)))
    monkeypatch.setattr(evaluator.preflight, '_identity',
                        lambda path: next(calls))
    with pytest.raises(ValueError, match='changed after validation'):
        evaluator.axis_a_ratios(tmp_path)


def test_axis_a_ratio_load_rejects_post_validation_run_swap(
        monkeypatch, tmp_path):
    manifest = tmp_path / 'axis_a_manifest.json'
    manifest.write_text('{}', encoding='utf-8')
    run_path = tmp_path / 'run.json'
    run_path.write_text('{}', encoding='utf-8')
    record = {
        'relative_path': 'run.json', 'size_bytes': run_path.stat().st_size,
        'sha256': sha256_file(run_path)}

    def validated_then_swapped(root):
        assert root == tmp_path
        run_path.write_text('{"changed":true}', encoding='utf-8')
        return {'runs': [record]}

    monkeypatch.setattr(
        evaluator, 'validate_axis_a_full', validated_then_swapped)
    with pytest.raises(ValueError, match='run changed after validation'):
        evaluator.axis_a_ratios(tmp_path)


def test_axis_a_ratio_load_rejects_late_earlier_run_swap(
        monkeypatch, tmp_path):
    manifest_path = tmp_path / 'axis_a_manifest.json'
    manifest_path.write_text('{}', encoding='utf-8')
    records = []
    paths = []
    for profile in ('P0', 'P2'):
        for seed in evaluator.SEEDS:
            path = tmp_path / f'{profile}_{seed}.json'
            path.write_bytes(canonical_json_bytes({
                'profile': profile, 'seed': seed,
                'metrics': {
                    'amcl_cpu_seconds_per_1000_scans': 1.0,
                    'scan_to_pose_latency_ms': {'p95': 1.0}}}))
            paths.append(path)
            records.append({
                'relative_path': path.name,
                'size_bytes': path.stat().st_size,
                'sha256': sha256_file(path)})
    monkeypatch.setattr(
        evaluator, 'validate_axis_a_full', lambda root: {'runs': records})
    original_median = evaluator.statistics.median
    changed = False

    def median_then_swap(values):
        nonlocal changed
        if not changed:
            paths[0].write_text('{"late":"swap"}', encoding='utf-8')
            changed = True
        return original_median(values)

    monkeypatch.setattr(evaluator.statistics, 'median', median_then_swap)
    with pytest.raises(ValueError, match='during ratio computation'):
        evaluator.axis_a_ratios(tmp_path)


def test_scenarios_and_topic_mapping_are_preregistered():
    assert list(SCENARIOS) == ['correct_init', 'initial_offset', 'kidnapped']
    assert SCENARIOS['initial_offset']['initial_estimate_offset'] == [
        0.5, 0.0, 0.2617993877991494]
    assert SCENARIOS['kidnapped']['teleport_pose'] == [
        0.0, 0.0, 3.141592653589793]
    assert SOURCE_TOPICS == {
        '/sim_raw/scan': '/scan',
        '/odom': '/odom',
        '/tf': '/tf',
        '/tf_static': '/tf_static',
        '/ground_truth_pose': '/ground_truth_pose',
        '/cmd_vel': '/cmd_vel',
        input_generator.CONTACT_TOPIC: '/contact',
    }
    assert set(SOURCE_TOPICS.values()) == {
        '/scan', '/odom', '/tf', '/tf_static',
        '/ground_truth_pose', '/cmd_vel', '/contact'}
    assert initial_estimate('initial_offset') == [
        -7.5, 0.0, 0.2617993877991494]


def test_every_scenario_preregisters_forced_rotation_observations():
    for scenario, contract in SCENARIOS.items():
        assert contract['observation_rotation_rad'] > 0.0


def test_non_kidnapped_capture_driver_is_not_stationary_only():
    source = Path(input_generator.__file__).read_text(encoding='utf-8')
    assert 'run_stationary_capture_driver' not in source
    assert 'command.angular.z = OBSERVATION_ANGULAR_SPEED_RADPS' in source


def _full_rotation_samples():
    period_ns = input_generator.LIDAR_PERIOD_NS
    count = 127
    commands = [(index * period_ns, 0.5) for index in range(count)]
    scans = [(index * period_ns, index * period_ns)
             for index in range(count)]
    poses = [(index * period_ns, index * 0.05)
             for index in range(count)]
    return commands, scans, poses


def test_motion_evidence_proves_cmd_vel_gt_rotation_and_scan_continuity():
    commands, scans, poses = _full_rotation_samples()
    evidence = input_generator._motion_evidence_from_samples(
        commands, scans, poses, 'correct_init')
    assert evidence['nonzero_cmd_vel_count'] == 127
    assert evidence['rotation_window_scan_count'] == 127
    assert evidence['scan_header_stamps_unique_monotonic'] is True
    assert evidence['max_scan_header_gap_ns'] == 100_000_000
    assert evidence['signed_gt_rotation_rad'] == pytest.approx(6.3)
    assert evidence['reverse_gt_rotation_rad'] == 0.0
    assert evidence['maximum_duration_s'] > evidence['scheduled_duration_s']


def test_motion_evidence_rejects_yaw_wrap_without_physical_rotation():
    commands, scans, _ = _full_rotation_samples()
    poses = [(0, 0.0), (commands[-1][0], 2.0 * 3.141592653589793)]
    with pytest.raises(ValueError, match='scan continuity gate'):
        input_generator._motion_evidence_from_samples(
            commands, scans, poses, 'correct_init')


def test_motion_evidence_rejects_lidar_gap_in_rotation_window():
    commands, scans, poses = _full_rotation_samples()
    del scans[50:54]
    with pytest.raises(ValueError, match='scan continuity gate'):
        input_generator._motion_evidence_from_samples(
            commands, scans, poses, 'correct_init')


def test_motion_evidence_rejects_back_and_forth_yaw_oscillation():
    commands, scans, _ = _full_rotation_samples()
    poses = [(stamp, 0.1 if index % 2 else 0.0)
             for index, (stamp, _) in enumerate(commands)]
    with pytest.raises(ValueError, match='scan continuity gate'):
        input_generator._motion_evidence_from_samples(
            commands, scans, poses, 'correct_init')


def test_motion_evidence_rejects_non_finite_gt_yaw():
    commands, scans, poses = _full_rotation_samples()
    poses[50] = (poses[50][0], float('nan'))
    with pytest.raises(ValueError, match='GT yaw is non-finite'):
        input_generator._motion_evidence_from_samples(
            commands, scans, poses, 'correct_init')


def test_motion_evidence_rejects_four_pi_rotation():
    commands, scans, _ = _full_rotation_samples()
    poses = [(stamp, index * 0.1)
             for index, (stamp, _) in enumerate(commands)]
    with pytest.raises(ValueError, match='scan continuity gate'):
        input_generator._motion_evidence_from_samples(
            commands, scans, poses, 'correct_init')


def test_motion_evidence_rejects_excessive_rotation_duration():
    commands, scans, poses = _full_rotation_samples()
    scale = 2
    commands = [(stamp * scale, angular) for stamp, angular in commands]
    scans = [(storage * scale, header * scale) for storage, header in scans]
    poses = [(stamp * scale, yaw) for stamp, yaw in poses]
    with pytest.raises(ValueError, match='scan continuity gate'):
        input_generator._motion_evidence_from_samples(
            commands, scans, poses, 'correct_init')


def test_scan_sampled_odom_tf_update_count_uses_strict_threshold():
    scans = [0, 100, 200, 300, 400, 500]
    odom = [
        {'stamp_ns': 0, 'pose': [0.0, 0.0, 0.0]},
        {'stamp_ns': 100, 'pose': [0.0, 0.0, 0.0]},
        {'stamp_ns': 200, 'pose': [0.0, 0.0, 0.2]},
        {'stamp_ns': 300, 'pose': [0.0, 0.0, 0.200000001]},
        {'stamp_ns': 400, 'pose': [0.0, 0.0, 0.400000001]},
        {'stamp_ns': 500, 'pose': [0.0, 0.0, 0.400000002]},
    ]
    assert evaluator._accepted_scan_indices(scans, odom) == [0, 3, 5]


def test_amcl_motion_gate_is_bound_to_frozen_parameter_values(tmp_path):
    params = tmp_path / 'nav2_params.yaml'
    params.write_text(
        'amcl:\n  ros__parameters:\n'
        '    update_min_a: 0.2\n    update_min_d: 0.25\n',
        encoding='utf-8')
    contract = evaluator._amcl_motion_gate_contract(params)
    assert contract['comparison'] == 'STRICT_GREATER_THAN'
    assert contract['pose_source'] == \
        'SCAN_TIME_ODOM_TO_BASE_FOOTPRINT_TF'
    params.write_text(
        'amcl:\n  ros__parameters:\n'
        '    update_min_a: 0.21\n    update_min_d: 0.25\n',
        encoding='utf-8')
    with pytest.raises(ValueError, match='threshold drift'):
        evaluator._amcl_motion_gate_contract(params)


def _kidnapped_trace():
    names = ['initial_observation_complete', 'stationary_confirmed',
             'teleport_requested', 'teleport_ack',
             'teleport_gt_verified', 't0_scan', 'rotation_complete',
             'zero_hold_complete', 'post_zero_observation_complete',
             'final_zero_hold_complete']
    return {
        'schema_version': 1,
        'driver_source': {
            'path': str(Path(kidnapped_driver.__file__).resolve()),
            'size_bytes': Path(kidnapped_driver.__file__).resolve().stat().st_size,
            'sha256': sha256_file(Path(kidnapped_driver.__file__).resolve())},
        'events': [dict(
            {'name': name, 'steady_ns': index + 1,
             'ros_ns': 1000 + index},
            **({'scan_header_stamp_ns': 175}
               if name == 't0_scan' else {}))
            for index, name in enumerate(names)],
        'pre_teleport_gt': [-8.0, 0.0, 0.0],
        'post_teleport_gt': [0.0, 0.0, 3.141592653589793],
        'pre_teleport_odom': [1.0, 2.0, 0.0],
        'post_teleport_odom': [1.05, 2.0, 0.0],
        'teleport_gt_verified_header_stamp_ns': 150,
        't0_scan_stamp_ns': 175,
        'unwrapped_observation_yaw_rad': 6.283185307179586,
        'reverse_observation_yaw_rad': 0.0,
        'initial_observation_yaw_rad': 0.8,
        'initial_reverse_yaw_rad': 0.0,
        'post_zero_observation_yaw_rad': 1.2,
        'post_zero_reverse_yaw_rad': 0.0,
        'final_zero': True, 'zero_hold_s': 1.0,
        'stationary_sample_count': 10,
        'post_teleport_initialpose_count': 0, 'failure': None,
    }


def test_kidnapped_trace_accepts_exact_boundary():
    assert validate_kidnapped_trace(_kidnapped_trace())['final_zero'] is True


def test_kidnapped_trace_uses_gt_header_not_later_callback_clock():
    value = _kidnapped_trace()
    verified = next(event for event in value['events']
                    if event['name'] == 'teleport_gt_verified')
    assert verified['ros_ns'] > value['t0_scan_stamp_ns']
    assert validate_kidnapped_trace(value) == value


def test_kidnapped_boundary_gt_sample_matches_trace_header_and_pose():
    trace = _kidnapped_trace()
    samples = [
        {'stamp_ns': 149, 'pose': [-8.0, 0.0, 0.0]},
        {'stamp_ns': 150, 'pose': [0.0, 0.0, math.pi]},
        {'stamp_ns': 175, 'pose': [0.0, 0.0, math.pi]},
    ]
    assert input_generator._validate_kidnapped_boundary_samples(
        samples, trace) == {
            'header_stamp_ns': 150,
            'pose': [0.0, 0.0, math.pi],
            'matching_sample_count': 1,
        }


@pytest.mark.parametrize('samples', [
    [{'stamp_ns': 149, 'pose': [-8.0, 0.0, 0.0]}],
    [{'stamp_ns': 150, 'pose': [-8.0, 0.0, 0.0]}],
    [
        {'stamp_ns': 150, 'pose': [0.0, 0.0, math.pi]},
        {'stamp_ns': 150, 'pose': [0.0, 0.0, math.pi]},
    ],
])
def test_kidnapped_boundary_gt_sample_hostile_cases_fail(samples):
    with pytest.raises(ValueError, match='boundary'):
        input_generator._validate_kidnapped_boundary_samples(
            samples, _kidnapped_trace())


def test_generator_source_identity_rejects_path_and_hash_tampering():
    identity = input_generator._generator_source_identity()
    assert input_generator._generator_source_valid(identity) is True
    changed = dict(identity, sha256='0' * 64)
    assert input_generator._generator_source_valid(changed) is False
    changed = dict(identity, path='/tmp/fake-generator.py')
    assert input_generator._generator_source_valid(changed) is False


def test_frozen_generator_source_does_not_depend_on_live_worktree(
        monkeypatch):
    frozen = input_generator._generator_source_identity()
    live = dict(frozen, sha256='0' * 64)
    monkeypatch.setattr(
        input_generator, '_generator_source_identity', lambda: live)

    assert input_generator._generator_source_valid(
        frozen, frozen) is True
    assert input_generator._generator_source_valid(frozen) is False


def test_kidnapped_rotation_command_slows_before_exact_target():
    assert rotation_command_radps(0.0) == 0.5
    assert rotation_command_radps(
        ROTATION_TARGET_RAD - 0.10) == pytest.approx(0.10)
    assert rotation_command_radps(ROTATION_TARGET_RAD - 0.01) == 0.05
    assert rotation_command_radps(ROTATION_TARGET_RAD) == 0.0


@pytest.mark.parametrize('mutation', [
    lambda value: value.update({'final_zero': False}),
    lambda value: value.update({'unwrapped_observation_yaw_rad': 6.0}),
    lambda value: value.update({'reverse_observation_yaw_rad': 0.1}),
    lambda value: value.update({'reverse_observation_yaw_rad': -0.1}),
    lambda value: value.update({'initial_reverse_yaw_rad': -0.1}),
    lambda value: value.update({'post_zero_reverse_yaw_rad': -0.1}),
    lambda value: value.update({'post_teleport_gt': [0.2, 0.0, 3.14]}),
    lambda value: value.update({'post_teleport_odom': [1.2, 2.0, 0.0]}),
    lambda value: value.update({'post_teleport_initialpose_count': 1}),
    lambda value: value.update({'t0_scan_stamp_ns': True}),
    lambda value: next(event for event in value['events']
                       if event['name'] == 't0_scan').update(
                           {'scan_header_stamp_ns': 11}),
    lambda value: value.update({'t0_scan_stamp_ns': 149}) or
    next(event for event in value['events']
         if event['name'] == 't0_scan').update({'scan_header_stamp_ns': 149}),
    lambda value: value.update({
        'teleport_gt_verified_header_stamp_ns': 176}),
    lambda value: next(event for event in value['events']
                       if event['name'] == 'zero_hold_complete').update(
                           {'ros_ns': 6}),
    lambda value: value.update({'zero_hold_s': float('nan')}),
    lambda value: value['events'][0].update({'extra': 1}),
    lambda value: value.update({'unwrapped_observation_yaw_rad': 6.31}),
    lambda value: value['driver_source'].update({'sha256': '0' * 64}),
    lambda value: value['driver_source'].update({'path': '/tmp/fake.py'}),
    lambda value: value['events'].reverse(),
])
def test_kidnapped_trace_hostile_mutations_fail(mutation):
    value = _kidnapped_trace()
    mutation(value)
    with pytest.raises(ValueError):
        validate_kidnapped_trace(value)


def test_manifest_rejects_unknown_before_reading_external_inputs(tmp_path):
    full_plan = [{'scenario': scenario, 'profile': profile, 'seed': seed}
                 for scenario, profile, seed in evaluator.FULL_PLAN]
    value = {'schema_version': 2, 'mode': 'smoke',
             'claim_scope': 'AXIS_B_RELOCALIZATION_SMOKE_SIMULATION_ONLY',
             'full_plan': full_plan,
             'executed_plan': [
                 {'scenario': 'correct_init', 'profile': 'P0', 'seed': 11}],
             'input_roots': [], 'axis_a_artifact': None,
             'prepared_contract': {}, 'runtime_attestation': {},
             'harness_sources': {}, 'run_contract': {},
             'runs': [{'relative_path': 'run_1/axis_b_evidence.json'}],
             'promotion': {'status': 'NOT_EVALUATED', 'promote_p2': False,
                           'selected_profile': 'P0'},
             'tree_records': [], 'tree_sha256': '0' * 64,
             'tree_bytes': 0, 'production_unchanged': True,
             'unknown': True}
    path = tmp_path / 'manifest.json'
    path.write_bytes(canonical_json_bytes(value))
    with pytest.raises(ValueError, match='schema'):
        evaluator.validate_manifest(path, 'smoke')


def test_full_readiness_is_pending_without_three_inputs_and_axis_a():
    result = readiness('full', {}, None)
    assert result['status'] == 'PENDING'
    assert result['executes_ros'] is False
    assert result['synthetic_inputs_canonical_eligible'] is False
    assert set(result['blockers']) == {
        'missing_canonical_input:correct_init',
        'missing_canonical_input:initial_offset',
        'missing_canonical_input:kidnapped',
        'missing_axis_a_full_artifact',
    }
    assert len(result['plan']) == 45


def test_smoke_execution_contract_ignores_unused_cli_inputs(monkeypatch):
    roots = {scenario: Path('/tmp') / scenario for scenario in SCENARIOS}
    validated = []

    def validate_only_required(root):
        validated.append(root.name)
        if root.name != 'correct_init':
            raise AssertionError('smoke validated an unused CLI input')
        return {'scenario': root.name}

    monkeypatch.setattr(
        axis_b_runner, 'validate_input', validate_only_required)
    monkeypatch.setattr(preflight, '_identity', lambda path: {
        'path': str(path), 'size_bytes': 1, 'sha256': '0' * 64})
    state = readiness('smoke', roots, None)
    plan, selected = _execution_contract(state)
    assert plan == [('correct_init', 'P0', 11)]
    assert selected == {'correct_init': roots['correct_init']}
    assert validated == ['correct_init']
    assert state['canonical_inputs'][0]['scenario'] == 'correct_init'


def test_full_execution_contract_keeps_all_three_inputs(monkeypatch):
    roots = {scenario: Path('/tmp') / scenario for scenario in SCENARIOS}
    monkeypatch.setattr(axis_b_runner, 'validate_input', lambda root: {
        'scenario': root.name})
    monkeypatch.setattr(axis_b_runner, 'axis_a_ratios', lambda root: {})
    monkeypatch.setattr(preflight, '_identity', lambda path: {
        'path': str(path), 'size_bytes': 1, 'sha256': '0' * 64})
    state = readiness('full', roots, Path('/tmp/axis-a'))
    plan, selected = _execution_contract(state)
    assert plan == list(evaluator.FULL_PLAN)
    assert selected == roots
    assert list(selected) == list(SCENARIOS)


def test_execution_contract_rejects_readiness_input_drift():
    state = {
        'plan': [{'scenario': 'correct_init', 'profile': 'P0', 'seed': 11}],
        'canonical_inputs': [{
            'scenario': 'initial_offset', 'root': '/tmp/initial_offset',
            'manifest': {}}],
    }
    with pytest.raises(ValueError, match='execution contract drift'):
        _execution_contract(state)


def test_axis_b_bootstrap_uses_preflight_canonical_call_boundary(
        monkeypatch):
    root = Path('/tmp/canonical-correct-init')
    expected = {
        'source_start_storage_ns': 1000,
        'previous_scan_storage_ns': 1100,
        'first_main_scan_storage_ns': 1200,
        'first_main_scan_header_ns': 100,
        'lower_tf_storage_ns': 1070,
        'lower_tf_header_ns': 70,
        'upper_tf_storage_ns': 1175,
        'upper_tf_header_ns': 130,
        'start_offset_ns': 176,
        'start_offset_s': 1.76e-7,
    }
    calls = []

    def canonical_bootstrap(value):
        calls.append(value)
        return expected

    monkeypatch.setattr(
        preflight, '_tf_bootstrap_plan', canonical_bootstrap)
    assert axis_b_runner._axis_b_bootstrap(root) is expected
    assert calls == [root]
    assert expected['start_offset_ns'] != 187


def test_input_binding_parser_rejects_duplicate_relative_and_unknown():
    assert parse_input_roots(['correct_init=/tmp/correct']) == {
        'correct_init': Path('/tmp/correct')}
    for bindings in (
            ['correct_init=relative'],
            ['unknown=/tmp/value'],
            ['correct_init=/tmp/a', 'correct_init=/tmp/b'],
            ['missing-equals']):
        with pytest.raises(ValueError):
            parse_input_roots(bindings)


def test_generation_request_is_three_scenario_no_execution_contract():
    roots = {scenario: Path('/tmp') / f'axis-b-{scenario}' / 'bag'
             for scenario in SCENARIOS}
    value = input_generator.generation_request(roots)
    assert input_generator.validate_generation_request(value) == value
    assert value['executes_ros_or_gazebo'] is False
    assert value['synthetic_bag_canonical_eligible'] is False
    assert [row['scenario'] for row in value['requests']] == list(SCENARIOS)
    for row in value['requests']:
        assert '--use-sim-time' in row['expected_commands']['recorder']
        bridge_arg = f'bridge_config:={row["bridge"]["path"]}'
        assert bridge_arg in row['expected_commands']['gazebo']
        assert 'enable_image_bridges:=false' in \
            row['expected_commands']['gazebo']
        assert 'robot_state_publisher_respawn:=false' in \
            row['expected_commands']['gazebo']
        assert any(argument.startswith(input_generator.CONTACT_TOPIC + '@')
                   for argument in
                   row['expected_commands']['contact_bridge'])
        assert row['expected_commands']['contact_probe'][-1] == \
            input_generator.CONTACT_TOPIC
        assert input_generator.CONTACT_TOPIC in \
            row['expected_commands']['recorder']
    kidnapped_row = next(
        row for row in value['requests'] if row['scenario'] == 'kidnapped')
    command = kidnapped_row['expected_commands']['scenario_driver']
    evidence_index = command.index('--evidence')
    assert Path(command[evidence_index + 1]) == \
        roots['kidnapped'] / 'kidnapped_trace.json'
    assert '--contact-topic' not in command
    missing_bridge = copy.deepcopy(value)
    del missing_bridge['requests'][0]['bridge']
    with pytest.raises(ValueError, match='schema drift'):
        input_generator.validate_generation_request(missing_bridge)
    wrong_bridge = copy.deepcopy(value)
    wrong_bridge['requests'][0]['bridge']['sha256'] = '0' * 64
    with pytest.raises(ValueError, match='row drift'):
        input_generator.validate_generation_request(wrong_bridge)
    command_drift = copy.deepcopy(value)
    command_drift['requests'][0]['expected_commands']['gazebo'].remove(
        'enable_image_bridges:=false')
    with pytest.raises(ValueError, match='row drift'):
        input_generator.validate_generation_request(command_drift)
    changed = dict(value, synthetic_bag_canonical_eligible=True)
    with pytest.raises(ValueError):
        input_generator.validate_generation_request(changed)


def test_required_axis_b_source_topic_set_is_exact_and_fail_closed():
    exact = {name: object() for name in SOURCE_TOPICS}
    assert input_generator._require_source_topic_set(exact) is None
    for missing in SOURCE_TOPICS:
        changed = dict(exact)
        del changed[missing]
        with pytest.raises(ValueError, match='required Axis B source'):
            input_generator._require_source_topic_set(changed)
    with pytest.raises(ValueError, match='required Axis B source'):
        input_generator._require_source_topic_set(
            dict(exact, **{'/unexpected': object()}))


def test_axis_b_contact_claim_is_bound_to_observed_source():
    generator_source = Path(input_generator.__file__).read_text(
        encoding='utf-8')
    runner_source = Path(axis_b_runner.__file__).read_text(encoding='utf-8')
    evaluator_source = Path(evaluator.__file__).read_text(encoding='utf-8')
    assert input_generator.SOURCE_TOPICS[
        input_generator.CONTACT_TOPIC] == '/contact'
    assert 'contact_endpoint_probe' in generator_source
    assert "'contact_zero'" in runner_source
    assert "'contact_source_connected'" in runner_source
    assert "'contact_zero'" in evaluator_source
    assert "'contact_source_connected'" in evaluator_source


def test_capture_attestation_rejects_synthetic_summary(tmp_path):
    roots = {scenario: tmp_path / scenario / 'bag' for scenario in SCENARIOS}
    for root in roots.values():
        root.mkdir(parents=True)
    request_path = tmp_path / 'request.json'
    request = input_generator.generation_request(roots)
    request_path.write_bytes(canonical_json_bytes(request))
    source = roots['correct_init']
    (source / 'bag_0.mcap').write_bytes(b'mcap')
    (source / 'metadata.yaml').write_text('metadata', encoding='utf-8')
    row = request['requests'][0]
    processes = []
    for index, name in enumerate(('gazebo', 'recorder', 'scenario_driver')):
        log = tmp_path / f'{name}.log'
        log.write_text('completed', encoding='utf-8')
        processes.append({
            'name': name, 'command': row['expected_commands'][name],
            'pid': index + 1,
            'returncode': 0, 'survivors': [],
            'log': input_generator._identity(log),
            'started_steady_ns': index * 10 + 1,
            'finished_steady_ns': index * 10 + 2,
            'environment': {
                'ROS_DOMAIN_ID': str(row['ros_domain_id']),
                'ROS_LOCALHOST_ONLY': '1',
                'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}})
    value = {
        'schema_version': 1, 'status': 'PASS',
        'execution_mode': 'GAZEBO_RUNTIME_CAPTURE',
        'scenario': 'correct_init',
        'source_mcap': input_generator._identity(source / 'bag_0.mcap'),
        'source_metadata': input_generator._identity(
            source / 'metadata.yaml'),
        'generation_request': input_generator._identity(request_path),
        'synthetic_input': False, 'rosbag_record_exit_code': 0,
        'gazebo_exit_code': 0, 'survivor_count': 0,
        'runner_source': row['runner_source'],
        'gazebo_seed': row['gazebo_seed'],
        'ros_domain_id': row['ros_domain_id'],
        'g004_contract': row['g004_contract'], 'world': row['world'],
        'urdf': row['urdf'], 'bridge': row['bridge'],
        'lidar_profile': row['lidar_profile'],
        'processes': processes, 'atomic_finalized': True,
    }
    evidence = tmp_path / 'capture.json'
    evidence.write_bytes(canonical_json_bytes(value))
    with pytest.raises(ValueError, match='schema drift'):
        input_generator.validate_capture_attestation(
            evidence, source, 'correct_init')
    value['synthetic_input'] = True
    evidence.write_bytes(canonical_json_bytes(value))
    with pytest.raises(ValueError):
        input_generator.validate_capture_attestation(
            evidence, source, 'correct_init')


@pytest.mark.parametrize('recorder_returncode', [0, -signal.SIGINT])
def test_capture_executor_creates_only_valid_attestation(
        monkeypatch, tmp_path, recorder_returncode):
    roots = {scenario: tmp_path / scenario / 'bag' for scenario in SCENARIOS}
    request_path = tmp_path / 'request.json'
    request_path.write_bytes(canonical_json_bytes(
        input_generator.generation_request(roots)))
    started = []
    stopped = []

    class Process:
        def __init__(self, returncode):
            self.returncode = returncode

        def wait(self, timeout):
            assert timeout == 360.0
            return self.returncode

    def fake_start(name, command, log_path, env):
        log_path.write_text(f'{name} completed', encoding='utf-8')
        record = {
            'name': name, 'command': command, 'pid': len(started) + 100,
            'started': {'name': 'started',
                        'steady_ns': len(started) * 10 + 1,
                        'wall_ns': len(started) * 10 + 1},
            'process': Process(0)}
        started.append((record, dict(env)))
        return record

    def fake_stop(record, log_root):
        assert log_root == tmp_path / 'correct_init_capture_logs'
        if record['name'] == 'gazebo':
            (log_root / 'gazebo.log').write_text(
                "[ERROR] [gazebo-1]: process[gazebo-1] failed to terminate '5' "
                "seconds after receiving 'SIGINT', escalating to 'SIGTERM'\n"
                "[INFO] [gazebo-1]: sending signal 'SIGTERM' to "
                'process[gazebo-1]\n'
                '[ERROR] [gazebo-1]: process has died '
                '[pid 10, exit code -15, cmd gazebo].\n',
                encoding='utf-8')
        if record['name'] == 'recorder':
            source = roots['correct_init']
            source.mkdir(parents=True)
            (source / 'bag_0.mcap').write_bytes(b'mcap')
            (source / 'metadata.yaml').write_text(
                'metadata', encoding='utf-8')
        stopped.append(record['name'])
        return {
            'name': record['name'], 'command': record['command'],
            'pid': record['pid'],
            'returncode': (
                recorder_returncode if record['name'] == 'recorder' else 0),
            'survivors': [], 'started': record['started'],
            'stop_stages': (
                [] if record['name'] == 'scenario_driver' else
                [{'signal': 'SIGINT', 'steady_ns': len(stopped) * 100}]
                if record['name'] in ('recorder', 'contact_bridge') else [
                    {'signal': 'SIGINT', 'steady_ns': len(stopped) * 100},
                    {'signal': 'SIGTERM',
                     'steady_ns': len(stopped) * 100 + 1}]),
        }

    evidence = tmp_path / 'capture.json'
    monkeypatch.setattr(preflight, '_start', fake_start)
    monkeypatch.setattr(preflight, '_stop', fake_stop)
    monkeypatch.setattr(input_generator.subprocess, 'run', lambda *args, **kwargs:
                        SimpleNamespace(
                            returncode=0,
                            stdout=('Publisher count: 1\n'
                                    'Node name: ros_gz_bridge\n'),
                            stderr=''))
    monkeypatch.setattr(input_generator.time, 'sleep', lambda _: None)
    monkeypatch.setattr(
        input_generator, '_proc_starttime_ticks', lambda pid: pid * 10)
    result = input_generator.execute_capture(
        request_path, 'correct_init', evidence)
    assert result['status'] == 'PASS'
    assert [record[0]['name'] for record in started] == [
        'gazebo', 'contact_bridge', 'recorder', 'scenario_driver']
    assert [item['name'] for item in result['processes']] == [
        'gazebo', 'contact_bridge', 'recorder', 'scenario_driver']
    assert input_generator.validate_capture_attestation(
        evidence, roots['correct_init'], 'correct_init') == result
    hostile_mutations = [
        lambda value: value['processes'][0].update({'stop_stages': []}),
        lambda value: value['processes'][2]['stop_stages'].append(
            {'signal': 'SIGTERM', 'steady_ns': 999}),
        lambda value: value['processes'][0].update({'survivors': [123]}),
    ]
    for mutation in hostile_mutations:
        changed = copy.deepcopy(result)
        mutation(changed)
        evidence.write_bytes(canonical_json_bytes(changed))
        with pytest.raises(ValueError, match='graceful stop'):
            input_generator.validate_capture_attestation(
                evidence, roots['correct_init'], 'correct_init')
    for mutation in (
            lambda value: value['contact_endpoint_probe'].update({
                'publisher_count': 0}),
            lambda value: value['contact_endpoint_probe'].update({
                'node_names': ['unexpected_bridge']})):
        changed = copy.deepcopy(result)
        mutation(changed)
        evidence.write_bytes(canonical_json_bytes(changed))
        with pytest.raises(ValueError, match='contact endpoint probe'):
            input_generator.validate_capture_attestation(
                evidence, roots['correct_init'], 'correct_init')
    contact_bridge = copy.deepcopy(result['processes'][1])
    contact_bridge['returncode'] = -signal.SIGTERM
    contact_bridge['wait_returncode'] = -signal.SIGTERM
    contact_bridge['stop_stages'].append({
        'signal': 'SIGTERM',
        'steady_ns': contact_bridge['stop_stages'][-1]['steady_ns'] + 1})
    assert input_generator._capture_stop_valid(contact_bridge) is True
    contact_bridge['stop_stages'] = contact_bridge['stop_stages'][1:]
    assert input_generator._capture_stop_valid(contact_bridge) is False
    gazebo = result['processes'][0]
    supervisor = input_generator.GAZEBO_LOG_SUPERVISOR_ORPHAN_CLEANUP
    graceful = input_generator.GAZEBO_LOG_GRACEFUL
    assert input_generator._capture_stop_valid(gazebo, supervisor) is True
    for mutation in (
            lambda value: value.update({'returncode': -signal.SIGTERM}),
            lambda value: value.update({'stop_stages': [
                value['stop_stages'][0]]}),
            lambda value: value.update({'stop_stages': list(reversed(
                value['stop_stages']))})):
        changed = copy.deepcopy(gazebo)
        mutation(changed)
        assert input_generator._capture_stop_valid(
            changed, supervisor) is False
    assert input_generator._capture_stop_valid(gazebo, None) is False
    for returncode in (0, -signal.SIGINT):
        graceful_process = copy.deepcopy(gazebo)
        graceful_process['returncode'] = returncode
        graceful_process['stop_stages'] = [gazebo['stop_stages'][0]]
        assert input_generator._capture_stop_valid(
            graceful_process, graceful) is True
        assert input_generator._capture_stop_valid(
            graceful_process, supervisor) is False


def test_capture_teardown_diagnostic_is_stable_and_complete(tmp_path):
    stopped = [
        {'name': 'scenario_driver', 'returncode': 0,
         'stop_stages': [], 'survivors': []},
        {'name': 'contact_bridge', 'returncode': 0,
         'stop_stages': [{'signal': 'SIGINT', 'steady_ns': 15}],
         'survivors': []},
        {'name': 'recorder', 'returncode': 0,
         'stop_stages': [{'signal': 'SIGINT', 'steady_ns': 20}],
         'survivors': []},
        {'name': 'gazebo', 'returncode': -15,
         'stop_stages': [
             {'signal': 'SIGINT', 'steady_ns': 10},
             {'signal': 'SIGTERM', 'steady_ns': 30}],
         'survivors': [123]},
    ]
    assert input_generator._capture_teardown_diagnostic(stopped) == [
        {'name': 'gazebo', 'returncode': -15,
         'stop_stages': [
             {'signal': 'SIGINT', 'steady_ns': 10},
             {'signal': 'SIGTERM', 'steady_ns': 30}],
         'survivors': [123]},
        {'name': 'contact_bridge', 'returncode': 0,
         'stop_stages': [{'signal': 'SIGINT', 'steady_ns': 15}],
         'survivors': []},
        {'name': 'recorder', 'returncode': 0,
         'stop_stages': [{'signal': 'SIGINT', 'steady_ns': 20}],
         'survivors': []},
        {'name': 'scenario_driver', 'returncode': 0,
         'stop_stages': [], 'survivors': []},
    ]
    log = tmp_path / 'gazebo.log'
    log.write_text(
        "[ERROR] [gazebo-1]: process[gazebo-1] failed to terminate '5' "
        "seconds after receiving 'SIGINT', escalating to 'SIGTERM'\n"
        "[INFO] [gazebo-1]: sending signal 'SIGTERM' to process[gazebo-1]\n"
        '[ERROR] [gazebo-1]: process has died '
        '[pid 10, exit code -15, cmd gazebo].\n',
        encoding='utf-8')
    with pytest.raises(RuntimeError) as caught:
        input_generator._require_capture_teardown(stopped, log)
    message = str(caught.value)
    assert 'Axis B capture teardown failed:' in message
    assert '"name":"gazebo"' in message
    assert '"returncode":-15' in message
    assert '"signal":"SIGTERM"' in message
    assert '"survivors":[123]' in message


def test_gazebo_launch_log_rejects_hidden_child_failure(tmp_path):
    log = tmp_path / 'gazebo.log'
    log.write_text(
        '[ERROR] [gazebo-1]: process has died '
        '[pid 10, exit code -2, cmd x].\n'
        '[INFO] [robot_state_publisher-2]: process has finished cleanly\n',
        encoding='utf-8')
    assert input_generator._gazebo_launch_log_valid(log) is True
    assert input_generator._gazebo_launch_log_classification(log) == \
        input_generator.GAZEBO_LOG_GRACEFUL

    log.write_text(
        '[parameter_bridge-4] corrupted double-linked list\n'
        '[ERROR] [parameter_bridge-4]: process has died '
        '[pid 11, exit code -6, cmd bridge].\n',
        encoding='utf-8')
    assert input_generator._gazebo_launch_log_valid(log) is False

    log.write_text(
        '[image_bridge-6] [FATAL] [clock] [ros_gz_image]: '
        'Call to publish() on an invalid Publisher\n',
        encoding='utf-8')
    assert input_generator._gazebo_launch_log_valid(log) is False

    log.write_text(
        '[parameter_bridge-4] malloc(): unaligned tcache chunk detected\n',
        encoding='utf-8')
    assert input_generator._gazebo_launch_log_valid(log) is False


def test_gazebo_launch_log_accepts_only_exact_supervisor_sigterm(tmp_path):
    log = tmp_path / 'gazebo.log'
    markers = [
        "[ERROR] [gazebo-1]: process[gazebo-1] failed to terminate '5' "
        "seconds after receiving 'SIGINT', escalating to 'SIGTERM'",
        "[INFO] [gazebo-1]: sending signal 'SIGTERM' to process[gazebo-1]",
        '[ERROR] [gazebo-1]: process has died '
        '[pid 10, exit code -15, cmd gazebo].',
    ]
    log.write_text('\n'.join(markers), encoding='utf-8')
    assert input_generator._gazebo_launch_log_valid(log) is True
    assert input_generator._gazebo_launch_log_classification(log) == \
        input_generator.GAZEBO_LOG_SUPERVISOR_ORPHAN_CLEANUP

    for missing_index in range(len(markers) - 1):
        log.write_text(
            '\n'.join(marker for index, marker in enumerate(markers)
                      if index != missing_index),
            encoding='utf-8')
        assert input_generator._gazebo_launch_log_valid(log) is False

    log.write_text(
        '[ERROR] [gazebo-1]: process has died '
        '[pid 10, exit code -15, cmd gazebo].',
        encoding='utf-8')
    assert input_generator._gazebo_launch_log_valid(log) is False

    parameter_bridge = [marker.replace('gazebo-1', 'parameter_bridge-4')
                        for marker in markers]
    log.write_text('\n'.join(parameter_bridge), encoding='utf-8')
    assert input_generator._gazebo_launch_log_valid(log) is False


def test_stale_triggering_scan_field_is_not_used():
    for module in (evaluator,):
        source = Path(module.__file__).read_text(encoding='utf-8')
        assert 'triggering_scan_header_stamp_ns' not in source


def test_gt_pair_rejects_review_reproduction_with_ancient_gt():
    cloud = _cloud(0.0)
    truth = {
        'scan_stamps_ns': [100],
        'gt_poses': [
            {'stamp_ns': 1_000_000_000_000_000, 'frame_id': 'map',
             'pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]},
            {'stamp_ns': 1_000_000_000_100_000, 'frame_id': 'map',
             'pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]},
        ],
    }
    with pytest.raises(ValueError, match='older than bracketed GT'):
        _pair_gt([cloud], truth)


def test_gt_pair_rejects_interpolation_across_teleport_discontinuity():
    cloud = _cloud(0.0)
    cloud['fifo_associated_pose_scan_header_stamp_ns'] = 150
    truth = {
        'scan_stamps_ns': [150],
        'teleport_discontinuity_header_stamp_ns': 150,
        't0_scan_stamp_ns': 175,
        'gt_poses': [
            {'stamp_ns': 100, 'frame_id': 'map',
             'pose': [-8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]},
            {'stamp_ns': 200, 'frame_id': 'map',
             'pose': [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]},
        ],
    }
    with pytest.raises(ValueError, match='teleport discontinuity'):
        _pair_gt([cloud], truth)


def test_gt_pair_rejects_exact_gt_inside_teleport_to_t0_gap():
    cloud = _cloud(0.0)
    cloud['fifo_associated_pose_scan_header_stamp_ns'] = 160
    truth = {
        'scan_stamps_ns': [160],
        'teleport_discontinuity_header_stamp_ns': 150,
        't0_scan_stamp_ns': 175,
        'gt_poses': [
            {'stamp_ns': 160, 'frame_id': 'map',
             'pose': [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]},
        ],
    }
    with pytest.raises(ValueError, match='teleport gap'):
        _pair_gt([cloud], truth)


def test_kidnapped_metrics_require_pre_t0_and_post_zero_coverage():
    clouds = [_cloud(0.0) for _ in range(6)]
    pairs = [_pair(index) for index in range(6)]
    for index, pair in enumerate(pairs):
        stamp = (index + 1) * 100
        pair['scan_header_stamp_ns'] = stamp
        pair['gt_before_stamp_ns'] = stamp
        pair['gt_after_stamp_ns'] = stamp
        clouds[index]['fifo_associated_pose_scan_header_stamp_ns'] = stamp
    trace = {'events': [
        {'name': 'rotation_complete', 'ros_ns': 500},
        {'name': 'zero_hold_complete', 'ros_ns': 600}]}
    result = evaluator.kidnapped_metrics(clouds, pairs, 3, trace)
    assert result['pre_t0_initial_converged'] is True
    assert result['post_zero_hold_observed'] is True
    pairs[-1]['scan_header_stamp_ns'] = 599
    pairs[-1]['gt_before_stamp_ns'] = 599
    pairs[-1]['gt_after_stamp_ns'] = 599
    clouds[-1]['fifo_associated_pose_scan_header_stamp_ns'] = 599
    result = evaluator.kidnapped_metrics(clouds, pairs, 3, trace)
    assert result['post_zero_hold_observed'] is False


def test_kidnapped_cloud_target_covers_complete_event_motion():
    scans = [index * 100 for index in range(8)]
    odom = [{'stamp_ns': stamp, 'pose': [0.0, 0.0, index * 0.201]}
            for index, stamp in enumerate(scans)]
    assert evaluator._accepted_scan_indices(scans, odom) == list(range(8))


def test_kidnapped_false_convergence_covers_pre_t0_window():
    variance = (0.15 / 3.0) ** 2
    clouds = [_cloud(1.0, variance=variance) for _ in range(3)] + [
        _cloud(0.0) for _ in range(3)]
    pairs = [_pair(index) for index in range(6)]
    for index, pair in enumerate(pairs):
        stamp = index + 1
        pair['scan_header_stamp_ns'] = stamp
        pair['gt_before_stamp_ns'] = stamp
        pair['gt_after_stamp_ns'] = stamp
        clouds[index]['fifo_associated_pose_scan_header_stamp_ns'] = stamp
    trace = {'events': [
        {'name': 'rotation_complete', 'ros_ns': 5},
        {'name': 'zero_hold_complete', 'ros_ns': 6}]}
    result = evaluator.kidnapped_metrics(clouds, pairs, 3, trace)
    assert result['false_convergence'] is True


def test_runtime_tree_bytes_tolerates_vanished_atomic_temp(monkeypatch):
    class Vanished:
        def is_file(self):
            return True

        def is_symlink(self):
            return False

        def stat(self):
            raise FileNotFoundError('atomic temp was renamed')

    monkeypatch.setattr(Path, 'rglob', lambda self, pattern: [Vanished()])
    assert preflight._runtime_tree_bytes(Path('/tmp/run')) == 0


def test_source_projection_detects_drop_alter_and_reorder(monkeypatch):
    messages = [('/odom', b'a', 1), ('/scan', b'b', 2),
                ('/cmd_vel', b'c', 3)]

    class Reader:
        def open(self, *args):  # noqa: A003
            self.rows = list(messages)

        def has_next(self):
            return bool(self.rows)

        def get_all_topics_and_types(self):
            return [SimpleNamespace(
                name=name,
                type=('ros_gz_interfaces/msg/Contacts'
                      if name == input_generator.CONTACT_TOPIC else
                      'test_msgs/msg/Value'))
                    for name in SOURCE_TOPICS]

        def read_next(self):
            return self.rows.pop(0)

        def close(self):
            pass

    monkeypatch.setattr(
        input_generator.rosbag2_py, 'SequentialReader', Reader)
    baseline = input_generator._source_projection(Path('/tmp/source'))
    messages.pop()
    dropped = input_generator._source_projection(Path('/tmp/source'))
    assert dropped != baseline
    messages[:] = [('/odom', b'x', 1), ('/scan', b'b', 2),
                   ('/cmd_vel', b'c', 3)]
    altered = input_generator._source_projection(Path('/tmp/source'))
    assert altered != baseline
    messages.reverse()
    reordered = input_generator._source_projection(Path('/tmp/source'))
    assert reordered != altered


def test_numeric_constants_have_unit_suffixes_in_contract_source():
    source = Path(evaluator.__file__).read_text(encoding='utf-8')
    assert 'TRANSLATION_THRESHOLD_M' in source
    assert 'YAW_THRESHOLD_RAD' in source
