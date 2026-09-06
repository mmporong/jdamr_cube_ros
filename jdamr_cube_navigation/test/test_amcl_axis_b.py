#!/usr/bin/env python3
"""Boundary and hostile tests for G002 Axis B contracts."""

from pathlib import Path

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
from run_amcl_axis_b import initial_estimate


def _cloud(x, yaw=0.0, variance=1.0):
    import math
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = covariance[35] = variance
    return {'pose': [x, 0.0, 0.0, 0.0, 0.0,
                     math.sin(yaw / 2.0), math.cos(yaw / 2.0)],
            'covariance': covariance,
            'triggering_scan_header_stamp_ns': 100}


def _pair(index, x=0.0):
    return {'scan_index': index, 'gt_pose': [x, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]}


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
        'status': 'NOT_EVALUATED', 'promote_p2': False}


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
        'cpu_median': 1.05, 'latency_p95_median': 1.05,
        'individual_max': 1.10})
    assert result['promote_p2'] is True
    rows[-1]['false_convergence'] = True
    assert evaluator.promotion_decision(rows, {
        'cpu_median': 1.0, 'latency_p95_median': 1.0,
        'individual_max': 1.0})['promote_p2'] is False


def test_scenarios_and_topic_mapping_are_preregistered():
    assert list(SCENARIOS) == ['correct_init', 'initial_offset', 'kidnapped']
    assert SCENARIOS['initial_offset']['initial_estimate_offset'] == [
        0.5, 0.0, 0.2617993877991494]
    assert SCENARIOS['kidnapped']['teleport_pose'] == [
        0.0, 0.0, 3.141592653589793]
    assert SOURCE_TOPICS['/sim_raw/scan'] == '/scan'
    assert initial_estimate('initial_offset') == [
        -7.5, 0.0, 0.2617993877991494]


def _kidnapped_trace():
    names = ['stationary_confirmed', 'teleport_requested', 'teleport_ack',
             'teleport_gt_verified', 't0_scan', 'rotation_complete',
             'zero_hold_complete']
    return {
        'schema_version': 1,
        'driver_source': {
            'path': str(Path(kidnapped_driver.__file__).resolve()),
            'size_bytes': Path(kidnapped_driver.__file__).resolve().stat().st_size,
            'sha256': sha256_file(Path(kidnapped_driver.__file__).resolve())},
        'events': [{'name': name, 'steady_ns': index + 1,
                    'ros_ns': index + 1} for index, name in enumerate(names)],
        'pre_teleport_gt': [-8.0, 0.0, 0.0],
        'post_teleport_gt': [0.0, 0.0, 3.141592653589793],
        'pre_teleport_odom': [1.0, 2.0, 0.0],
        'post_teleport_odom': [1.05, 2.0, 0.0],
        't0_scan_stamp_ns': 10,
        'unwrapped_observation_yaw_rad': 6.283185307179586,
        'contact_count': 0, 'final_zero': True, 'zero_hold_s': 1.0,
        'stationary_sample_count': 10, 'failure': None,
    }


def test_kidnapped_trace_accepts_exact_boundary():
    assert validate_kidnapped_trace(_kidnapped_trace())['final_zero'] is True


def test_generator_source_identity_rejects_path_and_hash_tampering():
    identity = input_generator._generator_source_identity()
    assert input_generator._generator_source_valid(identity) is True
    changed = dict(identity, sha256='0' * 64)
    assert input_generator._generator_source_valid(changed) is False
    changed = dict(identity, path='/tmp/fake-generator.py')
    assert input_generator._generator_source_valid(changed) is False


def test_kidnapped_rotation_command_slows_before_exact_target():
    assert rotation_command_radps(0.0) == 0.5
    assert rotation_command_radps(
        ROTATION_TARGET_RAD - 0.10) == pytest.approx(0.10)
    assert rotation_command_radps(ROTATION_TARGET_RAD - 0.01) == 0.05
    assert rotation_command_radps(ROTATION_TARGET_RAD) == 0.0


@pytest.mark.parametrize('mutation', [
    lambda value: value.update({'contact_count': 1}),
    lambda value: value.update({'final_zero': False}),
    lambda value: value.update({'unwrapped_observation_yaw_rad': 6.0}),
    lambda value: value.update({'post_teleport_gt': [0.2, 0.0, 3.14]}),
    lambda value: value.update({'post_teleport_odom': [1.2, 2.0, 0.0]}),
    lambda value: value.update({'contact_count': False}),
    lambda value: value.update({'t0_scan_stamp_ns': True}),
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


def test_smoke_manifest_rejects_unknown_and_full_cardinality(tmp_path):
    full_plan = [{'scenario': scenario, 'profile': profile, 'seed': seed}
                 for scenario, profile, seed in evaluator.FULL_PLAN]
    value = {'schema_version': 1, 'mode': 'smoke',
             'claim_scope': 'AXIS_B_RELOCALIZATION_SMOKE_SIMULATION_ONLY',
             'full_plan': full_plan,
             'executed_plan': [
                 {'scenario': 'correct_init', 'profile': 'P0', 'seed': 11}],
             'runs': [{'relative_path': 'run_1/axis_b_evidence.json'}],
             'promotion': {'status': 'NOT_EVALUATED', 'promote_p2': False}}
    path = tmp_path / 'manifest.json'
    path.write_bytes(canonical_json_bytes(value))
    assert evaluator.validate_manifest(path, 'smoke')['mode'] == 'smoke'
    value['unknown'] = True
    path.write_bytes(canonical_json_bytes(value))
    with pytest.raises(ValueError, match='schema'):
        evaluator.validate_manifest(path, 'smoke')


def test_numeric_constants_have_unit_suffixes_in_contract_source():
    source = Path(evaluator.__file__).read_text(encoding='utf-8')
    assert 'TRANSLATION_THRESHOLD_M' in source
    assert 'YAW_THRESHOLD_RAD' in source
