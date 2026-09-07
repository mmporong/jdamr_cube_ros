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
from run_amcl_axis_b import _pair_gt, initial_estimate, parse_input_roots
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


def test_full_promotion_is_blocked_without_publisher_gid_proof():
    decision = {'status': 'EVALUATED', 'promote_p2': True,
                'selected_profile': 'P2'}
    result = evaluator.authority_limited_promotion(decision)
    assert result == {
        'status': 'NOT_EVALUATED_AUTHORITY_NOT_PROVEN',
        'promote_p2': False,
        'selected_profile': 'P0',
        'parameter_rule_passed_without_authority': True,
    }


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
    assert SOURCE_TOPICS['/sim_raw/scan'] == '/scan'
    assert initial_estimate('initial_offset') == [
        -7.5, 0.0, 0.2617993877991494]


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
            {'name': name, 'steady_ns': index + 1, 'ros_ns': index + 1},
            **({'scan_header_stamp_ns': 10} if name == 't0_scan' else {}))
            for index, name in enumerate(names)],
        'pre_teleport_gt': [-8.0, 0.0, 0.0],
        'post_teleport_gt': [0.0, 0.0, 3.141592653589793],
        'pre_teleport_odom': [1.0, 2.0, 0.0],
        'post_teleport_odom': [1.05, 2.0, 0.0],
        't0_scan_stamp_ns': 10,
        'unwrapped_observation_yaw_rad': 6.283185307179586,
        'initial_observation_yaw_rad': 0.8,
        'post_zero_observation_yaw_rad': 1.2,
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
    lambda value: next(event for event in value['events']
                       if event['name'] == 't0_scan').update(
                           {'scan_header_stamp_ns': 11}),
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
    changed = dict(value, synthetic_bag_canonical_eligible=True)
    with pytest.raises(ValueError):
        input_generator.validate_generation_request(changed)


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
        'urdf': row['urdf'], 'lidar_profile': row['lidar_profile'],
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


def test_capture_executor_creates_only_valid_attestation(monkeypatch, tmp_path):
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
            'returncode': (0 if record['name'] == 'scenario_driver'
                           else -2),
            'survivors': [], 'started': record['started'],
            'stop_stages': [{'steady_ns': len(stopped) * 100}],
        }

    evidence = tmp_path / 'capture.json'
    monkeypatch.setattr(preflight, '_start', fake_start)
    monkeypatch.setattr(preflight, '_stop', fake_stop)
    monkeypatch.setattr(input_generator.time, 'sleep', lambda _: None)
    monkeypatch.setattr(
        input_generator, '_proc_starttime_ticks', lambda pid: pid * 10)
    result = input_generator.execute_capture(
        request_path, 'correct_init', evidence)
    assert result['status'] == 'PASS'
    assert [record[0]['name'] for record in started] == [
        'gazebo', 'recorder', 'scenario_driver']
    assert [item['name'] for item in result['processes']] == [
        'gazebo', 'recorder', 'scenario_driver']
    assert input_generator.validate_capture_attestation(
        evidence, roots['correct_init'], 'correct_init') == result


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
    assert evaluator.KIDNAPPED_TARGET_ACCEPTED_CLOUDS == 41
    assert evaluator.KIDNAPPED_TARGET_ACCEPTED_CLOUDS > \
        evaluator.FULL_CLOUD_COUNT


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
