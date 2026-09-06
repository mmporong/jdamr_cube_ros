#!/usr/bin/env python3
"""Hostile regression tests for the G002 Axis A evaluator."""

import hashlib
from pathlib import Path

from amcl_fault_contract import canonical_json_bytes, strict_json_load
import evaluate_amcl_axis_a as evaluator
import pytest
import run_amcl_determinism_preflight as preflight


RECORDED = [{
    'header_stamp_ns': 11, 'storage_stamp_ns': 12, 'frame_id': 'map',
    'pose': [1.1, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    'covariance': [0.0] * 36,
}]
RECORDED_IDENTITY = {
    'message_count': 1, 'ordered_semantic_sha256': 'a' * 64,
    'first_header_stamp_ns': 11, 'last_header_stamp_ns': 11,
}


@pytest.fixture(autouse=True)
def _isolate_external_sources(monkeypatch):
    monkeypatch.setattr(evaluator, '_validate_source_records', lambda _: None)
    monkeypatch.setattr(evaluator, '_validate_sanitizer_snapshot', lambda _: None)
    monkeypatch.setattr(
        evaluator, 'recorded_reference',
        lambda _root: (RECORDED, RECORDED_IDENTITY))


def _base(profile='P0', seed=11, domain_id=180):
    pose = [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    covariance = [0.0] * 36
    covariance[0] = 0.2
    covariance[7] = 0.3
    covariance[35] = 0.1
    overrides = [value for item in evaluator.profile_overrides(profile)
                 for value in ('-p', f'{item[0]}:={item[1]}')]
    return {
        'profile': profile, 'seed': seed, 'domain_id': domain_id,
        'status': 'PASS', 'survivor_count': 0, 'output_mcap_count': 0,
        'transform_lookup_drop_count': 0,
        'prefix_s': 30.0, 'playback_rate': 2.0,
        'resource': {'cpu_seconds': 0.2},
        'observer': {
            'schema_version': 1, 'run_id': f'axis_a__{profile}__seed_{seed}',
            'seed': seed, 'max_clouds': 1,
            'publisher_matched_count': 1,
            'pose_publisher_matched_count': 1,
            'events': [],
            'readiness': {
                'map_count': 1, 'clock_count': 1, 'odom_count': 1,
                'tf_count': 1, 'map_odom_tf_count': 1,
                'tf_static_count': 1},
            'initialpose_count': 1, 'pre_initial_scan_count': 0,
            'pre_initial_cloud_count': 0, 'raw_cloud_received_count': 1,
            'amcl_pose_received_count': 1, 'pending_cloud_count': 0,
            'pending_pose_count': 0, 'scan_count': 2,
            'scan_header_stamps_ns': [10, 11],
            'scan_header_stamp_sha256': hashlib.sha256(
                b'10\n11\n').hexdigest(),
            'scan_payload_sha256': '8' * 64,
            'clock_samples': [{'ros_ns': 1, 'arrival_steady_ns': 1}],
            'callback_trace': [
                {'kind': 'particle_cloud', 'stream_index': 0,
                 'header_stamp_ns': 12, 'arrival_steady_ns': 5_000_014},
                {'kind': 'amcl_pose', 'stream_index': 0,
                 'header_stamp_ns': 10, 'arrival_steady_ns': 5_000_015}],
            'pairing_contract': preflight.PAIRING_CONTRACT,
            'observation_qos_contract': preflight.OBSERVATION_QOS_CONTRACT,
            'pose_causality': 'NOT_PROVEN', 'failure': None, 'done': True,
            'motion_command_applicability': 'NOT_APPLICABLE',
            'clouds': [{
                'header_stamp_ns': 12, 'arrival_steady_ns': 5_000_014,
                'callback_ros_ns': 12, 'frame_id': 'map',
                'particle_count': 500, 'payload_sha256': 'a' * 64,
                'index': 0,
                'fifo_associated_pose_scan_header_stamp_ns': 10,
                'pose_header_stamp_ns': 10,
                'pose_arrival_steady_ns': 5_000_015,
                'pose_callback_ros_ns': 10, 'pose_frame_id': 'map',
                'pose': pose, 'covariance': covariance,
                'scan_arrival_steady_ns': 15,
                'scan_to_pose_steady_ns': 5_000_000,
                'cloud_stream_index': 0, 'pose_stream_index': 0,
                'pair_arrival_delta_ns': 1,
            }],
        },
        'teardown': [{
            'name': 'amcl',
            'command': ['amcl', '--ros-args', *overrides],
        }],
        'operations': [
            {'command': ['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
             'returncode': 0},
            {'command': ['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
             'returncode': 0},
            {'command': ['ros2', 'lifecycle', 'set', '/amcl', 'configure'],
             'returncode': 0},
            {'command': ['ros2', 'lifecycle', 'set', '/amcl', 'activate'],
             'returncode': 0},
            {'command': ['ros2', 'param', 'get'], 'returncode': 0},
            {'command': ['ros2', 'service', 'call'], 'returncode': 0},
        ],
        'map_yaml': {'sha256': 'a'}, 'params_file': {'sha256': 'b'},
        'sanitized_manifest': None,
        'loaded_runtime': {'sha256': 'd'}, 'tf_bootstrap': {'offset': 1},
    }


def _pair():
    pose = [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    recorded = [1.1, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    return {
        'generated_scan_header_stamp_ns': 10,
        'recorded_header_stamp_ns': 11,
        'recorded_storage_stamp_ns': 12,
        'pair_delta_ns': 1, 'recorded_frame_id': 'map',
        'generated_pose': pose, 'recorded_pose': recorded,
        'translation_disagreement_m': 0.10000000000000009,
        'yaw_disagreement_rad': 0.0,
    }


def _artifact(root: Path):
    root.mkdir()
    base = _base()
    contract = {
        'production_inputs': {
            'production_params': base['params_file']},
        'axis_a': {'map': {'yaml': base['map_yaml']}},
    }
    (root / 'contract_snapshot.json').write_bytes(canonical_json_bytes(contract))
    (root / 'sanitizer_manifest_snapshot.json').write_bytes(
        canonical_json_bytes({'source': {'path': '/test'}}))
    (root / 'runtime_attestation_snapshot.json').write_bytes(
        canonical_json_bytes({'loaded_runtime': base['loaded_runtime']}))
    run_dir = root / 'run_1'
    run_dir.mkdir()
    base['sanitized_manifest'] = preflight._identity(
        root / 'sanitizer_manifest_snapshot.json')
    resource_path = run_dir / 'amcl_resource.jsonl'
    resource_path.write_text(
        '{"monotonic_s":1.0,"cpu_total_s":0.0,'
        '"cpu_pct_one_core":0.0,"rss_mb":1.0,"process_count":1}\n'
        '{"monotonic_s":2.0,"cpu_total_s":0.2,'
        '"cpu_pct_one_core":20.0,"rss_mb":2.0,"process_count":1}\n',
        encoding='utf-8')
    base['resource'] = preflight._resource_summary(resource_path, run_dir)
    observer_state = run_dir / 'observer_state.json'
    observer_state.write_bytes(canonical_json_bytes(base.pop('observer')))
    base['observer_state'] = preflight._relative_identity(
        observer_state, run_dir)
    base_path = run_dir / 'evidence.json'
    base_path.write_bytes(canonical_json_bytes(base))
    base_view = evaluator._load_base_view(base_path)
    pairs = [_pair()]
    axis = {
        'schema_version': 1, 'run_id': 'axis_a__P0__seed_11',
        'profile': 'P0', 'seed': 11, 'domain_id': 180,
        'status': 'PASS', 'failure': None,
        'base_evidence': preflight._relative_identity(base_path, root),
        'profile_parameters': evaluator.PROFILES['P0'],
        'recorded_pose_pairs': pairs,
        'metrics': evaluator.derive_metrics(base_view, pairs),
        'map_odom_authority': {
            'sanitized_input_count': 0, 'generated_observed_count': 1,
            'sole_runtime_authority': True},
    }
    axis_path = run_dir / 'axis_a_evidence.json'
    axis_path.write_bytes(canonical_json_bytes(axis))
    records = evaluator._tree_records(root)
    manifest = {
        'schema_version': 1, 'mode': 'smoke',
        'claim_scope': evaluator.CLAIM_SMOKE,
        'plan': [{'run_id': 'axis_a__P0__seed_11', 'profile': 'P0',
                  'seed': 11, 'domain_id': 180}],
        'source_contract': preflight._relative_identity(
            root / 'contract_snapshot.json', root),
        'sanitizer_snapshot': preflight._relative_identity(
            root / 'sanitizer_manifest_snapshot.json', root),
        'runtime_attestation': preflight._relative_identity(
            root / 'runtime_attestation_snapshot.json', root),
        'harness_sources': {
            path.name: evaluator._source_identity(path)
            for path in (evaluator.EVALUATOR, evaluator.RUNNER,
                         evaluator.OBSERVER, evaluator.PREFLIGHT)},
        'runs': [preflight._relative_identity(axis_path, root)],
        'tree_records': records,
        'tree_sha256': evaluator._tree_digest(records),
        'tree_bytes': evaluator._tree_bytes(root),
        'production_unchanged': True,
        'run_contract': {
            'max_clouds': 1, 'prefix_s': 30.0, 'playback_rate': 2.0,
            'output_mcap_count': 0, 'per_run_limit_bytes': 2097152},
        'recorded_reference': RECORDED_IDENTITY,
    }
    (root / 'axis_a_manifest.json').write_bytes(canonical_json_bytes(manifest))
    return root


def _rewrite(root: Path, axis_mutator=None, manifest_mutator=None):
    axis_path = root / 'run_1/axis_a_evidence.json'
    axis = strict_json_load(axis_path)
    if axis_mutator:
        axis_mutator(axis)
    axis_path.write_bytes(canonical_json_bytes(axis))
    manifest_path = root / 'axis_a_manifest.json'
    manifest = strict_json_load(manifest_path)
    if manifest_mutator:
        manifest_mutator(manifest)
    manifest['runs'] = [preflight._relative_identity(axis_path, root)]
    records = evaluator._tree_records(root)
    manifest['tree_records'] = records
    manifest['tree_sha256'] = evaluator._tree_digest(records)
    manifest['tree_bytes'] = evaluator._tree_bytes(root)
    manifest_path.write_bytes(canonical_json_bytes(manifest))


def _refresh_base_and_manifest(root: Path) -> None:
    base_path = root / 'run_1/evidence.json'
    axis_path = root / 'run_1/axis_a_evidence.json'
    axis = strict_json_load(axis_path)
    axis['base_evidence'] = preflight._relative_identity(base_path, root)
    axis_path.write_bytes(canonical_json_bytes(axis))
    _rewrite(root)


def test_profile_overrides_are_exact_and_unknown_rejected():
    assert evaluator.profile_overrides('P1') == (
        ('min_particles', '2000'), ('max_particles', '2000'),
        ('recovery_alpha_fast', '0.0'), ('recovery_alpha_slow', '0.0'))
    with pytest.raises(ValueError, match='unknown'):
        evaluator.profile_overrides('P9')


def test_metrics_are_finite_and_recomputed():
    metrics = evaluator.derive_metrics(_base(), [_pair()])
    assert metrics['accepted_update_count'] == 1
    assert metrics['scan_to_pose_latency_ms']['max'] == 5.0
    assert metrics['amcl_cpu_seconds_per_1000_scans'] == 100.0


def test_observer_waits_for_map_odom_authority_after_cloud_limit():
    source = Path(evaluator.OBSERVER).read_text(encoding='utf-8')
    tick = source[source.index('    def _tick'):source.index(
        '    def _snapshot')]
    assert 'len(self.clouds) >= self.args.max_clouds' in tick
    assert 'self.map_odom_tf_count > 0' in tick


def test_smoke_validator_accepts_exact_fixture(tmp_path):
    root = _artifact(tmp_path / 'artifact')
    assert evaluator.validate_axis_a_smoke(root)['mode'] == 'smoke'


def test_runtime_shaped_base_view_is_memory_only(tmp_path):
    root = _artifact(tmp_path / 'artifact')
    base_path = root / 'run_1/evidence.json'
    persisted = strict_json_load(base_path)
    assert 'observer' not in persisted
    view = evaluator._load_base_view(base_path)
    assert view['observer']['clouds'][0][
        'fifo_associated_pose_scan_header_stamp_ns'] == 10
    assert 'observer' not in strict_json_load(base_path)


@pytest.mark.parametrize(
    'attack', ['escape', 'symlink', 'stale_hash', 'missing'])
def test_observer_state_reference_hostile_inputs_fail_closed(
        tmp_path, attack):
    root = _artifact(tmp_path / 'artifact')
    run_dir = root / 'run_1'
    state_path = run_dir / 'observer_state.json'
    base_path = run_dir / 'evidence.json'
    if attack == 'escape':
        outside = root / 'observer_state.json'
        outside.write_bytes(state_path.read_bytes())
        base = strict_json_load(base_path)
        base['observer_state'] = {
            'relative_path': '../observer_state.json',
            'size_bytes': outside.stat().st_size,
            'sha256': hashlib.sha256(outside.read_bytes()).hexdigest()}
        base_path.write_bytes(canonical_json_bytes(base))
        _refresh_base_and_manifest(root)
    elif attack == 'symlink':
        outside = root / 'outside_observer_state.json'
        state_path.replace(outside)
        state_path.symlink_to(outside)
        _refresh_base_and_manifest(root)
    elif attack == 'stale_hash':
        state = strict_json_load(state_path)
        state['run_id'] = 'mutated'
        state_path.write_bytes(canonical_json_bytes(state))
        _rewrite(root)
    else:
        state_path.unlink()
        _rewrite(root)
    with pytest.raises((ValueError, FileNotFoundError)):
        evaluator.validate_axis_a_smoke(root)


@pytest.mark.parametrize('mutation', [
    lambda value: value.update({'unexpected': 1}),
    lambda value: value['profile_parameters'].update({'min_particles': 499}),
    lambda value: value['metrics']['closure_m'].update({'generated': 1.0}),
    lambda value: value['recorded_pose_pairs'][0].update({'pair_delta_ns': 99}),
    lambda value: value['map_odom_authority'].update(
        {'sole_runtime_authority': False}),
])
def test_axis_evidence_hostile_mutations_fail_closed(tmp_path, mutation):
    root = _artifact(tmp_path / 'artifact')
    _rewrite(root, axis_mutator=mutation)
    with pytest.raises(ValueError):
        evaluator.validate_axis_a_smoke(root)


def test_full_validator_rejects_one_run_smoke(tmp_path):
    root = _artifact(tmp_path / 'artifact')
    with pytest.raises(ValueError):
        evaluator.validate_axis_a_full(root)


def test_strict_json_rejects_nonfinite_axis_metric(tmp_path):
    path = tmp_path / 'bad.json'
    path.write_text('{"value":NaN}', encoding='utf-8')
    with pytest.raises(ValueError, match='non-finite'):
        strict_json_load(path)
