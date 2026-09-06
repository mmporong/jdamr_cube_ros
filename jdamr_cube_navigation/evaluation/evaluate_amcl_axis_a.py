#!/usr/bin/env python3
"""Validate G002 Axis A AMCL replay artifacts and derived metrics."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
import statistics
from typing import Any

from amcl_fault_contract import canonical_json_bytes, STORAGE_LIMITS
from amcl_fault_contract import strict_json_load
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from run_amcl_determinism_preflight import _relative_identity
from run_amcl_determinism_preflight import _resource_summary
from run_amcl_determinism_preflight import _validate_sanitizer_snapshot
from run_amcl_determinism_preflight import _validate_source_records


EVALUATOR = Path(__file__).resolve()
RUNNER = EVALUATOR.with_name('run_amcl_axis_a.py')
OBSERVER = EVALUATOR.with_name('amcl_particle_observer.py')
PREFLIGHT = EVALUATOR.with_name('run_amcl_determinism_preflight.py')


SCHEMA_VERSION = 1
CLAIM_SMOKE = 'AXIS_A_AMCL_METRICS_SMOKE_NO_GT'
CLAIM_FULL = 'AXIS_A_AMCL_PROFILE_MATRIX_NO_GT'
PROFILES = {
    'P0': {'min_particles': 500, 'max_particles': 2000,
           'recovery_alpha_fast': 0.0, 'recovery_alpha_slow': 0.0},
    'P1': {'min_particles': 2000, 'max_particles': 2000,
           'recovery_alpha_fast': 0.0, 'recovery_alpha_slow': 0.0},
    'P2': {'min_particles': 500, 'max_particles': 2000,
           'recovery_alpha_fast': 0.1, 'recovery_alpha_slow': 0.001},
}
SEEDS = (11, 23, 42, 67, 89)
FULL_PLAN = tuple((profile, seed) for profile in PROFILES for seed in SEEDS)
FULL_CLOUD_COUNT = 30
FULL_PREFIX_S = 220.0
SMOKE_CLOUD_COUNT = 1
SMOKE_PREFIX_S = 30.0
PLAYBACK_RATE = 2.0


def _exact_keys(value: Any, expected: set[str], label: str) -> None:
    if type(value) is not dict or set(value) != expected:
        raise ValueError(f'{label} schema drift')


def _finite(value: Any, label: str, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be numeric')
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        raise ValueError(f'{label} is outside its finite range')
    return result


def percentile(values: list[float], fraction: float) -> float:
    """
    Return a deterministic nearest-rank percentile.

    The rank is ``ceil(fraction * sample_count) - 1`` in zero-based indexing.
    This keeps the observed value rather than interpolating particle counts.
    """
    if not values or not 0.0 < fraction <= 1.0:
        raise ValueError('invalid percentile input')
    ordered = sorted(float(value) for value in values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def profile_overrides(profile: str) -> tuple[tuple[str, str], ...]:
    """Return the exact allowed AMCL profile parameter overrides."""
    if profile not in PROFILES:
        raise ValueError('unknown AMCL profile')
    return tuple((name, str(value).lower())
                 for name, value in PROFILES[profile].items())


def derive_metrics(evidence: dict, recorded_pairs: list[dict]) -> dict:
    """Derive finite Axis A metrics without making an accuracy claim."""
    clouds = evidence['observer']['clouds']
    if not clouds:
        raise ValueError('no causal AMCL updates')
    latencies_ms = [
        _finite(cloud['scan_to_pose_steady_ns'], 'scan latency', 0.0) / 1e6
        for cloud in clouds]
    particles = [
        _finite(cloud['particle_count'], 'particle count', 1.0)
        for cloud in clouds]
    scans = evidence['observer']['scan_count']
    if type(scans) is not int or scans <= 0:
        raise ValueError('scan count must be positive')
    cpu_seconds = _finite(evidence['resource']['cpu_seconds'], 'CPU seconds', 0.0)
    covariance = [cloud['covariance'] for cloud in clouds]
    if any(type(row) is not list or len(row) != 36 for row in covariance):
        raise ValueError('AMCL covariance shape drift')
    covariance_summary = {
        'x_mean_m2': statistics.fmean(
            _finite(row[0], 'covariance x', 0.0) for row in covariance),
        'y_mean_m2': statistics.fmean(
            _finite(row[7], 'covariance y', 0.0) for row in covariance),
        'yaw_mean_rad2': statistics.fmean(
            _finite(row[35], 'covariance yaw', 0.0) for row in covariance),
    }
    if len(recorded_pairs) != len(clouds):
        raise ValueError('recorded AMCL pairing cardinality drift')
    disagreements = [
        _finite(pair['translation_disagreement_m'], 'pose disagreement', 0.0)
        for pair in recorded_pairs]
    generated = [cloud['pose'] for cloud in clouds]
    recorded = [pair['recorded_pose'] for pair in recorded_pairs]
    return {
        'accepted_update_count': len(clouds),
        'scan_count': scans,
        'scan_to_pose_latency_ms': {
            'p50': percentile(latencies_ms, 0.50),
            'p95': percentile(latencies_ms, 0.95),
            'max': max(latencies_ms),
        },
        'particle_count': {
            'p50': percentile(particles, 0.50),
            'p95': percentile(particles, 0.95),
            'max': max(particles),
        },
        # CPU seconds are normalized by accepted input scan count, not wall time.
        'amcl_cpu_seconds_per_1000_scans': cpu_seconds * 1000.0 / scans,
        'covariance': covariance_summary,
        'recorded_pose_disagreement_m': {
            'p50': percentile(disagreements, 0.50),
            'p95': percentile(disagreements, 0.95),
            'max': max(disagreements),
        },
        'closure_m': {
            'generated': math.dist(generated[0][:2], generated[-1][:2]),
            'recorded_reference': math.dist(recorded[0][:2], recorded[-1][:2]),
        },
    }


def _validate_metric_group(metrics: dict) -> None:
    _exact_keys(metrics, {
        'accepted_update_count', 'scan_count', 'scan_to_pose_latency_ms',
        'particle_count', 'amcl_cpu_seconds_per_1000_scans', 'covariance',
        'recorded_pose_disagreement_m', 'closure_m'}, 'metrics')
    for name in ('accepted_update_count', 'scan_count'):
        if type(metrics[name]) is not int or metrics[name] <= 0:
            raise ValueError(f'invalid metric count: {name}')
    for name in ('scan_to_pose_latency_ms', 'particle_count',
                 'recorded_pose_disagreement_m'):
        _exact_keys(metrics[name], {'p50', 'p95', 'max'}, name)
        values = [_finite(metrics[name][key], f'{name}.{key}', 0.0)
                  for key in ('p50', 'p95', 'max')]
        if values != sorted(values):
            raise ValueError(f'non-monotonic percentile: {name}')
    _exact_keys(metrics['covariance'], {
        'x_mean_m2', 'y_mean_m2', 'yaw_mean_rad2'}, 'covariance')
    _exact_keys(metrics['closure_m'], {
        'generated', 'recorded_reference'}, 'closure')
    for value in metrics['covariance'].values():
        _finite(value, 'covariance', 0.0)
    for value in metrics['closure_m'].values():
        _finite(value, 'closure', 0.0)
    _finite(metrics['amcl_cpu_seconds_per_1000_scans'], 'normalized CPU', 0.0)


def _tree_records(root: Path) -> list[dict]:
    return sorted((
        _relative_identity(path, root) for path in root.rglob('*')
        if path.is_file() and not path.is_symlink() and
        path.name != 'axis_a_manifest.json'),
        key=lambda record: record['relative_path'])


def _tree_digest(records: list[dict]) -> str:
    return hashlib.sha256(canonical_json_bytes(records)).hexdigest()


def _tree_bytes(root: Path) -> int:
    return sum(path.stat().st_size for path in root.rglob('*')
               if path.is_file() and not path.is_symlink() and
               path.name != 'axis_a_manifest.json')


def _validate_recorded_pairs(base: dict, pairs: list[dict]) -> None:
    if type(pairs) is not list or len(pairs) != len(base['observer']['clouds']):
        raise ValueError('recorded pair cardinality drift')
    for cloud, pair in zip(base['observer']['clouds'], pairs):
        _exact_keys(pair, {
            'generated_scan_header_stamp_ns', 'recorded_header_stamp_ns',
            'recorded_storage_stamp_ns', 'pair_delta_ns', 'recorded_frame_id',
            'generated_pose', 'recorded_pose',
            'translation_disagreement_m', 'yaw_disagreement_rad'},
            'recorded pair')
        for key in ('generated_scan_header_stamp_ns',
                    'recorded_header_stamp_ns', 'recorded_storage_stamp_ns',
                    'pair_delta_ns'):
            if type(pair[key]) is not int or pair[key] < 0:
                raise ValueError(f'invalid recorded pair integer: {key}')
        if (pair['generated_scan_header_stamp_ns'] !=
                cloud['triggering_scan_header_stamp_ns'] or
                pair['generated_pose'] != cloud['pose'] or
                pair['recorded_frame_id'] != 'map'):
            raise ValueError('recorded pair source binding drift')
        for key in ('generated_pose', 'recorded_pose'):
            if (type(pair[key]) is not list or len(pair[key]) != 7 or
                    any(not math.isfinite(float(value)) or isinstance(value, bool)
                        for value in pair[key])):
                raise ValueError(f'invalid recorded pair pose: {key}')
        if pair['pair_delta_ns'] != abs(
                pair['recorded_header_stamp_ns'] -
                pair['generated_scan_header_stamp_ns']):
            raise ValueError('recorded pair time delta drift')
        translation_m = math.dist(
            pair['generated_pose'][:2], pair['recorded_pose'][:2])
        yaw_rad = abs(math.remainder(
            _pose_yaw(pair['generated_pose']) -
            _pose_yaw(pair['recorded_pose']), 2.0 * math.pi))
        if (pair['translation_disagreement_m'] != translation_m or
                pair['yaw_disagreement_rad'] != yaw_rad):
            raise ValueError('recorded pair metric drift')


def _pose_yaw(pose: list[float]) -> float:
    x, y, z, w = pose[3:7]
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def recorded_reference(source_root: Path) -> tuple[list[dict], dict]:
    """Read the canonical recorded AMCL reference stream once."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(source_root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''))
    topic_types = {item.name: get_message(item.type)
                   for item in reader.get_all_topics_and_types()}
    poses = []
    while reader.has_next():
        topic, serialized, storage_ns = reader.read_next()
        if topic != '/amcl_pose':
            continue
        message = deserialize_message(serialized, topic_types[topic])
        pose = message.pose.pose
        values = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w]
        covariance = [float(value) for value in message.pose.covariance]
        if any(isinstance(value, bool) or not math.isfinite(float(value))
               for value in values + covariance):
            raise ValueError('recorded AMCL pose is non-finite')
        poses.append({
            'header_stamp_ns': int(message.header.stamp.sec) * 1_000_000_000 +
            int(message.header.stamp.nanosec),
            'storage_stamp_ns': int(storage_ns),
            'frame_id': message.header.frame_id,
            'pose': [float(value) for value in values],
            'covariance': covariance,
        })
    if (not poses or any(left['header_stamp_ns'] > right['header_stamp_ns']
                         for left, right in zip(poses, poses[1:]))):
        raise ValueError('recorded AMCL reference is absent or unordered')
    identity = {
        'message_count': len(poses),
        'ordered_semantic_sha256': hashlib.sha256(
            canonical_json_bytes(poses)).hexdigest(),
        'first_header_stamp_ns': poses[0]['header_stamp_ns'],
        'last_header_stamp_ns': poses[-1]['header_stamp_ns'],
    }
    return poses, identity


def pair_recorded(clouds: list[dict], recorded: list[dict]) -> list[dict]:
    """Pair each generated update to the nearest recorded pose timestamp."""
    pairs = []
    for cloud in clouds:
        stamp_ns = cloud['triggering_scan_header_stamp_ns']
        reference = min(
            recorded, key=lambda item: abs(item['header_stamp_ns'] - stamp_ns))
        generated = cloud['pose']
        recorded_pose = reference['pose']
        yaw_delta = math.remainder(
            _pose_yaw(generated) - _pose_yaw(recorded_pose), 2.0 * math.pi)
        pairs.append({
            'generated_scan_header_stamp_ns': stamp_ns,
            'recorded_header_stamp_ns': reference['header_stamp_ns'],
            'recorded_storage_stamp_ns': reference['storage_stamp_ns'],
            'pair_delta_ns': abs(reference['header_stamp_ns'] - stamp_ns),
            'recorded_frame_id': reference['frame_id'],
            'generated_pose': generated, 'recorded_pose': recorded_pose,
            'translation_disagreement_m': math.dist(
                generated[:2], recorded_pose[:2]),
            'yaw_disagreement_rad': abs(yaw_delta),
        })
    return pairs


def validate_axis_a_artifact(root: Path, expected_mode: str) -> dict:
    """Validate one smoke or the exact fifteen-run Axis A matrix."""
    if expected_mode not in ('smoke', 'full'):
        raise ValueError('unknown validation mode')
    if (not root.is_absolute() or root != root.resolve() or root.is_symlink() or
            not root.is_dir() or any(path.is_symlink() for path in root.rglob('*'))):
        raise ValueError('artifact root must be a canonical non-symlink tree')
    manifest = strict_json_load(root / 'axis_a_manifest.json')
    _exact_keys(manifest, {
        'schema_version', 'mode', 'claim_scope', 'plan', 'source_contract',
        'sanitizer_snapshot', 'runtime_attestation', 'runs', 'tree_records',
        'tree_sha256', 'tree_bytes', 'production_unchanged',
        'harness_sources', 'run_contract', 'recorded_reference'}, 'manifest')
    claim = CLAIM_FULL if expected_mode == 'full' else CLAIM_SMOKE
    if (manifest['schema_version'] != SCHEMA_VERSION or
            manifest['mode'] != expected_mode or
            manifest['claim_scope'] != claim or
            manifest['production_unchanged'] is not True):
        raise ValueError('manifest scalar contract drift')
    expected_plan = list(FULL_PLAN) if expected_mode == 'full' else [('P0', 11)]
    expected_run_contract = {
        'max_clouds': FULL_CLOUD_COUNT if expected_mode == 'full'
        else SMOKE_CLOUD_COUNT,
        'prefix_s': FULL_PREFIX_S if expected_mode == 'full'
        else SMOKE_PREFIX_S,
        'playback_rate': PLAYBACK_RATE,
        'output_mcap_count': 0,
        'per_run_limit_bytes': STORAGE_LIMITS['run_output_limit_bytes'],
    }
    if manifest['run_contract'] != expected_run_contract:
        raise ValueError('Axis A run contract drift')
    normalized_plan = [(row['profile'], row['seed']) for row in manifest['plan']]
    if normalized_plan != expected_plan:
        raise ValueError('Axis A execution plan drift')
    for index, row in enumerate(manifest['plan']):
        _exact_keys(row, {'run_id', 'profile', 'seed', 'domain_id'}, 'plan row')
        profile, seed = expected_plan[index]
        if (row['run_id'] != f'axis_a__{profile}__seed_{seed}' or
                type(row['domain_id']) is not int or
                not 0 <= row['domain_id'] <= 232):
            raise ValueError('Axis A plan identity drift')
    if len({row['domain_id'] for row in manifest['plan']}) != len(expected_plan):
        raise ValueError('Axis A domain collision')
    for key, filename in (
            ('source_contract', 'contract_snapshot.json'),
            ('sanitizer_snapshot', 'sanitizer_manifest_snapshot.json'),
            ('runtime_attestation', 'runtime_attestation_snapshot.json')):
        if manifest[key] != _relative_identity(root / filename, root):
            raise ValueError(f'{key} identity drift')
    expected_sources = {
        path.name: _source_identity(path)
        for path in (EVALUATOR, RUNNER, OBSERVER, PREFLIGHT)}
    if manifest['harness_sources'] != expected_sources:
        raise ValueError('Axis A harness source identity drift')
    contract = strict_json_load(root / 'contract_snapshot.json')
    _validate_source_records(contract['production_inputs'])
    sanitizer = strict_json_load(root / 'sanitizer_manifest_snapshot.json')
    _validate_sanitizer_snapshot(sanitizer)
    source_root = Path(sanitizer['source']['path'])
    recorded, recorded_identity = recorded_reference(source_root)
    if manifest['recorded_reference'] != recorded_identity:
        raise ValueError('recorded AMCL reference identity drift')
    attestation = strict_json_load(root / 'runtime_attestation_snapshot.json')
    records = _tree_records(root)
    if (manifest['tree_records'] != records or
            manifest['tree_sha256'] != _tree_digest(records) or
            manifest['tree_bytes'] != _tree_bytes(root) or
            manifest['tree_bytes'] > 32 * 1024 * 1024):
        raise ValueError('artifact tree identity or cap drift')
    if len(manifest['runs']) != len(expected_plan):
        raise ValueError('run count drift')
    for index, record in enumerate(manifest['runs']):
        path = root / record['relative_path']
        if record != _relative_identity(path, root):
            raise ValueError('run evidence identity drift')
        evidence = strict_json_load(path)
        _exact_keys(evidence, {
            'schema_version', 'run_id', 'profile', 'seed', 'domain_id',
            'status', 'failure', 'base_evidence', 'profile_parameters',
            'recorded_pose_pairs', 'metrics', 'map_odom_authority'},
            'Axis A evidence')
        plan = manifest['plan'][index]
        if (evidence['schema_version'] != SCHEMA_VERSION or
                evidence['run_id'] != plan['run_id'] or
                evidence['profile'] != plan['profile'] or
                evidence['seed'] != plan['seed'] or
                evidence['domain_id'] != plan['domain_id'] or
                evidence['status'] != 'PASS' or evidence['failure'] is not None):
            raise ValueError('Axis A evidence scalar drift')
        if evidence['profile_parameters'] != PROFILES[evidence['profile']]:
            raise ValueError('profile parameter diff drift')
        base_path = root / evidence['base_evidence']['relative_path']
        if evidence['base_evidence'] != _relative_identity(base_path, root):
            raise ValueError('base evidence identity drift')
        base = strict_json_load(base_path)
        if (base['profile'] != evidence['profile'] or
                base['seed'] != evidence['seed'] or
                base['domain_id'] != evidence['domain_id'] or
                base['status'] != 'PASS' or base['survivor_count'] != 0 or
                base['output_mcap_count'] != 0 or
                base['transform_lookup_drop_count'] != 0):
            raise ValueError('base run contract failed')
        if (base['map_yaml'] != contract['axis_a']['map']['yaml'] or
                base['params_file'] != contract['production_inputs'][
                    'production_params'] or
                base['sanitized_manifest']['sha256'] !=
                manifest['sanitizer_snapshot']['sha256'] or
                base['loaded_runtime'] != attestation['loaded_runtime'] or
                base['prefix_s'] != expected_run_contract['prefix_s'] or
                base['playback_rate'] != expected_run_contract['playback_rate']):
            raise ValueError('map, sensor, motion, or runtime identity drift')
        expected_overrides = [
            value for item in profile_overrides(evidence['profile'])
            for value in ('-p', f'{item[0]}:={item[1]}')]
        amcl_process = next(
            item for item in base['teardown'] if item['name'] == 'amcl')
        if amcl_process['command'][-len(expected_overrides):] != expected_overrides:
            raise ValueError('profile launch override drift')
        operations = base['operations']
        if ([operation['command'][1:] for operation in operations[:4]] != [
                ['lifecycle', 'set', '/map_server', 'configure'],
                ['lifecycle', 'set', '/map_server', 'activate'],
                ['lifecycle', 'set', '/amcl', 'configure'],
                ['lifecycle', 'set', '/amcl', 'activate']] or
                any(operation['returncode'] != 0 for operation in operations)):
            raise ValueError('lifecycle active contract drift')
        authority = evidence['map_odom_authority']
        _exact_keys(authority, {
            'sanitized_input_count', 'generated_observed_count',
            'sole_runtime_authority'}, 'map odom authority')
        if (authority['sanitized_input_count'] != 0 or
                type(authority['generated_observed_count']) is not int or
                authority['generated_observed_count'] <= 0 or
                authority['sole_runtime_authority'] is not True):
            raise ValueError('map to odom authority drift')
        _validate_metric_group(evidence['metrics'])
        _validate_recorded_pairs(base, evidence['recorded_pose_pairs'])
        if evidence['recorded_pose_pairs'] != pair_recorded(
                base['observer']['clouds'], recorded):
            raise ValueError('recorded AMCL direct pairing drift')
        recomputed = derive_metrics(base, evidence['recorded_pose_pairs'])
        if evidence['metrics'] != recomputed:
            raise ValueError('Axis A metrics recomputation drift')
        resource_path = path.parent / base['resource']['file']['relative_path']
        if base['resource'] != _resource_summary(resource_path, path.parent):
            raise ValueError('AMCL resource summary drift')
        if sum(item.stat().st_size for item in path.parent.rglob('*')
               if item.is_file() and not item.is_symlink()) > \
                STORAGE_LIMITS['run_output_limit_bytes']:
            raise ValueError('Axis A run exceeds 2 MiB cap')
    if runs := [strict_json_load(root / record['relative_path'])
                for record in manifest['runs']]:
        bases = [strict_json_load(
            root / run['base_evidence']['relative_path']) for run in runs]
        parity_keys = ('map_yaml', 'params_file', 'sanitized_manifest',
                       'loaded_runtime', 'tf_bootstrap')
        for key in parity_keys:
            if any(base[key] != bases[0][key] for base in bases[1:]):
                raise ValueError(f'cross-run input parity drift: {key}')
        expected_clouds = expected_run_contract['max_clouds']
        if any(run['metrics']['accepted_update_count'] != expected_clouds
               for run in runs):
            raise ValueError('accepted update count drift')
        if expected_mode == 'full':
            scan_views = [base['scan_parity'] for base in bases]
            if any(view != scan_views[0] for view in scan_views[1:]):
                raise ValueError('full source scan view parity drift')
    return manifest


def _source_identity(path: Path) -> dict:
    return {
        'path': str(path), 'size_bytes': path.stat().st_size,
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def validate_axis_a_smoke(root: Path) -> dict:
    """Validate exactly P0 seed 11."""
    return validate_axis_a_artifact(root, 'smoke')


def validate_axis_a_full(root: Path) -> dict:
    """Validate exactly P0/P1/P2 by five seeds."""
    return validate_axis_a_artifact(root, 'full')
