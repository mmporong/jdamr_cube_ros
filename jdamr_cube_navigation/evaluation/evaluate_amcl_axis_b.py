#!/usr/bin/env python3
"""Validate G002 Axis B inputs, metrics, and promotion evidence."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
import statistics

from amcl_fault_contract import canonical_json_bytes, G002_ARTIFACT_LIMIT_BYTES
from amcl_fault_contract import PROFILES, SEEDS, STORAGE_LIMITS
from amcl_fault_contract import strict_json_load, strict_json_loads
from axis_b_kidnapped_driver import (
    INITIAL_OBSERVATION_RAD,
    POST_ZERO_OBSERVATION_RAD,
    ROTATION_TARGET_RAD,
)
from evaluate_amcl_axis_a import profile_overrides
from evaluate_amcl_axis_a import validate_axis_a_full
from generate_amcl_axis_b_input import SCENARIOS, validate_input
import run_amcl_determinism_preflight as preflight


SCHEMA_VERSION = 2
CLAIM_SMOKE = 'AXIS_B_RELOCALIZATION_SMOKE_SIMULATION_ONLY'
CLAIM_FULL = 'AXIS_B_RELOCALIZATION_PROFILE_MATRIX_SIMULATION_ONLY'
TRANSLATION_THRESHOLD_M = 0.15
YAW_THRESHOLD_RAD = 0.25
CONSECUTIVE_REQUIRED = 3
FULL_CLOUD_COUNT = 30
FULL_PREFIX_S = 220.0
SMOKE_CLOUD_COUNT = 3
SMOKE_PREFIX_S = 30.0
PLAYBACK_RATE = 2.0
AMCL_UPDATE_MIN_A_RAD = 0.2
# The observer stays alive through every commanded angular update, including
# the complete post-zero motion. This is derived from the event schedule, not
# from raw LaserScan count.
KIDNAPPED_TARGET_ACCEPTED_CLOUDS = (
    int(INITIAL_OBSERVATION_RAD / AMCL_UPDATE_MIN_A_RAD) +
    int(ROTATION_TARGET_RAD / AMCL_UPDATE_MIN_A_RAD) +
    int(round(POST_ZERO_OBSERVATION_RAD / AMCL_UPDATE_MIN_A_RAD)))
FULL_PLAN = tuple((scenario, profile, seed)
                  for scenario in SCENARIOS
                  for profile in PROFILES for seed in SEEDS)


def _exact_keys(value: object, expected: set[str], label: str) -> None:
    if type(value) is not dict or set(value) != expected:
        raise ValueError(f'{label} schema drift')


def _finite(value: object, label: str, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be numeric')
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        raise ValueError(f'{label} is outside its finite range')
    return result


def _yaw(pose: list[float]) -> float:
    x, y, z, w = pose[3:7]
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def recovery_metrics(clouds: list[dict], gt_pairs: list[dict],
                     t0_scan_index: int) -> dict:
    """Calculate three-consecutive recovery and false convergence."""
    if (len(clouds) != len(gt_pairs) or type(t0_scan_index) is not int or
            t0_scan_index < 0):
        raise ValueError('Axis B causal cardinality drift')
    consecutive = 0
    false_consecutive = 0
    recovery = None
    false_convergence = False
    samples = []
    for cloud, pair in zip(clouds, gt_pairs):
        _exact_keys(pair, {
            'scan_index', 'scan_header_stamp_ns', 'gt_before_stamp_ns',
            'gt_after_stamp_ns', 'gt_max_delta_ns',
            'gt_interpolation_fraction', 'gt_frame_id', 'gt_pose'}, 'GT pair')
        if (type(pair['gt_frame_id']) is not str or not pair['gt_frame_id'] or
                type(pair['gt_max_delta_ns']) is not int or
                pair['gt_max_delta_ns'] < 0 or
                pair['scan_header_stamp_ns'] !=
                cloud['fifo_associated_pose_scan_header_stamp_ns'] or
                not pair['gt_before_stamp_ns'] <=
                pair['scan_header_stamp_ns'] <= pair['gt_after_stamp_ns'] or
                not 0.0 <= pair['gt_interpolation_fraction'] <= 1.0):
            raise ValueError('GT pair time or frame drift')
        translation_m = math.dist(cloud['pose'][:2], pair['gt_pose'][:2])
        yaw_rad = abs(math.remainder(
            _yaw(cloud['pose']) - _yaw(pair['gt_pose']), 2.0 * math.pi))
        covariance = cloud['covariance']
        if type(covariance) is not list or len(covariance) != 36:
            raise ValueError('Axis B covariance shape drift')
        sigma_translation_m = 3.0 * math.sqrt(max(
            _finite(covariance[0], 'covariance x', 0.0),
            _finite(covariance[7], 'covariance y', 0.0)))
        sigma_yaw_rad = 3.0 * math.sqrt(
            _finite(covariance[35], 'covariance yaw', 0.0))
        inside = (translation_m <= TRANSLATION_THRESHOLD_M and
                  yaw_rad <= YAW_THRESHOLD_RAD)
        confident = (sigma_translation_m <= TRANSLATION_THRESHOLD_M and
                     sigma_yaw_rad <= YAW_THRESHOLD_RAD)
        consecutive = consecutive + 1 if inside else 0
        false_consecutive = false_consecutive + 1 if confident and not inside else 0
        samples.append({
            'scan_index': pair['scan_index'],
            'translation_error_m': translation_m,
            'yaw_error_rad': yaw_rad,
            'three_sigma_translation_m': sigma_translation_m,
            'three_sigma_yaw_rad': sigma_yaw_rad,
        })
        if pair['scan_index'] < t0_scan_index:
            consecutive = 0
            if false_consecutive >= CONSECUTIVE_REQUIRED:
                false_convergence = True
            continue
        if consecutive >= CONSECUTIVE_REQUIRED and recovery is None:
            recovery = {
                'scan_index': pair['scan_index'],
                'scans_after_t0': pair['scan_index'] - t0_scan_index,
                'fifo_associated_pose_scan_header_stamp_ns':
                    cloud['fifo_associated_pose_scan_header_stamp_ns'],
            }
        if false_consecutive >= CONSECUTIVE_REQUIRED:
            false_convergence = True
    return {'recovered': recovery is not None, 'first_recovery': recovery,
            'false_convergence': false_convergence, 'samples': samples}


def kidnapped_metrics(clouds: list[dict], pairs: list[dict],
                      t0_scan_index: int, trace: dict) -> dict:
    """Add initial convergence and post-zero coverage to kidnapped metrics."""
    result = recovery_metrics(clouds, pairs, t0_scan_index)
    consecutive = 0
    pre_converged = False
    for sample in result['samples']:
        if sample['scan_index'] >= t0_scan_index:
            break
        inside = (sample['translation_error_m'] <= TRANSLATION_THRESHOLD_M and
                  sample['yaw_error_rad'] <= YAW_THRESHOLD_RAD)
        consecutive = consecutive + 1 if inside else 0
        pre_converged = pre_converged or consecutive >= CONSECUTIVE_REQUIRED
    zero_ros_ns = next(item['ros_ns'] for item in trace['events']
                       if item['name'] == 'zero_hold_complete')
    rotation_ros_ns = next(item['ros_ns'] for item in trace['events']
                           if item['name'] == 'rotation_complete')
    last_stamp_ns = pairs[-1]['scan_header_stamp_ns'] if pairs else -1
    return {**result,
            'pre_t0_initial_converged': pre_converged,
            'last_accepted_scan_header_stamp_ns': last_stamp_ns,
            'rotation_complete_ros_ns': rotation_ros_ns,
            'zero_hold_complete_ros_ns': zero_ros_ns,
            'post_zero_hold_observed': last_stamp_ns >= zero_ros_ns}


def axis_a_ratios(axis_a_root: Path) -> dict:
    """Recompute paired P2/P0 resource ratios from validated Axis A."""
    manifest_path = axis_a_root / 'axis_a_manifest.json'
    manifest_identity = preflight._identity(manifest_path)
    manifest = validate_axis_a_full(axis_a_root)
    if preflight._identity(manifest_path) != manifest_identity:
        raise ValueError('Axis A manifest changed after validation')
    runs = {}
    run_identities = []
    for record in manifest['runs']:
        path = axis_a_root / record['relative_path']
        payload = path.read_bytes()
        payload_identity = {
            'relative_path': record['relative_path'],
            'size_bytes': len(payload),
            'sha256': hashlib.sha256(payload).hexdigest()}
        if payload_identity != record:
            raise ValueError('Axis A run changed after validation')
        run_identities.append((path, {
            'path': str(path), 'size_bytes': len(payload),
            'sha256': payload_identity['sha256']}))
        row = strict_json_loads(payload.decode('utf-8'))
        runs[(row['profile'], row['seed'])] = row['metrics']
    if preflight._identity(manifest_path) != manifest_identity:
        raise ValueError('Axis A manifest changed during ratio load')
    cpu = []
    latency = []
    for seed in SEEDS:
        p0 = runs[('P0', seed)]
        p2 = runs[('P2', seed)]
        denominator_cpu = _finite(
            p0['amcl_cpu_seconds_per_1000_scans'], 'P0 CPU', 0.0)
        denominator_latency = _finite(
            p0['scan_to_pose_latency_ms']['p95'], 'P0 latency', 0.0)
        if denominator_cpu <= 0.0 or denominator_latency <= 0.0:
            raise ValueError('Axis A ratio denominator must be positive')
        cpu.append(_finite(
            p2['amcl_cpu_seconds_per_1000_scans'], 'P2 CPU', 0.0) /
            denominator_cpu)
        latency.append(_finite(
            p2['scan_to_pose_latency_ms']['p95'], 'P2 latency', 0.0) /
            denominator_latency)
    result = {
        'cpu_ratios_by_seed': cpu,
        'latency_p95_ratios_by_seed': latency,
        'cpu_median': statistics.median(cpu),
        'latency_p95_median': statistics.median(latency),
        'individual_max': max(cpu + latency),
    }
    if any(preflight._identity(path) != identity
           for path, identity in run_identities):
        raise ValueError('Axis A run changed during ratio computation')
    if preflight._identity(manifest_path) != manifest_identity:
        raise ValueError('Axis A manifest changed during ratio computation')
    return result


def promotion_decision(full_results: list[dict], ratios: dict | None) -> dict:
    """Recompute the preregistered P2 promotion gate without fallback."""
    if len(full_results) != len(FULL_PLAN) or ratios is None:
        return {'status': 'NOT_EVALUATED', 'promote_p2': False,
                'selected_profile': 'P0'}
    by_key = {}
    for row in full_results:
        _exact_keys(row, {'scenario', 'profile', 'seed', 'recovered',
                          'false_convergence', 'recovery_scan'},
                    'promotion row')
        key = (row['scenario'], row['profile'], row['seed'])
        if key in by_key or key not in FULL_PLAN:
            raise ValueError('Axis B full result set drift')
        if (type(row['recovered']) is not bool or
                type(row['false_convergence']) is not bool or
                (row['recovery_scan'] is not None and
                 type(row['recovery_scan']) is not int)):
            raise ValueError('Axis B promotion scalar drift')
        by_key[key] = row
    if set(by_key) != set(FULL_PLAN):
        raise ValueError('Axis B full result set drift')
    _exact_keys(ratios, {
        'cpu_ratios_by_seed', 'latency_p95_ratios_by_seed', 'cpu_median',
        'latency_p95_median', 'individual_max'}, 'Axis A ratios')
    for key in ('cpu_ratios_by_seed', 'latency_p95_ratios_by_seed'):
        if type(ratios[key]) is not list or len(ratios[key]) != len(SEEDS):
            raise ValueError('Axis A ratio cardinality drift')
        for value in ratios[key]:
            _finite(value, key, 0.0)
    for key in ('cpu_median', 'latency_p95_median', 'individual_max'):
        _finite(ratios[key], key, 0.0)
    if (ratios['cpu_median'] != statistics.median(ratios['cpu_ratios_by_seed']) or
            ratios['latency_p95_median'] != statistics.median(
                ratios['latency_p95_ratios_by_seed']) or
            ratios['individual_max'] != max(
                ratios['cpu_ratios_by_seed'] +
                ratios['latency_p95_ratios_by_seed'])):
        raise ValueError('Axis A ratio recomputation drift')
    kidnapped = [(by_key[('kidnapped', 'P0', seed)],
                  by_key[('kidnapped', 'P2', seed)]) for seed in SEEDS]
    p2_recovers = all(right['recovered'] for _, right in kidnapped)
    false_count = sum(right['false_convergence'] for _, right in kidnapped)
    comparable = all(left['recovery_scan'] is not None and
                     right['recovery_scan'] is not None
                     for left, right in kidnapped)
    improvements = ([left['recovery_scan'] - right['recovery_scan']
                     for left, right in kidnapped] if comparable else [])
    non_worse = [value >= 0 for value in improvements]
    correct_offset_pairs = [
        (by_key[(scenario, 'P0', seed)], by_key[(scenario, 'P2', seed)])
        for scenario in ('correct_init', 'initial_offset') for seed in SEEDS]
    correct_offset = all(right['recovered'] and
                         not right['false_convergence']
                         for _, right in correct_offset_pairs)
    regressions = [right['recovery_scan'] - left['recovery_scan']
                   for left, right in correct_offset_pairs
                   if left['recovery_scan'] is not None and
                   right['recovery_scan'] is not None]
    regression_ready = len(regressions) == 10
    passed = (p2_recovers and false_count == 0 and comparable and
              all(non_worse) and
              sum(value > 0 for value in improvements) >= 3 and
              statistics.median(improvements) >= 1 and correct_offset and
              regression_ready and statistics.median(regressions) <= 1 and
              ratios['cpu_median'] <= 1.05 and
              ratios['latency_p95_median'] <= 1.05 and
              ratios['individual_max'] <= 1.10)
    return {
        'status': 'EVALUATED', 'promote_p2': passed,
        'selected_profile': 'P2' if passed else 'P0',
        'kidnapped_recovered_count': sum(
            right['recovered'] for _, right in kidnapped),
        'kidnapped_false_convergence_count': false_count,
        'kidnapped_non_worse_count': sum(non_worse),
        'kidnapped_strict_improvement_count': sum(
            value > 0 for value in improvements),
        'kidnapped_paired_median_improvement_scans': (
            statistics.median(improvements) if comparable else None),
        'correct_offset_recovered_count': sum(
            right['recovered'] for _, right in correct_offset_pairs),
        'correct_offset_false_convergence_count': sum(
            right['false_convergence'] for _, right in correct_offset_pairs),
        'correct_offset_median_regression_scans': (
            statistics.median(regressions) if regression_ready else None),
        'axis_a_ratios': ratios,
    }


def authority_limited_promotion(decision: dict) -> dict:
    """Retain P0 because rclpy MessageInfo cannot prove the TF publisher GID."""
    if decision['status'] != 'EVALUATED':
        return decision
    return {**decision,
            'status': 'NOT_EVALUATED_AUTHORITY_NOT_PROVEN',
            'parameter_rule_passed_without_authority': decision['promote_p2'],
            'promote_p2': False, 'selected_profile': 'P0'}


def _tree_records(root: Path) -> list[dict]:
    return sorted((preflight._relative_identity(path, root)
                   for path in root.rglob('*')
                   if path.is_file() and not path.is_symlink() and
                   path.name != 'axis_b_manifest.json'),
                  key=lambda record: record['relative_path'])


def _tree_digest(records: list[dict]) -> str:
    return hashlib.sha256(canonical_json_bytes(records)).hexdigest()


def validate_input_roots(records: list[dict]) -> dict[str, Path]:
    """Reopen exactly one current canonical input per Axis B scenario."""
    if type(records) is not list or not records:
        raise ValueError('Axis B canonical inputs are absent')
    result = {}
    for record in records:
        _exact_keys(record, {'scenario', 'root', 'manifest'}, 'input root')
        scenario = record['scenario']
        root = Path(record['root'])
        if scenario in result or scenario not in SCENARIOS:
            raise ValueError('Axis B input scenario drift')
        manifest = validate_input(root)
        identity = preflight._identity(root / 'axis_b_input_manifest.json')
        if manifest['scenario'] != scenario or record['manifest'] != identity:
            raise ValueError('Axis B canonical input identity drift')
        result[scenario] = root
    return result


def validate_manifest(path: Path, expected_mode: str) -> dict:
    """Reopen an artifact and recompute its identities and decision."""
    root = path.parent
    if (expected_mode not in ('smoke', 'full') or not root.is_absolute() or
            root != root.resolve() or root.is_symlink() or not root.is_dir() or
            any(item.is_symlink() for item in root.rglob('*'))):
        raise ValueError('Axis B artifact root drift')
    value = strict_json_load(path)
    _exact_keys(value, {
        'schema_version', 'mode', 'claim_scope', 'full_plan', 'executed_plan',
        'input_roots', 'axis_a_artifact', 'prepared_contract',
        'runtime_attestation', 'harness_sources', 'run_contract', 'runs',
        'promotion', 'tree_records', 'tree_sha256', 'tree_bytes',
        'production_unchanged'}, 'Axis B manifest')
    claim = CLAIM_FULL if expected_mode == 'full' else CLAIM_SMOKE
    if (value['schema_version'] != SCHEMA_VERSION or
            value['mode'] != expected_mode or value['claim_scope'] != claim or
            value['production_unchanged'] is not True):
        raise ValueError('Axis B manifest scalar drift')
    expected_full = [{'scenario': scenario, 'profile': profile, 'seed': seed}
                     for scenario, profile, seed in FULL_PLAN]
    expected = expected_full if expected_mode == 'full' else [
        {'scenario': 'correct_init', 'profile': 'P0', 'seed': 11}]
    if value['full_plan'] != expected_full or value['executed_plan'] != expected:
        raise ValueError('Axis B execution plan drift')
    inputs = validate_input_roots(value['input_roots'])
    expected_inputs = set(SCENARIOS) if expected_mode == 'full' else {
        'correct_init'}
    if set(inputs) != expected_inputs:
        raise ValueError('Axis B canonical input set drift')
    cloud_counts = {
        scenario: _expected_cloud_count(
            scenario, inputs[scenario], expected_mode)
        for scenario in inputs}
    expected_contract = {
        'cloud_counts': cloud_counts,
        'prefix_s': FULL_PREFIX_S if expected_mode == 'full' else SMOKE_PREFIX_S,
        'playback_rate': PLAYBACK_RATE, 'output_mcap_count': 0,
    }
    if value['run_contract'] != expected_contract:
        raise ValueError('Axis B run contract drift')
    for key, filename in (('prepared_contract', 'contract_snapshot.json'),
                          ('runtime_attestation',
                           'runtime_attestation_snapshot.json')):
        if value[key] != preflight._relative_identity(root / filename, root):
            raise ValueError(f'Axis B {key} drift')
    contract = strict_json_load(root / 'contract_snapshot.json')
    preflight._validate_source_records(contract['production_inputs'])
    attestation = strict_json_load(root / 'runtime_attestation_snapshot.json')
    expected_sources = {
        source.name: preflight._identity(source) for source in (
            Path(__file__).resolve(),
            Path(__file__).with_name('run_amcl_axis_b.py').resolve(),
            Path(__file__).with_name('generate_amcl_axis_b_input.py').resolve(),
            Path(__file__).with_name('axis_b_kidnapped_driver.py').resolve(),
            Path(__file__).with_name('amcl_particle_observer.py').resolve(),
            Path(__file__).with_name(
                'run_amcl_determinism_preflight.py').resolve())}
    if value['harness_sources'] != expected_sources:
        raise ValueError('Axis B harness source identity drift')
    records = _tree_records(root)
    if (value['tree_records'] != records or
            value['tree_sha256'] != _tree_digest(records) or
            value['tree_bytes'] != sum(row['size_bytes'] for row in records) or
            value['tree_bytes'] > G002_ARTIFACT_LIMIT_BYTES):
        raise ValueError('Axis B artifact tree identity or cap drift')
    if len(value['runs']) != len(expected):
        raise ValueError('Axis B run cardinality drift')
    promotion_rows = []
    parity = {}
    for index, (record, plan) in enumerate(zip(value['runs'], expected)):
        evidence_path = root / record['relative_path']
        if record != preflight._relative_identity(evidence_path, root):
            raise ValueError('Axis B run identity drift')
        evidence = strict_json_load(evidence_path)
        _exact_keys(evidence, {
            'schema_version', 'run_id', 'scenario', 'profile', 'seed',
            'domain_id', 'status', 'failure', 'base_evidence',
            'input_manifest', 'profile_parameters', 't0_scan_index',
            'metrics', 'gates'}, 'Axis B evidence')
        run_id = (f"axis_b__{plan['scenario']}__{plan['profile']}__"
                  f"seed_{plan['seed']}")
        if (evidence['schema_version'] != SCHEMA_VERSION or
                evidence['run_id'] != run_id or evidence['scenario'] !=
                plan['scenario'] or evidence['profile'] != plan['profile'] or
                evidence['seed'] != plan['seed'] or
                evidence['domain_id'] != 180 + index or
                evidence['status'] != 'PASS' or evidence['failure'] is not None or
                evidence['profile_parameters'] !=
                dict(profile_overrides(plan['profile']))):
            raise ValueError('Axis B run scalar drift')
        input_root = inputs[plan['scenario']]
        if evidence['input_manifest'] != preflight._identity(
                input_root / 'axis_b_input_manifest.json'):
            raise ValueError('Axis B run input identity drift')
        base_path = root / evidence['base_evidence']['relative_path']
        if evidence['base_evidence'] != preflight._relative_identity(base_path, root):
            raise ValueError('Axis B base evidence identity drift')
        bootstrap = _bootstrap_view(input_root)
        base = preflight.validate_replay_run_evidence(
            base_path, expected_run_id=run_id,
            expected_profile=plan['profile'], expected_seed=plan['seed'],
            expected_domain_id=evidence['domain_id'],
            expected_prefix_s=expected_contract['prefix_s'],
            expected_playback_rate=PLAYBACK_RATE,
            expected_max_clouds=cloud_counts[plan['scenario']],
            expected_observer_source=preflight._identity(
                Path(__file__).with_name('amcl_particle_observer.py')),
            expected_amcl_executable=attestation['loaded_runtime'][
                'amcl_executable'],
            expected_map_yaml=contract['axis_b']['map_profile']['yaml'],
            expected_params_file=contract['production_inputs'][
                'production_params'],
            expected_sanitized_manifest=preflight._identity(
                input_root / 'sanitizer_manifest.json'),
            expected_loaded_runtime=attestation['loaded_runtime'],
            sanitized_root=input_root, expected_bootstrap=bootstrap,
            allowed_extra_files=('axis_b_evidence.json',))
        _exact_keys(base, {
            'schema_version', 'run_id', 'profile', 'seed', 'domain_id',
            'status', 'failure', 'events', 'operations', 'observer_source',
            'observer_state', 'amcl_executable', 'map_yaml', 'params_file',
            'sanitized_manifest', 'resource', 'teardown', 'loaded_runtime',
            'prelude_observer', 'prelude_snapshot', 'clock_handoff',
            'tf_bootstrap', 'amcl_tf_error_counts',
            'transform_lookup_drop_count', 'prefix_s', 'playback_rate',
            'survivor_count', 'output_mcap_count', 'cmd_vel_publisher',
            'scan_parity', 'observer'}, 'Axis B base evidence')
        if (base['run_id'] != run_id or base['profile'] != plan['profile'] or
                base['seed'] != plan['seed'] or
                base['domain_id'] != evidence['domain_id'] or
                base['status'] != 'PASS' or base['survivor_count'] != 0 or
                base['output_mcap_count'] != 0 or
                base['transform_lookup_drop_count'] != 0 or
                base['cmd_vel_publisher'] != 'NOT_APPLICABLE' or
                base['prefix_s'] != expected_contract['prefix_s'] or
                base['playback_rate'] != PLAYBACK_RATE or
                base['loaded_runtime'] != attestation['loaded_runtime']):
            raise ValueError('Axis B base run gate failed')
        if (base['map_yaml'] != contract['axis_b']['map_profile']['yaml'] or
                base['params_file'] !=
                contract['production_inputs']['production_params'] or
                base['sanitized_manifest'] != preflight._identity(
                    input_root / 'sanitizer_manifest.json') or
                base['tf_bootstrap'] != bootstrap or
                base['scan_parity'] != preflight._scan_parity(
                    base['observer'], input_root)):
            raise ValueError('Axis B map, input, or scan parity drift')
        preflight._validate_run_operations(base['operations'], plan['seed'])
        if any(value != 0 for value in base['amcl_tf_error_counts'].values()):
            raise ValueError('Axis B AMCL TF error gate failed')
        expected_processes = {
            'player': 0, 'tf_prelude_player': 0, 'resource_sampler': -2,
            'observer': 0, 'amcl': 0, 'map_server': 0}
        processes = {item['name']: item for item in base['teardown']}
        if (set(processes) != set(expected_processes) or
                len(processes) != len(base['teardown']) or
                any(processes[name]['returncode'] != code or
                    processes[name]['survivors']
                    for name, code in expected_processes.items())):
            raise ValueError('Axis B teardown gate failed')
        observer = base['observer']
        if (observer['failure'] is not None or observer['initialpose_count'] != 1 or
                observer['pending_cloud_count'] != 0 or
                observer['pending_pose_count'] != 0 or
                len(observer['clouds']) != cloud_counts[plan['scenario']] or
                observer['raw_cloud_received_count'] !=
                cloud_counts[plan['scenario']] or
                observer['amcl_pose_received_count'] !=
                cloud_counts[plan['scenario']] or
                observer['readiness']['map_odom_tf_count'] <= 0):
            raise ValueError('Axis B observer gate failed')
        resource_path = (base_path.parent /
                         base['resource']['file']['relative_path'])
        if base['resource'] != preflight._resource_summary(
                resource_path, base_path.parent):
            raise ValueError('Axis B resource evidence drift')
        if sum(item.stat().st_size for item in base_path.parent.rglob('*')
               if item.is_file() and not item.is_symlink()) > \
                STORAGE_LIMITS['run_output_limit_bytes']:
            raise ValueError('Axis B run exceeds metrics-only cap')
        expected_overrides = [item for pair in profile_overrides(plan['profile'])
                              for item in ('-p', f'{pair[0]}:={pair[1]}')]
        amcl = next(item for item in base['teardown'] if item['name'] == 'amcl')
        if amcl['command'][-len(expected_overrides):] != expected_overrides:
            raise ValueError('Axis B profile override drift')
        truth = _input_truth_view(input_root)
        pairs = _pair_clouds(base['observer']['clouds'], truth)
        t0 = _t0_scan_index(plan['scenario'], truth, input_root)
        if plan['scenario'] == 'kidnapped':
            input_manifest = strict_json_load(
                input_root / 'axis_b_input_manifest.json')
            trace = strict_json_load(Path(
                input_manifest['trace_evidence']['path']))
            recomputed = kidnapped_metrics(
                base['observer']['clouds'], pairs, t0, trace)
        else:
            recomputed = recovery_metrics(base['observer']['clouds'], pairs, t0)
        if evidence['t0_scan_index'] != t0 or evidence['metrics'] != recomputed:
            raise ValueError('Axis B metric recomputation drift')
        expected_gates = {
            'input_parity': True, 'profile_parity': True,
            'map_odom_authority': {
                'input_transform_count': 0,
                'isolated_expected_runtime_sources': ['amcl'],
                'proof_status':
                    'ISOLATED_EXPECTED_RUNTIME_SOURCE_NO_GID_PROOF',
                'observed_transform_count':
                    base['observer']['readiness']['map_odom_tf_count']},
            'lifecycle_active': all(
                operation['returncode'] == 0 for operation in base['operations']),
            'finite_metrics': True,
            'stationary_window': ('PASS' if plan['scenario'] == 'kidnapped'
                                  else 'NOT_APPLICABLE'),
            'kidnapped_pre_t0_initial_convergence': (
                recomputed['pre_t0_initial_converged']
                if plan['scenario'] == 'kidnapped' else 'NOT_APPLICABLE'),
            'kidnapped_post_zero_observation': (
                recomputed['post_zero_hold_observed']
                if plan['scenario'] == 'kidnapped' else 'NOT_APPLICABLE'),
            'contact_zero': truth['contact_count'] == 0,
            'final_zero': all(abs(value) <= 1e-9
                              for value in truth['final_twist']),
            'survivor_zero': base['survivor_count'] == 0,
            'production_hashes_unchanged': True,
        }
        scalar_gates = [gate for gate in expected_gates.values()
                        if type(gate) is not dict]
        if (evidence['gates'] != expected_gates or
                expected_gates['map_odom_authority'][
                    'observed_transform_count'] <= 0 or
                not all(gate is True or gate in ('PASS', 'NOT_APPLICABLE')
                        for gate in scalar_gates)):
            raise ValueError('Axis B common gate drift')
        parity.setdefault(plan['scenario'], base['scan_parity'])
        if parity[plan['scenario']] != base['scan_parity']:
            raise ValueError('Axis B input replay parity drift')
        promotion_rows.append({
            'scenario': plan['scenario'], 'profile': plan['profile'],
            'seed': plan['seed'], 'recovered': recomputed['recovered'],
            'false_convergence': recomputed['false_convergence'],
            'recovery_scan': (recomputed['first_recovery']['scans_after_t0']
                              if recomputed['first_recovery'] else None),
        })
    if expected_mode == 'smoke':
        expected_promotion = promotion_decision([], None)
        if value['axis_a_artifact'] is not None:
            raise ValueError('smoke must not bind Axis A full evidence')
    else:
        _exact_keys(value['axis_a_artifact'], {'root', 'manifest'},
                    'Axis A artifact')
        axis_a_root = Path(value['axis_a_artifact']['root'])
        if value['axis_a_artifact']['manifest'] != preflight._identity(
                axis_a_root / 'axis_a_manifest.json'):
            raise ValueError('Axis A artifact identity drift')
        expected_promotion = authority_limited_promotion(promotion_decision(
            promotion_rows, axis_a_ratios(axis_a_root)))
    if value['promotion'] != expected_promotion:
        raise ValueError('Axis B promotion recomputation drift')
    preflight._validate_source_records(contract['production_inputs'])
    return value


def _input_truth_view(root: Path) -> dict:
    from run_amcl_axis_b import _input_truth
    return _input_truth(root)


def _pair_clouds(clouds: list[dict], truth: dict) -> list[dict]:
    from run_amcl_axis_b import _pair_gt
    return _pair_gt(clouds, truth)


def _bootstrap_view(root: Path) -> dict:
    from run_amcl_axis_b import _axis_b_bootstrap
    return _axis_b_bootstrap(root)


def _t0_scan_index(scenario: str, truth: dict, root: Path) -> int:
    if scenario != 'kidnapped':
        return 0
    manifest = strict_json_load(root / 'axis_b_input_manifest.json')
    trace = strict_json_load(Path(manifest['trace_evidence']['path']))
    stamp = trace['t0_scan_stamp_ns']
    try:
        return truth['scan_stamps_ns'].index(stamp)
    except ValueError as exc:
        raise ValueError('kidnapped t0 scan is absent from canonical input') from exc


def _expected_cloud_count(scenario: str, root: Path,
                          mode: str = 'full') -> int:
    if scenario != 'kidnapped':
        return SMOKE_CLOUD_COUNT if mode == 'smoke' else FULL_CLOUD_COUNT
    manifest = strict_json_load(root / 'axis_b_input_manifest.json')
    trace = strict_json_load(Path(manifest['trace_evidence']['path']))
    zero_ros_ns = next(item['ros_ns'] for item in trace['events']
                       if item['name'] == 'zero_hold_complete')
    final_ros_ns = next(item['ros_ns'] for item in trace['events']
                        if item['name'] == 'final_zero_hold_complete')
    scans = _input_truth_view(root)['scan_stamps_ns']
    if (POST_ZERO_OBSERVATION_RAD < AMCL_UPDATE_MIN_A_RAD or
            not any(zero_ros_ns <= stamp <= final_ros_ns for stamp in scans)):
        raise ValueError('kidnapped input lacks post-zero scan schedule')
    return KIDNAPPED_TARGET_ACCEPTED_CLOUDS
