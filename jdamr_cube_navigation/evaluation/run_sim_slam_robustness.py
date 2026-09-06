#!/usr/bin/env python3
"""Build, validate, run, and aggregate a 5-seed one-factor SLAM matrix."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
from typing import Any
import xml.etree.ElementTree as ET

from make_sim_sensor_variant import apply_sensor_settings

from run_sim_slam_experiment import (
    _git_commit,
    _workspace_fingerprint,
    summarize_resources,
    validate_json_schema,
)

import yaml


SEEDS = (11, 23, 42, 67, 89)
PROVENANCE_KINDS = (
    'measured_observation', 'derived_stress', 'synthetic_stress')
FAULT_DEFAULTS = {
    'scan_range_noise_stddev_m': 0.0,
    'scan_message_dropout_probability': 0.0,
    'scan_stamp_jitter_max_s': 0.0,
    'scan_transport_delay_s': 0.0,
    'odom_transport_delay_s': 0.0,
    'tf_transport_delay_s': 0.0,
    'wheel_translation_scale': 1.0,
    'wheel_slip_stddev_fraction': 0.0,
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _write_reproducible(path: Path, content: str) -> None:
    """Write generated evidence once, or accept an identical prior file."""
    if path.exists():
        if path.read_text(encoding='utf-8') != content:
            raise ValueError(f'generated artifact already differs: {path}')
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding='utf-8')


def _pointer(document: Any, pointer: str) -> Any:
    if not pointer.startswith('/'):
        raise ValueError(f'JSON pointer must start with /: {pointer}')
    value = document
    for token in pointer[1:].split('/'):
        token = token.replace('~1', '/').replace('~0', '~')
        value = value[int(token)] if isinstance(value, list) else value[token]
    return value


def default_conditions(sensor_profile: Path) -> list[dict[str, Any]]:
    """Return the frozen baseline and one-factor robustness conditions."""
    source = json.loads(sensor_profile.read_text(encoding='utf-8'))
    source_sha = _sha256(sensor_profile)

    def provenance(kind: str, pointer: str, rationale: str) -> dict[str, Any]:
        return {
            'kind': kind,
            'source_path': str(sensor_profile.resolve()),
            'source_sha256': source_sha,
            'json_pointer': pointer,
            'source_value': _pointer(source, pointer),
            'rationale': rationale,
        }

    baseline = {
        'label': 'measured_timing_baseline',
        'lidar_update_rate_hz': 9.653,
        'fault_values': copy.deepcopy(FAULT_DEFAULTS),
        'changed_factor': None,
        'provenance': {
            'lidar_update_rate_hz': provenance(
                'measured_observation', '/timing/scan/observed_rate_hz',
                'Successful real run observed scan rate.'),
        },
    }

    def changed(label: str, factor: str, value: float,
                kind: str, pointer: str, rationale: str) -> dict[str, Any]:
        condition = copy.deepcopy(baseline)
        condition['label'] = label
        condition['changed_factor'] = factor
        if factor == 'lidar_update_rate_hz':
            condition[factor] = value
        else:
            condition['fault_values'][factor] = value
        condition['provenance'][factor] = provenance(
            kind, pointer, rationale)
        return condition

    return [
        baseline,
        changed(
            'scan_period_8hz', 'lidar_update_rate_hz', 8.0,
            'derived_stress', '/timing/scan/observed_rate_hz',
            'A lower-than-observed rate stress; not a measured real rate.'),
        changed(
            'scan_jitter_p99', 'scan_stamp_jitter_max_s', 0.002164,
            'derived_stress',
            '/simulation_initial_candidates/scan_period_abs_jitter_p99_s',
            'Measured p99 magnitude injected with a bounded uniform model; '
            'the model itself is not a measured distribution.'),
        changed(
            'range_noise_5cm', 'scan_range_noise_stddev_m', 0.05,
            'synthetic_stress', '/limitations/0',
            'Range noise is unidentifiable without external truth.'),
        changed(
            'scan_dropout_5pct',
            'scan_message_dropout_probability', 0.05,
            'synthetic_stress', '/limitations/2',
            'No-return rays do not measure transport message loss.'),
        changed(
            'wheel_scale_0p95', 'wheel_translation_scale', 0.95,
            'synthetic_stress', '/limitations/0',
            'Odometry accuracy is unidentifiable in the real bag.'),
        changed(
            'wheel_slip_3pct', 'wheel_slip_stddev_fraction', 0.03,
            'synthetic_stress', '/limitations/0',
            'Incremental wheel slip is an explicit synthetic stress.'),
        changed(
            'tf_delay_p99', 'tf_transport_delay_s', 0.029564,
            'derived_stress',
            '/cross_stream_alignment/scan_to_odom_base_tf/'
            'absolute_nearest_header_offset_s/p99',
            'Cross-stream alignment p99 reused as a delay stress, not '
            'latency.'),
        changed(
            'transport_delay_p95', 'scan_transport_delay_s', 0.107408,
            'derived_stress',
            '/simulation_initial_candidates/scan_recorder_minus_header_p95_s',
            'Composite recorder latency reused as a stress, not sensor '
            'latency.'),
    ]


def _flatten(condition: dict[str, Any]) -> dict[str, float]:
    return {
        'lidar_update_rate_hz': condition['lidar_update_rate_hz'],
        **condition['fault_values'],
    }


def validate_contract(
        conditions: list[dict[str, Any]], seeds: tuple[int, ...],
        sensor_profile: Path) -> None:
    """Reject non-5-seed, multi-factor, or untraceable matrices."""
    if len(seeds) != 5 or len(set(seeds)) != 5:
        raise ValueError('matrix requires exactly five distinct seeds')
    if any(seed < 0 for seed in seeds):
        raise ValueError('seeds must be non-negative')
    labels = [condition['label'] for condition in conditions]
    if len(labels) != len(set(labels)):
        raise ValueError('condition labels must be unique')
    baselines = [condition for condition in conditions
                 if condition['changed_factor'] is None]
    if len(baselines) != 1:
        raise ValueError('matrix requires exactly one baseline')
    baseline = baselines[0]
    baseline_flat = _flatten(baseline)
    source = json.loads(sensor_profile.read_text(encoding='utf-8'))
    for condition in conditions:
        changed_fields = [
            name for name, value in _flatten(condition).items()
            if value != baseline_flat[name]
        ]
        declared = condition['changed_factor']
        if declared is None and changed_fields:
            raise ValueError('baseline differs from itself')
        if declared is not None and changed_fields != [declared]:
            raise ValueError(
                f'{condition["label"]} must change exactly one factor; '
                f'found {changed_fields}')
        for factor, provenance in condition['provenance'].items():
            if provenance['kind'] not in PROVENANCE_KINDS:
                raise ValueError(f'invalid provenance kind for {factor}')
            if provenance['source_sha256'] != _sha256(sensor_profile):
                raise ValueError(f'source hash mismatch for {factor}')
            if _pointer(source, provenance['json_pointer']) != (
                    provenance['source_value']):
                raise ValueError(f'JSON pointer mismatch for {factor}')
        required_provenance = set(
            baseline['provenance'] if declared is None
            else (*baseline['provenance'], declared))
        missing_provenance = required_provenance - set(condition['provenance'])
        if missing_provenance:
            raise ValueError(
                f'{condition["label"]} lacks provenance: '
                f'{sorted(missing_provenance)}')
        if declared in (
                'scan_range_noise_stddev_m',
                'wheel_translation_scale', 'wheel_slip_stddev_fraction',
                'scan_transport_delay_s', 'odom_transport_delay_s',
                'tf_transport_delay_s'):
            if condition['provenance'][declared]['kind'] == (
                    'measured_observation'):
                raise ValueError(f'{declared} cannot be claimed as measured')


def write_raw_bridge(base_bridge: Path, output: Path) -> None:
    """Write a bridge that exposes faultable Gazebo topics under /sim_raw."""
    entries = yaml.safe_load(base_bridge.read_text(encoding='utf-8'))
    replacements = {
        'scan': 'sim_raw/scan', 'odom': 'sim_raw/odom', 'tf': 'sim_raw/tf'}
    changed = []
    for entry in entries:
        name = entry.get('ros_topic_name')
        if name in replacements and entry.get('direction') == 'GZ_TO_ROS':
            entry['ros_topic_name'] = replacements[name]
            changed.append(name)
    if sorted(changed) != sorted(replacements):
        raise ValueError(
            f'bridge must replace scan, odom, and tf exactly once: {changed}')
    _write_reproducible(output, yaml.safe_dump(entries, sort_keys=False))


def write_urdf_variant(
        base_urdf: Path, output: Path, update_rate_hz: float) -> None:
    """Write a LiDAR-rate-only URDF variant with its exact input hash."""
    tree = ET.parse(base_urdf)
    changes = apply_sensor_settings(
        tree.getroot(), {'lidar_update_rate_hz': update_rate_hz})
    disabled_cameras = []
    for sensor in tree.getroot().iter('sensor'):
        if sensor.get('type') not in ('camera', 'rgbd_camera', 'depth_camera'):
            continue
        always_on = sensor.find('always_on')
        if always_on is not None:
            always_on.text = 'false'
            disabled_cameras.append(sensor.get('name'))
    ET.indent(tree, space='  ')
    serialized = ET.tostring(
        tree.getroot(), encoding='unicode', xml_declaration=True)
    _write_reproducible(output, serialized)
    manifest = {
        'schema_version': 1,
        'base': {
            'path': str(base_urdf.resolve()), 'sha256': _sha256(base_urdf)},
        'output': {'path': str(output.resolve()), 'sha256': _sha256(output)},
        'changes': changes,
        'disabled_camera_sensors': disabled_cameras,
    }
    _write_reproducible(
        output.with_suffix('.manifest.json'), json.dumps(manifest, indent=2))


def _distribution(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {
        'count': len(values), 'min': min(values), 'max': max(values),
        'mean': statistics.fmean(values),
        'stddev': statistics.pstdev(values),
    }


def _command_argument(command: list[str], name: str) -> str:
    """Return a required command argument from the frozen matrix command."""
    try:
        return command[command.index(name) + 1]
    except (ValueError, IndexError) as error:
        raise ValueError(f'matrix command lacks {name}') from error


def _finite_number(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _read_resource_samples(path: Path) -> list[dict[str, Any]]:
    """Parse resource JSONL and reject malformed or non-finite samples."""
    samples = []
    for line_number, line in enumerate(
            path.read_text(encoding='utf-8').splitlines(), start=1):
        if not line.strip():
            continue
        try:
            sample = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(
                f'resource sample line {line_number} is invalid '
                'JSON') from error
        required = {
            'monotonic_s', 'label', 'cpu_pct_one_core', 'rss_mb',
            'process_count'}
        if not required <= set(sample):
            raise ValueError(
                f'resource sample line {line_number} lacks required fields')
        if (not isinstance(sample['label'], str)
                or not _finite_number(sample['monotonic_s'])
                or not _finite_number(sample['rss_mb'])
                or sample['rss_mb'] < 0
                or not isinstance(sample['process_count'], int)
                or sample['process_count'] < 0
                or (sample['cpu_pct_one_core'] is not None
                    and (not _finite_number(sample['cpu_pct_one_core'])
                         or sample['cpu_pct_one_core'] < 0))):
            raise ValueError(
                f'resource sample line {line_number} has invalid values')
        samples.append(sample)
    if not samples:
        raise ValueError('resource sample file is empty')
    return samples


def _resource_summary_matches(
        metrics: dict[str, Any], samples: list[dict[str, Any]]) -> bool:
    """Require the recorded JSONL to reproduce the metrics resource summary."""
    expected = summarize_resources(samples)
    actual = metrics.get('resources')
    if not isinstance(actual, dict):
        return False
    if actual.get('sampling_interval_s') != expected['sampling_interval_s']:
        return False
    actual_groups = actual.get('by_process_group', {})
    if set(actual_groups) != set(expected['by_process_group']):
        return False
    for label, expected_group in expected['by_process_group'].items():
        actual_group = actual_groups[label]
        for key, expected_value in expected_group.items():
            actual_value = actual_group.get(key)
            if isinstance(expected_value, float):
                if (not _finite_number(actual_value)
                        or not math.isclose(actual_value, expected_value,
                                            rel_tol=1e-12, abs_tol=1e-12)):
                    return False
            elif actual_value != expected_value:
                return False
    return True


def _pgm_sanity(path: Path) -> dict[str, int] | None:
    """Return Cartographer PGM pixel classes when dimensions are consistent."""
    data = path.read_bytes()
    if not data.startswith(b'P5\n'):
        return None
    offset = 3
    header_lines = []
    while len(header_lines) < 2 and offset < len(data):
        end = data.find(b'\n', offset)
        if end < 0:
            return None
        line = data[offset:end]
        offset = end + 1
        if line.startswith(b'#') or not line:
            continue
        header_lines.append(line)
    if len(header_lines) != 2:
        return None
    try:
        width, height = (int(value) for value in header_lines[0].split())
        maximum = int(header_lines[1])
    except (TypeError, ValueError):
        return None
    pixels = data[offset:]
    if width <= 0 or height <= 0 or maximum != 255:
        return None
    if len(pixels) != width * height:
        return None
    classes = {
        'unknown': sum(value == 128 for value in pixels),
        'free': sum(value >= 250 for value in pixels),
        'occupied': sum(value <= 50 for value in pixels),
    }
    return classes if all(classes.values()) else None


def realized_fault_passed(
        condition: dict[str, Any], stats: dict[str, Any]) -> bool:
    """Confirm that the requested fault produced a nonzero observed effect."""
    factor = condition['changed_factor']
    if factor is None:
        return stats.get('scan_received', 0) > 1
    if factor == 'lidar_update_rate_hz':
        rate = stats.get('scan_observed_rate_hz')
        return rate is not None and abs(
            rate - condition['lidar_update_rate_hz']) <= max(0.5, rate * 0.1)
    if factor == 'scan_message_dropout_probability':
        received = stats.get('scan_received', 0)
        if received <= 0:
            return False
        realized = stats.get('scan_dropped', 0) / received
        requested = condition['fault_values'][factor]
        tolerance = max(
            0.03, 4.0 * math.sqrt(requested * (1.0 - requested) / received))
        return abs(realized - requested) <= tolerance
    if factor == 'scan_range_noise_stddev_m':
        realized = stats.get('scan_noise_rms_m') or 0.0
        requested = condition['fault_values'][factor]
        return 0.75 * requested <= realized <= 1.25 * requested
    if factor == 'scan_stamp_jitter_max_s':
        realized = stats.get('scan_abs_stamp_jitter_max_s', 0.0)
        requested = condition['fault_values'][factor]
        return 0.7 * requested <= realized <= requested * 1.001
    if factor == 'wheel_translation_scale':
        realized = stats.get('wheel_scale_mean')
        requested = condition['fault_values'][factor]
        tf_received = stats.get('wheel_tf_received', 0)
        tf_corrected = stats.get('tf_wheel_corrections', 0)
        return (stats.get('wheel_scale_samples', 0) > 0
                and tf_received > 0
                and tf_corrected / tf_received >= 0.95
                and realized is not None
                and math.isclose(realized, requested, rel_tol=0.01))
    if factor == 'wheel_slip_stddev_fraction':
        realized = stats.get('wheel_scale_stddev')
        requested = condition['fault_values'][factor]
        tf_received = stats.get('wheel_tf_received', 0)
        tf_corrected = stats.get('tf_wheel_corrections', 0)
        return (stats.get('wheel_scale_samples', 0) > 0
                and tf_received > 0
                and tf_corrected / tf_received >= 0.95
                and realized is not None
                and 0.75 * requested <= realized <= 1.25 * requested)
    labels = {
        'scan_transport_delay_s': 'scan',
        'odom_transport_delay_s': 'odom',
        'tf_transport_delay_s': 'tf',
    }
    label = labels.get(factor)
    if label:
        observed = stats.get('delay_observations', {}).get(label, {})
        requested = condition['fault_values'][factor]
        return (observed.get('messages', 0) > 0
                and observed.get('mean_s', 0.0) >= requested)
    return False


def aggregate(
        matrix_manifest: dict[str, Any], results_root: Path) -> dict[str, Any]:
    """Aggregate PASS, FAIL, and INVALID runs without dropping failures."""
    conditions = matrix_manifest['conditions']
    seeds = tuple(matrix_manifest['seeds'])
    sensor_profile = Path(matrix_manifest['sensor_profile']['path'])
    if (not sensor_profile.is_file()
            or _sha256(sensor_profile) != matrix_manifest[
                'sensor_profile']['sha256']):
        raise ValueError('matrix sensor profile is missing or changed')
    validate_contract(conditions, seeds, sensor_profile)
    fixed = matrix_manifest['fixed']
    generated_profiles = matrix_manifest['generated_profiles']
    generated_faults = matrix_manifest['generated_faults']
    command_by_run_id = {
        _command_argument(command, '--run-id'): command
        for command in matrix_manifest['commands']}
    schema_path = Path(__file__).with_name('experiment_manifest.schema.json')
    run_schema = json.loads(schema_path.read_text(encoding='utf-8'))
    fixed_artifacts = (
        fixed['world'], fixed['cartographer_config'], fixed['bridge_config'])
    if any(not Path(item['path']).is_file()
           or _sha256(Path(item['path'])) != item['sha256']
           for item in fixed_artifacts):
        raise ValueError('a fixed matrix artifact is missing or changed')
    for label in (condition['label'] for condition in conditions):
        for collection in (generated_profiles, generated_faults):
            item = collection[label]
            if (not Path(item['path']).is_file()
                    or _sha256(Path(item['path'])) != item['sha256']):
                raise ValueError(
                    'generated matrix artifact is missing or changed: '
                    f'{label}')
        fault_document = json.loads(
            Path(generated_faults[label]['path']).read_text(encoding='utf-8'))
        condition = next(item for item in conditions if item['label'] == label)
        if fault_document != {
                'schema_version': 1, 'label': label,
                'changed_factor': condition['changed_factor'],
                'values': condition['fault_values'],
                'provenance': condition['provenance']}:
            raise ValueError(f'generated fault document differs: {label}')
    output = {'schema_version': 1, 'conditions': {}, 'overall': 'PASS'}
    for condition in conditions:
        rows = []
        ate_values = []
        ate_yaw_values = []
        rpe_values = []
        rpe_yaw_values = []
        completion_values = []
        cpu_values = []
        rss_values = []
        for seed in seeds:
            run_id = f'{condition["label"]}__cartographer__seed_{seed}'
            run_dir = results_root / run_id
            try:
                command = command_by_run_id[run_id]
                manifest = json.loads(
                    (run_dir / 'execution_manifest.json').read_text())
                run_manifest = json.loads(
                    (run_dir / 'run_manifest.json').read_text())
                validate_json_schema(run_manifest, run_schema, run_schema)
                metrics = json.loads((run_dir / 'metrics.json').read_text())
                stats = manifest['fault_realization']['observed']
                valid = bool(metrics['validity']['valid'])
                completed = bool(metrics['completion']['completed'])
                realized = realized_fault_passed(condition, stats)
                expected_fault_document = {
                    'schema_version': 1,
                    'label': condition['label'],
                    'changed_factor': condition['changed_factor'],
                    'values': condition['fault_values'],
                    'provenance': condition['provenance'],
                }
                expected_profile = generated_profiles[condition['label']]
                expected_fault = generated_faults[condition['label']]
                identity_ok = all((
                    manifest.get('run_label') == run_id,
                    manifest.get('backend') == matrix_manifest['backend'],
                    manifest.get('seed') == seed,
                    manifest.get('route') == fixed['route'],
                    manifest.get('spawn_xy_m') == fixed['spawn_xy_m'],
                    manifest.get('corridor_distance_m') == (
                        fixed['corridor_distance_m']),
                    manifest.get('profile', {}).get('label') == (
                        condition['label']),
                    manifest.get('profile', {}).get('sha256') == (
                        expected_profile['sha256']),
                    manifest.get('profile', {}).get('urdf') == (
                        expected_profile['path']),
                    manifest.get('fault_profile', {}).get('document') == (
                        expected_fault_document),
                    manifest.get('fault_profile', {}).get('sha256') == (
                        expected_fault['sha256']),
                    manifest.get('fault_profile', {}).get('path') == (
                        expected_fault['path']),
                    manifest.get('bridge_config') == fixed['bridge_config'],
                    manifest.get('world') == fixed['world'],
                    manifest.get('cartographer_config') == (
                        fixed['cartographer_config']),
                    run_manifest.get('run_id') == run_id,
                    run_manifest.get('config', {}).get('backend') == (
                        matrix_manifest['backend']),
                    run_manifest.get('config', {}).get('config_sha256') == (
                        fixed['cartographer_config']['sha256']),
                    run_manifest.get('motion', {}).get('schedule_id') == (
                        f'{fixed["route"]}_{fixed["corridor_distance_m"]:g}m'),
                    run_manifest.get('randomization', {}).get('seed') == seed,
                    run_manifest.get('randomization', {}).get('theta') == (
                        condition['fault_values']),
                    run_manifest.get('software', {}).get('git_sha') == (
                        fixed['software']['git_sha']),
                    run_manifest.get('software', {}).get(
                        'workspace_dirty') == fixed['software'][
                            'workspace_dirty'],
                    run_manifest.get('software', {}).get(
                        'workspace_diff_sha256') == fixed['software'][
                            'workspace_diff_sha256'],
                    Path(_command_argument(command, '--profile-urdf')) == (
                        Path(expected_profile['path'])),
                    Path(_command_argument(command, '--fault-profile')) == (
                        Path(expected_fault['path'])),
                    Path(_command_argument(command, '--world')) == (
                        Path(fixed['world']['path'])),
                    int(_command_argument(command, '--seed')) == seed,
                ))
                required_exit_codes = {
                    'route', 'map_state', 'map_render', 'recorder',
                    'backend', 'fault_injector', 'gazebo'}
                required_teardown = {
                    'recorder', 'backend', 'fault_injector', 'gazebo'}
                exit_codes = manifest.get('exit_codes', {})
                teardown = manifest.get('teardown_remaining_processes', {})
                execution_ok = (
                    manifest.get('status') == 'complete'
                    and required_exit_codes <= set(exit_codes)
                    and all(
                        exit_codes[key] == 0 for key in required_exit_codes)
                    and required_teardown <= set(teardown)
                    and all(not teardown[key] for key in required_teardown))
                map_state = manifest.get('map_artifact', {})
                map_images = manifest.get('map_images', ())
                map_image_by_suffix = {
                    Path(item.get('path', '')).suffix: item
                    for item in map_images}
                map_yaml = Path(
                    map_image_by_suffix.get('.yaml', {}).get('path', ''))
                map_pgm = Path(
                    map_image_by_suffix.get('.pgm', {}).get('path', ''))
                map_metadata = (
                    yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
                    if map_yaml.is_file() else {})
                pgm_classes = (
                    _pgm_sanity(map_pgm) if map_pgm.is_file() else None)
                map_ok = (
                    map_state.get('bytes', 0) > 0
                    and Path(map_state.get('path', '')).is_file()
                    and Path(map_state['path']).stat().st_size == (
                        map_state['bytes'])
                    and _sha256(Path(map_state['path'])) == map_state['sha256']
                    and len(map_images) == 2
                    and all(Path(item['path']).is_file()
                            and Path(item['path']).stat().st_size
                            == item['bytes']
                            and _sha256(Path(item['path'])) == item['sha256']
                            for item in map_images)
                    and Path(map_metadata.get('image', '')).resolve()
                    == map_pgm.resolve()
                    and _finite_number(map_metadata.get('resolution'))
                    and map_metadata['resolution'] > 0
                    and pgm_classes is not None)
                expected_outcome = (
                    'PASS' if valid and completed
                    else 'FAIL' if valid else 'INVALID')
                schema_outcome_ok = run_manifest.get('outcome', {}).get(
                    'status') == expected_outcome
                terminal_consistent = (
                    manifest.get('failure') is None
                    and schema_outcome_ok
                    and bool(run_manifest['outcome'].get('terminal_reason')))
                backend_resources = metrics.get('resources', {}).get(
                    'by_process_group', {}).get('backend', {})
                resource_samples = manifest.get('resource_samples', {})
                sample_path = Path(resource_samples.get('path', ''))
                samples = (
                    _read_resource_samples(sample_path)
                    if sample_path.is_file() else [])
                resources_ok = (
                    backend_resources.get('cpu_samples', 0) > 0
                    and backend_resources.get(
                        'cpu_mean_pct_one_core') is not None
                    and backend_resources.get('rss_peak_mb') is not None
                    and resource_samples.get('samples', 0) > 0
                    and sample_path.is_file()
                    and len(samples) == resource_samples.get('samples')
                    and _sha256(sample_path) == resource_samples.get('sha256')
                    and _resource_summary_matches(metrics, samples))
                artifact_items = run_manifest.get('artifacts', ())
                artifact_hashes = {
                    item['uri']: item['sha256']
                    for item in artifact_items}
                all_manifest_artifacts_ok = (
                    len(artifact_hashes) == len(artifact_items)
                    and all(Path(item['uri']).is_file()
                            and _sha256(Path(item['uri'])) == item['sha256']
                            for item in artifact_items))
                source_bag = Path(run_manifest['source']['bag_uri'])
                source_bag_ok = (
                    source_bag.is_file()
                    and _sha256(source_bag) == run_manifest[
                        'source']['bag_sha256'])
                evidence_paths = [
                    run_dir / 'metrics.json', sample_path,
                    Path(map_state.get('path', '')), map_yaml, map_pgm,
                    Path(manifest['fault_realization']['path']),
                    Path(run_manifest['source']['bag_uri']),
                ]
                artifacts_ok = (
                    all_manifest_artifacts_ok
                    and source_bag_ok
                    and all(
                        path.is_file()
                        and artifact_hashes.get(str(path.resolve()))
                        == _sha256(path)
                        for path in evidence_paths))
                fault_path = Path(manifest['fault_realization']['path'])
                fault_ok = (
                    fault_path.is_file()
                    and _sha256(fault_path) == manifest[
                        'fault_realization']['sha256']
                    and json.loads(
                        fault_path.read_text(encoding='utf-8')) == stats
                    and stats.get('seed') == seed
                    and stats.get('profile') == condition['fault_values'])
                finite_metrics = all(_finite_number(value) for value in (
                    metrics['ate']['translation_rms_m'],
                    metrics['ate']['yaw_rms_rad'],
                    metrics['rpe']['translation_rms_m'],
                    metrics['rpe']['yaw_rms_rad'],
                    metrics['completion']['ground_truth_path_ratio'],
                ))
                evidence_ok = all((
                    valid, realized, execution_ok, identity_ok, map_ok,
                    schema_outcome_ok, terminal_consistent, resources_ok,
                    artifacts_ok, fault_ok, finite_metrics))
                status = (
                    'PASS' if evidence_ok and completed
                    else 'FAIL' if evidence_ok else 'INVALID')
                if status in ('PASS', 'FAIL'):
                    ate_values.append(metrics['ate']['translation_rms_m'])
                    ate_yaw_values.append(metrics['ate']['yaw_rms_rad'])
                    rpe_values.append(metrics['rpe']['translation_rms_m'])
                    rpe_yaw_values.append(metrics['rpe']['yaw_rms_rad'])
                    completion_values.append(
                        metrics['completion']['ground_truth_path_ratio'])
                    cpu_values.append(
                        backend_resources['cpu_mean_pct_one_core'])
                    rss_values.append(backend_resources['rss_peak_mb'])
                rows.append({
                    'seed': seed, 'status': status,
                    'completion': metrics['completion'],
                    'validity': metrics['validity'],
                    'realized_fault_passed': realized,
                    'execution_ok': execution_ok,
                    'map_artifacts_ok': map_ok,
                    'schema_outcome_ok': schema_outcome_ok,
                    'terminal_consistent': terminal_consistent,
                    'resources_ok': resources_ok,
                    'identity_ok': identity_ok,
                    'artifacts_ok': artifacts_ok,
                    'fault_artifact_ok': fault_ok,
                    'finite_metrics': finite_metrics,
                    'map_pixel_classes': pgm_classes,
                    'fault_realization': stats,
                    'run_dir': str(run_dir.resolve()),
                })
            except (OSError, KeyError, TypeError, ValueError,
                    json.JSONDecodeError) as error:
                rows.append({'seed': seed, 'status': 'INVALID',
                             'error': f'{type(error).__name__}: {error}',
                             'run_dir': str(run_dir.resolve())})
        pass_count = sum(row['status'] == 'PASS' for row in rows)
        fail_count = sum(row['status'] == 'FAIL' for row in rows)
        invalid_count = sum(row['status'] == 'INVALID' for row in rows)
        evidence_valid_count = pass_count + fail_count
        condition_status = (
            'INCOMPLETE' if evidence_valid_count != 5
            else 'FAIL' if fail_count else 'PASS')
        if condition_status == 'INCOMPLETE':
            output['overall'] = 'INCOMPLETE'
        elif condition_status == 'FAIL' and output['overall'] == 'PASS':
            output['overall'] = 'FAIL'
        output['conditions'][condition['label']] = {
            'status': condition_status,
            'valid_seed_count': evidence_valid_count,
            'required_valid_seed_count': 5,
            'pass_count': pass_count,
            'fail_count': fail_count,
            'invalid_count': invalid_count,
            'completion_rate': pass_count / 5.0,
            'runs': rows,
            'ate_translation_rms_m': _distribution(ate_values),
            'ate_yaw_rms_rad': _distribution(ate_yaw_values),
            'rpe_translation_rms_m': _distribution(rpe_values),
            'rpe_yaw_rms_rad': _distribution(rpe_yaw_values),
            'ground_truth_path_ratio': _distribution(completion_values),
            'backend_cpu_mean_pct_one_core': _distribution(cpu_values),
            'backend_rss_peak_mb': _distribution(rss_values),
        }
    return output


def main(argv: list[str] | None = None) -> int:
    """Build the frozen matrix and optionally execute and aggregate it."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensor-profile', type=Path, required=True)
    parser.add_argument('--base-urdf', type=Path, required=True)
    parser.add_argument('--base-bridge', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--out-root', type=Path, required=True)
    parser.add_argument('--corridor-distance-m', type=float, default=14.0)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--only', action='append', default=[])
    args = parser.parse_args(argv)
    for path in (args.sensor_profile, args.base_urdf,
                 args.base_bridge, args.world):
        if not path.is_file():
            parser.error(f'input does not exist: {path}')
    all_conditions = default_conditions(args.sensor_profile)
    validate_contract(all_conditions, SEEDS, args.sensor_profile)
    conditions = all_conditions
    if args.only:
        selected = set(args.only)
        conditions = [condition for condition in conditions
                      if condition['label'] in selected]
        if not conditions:
            parser.error('--only did not match a condition')
    generated = args.out_root / 'generated'
    bridge = generated / 'bridge_raw.yaml'
    write_raw_bridge(args.base_bridge, bridge)
    commands = []
    generated_profiles = {}
    generated_faults = {}
    for condition in conditions:
        urdf = generated / f'{condition["label"]}.urdf'
        write_urdf_variant(
            args.base_urdf, urdf, condition['lidar_update_rate_hz'])
        fault = generated / f'{condition["label"]}.fault.json'
        _write_reproducible(fault, json.dumps({
            'schema_version': 1,
            'label': condition['label'],
            'changed_factor': condition['changed_factor'],
            'values': condition['fault_values'],
            'provenance': condition['provenance'],
        }, ensure_ascii=False, indent=2))
        generated_profiles[condition['label']] = {
            'path': str(urdf.resolve()), 'sha256': _sha256(urdf)}
        generated_faults[condition['label']] = {
            'path': str(fault.resolve()), 'sha256': _sha256(fault)}
        for seed in SEEDS:
            commands.append([
                sys.executable,
                str(Path(__file__).with_name('run_sim_slam_experiment.py')),
                '--backend', 'cartographer',
                '--profile-urdf', str(urdf.resolve()),
                '--profile-label', condition['label'],
                '--run-id',
                f'{condition["label"]}__cartographer__seed_{seed}',
                '--bridge-config', str(bridge.resolve()),
                '--fault-profile', str(fault.resolve()),
                '--world', str(args.world.resolve()),
                '--out-root', str(args.out_root.resolve()),
                '--seed', str(seed), '--route', 'corridor',
                '--spawn-x', '-8.0', '--spawn-y', '0.0',
                '--corridor-distance-m', str(args.corridor_distance_m),
            ])
    repository = Path(__file__).resolve().parents[2]
    workspace_dirty, workspace_diff_sha256 = _workspace_fingerprint(repository)
    cartographer_config = (
        repository / 'jdamr_cube_cartographer' / 'config'
        / 'jdamr_cube_2d_real.lua')
    matrix_manifest = {
        'schema_version': 1,
        'backend': 'cartographer',
        'seeds': list(SEEDS),
        'fixed': {
            'world': {'path': str(args.world.resolve()),
                      'sha256': _sha256(args.world)},
            'route': 'corridor',
            'spawn_xy_m': [-8.0, 0.0],
            'corridor_distance_m': args.corridor_distance_m,
            'cartographer_config': {
                'path': str(cartographer_config.resolve()),
                'sha256': _sha256(cartographer_config),
            },
            'bridge_config': {
                'path': str(bridge.resolve()), 'sha256': _sha256(bridge)},
            'software': {
                'git_sha': _git_commit(repository),
                'workspace_dirty': workspace_dirty,
                'workspace_diff_sha256': workspace_diff_sha256,
            },
        },
        'sensor_profile': {
            'path': str(args.sensor_profile.resolve()),
            'sha256': _sha256(args.sensor_profile),
        },
        'conditions': conditions,
        'generated_profiles': generated_profiles,
        'generated_faults': generated_faults,
        'commands': commands,
    }
    args.out_root.mkdir(parents=True, exist_ok=True)
    manifest_path = args.out_root / 'matrix_manifest.json'
    _write_reproducible(
        manifest_path,
        json.dumps(matrix_manifest, ensure_ascii=False, indent=2))
    if args.execute:
        for command in commands:
            result = subprocess.run(command, check=False)
            if result.returncode != 0:
                print(json.dumps({'failed_command': command,
                                  'returncode': result.returncode}))
        summary = aggregate(matrix_manifest, args.out_root)
        (args.out_root / 'matrix_summary.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding='utf-8')
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0 if summary['overall'] == 'PASS' else 2
    print(json.dumps({
        'status': 'dry-run', 'runs': len(commands),
        'manifest': str(manifest_path.resolve()),
    }, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
