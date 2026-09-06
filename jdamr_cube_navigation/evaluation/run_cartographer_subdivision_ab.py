#!/usr/bin/env python3
"""Generate and validate Cartographer laser-scan subdivision inputs."""

from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import signal
import statistics
import subprocess
from typing import Any, Iterable

from compare_slam_runs import (
    _analysis_window,
    analyse,
    common_reference_deviations,
    estimated_trajectory,
    read_amcl,
    read_map_to_odom,
    read_odometry,
)
from inspect_mcap import inspect as inspect_mcap

import yaml


SUBDIVISIONS = (1, 2, 4)
SOURCE_BAG_SHA256 = (
    '65105177e43eca6153d2543f3cf8e9cc9f7c62b77f258975fdcdbd10b5e2e16a')
SETTING_NAME = 'num_subdivisions_per_laser_scan'
SETTING_PATTERN = re.compile(
    rf'^(?P<prefix>\s*{SETTING_NAME}\s*=\s*)(?P<value>\d+)'
    r'(?P<suffix>\s*,?\s*(?:--.*)?)$',
    re.MULTILINE,
)
FATAL_CARTOGRAPHER_PATTERNS = (
    re.compile(r'num_subdivisions_per_laser_scan.*(?:ignored|invalid)', re.I),
    re.compile(r'(?:scan_time|time_increment).*(?:zero|invalid|missing)', re.I),
    re.compile(r'dropped.*laser scan', re.I),
    re.compile(r'check failed.*(?:subdivision|laser|range data)', re.I),
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _write_reproducible(path: Path, content: str) -> None:
    """Write a generated artifact once or accept identical content."""
    if path.exists():
        if path.read_text(encoding='utf-8') != content:
            raise ValueError(f'generated artifact already differs: {path}')
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding='utf-8')


def _record(path: Path) -> dict[str, Any]:
    return {
        'path': str(path.resolve()),
        'sha256': _sha256(path),
        'bytes': path.stat().st_size,
    }


def _pointer(document: Any, pointer: str) -> Any:
    value = document
    for token in pointer.removeprefix('/').split('/'):
        value = value[int(token)] if isinstance(value, list) else value[token]
    return value


def write_subdivision_config(
        base_config: Path, output: Path, subdivision: int,
        sensor_profile: Path) -> dict[str, Any]:
    """Generate one config and prove that only the subdivision changed."""
    if subdivision not in SUBDIVISIONS:
        raise ValueError(f'unsupported subdivision: {subdivision}')
    source_text = base_config.read_text(encoding='utf-8')
    matches = list(SETTING_PATTERN.finditer(source_text))
    if len(matches) != 1:
        raise ValueError(
            f'expected one active {SETTING_NAME} assignment, found '
            f'{len(matches)}')
    original_value = int(matches[0].group('value'))
    if original_value != 1:
        raise ValueError('production subdivision must be exactly one')

    def replace(match: re.Match[str], value: int) -> str:
        return f'{match.group("prefix")}{value}{match.group("suffix")}'

    generated_text, replacement_count = SETTING_PATTERN.subn(
        lambda match: replace(match, subdivision), source_text)
    if replacement_count != 1:
        raise ValueError('subdivision replacement count changed unexpectedly')
    restored_text, restore_count = SETTING_PATTERN.subn(
        lambda match: replace(match, original_value), generated_text)
    if restore_count != 1 or restored_text != source_text:
        raise ValueError('generated config changed more than one setting')
    _write_reproducible(output, generated_text)

    profile = json.loads(sensor_profile.read_text(encoding='utf-8'))
    measurement_pointers = {
        'scan_time_median_s': '/scan/declared_scan_time_s/p50',
        'scan_time_p99_s': '/scan/declared_scan_time_s/p99',
        'time_increment_median_s': '/scan/declared_time_increment_s/p50',
    }
    provenance = {
        'schema_version': 1,
        'setting': SETTING_NAME,
        'original_value': original_value,
        'generated_value': subdivision,
        'only_setting_changed': restored_text == source_text,
        'source_config': _record(base_config),
        'generated_config': _record(output),
        'measurement_source': {
            **_record(sensor_profile),
            'values': {
                name: {
                    'json_pointer': pointer,
                    'value': _pointer(profile, pointer),
                }
                for name, pointer in measurement_pointers.items()
            },
        },
        'references': [
            {
                'title': 'Cartographer ROS configuration reference',
                'url': ('https://google-cartographer-ros.readthedocs.io/'
                        'en/latest/configuration.html'),
                'claim': 'Subdivision supports scan unwarping while moving.',
            },
            {
                'title': 'Li et al. 2016 mobile 2D LiDAR distortion',
                'url': 'https://doi.org/10.5194/isprsannals-III-4-119-2016',
                'claim': 'Platform motion can distort a scan during capture.',
            },
        ],
    }
    _write_reproducible(
        output.with_suffix('.manifest.json'),
        json.dumps(provenance, ensure_ascii=False, indent=2),
    )
    return provenance


def validate_scan_timing_samples(
        samples: Iterable[tuple[float, float, int]],
        *, relative_tolerance: float = 1e-5) -> dict[str, Any]:
    """Reject scans that cannot support intra-scan time subdivision."""
    rows = list(samples)
    if not rows:
        return {'valid': False, 'reason': 'no LaserScan frames', 'frames': 0}
    invalid_frames = 0
    inconsistent_frames = 0
    for scan_time_s, time_increment_s, beam_count in rows:
        finite_positive = all((
            math.isfinite(scan_time_s), scan_time_s > 0.0,
            math.isfinite(time_increment_s), time_increment_s > 0.0,
            beam_count > 1,
        ))
        if not finite_positive:
            invalid_frames += 1
            continue
        represented_scan_time_s = time_increment_s * beam_count
        relative_error = abs(
            represented_scan_time_s - scan_time_s) / scan_time_s
        if relative_error > relative_tolerance:
            inconsistent_frames += 1
    valid = invalid_frames == 0 and inconsistent_frames == 0
    reason = (
        'scan timing supports intra-scan subdivision'
        if valid else 'nonfinite, zero, or frame-inconsistent scan timing')
    return {
        'valid': valid,
        'reason': reason,
        'frames': len(rows),
        'invalid_frames': invalid_frames,
        'inconsistent_frames': inconsistent_frames,
        'relative_tolerance': relative_tolerance,
    }


def fatal_cartographer_log_lines(log_text: str) -> list[str]:
    """Return warnings that invalidate a subdivision experiment."""
    return [
        line for line in log_text.splitlines()
        if any(pattern.search(line) for pattern in FATAL_CARTOGRAPHER_PATTERNS)
    ]


def inspect_source_bag(
        source_mcap: Path,
        analysis_window: dict[str, int]) -> dict[str, Any]:
    """Hash the frozen source and validate timing on every LaserScan."""
    from mcap_ros2.reader import read_ros2_messages

    source_sha256 = _sha256(source_mcap)
    if source_sha256 != SOURCE_BAG_SHA256:
        raise ValueError('source bag SHA-256 does not match G008')
    samples = []
    scan_times_s = []
    time_increments_s = []
    beam_counts = []
    scan_timestamps_ns = []
    odometry_timestamps_ns = []
    for message in read_ros2_messages(
            str(source_mcap), topics=['/scan', '/odom']):
        if message.channel.topic == '/scan':
            scan = message.ros_msg
            scan_time_s = float(scan.scan_time)
            time_increment_s = float(scan.time_increment)
            beam_count = len(scan.ranges)
            stamp = scan.header.stamp
            scan_timestamps_ns.append(
                stamp.sec * 1_000_000_000 + stamp.nanosec)
            samples.append((scan_time_s, time_increment_s, beam_count))
            scan_times_s.append(scan_time_s)
            time_increments_s.append(time_increment_s)
            beam_counts.append(beam_count)
        else:
            stamp = message.ros_msg.header.stamp
            odometry_timestamps_ns.append(
                stamp.sec * 1_000_000_000 + stamp.nanosec)
    timing = validate_scan_timing_samples(samples)
    if not timing['valid']:
        raise ValueError(timing['reason'])
    timestamp_digest = hashlib.sha256()
    window_timestamp_digest = hashlib.sha256()
    window_scan_count = 0
    for timestamp_ns in scan_timestamps_ns:
        timestamp_digest.update(f'{timestamp_ns}\n'.encode())
        if (analysis_window['start_unix_ns'] <= timestamp_ns <=
                analysis_window['end_unix_ns']):
            window_timestamp_digest.update(f'{timestamp_ns}\n'.encode())
            window_scan_count += 1
    source_integrity = inspect_mcap(source_mcap)
    return {
        **_record(source_mcap),
        'scan_timing_gate': timing,
        'scan_time_s': {
            'min': min(scan_times_s),
            'median': statistics.median(scan_times_s),
            'max': max(scan_times_s),
        },
        'time_increment_s': {
            'min': min(time_increments_s),
            'median': statistics.median(time_increments_s),
            'max': max(time_increments_s),
        },
        'beam_count': {
            'min': min(beam_counts),
            'median': statistics.median(beam_counts),
            'max': max(beam_counts),
        },
        'scan_timestamp_sha256': timestamp_digest.hexdigest(),
        'scan_timestamp_first_unix_ns': scan_timestamps_ns[0],
        'scan_timestamp_last_unix_ns': scan_timestamps_ns[-1],
        'analysis_window': analysis_window,
        'analysis_window_scan_count': window_scan_count,
        'analysis_window_scan_timestamp_sha256': (
            window_timestamp_digest.hexdigest()),
        'odometry_count': len(odometry_timestamps_ns),
        'odometry_timestamp_first_unix_ns': odometry_timestamps_ns[0],
        'odometry_timestamp_last_unix_ns': odometry_timestamps_ns[-1],
        'mcap_integrity': source_integrity['integrity'],
    }


def _one_mcap(directory: Path) -> Path:
    matches = sorted(directory.glob('*.mcap'))
    if len(matches) != 1:
        raise ValueError(f'expected one result MCAP, found {len(matches)}')
    return matches[0]


def _resource_summary(path: Path) -> dict[str, Any]:
    samples = [json.loads(line) for line in path.read_text().splitlines()
               if line.strip()]
    if not samples:
        raise ValueError('resource sampler produced no rows')
    cpu_values = [row['cpu_pct_one_core'] for row in samples
                  if row['cpu_pct_one_core'] is not None]
    rss_values_mb = [row['rss_mb'] for row in samples]
    if not all(math.isfinite(value) and value >= 0.0
               for value in (*cpu_values, *rss_values_mb)):
        raise ValueError('resource samples contain invalid values')
    if not any(row['process_count'] > 0 for row in samples):
        raise ValueError('resource sampler never observed the backend')
    return {
        'samples': len(samples),
        'cpu_mean_pct_one_core': statistics.fmean(cpu_values),
        'cpu_max_pct_one_core': max(cpu_values),
        'rss_peak_mb': max(rss_values_mb),
        'semantics': (
            'procfs CPU-time delta over each common sampling interval; '
            'RSS is the process-group sum'),
    }


def _validate_pgm(
        map_yaml: Path, map_pgm: Path, *, require_all_classes: bool = True,
) -> dict[str, Any]:
    """Validate YAML linkage and binary PGM dimensions and pixel classes."""
    metadata = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    declared_image = Path(metadata['image'])
    if (map_yaml.parent / declared_image).resolve() != map_pgm.resolve():
        raise ValueError('map YAML does not reference the expected PGM')
    with map_pgm.open('rb') as stream:
        magic = stream.readline().strip()
        line = stream.readline()
        while line.startswith(b'#'):
            line = stream.readline()
        width, height = (int(value) for value in line.split())
        max_value = int(stream.readline())
        pixels = stream.read()
    if magic != b'P5' or max_value != 255 or len(pixels) != width * height:
        raise ValueError('invalid binary PGM header or dimensions')
    classes = {
        'occupied': sum(value < 65 for value in pixels),
        'unknown': sum(65 <= value <= 250 for value in pixels),
        'free': sum(value > 250 for value in pixels),
    }
    if require_all_classes and not all(classes.values()):
        raise ValueError('PGM lacks occupied, unknown, or free pixels')
    if not require_all_classes and (
            len(set(pixels)) < 2 or not (classes['occupied'] or classes['free'])):
        raise ValueError('smoke PGM lacks a nonuniform observed class')
    return {
        'width': width, 'height': height, 'max_value': max_value,
        'pixel_count': len(pixels), 'pixel_classes': classes,
        'resolution_m_per_cell': metadata['resolution'],
        'yaml_linkage': 'direct',
        'declared_image_path': str(declared_image),
        'required_distribution': (
            'occupied_free_unknown' if require_all_classes
            else 'nonuniform_with_observed_class'),
    }


def _validate_map_relocation(
        map_yaml: Path, map_pgm: Path) -> dict[str, Any] | None:
    """Verify optional durable-map relocation provenance and parent hashes."""
    raw_yaml = map_yaml.with_suffix('.raw.yaml')
    provenance_path = map_yaml.with_suffix('.relocation.json')
    if not raw_yaml.exists() and not provenance_path.exists():
        return None
    if not raw_yaml.is_file() or not provenance_path.is_file():
        raise ValueError('map relocation evidence is incomplete')
    provenance = json.loads(provenance_path.read_text(encoding='utf-8'))
    checks = (
        provenance.get('transform_version') == 'map_yaml_image_relocation_v1',
        provenance.get('image_only_change_verified') is True,
        provenance.get('basename_equal') is True,
        provenance.get('pgm_sha256_equal') is True,
        provenance['parent_raw_yaml']['sha256'] == _sha256(raw_yaml),
        provenance['derived_yaml']['sha256'] == _sha256(map_yaml),
        provenance['colocated_pgm']['sha256'] == _sha256(map_pgm),
        provenance['new_relative_target'] == map_pgm.name,
    )
    if not all(checks):
        raise ValueError('map relocation provenance does not match artifacts')
    return {
        'provenance': _record(provenance_path),
        'parent_raw_yaml': _record(raw_yaml),
        'verified': True,
    }


def _timestamp_span_gate(
        replay_guard: dict[str, Any], source_evidence: dict[str, Any],
) -> dict[str, Any]:
    """Compare integer stamps, migrating legacy float evidence by its ULP."""
    pairs = (
        ('scan_first', 'scan_timestamp_first_unix_ns'),
        ('scan_last', 'scan_timestamp_last_unix_ns'),
        ('odometry_received_first', 'odometry_timestamp_first_unix_ns'),
        ('odometry_received_last', 'odometry_timestamp_last_unix_ns'),
    )
    comparisons = {}
    for guard_prefix, source_key in pairs:
        integer_key = f'{guard_prefix}_unix_ns'
        expected_ns = source_evidence[source_key]
        if integer_key in replay_guard:
            observed_ns = replay_guard[integer_key]
            absolute_error_ns = abs(observed_ns - expected_ns)
            maximum_error_ns = 0.0
            evidence_kind = 'integer_nanoseconds'
            matches = absolute_error_ns == 0
        else:
            seconds = replay_guard[f'{guard_prefix}_s']
            observed_ns = round(seconds * 1e9)
            error = abs(
                Fraction.from_float(seconds) * 1_000_000_000 - expected_ns)
            maximum_error = (
                Fraction.from_float(math.ulp(seconds)) * 1_000_000_000 / 2)
            absolute_error_ns = float(error)
            maximum_error_ns = float(maximum_error)
            evidence_kind = 'legacy_float_seconds_half_ulp_verified'
            matches = error <= maximum_error
        comparisons[guard_prefix] = {
            'evidence_kind': evidence_kind,
            'observed_unix_ns': observed_ns,
            'expected_unix_ns': expected_ns,
            'absolute_error_ns': absolute_error_ns,
            'maximum_error_ns': maximum_error_ns,
            'matches': matches,
        }
    exact_ns_available = all(
        item['evidence_kind'] == 'integer_nanoseconds'
        for item in comparisons.values())
    all_match = all(item['matches'] for item in comparisons.values())
    return {
        'all_match': all_match,
        'exact_ns_available': exact_ns_available,
        'legacy_float_timestamp_ulp_verified': (
            all_match and not exact_ns_available),
        'comparisons': comparisons,
    }


def collect_replay(
        output_root: Path, source_mcap: Path, label: str,
        config: Path, returncode: int, *, smoke: bool,
        analysis_window: dict[str, int],
        source_evidence: dict[str, Any]) -> dict[str, Any]:
    """Validate one replay and return immutable evidence metadata."""
    run_id = f'{source_mcap.parent.name}__{label}'
    result_dir = output_root / f'{run_id}_result'
    result_mcap = _one_mcap(result_dir)
    artifacts = {
        'result_mcap': _record(result_mcap),
        'result_metadata': _record(result_dir / 'metadata.yaml'),
        'pbstream': _record(output_root / f'{run_id}_map.pbstream'),
        'map_yaml': _record(output_root / f'{run_id}_map.yaml'),
        'map_pgm': _record(output_root / f'{run_id}_map.pgm'),
        'resources': _record(output_root / f'{run_id}_resources.jsonl'),
        'teardown': _record(output_root / f'{run_id}_teardown.json'),
        'replay_guard': _record(
            output_root / f'{run_id}_replay_guard.json'),
        'finalizer': _record(output_root / f'{run_id}_finalizer.json'),
        'log': _record(output_root / f'{run_id}.log'),
    }
    map_yaml_path = Path(artifacts['map_yaml']['path'])
    map_pgm_path = Path(artifacts['map_pgm']['path'])
    map_relocation = _validate_map_relocation(map_yaml_path, map_pgm_path)
    if map_relocation is not None:
        artifacts['map_raw_yaml'] = map_relocation['parent_raw_yaml']
        artifacts['map_relocation'] = map_relocation['provenance']
    log_text = Path(artifacts['log']['path']).read_text(
        encoding='utf-8', errors='replace')
    fatal_lines = fatal_cartographer_log_lines(log_text)
    teardown = json.loads(Path(artifacts['teardown']['path']).read_text())
    resources = _resource_summary(Path(artifacts['resources']['path']))
    result_mcap_integrity = inspect_mcap(result_mcap)
    integrity = result_mcap_integrity['integrity']
    result_mcap_ok = all((
        integrity['validate_crcs'] == 'PASS',
        integrity['chunk_count'] > 0,
        integrity['chunks_with_crc'] == integrity['chunk_count'],
        integrity['data_section_crc'] != 0,
        integrity['summary_crc'] != 0,
        integrity['chunk_index_count'] > 0,
        integrity['message_index_count'] > 0,
    ))
    replay_guard = json.loads(
        Path(artifacts['replay_guard']['path']).read_text())
    map_validation = _validate_pgm(
        map_yaml_path, map_pgm_path, require_all_classes=not smoke)
    stage_markers = (
        'evidence_stage=finish_trajectory_ok',
        'evidence_stage=shutdown_final_optimization_ok',
        'evidence_stage=pbstream_to_ros_map_ok',
    )
    stage_indexes = [log_text.find(marker) for marker in stage_markers]
    service_order_ok = (
        all(index >= 0 for index in stage_indexes)
        and stage_indexes == sorted(stage_indexes))
    metrics = None
    trajectory_complete = None
    if not smoke:
        metrics = analyse(
            result_mcap, source_mcap,
            Path(artifacts['map_yaml']['path']),
            start_unix_ns=analysis_window['start_unix_ns'],
            end_unix_ns=analysis_window['end_unix_ns'])
        trajectory_complete = (
            metrics.get('trajectory_samples') == metrics.get(
                'odometry_samples')
            and metrics.get('map_to_odom_updates', 0) > 0)
        route_window_covered = metrics.get('map_to_odom_window_covered', False)
    else:
        route_window_covered = None
    replay_counts_ok = all((
        replay_guard.get('scan_forwarded', 0) > 0,
        replay_guard.get('odometry_forwarded', 0) > 0,
        replay_guard.get('recorded_map_to_odom_dropped', 0) > 0,
    ))
    timestamp_span = None
    if not smoke:
        timestamp_span = _timestamp_span_gate(replay_guard, source_evidence)
        replay_counts_ok = replay_counts_ok and all((
            replay_guard['scan_forwarded'] ==
            source_evidence['scan_timing_gate']['frames'],
            timestamp_span['all_match'],
            replay_guard['odometry_forwarded'] +
            replay_guard['odometry_dropped_late'] ==
            source_evidence['odometry_count'],
        ))
    evidence_valid = all((
        teardown.get('remaining_process_groups') == [],
        teardown.get('identity_survivors', []) == [],
        all(item['bytes'] > 0 for item in artifacts.values()),
        trajectory_complete is not False,
        route_window_covered is not False,
        replay_counts_ok,
        result_mcap_ok,
    ))
    backend_pass = returncode == 0 and not fatal_lines and service_order_ok
    status = (
        'PASS' if evidence_valid and backend_pass
        else 'FAIL' if evidence_valid else 'INVALID')
    return {
        'label': label,
        'subdivision': int(label.rsplit('_', 1)[1]),
        'status': status,
        'validity_status': 'VALID' if evidence_valid else 'INVALID',
        'performance_status': (
            'PASS' if backend_pass else 'FAIL' if evidence_valid else None),
        'returncode': returncode,
        'source_bag': _record(source_mcap),
        'config': _record(config),
        'artifacts': artifacts,
        'fatal_log_lines': fatal_lines,
        'teardown': teardown,
        'resources': resources,
        'result_mcap_integrity': result_mcap_integrity,
        'replay_guard': replay_guard,
        'replay_counts_and_span_ok': replay_counts_ok,
        'timestamp_span_evidence': timestamp_span,
        'map_validation': map_validation,
        'map_relocation': map_relocation,
        'service_order_ok': service_order_ok,
        'route_window_covered': route_window_covered,
        'trajectory_complete': trajectory_complete,
        'reference_metrics': metrics,
        'reference_contract': {
            'reference_kind': 'saved_map_amcl_reference',
            'ground_truth_available': False,
            'ate_rpe_reported': False,
            'absolute_accuracy_claim': False,
            'qualification_claim': False,
        },
    }


def invalid_replay(
        source_mcap: Path, label: str, config: Path,
        returncode: int, error: Exception) -> dict[str, Any]:
    """Preserve an infrastructure failure without aborting later variants."""
    return {
        'label': label,
        'subdivision': int(label.rsplit('_', 1)[1]),
        'status': 'INVALID',
        'validity_status': 'INVALID',
        'performance_status': None,
        'returncode': returncode,
        'source_bag': _record(source_mcap),
        'config': _record(config),
        'error': f'{type(error).__name__}: {error}',
    }


def _process_groups_still_present(process_groups: list[int]) -> list[int]:
    """Return experiment process groups that still have a procfs member."""
    present = set()
    for stat_path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = stat_path.read_text().split()
            process_group = int(fields[4])
        except (OSError, IndexError, ValueError):
            continue
        if process_group in process_groups:
            present.add(process_group)
    return sorted(present)


def collect_finalization_budget_failure(
        output_root: Path, source_mcap: Path, label: str, config: Path,
        *, analysis_window: dict[str, int],
        source_evidence: dict[str, Any],
) -> dict[str, Any]:
    """Preserve a complete replay that exceeded its fixed finalization budget."""
    run_id = f'{source_mcap.parent.name}__{label}'
    result_dir = output_root / f'{run_id}_result'
    paths = {
        'result_mcap': _one_mcap(result_dir),
        'result_metadata': result_dir / 'metadata.yaml',
        'resources': output_root / f'{run_id}_resources.jsonl',
        'teardown': output_root / f'{run_id}_teardown.json',
        'replay_guard': output_root / f'{run_id}_replay_guard.json',
        'finalizer': output_root / f'{run_id}_finalizer.json',
        'log': output_root / f'{run_id}.log',
    }
    artifacts = {name: _record(path) for name, path in paths.items()}
    log_text = paths['log'].read_text(encoding='utf-8', errors='replace')
    required_markers = (
        'evidence_stage=finish_trajectory_ok',
        'Running final trajectory optimization',
        'Cartographer graceful shutdown 제한 시간 초과',
    )
    finalizer = json.loads(paths['finalizer'].read_text())
    replay_guard = json.loads(paths['replay_guard'].read_text())
    teardown = json.loads(paths['teardown'].read_text())
    timestamp_span = _timestamp_span_gate(replay_guard, source_evidence)
    replay_counts_ok = all((
        replay_guard['scan_forwarded'] ==
        source_evidence['scan_timing_gate']['frames'],
        replay_guard['odometry_forwarded'] +
        replay_guard['odometry_dropped_late'] ==
        source_evidence['odometry_count'],
        timestamp_span['all_match'],
    ))
    result_integrity = inspect_mcap(paths['result_mcap'])
    integrity = result_integrity['integrity']
    mcap_ok = all((
        integrity['validate_crcs'] == 'PASS',
        integrity['chunk_count'] > 0,
        integrity['chunks_with_crc'] == integrity['chunk_count'],
        integrity['data_section_crc'] != 0,
        integrity['summary_crc'] != 0,
        integrity['chunk_index_count'] > 0,
        integrity['message_index_count'] > 0,
    ))
    metrics = analyse(
        paths['result_mcap'], source_mcap, None,
        start_unix_ns=analysis_window['start_unix_ns'],
        end_unix_ns=analysis_window['end_unix_ns'])
    trajectory_complete = (
        metrics.get('trajectory_samples') == metrics.get('odometry_samples')
        and metrics.get('map_to_odom_updates', 0) > 0
        and metrics.get('map_to_odom_window_covered', False))
    original_groups = teardown.get('remaining_process_groups', [])
    current_groups = _process_groups_still_present(original_groups)
    evidence_valid = all((
        all(marker in log_text for marker in required_markers),
        finalizer.get('status') == 'PASS',
        replay_counts_ok,
        mcap_ok,
        trajectory_complete,
        not fatal_cartographer_log_lines(log_text),
        current_groups == [],
        teardown.get('identity_survivors', []) == [],
    ))
    if not evidence_valid:
        raise ValueError('finalization budget failure evidence is incomplete')
    return {
        'label': label,
        'subdivision': int(label.rsplit('_', 1)[1]),
        'status': 'FAIL',
        'validity_status': 'VALID',
        'performance_status': 'FAIL',
        'failure_kind': 'FAIL_FINALIZATION_BUDGET',
        'returncode': 9,
        'source_bag': _record(source_mcap),
        'config': _record(config),
        'artifacts': artifacts,
        'fatal_log_lines': [],
        'teardown': {
            'original_remaining_process_groups': original_groups,
            'current_remaining_process_groups': current_groups,
            'cleanup_defect_fixed_after_run': True,
        },
        'resources': _resource_summary(paths['resources']),
        'result_mcap_integrity': result_integrity,
        'replay_guard': replay_guard,
        'replay_counts_and_span_ok': replay_counts_ok,
        'timestamp_span_evidence': timestamp_span,
        'map_validation': None,
        'service_order_ok': False,
        'route_window_covered': metrics['map_to_odom_window_covered'],
        'trajectory_complete': trajectory_complete,
        'reference_metrics': metrics,
        'reference_contract': {
            'reference_kind': 'saved_map_amcl_reference',
            'ground_truth_available': False,
            'ate_rpe_reported': False,
            'absolute_accuracy_claim': False,
            'qualification_claim': False,
        },
        'failure_boundary': (
            'Replay and pre-shutdown TF evidence completed, but the fixed '
            '90-second finalization budget expired before pbstream/map output.'),
    }


def validate_run_sidecar(document: dict[str, Any]) -> None:
    """Validate the closed run manifest before accepting its hash."""
    required = {
        'schema_version', 'run_id', 'status', 'source_bag', 'config',
        'artifacts', 'resources', 'replay_guard', 'reference_contract'}
    missing = required - set(document)
    if missing:
        raise ValueError(f'run sidecar missing keys: {sorted(missing)}')
    if document['schema_version'] != 1:
        raise ValueError('run sidecar schema version is unsupported')
    for item in document['artifacts'].values():
        path = Path(item['path'])
        if not path.is_file() or _sha256(path) != item['sha256']:
            raise ValueError('run sidecar artifact hash mismatch')


def write_run_sidecar(output_root: Path, run: dict[str, Any]) -> dict[str, Any]:
    """Write and revalidate one immutable replay evidence sidecar."""
    if run['status'] == 'INVALID':
        return run
    run_id = f"{Path(run['source_bag']['path']).parent.name}__{run['label']}"
    document = {'schema_version': 1, 'run_id': run_id, **run}
    validate_run_sidecar(document)
    path = output_root / f'{run_id}_manifest.json'
    _write_reproducible(
        path, json.dumps(document, ensure_ascii=False, indent=2))
    loaded = json.loads(path.read_text(encoding='utf-8'))
    validate_run_sidecar(loaded)
    run['run_sidecar'] = _record(path)
    run['sidecar_schema_and_hash_chain_ok'] = True
    return run


def posthoc_recollection_provenance(
        run: dict[str, Any], repository: Path, artifact_root: Path,
) -> dict[str, Any]:
    """Record why a completed raw replay was analysed without rerunning it."""
    status = subprocess.run(
        ['git', '-C', str(repository), 'status', '--porcelain=v1'],
        capture_output=True, check=True).stdout
    diff = subprocess.run(
        ['git', '-C', str(repository), 'diff', '--binary', 'HEAD'],
        capture_output=True, check=True).stdout
    workspace_digest = hashlib.sha256(status + b'\0' + diff).hexdigest()
    return {
        'collection_recomputed_after_parser_fix': True,
        'rerun_performed': False,
        'artifact_storage': {
            'kind': 'immutable_external_reference',
            'root': str(artifact_root.resolve()),
        },
        'reuse_preconditions': {
            'replay_outcome_acceptable': (
                run['returncode'] == 0
                or run.get('failure_kind') == 'FAIL_FINALIZATION_BUDGET'),
            'all_semantic_gates_passed': run['status'] in ('PASS', 'FAIL'),
            'teardown_empty': (
                run['teardown'].get('remaining_process_groups',
                                    run['teardown'].get(
                                        'current_remaining_process_groups'))
                == []),
        },
        'evaluator_files': {
            'runner': _record(Path(__file__)),
            'comparison': _record(
                Path(__file__).with_name('compare_slam_runs.py')),
        },
        'workspace': {
            'head': subprocess.run(
                ['git', '-C', str(repository), 'rev-parse', 'HEAD'],
                capture_output=True, text=True, check=True).stdout.strip(),
            'status_and_diff_sha256': workspace_digest,
        },
        'raw_artifact_hashes': {
            name: artifact['sha256']
            for name, artifact in run['artifacts'].items()
        },
    }


def run_with_hard_timeout(
        command: list[str], timeout_s: float) -> tuple[int, str | None]:
    """Run one replay in its own group and kill only that group on timeout."""
    process = subprocess.Popen(command, start_new_session=True)
    try:
        return process.wait(timeout=timeout_s), None
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)
        return 124, f'per-run hard timeout after {timeout_s:g}s'


def choose_candidate(runs: list[dict[str, Any]]) -> dict[str, Any]:
    """Promote only strict quality gains without CPU or RSS increase."""
    by_value = {run['subdivision']: run for run in runs}
    baseline = by_value[1]
    decisions = {}
    eligible = []
    for subdivision in (2, 4):
        candidate = by_value[subdivision]
        valid = baseline['status'] == candidate['status'] == 'PASS'
        if valid:
            base_metrics = baseline['reference_metrics']
            candidate_metrics = candidate['reference_metrics']
            quality_better = all((
                candidate_metrics['start_to_end_m_raw'] <
                base_metrics['start_to_end_m_raw'],
                candidate_metrics['deviation_from_amcl']['rms_m_raw'] <
                base_metrics['deviation_from_amcl']['rms_m_raw'],
            ))
            resources_no_higher = all((
                candidate['resources'][
                    'cpu_mean_pct_one_core'] <=
                baseline['resources'][
                    'cpu_mean_pct_one_core'],
                candidate['resources']['rss_peak_mb'] <=
                baseline['resources']['rss_peak_mb'],
            ))
        else:
            quality_better = False
            resources_no_higher = False
        decisions[str(subdivision)] = {
            'all_validity_gates_passed': valid,
            'both_reference_quality_metrics_strictly_lower': quality_better,
            'cpu_and_rss_not_higher': resources_no_higher,
            'eligible': valid and quality_better and resources_no_higher,
            'raw_comparison': {
                'start_to_end_m': {
                    'baseline': baseline['reference_metrics'][
                        'start_to_end_m_raw'],
                    'candidate': candidate['reference_metrics'][
                        'start_to_end_m_raw'],
                    'candidate_minus_baseline': (
                        candidate['reference_metrics']['start_to_end_m_raw'] -
                        baseline['reference_metrics']['start_to_end_m_raw']),
                },
                'amcl_reference_rms_m': {
                    'baseline': baseline['reference_metrics'][
                        'deviation_from_amcl']['rms_m_raw'],
                    'candidate': candidate['reference_metrics'][
                        'deviation_from_amcl']['rms_m_raw'],
                    'candidate_minus_baseline': (
                        candidate['reference_metrics']['deviation_from_amcl'][
                            'rms_m_raw'] -
                        baseline['reference_metrics']['deviation_from_amcl'][
                            'rms_m_raw']),
                },
                'cpu_mean_pct_one_core': {
                    'baseline': baseline['resources'][
                        'cpu_mean_pct_one_core'],
                    'candidate': candidate['resources'][
                        'cpu_mean_pct_one_core'],
                },
                'rss_peak_mb': {
                    'baseline': baseline['resources']['rss_peak_mb'],
                    'candidate': candidate['resources']['rss_peak_mb'],
                },
            },
        }
        if decisions[str(subdivision)]['eligible']:
            eligible.append(subdivision)
    selected = eligible[0] if len(eligible) == 1 else 1
    if len(eligible) == 2:
        for candidate, other in ((2, 4), (4, 2)):
            candidate_metrics = by_value[candidate]['reference_metrics']
            other_metrics = by_value[other]['reference_metrics']
            quality_dominates = all((
                candidate_metrics['start_to_end_m_raw'] <=
                other_metrics['start_to_end_m_raw'],
                candidate_metrics['deviation_from_amcl']['rms_m_raw'] <=
                other_metrics['deviation_from_amcl']['rms_m_raw'],
            )) and any((
                candidate_metrics['start_to_end_m_raw'] <
                other_metrics['start_to_end_m_raw'],
                candidate_metrics['deviation_from_amcl']['rms_m_raw'] <
                other_metrics['deviation_from_amcl']['rms_m_raw'],
            ))
            resource_no_worse = all((
                by_value[candidate]['resources'][
                    'cpu_mean_pct_one_core'] <=
                by_value[other]['resources'][
                    'cpu_mean_pct_one_core'],
                by_value[candidate]['resources']['rss_peak_mb'] <=
                by_value[other]['resources']['rss_peak_mb'],
            ))
            if quality_dominates and resource_no_worse:
                selected = candidate
    return {
        'production_value': 1,
        'selected_candidate': selected,
        'decisions': decisions,
        'result': (
            f'diagnostic candidate subdivision {selected}'
            if selected != 1 else 'retain production value 1'),
        'claim_boundary': (
            'Same-bag operational A/B only. AMCL is a recorded reference, '
            'not ground truth; this is not ATE/RPE, qualification, or safety '
            'evidence.'),
        'selection_uses_unrounded_raw_values': True,
    }


def aggregate_status(runs: list[dict[str, Any]]) -> dict[str, Any]:
    """Retain production when candidates fail but the baseline is valid."""
    baseline = next(
        (run for run in runs if run['subdivision'] == 1), None)
    if baseline is None or baseline['status'] != 'PASS':
        return {'overall': 'INVALID', 'final_status': 'INCONCLUSIVE'}
    if len(runs) != len(SUBDIVISIONS) or any(
            run['status'] == 'INVALID' for run in runs):
        return {'overall': 'INVALID', 'final_status': 'INCONCLUSIVE'}
    selection = choose_candidate(runs)
    selected = selection['selected_candidate']
    return {
        'overall': (
            'PASS' if all(run['status'] == 'PASS' for run in runs)
            else 'FAIL'),
        'final_status': (
            'DIAGNOSTIC_CANDIDATE' if selected != 1
            else 'RETAIN_PRODUCTION'),
        'selection': selection,
    }


def apply_common_reference_sample_set(
        runs: list[dict[str, Any]], source_mcap: Path,
        analysis_window: dict[str, int],
) -> dict[str, Any]:
    """Replace independent pairing with one identical AMCL timestamp set."""
    odometry = _analysis_window(
        read_odometry(source_mcap),
        analysis_window['start_unix_ns'], analysis_window['end_unix_ns'])
    reference = _analysis_window(
        read_amcl(source_mcap),
        analysis_window['start_unix_ns'], analysis_window['end_unix_ns'])
    trajectories = {}
    for run in runs:
        result_mcap = Path(run['artifacts']['result_mcap']['path'])
        map_to_odom = read_map_to_odom(result_mcap)
        trajectories[run['label']] = _analysis_window(
            estimated_trajectory(map_to_odom, odometry),
            analysis_window['start_unix_ns'], analysis_window['end_unix_ns'])
    common = common_reference_deviations(trajectories, reference)
    trajectory_hashes = {
        run['reference_metrics']['trajectory_timestamp_sha256']
        for run in runs}
    if len(trajectory_hashes) != 1:
        raise ValueError('trajectory timestamp sample sets differ')
    for run in runs:
        metrics = run['reference_metrics']
        metrics['independent_pairing_diagnostic'] = (
            metrics['deviation_from_amcl'])
        metrics['deviation_from_amcl'] = common['variants'][run['label']]
    return {
        'analysis_window': analysis_window,
        'trajectory_timestamp_sha256': next(iter(trajectory_hashes)),
        'amcl_reference_timestamp_sha256': common['timestamp_sha256'],
        'amcl_reference_samples': common['samples'],
        'identical_timestamp_and_sample_set': True,
    }


def cartographer_effective_contract(base_config: Path) -> dict[str, Any]:
    """Prove the inherited range-data accumulation setting is one."""
    if re.search(
            r'^\s*TRAJECTORY_BUILDER_2D\.num_accumulated_range_data\s*=',
            base_config.read_text(encoding='utf-8'), re.MULTILINE):
        raise ValueError('base config overrides num_accumulated_range_data')
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    defaults = Path(
        f'/opt/ros/{ros_distro}/share/cartographer/configuration_files/'
        'trajectory_builder_2d.lua')
    default_text = defaults.read_text(encoding='utf-8')
    match = re.search(
        r'^\s*num_accumulated_range_data\s*=\s*(\d+)\s*,',
        default_text, re.MULTILINE)
    if match is None or int(match.group(1)) != 1:
        raise ValueError('effective num_accumulated_range_data is not one')
    return {
        'num_accumulated_range_data': 1,
        'inherited_from': _record(defaults),
        'coupled_effects': [
            'subdivision',
            'Cartographer internal accumulation and update behavior',
        ],
        'causal_deskew_claim': False,
    }


def main(argv: list[str] | None = None) -> int:
    """Generate configs and optionally run isolated same-bag replays."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-config', type=Path, required=True)
    parser.add_argument('--sensor-profile', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--source-bag-dir', type=Path)
    parser.add_argument('--replay-script', type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--smoke', action='store_true')
    parser.add_argument('--smoke-duration-s', type=float, default=30.0)
    parser.add_argument('--run-timeout-s', type=float)
    parser.add_argument(
        '--resume-complete-label', action='append', default=[],
        choices=[f'subdivision_{value}' for value in SUBDIVISIONS])
    parser.add_argument('--resume-source-root', type=Path)
    parser.add_argument('--environment-audit', type=Path)
    parser.add_argument(
        '--resume-artifact-root', action='append', default=[],
        metavar='LABEL=PATH')
    args = parser.parse_args(argv)
    for path in (args.base_config, args.sensor_profile):
        if not path.is_file():
            parser.error(f'input does not exist: {path}')
    variants = {}
    for subdivision in SUBDIVISIONS:
        output = args.out_dir / f'subdivision_{subdivision}.lua'
        provenance = write_subdivision_config(
            args.base_config, output, subdivision, args.sensor_profile)
        variants[str(subdivision)] = {
            'config': _record(output),
            'provenance': provenance,
        }
    manifest: dict[str, Any] = {
        'schema_version': 1,
        'status': 'GENERATED_NOT_EXECUTED',
        'real_motion_authorized': False,
        'base_config': _record(args.base_config),
        'sensor_profile': _record(args.sensor_profile),
        'variants': variants,
        'effective_cartographer_contract': cartographer_effective_contract(
            args.base_config),
        'reference_kind': 'saved_map_amcl_reference',
        'ground_truth_available': False,
        'ate_rpe_reported': False,
        'absolute_accuracy_claim': False,
        'qualification_claim': False,
        'runtime_environment': {
            'hostname': platform.node(),
            'cpu_cores': os.cpu_count(),
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'cartographer_version': subprocess.run(
                ['dpkg-query', '-W', '-f=${Version}',
                 'ros-jazzy-cartographer-ros'],
                capture_output=True, text=True, check=False).stdout.strip(),
            'execution_order': [1, 2, 4],
            'ros_domain_id': 199,
            'replay_rate': 1.0,
        },
        'finalization_capability': {
            'run_final_optimization_service_available': False,
            'installed_api_path': (
                'finish_trajectory code 0, SIGINT graceful shutdown, '
                'node final optimization and save_state_filename, then '
                'cartographer_pbstream_to_ros_map'),
            'quality_metric_timing': 'pre_shutdown_optimization_result_tf',
            'final_trajectory_extraction_available': False,
            'claim_boundary': (
                'The installed node exposes no final-optimization service. '
                'Quality metrics use result TF recorded before shutdown; the '
                'final optimized pbstream is validated only as map evidence.'),
        },
    }
    source_mcap = None
    if args.execute:
        if args.source_bag_dir is None or args.replay_script is None:
            parser.error('--execute requires source bag and replay script')
        if not args.source_bag_dir.is_dir():
            parser.error('--source-bag-dir must be an existing directory')
        if not args.replay_script.is_file():
            parser.error('--replay-script must be an existing file')
        profile_document = json.loads(
            args.sensor_profile.read_text(encoding='utf-8'))
        manifest['analysis_window'] = profile_document['analysis_window']
        source_mcap = _one_mcap(args.source_bag_dir)
        manifest['source_bag'] = inspect_source_bag(
            source_mcap, manifest['analysis_window'])
        manifest['status'] = 'RUNNING'
    manifest_path = args.out_dir / 'subdivision_config_manifest.json'
    _write_reproducible(
        manifest_path, json.dumps(manifest, ensure_ascii=False, indent=2))
    if not args.execute:
        print(json.dumps({
            'status': manifest['status'],
            'manifest': str(manifest_path.resolve()),
        }, ensure_ascii=False))
        return 0
    runs = []
    resume_labels = set(args.resume_complete_label)
    resume_roots = {}
    for value in args.resume_artifact_root:
        label, separator, root_text = value.partition('=')
        if separator != '=' or label not in {
                f'subdivision_{item}' for item in SUBDIVISIONS}:
            parser.error('--resume-artifact-root requires subdivision_N=PATH')
        root = Path(root_text)
        if not root.is_dir():
            parser.error(f'resume artifact root does not exist: {root}')
        resume_labels.add(label)
        resume_roots[label] = root
    if resume_labels and args.resume_source_root is None:
        missing_roots = resume_labels - set(resume_roots)
        if missing_roots:
            parser.error('resumed labels require an artifact root')
    if args.resume_source_root is not None and not args.resume_source_root.is_dir():
        parser.error('--resume-source-root must be an existing directory')
    for label in args.resume_complete_label:
        resume_roots.setdefault(label, args.resume_source_root)
    repository = Path(__file__).resolve().parents[2]
    for subdivision in SUBDIVISIONS:
        label = f'subdivision_{subdivision}'
        config = Path(variants[str(subdivision)]['config']['path'])
        if label in resume_labels:
            artifact_root = resume_roots[label]
            try:
                log_path = artifact_root / (
                    f'{source_mcap.parent.name}__{label}.log')
                log_text = log_path.read_text(
                    encoding='utf-8', errors='replace')
                if (not args.smoke and
                        'Cartographer graceful shutdown 제한 시간 초과'
                        in log_text):
                    replay = collect_finalization_budget_failure(
                        artifact_root, source_mcap, label, config,
                        analysis_window=manifest['analysis_window'],
                        source_evidence=manifest['source_bag'])
                else:
                    replay = collect_replay(
                        artifact_root, source_mcap, label, config, 0,
                        smoke=args.smoke,
                        analysis_window=manifest['analysis_window'],
                        source_evidence=manifest['source_bag'])
                replay['posthoc_recollection'] = (
                    posthoc_recollection_provenance(
                        replay, repository, artifact_root))
                if not all(replay['posthoc_recollection'][
                        'reuse_preconditions'].values()):
                    raise ValueError('completed replay reuse gates failed')
            except (OSError, KeyError, TypeError, ValueError,
                    json.JSONDecodeError, subprocess.CalledProcessError) as error:
                replay = invalid_replay(
                    source_mcap, label, config, 0, error)
            runs.append(write_run_sidecar(args.out_dir, replay))
            if replay['status'] == 'INVALID' or (
                    args.smoke and replay['status'] != 'PASS'):
                break
            continue
        command = [
            'bash', str(args.replay_script.resolve()),
            '--bag', str(args.source_bag_dir.resolve()),
            '--backend', 'cartographer',
            '--label', label,
            '--cartographer-config', str(config),
            '--out', str(args.out_dir.resolve()),
            '--rate', '1.0',
        ]
        if args.smoke:
            command.extend(['--duration', str(args.smoke_duration_s)])
        timeout_s = args.run_timeout_s
        if timeout_s is None:
            timeout_s = 240.0 if args.smoke else 1800.0
        returncode, timeout_error = run_with_hard_timeout(command, timeout_s)
        if timeout_error is not None:
            replay = invalid_replay(
                source_mcap, label, config, returncode,
                TimeoutError(timeout_error))
            runs.append(replay)
            break
        try:
            log_path = args.out_dir / (
                f'{source_mcap.parent.name}__{label}.log')
            log_text = log_path.read_text(
                encoding='utf-8', errors='replace')
            if (not args.smoke and returncode in (8, 9) and
                    'Cartographer graceful shutdown 제한 시간 초과'
                    in log_text):
                replay = collect_finalization_budget_failure(
                    args.out_dir, source_mcap, label, config,
                    analysis_window=manifest['analysis_window'],
                    source_evidence=manifest['source_bag'])
            else:
                replay = collect_replay(
                    args.out_dir, source_mcap, label, config, returncode,
                    smoke=args.smoke,
                    analysis_window=manifest['analysis_window'],
                    source_evidence=manifest['source_bag'])
        except (OSError, KeyError, TypeError, ValueError,
                json.JSONDecodeError) as error:
            replay = invalid_replay(
                source_mcap, label, config, returncode, error)
        runs.append(write_run_sidecar(args.out_dir, replay))
        if replay['status'] == 'INVALID' or (
                args.smoke and replay['status'] != 'PASS'):
            break
    if not args.smoke and all(run['status'] == 'PASS' for run in runs):
        trajectory_hashes = {
            run['reference_metrics']['trajectory_timestamp_sha256']
            for run in runs}
        reference_hashes = {
            run['reference_metrics']['deviation_from_amcl'][
                'timestamp_sha256'] for run in runs}
        if len(trajectory_hashes) != 1 or len(reference_hashes) != 1:
            for run in runs:
                run['status'] = 'INVALID'
                run['sample_set_mismatch'] = True
    common_sample_contract = None
    if not args.smoke and len(runs) == len(SUBDIVISIONS) and not any(
            run['status'] == 'INVALID' for run in runs):
        common_sample_contract = apply_common_reference_sample_set(
            runs, source_mcap, manifest['analysis_window'])
    all_pass = all(run['status'] == 'PASS' for run in runs)
    any_invalid = any(run['status'] == 'INVALID' for run in runs)
    summary: dict[str, Any] = {
        'schema_version': 1,
        'mode': 'smoke' if args.smoke else 'full',
        'runs': runs,
        'overall': (
            'PASS' if all_pass
            else 'INVALID' if any_invalid else 'FAIL'),
        'same_domain_id': 199,
        'same_replay_rate': 1.0,
    }
    if common_sample_contract is not None:
        summary['common_sample_set'] = common_sample_contract
    if not args.smoke:
        aggregate = aggregate_status(runs)
        summary.update(aggregate)
    if args.environment_audit is not None:
        audit_document = json.loads(
            args.environment_audit.read_text(encoding='utf-8'))
        summary['environment_audit'] = {
            'artifact': _record(args.environment_audit),
            'evidence': audit_document,
        }
    if not args.smoke and 'selection' in summary:
        selected = summary['selection']['selected_candidate']
        for run in runs:
            run['candidate_status'] = (
                'VALID_BASELINE' if run['subdivision'] == 1
                else 'DIAGNOSTIC_CANDIDATE'
                if run['subdivision'] == selected and selected != 1
                else 'VALID_NON_CANDIDATE')
    summary_path = args.out_dir / 'subdivision_replay_summary.json'
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    manifest['status'] = summary['overall']
    manifest['commands_completed'] = 3
    manifest['summary'] = _record(summary_path)
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({
        'status': summary['overall'],
        'summary': str(summary_path.resolve()),
    }, ensure_ascii=False))
    return 0 if summary['overall'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
