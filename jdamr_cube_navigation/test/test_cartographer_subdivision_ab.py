"""Contract tests for the Cartographer subdivision operational A/B."""

import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

import pytest
import yaml


EVALUATION = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(EVALUATION))

from run_cartographer_subdivision_ab import (  # noqa: E402,I100
    _resource_summary,
    _timestamp_span_gate,
    _validate_map_relocation,
    _validate_pgm,
    aggregate_status,
    choose_candidate,
    fatal_cartographer_log_lines,
    posthoc_recollection_provenance,
    validate_scan_timing_samples,
    write_subdivision_config,
)
from canonical_evidence_tree import (  # noqa: E402,I100
    canonical_tree_manifest,
    verify_manifest,
)
from normalize_map_yaml import (  # noqa: E402,I100
    normalize_map_yaml,
    verify_relocated_copy,
)
from scan_replay_processes import scan_processes  # noqa: E402


def _profile(path):
    path.write_text(json.dumps({
        'scan': {
            'declared_scan_time_s': {'p50': 0.1, 'p99': 0.11},
            'declared_time_increment_s': {'p50': 0.001},
        },
    }))


def _fake_process(proc_root, pid, *, run_id, domain_id, process_group):
    process = proc_root / str(pid)
    process.mkdir()
    process.joinpath('environ').write_bytes(
        f'JDAMR_REPLAY_RUN_ID={run_id}\0ROS_DOMAIN_ID={domain_id}\0'.encode())
    process.joinpath('cmdline').write_bytes(b'ros2\0run\0worker\0')
    fields = ['S', '1', str(process_group), str(process_group)]
    fields.extend(['0'] * 15)
    fields.append('12345')
    process.joinpath('stat').write_text(
        f'{pid} (worker) ' + ' '.join(fields))


def test_identity_scan_isolates_run_domain_and_preserves_process_identity(
        tmp_path):
    """Cleanup discovery must neither miss owned groups nor match neighbours."""
    proc_root = tmp_path / 'proc'
    proc_root.mkdir()
    _fake_process(
        proc_root, 101, run_id='bag__subdivision_1', domain_id=199,
        process_group=101)
    _fake_process(
        proc_root, 102, run_id='another_run', domain_id=199,
        process_group=102)
    _fake_process(
        proc_root, 103, run_id='bag__subdivision_1', domain_id=12,
        process_group=103)
    racing = proc_root / '104'
    racing.mkdir()
    racing.joinpath('environ').write_bytes(
        b'JDAMR_REPLAY_RUN_ID=bag__subdivision_1\0ROS_DOMAIN_ID=199\0')

    matches = scan_processes(
        proc_root, 'bag__subdivision_1', 199, set())

    assert matches == [{
        'pid': 101,
        'ppid': 1,
        'process_group': 101,
        'session': 101,
        'start_ticks': 12345,
        'command': ['ros2', 'run', 'worker'],
    }]


def test_canonical_tree_hash_is_order_independent_and_self_excluding(tmp_path):
    """A copied tree verifies while changed bytes or circular manifests fail."""
    root = tmp_path / 'evidence'
    root.mkdir()
    root.joinpath('z.txt').write_text('last')
    root.joinpath('a.txt').write_text('first')
    output_name = 'tree_manifest.json'
    manifest = canonical_tree_manifest(root, {output_name})
    root.joinpath(output_name).write_text(json.dumps(manifest))

    assert [item['relative_path'] for item in manifest['entries']] == [
        'a.txt', 'z.txt']
    assert verify_manifest(manifest) is True
    root.joinpath('a.txt').write_text('changed')
    assert verify_manifest(manifest) is False

    with pytest.raises(ValueError, match='symlink'):
        root.joinpath('link').symlink_to(root / 'z.txt')
        canonical_tree_manifest(root, {output_name})


def test_map_yaml_relocation_preserves_raw_and_changes_only_image(tmp_path):
    """A durable map stays self-contained with byte-preserved parent evidence."""
    source = tmp_path / 'source'
    durable = tmp_path / 'durable'
    source.mkdir()
    durable.mkdir()
    source_pgm = source / 'map.pgm'
    durable_pgm = durable / 'map.pgm'
    source_pgm.write_bytes(b'P5\n1 1\n255\n\x00')
    shutil.copyfile(source_pgm, durable_pgm)
    source_yaml = source / 'map.yaml'
    source_yaml.write_text(
        f'image: {source_pgm}\nresolution: 0.05\norigin: [0, 0, 0]\n')
    map_yaml = durable / 'map.yaml'
    shutil.copyfile(source_yaml, map_yaml)
    raw_yaml = durable / 'map.raw.yaml'
    provenance_path = durable / 'map.relocation.json'

    provenance = normalize_map_yaml(
        map_yaml, durable_pgm, raw_yaml, provenance_path)

    assert yaml.safe_load(raw_yaml.read_text())['image'] == str(source_pgm)
    assert yaml.safe_load(map_yaml.read_text())['image'] == 'map.pgm'
    assert provenance['image_only_change_verified'] is True
    assert provenance['pgm_sha256_equal'] is True
    assert _validate_map_relocation(map_yaml, durable_pgm)['verified'] is True
    copy_report = verify_relocated_copy(source, durable)
    assert copy_report['semantic_equivalence_verified'] is True


def test_generated_config_changes_only_the_subdivision(tmp_path):
    """Production content remains recoverable by restoring one setting."""
    base = tmp_path / 'base.lua'
    output = tmp_path / 'generated' / 'subdivision_4.lua'
    profile = tmp_path / 'sensor_profile.json'
    base.write_text(
        'options = {\n'
        '  num_laser_scans = 1,\n'
        '  num_subdivisions_per_laser_scan = 1, -- production\n'
        '}\nreturn options\n')
    _profile(profile)

    provenance = write_subdivision_config(base, output, 4, profile)

    assert 'num_subdivisions_per_laser_scan = 4' in output.read_text()
    assert provenance['only_setting_changed'] is True
    assert provenance['source_config']['sha256'] == hashlib.sha256(
        base.read_bytes()).hexdigest()
    assert provenance['generated_config']['sha256'] == hashlib.sha256(
        output.read_bytes()).hexdigest()
    assert provenance['measurement_source']['values'][
        'scan_time_median_s']['json_pointer'] == (
            '/scan/declared_scan_time_s/p50')


def test_generator_rejects_missing_duplicate_or_unsupported_setting(tmp_path):
    """Ambiguous Lua input cannot become experiment evidence."""
    profile = tmp_path / 'sensor_profile.json'
    _profile(profile)
    output = tmp_path / 'output.lua'
    missing = tmp_path / 'missing.lua'
    duplicate = tmp_path / 'duplicate.lua'
    missing.write_text('return {}\n')
    duplicate.write_text(
        'num_subdivisions_per_laser_scan = 1,\n'
        'num_subdivisions_per_laser_scan = 1,\n')

    with pytest.raises(ValueError, match='expected one'):
        write_subdivision_config(missing, output, 2, profile)
    with pytest.raises(ValueError, match='expected one'):
        write_subdivision_config(duplicate, output, 2, profile)
    with pytest.raises(ValueError, match='unsupported'):
        write_subdivision_config(duplicate, output, 3, profile)

    wrong_baseline = tmp_path / 'wrong.lua'
    wrong_baseline.write_text('num_subdivisions_per_laser_scan = 2,\n')
    with pytest.raises(ValueError, match='exactly one'):
        write_subdivision_config(wrong_baseline, output, 2, profile)


def test_scan_timing_gate_rejects_zero_nonfinite_and_inconsistent_frames():
    """Subdivision is invalid unless every scan carries coherent timing."""
    valid = validate_scan_timing_samples([
        (0.1, 0.001, 100),
        (0.2, 0.001, 200),
    ])
    assert valid['valid'] is True

    invalid = validate_scan_timing_samples([
        (0.0, 0.0, 100),
        (math.nan, 0.001, 100),
        (0.1, 0.0005, 100),
    ])
    assert invalid['valid'] is False
    assert invalid['invalid_frames'] == 2
    assert invalid['inconsistent_frames'] == 1
    assert validate_scan_timing_samples([])['valid'] is False


def test_timestamp_span_prefers_exact_ns_and_bounds_legacy_float_error():
    """New evidence is exact; old float evidence is bounded by its own ULP."""
    expected = {
        'scan_timestamp_first_unix_ns': 1_788_502_857_241_324_128,
        'scan_timestamp_last_unix_ns': 1_788_503_613_444_269_789,
        'odometry_timestamp_first_unix_ns': 1_788_502_857_379_109_170,
        'odometry_timestamp_last_unix_ns': 1_788_503_613_570_062_077,
    }
    legacy = {
        'scan_first_s': 1788502857.2413242,
        'scan_last_s': 1788503613.44427,
        'odometry_received_first_s': 1788502857.3791091,
        'odometry_received_last_s': 1788503613.5700622,
    }

    migrated = _timestamp_span_gate(legacy, expected)

    assert migrated['all_match'] is True
    assert migrated['exact_ns_available'] is False
    assert migrated['legacy_float_timestamp_ulp_verified'] is True
    assert all(
        item['evidence_kind'].startswith('legacy_float')
        for item in migrated['comparisons'].values())
    exact = dict(legacy)
    for prefix, source_key in (
            ('scan_first', 'scan_timestamp_first_unix_ns'),
            ('scan_last', 'scan_timestamp_last_unix_ns'),
            ('odometry_received_first', 'odometry_timestamp_first_unix_ns'),
            ('odometry_received_last', 'odometry_timestamp_last_unix_ns')):
        exact[f'{prefix}_unix_ns'] = expected[source_key]
    exact_result = _timestamp_span_gate(exact, expected)
    assert exact_result['all_match'] is True
    assert exact_result['exact_ns_available'] is True
    assert exact_result['legacy_float_timestamp_ulp_verified'] is False
    assert all(
        item['maximum_error_ns'] == 0
        for item in exact_result['comparisons'].values())

    legacy['scan_first_s'] = math.nextafter(
        math.nextafter(legacy['scan_first_s'], math.inf), math.inf)
    assert _timestamp_span_gate(legacy, expected)['all_match'] is False


def test_cartographer_fatal_log_gate_is_specific():
    """Ignored subdivision and dropped scans invalidate a replay."""
    log = '\n'.join((
        'INFO trajectory started',
        'WARNING num_subdivisions_per_laser_scan ignored: invalid scan_time',
        'ERROR dropped laser scan because timing is missing',
        'INFO finished trajectory',
    ))

    fatal = fatal_cartographer_log_lines(log)

    assert len(fatal) == 2
    assert fatal_cartographer_log_lines('INFO finished trajectory') == []


def test_resource_summary_recomputes_procfs_delta_mean_and_peak(tmp_path):
    """The report must be derivable from raw interval samples."""
    path = tmp_path / 'resources.jsonl'
    rows = [
        {'cpu_pct_one_core': None, 'rss_mb': 90.0, 'process_count': 1},
        {'cpu_pct_one_core': 40.0, 'rss_mb': 110.0, 'process_count': 2},
        {'cpu_pct_one_core': 60.0, 'rss_mb': 100.0, 'process_count': 1},
    ]
    path.write_text(''.join(json.dumps(row) + '\n' for row in rows))

    summary = _resource_summary(path)

    assert summary['cpu_mean_pct_one_core'] == 50.0
    assert summary['rss_peak_mb'] == 110.0
    assert 'procfs CPU-time delta' in summary['semantics']


def test_map_gate_checks_yaml_link_dimensions_and_pixel_classes(tmp_path):
    """A nonempty but malformed or single-class map is not evidence."""
    pgm = tmp_path / 'map.pgm'
    metadata = tmp_path / 'map.yaml'
    pgm.write_bytes(b'P5\n3 1\n255\n' + bytes((0, 128, 255)))
    metadata.write_text('image: map.pgm\nresolution: 0.05\n')

    result = _validate_pgm(metadata, pgm)

    assert result['pixel_count'] == 3
    assert result['pixel_classes'] == {
        'occupied': 1, 'unknown': 1, 'free': 1}
    metadata.write_text('image: other.pgm\nresolution: 0.05\n')
    with pytest.raises(ValueError, match='reference'):
        _validate_pgm(metadata, pgm)


def test_smoke_map_gate_allows_partial_but_nonuniform_distribution(tmp_path):
    """A short smoke need not accumulate every full-route map class."""
    pgm = tmp_path / 'map.pgm'
    metadata = tmp_path / 'map.yaml'
    pgm.write_bytes(b'P5\n3 1\n255\n' + bytes((0, 128, 128)))
    metadata.write_text('image: map.pgm\nresolution: 0.05\n')

    result = _validate_pgm(
        metadata, pgm, require_all_classes=False)

    assert result['required_distribution'] == 'nonuniform_with_observed_class'
    with pytest.raises(ValueError, match='occupied, unknown, or free'):
        _validate_pgm(metadata, pgm)
    pgm.write_bytes(b'P5\n3 1\n255\n' + bytes((128, 128, 128)))
    with pytest.raises(ValueError, match='nonuniform observed'):
        _validate_pgm(metadata, pgm, require_all_classes=False)


def _run(subdivision, closure_m, reference_rms_m, cpu, rss):
    return {
        'subdivision': subdivision,
        'status': 'PASS',
        'reference_metrics': {
            'start_to_end_m_raw': closure_m,
            'deviation_from_amcl': {'rms_m_raw': reference_rms_m},
        },
        'resources': {
            'cpu_mean_pct_one_core': cpu,
            'rss_peak_mb': rss,
        },
    }


def test_selection_requires_strict_quality_and_unambiguous_pareto_winner():
    """Two eligible but crossing candidates retain production value one."""
    crossing = [
        _run(1, 1.0, 1.0, 100.0, 100.0),
        _run(2, 0.8, 0.9, 90.0, 90.0),
        _run(4, 0.9, 0.8, 90.0, 90.0),
    ]
    assert choose_candidate(crossing)['selected_candidate'] == 1

    dominated = [
        _run(1, 1.0, 1.0, 100.0, 100.0),
        _run(2, 0.8, 0.8, 90.0, 90.0),
        _run(4, 0.9, 0.9, 95.0, 95.0),
    ]
    decision = choose_candidate(dominated)
    assert decision['selected_candidate'] == 2
    assert decision['selection_uses_unrounded_raw_values'] is True


def test_invalid_baseline_cannot_select_a_candidate():
    """A missing production reference makes the comparison inconclusive."""
    runs = [
        _run(1, 1.0, 1.0, 100.0, 100.0),
        _run(2, 0.5, 0.5, 50.0, 50.0),
        _run(4, 0.6, 0.6, 60.0, 60.0),
    ]
    runs[0]['status'] = 'INVALID'

    assert choose_candidate(runs)['selected_candidate'] == 1


def test_failed_candidate_can_retain_a_valid_production_baseline():
    """A treatment budget failure is non-candidate, not inconclusive baseline."""
    runs = [
        _run(1, 1.0, 1.0, 100.0, 100.0),
        _run(2, 2.0, 2.0, 200.0, 200.0),
        _run(4, 3.0, 3.0, 300.0, 300.0),
    ]
    runs[2]['status'] = 'FAIL'

    result = aggregate_status(runs)

    assert result['overall'] == 'FAIL'
    assert result['final_status'] == 'RETAIN_PRODUCTION'
    assert result['selection']['selected_candidate'] == 1

    runs[0]['status'] = 'INVALID'
    assert aggregate_status(runs)['final_status'] == 'INCONCLUSIVE'


def test_posthoc_recollection_pins_raw_hashes_and_evaluator(tmp_path):
    """A parser-only recollection must retain auditable raw identities."""
    artifact = tmp_path / 'raw.mcap'
    artifact.write_bytes(b'raw evidence')
    runner = EVALUATION / 'run_cartographer_subdivision_ab.py'
    run = {
        'status': 'PASS',
        'returncode': 0,
        'teardown': {'remaining_process_groups': []},
        'artifacts': {
            'result_mcap': {
                'path': str(artifact),
                'sha256': hashlib.sha256(artifact.read_bytes()).hexdigest(),
            },
        },
    }

    result = posthoc_recollection_provenance(
        run, runner.parents[2], tmp_path)

    assert result['collection_recomputed_after_parser_fix'] is True
    assert result['rerun_performed'] is False
    assert result['artifact_storage']['root'] == str(tmp_path.resolve())
    assert all(result['reuse_preconditions'].values())
    assert result['raw_artifact_hashes']['result_mcap'] == (
        run['artifacts']['result_mcap']['sha256'])


def test_posthoc_recollection_accepts_preserved_finalization_budget_failure(
        tmp_path):
    """A complete replay can be reused when only bounded finalization failed."""
    artifact = tmp_path / 'raw.mcap'
    artifact.write_bytes(b'raw evidence')
    run = {
        'status': 'FAIL',
        'returncode': 9,
        'failure_kind': 'FAIL_FINALIZATION_BUDGET',
        'teardown': {'current_remaining_process_groups': []},
        'artifacts': {
            'result_mcap': {
                'path': str(artifact),
                'sha256': hashlib.sha256(artifact.read_bytes()).hexdigest(),
            },
        },
    }

    result = posthoc_recollection_provenance(
        run, EVALUATION.parents[1], tmp_path)

    assert all(result['reuse_preconditions'].values())
