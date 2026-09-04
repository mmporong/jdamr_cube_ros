"""Regression tests for corridor evidence extraction and media helpers."""

from pathlib import Path
import sys

import pytest
import yaml


EVALUATION_ROOT = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(EVALUATION_ROOT))

from corridor_run_media import (  # noqa: E402,I100,I201
    gap_statistics,
    parse_route_log,
    write_media_manifest,
)


def test_parse_route_log_preserves_success_evidence():
    """Derive the route verdict only from timestamped node output."""
    text = '\n'.join((
        '[INFO] [100.000000000] [jdamr_corridor_route]: route preflight '
        'passed: poses=50 length=8.250m',
        '[INFO] [101.000000000] [jdamr_corridor_route]: send 1/2 '
        'outbound=(4.00,-0.50)',
        '[INFO] [103.000000000] [jdamr_corridor_route]: waypoint=1/2 '
        'remaining=0.20m recoveries=0 battery=12.10V',
        '[INFO] [104.000000000] [jdamr_corridor_route]: send 2/2 '
        'home=(0.00,-0.10)',
        '[INFO] [108.000000000] [jdamr_corridor_route]: waypoint=2/2 '
        'remaining=0.10m recoveries=0 battery=12.00V',
        '[INFO] [109.000000000] [jdamr_corridor_route]: corridor '
        'roundtrip succeeded',
    ))
    result = parse_route_log(text)

    assert result['success'] is True
    assert result['sent_waypoints'] == 2
    assert result['declared_waypoints'] == 2
    assert result['duration_s'] == pytest.approx(8.0)
    assert result['max_recoveries'] == 0
    assert result['battery_min_v'] == pytest.approx(12.0)
    assert result['preflight']['planned_path_length_m'] == pytest.approx(8.25)


def test_parse_route_log_does_not_promote_an_incomplete_run():
    """Keep a cancelled run incomplete when no success line exists."""
    text = '\n'.join((
        '[INFO] [200.000000000] [jdamr_corridor_route]: send 1/2 '
        'outbound=(4.00,-0.50)',
        '[INFO] [203.000000000] [jdamr_corridor_route]: waypoint=1/2 '
        'remaining=1.20m recoveries=1 battery=11.90V',
        '[WARN] [205.000000000] [jdamr_corridor_route]: cancel requested: '
        'scan stale',
    ))
    result = parse_route_log(text)

    assert result['success'] is False
    assert result['sent_waypoints'] == 1
    assert result['max_recoveries'] == 1
    assert result['end_stamp_ns'] == 205_000_000_000


def test_gap_statistics_reports_rate_and_tail_gap():
    """Calculate message continuity from recorder time in SI units."""
    result = gap_statistics([
        1_000_000_000,
        1_100_000_000,
        1_200_000_000,
        1_500_000_000,
    ])

    assert result['messages'] == 4
    assert result['rate_hz'] == pytest.approx(8.0)
    assert result['max_gap_s'] == pytest.approx(0.3)
    assert result['p99_gap_s'] == pytest.approx(0.296)


def test_gap_statistics_includes_silent_window_edges():
    """Count silence after the last message as a control-path gap."""
    result = gap_statistics(
        [1_100_000_000, 1_200_000_000],
        window_start_ns=1_000_000_000,
        window_end_ns=2_000_000_000,
    )

    assert result['messages'] == 2
    assert result['rate_hz'] == pytest.approx(2.0)
    assert result['max_gap_s'] == pytest.approx(0.8)


def test_gap_statistics_handles_empty_and_singleton_streams():
    """Represent missing continuity evidence without inventing a rate."""
    assert gap_statistics([]) == {
        'messages': 0,
        'rate_hz': None,
        'max_gap_s': None,
        'p99_gap_s': None,
    }
    assert gap_statistics([1]) == {
        'messages': 1,
        'rate_hz': None,
        'max_gap_s': None,
        'p99_gap_s': None,
    }


def test_media_manifest_hashes_generated_artifacts(tmp_path):
    """Make the exported portfolio bundle independently verifiable."""
    artifact = tmp_path / 'sample.csv'
    artifact.write_text('value\n1\n', encoding='utf-8')
    metrics = {
        'run_id': 'sample_run',
        'source': {
            'mcap_sha256': 'mcap-hash',
            'route_log_sha256': 'route-hash',
        },
    }

    manifest_path = write_media_manifest(tmp_path, metrics)
    manifest = yaml.safe_load(
        manifest_path.read_text(encoding='utf-8'))

    assert manifest['run_id'] == 'sample_run'
    assert manifest['files'][0]['file'] == 'sample.csv'
    assert manifest['files'][0]['size_bytes'] == artifact.stat().st_size
    assert len(manifest['files'][0]['sha256']) == 64
