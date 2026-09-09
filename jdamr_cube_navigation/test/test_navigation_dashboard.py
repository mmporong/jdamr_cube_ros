"""Tests for the read-only navigation supervision dashboard."""

import importlib.util
import json
import math
from pathlib import Path
import sys

import pytest


EVALUATION = Path(__file__).resolve().parents[1] / 'evaluation'
SPEC = importlib.util.spec_from_file_location(
    'navigation_dashboard', EVALUATION / 'navigation_dashboard.py')
DASHBOARD = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = DASHBOARD
SPEC.loader.exec_module(DASHBOARD)


def test_replay_rejects_wrong_video_and_noncausal_time(tmp_path):
    """A highlight or reversed timeline must not masquerade as aligned data."""
    import hashlib
    video = tmp_path / 'video.mp4'
    video.write_bytes(b'recorded-video')
    path = tmp_path / 'replay.json'
    document = {'schema_version': 1, 'samples': [{'time_s': 0.0}],
                'video_sha256': hashlib.sha256(video.read_bytes()).hexdigest()}
    path.write_text(json.dumps(document))
    assert DASHBOARD.load_replay(path, video) == document
    document['samples'].append({'time_s': 0.0})
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match='time ordered'):
        DASHBOARD.load_replay(path, video)
    document['samples'].pop()
    document['video_sha256'] = 'wrong'
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match='identity mismatch'):
        DASHBOARD.load_replay(path, video)


def test_export_replay_uses_causal_samples_and_captured_laser(tmp_path, monkeypatch):
    """Future commands stay absent and front is measured in the base frame."""
    monkeypatch.syspath_prepend(str(EVALUATION))
    import export_dashboard_replay as exporter
    capture = tmp_path / 'capture.json'
    capture.write_text(json.dumps({'frame_timestamps': [
        {'wall_ns': 1_000_000_000}, {'wall_ns': 1_400_000_000}]}))
    (tmp_path / 'jdamr_cube_capture.urdf').write_text(
        '<robot><joint name="laser_joint"><origin rpy="0 0 3.141592653589793"/>'
        '</joint></robot>')
    mcap = tmp_path / 'bag.mcap'
    mcap.write_bytes(b'test fixture')
    monkeypatch.setattr(exporter, '_read_telemetry', lambda _: {
        'scans': [(1_100_000_000, -math.pi, math.pi, 0.05, 8.0, [0.4, 2.0])],
        'cmd': [(1_300_000_000, 0.18, 0.0)], 'odom': [], 'plans': []})
    monkeypatch.setattr(exporter, 'read_navigation_messages', lambda *a, **k: [])
    result = exporter.export_replay(mcap, capture)
    assert result['samples'][0]['command'] == {}
    assert 'scan' not in result['samples'][0]
    assert result['samples'][1]['command'] == {}
    assert result['samples'][1]['scan']['front_minimum_m'] == pytest.approx(0.4)
    assert result['samples'][2]['command']['linear_mps'] == pytest.approx(0.18)
    assert result['bt_ticks_available'] is False
    trimmed = exporter.export_replay(mcap, capture, 0.2)
    assert trimmed['start_offset_s'] == 0.2
    assert trimmed['duration_s'] == pytest.approx(0.2)
    assert trimmed['samples'][0]['time_s'] == 0.0
    assert trimmed['samples'][0]['command'] == {}
    assert trimmed['samples'][0]['scan'] == result['samples'][1]['scan']
    assert trimmed['samples'][1]['command'] == result['samples'][2]['command']


@pytest.mark.parametrize('offset', [-1.0, math.nan, math.inf, 4.0, 5.0])
def test_trim_rejects_offsets_outside_capture(monkeypatch, offset):
    monkeypatch.syspath_prepend(str(EVALUATION))
    import export_dashboard_replay as exporter
    with pytest.raises(ValueError):
        exporter.trimmed_start_ns(1_000_000_000, 5_000_000_000, offset)


def test_replay_rejects_speedup_even_with_matching_file_hash(tmp_path, monkeypatch):
    """Hash equality does not prove that a video retained wall-time duration."""
    monkeypatch.syspath_prepend(str(EVALUATION))
    import export_dashboard_replay as exporter
    from types import SimpleNamespace
    video, capture = tmp_path / 'video.mp4', tmp_path / 'capture.json'
    raw = tmp_path / 'gazebo_sensor_raw.mp4'
    for path in (video, capture, raw):
        path.write_bytes(b'test fixture')
    timing = {'method': 'causal_camera_frame_hold_on_wall_time',
              'video_sha256': exporter._sha256(video),
              'capture_sha256': exporter._sha256(capture),
              'raw_sha256': exporter._sha256(raw)}
    video.with_suffix('.timing.json').write_text(json.dumps(timing))
    with pytest.raises(ValueError, match='provenance mismatch'):
        exporter.validate_video(video, capture, 103.6, 3.0)
    monkeypatch.setattr(exporter.subprocess, 'run', lambda *a, **k:
                        SimpleNamespace(stdout='{"format":{"duration":"78.3"}}'))
    with pytest.raises(ValueError, match='duration differs'):
        exporter.validate_video(video, capture, 103.6)


def test_scan_projection_is_bounded_and_reports_front_obstacle():
    """Downsample display rays without losing full-scan safety metrics."""
    state = DASHBOARD.LiveState()
    ranges = [2.0] * 360
    ranges[180] = 0.31
    ranges[90] = math.inf

    state.update_scan(
        ranges, -math.pi, 2.0 * math.pi / 360.0, 0.05, 8.0)
    snapshot = state.snapshot({'available': False})

    assert snapshot['scan']['sample_count'] == 360
    assert len(snapshot['scan']['points']) <= DASHBOARD.MAX_SCAN_POINTS
    assert snapshot['scan']['minimum_m'] == pytest.approx(0.31)
    assert snapshot['scan']['front_minimum_m'] == pytest.approx(0.31)
    assert snapshot['scan']['closest_angle_deg'] == pytest.approx(0.0)


def test_verified_run_extracts_only_evidence_fields(tmp_path):
    """Present the persisted verdict and metrics without synthetic defaults."""
    summary = tmp_path / 'summary.json'
    summary.write_text(json.dumps({
        'status': 'PASS', 'claim_scope': 'SIM_INTEGRATION',
        'results': [{
            'status': 'PASS', 'run_id': 'run-1',
            'case': 'sudden_stop_resume',
            'detour_evidence': {
                'straight_centerline_blocked': True,
                'maximum_abs_lateral_offset_m': 0.57,
                'minimum_clearance_m': 0.148,
            },
            'scenario': {
                'goal_send_count': 1, 'goal_cancel_count': 0,
                'contact_count': 0,
                'protected_envelope_minimum_clearance_m': 0.047,
                'observer_scan_to_zero_command_s': 0.067,
            },
        }],
    }), encoding='utf-8')

    run = DASHBOARD.load_verified_run(summary)

    assert run['status'] == 'PASS'
    assert run['claim_scope'] == 'SIM_INTEGRATION'
    assert run['goal_send_count'] == 1
    assert run['contact_count'] == 0
    assert run['straight_centerline_blocked'] is True
    assert run['maximum_lateral_offset_m'] == pytest.approx(0.57)
    assert run['static_obstacle_clearance_m'] == pytest.approx(0.148)


def test_verified_bundle_aggregates_two_directional_runs(tmp_path):
    """Show aggregate status without presenting two runs as one scenario."""
    manifest = tmp_path / 'portfolio_media_manifest.json'
    manifest.write_text(json.dumps({
        'status': 'PASS',
        'claim_scope': 'TWO_INDEPENDENT_SIM_INTEGRATION_RUNS',
        'runs': [
            {'entry_side': 'left', 'metrics': {
                'goal_send_count': 1, 'goal_cancel_count': 0,
                'contact_count': 0, 'protected_clearance_m': 0.039,
                'observer_scan_to_zero_command_s': 0.045,
                'same_goal_resume': True,
                'action_terminal': 'succeeded'}},
            {'entry_side': 'right', 'metrics': {
                'goal_send_count': 1, 'goal_cancel_count': 0,
                'contact_count': 0, 'protected_clearance_m': 0.045,
                'observer_scan_to_zero_command_s': 0.069,
                'same_goal_resume': True,
                'action_terminal': 'succeeded'}},
        ],
    }), encoding='utf-8')

    bundle = DASHBOARD.load_verified_bundle(manifest)

    assert bundle['directional_run_count'] == 2
    assert bundle['entry_sides'] == ['left', 'right']
    assert bundle['goal_send_count'] == 2
    assert bundle['contact_count'] == 0
    assert bundle['protected_clearance_m'] == pytest.approx(0.039)
    assert bundle['observer_latency_range_s'] == pytest.approx([0.045, 0.069])


@pytest.mark.parametrize('header,expected', [
    (None, None), ('bytes=0-99', (0, 99)),
    ('bytes=900-', (900, 999)), ('bytes=-100', (900, 999)),
])
def test_video_range_is_seekable(header, expected):
    """Support bounded HTTP byte ranges required by browser video players."""
    assert DASHBOARD._media_range(header, 1000) == expected


@pytest.mark.parametrize('header', ['items=0-1', 'bytes=0-1,5-6',
                                    'bytes=1000-', 'bytes=9-2'])
def test_video_range_rejects_invalid_requests(header):
    """Reject malformed or out-of-bounds media requests."""
    with pytest.raises(ValueError):
        DASHBOARD._media_range(header, 1000)
