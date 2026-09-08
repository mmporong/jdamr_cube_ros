"""Tests for the read-only navigation supervision dashboard."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest


EVALUATION = Path(__file__).resolve().parents[1] / 'evaluation'
SPEC = importlib.util.spec_from_file_location(
    'navigation_dashboard', EVALUATION / 'navigation_dashboard.py')
DASHBOARD = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = DASHBOARD
SPEC.loader.exec_module(DASHBOARD)


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
