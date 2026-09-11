"""Terminal-gate tests for the integrated restaurant replay runner."""

import sys
from pathlib import Path

import yaml


EVALUATION = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(EVALUATION))

from prepare_sim_nav_obstacle_run import prepare  # noqa: E402,I100

from run_restaurant_replay_sim import classify_result  # noqa: E402,I100


def _scenario():
    return {
        'status': 'PASS',
        'successful_goal_count': 20,
        'successful_intervention_count': 6,
    }


def _guard():
    return {'recovery_count': 1, 'final_state': 'RECOVERED'}


def test_integrated_recovery_gets_explicit_progress_time_budget(tmp_path):
    """The guard sequence must fit inside the evaluation progress window."""
    outputs = prepare(tmp_path, movement_time_allowance_s=25.0)
    params = yaml.safe_load(outputs['params'].read_text(encoding='utf-8'))

    assert params['controller_server']['ros__parameters'][
        'progress_checker']['movement_time_allowance'] == 25.0


def test_complete_replay_passes_every_terminal_gate(tmp_path):
    """One recovery, full route, six events, and MCAP are all mandatory."""
    mcap = tmp_path / 'bag.mcap'
    mcap.write_bytes(b'mcap')

    outcome, failures = classify_result(
        _scenario(), _guard(), mcap, [])

    assert outcome == 'PASS'
    assert failures == []


def test_partial_or_latched_replay_fails_closed(tmp_path):
    """A safe latch cannot be reported as a successful route recovery."""
    scenario = _scenario()
    scenario['successful_intervention_count'] = 5
    guard = _guard()
    guard['final_state'] = 'FAULT_LATCHED'

    outcome, failures = classify_result(
        scenario, guard, tmp_path / 'missing.mcap', [])

    assert outcome == 'FAIL'
    assert failures == [
        'obstacle_intervention_gate_failed',
        'traction_guard_latched',
    ]


def test_survivors_and_missing_evidence_are_rejected():
    """No producer or teardown gap is accepted as an experiment result."""
    outcome, failures = classify_result(
        None, None, None, [{'pid': 123}])

    assert outcome == 'FAIL'
    assert failures == [
        'scenario_evidence_missing',
        'guard_evidence_missing',
        'mcap_missing_or_unfinalized',
        'process_survivors_present',
    ]
