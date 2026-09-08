"""Tests for the bounded real-onboard candidate simulation smoke."""

import importlib.util
from pathlib import Path
import sys  # noqa: I100

import pytest

import yaml


EVALUATION = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(EVALUATION))
SPEC = importlib.util.spec_from_file_location(
    'run_onboard_candidate_smoke',
    EVALUATION / 'run_onboard_candidate_smoke.py')
SMOKE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SMOKE)


def test_candidate_params_have_only_allowed_measurement_deltas(tmp_path):
    """Keep the candidate copy equal to production outside measurement."""
    prepared = SMOKE.prepare_candidate_assets(tmp_path)
    production = yaml.safe_load(
        SMOKE.PRODUCTION_PARAMS.read_text(encoding='utf-8'))
    candidate = yaml.safe_load(prepared['params'].read_text(encoding='utf-8'))

    assert SMOKE._leaf_differences(production, candidate) == (
        SMOKE.ALLOWED_PARAM_DELTAS)
    assert prepared['mask_report']['keepout_cells'] > 0
    assert prepared['mask_report']['zones'] == [
        'sim_route_away_northeast_corner']


@pytest.mark.parametrize('domain_id', [12, 185, 188])
def test_main_rejects_physical_or_unassigned_domains(
        monkeypatch, tmp_path, domain_id):
    """Never allow the physical domain or domains outside the allocation."""
    monkeypatch.setattr(sys, 'argv', [
        'run_onboard_candidate_smoke.py', '--output-root', str(tmp_path),
        '--domain-id', str(domain_id), '--startup-only'])
    with pytest.raises(SystemExit) as error:
        SMOKE.main()
    assert error.value.code == 2


def test_main_rejects_two_cases_that_would_reach_domain_188(
        monkeypatch, tmp_path):
    """Validate every allocated case domain before preparing any output."""
    monkeypatch.setattr(sys, 'argv', [
        'run_onboard_candidate_smoke.py', '--output-root', str(tmp_path),
        '--domain-id', '187', '--startup-only'])
    with pytest.raises(SystemExit) as error:
        SMOKE.main()
    assert error.value.code == 2


def test_main_rejects_nonempty_output_root(monkeypatch, tmp_path):
    """Preserve all evidence owned by a previous smoke attempt."""
    (tmp_path / 'owned-by-earlier-run').write_text('preserve')
    monkeypatch.setattr(sys, 'argv', [
        'run_onboard_candidate_smoke.py', '--output-root', str(tmp_path),
        '--domain-id', '186', '--case', 'detour', '--startup-only'])
    with pytest.raises(SystemExit) as error:
        SMOKE.main()
    assert error.value.code == 2


def test_status_summary_extracts_uuid_and_terminal(tmp_path):
    """Retain the raw action goal identity and terminal status."""
    path = tmp_path / 'status.log'
    path.write_text(
        """status_list:
- goal_info:
    goal_id:
      uuid:
""" + ''.join(f'      - {value}\n' for value in range(16))
        + '  status: 2\n  status: 4\n', encoding='utf-8')

    summary = SMOKE._status_summary(path)

    assert summary['goal_uuids'] == [
        '000102030405060708090a0b0c0d0e0f']
    assert summary['final_status'] == 4


def test_action_success_without_ground_truth_motion_is_not_a_detour():
    """Reject Nav2 success caused by a wrong localization transform."""
    document = {
        'action_terminal': 'succeeded', 'contact_count': 0,
        'final_cmd_vel_zero': True,
        'final_robot_pose_xy_yaw': [-8.0, 0.0, 0.0],
        'mark_eligible_scan': None, 'plans_after_mark': [],
    }

    assert not SMOKE.scenario_passed(document, 'detour', 0)


def _valid_detour():
    return {
        'action_terminal': 'succeeded', 'contact_count': 0,
        'contact_matched_publisher_count_max': 1,
        'global_blocking': True, 'local_blocking': True,
        'final_cmd_vel_zero': True,
        'final_robot_pose_xy_yaw': [6.0, 0.0, 0.0],
        'mark_eligible_scan': {'stamp_ns': 10},
        'plans_after_mark': [{'stamp_ns': 20}],
    }


def test_detour_requires_observed_costmap_and_contact_evidence():
    """Accept a completed detour only with all independent observation paths."""
    assert SMOKE.scenario_passed(_valid_detour(), 'detour', 0)


@pytest.mark.parametrize('field,missing', [
    ('global_blocking', False), ('local_blocking', False),
    ('contact_matched_publisher_count_max', 0),
])
def test_missing_obstacle_or_contact_observation_cannot_pass(field, missing):
    """Zero contact count is not evidence if no contact publisher was observed."""
    document = _valid_detour()
    document[field] = missing
    assert not SMOKE.scenario_passed(document, 'detour', 0)
    del document[field]
    assert not SMOKE.scenario_passed(document, 'detour', 0)
