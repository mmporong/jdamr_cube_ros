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


def test_status_summary_extracts_uuid_and_terminal(monkeypatch, tmp_path):
    """Retain the raw action goal identity and terminal status."""
    path = tmp_path / 'bag.mcap'
    path.write_bytes(b'evidence identity')

    class Value:
        pass

    def status(code):
        value = Value()
        value.status = code
        value.goal_info = Value()
        value.goal_info.goal_id = Value()
        value.goal_info.goal_id.uuid = list(range(16))
        return value

    message = Value()
    message.ros_msg = Value()
    message.ros_msg.status_list = [status(2), status(4)]
    monkeypatch.setattr(
        SMOKE, 'read_navigation_messages', lambda *_args, **_kwargs: [message])

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
    """Accept detour only with all independent observation paths."""
    assert SMOKE.scenario_passed(_valid_detour(), 'detour', 0)


@pytest.mark.parametrize('field,missing', [
    ('global_blocking', False), ('local_blocking', False),
    ('contact_matched_publisher_count_max', 0),
])
def test_missing_obstacle_or_contact_observation_cannot_pass(field, missing):
    """Require the contact publisher before accepting zero contacts."""
    document = _valid_detour()
    document[field] = missing
    assert not SMOKE.scenario_passed(document, 'detour', 0)
    del document[field]
    assert not SMOKE.scenario_passed(document, 'detour', 0)


def test_compact_recorder_uses_wall_log_time_and_hidden_status_qos(
        monkeypatch, tmp_path):
    """Keep same-goal ordering on recorder wall time without scan payloads."""
    captured = {}

    def fake_start(command, log_path, environment):
        captured.update(command=command, log_path=log_path,
                        environment=environment)
        return object(), object()

    monkeypatch.setattr(SMOKE, '_start', fake_start)
    SMOKE._start_compact_recorder(tmp_path, {'ROS_DOMAIN_ID': '186'})

    command = captured['command']
    assert command.count('ros2') == 1
    assert '--include-hidden-topics' in command
    assert '--use-sim-time' not in command
    assert '/scan' not in command
    assert set(SMOKE.RECORDED_TOPICS) <= set(command)
    qos = yaml.safe_load(
        (tmp_path / 'recording_qos.yaml').read_text(encoding='utf-8'))
    status = qos['/navigate_to_pose/_action/status']
    assert status == {
        'depth': 1, 'durability': 'transient_local',
        'history': 'keep_last', 'reliability': 'reliable'}
    assert all(
        profile['reliability'] == 'best_effort'
        for topic, profile in qos.items()
        if topic != '/navigate_to_pose/_action/status')


def test_output_cap_fails_without_rewriting_raw_logs(monkeypatch, tmp_path):
    """Preserve original evidence when the hard size cap is exceeded."""
    raw = b'original recorder evidence'
    log = tmp_path / 'recorder.log'
    log.write_bytes(raw)
    monkeypatch.setattr(SMOKE, 'CAP_BYTES', len(raw) - 1)

    with pytest.raises(RuntimeError, match='exceeds 64 MiB cap'):
        SMOKE._cap_output(tmp_path)

    assert log.read_bytes() == raw
