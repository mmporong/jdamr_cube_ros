"""Terminal-gate tests for the integrated restaurant replay runner."""

import sys
from pathlib import Path

import pytest
import yaml  # noqa: I201


EVALUATION = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(EVALUATION))

from prepare_sim_nav_obstacle_run import prepare  # noqa: E402,I100

from run_restaurant_replay_sim import (  # noqa: E402,I100
    classify_result,
    encode_camera_video,
    wait_file_ready,
)
from resynchronize_restaurant_video import (  # noqa: E402,I100,I201
    is_recoverable_video_failure,
    verify_evidence_record,
)


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


def test_actual_map_start_pose_replaces_corridor_fixture_pose(tmp_path):
    """AMCL starts at the real route home coordinates without scaling."""
    outputs = prepare(
        tmp_path, initial_pose_xy=(0.0, -0.1), stop_zone_front_m=0.35)
    params = yaml.safe_load(outputs['params'].read_text(encoding='utf-8'))
    initial = params['amcl']['ros__parameters']['initial_pose']
    stop_points = yaml.safe_load(
        params['collision_monitor']['ros__parameters']['StopZone']['points'])

    assert initial['x'] == 0.0
    assert initial['y'] == -0.1
    assert stop_points[0][0] == 0.35
    assert stop_points[1][0] == 0.35


def test_two_camera_encoder_builds_wide_and_robot_front_composite(
        tmp_path, monkeypatch):
    """The published 3D video retains both synchronized Gazebo views."""
    captured = {}

    def run(command, check):
        captured['command'] = command
        captured['check'] = check

    monkeypatch.setattr('run_restaurant_replay_sim.subprocess.run', run)
    encode_camera_video(
        [tmp_path / 'wide.mp4', tmp_path / 'front.mp4'],
        tmp_path / 'final.mp4', 15.0, timing_scales=[1.2, 1.6])

    command = captured['command']
    assert command.count('-i') == 2
    assert 'hstack=inputs=2' in command[command.index('-filter_complex') + 1]
    assert 'setpts=0.5*PTS' in command[command.index('-filter_complex') + 1]
    assert 'setpts=1.200000000*PTS' in command[
        command.index('-filter_complex') + 1]
    assert 'setpts=1.600000000*PTS' in command[
        command.index('-filter_complex') + 1]
    assert 'libx264' in command
    assert 'ultrafast' in command
    assert 'yuv420p' in command
    assert captured['check'] is True


def test_two_camera_encoder_supports_verified_nvidia_path(
        tmp_path, monkeypatch):
    """The hardware encoder keeps the same H.264 browser output contract."""
    captured = {}

    def run(command, check):
        captured['command'] = command
        captured['check'] = check

    monkeypatch.setattr('run_restaurant_replay_sim.subprocess.run', run)
    encode_camera_video(
        [tmp_path / 'wide.mp4', tmp_path / 'front.mp4'],
        tmp_path / 'final.mp4', 15.0, 'h264_nvenc')

    command = captured['command']
    assert command[command.index('-c:v') + 1] == 'h264_nvenc'
    assert command[command.index('-preset') + 1] == 'p1'
    assert command[command.index('-cq') + 1] == '23'
    assert captured['check'] is True


def test_camera_ready_gate_requires_nonempty_first_frame_marker(tmp_path):
    """A topic alone cannot admit a camera that never produced a frame."""
    ready = tmp_path / 'ready'
    ready.write_text('1280x720@15\n', encoding='utf-8')

    class RunningProcess:
        @staticmethod
        def poll():
            return None

    wait_file_ready(RunningProcess(), ready, timeout_s=0.01)


def test_video_only_interruption_can_be_resealed_without_replaying_route():
    """Core evidence and clean teardown are mandatory for video recovery."""
    summary = {
        'status': 'FAIL', 'gate_failures': ['runner_failure'],
        'failure': 'camera_encode_CalledProcessError: interrupted',
        'scenario': {'status': 'PASS'},
        'traction_guard': {'final_state': 'RECOVERED'},
        'bag': {'path': '/tmp/evidence.mcap'},
        'teardown': {
            'remaining_process_groups': [], 'identity_survivors': []},
    }

    assert is_recoverable_video_failure(summary)
    summary['teardown']['identity_survivors'] = [{'pid': 123}]
    assert not is_recoverable_video_failure(summary)


def test_video_recovery_rejects_changed_raw_evidence(tmp_path):
    """Derived media cannot silently reseal modified camera inputs."""
    raw = tmp_path / 'camera.mp4'
    raw.write_bytes(b'original')
    record = {
        'path': str(raw), 'bytes': raw.stat().st_size,
        'sha256': (
            '0682c5f2076f099c34cfdd15a9e063849ed437a4'
            '9677e6fcc5b4198c76575be5'),
    }

    assert verify_evidence_record(record, 'camera') == raw
    raw.write_bytes(b'changed!')
    with pytest.raises(ValueError, match='digest changed'):
        verify_evidence_record(record, 'camera')


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
