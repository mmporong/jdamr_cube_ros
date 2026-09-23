"""Offline tests for motion-safe restaurant evidence replay."""

import importlib.util
import json
import os
from pathlib import Path
import signal

import pytest

PACKAGE = Path(__file__).parents[1]
LIVE_RVIZ = PACKAGE / 'rviz/restaurant_service.rviz'
REPLAY_SCRIPT = PACKAGE / 'evaluation/replay_service_evidence.py'
SPEC = importlib.util.spec_from_file_location(
    'replay_service_evidence', REPLAY_SCRIPT)
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)


@pytest.fixture
def bag(tmp_path):
    path = tmp_path / 'bag'
    path.mkdir()
    return path


def test_generated_rviz_is_recorded_only_and_source_is_unchanged(tmp_path):
    before = LIVE_RVIZ.read_bytes()
    document = replay.recorded_rviz_document(LIVE_RVIZ)
    assert LIVE_RVIZ.read_bytes() == before
    manager = document['Visualization Manager']
    assert manager['Global Options']['Use Sim Time'] is True
    assert all(display['Name'].startswith('Recorded ')
               for display in manager['Displays'])
    robot = next(display for display in manager['Displays']
                 if display['Class'] == 'rviz_default_plugins/RobotModel')
    assert robot['Enabled'] is False
    axes = next(display for display in manager['Displays']
                if display['Class'] == 'rviz_default_plugins/Axes')
    assert axes['Reference Frame'] == 'base_footprint'
    odometry = next(display for display in manager['Displays']
                    if display['Class'] == 'rviz_default_plugins/Odometry')
    assert odometry['Topic']['Value'] == '/odom'
    assert odometry['Keep'] == 100


def test_player_command_has_exact_allowlist_and_no_motion_surfaces(bag, tmp_path):
    generated = tmp_path / 'recorded.rviz'
    commands = replay.replay_commands(bag, generated, 1.0)
    player = commands['player']
    assert player[:5] == ['ros2', 'bag', 'play', str(bag), '--clock']
    assert player[player.index('--clock') + 1] == '30'
    assert player[player.index('--delay') + 1] == '3'
    assert '--start-offset' not in player
    topics = player[player.index('--topics') + 1:]
    assert topics == list(replay.REPLAY_TOPICS)
    joined = ' '.join(player).lower()
    for forbidden in (
            'cmd_vel', 'navigate_to_pose', 'follow_path', 'initialpose',
            'set_initial_pose', 'service call', 'action send_goal'):
        assert forbidden not in joined


def test_print_command_reports_forced_environment_and_argv(
        bag, capsys, monkeypatch):
    monkeypatch.setattr(replay, 'default_rviz_config', lambda: LIVE_RVIZ)
    assert replay.main(['--bag', str(bag), '--print-command']) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload['allowenv'] == {
        'ROS_DOMAIN_ID': '78',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'ROS_STATIC_PEERS': '',
        'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4',
    }
    assert payload['argv']['player'][-len(replay.REPLAY_TOPICS):] == list(
        replay.REPLAY_TOPICS)
    assert payload['argv']['rviz'][-3:] == [
        '--ros-args', '-p', 'use_sim_time:=true']
    assert payload['rviz_config']['Visualization Manager'][
        'Global Options']['Use Sim Time'] is True


class _Process:
    next_pid = 7000

    def __init__(self, argv, role, running=True):
        self.argv = argv
        self.role = role
        self.running = running
        self.returncode = None if running else 0
        self.pid = _Process.next_pid
        _Process.next_pid += 1
        self.poll_count = 0

    def poll(self):
        self.poll_count += 1
        if self.role == 'rviz' and self.poll_count >= 3:
            self.running = False
            self.returncode = 0
        return None if self.running else self.returncode

    def wait(self, timeout=None):
        self.running = False
        self.returncode = 0
        return 0


def test_subprocess_launch_keeps_rviz_after_player_exit_and_cleans_own_group(
        bag, tmp_path, monkeypatch):
    commands = replay.replay_commands(bag, tmp_path / 'recorded.rviz', 0.5)
    launched = []

    def popen(argv, env, start_new_session):
        role = 'rviz' if argv == commands['rviz'] else 'player'
        process = _Process(argv, role, running=role == 'rviz')
        launched.append((process, env, start_new_session))
        return process

    killed = []
    monkeypatch.setattr(replay.subprocess, 'Popen', popen)
    monkeypatch.setattr(replay.time, 'sleep', lambda _: None)
    monkeypatch.setattr(
        replay.os, 'killpg', lambda pid, sig: killed.append((pid, sig)))
    environment = os.environ.copy()
    environment.update(replay.ISOLATED_ENVIRONMENT)
    assert replay._run_children(commands, environment) == 0
    assert [entry[0].role for entry in launched] == ['rviz', 'player']
    assert all(entry[2] is True for entry in launched)
    assert launched[0][0].poll_count >= 3
    assert launched[1][0].poll_count >= 1
    assert killed == []


def test_window_close_stops_running_player_group(bag, tmp_path, monkeypatch):
    commands = replay.replay_commands(bag, tmp_path / 'recorded.rviz', 1.0)
    processes = {
        'rviz': _Process(commands['rviz'], 'rviz', running=False),
        'player': _Process(commands['player'], 'player', running=True),
    }

    def popen(argv, **_kwargs):
        return processes['rviz' if argv == commands['rviz'] else 'player']

    killed = []
    monkeypatch.setattr(replay.subprocess, 'Popen', popen)
    monkeypatch.setattr(
        replay.os, 'killpg', lambda pid, sig: killed.append((pid, sig)))
    assert replay._run_children(commands, replay.ISOLATED_ENVIRONMENT) == 0
    assert killed == [(processes['player'].pid, signal.SIGINT)]


def test_ctrl_c_stops_only_both_replay_child_groups(bag, tmp_path, monkeypatch):
    commands = replay.replay_commands(bag, tmp_path / 'recorded.rviz', 1.0)
    processes = {
        'rviz': _Process(commands['rviz'], 'rviz', running=True),
        'player': _Process(commands['player'], 'player', running=True),
    }

    def popen(argv, **_kwargs):
        return processes['rviz' if argv == commands['rviz'] else 'player']

    killed = []
    monkeypatch.setattr(replay.subprocess, 'Popen', popen)
    monkeypatch.setattr(
        replay.time, 'sleep',
        lambda _: (_ for _ in ()).throw(KeyboardInterrupt()))
    monkeypatch.setattr(
        replay.os, 'killpg', lambda pid, sig: killed.append((pid, sig)))
    assert replay._run_children(commands, replay.ISOLATED_ENVIRONMENT) == 130
    assert killed == [
        (processes['player'].pid, signal.SIGINT),
        (processes['rviz'].pid, signal.SIGINT),
    ]


def test_rejects_non_absolute_or_missing_bag(tmp_path, capsys):
    assert replay.main([
        '--bag', 'relative-bag', '--rviz-config', str(LIVE_RVIZ),
        '--print-command',
    ]) == 2
    assert 'existing absolute directory' in capsys.readouterr().err
    with pytest.raises(SystemExit):
        replay.parse_args(['--bag', str(tmp_path), '--rate', 'nan'])
