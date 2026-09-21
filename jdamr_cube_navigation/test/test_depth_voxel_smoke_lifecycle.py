"""Regression coverage for isolated costmap ownership and final verdicts."""

import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess

import pytest


SCRIPT = (Path(__file__).resolve().parents[1]
          / 'evaluation' / 'smoke_depth_voxel_layer.py')


@pytest.fixture
def probe():
    """Load a fresh harness without creating nodes or child processes."""
    spec = importlib.util.spec_from_file_location('voxel_probe_test', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('code', [-11, -9, -2, None, 0])
def test_final_verdict_requires_clean_child_exit(
        probe, monkeypatch, tmp_path, code):
    """Passing geometry does not hide a crashed or unaccounted-for child."""
    monkeypatch.setattr(probe, '_installed_version', lambda: 'test-version')
    monkeypatch.setenv('ROS_DOMAIN_ID', '12')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    monkeypatch.setenv('ROS_STATIC_PEERS', 'example.invalid')

    def result(_log):
        assert os.environ['ROS_DOMAIN_ID'] == '197'
        assert os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
        assert os.environ['ROS_STATIC_PEERS'] == ''
        probe.PROGRESS.update(
            stages=['deactivated_costmap', 'cleaned_costmap'],
            child_cleanup_exit_code=code)
        return {'status': 'pass', 'cases': {'synthetic': {'pass': True}}}

    monkeypatch.setattr(probe, 'run_smoke', result)
    output = tmp_path / 'summary.json'
    exit_code = probe.main(['--output', str(output)])
    report = json.loads(output.read_text())
    assert exit_code == (0 if code == 0 else 1)
    assert (report['status'] == 'pass') == (code == 0)
    assert report['cleanup']['anomaly'] == (code != 0)
    assert report['cases']['synthetic']['pass']
    assert os.environ['ROS_DOMAIN_ID'] == '12'
    assert os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SUBNET'
    assert os.environ['ROS_STATIC_PEERS'] == 'example.invalid'


@pytest.mark.parametrize('stages,error', [
    ([], None),
    (['deactivated_costmap'], None),
    (['deactivated_costmap', 'cleaned_costmap'], 'cleanup service failed'),
])
def test_incomplete_lifecycle_is_failure_even_with_exit_zero(
        probe, monkeypatch, tmp_path, stages, error):
    """A signal exit is not a substitute for completed lifecycle cleanup."""
    def result(_log):
        probe.PROGRESS.update(stages=stages, child_cleanup_exit_code=0)
        if error:
            probe.PROGRESS['lifecycle_cleanup_error'] = error
        return {'status': 'pass'}

    monkeypatch.setattr(probe, 'run_smoke', result)
    output = tmp_path / 'summary.json'
    assert probe.main(['--output', str(output)]) == 1
    report = json.loads(output.read_text())
    assert report['status'] != 'pass'
    assert report['cleanup']['anomaly']


def test_lifetime_pin_is_child_only_and_preserves_existing_environment(
        probe, monkeypatch, tmp_path):
    """Retain the installed plugin DSO without altering the parent process."""
    library = tmp_path / 'lib' / 'liblayers.so'
    library.parent.mkdir()
    library.touch()
    monkeypatch.setattr(probe, 'get_package_prefix', lambda name: str(tmp_path))
    monkeypatch.setenv('LD_PRELOAD', '/example/previous.so')
    monkeypatch.setenv('ROS_DOMAIN_ID', '12')
    environment, pinned = probe._child_environment()
    assert pinned == library
    assert environment['LD_PRELOAD'] == f'{library}:/example/previous.so'
    assert environment['ROS_DOMAIN_ID'] == '197'
    assert environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert environment['ROS_STATIC_PEERS'] == ''
    assert os.environ['LD_PRELOAD'] == '/example/previous.so'
    assert os.environ['ROS_DOMAIN_ID'] == '12'


def test_missing_plugin_library_fails_before_launch(probe, monkeypatch, tmp_path):
    """Do not run a probe with a silently ineffective lifetime workaround."""
    monkeypatch.setattr(probe, 'get_package_prefix', lambda name: str(tmp_path))
    with pytest.raises(FileNotFoundError, match='liblayers'):
        probe._child_environment()


def test_launch_failure_removes_parameters_and_reports_failure(
        probe, monkeypatch, tmp_path):
    """Clean temporary parameters even when the child cannot be created."""
    captured = {}
    monkeypatch.setattr(probe, '_installed_version', lambda: 'test-version')
    monkeypatch.setattr(probe.rclpy, 'ok', lambda: False)
    monkeypatch.delenv('LD_PRELOAD', raising=False)

    def failed_launch(command, **kwargs):
        captured['parameters'] = Path(command[-1])
        assert captured['parameters'].is_file()
        assert kwargs['env']['LD_PRELOAD'].endswith('/lib/liblayers.so')
        assert kwargs['env']['ROS_DOMAIN_ID'] == '197'
        assert kwargs['start_new_session']
        raise OSError('child launch failed')

    monkeypatch.setattr(probe.subprocess, 'Popen', failed_launch)
    output = tmp_path / 'summary.json'
    assert probe.main(['--output', str(output)]) == 1
    assert not captured['parameters'].exists()
    assert 'LD_PRELOAD' not in os.environ
    report = json.loads(output.read_text())
    assert report['error'] == 'child launch failed'
    assert report['cleanup']['anomaly']


def test_previous_cleanup_error_does_not_leak_to_next_run(
        probe, monkeypatch, tmp_path):
    """Fresh runs have fresh ownership and verdict evidence."""
    probe.PROGRESS['lifecycle_cleanup_error'] = 'previous run failure'

    def result(_log):
        assert 'lifecycle_cleanup_error' not in probe.PROGRESS
        probe.PROGRESS.update(
            stages=['deactivated_costmap', 'cleaned_costmap'],
            child_cleanup_exit_code=0)
        return {'status': 'pass'}

    monkeypatch.setattr(probe, 'run_smoke', result)
    assert probe.main(['--output', str(tmp_path / 'summary.json')]) == 0


def test_functional_failure_cannot_become_pass_on_clean_exit(
        probe, monkeypatch, tmp_path):
    """Preserve the original test failure after successful cleanup."""
    monkeypatch.setattr(probe, '_installed_version', lambda: 'test-version')

    def result(_log):
        probe.PROGRESS.update(
            stages=['deactivated_costmap', 'cleaned_costmap'],
            child_cleanup_exit_code=0)
        raise RuntimeError('obstacle not marked')

    monkeypatch.setattr(probe, 'run_smoke', result)
    output = tmp_path / 'summary.json'
    assert probe.main(['--output', str(output)]) == 1
    report = json.loads(output.read_text())
    assert report['error'] == 'obstacle not marked'
    assert report['cleanup']['anomaly'] is False


def test_timeout_reaps_only_the_owned_child(probe):
    """Emergency process cleanup remains visible as a failed exit code."""
    class Child:
        def __init__(self):
            self.signals = []
            self.waits = 0
            self.killed = False

        def poll(self):
            return None

        def send_signal(self, value):
            self.signals.append(value)

        def wait(self, timeout):
            self.waits += 1
            if self.waits == 1:
                raise subprocess.TimeoutExpired('owned_costmap', timeout)
            return -signal.SIGKILL

        def kill(self):
            self.killed = True

    child = Child()
    assert probe._stop_child(child) == -signal.SIGKILL
    assert child.signals == [signal.SIGINT]
    assert child.killed and child.waits == 2
