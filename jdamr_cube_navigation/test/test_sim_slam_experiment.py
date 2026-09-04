"""Tests for the deterministic simulated SLAM experiment harness."""

from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from run_sim_slam_experiment import backend_command  # noqa: E402,I100


def test_cartographer_command_is_headless_and_uses_sim_time():
    """The experiment backend must not start RViz or wall-time mapping."""
    command = backend_command('cartographer', None, Path('/tmp/config'))

    assert command[:4] == [
        'ros2', 'run', 'cartographer_ros', 'cartographer_node']
    assert 'jdamr_cube_2d_real.lua' in command
    assert 'use_sim_time:=true' in command
    assert 'rviz2' not in command


def test_slam_toolbox_command_requires_explicit_parameters(tmp_path):
    """Every Toolbox result must remain attributable to one config file."""
    with pytest.raises(ValueError, match='requires'):
        backend_command('slam_toolbox', None)

    params = tmp_path / 'params.yaml'
    command = backend_command('slam_toolbox', params)

    assert f'slam_params_file:={params}' in command


def test_unknown_backend_is_rejected():
    """A typo must not silently select another mapping process."""
    with pytest.raises(ValueError, match='unsupported backend'):
        backend_command('unknown', None)
