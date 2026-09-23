"""Validate bounded ROS command execution for the physical Pi."""

import os
import subprocess
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'scripts/jdamr-ros-run'


def fake_setup(tmp_path, name):
    """Create a readable no-op setup file."""
    setup = tmp_path / name
    setup.write_text(':\n', encoding='utf-8')
    return setup


def environment(tmp_path):
    """Use test setup files instead of a host ROS installation."""
    return {
        **os.environ,
        'JDAMR_ROS_SETUP': str(fake_setup(tmp_path, 'ros_setup.bash')),
        'JDAMR_WORKSPACE_SETUP': str(fake_setup(tmp_path, 'ws_setup.bash')),
    }


def test_exports_physical_transport_contract(tmp_path):
    """Supply the same transport values used by the base service."""
    command = [
        str(SCRIPT), '2', '/usr/bin/env',
    ]
    result = subprocess.run(
        command, env=environment(tmp_path), capture_output=True, text=True,
        timeout=5, check=False)

    assert result.returncode == 0
    values = dict(
        line.split('=', 1) for line in result.stdout.splitlines()
        if '=' in line)
    assert values['ROS_DOMAIN_ID'] == '12'
    assert values['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SUBNET'
    assert values['FASTDDS_BUILTIN_TRANSPORTS'] == 'UDPv4'


def test_timeout_terminates_command(tmp_path):
    """Do not leave an unbounded diagnostic command behind."""
    result = subprocess.run(
        [str(SCRIPT), '1', '/bin/sleep', '10'],
        env=environment(tmp_path), capture_output=True, text=True,
        timeout=5, check=False)

    assert result.returncode == 124


def test_rejects_invalid_timeout_before_loading_ros(tmp_path):
    """Reject an invalid bound without trying to source ROS."""
    result = subprocess.run(
        [str(SCRIPT), '0', '/bin/true'],
        env=environment(tmp_path), capture_output=True, text=True,
        timeout=5, check=False)

    assert result.returncode == 2
    assert 'positive integer' in result.stderr
