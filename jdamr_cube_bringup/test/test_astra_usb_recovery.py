"""Validate the boot-time Astra USB recovery contract without USB hardware."""

import os
import subprocess
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'scripts/jdamr-astra-usb-recover'
RECOVERY_UNIT = PACKAGE / 'systemd/jdamr-astra-usb-recover.service'
CAMERA_UNIT = PACKAGE / 'systemd/jdamr-box-rgbd.service'
BASE_OVERRIDE = (
    PACKAGE / 'systemd/jdamr-base.service.d/15-astra-usb-recover.conf')


def run_recovery(tmp_path, lsusb_bin):
    """Run the helper against a temporary port-control file."""
    port = tmp_path / 'disable'
    port.write_text('unchanged', encoding='utf-8')
    environment = {
        **os.environ,
        'ASTRA_USB_PORT_DISABLE': str(port),
        'ASTRA_USB_INITIAL_WAIT_S': '0',
        'ASTRA_USB_PORT_OFF_S': '0',
        'ASTRA_USB_RECOVERY_WAIT_S': '0',
        'ASTRA_USB_RETRY_COUNT': '1',
        'LSUSB_BIN': lsusb_bin,
        'SLEEP_BIN': '/bin/true',
    }
    result = subprocess.run(
        [str(SCRIPT)], env=environment, capture_output=True, text=True,
        timeout=5, check=False)
    return result, port.read_text(encoding='utf-8')


def test_present_camera_does_not_cycle_port(tmp_path):
    """Leave the hub port untouched when USB enumeration already succeeded."""
    result, port_state = run_recovery(tmp_path, '/bin/true')

    assert result.returncode == 0
    assert port_state == 'unchanged'
    assert 'already present' in result.stdout


def test_failed_recovery_leaves_port_enabled(tmp_path):
    """Re-enable the camera port even when the device never enumerates."""
    result, port_state = run_recovery(tmp_path, '/bin/false')

    assert result.returncode == 1
    assert port_state == '0'
    assert 'recovery failed' in result.stderr


def interrupting_sleep(tmp_path, port, destroy_port=False):
    """Build a sleep command that terminates its parent during port-off."""
    helper = tmp_path / 'interrupt-sleep'
    actions = (
        'rm -f "$FAKE_PORT_PATH"\nmkdir "$FAKE_PORT_PATH"\n'
        if destroy_port else '')
    helper.write_text(
        '#!/bin/bash\n' + actions + 'kill -TERM "$PPID"\n',
        encoding='utf-8')
    helper.chmod(0o755)
    return {
        **os.environ,
        'ASTRA_USB_PORT_DISABLE': str(port),
        'ASTRA_USB_INITIAL_WAIT_S': '0',
        'ASTRA_USB_PORT_OFF_S': '1',
        'ASTRA_USB_RECOVERY_WAIT_S': '0',
        'ASTRA_USB_RETRY_COUNT': '1',
        'LSUSB_BIN': '/bin/false',
        'SLEEP_BIN': str(helper),
        'FAKE_PORT_PATH': str(port),
    }


def test_term_during_port_off_reenables_port(tmp_path):
    """A termination signal must leave the controlled port enabled."""
    port = tmp_path / 'disable'
    port.write_text('0', encoding='utf-8')
    result = subprocess.run(
        [str(SCRIPT)], env=interrupting_sleep(tmp_path, port),
        capture_output=True, text=True, timeout=5, check=False)

    assert result.returncode == 143
    assert port.read_text(encoding='utf-8') == '0'


def test_reenable_failure_is_reported_and_unsuccessful(tmp_path):
    """Do not hide a failed write that could leave the USB port disabled."""
    port = tmp_path / 'disable'
    port.write_text('0', encoding='utf-8')
    environment = interrupting_sleep(tmp_path, port, destroy_port=True)
    result = subprocess.run(
        [str(SCRIPT)], env=environment, capture_output=True, text=True,
        timeout=5, check=False)

    assert result.returncode != 0
    assert port.is_dir()
    assert 'port re-enable failed' in result.stderr


def test_systemd_orders_recovery_before_camera_and_base():
    """Finish USB recovery before either robot service starts."""
    recovery = RECOVERY_UNIT.read_text(encoding='utf-8')
    camera = CAMERA_UNIT.read_text(encoding='utf-8')
    base = BASE_OVERRIDE.read_text(encoding='utf-8')

    assert 'Before=jdamr-box-rgbd.service jdamr-base.service' in recovery
    assert (
        'After=network-online.target jdamr-astra-usb-recover.service'
        in camera)
    assert 'After=jdamr-astra-usb-recover.service' in base
    assert 'Wants=jdamr-astra-usb-recover.service' in camera
    assert 'Wants=jdamr-astra-usb-recover.service' in base
