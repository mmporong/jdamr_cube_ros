"""Validate the boot-time Astra USB recovery contract without USB hardware."""

import os
from pathlib import Path
import subprocess


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'scripts/jdamr-astra-usb-recover'
RECOVERY_UNIT = PACKAGE / 'systemd/jdamr-astra-usb-recover.service'
CAMERA_UNIT = PACKAGE / 'systemd/jdamr-box-rgbd.service'
BASE_OVERRIDE = (
    PACKAGE / 'systemd/jdamr-base.service.d/15-astra-usb-recover.conf')


def create_enumerated_camera(
        sysfs_root, interface_count='3', endpoint_addresses=('81', '82')):
    """Create the Astra descriptor shape observed on the robot."""
    device = sysfs_root / '1-1.2'
    device.mkdir(parents=True)
    descriptor_values = {
        'idVendor': '2bc5',
        'idProduct': '0402',
        'authorized': '1',
        'bConfigurationValue': '1',
        'bNumConfigurations': '1',
        'bNumInterfaces': interface_count,
    }
    for name, value in descriptor_values.items():
        (device / name).write_text(value, encoding='utf-8')
    (device / 'descriptors').write_bytes(b'usb-descriptors')

    vendor_interface = sysfs_root / '1-1.2:1.0'
    vendor_interface.mkdir()
    (vendor_interface / 'bInterfaceClass').write_text('ff', encoding='utf-8')
    (vendor_interface / 'bNumEndpoints').write_text('02', encoding='utf-8')
    for address in endpoint_addresses:
        endpoint = vendor_interface / f'ep_{address}'
        endpoint.mkdir()
        (endpoint / 'type').write_text('Bulk', encoding='utf-8')
        (endpoint / 'direction').write_text('in', encoding='utf-8')
    return device


def run_recovery(
        tmp_path, lsusb_bin, *, create_camera=True, interface_count='3',
        endpoint_addresses=('81', '82'),
        **overrides):
    """Run the helper against a temporary port-control file."""
    port = tmp_path / 'disable'
    port.write_text('unchanged', encoding='utf-8')
    sysfs_root = tmp_path / 'usb-devices'
    sysfs_root.mkdir()
    if create_camera:
        create_enumerated_camera(
            sysfs_root, interface_count, endpoint_addresses)
    environment = {
        **os.environ,
        'ASTRA_USB_PORT_DISABLE': str(port),
        'ASTRA_USB_INITIAL_WAIT_S': '0',
        'ASTRA_USB_PORT_OFF_S': '0',
        'ASTRA_USB_RECOVERY_WAIT_S': '0',
        'ASTRA_USB_RETRY_COUNT': '1',
        'LSUSB_BIN': lsusb_bin,
        'SLEEP_BIN': '/bin/true',
        'ASTRA_USB_FORCE_RECOVERY': '0',
        'ASTRA_USB_SYSFS_ROOT': str(sysfs_root),
        **overrides,
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
    assert 'USB enumeration already valid' in result.stdout


def test_lsusb_name_without_sysfs_descriptors_is_not_accepted(tmp_path):
    """A matching lsusb line alone does not prove usable USB enumeration."""
    result, port_state = run_recovery(
        tmp_path, '/bin/true', create_camera=False)

    assert result.returncode == 1
    assert port_state == '0'
    assert 'recovery failed' in result.stderr


def test_zero_interface_descriptor_is_not_accepted(tmp_path):
    """Reject the boot failure that exposed bNumInterfaces as zero."""
    result, port_state = run_recovery(
        tmp_path, '/bin/true', interface_count='0')

    assert result.returncode == 1
    assert port_state == '0'


def test_incomplete_vendor_depth_interface_is_not_accepted(tmp_path):
    """Require the two Bulk IN endpoints observed on Astra depth USB."""
    result, port_state = run_recovery(
        tmp_path, '/bin/true', endpoint_addresses=('81',))

    assert result.returncode == 1
    assert port_state == '0'


def test_recovery_rechecks_full_enumeration_contract(tmp_path):
    """Accept recovery only after descriptors and depth endpoints appear."""
    port = tmp_path / 'disable'
    port.write_text('unchanged', encoding='utf-8')
    sysfs_root = tmp_path / 'usb-devices'
    sysfs_root.mkdir()
    helper = tmp_path / 'enumerate-camera'
    helper.write_text(
        '#!/bin/bash\n'
        'root="$FAKE_SYSFS_ROOT"\n'
        'device="$root/1-1.2"\n'
        'mkdir -p "$device" "$root/1-1.2:1.0/ep_81" '
        '"$root/1-1.2:1.0/ep_82"\n'
        'printf 2bc5 > "$device/idVendor"\n'
        'printf 0402 > "$device/idProduct"\n'
        'printf 1 > "$device/authorized"\n'
        'printf 1 > "$device/bConfigurationValue"\n'
        'printf 1 > "$device/bNumConfigurations"\n'
        'printf 3 > "$device/bNumInterfaces"\n'
        'printf descriptors > "$device/descriptors"\n'
        'printf ff > "$root/1-1.2:1.0/bInterfaceClass"\n'
        'printf 02 > "$root/1-1.2:1.0/bNumEndpoints"\n'
        'printf Bulk > "$root/1-1.2:1.0/ep_81/type"\n'
        'printf in > "$root/1-1.2:1.0/ep_81/direction"\n'
        'printf Bulk > "$root/1-1.2:1.0/ep_82/type"\n'
        'printf in > "$root/1-1.2:1.0/ep_82/direction"\n',
        encoding='utf-8')
    helper.chmod(0o755)
    environment = {
        **os.environ,
        'ASTRA_USB_PORT_DISABLE': str(port),
        'ASTRA_USB_INITIAL_WAIT_S': '0',
        'ASTRA_USB_PORT_OFF_S': '0',
        'ASTRA_USB_RECOVERY_WAIT_S': '0',
        'ASTRA_USB_RETRY_COUNT': '1',
        'ASTRA_USB_FORCE_RECOVERY': '0',
        'ASTRA_USB_SYSFS_ROOT': str(sysfs_root),
        'LSUSB_BIN': '/bin/true',
        'SLEEP_BIN': str(helper),
        'FAKE_SYSFS_ROOT': str(sysfs_root),
    }
    result = subprocess.run(
        [str(SCRIPT)], env=environment, capture_output=True, text=True,
        timeout=5, check=False)

    assert result.returncode == 0
    assert port.read_text(encoding='utf-8') == '0'
    assert 'USB enumeration recovered' in result.stdout


def test_explicit_recovery_cycles_enumerated_camera(tmp_path):
    result, port_state = run_recovery(
        tmp_path, '/bin/true', ASTRA_USB_FORCE_RECOVERY='1')
    assert result.returncode == 0
    assert port_state == '0'
    assert 'recovery attempt 1/1' in result.stdout


def test_invalid_recovery_mode_leaves_port_untouched(tmp_path):
    result, port_state = run_recovery(
        tmp_path, '/bin/true', ASTRA_USB_FORCE_RECOVERY='yes')
    assert result.returncode == 2
    assert port_state == 'unchanged'


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


def test_camera_service_propagates_one_failure_without_retrying():
    """Expose a launch child crash and wait for an explicit next start."""
    camera = CAMERA_UNIT.read_text(encoding='utf-8')

    assert 'jdamr-launch-supervisor -- ros2 launch astra_camera' in camera
    assert 'Restart=no' in camera
    assert 'Restart=on-failure' not in camera
    assert 'RestartSec=' not in camera
    assert 'KillMode=mixed' in camera
    assert 'TimeoutStopSec=10' in camera


def test_camera_service_preserves_depth_only_runtime_arguments():
    """Keep the measured low-load Astra stream configuration unchanged."""
    camera = CAMERA_UNIT.read_text(encoding='utf-8')

    expected_arguments = (
        'enable_color:=false',
        'enable_depth:=true',
        'enable_ir:=false',
        'enable_point_cloud:=false',
        'enable_colored_point_cloud:=false',
        'depth_registration:=true',
        'color_depth_synchronization:=false',
        'tf_publish_rate:=0.0',
        'depth_width:=320',
        'depth_height:=240',
        'depth_fps:=30',
    )
    assert all(argument in camera for argument in expected_arguments)
