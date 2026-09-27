"""Validate Astra boot recovery packaging and safe local installation."""

import os
from pathlib import Path
import runpy
import shutil
import subprocess

import setuptools


PACKAGE = Path(__file__).resolve().parents[1]
INSTALLER = PACKAGE / 'scripts/install_astra_boot_recovery.sh'


def collect_data_files(monkeypatch):
    """Load setup.py and return the data_files passed to setuptools."""
    captured = {}
    monkeypatch.chdir(PACKAGE)
    monkeypatch.setattr(
        setuptools, 'setup', lambda **kwargs: captured.update(kwargs))

    runpy.run_path(str(PACKAGE / 'setup.py'), run_name='__main__')
    return captured['data_files']


def test_setup_packages_astra_install_assets(monkeypatch):
    """Install both scripts and preserve the systemd drop-in directory."""
    data_files = collect_data_files(monkeypatch)
    installed = {
        (destination, Path(source).as_posix())
        for destination, sources in data_files
        for source in sources
    }
    installed_sources = {source for _, source in installed}
    expected_sources = {
        path.relative_to(PACKAGE).as_posix()
        for directory in ('scripts', 'systemd')
        for path in (PACKAGE / directory).rglob('*')
        if path.is_file()
    }

    assert expected_sources <= installed_sources

    assert (
        'lib/jdamr_cube_bringup',
        'scripts/jdamr-astra-usb-recover',
    ) in installed
    assert (
        'lib/jdamr_cube_bringup',
        'scripts/install_astra_boot_recovery.sh',
    ) in installed
    assert (
        'share/jdamr_cube_bringup/systemd',
        'systemd/jdamr-astra-usb-recover.service',
    ) in installed
    assert (
        'share/jdamr_cube_bringup/systemd/jdamr-base.service.d',
        'systemd/jdamr-base.service.d/15-astra-usb-recover.conf',
    ) in installed


def make_mock_command(tmp_path, name, body):
    """Create an absolute executable command used by the shell installer."""
    command = tmp_path / name
    command.write_text('#!/bin/bash\nset -eu\n' + body, encoding='utf-8')
    command.chmod(0o755)
    return command


def run_installer(
        tmp_path, systemctl_body=None, extra_env=None, installer=INSTALLER):
    """Run the installer with non-privileged command mocks."""
    log = tmp_path / 'commands.log'
    sudo = make_mock_command(
        tmp_path, 'sudo',
        'printf \'sudo:%s\\n\' "$*" >>"$COMMAND_LOG"\n'
        'test "$1" = "-n"\nshift\nexec "$@"\n')
    install = make_mock_command(
        tmp_path, 'install',
        'printf \'install:%s\\n\' "$*" >>"$COMMAND_LOG"\n'
        'exec /usr/bin/install "$@"\n')
    systemctl = make_mock_command(
        tmp_path, 'systemctl',
        systemctl_body or
        'printf \'systemctl:%s\\n\' "$*" >>"$COMMAND_LOG"\n')
    environment = {
        **os.environ,
        'ASTRA_INSTALL_ROOT': str(tmp_path / 'root'),
        'SUDO_BIN': str(sudo),
        'INSTALL_BIN': str(install),
        'SYSTEMCTL_BIN': str(systemctl),
        'COMMAND_LOG': str(log),
        **(extra_env or {}),
    }
    result = subprocess.run(
        [str(installer)], env=environment, capture_output=True, text=True,
        timeout=5, check=False)
    return result, log


def test_installer_only_installs_and_enables_recovery(tmp_path):
    """Install the three assets without touching service or USB state."""
    usb_port = tmp_path / 'usb-port-disable'
    usb_port.write_text('unchanged', encoding='utf-8')
    result, log = run_installer(
        tmp_path,
        extra_env={
            'ASTRA_USB_PORT_DISABLE': str(usb_port),
            'ASTRA_USB_INITIAL_WAIT_S': '0',
            'ASTRA_USB_PORT_OFF_S': '0',
            'ASTRA_USB_RECOVERY_WAIT_S': '0',
            'ASTRA_USB_RETRY_COUNT': '1',
            'LSUSB_BIN': '/bin/false',
            'SLEEP_BIN': '/bin/true',
        })

    assert result.returncode == 0, result.stderr
    root = tmp_path / 'root'
    assert (root / 'usr/local/sbin/jdamr-astra-usb-recover').read_bytes() == (
        PACKAGE / 'scripts/jdamr-astra-usb-recover').read_bytes()
    installed_unit = (
        root / 'etc/systemd/system/jdamr-astra-usb-recover.service')
    assert installed_unit.read_bytes() == (
        PACKAGE / 'systemd/jdamr-astra-usb-recover.service').read_bytes()
    assert (
        root
        / 'etc/systemd/system/jdamr-base.service.d'
        / '15-astra-usb-recover.conf'
    ).read_bytes() == (
        PACKAGE
        / 'systemd/jdamr-base.service.d'
        / '15-astra-usb-recover.conf'
    ).read_bytes()
    assert not (root / 'etc/systemd/system/jdamr-base.service').exists()
    assert usb_port.read_text(encoding='utf-8') == 'unchanged'

    commands = log.read_text(encoding='utf-8')
    assert 'systemctl:daemon-reload' in commands
    assert 'systemctl:enable jdamr-astra-usb-recover.service' in commands
    assert commands.count('sudo:-n ') == 5
    forbidden = (' --now', ' start ', ' stop ', ' restart ', 'disable')
    assert not any(token in f' {commands} ' for token in forbidden)


def test_installer_is_idempotent(tmp_path):
    """Repeated installation succeeds and leaves identical file content."""
    first, log = run_installer(tmp_path)
    first_commands = log.read_text(encoding='utf-8')
    second, log = run_installer(tmp_path)

    assert first.returncode == 0, first.stderr
    assert second.returncode == 0, second.stderr
    assert log.read_text(encoding='utf-8') == first_commands * 2


def test_installer_resolves_packaged_install_layout(tmp_path):
    """Resolve helpers under lib and systemd assets under share."""
    prefix = tmp_path / 'prefix'
    script_directory = prefix / 'lib/jdamr_cube_bringup'
    asset_directory = prefix / 'share/jdamr_cube_bringup/systemd'
    drop_in_directory = asset_directory / 'jdamr-base.service.d'
    script_directory.mkdir(parents=True)
    drop_in_directory.mkdir(parents=True)
    installed_installer = script_directory / INSTALLER.name
    shutil.copy2(INSTALLER, installed_installer)
    shutil.copy2(
        PACKAGE / 'scripts/jdamr-astra-usb-recover', script_directory)
    shutil.copy2(
        PACKAGE / 'systemd/jdamr-astra-usb-recover.service',
        asset_directory)
    shutil.copy2(
        PACKAGE
        / 'systemd/jdamr-base.service.d'
        / '15-astra-usb-recover.conf',
        drop_in_directory)

    result, _ = run_installer(
        tmp_path, installer=installed_installer)

    assert result.returncode == 0, result.stderr


def test_installer_stops_after_daemon_reload_failure(tmp_path):
    """Return nonzero and do not enable when daemon-reload fails."""
    result, log = run_installer(
        tmp_path,
        'printf \'systemctl:%s\\n\' "$*" >>"$COMMAND_LOG"\n'
        'test "$1" != "daemon-reload"\n')

    assert result.returncode != 0
    commands = log.read_text(encoding='utf-8')
    assert 'systemctl:daemon-reload' in commands
    assert 'systemctl:enable ' not in commands


def test_installer_rejects_untrusted_command_path(tmp_path):
    """Reject relative command overrides before attempting installation."""
    result, log = run_installer(
        tmp_path, extra_env={'INSTALL_BIN': 'relative-install'})

    assert result.returncode != 0
    assert 'absolute executable' in result.stderr
    assert not log.exists()
