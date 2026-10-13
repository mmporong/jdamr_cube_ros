"""Exercise launch child-failure propagation without ROS camera hardware."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


PACKAGE = Path(__file__).resolve().parents[1]
SUPERVISOR = PACKAGE / 'scripts/jdamr-launch-supervisor'


def make_child(tmp_path, source):
    """Create a Python child command used by the supervisor tests."""
    child = tmp_path / 'child.py'
    child.write_text(source, encoding='utf-8')
    return [sys.executable, str(child)]


def run_supervisor(command):
    """Run the supervisor with one isolated fake launch command."""
    return subprocess.run(
        [str(SUPERVISOR), '--', *command],
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )


def process_exists(pid):
    """Return whether a PID still has a live procfs entry."""
    return Path(f'/proc/{pid}').exists()


def wait_for_process_exit(pid, timeout_s=2.0):
    """Wait for one test process to be fully removed from procfs."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and process_exists(pid):
        time.sleep(0.02)
    return not process_exists(pid)


def make_process_tree_child(
        tmp_path, *, crash, ignore_signals=False, report_stop=False):
    """Create a fake launch process with a long-lived subprocess."""
    parent_pid = tmp_path / 'parent.pid'
    descendant_pid = tmp_path / 'descendant.pid'
    child = tmp_path / 'tree_child.py'
    signal_setup = ''
    if ignore_signals:
        signal_setup = (
            'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
            'signal.signal(signal.SIGTERM, signal.SIG_IGN)\n'
        )
    elif report_stop:
        signal_setup = (
            'def report_stop(signum, _frame):\n'
            '    print("[ERROR] [camera_node-1]: process has died '
            '[pid 3628, exit code {}, cmd \'camera_node\']."'
            '.format(-signum), flush=True)\n'
            '    raise SystemExit(128 + signum)\n'
            'signal.signal(signal.SIGINT, report_stop)\n'
            'signal.signal(signal.SIGTERM, report_stop)\n'
        )
    crash_output = ''
    if crash:
        crash_output = (
            'print("[ERROR] [camera_node-1]: process has died '
            '[pid 3628, exit code -11, cmd \'camera_node\'].", flush=True)\n'
        )
    child.write_text(
        'import os\n'
        'from pathlib import Path\n'
        'import signal\n'
        'import subprocess\n'
        'import sys\n'
        'import time\n'
        + signal_setup
        + 'descendant = subprocess.Popen([sys.executable, "-c", '
        '"import time; time.sleep(60)"])\n'
        + f'Path({str(parent_pid)!r}).write_text(str(os.getpid()))\n'
        + f'Path({str(descendant_pid)!r}).write_text(str(descendant.pid))\n'
        + crash_output
        + "print('ready', flush=True)\n"
        + 'time.sleep(60)\n',
        encoding='utf-8',
    )
    return [sys.executable, str(child)], parent_pid, descendant_pid


def read_pid(path):
    """Read a PID file produced before the fake launch reports readiness."""
    return int(path.read_text(encoding='utf-8'))


def test_clean_launch_exit_remains_successful(tmp_path):
    command = make_child(tmp_path, "print('camera ready')\n")

    result = run_supervisor(command)

    assert result.returncode == 0
    assert result.stdout == 'camera ready\n'
    assert result.stderr == ''


def test_reported_child_crash_makes_launch_unsuccessful(tmp_path):
    command = make_child(
        tmp_path,
        'print("[ERROR] [camera_node-1]: process has died '
        '[pid 3628, exit code -11, cmd \'camera_node\'].")\n',
    )

    result = run_supervisor(command)

    assert result.returncode == 1
    assert 'process has died' in result.stdout
    assert 'child-process failure: -11' in result.stderr


def test_crash_stops_long_lived_launch_tree_without_waiting_for_eof(tmp_path):
    command, parent_path, descendant_path = make_process_tree_child(
        tmp_path, crash=True)
    started = time.monotonic()

    result = run_supervisor(command)

    elapsed = time.monotonic() - started
    parent_pid = read_pid(parent_path)
    descendant_pid = read_pid(descendant_path)
    assert result.returncode == 1
    assert elapsed < 8.0
    assert wait_for_process_exit(parent_pid)
    assert wait_for_process_exit(descendant_pid)


def test_crash_is_not_hidden_by_later_operator_signal(tmp_path):
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal_dir = tmp_path / str(signum)
        signal_dir.mkdir()
        command, parent_path, descendant_path = make_process_tree_child(
            signal_dir, crash=True, ignore_signals=True)
        process = subprocess.Popen(
            [str(SUPERVISOR), '--', *command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        assert process.stdout is not None
        assert 'process has died' in process.stdout.readline()

        process.send_signal(signum)
        stdout, stderr = process.communicate(timeout=8)

        assert process.returncode == 1
        assert 'child-process failure: -11' in stderr
        assert wait_for_process_exit(read_pid(parent_path))
        assert wait_for_process_exit(read_pid(descendant_path))


def test_operator_stop_cleans_launch_tree_and_remains_successful(tmp_path):
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal_dir = tmp_path / str(signum)
        signal_dir.mkdir()
        command, parent_path, descendant_path = make_process_tree_child(
            signal_dir, crash=False, report_stop=True)
        process = subprocess.Popen(
            [str(SUPERVISOR), '--', *command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=os.environ,
        )
        assert process.stdout is not None
        assert process.stdout.readline() == 'ready\n'

        process.send_signal(signum)
        stdout, stderr = process.communicate(timeout=8)

        assert process.returncode == 0
        assert 'process has died' in stdout
        assert stderr == ''
        assert wait_for_process_exit(read_pid(parent_path))
        assert wait_for_process_exit(read_pid(descendant_path))


def test_launch_command_failure_code_is_preserved(tmp_path):
    command = make_child(tmp_path, 'raise SystemExit(7)\n')

    result = run_supervisor(command)

    assert result.returncode == 7


def test_missing_launch_command_returns_127(tmp_path):
    result = run_supervisor([str(tmp_path / 'missing-command')])

    assert result.returncode == 127
