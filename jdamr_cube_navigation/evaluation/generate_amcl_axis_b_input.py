#!/usr/bin/env python3
"""Generate one canonical G002 Axis B localization replay input."""

from __future__ import annotations

import argparse
import hashlib
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile
import time

from amcl_fault_contract import (
    canonical_json_bytes,
    sha256_file,
    strict_json_load,
)
from axis_b_kidnapped_driver import (
    GT_TRANSLATION_TOLERANCE_M,
    GT_YAW_TOLERANCE_RAD,
    ROTATION_CRUISE_RADPS,
    ROTATION_MIN_RADPS,
    ROTATION_SLOW_ZONE_RAD,
    TELEPORT_POSE,
    validate_kidnapped_trace,
)
from g002_tf_sanitizer import _canonical_tf_serialization, _tf_record
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.serialization import deserialize_message, serialize_message
from ros_gz_interfaces.msg import Contacts
import rosbag2_py
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


SCENARIOS = {
    'correct_init': {
        'true_start_pose': [-8.0, 0.0, 0.0],
        'initial_estimate_offset': [0.0, 0.0, 0.0],
        'teleport_pose': None,
        'observation_rotation_rad': 6.283185307179586},
    'initial_offset': {
        'true_start_pose': [-8.0, 0.0, 0.0],
        'initial_estimate_offset': [0.5, 0.0, 0.2617993877991494],
        'teleport_pose': None,
        'observation_rotation_rad': 6.283185307179586},
    'kidnapped': {
        'true_start_pose': [-8.0, 0.0, 0.0],
        'initial_estimate_offset': [0.0, 0.0, 0.0],
        'teleport_pose': [0.0, 0.0, 3.141592653589793],
        'observation_rotation_rad': 6.283185307179586},
}
CONTACT_TOPIC = (
    '/world/slam_corridor/model/g003_preloaded_front_observation_probe/'
    'link/body/sensor/contact_sensor/contact')
SOURCE_TOPICS = {
    '/sim_raw/scan': '/scan', '/odom': '/odom', '/tf': '/tf',
    '/tf_static': '/tf_static', '/ground_truth_pose': '/ground_truth_pose',
    '/cmd_vel': '/cmd_vel',
    CONTACT_TOPIC: '/contact',
}
GENERATION_REQUEST_SCHEMA_VERSION = 1
OBSERVATION_ANGULAR_SPEED_RADPS = 0.5
OBSERVATION_ZERO_HOLD_S = 1.0
OBSERVATION_TIMEOUT_S = 60.0
LIDAR_RATE_HZ = 10.0
LIDAR_PERIOD_NS = 100_000_000
LIDAR_ALLOWED_MAX_GAP_NS = 2 * LIDAR_PERIOD_NS


def _parse_bindings(values: list[str]) -> dict[str, Path]:
    result = {}
    for value in values:
        if '=' not in value:
            raise ValueError('capture binding must be scenario=/absolute/root')
        scenario, raw_path = value.split('=', 1)
        path = Path(raw_path)
        if scenario in result or scenario not in SCENARIOS or not path.is_absolute():
            raise ValueError('Axis B capture binding drift')
        result[scenario] = path
    return result


def _g004_bridge_identity(runtime: dict) -> dict:
    """Load the exact evaluation bridge owned by the frozen G004 contract."""
    contract_path = Path(runtime['g004_contract']['path'])
    if runtime['g004_contract'] != _identity(contract_path):
        raise ValueError('G004 contract identity drift while loading bridge')
    contract = strict_json_load(contract_path)
    try:
        record = contract['evaluation_assets']['bridge']
    except (KeyError, TypeError) as exc:
        raise ValueError('frozen G004 bridge is absent') from exc
    expected_keys = {
        'path', 'size_bytes', 'sha256', 'source_path',
        'source_size_bytes', 'source_sha256'}
    if type(record) is not dict or set(record) != expected_keys:
        raise ValueError('frozen G004 bridge identity drift')
    path = Path(record['path'])
    if (path.parent != contract_path.parent / 'assets' or
            path.name != 'collision_monitor_bridge.yaml' or
            not _identity_matches(
                {key: record[key] for key in ('path', 'size_bytes', 'sha256')},
                path)):
        raise ValueError('frozen G004 bridge identity drift')
    return record


def generation_request(capture_roots: dict[str, Path]) -> dict:
    """Create a forced-observation request for three Gazebo captures."""
    if set(capture_roots) != set(SCENARIOS):
        raise ValueError('generation request requires all three scenarios')
    from prepare_amcl_fault_benchmark import G004_ROOT, _axis_b_identity
    runtime = _axis_b_identity(G004_ROOT)
    bridge = _g004_bridge_identity(runtime)
    rows = []
    for index, scenario in enumerate(SCENARIOS):
        root = capture_roots[scenario]
        if (not root.is_absolute() or root != root.resolve() or
                root.name != 'bag'):
            raise ValueError(
                'capture root must be an absolute canonical bag directory')
        commands = {
            'gazebo': [
                '/opt/ros/jazzy/bin/ros2', 'launch', 'jdamr_cube_gazebo',
                'gazebo.launch.py', f'world:={runtime["assets"]["world"]["path"]}',
                f'urdf_file:={runtime["assets"]["urdf"]["path"]}',
                f'bridge_config:={bridge["path"]}',
                'enable_image_bridges:=false',
                'robot_state_publisher_respawn:=false',
                'seed:=11', 'gui:=false', 'x_pose:=-8.0', 'y_pose:=0.0',
                'z_pose:=0.01'],
            'contact_bridge': [
                '/opt/ros/jazzy/bin/ros2', 'run', 'ros_gz_bridge',
                'parameter_bridge',
                f'{CONTACT_TOPIC}@ros_gz_interfaces/msg/Contacts'
                '[gz.msgs.Contacts'],
            'contact_probe': [
                '/opt/ros/jazzy/bin/ros2', 'topic', 'info', '--verbose',
                CONTACT_TOPIC],
            'recorder': [
                '/opt/ros/jazzy/bin/ros2', 'bag', 'record', '--storage', 'mcap',
                '--use-sim-time', '--output', str(root), '--topics',
                *sorted(SOURCE_TOPICS)],
            'scenario_driver': ([
                '/usr/bin/python3', str(Path(__file__).with_name(
                    'axis_b_kidnapped_driver.py').resolve()), '--evidence',
                str(root / 'kidnapped_trace.json')]
                if scenario == 'kidnapped' else [
                    '/usr/bin/python3', str(Path(__file__).resolve()),
                    '--capture-driver', scenario, '--domain-id',
                    str(150 + index)]),
        }
        rows.append({
            'scenario': scenario,
            'capture_root': str(root),
            'scenario_contract': SCENARIOS[scenario],
            'gazebo_seed': 11,
            'ros_domain_id': 150 + index,
            'g004_contract': runtime['g004_contract'],
            'world': runtime['assets']['world'],
            'urdf': runtime['assets']['urdf'],
            'bridge': bridge,
            'lidar_profile': runtime['lidar_profile'],
            'runner_source': _generator_source_identity(),
            'expected_commands': commands,
            'gazebo_runtime_capture_required': True,
            'kidnapped_driver_required': scenario == 'kidnapped',
            'expected_topics': sorted(SOURCE_TOPICS),
        })
    return {
        'schema_version': GENERATION_REQUEST_SCHEMA_VERSION,
        'claim_scope': 'DETERMINISTIC_GAZEBO_INPUT_CAPTURE_REQUEST_NO_EXECUTION',
        'status': 'PENDING_GAZEBO_CAPTURE',
        'executes_ros_or_gazebo': False,
        'synthetic_bag_canonical_eligible': False,
        'requests': rows,
    }


def validate_generation_request(
        value: dict, expected_runner_source: dict | None = None) -> dict:
    """Reject incomplete or synthetic canonical-input requests."""
    expected = {'schema_version', 'claim_scope', 'status',
                'executes_ros_or_gazebo', 'synthetic_bag_canonical_eligible',
                'requests'}
    if type(value) is not dict or set(value) != expected:
        raise ValueError('Axis B generation request schema drift')
    roots = {}
    if (value['schema_version'] != GENERATION_REQUEST_SCHEMA_VERSION or
            value['claim_scope'] !=
            'DETERMINISTIC_GAZEBO_INPUT_CAPTURE_REQUEST_NO_EXECUTION' or
            value['status'] != 'PENDING_GAZEBO_CAPTURE' or
            value['executes_ros_or_gazebo'] is not False or
            value['synthetic_bag_canonical_eligible'] is not False or
            type(value['requests']) is not list or
            len(value['requests']) != len(SCENARIOS)):
        raise ValueError('Axis B generation request scalar drift')
    for row, scenario in zip(value['requests'], SCENARIOS):
        if type(row) is not dict or set(row) != {
                'scenario', 'capture_root', 'scenario_contract',
                'gazebo_seed', 'ros_domain_id', 'g004_contract', 'world',
                'urdf', 'bridge', 'lidar_profile', 'runner_source',
                'expected_commands',
                'gazebo_runtime_capture_required',
                'kidnapped_driver_required', 'expected_topics'}:
            raise ValueError('Axis B generation row schema drift')
        root = Path(row['capture_root'])
        expected_row = generation_request(
            {name: Path(item['capture_root']) for name, item in
             zip(SCENARIOS, value['requests'])})['requests'][len(roots)]
        if expected_runner_source is not None:
            expected_row['runner_source'] = expected_runner_source
        if (row != expected_row or row['scenario'] != scenario or root in roots or
                not root.is_absolute() or root != root.resolve() or
                row['scenario_contract'] != SCENARIOS[scenario] or
                row['gazebo_runtime_capture_required'] is not True or
                row['kidnapped_driver_required'] is not
                (scenario == 'kidnapped') or
                row['expected_topics'] != sorted(SOURCE_TOPICS)):
            raise ValueError('Axis B generation row drift')
        roots[root] = scenario
    return value


def write_generation_request(path: Path,
                             capture_roots: dict[str, Path]) -> dict:
    """Atomically persist a validated no-execution generation request."""
    if not path.is_absolute() or path.exists() or path.parent != path.parent.resolve():
        raise ValueError('request output must be absent and canonical')
    value = validate_generation_request(generation_request(capture_roots))
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(canonical_json_bytes(value))
    temporary.replace(path)
    return validate_generation_request(strict_json_load(path))


def validate_capture_attestation(
        path: Path, source_root: Path, scenario: str,
        expected_runner_source: dict | None = None) -> dict:
    """Require direct Gazebo capture evidence; summaries are not inputs."""
    value = strict_json_load(path)
    expected = {
        'schema_version', 'status', 'execution_mode', 'scenario',
        'source_mcap', 'source_metadata', 'generation_request',
        'synthetic_input', 'rosbag_record_exit_code', 'gazebo_exit_code',
        'survivor_count', 'runner_source', 'gazebo_seed', 'ros_domain_id',
        'g004_contract', 'world', 'urdf', 'bridge', 'lidar_profile', 'processes',
        'contact_endpoint_probe',
        'atomic_finalized', 'executor_receipt', 'claim_scope'}
    if type(value) is not dict or set(value) != expected:
        raise ValueError('Axis B capture attestation schema drift')
    request_path = Path(value['generation_request']['path']) \
        if type(value['generation_request']) is dict else Path()
    if (value['schema_version'] != 1 or value['status'] != 'PASS' or
            value['execution_mode'] != 'GAZEBO_RUNTIME_CAPTURE' or
            value['scenario'] != scenario or value['synthetic_input'] is not False or
            value['claim_scope'] !=
            'RUNNER_SELF_ATTESTED_OS_EXECUTION_NOT_EXTERNALLY_SIGNED' or
            value['rosbag_record_exit_code'] not in (0, -signal.SIGINT) or
            value['gazebo_exit_code'] not in (0, -signal.SIGINT) or
            value['survivor_count'] != 0 or
            value['atomic_finalized'] is not True or
            not _identity_matches(value['source_mcap'],
                                  source_root / 'bag_0.mcap') or
            not _identity_matches(value['source_metadata'],
                                  source_root / 'metadata.yaml') or
            not _identity_matches(value['generation_request'], request_path)):
        raise ValueError('Axis B capture attestation gate failed')
    request = validate_generation_request(
        strict_json_load(request_path), expected_runner_source)
    row = next((item for item in request['requests']
                if item['scenario'] == scenario), None)
    if row is None or Path(row['capture_root']) != source_root:
        raise ValueError('Axis B capture request binding drift')
    for key in ('runner_source', 'gazebo_seed', 'ros_domain_id',
                'g004_contract', 'world', 'urdf', 'bridge', 'lidar_profile'):
        if value[key] != row[key]:
            raise ValueError(f'Axis B capture runtime drift: {key}')
    if (type(value['processes']) is not list or
            [item.get('name') for item in value['processes']] !=
            ['gazebo', 'contact_bridge', 'recorder', 'scenario_driver']):
        raise ValueError('Axis B capture process inventory drift')
    for process in value['processes']:
        if (type(process) is not dict or set(process) != {
                'name', 'command', 'pid', 'returncode', 'survivors', 'log',
                'started_steady_ns', 'finished_steady_ns', 'environment',
                'pid_starttime_ticks', 'wait_returncode', 'stop_stages'} or
                type(process['command']) is not list or
                not process['command'] or
                any(type(item) is not str for item in process['command']) or
                type(process['pid']) is not int or process['pid'] <= 0 or
                type(process['pid_starttime_ticks']) is not int or
                process['pid_starttime_ticks'] <= 0 or
                process['wait_returncode'] != process['returncode'] or
                type(process['started_steady_ns']) is not int or
                type(process['finished_steady_ns']) is not int or
                process['started_steady_ns'] >= process['finished_steady_ns']):
            raise ValueError('Axis B capture process evidence drift')
        if process['environment'] != {
                'ROS_DOMAIN_ID': str(row['ros_domain_id']),
                'ROS_LOCALHOST_ONLY': '1',
                'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}:
            raise ValueError('Axis B capture process environment drift')
        log_path = Path(process['log']['path']) \
            if type(process['log']) is dict else Path()
        if not _identity_matches(process['log'], log_path):
            raise ValueError('Axis B capture process log identity drift')
        gazebo_log_classification = (
            _gazebo_launch_log_classification(log_path)
            if process['name'] == 'gazebo' else None)
        if not _capture_stop_valid(process, gazebo_log_classification):
            raise ValueError('Axis B capture graceful stop evidence drift')
        if process['command'] != row['expected_commands'][process['name']]:
            raise ValueError('Axis B capture process command drift')
    probe = value['contact_endpoint_probe']
    if (type(probe) is not dict or set(probe) != {
            'command', 'returncode', 'publisher_count', 'node_names',
            'output'} or
            probe['command'] != row['expected_commands']['contact_probe'] or
            probe['returncode'] != 0 or probe['publisher_count'] != 1 or
            probe['node_names'] != ['ros_gz_bridge'] or
            not _identity_matches(probe['output'], Path(probe['output']['path']))):
        raise ValueError('Axis B contact endpoint probe drift')
    processes = value['processes']
    if (len({item['pid'] for item in processes}) != len(processes) or
            len({item['log']['path'] for item in processes}) != len(processes) or
            any(left['started_steady_ns'] >= right['started_steady_ns']
                for left, right in zip(processes, processes[1:])) or
            not all(left['finished_steady_ns'] > right['finished_steady_ns']
                    for left, right in zip(processes, processes[1:]))):
        raise ValueError('Axis B capture process chronology drift')
    by_name = {process['name']: process for process in processes}
    if (value['rosbag_record_exit_code'] !=
            by_name['recorder']['returncode'] or
            value['gazebo_exit_code'] != by_name['gazebo']['returncode']):
        raise ValueError('Axis B capture top-level exit code drift')
    receipt_path = Path(value['executor_receipt']['path']) \
        if type(value['executor_receipt']) is dict else Path()
    if not _identity_matches(value['executor_receipt'], receipt_path):
        raise ValueError('Axis B capture executor receipt identity drift')
    receipt = strict_json_load(receipt_path)
    fingerprint = hashlib.sha256(canonical_json_bytes({
        'processes': value['processes'],
        'contact_endpoint_probe': value['contact_endpoint_probe'],
        'source_mcap': value['source_mcap'],
        'source_metadata': value['source_metadata']})).hexdigest()
    if receipt != {
            'schema_version': 1,
            'claim_scope':
            'RUNNER_SELF_ATTESTED_OS_EXECUTION_NOT_EXTERNALLY_SIGNED',
            'runner_source': row['runner_source'],
            'capture_fingerprint_sha256': fingerprint}:
        raise ValueError('Axis B capture executor receipt drift')
    return value


def _stop_stages_exact(stages, expected_signals: list[str]) -> bool:
    return (
        type(stages) is list and len(stages) == len(expected_signals) and
        all(type(stage) is dict and
            set(stage) == {'signal', 'steady_ns'} and
            stage['signal'] == expected_signal and
            type(stage['steady_ns']) is int and stage['steady_ns'] > 0
            for stage, expected_signal in zip(stages, expected_signals)) and
        all(left['steady_ns'] < right['steady_ns']
            for left, right in zip(stages, stages[1:])))


def _capture_stop_valid(process: dict,
                        gazebo_log_classification: str | None = None) -> bool:
    """Bind each wrapper outcome to its exact shutdown evidence."""
    if process.get('survivors') != []:
        return False
    if process.get('name') == 'scenario_driver':
        return (process.get('returncode') == 0 and
                process.get('stop_stages') == [])
    if process.get('name') == 'recorder':
        return (
            process.get('returncode') in (0, -signal.SIGINT) and
            _stop_stages_exact(
                process.get('stop_stages'), [signal.SIGINT.name]))
    if process.get('name') == 'contact_bridge':
        graceful = (
            process.get('returncode') in (0, -signal.SIGINT) and
            _stop_stages_exact(
                process.get('stop_stages'), [signal.SIGINT.name]))
        supervised = (
            process.get('returncode') == -signal.SIGTERM and
            _stop_stages_exact(process.get('stop_stages'), [
                signal.SIGINT.name, signal.SIGTERM.name]))
        return graceful or supervised
    if process.get('name') != 'gazebo':
        return False
    if gazebo_log_classification == GAZEBO_LOG_GRACEFUL:
        return (
            process.get('returncode') in (0, -signal.SIGINT) and
            _stop_stages_exact(
                process.get('stop_stages'), [signal.SIGINT.name]))
    return (
        gazebo_log_classification == GAZEBO_LOG_SUPERVISOR_ORPHAN_CLEANUP and
        process.get('returncode') == 0 and
        _stop_stages_exact(process.get('stop_stages'), [
            signal.SIGINT.name, signal.SIGTERM.name]))


_LAUNCH_PROCESS_DIED = re.compile(
    r'\[ERROR\] \[([^\]]+)\]: process has died '
    r'\[[^\n]*exit code (-?\d+),')
_LAUNCH_SIGINT_TIMEOUT = re.compile(
    r"\[ERROR\] \[([^\]]+)\]: process\[\1\] failed to terminate '5' "
    r"seconds after receiving 'SIGINT', escalating to 'SIGTERM'")
_LAUNCH_SEND_SIGTERM = re.compile(
    r"\[INFO\] \[([^\]]+)\]: sending signal 'SIGTERM' to process\[\1\]")
_ALLOCATOR_FAILURE_MARKERS = (
    'malloc():', 'corrupted double-linked list')
GAZEBO_LOG_GRACEFUL = 'GRACEFUL'
GAZEBO_LOG_SUPERVISOR_ORPHAN_CLEANUP = 'SUPERVISOR_ORPHAN_CLEANUP'


def _gazebo_launch_log_classification(path: Path) -> str | None:
    """Classify only healthy or exact supervisor-orphan shutdown logs."""
    text = path.read_text(encoding='utf-8', errors='replace')
    if ('[FATAL]' in text or
            any(marker in text for marker in _ALLOCATOR_FAILURE_MARKERS)):
        return None
    timeout_positions = {
        match.group(1): match.start()
        for match in _LAUNCH_SIGINT_TIMEOUT.finditer(text)}
    send_sigterm_positions = {
        match.group(1): match.start()
        for match in _LAUNCH_SEND_SIGTERM.finditer(text)}
    supervisor_sigterm_deaths = set()
    graceful_sigint_deaths = set()
    for match in _LAUNCH_PROCESS_DIED.finditer(text):
        process_name, raw_returncode = match.groups()
        returncode = int(raw_returncode)
        if not process_name.startswith('gazebo-'):
            return None
        if returncode == -signal.SIGINT:
            graceful_sigint_deaths.add(process_name)
            continue
        timeout_index = timeout_positions.get(process_name, -1)
        send_index = send_sigterm_positions.get(process_name, -1)
        if (returncode != -signal.SIGTERM or timeout_index < 0 or
                not timeout_index < send_index < match.start()):
            return None
        supervisor_sigterm_deaths.add(process_name)
    if (supervisor_sigterm_deaths and not graceful_sigint_deaths and
            set(timeout_positions) == set(send_sigterm_positions) ==
            supervisor_sigterm_deaths):
        return GAZEBO_LOG_SUPERVISOR_ORPHAN_CLEANUP
    if (not supervisor_sigterm_deaths and not timeout_positions and
            not send_sigterm_positions):
        return GAZEBO_LOG_GRACEFUL
    return None


def _gazebo_launch_log_valid(path: Path) -> bool:
    """Return whether a Gazebo launch log has a recognized clean outcome."""
    return _gazebo_launch_log_classification(path) is not None


def _capture_teardown_diagnostic(stopped: list[dict]) -> list[dict]:
    """Return stable evidence for a rejected capture teardown."""
    by_name = {item['name']: item for item in stopped}
    return [{key: by_name[name][key] for key in (
        'name', 'returncode', 'stop_stages', 'survivors')}
        for name in ('gazebo', 'contact_bridge', 'recorder', 'scenario_driver')
        if name in by_name]


def _require_capture_teardown(stopped: list[dict],
                              gazebo_log_path: Path) -> dict:
    """Require graceful capture shutdown or raise with exact diagnostics."""
    by_name = {item['name']: item for item in stopped}
    if set(by_name) != {
            'gazebo', 'contact_bridge', 'recorder', 'scenario_driver'}:
        raise RuntimeError('Axis B capture process inventory failed')
    gazebo_log_classification = _gazebo_launch_log_classification(
        gazebo_log_path)
    if any(not _capture_stop_valid(
            item, gazebo_log_classification
            if item['name'] == 'gazebo' else None)
            for item in stopped):
        diagnostic = canonical_json_bytes(
            _capture_teardown_diagnostic(stopped)).decode('utf-8')
        raise RuntimeError(f'Axis B capture teardown failed: {diagnostic}')
    return by_name


def _write_capture_attestation(path: Path, value: dict,
                               source_root: Path, scenario: str) -> dict:
    """Atomically finalize and reopen one completed capture attestation."""
    if path.exists() or not path.is_absolute() or path.parent != path.parent.resolve():
        raise ValueError('capture attestation output drift')
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(canonical_json_bytes(value))
    temporary.replace(path)
    try:
        return validate_capture_attestation(path, source_root, scenario)
    except Exception:
        path.unlink(missing_ok=True)
        raise


def _proc_starttime_ticks(pid: int) -> int:
    """Read Linux process starttime before PID reuse can obscure identity."""
    text = Path(f'/proc/{pid}/stat').read_text(encoding='utf-8')
    value = int(text.rsplit(')', 1)[1].split()[19])
    if value <= 0:
        raise ValueError('capture process starttime drift')
    return value


def execute_capture(request_path: Path, scenario: str,
                    attestation_path: Path) -> dict:
    """Execute and attest the exact Gazebo, recorder, and driver processes."""
    import run_amcl_determinism_preflight as preflight
    request = validate_generation_request(strict_json_load(request_path))
    row = next((item for item in request['requests']
                if item['scenario'] == scenario), None)
    if row is None:
        raise ValueError('capture scenario is absent from request')
    source_root = Path(row['capture_root'])
    log_root = attestation_path.parent / f'{scenario}_capture_logs'
    if (source_root.exists() or log_root.exists() or attestation_path.exists() or
            not attestation_path.is_absolute()):
        raise ValueError('capture outputs must be absent and canonical')
    log_root.mkdir(parents=True)
    env = dict(os.environ)
    environment = {
        'ROS_DOMAIN_ID': str(row['ros_domain_id']),
        'ROS_LOCALHOST_ONLY': '1',
        'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}
    env.update(environment)
    launched = []
    try:
        launched.append(preflight._start(
            'gazebo', row['expected_commands']['gazebo'],
            log_root / 'gazebo.log', env))
        launched[-1]['pid_starttime_ticks'] = _proc_starttime_ticks(
            launched[-1]['pid'])
        time.sleep(5.0)
        launched.append(preflight._start(
            'contact_bridge', row['expected_commands']['contact_bridge'],
            log_root / 'contact_bridge.log', env))
        launched[-1]['pid_starttime_ticks'] = _proc_starttime_ticks(
            launched[-1]['pid'])
        time.sleep(2.0)
        probe_log = log_root / 'contact_endpoint_probe.log'
        completed = subprocess.run(
            row['expected_commands']['contact_probe'], env=env,
            capture_output=True, text=True, timeout=10.0, check=False)
        probe_log.write_text(completed.stdout + completed.stderr,
                             encoding='utf-8')
        publisher_match = re.search(
            r'Publisher count:\s*(\d+)', completed.stdout)
        node_names = sorted(set(re.findall(
            r'Node name:\s*([^\s]+)', completed.stdout)))
        if (completed.returncode != 0 or publisher_match is None or
                int(publisher_match.group(1)) != 1 or
                node_names != ['ros_gz_bridge']):
            raise RuntimeError('Axis B contact endpoint probe failed')
        launched.append(preflight._start(
            'recorder', row['expected_commands']['recorder'],
            log_root / 'recorder.log', env))
        launched[-1]['pid_starttime_ticks'] = _proc_starttime_ticks(
            launched[-1]['pid'])
        time.sleep(2.0)
        driver = preflight._start(
            'scenario_driver', row['expected_commands']['scenario_driver'],
            log_root / 'scenario_driver.log', env)
        driver['pid_starttime_ticks'] = _proc_starttime_ticks(driver['pid'])
        launched.append(driver)
        returncode = driver['process'].wait(timeout=360.0)
        if returncode != 0:
            raise RuntimeError('Axis B scenario driver failed')
    finally:
        stopped = []
        for item in reversed(launched):
            stopped_item = preflight._stop(item, log_root)
            stopped_item['pid_starttime_ticks'] = item['pid_starttime_ticks']
            stopped_item['finished_steady_ns'] = time.monotonic_ns()
            stopped.append(stopped_item)
    by_name = _require_capture_teardown(
        stopped, log_root / 'gazebo.log')
    processes = []
    for name in ('gazebo', 'contact_bridge', 'recorder', 'scenario_driver'):
        item = by_name[name]
        log_path = log_root / f'{name}.log'
        processes.append({
            'name': name, 'command': item['command'], 'pid': item['pid'],
            'returncode': item['returncode'], 'survivors': item['survivors'],
            'stop_stages': item['stop_stages'],
            'pid_starttime_ticks': item['pid_starttime_ticks'],
            'wait_returncode': item['returncode'],
            'log': _identity(log_path),
            'started_steady_ns': item['started']['steady_ns'],
            'finished_steady_ns': item['finished_steady_ns'],
            'environment': environment})
    value = {
        'schema_version': 1, 'status': 'PASS',
        'execution_mode': 'GAZEBO_RUNTIME_CAPTURE', 'scenario': scenario,
        'source_mcap': _identity(source_root / 'bag_0.mcap'),
        'source_metadata': _identity(source_root / 'metadata.yaml'),
        'generation_request': _identity(request_path),
        'synthetic_input': False,
        'rosbag_record_exit_code': by_name['recorder']['returncode'],
        'gazebo_exit_code': by_name['gazebo']['returncode'],
        'survivor_count': 0, 'runner_source': row['runner_source'],
        'gazebo_seed': row['gazebo_seed'],
        'ros_domain_id': row['ros_domain_id'],
        'g004_contract': row['g004_contract'], 'world': row['world'],
        'urdf': row['urdf'], 'bridge': row['bridge'],
        'lidar_profile': row['lidar_profile'],
        'contact_endpoint_probe': {
            'command': row['expected_commands']['contact_probe'],
            'returncode': completed.returncode,
            'publisher_count': int(publisher_match.group(1)),
            'node_names': node_names,
            'output': _identity(probe_log)},
        'processes': processes, 'atomic_finalized': True,
        'claim_scope':
        'RUNNER_SELF_ATTESTED_OS_EXECUTION_NOT_EXTERNALLY_SIGNED'}
    receipt_path = attestation_path.with_suffix('.executor_receipt.json')
    receipt = {
        'schema_version': 1,
        'claim_scope':
        'RUNNER_SELF_ATTESTED_OS_EXECUTION_NOT_EXTERNALLY_SIGNED',
        'runner_source': row['runner_source'],
        'capture_fingerprint_sha256': hashlib.sha256(canonical_json_bytes({
            'processes': processes,
            'contact_endpoint_probe': value['contact_endpoint_probe'],
            'source_mcap': value['source_mcap'],
            'source_metadata': value['source_metadata']})).hexdigest()}
    if receipt_path.exists():
        raise ValueError('capture executor receipt already exists')
    try:
        with tempfile.NamedTemporaryFile(
                dir=receipt_path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(canonical_json_bytes(receipt))
        temporary.replace(receipt_path)
        value['executor_receipt'] = _identity(receipt_path)
        return _write_capture_attestation(
            attestation_path, value, source_root, scenario)
    except Exception:
        receipt_path.unlink(missing_ok=True)
        raise


def _identity(path: Path) -> dict:
    if (not path.is_absolute() or path != path.resolve() or path.is_symlink() or
            not path.is_file()):
        raise ValueError('identity path must be a canonical regular file')
    return {'path': str(path), 'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path)}


def _output_identity(path: Path) -> dict:
    return {'name': path.name, 'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path)}


def _generator_source_identity() -> dict:
    return _identity(Path(__file__).resolve())


def _generator_source_valid(
        record: dict, expected_source: dict | None = None) -> bool:
    if type(record) is not dict or set(record) != {
            'path', 'size_bytes', 'sha256'}:
        return False
    path = Path(record['path']) if type(record['path']) is str else Path()
    if (not path.is_absolute() or path != path.resolve() or
            path.name != 'generate_amcl_axis_b_input.py' or
            type(record['size_bytes']) is not int or
            record['size_bytes'] < 0 or type(record['sha256']) is not str or
            not re.fullmatch(r'[0-9a-f]{64}', record['sha256'])):
        return False
    return record == (expected_source or _generator_source_identity())


def _source_from_snapshot(
        source: dict, snapshots: dict | None, filename: str) -> dict:
    """Resolve expected historical bytes without reading the live worktree."""
    if snapshots is None:
        path = Path(__file__).resolve().with_name(filename)
        return _identity(path)
    if type(snapshots) is not dict or filename not in snapshots:
        raise ValueError('Axis B source snapshot is absent')
    snapshot = snapshots[filename]
    if (type(snapshot) is not dict or set(snapshot) != {
            'relative_path', 'size_bytes', 'sha256'} or
            snapshot['relative_path'] != f'harness_sources/{filename}' or
            type(source) is not dict or set(source) != {
                'path', 'size_bytes', 'sha256'}):
        raise ValueError('Axis B source snapshot schema drift')
    path = Path(source['path']) if type(source['path']) is str else Path()
    if (not path.is_absolute() or path != path.resolve() or
            path.name != filename or
            type(snapshot['size_bytes']) is not int or
            snapshot['size_bytes'] < 0 or
            type(snapshot['sha256']) is not str or
            not re.fullmatch(r'[0-9a-f]{64}', snapshot['sha256'])):
        raise ValueError('Axis B source snapshot identity drift')
    expected = {'path': source['path'],
                'size_bytes': snapshot['size_bytes'],
                'sha256': snapshot['sha256']}
    if source != expected:
        raise ValueError('Axis B executed source snapshot binding drift')
    return expected


def _frozen_g004_contract_valid(record: dict) -> bool:
    """Bind input generation to the verified frozen G004 profile."""
    try:
        from prepare_amcl_fault_benchmark import _axis_b_identity
        actual = _axis_b_identity(Path(record['path']).parent)
        return record == actual['g004_contract']
    except (KeyError, OSError, TypeError, ValueError):
        return False


def _identity_matches(record: dict, path: Path, output: bool = False) -> bool:
    expected_path_key = 'name' if output else 'path'
    expected_path = path.name if output else str(path)
    canonical = (path.is_absolute() and path == path.resolve() and
                 not path.is_symlink() and path.is_file())
    return (canonical and type(record) is dict and set(record) == {
        expected_path_key, 'size_bytes', 'sha256'} and
        record[expected_path_key] == expected_path and
        type(record['size_bytes']) is int and
        record['size_bytes'] == path.stat().st_size and
        record['sha256'] == sha256_file(path))


def _source_projection(source_root: Path) -> dict:
    """Recompute the only allowed source-to-canonical bag transformation."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(source_root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    source_meta = {item.name: item
                   for item in reader.get_all_topics_and_types()}
    _require_source_topic_set(source_meta)
    if source_meta[CONTACT_TOPIC].type != 'ros_gz_interfaces/msg/Contacts':
        raise ValueError('Axis B contact source type drift')
    source_counts = {name: 0 for name in SOURCE_TOPICS}
    projected_counts = {name: 0 for name in SOURCE_TOPICS.values()}
    removed = 0
    digest = hashlib.sha256()
    while reader.has_next():
        topic, serialized, storage_ns = reader.read_next()
        if topic not in SOURCE_TOPICS:
            continue
        source_counts[topic] += 1
        output_topic = SOURCE_TOPICS[topic]
        payload = bytes(serialized)
        if topic == CONTACT_TOPIC:
            deserialize_message(payload, Contacts)
            raise ValueError('Axis B contact stream is not zero')
        if topic == '/tf':
            message = deserialize_message(payload, TFMessage)
            kept = []
            for transform in message.transforms:
                record = _tf_record(transform)
                if (record['parent'].lstrip('/') == 'map' and
                        record['child'].lstrip('/') == 'odom'):
                    removed += 1
                else:
                    kept.append(transform)
            message.transforms = kept
            payload = _canonical_tf_serialization(
                bytes(serialize_message(message)))
        projected_counts[output_topic] += 1
        digest.update(output_topic.encode() + b'\0')
        digest.update(str(storage_ns).encode() + b'\0' + payload)
    reader.close()
    return {'source_topic_counts': source_counts,
            'projected_topic_counts': projected_counts,
            'removed_map_to_odom_count': removed,
            'projected_ordered_payload_sha256': digest.hexdigest()}


def _require_source_topic_set(source_meta: dict) -> None:
    if set(source_meta) != set(SOURCE_TOPICS):
        raise ValueError('required Axis B source topic set drift')


def _yaw(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z +
               orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))


def _observation_motion_evidence(
        root: Path, scenario: str, scan_topic: str,
        expected_driver_source: dict | None = None) -> dict:
    """Recompute forced rotation and scan continuity from one bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    commands = []
    scans = []
    poses = []
    while reader.has_next():
        topic, serialized, storage_ns = reader.read_next()
        if topic == '/cmd_vel':
            command = deserialize_message(serialized, Twist)
            commands.append((storage_ns, float(command.angular.z)))
        elif topic == scan_topic:
            scan = deserialize_message(serialized, LaserScan)
            header_ns = (int(scan.header.stamp.sec) * 1_000_000_000 +
                         int(scan.header.stamp.nanosec))
            scans.append((storage_ns, header_ns))
        elif topic == '/ground_truth_pose':
            pose = deserialize_message(serialized, PoseStamped)
            poses.append((storage_ns, _yaw(pose.pose.orientation)))
    reader.close()
    window = None
    if scenario == 'kidnapped':
        manifest_path = root / 'axis_b_input_manifest.json'
        if manifest_path.is_file():
            manifest = strict_json_load(manifest_path)
            trace_path = Path(manifest['trace_evidence']['path'])
        else:
            trace_path = root / 'kidnapped_trace.json'
        trace = validate_kidnapped_trace(
            strict_json_load(trace_path), expected_driver_source)
        window = (
            trace['t0_scan_stamp_ns'],
            next(event['ros_ns'] for event in trace['events']
                 if event['name'] == 'rotation_complete'))
    return _motion_evidence_from_samples(
        commands, scans, poses, scenario, window)


def _motion_evidence_from_samples(commands: list[tuple[int, float]],
                                  scans: list[tuple[int, int]],
                                  poses: list[tuple[int, float]],
                                  scenario: str,
                                  window: tuple[int, int] | None = None
                                  ) -> dict:
    """Validate one actual-rotation window without trusting yaw wrapping."""
    nonzero = [(stamp, angular) for stamp, angular in commands
               if abs(angular) > 1e-9]
    if not nonzero:
        raise ValueError('Axis B observation has no angular cmd_vel motion')
    if window is not None:
        window_start_ns, window_end_ns = window
    else:
        window_start_ns = nonzero[0][0]
        window_end_ns = nonzero[-1][0]
    target_rotation_rad = SCENARIOS[scenario]['observation_rotation_rad']
    window_commands = [row for row in nonzero
                       if window_start_ns <= row[0] <= window_end_ns]
    window_scans = [row for row in scans
                    if window_start_ns <= row[0] <= window_end_ns]
    window_poses = [row for row in poses
                    if window_start_ns <= row[0] <= window_end_ns]
    if not window_commands or len(window_scans) < 2 or len(window_poses) < 2:
        raise ValueError('Axis B forced-rotation window is incomplete')
    if any(not math.isfinite(float(yaw)) for _, yaw in window_poses):
        raise ValueError('Axis B forced-rotation GT yaw is non-finite')
    scan_header_stamps = [row[1] for row in window_scans]
    unique_monotonic = (
        len(set(scan_header_stamps)) == len(scan_header_stamps) and
        all(left < right for left, right in
            zip(scan_header_stamps, scan_header_stamps[1:])))
    gaps = [right - left for left, right in
            zip(scan_header_stamps, scan_header_stamps[1:])]
    max_gap_ns = max(gaps)
    yaw_deltas = [math.remainder(
        right[1] - left[1], 2.0 * math.pi)
        for left, right in zip(window_poses, window_poses[1:])]
    signed_rotation_rad = sum(yaw_deltas)
    reverse_rotation_rad = sum(abs(delta) for delta in yaw_deltas
                               if delta < 0.0)
    duration_ns = window_end_ns - window_start_ns
    if not all(math.isfinite(float(value)) for value in (
            signed_rotation_rad, reverse_rotation_rad, duration_ns)):
        raise ValueError('Axis B forced-rotation evidence is non-finite')
    scheduled_duration_s = (
        target_rotation_rad / OBSERVATION_ANGULAR_SPEED_RADPS)
    maximum_duration_s = (
        max(0.0, target_rotation_rad - ROTATION_SLOW_ZONE_RAD) /
        ROTATION_CRUISE_RADPS +
        (ROTATION_SLOW_ZONE_RAD + GT_YAW_TOLERANCE_RAD) /
        ROTATION_MIN_RADPS +
        2.0 * LIDAR_ALLOWED_MAX_GAP_NS / 1e9)
    minimum_scan_count = max(
        3, int(math.floor(scheduled_duration_s * LIDAR_RATE_HZ)) - 1)
    evidence = {
        'schedule_source': 'PRERECORDED_FORCED_ROTATION',
        'target_rotation_rad': target_rotation_rad,
        'angular_speed_radps': OBSERVATION_ANGULAR_SPEED_RADPS,
        'scheduled_duration_s': scheduled_duration_s,
        'maximum_duration_s': maximum_duration_s,
        'window_start_ns': window_start_ns,
        'window_end_ns': window_end_ns,
        'nonzero_cmd_vel_count': len(window_commands),
        'signed_gt_rotation_rad': signed_rotation_rad,
        'reverse_gt_rotation_rad': reverse_rotation_rad,
        'lidar_rate_hz': LIDAR_RATE_HZ,
        'lidar_period_ns': LIDAR_PERIOD_NS,
        'allowed_max_scan_gap_ns': LIDAR_ALLOWED_MAX_GAP_NS,
        'minimum_scan_count': minimum_scan_count,
        'rotation_window_scan_count': len(window_scans),
        'scan_header_stamps_unique_monotonic': unique_monotonic,
        'first_scan_storage_ns': window_scans[0][0],
        'last_scan_storage_ns': window_scans[-1][0],
        'start_boundary_delta_ns': window_scans[0][0] - window_start_ns,
        'end_boundary_delta_ns': window_end_ns - window_scans[-1][0],
        'max_scan_header_gap_ns': max_gap_ns,
    }
    yaw_tolerance_rad = (OBSERVATION_ANGULAR_SPEED_RADPS *
                         LIDAR_ALLOWED_MAX_GAP_NS / 1e9)
    minimum_duration_ns = int(
        scheduled_duration_s * 1e9) - LIDAR_ALLOWED_MAX_GAP_NS
    maximum_duration_ns = int(maximum_duration_s * 1e9)
    if (duration_ns < minimum_duration_ns or
            duration_ns > maximum_duration_ns or
            not unique_monotonic or
            any(angular <= 0.0 or
                angular > OBSERVATION_ANGULAR_SPEED_RADPS + 1e-9
                for _, angular in window_commands) or
            len(window_scans) < minimum_scan_count or
            max_gap_ns > LIDAR_ALLOWED_MAX_GAP_NS or
            evidence['start_boundary_delta_ns'] < 0 or
            evidence['start_boundary_delta_ns'] > LIDAR_ALLOWED_MAX_GAP_NS or
            evidence['end_boundary_delta_ns'] < 0 or
            evidence['end_boundary_delta_ns'] > LIDAR_ALLOWED_MAX_GAP_NS or
            signed_rotation_rad + yaw_tolerance_rad < target_rotation_rad or
            signed_rotation_rad > target_rotation_rad + yaw_tolerance_rad or
            reverse_rotation_rad > yaw_tolerance_rad):
        raise ValueError('Axis B forced-rotation scan continuity gate failed')
    return evidence


def _validate_kidnapped_boundary_samples(
        samples: list[dict], trace: dict) -> dict:
    """Bind the trace discontinuity to the exact recorded GT sample."""
    boundary_stamp_ns = trace['teleport_gt_verified_header_stamp_ns']
    matches = [sample for sample in samples
               if sample['stamp_ns'] == boundary_stamp_ns]
    if len(matches) != 1:
        raise ValueError('kidnapped GT boundary sample cardinality drift')
    pose = matches[0]['pose']
    traced = trace['post_teleport_gt']
    if (math.dist(pose[:2], TELEPORT_POSE[:2]) >
            GT_TRANSLATION_TOLERANCE_M or
            math.dist(pose[:2], traced[:2]) > GT_TRANSLATION_TOLERANCE_M or
            abs(math.remainder(pose[2] - TELEPORT_POSE[2],
                               2.0 * math.pi)) > GT_YAW_TOLERANCE_RAD or
            abs(math.remainder(pose[2] - traced[2], 2.0 * math.pi)) >
            GT_YAW_TOLERANCE_RAD):
        raise ValueError('kidnapped GT boundary pose drift')
    return {'header_stamp_ns': boundary_stamp_ns, 'pose': pose,
            'matching_sample_count': 1}


def _kidnapped_boundary_gt_evidence(root: Path, trace: dict) -> dict:
    """Recompute the exact post-teleport GT boundary from one bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    samples = []
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != '/ground_truth_pose':
            continue
        message = deserialize_message(serialized, PoseStamped)
        samples.append({
            'stamp_ns': (int(message.header.stamp.sec) * 1_000_000_000 +
                         int(message.header.stamp.nanosec)),
            'pose': [message.pose.position.x, message.pose.position.y,
                     _yaw(message.pose.orientation)]})
    reader.close()
    return _validate_kidnapped_boundary_samples(samples, trace)


def validate_input(output_root: Path,
                   source_snapshots: dict | None = None) -> dict:
    """Reopen and independently validate one generated Axis B input."""
    if (not output_root.is_absolute() or not output_root.is_dir() or
            output_root.is_symlink()):
        raise ValueError('Axis B input root is not canonical')
    files = {path.name for path in output_root.iterdir()
             if path.is_file() and not path.is_symlink()}
    expected_files = {'bag_0.mcap', 'metadata.yaml',
                      'axis_b_input_manifest.json',
                      'sanitizer_manifest.json'}
    if (files != expected_files or any(
            path.is_symlink() or not path.is_file()
            for path in output_root.iterdir())):
        raise ValueError('Axis B input file inventory drift')
    manifest_path = output_root / 'axis_b_input_manifest.json'
    alias_path = output_root / 'sanitizer_manifest.json'
    if manifest_path.read_bytes() != alias_path.read_bytes():
        raise ValueError('Axis B manifest alias drift')
    manifest = strict_json_load(manifest_path)
    generator_source = manifest.get('generator_source') \
        if type(manifest) is dict else None
    expected_generator_source = _source_from_snapshot(
        generator_source, source_snapshots,
        'generate_amcl_axis_b_input.py')
    expected_keys = {
        'schema_version', 'scenario', 'scenario_contract', 'source_bag',
        'source_metadata', 'source_evidence', 'g004_contract',
        'trace_evidence', 'topic_mapping', 'topic_counts',
        'observation_motion_evidence',
        'kidnapped_boundary_gt_evidence',
        'removed_map_to_odom_count', 'ordered_payload_sha256',
        'source_projection', 'output',
        'output_limit_bytes', 'generator_source'}
    if (type(manifest) is not dict or set(manifest) != expected_keys or
            manifest['schema_version'] != 1 or
            manifest['scenario'] not in SCENARIOS or
            manifest['scenario_contract'] != SCENARIOS[manifest['scenario']] or
            manifest['topic_mapping'] != SOURCE_TOPICS or
            not _generator_source_valid(
                manifest['generator_source'], expected_generator_source) or
            manifest['output_limit_bytes'] != 134217728):
        raise ValueError('Axis B input manifest schema drift')
    for key in ('source_bag', 'source_metadata', 'source_evidence',
                'g004_contract'):
        record = manifest[key]
        if (type(record) is not dict or set(record) != {
                'path', 'size_bytes', 'sha256'} or
                not _identity_matches(record, Path(record['path']))):
            raise ValueError(f'Axis B source identity drift: {key}')
    if not _frozen_g004_contract_valid(manifest['g004_contract']):
        raise ValueError('Axis B input is not bound to frozen G004')
    capture = validate_capture_attestation(
        Path(manifest['source_evidence']['path']),
        Path(manifest['source_bag']['path']).parent, manifest['scenario'],
        expected_generator_source)
    expected_driver_source = None
    if manifest['scenario'] == 'kidnapped':
        trace_record = strict_json_load(
            Path(manifest['trace_evidence']['path']))['driver_source']
        expected_driver_source = _source_from_snapshot(
            trace_record, source_snapshots, 'axis_b_kidnapped_driver.py')
    if manifest['scenario'] == 'kidnapped':
        driver_command = next(
            process['command'] for process in capture['processes']
            if process['name'] == 'scenario_driver')
        evidence_index = driver_command.index('--evidence')
        expected_trace_path = (
            Path(manifest['source_bag']['path']).parent /
            'kidnapped_trace.json')
        if Path(driver_command[evidence_index + 1]) != expected_trace_path:
            raise ValueError('kidnapped trace output command drift')
    projection = _source_projection(Path(manifest['source_bag']['path']).parent)
    if (manifest['source_projection'] != projection or
            manifest['topic_counts'] != projection['projected_topic_counts'] or
            manifest['removed_map_to_odom_count'] !=
            projection['removed_map_to_odom_count'] or
            manifest['ordered_payload_sha256'] !=
            projection['projected_ordered_payload_sha256']):
        raise ValueError('Axis B source projection parity drift')
    if (projection['source_topic_counts'][CONTACT_TOPIC] != 0 or
            projection['projected_topic_counts']['/contact'] != 0 or
            capture['contact_endpoint_probe']['publisher_count'] != 1):
        raise ValueError('Axis B contact source contract drift')
    trace = manifest['trace_evidence']
    boundary_gt = manifest['kidnapped_boundary_gt_evidence']
    if manifest['scenario'] == 'kidnapped':
        expected_trace_path = (
            Path(manifest['source_bag']['path']).parent /
            'kidnapped_trace.json')
        if (type(trace) is not dict or set(trace) != {
                'path', 'size_bytes', 'sha256'} or
                Path(trace['path']) != expected_trace_path or
                Path(trace['path']).resolve() != expected_trace_path or
                not _identity_matches(trace, expected_trace_path)):
            raise ValueError('kidnapped trace identity drift')
        trace_value = validate_kidnapped_trace(
            strict_json_load(expected_trace_path), expected_driver_source)
        source_boundary = _kidnapped_boundary_gt_evidence(
            expected_trace_path.parent, trace_value)
        output_boundary = _kidnapped_boundary_gt_evidence(
            output_root, trace_value)
        if boundary_gt != source_boundary or output_boundary != source_boundary:
            raise ValueError('kidnapped GT boundary evidence drift')
    elif trace is not None or boundary_gt is not None:
        raise ValueError('unexpected non-kidnapped trace')
    source_motion = _observation_motion_evidence(
        Path(manifest['source_bag']['path']).parent, manifest['scenario'],
        '/sim_raw/scan', expected_driver_source)
    output_motion = _observation_motion_evidence(
        output_root, manifest['scenario'], '/scan', expected_driver_source)
    if (manifest['observation_motion_evidence'] != source_motion or
            output_motion != source_motion):
        raise ValueError('Axis B observation motion evidence drift')
    mcap_path = output_root / 'bag_0.mcap'
    metadata_path = output_root / 'metadata.yaml'
    output = manifest['output']
    if (type(output) is not dict or set(output) != {'mcap', 'metadata'} or
            not _identity_matches(output['mcap'], mcap_path, output=True) or
            not _identity_matches(
                output['metadata'], metadata_path, output=True) or
            mcap_path.stat().st_size > manifest['output_limit_bytes']):
        raise ValueError('Axis B output identity drift')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(output_root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    output_meta = {item.name: item
                   for item in reader.get_all_topics_and_types()}
    if (set(output_meta) != set(SOURCE_TOPICS.values()) or
            output_meta['/contact'].type !=
            'ros_gz_interfaces/msg/Contacts'):
        raise ValueError('Axis B output topic metadata drift')
    counts = {name: 0 for name in SOURCE_TOPICS.values()}
    digest = hashlib.sha256()
    while reader.has_next():
        topic, serialized, storage_ns = reader.read_next()
        if topic not in counts:
            raise ValueError('unexpected Axis B output topic')
        payload = bytes(serialized)
        if topic == '/contact':
            deserialize_message(payload, Contacts)
            raise ValueError('Axis B contact output is not zero')
        if topic == '/tf':
            message = deserialize_message(payload, TFMessage)
            if any(_tf_record(transform)['parent'].lstrip('/') == 'map' and
                   _tf_record(transform)['child'].lstrip('/') == 'odom'
                   for transform in message.transforms):
                raise ValueError('map-to-odom remained in Axis B input')
            if _canonical_tf_serialization(
                    bytes(serialize_message(message))) != payload:
                raise ValueError('Axis B TF serialization is not canonical')
        counts[topic] += 1
        digest.update(topic.encode() + b'\0')
        digest.update(str(storage_ns).encode() + b'\0' + payload)
    reader.close()
    if (counts != manifest['topic_counts'] or counts['/contact'] != 0 or
            digest.hexdigest() != manifest['ordered_payload_sha256'] or
            type(manifest['removed_map_to_odom_count']) is not int or
            manifest['removed_map_to_odom_count'] < 0):
        raise ValueError('Axis B output payload parity drift')
    return manifest


def generate(source_root: Path, output_root: Path, scenario: str,
             source_evidence: Path, g004_contract: Path,
             trace_evidence: Path | None = None) -> dict:
    """Filter one validated simulation bag without recorded map-to-odom."""
    if scenario not in SCENARIOS:
        raise ValueError('unknown Axis B scenario')
    if (not source_root.is_absolute() or source_root != source_root.resolve() or
            source_root.is_symlink() or not source_root.is_dir()):
        raise ValueError('source root must be a canonical bag directory')
    g004_record = _identity(g004_contract)
    if not _frozen_g004_contract_valid(g004_record):
        raise ValueError('Axis B generation requires frozen G004 contract')
    if scenario == 'kidnapped':
        if trace_evidence is None:
            raise ValueError('kidnapped input requires live trace evidence')
        expected_trace_path = source_root / 'kidnapped_trace.json'
        if (trace_evidence.resolve() != expected_trace_path or
                trace_evidence != expected_trace_path):
            raise ValueError('kidnapped trace escaped source capture root')
        trace_value = validate_kidnapped_trace(strict_json_load(trace_evidence))
        boundary_gt_evidence = _kidnapped_boundary_gt_evidence(
            source_root, trace_value)
    elif trace_evidence is not None:
        raise ValueError('trace evidence is only valid for kidnapped input')
    else:
        boundary_gt_evidence = None
    validate_capture_attestation(source_evidence, source_root, scenario)
    motion_evidence = _observation_motion_evidence(
        source_root, scenario, '/sim_raw/scan')
    if output_root.exists() or output_root.is_symlink() or not output_root.is_absolute():
        raise ValueError('output root must be absent and absolute')
    converter = rosbag2_py.ConverterOptions('cdr', 'cdr')
    with tempfile.TemporaryDirectory(
            prefix='.g002-axis-b-input-', dir=output_root.parent) as temp_name:
        stage = Path(temp_name) / 'bag'
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(
            uri=str(source_root), storage_id='mcap'), converter)
        source_meta = {item.name: item
                       for item in reader.get_all_topics_and_types()}
        _require_source_topic_set(source_meta)
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(
            uri=str(stage), storage_id='mcap'), converter)
        for source_name, output_name in SOURCE_TOPICS.items():
            item = source_meta[source_name]
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=0, name=output_name, type=item.type,
                serialization_format=item.serialization_format,
                offered_qos_profiles=item.offered_qos_profiles,
                type_description_hash=item.type_description_hash))
        counts = {name: 0 for name in SOURCE_TOPICS.values()}
        removed = 0
        digest = hashlib.sha256()
        while reader.has_next():
            topic, serialized, storage_ns = reader.read_next()
            if topic not in SOURCE_TOPICS:
                continue
            output_topic = SOURCE_TOPICS[topic]
            payload = bytes(serialized)
            if topic == CONTACT_TOPIC:
                deserialize_message(payload, Contacts)
                raise ValueError('Axis B source contact is not zero')
            if topic == '/tf':
                message = deserialize_message(payload, TFMessage)
                kept = []
                for transform in message.transforms:
                    record = _tf_record(transform)
                    if (record['parent'].lstrip('/') == 'map' and
                            record['child'].lstrip('/') == 'odom'):
                        removed += 1
                    else:
                        kept.append(transform)
                message.transforms = kept
                payload = _canonical_tf_serialization(
                    bytes(serialize_message(message)))
            writer.write(output_topic, payload, storage_ns)
            counts[output_topic] += 1
            digest.update(output_topic.encode() + b'\0')
            digest.update(str(storage_ns).encode() + b'\0' + payload)
        reader.close()
        writer.close()
        del reader
        del writer
        mcap = next(stage.glob('*.mcap'))
        metadata = stage / 'metadata.yaml'
        manifest = {
            'schema_version': 1, 'scenario': scenario,
            'scenario_contract': SCENARIOS[scenario],
            'source_bag': _identity(source_root / 'bag_0.mcap'),
            'source_metadata': _identity(source_root / 'metadata.yaml'),
            'source_evidence': _identity(source_evidence),
            'g004_contract': g004_record,
            'trace_evidence': (_identity(trace_evidence)
                               if trace_evidence is not None else None),
            'observation_motion_evidence': motion_evidence,
            'kidnapped_boundary_gt_evidence': boundary_gt_evidence,
            'generator_source': _generator_source_identity(),
            'topic_mapping': SOURCE_TOPICS,
            'topic_counts': counts,
            'removed_map_to_odom_count': removed,
            'ordered_payload_sha256': digest.hexdigest(),
            'source_projection': _source_projection(source_root),
            'output': {'mcap': _output_identity(mcap),
                       'metadata': _output_identity(metadata)},
            'output_limit_bytes': 134217728,
        }
        required_nonempty = set(SOURCE_TOPICS.values()) - {'/contact'}
        if (any(counts[name] <= 0 for name in required_nonempty) or
                counts['/contact'] != 0 or
                mcap.stat().st_size > manifest['output_limit_bytes']):
            raise ValueError('Axis B input parity or cap failed')
        (stage / 'axis_b_input_manifest.json').write_bytes(
            canonical_json_bytes(manifest))
        # Preflight runner consumes this stable alias without interpreting it.
        shutil.copyfile(stage / 'axis_b_input_manifest.json',
                        stage / 'sanitizer_manifest.json')
        validate_input(stage)
        stage.rename(output_root)
    return validate_input(output_root)


def run_observation_capture_driver(scenario: str, domain_id: int) -> int:
    """Execute the preregistered rotation and finish at zero velocity."""
    if scenario not in ('correct_init', 'initial_offset'):
        raise ValueError('observation capture driver scenario drift')
    import time
    from geometry_msgs.msg import PoseStamped, Twist
    import rclpy
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    rclpy.init()
    node = rclpy.create_node('g002_axis_b_observation_capture_driver')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    target_rad = SCENARIOS[scenario]['observation_rotation_rad']
    accumulated_rad = 0.0
    last_yaw = None

    def observe_ground_truth(message: PoseStamped) -> None:
        nonlocal accumulated_rad, last_yaw
        orientation = message.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z +
                   orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))
        if last_yaw is not None:
            accumulated_rad += math.remainder(
                yaw - last_yaw, 2.0 * math.pi)
        last_yaw = yaw

    node.create_subscription(
        PoseStamped, '/ground_truth_pose', observe_ground_truth, 10)
    deadline = time.monotonic() + OBSERVATION_TIMEOUT_S
    try:
        while (rclpy.ok() and time.monotonic() < deadline and
               accumulated_rad < target_rad):
            command = Twist()
            command.angular.z = OBSERVATION_ANGULAR_SPEED_RADPS
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
        if accumulated_rad < target_rad:
            raise RuntimeError('Axis B observation rotation timed out')
        zero_deadline = time.monotonic() + OBSERVATION_ZERO_HOLD_S
        while rclpy.ok() and time.monotonic() < zero_deadline:
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)
        publisher.publish(Twist())
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    """Parse canonical Axis B input generation arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture-driver', choices=tuple(SCENARIOS))
    parser.add_argument('--domain-id', type=int)
    parser.add_argument('--execute-capture-request', type=Path)
    parser.add_argument('--capture-scenario', choices=tuple(SCENARIOS))
    parser.add_argument('--attestation-output', type=Path)
    parser.add_argument('--request-output', type=Path)
    parser.add_argument('--capture-root', action='append', default=[],
                        metavar='SCENARIO=/ABSOLUTE/ROOT')
    parser.add_argument('--source-root', type=Path)
    parser.add_argument('--output-root', type=Path)
    parser.add_argument('--scenario', choices=tuple(SCENARIOS))
    parser.add_argument('--source-evidence', type=Path)
    parser.add_argument('--g004-contract', type=Path)
    parser.add_argument('--trace-evidence', type=Path)
    args = parser.parse_args()
    if args.capture_driver is not None:
        if args.domain_id is None or not 0 <= args.domain_id <= 232:
            parser.error('capture driver requires a valid --domain-id')
        if args.capture_driver == 'kidnapped':
            parser.error('kidnapped capture uses axis_b_kidnapped_driver.py')
        return run_observation_capture_driver(
            args.capture_driver, args.domain_id)
    if args.execute_capture_request is not None:
        if (args.capture_scenario is None or args.attestation_output is None or
                args.domain_id is not None or args.request_output is not None or
                args.capture_root or any(value is not None for value in (
                    args.source_root, args.output_root, args.scenario,
                    args.source_evidence, args.g004_contract,
                    args.trace_evidence))):
            parser.error(
                'capture execution requires only --execute-capture-request, '
                '--capture-scenario, and --attestation-output')
        execute_capture(args.execute_capture_request, args.capture_scenario,
                        args.attestation_output)
        return 0
    if args.capture_scenario is not None or args.attestation_output is not None:
        parser.error('capture execution arguments are incomplete')
    if args.request_output is not None:
        if any(value is not None for value in (
                args.source_root, args.output_root, args.scenario,
                args.source_evidence, args.g004_contract,
                args.trace_evidence)):
            parser.error('request mode cannot generate an input bag')
        write_generation_request(
            args.request_output, _parse_bindings(args.capture_root))
        return 0
    if args.capture_root:
        parser.error('--capture-root is only valid with --request-output')
    required = ('source_root', 'output_root', 'scenario', 'source_evidence',
                'g004_contract')
    missing = [name for name in required if getattr(args, name) is None]
    if missing:
        parser.error('generation requires: ' + ', '.join(missing))
    generate(args.source_root, args.output_root, args.scenario,
             args.source_evidence, args.g004_contract, args.trace_evidence)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
