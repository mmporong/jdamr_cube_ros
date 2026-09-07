#!/usr/bin/env python3
"""Generate one canonical G002 Axis B localization replay input."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import signal
import tempfile
import time

from amcl_fault_contract import (
    canonical_json_bytes,
    sha256_file,
    strict_json_load,
)
from axis_b_kidnapped_driver import validate_kidnapped_trace
from g002_tf_sanitizer import _canonical_tf_serialization, _tf_record
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from tf2_msgs.msg import TFMessage


SCENARIOS = {
    'correct_init': {
        'true_start_pose': [-8.0, 0.0, 0.0],
        'initial_estimate_offset': [0.0, 0.0, 0.0],
        'teleport_pose': None, 'observation_rotation_rad': 0.0},
    'initial_offset': {
        'true_start_pose': [-8.0, 0.0, 0.0],
        'initial_estimate_offset': [0.5, 0.0, 0.2617993877991494],
        'teleport_pose': None, 'observation_rotation_rad': 0.0},
    'kidnapped': {
        'true_start_pose': [-8.0, 0.0, 0.0],
        'initial_estimate_offset': [0.0, 0.0, 0.0],
        'teleport_pose': [0.0, 0.0, 3.141592653589793],
        'observation_rotation_rad': 6.283185307179586},
}
SOURCE_TOPICS = {
    '/sim_raw/scan': '/scan', '/odom': '/odom', '/tf': '/tf',
    '/tf_static': '/tf_static', '/ground_truth_pose': '/ground_truth_pose',
    '/cmd_vel': '/cmd_vel',
    ('/world/slam_corridor/model/g003_preloaded_front_observation_probe/'
     'link/body/sensor/contact_sensor/contact'): '/contact',
}
GENERATION_REQUEST_SCHEMA_VERSION = 1


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


def generation_request(capture_roots: dict[str, Path]) -> dict:
    """Create a no-motion request for three future Gazebo captures."""
    if set(capture_roots) != set(SCENARIOS):
        raise ValueError('generation request requires all three scenarios')
    from prepare_amcl_fault_benchmark import G004_ROOT, _axis_b_identity
    runtime = _axis_b_identity(G004_ROOT)
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
                'seed:=11', 'gui:=false', 'x_pose:=-8.0', 'y_pose:=0.0',
                'z_pose:=0.01'],
            'recorder': [
                '/opt/ros/jazzy/bin/ros2', 'bag', 'record', '--storage', 'mcap',
                '--output', str(root), '--topics', *sorted(SOURCE_TOPICS)],
            'scenario_driver': ([
                '/usr/bin/python3', str(Path(__file__).with_name(
                    'axis_b_kidnapped_driver.py').resolve()), '--evidence',
                str(root / 'kidnapped_trace.json'), '--contact-topic',
                next(name for name in SOURCE_TOPICS if 'contact' in name)]
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


def validate_generation_request(value: dict) -> dict:
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
                'urdf', 'lidar_profile', 'runner_source',
                'expected_commands',
                'gazebo_runtime_capture_required',
                'kidnapped_driver_required', 'expected_topics'}:
            raise ValueError('Axis B generation row schema drift')
        root = Path(row['capture_root'])
        expected_row = generation_request(
            {name: Path(item['capture_root']) for name, item in
             zip(SCENARIOS, value['requests'])})['requests'][len(roots)]
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


def validate_capture_attestation(path: Path, source_root: Path,
                                 scenario: str) -> dict:
    """Require direct Gazebo capture evidence; summaries are not inputs."""
    value = strict_json_load(path)
    expected = {
        'schema_version', 'status', 'execution_mode', 'scenario',
        'source_mcap', 'source_metadata', 'generation_request',
        'synthetic_input', 'rosbag_record_exit_code', 'gazebo_exit_code',
        'survivor_count', 'runner_source', 'gazebo_seed', 'ros_domain_id',
        'g004_contract', 'world', 'urdf', 'lidar_profile', 'processes',
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
            value['rosbag_record_exit_code'] != -signal.SIGINT or
            value['gazebo_exit_code'] != -signal.SIGINT or
            value['survivor_count'] != 0 or
            value['atomic_finalized'] is not True or
            not _identity_matches(value['source_mcap'],
                                  source_root / 'bag_0.mcap') or
            not _identity_matches(value['source_metadata'],
                                  source_root / 'metadata.yaml') or
            not _identity_matches(value['generation_request'], request_path)):
        raise ValueError('Axis B capture attestation gate failed')
    request = validate_generation_request(strict_json_load(request_path))
    row = next((item for item in request['requests']
                if item['scenario'] == scenario), None)
    if row is None or Path(row['capture_root']) != source_root:
        raise ValueError('Axis B capture request binding drift')
    for key in ('runner_source', 'gazebo_seed', 'ros_domain_id',
                'g004_contract', 'world', 'urdf', 'lidar_profile'):
        if value[key] != row[key]:
            raise ValueError(f'Axis B capture runtime drift: {key}')
    if (type(value['processes']) is not list or
            [item.get('name') for item in value['processes']] !=
            ['gazebo', 'recorder', 'scenario_driver']):
        raise ValueError('Axis B capture process inventory drift')
    for process in value['processes']:
        if (type(process) is not dict or set(process) != {
                'name', 'command', 'pid', 'returncode', 'survivors', 'log',
                'started_steady_ns', 'finished_steady_ns', 'environment',
                'pid_starttime_ticks', 'wait_returncode'} or
                type(process['command']) is not list or
                not process['command'] or
                any(type(item) is not str for item in process['command']) or
                type(process['pid']) is not int or process['pid'] <= 0 or
                type(process['pid_starttime_ticks']) is not int or
                process['pid_starttime_ticks'] <= 0 or
                process['returncode'] != (
                    0 if process['name'] == 'scenario_driver'
                    else -signal.SIGINT) or
                process['wait_returncode'] != process['returncode'] or
                process['survivors'] != [] or
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
        if process['command'] != row['expected_commands'][process['name']]:
            raise ValueError('Axis B capture process command drift')
    processes = value['processes']
    if (len({item['pid'] for item in processes}) != len(processes) or
            len({item['log']['path'] for item in processes}) != len(processes) or
            any(left['started_steady_ns'] >= right['started_steady_ns']
                for left, right in zip(processes, processes[1:])) or
            not (processes[2]['finished_steady_ns'] <
                 processes[1]['finished_steady_ns'] <
                 processes[0]['finished_steady_ns'])):
        raise ValueError('Axis B capture process chronology drift')
    receipt_path = Path(value['executor_receipt']['path']) \
        if type(value['executor_receipt']) is dict else Path()
    if not _identity_matches(value['executor_receipt'], receipt_path):
        raise ValueError('Axis B capture executor receipt identity drift')
    receipt = strict_json_load(receipt_path)
    fingerprint = hashlib.sha256(canonical_json_bytes({
        'processes': value['processes'],
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
    by_name = {item['name']: item for item in stopped}
    if set(by_name) != {'gazebo', 'recorder', 'scenario_driver'}:
        raise RuntimeError('Axis B capture process inventory failed')
    if (by_name['scenario_driver']['returncode'] != 0 or
            by_name['recorder']['returncode'] != -signal.SIGINT or
            by_name['gazebo']['returncode'] != -signal.SIGINT or
            any(item['survivors'] for item in stopped)):
        raise RuntimeError('Axis B capture teardown failed')
    processes = []
    for name in ('gazebo', 'recorder', 'scenario_driver'):
        item = by_name[name]
        log_path = log_root / f'{name}.log'
        processes.append({
            'name': name, 'command': item['command'], 'pid': item['pid'],
            'returncode': item['returncode'], 'survivors': item['survivors'],
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
        'urdf': row['urdf'], 'lidar_profile': row['lidar_profile'],
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


def _generator_source_valid(record: dict) -> bool:
    return record == _generator_source_identity()


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


def validate_input(output_root: Path) -> dict:
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
    expected_keys = {
        'schema_version', 'scenario', 'scenario_contract', 'source_bag',
        'source_metadata', 'source_evidence', 'g004_contract',
        'trace_evidence', 'topic_mapping', 'topic_counts',
        'removed_map_to_odom_count', 'ordered_payload_sha256',
        'source_projection', 'output',
        'output_limit_bytes', 'generator_source'}
    if (type(manifest) is not dict or set(manifest) != expected_keys or
            manifest['schema_version'] != 1 or
            manifest['scenario'] not in SCENARIOS or
            manifest['scenario_contract'] != SCENARIOS[manifest['scenario']] or
            manifest['topic_mapping'] != SOURCE_TOPICS or
            not _generator_source_valid(manifest['generator_source']) or
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
    validate_capture_attestation(
        Path(manifest['source_evidence']['path']),
        Path(manifest['source_bag']['path']).parent, manifest['scenario'])
    projection = _source_projection(Path(manifest['source_bag']['path']).parent)
    if (manifest['source_projection'] != projection or
            manifest['topic_counts'] != projection['projected_topic_counts'] or
            manifest['removed_map_to_odom_count'] !=
            projection['removed_map_to_odom_count'] or
            manifest['ordered_payload_sha256'] !=
            projection['projected_ordered_payload_sha256']):
        raise ValueError('Axis B source projection parity drift')
    trace = manifest['trace_evidence']
    if manifest['scenario'] == 'kidnapped':
        if (type(trace) is not dict or set(trace) != {
                'path', 'size_bytes', 'sha256'} or
                not _identity_matches(trace, Path(trace['path']))):
            raise ValueError('kidnapped trace identity drift')
        validate_kidnapped_trace(strict_json_load(Path(trace['path'])))
    elif trace is not None:
        raise ValueError('unexpected non-kidnapped trace')
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
    counts = {name: 0 for name in SOURCE_TOPICS.values()}
    digest = hashlib.sha256()
    while reader.has_next():
        topic, serialized, storage_ns = reader.read_next()
        if topic not in counts:
            raise ValueError('unexpected Axis B output topic')
        payload = bytes(serialized)
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
    if (counts != manifest['topic_counts'] or
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
        validate_kidnapped_trace(strict_json_load(trace_evidence))
    elif trace_evidence is not None:
        raise ValueError('trace evidence is only valid for kidnapped input')
    validate_capture_attestation(source_evidence, source_root, scenario)
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
        if not set(SOURCE_TOPICS).issubset(source_meta):
            raise ValueError('G004 representative topic set drift')
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
        if (counts['/scan'] <= 0 or counts['/ground_truth_pose'] <= 0 or
                counts['/odom'] <= 0 or counts['/tf'] <= 0 or
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


def run_stationary_capture_driver(scenario: str, domain_id: int) -> int:
    """Publish bounded zero velocity while a Gazebo input is recorded."""
    if scenario not in ('correct_init', 'initial_offset'):
        raise ValueError('stationary capture driver scenario drift')
    import time
    from geometry_msgs.msg import Twist
    import rclpy
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    rclpy.init()
    node = rclpy.create_node('g002_axis_b_stationary_capture_driver')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    deadline = time.monotonic() + 30.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.1)
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
        return run_stationary_capture_driver(
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
