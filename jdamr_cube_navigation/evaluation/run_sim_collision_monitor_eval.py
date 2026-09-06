#!/usr/bin/env python3
"""Run the bounded G004 matrix with compact evidence retention."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import shutil
import signal
import statistics
import subprocess
import time
from typing import Any

from evaluate_sim_collision_monitor import (
    _representative_retention_checks, aggregate, evaluate_run,
    promotion_runtime_valid,
    strict_json_loads)
from finalize_sim_collision_monitor_eval import finalize

from run_sim_nav_obstacle_eval import (
    _cleanup_identity, _environment, _start, _stop,
    _wait_lifecycle_active, _wait_tf_available, _wait_topics)
from run_sim_nav_obstacle_eval import (
    configure_evaluation_rmw, evaluation_rmw_evidence)

from sim_collision_monitor_contract import (
    REPRESENTATIVE_BAG_LIMIT_BYTES, scenario_matrix, sha256_file,
    TOTAL_RETENTION_LIMIT_BYTES)

import yaml


CONTACT_TOPIC = (
    '/world/slam_corridor/model/g003_preloaded_front_observation_probe/'
    'link/body/sensor/contact_sensor/contact')
RESOURCE_SAMPLER = (
    Path(__file__).resolve().parent / 'sample_process_group_resources.py')
MAX_SAFE_ROS_DOMAIN_ID = 232
PHYSICAL_ROBOT_DOMAIN_ID = 12
PROCESS_DEATH_PATTERN = re.compile(
    rb'\[ERROR\] \[([^\]]+)\]: process has died '
    rb'\[pid (\d+), exit code (-?\d+), cmd \'([^\']+)\'\]\.\Z')
GAZEBO_BRIDGE_EXECUTABLE = (
    '/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge')
HIDDEN_TOPIC_RECORD_FLAG = '--include-hidden-topics'


def _logged_process_exits(
        content: bytes) -> tuple[bool, list[dict[str, Any]]]:
    """Parse ordered ros2 launch child deaths with raw byte provenance."""
    records = []
    offset = 0
    try:
        for raw_line in content.splitlines(keepends=True):
            line = raw_line.rstrip(b'\r\n')
            if b'process has died' in line:
                match = PROCESS_DEATH_PATTERN.fullmatch(line)
                if match is None:
                    return False, []
                records.append({
                    'label': match.group(1).decode('utf-8', errors='strict'),
                    'pid': int(match.group(2)),
                    'exit_code': int(match.group(3)),
                    'command': match.group(4).decode(
                        'utf-8', errors='strict'),
                    'byte_offset': offset,
                })
            offset += len(raw_line)
    except (UnicodeDecodeError, ValueError):
        return False, []
    pids = [record['pid'] for record in records]
    identities = [(record['label'], record['pid']) for record in records]
    return (len(pids) == len(set(pids))
            and len(identities) == len(set(identities))), records


def _logged_process_exits_allowed(
        owner: str, records: list[dict[str, Any]]) -> bool:
    """Allow only exact runner-induced launch child termination shapes."""
    gazebo_bridge_aborts = 0
    for record in records:
        if (not isinstance(record, dict) or set(record) != {
                'label', 'pid', 'exit_code', 'command', 'byte_offset'}
                or not isinstance(record['label'], str)
                or type(record['pid']) is not int or record['pid'] <= 0
                or type(record['exit_code']) is not int
                or not isinstance(record['command'], str)
                or type(record['byte_offset']) is not int
                or record['byte_offset'] < 0):
            return False
        code = record['exit_code']
        if code in {-signal.SIGINT, -signal.SIGTERM}:
            continue
        if code == -signal.SIGABRT and owner in {
                'navigation', 'contact_bridge'}:
            continue
        if (code == -signal.SIGABRT and owner == 'gazebo'
                and re.fullmatch(r'parameter_bridge-\d+', record['label'])
                and record['command'].split(' ', 1)[0]
                == GAZEBO_BRIDGE_EXECUTABLE):
            gazebo_bridge_aborts += 1
            continue
        return False
    return gazebo_bridge_aborts <= 1


def _stop_with_evidence(
        name: str, process: subprocess.Popen, log_path: Path,
        measurement_finalized_steady_ns: int | None,
        returncode_at_measurement: int | None,
        *, may_exit_after_parent: bool = False) -> tuple[dict[str, Any], str | None]:
    """Stop one process group and classify its observed teardown outcome."""
    returncode_before = process.poll()
    log_size_before = log_path.stat().st_size if log_path.is_file() else 0
    stop_requested_ns = time.monotonic_ns()
    runner_initiated = returncode_before is None
    _stop(process)
    stop_completed_ns = time.monotonic_ns()
    returncode_after = process.poll()
    log_valid, all_failures = _logged_process_exits(
        log_path.read_bytes() if log_path.is_file() else b'')
    failures_before = [record for record in all_failures
                       if record['byte_offset'] < log_size_before]
    failures_after = [record for record in all_failures
                      if record['byte_offset'] >= log_size_before]
    record = {
        'name': name,
        'pid': process.pid,
        'returncode_at_measurement': returncode_at_measurement,
        'returncode_before_stop': returncode_before,
        'runner_initiated': runner_initiated,
        'requested_signal': signal.SIGINT if runner_initiated else None,
        'stop_requested_steady_ns': stop_requested_ns,
        'stop_completed_steady_ns': stop_completed_ns,
        'returncode_after_stop': returncode_after,
        'logged_process_exits_before_stop': failures_before,
        'logged_process_exits_after_stop': failures_after,
        'log_path': str(log_path.resolve()),
        'log_size_bytes': log_path.stat().st_size if log_path.is_file() else 0,
        'log_sha256': sha256_file(log_path) if log_path.is_file() else None,
        'log_pre_stop_offset_bytes': log_size_before,
    }
    error = None
    if not log_valid:
        error = f'{name}:invalid_process_death_log'
    elif name == 'scenario':
        measurement_state_valid = returncode_at_measurement == 0
    else:
        measurement_state_valid = returncode_at_measurement is None
    if not log_valid:
        pass
    elif not measurement_state_valid:
        error = f'{name}:invalid_state_at_measurement:{returncode_at_measurement}'
    elif failures_before:
        error = f'{name}:child_exit_before_measurement:{failures_before}'
    elif returncode_before not in (None, 0):
        error = f'{name}:exit_before_teardown:{returncode_before}'
    elif (returncode_before == 0 and name != 'scenario'
          and not may_exit_after_parent):
        error = f'{name}:unexpected_early_clean_exit'
    elif returncode_after is None:
        error = f'{name}:teardown_did_not_finish'
    elif runner_initiated:
        allowed = {0, -signal.SIGINT, 128 + signal.SIGINT,
                   -signal.SIGTERM, 128 + signal.SIGTERM}
        if name in {'navigation', 'contact_bridge'}:
            allowed.add(-signal.SIGABRT)
        if returncode_after not in allowed:
            error = f'{name}:unexpected_teardown_exit:{returncode_after}'
        if not _logged_process_exits_allowed(name, failures_after):
            error = f'{name}:unexpected_child_teardown_exit'
    if (runner_initiated and measurement_finalized_steady_ns is None):
        error = f'{name}:runner_teardown_before_measurement_finalized'
    elif (runner_initiated and stop_requested_ns
          < measurement_finalized_steady_ns):
        error = f'{name}:runner_teardown_precedes_measurement'
    return record, error


def harness_source_records(
        output_root: Path | None = None,
) -> dict[str, dict[str, Any]]:
    """Return and optionally retain the exact G004 source inventory."""
    evaluation = Path(__file__).resolve().parent
    package = evaluation.parent / 'jdamr_cube_navigation'
    paths = {
        'contract': evaluation / 'sim_collision_monitor_contract.py',
        'prepare': evaluation / 'prepare_sim_collision_monitor_run.py',
        'runner': evaluation / 'run_sim_collision_monitor_eval.py',
        'evaluator': evaluation / 'evaluate_sim_collision_monitor.py',
        'finalizer': evaluation / 'finalize_sim_collision_monitor_eval.py',
        'resource_sampler': evaluation / 'sample_process_group_resources.py',
        'scenario': package / 'sim_collision_monitor_scenario.py',
        'scan_gate': package / 'sim_scan_gate.py',
    }
    records = {}
    for name, path in paths.items():
        source = path.resolve()
        record = {
            'path': str(source), 'size_bytes': source.stat().st_size,
            'sha256': sha256_file(source)}
        if output_root is not None:
            retained = output_root / 'runtime_sources' / f'{name}.py'
            retained.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, retained)
            record.update({
                'retained_path': str(retained.resolve()),
                'retained_size_bytes': retained.stat().st_size,
                'retained_sha256': sha256_file(retained),
            })
        records[name] = record
    return records


def retain_rmw_library(
        rmw: dict[str, Any], output_root: Path,
) -> dict[str, Any]:
    """Retain the exact middleware library bytes used by this matrix."""
    source = Path(rmw['library_path']).resolve()
    retained = output_root / 'runtime_dependencies' / source.name
    retained.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, retained)
    rmw.update({
        'retained_library_path': str(retained.resolve()),
        'retained_library_size_bytes': retained.stat().st_size,
        'retained_library_sha256': sha256_file(retained),
    })
    return rmw


def validate_domain_block(base_domain_id: int) -> None:
    """Reserve 15 simulator domains without touching the physical domain."""
    allocated = range(base_domain_id, base_domain_id + 15)
    if (base_domain_id < 0
            or base_domain_id + 14 > MAX_SAFE_ROS_DOMAIN_ID
            or PHYSICAL_ROBOT_DOMAIN_ID in allocated):
        raise ValueError(
            'domain block must stay in 0..232 and exclude physical domain 12')


def retention_allowed(
        current_bytes: int, predicted_bytes: int,
        total_limit_bytes: int = TOTAL_RETENTION_LIMIT_BYTES,
) -> bool:
    """Return whether predicted retained output fits the total cap."""
    return current_bytes >= 0 and predicted_bytes >= 0 and (
        current_bytes + predicted_bytes <= total_limit_bytes)


def representative_bag_allowed(size_bytes: int) -> bool:
    """Apply the per-bag retention cap."""
    return 0 <= size_bytes <= REPRESENTATIVE_BAG_LIMIT_BYTES


def tree_size_bytes(path: Path) -> int:
    """Return the regular-file size beneath a retained root."""
    return sum(item.stat().st_size for item in path.rglob('*')
               if item.is_file())


def production_hashes_unchanged(contract: dict[str, Any]) -> bool:
    """Re-hash every production input captured by preparation."""
    return all(
        Path(record['path']).is_file()
        and sha256_file(Path(record['path'])) == record['sha256']
        for record in contract['production_inputs'].values())


def validate_preparation_manifest(prepared_root: Path) -> None:
    """Reject missing, extra, moved, or modified prepared assets."""
    path = prepared_root / 'preparation_manifest.json'
    document = strict_json_loads(path.read_text())
    expected = set(document['expected_generated_paths'])
    records = document['records']
    actual = [record.get('relative_path') for record in records]
    if len(actual) != len(expected) or set(actual) != expected:
        raise ValueError('prepared asset record set mismatch')
    actual_files = {
        str(asset.relative_to(prepared_root))
        for asset in prepared_root.rglob('*')
        if asset.is_file() and asset.name != 'preparation_manifest.json'}
    if actual_files != expected:
        raise ValueError('prepared asset file set mismatch')
    for record in records:
        asset = prepared_root / record['relative_path']
        if (not asset.is_file()
                or asset.resolve().parent != prepared_root.resolve()
                or asset.stat().st_size != record['size_bytes']
                or sha256_file(asset) != record['sha256']):
            raise ValueError(
                f'prepared asset identity mismatch: {record["relative_path"]}')


def runtime_rmw_evidence(
        environment: dict[str, str], domain_id: int) -> dict[str, Any]:
    """Add re-hashable library and domain identity to RMW evidence."""
    evidence = evaluation_rmw_evidence(environment)
    library = (Path(evidence['prefix']) / 'lib'
               / f'lib{evidence["actual_identifier"]}.so')
    evidence['library_path'] = str(library.resolve())
    evidence['library_size_bytes'] = library.stat().st_size
    evidence['domain_id'] = domain_id
    return evidence


def materialize_prepared_assets(
        prepared_root: Path, output_root: Path) -> Path:
    """Copy prepared assets into the durable root and rewrite local paths."""
    source_manifest = strict_json_loads(
        (prepared_root / 'preparation_manifest.json').read_text())
    assets_root = output_root / 'assets'
    assets_root.mkdir()
    contract = strict_json_loads(
        (prepared_root / 'contract.json').read_text())
    key_by_name = {
        'jdamr_cube_collision_monitor_eval.urdf': 'urdf',
        'collision_monitor_overlay.yaml': 'collision_monitor_overlay',
        'nav2_collision_monitor_eval.params.yaml': 'nav2_evaluation_params',
        'slam_corridor_eval.pgm': 'slam_corridor_eval.pgm',
        'slam_corridor_eval.yaml': 'slam_corridor_eval.yaml',
        'slam_corridor_contact.world': 'world',
        'collision_monitor_bridge.yaml': 'bridge',
    }
    for name, key in key_by_name.items():
        target = assets_root / name
        shutil.copy2(prepared_root / name, target)
        contract['evaluation_assets'][key]['path'] = str(target.resolve())
    contract_path = output_root / 'contract.json'
    contract_path.write_text(
        json.dumps(contract, indent=2, sort_keys=True) + '\n')
    generated_names = {'contract.json', *key_by_name}
    records = []
    for name in sorted(generated_names):
        path = contract_path if name == 'contract.json' else assets_root / name
        records.append({
            'relative_path': name, 'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path)})
    manifest_path = output_root / 'preparation_manifest.json'
    manifest_path.write_text(json.dumps({
        'schema_version': 1,
        'prepared_root': str(assets_root.resolve()),
        'expected_generated_paths': sorted(generated_names),
        'records': records,
    }, indent=2, sort_keys=True) + '\n')
    (output_root / 'source_preparation_manifest.json').write_text(
        json.dumps(source_manifest, indent=2, sort_keys=True) + '\n')
    return assets_root


def summarize_resource_samples(path: Path) -> dict[str, Any]:
    """Summarize one compact process-group JSONL trace."""
    rows = [strict_json_loads(line) for line in path.read_text().splitlines()
            if line.strip()]
    cpu_values = [row['cpu_pct_one_core'] for row in rows
                  if row['cpu_pct_one_core'] is not None]
    rss_values = [row['rss_mb'] for row in rows]
    if not rows or not cpu_values or not all(
            math.isfinite(value) for value in [*cpu_values, *rss_values]):
        raise ValueError('resource trace is empty or non-finite')
    ordered_cpu = sorted(cpu_values)
    p95_index = math.ceil(0.95 * len(ordered_cpu)) - 1
    return {
        'path': str(path.resolve()),
        'sha256': sha256_file(path),
        'size_bytes': path.stat().st_size,
        'sample_count': len(rows),
        'cpu_sample_count': len(cpu_values),
        'median_cpu_pct_one_core': statistics.median(cpu_values),
        'p95_cpu_pct_one_core': ordered_cpu[p95_index],
        'max_rss_mb': max(rss_values),
        'max_process_count': max(row['process_count'] for row in rows),
    }


def storage_preflight(
        output_root: Path, predicted_bytes: int) -> dict[str, Any]:
    """Require enough home space for one run plus the full retention cap."""
    home_free_bytes = shutil.disk_usage(Path.home()).free
    required_free_bytes = predicted_bytes + TOTAL_RETENTION_LIMIT_BYTES
    return {
        'home_path': str(Path.home().resolve()),
        'home_free_bytes': home_free_bytes,
        'required_free_bytes': required_free_bytes,
        'predicted_run_bytes': predicted_bytes,
        'output_path': str(output_root.resolve()),
        'passed': home_free_bytes >= required_free_bytes,
    }


def _start_resource_sampler(
        process_group: int, output: Path, log: Path,
        environment: dict[str, str]):
    """Start the existing sampler for one owned process group."""
    return _start([
        'python3', str(RESOURCE_SAMPLER),
        '--process-group', str(process_group),
        '--output', str(output), '--interval-s', '0.5',
    ], log, environment)


def write_recording_qos(path: Path) -> None:
    """Use non-blocking sensor subscriptions for representative evidence."""
    profile = {
        topic: {
            'history': 'keep_last', 'depth': 10,
            'reliability': 'best_effort', 'durability': 'volatile',
        }
        for topic in (
            '/ground_truth_pose', '/sim_raw/scan',
            '/collision_monitor_scan', '/odom')
    }
    path.write_text(yaml.safe_dump(profile, sort_keys=True))


def _wait_bag_ready(
        process: subprocess.Popen, bag_dir: Path, timeout_s: float = 10.0,
) -> None:
    """Wait until the MCAP writer has created its output file."""
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if process.poll() is not None:
            raise RuntimeError('representative recorder exited before ready')
        if any(bag_dir.glob('*.mcap')):
            return
        time.sleep(0.1)
    raise TimeoutError('representative recorder readiness timeout')


def inspect_representative_bag(
        bag_dir: Path, item: dict[str, Any], representative_topics: tuple,
        prior_error: str | None = None,
) -> tuple[dict[str, Any] | None, str | None, list[Path]]:
    """Inspect a finalized representative bag without escaping exceptions."""
    bag_paths = list(bag_dir.glob('*.mcap'))
    if prior_error is not None or len(bag_paths) != 1:
        error = prior_error or (
            f'expected one finalized MCAP, found {len(bag_paths)}')
        return None, error, bag_paths
    try:
        bag_path = bag_paths[0]
        metadata_path = bag_dir / 'metadata.yaml'
        expected_bag_path = bag_dir / 'bag_0.mcap'
        expected_entries = {expected_bag_path, metadata_path}
        if (bag_dir.is_symlink() or bag_dir.parent.is_symlink()
                or bag_path != expected_bag_path
                or bag_path.resolve().parent != bag_dir.absolute()
                or set(bag_dir.iterdir()) != expected_entries
                or any(path.is_symlink() or not path.is_file()
                       for path in expected_entries)
                or list(bag_dir.rglob('*.mcap')) != [expected_bag_path]):
            raise ValueError('non-canonical representative bag tree')
        metadata = yaml.safe_load(metadata_path.read_text())[
            'rosbag2_bagfile_information']
        with bag_path.open('rb') as stream:
            start_magic = stream.read(8)
            stream.seek(-8, 2)
            end_magic = stream.read(8)
        if (metadata['storage_identifier'] != 'mcap'
                or metadata['relative_file_paths'] != ['bag_0.mcap']
                or start_magic != b'\x89MCAP0\r\n'
                or end_magic != b'\x89MCAP0\r\n'):
            raise ValueError('finalized rosbag2 MCAP identity mismatch')
        evidence = {
            'run_id': item['run_id'],
            'scenario': item['scenario'],
            'seed': item['seed'],
            'path': str(bag_path.resolve()),
            'size_bytes': bag_path.stat().st_size,
            'sha256': sha256_file(bag_path),
            'metadata_size_bytes': metadata_path.stat().st_size,
            'metadata_sha256': sha256_file(metadata_path),
            'topic_inventory': metadata['topics_with_message_count'],
            'requested_topics': list(representative_topics),
        }
        return evidence, None, bag_paths
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
        return None, f'{type(error).__name__}: {error}', bag_paths


def _start_representative_recorder(
        enabled: bool, bag_dir: Path, qos_path: Path,
        representative_topics: tuple[str, ...], log_path: Path,
        environment: dict[str, str],
) -> tuple[subprocess.Popen | None, Any | None]:
    """Start the opt-in recorder only when hidden topics are supported."""
    if not enabled:
        return None, None
    help_result = subprocess.run(
        ['ros2', 'bag', 'record', '--help'],
        capture_output=True, check=False, text=True, timeout=10.0,
        env=environment)
    if (help_result.returncode != 0
            or HIDDEN_TOPIC_RECORD_FLAG not in help_result.stdout):
        raise RuntimeError('rosbag2 does not support hidden topic recording')
    recorder_command = [
        'ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_dir),
        HIDDEN_TOPIC_RECORD_FLAG,
        '--qos-profile-overrides-path', str(qos_path),
        '--topics', *representative_topics,
    ]
    if recorder_command.count(HIDDEN_TOPIC_RECORD_FLAG) != 1:
        raise RuntimeError('hidden topic recording flag is not exact')
    return _start(recorder_command, log_path, environment)


def _run_one(
        item: dict[str, Any], prepared_root: Path, output_root: Path,
        domain_id: int, record_bag: bool = False) -> Path:
    """Run one fresh simulation and return its evidence path."""
    run_dir = output_root / item['run_id']
    run_dir.mkdir(parents=True, exist_ok=False)
    contract_path = output_root / 'contract.json'
    contract = strict_json_loads(contract_path.read_text())
    representative_topics = tuple(
        contract['retention']['representative_topics'])
    predicted_bytes = 8 * 1024 * 1024
    storage = storage_preflight(output_root, predicted_bytes)
    if not storage['passed']:
        raise RuntimeError('home free space is below the retention reserve')
    environment = _environment(item['run_id'], domain_id)
    launched = []
    resource_samplers = []
    recorder = None
    recorder_stream = None
    bag_dir = run_dir / 'bag'
    bag_error = None
    measurement_finalized_steady_ns = None
    measurement_returncodes = {}
    process_identity = []
    evidence_path = run_dir / 'evidence.json'
    scan_gate_evidence_path = run_dir / 'scan_gate_evidence.json'
    scan_gate_source_path = (
        Path(__file__).resolve().parent.parent / 'jdamr_cube_navigation'
        / 'sim_scan_gate.py').resolve()
    try:
        gazebo = _start([
            'ros2', 'launch', 'jdamr_cube_gazebo', 'gazebo.launch.py',
            f'world:={prepared_root / "slam_corridor_contact.world"}',
            'urdf_file:='
            f'{prepared_root / "jdamr_cube_collision_monitor_eval.urdf"}',
            'bridge_config:='
            f'{prepared_root / "collision_monitor_bridge.yaml"}',
            'gui:=false', 'enable_image_bridges:=false',
            f'seed:={item["seed"]}', 'x_pose:=-8.0', 'y_pose:=0.0',
            'z_pose:=0.01',
        ], run_dir / 'gazebo.log', environment)
        launched.append(('gazebo', *gazebo, run_dir / 'gazebo.log'))
        resource_samplers.append((
            'gazebo', _start_resource_sampler(
                gazebo[0].pid, run_dir / 'gazebo_resources.jsonl',
                run_dir / 'gazebo_resource_sampler.log', environment)))
        _wait_topics(
            {'/sim_raw/scan', '/odom', '/ground_truth_pose'},
            environment, 60.0)
        gate = _start([
            'ros2', 'run', 'jdamr_cube_navigation', 'sim_scan_gate',
            '--contract', str(contract_path),
            '--evidence', str(scan_gate_evidence_path),
            '--run-id', item['run_id'], '--scenario', item['scenario'],
            '--seed', str(item['seed']),
        ], run_dir / 'scan_gate.log', environment)
        launched.append(('scan_gate', *gate, run_dir / 'scan_gate.log'))
        contact_bridge = _start([
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'{CONTACT_TOPIC}@ros_gz_interfaces/msg/Contacts'
            '[gz.msgs.Contacts',
        ], run_dir / 'contact_bridge.log', environment)
        launched.append((
            'contact_bridge', *contact_bridge,
            run_dir / 'contact_bridge.log'))
        navigation = _start([
            'ros2', 'launch', 'jdamr_cube_navigation',
            'navigation.launch.py',
            f'map:={prepared_root / "slam_corridor_eval.yaml"}',
            'params_file:='
            f'{prepared_root / "nav2_collision_monitor_eval.params.yaml"}',
            'use_keepout:=false', 'use_sim_time:=true',
            'use_composition:=True',
        ], run_dir / 'navigation.log', environment)
        launched.append((
            'navigation', *navigation, run_dir / 'navigation.log'))
        resource_samplers.append((
            'nav2', _start_resource_sampler(
                navigation[0].pid, run_dir / 'nav2_resources.jsonl',
                run_dir / 'nav2_resource_sampler.log', environment)))
        _wait_topics({
            '/scan', '/collision_monitor_scan', '/cmd_vel'},
            environment, 60.0,
            run_dir / 'navigation.log')
        _wait_lifecycle_active((
            '/amcl', '/controller_server', '/planner_server',
            '/bt_navigator', '/velocity_smoother', '/collision_monitor'),
            environment, 60.0)
        _wait_tf_available(
            'map', 'base_footprint', environment, 60.0,
            run_dir / 'tf_readiness.log')
        qos_path = run_dir / 'recording_qos.yaml'
        if record_bag:
            write_recording_qos(qos_path)
        recorder, recorder_stream = _start_representative_recorder(
            record_bag, bag_dir, qos_path, representative_topics,
            run_dir / 'recorder.log', environment)
        if record_bag:
            _wait_bag_ready(recorder, bag_dir)
        scenario = _start([
            'ros2', 'run', 'jdamr_cube_navigation',
            'sim_collision_monitor_scenario',
            '--scenario', item['scenario'], '--seed', str(item['seed']),
            '--output', str(evidence_path),
            '--contract', str(contract_path),
            '--contact-topic', CONTACT_TOPIC,
            '--ros-args', '-p', 'use_sim_time:=true',
        ], run_dir / 'scenario.log', environment)
        launched.append(('scenario', *scenario, run_dir / 'scenario.log'))
        if record_bag:
            deadline_s = time.monotonic() + 210.0
            while scenario[0].poll() is None:
                bag_bytes = tree_size_bytes(bag_dir)
                if not representative_bag_allowed(bag_bytes):
                    bag_error = 'representative bag exceeded 128 MiB cap'
                    _stop(scenario[0])
                    break
                if tree_size_bytes(output_root) > TOTAL_RETENTION_LIMIT_BYTES:
                    bag_error = 'promotion root exceeded 512 MiB cap'
                    _stop(scenario[0])
                    break
                if time.monotonic() >= deadline_s:
                    raise subprocess.TimeoutExpired('scenario', 210.0)
                time.sleep(0.25)
            returncode = scenario[0].wait(timeout=3.0)
        else:
            returncode = scenario[0].wait(timeout=210.0)
        if returncode != 0:
            raise RuntimeError(f'scenario_returncode:{returncode}')
        if not evidence_path.is_file():
            raise RuntimeError('scenario evidence missing after clean exit')
        measurement_finalized_steady_ns = time.monotonic_ns()
        all_processes = [
            (name, process) for name, process, _, _ in launched]
        all_processes.extend(
            (f'{name}_resource_sampler', process)
            for name, (process, _) in resource_samplers)
        if recorder is not None:
            all_processes.append(('recorder', recorder))
        measurement_returncodes = {
            name: process.poll() for name, process in all_processes}
        process_identity = [
            {'name': name, 'pid': process.pid}
            for name, process in all_processes]
    except Exception as error:
        if evidence_path.is_file():
            evidence = strict_json_loads(evidence_path.read_text())
            evidence['harness_error'] = (
                evidence.get('harness_error')
                or f'{type(error).__name__}: {error}')
        else:
            evidence = {
                'run_id': item['run_id'], 'scenario': item['scenario'],
                'seed': item['seed'], 'contract': contract,
                'harness_error': f'{type(error).__name__}: {error}',
                'activation_error': None,
            }
        evidence_path.write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + '\n')
    finally:
        groups = [entry[1].pid for entry in launched]
        teardown_processes = []
        teardown_errors = []
        bag_evidence = None
        if recorder is not None:
            record, error = _stop_with_evidence(
                'recorder', recorder, run_dir / 'recorder.log',
                measurement_finalized_steady_ns,
                measurement_returncodes.get('recorder'))
            teardown_processes.append(record)
            if error is not None:
                teardown_errors.append(error)
            if recorder_stream is not None:
                recorder_stream.close()
        for name, process, stream, log_path in reversed(launched):
            record, error = _stop_with_evidence(
                name, process, log_path, measurement_finalized_steady_ns,
                measurement_returncodes.get(name))
            teardown_processes.append(record)
            if error is not None:
                teardown_errors.append(error)
            stream.close()
        resource_evidence = {}
        for name, (process, stream) in resource_samplers:
            record, error = _stop_with_evidence(
                f'{name}_resource_sampler', process,
                run_dir / f'{name}_resource_sampler.log',
                measurement_finalized_steady_ns,
                measurement_returncodes.get(f'{name}_resource_sampler'),
                may_exit_after_parent=True)
            teardown_processes.append(record)
            if error is not None:
                teardown_errors.append(error)
            stream.close()
            trace_path = run_dir / f'{name}_resources.jsonl'
            if trace_path.is_file():
                resource_evidence[name] = summarize_resource_samples(
                    trace_path)
        survivors = _cleanup_identity(item['run_id'], domain_id)
        if recorder is not None:
            bag_evidence, bag_error, bag_paths = inspect_representative_bag(
                bag_dir, item, representative_topics, bag_error)
            if bag_error is not None:
                quarantine_path = run_dir / 'quarantine_manifest.json'
                quarantine_path.write_text(json.dumps({
                    'cleanup_performed': False,
                    'reason': bag_error,
                    'incomplete_bags': [str(path.resolve())
                                        for path in bag_paths],
                }, indent=2, sort_keys=True) + '\n')
        evidence = strict_json_loads(evidence_path.read_text())
        scan_gate_evidence = None
        if scan_gate_evidence_path.is_file():
            scan_gate_evidence = {
                'path': str(scan_gate_evidence_path.resolve()),
                'size_bytes': scan_gate_evidence_path.stat().st_size,
                'sha256': sha256_file(scan_gate_evidence_path),
            }
        evidence.update({
            'identity_process_groups': groups,
            'process_identity': process_identity,
            'identity_survivors': survivors,
            'production_hashes_unchanged': (
                production_hashes_unchanged(contract)),
            'full_matrix_bag_recorded': False,
            'representative_bag_recorded': record_bag,
            'resource_evidence': resource_evidence,
            'storage_preflight': storage,
            'representative_bag': bag_evidence,
            'scan_gate_evidence': scan_gate_evidence,
            'scan_gate_source': {
                'path': str(scan_gate_source_path),
                'size_bytes': scan_gate_source_path.stat().st_size,
                'sha256': sha256_file(scan_gate_source_path),
            },
            'canonical_contract_sha256': sha256_file(contract_path),
            'domain_id': domain_id,
        })
        gate_document = (
            strict_json_loads(scan_gate_evidence_path.read_text())
            if scan_gate_evidence_path.is_file() else None)
        evidence['scan_header_to_stop_observer_ros_signed_s'] = (
            (evidence['stop_state_ros_ns'] - gate_document['stamp_ns']) / 1e9
            if gate_document is not None
            and evidence.get('stop_state_ros_ns') is not None else None)
        if bag_error is not None:
            evidence['harness_error'] = (
                evidence.get('harness_error') or bag_error)
        if teardown_errors:
            evidence['harness_error'] = (
                evidence.get('harness_error')
                or f'teardown:{";".join(teardown_errors)}')
        evidence['teardown'] = {
            'measurement_finalized_steady_ns': (
                measurement_finalized_steady_ns),
            'processes': teardown_processes,
            'errors': teardown_errors,
            'status': 'PASS' if not teardown_errors else 'FAIL',
        }
        evidence_path.write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + '\n')
        evidence['retention_excluded_paths'] = [
            'evidence.json', 'evaluation.json']
        evidence['retained_run_bytes'] = sum(
            path.stat().st_size for path in run_dir.rglob('*')
            if path.is_file()
            and str(path.relative_to(run_dir))
            not in evidence['retention_excluded_paths'])
        evidence_path.write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + '\n')
    return evidence_path


def parse_selection(values: list[str] | None, key: str) -> set[Any] | None:
    """Parse an optional CLI filter into a typed set."""
    if not values:
        return None
    return {int(value) if key == 'seed' else value for value in values}


def validate_source_matrix(root: Path) -> dict[str, Any]:
    """Validate and hash-bind the accepted bags-off 15-run matrix."""
    expected_ids = {item['run_id'] for item in scenario_matrix()}
    paths = [root / item['run_id'] / 'evidence.json'
             for item in scenario_matrix()]
    if ({path.parent.name for path in paths} != expected_ids
            or len(paths) != len(expected_ids)):
        raise ValueError('source matrix is not the exact 15-run set')
    result = aggregate(paths)
    if result['status'] != 'PASS':
        raise ValueError('source matrix must freshly pass before promotion')
    aggregate_path = root / 'aggregate.json'
    final_path = root / 'final_summary.json'
    if not aggregate_path.is_file() or not final_path.is_file():
        raise ValueError('source matrix aggregate/final summary is missing')
    final = strict_json_loads(final_path.read_text())
    stored_aggregate = strict_json_loads(aggregate_path.read_text())
    recomputed_final = finalize(root)
    runtime_path = root / 'runtime_manifest.json'
    runtime = strict_json_loads(runtime_path.read_text())
    if (stored_aggregate != result
            or final != recomputed_final
            or final.get('matrix', {}).get('status') != 'PASS'
            or final.get('matrix', {}).get('run_count') != 15
            or final.get('aggregate_input_sha256')
            != sha256_file(aggregate_path)
            or final.get('contract_input_sha256')
            != sha256_file(root / 'contract.json')):
        raise ValueError('source matrix final summary binding mismatch')
    return {
        'path': str(root.resolve()),
        'contract_sha256': sha256_file(root / 'contract.json'),
        'preparation_manifest_sha256': sha256_file(
            root / 'preparation_manifest.json'),
        'aggregate_size_bytes': aggregate_path.stat().st_size,
        'aggregate_sha256': sha256_file(aggregate_path),
        'final_summary_size_bytes': final_path.stat().st_size,
        'final_summary_sha256': sha256_file(final_path),
        'runtime_manifest_size_bytes': runtime_path.stat().st_size,
        'runtime_manifest_sha256': sha256_file(runtime_path),
        'evaluation_rmw_identity': rmw_comparison_identity(
            runtime['evaluation_rmw']),
        'evidence_manifest': result['evidence_manifest'],
    }


def rmw_comparison_identity(rmw: dict[str, Any]) -> dict[str, Any]:
    """Return middleware fields that must match matrix and promotion."""
    keys = (
        'requested_identifier', 'actual_identifier', 'version', 'source',
        'library_size_bytes', 'library_sha256', 'environment_fields',
        'environment_sha256')
    return {key: rmw[key] for key in keys}


def promotion_rmw_matches(
        source_matrix: dict[str, Any], promotion_rmw: dict[str, Any],
) -> bool:
    """Require representative replay to use the matrix middleware bytes."""
    return source_matrix['evaluation_rmw_identity'] == (
        rmw_comparison_identity(promotion_rmw))


def main() -> int:
    """Execute fresh runs, fail on the first invalid result, and aggregate."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--prepared-root', type=Path)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument(
        '--rmw-implementation', default='rmw_fastrtps_cpp',
        choices=('rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp'))
    parser.add_argument('--rmw-overlay-prefix', type=Path)
    parser.add_argument('--scenario', action='append')
    parser.add_argument('--seed', action='append')
    parser.add_argument('--retain-representatives', action='store_true')
    parser.add_argument('--source-matrix-root', type=Path)
    args = parser.parse_args()
    source_matrix = None
    if args.retain_representatives:
        if args.source_matrix_root is None:
            raise SystemExit('--source-matrix-root is required for promotion')
        if args.prepared_root is not None:
            raise SystemExit('--prepared-root is not accepted for promotion')
        if args.scenario or args.seed:
            raise SystemExit('representative selection is fixed to seed 11')
        source_matrix = validate_source_matrix(args.source_matrix_root)
    elif args.prepared_root is None:
        raise SystemExit('--prepared-root is required for matrix evaluation')
    validate_domain_block(args.domain_id)
    configure_evaluation_rmw(
        args.rmw_implementation, args.rmw_overlay_prefix)
    if source_matrix is None:
        validate_preparation_manifest(args.prepared_root)
    scenarios = parse_selection(args.scenario, 'scenario')
    seeds = parse_selection(args.seed, 'seed')
    selected = [item for item in scenario_matrix()
                if ((args.retain_representatives and item['seed'] == 11)
                    or (not args.retain_representatives
                        and (scenarios is None
                             or item['scenario'] in scenarios)
                        and (seeds is None or item['seed'] in seeds)))]
    args.output_root.mkdir(parents=True, exist_ok=False)
    if source_matrix is None:
        runtime_prepared_root = materialize_prepared_assets(
            args.prepared_root, args.output_root)
    else:
        source_root = Path(source_matrix['path'])
        runtime_prepared_root = source_root / 'assets'
        shutil.copy2(source_root / 'contract.json',
                     args.output_root / 'contract.json')
        shutil.copy2(source_root / 'preparation_manifest.json',
                     args.output_root / 'preparation_manifest.json')
    output_contract_path = args.output_root / 'contract.json'
    if (source_matrix is not None
            and source_matrix['contract_sha256']
            != sha256_file(output_contract_path)):
        raise ValueError('promotion contract differs from source matrix')
    rmw = runtime_rmw_evidence(
        _environment('g004_rmw_probe', args.domain_id), args.domain_id)
    rmw = retain_rmw_library(rmw, args.output_root)
    (args.output_root / 'runtime_manifest.json').write_text(json.dumps({
        'evaluation_rmw': rmw,
        'canonical_contract_sha256': sha256_file(output_contract_path),
        'preparation_manifest_sha256': sha256_file(
            args.output_root / 'preparation_manifest.json'),
        'execution_mode': (
            'representative_promotion' if args.retain_representatives
            else 'matrix_evaluation'),
        'domain_id_base': args.domain_id,
        'domain_ids': [args.domain_id + index
                       for index in range(len(selected))],
        'planned_runs': [
            {'run_id': item['run_id'], 'domain_id': args.domain_id + index}
            for index, item in enumerate(selected)],
        'harness_sources': harness_source_records(args.output_root),
    }, indent=2, sort_keys=True) + '\n')
    if (source_matrix is not None
            and not promotion_rmw_matches(source_matrix, rmw)):
        raise ValueError('promotion RMW differs from source matrix')
    evidence_paths = []
    for index, item in enumerate(selected):
        predicted_bytes = 8 * 1024 * 1024
        if not retention_allowed(
                tree_size_bytes(args.output_root), predicted_bytes):
            raise RuntimeError('predicted retention exceeds 512 MiB cap')
        path = _run_one(
            item, runtime_prepared_root, args.output_root,
            args.domain_id + index,
            record_bag=args.retain_representatives)
        evidence_paths.append(path)
        result = evaluate_run(
            strict_json_loads(path.read_text()), path,
            strict_json_loads(output_contract_path.read_text()),
            sha256_file(output_contract_path),
            representative_run=args.retain_representatives)
        (path.parent / 'evaluation.json').write_text(
            json.dumps(result, indent=2, sort_keys=True) + '\n')
        if result['status'] != 'PASS':
            break
    if args.retain_representatives:
        records = [record for record in (
            strict_json_loads(path.read_text()).get('representative_bag')
            for path in evidence_paths) if isinstance(record, dict)]
        contract = strict_json_loads(output_contract_path.read_text())
        checks = _representative_retention_checks(
            {'representative_bags': records}, len(records) == 3,
            contract, args.output_root)
        checks['promotion_runtime_identity'] = promotion_runtime_valid(
            args.output_root, records, source_matrix)
        total_bytes = tree_size_bytes(args.output_root)
        checks['promotion_total_retention'] = (
            total_bytes <= TOTAL_RETENTION_LIMIT_BYTES)
        retained_files = [{
            'relative_path': str(path.relative_to(args.output_root)),
            'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path),
        } for path in sorted(args.output_root.rglob('*')) if path.is_file()
            and path.name != 'promotion_manifest.json']
        retained_directories = sorted(
            str(path.relative_to(args.output_root))
            for path in args.output_root.rglob('*') if path.is_dir())
        checks['promotion_file_exact_set'] = True
        checks['promotion_directory_exact_set'] = True
        summary = {
            'schema_version': 1,
            'status': ('PASS' if len(records) == 3
                       and all(checks.values()) else 'FAIL'),
            'source_matrix': source_matrix,
            'representative_bags': records,
            'checks': checks,
            'retained_bytes_excluding_manifest': total_bytes,
            'retained_file_manifest': retained_files,
            'retained_directories': retained_directories,
        }
        (args.output_root / 'promotion_manifest.json').write_text(
            json.dumps(summary, indent=2, sort_keys=True) + '\n')
        return 0 if summary['status'] == 'PASS' else 2
    summary = aggregate(evidence_paths)
    (args.output_root / 'aggregate.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n')
    retention_manifest = {
        'full_matrix_bag_recording': False,
        'representative_opt_in': False,
        'retained_bytes': tree_size_bytes(args.output_root),
        'total_limit_bytes': TOTAL_RETENTION_LIMIT_BYTES,
        'individual_bag_limit_bytes': REPRESENTATIVE_BAG_LIMIT_BYTES,
        'quarantine_cleanup_performed': False,
        'quarantine_paths': [],
    }
    (args.output_root / 'retention_manifest.json').write_text(
        json.dumps(retention_manifest, indent=2, sort_keys=True) + '\n')
    return 0 if summary['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
