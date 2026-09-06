#!/usr/bin/env python3
"""Run and validate the G002 AMCL particle determinism preflight."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import statistics
import subprocess
import tempfile
import time

from amcl_fault_contract import canonical_json_bytes
from amcl_fault_contract import current_free_bytes
from amcl_fault_contract import exact_regular_file
from amcl_fault_contract import REAL_BAG_SHA256
from amcl_fault_contract import REAL_MAP_PGM_SHA256
from amcl_fault_contract import REAL_MAP_YAML_SHA256
from amcl_fault_contract import sha256_file
from amcl_fault_contract import STORAGE_LIMITS
from amcl_fault_contract import strict_json_load
from amcl_fault_contract import validate_storage_budget
from g002_tf_sanitizer import validate_sanitized_bag


ROOT = Path(__file__).resolve().parents[2]
NAV = ROOT / 'jdamr_cube_navigation'
OBSERVER = Path(__file__).resolve().parent / 'amcl_particle_observer.py'
SAMPLER = Path(__file__).resolve().parent / 'sample_process_group_resources.py'
ROS2 = Path('/opt/ros/jazzy/bin/ros2')
MAP_SERVER = Path('/opt/ros/jazzy/lib/nav2_map_server/map_server')
ALLOWED_SEEDS = (11, 23)
MAX_LOG_BYTES = 2 * 1024 * 1024
MAX_ARTIFACT_BYTES = 32 * 1024 * 1024
AMCL_TF_ERROR_MARKERS = (
    'Message Filter dropping message',
    'Failed to transform',
    'Lookup would require extrapolation',
)
CLAIM_SMOKE = 'AMCL_PARTICLE_PIPELINE_SMOKE_NO_MOTION'
CLAIM_FULL = 'AMCL_PARTICLE_DETERMINISM_PREFLIGHT_NO_MOTION'
FULL_SEEDS = (11, 11, 23)
FULL_CLOUD_COUNT = 30
FULL_SCAN_COUNT = 1937


def _event(name: str) -> dict:
    return {'name': name, 'steady_ns': time.monotonic_ns(),
            'wall_ns': time.time_ns()}


def _identity(path: Path) -> dict:
    return exact_regular_file(path)


def _relative_identity(path: Path, root: Path) -> dict:
    record = _identity(path)
    return {'relative_path': str(path.relative_to(root)),
            'size_bytes': record['size_bytes'], 'sha256': record['sha256']}


def _validate_source_records(records: dict) -> None:
    for record in records.values():
        actual = _identity(Path(record['path']))
        if (actual['size_bytes'] != record['size_bytes'] or
                actual['sha256'] != record['sha256']):
            raise ValueError('prepared source or production identity drift')


def _tree_bytes(root: Path) -> int:
    return sum(path.stat().st_size for path in root.rglob('*')
               if path.is_file() and not path.is_symlink())


def _payload_tree_bytes(root: Path) -> int:
    return sum(path.stat().st_size for path in root.rglob('*')
               if path.is_file() and not path.is_symlink() and
               path.name != 'preflight_manifest.json')


def _exact_keys(value: dict, expected: set[str], label: str) -> None:
    if type(value) is not dict or set(value) != expected:
        raise ValueError(f'{label} schema drift')


def _runtime_guard(run_dir: Path) -> None:
    if current_free_bytes(run_dir) < STORAGE_LIMITS['abort_free_floor_bytes']:
        raise RuntimeError('runtime free-space floor crossed')
    for path in run_dir.glob('*.log'):
        if path.stat().st_size > MAX_LOG_BYTES:
            raise RuntimeError(f'run log exceeds 2 MiB cap: {path.name}')
    if _tree_bytes(run_dir) > STORAGE_LIMITS['run_output_limit_bytes']:
        raise RuntimeError('run output exceeds 2 MiB cap')


def _canonical_output_root(path: Path) -> Path:
    path = path.expanduser()
    if (not path.is_absolute() or path.exists() or path.is_symlink() or
            path.parent != path.parent.resolve()):
        raise ValueError('output root must be absent and canonical')
    return path


def _start(name: str, command: list[str], log_path: Path,
           env: dict[str, str]) -> dict:
    stream = log_path.open('wb')
    process = subprocess.Popen(
        command, stdout=stream, stderr=subprocess.STDOUT,
        env=env, start_new_session=True)
    return {'name': name, 'command': command, 'process': process,
            'log_path': log_path, 'stream': stream, 'pid': process.pid,
            'pgid': os.getpgid(process.pid), 'started': _event('started')}


def _members(pgid: int) -> list[int]:
    result = []
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text(encoding='utf-8').split()
            if int(fields[4]) == pgid:
                result.append(int(path.parent.name))
        except (OSError, ValueError, IndexError):
            continue
    return sorted(result)


def _stop(record: dict, run_dir: Path) -> dict:
    process = record['process']
    stages = []
    if process.poll() is None:
        for sig, wait_s in ((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0),
                            (signal.SIGKILL, 2.0)):
            if process.poll() is not None:
                break
            os.killpg(record['pgid'], sig)
            stages.append({'signal': sig.name, 'steady_ns': time.monotonic_ns()})
            try:
                process.wait(timeout=wait_s)
            except subprocess.TimeoutExpired:
                pass
    record['stream'].close()
    return {'name': record['name'], 'pid': record['pid'],
            'pgid': record['pgid'], 'command': record['command'],
            'started': record['started'],
            'returncode': process.poll(), 'stop_stages': stages,
            'survivors': _members(record['pgid']),
            'log': _relative_identity(record['log_path'], run_dir)}


def _run_cli(args: list[str], env: dict[str, str], timeout_s: float = 20.0
             ) -> dict:
    started = _event('request')
    result = subprocess.run(
        [str(ROS2), *args], env=env, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, timeout=timeout_s, check=False)
    return {'command': [str(ROS2), *args], 'started': started,
            'completed': _event('response'), 'returncode': result.returncode,
            'output': result.stdout.decode('utf-8', errors='strict')}


def _wait_state(path: Path, predicate, timeout_s: float) -> dict:
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        if path.is_file():
            try:
                last = strict_json_load(path)
                if predicate(last):
                    return last
            except (OSError, ValueError, json.JSONDecodeError):
                pass
        _runtime_guard(path.parent)
        time.sleep(0.02)
    raise TimeoutError(f'state timeout: {path}: {last}')


def _wait_measurement(path: Path, player: dict, timeout_s: float) -> dict:
    deadline = time.monotonic() + timeout_s
    player_ended_at = None
    state = None
    while time.monotonic() < deadline:
        if path.is_file():
            state = strict_json_load(path)
            if state['done']:
                return state
        _runtime_guard(path.parent)
        if player['process'].poll() is not None:
            if player_ended_at is None:
                player_ended_at = time.monotonic()
            elif time.monotonic() - player_ended_at > 2.0:
                raise RuntimeError(
                    f'player ended before cloud limit: {state}')
        time.sleep(0.02)
    raise TimeoutError(f'measurement timeout: {path}')


def _select_tf_bootstrap_plan(scans: list[tuple[int, int]],
                              odom_base_tf: list[tuple[int, int]],
                              source_start_ns: int) -> dict:
    """Select the first scan with a source-recorded TF time bracket."""
    for index, (scan_storage_ns, scan_header_ns) in enumerate(scans[1:], 1):
        before = [item for item in odom_base_tf
                  if item[1] <= scan_header_ns]
        after = [item for item in odom_base_tf
                 if item[1] >= scan_header_ns]
        if not before or not after:
            continue
        lower = max(before, key=lambda item: item[1])
        upper = min(after, key=lambda item: item[1])
        if upper[0] >= scan_storage_ns:
            continue
        previous_storage_ns = scans[index - 1][0]
        start_offset_ns = (
            previous_storage_ns + scan_storage_ns) // 2 - source_start_ns
        if start_offset_ns <= upper[0] - source_start_ns:
            continue
        return {
            'source_start_storage_ns': source_start_ns,
            'previous_scan_storage_ns': previous_storage_ns,
            'first_main_scan_storage_ns': scan_storage_ns,
            'first_main_scan_header_ns': scan_header_ns,
            'lower_tf_storage_ns': lower[0],
            'lower_tf_header_ns': lower[1],
            'upper_tf_storage_ns': upper[0],
            'upper_tf_header_ns': upper[1],
            'start_offset_ns': start_offset_ns,
            'start_offset_s': start_offset_ns / 1_000_000_000,
        }
    raise ValueError('sanitized bag has no scan with odom TF bracket')


def _tf_bootstrap_plan(sanitized_root: Path) -> dict:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(sanitized_root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    scans = []
    odom_base_tf = []
    source_start_ns = None
    while reader.has_next():
        topic, serialized, storage_ns = reader.read_next()
        if source_start_ns is None:
            source_start_ns = storage_ns
        if topic == '/scan':
            message = deserialize_message(serialized, LaserScan)
            scans.append((storage_ns,
                          int(message.header.stamp.sec) * 1_000_000_000 +
                          int(message.header.stamp.nanosec)))
        elif topic == '/tf':
            message = deserialize_message(serialized, TFMessage)
            for transform in message.transforms:
                if (transform.header.frame_id == 'odom' and
                        transform.child_frame_id == 'base_footprint'):
                    header_ns = (
                        int(transform.header.stamp.sec) * 1_000_000_000 +
                        int(transform.header.stamp.nanosec))
                    odom_base_tf.append((storage_ns, header_ns))
    reader.close()
    if source_start_ns is None:
        raise ValueError('sanitized bag is empty')
    return _select_tf_bootstrap_plan(scans, odom_base_tf, source_start_ns)


def _semantic_tf_payload(serialized: bytes) -> bytes:
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    message = deserialize_message(serialized, TFMessage)
    records = []
    for transform in message.transforms:
        values = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        if any(not math.isfinite(float(value)) for value in values):
            raise ValueError('non-finite transform in replay view')
        records.append({
            'stamp_ns': (int(transform.header.stamp.sec) * 1_000_000_000 +
                         int(transform.header.stamp.nanosec)),
            'parent': transform.header.frame_id,
            'child': transform.child_frame_id,
            'pose': list(map(float, values)),
        })
    return canonical_json_bytes(records)


def _replay_view_manifest(sanitized_root: Path, bootstrap: dict,
                          last_scan_header_ns: int) -> dict:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(sanitized_root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    rows = []
    last_scan_storage_ns = None
    while reader.has_next():
        topic, serialized, storage_ns = reader.read_next()
        header_ns = None
        if topic == '/scan':
            scan = deserialize_message(serialized, LaserScan)
            header_ns = (int(scan.header.stamp.sec) * 1_000_000_000 +
                         int(scan.header.stamp.nanosec))
            if header_ns == last_scan_header_ns:
                last_scan_storage_ns = storage_ns
        if topic in ('/scan', '/odom', '/tf', '/tf_static'):
            rows.append((topic, storage_ns, header_ns, bytes(serialized)))
    reader.close()
    if last_scan_storage_ns is None:
        raise ValueError('last observed scan missing from sanitized input')

    def summarize(selected):
        result = {}
        for topic in ('/scan', '/odom', '/tf', '/tf_static'):
            topic_rows = [row for row in selected if row[0] == topic]
            raw = hashlib.sha256()
            storage = hashlib.sha256()
            semantic = hashlib.sha256()
            for _, storage_ns, _, serialized in topic_rows:
                raw.update(serialized)
                storage.update(f'{storage_ns}\n'.encode())
                if topic in ('/tf', '/tf_static'):
                    semantic.update(_semantic_tf_payload(serialized))
            result[topic] = {
                'message_count': len(topic_rows),
                'storage_stamp_sha256': storage.hexdigest(),
                'raw_payload_sha256': raw.hexdigest(),
                'tf_semantic_sha256': (semantic.hexdigest()
                                       if topic.startswith('/tf') else None),
            }
        return result

    split_ns = bootstrap['source_start_storage_ns'] + bootstrap['start_offset_ns']
    prelude = [row for row in rows if row[0] != '/scan' and row[1] < split_ns]
    main = [row for row in rows if split_ns <= row[1] <= last_scan_storage_ns]
    skipped_scans = [row for row in rows
                     if row[0] == '/scan' and row[1] < split_ns]
    return {
        'schema_version': 1,
        'source_sanitizer_manifest_sha256': sha256_file(
            sanitized_root / 'sanitizer_manifest.json'),
        'bootstrap': bootstrap,
        'skipped_scan_count': len(skipped_scans),
        'prelude_topics': ['/odom', '/tf', '/tf_static'],
        'main_topics': ['/scan', '/odom', '/tf', '/tf_static'],
        'prelude': summarize(prelude),
        'main': summarize(main),
        'prelude_last_storage_ns': max(row[1] for row in prelude),
        'main_first_storage_ns': min(row[1] for row in main),
        'main_last_scan_storage_ns': last_scan_storage_ns,
        'storage_gap_ns': min(row[1] for row in main) -
        max(row[1] for row in prelude),
        'storage_overlap_count': 0,
    }


def _loaded_runtime_files(pid: int, executable: Path,
                          rmw_library: Path) -> dict:
    library_root = executable.parent.parent
    expected = {
        'amcl_executable': executable,
        'libamcl_core': library_root / 'libamcl_core.so',
        'libpf_lib': library_root / 'libpf_lib.so',
        'rmw_library': rmw_library,
    }
    mapped = set()
    for line in Path(f'/proc/{pid}/maps').read_text(encoding='utf-8').splitlines():
        candidate = line.split()[-1]
        if candidate.startswith('/'):
            mapped.add(str(Path(candidate).resolve()))
    actual_executable = Path(f'/proc/{pid}/exe').resolve()
    if actual_executable != executable:
        raise RuntimeError('AMCL executable mapping drift')
    result = {}
    for name, path in expected.items():
        path = path.resolve()
        if name != 'amcl_executable' and str(path) not in mapped:
            raise RuntimeError(f'AMCL required library not loaded: {name}')
        result[name] = _identity(path)
    return result


def _needed_sonames(files: list[Path]) -> dict[str, list[str]]:
    records = {}
    for path in files:
        result = subprocess.run(
            ['/usr/bin/readelf', '-d', str(path)], check=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        names = re.findall(r'\(NEEDED\).*?\[(.*?)\]', result.stdout)
        if len(names) != len(set(names)):
            raise ValueError('duplicate ELF NEEDED entry')
        records[path.name] = sorted(names)
    return records


def _build_provenance(executable: Path, overlay_root: Path,
                      loaded_runtime: dict) -> dict:
    build_root = executable.parents[4]
    cache = build_root / 'build/nav2_amcl/CMakeCache.txt'
    build_rc = build_root / 'build/nav2_amcl/colcon_build.rc'
    cache_text = cache.read_text(encoding='utf-8')
    source_matches = re.findall(
        r'^CMAKE_HOME_DIRECTORY:INTERNAL=(.*)$', cache_text, re.MULTILINE)
    if source_matches != [str(overlay_root / 'nav2_amcl')]:
        raise ValueError('AMCL build source directory drift')
    if build_rc.read_text(encoding='utf-8').strip() != '0':
        raise ValueError('AMCL build did not complete successfully')
    library_root = executable.parent.parent
    files = [executable, library_root / 'libamcl_core.so',
             library_root / 'libpf_lib.so']
    return {
        'schema_version': 1,
        'build_root': str(build_root),
        'source_root': str(overlay_root / 'nav2_amcl'),
        'cmake_cache': _identity(cache),
        'build_returncode_file': _identity(build_rc),
        'install_files': {path.name: _identity(path) for path in files},
        'needed_sonames': _needed_sonames(
            [executable, library_root / 'libamcl_core.so']),
        'loaded_runtime': loaded_runtime,
    }


def _amcl_tf_error_counts(path: Path) -> dict[str, int]:
    content = path.read_text(encoding='utf-8', errors='strict')
    return {marker: content.count(marker) for marker in AMCL_TF_ERROR_MARKERS}


def _observer_event(observer: dict, name: str) -> dict:
    matches = [event for event in observer['events'] if event['name'] == name]
    if len(matches) != 1:
        raise ValueError(f'observer event must occur exactly once: {name}')
    event = matches[0]
    if (type(event['steady_ns']) is not int or event['steady_ns'] <= 0 or
            type(event['ros_ns']) is not int or event['ros_ns'] < 0):
        raise ValueError(f'observer event timestamp invalid: {name}')
    return event


def _validate_replay_bootstrap(evidence: dict, run_dir: Path,
                               sanitized_root: Path,
                               expected: dict | None = None) -> None:
    if expected is None:
        if not sanitized_root.exists():
            raise ValueError('live sanitized input required for bootstrap')
        expected = _tf_bootstrap_plan(sanitized_root)
    elif sanitized_root.exists() and _tf_bootstrap_plan(sanitized_root) != expected:
        raise ValueError('live TF bootstrap differs from snapshot')
    if evidence['tf_bootstrap'] != expected:
        raise ValueError('TF bootstrap plan drift')
    errors = _amcl_tf_error_counts(run_dir / 'amcl.log')
    if evidence['amcl_tf_error_counts'] != errors or any(errors.values()):
        raise ValueError('AMCL transform lookup error observed')
    processes = evidence['teardown']
    by_name = {process['name']: process for process in processes}
    expected_names = {
        'map_server', 'amcl', 'observer', 'resource_sampler',
        'tf_prelude_player', 'player'}
    if len(by_name) != len(processes) or set(by_name) != expected_names:
        raise ValueError('run process inventory drift')
    prelude = by_name['tf_prelude_player']
    player = by_name['player']
    if (prelude['returncode'] != 0 or player['returncode'] != 0 or
            prelude['survivors'] or player['survivors']):
        raise ValueError('replay player teardown failed')
    expected_prelude = [
        str(ROS2), 'bag', 'play', str(sanitized_root), '--storage', 'mcap',
        '--topics', '/odom', '/tf', '/tf_static', '--clock-topics-all',
        '--disable-keyboard-controls', '--playback-duration',
        str(expected['start_offset_s']), '--rate',
        str(evidence['playback_rate'])]
    expected_main = [
        str(ROS2), 'bag', 'play', str(sanitized_root), '--storage', 'mcap',
        '--topics', '/scan', '/odom', '/tf', '/tf_static',
        '--clock-topics-all', '--start-paused', '--disable-keyboard-controls',
        '--start-offset', str(expected['start_offset_s']),
        '--playback-duration', str(evidence['prefix_s']), '--rate',
        str(evidence['playback_rate'])]
    if prelude['command'] != expected_prelude or player['command'] != expected_main:
        raise ValueError('replay player command drift')
    completed = [event for event in evidence['events']
                 if event['name'] == 'tf_prelude_completed']
    if len(completed) != 1 or not (
            prelude['started']['steady_ns'] <
            completed[0]['steady_ns'] < player['started']['steady_ns']):
        raise ValueError('prelude and main player order drift')
    initialpose = _observer_event(evidence['observer'], 'initialpose_published')
    first_scan = _observer_event(evidence['observer'], 'first_scan_received')
    first_cloud = _observer_event(
        evidence['observer'], 'first_post_scan_cloud_received')
    if not (initialpose['steady_ns'] < first_scan['steady_ns'] <
            first_cloud['steady_ns'] and
            initialpose['ros_ns'] <= first_scan['ros_ns'] <=
            first_cloud['ros_ns']):
        raise ValueError('initialpose, scan, and cloud causal order drift')
    if (evidence['observer']['scan_header_stamps_ns'][0] !=
            expected['first_main_scan_header_ns']):
        raise ValueError('first main scan does not match TF bootstrap plan')


def _resource_summary(path: Path, run_dir: Path) -> dict:
    rows = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()
            if line]
    finite_cpu = [row['cpu_pct_one_core'] for row in rows
                  if isinstance(row['cpu_pct_one_core'], (int, float))]
    return {
        'sample_count': len(rows),
        'cpu_total_start_s': rows[0]['cpu_total_s'],
        'cpu_total_end_s': rows[-1]['cpu_total_s'],
        'cpu_seconds': rows[-1]['cpu_total_s'] - rows[0]['cpu_total_s'],
        'cpu_pct_median': statistics.median(finite_cpu) if finite_cpu else 0.0,
        'cpu_pct_p95': (sorted(finite_cpu)[int(0.95 * (len(finite_cpu) - 1))]
                        if finite_cpu else 0.0),
        'max_rss_mb': max(row['rss_mb'] for row in rows),
        'file': _relative_identity(path, run_dir),
    }


def _scan_parity(observer: dict, sanitized_root: Path) -> dict:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(sanitized_root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    source = []
    while reader.has_next():
        topic, serialized, storage_ns = reader.read_next()
        if topic == '/scan':
            message = deserialize_message(serialized, LaserScan)
            header_ns = (int(message.header.stamp.sec) * 1_000_000_000 +
                         int(message.header.stamp.nanosec))
            source.append((header_ns, storage_ns, bytes(serialized)))
    reader.close()
    headers = observer['scan_header_stamps_ns']
    if not headers:
        raise ValueError('no accepted scan prefix')
    candidates = [index for index, item in enumerate(source)
                  if item[0] == headers[0]]
    if len(candidates) != 1:
        raise ValueError('scan prefix start is not unique in source')
    selected = source[candidates[0]:candidates[0] + len(headers)]
    if [item[0] for item in selected] != headers:
        raise ValueError('accepted scan prefix is not contiguous source input')
    payload_digest = hashlib.sha256()
    storage_digest = hashlib.sha256()
    for _, storage_ns, serialized in selected:
        payload_digest.update(serialized)
        storage_digest.update(f'{storage_ns}\n'.encode())
    result = {
        'message_count': len(selected),
        'header_stamp_sha256': hashlib.sha256(''.join(
            f'{value}\n' for value in headers).encode()).hexdigest(),
        'storage_stamp_sha256': storage_digest.hexdigest(),
        'payload_sha256': payload_digest.hexdigest(),
        'first_header_stamp_ns': headers[0],
        'last_header_stamp_ns': headers[-1],
    }
    if (result['message_count'] != observer['scan_count'] or
            result['header_stamp_sha256'] !=
            observer['scan_header_stamp_sha256'] or
            result['payload_sha256'] != observer['scan_payload_sha256']):
        raise ValueError('published scan prefix parity drift')
    return result


def _run_one(run_dir: Path, seed: int, domain_id: int, args,
             env_base: dict[str, str], bootstrap: dict) -> dict:
    profile = getattr(args, 'profile', 'P0')
    run_id = getattr(
        args, 'run_id', f'p0__seed_{seed}__attempt_{args.attempt_index}')
    run_dir.mkdir()
    env = dict(env_base)
    env.update({'ROS_DOMAIN_ID': str(domain_id),
                'ROS_LOCALHOST_ONLY': '1',
                'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'})
    state = run_dir / 'observer_state.json'
    request = run_dir / 'initialpose.request'
    events = [_event('run_started')]
    launched = []
    operations = []
    observer_source = _identity(OBSERVER)
    loaded_runtime = None
    prelude_observer = None
    try:
        launched.append(_start('map_server', [
            str(MAP_SERVER), '--ros-args', '-r', '__node:=map_server',
            '-p', f'yaml_filename:={args.map_yaml}',
            '-p', 'use_sim_time:=true'], run_dir / 'map_server.log', env))
        amcl_command = [
            str(args.amcl_executable), '--ros-args',
            '--params-file', str(args.params_file),
            '-p', 'use_sim_time:=true', '-p', 'set_initial_pose:=false',
            '-p', f'random_seed:={seed}']
        for name, value in getattr(args, 'amcl_overrides', ()):
            amcl_command.extend(['-p', f'{name}:={value}'])
        amcl = _start(
            'amcl', amcl_command, run_dir / 'amcl.log', env)
        launched.append(amcl)
        launched.append(_start('observer', [
            '/usr/bin/python3', str(OBSERVER), '--run-id', run_id,
            '--seed', str(seed), '--max-clouds', str(args.max_clouds),
            '--state', str(state), '--initialpose-request', str(request),
            '--initial-x-m', str(getattr(args, 'initial_x_m', 0.0)),
            '--initial-y-m', str(getattr(args, 'initial_y_m', 0.0)),
            '--initial-yaw-rad', str(getattr(args, 'initial_yaw_rad', 0.0))],
            run_dir / 'observer.log', env))
        resource = _start('resource_sampler', [
            '/usr/bin/python3', str(SAMPLER), '--process-group',
            str(amcl['pgid']), '--output', str(run_dir / 'amcl_resource.jsonl'),
            '--interval-s', '0.5'], run_dir / 'resource_sampler.log', env)
        launched.append(resource)
        time.sleep(1.0)
        for node, transition in (
                ('/map_server', 'configure'), ('/map_server', 'activate')):
            operation = _run_cli(['lifecycle', 'set', node, transition], env)
            operations.append(operation)
            if operation['returncode'] != 0 or 'successful' not in operation['output']:
                raise RuntimeError(f'lifecycle failed: {node} {transition}')
        _wait_state(state, lambda value: value['readiness']['map_count'] > 0, 20.0)
        for transition in ('configure', 'activate'):
            operation = _run_cli(['lifecycle', 'set', '/amcl', transition], env)
            operations.append(operation)
            if operation['returncode'] != 0 or 'successful' not in operation['output']:
                raise RuntimeError(f'AMCL lifecycle failed: {transition}')
        readback = _run_cli(['param', 'get', '/amcl', 'random_seed'], env)
        operations.append(readback)
        readback_values = re.findall(
            r'^Integer value is: (-?[0-9]+)$', readback['output'],
            re.MULTILINE)
        if (readback['returncode'] != 0 or
                readback_values != [str(seed)]):
            raise RuntimeError('AMCL random_seed readback mismatch')
        loaded_runtime = _loaded_runtime_files(
            amcl['pid'], args.amcl_executable, args.rmw_library)
        prelude = _start('tf_prelude_player', [
            str(ROS2), 'bag', 'play', str(args.sanitized_root),
            '--storage', 'mcap', '--topics', '/odom', '/tf', '/tf_static',
            '--clock-topics-all', '--disable-keyboard-controls',
            '--playback-duration', str(bootstrap['start_offset_s']),
            '--rate', str(args.playback_rate)],
            run_dir / 'tf_prelude_player.log', env)
        launched.append(prelude)
        if prelude['process'].wait(timeout=20.0) != 0:
            raise RuntimeError('TF prelude player failed')
        events.append(_event('tf_prelude_completed'))
        prelude_observer = _wait_state(
            state, lambda value: value['readiness']['odom_count'] > 0 and
            value['readiness']['tf_count'] > 0 and
            value['readiness']['tf_static_count'] > 0 and
            value['readiness']['clock_count'] > 0, 20.0)
        player = _start('player', [
            str(ROS2), 'bag', 'play', str(args.sanitized_root),
            '--storage', 'mcap', '--topics', '/scan', '/odom', '/tf',
            '/tf_static', '--clock-topics-all', '--start-paused',
            '--disable-keyboard-controls', '--start-offset',
            str(bootstrap['start_offset_s']),
            '--playback-duration', str(args.prefix_s),
            '--rate', str(args.playback_rate)],
            run_dir / 'player.log', env)
        launched.append(player)
        _wait_state(state, lambda value: value['publisher_matched_count'] > 0, 20.0)
        before_init = strict_json_load(state)
        if (before_init['scan_count'] != 0 or before_init['clouds'] or
                before_init['pre_initial_cloud_count'] != 0):
            raise RuntimeError('scan or particle observed before initialization')
        request.write_text('publish once\n', encoding='utf-8')
        _wait_state(state, lambda value: value['initialpose_count'] == 1, 10.0)
        resume = _run_cli([
            'service', 'call', '/rosbag2_player/resume',
            'rosbag2_interfaces/srv/Resume', '{}'], env)
        operations.append(resume)
        if resume['returncode'] != 0:
            raise RuntimeError('player resume failed')
        final = _wait_measurement(state, player, 180.0)
        if final['failure'] is not None:
            raise RuntimeError(final['failure'])
        events.append(_event('measurement_finalized'))
        status = 'PASS'
        failure = None
    except Exception as exc:
        final = strict_json_load(state) if state.is_file() else None
        status = 'FAIL'
        failure = f'{type(exc).__name__}: {exc}'
    finally:
        teardown = [_stop(record, run_dir) for record in reversed(launched)]
    survivors = [pid for record in teardown for pid in record['survivors']]
    if survivors:
        status = 'FAIL'
        failure = f'surviving processes: {survivors}'
    resource_path = run_dir / 'amcl_resource.jsonl'
    resource_summary = (_resource_summary(resource_path, run_dir)
                        if resource_path.is_file() else None)
    amcl_tf_error_counts = _amcl_tf_error_counts(run_dir / 'amcl.log')
    transform_lookup_drop_count = amcl_tf_error_counts[
        'Message Filter dropping message']
    if any(amcl_tf_error_counts.values()):
        status = 'FAIL'
        failure = 'AMCL transform lookup drop observed'
    evidence = {
        'schema_version': 1, 'run_id': run_id, 'profile': profile,
        'seed': seed, 'domain_id': domain_id, 'status': status,
        'failure': failure, 'events': events, 'operations': operations,
        'observer_source': observer_source, 'observer': final,
        'amcl_executable': _identity(args.amcl_executable),
        'map_yaml': _identity(args.map_yaml),
        'params_file': _identity(args.params_file),
        'sanitized_manifest': _identity(
            args.sanitized_root / 'sanitizer_manifest.json'),
        'resource': resource_summary, 'teardown': teardown,
        'loaded_runtime': loaded_runtime,
        'prelude_observer': prelude_observer,
        'tf_bootstrap': bootstrap,
        'amcl_tf_error_counts': amcl_tf_error_counts,
        'transform_lookup_drop_count': transform_lookup_drop_count,
        'prefix_s': args.prefix_s, 'playback_rate': args.playback_rate,
        'survivor_count': len(survivors),
        'output_mcap_count': len(list(run_dir.rglob('*.mcap'))),
        'cmd_vel_publisher': 'NOT_APPLICABLE',
    }
    if final is not None and final['scan_count']:
        evidence['scan_parity'] = _scan_parity(final, args.sanitized_root)
    else:
        evidence['scan_parity'] = None
    (run_dir / 'evidence.json').write_bytes(canonical_json_bytes(evidence))
    if _tree_bytes(run_dir) > STORAGE_LIMITS['run_output_limit_bytes']:
        raise RuntimeError('run metrics/log output exceeds 2 MiB cap')
    return evidence


def _comparison(runs: list[dict]) -> dict:
    passed = [run for run in runs if run['status'] == 'PASS']
    result = {'evaluated': len(runs) == 3, 'same_seed_equal': None,
              'different_seed_differs': None}
    if len(runs) != 3 or len(passed) != 3:
        return result
    sequences = [[item['payload_sha256']
                  for item in run['observer']['clouds']] for run in runs]
    counts = [len(value) for value in sequences]
    result['same_seed_equal'] = (
        runs[0]['seed'] == runs[1]['seed'] == 11 and
        counts[0] == counts[1] and sequences[0] == sequences[1])
    result['different_seed_differs'] = (
        runs[2]['seed'] == 23 and counts[2] > 0 and
        any(left != right for left, right in zip(sequences[0], sequences[2])))
    return result


def _validate_sanitizer_snapshot(manifest: dict) -> None:
    _exact_keys(manifest, {
        'schema_version', 'source', 'output_topics', 'input_topic_inventory',
        'removed_map_to_odom_transforms', 'tf_semantic_parity',
        'topic_parity', 'output_limit_bytes', 'output'}, 'sanitizer snapshot')
    if (manifest['schema_version'] != 1 or
            manifest['output_topics'] != ['/scan', '/odom', '/tf', '/tf_static'] or
            manifest['removed_map_to_odom_transforms'] != 7290 or
            manifest['source']['mcap_sha256'] != REAL_BAG_SHA256):
        raise ValueError('sanitizer snapshot canonical identity drift')
    parity = manifest['tf_semantic_parity']
    _exact_keys(parity, {'dynamic_non_map', 'static'}, 'TF parity')
    for record in parity.values():
        _exact_keys(record, {
            'message_count', 'source_transform_count',
            'output_transform_count', 'source_ordered_digest',
            'output_ordered_digest'}, 'TF parity record')
        if (record['source_transform_count'] != record['output_transform_count'] or
                record['source_ordered_digest'] != record['output_ordered_digest']):
            raise ValueError('sanitizer TF parity mismatch')
    if set(manifest['topic_parity']) != {'/scan', '/odom', '/tf', '/tf_static'}:
        raise ValueError('sanitizer topic parity set drift')
    for topic, record in manifest['topic_parity'].items():
        expected = {'message_count', 'storage_stamp_sha256'}
        if topic in ('/scan', '/odom'):
            expected |= {'header_stamp_sha256', 'payload_sha256'}
        _exact_keys(record, expected, f'sanitizer parity {topic}')
        if type(record['message_count']) is not int or record['message_count'] <= 0:
            raise ValueError('sanitizer parity count invalid')


def _tree_records(root: Path) -> list[dict]:
    return sorted([
        _relative_identity(path, root) for path in root.rglob('*')
        if path.is_file() and not path.is_symlink() and
        path.name != 'preflight_manifest.json'],
        key=lambda record: record['relative_path'])


def _tree_digest(records: list[dict]) -> str:
    return hashlib.sha256(canonical_json_bytes(records)).hexdigest()


def _safe_remove_created_root(root: Path) -> None:
    if root.is_symlink() or not root.is_dir() or root != root.resolve():
        raise RuntimeError('refusing unsafe artifact cleanup')
    for path in sorted(root.rglob('*'), reverse=True):
        if path.is_symlink() or path.is_file():
            path.unlink()
        elif path.is_dir():
            path.rmdir()
        else:
            raise RuntimeError('special file in created artifact')
    root.rmdir()


def _validate_artifact(root: Path, expected_mode: str) -> dict:
    if (not root.is_absolute() or root.is_symlink() or not root.is_dir() or
            root != root.resolve()):
        raise ValueError('artifact root must be canonical')
    if any(path.is_symlink() for path in root.rglob('*')):
        raise ValueError('artifact contains symlink')
    expected_root_entries = {
        'contract_snapshot.json', 'sanitizer_manifest_snapshot.json',
        'upstream_lock_snapshot.json', 'build_attestation.json',
        'replay_view_manifest.json', 'preflight_manifest.json',
        *{f'run_{index}' for index in range(1, 4)}}
    actual_root_entries = {path.name for path in root.iterdir()}
    allowed_root_entries = {
        'contract_snapshot.json', 'sanitizer_manifest_snapshot.json',
        'upstream_lock_snapshot.json', 'build_attestation.json',
        'replay_view_manifest.json', 'preflight_manifest.json'}
    if (not actual_root_entries.issubset(expected_root_entries) or
            not allowed_root_entries.issubset(actual_root_entries)):
        raise ValueError('artifact root inventory drift')
    manifest = strict_json_load(root / 'preflight_manifest.json')
    if set(manifest) != {
            'schema_version', 'mode', 'claim_scope', 'source_contract',
            'sanitizer_snapshot', 'upstream_lock_snapshot',
            'build_attestation', 'replay_view', 'runtime', 'runs',
            'comparison', 'tree_bytes', 'tree_records', 'tree_sha256'}:
        raise ValueError('preflight manifest schema drift')
    if manifest['schema_version'] != 2 or manifest['mode'] != expected_mode:
        raise ValueError('preflight manifest version drift')
    expected_claim = CLAIM_FULL if expected_mode == 'full' else CLAIM_SMOKE
    if manifest['claim_scope'] != expected_claim:
        raise ValueError('preflight claim enum drift')
    source_contract = manifest['source_contract']
    contract_snapshot = root / source_contract['relative_path']
    if (source_contract != _relative_identity(contract_snapshot, root) or
            contract_snapshot.name != 'contract_snapshot.json'):
        raise ValueError('source contract snapshot identity drift')
    contract = strict_json_load(contract_snapshot)
    _validate_source_records(contract['production_inputs'])
    _validate_source_records(contract['harness_sources'])
    for key, filename in (
            ('sanitizer_snapshot', 'sanitizer_manifest_snapshot.json'),
            ('upstream_lock_snapshot', 'upstream_lock_snapshot.json'),
            ('build_attestation', 'build_attestation.json'),
            ('replay_view', 'replay_view_manifest.json')):
        snapshot_path = root / filename
        if manifest[key] != _relative_identity(snapshot_path, root):
            raise ValueError(f'{key} identity drift')
    sanitizer_snapshot = strict_json_load(
        root / 'sanitizer_manifest_snapshot.json')
    _validate_sanitizer_snapshot(sanitizer_snapshot)
    runtime = manifest['runtime']
    replay_view = strict_json_load(root / 'replay_view_manifest.json')
    _exact_keys(replay_view, {
        'schema_version', 'source_sanitizer_manifest_sha256', 'bootstrap',
        'skipped_scan_count', 'prelude_topics', 'main_topics', 'prelude',
        'main', 'prelude_last_storage_ns', 'main_first_storage_ns',
        'main_last_scan_storage_ns', 'storage_gap_ns',
        'storage_overlap_count'}, 'replay view')
    if (replay_view['schema_version'] != 1 or
            replay_view['source_sanitizer_manifest_sha256'] !=
            manifest['sanitizer_snapshot']['sha256'] or
            replay_view['skipped_scan_count'] != 2 or
            replay_view['storage_overlap_count'] != 0 or
            replay_view['storage_gap_ns'] <= 0):
        raise ValueError('replay view boundary drift')
    build_attestation = strict_json_load(root / 'build_attestation.json')
    _exact_keys(build_attestation, {
        'schema_version', 'upstream_lock_sha256', 'build_root',
        'source_root', 'cmake_cache', 'build_returncode_file',
        'install_files', 'needed_sonames', 'loaded_runtime'},
        'build attestation')
    if (build_attestation['schema_version'] != 1 or
            build_attestation['upstream_lock_sha256'] !=
            manifest['upstream_lock_snapshot']['sha256'] or
            not {'libamcl_core.so', 'libpf_lib.so'}.issubset(
                {name for values in build_attestation['needed_sonames'].values()
                 for name in values}) or
            build_attestation['source_root'] !=
            str(Path(contract['source_overlay']['root']) / 'nav2_amcl') or
            build_attestation['install_files']['amcl'] !=
            build_attestation['loaded_runtime']['amcl_executable'] or
            build_attestation['install_files']['libamcl_core.so'] !=
            build_attestation['loaded_runtime']['libamcl_core'] or
            build_attestation['install_files']['libpf_lib.so'] !=
            build_attestation['loaded_runtime']['libpf_lib']):
        raise ValueError('build attestation drift')
    build_root = Path(build_attestation['build_root'])
    expected_install = build_root / 'install/nav2_amcl/lib'
    if (Path(build_attestation['loaded_runtime']['amcl_executable']['path']) !=
            expected_install / 'nav2_amcl/amcl' or
            Path(build_attestation['loaded_runtime']['libamcl_core']['path']) !=
            expected_install / 'libamcl_core.so' or
            Path(build_attestation['loaded_runtime']['libpf_lib']['path']) !=
            expected_install / 'libpf_lib.so' or
            runtime['amcl_executable'] !=
            build_attestation['loaded_runtime']['amcl_executable'] or
            runtime['rmw_library'] !=
            build_attestation['loaded_runtime']['rmw_library']):
        raise ValueError('runtime is not the attested overlay build')
    if (manifest['upstream_lock_snapshot']['sha256'] !=
            contract['source_overlay']['lock']['sha256']):
        raise ValueError('source snapshot binding drift')
    if (set(runtime) != {
            'rmw_implementation', 'rmw_library', 'ros_localhost_only',
            'domain_ids', 'amcl_executable'} or
            runtime['rmw_implementation'] != 'rmw_cyclonedds_cpp' or
            runtime['ros_localhost_only'] != '1'):
        raise ValueError('runtime identity schema drift')
    for key in ('rmw_library', 'amcl_executable'):
        record = runtime[key]
        if _identity(Path(record['path'])) != record:
            raise ValueError(f'runtime file identity drift: {key}')
    if (any(type(value) is not int or not 0 <= value <= 232
            for value in runtime['domain_ids']) or
            len(set(runtime['domain_ids'])) != len(runtime['domain_ids'])):
        raise ValueError('runtime domain plan drift')
    records = _tree_records(root)
    if (manifest['tree_records'] != records or
            manifest['tree_sha256'] != _tree_digest(records) or
            manifest['tree_bytes'] != _payload_tree_bytes(root) or
            manifest['tree_bytes'] > MAX_ARTIFACT_BYTES):
        raise ValueError('preflight artifact exceeds 32 MiB')
    runs = []
    for record in manifest['runs']:
        path = root / record['relative_path']
        if path.is_symlink() or path.parent.parent != root:
            raise ValueError('run evidence path drift')
        if (_identity(path)['sha256'] != record['sha256'] or
                path.stat().st_size != record['size_bytes']):
            raise ValueError('run evidence identity drift')
        evidence = strict_json_load(path)
        _exact_keys(evidence, {
            'schema_version', 'run_id', 'profile', 'seed', 'domain_id',
            'status', 'failure', 'events', 'operations', 'observer_source',
            'observer', 'amcl_executable', 'map_yaml', 'params_file',
            'sanitized_manifest', 'resource', 'teardown', 'loaded_runtime',
            'prelude_observer', 'tf_bootstrap', 'amcl_tf_error_counts',
            'transform_lookup_drop_count', 'prefix_s', 'playback_rate',
            'survivor_count', 'output_mcap_count', 'cmd_vel_publisher',
            'scan_parity'}, 'run evidence')
        if (evidence['schema_version'] != 1 or
                evidence['profile'] != 'P0' or
                evidence['cmd_vel_publisher'] != 'NOT_APPLICABLE' or
                type(evidence['seed']) is not int or
                type(evidence['domain_id']) is not int or
                not 0 <= evidence['domain_id'] <= 232):
            raise ValueError('run scalar contract drift')
        expected_run_files = {
            'amcl.log', 'amcl_resource.jsonl', 'evidence.json',
            'initialpose.request', 'map_server.log', 'observer.log',
            'observer_state.json', 'player.log', 'resource_sampler.log'}
        expected_run_files.add('tf_prelude_player.log')
        if {item.name for item in path.parent.iterdir()} != expected_run_files:
            raise ValueError('run file inventory drift')
        for process in evidence['teardown']:
            log_path = path.parent / process['log']['relative_path']
            if process['log'] != _relative_identity(log_path, path.parent):
                raise ValueError('process log identity drift')
        teardown_by_name = {item['name']: item for item in evidence['teardown']}
        if len(teardown_by_name) != len(evidence['teardown']):
            raise ValueError('duplicate teardown process')
        expected_returncodes = {
            'player': 0, 'tf_prelude_player': 0, 'resource_sampler': -2,
            'observer': 0, 'amcl': 0, 'map_server': 0}
        if (set(teardown_by_name) != set(expected_returncodes) or
                any(teardown_by_name[name]['returncode'] != code or
                    teardown_by_name[name]['survivors']
                    for name, code in expected_returncodes.items())):
            raise ValueError('process termination contract drift')
        sampler_stages = teardown_by_name['resource_sampler']['stop_stages']
        if ([stage['signal'] for stage in sampler_stages] != ['SIGINT']):
            raise ValueError('resource sampler termination drift')
        resource_path = path.parent / evidence['resource']['file']['relative_path']
        if evidence['resource']['file'] != _relative_identity(
                resource_path, path.parent):
            raise ValueError('resource identity drift')
        if (evidence['output_mcap_count'] != 0 or
                evidence['survivor_count'] != 0 or
                evidence['transform_lookup_drop_count'] != 0):
            raise ValueError('run output or survivor contract failed')
        if evidence['status'] != 'PASS':
            raise ValueError('individual preflight run failed')
        sanitized_output_root = Path(evidence['sanitized_manifest']['path']).parent
        if (evidence['sanitized_manifest']['sha256'] !=
                manifest['sanitizer_snapshot']['sha256'] or
                evidence['sanitized_manifest']['size_bytes'] !=
                manifest['sanitizer_snapshot']['size_bytes']):
            raise ValueError('run sanitizer snapshot identity drift')
        if sanitized_output_root.exists() and evidence['scan_parity'] != _scan_parity(
                evidence['observer'], sanitized_output_root):
            raise ValueError('run scan parity recomputation drift')
        _validate_replay_bootstrap(
            evidence, path.parent, sanitized_output_root,
            replay_view['bootstrap'])
        if evidence['loaded_runtime'] != build_attestation['loaded_runtime']:
            raise ValueError('loaded runtime differs from build attestation')
        observer = evidence['observer']
        _exact_keys(observer, {
            'schema_version', 'run_id', 'seed', 'max_clouds',
            'publisher_matched_count', 'events', 'readiness',
            'initialpose_count', 'pre_initial_scan_count',
            'pre_initial_cloud_count', 'raw_cloud_received_count',
            'amcl_pose_received_count', 'pending_cloud_count',
            'pending_pose_count', 'scan_count', 'scan_header_stamps_ns',
            'scan_header_stamp_sha256', 'scan_payload_sha256', 'clouds',
            'failure', 'done', 'motion_command_applicability'},
            'observer state')
        if (observer['initialpose_count'] != 1 or
                observer['pre_initial_scan_count'] != 0 or
                observer['pre_initial_cloud_count'] != 0 or
                observer['pending_cloud_count'] != 0 or
                observer['pending_pose_count'] != 0 or
                len(observer['clouds']) != observer['max_clouds']):
            raise ValueError('observer causal cardinality drift')
        if (observer['schema_version'] != 1 or observer['failure'] is not None or
                observer['done'] is not True or
                observer['publisher_matched_count'] < 1 or
                observer['motion_command_applicability'] != 'NOT_APPLICABLE'):
            raise ValueError('observer completion contract drift')
        triggers = [cloud['triggering_scan_header_stamp_ns']
                    for cloud in observer['clouds']]
        if (len(triggers) != len(set(triggers)) or
                any(value not in observer['scan_header_stamps_ns']
                    for value in triggers)):
            raise ValueError('cloud triggering scan membership drift')
        if any(cloud['pose_header_stamp_ns'] !=
               cloud['triggering_scan_header_stamp_ns'] or
               cloud['pose_frame_id'] != 'map' or cloud['frame_id'] != 'map' or
               type(cloud['particle_count']) is not int or
               cloud['particle_count'] <= 0 for cloud in observer['clouds']):
            raise ValueError('particle/pose FIFO evidence drift')
        if len(evidence['operations']) != 6 or any(
                operation['returncode'] != 0 for operation in
                evidence['operations']):
            raise ValueError('lifecycle/readback operation drift')
        runs.append(evidence)
    if manifest['comparison'] != _comparison(runs):
        raise ValueError('preflight comparison drift')
    if expected_mode == 'full':
        if (len(runs) != 3 or tuple(run['seed'] for run in runs) != FULL_SEEDS or
                [run['run_id'] for run in runs] != [
                    'p0__seed_11__attempt_1', 'p0__seed_11__attempt_2',
                    'p0__seed_23__attempt_3'] or
                any(len(run['observer']['clouds']) != FULL_CLOUD_COUNT or
                    run['observer']['scan_count'] != FULL_SCAN_COUNT
                    for run in runs) or
                not all(manifest['comparison'].values())):
            raise ValueError('full determinism contract failed')
        if (runtime['domain_ids'] != [
                runs[0]['domain_id'] + index for index in range(3)] or
                [run['domain_id'] for run in runs] != runtime['domain_ids']):
            raise ValueError('full domain mapping drift')
    elif len(runs) != 1:
        raise ValueError('smoke artifact must contain exactly one run')
    return manifest


def validate_smoke_artifact(root: Path) -> dict:
    """Validate exactly one no-motion pipeline smoke run."""
    return _validate_artifact(root, 'smoke')


def validate_full_artifact(root: Path) -> dict:
    """Validate the exact seed-11/11/23 determinism preflight."""
    return _validate_artifact(root, 'full')


def run_preflight(args) -> dict:
    """Run one smoke or the exact same-seed/different-seed preflight."""
    output_root = _canonical_output_root(args.output_root)
    sanitized = validate_sanitized_bag(args.sanitized_root)
    if sanitized['source']['mcap_sha256'] != REAL_BAG_SHA256:
        raise ValueError('sanitized source is not canonical Axis A')
    contract = strict_json_load(args.prepared_root / 'contract.json')
    if (contract['axis_a']['map']['yaml']['sha256'] != REAL_MAP_YAML_SHA256 or
            contract['axis_a']['map']['pgm']['sha256'] != REAL_MAP_PGM_SHA256):
        raise ValueError('Axis A map identity drift')
    if args.map_yaml != Path(contract['axis_a']['map']['yaml']['path']):
        raise ValueError('map path differs from prepared contract')
    _validate_source_records(contract['production_inputs'])
    _validate_source_records(contract['harness_sources'])
    budget = validate_storage_budget(
        current_free_bytes(output_root.parent), MAX_ARTIFACT_BYTES,
        _tree_bytes(args.sanitized_root))
    if not budget['pass']:
        raise RuntimeError(f'storage gate failed: {budget["failures"]}')
    env = dict(os.environ)
    for path in (args.prepared_root, args.sanitized_root,
                 args.amcl_executable, args.params_file, args.map_yaml,
                 args.rmw_library):
        if not path.is_absolute() or path != path.resolve():
            raise ValueError(f'input path must be absolute and canonical: {path}')
    bootstrap = _tf_bootstrap_plan(args.sanitized_root)
    seeds = args.seeds
    if tuple(seeds) not in ((11,), (11, 11, 23)):
        raise ValueError('preflight seed plan must be [11] or [11,11,23]')
    with tempfile.TemporaryDirectory(
            prefix='.g002-preflight-', dir=output_root.parent) as temp_name:
        stage = Path(temp_name) / 'artifact'
        stage.mkdir()
        shutil.copyfile(
            args.prepared_root / 'contract.json', stage / 'contract_snapshot.json')
        shutil.copyfile(
            args.sanitized_root / 'sanitizer_manifest.json',
            stage / 'sanitizer_manifest_snapshot.json')
        shutil.copyfile(
            Path(contract['source_overlay']['lock']['path']),
            stage / 'upstream_lock_snapshot.json')
        runs = []
        for index, seed in enumerate(seeds):
            args.attempt_index = index + 1
            run = _run_one(stage / f'run_{index + 1}', seed,
                           args.domain_base + index, args, env, bootstrap)
            runs.append(run)
            if run['status'] != 'PASS':
                break
        failed = [run['failure'] for run in runs if run['status'] != 'PASS']
        if failed:
            raise RuntimeError(f'preflight run failed: {failed}')
        if not runs or any(run['loaded_runtime'] is None for run in runs):
            raise RuntimeError('runtime load attestation missing')
        if any(run['loaded_runtime'] != runs[0]['loaded_runtime']
               for run in runs[1:]):
            raise RuntimeError('loaded runtime differs between runs')
        build_attestation = {
            'upstream_lock_sha256': sha256_file(
                stage / 'upstream_lock_snapshot.json'),
            **_build_provenance(
                args.amcl_executable,
                Path(contract['source_overlay']['root']),
                runs[0]['loaded_runtime']),
        }
        (stage / 'build_attestation.json').write_bytes(
            canonical_json_bytes(build_attestation))
        scan_views = [run['observer']['scan_header_stamps_ns'] for run in runs]
        if any(view != scan_views[0] for view in scan_views[1:]):
            raise RuntimeError('run source scan views differ')
        replay_view = _replay_view_manifest(
            args.sanitized_root, bootstrap, scan_views[0][-1])
        (stage / 'replay_view_manifest.json').write_bytes(
            canonical_json_bytes(replay_view))
        records = []
        for run_dir in sorted(stage.glob('run_*')):
            path = run_dir / 'evidence.json'
            records.append({'relative_path': str(path.relative_to(stage)),
                            'size_bytes': path.stat().st_size,
                            'sha256': sha256_file(path)})
        mode = 'full' if tuple(seeds) == FULL_SEEDS else 'smoke'
        manifest = {
            'schema_version': 2, 'mode': mode,
            'claim_scope': CLAIM_FULL if mode == 'full' else CLAIM_SMOKE,
            'source_contract': _relative_identity(
                stage / 'contract_snapshot.json', stage),
            'sanitizer_snapshot': _relative_identity(
                stage / 'sanitizer_manifest_snapshot.json', stage),
            'upstream_lock_snapshot': _relative_identity(
                stage / 'upstream_lock_snapshot.json', stage),
            'build_attestation': _relative_identity(
                stage / 'build_attestation.json', stage),
            'replay_view': _relative_identity(
                stage / 'replay_view_manifest.json', stage),
            'runtime': {
                'rmw_implementation': 'rmw_cyclonedds_cpp',
                'rmw_library': _identity(args.rmw_library),
                'ros_localhost_only': '1',
                'domain_ids': [args.domain_base + index
                               for index in range(len(seeds))],
                'amcl_executable': _identity(args.amcl_executable),
            },
            'runs': records, 'comparison': _comparison(runs),
            'tree_bytes': _payload_tree_bytes(stage),
            'tree_records': _tree_records(stage),
            'tree_sha256': _tree_digest(_tree_records(stage)),
        }
        (stage / 'preflight_manifest.json').write_bytes(
            canonical_json_bytes(manifest))
        validator = validate_full_artifact if mode == 'full' else validate_smoke_artifact
        validator(stage)
        _validate_source_records(contract['production_inputs'])
        _validate_source_records(contract['harness_sources'])
        stage.rename(output_root)
    _validate_source_records(contract['production_inputs'])
    _validate_source_records(contract['harness_sources'])
    try:
        return validator(output_root)
    except Exception:
        _safe_remove_created_root(output_root)
        raise


def main() -> int:
    """Parse the exact no-motion preflight runner arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--prepared-root', required=True, type=Path)
    parser.add_argument('--sanitized-root', required=True, type=Path)
    parser.add_argument('--amcl-executable', required=True, type=Path)
    parser.add_argument('--params-file', required=True, type=Path)
    parser.add_argument('--map-yaml', required=True, type=Path)
    parser.add_argument('--rmw-library', required=True, type=Path)
    parser.add_argument('--domain-base', type=int, default=190)
    parser.add_argument('--seeds', type=int, nargs='+', default=[11])
    parser.add_argument('--max-clouds', type=int, default=30)
    parser.add_argument('--prefix-s', type=float, default=90.0)
    parser.add_argument('--playback-rate', type=float, default=2.0)
    args = parser.parse_args()
    run_preflight(args)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
