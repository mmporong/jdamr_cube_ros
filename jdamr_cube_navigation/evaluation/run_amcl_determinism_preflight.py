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

from amcl_fault_contract import AMCL_BASELINE_FILES
from amcl_fault_contract import canonical_json_bytes
from amcl_fault_contract import current_free_bytes
from amcl_fault_contract import exact_regular_file
from amcl_fault_contract import G002_ARTIFACT_LIMIT_BYTES
from amcl_fault_contract import G002_CLOCK_QUIET_NS
from amcl_fault_contract import G002_RUN_OUTPUT_LIMIT_BYTES
from amcl_fault_contract import OVERLAY_CHANGED_FILES
from amcl_fault_contract import PF_C_PATH
from amcl_fault_contract import REAL_BAG_SHA256
from amcl_fault_contract import REAL_BAG_TOPIC_INVENTORY
from amcl_fault_contract import REAL_MAP_PGM_SHA256
from amcl_fault_contract import REAL_MAP_YAML_SHA256
from amcl_fault_contract import sha256_file
from amcl_fault_contract import STORAGE_LIMITS
from amcl_fault_contract import strict_json_load
from amcl_fault_contract import strict_json_loads
from amcl_fault_contract import UPSTREAM_AMCL_TREE_FILE_COUNT
from amcl_fault_contract import UPSTREAM_AMCL_TREE_SHA256
from amcl_fault_contract import UPSTREAM_ARCHIVE_SHA256
from amcl_fault_contract import UPSTREAM_ARCHIVE_SIZE_BYTES
from amcl_fault_contract import UPSTREAM_ARCHIVE_URL
from amcl_fault_contract import UPSTREAM_COMMIT
from amcl_fault_contract import UPSTREAM_LICENSE_SHA256
from amcl_fault_contract import validate_storage_budget
from g002_tf_sanitizer import validate_sanitized_bag
import yaml


ROOT = Path(__file__).resolve().parents[2]
NAV = ROOT / 'jdamr_cube_navigation'
OBSERVER = Path(__file__).resolve().parent / 'amcl_particle_observer.py'
SAMPLER = Path(__file__).resolve().parent / 'sample_process_group_resources.py'
ROS2 = Path('/opt/ros/jazzy/bin/ros2')
MAP_SERVER = Path('/opt/ros/jazzy/lib/nav2_map_server/map_server')
ALLOWED_SEEDS = (11, 23)
MAX_LOG_BYTES = 2 * 1024 * 1024
AMCL_TF_ERROR_MARKERS = (
    'Message Filter dropping message',
    'Failed to transform',
    'Lookup would require extrapolation',
)
CLAIM_SMOKE = (
    'ORDERED_OBSERVED_PARTICLE_PAYLOAD_SEQUENCE_SMOKE_NO_MOTION')
CLAIM_FULL = (
    'ORDERED_OBSERVED_PARTICLE_PAYLOAD_SEQUENCE_DETERMINISM_NO_MOTION')
FULL_SEEDS = (11, 11, 23)
FULL_CLOUD_COUNT = 30
FULL_SCAN_COUNT = 1937
FULL_PREFIX_S = 220.0
SMOKE_SEEDS = (11,)
SMOKE_CLOUD_COUNT = 1
SMOKE_PREFIX_S = 30.0
PLAYBACK_RATE = 2.0
PAIRING_CONTRACT = 'FIFO_PAIRED_NO_COMMON_UPDATE_ID'
CLAIM_BOUNDARY = (
    'PUBLISHER_BEST_EFFORT_DROP_FREE_AND_PER_UPDATE_CAUSALITY_NOT_PROVEN')
OBSERVATION_QOS_CONTRACT = {
    'particle_cloud': 'BEST_EFFORT_VOLATILE_SENSOR_DATA_QOS_COMPATIBLE',
    'amcl_pose': 'RELIABLE_TRANSIENT_LOCAL_KEEP_LAST_1',
}
PAIRING_WINDOW_NS = 1_000_000_000
P0_PARAMS = {
    'min_particles': 500,
    'max_particles': 2000,
    'pf_err': 0.05,
    'pf_z': 0.99,
    'recovery_alpha_fast': 0.0,
    'recovery_alpha_slow': 0.0,
}


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
        _identity_schema(record, 'prepared source identity')
        actual = _identity(Path(record['path']))
        if (actual['size_bytes'] != record['size_bytes'] or
                actual['sha256'] != record['sha256']):
            raise ValueError('prepared source or production identity drift')


def _validate_p0_params(contract: dict, params_path: Path) -> dict:
    production_record = contract['production_inputs']['production_params']
    _identity_schema(production_record, 'production P0 params')
    actual = _identity(params_path)
    if actual != production_record or contract['profiles']['P0'] != P0_PARAMS:
        raise ValueError('P0 prepared contract identity drift')
    document = yaml.safe_load(params_path.read_text(encoding='utf-8'))
    parameters = document['amcl']['ros__parameters']
    selected = {key: parameters[key] for key in P0_PARAMS}
    if selected != P0_PARAMS:
        raise ValueError('P0 AMCL parameter values drift')
    return actual


def _validate_run_artifact_policy(contract: dict) -> None:
    policy = contract['run_artifact_policy']
    _exact_keys(policy, {
        'output_mcap_count', 'per_run_limit_bytes', 'total_limit_bytes',
        'repeated_runs'}, 'G002 run artifact policy')
    if (policy['output_mcap_count'] != 0 or
            policy['per_run_limit_bytes'] != G002_RUN_OUTPUT_LIMIT_BYTES or
            policy['total_limit_bytes'] != G002_ARTIFACT_LIMIT_BYTES or
            policy['repeated_runs'] != 'metrics-only'):
        raise ValueError('G002 run artifact policy drift')


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


def _canonical_json_load(path: Path) -> dict:
    value = strict_json_load(path)
    if path.read_bytes() != canonical_json_bytes(value):
        raise ValueError(f'non-canonical JSON encoding: {path.name}')
    return value


def _sha256(value, label: str) -> str:
    if (type(value) is not str or len(value) != 64 or
            any(char not in '0123456789abcdef' for char in value)):
        raise ValueError(f'{label} SHA-256 drift')
    return value


def _identity_schema(record: dict, label: str,
                     relative: bool = False) -> None:
    path_key = 'relative_path' if relative else 'path'
    _exact_keys(record, {path_key, 'size_bytes', 'sha256'}, label)
    path = record[path_key]
    if type(path) is not str or not path:
        raise ValueError(f'{label} path drift')
    parsed = Path(path)
    if relative:
        if parsed.is_absolute() or '..' in parsed.parts or parsed.as_posix() != path:
            raise ValueError(f'{label} relative path drift')
    elif (not parsed.is_absolute() or parsed != parsed.expanduser() or
          parsed != parsed.resolve()):
        raise ValueError(f'{label} absolute path drift')
    if type(record['size_bytes']) is not int or record['size_bytes'] < 0:
        raise ValueError(f'{label} size drift')
    _sha256(record['sha256'], label)


def _tree_manifest(root: Path) -> dict:
    records = sorted([
        _relative_identity(path, root) for path in root.rglob('*')
        if path.is_file() and not path.is_symlink()],
        key=lambda record: record['relative_path'])
    if any(path.is_symlink() or not (path.is_file() or path.is_dir())
           for path in root.rglob('*')):
        raise ValueError('tree contains symlink or special entry')
    return {'root': str(root), 'file_count': len(records), 'files': records,
            'tree_sha256': _tree_digest(records)}


def _validate_tree_manifest(tree: dict, expected_names: set[str] | None,
                            label: str) -> None:
    _exact_keys(tree, {'root', 'file_count', 'files', 'tree_sha256'}, label)
    root = Path(tree['root'])
    if (type(tree['root']) is not str or not root.is_absolute() or
            root != root.expanduser() or root != root.resolve()):
        raise ValueError(f'{label} root drift')
    records = tree['files']
    if type(records) is not list or type(tree['file_count']) is not int:
        raise ValueError(f'{label} cardinality drift')
    for record in records:
        _identity_schema(record, f'{label} file', relative=True)
    paths = [record['relative_path'] for record in records]
    if (tree['file_count'] != len(records) or paths != sorted(set(paths)) or
            (expected_names is not None and set(paths) != expected_names) or
            tree['tree_sha256'] != _tree_digest(records)):
        raise ValueError(f'{label} inventory or digest drift')
    if root.exists() and tree != _tree_manifest(root):
        raise ValueError(f'{label} live tree drift')


def _validate_source_tree(tree: dict, label: str) -> None:
    _exact_keys(tree, {'file_count', 'files', 'tree_sha256'}, label)
    records = tree['files']
    if type(records) is not list or type(tree['file_count']) is not int:
        raise ValueError(f'{label} cardinality drift')
    for record in records:
        _identity_schema(record, f'{label} file', relative=True)
    paths = [record['relative_path'] for record in records]
    if (tree['file_count'] != len(records) or paths != sorted(set(paths)) or
            not paths or tree['tree_sha256'] != _tree_digest(records)):
        raise ValueError(f'{label} inventory or digest drift')


def _source_tree(root: Path) -> dict:
    tree = _tree_manifest(root)
    return {key: tree[key] for key in ('file_count', 'files', 'tree_sha256')}


def _validate_overlay_snapshot(contract: dict, lock: dict) -> dict:
    overlay = contract['source_overlay']
    _exact_keys(overlay, {'root', 'lock', 'changed_files'}, 'source overlay')
    _identity_schema(overlay['lock'], 'source overlay lock')
    _exact_keys(lock, {
        'schema_version', 'scope', 'upstream', 'baseline_tree', 'overlay_tree',
        'changed_files', 'unchanged_pf_c', 'patch_classification'},
        'source overlay snapshot')
    changed_files = sorted(OVERLAY_CHANGED_FILES)
    if (lock['schema_version'] != 1 or
            lock['scope'] != 'evaluation-only; not a production install' or
            lock['patch_classification'] !=
            'upstream random_seed surface plus local pf_pdf corrective' or
            lock['changed_files'] != changed_files or
            overlay['changed_files'] != changed_files or
            overlay['lock']['sha256'] !=
            hashlib.sha256(canonical_json_bytes(lock)).hexdigest() or
            overlay['lock']['size_bytes'] != len(canonical_json_bytes(lock)) or
            type(lock['changed_files']) is not list or
            lock['changed_files'] != sorted(set(lock['changed_files'])) or
            any(type(value) is not str for value in lock['changed_files']) or
            lock['unchanged_pf_c'] is not True):
        raise ValueError('source overlay snapshot binding drift')
    upstream = lock['upstream']
    _exact_keys(upstream, {
        'commit', 'archive_url', 'archive_size_bytes', 'archive_sha256',
        'license'}, 'overlay upstream')
    if (upstream['commit'] != UPSTREAM_COMMIT or
            upstream['archive_url'] != UPSTREAM_ARCHIVE_URL or
            upstream['archive_size_bytes'] != UPSTREAM_ARCHIVE_SIZE_BYTES or
            upstream['archive_sha256'] != UPSTREAM_ARCHIVE_SHA256):
        raise ValueError('overlay upstream canonical identity drift')
    _identity_schema(upstream['license'], 'overlay upstream license', relative=True)
    if (upstream['license']['relative_path'] != 'LICENSE' or
            upstream['license']['sha256'] != UPSTREAM_LICENSE_SHA256):
        raise ValueError('overlay upstream license identity drift')
    _validate_source_tree(lock['baseline_tree'], 'baseline source tree')
    _validate_source_tree(lock['overlay_tree'], 'overlay source tree')
    if (lock['baseline_tree']['file_count'] !=
            UPSTREAM_AMCL_TREE_FILE_COUNT or
            lock['baseline_tree']['tree_sha256'] !=
            UPSTREAM_AMCL_TREE_SHA256 or
            lock['overlay_tree']['file_count'] !=
            UPSTREAM_AMCL_TREE_FILE_COUNT):
        raise ValueError('canonical AMCL source tree drift')
    before = {
        item['relative_path']: item for item in lock['baseline_tree']['files']}
    after = {
        item['relative_path']: item for item in lock['overlay_tree']['files']}
    if set(before) != set(after):
        raise ValueError('overlay source inventory drift')
    actual_changed = {
        f'nav2_amcl/{path}' for path in before if before[path] != after[path]}
    pf_relative = PF_C_PATH.removeprefix('nav2_amcl/')
    if (actual_changed != OVERLAY_CHANGED_FILES or
            after.get(pf_relative, {}).get('sha256') !=
            AMCL_BASELINE_FILES[PF_C_PATH]):
        raise ValueError('overlay changed-file or pf.c invariant drift')
    source_root = Path(overlay['root']) / 'nav2_amcl'
    if source_root.exists() and _source_tree(source_root) != lock['overlay_tree']:
        raise ValueError('live overlay source tree drift')
    return lock['overlay_tree']


def _validate_loaded_runtime(value: dict, label: str) -> None:
    _exact_keys(value, {
        'amcl_executable', 'libamcl_core', 'libpf_lib', 'rmw_library',
        'procfs_load_evidence'}, label)
    for name in ('amcl_executable', 'libamcl_core', 'libpf_lib', 'rmw_library'):
        record = value[name]
        _identity_schema(record, f'{label} {name}')
        if _identity(Path(record['path'])) != record:
            raise ValueError(f'{label} file identity drift: {name}')
    procfs = value['procfs_load_evidence']
    _exact_keys(procfs, {
        'observation_method', 'executable_target',
        'required_mapped_library_paths'}, f'{label} procfs evidence')
    _exact_keys(procfs['required_mapped_library_paths'], {
        'libamcl_core', 'libpf_lib', 'rmw_library'},
        f'{label} procfs mapped paths')
    if (procfs['observation_method'] !=
            'proc_pid_exe_and_maps_before_replay' or
            procfs['executable_target'] != value['amcl_executable']['path'] or
            any(procfs['required_mapped_library_paths'][name] !=
                value[name]['path'] for name in (
                    'libamcl_core', 'libpf_lib', 'rmw_library'))):
        raise ValueError(f'{label} procfs load evidence drift')


def _finite_number(value, label: str, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} numeric type drift')
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        raise ValueError(f'{label} finite range drift')
    return result


def _validate_event_record(event: dict, label: str,
                           ros_clock: bool) -> None:
    clock_key = 'ros_ns' if ros_clock else 'wall_ns'
    _exact_keys(event, {'name', 'steady_ns', clock_key}, label)
    if (type(event['name']) is not str or not event['name'] or
            type(event['steady_ns']) is not int or event['steady_ns'] < 0 or
            type(event[clock_key]) is not int or event[clock_key] < 0):
        raise ValueError(f'{label} scalar drift')


def _validate_operation(operation: dict) -> None:
    _exact_keys(operation, {
        'command', 'started', 'completed', 'returncode', 'output'},
        'ROS CLI operation')
    if (type(operation['command']) is not list or
            any(type(value) is not str for value in operation['command']) or
            type(operation['returncode']) is not int or
            type(operation['output']) is not str or
            len(operation['output'].encode('utf-8')) > MAX_LOG_BYTES):
        raise ValueError('ROS CLI operation scalar drift')
    _validate_event_record(operation['started'], 'operation start', False)
    _validate_event_record(operation['completed'], 'operation completion', False)
    if operation['completed']['steady_ns'] < operation['started']['steady_ns']:
        raise ValueError('ROS CLI operation time order drift')


def _operation_contract(seed: int) -> list[dict]:
    return [
        {'args': ['lifecycle', 'set', '/map_server', 'configure'],
         'output_contract': 'LIFECYCLE_SUCCESS'},
        {'args': ['lifecycle', 'set', '/map_server', 'activate'],
         'output_contract': 'LIFECYCLE_SUCCESS'},
        {'args': ['lifecycle', 'set', '/amcl', 'configure'],
         'output_contract': 'LIFECYCLE_SUCCESS'},
        {'args': ['lifecycle', 'set', '/amcl', 'activate'],
         'output_contract': 'LIFECYCLE_SUCCESS'},
        {'args': ['param', 'get', '/amcl', 'random_seed'],
         'output_contract': f'Integer value is: {seed}'},
        {'args': [
            'service', 'call', '/rosbag2_player/resume',
            'rosbag2_interfaces/srv/Resume', '{}'],
         'output_contract': 'RESUME_RESPONSE'},
    ]


_ANSI_ESCAPE_RE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
_RCL_WARNING_RE = re.compile(
    r'^\[WARN\] \[[0-9]+\.[0-9]{9}\] \[rcl\]: ('
    r'ROS_LOCALHOST_ONLY is deprecated but still honored if it is enabled\. '
    r'Use ROS_AUTOMATIC_DISCOVERY_RANGE and ROS_STATIC_PEERS instead\.|'
    r"'localhost_only' is enabled, 'automatic_discovery_range' and "
    r"'static_peers' will be ignored\.)$")


def _semantic_output_lines(output: str) -> list[str]:
    lines = [line.strip() for line in
             _ANSI_ESCAPE_RE.sub('', output).splitlines() if line.strip()]
    warning_count = 0
    while lines and _RCL_WARNING_RE.fullmatch(lines[0]):
        warning_count += 1
        lines.pop(0)
    if warning_count > 4:
        raise ValueError('ROS CLI warning prefix cardinality drift')
    return lines


def _validate_operation_output(operation: dict, output_contract: str) -> None:
    lines = _semantic_output_lines(operation['output'])
    if output_contract == 'LIFECYCLE_SUCCESS':
        expected = ['Transitioning successful']
    elif output_contract == 'RESUME_RESPONSE':
        expected = [
            'requester: making request: '
            'rosbag2_interfaces.srv.Resume_Request()',
            'response:', 'rosbag2_interfaces.srv.Resume_Response()']
    else:
        expected = [output_contract]
    if lines != expected:
        raise ValueError('ROS CLI semantic output drift')


def _validate_run_operations(operations: list[dict], seed: int) -> None:
    contract = _operation_contract(seed)
    if type(operations) is not list or len(operations) != len(contract):
        raise ValueError('run operation cardinality drift')
    for operation, expected in zip(operations, contract):
        _validate_operation(operation)
        if (operation['command'] != [str(ROS2), *expected['args']] or
                operation['returncode'] != 0):
            raise ValueError('run operation command or order drift')
        output_contract = expected['output_contract']
        _validate_operation_output(operation, output_contract)


def _validate_bootstrap_plan(plan: dict) -> None:
    _exact_keys(plan, {
        'source_start_storage_ns', 'previous_scan_storage_ns',
        'first_main_scan_storage_ns', 'first_main_scan_header_ns',
        'lower_tf_storage_ns', 'lower_tf_header_ns', 'upper_tf_storage_ns',
        'upper_tf_header_ns', 'start_offset_ns', 'start_offset_s'},
        'TF bootstrap plan')
    integer_keys = set(plan) - {'start_offset_s'}
    if any(type(plan[key]) is not int for key in integer_keys):
        raise ValueError('TF bootstrap integer drift')
    _finite_number(plan['start_offset_s'], 'TF bootstrap offset', 0.0)
    if (plan['start_offset_ns'] <= 0 or
            plan['start_offset_s'] != plan['start_offset_ns'] / 1_000_000_000 or
            not plan['previous_scan_storage_ns'] <
            plan['first_main_scan_storage_ns'] or
            not plan['lower_tf_header_ns'] <=
            plan['first_main_scan_header_ns'] <= plan['upper_tf_header_ns']):
        raise ValueError('TF bootstrap consistency drift')


def _validate_replay_summary(summary: dict, allowed_topics: list[str],
                             label: str) -> None:
    if (type(summary) is not dict or not summary or
            not set(summary).issubset(set(allowed_topics))):
        raise ValueError(f'{label} topic inventory drift')
    for topic, record in summary.items():
        _exact_keys(record, {
            'message_count', 'storage_stamp_sha256', 'raw_payload_sha256',
            'tf_semantic_sha256'}, f'{label} {topic}')
        if type(record['message_count']) is not int or record['message_count'] <= 0:
            raise ValueError(f'{label} message count drift')
        _sha256(record['storage_stamp_sha256'], f'{label} storage stamps')
        _sha256(record['raw_payload_sha256'], f'{label} raw payload')
        semantic = record['tf_semantic_sha256']
        if topic.startswith('/tf'):
            _sha256(semantic, f'{label} TF semantics')
        elif semantic is not None:
            raise ValueError(f'{label} non-TF semantic digest drift')


def _validate_observer_snapshot(observer: dict, label: str) -> None:
    _exact_keys(observer, {
        'schema_version', 'run_id', 'seed', 'max_clouds',
        'publisher_matched_count', 'pose_publisher_matched_count',
        'events', 'readiness',
        'initialpose_count', 'pre_initial_scan_count',
        'pre_initial_cloud_count', 'raw_cloud_received_count',
        'amcl_pose_received_count', 'pending_cloud_count',
        'pending_pose_count', 'scan_count', 'scan_header_stamps_ns',
        'scan_header_stamp_sha256', 'scan_payload_sha256', 'clouds',
        'clock_samples', 'callback_trace', 'pairing_contract',
        'observation_qos_contract',
        'pose_causality', 'failure', 'done',
        'motion_command_applicability'}, label)
    if (observer['schema_version'] != 1 or type(observer['run_id']) is not str or
            type(observer['seed']) is not int or
            type(observer['max_clouds']) is not int or
            observer['max_clouds'] <= 0 or type(observer['done']) is not bool or
            observer['pairing_contract'] != PAIRING_CONTRACT or
            observer['observation_qos_contract'] !=
            OBSERVATION_QOS_CONTRACT or
            observer['pose_causality'] != 'NOT_PROVEN' or
            observer['motion_command_applicability'] != 'NOT_APPLICABLE'):
        raise ValueError(f'{label} scalar drift')
    count_keys = (
        'publisher_matched_count', 'pose_publisher_matched_count',
        'initialpose_count',
        'pre_initial_scan_count', 'pre_initial_cloud_count',
        'raw_cloud_received_count', 'amcl_pose_received_count',
        'pending_cloud_count', 'pending_pose_count', 'scan_count')
    if any(type(observer[key]) is not int or observer[key] < 0
           for key in count_keys):
        raise ValueError(f'{label} count drift')
    _exact_keys(observer['readiness'], {
        'map_count', 'clock_count', 'odom_count', 'tf_count',
        'map_odom_tf_count', 'tf_static_count'}, f'{label} readiness')
    if any(type(value) is not int or value < 0
           for value in observer['readiness'].values()):
        raise ValueError(f'{label} readiness count drift')
    if type(observer['events']) is not list:
        raise ValueError(f'{label} events drift')
    for event in observer['events']:
        _validate_event_record(event, f'{label} event', True)
    if (type(observer['clock_samples']) is not list or
            len(observer['clock_samples']) != observer['readiness']['clock_count']):
        raise ValueError(f'{label} clock sample cardinality drift')
    previous_ros_ns = None
    previous_arrival_ns = None
    for sample in observer['clock_samples']:
        _exact_keys(sample, {'ros_ns', 'arrival_steady_ns'}, f'{label} clock')
        if (type(sample['ros_ns']) is not int or sample['ros_ns'] < 0 or
                type(sample['arrival_steady_ns']) is not int or
                sample['arrival_steady_ns'] < 0 or
                (previous_ros_ns is not None and
                 sample['ros_ns'] < previous_ros_ns) or
                (previous_arrival_ns is not None and
                 sample['arrival_steady_ns'] <= previous_arrival_ns)):
            raise ValueError(f'{label} clock monotonicity drift')
        previous_ros_ns = sample['ros_ns']
        previous_arrival_ns = sample['arrival_steady_ns']
    if type(observer['callback_trace']) is not list:
        raise ValueError(f'{label} callback trace drift')
    stream_counts = {'particle_cloud': 0, 'amcl_pose': 0}
    previous_arrival_ns = None
    for item in observer['callback_trace']:
        _exact_keys(item, {
            'kind', 'stream_index', 'header_stamp_ns', 'arrival_steady_ns'},
            f'{label} callback')
        if (item['kind'] not in stream_counts or
                item['stream_index'] != stream_counts[item['kind']] or
                type(item['header_stamp_ns']) is not int or
                item['header_stamp_ns'] < 0 or
                type(item['arrival_steady_ns']) is not int or
                item['arrival_steady_ns'] < 0 or
                (previous_arrival_ns is not None and
                 item['arrival_steady_ns'] < previous_arrival_ns)):
            raise ValueError(f'{label} callback order drift')
        stream_counts[item['kind']] += 1
        previous_arrival_ns = item['arrival_steady_ns']
    if (stream_counts['particle_cloud'] !=
            observer['raw_cloud_received_count'] or
            stream_counts['amcl_pose'] != observer['amcl_pose_received_count']):
        raise ValueError(f'{label} callback cardinality drift')
    pair_orders = []
    trace = observer['callback_trace']
    if len(trace) % 2:
        raise ValueError(f'{label} callback one-to-one window drift')
    for pair_index in range(0, len(trace), 2):
        pair = trace[pair_index:pair_index + 2]
        kinds = tuple(item['kind'] for item in pair)
        if (set(kinds) != {'particle_cloud', 'amcl_pose'} or
                any(item['stream_index'] != pair_index // 2 for item in pair) or
                pair[-1]['arrival_steady_ns'] - pair[0]['arrival_steady_ns'] >
                PAIRING_WINDOW_NS):
            raise ValueError(f'{label} callback one-to-one window drift')
        pair_orders.append(kinds)
    if pair_orders and len(set(pair_orders)) != 1:
        raise ValueError(f'{label} callback cross-stream reorder drift')
    stamps = observer['scan_header_stamps_ns']
    if (type(stamps) is not list or
            any(type(value) is not int or value < 0 for value in stamps) or
            len(stamps) != observer['scan_count']):
        raise ValueError(f'{label} scan stamps drift')
    _sha256(observer['scan_header_stamp_sha256'], f'{label} scan headers')
    _sha256(observer['scan_payload_sha256'], f'{label} scan payload')
    if hashlib.sha256(''.join(f'{value}\n' for value in stamps).encode()).hexdigest() != \
            observer['scan_header_stamp_sha256']:
        raise ValueError(f'{label} scan header digest drift')
    if type(observer['clouds']) is not list:
        raise ValueError(f'{label} cloud list drift')
    cloud_keys = {
        'header_stamp_ns', 'arrival_steady_ns', 'callback_ros_ns', 'frame_id',
        'particle_count', 'payload_sha256', 'index',
        'fifo_associated_pose_scan_header_stamp_ns', 'pose_header_stamp_ns',
        'pose_arrival_steady_ns', 'pose_callback_ros_ns', 'pose_frame_id',
        'pose', 'covariance', 'scan_arrival_steady_ns',
        'scan_to_pose_steady_ns', 'cloud_stream_index',
        'pose_stream_index', 'pair_arrival_delta_ns'}
    for index, cloud in enumerate(observer['clouds']):
        _exact_keys(cloud, cloud_keys, f'{label} cloud')
        if (cloud['index'] != index or type(cloud['particle_count']) is not int or
                cloud['particle_count'] <= 0 or cloud['frame_id'] != 'map' or
                cloud['pose_frame_id'] != 'map' or
                cloud['cloud_stream_index'] != index or
                cloud['pose_stream_index'] != index or
                type(cloud['pair_arrival_delta_ns']) is not int or
                not 0 <= cloud['pair_arrival_delta_ns'] <= PAIRING_WINDOW_NS):
            raise ValueError(f'{label} cloud scalar drift')
        _sha256(cloud['payload_sha256'], f'{label} particle payload')
        if type(cloud['pose']) is not list or len(cloud['pose']) != 7 or \
                type(cloud['covariance']) is not list or \
                len(cloud['covariance']) != 36:
            raise ValueError(f'{label} pose shape drift')
        for value in (*cloud['pose'], *cloud['covariance']):
            _finite_number(value, f'{label} pose value')
        for key in (
                'header_stamp_ns', 'arrival_steady_ns', 'callback_ros_ns',
                'fifo_associated_pose_scan_header_stamp_ns',
                'pose_header_stamp_ns',
                'pose_arrival_steady_ns', 'pose_callback_ros_ns',
                'scan_arrival_steady_ns', 'scan_to_pose_steady_ns'):
            if type(cloud[key]) is not int or cloud[key] < 0:
                raise ValueError(f'{label} cloud timestamp drift')


def _load_observer_state(record: dict, run_dir: Path) -> dict:
    _identity_schema(record, 'observer state reference', relative=True)
    if record['relative_path'] != 'observer_state.json':
        raise ValueError('observer state reference path drift')
    path = run_dir / record['relative_path']
    if (path.is_symlink() or path.parent != run_dir or
            record != _relative_identity(path, run_dir)):
        raise ValueError('observer state reference identity drift')
    observer = _canonical_json_load(path)
    _validate_observer_snapshot(observer, 'observer state')
    return observer


def _runtime_guard(run_dir: Path) -> None:
    if current_free_bytes(run_dir) < STORAGE_LIMITS['abort_free_floor_bytes']:
        raise RuntimeError('runtime free-space floor crossed')
    for path in run_dir.glob('*.log'):
        if path.stat().st_size > MAX_LOG_BYTES:
            raise RuntimeError(f'run log exceeds 2 MiB cap: {path.name}')
    if _tree_bytes(run_dir) > G002_RUN_OUTPUT_LIMIT_BYTES:
        raise RuntimeError('G002 run output exceeds 8 MiB cap')


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
                _wait_process(record, run_dir, wait_s, enforce_guard=False)
            except subprocess.TimeoutExpired:
                pass
    record['stream'].close()
    return {'name': record['name'], 'pid': record['pid'],
            'pgid': record['pgid'], 'command': record['command'],
            'started': record['started'],
            'returncode': process.poll(), 'stop_stages': stages,
            'survivors': _members(record['pgid']),
            'log': _relative_identity(record['log_path'], run_dir)}


def _wait_process(record: dict, run_dir: Path, timeout_s: float,
                  enforce_guard: bool = True) -> int:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        returncode = record['process'].poll()
        if returncode is not None:
            return returncode
        if enforce_guard:
            _runtime_guard(run_dir)
        time.sleep(0.02)
    raise subprocess.TimeoutExpired(record['command'], timeout_s)


def _guarded_sleep(duration_s: float, run_dir: Path) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        _runtime_guard(run_dir)
        time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))


def _atomic_replace_text(path: Path, value: str) -> None:
    with tempfile.NamedTemporaryFile(
            dir=path.parent, mode='w', encoding='utf-8', delete=False) as stream:
        temp_path = Path(stream.name)
        stream.write(value)
        stream.flush()
        os.fsync(stream.fileno())
    temp_path.replace(path)


def _run_cli(args: list[str], env: dict[str, str], run_dir: Path,
             timeout_s: float = 20.0
             ) -> dict:
    started = _event('request')
    command = [str(ROS2), *args]
    with tempfile.NamedTemporaryFile(dir=run_dir) as stream:
        process = subprocess.Popen(
            command, env=env, stdout=stream, stderr=subprocess.STDOUT,
            start_new_session=True)
        record = {'process': process, 'command': command}
        try:
            returncode = _wait_process(record, run_dir, timeout_s)
        except Exception:
            if process.poll() is None:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=2.0)
            raise
        stream.flush()
        if stream.tell() > MAX_LOG_BYTES:
            raise RuntimeError('ROS CLI output exceeds 2 MiB cap')
        stream.seek(0)
        output = stream.read().decode('utf-8', errors='strict')
    return {'command': command, 'started': started,
            'completed': _event('response'), 'returncode': returncode,
            'output': output}


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


def _request_observer_snapshot(state: Path, request: Path, ack: Path,
                               token: str, run_dir: Path,
                               timeout_s: float) -> tuple[dict, dict]:
    _atomic_replace_text(request, token + '\n')
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        _runtime_guard(run_dir)
        if ack.is_file():
            acknowledgement = _canonical_json_load(ack)
            _exact_keys(acknowledgement, {
                'request_token', 'snapshot_ref', 'snapshot_sha256',
                'clock_count', 'last_clock_arrival_steady_ns',
                'ack_steady_ns', 'observed_quiet_ns',
                'publisher_endpoint_count'},
                'observer snapshot acknowledgement')
            snapshot_path = run_dir / acknowledgement['snapshot_ref']
            snapshot = _canonical_json_load(snapshot_path)
            snapshot_sha256 = hashlib.sha256(
                canonical_json_bytes(snapshot)).hexdigest()
            if (acknowledgement['request_token'] != token or
                    acknowledgement['snapshot_ref'] !=
                    f'prelude_snapshot.{token}.json' or
                    acknowledgement['snapshot_sha256'] != snapshot_sha256 or
                    acknowledgement['clock_count'] !=
                    snapshot['readiness']['clock_count'] or
                    acknowledgement['publisher_endpoint_count'] != 0 or
                    acknowledgement['last_clock_arrival_steady_ns'] !=
                    snapshot['clock_samples'][-1]['arrival_steady_ns'] or
                    acknowledgement['observed_quiet_ns'] !=
                    acknowledgement['ack_steady_ns'] -
                    acknowledgement['last_clock_arrival_steady_ns'] or
                    acknowledgement['observed_quiet_ns'] <
                    G002_CLOCK_QUIET_NS):
                raise ValueError('observer snapshot acknowledgement drift')
            evidence = {
                'request': _relative_identity(request, run_dir),
                'ack': _relative_identity(ack, run_dir),
                'snapshot': _relative_identity(snapshot_path, run_dir),
                'snapshot_sha256': snapshot_sha256,
                'clock_count': acknowledgement['clock_count'],
                'last_clock_arrival_steady_ns':
                    acknowledgement['last_clock_arrival_steady_ns'],
                'ack_steady_ns': acknowledgement['ack_steady_ns'],
                'observed_quiet_ns': acknowledgement['observed_quiet_ns'],
                'publisher_endpoint_count': 0,
            }
            return snapshot, evidence
        time.sleep(0.02)
    raise TimeoutError('observer snapshot acknowledgement timeout')


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
            if not topic_rows:
                continue
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
        'clock_contract': {
            'source': 'rosbag2_player_synthesized_clock',
            'handoff': 'SEQUENTIAL_DUAL_PLAYER_CLOCK_HANDOFF',
            'per_run_proof': 'run_evidence_clock_handoff'},
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
    result['procfs_load_evidence'] = {
        'observation_method': 'proc_pid_exe_and_maps_before_replay',
        'executable_target': str(actual_executable),
        'required_mapped_library_paths': {
            name: str(path.resolve()) for name, path in expected.items()
            if name != 'amcl_executable'},
    }
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
                      loaded_runtime: dict, overlay_tree_sha256: str) -> dict:
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
        'overlay_tree_sha256': overlay_tree_sha256,
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
        raise ValueError('initialpose, scan, and cloud event order drift')
    if (evidence['observer']['scan_header_stamps_ns'][0] !=
            expected['first_main_scan_header_ns']):
        raise ValueError('first main scan does not match TF bootstrap plan')


def _resource_rows(path: Path) -> list[dict]:
    rows = [strict_json_loads(line)
            for line in path.read_text(encoding='utf-8').splitlines()
            if line]
    if not rows:
        raise ValueError('resource sampler produced no rows')
    previous_monotonic_s = None
    for row in rows:
        _exact_keys(row, {
            'monotonic_s', 'cpu_total_s', 'cpu_pct_one_core', 'rss_mb',
            'process_count'}, 'resource sample')
        monotonic_s = _finite_number(
            row['monotonic_s'], 'resource monotonic time', 0.0)
        _finite_number(row['cpu_total_s'], 'resource CPU total', 0.0)
        _finite_number(row['rss_mb'], 'resource RSS', 0.0)
        if (row['cpu_pct_one_core'] is not None and
                _finite_number(row['cpu_pct_one_core'], 'resource CPU', 0.0) < 0.0):
            raise ValueError('resource CPU percentage drift')
        if type(row['process_count']) is not int or row['process_count'] < 0:
            raise ValueError('resource process count drift')
        if previous_monotonic_s is not None and monotonic_s <= previous_monotonic_s:
            raise ValueError('resource sample time order drift')
        previous_monotonic_s = monotonic_s
    return rows


def _resource_summary(path: Path, run_dir: Path) -> dict:
    rows = _resource_rows(path)
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


def _clock_handoff(prelude_observer: dict, final_observer: dict,
                   prelude_snapshot: dict,
                   main_player_started: dict) -> dict:
    _validate_event_record(main_player_started, 'main player start', False)
    if (main_player_started['name'] != 'started' or
            main_player_started['steady_ns'] <=
            prelude_snapshot['ack_steady_ns']):
        raise ValueError('main player started before clock source barrier')
    publishers_excluded_by_order = (
        main_player_started['steady_ns'] >
        prelude_snapshot['ack_steady_ns'])
    prelude_count = prelude_observer['readiness']['clock_count']
    prelude_samples = prelude_observer['clock_samples']
    samples = final_observer['clock_samples']
    if (prelude_count <= 0 or len(prelude_samples) != prelude_count or
            len(samples) <= prelude_count):
        raise ValueError('dual-player clock handoff samples missing')
    if samples[:prelude_count] != prelude_samples:
        raise ValueError('dual-player clock prelude prefix mismatch')
    prelude_last = prelude_samples[-1]
    main_first = samples[prelude_count]
    if (main_first['ros_ns'] < prelude_last['ros_ns'] or
            main_first['arrival_steady_ns'] <=
            prelude_snapshot['ack_steady_ns'] or
            main_first['arrival_steady_ns'] <
            main_player_started['steady_ns']):
        raise ValueError('dual-player clock handoff is not monotonic')
    return {
        'contract': 'BOUNDED_QUIESCENCE_CLOCK_SOURCE_HANDOFF',
        'source_attribution_boundary':
            'GID_NOT_EXPOSED_BY_RCLPY_MESSAGE_INFO',
        'quiet_window_ns': G002_CLOCK_QUIET_NS,
        'prelude_clock_count': prelude_count,
        'final_clock_count': len(samples),
        'prelude_last': prelude_last,
        'barrier_ack_steady_ns': prelude_snapshot['ack_steady_ns'],
        'main_player_started': main_player_started,
        'main_first': main_first,
        'monotonic': True,
        'simultaneous_publishers_excluded_by_process_order':
            publishers_excluded_by_order,
    }


def _validate_clock_handoff(value: dict, prelude_observer: dict,
                            final_observer: dict, prelude_snapshot: dict,
                            main_player_started: dict) -> None:
    _exact_keys(value, {
        'contract', 'source_attribution_boundary', 'quiet_window_ns',
        'prelude_clock_count', 'final_clock_count', 'prelude_last',
        'barrier_ack_steady_ns', 'main_player_started', 'main_first', 'monotonic',
        'simultaneous_publishers_excluded_by_process_order'},
        'clock handoff')
    expected = _clock_handoff(
        prelude_observer, final_observer, prelude_snapshot,
        main_player_started)
    if value != expected:
        raise ValueError('clock handoff evidence drift')


def _validate_prelude_snapshot(value: dict, observer: dict,
                               run_dir: Path) -> None:
    _exact_keys(value, {
        'request', 'ack', 'snapshot', 'snapshot_sha256', 'clock_count',
        'last_clock_arrival_steady_ns', 'ack_steady_ns',
        'observed_quiet_ns', 'publisher_endpoint_count'},
        'prelude snapshot evidence')
    for key, filename in (
            ('request', 'prelude_snapshot.request'),
            ('ack', 'prelude_snapshot.ack.json'),
            ('snapshot', 'prelude_snapshot.prelude_complete.json')):
        _identity_schema(value[key], f'prelude snapshot {key}', relative=True)
        path = run_dir / value[key]['relative_path']
        if (value[key]['relative_path'] != filename or
                value[key] != _relative_identity(path, run_dir)):
            raise ValueError('prelude snapshot file identity drift')
    acknowledgement = _canonical_json_load(
        run_dir / value['ack']['relative_path'])
    _exact_keys(acknowledgement, {
        'request_token', 'snapshot_ref', 'snapshot_sha256', 'clock_count',
        'last_clock_arrival_steady_ns', 'ack_steady_ns',
        'observed_quiet_ns', 'publisher_endpoint_count'},
        'prelude snapshot acknowledgement')
    persisted_snapshot = _canonical_json_load(
        run_dir / value['snapshot']['relative_path'])
    expected_sha256 = hashlib.sha256(
        canonical_json_bytes(observer)).hexdigest()
    if ((run_dir / value['request']['relative_path']).read_text(
            encoding='utf-8') != 'prelude_complete\n' or
            value['snapshot_sha256'] != expected_sha256 or
            persisted_snapshot != observer or
            value['clock_count'] != observer['readiness']['clock_count'] or
            value['last_clock_arrival_steady_ns'] !=
            observer['clock_samples'][-1]['arrival_steady_ns'] or
            value['ack_steady_ns'] <=
            value['last_clock_arrival_steady_ns'] or
            value['observed_quiet_ns'] !=
            value['ack_steady_ns'] - value['last_clock_arrival_steady_ns'] or
            value['observed_quiet_ns'] < G002_CLOCK_QUIET_NS or
            value['publisher_endpoint_count'] != 0 or
            acknowledgement['snapshot_sha256'] != expected_sha256 or
            acknowledgement['clock_count'] != value['clock_count'] or
            acknowledgement['request_token'] != 'prelude_complete' or
            acknowledgement['snapshot_ref'] !=
            'prelude_snapshot.prelude_complete.json' or
            acknowledgement['last_clock_arrival_steady_ns'] !=
            value['last_clock_arrival_steady_ns'] or
            acknowledgement['ack_steady_ns'] != value['ack_steady_ns'] or
            acknowledgement['observed_quiet_ns'] !=
            value['observed_quiet_ns'] or
            acknowledgement['publisher_endpoint_count'] != 0):
        raise ValueError('prelude snapshot acknowledgement drift')


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
    snapshot_request = run_dir / 'prelude_snapshot.request'
    snapshot_ack = run_dir / 'prelude_snapshot.ack.json'
    operation_contract = _operation_contract(seed)
    events = [_event('run_started')]
    launched = []
    operations = []
    observer_source = _identity(OBSERVER)
    loaded_runtime = None
    prelude_observer = None
    prelude_snapshot = None
    main_player_started = None
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
            '--snapshot-request', str(snapshot_request),
            '--snapshot-ack', str(snapshot_ack),
            '--initial-x-m', str(getattr(args, 'initial_x_m', 0.0)),
            '--initial-y-m', str(getattr(args, 'initial_y_m', 0.0)),
            '--initial-yaw-rad', str(getattr(args, 'initial_yaw_rad', 0.0))],
            run_dir / 'observer.log', env))
        resource = _start('resource_sampler', [
            '/usr/bin/python3', str(SAMPLER), '--process-group',
            str(amcl['pgid']), '--output', str(run_dir / 'amcl_resource.jsonl'),
            '--interval-s', '0.5'], run_dir / 'resource_sampler.log', env)
        launched.append(resource)
        _guarded_sleep(1.0, run_dir)
        for expected in operation_contract[:2]:
            operation = _run_cli(expected['args'], env, run_dir)
            operations.append(operation)
            if operation['returncode'] != 0:
                raise RuntimeError('map lifecycle transition failed')
            _validate_operation_output(
                operation, expected['output_contract'])
        _wait_state(state, lambda value: value['readiness']['map_count'] > 0, 20.0)
        for expected in operation_contract[2:4]:
            operation = _run_cli(expected['args'], env, run_dir)
            operations.append(operation)
            if operation['returncode'] != 0:
                raise RuntimeError('AMCL lifecycle transition failed')
            _validate_operation_output(
                operation, expected['output_contract'])
        readback = _run_cli(operation_contract[4]['args'], env, run_dir)
        operations.append(readback)
        if readback['returncode'] != 0:
            raise RuntimeError('AMCL random_seed readback failed')
        _validate_operation_output(
            readback, operation_contract[4]['output_contract'])
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
        if _wait_process(prelude, run_dir, 20.0) != 0:
            raise RuntimeError('TF prelude player failed')
        events.append(_event('tf_prelude_completed'))
        prelude_observer, prelude_snapshot = _request_observer_snapshot(
            state, snapshot_request, snapshot_ack, 'prelude_complete',
            run_dir, 20.0)
        _validate_observer_snapshot(
            prelude_observer, 'runtime prelude observer state')
        if (prelude_observer['readiness']['odom_count'] <= 0 or
                prelude_observer['readiness']['tf_count'] <= 0 or
                prelude_observer['readiness']['tf_static_count'] <= 0 or
                prelude_observer['readiness']['clock_count'] <= 0):
            raise RuntimeError('prelude snapshot readiness failed')
        player = _start('player', [
            str(ROS2), 'bag', 'play', str(args.sanitized_root),
            '--storage', 'mcap', '--topics', '/scan', '/odom', '/tf',
            '/tf_static', '--clock-topics-all', '--start-paused',
            '--disable-keyboard-controls', '--start-offset',
            str(bootstrap['start_offset_s']),
            '--playback-duration', str(args.prefix_s),
            '--rate', str(args.playback_rate)],
            run_dir / 'player.log', env)
        main_player_started = player['started']
        launched.append(player)
        _wait_state(
            state, lambda value: value['publisher_matched_count'] > 0 and
            value['pose_publisher_matched_count'] > 0, 20.0)
        before_init = strict_json_load(state)
        if (before_init['scan_count'] != 0 or before_init['clouds'] or
                before_init['pre_initial_cloud_count'] != 0):
            raise RuntimeError('scan or particle observed before initialization')
        request.write_text('publish once\n', encoding='utf-8')
        _wait_state(state, lambda value: value['initialpose_count'] == 1, 10.0)
        resume = _run_cli(operation_contract[5]['args'], env, run_dir)
        operations.append(resume)
        if resume['returncode'] != 0:
            raise RuntimeError('player resume failed')
        _validate_run_operations(operations, seed)
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
    persisted_observer = (_canonical_json_load(state)
                          if state.is_file() else None)
    observer_state = (_relative_identity(state, run_dir)
                      if state.is_file() else None)
    clock_handoff = (_clock_handoff(
        prelude_observer, persisted_observer, prelude_snapshot,
        main_player_started)
                     if prelude_observer is not None and
                     persisted_observer is not None and
                     prelude_snapshot is not None and
                     main_player_started is not None
                     else None)
    _runtime_guard(run_dir)
    evidence = {
        'schema_version': 1, 'run_id': run_id, 'profile': profile,
        'seed': seed, 'domain_id': domain_id, 'status': status,
        'failure': failure, 'events': events, 'operations': operations,
        'observer_source': observer_source, 'observer_state': observer_state,
        'amcl_executable': _identity(args.amcl_executable),
        'map_yaml': _identity(args.map_yaml),
        'params_file': _identity(args.params_file),
        'sanitized_manifest': _identity(
            args.sanitized_root / 'sanitizer_manifest.json'),
        'resource': resource_summary, 'teardown': teardown,
        'loaded_runtime': loaded_runtime,
        'prelude_observer': prelude_observer,
        'prelude_snapshot': prelude_snapshot,
        'clock_handoff': clock_handoff,
        'tf_bootstrap': bootstrap,
        'amcl_tf_error_counts': amcl_tf_error_counts,
        'transform_lookup_drop_count': transform_lookup_drop_count,
        'prefix_s': args.prefix_s, 'playback_rate': args.playback_rate,
        'survivor_count': len(survivors),
        'output_mcap_count': len(list(run_dir.rglob('*.mcap'))),
        'cmd_vel_publisher': 'NOT_APPLICABLE',
    }
    if persisted_observer is not None and persisted_observer['scan_count']:
        evidence['scan_parity'] = _scan_parity(
            persisted_observer, args.sanitized_root)
    else:
        evidence['scan_parity'] = None
    (run_dir / 'evidence.json').write_bytes(canonical_json_bytes(evidence))
    if _tree_bytes(run_dir) > G002_RUN_OUTPUT_LIMIT_BYTES:
        raise RuntimeError('G002 run metrics/log output exceeds 8 MiB cap')
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
    source = manifest['source']
    _exact_keys(source, {
        'path', 'mcap_size_bytes', 'mcap_sha256', 'metadata_sha256'},
        'sanitizer source')
    if (type(source['path']) is not str or not Path(source['path']).is_absolute() or
            type(source['mcap_size_bytes']) is not int or
            source['mcap_size_bytes'] <= 0):
        raise ValueError('sanitizer source scalar drift')
    _sha256(source['mcap_sha256'], 'sanitizer source MCAP')
    _sha256(source['metadata_sha256'], 'sanitizer source metadata')
    if manifest['input_topic_inventory'] != REAL_BAG_TOPIC_INVENTORY:
        raise ValueError('sanitizer input topic inventory drift')
    if (type(manifest['output_limit_bytes']) is not int or
            manifest['output_limit_bytes'] <= 0):
        raise ValueError('sanitizer output cap drift')
    output = manifest['output']
    _exact_keys(output, {
        'mcap_name', 'mcap_size_bytes', 'mcap_sha256', 'metadata_sha256'},
        'sanitizer output')
    if (output['mcap_name'] != 'bag_0.mcap' or
            type(output['mcap_size_bytes']) is not int or
            output['mcap_size_bytes'] <= 0 or
            output['mcap_size_bytes'] > manifest['output_limit_bytes']):
        raise ValueError('sanitizer output scalar drift')
    _sha256(output['mcap_sha256'], 'sanitizer output MCAP')
    _sha256(output['metadata_sha256'], 'sanitizer output metadata')
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
        for key in ('source_ordered_digest', 'output_ordered_digest'):
            _sha256(record[key], f'sanitizer TF parity {key}')
        for key in ('message_count', 'source_transform_count',
                    'output_transform_count'):
            if type(record[key]) is not int or record[key] < 0:
                raise ValueError('sanitizer TF parity count drift')
    if (parity['dynamic_non_map']['message_count'] !=
            REAL_BAG_TOPIC_INVENTORY['/tf'] or
            parity['static']['message_count'] !=
            REAL_BAG_TOPIC_INVENTORY['/tf_static']):
        raise ValueError('sanitizer TF message cardinality drift')
    if set(manifest['topic_parity']) != {'/scan', '/odom', '/tf', '/tf_static'}:
        raise ValueError('sanitizer topic parity set drift')
    for topic, record in manifest['topic_parity'].items():
        expected = {'message_count', 'storage_stamp_sha256'}
        if topic in ('/scan', '/odom'):
            expected |= {'header_stamp_sha256', 'payload_sha256'}
        _exact_keys(record, expected, f'sanitizer parity {topic}')
        if (type(record['message_count']) is not int or
                record['message_count'] != REAL_BAG_TOPIC_INVENTORY[topic]):
            raise ValueError('sanitizer parity count invalid')
        for key, value in record.items():
            if key != 'message_count':
                _sha256(value, f'sanitizer parity {topic} {key}')


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
    actual_root_entries = {path.name for path in root.iterdir()}
    expected_root_entries = {
        'contract_snapshot.json', 'sanitizer_manifest_snapshot.json',
        'upstream_lock_snapshot.json', 'build_attestation.json',
        'replay_view_manifest.json', 'preflight_manifest.json',
        *{f'run_{index}' for index in range(
            1, 4 if expected_mode == 'full' else 2)}}
    if actual_root_entries != expected_root_entries:
        raise ValueError('artifact root inventory drift')
    manifest = _canonical_json_load(root / 'preflight_manifest.json')
    if set(manifest) != {
            'schema_version', 'mode', 'claim_scope', 'claim_boundary',
            'pairing_contract',
            'observation_qos_contract',
            'pose_causality', 'source_contract',
            'sanitized_input', 'sanitizer_snapshot', 'upstream_lock_snapshot',
            'build_attestation', 'replay_view', 'runtime', 'runs',
            'comparison', 'tree_bytes', 'tree_records', 'tree_sha256'}:
        raise ValueError('preflight manifest schema drift')
    if manifest['schema_version'] != 3 or manifest['mode'] != expected_mode:
        raise ValueError('preflight manifest version drift')
    expected_claim = CLAIM_FULL if expected_mode == 'full' else CLAIM_SMOKE
    if manifest['claim_scope'] != expected_claim:
        raise ValueError('preflight claim enum drift')
    if (manifest['claim_boundary'] != CLAIM_BOUNDARY or
            manifest['pairing_contract'] != PAIRING_CONTRACT or
            manifest['observation_qos_contract'] !=
            OBSERVATION_QOS_CONTRACT or
            manifest['pose_causality'] != 'NOT_PROVEN'):
        raise ValueError('particle/pose claim boundary drift')
    source_contract = manifest['source_contract']
    contract_snapshot = root / source_contract['relative_path']
    if (source_contract != _relative_identity(contract_snapshot, root) or
            contract_snapshot.name != 'contract_snapshot.json'):
        raise ValueError('source contract snapshot identity drift')
    contract = _canonical_json_load(contract_snapshot)
    _validate_run_artifact_policy(contract)
    _validate_source_records(contract['production_inputs'])
    _validate_source_records(contract['harness_sources'])
    p0_params_identity = _validate_p0_params(
        contract, Path(contract['production_inputs']['production_params']['path']))
    for key, filename in (
            ('sanitizer_snapshot', 'sanitizer_manifest_snapshot.json'),
            ('upstream_lock_snapshot', 'upstream_lock_snapshot.json'),
            ('build_attestation', 'build_attestation.json'),
            ('replay_view', 'replay_view_manifest.json')):
        snapshot_path = root / filename
        if manifest[key] != _relative_identity(snapshot_path, root):
            raise ValueError(f'{key} identity drift')
    sanitizer_snapshot = _canonical_json_load(
        root / 'sanitizer_manifest_snapshot.json')
    _validate_sanitizer_snapshot(sanitizer_snapshot)
    _validate_tree_manifest(
        manifest['sanitized_input'],
        {'bag_0.mcap', 'metadata.yaml', 'sanitizer_manifest.json'},
        'sanitized input')
    sanitized_records = {
        item['relative_path']: item for item in manifest['sanitized_input']['files']}
    if (sanitized_records['sanitizer_manifest.json']['size_bytes'] !=
            manifest['sanitizer_snapshot']['size_bytes'] or
            sanitized_records['sanitizer_manifest.json']['sha256'] !=
            manifest['sanitizer_snapshot']['sha256'] or
            sanitized_records['bag_0.mcap']['size_bytes'] !=
            sanitizer_snapshot['output']['mcap_size_bytes'] or
            sanitized_records['bag_0.mcap']['sha256'] !=
            sanitizer_snapshot['output']['mcap_sha256'] or
            sanitized_records['metadata.yaml']['sha256'] !=
            sanitizer_snapshot['output']['metadata_sha256']):
        raise ValueError('sanitized input leaf binding drift')
    runtime = manifest['runtime']
    replay_view = _canonical_json_load(root / 'replay_view_manifest.json')
    _exact_keys(replay_view, {
        'schema_version', 'clock_contract',
        'source_sanitizer_manifest_sha256', 'bootstrap',
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
    if replay_view['clock_contract'] != {
            'source': 'rosbag2_player_synthesized_clock',
            'handoff': 'SEQUENTIAL_DUAL_PLAYER_CLOCK_HANDOFF',
            'per_run_proof': 'run_evidence_clock_handoff'}:
        raise ValueError('replay clock contract drift')
    if (replay_view['prelude_topics'] != ['/odom', '/tf', '/tf_static'] or
            replay_view['main_topics'] !=
            ['/scan', '/odom', '/tf', '/tf_static'] or
            replay_view['storage_gap_ns'] !=
            replay_view['main_first_storage_ns'] -
            replay_view['prelude_last_storage_ns'] or
            replay_view['main_last_scan_storage_ns'] <
            replay_view['main_first_storage_ns']):
        raise ValueError('replay view internal consistency drift')
    _validate_bootstrap_plan(replay_view['bootstrap'])
    _validate_replay_summary(
        replay_view['prelude'], replay_view['prelude_topics'], 'replay prelude')
    _validate_replay_summary(
        replay_view['main'], replay_view['main_topics'], 'replay main')
    if '/scan' not in replay_view['main']:
        raise ValueError('replay main scan evidence missing')
    build_attestation = _canonical_json_load(root / 'build_attestation.json')
    overlay_lock = _canonical_json_load(root / 'upstream_lock_snapshot.json')
    overlay_tree = _validate_overlay_snapshot(contract, overlay_lock)
    _exact_keys(build_attestation, {
        'schema_version', 'upstream_lock_sha256', 'build_root',
        'source_root', 'overlay_tree_sha256', 'cmake_cache',
        'build_returncode_file',
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
            build_attestation['overlay_tree_sha256'] !=
            overlay_tree['tree_sha256'] or
            build_attestation['install_files']['amcl'] !=
            build_attestation['loaded_runtime']['amcl_executable'] or
            build_attestation['install_files']['libamcl_core.so'] !=
            build_attestation['loaded_runtime']['libamcl_core'] or
            build_attestation['install_files']['libpf_lib.so'] !=
            build_attestation['loaded_runtime']['libpf_lib']):
        raise ValueError('build attestation drift')
    _validate_loaded_runtime(
        build_attestation['loaded_runtime'], 'build loaded runtime')
    _exact_keys(build_attestation['install_files'], {
        'amcl', 'libamcl_core.so', 'libpf_lib.so'}, 'build install files')
    for name, record in build_attestation['install_files'].items():
        _identity_schema(record, f'build install {name}')
    _exact_keys(build_attestation['needed_sonames'], {
        'amcl', 'libamcl_core.so'}, 'build needed SONAMEs')
    if any(type(names) is not list or names != sorted(set(names)) or
           any(type(name) is not str or not name for name in names)
           for names in build_attestation['needed_sonames'].values()):
        raise ValueError('build needed SONAME schema drift')
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
    if Path(runtime['rmw_library']['path']).name != 'librmw_cyclonedds_cpp.so':
        raise ValueError('runtime RMW library does not match implementation')
    for key in ('rmw_library', 'amcl_executable'):
        record = runtime[key]
        _identity_schema(record, f'runtime {key}')
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
            manifest['tree_bytes'] > G002_ARTIFACT_LIMIT_BYTES):
        raise ValueError('preflight artifact exceeds 32 MiB')
    runs = []
    for index, record in enumerate(manifest['runs'], 1):
        _identity_schema(record, 'run evidence record', relative=True)
        if record['relative_path'] != f'run_{index}/evidence.json':
            raise ValueError('run evidence order drift')
        path = root / record['relative_path']
        if path.is_symlink() or path.parent.parent != root:
            raise ValueError('run evidence path drift')
        if (_identity(path)['sha256'] != record['sha256'] or
                path.stat().st_size != record['size_bytes']):
            raise ValueError('run evidence identity drift')
        if _tree_bytes(path.parent) > G002_RUN_OUTPUT_LIMIT_BYTES:
            raise ValueError('run tree exceeds 8 MiB output cap')
        evidence = _canonical_json_load(path)
        _exact_keys(evidence, {
            'schema_version', 'run_id', 'profile', 'seed', 'domain_id',
            'status', 'failure', 'events', 'operations', 'observer_source',
            'observer_state', 'amcl_executable', 'map_yaml', 'params_file',
            'sanitized_manifest', 'resource', 'teardown', 'loaded_runtime',
            'prelude_observer', 'prelude_snapshot', 'clock_handoff',
            'tf_bootstrap',
            'amcl_tf_error_counts',
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
        if evidence['tf_bootstrap'] != replay_view['bootstrap']:
            raise ValueError('run replay bootstrap binding drift')
        if (evidence['failure'] is not None or
                evidence['status'] != 'PASS' or
                evidence['prefix_s'] not in {SMOKE_PREFIX_S, FULL_PREFIX_S} or
                evidence['playback_rate'] != PLAYBACK_RATE):
            raise ValueError('run status or playback contract drift')
        _exact_keys(evidence['amcl_tf_error_counts'],
                    set(AMCL_TF_ERROR_MARKERS), 'AMCL TF error counts')
        if any(type(value) is not int or value != 0
               for value in evidence['amcl_tf_error_counts'].values()):
            raise ValueError('AMCL TF error count drift')
        _exact_keys(evidence['scan_parity'], {
            'message_count', 'header_stamp_sha256', 'storage_stamp_sha256',
            'payload_sha256', 'first_header_stamp_ns',
            'last_header_stamp_ns'}, 'scan parity')
        if (type(evidence['scan_parity']['message_count']) is not int or
                evidence['scan_parity']['message_count'] <= 0 or
                any(type(evidence['scan_parity'][key]) is not int or
                    evidence['scan_parity'][key] < 0
                    for key in ('first_header_stamp_ns',
                                'last_header_stamp_ns'))):
            raise ValueError('scan parity scalar drift')
        for key in ('header_stamp_sha256', 'storage_stamp_sha256',
                    'payload_sha256'):
            _sha256(evidence['scan_parity'][key], f'scan parity {key}')
        observer = _load_observer_state(evidence['observer_state'], path.parent)
        evidence_with_observer = {**evidence, 'observer': observer}
        if evidence['run_id'] != observer['run_id'] or \
                evidence['seed'] != observer['seed']:
            raise ValueError('run and observer identity drift')
        for key in ('observer_source', 'amcl_executable', 'map_yaml',
                    'params_file', 'sanitized_manifest'):
            _identity_schema(evidence[key], f'run {key}')
        if (evidence['observer_source'] !=
                contract['harness_sources']['amcl_particle_observer.py'] or
                evidence['amcl_executable'] != evidence['loaded_runtime'][
                    'amcl_executable'] or
                evidence['map_yaml'] != contract['axis_a']['map']['yaml'] or
                evidence['params_file'] != p0_params_identity):
            raise ValueError('run source or executable identity drift')
        if type(evidence['events']) is not list:
            raise ValueError('run event list drift')
        for event in evidence['events']:
            _validate_event_record(event, 'run event', False)
        _validate_run_operations(evidence['operations'], evidence['seed'])
        _validate_loaded_runtime(evidence['loaded_runtime'], 'run loaded runtime')
        _validate_observer_snapshot(evidence['prelude_observer'],
                                    'prelude observer state')
        _validate_prelude_snapshot(
            evidence['prelude_snapshot'], evidence['prelude_observer'],
            path.parent)
        player_records = [record for record in evidence['teardown']
                          if type(record) is dict and
                          record.get('name') == 'player']
        if len(player_records) != 1 or 'started' not in player_records[0]:
            raise ValueError('main player start evidence missing')
        _validate_clock_handoff(
            evidence['clock_handoff'], evidence['prelude_observer'],
            observer, evidence['prelude_snapshot'],
            player_records[0]['started'])
        expected_run_files = {
            'amcl.log', 'amcl_resource.jsonl', 'evidence.json',
            'initialpose.request', 'map_server.log', 'observer.log',
            'observer_state.json', 'player.log', 'prelude_snapshot.request',
            'prelude_snapshot.ack.json',
            'prelude_snapshot.prelude_complete.json',
            'resource_sampler.log'}
        expected_run_files.add('tf_prelude_player.log')
        if {item.name for item in path.parent.iterdir()} != expected_run_files:
            raise ValueError('run file inventory drift')
        for process in evidence['teardown']:
            _exact_keys(process, {
                'name', 'pid', 'pgid', 'command', 'started', 'returncode',
                'stop_stages', 'survivors', 'log'}, 'teardown process')
            if (type(process['name']) is not str or
                    type(process['pid']) is not int or process['pid'] <= 0 or
                    type(process['pgid']) is not int or process['pgid'] <= 0 or
                    type(process['command']) is not list or
                    any(type(value) is not str for value in process['command']) or
                    type(process['returncode']) is not int or
                    type(process['survivors']) is not list or
                    any(type(value) is not int for value in process['survivors']) or
                    type(process['stop_stages']) is not list):
                raise ValueError('teardown process scalar drift')
            _validate_event_record(process['started'], 'process start', False)
            for stage_record in process['stop_stages']:
                _exact_keys(stage_record, {'signal', 'steady_ns'}, 'stop stage')
                if (stage_record['signal'] not in {
                        'SIGINT', 'SIGTERM', 'SIGKILL'} or
                        type(stage_record['steady_ns']) is not int or
                        stage_record['steady_ns'] < 0):
                    raise ValueError('stop stage scalar drift')
            _identity_schema(process['log'], 'process log', relative=True)
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
        _exact_keys(evidence['resource'], {
            'sample_count', 'cpu_total_start_s', 'cpu_total_end_s',
            'cpu_seconds', 'cpu_pct_median', 'cpu_pct_p95', 'max_rss_mb',
            'file'}, 'resource summary')
        _identity_schema(evidence['resource']['file'], 'resource file', relative=True)
        for key in ('cpu_total_start_s', 'cpu_total_end_s', 'cpu_seconds',
                    'cpu_pct_median', 'cpu_pct_p95', 'max_rss_mb'):
            _finite_number(evidence['resource'][key], f'resource {key}', 0.0)
        if (type(evidence['resource']['sample_count']) is not int or
                evidence['resource']['sample_count'] <= 0 or
                evidence['resource']['cpu_total_end_s'] <
                evidence['resource']['cpu_total_start_s'] or
                evidence['resource']['cpu_seconds'] !=
                evidence['resource']['cpu_total_end_s'] -
                evidence['resource']['cpu_total_start_s']):
            raise ValueError('resource summary consistency drift')
        if evidence['resource']['file'] != _relative_identity(
                resource_path, path.parent):
            raise ValueError('resource identity drift')
        if len(_resource_rows(resource_path)) != evidence['resource']['sample_count']:
            raise ValueError('resource raw sample cardinality drift')
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
                observer, sanitized_output_root):
            raise ValueError('run scan parity recomputation drift')
        _validate_replay_bootstrap(
            evidence_with_observer, path.parent, sanitized_output_root,
            replay_view['bootstrap'])
        if evidence['loaded_runtime'] != build_attestation['loaded_runtime']:
            raise ValueError('loaded runtime differs from build attestation')
        if (observer['initialpose_count'] != 1 or
                observer['pre_initial_scan_count'] != 0 or
                observer['pre_initial_cloud_count'] != 0 or
                observer['pending_cloud_count'] != 0 or
                observer['pending_pose_count'] != 0 or
                len(observer['clouds']) != observer['max_clouds'] or
                observer['raw_cloud_received_count'] !=
                observer['max_clouds'] or
                observer['amcl_pose_received_count'] !=
                observer['max_clouds'] or
                len(observer['callback_trace']) !=
                2 * observer['max_clouds']):
            raise ValueError('observer FIFO cardinality drift')
        if (observer['schema_version'] != 1 or observer['failure'] is not None or
                observer['done'] is not True or
                observer['publisher_matched_count'] < 1 or
                observer['pose_publisher_matched_count'] < 1 or
                observer['motion_command_applicability'] != 'NOT_APPLICABLE'):
            raise ValueError('observer completion contract drift')
        associated_scans = [
            cloud['fifo_associated_pose_scan_header_stamp_ns']
            for cloud in observer['clouds']]
        if (len(associated_scans) != len(set(associated_scans)) or
                any(value not in observer['scan_header_stamps_ns']
                    for value in associated_scans)):
            raise ValueError('FIFO-associated pose scan membership drift')
        if any(cloud['pose_header_stamp_ns'] !=
               cloud['fifo_associated_pose_scan_header_stamp_ns'] or
               cloud['pose_frame_id'] != 'map' or cloud['frame_id'] != 'map' or
               type(cloud['particle_count']) is not int or
               cloud['particle_count'] <= 0 for cloud in observer['clouds']):
            raise ValueError('particle/pose FIFO evidence drift')
        runs.append(evidence_with_observer)
    replay_scan = replay_view['main']['/scan']
    first_scan_parity = runs[0]['scan_parity']
    if (replay_scan['message_count'] != first_scan_parity['message_count'] or
            replay_scan['storage_stamp_sha256'] !=
            first_scan_parity['storage_stamp_sha256'] or
            replay_scan['raw_payload_sha256'] !=
            first_scan_parity['payload_sha256']):
        raise ValueError('replay view and accepted scan prefix drift')
    sanitized_root = Path(manifest['sanitized_input']['root'])
    if sanitized_root.exists():
        expected_replay = _replay_view_manifest(
            sanitized_root, replay_view['bootstrap'],
            runs[0]['observer']['scan_header_stamps_ns'][-1])
        if replay_view != expected_replay:
            raise ValueError('live replay view recomputation drift')
    if manifest['comparison'] != _comparison(runs):
        raise ValueError('preflight comparison drift')
    if ([run['domain_id'] for run in runs] != runtime['domain_ids'] or
            len(runs) != len(runtime['domain_ids'])):
        raise ValueError('run domain mapping drift')
    if expected_mode == 'full':
        if (len(runs) != 3 or tuple(run['seed'] for run in runs) != FULL_SEEDS or
                [run['run_id'] for run in runs] != [
                    'p0__seed_11__attempt_1', 'p0__seed_11__attempt_2',
                    'p0__seed_23__attempt_3'] or
                any(len(run['observer']['clouds']) != FULL_CLOUD_COUNT or
                    run['observer']['scan_count'] != FULL_SCAN_COUNT or
                    run['observer']['max_clouds'] != FULL_CLOUD_COUNT or
                    run['prefix_s'] != FULL_PREFIX_S or
                    run['playback_rate'] != PLAYBACK_RATE
                    for run in runs) or
                any(run['scan_parity'] != runs[0]['scan_parity']
                    for run in runs[1:]) or
                not all(manifest['comparison'].values())):
            raise ValueError('full determinism contract failed')
        if (runtime['domain_ids'] != [
                runs[0]['domain_id'] + index for index in range(3)] or
                [run['domain_id'] for run in runs] != runtime['domain_ids']):
            raise ValueError('full domain mapping drift')
    elif (len(runs) != 1 or runs[0]['seed'] != SMOKE_SEEDS[0] or
          runs[0]['run_id'] != 'p0__seed_11__attempt_1' or
          runs[0]['observer']['max_clouds'] != SMOKE_CLOUD_COUNT or
          len(runs[0]['observer']['clouds']) != SMOKE_CLOUD_COUNT or
          runs[0]['prefix_s'] != SMOKE_PREFIX_S or
          runs[0]['playback_rate'] != PLAYBACK_RATE or
          manifest['comparison'] != {
              'evaluated': False, 'same_seed_equal': None,
              'different_seed_differs': None}):
        raise ValueError('smoke artifact contract failed')
    return manifest


def validate_smoke_artifact(root: Path) -> dict:
    """Validate exactly one no-motion pipeline smoke run."""
    return _validate_artifact(root, 'smoke')


def validate_full_artifact(root: Path) -> dict:
    """Validate the exact seed-11/11/23 determinism preflight."""
    return _validate_artifact(root, 'full')


def _mode_contract(args) -> tuple[str, tuple[int, ...]]:
    mode = args.mode
    if mode == 'smoke':
        expected = (SMOKE_SEEDS, SMOKE_CLOUD_COUNT, SMOKE_PREFIX_S)
    elif mode == 'full':
        expected = (FULL_SEEDS, FULL_CLOUD_COUNT, FULL_PREFIX_S)
    else:
        raise ValueError('preflight mode must be smoke or full')
    seeds, cloud_count, prefix_s = expected
    if (tuple(args.seeds) != seeds or args.max_clouds != cloud_count or
            args.prefix_s != prefix_s or args.playback_rate != PLAYBACK_RATE):
        raise ValueError(f'{mode} preflight execution contract drift')
    return mode, seeds


def _publish_validated_stage(stage: Path, output_root: Path, validator,
                             source_groups: tuple[dict, ...]) -> dict:
    validator(stage)
    for records in source_groups:
        _validate_source_records(records)
    stage.rename(output_root)
    try:
        for records in source_groups:
            _validate_source_records(records)
        return validator(output_root)
    except Exception:
        _safe_remove_created_root(output_root)
        raise


def run_preflight(args) -> dict:
    """Run one smoke or the exact same-seed/different-seed preflight."""
    output_root = _canonical_output_root(args.output_root)
    sanitized = validate_sanitized_bag(args.sanitized_root)
    if sanitized['source']['mcap_sha256'] != REAL_BAG_SHA256:
        raise ValueError('sanitized source is not canonical Axis A')
    contract = strict_json_load(args.prepared_root / 'contract.json')
    mode, seeds = _mode_contract(args)
    _validate_run_artifact_policy(contract)
    _validate_p0_params(contract, args.params_file)
    if (contract['axis_a']['map']['yaml']['sha256'] != REAL_MAP_YAML_SHA256 or
            contract['axis_a']['map']['pgm']['sha256'] != REAL_MAP_PGM_SHA256):
        raise ValueError('Axis A map identity drift')
    if args.map_yaml != Path(contract['axis_a']['map']['yaml']['path']):
        raise ValueError('map path differs from prepared contract')
    _validate_source_records(contract['production_inputs'])
    _validate_source_records(contract['harness_sources'])
    budget = validate_storage_budget(
        current_free_bytes(output_root.parent), G002_ARTIFACT_LIMIT_BYTES,
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
    overlay_lock = strict_json_load(Path(contract['source_overlay']['lock']['path']))
    overlay_tree = _validate_overlay_snapshot(contract, overlay_lock)
    sanitized_input = _tree_manifest(args.sanitized_root)
    _validate_tree_manifest(
        sanitized_input,
        {'bag_0.mcap', 'metadata.yaml', 'sanitizer_manifest.json'},
        'sanitized input')
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
        run_views = [
            {**run, 'observer': _load_observer_state(
                run['observer_state'], stage / f'run_{index}')}
            for index, run in enumerate(runs, 1)]
        build_attestation = {
            'upstream_lock_sha256': sha256_file(
                stage / 'upstream_lock_snapshot.json'),
            **_build_provenance(
                args.amcl_executable,
                Path(contract['source_overlay']['root']),
                runs[0]['loaded_runtime'], overlay_tree['tree_sha256']),
        }
        (stage / 'build_attestation.json').write_bytes(
            canonical_json_bytes(build_attestation))
        scan_views = [
            run['observer']['scan_header_stamps_ns'] for run in run_views]
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
        manifest = {
            'schema_version': 3, 'mode': mode,
            'claim_scope': CLAIM_FULL if mode == 'full' else CLAIM_SMOKE,
            'claim_boundary': CLAIM_BOUNDARY,
            'pairing_contract': PAIRING_CONTRACT,
            'observation_qos_contract': OBSERVATION_QOS_CONTRACT,
            'pose_causality': 'NOT_PROVEN',
            'source_contract': _relative_identity(
                stage / 'contract_snapshot.json', stage),
            'sanitized_input': sanitized_input,
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
            'runs': records, 'comparison': _comparison(run_views),
            'tree_bytes': _payload_tree_bytes(stage),
            'tree_records': _tree_records(stage),
            'tree_sha256': _tree_digest(_tree_records(stage)),
        }
        (stage / 'preflight_manifest.json').write_bytes(
            canonical_json_bytes(manifest))
        validator = validate_full_artifact if mode == 'full' else validate_smoke_artifact
        return _publish_validated_stage(
            stage, output_root, validator,
            (contract['production_inputs'], contract['harness_sources']))


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
    parser.add_argument('--mode', choices=('smoke', 'full'), required=True)
    args = parser.parse_args()
    if args.mode == 'smoke':
        args.seeds = list(SMOKE_SEEDS)
        args.max_clouds = SMOKE_CLOUD_COUNT
        args.prefix_s = SMOKE_PREFIX_S
    else:
        args.seeds = list(FULL_SEEDS)
        args.max_clouds = FULL_CLOUD_COUNT
        args.prefix_s = FULL_PREFIX_S
    args.playback_rate = PLAYBACK_RATE
    run_preflight(args)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
