#!/usr/bin/env python3
"""Run the deterministic 25-case Gazebo fixed-obstacle Nav2 evaluation."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import selectors
import shutil
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path  # noqa: I100
from statistics import NormalDist  # noqa: I100
from typing import Any  # noqa: I100

from evaluate_sim_nav_obstacle import aggregate

from jdamr_cube_navigation.sim_nav_obstacle_scenario import read_entity_pose

from prepare_sim_nav_obstacle_run import archive_existing_attempt, prepare

from sim_nav_obstacle_contract import (  # noqa: I101
    SCENARIOS, SEEDS, scenario_matrix)

import yaml


EVALUATION_DIR = Path(__file__).resolve().parent
ROOT = EVALUATION_DIR.parents[1]
ASSETS = EVALUATION_DIR / 'assets' / 'nav_obstacle'
BT = (ROOT / 'jdamr_cube_navigation' / 'behavior_trees'
      / 'navigate_to_pose_dynamic_obstacle_eval.xml')
NAV2_USE_COMPOSITION = True
PROCESS_MARKER = 'JDAMR_NAV_EVAL_RUN_ID'
ARM_PROBE_WORLD_X_M = -7.80
ARM_PROBE_WORLD_Y_M = 0.0
ARM_PROBE_WORLD_Z_M = 0.30
POSE_SETTLE_POSITION_TOLERANCE_M = 0.002
POSE_SETTLE_ANGLE_TOLERANCE_RAD = 0.01
SCAN_SELF_RETURN_MAX_RANGE_M = 0.90
SCAN_SELF_RETURN_MIN_ABS_ANGLE_RAD = math.radians(145.0)
CONTACT_SMOKE_MAX_ATTEMPTS = 2
EVALUATION_RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
EVALUATION_RMW_OVERLAY_PREFIX: Path | None = None
FASTDDS_LOAD_NODE_TIMEOUT_SIGNATURE = (
    'failed to send response to /nav2_container/_container/load_node '
    '(timeout)')


class InfrastructureInvalid(RuntimeError):
    """Identify a startup failure before any scenario action begins."""


def configure_evaluation_rmw(
        implementation: str, overlay_prefix: Path | None) -> None:
    """Configure one evaluation-only RMW without mutating the host setup."""
    global EVALUATION_RMW_IMPLEMENTATION, EVALUATION_RMW_OVERLAY_PREFIX
    if implementation not in {'rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp'}:
        raise ValueError(f'unsupported evaluation RMW: {implementation}')
    if implementation == 'rmw_cyclonedds_cpp':
        if overlay_prefix is None or not (
                overlay_prefix / 'lib' / 'librmw_cyclonedds_cpp.so').is_file():
            raise ValueError('CycloneDDS overlay prefix is incomplete')
    elif overlay_prefix is not None:
        raise ValueError('FastDDS evaluation does not accept an overlay')
    EVALUATION_RMW_IMPLEMENTATION = implementation
    EVALUATION_RMW_OVERLAY_PREFIX = overlay_prefix


def classify_startup_log(text: str) -> str | None:
    """Classify the known FastDDS composable service response race."""
    if FASTDDS_LOAD_NODE_TIMEOUT_SIGNATURE in text:
        return 'INFRA_INVALID_FASTDDS_SERVICE_DISCOVERY'
    return None


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _evaluation_input_hashes(raw_root: Path) -> dict[str, str]:
    """Hash every shared input that permits preflight evidence reuse."""
    paths = {
        'contract': raw_root / 'contract.json',
        'evaluation_params': raw_root / 'nav2_obstacle_eval.params.yaml',
        'asset_manifest': ASSETS / 'asset_manifest.json',
        'evaluation_world': ASSETS / 'slam_corridor_contact.world',
        'evaluation_urdf': ASSETS / 'jdamr_cube_nav_eval.urdf',
        'evaluation_bt': BT,
        'evaluation_runner': Path(__file__).resolve(),
        'scenario_node': (ROOT / 'jdamr_cube_navigation'
                          / 'jdamr_cube_navigation'
                          / 'sim_nav_obstacle_scenario.py'),
    }
    return {name: _sha256(path) for name, path in paths.items()}


def _preflight_cache_matches(
        document: dict[str, Any], input_hashes: dict[str, str],
        seed: int | None = None) -> bool:
    """Fail closed unless status, hashes, and optional seed all match."""
    return (
        document.get('status') == 'PASS'
        and document.get('input_hashes') == input_hashes
        and (seed is None or document.get('seed') == seed))


def _contact_smoke_cache_matches(
        path: Path, input_hashes: dict[str, str]) -> bool:
    """Require a successful contact smoke from the exact evaluation inputs."""
    if not path.is_file():
        return False
    document = json.loads(path.read_text(encoding='utf-8'))
    return _preflight_cache_matches(document, input_hashes)


def _archive_contact_smoke(raw_root: Path) -> Path:
    """Preserve one failed contact preflight and its content hash."""
    source = raw_root / 'contact_smoke.json'
    audit_root = raw_root / 'development_audit'
    audit_root.mkdir(exist_ok=True)
    attempt = 1
    while (audit_root / f'contact_smoke_attempt_{attempt}').exists():
        attempt += 1
    archived = audit_root / f'contact_smoke_attempt_{attempt}'
    archived.mkdir()
    destination = archived / source.name
    file_evidence = None
    if source.is_file():
        shutil.copy2(source, destination)
        file_evidence = {
            'path': source.name,
            'size_bytes': destination.stat().st_size,
            'sha256': _sha256(destination),
        }
    (archived / 'attempt_manifest.json').write_text(json.dumps({
        'schema_version': 1,
        'attempt': attempt,
        'status': 'FAIL',
        'failure_reason': ('contact_smoke_artifact_missing'
                           if file_evidence is None else None),
        'file': file_evidence,
    }, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return archived


def _start(command: list[str], log_path: Path,
           environment: dict[str, str]) -> tuple[subprocess.Popen, Any]:
    stream = log_path.open('wb')
    process = subprocess.Popen(
        command, env=environment, stdout=stream, stderr=subprocess.STDOUT,
        start_new_session=True)
    return process, stream


def _group_members(process_group: int) -> list[int]:
    members = []
    for stat_path in Path('/proc').glob('[0-9]*/stat'):
        try:
            raw = stat_path.read_text()
            fields = raw[raw.rfind(')') + 2:].split()
            if int(fields[2]) == process_group:
                members.append(int(stat_path.parent.name))
        except (OSError, ValueError, IndexError):
            continue
    return sorted(members)


def _wait_process_group_empty(process_group: int, timeout_s: float) -> bool:
    """Bound teardown evidence until the process group is empty or expires."""
    deadline_s = time.monotonic() + timeout_s
    while _group_members(process_group):
        if time.monotonic() >= deadline_s:
            return False
        time.sleep(0.05)
    return True


def _stop(process: subprocess.Popen | None) -> None:
    if process is None:
        return
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=12.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    return
                process.wait(timeout=3.0)
    if _group_members(process.pid):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    _wait_process_group_empty(process.pid, 2.0)


def _identity_survivors(run_id: str, domain_id: int) -> list[dict[str, Any]]:
    expected = {
        f'{PROCESS_MARKER}={run_id}', f'ROS_DOMAIN_ID={domain_id}'}
    survivors = []
    for environ_path in Path('/proc').glob('[0-9]*/environ'):
        try:
            values = set(environ_path.read_bytes().decode(
                'utf-8', errors='replace').split('\0'))
            if expected <= values:
                pid = int(environ_path.parent.name)
                stat = (environ_path.parent / 'stat').read_text()
                fields = stat[stat.rfind(')') + 2:].split()
                cmdline = (environ_path.parent / 'cmdline').read_bytes()
                survivors.append({
                    'pid': pid,
                    'process_group': int(fields[2]),
                    'command': cmdline.replace(b'\0', b' ').decode(
                        'utf-8', errors='replace').strip(),
                })
        except (OSError, ValueError, IndexError):
            continue
    return sorted(survivors, key=lambda item: item['pid'])


def _cleanup_identity(run_id: str, domain_id: int) -> list[dict[str, Any]]:
    """Stop exact run-id/domain descendants left outside owned groups."""
    for signum, timeout_s in ((signal.SIGTERM, 3.0),
                              (signal.SIGKILL, 2.0)):
        matches = _identity_survivors(run_id, domain_id)
        if not matches:
            return []
        for match in matches:
            try:
                os.kill(match['pid'], signum)
            except ProcessLookupError:
                pass
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            if not _identity_survivors(run_id, domain_id):
                return []
            time.sleep(0.1)
    return _identity_survivors(run_id, domain_id)


def _wait_topics(required: set[str], environment: dict[str, str],
                 timeout_s: float, startup_log: Path | None = None) -> None:
    deadline_s = time.monotonic() + timeout_s
    seen: set[str] = set()
    while time.monotonic() < deadline_s:
        if startup_log is not None and startup_log.is_file():
            classification = classify_startup_log(
                startup_log.read_text(errors='replace'))
            if classification is not None:
                raise InfrastructureInvalid(classification)
        result = subprocess.run(
            ['ros2', 'topic', 'list'], env=environment,
            capture_output=True, text=True, timeout=5.0, check=False)
        if result.returncode == 0:
            seen = set(result.stdout.splitlines())
            if required <= seen:
                return
        time.sleep(0.5)
    raise RuntimeError(f'topic readiness timeout: {sorted(required - seen)}')


def _wait_gz_topic(topic: str, environment: dict[str, str],
                   timeout_s: float) -> bool:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        result = subprocess.run(
            ['gz', 'topic', '-l'], env=environment,
            capture_output=True, text=True, timeout=5.0, check=False)
        if result.returncode == 0 and topic in result.stdout.splitlines():
            return True
        time.sleep(0.1)
    return False


def _wait_lifecycle_active(
        nodes: tuple[str, ...], environment: dict[str, str],
        timeout_s: float) -> None:
    """Wait until every required Nav2 lifecycle node reports active."""
    deadline_s = time.monotonic() + timeout_s
    pending = set(nodes)
    while time.monotonic() < deadline_s:
        for node in tuple(pending):
            try:
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'get', node], env=environment,
                    capture_output=True, text=True, timeout=5.0,
                    check=False)
            except subprocess.TimeoutExpired:
                continue
            if result.returncode == 0 and _lifecycle_state_is_active(
                    result.stdout):
                pending.remove(node)
        if not pending:
            return
        time.sleep(0.5)
    raise RuntimeError(f'lifecycle readiness timeout: {sorted(pending)}')


def _lifecycle_state_is_active(output: str) -> bool:
    """Accept only the exact ROS lifecycle active state and numeric ID."""
    return re.fullmatch(r'\s*active\s*\[3\]\s*', output,
                        flags=re.IGNORECASE) is not None


def _tf_echo_has_transform(output: str | bytes | None) -> bool:
    """Recognize a complete transform sample from tf2_echo output."""
    if output is None:
        return False
    if isinstance(output, bytes):
        output = output.decode(errors='replace')
    return (
        re.search(r'^At time ', output, flags=re.MULTILINE) is not None
        and re.search(r'^- Translation:', output, flags=re.MULTILINE)
        is not None
        and re.search(r'^- Rotation: in Quaternion', output,
                      flags=re.MULTILINE) is not None)


def _wait_tf_available(source_frame: str, target_frame: str,
                       environment: dict[str, str], timeout_s: float,
                       evidence_path: Path | None = None) -> None:
    """Wait fail-closed for one complete source-to-target TF sample."""
    process = subprocess.Popen([
        'ros2', 'run', 'tf2_ros', 'tf2_echo', source_frame,
        target_frame, '-r', '10.0'], env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, start_new_session=True)
    if process.stdout is None:
        raise RuntimeError('tf2_echo stdout pipe unavailable')
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    deadline_s = time.monotonic() + timeout_s
    lines = []
    try:
        while time.monotonic() < deadline_s:
            events = selector.select(
                timeout=max(
                    0.0, min(0.5, deadline_s - time.monotonic())))
            for key, _ in events:
                line = key.fileobj.readline()
                if line:
                    lines.append(line)
            if _tf_echo_has_transform(''.join(lines)):
                return
            if process.poll() is not None:
                break
    finally:
        selector.close()
        _stop(process)
        process.stdout.close()
        if evidence_path is not None:
            evidence_path.write_text(''.join(lines), encoding='utf-8')
    raise RuntimeError(
        f'TF readiness timeout: {source_frame} -> {target_frame}')


def _set_pose(name: str, x_m: float, y_m: float, z_m: float,
              environment: dict[str, str]) -> bool:
    request = (
        f'name: "{name}", position {{x: {x_m}, y: {y_m}, z: {z_m}}}')
    result = subprocess.run([
        'gz', 'service', '-s', '/world/slam_corridor/set_pose',
        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', request,
    ], env=environment, capture_output=True, text=True, timeout=10.0,
        check=False)
    return result.returncode == 0 and 'data: true' in result.stdout.lower()


def _wait_entity_pose(
        name: str, expected_pose_m: list[float],
        environment: dict[str, str], timeout_s: float = 5.0
        ) -> dict[str, Any]:
    deadline_s = time.monotonic() + timeout_s
    last_state = None
    last_error = None
    while time.monotonic() < deadline_s:
        try:
            last_state = read_entity_pose(
                name, timeout_s=min(2.0, timeout_s), environment=environment)
        except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
            last_error = error
            time.sleep(0.05)
            continue
        if math.dist(last_state['pose_m'], expected_pose_m) <= 0.02:
            return last_state
    raise RuntimeError(
        f'entity pose did not reach target: {name}: '
        f'state={last_state}, error={last_error}')


def _echo_once(topic: str, message_type: str,
               environment: dict[str, str], timeout_s: float,
               sensor_data_qos: bool = False
               ) -> subprocess.CompletedProcess:
    qos_arguments = []
    if sensor_data_qos:
        qos_arguments = [
            '--qos-reliability', 'best_effort',
            '--qos-durability', 'volatile']
    try:
        return subprocess.run([
            'ros2', 'topic', 'echo', topic, message_type, '--once',
            '--timeout', str(timeout_s), '--full-length', *qos_arguments,
        ], env=environment, capture_output=True, text=True,
            timeout=timeout_s + 3.0, check=False)
    except subprocess.TimeoutExpired as error:
        return subprocess.CompletedProcess(
            error.cmd, 124, error.stdout or '', error.stderr or '')


def _scan_stamp_ns(output: str) -> int | None:
    match = re.search(
        r'stamp:\s*\n\s*sec:\s*(\d+)\s*\n\s*nanosec:\s*(\d+)', output)
    if match is None:
        return None
    return int(match.group(1)) * 1_000_000_000 + int(match.group(2))


def _newer_scan_document(
        output: str, newer_than_ns: int | None) -> str | None:
    """Return the first complete scan document newer than the reference."""
    documents = output.split('\n---')
    for document in documents[:-1]:
        stamp_ns = _scan_stamp_ns(document)
        if (stamp_ns is not None
                and (newer_than_ns is None or stamp_ns > newer_than_ns)):
            return document.strip() + '\n'
    return None


def _wait_scan(environment: dict[str, str], timeout_s: float,
               newer_than_ns: int | None = None
               ) -> subprocess.CompletedProcess:
    """Wait on one persistent SensorDataQoS subscriber for a fresh scan."""
    started_s = time.monotonic()
    deadline_s = time.monotonic() + timeout_s
    command = [
        'ros2', 'topic', 'echo', '/scan', 'sensor_msgs/msg/LaserScan',
        '--full-length', '--qos-reliability', 'best_effort',
        '--qos-durability', 'volatile']
    process = subprocess.Popen(
        command, env={**environment, 'PYTHONUNBUFFERED': '1'},
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, start_new_session=True)
    if process.stdout is None:
        raise RuntimeError('scan subscriber stdout pipe unavailable')
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    lines = []
    result = subprocess.CompletedProcess(command, 124, '', '')
    try:
        while time.monotonic() < deadline_s:
            events = selector.select(
                timeout=max(
                    0.0, min(0.5, deadline_s - time.monotonic())))
            for key, _ in events:
                line = key.fileobj.readline()
                if line:
                    lines.append(line)
            document = _newer_scan_document(
                ''.join(lines), newer_than_ns)
            if document is not None:
                result = subprocess.CompletedProcess(
                    command, 0, document, '')
                break
            if process.poll() is not None:
                break
    finally:
        selector.close()
        _stop(process)
        process.stdout.close()
    result.scan_attempt_count = 1
    result.scan_wait_elapsed_s = time.monotonic() - started_s
    return result


def _collision_names(output: str) -> list[str]:
    return sorted(set(re.findall(
        r'name:\s*["\']?([^\s"\']+::[^\s"\']+)["\']?', output)))


def _arm_link_tokens(robot_urdf: Path) -> tuple[str, ...]:
    root = ET.parse(robot_urdf).getroot()
    return tuple(sorted(
        link.attrib['name'] for link in root.findall('link')
        if link.attrib.get('name', '').startswith('arm_')))


def _classify_robot_collisions(
        names: list[str], arm_link_tokens: tuple[str, ...]
        ) -> tuple[list[str], list[str]]:
    robot_names = [name for name in names if name.startswith('jdamr_cube::')]
    arm_names = [
        name for name in robot_names
        if any(token in name.split('::', 1)[1]
               for token in arm_link_tokens)]
    unexpected_names = [name for name in robot_names if name not in arm_names]
    return arm_names, unexpected_names


def _wait_arm_contact(topic: str, environment: dict[str, str],
                      timeout_s: float,
                      arm_link_tokens: tuple[str, ...]
                      ) -> subprocess.CompletedProcess:
    deadline_s = time.monotonic() + timeout_s
    last = subprocess.CompletedProcess([], 124, '', '')
    while time.monotonic() < deadline_s:
        last = _echo_once(
            topic, 'ros_gz_interfaces/msg/Contacts', environment, 2.0)
        names = _collision_names(last.stdout)
        arm_names, unexpected_names = _classify_robot_collisions(
            names, arm_link_tokens)
        if arm_names and not unexpected_names:
            return last
    return last


def _yaml_message(output: str) -> dict[str, Any]:
    header = re.search(r'(?m)^header:\s*$', output)
    if header is None:
        raise ValueError('ROS topic echo returned no header')
    message = yaml.safe_load(output[header.start():].split('\n---', 1)[0])
    if not isinstance(message, dict):
        raise ValueError('ROS topic echo returned no message document')
    return message


def _pose_vector(output: str) -> list[float]:
    message = _yaml_message(output)
    position = message['pose']['position']
    orientation = message['pose']['orientation']
    x = float(orientation['x'])
    y = float(orientation['y'])
    z = float(orientation['z'])
    w = float(orientation['w'])
    roll = math.atan2(
        2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(
        1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(
        2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [float(position['x']), float(position['y']),
            float(position['z']), roll, pitch, yaw]


def _pose_close(first: list[float], second: list[float]) -> bool:
    return (
        max(abs(first[index] - second[index]) for index in range(3))
        <= POSE_SETTLE_POSITION_TOLERANCE_M
        and max(abs(first[index] - second[index]) for index in range(3, 6))
        <= POSE_SETTLE_ANGLE_TOLERANCE_RAD)


def _settled_pose(
        environment: dict[str, str], timeout_s: float) -> list[float]:
    deadline_s = time.monotonic() + timeout_s
    previous = None
    while time.monotonic() < deadline_s:
        echo = _echo_once(
            '/ground_truth_pose', 'geometry_msgs/msg/PoseStamped',
            environment, 5.0)
        if echo.returncode != 0:
            continue
        try:
            current = _pose_vector(echo.stdout)
        except (KeyError, TypeError, ValueError):
            continue
        if previous is not None and _pose_close(previous, current):
            return current
        previous = current
    raise RuntimeError('robot pose did not settle before scan capture')


def _scan_profile(output: str) -> dict[str, Any]:
    scan = _yaml_message(output)
    ranges = [float(value) for value in scan['ranges']]
    angle_min = float(scan['angle_min'])
    angle_increment = float(scan['angle_increment'])
    close_edge = [
        index for index, distance_m in enumerate(ranges)
        if abs(angle_min + index * angle_increment)
        >= SCAN_SELF_RETURN_MIN_ABS_ANGLE_RAD
        and 0.0 < distance_m < SCAN_SELF_RETURN_MAX_RANGE_M]
    wall_beams = {}
    for label, angle_rad in (
            ('south', -math.pi / 2.0), ('west', 0.0),
            ('north', math.pi / 2.0)):
        index = round((angle_rad - angle_min) / angle_increment)
        wall_beams[label] = ranges[index]
    return {
        'frame_id': scan['header']['frame_id'],
        'sample_count': len(ranges),
        'angle_min_rad': angle_min,
        'angle_max_rad': float(scan['angle_max']),
        'angle_increment_rad': angle_increment,
        'range_min_m': float(scan['range_min']),
        'range_max_m': float(scan['range_max']),
        'unexpected_near_edge_ground_return_indices': close_edge,
        'unexpected_near_edge_ground_return_count': len(close_edge),
        'all_beam_near_return_count': sum(
            0.0 < distance_m < SCAN_SELF_RETURN_MAX_RANGE_M
            for distance_m in ranges),
        'wall_beams_m': wall_beams,
    }


def _wall_beam_comparison_evidence(
        evaluations: list[dict[str, Any]], seeds: tuple[int, ...],
        scan_contract: dict[str, Any]) -> dict[str, Any]:
    """Evaluate direction mean biases and retain per-seed diagnostics."""
    expected = scan_contract['expected_wall_beams_m']
    gate = scan_contract['wall_beam_multiple_comparison']
    direction_count = len(expected)
    replicate_count = len(seeds)
    raw_comparison_count = direction_count * replicate_count
    alpha = float(gate['familywise_alpha'])
    sigma_m = float(gate['noise_sigma_m'])
    scalar_inputs_valid = (
        direction_count > 0 and replicate_count > 0
        and len(set(seeds)) == replicate_count and math.isfinite(alpha)
        and 0.0 < alpha < 1.0 and math.isfinite(sigma_m)
        and sigma_m > 0.0)
    derived_z = (NormalDist().inv_cdf(
        1.0 - alpha / (2.0 * direction_count))
        if scalar_inputs_valid else math.nan)
    standard_error_m = (sigma_m / math.sqrt(replicate_count)
                        if scalar_inputs_valid else math.nan)
    derived_tolerance_m = derived_z * standard_error_m
    contract_valid = (
        scalar_inputs_valid and gate.get('protocol_version') == 2
        and gate.get('unit_of_analysis') == 'direction_mean_bias'
        and gate.get('two_sided') is True
        and gate.get('correction') == 'bonferroni'
        and gate.get('comparison_count') == direction_count
        and gate.get('direction_count') == direction_count
        and gate.get('replicate_count') == replicate_count
        and gate.get('replicate_seeds') == list(seeds)
        and math.isclose(float(gate.get('critical_z', math.nan)), derived_z,
                         rel_tol=0.0, abs_tol=1e-15)
        and math.isclose(
            float(gate.get('standard_error_m', math.nan)), standard_error_m,
            rel_tol=0.0, abs_tol=1e-15)
        and math.isclose(
            float(gate.get('wall_beam_mean_tolerance_m', math.nan)),
            derived_tolerance_m, rel_tol=0.0, abs_tol=1e-15))
    records = []
    complete = len(evaluations) == len(seeds)
    for seed, item in zip(seeds, evaluations):
        observed_beams = item.get('profile', {}).get('wall_beams_m', {})
        for direction, expected_m in expected.items():
            observed_m = observed_beams.get(direction)
            finite = (scalar_inputs_valid and observed_m is not None
                      and math.isfinite(float(expected_m))
                      and math.isfinite(float(observed_m)))
            if observed_m is None or not finite:
                complete = False
                signed_error_m = math.nan
                absolute_error_m = math.nan
                z_score = math.nan
                accepted = False
            else:
                signed_error_m = float(observed_m) - float(expected_m)
                absolute_error_m = abs(signed_error_m)
                z_score = signed_error_m / sigma_m
            records.append({
                'seed': seed,
                'direction': direction,
                'observed_m': observed_m,
                'expected_m': expected_m,
                'signed_error_m': signed_error_m,
                'absolute_error_m': absolute_error_m,
                'z_score': z_score,
            })
    unique_comparisons = {
        (record['seed'], record['direction']) for record in records}
    complete = (complete and len(records) == raw_comparison_count
                and len(unique_comparisons) == raw_comparison_count)
    finite_errors = [record['absolute_error_m'] for record in records
                     if math.isfinite(record['absolute_error_m'])]
    direction_records = []
    for direction, expected_m in expected.items():
        residuals = [record['signed_error_m'] for record in records
                     if record['direction'] == direction
                     and math.isfinite(record['signed_error_m'])]
        direction_complete = len(residuals) == replicate_count
        mean_bias_m = (sum(residuals) / replicate_count
                       if direction_complete else math.nan)
        accepted = (direction_complete
                    and abs(mean_bias_m) <= derived_tolerance_m)
        direction_records.append({
            'direction': direction,
            'expected_m': expected_m,
            'replicate_count': len(residuals),
            'mean_bias_m': mean_bias_m,
            'absolute_mean_bias_m': abs(mean_bias_m),
            'z_score': mean_bias_m / standard_error_m,
            'accepted': accepted,
        })
    passed = (contract_valid and complete
              and all(record['accepted'] for record in direction_records))
    return {
        'status': 'PASS' if passed else 'FAIL',
        'contract_valid': contract_valid,
        'complete': complete,
        'familywise_alpha': alpha,
        'protocol_version': 2,
        'unit_of_analysis': 'direction_mean_bias',
        'per_comparison_alpha': alpha / direction_count,
        'comparison_count': direction_count,
        'direction_count': direction_count,
        'replicate_count': replicate_count,
        'raw_comparison_count': raw_comparison_count,
        'two_sided': True,
        'correction': 'bonferroni',
        'formula': 'NormalDist.inv_cdf(1-alpha/(2*m))*sigma/sqrt(n)',
        'critical_z': derived_z,
        'noise_sigma_m': sigma_m,
        'standard_error_m': standard_error_m,
        'tolerance_m': derived_tolerance_m,
        'max_absolute_error_m': max(finite_errors, default=None),
        'records': records,
        'direction_records': direction_records,
    }


def _scan_ab_preflight(raw_root: Path, domain_id: int) -> dict[str, Any]:
    preflight_root = raw_root / 'scan_ab_preflight'
    evidence_path = preflight_root / 'evidence.json'
    if evidence_path.is_file():
        cached = json.loads(evidence_path.read_text(encoding='utf-8'))
        captures = cached.get('captures', {})
        expected_hashes = {
            'source_seed_11': _sha256(
                ROOT / 'jdamr_cube_description' / 'urdf'
                / 'jdamr_cube.urdf'),
            **{
                f'evaluation_seed_{seed}': _sha256(
                    ASSETS / 'jdamr_cube_nav_eval.urdf')
                for seed in SEEDS},
        }
        if (_preflight_cache_matches(
                cached, _evaluation_input_hashes(raw_root))
                and all(captures.get(label, {}).get('urdf_sha256') == digest
                        for label, digest in expected_hashes.items())):
            return {**cached, 'cache_reused': True}
        archive_existing_attempt(preflight_root)
    preflight_root.mkdir(exist_ok=True)
    run_id = 'scan_ab_preflight'
    environment = _environment(run_id, domain_id)
    captures = {}
    groups = []
    capture_error = None
    source_urdf = (ROOT / 'jdamr_cube_description' / 'urdf'
                   / 'jdamr_cube.urdf')
    variants = [('source_seed_11', source_urdf, 11)]
    variants.extend(
        (f'evaluation_seed_{seed}', ASSETS / 'jdamr_cube_nav_eval.urdf',
         seed)
        for seed in SEEDS)
    for label, urdf, seed in variants:
        process, stream = _start([
            'ros2', 'launch', 'jdamr_cube_gazebo', 'gazebo.launch.py',
            f'world:={ASSETS / "slam_corridor_contact.world"}',
            f'urdf_file:={urdf}', 'gui:=false',
            'enable_image_bridges:=false', f'seed:={seed}',
            'x_pose:=-8.0', 'y_pose:=0.0', 'z_pose:=0.01',
        ], preflight_root / f'{label}_gazebo.log', environment)
        groups.append(process.pid)
        try:
            _wait_topics({'/scan', '/ground_truth_pose'}, environment, 60.0)
            pose = _settled_pose(environment, 30.0)
            scan = _echo_once(
                '/scan', 'sensor_msgs/msg/LaserScan', environment, 8.0)
            if scan.returncode != 0:
                raise RuntimeError(f'{label} scan capture failed')
            scan_path = preflight_root / f'{label}_scan.yaml'
            scan_path.write_text(scan.stdout, encoding='utf-8')
            captures[label] = {
                'urdf_sha256': _sha256(urdf),
                'settled_pose_xyz_rpy': pose,
                'scan_sha256': _sha256(scan_path),
                'profile': _scan_profile(scan.stdout),
            }
        except Exception as error:
            capture_error = f'{type(error).__name__}: {error}'
        finally:
            _stop(process)
            stream.close()
        if capture_error is not None:
            break
    survivors = _cleanup_identity(run_id, domain_id)
    if capture_error is not None:
        document = {
            'schema_version': 1,
            'status': 'FAIL',
            'harness_error': capture_error,
            'captures': captures,
            'input_hashes': _evaluation_input_hashes(raw_root),
            'teardown': {
                'launched_process_groups': groups,
                'identity_survivors': survivors,
                'remaining_process_groups': [
                    group for group in groups if _group_members(group)],
            },
        }
        (preflight_root / 'evidence.json').write_text(json.dumps(
            document, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        raise RuntimeError(f'scan A/B preflight failed: {capture_error}')
    source = captures['source_seed_11']
    evaluations = [captures[f'evaluation_seed_{seed}'] for seed in SEEDS]
    evaluation = evaluations[0]
    metadata_keys = (
        'frame_id', 'sample_count', 'angle_min_rad', 'angle_max_rad',
        'angle_increment_rad', 'range_min_m', 'range_max_m')
    evaluation_metadata_consistent = all(
        evaluation['profile'][key] == item['profile'][key]
        for item in evaluations for key in metadata_keys)
    run_contract = json.loads(
        (raw_root / 'contract.json').read_text(encoding='utf-8'))
    scan_contract = run_contract['simulation_support_plane_correction'][
        'scan_preflight_contract']
    full_azimuth_contract = scan_contract['full_azimuth']
    profile = evaluation['profile']
    sample_count = profile['sample_count']
    angle_increment_rad = profile['angle_increment_rad']
    transformed_directions = sorted(
        math.atan2(math.sin(math.pi + profile['angle_min_rad']
                            + index * angle_increment_rad),
                   math.cos(math.pi + profile['angle_min_rad']
                            + index * angle_increment_rad))
        for index in range(sample_count))
    circular_gaps = [
        transformed_directions[index + 1] - transformed_directions[index]
        for index in range(sample_count - 1)]
    circular_gaps.append(
        2.0 * math.pi + transformed_directions[0]
        - transformed_directions[-1])
    max_circular_gap_rad = max(circular_gaps)
    forward_error_rad = min(abs(direction)
                            for direction in transformed_directions)
    full_azimuth_valid = (
        profile['frame_id'] == 'laser_link'
        and sample_count == full_azimuth_contract['sample_count']
        and math.isclose(
            profile['angle_max_rad'],
            profile['angle_min_rad']
            + (sample_count - 1) * angle_increment_rad,
            abs_tol=1e-6)
        and math.isclose(
            sample_count * abs(angle_increment_rad), 2.0 * math.pi,
            abs_tol=1e-5)
        and len({round(direction, 12)
                 for direction in transformed_directions}) == sample_count
        and max_circular_gap_rad
        <= 1.5 * full_azimuth_contract['angle_increment_rad']
        and forward_error_rad
        <= full_azimuth_contract['angle_increment_rad'] / 2.0)
    expected_wall_beams_m = scan_contract['expected_wall_beams_m']
    wall_beam_comparisons = _wall_beam_comparison_evidence(
        evaluations, SEEDS, scan_contract)
    walls_preserved = wall_beam_comparisons['status'] == 'PASS'
    ground_return_removed = (
        source['profile']['unexpected_near_edge_ground_return_count'] > 0
        and all(item['profile'][
            'unexpected_near_edge_ground_return_count'] == 0
                for item in evaluations))
    all_beam_near_envelope_clear = all(
        item['profile']['all_beam_near_return_count'] == 0
        for item in evaluations)
    laser_height_from_model_m = sum(
        float(ET.parse(source_urdf).getroot().find(
            f"./joint[@name='{name}']/origin").attrib['xyz'].split()[2])
        for name in ('base_joint', 'laser_joint'))
    ground_intersections_m = []
    for item in evaluations:
        pose = item['settled_pose_xyz_rpy']
        tilt_rad = math.hypot(pose[3], pose[4])
        laser_height_m = (
            pose[2] + laser_height_from_model_m * math.cos(tilt_rad))
        ground_intersections_m.append(
            float('inf') if tilt_rad == 0.0
            else laser_height_m / math.tan(tilt_rad))
    orientation_valid = all(
        abs(item['settled_pose_xyz_rpy'][3])
        <= math.atan((item['settled_pose_xyz_rpy'][2]
                      + laser_height_from_model_m) / 8.0)
        and abs(item['settled_pose_xyz_rpy'][4])
        <= math.atan((item['settled_pose_xyz_rpy'][2]
                      + laser_height_from_model_m) / 8.0)
        for item in evaluations)
    ground_clear_beyond_sensor = all(
        distance_m > evaluation['profile']['range_max_m']
        for distance_m in ground_intersections_m)
    evaluation_poses = [item['settled_pose_xyz_rpy'] for item in evaluations]
    pose_spreads = [
        max(pose[index] for pose in evaluation_poses)
        - min(pose[index] for pose in evaluation_poses)
        for index in range(6)]
    settle_repeatable = (
        max(pose_spreads[:3]) <= POSE_SETTLE_POSITION_TOLERANCE_M
        and max(pose_spreads[3:]) <= POSE_SETTLE_ANGLE_TOLERANCE_RAD)
    passed = (evaluation_metadata_consistent and full_azimuth_valid
              and all_beam_near_envelope_clear
              and settle_repeatable and orientation_valid
              and ground_clear_beyond_sensor and walls_preserved
              and ground_return_removed and not survivors)
    document = {
        'schema_version': 1,
        'status': 'PASS' if passed else 'FAIL',
        'seed': 11,
        'start_pose_xyz': [-8.0, 0.0, 0.01],
        'pose_position_tolerance_m': POSE_SETTLE_POSITION_TOLERANCE_M,
        'pose_angle_tolerance_rad': POSE_SETTLE_ANGLE_TOLERANCE_RAD,
        'expected_wall_beams_m': expected_wall_beams_m,
        'wall_beam_multiple_comparison': wall_beam_comparisons,
        'evaluation_metadata_consistent': evaluation_metadata_consistent,
        'full_azimuth_valid': full_azimuth_valid,
        'max_circular_gap_rad': max_circular_gap_rad,
        'forward_direction_error_rad': forward_error_rad,
        'all_beam_near_envelope_clear': all_beam_near_envelope_clear,
        'evaluation_settle_repeatable': settle_repeatable,
        'evaluation_pose_spreads_xyz_rpy': pose_spreads,
        'evaluation_absolute_orientation_valid': orientation_valid,
        'evaluation_ground_intersections_m': ground_intersections_m,
        'ground_clear_beyond_sensor_range': ground_clear_beyond_sensor,
        'wall_beams_preserved': walls_preserved,
        'unexpected_near_edge_ground_return_removed': ground_return_removed,
        'captures': captures,
        'input_hashes': _evaluation_input_hashes(raw_root),
        'teardown': {
            'launched_process_groups': groups,
            'identity_survivors': survivors,
            'remaining_process_groups': [
                group for group in groups if _group_members(group)],
        },
    }
    (preflight_root / 'evidence.json').write_text(json.dumps(
        document, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    if not passed:
        raise RuntimeError('scan A/B preflight failed')
    return {**document, 'cache_reused': False}


def _contact_smoke(
        environment: dict[str, str], output: Path,
        post_deactivate_window_s: float,
        initial_scan: subprocess.CompletedProcess,
        contract: dict[str, Any]) -> dict[str, Any]:
    sensor_spec = contract['preloaded_obstacles']['models'][
        'contact_control']
    sensor_name = sensor_spec['name']
    topic = (
        f'/world/slam_corridor/model/{sensor_name}/link/body/'
        'sensor/contact_sensor/contact')
    sensor_topic_ready = _wait_gz_topic(topic, environment, 10.0)
    park_before = _wait_entity_pose(
        sensor_name, sensor_spec['park_pose_m'], environment)
    scan_before = initial_scan
    before_stamp_ns = _scan_stamp_ns(scan_before.stdout)
    moved = sensor_topic_ready and _set_pose(
        sensor_name, ARM_PROBE_WORLD_X_M, ARM_PROBE_WORLD_Y_M,
        ARM_PROBE_WORLD_Z_M, environment)
    overlap_state = _wait_entity_pose(sensor_name, [
        ARM_PROBE_WORLD_X_M, ARM_PROBE_WORLD_Y_M,
        ARM_PROBE_WORLD_Z_M], environment) if moved else None
    arm_link_tokens = _arm_link_tokens(ASSETS / 'jdamr_cube_nav_eval.urdf')
    echo = _wait_arm_contact(topic, environment, 12.0, arm_link_tokens)
    deactivated = _set_pose(
        sensor_name, *sensor_spec['park_pose_m'], environment)
    park_after = _wait_entity_pose(
        sensor_name, sensor_spec['park_pose_m'],
        environment) if deactivated else None
    scan_after = _wait_scan(
        environment, 10.0, newer_than_ns=before_stamp_ns)
    quiet = _echo_once(
        topic, 'ros_gz_interfaces/msg/Contacts', environment,
        post_deactivate_window_s)
    after_stamp_ns = _scan_stamp_ns(scan_after.stdout)
    collision_names = _collision_names(echo.stdout)
    arm_collision_names, unexpected_robot_collision_names = (
        _classify_robot_collisions(collision_names, arm_link_tokens))
    base_collision_names = [
        name for name in unexpected_robot_collision_names
        if name.endswith('__base_link_collision')]
    arm_contact = (
        bool(arm_collision_names) and not unexpected_robot_collision_names)
    new_scan_after_deactivate = (
        before_stamp_ns is not None and after_stamp_ns is not None
        and after_stamp_ns > before_stamp_ns)
    no_contact_after_deactivate = not _collision_names(quiet.stdout)
    identity_stable = (
        overlap_state is not None and park_after is not None
        and len({park_before['entity_id'], overlap_state['entity_id'],
                 park_after['entity_id']}) == 1)
    overlap_pose_verified = (
        overlap_state is not None
        and math.dist(overlap_state['pose_m'], [
            ARM_PROBE_WORLD_X_M, ARM_PROBE_WORLD_Y_M,
            ARM_PROBE_WORLD_Z_M]) <= 0.02)
    park_pose_verified = (
        math.dist(park_before['pose_m'], sensor_spec['park_pose_m']) <= 0.02
        and park_after is not None
        and math.dist(
            park_after['pose_m'], sensor_spec['park_pose_m']) <= 0.02)
    passed = (
        moved and echo.returncode == 0 and 'contacts:' in echo.stdout
        and sensor_name in echo.stdout and arm_contact and deactivated
        and new_scan_after_deactivate and no_contact_after_deactivate
        and identity_stable and overlap_pose_verified and park_pose_verified)
    document = {
        'schema_version': 1,
        'status': 'PASS' if passed else 'FAIL',
        'input_hashes': _evaluation_input_hashes(output.parent),
        'ros_bridge_received_nonempty_arm_contacts': arm_contact,
        'unique_arm_collision_count': len(arm_collision_names),
        'arm_collision_names': arm_collision_names,
        'base_collision_names': base_collision_names,
        'unexpected_robot_collision_names': unexpected_robot_collision_names,
        'probe_entity_deactivated': deactivated,
        'no_contact_after_deactivate': no_contact_after_deactivate,
        'new_scan_after_deactivate': new_scan_after_deactivate,
        'scan_before_deactivate_stamp_ns': before_stamp_ns,
        'scan_after_deactivate_stamp_ns': after_stamp_ns,
        'scan_before_returncode': scan_before.returncode,
        'scan_before_attempt_count': scan_before.scan_attempt_count,
        'scan_before_wait_elapsed_s': scan_before.scan_wait_elapsed_s,
        'scan_before_stderr': scan_before.stderr,
        'scan_before_sha256': hashlib.sha256(
            scan_before.stdout.encode()).hexdigest(),
        'scan_after_returncode': scan_after.returncode,
        'scan_after_attempt_count': scan_after.scan_attempt_count,
        'scan_after_wait_elapsed_s': scan_after.scan_wait_elapsed_s,
        'scan_after_stderr': scan_after.stderr,
        'scan_after_sha256': hashlib.sha256(
            scan_after.stdout.encode()).hexdigest(),
        'probe_pose_world_m': {
            'x': ARM_PROBE_WORLD_X_M, 'y': ARM_PROBE_WORLD_Y_M,
            'z': ARM_PROBE_WORLD_Z_M},
        'preloaded_entity_name': sensor_name,
        'entity_states': {
            'park_before_overlap': park_before,
            'overlap': overlap_state,
            'park_after_overlap': park_after,
        },
        'entity_identity_stable': identity_stable,
        'overlap_pose_verified': overlap_pose_verified,
        'park_pose_verified': park_pose_verified,
        'sensor_topic_ready': sensor_topic_ready,
        'set_pose_ack': moved,
        'echo_returncode': echo.returncode,
        'echo_sha256': hashlib.sha256(echo.stdout.encode()).hexdigest(),
        'post_deactivate_echo_returncode': quiet.returncode,
    }
    output.write_text(json.dumps(
        document, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return document


def _sensor_probe_smoke(
        environment: dict[str, str], raw_root: Path,
        launched: list[tuple[subprocess.Popen, Any]], seed: int
        ) -> dict[str, Any]:
    output_root = raw_root / 'sensor_probe_preflight' / f'seed_{seed}'
    output_root.mkdir(parents=True, exist_ok=True)
    results = {}
    failed_scenario = None
    for scenario in ('sub_minimum_probe', 'front_observation_probe'):
        run_contract = json.loads(
            (raw_root / 'contract.json').read_text(encoding='utf-8'))
        entity_name = run_contract['preloaded_obstacles']['models'][
            scenario]['name']
        contact_topic = (
            f'/world/slam_corridor/model/{entity_name}/link/body/'
            'sensor/contact_sensor/contact')
        bridge = _start([
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'{contact_topic}@ros_gz_interfaces/msg/Contacts'
            '[gz.msgs.Contacts',
        ], output_root / f'{scenario}_bridge.log', environment)
        launched.append(bridge)
        output = output_root / f'{scenario}.json'
        process = _start([
            'ros2', 'run', 'jdamr_cube_navigation',
            'sim_nav_obstacle_scenario', '--scenario', scenario,
            '--seed', str(seed), '--output', str(output),
            '--contract', str(raw_root / 'contract.json'),
            '--behavior-tree', str(BT), '--run-timeout-s', '60',
            '--observation-persistence-s', '0.0',
        ], output_root / f'{scenario}.log', environment)
        launched.append(process)
        returncode = process[0].wait(timeout=75.0)
        if not output.is_file():
            raise RuntimeError(f'{scenario} produced no evidence')
        document = json.loads(output.read_text(encoding='utf-8'))
        document['process_returncode'] = returncode
        output.write_text(json.dumps(
            document, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
        results[scenario] = document
        if returncode != 0 or document.get('status') != 'PASS':
            failed_scenario = scenario
            break
    summary = {
        'schema_version': 1,
        'seed': seed,
        'input_hashes': _evaluation_input_hashes(raw_root),
        'status': 'FAIL' if failed_scenario else 'PASS',
        'failed_scenario': failed_scenario,
        'results': {
            name: {
                'path': str((output_root / f'{name}.json').resolve()),
                'sha256': _sha256(output_root / f'{name}.json'),
            } for name in results},
    }
    (output_root / 'summary.json').write_text(json.dumps(
        summary, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    if failed_scenario:
        raise RuntimeError(f'{failed_scenario} failed')
    return summary


def _environment(run_id: str, domain_id: int) -> dict[str, str]:
    environment = os.environ.copy()
    environment.update({
        PROCESS_MARKER: run_id,
        'ROS_DOMAIN_ID': str(domain_id),
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'ROS2CLI_DISABLE_DAEMON': '1',
        'RMW_IMPLEMENTATION': EVALUATION_RMW_IMPLEMENTATION,
    })
    if EVALUATION_RMW_IMPLEMENTATION == 'rmw_fastrtps_cpp':
        environment['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
    else:
        environment.pop('FASTDDS_BUILTIN_TRANSPORTS', None)
    if EVALUATION_RMW_OVERLAY_PREFIX is not None:
        prefix = str(EVALUATION_RMW_OVERLAY_PREFIX)
        for name, entries in {
                'AMENT_PREFIX_PATH': [prefix],
                'LD_LIBRARY_PATH': [
                    str(EVALUATION_RMW_OVERLAY_PREFIX / 'lib'),
                    str(EVALUATION_RMW_OVERLAY_PREFIX
                        / 'lib' / 'x86_64-linux-gnu')],
                'PATH': [str(EVALUATION_RMW_OVERLAY_PREFIX / 'bin')],
                }.items():
            previous = environment.get(name, '')
            environment[name] = ':'.join(
                [*entries, *([previous] if previous else [])])
    return environment


def evaluation_rmw_evidence(environment: dict[str, str]) -> dict[str, Any]:
    """Verify and fingerprint the actual RMW selected for child processes."""
    result = subprocess.run([
        'python3', '-c',
        'import rclpy; print(rclpy.get_rmw_implementation_identifier())',
    ], env=environment, capture_output=True, text=True, timeout=10.0,
        check=False)
    actual = result.stdout.strip()
    if result.returncode != 0 or actual != EVALUATION_RMW_IMPLEMENTATION:
        requested = EVALUATION_RMW_IMPLEMENTATION
        raise RuntimeError(
            f'evaluation RMW mismatch: requested={requested} '
            f'actual={actual or result.stderr.strip()}')
    prefix_result = subprocess.run(
        ['ros2', 'pkg', 'prefix', actual], env=environment,
        capture_output=True, text=True, timeout=10.0, check=False)
    prefix = Path(prefix_result.stdout.strip())
    package_xml = prefix / 'share' / actual / 'package.xml'
    if prefix_result.returncode != 0 or not package_xml.is_file():
        raise RuntimeError('evaluation RMW package metadata unavailable')
    version = ET.parse(package_xml).getroot().findtext('version')
    library = prefix / 'lib' / f'lib{actual}.so'
    if version is None or not library.is_file():
        raise RuntimeError('evaluation RMW version or library unavailable')
    environment_fields = {
        name: environment.get(name) for name in (
            'AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PATH',
            'RMW_IMPLEMENTATION', 'ROS_AUTOMATIC_DISCOVERY_RANGE')}
    return {
        'requested_identifier': EVALUATION_RMW_IMPLEMENTATION,
        'actual_identifier': actual,
        'version': version,
        'source': ('temporary_deb_extract'
                   if EVALUATION_RMW_OVERLAY_PREFIX is not None
                   else 'system_ros_installation'),
        'prefix': str(prefix),
        'library_sha256': _sha256(library),
        'environment_sha256': hashlib.sha256(json.dumps(
            environment_fields, sort_keys=True).encode()).hexdigest(),
        'environment_fields': environment_fields,
    }


def _add_teardown(
        evidence_path: Path, groups: list[int],
        survivors: list[dict[str, Any]], contact_smoke: dict,
        cache_provenance: dict[str, Any]) -> None:
    document = json.loads(evidence_path.read_text(encoding='utf-8'))
    document['teardown'] = {
        'launched_process_groups': groups,
        'remaining_process_groups': [
            group for group in groups if _group_members(group)],
        'identity_survivors': survivors,
    }
    document['contact_smoke_status'] = contact_smoke['status']
    document['evaluation_rmw'] = document['contract']['evaluation_rmw']
    document['cache_provenance'] = cache_provenance
    document['artifacts'] = {
        'contract_sha256': _sha256(
            evidence_path.parent.parent / 'contract.json'),
        'asset_manifest_sha256': _sha256(ASSETS / 'asset_manifest.json'),
    }
    evidence_path.write_text(json.dumps(
        document, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')


def _run_one(item: dict[str, Any], raw_root: Path, domain_id: int,
             contact_smoke_required: bool,
             scan_preflight_reused: bool) -> dict[str, Any]:
    run_id = item['run_id']
    run_dir = raw_root / run_id
    archive_existing_attempt(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    environment = _environment(run_id, domain_id)
    launched: list[tuple[subprocess.Popen, Any]] = []
    contact_smoke_path = raw_root / 'contact_smoke.json'
    contact_smoke = (
        json.loads(contact_smoke_path.read_text(encoding='utf-8'))
        if contact_smoke_path.is_file() else {'status': 'NOT_RUN'})
    evidence_path = run_dir / 'evidence.json'
    sensor_probe_reused = False
    try:
        gazebo = _start([
            'ros2', 'launch', 'jdamr_cube_gazebo', 'gazebo.launch.py',
            f'world:={ASSETS / "slam_corridor_contact.world"}',
            f'urdf_file:={ASSETS / "jdamr_cube_nav_eval.urdf"}',
            'gui:=false', 'enable_image_bridges:=false',
            f'seed:={item["seed"]}', 'x_pose:=-8.0', 'y_pose:=0.0',
            'z_pose:=0.01',
        ], run_dir / 'gazebo.log', environment)
        launched.append(gazebo)
        _wait_topics(
            {'/scan', '/odom', '/ground_truth_pose'}, environment, 60.0)
        scan_ready = _wait_scan(environment, 60.0)
        if _scan_stamp_ns(scan_ready.stdout) is None:
            raise RuntimeError('scan data readiness failed')

        if contact_smoke_required:
            run_contract = json.loads(
                (raw_root / 'contract.json').read_text(encoding='utf-8'))
            sensor_name = run_contract['preloaded_obstacles']['models'][
                'contact_control']['name']
            smoke_topic = (
                f'/world/slam_corridor/model/{sensor_name}/'
                'link/body/sensor/contact_sensor/contact')
            smoke_bridge = _start([
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'{smoke_topic}@ros_gz_interfaces/msg/Contacts'
                '[gz.msgs.Contacts',
            ], run_dir / 'contact_smoke_bridge.log', environment)
            launched.append(smoke_bridge)
            contact_smoke = _contact_smoke(
                environment, contact_smoke_path,
                run_contract['global_update_period_s'], scan_ready,
                run_contract)
            if contact_smoke['status'] != 'PASS':
                raise RuntimeError('contact smoke failed')

        if SCENARIOS[item['scenario']]['obstacle'] is not None:
            run_contract = json.loads(
                (raw_root / 'contract.json').read_text(encoding='utf-8'))
            role = ('route' if item['scenario'] in {
                'detour', 'event_driven_removal'} else item['scenario'])
            entity_name = run_contract['preloaded_obstacles']['models'][
                role]['name']
            contact_topic = (
                f'/world/slam_corridor/model/{entity_name}/link/body/'
                'sensor/contact_sensor/contact')
            bridge = _start([
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'{contact_topic}@ros_gz_interfaces/msg/Contacts'
                '[gz.msgs.Contacts',
            ], run_dir / 'contact_bridge.log', environment)
            launched.append(bridge)

        navigation = _start([
            'ros2', 'launch', 'jdamr_cube_navigation',
            'navigation.launch.py',
            f'map:={ASSETS / "slam_corridor_eval.yaml"}',
            f'params_file:={raw_root / "nav2_obstacle_eval.params.yaml"}',
            'use_keepout:=false', 'use_sim_time:=true',
            f'use_composition:={str(NAV2_USE_COMPOSITION)}',
        ], run_dir / 'navigation.log', environment)
        launched.append(navigation)
        _wait_topics({
            '/global_costmap/costmap_raw', '/local_costmap/costmap_raw',
            '/cmd_vel', '/plan'}, environment, 60.0,
            run_dir / 'navigation.log')
        _wait_lifecycle_active((
            '/amcl', '/controller_server', '/planner_server',
            '/bt_navigator', '/velocity_smoother', '/collision_monitor'),
            environment, 60.0)
        _wait_tf_available(
            'map', 'base_footprint', environment, 60.0,
            run_dir / 'tf_readiness.log')

        probe_summary_path = (
            raw_root / 'sensor_probe_preflight'
            / f'seed_{item["seed"]}' / 'summary.json')
        probe_summary = (
            json.loads(probe_summary_path.read_text(encoding='utf-8'))
            if probe_summary_path.is_file() else {'status': 'NOT_RUN'})
        sensor_probe_reused = _preflight_cache_matches(
            probe_summary, _evaluation_input_hashes(raw_root), item['seed'])
        if not sensor_probe_reused:
            if probe_summary_path.parent.is_dir():
                archive_existing_attempt(probe_summary_path.parent)
            _sensor_probe_smoke(
                environment, raw_root, launched, item['seed'])

        scenario = _start([
            'ros2', 'run', 'jdamr_cube_navigation',
            'sim_nav_obstacle_scenario',
            '--scenario', item['scenario'], '--seed', str(item['seed']),
            '--output', str(evidence_path),
            '--contract', str(raw_root / 'contract.json'),
            '--behavior-tree', str(BT), '--run-timeout-s', '180',
            '--observation-persistence-s', '0.0',
        ], run_dir / 'scenario.log', environment)
        launched.append(scenario)
        scenario_returncode = scenario[0].wait(timeout=210.0)
        if scenario_returncode != 0 and evidence_path.is_file():
            document = json.loads(evidence_path.read_text(encoding='utf-8'))
            document['harness_error'] = (
                f'scenario_returncode:{scenario_returncode}')
            evidence_path.write_text(json.dumps(
                document, ensure_ascii=False, indent=2,
                sort_keys=True) + '\n', encoding='utf-8')
    except Exception as error:
        if not evidence_path.is_file():
            failure = {
                'schema_version': 1, 'run_id': run_id,
                'scenario': item['scenario'], 'seed': item['seed'],
                'contract': json.loads(
                    (raw_root / 'contract.json').read_text()),
                'events': [], 'action_terminal': 'unknown',
                'runner_cancelled': False, 'runner_timed_out': False,
                'contact_count': -1, 'final_cmd_vel_zero': False,
                'final_zero_hold_s': 0.0,
                'harness_error': f'{type(error).__name__}: {error}',
            }
            if isinstance(error, InfrastructureInvalid):
                failure['infrastructure_classification'] = str(error)
                failure['scenario_started'] = False
            evidence_path.write_text(json.dumps(
                failure, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    finally:
        groups = [process.pid for process, _ in launched]
        for process, _ in reversed(launched):
            _stop(process)
        for _, stream in launched:
            stream.close()
        survivors = _cleanup_identity(run_id, domain_id)
        _add_teardown(evidence_path, groups, survivors, contact_smoke, {
            'scan_ab_preflight_reused': scan_preflight_reused,
            'contact_smoke_reused': not contact_smoke_required,
            'sensor_probe_reused': sensor_probe_reused,
            'nav2_process_topology': 'component_container_isolated',
            'input_hashes': _evaluation_input_hashes(raw_root),
        })
    return {'run_id': run_id, 'evidence': str(evidence_path)}


def _copy_durable(raw_root: Path, durable_root: Path) -> None:
    durable_root.mkdir(parents=True, exist_ok=True)
    for filename in (
            'contract.json', 'nav2_obstacle_eval.params.yaml',
            'contact_smoke.json', 'aggregate.json'):
        source = raw_root / filename
        if source.is_file():
            shutil.copy2(source, durable_root / filename)
    preflight_root = raw_root / 'scan_ab_preflight'
    if preflight_root.is_dir():
        shutil.copytree(
            preflight_root, durable_root / 'scan_ab_preflight',
            dirs_exist_ok=True)
    sensor_probe_root = raw_root / 'sensor_probe_preflight'
    if sensor_probe_root.is_dir():
        shutil.copytree(
            sensor_probe_root, durable_root / 'sensor_probe_preflight',
            dirs_exist_ok=True)
    startup_topology_root = raw_root / 'startup_topology_ab'
    if startup_topology_root.is_dir():
        shutil.copytree(
            startup_topology_root, durable_root / 'startup_topology_ab',
            dirs_exist_ok=True)
    evidence_root = durable_root / 'evidence'
    evidence_root.mkdir(exist_ok=True)
    for evidence in raw_root.glob('*/evidence.json'):
        document = json.loads(evidence.read_text(encoding='utf-8'))
        plans_before = document.pop('plans_before_mark', [])
        plans_after = document.pop('plans_after_mark', [])
        document.pop('contract', None)
        document['raw_evidence'] = {
            'sha256': _sha256(evidence),
            'size_bytes': evidence.stat().st_size,
            'plans_before_mark_count': len(plans_before),
            'plans_after_mark_count': len(plans_after),
            'plans_before_mark_sha256': hashlib.sha256(
                json.dumps(plans_before, separators=(',', ':')).encode()
            ).hexdigest(),
            'plans_after_mark_sha256': hashlib.sha256(
                json.dumps(plans_after, separators=(',', ':')).encode()
            ).hexdigest(),
        }
        (evidence_root / f'{evidence.parent.name}.json').write_text(
            json.dumps(document, ensure_ascii=False, indent=2,
                       sort_keys=True) + '\n', encoding='utf-8')
    shutil.copytree(ASSETS, durable_root / 'assets', dirs_exist_ok=True)
    usage = shutil.disk_usage(durable_root)
    if usage.free < 1024 ** 3:
        raise RuntimeError('durable copy left less than 1 GiB free in home')


def main() -> int:
    """Run selected cases sequentially and evaluate the complete result set."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--raw-root', type=Path, required=True)
    parser.add_argument('--durable-root', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, default=193)
    parser.add_argument('--scenario', choices=tuple(SCENARIOS))
    parser.add_argument('--seed', type=int)
    parser.add_argument(
        '--rmw-implementation', default='rmw_fastrtps_cpp',
        choices=('rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp'))
    parser.add_argument('--rmw-overlay-prefix', type=Path)
    args = parser.parse_args()
    if args.domain_id == 12:
        parser.error('physical robot domain 12 is forbidden')
    configure_evaluation_rmw(
        args.rmw_implementation, args.rmw_overlay_prefix)
    args.raw_root.mkdir(parents=True, exist_ok=True)
    prepare(args.raw_root)
    contract_path = args.raw_root / 'contract.json'
    contract = json.loads(contract_path.read_text(encoding='utf-8'))
    contract['evaluation_rmw'] = evaluation_rmw_evidence(
        _environment('rmw_contract_probe', args.domain_id))
    contract_path.write_text(json.dumps(
        contract, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    scan_preflight = _scan_ab_preflight(args.raw_root, args.domain_id)
    selected = [item for item in scenario_matrix()
                if (args.scenario is None
                    or item['scenario'] == args.scenario)
                and (args.seed is None or item['seed'] == args.seed)]
    for index, item in enumerate(selected):
        contact_smoke_path = args.raw_root / 'contact_smoke.json'
        contact_smoke_required = (
            index == 0 and not _contact_smoke_cache_matches(
                contact_smoke_path,
                _evaluation_input_hashes(args.raw_root)))
        contact_attempt = 0
        while True:
            result = _run_one(
                item, args.raw_root, args.domain_id,
                contact_smoke_required=contact_smoke_required,
                scan_preflight_reused=scan_preflight['cache_reused'])
            print(json.dumps(result, sort_keys=True), flush=True)
            if not contact_smoke_required or _contact_smoke_cache_matches(
                    contact_smoke_path,
                    _evaluation_input_hashes(args.raw_root)):
                break
            contact_attempt += 1
            _archive_contact_smoke(args.raw_root)
            if contact_attempt >= CONTACT_SMOKE_MAX_ATTEMPTS:
                summary = aggregate(list(
                    args.raw_root.glob('*/evidence.json')))
                summary['matrix_aborted_before_scenarios'] = True
                summary['contact_smoke_attempt_count'] = contact_attempt
                (args.raw_root / 'aggregate.json').write_text(json.dumps(
                    summary, ensure_ascii=False, indent=2,
                    sort_keys=True) + '\n', encoding='utf-8')
                _copy_durable(args.raw_root, args.durable_root)
                return 1
    summary = aggregate(list(args.raw_root.glob('*/evidence.json')))
    (args.raw_root / 'aggregate.json').write_text(json.dumps(
        summary, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    _copy_durable(args.raw_root, args.durable_root)
    return 0 if summary['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
