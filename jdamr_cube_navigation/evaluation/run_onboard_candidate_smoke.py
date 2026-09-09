#!/usr/bin/env python3
"""Run a bounded simulation-only smoke of the real onboard Nav2 candidate."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path  # noqa: I100
from typing import Any

from evaluate_same_goal_bag import evaluate as evaluate_same_goal_bag

from jdamr_cube_navigation.keepout_mask import (
    _map_metadata, build_mask, KEEPOUT_PIXEL, validate_mask)
from jdamr_cube_navigation.mobile_manipulator_protection import (
    load_mobile_manipulator_protection,
)
from jdamr_cube_navigation.sim_collision_monitor_scenario import (
    rectangle_clearance,
)

from navigation_mcap_reader import read_navigation_messages

from onboard_stop_contract import (
    build_contract as build_stop_contract,
    scenario_passed as stop_scenario_passed,
)

from portfolio_capture_world import (
    build_capture_urdf,
    build_capture_world,
    CAMERA_RATE_HZ,
    CAMERA_TOPIC as SIM_CAMERA_TOPIC,
    PEDESTRIAN_DOORWAY_EDGE_Y_M,
)
from prepare_sim_nav_obstacle_run import prepare

from run_sim_nav_obstacle_eval import (  # noqa: I101
    _cleanup_identity, _environment, _group_members, _set_pose, _sha256,
    _start, _stop, _wait_entity_pose, _wait_lifecycle_active,
    _wait_tf_available, _wait_topics, ASSETS, BT,
)

import yaml


ROOT = Path(__file__).resolve().parents[2]
PRODUCTION_PARAMS = ROOT / 'jdamr_cube_navigation/config/nav2_params.yaml'
PROCESS_MARKER = 'JDAMR_NAV_EVAL_RUN_ID'
DOMAIN_IDS = {186, 187}
SEED = 11
PEDESTRIAN_EDGE_Y_M = 1.0
VIDEO_EMPTY_SCENE_OBSERVATION_S = 2.0
KEEPOUT_DEMO_SAFETY_MARGIN_M = 0.35
KEEPOUT_FILTER_INFLATION_RADIUS_M = 0.45
KEEPOUT_FILTER_COST_SCALING_FACTOR_PER_M = 3.0
KEEPOUT_DEMO_POLYGON_M = (
    (-4.5, -1.1), (-3.5, -1.1), (-3.5, 0.1), (-4.5, 0.1))
CASES = (
    'detour', 'event_driven_removal', 'sudden_stop_resume',
    'detour_sudden_stop_resume')
DEFAULT_CASES = ('detour', 'event_driven_removal')
ALLOWED_PARAM_DELTAS = {
    ('amcl', 'ros__parameters', 'initial_pose', 'x'),
    ('local_costmap', 'local_costmap', 'ros__parameters',
     'always_send_full_costmap'),
    ('global_costmap', 'global_costmap', 'ros__parameters',
     'always_send_full_costmap'),
}
KEEPOUT_DEMO_PARAM_DELTAS = {
    ('local_costmap', 'local_costmap', 'ros__parameters', 'filters'),
    ('local_costmap', 'local_costmap', 'ros__parameters',
     'keepout_inflation'),
    ('global_costmap', 'global_costmap', 'ros__parameters', 'filters'),
    ('global_costmap', 'global_costmap', 'ros__parameters',
     'keepout_inflation'),
}
CAP_BYTES = 64 * 1024 * 1024
BAG_LIVE_CAP_BYTES = 56 * 1024 * 1024
RECORDED_TOPICS = (
    '/scan', '/cmd_vel', '/collision_monitor_state',
    '/navigate_to_pose/_action/status', '/odom', '/ground_truth_pose',
    '/plan', '/amcl_pose', '/tf', '/tf_static')
NAV_SCENARIO_SOURCE = (
    ROOT / 'jdamr_cube_navigation/jdamr_cube_navigation/'
    'sim_nav_obstacle_scenario.py')
STOP_SCENARIO_SOURCE = (
    ROOT / 'jdamr_cube_navigation/jdamr_cube_navigation/'
    'sim_collision_monitor_scenario.py')
SIM_CAMERA_RECORDER = (
    ROOT / 'jdamr_cube_navigation/evaluation/record_simulator_camera.py')
CAPTURE_WORLD_BUILDER = (
    ROOT / 'jdamr_cube_navigation/evaluation/portfolio_capture_world.py')
MOBILE_MANIPULATOR_PROTECTION = (
    ROOT / 'jdamr_cube_navigation/config/mobile_manipulator_protection.yaml')


def _leaf_differences(
        left: Any, right: Any, prefix=()) -> set[tuple[str, ...]]:
    """Return exact leaf paths whose values or presence differ."""
    if isinstance(left, dict) and isinstance(right, dict):
        differences = set()
        for key in left.keys() | right.keys():
            if key not in left or key not in right:
                differences.add((*prefix, str(key)))
            else:
                differences |= _leaf_differences(
                    left[key], right[key], (*prefix, str(key)))
        return differences
    return set() if left == right else {prefix}


def prepare_candidate_assets(
        output_root: Path, keepout_demo: bool = False) -> dict[str, Any]:
    """Create production-faithful params and a nonempty route-away mask."""
    assets = output_root / 'assets'
    generated = prepare(assets)
    production = yaml.safe_load(
        PRODUCTION_PARAMS.read_text(encoding='utf-8'))
    candidate = yaml.safe_load(PRODUCTION_PARAMS.read_text(encoding='utf-8'))
    candidate['amcl']['ros__parameters']['initial_pose'].update({
        'x': -8.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0})
    for name in ('local_costmap', 'global_costmap'):
        costmap_params = candidate[name][name]['ros__parameters']
        costmap_params['always_send_full_costmap'] = True
        if keepout_demo:
            costmap_params['filters'] = [
                'keepout_filter', 'keepout_inflation']
            costmap_params['keepout_inflation'] = {
                'plugin': 'nav2_costmap_2d::InflationLayer',
                'inflation_radius': KEEPOUT_FILTER_INFLATION_RADIUS_M,
                'cost_scaling_factor': (
                    KEEPOUT_FILTER_COST_SCALING_FACTOR_PER_M),
            }
    differences = _leaf_differences(production, candidate)
    allowed_differences = ALLOWED_PARAM_DELTAS | (
        KEEPOUT_DEMO_PARAM_DELTAS if keepout_demo else set())
    if differences != allowed_differences:
        raise RuntimeError(f'candidate parameter delta drift: {differences}')
    generated['params'].write_text(
        yaml.safe_dump(candidate, sort_keys=False), encoding='utf-8')

    contract = json.loads(generated['contract'].read_text(encoding='utf-8'))
    contract['observation_persistence_s'] = 0.0
    contract['candidate_params'] = {
        'path': str(generated['params'].resolve()),
        'sha256': _sha256(generated['params']),
        'delta_paths': sorted('.'.join(path) for path in differences),
        'production_scan_overrides_added': [],
    }
    if keepout_demo:
        contract['candidate_params']['keepout_filter_chain'] = {
            'costmaps': ['local_costmap', 'global_costmap'],
            'filters': ['keepout_filter', 'keepout_inflation'],
            'inflation_radius_m': KEEPOUT_FILTER_INFLATION_RADIUS_M,
            'cost_scaling_factor_per_m': (
                KEEPOUT_FILTER_COST_SCALING_FACTOR_PER_M),
        }
    contract['claim_scope'] = (
        'SIM_INTEGRATION only: isolated Gazebo smoke of the real onboard '
        'obstacle_candidate launch; no physical-robot or safety claim.')
    generated['contract'].write_text(json.dumps(
        contract, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')

    direct_contract = build_stop_contract(PRODUCTION_PARAMS)
    probe = contract['preloaded_obstacles']['models'][
        'front_observation_probe']
    expected_dimensions_m = (
        direct_contract['sudden_obstacle']['length_m'],
        direct_contract['sudden_obstacle']['width_m'])
    if not all(math.isclose(actual, expected, abs_tol=1e-12)
               for actual, expected in zip(
                   (probe['length_m'], probe['width_m']),
                   expected_dimensions_m)):
        raise RuntimeError('direct-scan obstacle dimensions drifted')
    if probe['park_pose_m'] != direct_contract[
            'sudden_obstacle']['removal_pose_m']:
        raise RuntimeError('direct-scan obstacle parking pose drifted')
    direct_contract['preloaded_obstacle'] = {
        'role': probe['role'], 'name': probe['name'],
        'length_m': probe['length_m'], 'width_m': probe['width_m'],
        'park_pose_m': probe['park_pose_m'],
    }
    direct_contract_path = assets / 'onboard_stop_contract.json'
    protection = load_mobile_manipulator_protection(
        MOBILE_MANIPULATOR_PROTECTION)
    overrides = protection['collision_monitor_overrides']
    if (overrides['StopZone.points'] != direct_contract['stop_zone']['points']
            or overrides['SlowdownZone.points']
            != direct_contract['slowdown_zone']['points']):
        raise RuntimeError('runtime protective zones drifted from derivation')
    if (protection['travel_pose']['joints_rad']
            != direct_contract['travel_pose_envelope']['joint_positions_rad']
            or protection['travel_pose']['position_tolerance_rad']
            != direct_contract['travel_pose_gate']['position_tolerance_rad']
            or protection['travel_pose']['source_topic']
            != direct_contract['travel_pose_gate']['source_topic']):
        raise RuntimeError('runtime travel pose drifted from derivation')
    direct_contract['candidate_params'] = {
        'path': str(generated['params'].resolve()),
        'sha256': _sha256(generated['params']),
        'delta_paths': sorted('.'.join(path) for path in differences),
        'production_scan_overrides_added': [],
    }
    if keepout_demo:
        direct_contract['candidate_params']['keepout_filter_chain'] = (
            contract['candidate_params']['keepout_filter_chain'])
    direct_contract['runtime_protection_config'] = _source_identity(
        MOBILE_MANIPULATOR_PROTECTION)
    direct_contract_path.write_text(json.dumps(
        direct_contract, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')

    if keepout_demo:
        zone_id = 'sim_portfolio_corridor_keepout'
        polygon = [list(point) for point in KEEPOUT_DEMO_POLYGON_M]
    else:
        zone_id = 'sim_route_away_northeast_corner'
        polygon = [[9.7, 1.55], [10.25, 1.55],
                   [10.25, 1.85], [9.7, 1.85]]
    zone_config = {
        'schema_version': 1,
        'map_yaml': str((ASSETS / 'slam_corridor_eval.yaml').resolve()),
        'safety_margin_m': KEEPOUT_DEMO_SAFETY_MARGIN_M,
        'zones': [{
            'id': zone_id,
            'enabled': True,
            'polygon': polygon,
        }],
        'connectivity_checks': [{
            'id': 'candidate_route', 'start': [-8.0, 0.0],
            'goal': [6.0, 0.0], 'clearance_m': 0.0,
        }],
    }
    zones = assets / 'sim_keepout_zones.yaml'
    zones.write_text(yaml.safe_dump(zone_config, sort_keys=False),
                     encoding='utf-8')
    mask_report = build_mask(zones, assets / 'sim_keepout_mask')
    validate_mask(
        assets / 'sim_keepout_mask.yaml', ASSETS / 'slam_corridor_eval.yaml')
    keepout_demo_spec = None
    if keepout_demo:
        x_values_m = [point[0] for point in polygon]
        y_values_m = [point[1] for point in polygon]
        keepout_demo_spec = {
            'zone_id': zone_id,
            'polygon_m': polygon,
            'safety_margin_m': KEEPOUT_DEMO_SAFETY_MARGIN_M,
            'expanded_bounds_m': {
                'min_x_m': min(x_values_m) - KEEPOUT_DEMO_SAFETY_MARGIN_M,
                'max_x_m': max(x_values_m) + KEEPOUT_DEMO_SAFETY_MARGIN_M,
                'min_y_m': min(y_values_m) - KEEPOUT_DEMO_SAFETY_MARGIN_M,
                'max_y_m': max(y_values_m) + KEEPOUT_DEMO_SAFETY_MARGIN_M,
            },
        }
    return {
        'params': generated['params'], 'contract': generated['contract'],
        'stop_contract': direct_contract_path,
        'mask': assets / 'sim_keepout_mask.yaml',
        'mask_zones': zones, 'mask_report': mask_report,
        'keepout_demo': keepout_demo_spec, 'param_deltas': differences,
    }


def _run(command: list[str], environment: dict[str, str], timeout_s=10.0):
    return subprocess.run(command, env=environment, capture_output=True,
                          text=True, timeout=timeout_s, check=False)


def _parameter(node: str, name: str, environment: dict[str, str]) -> str:
    """Read directly and retry one transient parameter discovery timeout."""
    for attempt in range(2):
        result = _run(
            ['ros2', 'param', 'get', '--no-daemon', '--timeout', '5', node, name],
            environment, timeout_s=15.0)
        if result.returncode == 0:
            return result.stdout.strip()
        message = result.stderr.strip()
        transient = ('timed out waiting for parameter services' in message
                     or message.endswith('Node not found'))
        if attempt or not transient:
            break
    raise RuntimeError(f'parameter unavailable: {node}.{name}: {message}')


def _json_string_parameter(output: str) -> Any:
    """Decode one ROS string parameter whose payload is JSON."""
    prefix = 'String value is:'
    if not output.startswith(prefix):
        raise ValueError(f'expected ROS string parameter, got: {output}')
    return json.loads(output[len(prefix):].strip())


def _travel_pose_sample(
        output: str, expected_positions: dict[str, float],
        tolerance_rad: float,
) -> dict[str, Any]:
    """Validate one JointState YAML sample against the contracted stow pose."""
    start = output.find('header:')
    if start < 0:
        start = output.find('name:')
    if start < 0:
        raise ValueError('JointState document start not found')
    end = output.find('\n---', start)
    document = yaml.safe_load(output[start:end if end >= 0 else None])
    if not isinstance(document, dict):
        raise ValueError('expected one JointState document')
    names = document.get('name')
    positions = document.get('position')
    if (not isinstance(names, list) or not isinstance(positions, list)
            or len(names) != len(positions)):
        raise ValueError('invalid JointState name/position arrays')
    measured = dict(zip(names, positions))
    missing = sorted(set(expected_positions) - set(measured))
    errors = {
        name: abs(float(measured[name]) - expected)
        for name, expected in expected_positions.items() if name in measured
    }
    if any(not math.isfinite(error) for error in errors.values()):
        raise ValueError('non-finite JointState position')
    return {
        'status': ('PASS' if not missing and errors
                   and max(errors.values()) <= tolerance_rad else 'FAIL'),
        'source_topic': '/joint_states',
        'expected_positions_rad': expected_positions,
        'measured_positions_rad': {
            name: float(measured[name]) for name in expected_positions
            if name in measured},
        'absolute_errors_rad': errors,
        'maximum_error_rad': max(errors.values()) if errors else None,
        'tolerance_rad': tolerance_rad,
        'missing_joints': missing,
    }


def _read_travel_pose(
        environment: dict[str, str], contract: dict[str, Any],
        timeout_s: float = 15.0,
) -> dict[str, Any]:
    """Wait for a complete arm JointState sample, then validate its pose."""
    deadline_s = time.monotonic() + timeout_s
    last_sample = None
    last_error = ''
    while time.monotonic() < deadline_s:
        remaining_s = max(0.1, deadline_s - time.monotonic())
        try:
            result = _run(
                ['ros2', 'topic', 'echo', '--no-lost-messages', '--once',
                 '/joint_states'],
                environment, timeout_s=min(3.0, remaining_s))
        except subprocess.TimeoutExpired as exc:
            last_error = str(exc)
            continue
        if result.returncode != 0:
            last_error = result.stderr.strip()
            continue
        try:
            last_sample = _travel_pose_sample(
                result.stdout,
                contract['travel_pose_envelope']['joint_positions_rad'],
                float(contract['travel_pose_gate']['position_tolerance_rad']))
        except (TypeError, ValueError, yaml.YAMLError) as exc:
            last_error = str(exc)
            continue
        if not last_sample['missing_joints']:
            return last_sample
    if last_sample is not None:
        return last_sample
    raise RuntimeError(f'JointState sample unavailable: {last_error}')


def _lifecycle(node: str, environment: dict[str, str]) -> str:
    result = _run(['ros2', 'lifecycle', 'get', node], environment)
    return result.stdout.strip() if result.returncode == 0 else 'unavailable'


def _wait_log_markers(path: Path, markers: tuple[str, ...],
                      timeout_s: float) -> None:
    """Wait for required in-process startup facts without DDS CLI delay."""
    deadline_s = time.monotonic() + timeout_s
    missing = list(markers)
    while time.monotonic() < deadline_s:
        text = path.read_text(errors='replace') if path.is_file() else ''
        missing = [marker for marker in markers if marker not in text]
        if not missing:
            return
        time.sleep(0.1)
    raise RuntimeError(f'launch log readiness timeout: {missing}')


def _write_recording_qos(path: Path) -> None:
    """Write compatible QoS while preserving the hidden status snapshot."""
    profile = {
        topic: {
            'history': 'keep_last', 'depth': 10,
            'reliability': 'best_effort', 'durability': 'volatile',
        } for topic in RECORDED_TOPICS
    }
    profile['/navigate_to_pose/_action/status'].update({
        'depth': 1, 'reliability': 'reliable',
        'durability': 'transient_local'})
    profile['/tf_static'].update({
        'depth': 1, 'durability': 'transient_local'})
    path.write_text(yaml.safe_dump(profile, sort_keys=True), encoding='utf-8')


def _start_compact_recorder(case_dir: Path, environment: dict[str, str]):
    """Start one wall-log-time MCAP recorder for compact runtime evidence."""
    qos_path = case_dir / 'recording_qos.yaml'
    _write_recording_qos(qos_path)
    bag_dir = case_dir / 'bag'
    command = [
        'ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_dir),
        '--storage-preset-profile', 'zstd_fast',
        '--include-hidden-topics', '--qos-profile-overrides-path',
        str(qos_path), '--topics', *RECORDED_TOPICS,
    ]
    if '--use-sim-time' in command:
        raise RuntimeError('recorder log time must remain wall time')
    return _start(command, case_dir / 'recorder.log', environment), bag_dir


def _wait_bag_ready(process: subprocess.Popen, bag_dir: Path,
                    timeout_s: float = 10.0) -> None:
    """Wait until the single MCAP writer creates its data file."""
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if process.poll() is not None:
            raise RuntimeError('compact recorder exited before ready')
        if any(bag_dir.glob('*.mcap')):
            return
        time.sleep(0.1)
    raise RuntimeError('compact recorder readiness timeout')


def _wait_file_ready(process: subprocess.Popen, path: Path,
                     timeout_s: float = 20.0) -> None:
    """Wait for a sidecar to prove that its first source frame arrived."""
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if process.poll() is not None:
            raise RuntimeError('simulator camera recorder exited before ready')
        if path.is_file() and path.stat().st_size > 0:
            return
        time.sleep(0.1)
    raise RuntimeError('simulator camera first-frame timeout')


def _tree_size(root: Path) -> int:
    return sum(path.stat().st_size for path in root.rglob('*')
               if path.is_file())


def _one_finalized_mcap(bag_dir: Path) -> Path:
    matches = sorted(bag_dir.glob('*.mcap'))
    metadata = bag_dir / 'metadata.yaml'
    if len(matches) != 1 or not metadata.is_file():
        raise RuntimeError('compact recorder did not finalize one MCAP')
    mcap = matches[0]
    with mcap.open('rb') as stream:
        if stream.read(8) != b'\x89MCAP0\r\n':
            raise RuntimeError('invalid MCAP header')
        stream.seek(-8, 2)
        if stream.read(8) != b'\x89MCAP0\r\n':
            raise RuntimeError('invalid MCAP footer')
    if _tree_size(bag_dir) > CAP_BYTES:
        raise RuntimeError('finalized compact bag exceeds 64 MiB')
    return mcap


def _status_summary(mcap: Path) -> dict[str, Any]:
    uuids, statuses, message_count = [], [], 0
    for message in read_navigation_messages(
            mcap, topics=['/navigate_to_pose/_action/status']):
        message_count += 1
        for status in message.ros_msg.status_list:
            values = bytes(status.goal_info.goal_id.uuid)
            uuids.append(values.hex())
            statuses.append(int(status.status))
    return {
        'message_count': message_count,
        'goal_uuids': sorted(set(uuids)),
        'statuses': statuses,
        'final_status': statuses[-1] if statuses else None,
        'source_mcap_sha256': _sha256(mcap),
    }


def _source_identity(path: Path) -> dict[str, Any]:
    return {'path': str(path.resolve()), 'sha256': _sha256(path)}


def _cap_output(root: Path) -> int:
    """Enforce the evidence cap without deleting or rewriting raw data."""
    total = sum(
        path.stat().st_size for path in root.rglob('*') if path.is_file())
    if total > CAP_BYTES:
        raise RuntimeError(f'evidence exceeds 64 MiB cap: {total}')
    return total


def _pose_yaw(message: Any) -> float:
    orientation = message.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z
               + orientation.x * orientation.y),
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z))


def _detour_evidence(
        mcap: Path, route_spec: dict[str, Any],
        stop_contract: dict[str, Any]) -> dict[str, Any]:
    """Measure the physical route around the persistent centerline box."""
    center_x_m, center_y_m, _ = route_spec['active_pose_m']
    dimensions_m = [route_spec['length_m'], route_spec['width_m']]
    inputs = stop_contract['stop_zone']['inputs']
    footprint = {
        'front_m': inputs['footprint_front_m'],
        'rear_m': inputs['footprint_rear_m'],
        'half_width_m': inputs['footprint_half_width_m'],
    }
    approach_margin_m = 1.0
    window_min_x_m = center_x_m - route_spec['length_m'] / 2.0 - approach_margin_m
    window_max_x_m = center_x_m + route_spec['length_m'] / 2.0 + approach_margin_m
    samples = []
    for item in read_navigation_messages(mcap, topics=['/ground_truth_pose']):
        message = item.ros_msg
        x_m = float(message.pose.position.x)
        y_m = float(message.pose.position.y)
        if not window_min_x_m <= x_m <= window_max_x_m:
            continue
        stamp = message.header.stamp
        samples.append({
            'ros_ns': int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
            'pose_xy_yaw': [x_m, y_m, _pose_yaw(message)],
        })
    required_center_offset_m = (
        route_spec['width_m'] / 2.0 + footprint['half_width_m'])
    if not samples:
        return {
            'status': 'FAIL', 'reason': 'no_ground_truth_near_route_obstacle',
            'straight_centerline_blocked': True,
            'required_center_offset_m': required_center_offset_m,
        }
    for sample in samples:
        sample['clearance_m'] = rectangle_clearance(
            sample['pose_xy_yaw'], footprint,
            (center_x_m, center_y_m), dimensions_m)
    peak = max(samples, key=lambda item: abs(item['pose_xy_yaw'][1]))
    minimum = min(samples, key=lambda item: item['clearance_m'])
    maximum_abs_lateral_offset_m = abs(peak['pose_xy_yaw'][1])
    passed = (
        maximum_abs_lateral_offset_m >= required_center_offset_m
        and minimum['clearance_m'] > 0.0)
    return {
        'status': 'PASS' if passed else 'FAIL',
        'straight_centerline_blocked': True,
        'obstacle': {
            'name': route_spec['name'],
            'center_xy_m': [center_x_m, center_y_m],
            'dimensions_m': dimensions_m,
        },
        'observation_window_x_m': [window_min_x_m, window_max_x_m],
        'sample_count': len(samples),
        'required_center_offset_m': required_center_offset_m,
        'maximum_abs_lateral_offset_m': maximum_abs_lateral_offset_m,
        'detour_peak_ros_ns': peak['ros_ns'],
        'detour_peak_pose_xy_yaw': peak['pose_xy_yaw'],
        'minimum_clearance_m': minimum['clearance_m'],
        'minimum_clearance_pose_xy_yaw': minimum['pose_xy_yaw'],
    }


def _keepout_route_evidence(
        mcap: Path, mask_yaml: Path, keepout_spec: dict[str, Any],
        stop_contract: dict[str, Any]) -> dict[str, Any]:
    """Verify footprint clearance against the generated mask raster."""
    mask = _map_metadata(mask_yaml)
    resolution_m = mask['resolution']
    origin_x_m, origin_y_m, _origin_yaw_rad = mask['origin']
    occupied_cells = []
    for row in range(mask['height']):
        center_y_m = origin_y_m + (
            mask['height'] - row - 0.5) * resolution_m
        for column in range(mask['width']):
            if mask['pixels'][row * mask['width'] + column] != KEEPOUT_PIXEL:
                continue
            occupied_cells.append((
                origin_x_m + (column + 0.5) * resolution_m,
                center_y_m))
    if not occupied_cells:
        return {
            'status': 'FAIL', 'reason': 'keepout_mask_has_no_occupied_cells',
            'zone_id': keepout_spec['zone_id'],
        }
    occupied_min_x_m = min(cell[0] for cell in occupied_cells) - (
        resolution_m / 2.0)
    occupied_max_x_m = max(cell[0] for cell in occupied_cells) + (
        resolution_m / 2.0)
    occupied_min_y_m = min(cell[1] for cell in occupied_cells) - (
        resolution_m / 2.0)
    occupied_max_y_m = max(cell[1] for cell in occupied_cells) + (
        resolution_m / 2.0)
    inputs = stop_contract['stop_zone']['inputs']
    footprint = {
        'front_m': inputs['footprint_front_m'],
        'rear_m': inputs['footprint_rear_m'],
        'half_width_m': inputs['footprint_half_width_m'],
    }
    robot_radius_m = max(
        math.hypot(x_m, y_m)
        for x_m in (footprint['front_m'], footprint['rear_m'])
        for y_m in (-footprint['half_width_m'], footprint['half_width_m']))
    cell_half_diagonal_m = resolution_m / math.sqrt(2.0)
    samples = []
    before_zone = False
    after_zone = False
    for item in read_navigation_messages(mcap, topics=['/ground_truth_pose']):
        message = item.ros_msg
        x_m = float(message.pose.position.x)
        y_m = float(message.pose.position.y)
        before_zone = before_zone or x_m < occupied_min_x_m
        after_zone = after_zone or x_m > occupied_max_x_m
        stamp = message.header.stamp
        pose_xy_yaw = [x_m, y_m, _pose_yaw(message)]
        nearest_cell = min(
            occupied_cells,
            key=lambda cell: (cell[0] - x_m) ** 2 + (cell[1] - y_m) ** 2)
        clearance_m = rectangle_clearance(
            pose_xy_yaw, footprint, nearest_cell,
            (resolution_m, resolution_m))
        search_radius_m = (
            clearance_m + robot_radius_m + cell_half_diagonal_m)
        for cell in occupied_cells:
            if math.hypot(cell[0] - x_m, cell[1] - y_m) > search_radius_m:
                continue
            clearance_m = min(clearance_m, rectangle_clearance(
                pose_xy_yaw, footprint, cell,
                (resolution_m, resolution_m)))
        samples.append({
            'ros_ns': int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
            'pose_xy_yaw': pose_xy_yaw,
            'clearance_m': clearance_m,
        })
    if not samples:
        return {
            'status': 'FAIL',
            'reason': 'no_ground_truth_samples',
            'zone_id': keepout_spec['zone_id'],
        }
    minimum_clearance = min(samples, key=lambda item: item['clearance_m'])
    crossing_samples = [
        sample for sample in samples
        if occupied_min_x_m <= sample['pose_xy_yaw'][0] <= occupied_max_x_m]
    minimum_y = (
        min(crossing_samples, key=lambda item: item['pose_xy_yaw'][1])
        if crossing_samples else None)
    passed = (
        before_zone and after_zone
        and minimum_y is not None
        and minimum_y['pose_xy_yaw'][1] > occupied_max_y_m
        and minimum_clearance['clearance_m'] > 0.0)
    return {
        'status': 'PASS' if passed else 'FAIL',
        'zone_id': keepout_spec['zone_id'],
        'polygon_m': keepout_spec['polygon_m'],
        'safety_margin_m': keepout_spec['safety_margin_m'],
        'mask': {
            'yaml': _source_identity(mask_yaml),
            'image': _source_identity(mask['image']),
            'resolution_m_per_cell': resolution_m,
            'occupied_cell_count': len(occupied_cells),
            'occupied_bounds_m': {
                'min_x_m': occupied_min_x_m,
                'max_x_m': occupied_max_x_m,
                'min_y_m': occupied_min_y_m,
                'max_y_m': occupied_max_y_m,
            },
        },
        'route_side': 'north',
        'sample_count': len(samples),
        'crossing_sample_count': len(crossing_samples),
        'observed_before_zone': before_zone,
        'observed_after_zone': after_zone,
        'minimum_center_y_m': (
            minimum_y['pose_xy_yaw'][1] if minimum_y else None),
        'minimum_center_y_ros_ns': minimum_y['ros_ns'] if minimum_y else None,
        'minimum_footprint_clearance_m': minimum_clearance['clearance_m'],
        'minimum_clearance_ros_ns': minimum_clearance['ros_ns'],
        'minimum_clearance_pose_xy_yaw': minimum_clearance['pose_xy_yaw'],
    }


def _activate_route_obstacle(
        route_spec: dict[str, Any], environment: dict[str, str],
        activation: str, observation_s: float = 0.0) -> dict[str, Any]:
    """Activate and verify the persistent obstacle with clock evidence."""
    if observation_s > 0.0:
        time.sleep(observation_s)
    requested_wall_ns = time.time_ns()
    requested_steady_ns = time.monotonic_ns()
    active_pose_m = route_spec['active_pose_m']
    if not _set_pose(route_spec['name'], *active_pose_m, environment):
        raise RuntimeError('persistent route obstacle activation failed')
    route_state = _wait_entity_pose(
        route_spec['name'], active_pose_m, environment, timeout_s=15.0)
    return {
        'activation': activation,
        'activation_requested_wall_ns': requested_wall_ns,
        'activation_requested_steady_ns': requested_steady_ns,
        'activation_verified_wall_ns': time.time_ns(),
        'activation_verified_steady_ns': time.monotonic_ns(),
        'pre_activation_observation_s': observation_s,
        'entity_id': route_state['entity_id'],
        'expected_pose_m': active_pose_m,
        'observed_pose_m': route_state['pose_m'],
        'dimensions_m': [
            route_spec['length_m'], route_spec['width_m'],
            route_spec['height_m']],
        'straight_centerline_blocked': True,
    }


def scenario_passed(document: dict[str, Any], case: str,
                    returncode: int) -> bool:
    """Reject action success unless ground truth proves the intended route."""
    pose = document.get('final_robot_pose_xy_yaw')
    goal_reached = (
        isinstance(pose, list) and len(pose) >= 2
        and abs(float(pose[0]) - 6.0) <= 0.5
        and abs(float(pose[1])) <= 0.5)
    marked = document.get('mark_eligible_scan') is not None
    replanned = bool(document.get('plans_after_mark'))
    if case == 'event_driven_removal':
        replanned = (
            replanned and document.get('removal_passage') is not None)
    return (
        returncode == 0
        and document.get('action_terminal') == 'succeeded'
        and document.get('global_blocking') is True
        and document.get('local_blocking') is True
        and document.get('contact_matched_publisher_count_max', 0) > 0
        and document.get('contact_count') == 0
        and document.get('final_cmd_vel_zero') is True
        and goal_reached and marked and replanned)


def _scenario_command(
        case: str, prepared: dict[str, Any], evidence: Path,
        entity: str, contact_topic: str,
        obstacle_hold_s: float = 0.0, obstacle_crossing_s: float = 0.0,
        obstacle_entry_side: str = 'left',
        obstacle_crossing_edge_y_m: float = PEDESTRIAN_EDGE_Y_M) -> list[str]:
    if case in {'sudden_stop_resume', 'detour_sudden_stop_resume'}:
        command = [
            'ros2', 'run', 'jdamr_cube_navigation',
            'sim_collision_monitor_scenario',
            '--scenario', 'sudden_obstacle_stop_resume',
            '--seed', str(SEED), '--output', str(evidence),
            '--contract', str(prepared['stop_contract']),
            '--entity-name', entity, '--contact-topic', contact_topic,
            '--goal-x-m', '6.0', '--run-timeout-s', '180',
            '--obstacle-hold-s', str(obstacle_hold_s),
            '--obstacle-crossing-s', str(obstacle_crossing_s),
            '--obstacle-crossing-edge-y-m', str(obstacle_crossing_edge_y_m),
            '--obstacle-entry-side', obstacle_entry_side,
            '--direct-scan', '--ros-args', '-p', 'use_sim_time:=true',
        ]
        if case == 'detour_sudden_stop_resume':
            command[command.index('--direct-scan'):command.index(
                '--direct-scan')] = ['--trigger-after-x-m', '1.0']
        return command
    return [
        'ros2', 'run', 'jdamr_cube_navigation',
        'sim_nav_obstacle_scenario', '--scenario', case,
        '--seed', str(SEED), '--output', str(evidence),
        '--contract', str(prepared['contract']),
        '--behavior-tree', str(BT), '--run-timeout-s', '180',
        '--observation-persistence-s', '0.0',
        '--ros-args', '-p', 'use_sim_time:=true',
    ]


def _case_passed(
        document: dict[str, Any], case: str, returncode: int,
        same_goal_command_evidence: dict[str, Any],
        detour_evidence: dict[str, Any] | None = None) -> bool:
    if case not in {'sudden_stop_resume', 'detour_sudden_stop_resume'}:
        return scenario_passed(document, case, returncode)
    command_evidence = same_goal_command_evidence.get('evidence', {})
    return (
        stop_scenario_passed(document, returncode)
        and command_evidence.get('verdict') == 'CONFIRMED'
        and command_evidence.get('terminal_succeeded') is True
        and (case != 'detour_sudden_stop_resume'
             or (detour_evidence or {}).get('status') == 'PASS'))


def run_case(case: str, output_root: Path, domain_id: int,
             prepared: dict[str, Any], startup_only: bool = False,
             gui: bool = False, record_video: bool = False,
             pedestrian_entry: str = 'left') -> dict:
    """Launch isolated Gazebo plus the real onboard core and run one case."""
    direct_stop = case in {
        'sudden_stop_resume', 'detour_sudden_stop_resume'}
    combined_detour = case == 'detour_sudden_stop_resume'
    runtime_params = prepared['params']
    scenario_contract = (
        prepared['stop_contract'] if direct_stop else prepared['contract'])
    scenario_source = (
        STOP_SCENARIO_SOURCE if direct_stop else NAV_SCENARIO_SOURCE)
    run_id = f'onboard_candidate_{case}_seed_{SEED}'
    case_dir = output_root / case
    case_dir.mkdir(parents=True, exist_ok=True)
    environment = _environment(run_id, domain_id)
    environment.update({
        'ROS_LOCALHOST_ONLY': '1',
        'GZ_PARTITION': f'jdamr_onboard_{os.getpid()}_{case}',
    })
    launched = []
    bag_dir: Path | None = None
    mcap: Path | None = None
    video_recorder = None
    video_path = case_dir / 'gazebo_sensor_raw.mp4'
    video_metadata = case_dir / 'gazebo_sensor_capture.json'
    video_ready = case_dir / '.gazebo_sensor_ready'
    world_path = ASSETS / 'slam_corridor_contact.world'
    urdf_path = ASSETS / 'jdamr_cube_nav_eval.urdf'
    capture_world_report = None
    capture_urdf_report = None
    capture_world_manifest = case_dir / 'gazebo_capture_world.json'
    capture_urdf_manifest = case_dir / 'gazebo_capture_urdf.json'
    asset_contract = json.loads(
        prepared['contract'].read_text(encoding='utf-8'))
    route_spec = asset_contract['preloaded_obstacles']['models']['route']
    keepout_demo_spec = prepared.get('keepout_demo')
    if record_video:
        world_path = case_dir / 'gazebo_capture.world'
        urdf_path = case_dir / 'jdamr_cube_capture.urdf'
        capture_world_report = build_capture_world(
            ASSETS / 'slam_corridor_contact.world', world_path,
            keepout_polygon_m=(
                keepout_demo_spec['polygon_m']
                if keepout_demo_spec is not None else None))
        capture_urdf_report = build_capture_urdf(
            ASSETS / 'jdamr_cube_nav_eval.urdf', urdf_path)
        capture_world_manifest.write_text(json.dumps(
            capture_world_report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
        capture_urdf_manifest.write_text(json.dumps(
            capture_urdf_report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
    result: dict[str, Any] = {
        'case': case, 'seed': SEED, 'domain_id': domain_id,
        'run_id': run_id, 'status': 'FAIL', 'scenario_started': False,
        'claim_scope': 'SIM_INTEGRATION',
        'case_claim': {
            'detour': 'fixed_obstacle_detour',
            'event_driven_removal': (
                'obstacle_removal_and_replan_not_sudden_stop_resume'),
            'sudden_stop_resume': (
                'synthetic_sudden_obstacle_stop_same_goal_resume'),
            'detour_sudden_stop_resume': (
                'persistent_centerline_obstacle_detour_then_'
                'pedestrian_stop_same_goal_resume'),
        }[case],
        'runtime_profile': {
            'name': 'obstacle_candidate',
            'launch': 'onboard_nav2_core.launch.py',
            'mobile_manipulator_stop_field': direct_stop,
            'simulator_gui': gui,
            'simulator_capture': (
                'gazebo_camera_sensor' if record_video else 'disabled'),
            'pedestrian_corridor_edges_y_m': [
                PEDESTRIAN_DOORWAY_EDGE_Y_M if record_video else PEDESTRIAN_EDGE_Y_M,
                -(PEDESTRIAN_DOORWAY_EDGE_Y_M if record_video else PEDESTRIAN_EDGE_Y_M)],
            'keepout_demo': keepout_demo_spec is not None,
        },
        'source_identity': {
            'runner': _source_identity(Path(__file__)),
            'onboard_core': _source_identity(
                ROOT / 'jdamr_cube_navigation/launch/'
                'onboard_nav2_core.launch.py'),
            'production_params': _source_identity(PRODUCTION_PARAMS),
            'candidate_params': _source_identity(runtime_params),
            'candidate_bt': _source_identity(BT),
            'scenario_contract': _source_identity(scenario_contract),
            'scenario_source': _source_identity(scenario_source),
            'mobile_manipulator_protection': _source_identity(
                MOBILE_MANIPULATOR_PROTECTION),
            'capture_world_builder': _source_identity(
                CAPTURE_WORLD_BUILDER),
            'canonical_evaluation_world': _source_identity(
                ASSETS / 'slam_corridor_contact.world'),
            'keepout_mask': _source_identity(prepared['mask']),
            'keepout_zones': _source_identity(prepared['mask_zones']),
        },
    }
    try:
        launched.append(_start([
            'ros2', 'launch', 'jdamr_cube_gazebo', 'gazebo.launch.py',
            f'world:={world_path}',
            f'urdf_file:={urdf_path}',
            f'gui:={str(gui).lower()}', 'enable_image_bridges:=false',
            f'seed:={SEED}',
            'x_pose:=-8.0', 'y_pose:=0.0', 'z_pose:=0.01',
        ], case_dir / 'gazebo.log', environment))
        _wait_topics({'/scan', '/odom', '/ground_truth_pose', '/joint_states'},
                     environment, 60.0)
        if combined_detour and not record_video:
            result['static_route_obstacle'] = _activate_route_obstacle(
                route_spec, environment, 'verified_before_nav2_start')
        launched.append(_start([
            'ros2', 'launch', 'jdamr_cube_navigation',
            'onboard_nav2_core.launch.py',
            f'map:={ASSETS / "slam_corridor_eval.yaml"}',
            f'keepout_mask:={prepared["mask"]}',
            f'params_file:={runtime_params}',
            'use_sim_time:=true', 'autostart:=true',
            'navigation_profile:=obstacle_candidate',
        ], case_dir / 'onboard_nav2_core.log', environment))
        _wait_topics({
            '/global_costmap/costmap_raw', '/local_costmap/costmap_raw',
            '/clock', '/cmd_vel', '/plan', '/keepout_filter_mask',
            '/keepout_costmap_filter_info',
        }, environment, 75.0, case_dir / 'onboard_nav2_core.log')
        lifecycle_nodes = (
            '/keepout_filter_mask_server',
            '/keepout_costmap_filter_info_server', '/map_server', '/amcl',
            '/controller_server', '/planner_server', '/velocity_smoother',
            '/collision_monitor', '/behavior_server', '/bt_navigator')
        # bt_navigator is activated last by the navigation lifecycle manager.
        # Checking it plus both independently-managed keepout servers proves
        # readiness without ten serial ROS CLI discovery delays.  The complete
        # lifecycle inventory is captured after the time-sensitive scenario.
        _wait_lifecycle_active((
            '/keepout_filter_mask_server',
            '/keepout_costmap_filter_info_server', '/bt_navigator'),
            environment, 75.0)
        _wait_tf_available('map', 'base_footprint', environment, 60.0,
                           case_dir / 'tf_readiness.log')
        _wait_log_markers(case_dir / 'onboard_nav2_core.log', (
            '[behavior_server]: Activating wait',
            ('[local_costmap.local_costmap]: KeepoutFilter: '
             'Received filter mask'),
            ('[global_costmap.global_costmap]: KeepoutFilter: '
             'Received filter mask'),
        ), 15.0)
        pre_goal_clocks = {
            node: _parameter(node, 'use_sim_time', environment)
            for node in ('/controller_server', '/collision_monitor')}
        if not all(value.lower().endswith('true')
                   for value in pre_goal_clocks.values()):
            raise RuntimeError(
                f'pre-goal ROS time mismatch: {pre_goal_clocks}')
        travel_pose = None
        protective_parameters = None
        if direct_stop:
            stop_contract = json.loads(
                prepared['stop_contract'].read_text(encoding='utf-8'))
            travel_pose = _read_travel_pose(environment, stop_contract)
            result['travel_pose'] = travel_pose
            if travel_pose['status'] != 'PASS':
                raise RuntimeError('arm is outside the contracted travel pose')
            protective_parameters = {
                name: _parameter('/collision_monitor', name, environment)
                for name in ('StopZone.points', 'SlowdownZone.points')}
            expected_overrides = load_mobile_manipulator_protection(
                MOBILE_MANIPULATOR_PROTECTION)[
                    'collision_monitor_overrides']
            for name, output in protective_parameters.items():
                if (_json_string_parameter(output)
                        != json.loads(expected_overrides[name])):
                    raise RuntimeError(
                        f'Collision Monitor protective field drift: {name}')
        if startup_only:
            result['startup'] = {
                'status': 'PASS', 'pre_goal_use_sim_time': pre_goal_clocks,
                'travel_pose': travel_pose,
                'protective_parameters': protective_parameters}
            result['status'] = 'PASS'
            return result

        role = 'front_observation_probe' if direct_stop else 'route'
        entity = asset_contract[
            'preloaded_obstacles']['models'][role]['name']
        contact_topic = (f'/world/slam_corridor/model/{entity}/link/body/'
                         'sensor/contact_sensor/contact')
        launched.append(_start([
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'{contact_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ], case_dir / 'contact_bridge.log', environment))
        if record_video:
            launched.append(_start([
                'ros2', 'run', 'ros_gz_image', 'image_bridge',
                SIM_CAMERA_TOPIC.lstrip('/'),
            ], case_dir / 'simulator_camera_bridge.log', environment))
            _wait_topics({SIM_CAMERA_TOPIC}, environment, 20.0)
            video_recorder = _start([
                'python3', str(SIM_CAMERA_RECORDER),
                '--topic', SIM_CAMERA_TOPIC,
                '--output', str(video_path),
                '--metadata', str(video_metadata),
                '--ready-file', str(video_ready),
                '--fps', f'{CAMERA_RATE_HZ:g}',
            ], case_dir / 'simulator_camera_recorder.log', environment)
            launched.append(video_recorder)
            _wait_file_ready(video_recorder[0], video_ready)
        recorder, bag_dir = _start_compact_recorder(case_dir, environment)
        launched.append(recorder)
        _wait_bag_ready(recorder[0], bag_dir)
        if combined_detour and record_video:
            result['static_route_obstacle'] = _activate_route_obstacle(
                route_spec, environment,
                'verified_after_recorders_ready_before_goal',
                VIDEO_EMPTY_SCENE_OBSERVATION_S)
        evidence = case_dir / 'scenario.json'
        pedestrian_motion_enabled = record_video or combined_detour
        scenario = _start(_scenario_command(
            case, prepared, evidence, entity, contact_topic,
            obstacle_hold_s=2.0 if pedestrian_motion_enabled else 0.0,
            obstacle_crossing_s=0.8 if pedestrian_motion_enabled else 0.0,
            obstacle_entry_side=pedestrian_entry,
            obstacle_crossing_edge_y_m=(
                PEDESTRIAN_DOORWAY_EDGE_Y_M if record_video else PEDESTRIAN_EDGE_Y_M)),
            case_dir / 'scenario.log', environment)
        launched.append(scenario)
        result['scenario_started'] = True
        deadline_s = time.monotonic() + 210.0
        while scenario[0].poll() is None:
            if recorder[0].poll() is not None:
                raise RuntimeError('compact recorder exited during scenario')
            if (video_recorder is not None
                    and video_recorder[0].poll() is not None):
                raise RuntimeError(
                    'simulator camera recorder exited during scenario')
            if _tree_size(output_root) > BAG_LIVE_CAP_BYTES:
                raise RuntimeError('smoke evidence exceeded 56 MiB live cap')
            if time.monotonic() >= deadline_s:
                raise subprocess.TimeoutExpired('scenario', 210.0)
            time.sleep(0.25)
        returncode = scenario[0].wait(timeout=3.0)
        if recorder[0].poll() is not None:
            raise RuntimeError('compact recorder exited before scenario end')
        _stop(recorder[0])
        mcap = _one_finalized_mcap(bag_dir)
        if video_recorder is not None:
            _stop(video_recorder[0])
            if (video_recorder[0].returncode != 0
                    or not video_path.is_file()
                    or not video_metadata.is_file()):
                raise RuntimeError('simulator camera recording did not finalize')
            capture = json.loads(video_metadata.read_text(encoding='utf-8'))
            if (capture.get('frames', 0) < 10
                    or capture.get('source') != 'gazebo_camera_sensor'):
                raise RuntimeError('simulator camera recording is incomplete')
            result['simulator_video'] = {
                'raw_video': _source_identity(video_path),
                'capture_metadata': _source_identity(video_metadata),
                'source': capture['source'],
                'topic': capture['topic'],
                'width': capture['width'],
                'height': capture['height'],
                'frames': capture['frames'],
                'capture_duration_s': capture['capture_duration_s'],
                'first_frame_steady_ns': capture[
                    'first_frame_steady_ns'],
                'overlays_applied': False,
                'capture_world': capture_world_report,
                'capture_urdf': capture_urdf_report,
                'capture_world_manifest': _source_identity(
                    capture_world_manifest),
                'capture_urdf_manifest': _source_identity(
                    capture_urdf_manifest),
            }
        result['recording'] = {
            'mcap': _source_identity(mcap),
            'size_bytes': mcap.stat().st_size,
            'topics': list(RECORDED_TOPICS),
            'log_time_basis': 'wall_time',
            'message_header_time_basis': 'publisher_defined_sim_time',
            'time_basis_mixing_forbidden': True,
        }
        result['raw_action_status'] = _status_summary(mcap)
        same_goal_evidence = evaluate_same_goal_bag(mcap)
        result['same_goal_command_evidence'] = same_goal_evidence
        measured_detour = None
        if combined_detour:
            stop_contract = json.loads(
                prepared['stop_contract'].read_text(encoding='utf-8'))
            measured_detour = _detour_evidence(
                mcap, route_spec, stop_contract)
            result['detour_evidence'] = measured_detour
        keepout_route = None
        if keepout_demo_spec is not None:
            stop_contract = json.loads(
                prepared['stop_contract'].read_text(encoding='utf-8'))
            keepout_route = _keepout_route_evidence(
                mcap, prepared['mask'], keepout_demo_spec, stop_contract)
            result['keepout_route_evidence'] = keepout_route
        if not evidence.is_file():
            raise RuntimeError('scenario produced no evidence')
        scenario_document = json.loads(evidence.read_text(encoding='utf-8'))
        bt_parameter = None
        keepout_parameters = None
        clock_parameters = None
        postrun_diagnostics: dict[str, Any]
        try:
            bt_parameter = _parameter(
                '/bt_navigator', 'default_nav_to_pose_bt_xml', environment)
            keepout_parameters = {
                name: _parameter(
                    f'/{name}/{name}', 'keepout_filter.enabled', environment)
                for name in ('local_costmap', 'global_costmap')}
            clock_parameters = {
                node: _parameter(node, 'use_sim_time', environment)
                for node in ('/amcl', '/controller_server',
                             '/collision_monitor', '/bt_navigator')}
            if BT.name not in bt_parameter or not all(
                    value.lower().endswith('true')
                    for value in (*keepout_parameters.values(),
                                  *clock_parameters.values())):
                raise RuntimeError('post-run candidate configuration drift')
            postrun_diagnostics = {'status': 'PASS'}
        except RuntimeError as error:
            if 'configuration drift' in str(error):
                raise
            postrun_diagnostics = {
                'status': 'INCONCLUSIVE',
                'reason': str(error),
                'effect_on_behavior_verdict': 'none',
            }
        result['startup'] = {
            'status': 'PASS', 'candidate_bt': bt_parameter,
            'keepout_enabled': keepout_parameters,
            'use_sim_time': clock_parameters,
            'pre_goal_use_sim_time': pre_goal_clocks,
            'protective_parameters': protective_parameters,
            'timing_note': (
                'moving serial diagnostics after the scenario reduces delay; '
                'explicit component use_sim_time is the clock-domain fix'),
            'lifecycle': {node: _lifecycle(node, environment)
                          for node in lifecycle_nodes},
            'mask_keepout_cells': prepared['mask_report']['keepout_cells'],
            'gz_partition': environment['GZ_PARTITION'],
            'ros_localhost_only': environment['ROS_LOCALHOST_ONLY'],
            'postrun_diagnostics': postrun_diagnostics,
        }
        if direct_stop:
            result['scenario'] = {
                'returncode': returncode,
                'action_terminal': scenario_document.get('action_terminal'),
                'obstacle_crossing_edge_y_m': scenario_document.get(
                    'obstacle_crossing_edge_y_m'),
                'events': [event['name'] for event in
                           scenario_document.get('events', [])],
                'goal_uuid': scenario_document.get('goal_uuid'),
                'terminal_goal_uuid': scenario_document.get(
                    'terminal_goal_uuid'),
                'goal_send_count': scenario_document.get('goal_send_count'),
                'goal_cancel_count': scenario_document.get(
                    'goal_cancel_count'),
                'pre_stop_action_types': scenario_document.get(
                    'pre_stop_action_types'),
                'stop_action_type': scenario_document.get('stop_action_type'),
                'stop_polygon_name': scenario_document.get(
                    'stop_polygon_name'),
                'resume_action_type': scenario_document.get(
                    'resume_action_type'),
                'physical_stop_observed': scenario_document.get(
                    'physical_stop_observed'),
                'direct_scan_capture': scenario_document.get(
                    'direct_scan_capture'),
                'observer_scan_to_zero_command_s': (
                    (
                        scenario_document['direct_scan_capture'][
                            'zero_receive_steady_ns']
                        - scenario_document['direct_scan_capture'][
                            'scan_receive_steady_ns']) / 1e9
                    if (scenario_document.get('direct_scan_capture')
                        and 'zero_receive_steady_ns'
                        in scenario_document['direct_scan_capture'])
                    else None),
                'contact_count': scenario_document.get('contact_count'),
                'raw_contact_count': scenario_document.get(
                    'raw_contact_count'),
                'robot_contact_pairs': scenario_document.get(
                    'robot_contact_pairs'),
                'contact_matched_publisher_count_max': scenario_document.get(
                    'contact_matched_publisher_count_max'),
                'minimum_clearance_m': scenario_document.get(
                    'footprint_to_obstacle_clearance_m'),
                'protected_envelope_minimum_clearance_m': (
                    scenario_document.get(
                        'protected_envelope_to_obstacle_clearance_m')),
                'final_cmd_vel_zero': scenario_document.get(
                    'final_cmd_vel_zero'),
                'final_zero_hold_s': scenario_document.get(
                    'final_zero_hold_s'),
                'final_world_pose_m': scenario_document.get(
                    'final_world_pose_m'),
                'activation_error': scenario_document.get('activation_error'),
                'harness_error': scenario_document.get('harness_error'),
                'same_goal_command_verdict': same_goal_evidence[
                    'evidence']['verdict'],
            }
        else:
            result['scenario'] = {
                'returncode': returncode,
                'action_terminal': scenario_document.get('action_terminal'),
                'events': [event['name'] for event in
                           scenario_document.get('events', [])],
                'costmap_message_counts': scenario_document.get(
                    'costmap_message_counts'),
                'contact_count': scenario_document.get('contact_count'),
                'contact_matched_publisher_count_max': scenario_document.get(
                    'contact_matched_publisher_count_max'),
                'global_blocking': scenario_document.get('global_blocking'),
                'local_blocking': scenario_document.get('local_blocking'),
                'final_cmd_vel_zero': scenario_document.get(
                    'final_cmd_vel_zero'),
                'final_zero_hold_s': scenario_document.get(
                    'final_zero_hold_s'),
                'minimum_obstacle_aabb_distance_m': scenario_document.get(
                    'minimum_robot_obstacle_aabb_distance_m'),
                'final_robot_pose_xy_yaw': scenario_document.get(
                    'final_robot_pose_xy_yaw'),
                'mark_eligible_scan': scenario_document.get(
                    'mark_eligible_scan'),
                'plans_after_mark_count': len(
                    scenario_document.get('plans_after_mark', [])),
                'activation_error': scenario_document.get('activation_error'),
            }
        passed = _case_passed(
            scenario_document, case, returncode, same_goal_evidence,
            measured_detour)
        if keepout_demo_spec is not None:
            passed = passed and keepout_route['status'] == 'PASS'
        result['status'] = 'PASS' if passed else 'FAIL'
    except Exception as error:
        result['harness_error'] = f'{type(error).__name__}: {error}'
    finally:
        groups = [process.pid for process, _stream in launched]
        for process, _stream in reversed(launched):
            _stop(process)
        for _process, stream in launched:
            stream.close()
        survivors = _cleanup_identity(run_id, domain_id)
        if 'raw_action_status' not in result and mcap is not None:
            try:
                result['raw_action_status'] = _status_summary(mcap)
            except Exception as error:
                result['recording_error'] = (
                    f'{type(error).__name__}: {error}')
                result['status'] = 'FAIL'
        result['teardown'] = {
            'launched_process_groups': groups,
            'remaining_process_groups': [group for group in groups
                                         if _group_members(group)],
            'identity_survivors': survivors,
        }
        if (result['teardown']['remaining_process_groups'] or survivors):
            result['status'] = 'FAIL'
            result['teardown_error'] = 'owned_processes_remain'
        try:
            result['output_bytes'] = _cap_output(case_dir)
        except RuntimeError as error:
            result['output_bytes'] = _tree_size(case_dir)
            result['recording_error'] = str(error)
            result['status'] = 'FAIL'
        (case_dir / 'summary.json').write_text(json.dumps(
            result, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
    return result


def main() -> int:
    """Parse the bounded smoke request, execute it, and persist a summary."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--domain-id', type=int, default=186)
    parser.add_argument('--case', choices=CASES, action='append')
    parser.add_argument('--startup-only', action='store_true')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--record-simulator-video', action='store_true')
    parser.add_argument('--keepout-demo', action='store_true')
    parser.add_argument(
        '--pedestrian-entry', choices=('left', 'right'), default='left')
    args = parser.parse_args()
    if args.domain_id == 12 or args.domain_id not in DOMAIN_IDS:
        parser.error('only isolated ROS domain 186 or 187 is allowed')
    if args.gui and not os.environ.get('DISPLAY'):
        parser.error('--gui requires an available DISPLAY')
    if args.gui and args.record_simulator_video:
        parser.error(
            '--gui and --record-simulator-video are mutually exclusive')
    selected = args.case or list(DEFAULT_CASES)
    assigned_domains = [args.domain_id + index
                        for index in range(len(selected))]
    if any(domain not in DOMAIN_IDS for domain in assigned_domains):
        parser.error('selected cases require only domains 186 and 187')
    if args.output_root.exists() and any(args.output_root.iterdir()):
        parser.error('output root must be new or empty')
    args.output_root.mkdir(parents=True, exist_ok=True)
    prepared = prepare_candidate_assets(
        args.output_root, keepout_demo=args.keepout_demo)
    results = []
    for index, case in enumerate(selected):
        results.append(run_case(
            case, args.output_root, assigned_domains[index], prepared,
            startup_only=args.startup_only, gui=args.gui,
            record_video=args.record_simulator_video,
            pedestrian_entry=args.pedestrian_entry))
    summary = {
        'schema_version': 1, 'seed': SEED, 'cases_requested': selected,
        'claim_scope': 'SIM_INTEGRATION', 'results': results,
        'status': ('PASS' if all(item['status'] == 'PASS' for item in results)
                   else 'FAIL'),
    }
    (args.output_root / 'summary.json').write_text(json.dumps(
        summary, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    try:
        summary['output_bytes'] = _cap_output(args.output_root)
    except RuntimeError as error:
        summary['status'] = 'FAIL'
        summary['output_error'] = str(error)
    (args.output_root / 'summary.json').write_text(json.dumps(
        summary, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    return 0 if summary['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
