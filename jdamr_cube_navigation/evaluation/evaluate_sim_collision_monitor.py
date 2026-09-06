#!/usr/bin/env python3
"""Fail-closed evaluator for G004 Collision Monitor evidence."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
from typing import Any
import xml.etree.ElementTree as ET

from sim_collision_monitor_contract import scenario_matrix, sha256_file

import yaml


PROCESS_DEATH_PATTERN = re.compile(
    rb'\[ERROR\] \[([^\]]+)\]: process has died '
    rb'\[pid (\d+), exit code (-?\d+), cmd \'([^\']+)\'\]\.\Z')
GAZEBO_BRIDGE_EXECUTABLE = (
    '/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge')


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
    """Validate the narrow post-stop launch child allowlist."""
    gazebo_bridge_aborts = 0
    for record in records:
        if (not isinstance(record, dict) or set(record) != {
                'label', 'pid', 'exit_code', 'command', 'byte_offset'}
                or not isinstance(record['label'], str)
                or not _exact_int(record['pid'], positive=True)
                or type(record['exit_code']) is not int
                or not isinstance(record['command'], str)
                or not _exact_int(record['byte_offset'])):
            return False
        code = record['exit_code']
        if code in {-2, -15}:
            continue
        if code == -6 and owner in {'navigation', 'contact_bridge'}:
            continue
        if (code == -6 and owner == 'gazebo'
                and re.fullmatch(r'parameter_bridge-\d+', record['label'])
                and record['command'].split(' ', 1)[0]
                == GAZEBO_BRIDGE_EXECUTABLE):
            gazebo_bridge_aborts += 1
            continue
        return False
    return gazebo_bridge_aborts <= 1


def _finite_number(value: Any) -> bool:
    return (not isinstance(value, bool)
            and isinstance(value, (int, float)) and math.isfinite(value))


def _exact_int(value: Any, *, positive: bool = False) -> bool:
    """Accept JSON integers while rejecting booleans and float lookalikes."""
    return (type(value) is int and (value > 0 if positive else value >= 0))


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def strict_json_loads(text: str) -> Any:
    """Decode JSON while rejecting duplicate object keys."""
    return json.loads(text, object_pairs_hook=_reject_duplicate_keys)


def _matches(value: Any, expected: float) -> bool:
    return _finite_number(value) and math.isclose(
        value, expected, rel_tol=1e-9, abs_tol=1e-9)


def _canonical_tf_subset(records, query_stamp_ns):
    """Select the unique ordered minimal map-to-base TF witness."""
    grouped = {}
    for record in records:
        edge = (record.get('parent'), record.get('child'))
        grouped.setdefault(edge, {})[
            json.dumps(record, sort_keys=True)] = record
    graph = {}
    for parent, child in grouped:
        graph.setdefault(parent, []).append(child)
    paths = []

    def visit(frame, path, seen):
        if frame == 'base_link':
            paths.append(path)
            return
        for child in sorted(graph.get(frame, [])):
            if child not in seen:
                visit(child, path + [(frame, child)], seen | {child})

    visit('map', [], {'map'})
    if len(paths) != 1:
        return None
    selected = []
    for edge in paths[0]:
        edge_records = sorted(
            grouped[edge].values(), key=lambda item: item['stamp_ns'])
        static = [item for item in edge_records if item['is_static']]
        dynamic = [item for item in edge_records if not item['is_static']]
        exact = [item for item in dynamic
                 if item['stamp_ns'] == query_stamp_ns]
        if static:
            if len(static) != 1 or dynamic:
                return None
            chosen = static
        elif len(exact) == 1:
            chosen = exact
        else:
            before = [item for item in dynamic
                      if item['stamp_ns'] < query_stamp_ns]
            after = [item for item in dynamic
                     if item['stamp_ns'] > query_stamp_ns]
            if exact or not before or not after:
                return None
            chosen = [before[-1], after[0]]
        selected.extend(chosen)
    return selected


def _difference_s(document: dict[str, Any], end: str, start: str) -> float:
    end_ns = document.get(end)
    start_ns = document.get(start)
    if not _finite_number(end_ns) or not _finite_number(start_ns):
        return math.inf
    return (end_ns - start_ns) / 1e9


def _tree_size_excluding(root: Path, excluded: set[str]) -> int:
    return sum(
        path.stat().st_size for path in root.rglob('*')
        if path.is_file() and str(path.relative_to(root)) not in excluded)


def _bag_artifacts_absent(root: Path | None) -> bool:
    """Reject any bag or bag-quarantine artifact in bags-off runs."""
    if root is None or not root.is_dir():
        return False
    for path in root.rglob('*'):
        name = path.name.lower()
        if ((path.is_dir() and name == 'bag')
                or (path.is_file() and (
                    path.suffix.lower() == '.mcap'
                    or name == 'metadata.yaml'
                    or ('quarantine' in name and 'bag' in name)))):
            return False
    return True


def _resource_record_valid(
        name: str, record: dict[str, Any], run_dir: Path | None,
) -> bool:
    """Recompute one resource summary from its canonical JSONL bytes."""
    try:
        path = Path(record['path'])
        if (run_dir is None or path.resolve().parent != run_dir
                or path.name != f'{name}_resources.jsonl'
                or path.stat().st_size != record['size_bytes']
                or sha256_file(path) != record['sha256']):
            return False
        rows = [strict_json_loads(line) for line in path.read_text().splitlines()
                if line.strip()]
        required = {
            'monotonic_s', 'cpu_total_s', 'cpu_pct_one_core',
            'rss_mb', 'process_count'}
        if len(rows) < 2 or any(set(row) != required for row in rows):
            return False
        monotonic = [row['monotonic_s'] for row in rows]
        cpu_total = [row['cpu_total_s'] for row in rows]
        rss = [row['rss_mb'] for row in rows]
        counts = [row['process_count'] for row in rows]
        cpu = [row['cpu_pct_one_core'] for row in rows
               if row['cpu_pct_one_core'] is not None]
        finite = [*monotonic, *cpu_total, *rss, *cpu]
        if (not cpu or not all(_finite_number(value) for value in finite)
                or any(right <= left
                       for left, right in zip(monotonic, monotonic[1:]))
                or any(value < 0 for value in [*cpu_total, *rss, *cpu])
                or any(not _exact_int(value) for value in counts)):
            return False
        if rows[0]['cpu_pct_one_core'] is not None:
            return False
        for previous, current in zip(rows, rows[1:]):
            elapsed_s = current['monotonic_s'] - previous['monotonic_s']
            expected_cpu = max(
                0.0,
                (current['cpu_total_s'] - previous['cpu_total_s'])
                / elapsed_s * 100.0)
            if (current['cpu_pct_one_core'] is None
                    or not math.isclose(
                        current['cpu_pct_one_core'], expected_cpu,
                        rel_tol=1e-9, abs_tol=1e-9)):
                return False
        ordered_cpu = sorted(cpu)
        p95 = ordered_cpu[math.ceil(0.95 * len(ordered_cpu)) - 1]
        return (
            _exact_int(record['sample_count'], positive=True)
            and record['sample_count'] == len(rows)
            and _exact_int(record['cpu_sample_count'], positive=True)
            and record['cpu_sample_count'] == len(cpu)
            and math.isclose(record['median_cpu_pct_one_core'],
                             statistics.median(cpu), abs_tol=1e-9)
            and math.isclose(record['p95_cpu_pct_one_core'], p95,
                             abs_tol=1e-9)
            and math.isclose(record['max_rss_mb'], max(rss), abs_tol=1e-9)
            and record['max_process_count'] == max(counts))
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        return False


def _ordered_unique_events(
        document: dict[str, Any], required: list[str],
) -> bool:
    events = document.get('events')
    if not isinstance(events, list):
        return False
    selected = []
    for name in required:
        matches = [event for event in events if event.get('name') == name]
        if len(matches) != 1:
            return False
        selected.append(matches[0])
    return all(
        _exact_int(event.get('steady_ns'))
        and _exact_int(event.get('ros_ns'))
        for event in selected) and all(
        right['steady_ns'] > left['steady_ns']
        for left, right in zip(selected, selected[1:]))


def _event_scalar_bindings_valid(document: dict[str, Any]) -> bool:
    """Bind duplicated scalar timestamps to their originating events."""
    events = document.get('events')
    if not isinstance(events, list):
        return False
    by_name = {event.get('name'): event for event in events
               if isinstance(event, dict)}
    required = {
        'stop_state': ('stop_state_steady_ns', 'stop_state_ros_ns'),
        'final_zero': ('zero_steady_ns', 'zero_ros_ns'),
        'physical_stop': ('physical_stop_steady_ns', None),
    }
    if document.get('scenario') == 'scan_timeout_stop_resume':
        required['monitor_scan_freeze_requested'] = (
            'freeze_request_steady_ns', 'freeze_request_ros_ns')
        required['monitor_scan_frozen'] = (
            'freeze_steady_ns', 'freeze_ros_ns')
        required['monitor_scan_unfreeze_requested'] = (
            'unfreeze_request_steady_ns', 'unfreeze_request_ros_ns')
        required['monitor_scan_unfrozen'] = (
            'unfreeze_ack_steady_ns', 'unfreeze_ack_ros_ns')
    if document.get('scenario') == 'sudden_obstacle_stop_resume':
        required['scan_gate_arm_requested'] = (
            'scan_gate_arm_request_steady_ns',
            'scan_gate_arm_request_ros_ns')
        required['scan_gate_arm_ack'] = (
            'scan_gate_arm_ack_steady_ns', 'scan_gate_arm_ack_ros_ns')
        required['obstacle_set_pose_requested'] = (
            'obstacle_request_steady_ns', 'obstacle_request_ros_ns')
        required['obstacle_set_pose_ack'] = (
            'obstacle_activation_steady_ns', 'obstacle_activation_ros_ns')
        required['obstacle_pose_verified'] = (
            'obstacle_verified_steady_ns', 'obstacle_verified_ros_ns')
        required['clear_set_pose_requested'] = (
            'clear_request_steady_ns', 'clear_request_ros_ns')
        required['clear_set_pose_ack'] = (
            'clear_ack_steady_ns', 'clear_ack_ros_ns')
        required['clear_pose_verified'] = (
            'clear_verified_steady_ns', 'clear_verified_ros_ns')
    for event_name, (steady_key, ros_key) in required.items():
        matches = [candidate for candidate in events
                   if isinstance(candidate, dict)
                   and candidate.get('name') == event_name]
        event = by_name.get(event_name)
        if (len(matches) != 1 or not isinstance(event, dict)
                or event.get('steady_ns') != document.get(steady_key)
                or (ros_key is not None
                    and event.get('ros_ns') != document.get(ros_key))):
            return False
    return True


def _provenance_partial_order_valid(document: dict[str, Any]) -> bool:
    """Validate only causal orders guaranteed across callback groups."""
    events = document.get('events', [])
    by_name = {event.get('name'): event for event in events
               if isinstance(event, dict)}

    def ordered(*names: str) -> bool:
        try:
            values = [by_name[name]['steady_ns'] for name in names]
            return all(_exact_int(value) for value in values) and all(
                left < right for left, right in zip(values, values[1:]))
        except (KeyError, TypeError):
            return False

    if document.get('scenario') == 'sudden_obstacle_stop_resume':
        return all((
            ordered('scan_gate_arm_requested', 'scan_gate_arm_ack',
                    'obstacle_set_pose_requested'),
            ordered('obstacle_set_pose_requested', 'obstacle_set_pose_ack',
                    'obstacle_pose_verified', 'clear_set_pose_requested',
                    'succeeded'),
            ordered('obstacle_set_pose_requested', 'stop_state',
                    'final_zero', 'physical_stop',
                    'clear_set_pose_requested'),
            ordered('clear_set_pose_requested', 'clear_set_pose_ack',
                    'clear_pose_verified', 'succeeded'),
            ordered('clear_set_pose_requested', 'do_nothing_after_stop',
                    'succeeded'),
        ))
    if document.get('scenario') == 'scan_timeout_stop_resume':
        return all((
            ordered('monitor_scan_freeze_requested', 'monitor_scan_frozen',
                    'stop_state', 'final_zero', 'physical_stop',
                    'monitor_scan_unfreeze_requested'),
            ordered('monitor_scan_unfreeze_requested',
                    'monitor_scan_unfrozen', 'succeeded'),
            ordered('monitor_scan_unfreeze_requested',
                    'do_nothing_after_stop', 'succeeded'),
        ))
    return True


def _production_derivation_valid(contract: dict[str, Any]) -> bool:
    """Re-read production motion limits and bind them to the contract."""
    try:
        production = contract['production_inputs']
        params = yaml.safe_load(Path(
            production['production_params']['path']).read_text())
        urdf_path = Path(production['production_urdf']['path'])
        smoother = params['velocity_smoother']['ros__parameters']
        controller_params = params['controller_server']['ros__parameters']
        controller_speed = float(params['controller_server'][
            'ros__parameters']['FollowPath']['desired_linear_vel'])
        smoother_speed = float(smoother['max_velocity'][0])
        max_decel = abs(float(smoother['max_decel'][0]))
        local = params['local_costmap']['local_costmap']['ros__parameters']
        global_costmap = params['global_costmap']['global_costmap'][
            'ros__parameters']
        if (not _matches(controller_speed, smoother_speed)
                or local['footprint'] != global_costmap['footprint']
                or not _matches(local['resolution'],
                                global_costmap['resolution'])):
            return False
        points = strict_json_loads(local['footprint'])
        footprint = {
            'footprint_front_m': max(float(point[0]) for point in points),
            'footprint_rear_m': min(float(point[0]) for point in points),
            'footprint_half_width_m': max(
                abs(float(point[1])) for point in points),
        }
        tree = ET.parse(urdf_path)
        joints = {joint.get('name'): joint
                  for joint in tree.findall('.//joint')}
        links = {link.get('name'): link for link in tree.findall('.//link')}
        front_origin = joints['caster_front_joint'].find('origin')
        rear_origin = joints['caster_rear_joint'].find('origin')
        front_radius = float(links['caster_link_front'].find(
            'collision/geometry/sphere').get('radius'))
        rear_radius = float(links['caster_link_rear'].find(
            'collision/geometry/sphere').get('radius'))
        left_origin = joints['left_wheel_joint'].find('origin')
        left_width = float(links['left_wheel_link'].find(
            'collision/geometry/cylinder').get('length'))
        urdf_footprint = {
            'footprint_front_m': (
                float(front_origin.get('xyz').split()[0]) + front_radius),
            'footprint_rear_m': (
                float(rear_origin.get('xyz').split()[0]) - rear_radius),
            'footprint_half_width_m': (
                abs(float(left_origin.get('xyz').split()[1]))
                + left_width / 2.0),
        }
        inputs = contract['stop_zone']['inputs']
        expected = {
            **footprint,
            'max_forward_speed_mps': controller_speed,
            'max_decel_mps2': max_decel,
            'costmap_resolution_m': float(local['resolution']),
            'smoothing_frequency_hz': float(smoother['smoothing_frequency']),
        }
        return (
            all(_matches(footprint[key], urdf_footprint[key])
                for key in footprint)
            and all(_matches(inputs.get(key), value)
                    for key, value in expected.items())
            and _matches(
                contract['goal_verification']['position_tolerance_m'],
                controller_params['general_goal_checker'][
                    'xy_goal_tolerance'])
            and _matches(
                contract['goal_verification']['position_tolerance_m'],
                controller_params['position_goal_checker'][
                    'xy_goal_tolerance'])
            and _matches(
                contract['goal_verification'][
                    'estimate_stamp_tolerance_s'],
                params['amcl']['ros__parameters']['transform_tolerance'])
            and contract['goal_verification']['selected_goal_checker']
            == controller_params['goal_checker_plugins'][0]
            and contract['goal_verification'][
                'selected_goal_checker_plugin']
            == controller_params[controller_params[
                'goal_checker_plugins'][0]]['plugin']
            and contract['goal_verification'][
                'selected_goal_checker_stateful'] is True
            and controller_params[controller_params[
                'goal_checker_plugins'][0]]['stateful'] is True
            and contract['goal_verification']['orientation_verified'] is False)
    except (KeyError, OSError, TypeError, ValueError, ET.ParseError,
            json.JSONDecodeError, yaml.YAMLError):
        return False


def _entity_states_valid(
        document: dict[str, Any], contract: dict[str, Any]) -> bool:
    """Recompute the injected obstacle activation and removal truth."""
    states = document.get('entity_states')
    if document.get('scenario') != 'sudden_obstacle_stop_resume':
        return states == []
    try:
        if not isinstance(states, list) or len(states) != 2:
            return False
        entity_ids = {state['entity_id'] for state in states}
        if (len(entity_ids) != 1
                or not _exact_int(next(iter(entity_ids)), positive=True)):
            return False
        trigger = document['trigger_world_pose_m']
        expected_states = [
            [trigger[0] + contract['sudden_obstacle'][
                'activation_center_offset_x_m'], trigger[1], 0.5],
            contract['sudden_obstacle']['removal_pose_m'],
        ]
        for state, expected in zip(states, expected_states):
            pose = state['pose_m']
            recorded_expected = state['expected_pose_m']
            error_m = state['position_error_m']
            defaulted_axes = state['defaulted_axes']
            orientation_yaw_rad = state['orientation_yaw_rad']
            values = [
                *pose, *recorded_expected, error_m, orientation_yaw_rad]
            if (len(pose) != 3 or len(recorded_expected) != 3
                    or not isinstance(defaulted_axes, list)
                    or len(defaulted_axes) != len(set(defaulted_axes))
                    or any(axis not in {'x', 'y', 'z'}
                           for axis in defaulted_axes)
                    or any(expected[index] != 0.0
                           for index, axis in enumerate(('x', 'y', 'z'))
                           if axis in defaulted_axes)
                    or type(state['orientation_defaulted']) is not bool
                    or not _matches(orientation_yaw_rad, 0.0)
                    or not all(_finite_number(value) for value in values)
                    or any(not _matches(left, right)
                           for left, right in zip(recorded_expected, expected))
                    or not _matches(error_m, math.dist(pose, expected))
                    or error_m > 0.02):
                return False
        return True
    except (IndexError, KeyError, TypeError, ValueError):
        return False


def _point_segment_distance(point, start, end) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return math.dist(point, start)
    ratio = max(0.0, min(1.0, (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / denominator))
    projection = (start[0] + ratio * dx, start[1] + ratio * dy)
    return math.dist(point, projection)


def _rectangle_clearance(robot_pose, footprint, obstacle_center, dimensions):
    x_m, y_m, yaw_rad = robot_pose
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    local = [
        (footprint['front_m'], footprint['half_width_m']),
        (footprint['front_m'], -footprint['half_width_m']),
        (footprint['rear_m'], -footprint['half_width_m']),
        (footprint['rear_m'], footprint['half_width_m']),
    ]
    robot = [(x_m + cosine * x - sine * y,
              y_m + sine * x + cosine * y) for x, y in local]
    half_x, half_y = dimensions[0] / 2.0, dimensions[1] / 2.0
    ox, oy = obstacle_center
    obstacle = [
        (ox + half_x, oy + half_y), (ox + half_x, oy - half_y),
        (ox - half_x, oy - half_y), (ox - half_x, oy + half_y)]

    def projection(polygon, axis):
        values = [x * axis[0] + y * axis[1] for x, y in polygon]
        return min(values), max(values)

    intersects = True
    for polygon in (robot, obstacle):
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            axis = (-(end[1] - start[1]), end[0] - start[0])
            r_min, r_max = projection(robot, axis)
            o_min, o_max = projection(obstacle, axis)
            if r_max < o_min or o_max < r_min:
                intersects = False
                break
        if not intersects:
            break
    if intersects:
        return 0.0
    robot_edges = list(zip(robot, robot[1:] + robot[:1]))
    obstacle_edges = list(zip(obstacle, obstacle[1:] + obstacle[:1]))
    return min(
        [_point_segment_distance(point, *edge)
         for point in robot for edge in obstacle_edges]
        + [_point_segment_distance(point, *edge)
           for point in obstacle for edge in robot_edges])


def _clearance_evidence_valid(
        document: dict[str, Any], contract: dict[str, Any]) -> bool:
    """Recompute every retained GT footprint-to-obstacle sample."""
    if document.get('scenario') != 'sudden_obstacle_stop_resume':
        return (
            document.get('footprint_to_obstacle_clearance_m') is None
            and document.get('minimum_clearance_witness') is None
            and document.get('clearance_samples') == []
            and _exact_int(document.get('clearance_sample_count'))
            and document['clearance_sample_count'] == 0)
    try:
        samples = document['clearance_samples']
        if (not isinstance(samples, list) or not samples
                or document['clearance_sample_count'] != len(samples)):
            return False
        stamps = []
        for sample in samples:
            if (not isinstance(sample, list) or len(sample) != 4
                    or not _exact_int(sample[0])
                    or not all(_finite_number(value) for value in sample[1:])):
                return False
            stamps.append(sample[0])
        if any(right <= left for left, right in zip(stamps, stamps[1:])):
            return False
        inputs = contract['stop_zone']['inputs']
        footprint = {
            'front_m': inputs['footprint_front_m'],
            'rear_m': inputs['footprint_rear_m'],
            'half_width_m': inputs['footprint_half_width_m'],
        }
        dimensions = [contract['sudden_obstacle']['length_m'],
                      contract['sudden_obstacle']['width_m']]
        obstacle_center = document['entity_states'][0]['pose_m'][:2]
        clearances = [_rectangle_clearance(
            sample[1:], footprint, obstacle_center, dimensions)
            for sample in samples]
        minimum = min(clearances)
        index = clearances.index(minimum)
        witness = document['minimum_clearance_witness']
        expected_witness = {
            'robot_pose_xyyaw': samples[index][1:],
            'obstacle_center_xy': obstacle_center,
            'obstacle_dimensions_m': dimensions,
            'clearance_m': minimum,
        }
        return (
            set(witness) == set(expected_witness)
            and all(witness[key] == expected_witness[key]
                    for key in expected_witness if key != 'clearance_m')
            and _matches(witness['clearance_m'], minimum)
            and _matches(
                document['footprint_to_obstacle_clearance_m'], minimum)
            and minimum > 0.0)
    except (IndexError, KeyError, TypeError, ValueError):
        return False


def _scan_gate_evidence_document(
        document: dict[str, Any], contract: dict[str, Any],
        evidence_path: Path | None) -> dict[str, Any] | None:
    """Recompute StopZone membership from the retained raw scan values."""
    try:
        record = document.get('scan_gate_evidence')
        if (document.get('scenario') != 'sudden_obstacle_stop_resume'
                or evidence_path is None or set(record) != {
                    'path', 'size_bytes', 'sha256'}):
            return None
        path = Path(record['path'])
        expected_path = (
            evidence_path.resolve().parent / 'scan_gate_evidence.json')
        if (path.resolve() != expected_path
                or not path.is_file()
                or path.stat().st_size != record['size_bytes']
                or sha256_file(path) != record['sha256']):
            return None
        observation = strict_json_loads(path.read_text())
        if set(observation) != {
                'schema_version', 'run_id', 'scenario', 'seed',
                'contract_sha256', 'stamp_ns', 'publish_steady_ns',
                'zero_receive_steady_ns',
                'scan_publish_to_zero_receive_steady_s', 'frame_id',
                'angle_min_rad', 'angle_increment_rad', 'ranges_m',
                'stop_zone_points', 'source_path', 'source_size_bytes',
                'source_sha256', 'arm_receive_steady_ns'}:
            return None
        source = Path(observation['source_path'])
        expected_source = (
            Path(__file__).resolve().parent.parent
            / 'jdamr_cube_navigation' / 'sim_scan_gate.py').resolve()
        ranges = observation['ranges_m']
        angle_min = observation['angle_min_rad']
        increment = observation['angle_increment_rad']
        integer_fields_valid = (
            _exact_int(observation['schema_version'], positive=True)
            and _exact_int(observation['seed'])
            and _exact_int(observation['source_size_bytes'], positive=True)
            and _exact_int(observation['stamp_ns'])
            and _exact_int(observation['publish_steady_ns'], positive=True)
            and _exact_int(
                observation['zero_receive_steady_ns'], positive=True)
            and _exact_int(
                observation['arm_receive_steady_ns'], positive=True))
        if (not integer_fields_valid or not isinstance(ranges, list)
                or not _finite_number(angle_min)
                or not _finite_number(increment)
                or not isinstance(observation['stop_zone_points'], list)):
            return None
        zone = contract['stop_zone']
        transform = contract['scan_to_base_transform']
        cosine = math.cos(transform['yaw_rad'])
        sine = math.sin(transform['yaw_rad'])
        points = []
        for index, range_m in enumerate(ranges):
            if range_m is None:
                continue
            if not _finite_number(range_m):
                return None
            angle = angle_min + index * increment
            scan_x_m = range_m * math.cos(angle)
            scan_y_m = range_m * math.sin(angle)
            x_m = transform['x_m'] + cosine * scan_x_m - sine * scan_y_m
            y_m = transform['y_m'] + sine * scan_x_m + cosine * scan_y_m
            if (zone['rear_m'] <= x_m <= zone['front_m']
                    and abs(y_m) <= zone['half_width_m']):
                points.append([x_m, y_m])
        recorded = observation['stop_zone_points']
        if any(
                not isinstance(point, list) or len(point) != 2
                or not all(_finite_number(value) for value in point)
                for point in recorded):
            return None
        source_record = document.get('scan_gate_source', {})
        valid = (
            observation['schema_version'] == 1
            and observation['run_id'] == document['run_id']
            and observation['scenario'] == document['scenario']
            and observation['seed'] == document['seed']
            and observation['contract_sha256']
            == document['canonical_contract_sha256']
            and source.resolve() == expected_source
            and set(source_record) == {'path', 'size_bytes', 'sha256'}
            and Path(source_record['path']).resolve() == expected_source
            and source_record['size_bytes'] == observation['source_size_bytes']
            and source_record['sha256'] == observation['source_sha256']
            and source.is_file()
            and source.stat().st_size == observation['source_size_bytes']
            and sha256_file(source) == observation['source_sha256']
            and observation['frame_id'] == transform['source_frame']
            and len(points) >= 3 and len(recorded) == len(points)
            and all(math.dist(left, right) <= 1e-12
                    for left, right in zip(points, recorded))
            and observation['zero_receive_steady_ns']
            > observation['publish_steady_ns']
            and _matches(
                observation['scan_publish_to_zero_receive_steady_s'],
                (observation['zero_receive_steady_ns']
                 - observation['publish_steady_ns']) / 1e9)
            and 0.0 < observation[
                'scan_publish_to_zero_receive_steady_s']
            <= contract['sudden_stop_deadline_s']
            and document['scan_gate_arm_request_steady_ns']
            < observation['arm_receive_steady_ns']
            <= document['scan_gate_arm_ack_steady_ns']
            < document['obstacle_request_steady_ns']
            < observation['publish_steady_ns'])
        return observation if valid else None
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        return None


def _preparation_identity_valid(
        root: Path, contract_path: Path,
        expected_mode: str = 'matrix_evaluation',
        expected_planned: list[dict[str, Any]] | None = None,
) -> bool:
    """Verify the exact prepared asset set and immutable root contract."""
    try:
        manifest_path = root / 'preparation_manifest.json'
        runtime_path = root / 'runtime_manifest.json'
        manifest = strict_json_loads(manifest_path.read_text())
        runtime = strict_json_loads(runtime_path.read_text())
        contract = strict_json_loads(contract_path.read_text())
        expected = {
            'contract.json', 'collision_monitor_overlay.yaml',
            'nav2_collision_monitor_eval.params.yaml',
            'jdamr_cube_collision_monitor_eval.urdf',
            'slam_corridor_eval.pgm', 'slam_corridor_eval.yaml',
            'slam_corridor_contact.world', 'collision_monitor_bridge.yaml'}
        records = manifest['records']
        names = [record['relative_path'] for record in records]
        if (set(manifest) != {
                'schema_version', 'prepared_root',
                'expected_generated_paths', 'records'}
                or set(manifest['expected_generated_paths']) != expected
                or len(names) != len(expected) or set(names) != expected
                or set(runtime) != {
                    'evaluation_rmw', 'canonical_contract_sha256',
                    'preparation_manifest_sha256', 'domain_id_base',
                    'domain_ids', 'execution_mode', 'planned_runs',
                    'harness_sources'}
                or runtime['preparation_manifest_sha256']
                != sha256_file(manifest_path)
                or runtime['canonical_contract_sha256']
                != sha256_file(contract_path)):
            return False
        evaluation = Path(__file__).resolve().parent
        package = evaluation.parent / 'jdamr_cube_navigation'
        expected_sources = {
            'contract': evaluation / 'sim_collision_monitor_contract.py',
            'prepare': evaluation / 'prepare_sim_collision_monitor_run.py',
            'runner': evaluation / 'run_sim_collision_monitor_eval.py',
            'evaluator': evaluation / 'evaluate_sim_collision_monitor.py',
            'finalizer': evaluation / 'finalize_sim_collision_monitor_eval.py',
            'resource_sampler': (
                evaluation / 'sample_process_group_resources.py'),
            'scenario': package / 'sim_collision_monitor_scenario.py',
            'scan_gate': package / 'sim_scan_gate.py',
        }
        sources = runtime['harness_sources']
        if set(sources) != set(expected_sources):
            return False
        for name, expected_source in expected_sources.items():
            record = sources[name]
            retained = root / 'runtime_sources' / f'{name}.py'
            if (set(record) != {
                    'path', 'size_bytes', 'sha256', 'retained_path',
                    'retained_size_bytes', 'retained_sha256'}
                    or Path(record['path']).resolve()
                    != expected_source.resolve()
                    or not expected_source.is_file()
                    or record['size_bytes'] != expected_source.stat().st_size
                    or record['sha256'] != sha256_file(expected_source)
                    or Path(record['retained_path']).resolve()
                    != retained.resolve()
                    or not retained.is_file()
                    or record['retained_size_bytes'] != retained.stat().st_size
                    or record['retained_sha256'] != sha256_file(retained)
                    or record['retained_sha256'] != record['sha256']):
                return False
        rmw = runtime['evaluation_rmw']
        environment_fields = rmw.get('environment_fields', {})
        expected_rmw_keys = {
            'requested_identifier', 'actual_identifier', 'version',
            'source', 'prefix', 'library_path', 'library_size_bytes',
            'library_sha256', 'environment_sha256', 'environment_fields',
            'domain_id', 'retained_library_path',
            'retained_library_size_bytes', 'retained_library_sha256'}
        library = Path(rmw.get('library_path', ''))
        retained_library = root / 'runtime_dependencies' / library.name
        prefix = Path(rmw.get('prefix', '')).resolve()
        expected_environment_keys = {
            'AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PATH',
            'RMW_IMPLEMENTATION', 'ROS_AUTOMATIC_DISCOVERY_RANGE'}
        allowed_sources = {
            'rmw_cyclonedds_cpp': 'temporary_deb_extract',
            'rmw_fastrtps_cpp': 'system_ros_installation'}
        environment_sha256 = hashlib.sha256(json.dumps(
            environment_fields, sort_keys=True).encode()).hexdigest()
        domain_base = runtime.get('domain_id_base')
        if expected_planned is None:
            expected_planned = [
                {'run_id': item['run_id'],
                 'domain_id': domain_base + index}
                for index, item in enumerate(scenario_matrix())]
        expected_domains = [record['domain_id']
                            for record in expected_planned]
        if (set(rmw) != expected_rmw_keys
                or rmw.get('requested_identifier')
                != rmw.get('actual_identifier')
                or allowed_sources.get(rmw.get('actual_identifier'))
                != rmw.get('source')
                or set(environment_fields) != expected_environment_keys
                or environment_fields.get('RMW_IMPLEMENTATION')
                != rmw.get('actual_identifier')
                or environment_fields.get('ROS_AUTOMATIC_DISCOVERY_RANGE')
                != 'LOCALHOST'
                or rmw.get('environment_sha256') != environment_sha256
                or not library.is_file()
                or prefix not in library.resolve().parents
                or library.stat().st_size != rmw.get('library_size_bytes')
                or sha256_file(library) != rmw.get('library_sha256')
                or Path(rmw.get('retained_library_path', '')).resolve()
                != retained_library.resolve()
                or not retained_library.is_file()
                or retained_library.stat().st_size
                != rmw.get('retained_library_size_bytes')
                or sha256_file(retained_library)
                != rmw.get('retained_library_sha256')
                or rmw.get('retained_library_sha256')
                != rmw.get('library_sha256')
                or not _exact_int(domain_base)
                or rmw.get('domain_id') != domain_base
                or runtime.get('execution_mode') != expected_mode
                or runtime.get('domain_ids') != expected_domains
                or runtime.get('planned_runs') != expected_planned
                or not all(
                    _exact_int(value) and value <= 232
                    for value in runtime.get('domain_ids', []))
                or len(set(runtime.get('domain_ids', [])))
                != len(runtime.get('domain_ids', []))
                or not all(
                    _exact_int(record.get('domain_id'))
                    for record in runtime.get('planned_runs', []))
                or domain_base < 0 or domain_base + 14 > 232
                or 12 in runtime.get('domain_ids', [])):
            return False
        prepared_root = Path(manifest['prepared_root']).resolve()
        if prepared_root != (root / 'assets').resolve():
            return False
        expected_asset_files = expected - {'contract.json'}
        actual_asset_files = {
            str(path.relative_to(prepared_root))
            for path in prepared_root.rglob('*') if path.is_file()}
        if actual_asset_files != expected_asset_files:
            return False
        production = contract.get('production_inputs', {})
        if set(production) != {
                'production_params', 'navigation_launch',
                'onboard_nav2_core_launch', 'production_urdf'}:
            return False
        for record in production.values():
            source = Path(record.get('path', ''))
            if (set(record) != {'path', 'size_bytes', 'sha256'}
                    or not source.is_file()
                    or source.stat().st_size != record['size_bytes']
                    or sha256_file(source) != record['sha256']):
                return False
        if not _production_derivation_valid(contract):
            return False
        scan_gap = contract.get('scan_gap_provenance', {})
        scan_profile_path = Path(scan_gap.get('path', ''))
        if (set(scan_gap) != {
                'path', 'size_bytes', 'sha256', 'json_pointer', 'value_s',
                'observed_on', 'valid_for'}
                or scan_gap.get('json_pointer')
                != '/timing/scan/header_interval_s/max'
                or not scan_profile_path.is_file()
                or scan_profile_path.stat().st_size
                != scan_gap.get('size_bytes')
                or sha256_file(scan_profile_path) != scan_gap.get('sha256')):
            return False
        scan_profile = strict_json_loads(scan_profile_path.read_text())
        observed_gap_s = scan_profile['timing']['scan'][
            'header_interval_s']['max']
        if (not _matches(scan_gap.get('value_s'), observed_gap_s)
                or not _matches(
                    contract['stop_zone']['inputs']['max_scan_gap_s'],
                    observed_gap_s)):
            return False
        asset_names = {
            'urdf': 'jdamr_cube_collision_monitor_eval.urdf',
            'collision_monitor_overlay': 'collision_monitor_overlay.yaml',
            'nav2_evaluation_params': (
                'nav2_collision_monitor_eval.params.yaml'),
            'slam_corridor_eval.pgm': 'slam_corridor_eval.pgm',
            'slam_corridor_eval.yaml': 'slam_corridor_eval.yaml',
            'world': 'slam_corridor_contact.world',
            'bridge': 'collision_monitor_bridge.yaml',
        }
        assets = contract.get('evaluation_assets', {})
        if set(assets) != set(asset_names):
            return False
        for key, name in asset_names.items():
            record = assets[key]
            output = Path(record.get('path', ''))
            source = Path(record.get('source_path', ''))
            if (output.resolve() != prepared_root / name
                    or not output.is_file()
                    or output.stat().st_size != record.get('size_bytes')
                    or sha256_file(output) != record.get('sha256')
                    or not source.is_file()
                    or source.stat().st_size
                    != record.get('source_size_bytes')
                    or sha256_file(source) != record.get('source_sha256')):
                return False
        for record in records:
            asset = (
                contract_path
                if record['relative_path'] == 'contract.json'
                else prepared_root / record['relative_path'])
            if (not asset.is_file()
                    or asset.stat().st_size != record['size_bytes']
                    or sha256_file(asset) != record['sha256']):
                return False
        return True
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        return False


def _teardown_evidence_valid(
        document: dict[str, Any], evidence_path: Path | None) -> bool:
    """Recompute whether every process ended only in the cleanup window."""
    teardown = document.get('teardown')
    if not isinstance(teardown, dict) or set(teardown) != {
            'measurement_finalized_steady_ns', 'processes', 'errors',
            'status'}:
        return False
    finalized = teardown['measurement_finalized_steady_ns']
    processes = teardown['processes']
    terminal = [event for event in document.get('events', [])
                if event.get('name') == document.get('action_terminal')]
    expected = {
        'gazebo', 'scan_gate', 'contact_bridge', 'navigation', 'scenario',
        'gazebo_resource_sampler', 'nav2_resource_sampler'}
    if document.get('representative_bag_recorded') is True:
        expected.add('recorder')
    identity = document.get('process_identity')
    if (not _exact_int(finalized, positive=True)
            or len(terminal) != 1
            or not _exact_int(terminal[0].get('steady_ns'), positive=True)
            or terminal[0]['steady_ns'] > finalized
            or teardown['errors'] != [] or teardown['status'] != 'PASS'
            or not isinstance(processes, list)
            or {record.get('name') for record in processes} != expected
            or len(processes) != len(expected)
            or not isinstance(identity, list)
            or len(identity) != len(expected)
            or any(not isinstance(item, dict)
                   or set(item) != {'name', 'pid'}
                   or item.get('name') not in expected
                   or not _exact_int(item.get('pid'), positive=True)
                   for item in identity)
            or len({item['name'] for item in identity}) != len(expected)
            or len({item['pid'] for item in identity}) != len(expected)
            or {(record['name'], record['pid']) for record in processes}
            != {(item['name'], item['pid']) for item in identity}):
        return False
    launched_names = {
        'gazebo', 'scan_gate', 'contact_bridge', 'navigation', 'scenario'}
    launched_pids = [item['pid'] for item in identity
                     if item['name'] in launched_names]
    if document.get('identity_process_groups') != launched_pids:
        return False
    for record in processes:
        if not isinstance(record, dict) or set(record) != {
                'name', 'pid', 'returncode_at_measurement',
                'returncode_before_stop', 'runner_initiated',
                'requested_signal', 'stop_requested_steady_ns',
                'stop_completed_steady_ns', 'returncode_after_stop',
                'logged_process_exits_before_stop',
                'logged_process_exits_after_stop', 'log_path',
                'log_size_bytes', 'log_sha256',
                'log_pre_stop_offset_bytes'}:
            return False
        name = record['name']
        before = record['returncode_before_stop']
        after = record['returncode_after_stop']
        initiated = record['runner_initiated']
        if (not _exact_int(record['pid'], positive=True)
                or type(initiated) is not bool
                or not _exact_int(record['stop_requested_steady_ns'],
                                  positive=True)
                or not _exact_int(record['stop_completed_steady_ns'],
                                  positive=True)
                or record['stop_requested_steady_ns'] < finalized
                or record['stop_completed_steady_ns']
                < record['stop_requested_steady_ns']
                or record['logged_process_exits_before_stop'] != []
                or not isinstance(
                    record['logged_process_exits_after_stop'], list)):
            return False
        if (name == 'scenario'
                and record['returncode_at_measurement'] != 0):
            return False
        if (name != 'scenario'
                and record['returncode_at_measurement'] is not None):
            return False
        if evidence_path is not None:
            log_path = Path(record.get('log_path', ''))
            expected_log = evidence_path.resolve().parent / f'{name}.log'
            if name.endswith('_resource_sampler'):
                expected_log = evidence_path.resolve().parent / f'{name}.log'
            if (log_path != expected_log
                    or not log_path.is_file() or log_path.is_symlink()
                    or log_path.stat().st_size != record['log_size_bytes']
                    or sha256_file(log_path) != record['log_sha256']
                    or not _exact_int(record['log_pre_stop_offset_bytes'])
                    or record['log_pre_stop_offset_bytes']
                    > record['log_size_bytes']):
                return False
            content = log_path.read_bytes()
            log_valid, matches = _logged_process_exits(content)
            if not log_valid:
                return False
            before_records = [item for item in matches if item['byte_offset']
                              < record['log_pre_stop_offset_bytes']]
            after_records = [item for item in matches if item['byte_offset']
                             >= record['log_pre_stop_offset_bytes']]
            if (before_records
                    != record['logged_process_exits_before_stop']
                    or after_records
                    != record['logged_process_exits_after_stop']):
                return False
        if initiated:
            if (record['requested_signal'] != 2
                    or before is not None or type(after) is not int):
                return False
            allowed = {0, -2, 130, -15, 143}
            if name in {'navigation', 'contact_bridge'}:
                allowed.add(-6)
            if after not in allowed:
                return False
        else:
            if (record['requested_signal'] is not None
                    or not _exact_int(before) or before != 0 or after != 0
                    or (name != 'scenario'
                        and not name.endswith('_resource_sampler'))
                    or record['logged_process_exits_after_stop']):
                return False
        if initiated and not _logged_process_exits_allowed(
                name, record['logged_process_exits_after_stop']):
            return False
    return True


def evaluate_run(
        document: dict[str, Any], evidence_path: Path | None = None,
        canonical_contract: dict[str, Any] | None = None,
        canonical_contract_sha256: str | None = None,
        representative_run: bool = False,
) -> dict[str, Any]:
    """Evaluate common and scenario-specific G004 evidence gates."""
    scenario = document['scenario']
    contract = document['contract']
    canonical = (
        canonical_contract if canonical_contract is not None else contract)
    canonical_sha256 = (
        canonical_contract_sha256
        if canonical_contract_sha256 is not None
        else document.get('canonical_contract_sha256'))
    resources = document.get('resource_evidence', {})
    resource_values = [
        record.get(field)
        for record in resources.values()
        for field in ('median_cpu_pct_one_core', 'p95_cpu_pct_one_core',
                      'max_rss_mb')]
    run_dir = evidence_path.resolve().parent if evidence_path else None
    resource_identity_valid = (
        set(resources) == {'gazebo', 'nav2'}
        and all(_resource_record_valid(name, record, run_dir)
                for name, record in resources.items()))
    storage = document.get('storage_preflight', {})
    excluded = set(document.get('retention_excluded_paths', []))
    retained_size_valid = (
        run_dir is not None
        and excluded == {'evidence.json', 'evaluation.json'}
        and document.get('retained_run_bytes')
        == _tree_size_excluding(run_dir, excluded))
    event_contract = {
        'clear_baseline': ['goal_accepted', 'succeeded'],
        'sudden_obstacle_stop_resume': [
            'goal_accepted', 'scan_gate_arm_requested',
            'scan_gate_arm_ack', 'obstacle_set_pose_requested',
            'stop_state', 'final_zero', 'physical_stop',
            'clear_set_pose_requested',
            'do_nothing_after_stop', 'succeeded'],
        'scan_timeout_stop_resume': [
            'goal_accepted', 'monitor_scan_frozen', 'stop_state',
            'final_zero', 'physical_stop',
            'monitor_scan_unfreeze_requested',
            'do_nothing_after_stop', 'succeeded'],
    }
    events = document.get('events', [])
    goal_events = [event for event in events
                   if event.get('name') == 'goal_accepted']
    goal_entry_events = [event for event in events
                         if event.get('name')
                         == 'goal_position_tolerance_entered']
    stop_events = [event for event in events
                   if event.get('name') == 'stop_state']
    terminal_events = [event for event in events
                       if event.get('name') == document.get('action_terminal')]
    scan_gate_document = _scan_gate_evidence_document(
        document, contract, evidence_path)
    scan_gate_source = document.get('scan_gate_source', {})
    expected_scan_gate_source = (
        Path(__file__).resolve().parent.parent / 'jdamr_cube_navigation'
        / 'sim_scan_gate.py').resolve()
    expected_scenario_source = (
        Path(__file__).resolve().parent.parent / 'jdamr_cube_navigation'
        / 'sim_collision_monitor_scenario.py').resolve()
    scenario_source = document.get('scenario_source', {})
    goal_entry = document.get('goal_position_tolerance_entry')
    goal = [contract['goal_pose']['x_m'], contract['goal_pose']['y_m']]
    goal_entry_valid = goal_entry is None and not goal_entry_events
    if (isinstance(goal_entry, dict) and len(goal_entry_events) == 1
            and goal_entry == goal_entry_events[0]):
        pose = goal_entry.get('pose_m')
        entry_goal = goal_entry.get('goal_pose_m')
        try:
            import tf2_py
            from geometry_msgs.msg import TransformStamped
            from rclpy.time import Time
            inputs = goal_entry['tf_inputs']
            replay = tf2_py.BufferCore()
            inputs_valid = isinstance(inputs, list) and 0 < len(inputs) <= 128
            input_identities = set()
            edge_records: dict[tuple[str, str], list[dict[str, Any]]] = {}
            for item in inputs:
                inputs_valid = inputs_valid and (
                    set(item) == {'parent', 'child', 'stamp_ns',
                                  'translation', 'rotation', 'is_static'}
                    and isinstance(item['parent'], str) and item['parent']
                    and isinstance(item['child'], str) and item['child']
                    and _exact_int(item['stamp_ns'])
                    and isinstance(item['translation'], list)
                    and len(item['translation']) == 3
                    and isinstance(item['rotation'], list)
                    and len(item['rotation']) == 4
                    and all(_finite_number(value) for value in (
                        item['translation'] + item['rotation']))
                    and math.isclose(sum(value * value for value in
                                         item['rotation']), 1.0,
                                     rel_tol=0.0, abs_tol=1e-6)
                    and type(item['is_static']) is bool)
                if not inputs_valid:
                    break
                identity = json.dumps(item, sort_keys=True)
                inputs_valid = identity not in input_identities
                input_identities.add(identity)
                edge_records.setdefault(
                    (item['parent'], item['child']), []).append(item)
                transform = TransformStamped()
                transform.header.frame_id = item['parent']
                transform.child_frame_id = item['child']
                transform.header.stamp = Time(
                    nanoseconds=item['stamp_ns']).to_msg()
                transform.transform.translation.x = item['translation'][0]
                transform.transform.translation.y = item['translation'][1]
                transform.transform.translation.z = item['translation'][2]
                transform.transform.rotation.x = item['rotation'][0]
                transform.transform.rotation.y = item['rotation'][1]
                transform.transform.rotation.z = item['rotation'][2]
                transform.transform.rotation.w = item['rotation'][3]
                if item['is_static']:
                    replay.set_transform_static(transform, 'g004_evaluator')
                else:
                    replay.set_transform(transform, 'g004_evaluator')
            graph: dict[str, list[str]] = {}
            for parent, child in edge_records:
                graph.setdefault(parent, []).append(child)
            paths = []

            def visit(frame, path, seen):
                if frame == 'base_link':
                    paths.append(path)
                    return
                for child in graph.get(frame, []):
                    if child not in seen:
                        visit(child, path + [(frame, child)], seen | {child})

            visit('map', [], {'map'})
            inputs_valid = inputs_valid and len(paths) == 1 and (
                set(paths[0]) == set(edge_records))
            inputs_valid = inputs_valid and (
                _canonical_tf_subset(
                    inputs, goal_entry['transform_stamp_ns']) == inputs)
            if inputs_valid:
                for records in edge_records.values():
                    static = [item for item in records if item['is_static']]
                    dynamic = [item for item in records
                               if not item['is_static']]
                    exact = [item for item in dynamic if item['stamp_ns']
                             == goal_entry['transform_stamp_ns']]
                    before = [item for item in dynamic if item['stamp_ns']
                              < goal_entry['transform_stamp_ns']]
                    after = [item for item in dynamic if item['stamp_ns']
                             > goal_entry['transform_stamp_ns']]
                    inputs_valid = inputs_valid and (
                        (len(static) == 1 and not dynamic)
                        or (not static and len(exact) == 1
                            and len(dynamic) == 1)
                        or (not static and not exact and len(dynamic) == 2
                            and len(before) == len(after) == 1))
            replayed = replay.lookup_transform_core(
                'map', 'base_link', Time(
                    nanoseconds=goal_entry['transform_stamp_ns']).to_msg())
            replay_pose = [replayed.transform.translation.x,
                           replayed.transform.translation.y]
            goal_entry_valid = (
                set(goal_entry) == {
                    'name', 'steady_ns', 'ros_ns', 'goal_uuid', 'frame_id',
                    'child_frame_id', 'pose_m', 'goal_pose_m',
                    'transform_stamp_ns',
                    'transform_observer_ros_ns',
                    'transform_stamp_offset_s', 'position_error_m',
                    'tf_inputs'}
                and inputs_valid
                and math.dist(replay_pose, pose) <= 1e-6
                and goal_entry['goal_uuid'] == document.get('goal_uuid')
                and goal_entry['frame_id'] == 'map'
                and goal_entry['child_frame_id'] == 'base_link'
                and _exact_int(goal_entry['steady_ns'], positive=True)
                and _exact_int(goal_entry['ros_ns'], positive=True)
                and isinstance(pose, list) and len(pose) == 2
                and all(_finite_number(value) for value in pose)
                and entry_goal == goal
                and _exact_int(goal_entry['transform_stamp_ns'], positive=True)
                and _exact_int(
                    goal_entry['transform_observer_ros_ns'], positive=True)
                and goal_entry['transform_stamp_ns']
                <= goal_entry['transform_observer_ros_ns']
                <= goal_entry['ros_ns']
                and _matches(
                    goal_entry['transform_stamp_offset_s'],
                    (goal_entry['transform_stamp_ns']
                     - goal_entry['transform_observer_ros_ns']) / 1e9)
                and abs(goal_entry['transform_stamp_offset_s'])
                <= contract['goal_verification'][
                    'estimate_stamp_tolerance_s']
                and _matches(goal_entry['position_error_m'],
                             math.dist(pose, goal))
                and 0.0 <= goal_entry['position_error_m']
                and (goal_entry['position_error_m'] <= contract[
                    'goal_verification']['position_tolerance_m']
                    or math.isclose(
                        goal_entry['position_error_m'], contract[
                            'goal_verification']['position_tolerance_m'],
                        rel_tol=0.0, abs_tol=1e-12))
                and len(goal_events) == len(terminal_events) == 1
                and goal_events[0]['steady_ns'] < goal_entry['steady_ns']
                < terminal_events[0]['steady_ns']
                and goal_events[0]['ros_ns'] < goal_entry['ros_ns']
                < terminal_events[0]['ros_ns'])
        except (KeyError, RuntimeError, TypeError, ValueError,
                tf2_py.TransformException):
            goal_entry_valid = False
    runtime_scenario_source = scenario_source
    runtime_domain_valid = True
    if evidence_path is not None:
        runtime_path = (
            evidence_path.resolve().parent.parent / 'runtime_manifest.json')
        try:
            if runtime_path.is_file():
                runtime = strict_json_loads(runtime_path.read_text())
                runtime_scenario_source = runtime.get(
                    'harness_sources', {}).get('scenario', {})
                planned = runtime.get('planned_runs', [])
                runtime_domain_valid = (
                    isinstance(planned, list)
                    and _exact_int(document.get('domain_id'))
                    and sum(record.get('run_id') == document.get('run_id')
                            for record in planned) == 1
                    and any(
                        record == {
                            'run_id': document.get('run_id'),
                            'domain_id': document.get('domain_id')}
                        for record in planned))
        except (OSError, ValueError, TypeError):
            runtime_scenario_source = {}
            runtime_domain_valid = False
    checks = {
        'activation_error_absent': document.get('activation_error') is None,
        'canonical_contract_match': (
            contract == canonical
            and document.get('canonical_contract_sha256')
            == canonical_sha256),
        'required_events_ordered': _ordered_unique_events(
            document, event_contract[scenario]),
        'event_scalar_bindings': (
            scenario == 'clear_baseline'
            or _event_scalar_bindings_valid(document)),
        'provenance_partial_order': _provenance_partial_order_valid(document),
        'goal_event_uuid_match': (
            len(goal_events) == 1
            and goal_events[0].get('goal_uuid') == document.get('goal_uuid')),
        'entity_state_truth': _entity_states_valid(document, contract),
        'clearance_evidence': _clearance_evidence_valid(document, contract),
        'scan_gate_evidence_scope': (
            scan_gate_document is not None
            if scenario == 'sudden_obstacle_stop_resume'
            else document.get('scan_gate_evidence') is None),
        'scan_gate_source_identity': (
            set(scan_gate_source) == {'path', 'size_bytes', 'sha256'}
            and Path(scan_gate_source.get('path', '')).resolve()
            == expected_scan_gate_source
            and expected_scan_gate_source.is_file()
            and scan_gate_source.get('size_bytes')
            == expected_scan_gate_source.stat().st_size
            and scan_gate_source.get('sha256')
            == sha256_file(expected_scan_gate_source)),
        'scenario_source_identity': (
            set(scenario_source) == {'path', 'size_bytes', 'sha256'}
            and Path(scenario_source.get('path', '')).resolve()
            == expected_scenario_source
            and expected_scenario_source.is_file()
            and scenario_source.get('size_bytes')
            == expected_scenario_source.stat().st_size
            and scenario_source.get('sha256')
            == sha256_file(expected_scenario_source)
            and runtime_scenario_source.get('path')
            == scenario_source.get('path')
            and runtime_scenario_source.get('size_bytes')
            == scenario_source.get('size_bytes')
            and runtime_scenario_source.get('sha256')
            == scenario_source.get('sha256')),
        'runtime_domain_binding': runtime_domain_valid,
        'world_start_pose': (
            isinstance(document.get('initial_world_pose_m'), list)
            and abs(document['initial_world_pose_m'][0]
                    - contract['start_pose']['x_m']) <= 0.10
            and abs(document['initial_world_pose_m'][1]
                    - contract['start_pose']['y_m']) <= 0.10),
        'single_goal_uuid': (
            isinstance(document.get('goal_uuid'), str)
            and len(document['goal_uuid']) == 32
            and _exact_int(document.get('goal_send_count'), positive=True)
            and document['goal_send_count'] == 1),
        'goal_not_cancelled': (
            _exact_int(document.get('goal_cancel_count'))
            and document['goal_cancel_count'] == 0),
        'raw_scan_continues': document.get('raw_scan_continued') is True,
        'navigation_scan_continues': (
            document.get('navigation_scan_continued') is True),
        'contact_zero': (
            _exact_int(document.get('contact_count'))
            and document['contact_count'] == 0),
        'contact_source_connected': (
            _exact_int(document.get(
                'contact_matched_publisher_count_max'), positive=True)),
        'contact_subscription_created': (
            document.get('contact_subscription_created') is True),
        'final_zero_hold': (
            document.get('final_cmd_vel_zero') is True
            and document.get('final_zero_hold_s', -1.0)
            >= contract['final_zero_hold_s']),
        'survivor_zero': document.get('identity_survivors') == [],
        'retention_within_limits': (
            document.get('retained_run_bytes', -1)
            <= contract['retention']['total_limit_bytes']
            and document.get('full_matrix_bag_recorded') is False
            and document.get('representative_bag_recorded')
            is representative_run),
        'full_matrix_bag_artifacts_absent': (
            (representative_run
             and isinstance(document.get('representative_bag'), dict))
            or (not representative_run and _bag_artifacts_absent(run_dir))),
        'retained_size_recomputed': retained_size_valid,
        'production_hashes_unchanged': (
            document.get('production_hashes_unchanged') is True),
        'harness_error_absent': document.get('harness_error') is None,
        'teardown_evidence': _teardown_evidence_valid(
            document, evidence_path),
        'resource_evidence_complete': (
            set(resources) == {'gazebo', 'nav2'}
            and all(record.get('sample_count', 0) >= 2
                    for record in resources.values())
            and all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in resource_values)),
        'resource_files_match_evidence': (
            set(resources) == {'gazebo', 'nav2'}
            and resource_identity_valid),
        'home_storage_preflight': (
            storage.get('passed') is True
            and storage.get('home_free_bytes', -1)
            >= storage.get('required_free_bytes', 0)
            and storage.get('predicted_run_bytes', -1) > 0),
        'same_goal_succeeded': (
            document.get('action_terminal') == 'succeeded'
            and document.get('terminal_goal_uuid')
            == document.get('goal_uuid')
            and len(terminal_events) == 1
            and terminal_events[0] is events[-1]),
        'goal_position_tolerance_entry': goal_entry_valid,
        'terminal_estimated_pose_diagnostic': (
            document.get('final_estimated_pose_frame_id') == 'map'
            and isinstance(document.get('final_estimated_pose_m'), list)
            and len(document['final_estimated_pose_m']) == 2
            and all(_finite_number(value)
                    for value in document['final_estimated_pose_m'])),
        'estimated_pose_fresh_at_terminal': (
            len(terminal_events) == 1
            and document.get('final_estimated_pose_observed_ros_ns')
            == terminal_events[0].get('ros_ns')
            and _exact_int(document.get('final_estimated_pose_stamp_ns'))
            and _exact_int(document.get(
                'final_estimated_pose_observed_ros_ns'))
            and _matches(
                document.get('estimated_pose_stamp_offset_at_terminal_s'),
                _difference_s(
                    document, 'final_estimated_pose_stamp_ns',
                    'final_estimated_pose_observed_ros_ns'))
            and abs(document.get(
                'estimated_pose_stamp_offset_at_terminal_s', math.inf))
            <= contract['goal_verification'][
                'estimate_stamp_tolerance_s']),
        'ground_truth_position_diagnostic': (
            len(terminal_events) == 1
            and isinstance(document.get('final_world_pose_m'), list)
            and len(document['final_world_pose_m']) == 2
            and all(_finite_number(value)
                    for value in document['final_world_pose_m'])
            and isinstance(document.get(
                'terminal_ground_truth_pose_m'), list)
            and len(document['terminal_ground_truth_pose_m']) == 2
            and all(_finite_number(value)
                    for value in document['terminal_ground_truth_pose_m'])
            and document.get('terminal_ground_truth_observed_ros_ns')
            == terminal_events[0].get('ros_ns')
            and _matches(document.get(
                'terminal_ground_truth_goal_error_m'),
                         math.dist(document[
                             'terminal_ground_truth_pose_m'], [
                             contract['goal_pose']['x_m'],
                             contract['goal_pose']['y_m']]))
            and _matches(document.get(
                'terminal_estimated_to_gt_error_m'),
                         math.dist(document['terminal_ground_truth_pose_m'],
                                   document['final_estimated_pose_m']))),
        'clock_evidence': (
            _exact_int(document.get('ros_start_ns'))
            and _exact_int(document.get('ros_end_ns'))
            and _exact_int(document.get('steady_start_ns'))
            and _exact_int(document.get('steady_end_ns'))
            and document.get('ros_end_ns', -1)
            > document.get('ros_start_ns', -1)
            and document.get('steady_end_ns', -1)
            > document.get('steady_start_ns', -1)),
    }
    if scenario == 'clear_baseline':
        checks['state_clear'] = (
            _exact_int(document.get('stop_state_count'))
            and document['stop_state_count'] == 0
            and _exact_int(document.get('final_action_type'))
            and document['final_action_type'] == 0)
    else:
        if scenario == 'sudden_obstacle_stop_resume':
            stop_latency_steady_s = (
                scan_gate_document.get(
                    'scan_publish_to_zero_receive_steady_s')
                if scan_gate_document is not None else None)
            stop_latency_ros_s = None
        else:
            stop_latency_ros_s = document.get('freeze_ack_to_zero_ros_s')
            stop_latency_steady_s = document.get(
                'freeze_ack_to_zero_steady_s')
        stop_distance_m = document.get('stop_distance_m')
        deadline_s = (
            contract['sudden_stop_deadline_s']
            if scenario == 'sudden_obstacle_stop_resume'
            else contract['timeout_stop_deadline_s'])
        checks.update({
            'stop_observed': (
                _exact_int(document.get('stop_state_count'), positive=True)
                and document['stop_state_count'] == 1
                and _exact_int(document.get('stop_action_type'))
                and document['stop_action_type'] == 1),
            'stopped_within_deadline': (
                _finite_number(stop_latency_steady_s)
                and 0.0 < stop_latency_steady_s <= deadline_s),
            'ros_latency_observed': (
                scenario == 'sudden_obstacle_stop_resume'
                or (_finite_number(stop_latency_ros_s)
                    and 0.0 < stop_latency_ros_s <= deadline_s)),
            'steady_latency_observed': (
                _finite_number(stop_latency_steady_s)
                and 0.0 < stop_latency_steady_s <= deadline_s),
            'physical_stop': document.get('physical_stop_observed') is True,
            'bounded_stop_distance': (
                _finite_number(stop_distance_m)
                and stop_distance_m >= 0.0
                and stop_distance_m
                <= contract['maximum_stop_distance_m']),
            'scan_range_diagnostic': (
                _finite_number(document.get(
                    'minimum_observed_scan_range_m'))
                and document['minimum_observed_scan_range_m'] > 0.0),
            'resume_clear': (
                _exact_int(document.get('resume_action_type'))
                and document['resume_action_type'] == 0
                and _exact_int(document.get('final_action_type'))
                and document['final_action_type'] == 0),
            'strictly_newer_clear_scan': (
                document.get('clear_scan_stamp_ns', -1)
                > document.get('clear_reference_scan_stamp_ns', -1)),
        })
    if scenario == 'sudden_obstacle_stop_resume':
        checks['positive_clearance'] = (
            _clearance_evidence_valid(document, contract))
        checks['obstacle_bearing_trigger_scan'] = (
            scan_gate_document is not None)
        checks['stop_event_polygon_match'] = (
            len(stop_events) == 1
            and stop_events[0].get('polygon') == 'StopZone'
            and stop_events[0].get('polygon')
            == document.get('stop_polygon_name'))
        checks['stop_source_is_stop_zone'] = (
            document.get('stop_polygon_name') == 'StopZone')
        checks['strictly_newer_trigger_scan'] = (
            scan_gate_document is not None
            and _exact_int(scan_gate_document.get('stamp_ns'))
            and _finite_number(document.get('reference_scan_stamp_ns'))
            and scan_gate_document['stamp_ns']
            > document['reference_scan_stamp_ns'])
        checks['request_latency_recomputed'] = _matches(
            document.get('obstacle_request_to_zero_steady_s'),
            _difference_s(
                document, 'zero_steady_ns', 'obstacle_request_steady_ns'))
        checks['ack_latency_recomputed'] = _matches(
            document.get('obstacle_ack_to_zero_steady_s'),
            _difference_s(
                document, 'zero_steady_ns',
                'obstacle_activation_steady_ns'))
        checks['ros_clock_offset_diagnostic'] = (
            scan_gate_document is not None
            and _finite_number(document.get(
                'scan_header_to_stop_observer_ros_signed_s'))
            and _matches(
                document['scan_header_to_stop_observer_ros_signed_s'],
                (document['stop_state_ros_ns']
                 - scan_gate_document['stamp_ns']) / 1e9))
        checks['same_host_monotonic_causal_order'] = (
            scan_gate_document is not None
            and scan_gate_document['publish_steady_ns']
            < document.get('stop_state_steady_ns', -1)
            < document.get('zero_steady_ns', -1)
            <= document.get('physical_stop_steady_ns', -1))
    if scenario == 'scan_timeout_stop_resume':
        checks['stop_event_polygon_match'] = (
            len(stop_events) == 1
            and stop_events[0].get('polygon') == 'invalid source'
            and stop_events[0].get('polygon')
            == document.get('stop_polygon_name'))
        checks['stop_source_is_invalid_source'] = (
            document.get('stop_polygon_name') == 'invalid source')
        checks['monitor_only_freeze'] = (
            document.get('collision_monitor_scan_frozen') is True
            and document.get('raw_scan_continued') is True
            and document.get('navigation_scan_continued') is True)
        checks['freeze_count_causality'] = (
            document.get('monitor_scan_count_at_stop', -1)
            == document.get('monitor_scan_count_at_freeze', -2)
            and document.get('raw_scan_count_at_stop', -1)
            > document.get('raw_scan_count_at_trigger', -1)
            and document.get('navigation_scan_count_at_stop', -1)
            > document.get('navigation_scan_count_at_trigger', -1))
        checks['timeout_elapsed_without_monitor_scan'] = (
            _difference_s(
                document, 'stop_state_ros_ns',
                'last_monitor_scan_stamp_ns_at_freeze')
            > contract['selected_source_timeout_s'])
        checks['monitor_resumed_after_unfreeze'] = (
            document.get('monitor_scan_count_final', -1)
            > document.get('monitor_scan_count_at_freeze', -1))
        checks['latency_recomputed'] = _matches(
            document.get('freeze_ack_to_zero_ros_s'),
            _difference_s(document, 'zero_ros_ns', 'freeze_ros_ns'),
        )
        checks['steady_latency_recomputed'] = _matches(
            document.get('freeze_ack_to_zero_steady_s'),
            _difference_s(document, 'zero_steady_ns', 'freeze_steady_ns'),
        )
        checks['source_age_recomputed'] = (
            _difference_s(
                document, 'stop_state_ros_ns',
                'last_monitor_scan_stamp_ns_at_freeze')
            > contract['selected_source_timeout_s'])
    if scenario != 'clear_baseline':
        trigger_pose = document.get('trigger_world_pose_m')
        stop_pose = document.get('stop_world_pose_m')
        checks['stop_distance_recomputed'] = (
            isinstance(trigger_pose, list)
            and isinstance(stop_pose, list)
            and len(trigger_pose) == len(stop_pose) == 2
            and _matches(
                document.get('stop_distance_m'),
                math.dist(trigger_pose, stop_pose)))
    failures = [name for name, passed in checks.items() if not passed]
    return {
        'run_id': document['run_id'],
        'scenario': scenario,
        'seed': document['seed'],
        'status': 'PASS' if not failures else 'FAIL',
        'checks': checks,
        'failures': failures,
    }


def _representative_retention_checks(
        manifest: dict[str, Any] | None, matrix_passed: bool,
        contract: dict[str, Any], root: Path) -> dict[str, bool]:
    """Validate opt-in representative bag identities and size/hash records."""
    if manifest is None:
        return {'representative_retention_disabled': True}
    records = manifest.get('representative_bags', [])
    expected = Counter(
        (scenario, 11, f'{scenario}__seed_11')
        for scenario in contract['retention']['representative_scenarios'])
    actual = Counter(
        (record.get('scenario'), record.get('seed'), record.get('run_id'))
        for record in records)
    checks = {
        'representative_only_after_pass': matrix_passed,
        'representative_exact_set': actual == expected,
        'representative_unique_paths': (
            len({record.get('path') for record in records}) == len(records)),
        'representative_unique_hashes': (
            len({record.get('sha256') for record in records}) == len(records)),
    }
    for index, record in enumerate(records):
        path = Path(record.get('path', ''))
        metadata_path = path.parent / 'metadata.yaml'
        identity = False
        try:
            metadata = yaml.safe_load(metadata_path.read_text())[
                'rosbag2_bagfile_information']
            relative_paths = metadata['relative_file_paths']
            with path.open('rb') as stream:
                start_magic = stream.read(8)
                stream.seek(-8, 2)
                end_magic = stream.read(8)
            import rosbag2_py
            import tf2_py
            from action_msgs.msg import GoalStatus, GoalStatusArray
            from nav2_msgs.msg import CollisionMonitorState
            from rclpy.time import Time
            from rclpy.serialization import deserialize_message
            from tf2_msgs.msg import TFMessage
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(
                    uri=str(path.parent), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
            actual_counts: Counter[str] = Counter()
            action_states = []
            monitor_states = []
            transforms = []
            last_topic_timestamp: dict[str, int] = {}
            topic_timestamps_ordered = True
            while reader.has_next():
                topic, data, timestamp_ns = reader.read_next()
                if timestamp_ns < last_topic_timestamp.get(topic, -1):
                    topic_timestamps_ordered = False
                last_topic_timestamp[topic] = timestamp_ns
                actual_counts[topic] += 1
                if topic == '/navigate_to_pose/_action/status':
                    message = deserialize_message(data, GoalStatusArray)
                    action_states.extend(
                        (timestamp_ns, status)
                        for status in message.status_list)
                elif topic == '/collision_monitor_state':
                    message = deserialize_message(data, CollisionMonitorState)
                    monitor_states.append((
                        timestamp_ns, int(message.action_type),
                        message.polygon_name))
                elif topic in {'/tf', '/tf_static'}:
                    message = deserialize_message(data, TFMessage)
                    transforms.extend((topic, transform)
                                      for transform in message.transforms)
            actual_counts = dict(actual_counts)
            metadata_counts = {
                item['topic_metadata']['name']: item['message_count']
                for item in metadata['topics_with_message_count']}
            requested = set(record.get('requested_topics', []))
            canonical_topics = set(
                contract['retention']['representative_topics'])
            contact_topics = {
                topic for topic in canonical_topics
                if topic.endswith('/contact')}
            event_topics = contact_topics | {'/collision_monitor_state'}
            streaming_topics = canonical_topics - event_topics
            evidence_path = root / record['run_id'] / 'evidence.json'
            evidence = strict_json_loads(evidence_path.read_text())
            canonical_root = root.resolve()
            expected_run_dir = canonical_root / record['run_id']
            expected_bag_dir = expected_run_dir / 'bag'
            expected_path = expected_bag_dir / 'bag_0.mcap'
            expected_metadata_path = expected_path.parent / 'metadata.yaml'
            expected_record_keys = {
                'run_id', 'scenario', 'seed', 'path', 'size_bytes',
                'sha256', 'metadata_size_bytes', 'metadata_sha256',
                'topic_inventory', 'requested_topics'}
            bag_tree_valid = (
                path == expected_path
                and not expected_run_dir.is_symlink()
                and not expected_bag_dir.is_symlink()
                and not path.is_symlink() and not metadata_path.is_symlink()
                and path.is_file() and metadata_path.is_file()
                and path.resolve().is_relative_to(canonical_root)
                and metadata_path.resolve().is_relative_to(canonical_root)
                and path.resolve().parent == expected_bag_dir
                and set(path.parent.iterdir())
                == {expected_path, expected_metadata_path}
                and not any(item.is_dir() for item in path.parent.iterdir())
                and list(path.parent.rglob('*.mcap')) == [expected_path])
            goal_statuses = [
                (timestamp_ns, status.status)
                for timestamp_ns, status in action_states
                if bytes(status.goal_info.goal_id.uuid).hex()
                == evidence['goal_uuid']]
            nonzero_goal_ids = {
                bytes(status.goal_info.goal_id.uuid).hex()
                for _, status in action_states
                if any(status.goal_info.goal_id.uuid)}
            expected_polygon = (
                'invalid source'
                if record['scenario'] == 'scan_timeout_stop_resume'
                else 'StopZone')
            stop_states = [state for state in monitor_states
                           if state[1] == CollisionMonitorState.STOP]
            if record['scenario'] == 'clear_baseline':
                monitor_valid = (
                    all(state[1] == CollisionMonitorState.DO_NOTHING
                        for state in monitor_states))
            else:
                clear_states = [
                    state for state in monitor_states
                    if state[1] == CollisionMonitorState.DO_NOTHING
                    and stop_states and state[0] > stop_states[0][0]]
                monitor_valid = (
                    len(stop_states) == evidence['stop_state_count'] == 1
                    and all(state[1] in {
                        CollisionMonitorState.DO_NOTHING,
                        CollisionMonitorState.STOP}
                            for state in monitor_states)
                    and stop_states[0][2] == expected_polygon
                    and bool(clear_states)
                    and all(state[0] < clear_states[0][0]
                            for state in stop_states)
                    and all(not (
                        state[1] == CollisionMonitorState.STOP
                        and state[0] > clear_states[0][0])
                            for state in monitor_states)
                    and monitor_states[-1][1]
                    == CollisionMonitorState.DO_NOTHING)
            succeeded_states = [
                item for item in goal_statuses
                if item[1] == GoalStatus.STATUS_SUCCEEDED]
            action_valid = (
                nonzero_goal_ids == {evidence['goal_uuid']}
                and bool(goal_statuses)
                and len(succeeded_states) == 1
                and goal_statuses[-1][1] == GoalStatus.STATUS_SUCCEEDED
                and all(status in {
                    GoalStatus.STATUS_ACCEPTED,
                    GoalStatus.STATUS_EXECUTING,
                    GoalStatus.STATUS_SUCCEEDED}
                    for _, status in goal_statuses)
                and all(left[1] <= right[1]
                        for left, right in zip(
                            goal_statuses, goal_statuses[1:]))
                and any(
                    executing_time < succeeded_time
                    for executing_time, status in goal_statuses
                    for succeeded_time, later_status in goal_statuses
                    if status == GoalStatus.STATUS_EXECUTING
                    and later_status == GoalStatus.STATUS_SUCCEEDED))
            tf_valid = False
            try:
                buffer = tf2_py.BufferCore()
                raw_tf_records = []
                for topic, transform_item in transforms:
                    if topic == '/tf_static':
                        buffer.set_transform_static(
                            transform_item, 'g004_bag')
                    else:
                        buffer.set_transform(transform_item, 'g004_bag')
                    translation = transform_item.transform.translation
                    rotation = transform_item.transform.rotation
                    raw_tf_records.append({
                        'parent': transform_item.header.frame_id,
                        'child': transform_item.child_frame_id,
                        'stamp_ns': (
                            transform_item.header.stamp.sec * 1_000_000_000
                            + transform_item.header.stamp.nanosec),
                        'translation': [translation.x, translation.y,
                                        translation.z],
                        'rotation': [rotation.x, rotation.y, rotation.z,
                                     rotation.w],
                        'is_static': topic == '/tf_static',
                    })
                entry = evidence.get('goal_position_tolerance_entry')
                entry_events = [
                    event for event in evidence.get('events', [])
                    if event.get('name')
                    == 'goal_position_tolerance_entered']
                if entry is None and not entry_events:
                    tf_valid = True
                elif (isinstance(entry, dict) and len(entry_events) == 1
                      and entry == entry_events[0]):
                    transform_item = buffer.lookup_transform_core(
                        'map', 'base_link',
                        Time(nanoseconds=entry[
                            'transform_stamp_ns']).to_msg())
                    raw_pose = [transform_item.transform.translation.x,
                                transform_item.transform.translation.y]
                    goal = [contract['goal_pose']['x_m'],
                            contract['goal_pose']['y_m']]
                    tf_valid = (
                        all(_finite_number(value) for value in raw_pose)
                        and _canonical_tf_subset(
                            raw_tf_records, entry['transform_stamp_ns'])
                        == entry['tf_inputs']
                        and math.dist(raw_pose, entry['pose_m']) <= 1e-6
                        and math.dist(raw_pose, goal)
                        <= contract['goal_verification'][
                            'position_tolerance_m'])
            except (KeyError, TypeError, ValueError, RuntimeError,
                    tf2_py.TransformException):
                tf_valid = False
            identity = (
                set(record) == expected_record_keys
                and bag_tree_valid and path.suffix == '.mcap'
                and path.resolve() == expected_path
                and start_magic == b'\x89MCAP0\r\n'
                and end_magic == b'\x89MCAP0\r\n'
                and metadata['storage_identifier'] == 'mcap'
                and relative_paths == ['bag_0.mcap']
                and path.stat().st_size == record.get('size_bytes')
                and record.get('size_bytes', -1)
                <= contract['retention']['individual_bag_limit_bytes']
                and record.get('sha256') == sha256_file(path)
                and record.get('metadata_size_bytes')
                == metadata_path.stat().st_size
                and record.get('metadata_sha256')
                == sha256_file(metadata_path)
                and record.get('topic_inventory')
                == metadata.get('topics_with_message_count')
                and requested == canonical_topics
                and set(metadata_counts) == canonical_topics
                and set(actual_counts) <= canonical_topics
                and all(actual_counts[topic] > 0
                        for topic in streaming_topics)
                and len(contact_topics) == 1
                and actual_counts.get(next(iter(contact_topics)), 0)
                == evidence['contact_count'] == 0
                and all(actual_counts.get(topic, 0) == metadata_counts[topic]
                        for topic in canonical_topics)
                and evidence.get('representative_bag') == record
                and topic_timestamps_ordered
                and action_valid
                and monitor_valid
                and tf_valid)
        except (AttributeError, KeyError, OSError, RuntimeError, TypeError,
                ValueError):
            identity = False
        checks[f'representative_{index}_identity'] = identity
    return checks


def promotion_runtime_valid(
        root: Path, records: list[dict[str, Any]],
        source_matrix: dict[str, Any],
) -> bool:
    """Bind the exact three promotion runs to domains and source RMW."""
    try:
        runtime = strict_json_loads((root / 'runtime_manifest.json').read_text())
        base = runtime['domain_id_base']
        expected_planned = [
            {'run_id': record['run_id'], 'domain_id': base + index}
            for index, record in enumerate(records)]
        rmw_keys = (
            'requested_identifier', 'actual_identifier', 'version', 'source',
            'library_size_bytes', 'library_sha256', 'environment_fields',
            'environment_sha256')
        rmw_identity = {
            key: runtime['evaluation_rmw'][key] for key in rmw_keys}
        evidence_domains = [
            strict_json_loads(
                (root / record['run_id'] / 'evidence.json').read_text())[
                    'domain_id']
            for record in records]
        evaluation = Path(__file__).resolve().parent
        package = evaluation.parent / 'jdamr_cube_navigation'
        expected_sources = {
            'contract': evaluation / 'sim_collision_monitor_contract.py',
            'prepare': evaluation / 'prepare_sim_collision_monitor_run.py',
            'runner': evaluation / 'run_sim_collision_monitor_eval.py',
            'evaluator': evaluation / 'evaluate_sim_collision_monitor.py',
            'finalizer': evaluation / 'finalize_sim_collision_monitor_eval.py',
            'resource_sampler': evaluation
            / 'sample_process_group_resources.py',
            'scenario': package / 'sim_collision_monitor_scenario.py',
            'scan_gate': package / 'sim_scan_gate.py',
        }
        source_records_valid = set(runtime['harness_sources']) == set(
            expected_sources)
        for name, path in expected_sources.items():
            record = runtime['harness_sources'].get(name, {})
            retained = root / 'runtime_sources' / f'{name}.py'
            source_records_valid = source_records_valid and (
                Path(record.get('path', '')).resolve() == path.resolve()
                and path.is_file() and retained.is_file()
                and record.get('size_bytes') == path.stat().st_size
                and record.get('sha256') == sha256_file(path)
                and Path(record.get('retained_path', '')).resolve()
                == retained.resolve()
                and record.get('retained_size_bytes')
                == retained.stat().st_size
                and record.get('retained_sha256') == sha256_file(retained)
                and record.get('retained_sha256') == record.get('sha256'))
        rmw = runtime['evaluation_rmw']
        retained_library = Path(rmw['retained_library_path'])
        retained_rmw_valid = (
            retained_library.resolve().parent
            == (root / 'runtime_dependencies').resolve()
            and retained_library.is_file()
            and retained_library.stat().st_size
            == rmw['retained_library_size_bytes']
            and sha256_file(retained_library)
            == rmw['retained_library_sha256']
            and rmw['retained_library_sha256'] == rmw['library_sha256'])
        return (
            set(runtime) == {
                'evaluation_rmw', 'canonical_contract_sha256',
                'preparation_manifest_sha256', 'execution_mode',
                'domain_id_base', 'domain_ids', 'planned_runs',
                'harness_sources'}
            and runtime['execution_mode'] == 'representative_promotion'
            and _exact_int(base)
            and len(records) == 3
            and runtime['domain_ids'] == [base, base + 1, base + 2]
            and runtime['planned_runs'] == expected_planned
            and evidence_domains == runtime['domain_ids']
            and all(_exact_int(value) and value <= 232
                    for value in evidence_domains)
            and rmw_identity == source_matrix['evaluation_rmw_identity']
            and runtime['canonical_contract_sha256']
            == sha256_file(root / 'contract.json')
            and runtime['preparation_manifest_sha256']
            == sha256_file(root / 'preparation_manifest.json')
            and runtime['preparation_manifest_sha256']
            == source_matrix['preparation_manifest_sha256']
            and source_records_valid and retained_rmw_valid)
    except (KeyError, OSError, TypeError, ValueError):
        return False


def aggregate(
        paths: list[Path], retention_manifest: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Require the exact 15-run identity multiset."""
    if not paths:
        return {
            'status': 'FAIL', 'matrix_complete': False, 'run_count': 0,
            'passed': 0, 'failed': 0, 'total_retained_bytes': 0,
            'retention_checks': {'matrix_total_retention': False},
            'evidence_manifest': [], 'runs': [],
        }
    root = paths[0].resolve().parent.parent
    contract_path = root / 'contract.json'
    if not contract_path.is_file():
        raise ValueError('canonical root contract.json is missing')
    canonical = strict_json_loads(contract_path.read_text())
    canonical_sha256 = sha256_file(contract_path)
    documents = [strict_json_loads(path.read_text()) for path in paths]
    runs = [evaluate_run(document, path, canonical, canonical_sha256)
            for path, document in zip(paths, documents)]
    expected = Counter(
        (item['run_id'], item['scenario'], item['seed'])
        for item in scenario_matrix())
    actual = Counter(
        (item['run_id'], item['scenario'], item['seed']) for item in runs)
    passed = sum(item['status'] == 'PASS' for item in runs)
    path_identity = all(
        path.name == 'evidence.json'
        and path.resolve().parent.parent == root
        and path.parent.name == document.get('run_id')
        for path, document in zip(paths, documents))
    matrix_complete = actual == expected and len(runs) == 15 and path_identity
    matrix_passed = matrix_complete and passed == len(expected)
    matrix_retained_bytes = _tree_size_excluding(
        root, {'aggregate.json', 'final_summary.json',
               'retention_manifest.json'})
    total_retained_bytes = matrix_retained_bytes
    retention_checks = {
        'matrix_total_retention': (
            total_retained_bytes >= 0
            and bool(documents)
            and total_retained_bytes
            <= documents[0]['contract']['retention']['total_limit_bytes']),
        'preparation_identity': _preparation_identity_valid(
            root, contract_path),
        'full_matrix_bag_artifacts_absent': _bag_artifacts_absent(root),
    }
    if documents:
        retention_checks.update(_representative_retention_checks(
            retention_manifest, matrix_passed, documents[0]['contract'], root))
    status = (
        'PASS' if matrix_passed and all(retention_checks.values()) else 'FAIL')
    return {
        'status': status,
        'matrix_complete': matrix_complete,
        'run_count': len(runs),
        'passed': passed,
        'failed': len(runs) - passed,
        'total_retained_bytes': total_retained_bytes,
        'retention_checks': retention_checks,
        'evidence_manifest': [{
            'run_id': document['run_id'],
            'relative_path': str(path.resolve().relative_to(root)),
            'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path),
        } for path, document in zip(paths, documents)],
        'runs': runs,
    }


def verify_promotion(root: Path) -> dict[str, Any]:
    """Freshly verify a retained three-bag promotion without trusting it."""
    manifest_path = root / 'promotion_manifest.json'
    failures = []
    try:
        manifest = strict_json_loads(manifest_path.read_text())
        expected_keys = {
            'schema_version', 'status', 'source_matrix',
            'representative_bags', 'checks',
            'retained_bytes_excluding_manifest', 'retained_file_manifest',
            'retained_directories'}
        if (set(manifest) != expected_keys or manifest['schema_version'] != 1
                or manifest['status'] != 'PASS'):
            raise ValueError('promotion manifest schema mismatch')
        source = manifest['source_matrix']
        source_root = Path(source['path']).resolve()
        source_paths = [
            source_root / item['run_id'] / 'evidence.json'
            for item in scenario_matrix()]
        source_result = aggregate(source_paths)
        source_files = {
            'contract': source_root / 'contract.json',
            'aggregate': source_root / 'aggregate.json',
            'final_summary': source_root / 'final_summary.json',
            'runtime_manifest': source_root / 'runtime_manifest.json',
        }
        source_final = strict_json_loads(
            source_files['final_summary'].read_text())
        source_aggregate = strict_json_loads(
            source_files['aggregate'].read_text())
        from finalize_sim_collision_monitor_eval import finalize
        recomputed_final = finalize(source_root)
        source_runtime = strict_json_loads(
            source_files['runtime_manifest'].read_text())
        source_rmw_keys = tuple(source['evaluation_rmw_identity'])
        source_valid = (
            set(source) == {
                'path', 'contract_sha256', 'aggregate_size_bytes',
                'preparation_manifest_sha256',
                'aggregate_sha256', 'final_summary_size_bytes',
                'final_summary_sha256', 'runtime_manifest_size_bytes',
                'runtime_manifest_sha256', 'evaluation_rmw_identity',
                'evidence_manifest'}
            and source_result['status'] == 'PASS'
            and source_aggregate == source_result
            and source_final == recomputed_final
            and source['evidence_manifest']
            == source_result['evidence_manifest']
            and source['contract_sha256']
            == sha256_file(source_files['contract'])
            and source['preparation_manifest_sha256']
            == sha256_file(source_root / 'preparation_manifest.json')
            and source['aggregate_size_bytes']
            == source_files['aggregate'].stat().st_size
            and source['aggregate_sha256']
            == sha256_file(source_files['aggregate'])
            and source['final_summary_size_bytes']
            == source_files['final_summary'].stat().st_size
            and source['final_summary_sha256']
            == sha256_file(source_files['final_summary'])
            and source['runtime_manifest_size_bytes']
            == source_files['runtime_manifest'].stat().st_size
            and source['runtime_manifest_sha256']
            == sha256_file(source_files['runtime_manifest'])
            and source_final.get('matrix', {}).get('status') == 'PASS'
            and source_final.get('matrix', {}).get('run_count') == 15
            and source_final.get('aggregate_input_sha256')
            == sha256_file(source_files['aggregate'])
            and source_final.get('contract_input_sha256')
            == sha256_file(source_files['contract'])
            and source['evaluation_rmw_identity'] == {
                key: source_runtime['evaluation_rmw'][key]
                for key in source_rmw_keys}
            and source['contract_sha256']
            == sha256_file(root / 'contract.json'))
        if not source_valid:
            failures.append('source_matrix_identity')
        records = manifest['representative_bags']
        contract = strict_json_loads((root / 'contract.json').read_text())
        evidence_paths = [root / record['run_id'] / 'evidence.json'
                          for record in records]
        evaluations = [evaluate_run(
            strict_json_loads(path.read_text()), path, contract,
            sha256_file(root / 'contract.json'), representative_run=True)
            for path in evidence_paths]
        if len(evaluations) != 3 or any(
                item['status'] != 'PASS' for item in evaluations):
            failures.append('representative_run_evidence')
        checks = _representative_retention_checks(
            {'representative_bags': records}, True, contract, root)
        checks['promotion_runtime_identity'] = promotion_runtime_valid(
            root, records, source)
        actual_files = [{
            'relative_path': str(path.relative_to(root)),
            'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path),
        } for path in sorted(root.rglob('*')) if path.is_file()
            and path.name != 'promotion_manifest.json']
        actual_directories = sorted(
            str(path.relative_to(root))
            for path in root.rglob('*') if path.is_dir())
        actual_bytes = sum(item['size_bytes'] for item in actual_files)
        checks['promotion_file_exact_set'] = (
            actual_files == manifest['retained_file_manifest'])
        checks['promotion_directory_exact_set'] = (
            actual_directories == manifest['retained_directories'])
        checks['promotion_total_retention'] = (
            actual_bytes == manifest['retained_bytes_excluding_manifest']
            and actual_bytes <= contract['retention']['total_limit_bytes'])
        if checks != manifest['checks'] or not all(checks.values()):
            failures.append('promotion_checks')
    except (KeyError, OSError, TypeError, ValueError):
        failures.append('promotion_manifest_parse')
        checks = {}
        evaluations = []
    return {
        'status': 'PASS' if not failures else 'FAIL',
        'failures': failures, 'checks': checks,
        'representative_runs': evaluations,
    }


def main() -> int:
    """Verify a durable representative promotion root."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--promotion-root', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = verify_promotion(args.promotion_root)
    text = json.dumps(result, indent=2, sort_keys=True) + '\n'
    if args.output is None:
        print(text, end='')
    else:
        args.output.write_text(text)
    return 0 if result['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
