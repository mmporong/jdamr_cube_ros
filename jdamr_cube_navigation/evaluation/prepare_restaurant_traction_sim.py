#!/usr/bin/env python3
"""Prepare a data-derived restaurant route and low-traction Gazebo world."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from corridor_run_media import parse_route_log  # noqa: I201

import yaml


EXPECTED_WAYPOINTS = 20
EXPECTED_STOP_EVENTS = 6
SIM_START_X_M = -8.0
SIM_END_X_M = 6.0
SIM_OUTBOUND_Y_M = 0.35
SIM_RETURN_Y_M = -0.35
EVALUATION_URDF = (Path(__file__).resolve().parent / 'assets'
                   / 'nav_obstacle' / 'jdamr_cube_nav_eval.urdf')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return value


def load_route(route_path: Path) -> dict:
    """Load the recorded corridor route without resolving real map assets."""
    document = yaml.safe_load(route_path.read_text(encoding='utf-8'))
    waypoints = (
        document.get('waypoints') if isinstance(document, dict) else None)
    if not isinstance(waypoints, list) or len(waypoints) != EXPECTED_WAYPOINTS:
        raise ValueError(
            f'replay source must contain {EXPECTED_WAYPOINTS} waypoints')
    identifiers = [str(item.get('id', '')).strip() for item in waypoints]
    if any(not identifier for identifier in identifiers):
        raise ValueError('waypoint ids must be non-empty')
    if len(set(identifiers)) != len(identifiers):
        raise ValueError('waypoint ids must be unique')
    for waypoint in waypoints:
        waypoint['x'] = _finite(waypoint['x'], f'{waypoint["id"]}.x')
        waypoint['y'] = _finite(waypoint['y'], f'{waypoint["id"]}.y')
    return document


def load_stop_events(path: Path) -> list[dict]:
    """Load the six measured StopZone entries retained by the real analyser."""
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != EXPECTED_STOP_EVENTS:
        raise ValueError(
            f'replay source must contain {EXPECTED_STOP_EVENTS} stop events')
    events = []
    for index, row in enumerate(rows, start=1):
        event = int(row['event'])
        elapsed_s = _finite(row['stop_elapsed_s'], 'stop_elapsed_s')
        if event != index or elapsed_s < 0.0:
            raise ValueError('stop events must be ordered from one')
        events.append({'event': event, 'real_stop_elapsed_s': elapsed_s})
    return events


def _active_waypoint(route_log: dict, stamp_ns: int) -> dict:
    accepted = [
        event for event in route_log['goal_events']
        if event.get('event') == 'accepted' and event['stamp_ns'] <= stamp_ns
    ]
    if not accepted:
        raise ValueError('stop event precedes the first accepted goal')
    active = accepted[-1]
    result = next((
        event for event in route_log['goal_events']
        if event.get('event') == 'result'
        and event.get('goal_uuid') == active.get('goal_uuid')
        and event['stamp_ns'] >= stamp_ns
    ), None)
    if result is None:
        raise ValueError('stop event is not bounded by its goal result')
    return active


def bind_events_to_waypoints(
        events: list[dict], route_log: dict) -> list[dict]:
    """Replace fragile wall-clock triggers with active-waypoint triggers."""
    bound = []
    for event in events:
        stamp_ns = route_log['start_stamp_ns'] + round(
            event['real_stop_elapsed_s'] * 1e9)
        active = _active_waypoint(route_log, stamp_ns)
        bound.append({
            **event,
            'trigger': {
                'kind': 'active_waypoint',
                'waypoint_index': int(active['waypoint_index']),
                'waypoint_id': str(active['waypoint_id']),
            },
            'required_transition': [
                'NAVIGATING', 'PROTECTIVE_STOP', 'REPLAN',
                'SAME_GOAL_RESUME',
            ],
        })
    return bound


def transform_waypoints(waypoints: list[dict]) -> tuple[list[dict], float]:
    """Preserve route topology while fitting the real route in the sim lane."""
    maximum_x_m = max(float(item['x']) for item in waypoints)
    if maximum_x_m <= 0.0:
        raise ValueError('route requires a positive outbound extent')
    scale = (SIM_END_X_M - SIM_START_X_M) / maximum_x_m
    transformed = []
    for item in waypoints:
        identifier = str(item['id'])
        if identifier.startswith('outbound_'):
            y_m = SIM_OUTBOUND_Y_M
        elif identifier.startswith('return_') or identifier == 'home':
            y_m = SIM_RETURN_Y_M
        else:
            y_m = 0.0
        transformed.append({
            'id': identifier,
            'x_m': round(SIM_START_X_M + float(item['x']) * scale, 6),
            'y_m': y_m,
            'source_x_m': float(item['x']),
            'source_y_m': float(item['y']),
        })
    return transformed, scale


def add_low_traction_zone(
        source_world: Path, output_world: Path, *, center_x_m: float,
        center_y_m: float, length_m: float, width_m: float,
        friction_mu: float) -> None:
    """Replace the corridor floor contact with non-overlapping floor boxes."""
    if min(length_m, width_m) <= 0.0:
        raise ValueError('traction-zone dimensions must be positive')
    if not 0.0 <= friction_mu < 1.0:
        raise ValueError('friction mu must be in [0, 1)')
    tree = ET.parse(source_world)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('source SDF contains no world')
    if world.find("./model[@name='restaurant_low_traction_zone']") is not None:
        raise ValueError('source world already contains the traction zone')
    ground = world.find("./model[@name='ground_plane']")
    ground_link = ground.find('link') if ground is not None else None
    ground_collision = (
        ground_link.find('collision') if ground_link is not None else None)
    if ground_link is None or ground_collision is None:
        raise ValueError('source world must contain ground_plane collision')

    floor_x_min, floor_x_max = -10.0, 10.0
    floor_y_min, floor_y_max = -1.2, 1.2
    patch_x_min = center_x_m - length_m / 2.0
    patch_x_max = center_x_m + length_m / 2.0
    patch_y_min = center_y_m - width_m / 2.0
    patch_y_max = center_y_m + width_m / 2.0
    if not (floor_x_min < patch_x_min < patch_x_max < floor_x_max
            and floor_y_min < patch_y_min < patch_y_max < floor_y_max):
        raise ValueError('traction zone must be inside the corridor floor')
    ground_link.remove(ground_collision)

    def floor_box(name: str, x_min: float, x_max: float,
                  y_min: float, y_max: float) -> None:
        model = ET.SubElement(world, 'model', {'name': name})
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = (
            f'{(x_min + x_max) / 2.0} {(y_min + y_max) / 2.0} '
            '-0.01 0 0 0')
        link = ET.SubElement(model, 'link', {'name': 'surface'})
        collision = ET.SubElement(link, 'collision', {'name': 'collision'})
        geometry = ET.SubElement(collision, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = (
            f'{x_max - x_min} {y_max - y_min} 0.02')

    floor_box('restaurant_floor_left', floor_x_min, patch_x_min,
              floor_y_min, floor_y_max)
    floor_box('restaurant_floor_right', patch_x_max, floor_x_max,
              floor_y_min, floor_y_max)
    floor_box('restaurant_floor_south', patch_x_min, patch_x_max,
              floor_y_min, patch_y_min)
    floor_box('restaurant_floor_north', patch_x_min, patch_x_max,
              patch_y_max, floor_y_max)

    model = ET.SubElement(
        world, 'model', {'name': 'restaurant_low_traction_zone'})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = (
        f'{center_x_m} {center_y_m} -0.01 0 0 0')
    link = ET.SubElement(model, 'link', {'name': 'surface'})
    collision = ET.SubElement(link, 'collision', {'name': 'collision'})
    geometry = ET.SubElement(collision, 'geometry')
    box = ET.SubElement(geometry, 'box')
    ET.SubElement(box, 'size').text = f'{length_m} {width_m} 0.02'
    surface = ET.SubElement(collision, 'surface')
    friction = ET.SubElement(surface, 'friction')
    ode = ET.SubElement(friction, 'ode')
    ET.SubElement(ode, 'mu').text = str(friction_mu)
    ET.SubElement(ode, 'mu2').text = str(friction_mu)
    visual = ET.SubElement(link, 'visual', {'name': 'visual'})
    visual_geometry = ET.SubElement(visual, 'geometry')
    visual_box = ET.SubElement(visual_geometry, 'box')
    ET.SubElement(visual_box, 'size').text = f'{length_m} {width_m} 0.02'
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'diffuse').text = '0.20 0.35 0.55 0.55'
    ET.indent(tree, space='  ')
    output_world.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_world, encoding='utf-8', xml_declaration=True)


def write_guarded_bridge(base_bridge: Path, output_bridge: Path) -> None:
    """Route only the guarded velocity topic into Gazebo DiffDrive."""
    entries = yaml.safe_load(base_bridge.read_text(encoding='utf-8'))
    matches = [
        entry for entry in entries
        if entry.get('direction') == 'ROS_TO_GZ'
        and entry.get('gz_topic_name') == 'cmd_vel'
    ]
    if len(matches) != 1 or matches[0].get('ros_topic_name') != 'cmd_vel':
        raise ValueError('bridge must contain one canonical cmd_vel input')
    matches[0]['ros_topic_name'] = 'guarded_cmd_vel'
    output_bridge.parent.mkdir(parents=True, exist_ok=True)
    output_bridge.write_text(
        yaml.safe_dump(entries, sort_keys=False), encoding='utf-8')


def prepare(args: argparse.Namespace) -> dict:
    """Generate immutable world and replay contract inputs."""
    if not EVALUATION_URDF.is_file():
        raise ValueError(
            'evaluation URDF is missing; generate nav obstacle assets first')
    route = load_route(args.route)
    route_log = parse_route_log(args.route_log.read_text(encoding='utf-8'))
    if (route_log['sent_waypoints'] != EXPECTED_WAYPOINTS
            or not route_log['success']):
        raise ValueError('route log must prove a successful 20-waypoint run')
    events = bind_events_to_waypoints(
        load_stop_events(args.stop_events), route_log)
    transformed, scale = transform_waypoints(route['waypoints'])
    output_world = args.output_dir / 'slam_corridor_traction.world'
    add_low_traction_zone(
        args.source_world, output_world,
        center_x_m=args.traction_center_x_m,
        center_y_m=getattr(args, 'traction_center_y_m', 0.0),
        length_m=args.traction_length_m,
        width_m=args.traction_width_m,
        friction_mu=args.friction_mu)
    base_bridge = getattr(args, 'base_bridge', None)
    output_bridge = args.output_dir / 'bridge_guarded.yaml'
    if base_bridge is not None:
        write_guarded_bridge(base_bridge, output_bridge)
    contract = {
        'schema_version': 1,
        'scenario_id': 'restaurant_real_run_topology_with_traction_fault',
        'source': {
            'route': {'path': str(args.route.resolve()),
                      'sha256': _sha256(args.route)},
            'route_log': {'path': str(args.route_log.resolve()),
                          'sha256': _sha256(args.route_log)},
            'stop_events': {'path': str(args.stop_events.resolve()),
                            'sha256': _sha256(args.stop_events)},
            'world': {'path': str(args.source_world.resolve()),
                      'sha256': _sha256(args.source_world)},
            'base_bridge': (
                {'path': str(base_bridge.resolve()),
                 'sha256': _sha256(base_bridge)}
                if base_bridge is not None else None),
        },
        'route': {
            'source_waypoint_count': EXPECTED_WAYPOINTS,
            'simulation_waypoint_count': len(transformed),
            'longitudinal_scale': scale,
            'waypoints': transformed,
        },
        'obstacle_interventions': {
            'source_count': EXPECTED_STOP_EVENTS,
            'simulation_required_count': len(events),
            'events': events,
        },
        'traction_fault': {
            'cause_model': 'route_triggered_wheel_slip_compliance',
            'object_detection_or_avoidance': False,
            'robot_urdf': {'path': str(EVALUATION_URDF.resolve()),
                           'sha256': _sha256(EVALUATION_URDF)},
            'world': {'path': str(output_world.resolve()),
                      'sha256': _sha256(output_world)},
            'guarded_bridge': (
                {'path': str(output_bridge.resolve()),
                 'sha256': _sha256(output_bridge)}
                if output_bridge.is_file() else None),
            'zone': {
                'center_x_m': args.traction_center_x_m,
                'center_y_m': getattr(args, 'traction_center_y_m', 0.0),
                'length_m': args.traction_length_m,
                'width_m': args.traction_width_m,
                'mu': args.friction_mu,
                'mu2': args.friction_mu,
                'provenance': 'synthetic_stress_not_real_measurement',
            },
            'fault_injection': {
                'trigger_waypoint_index': 5,
                'trigger_waypoint_id': transformed[4]['id'],
                'wheel_links': [
                    'left_wheel_link', 'right_wheel_link'],
                'slip_compliance_lateral': 0.0,
                'slip_compliance_longitudinal': 100.0,
                'clear_condition': 'guarded_zero_while_nav_commanded',
                'surface_mu_role': 'visual_marker_and_secondary_stress',
            },
            'required_transition': [
                'NAVIGATING', 'PROTECTIVE_STOP', 'RELOCALIZE',
                'LOW_SPEED_RESUME', 'RECOVERED_OR_FAULT_LATCHED',
            ],
        },
        'fidelity': {
            'implementation_status': {
                'prepared': [
                    '20-waypoint topology transform',
                    'six real events bound to active waypoints',
                    'route-triggered Gazebo WheelSlip fault injection',
                ],
                'runtime_gates': [
                    'Nav2 execution of the transformed 20-waypoint route',
                    'six automatic obstacle activations',
                    'TRACTION_FAULT stop, relocalize, and resume supervisor',
                ],
            },
            'preserved': [
                '20-waypoint outbound-return topology',
                'six StopZone intervention points bound to active waypoints',
                'same-goal resume requirement',
                'Cartographer and Nav2 target decision stack contract',
            ],
            'not_identical': [
                'building geometry and metric route shape',
                '610-second wall-clock timing',
                'unmeasured real floor friction',
                'human or box semantic classification',
                'real sensor and network noise realization',
            ],
            'claim': 'functional_scenario_contract_not_digital_twin',
        },
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    contract_path = args.output_dir / 'scenario_contract.json'
    contract_path.write_text(
        json.dumps(contract, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    return contract


def main(argv: list[str] | None = None) -> int:
    """Run the preparation CLI and print its bounded replay claim."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', type=Path, required=True)
    parser.add_argument('--route-log', type=Path, required=True)
    parser.add_argument('--stop-events', type=Path, required=True)
    parser.add_argument('--source-world', type=Path, required=True)
    parser.add_argument('--base-bridge', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--traction-center-x-m', type=float, default=-1.28)
    parser.add_argument('--traction-center-y-m', type=float, default=0.0)
    parser.add_argument('--traction-length-m', type=float, default=1.5)
    parser.add_argument('--traction-width-m', type=float, default=2.1)
    parser.add_argument('--friction-mu', type=float, default=0.05)
    args = parser.parse_args(argv)
    for path in (
            args.route, args.route_log, args.stop_events, args.source_world):
        if not path.is_file():
            parser.error(f'input does not exist: {path}')
    if args.base_bridge is not None and not args.base_bridge.is_file():
        parser.error(f'input does not exist: {args.base_bridge}')
    contract = prepare(args)
    print(json.dumps({
        'status': 'prepared',
        'claim': contract['fidelity']['claim'],
        'waypoints': contract['route']['simulation_waypoint_count'],
        'stop_events': contract['obstacle_interventions'][
            'simulation_required_count'],
        'output_dir': str(args.output_dir.resolve()),
    }, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
