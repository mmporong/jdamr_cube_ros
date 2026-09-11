"""Tests for the data-derived restaurant traction simulation inputs."""

import csv
import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import yaml


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / 'jdamr_cube_navigation' / 'evaluation'
          / 'prepare_restaurant_traction_sim.py')
SPEC = importlib.util.spec_from_file_location('traction_prepare', SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _route(path):
    waypoints = []
    for index in range(1, 11):
        waypoints.append({'id': f'outbound_{index}', 'x': index, 'y': 0})
    waypoints[9]['id'] = 'turnaround'
    for index in range(9, 0, -1):
        waypoints.append({'id': f'return_{index}', 'x': index, 'y': -1})
    waypoints.append({'id': 'home', 'x': 0, 'y': 0})
    path.write_text(yaml.safe_dump({
        'schema_version': 1, 'waypoints': waypoints}), encoding='utf-8')


def _route_log(path):
    lines = []
    start = 1000.0
    for index in range(1, 21):
        stamp = start + (index - 1) * 10
        waypoint_id = 'turnaround' if index == 10 else f'wp_{index}'
        lines.extend([
            f'[INFO] [{stamp:.3f}] send {index}/20 {waypoint_id}=(0.00,0.00)',
            f'[INFO] [{stamp + 0.1:.3f}] route_event '
            + json.dumps({
                'event': 'accepted', 'goal_uuid': f'g{index}',
                'waypoint_id': waypoint_id, 'waypoint_index': index,
                'waypoint_total': 20}, separators=(',', ':')),
            f'[INFO] [{stamp + 9.0:.3f}] route_event '
            + json.dumps({
                'event': 'result', 'goal_uuid': f'g{index}',
                'waypoint_id': waypoint_id, 'waypoint_index': index,
                'waypoint_total': 20}, separators=(',', ':')),
        ])
    lines.append('[INFO] [1199.000] corridor roundtrip succeeded')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _events(path):
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(
            stream, fieldnames=['event', 'stop_elapsed_s'])
        writer.writeheader()
        for index, elapsed in enumerate((5, 15, 45, 85, 125, 165), start=1):
            writer.writerow({'event': index, 'stop_elapsed_s': elapsed})


def _world(path):
    path.write_text(
        '<sdf version="1.8"><world name="slam_corridor">'
        '<model name="ground"><static>true</static></model>'
        '</world></sdf>', encoding='utf-8')


def test_prepare_preserves_topology_and_builds_physical_friction_patch(
        tmp_path):
    """The generated inputs retain real topology and use surface friction."""
    route = tmp_path / 'route.yaml'
    route_log = tmp_path / 'route.log'
    events = tmp_path / 'events.csv'
    world = tmp_path / 'world.sdf'
    output = tmp_path / 'output'
    _route(route)
    _route_log(route_log)
    _events(events)
    _world(world)

    contract = MODULE.prepare(SimpleNamespace(
        route=route, route_log=route_log, stop_events=events,
        source_world=world, output_dir=output,
        traction_center_x_m=-1.28, traction_length_m=1.5,
        traction_width_m=2.1, friction_mu=0.05))

    assert contract['route']['source_waypoint_count'] == 20
    assert contract['route']['simulation_waypoint_count'] == 20
    assert contract['obstacle_interventions']['source_count'] == 6
    assert [
        event['trigger']['waypoint_index']
        for event in contract['obstacle_interventions']['events']
    ] == [1, 2, 5, 9, 13, 17]
    assert contract['traction_fault']['object_detection_or_avoidance'] is False
    assert contract['fidelity']['claim'] == (
        'functional_scenario_contract_not_digital_twin')
    assert len(contract['fidelity']['implementation_status']['pending']) == 3

    generated = ET.parse(output / 'slam_corridor_traction.world')
    patch = generated.getroot().find(
        "./world/model[@name='restaurant_low_traction_zone']")
    assert patch is not None
    assert float(patch.findtext('.//friction/ode/mu')) == 0.05
    assert float(patch.findtext('.//friction/ode/mu2')) == 0.05
    assert 'debris' not in ET.tostring(patch, encoding='unicode').lower()


def test_transform_fits_real_extent_and_separates_return_lane():
    """The metric transform fits the corridor and retains route direction."""
    points = [
        {'id': 'outbound_1', 'x': 2.0, 'y': -0.4},
        {'id': 'turnaround', 'x': 37.5, 'y': -4.2},
        {'id': 'return_1', 'x': 2.0, 'y': -0.4},
        {'id': 'home', 'x': 0.0, 'y': -0.1},
    ]

    transformed, scale = MODULE.transform_waypoints(points)

    assert scale == 14.0 / 37.5
    assert transformed[1]['x_m'] == 6.0
    assert transformed[0]['y_m'] == 0.35
    assert transformed[2]['y_m'] == -0.35
    assert transformed[3]['x_m'] == -8.0
