"""Self-filter for the bimanual frame columns in the LiDAR plane (2026-10-05)."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

from jdamr_cube_bringup.scan_self_filter import blind_sectors, filter_ranges, load_boxes
from launch import LaunchContext
from launch.actions import OpaqueFunction
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE / 'config/self_filter_hold_flow_model.yaml'
LASER = (0.10, 0.0, 0.0)   # hold_flow model laser_link in base_footprint


def _boxes():
    return load_boxes(__import__('json').dumps(yaml.safe_load(MODEL.read_text())['self_boxes']))


def _ranges_for(points, increment=math.radians(1.0)):
    ranges = [5.0] * 360
    for x, y in points:
        bearing = math.atan2(y - LASER[1], x - LASER[0])
        ranges[int(round((bearing + math.pi) / increment)) % 360] = math.hypot(
            x - LASER[0], y - LASER[1])
    return ranges


def test_returns_on_a_column_become_no_return_and_the_rest_stay():
    ranges = _ranges_for([(0.15, 0.205), (-0.15, -0.205), (0.60, 0.0)])
    ranges[10] = math.nan
    out, dropped = filter_ranges(ranges, -math.pi, math.radians(1.0), LASER, _boxes(), 0.01)
    assert dropped == 2
    assert sum(math.isinf(r) for r in out) == 2
    assert out.count(5.0) == ranges.count(5.0)                      # far returns kept
    assert any(abs(r - 0.5) < 1e-9 for r in out)                    # the obstacle ahead
    assert math.isnan(out[10])                                      # invalid stays invalid


def test_a_return_just_outside_a_column_is_kept():
    out, dropped = filter_ranges(_ranges_for([(0.15, 0.24)]), -math.pi, math.radians(1.0),
                                 LASER, _boxes(), 0.01)
    assert dropped == 0


def test_blind_sectors_are_reported_per_column():
    sectors = {name: (start, end, near) for name, start, end, near in
               blind_sectors(LASER, _boxes(), 0.01)}
    start, end, near = sectors['front_left_column']
    # 4x4 cm with the margin, 0.19 m away: 13 deg of bearing is dropped.
    assert start < 76.3 < end and 10.0 < end - start < 16.0
    assert near == pytest.approx(0.19, abs=0.01)
    start, end, _near = sectors['rear_right_column']
    assert start < -140.6 < end


def test_invalid_boxes_are_refused():
    with pytest.raises(ValueError):
        load_boxes('[{"center_xy_m": [0.1, 0.0], "size_xy_m": [0.0, 0.02]}]')
    assert load_boxes('') == []


def _lidar_nodes(config):
    spec = importlib.util.spec_from_file_location(
        'real_bringup', PACKAGE / 'launch/real_bringup.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.Node = lambda **kwargs: SimpleNamespace(**kwargs)    # capture what gets built
    description = module.generate_launch_description()
    function = next(entity for entity in description.entities
                    if isinstance(entity, OpaqueFunction))
    context = LaunchContext()
    context.launch_configurations['scan_self_filter_config'] = config
    context.launch_configurations['lidar_port'] = '/dev/ydlidar_g4'
    return function.execute(context)


def test_launch_keeps_the_driver_on_scan_without_a_config():
    nodes = _lidar_nodes('')
    assert [node.executable for node in nodes] == ['ydlidar_g4_node']
    assert nodes[0].parameters[0]['scan_topic'] == 'scan'


def test_launch_inserts_the_filter_between_scan_raw_and_scan():
    nodes = _lidar_nodes(str(MODEL))
    assert [node.executable for node in nodes] == ['ydlidar_g4_node', 'scan_self_filter']
    assert nodes[0].parameters[0]['scan_topic'] == 'scan_raw'
    filtered = nodes[1].parameters[0]
    assert (filtered['input_topic'], filtered['output_topic']) == ('scan_raw', 'scan')
    assert len(load_boxes(filtered['self_boxes_json'])) == 4
