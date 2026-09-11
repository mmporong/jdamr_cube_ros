"""Geometry tests for actual-map Gazebo reconstruction."""

import json
import sys
import xml.etree.ElementTree as ET
from argparse import Namespace
from pathlib import Path

import cv2
import numpy as np  # noqa: I201
import yaml  # noqa: I201


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from prepare_actual_map_restaurant_sim import (  # noqa: E402,I100,I201
    _rectangles,
    _world,
    _write_obj,
    prepare,
)


def test_rectangle_cover_preserves_every_occupied_cell_exactly():
    """Run merging changes representation, not occupancy geometry."""
    mask = np.asarray([
        [1, 1, 0, 1],
        [1, 1, 0, 1],
        [0, 1, 1, 1],
    ], dtype=bool)
    rectangles = _rectangles(mask)

    rebuilt = np.zeros_like(mask)
    for x_min, x_max, row_min, row_max in rectangles:
        rebuilt[row_min:row_max + 1, x_min:x_max + 1] = True
    assert np.array_equal(rebuilt, mask)


def test_obj_has_outward_normals_and_metric_map_coordinates(tmp_path):
    """DART receives one normal index for every generated triangle."""
    output = tmp_path / 'walls.obj'
    _write_obj(
        output, [(0, 1, 0, 0)], width=2, height=2,
        resolution_m=0.05, origin_x_m=-2.5, origin_y_m=-9.0,
        wall_height_m=2.4)
    lines = output.read_text(encoding='utf-8').splitlines()

    assert len([line for line in lines if line.startswith('vn ')]) == 6
    assert len([line for line in lines if line.startswith('v ')]) == 8
    faces = [line for line in lines if line.startswith('f ')]
    assert len(faces) == 12
    assert all('//' in line for line in faces)
    assert 'v -2.500000 -8.950000 0.000000' in lines
    assert 'v -2.400000 -8.900000 2.400000' in lines


def test_world_splits_floor_around_actual_route_traction_zone(tmp_path):
    """The actual-map world must retain the physical low-friction patch."""
    mesh = tmp_path / 'walls.obj'
    mesh.write_text('v 0 0 0\n', encoding='utf-8')
    world = tmp_path / 'world.sdf'
    zone = {
        'center_x_m': 5.0, 'center_y_m': 5.0,
        'length_m': 1.0, 'width_m': 0.8,
        'mu': 0.05, 'mu2': 0.05,
    }

    _world(
        world, mesh, {'resolution': 0.5, 'origin': [0.0, 0.0, 0.0]},
        (20, 20), 2.4, zone)
    root = ET.parse(world).getroot()
    patch = root.find(
        "./world/model[@name='restaurant_low_traction_zone']")

    assert patch is not None
    assert patch.findtext(
        './link/collision/surface/friction/ode/mu') == '0.05'
    assert patch.findtext(
        './link/collision/surface/friction/ode/mu2') == '0.05'
    assert len(root.findall('./world/model')) >= 8
    assert root.findtext(
        "./world/model[@name='actual_map_walls']/link/visual/"
        'visibility_flags') == '4'
    assert root.findtext(
        "./world/model[@name='g003_preloaded_route_obstacle']/link/visual/"
        'visibility_flags') == '4'


def test_actual_map_contract_removes_transformed_corridor_claims(tmp_path):
    """Identity replay metadata cannot retain the scaled fixture contract."""
    source = tmp_path / 'source'
    source.mkdir()
    image = np.full((20, 30), 254, dtype=np.uint8)
    image[0, :] = 0
    image[-1, :] = 0
    image[:, 0] = 0
    image[:, -1] = 0
    cv2.imwrite(str(source / 'map.pgm'), image)
    (source / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm', 'resolution': 0.5,
        'origin': [0.0, 0.0, 0.0], 'occupied_thresh': 0.65,
        'free_thresh': 0.196, 'negate': 0,
    }), encoding='utf-8')
    waypoints = [
        {'id': f'w{index}', 'x': 2.0 + index * 0.01, 'y': 2.0}
        for index in range(20)]
    (source / 'route.yaml').write_text(yaml.safe_dump({
        'start_pose': {'x': 2.0, 'y': 2.0}, 'waypoints': waypoints,
    }), encoding='utf-8')
    (source / 'robot.urdf').write_text(
        '<robot name="r"><gazebo><sensor type="gpu_lidar">'
        '<update_rate>2</update_rate><lidar><range><min>0.28</min>'
        '</range></lidar></sensor></gazebo></robot>', encoding='utf-8')
    (source / 'bridge.yaml').write_text('[]\n', encoding='utf-8')
    (source / 'base.json').write_text(json.dumps({
        'route': {'longitudinal_scale': 0.3},
        'traction_fault': {
            'zone': {'length_m': 1.0, 'width_m': 1.0,
                     'mu': 0.05, 'mu2': 0.05},
            'guarded_bridge': {'path': str(source / 'bridge.yaml')},
        },
        'obstacle_interventions': {},
        'fidelity': {'not_identical': ['building geometry']},
    }), encoding='utf-8')
    output = tmp_path / 'output'

    contract = prepare(Namespace(
        output_dir=output, base_contract=source / 'base.json',
        route=source / 'route.yaml', map_yaml=source / 'map.yaml',
        robot_urdf=source / 'robot.urdf', wall_height_m=2.4))

    assert contract['route']['longitudinal_scale'] == 1.0
    assert contract['route']['transform']['kind'] == (
        'identity_actual_map_coordinates')
    assert 'building geometry and metric route shape' not in (
        contract['fidelity']['not_identical'])
    assert 'source occupancy cells, resolution, origin, and metric extent' in (
        contract['fidelity']['preserved'])
