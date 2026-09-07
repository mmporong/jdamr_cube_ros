#!/usr/bin/env python3
"""Generate deterministic simulation assets for G005 frontier evaluation."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

from frontier_policy_contract import (
    ARTIFACT_LIMIT_BYTES,
    build_layout,
    canonical_json_bytes,
    connected_reachable_cells,
    file_identity,
    FOOTPRINT_CLEARANCE_M,
    FULL_PLAN,
    LAYOUT_SEEDS,
    LIDAR_BASE_YAW_RAD,
    LIDAR_BEAMS,
    LIDAR_MAX_ANGLE_RAD,
    LIDAR_MIN_ANGLE_RAD,
    LIDAR_MIN_RANGE_M,
    LIDAR_RANGE_M,
    LIDAR_RATE_HZ,
    POLICIES,
    sha256_file,
    strict_json_load,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
PRODUCTION_INPUTS = {
    'frontier_core': REPO_ROOT / 'jdamr_cube_navigation' /
    'jdamr_cube_navigation' / 'frontier_core.py',
    'frontier_explorer': REPO_ROOT / 'jdamr_cube_navigation' /
    'jdamr_cube_navigation' / 'frontier_explorer.py',
    'nav2_params': REPO_ROOT / 'jdamr_cube_navigation' / 'config' /
    'nav2_params.yaml',
    'cartographer_config': REPO_ROOT / 'jdamr_cube_cartographer' / 'config' /
    'jdamr_cube_2d.lua',
    'robot_urdf': REPO_ROOT / 'jdamr_cube_description' / 'urdf' /
    'jdamr_cube.urdf',
}
EVALUATION_ROBOT_NAME = 'g005_robot.urdf'
EVALUATION_NAV2_PARAMS_NAME = 'g005_nav2_params.yaml'


def _tree_records(root: Path, names: list[str]) -> tuple[list[dict], str]:
    records = [file_identity(root / name, relative_to=root)
               for name in sorted(names)]
    return records, hashlib.sha256(canonical_json_bytes(records)).hexdigest()


def _occupied_rectangles(layout: dict) -> list[tuple[int, int, int, int]]:
    """Merge occupied cells into deterministic axis-aligned rectangles."""
    width = layout['width']
    active = {}
    rectangles = []
    for y in range(layout['height']):
        runs = []
        x = 0
        while x < width:
            if layout['data'][y * width + x] < 65:
                x += 1
                continue
            x0 = x
            while x < width and layout['data'][y * width + x] >= 65:
                x += 1
            runs.append((x0, x))
        current = {}
        for run in runs:
            if run in active:
                x0, x1, y0, _ = active[run]
                current[run] = (x0, x1, y0, y + 1)
            else:
                current[run] = (run[0], run[1], y, y + 1)
        rectangles.extend(
            active[run] for run in sorted(set(active) - set(current)))
        active = current
    rectangles.extend(active[run] for run in sorted(active))
    return sorted(
        rectangles, key=lambda item: (item[2], item[0], item[3], item[1]))


def _sdf(layout: dict) -> bytes:
    resolution_m = layout['resolution_m_per_cell']
    origin_x_m, origin_y_m, _ = layout['origin_m_rad']
    models = []
    for index, (x0, x1, y0, y1) in enumerate(_occupied_rectangles(layout)):
        x_size_m = (x1 - x0) * resolution_m
        y_size_m = (y1 - y0) * resolution_m
        x_m = origin_x_m + (x0 + (x1 - x0) / 2.0) * resolution_m
        y_m = origin_y_m + (y0 + (y1 - y0) / 2.0) * resolution_m
        models.append(
            f'<model name="wall_rect_{index}"><static>true</static>'
            f'<pose>{x_m:.6f} {y_m:.6f} 0.5 0 0 0</pose><link name="body">'
            f'<collision name="collision"><geometry><box><size>'
            f'{x_size_m:.6f} {y_size_m:.6f} 1.0</size></box>'
            f'</geometry></collision><visual name="visual"><geometry>'
            f'<box><size>'
            f'{x_size_m:.6f} {y_size_m:.6f} 1.0</size></box>'
            f'</geometry></visual><sensor name="contact_sensor" '
            f'type="contact"><always_on>true</always_on>'
            f'<update_rate>50</update_rate><contact>'
            f'<collision>collision</collision><topic>/g005_contacts</topic>'
            f'</contact></sensor>'
            f'</link></model>')
    content = ('<?xml version="1.0"?><sdf version="1.9">'
               '<world name="g005_frontier"><physics name="fixed" '
               'type="ignored"><max_step_size>0.001</max_step_size>'
               '<real_time_factor>1.0</real_time_factor></physics>'
               '<plugin filename="gz-sim-physics-system" '
               'name="gz::sim::systems::Physics"/>'
               '<plugin filename="gz-sim-user-commands-system" '
               'name="gz::sim::systems::UserCommands"/>'
               '<plugin filename="gz-sim-scene-broadcaster-system" '
               'name="gz::sim::systems::SceneBroadcaster"/>'
               '<plugin filename="gz-sim-contact-system" '
               'name="gz::sim::systems::Contact"/>'
               '<plugin filename="gz-sim-sensors-system" '
               'name="gz::sim::systems::Sensors">'
               '<render_engine>ogre2</render_engine></plugin>'
               '<plugin filename="gz-sim-imu-system" '
               'name="gz::sim::systems::Imu"/>'
               '<light type="directional" name="sun"><cast_shadows>true'
               '</cast_shadows><pose>0 0 10 0 0 0</pose>'
               '<diffuse>0.8 0.8 0.8 1</diffuse>'
               '<specular>0.2 0.2 0.2 1</specular>'
               '<direction>-0.5 0.1 -0.9</direction></light>'
               '<model name="ground_plane"><static>true</static>'
               '<link name="link"><collision name="collision"><geometry>'
               '<plane><normal>0 0 1</normal><size>100 100</size></plane>'
               '</geometry></collision><visual name="visual"><geometry>'
               '<plane><normal>0 0 1</normal><size>100 100</size></plane>'
               '</geometry></visual></link></model>'
               + ''.join(models) + '</world></sdf>\n')
    return content.encode()


def _evaluation_robot_urdf() -> bytes:
    """Create the sealed robot variant with the preregistered LiDAR rate."""
    root = ET.parse(PRODUCTION_INPUTS['robot_urdf']).getroot()
    sensors = [sensor for gazebo in root.findall('gazebo')
               if gazebo.attrib.get('reference') == 'laser_link'
               for sensor in gazebo.findall('sensor')
               if sensor.attrib.get('name') == 'laser_sensor']
    if len(sensors) != 1:
        raise ValueError('G005 source LiDAR sensor identity drift')
    sensor = sensors[0]
    laser_joints = [joint for joint in root.findall('joint')
                    if joint.attrib.get('name') == 'laser_joint']
    if len(laser_joints) != 1:
        raise ValueError('G005 source LiDAR extrinsic identity drift')
    laser_origin = laser_joints[0].find('origin')
    update_rate = sensor.find('update_rate')
    horizontal = sensor.find('lidar/scan/horizontal')
    samples = (None if horizontal is None else horizontal.find('samples'))
    minimum_angle = (
        None if horizontal is None else horizontal.find('min_angle'))
    maximum_angle = (
        None if horizontal is None else horizontal.find('max_angle'))
    minimum = sensor.find('lidar/range/min')
    maximum = sensor.find('lidar/range/max')
    if (laser_origin is None or
            laser_origin.attrib.get('xyz') != '0 0 0.1' or
            laser_origin.attrib.get('rpy') != '0 0 3.141592653589793' or
            update_rate is None or samples is None or minimum_angle is None or
            maximum_angle is None or minimum is None or maximum is None or
            samples.text != str(LIDAR_BEAMS) or
            float(minimum_angle.text) != LIDAR_MIN_ANGLE_RAD or
            float(maximum_angle.text) != LIDAR_MAX_ANGLE_RAD or
            float(minimum.text) != LIDAR_MIN_RANGE_M or
            float(maximum.text) != LIDAR_RANGE_M):
        raise ValueError('G005 source LiDAR profile drift')
    update_rate.text = f'{LIDAR_RATE_HZ:.1f}'
    return ET.tostring(root, encoding='utf-8', xml_declaration=True) + b'\n'


def _evaluation_nav2_params() -> bytes:
    """Derive frozen-planner params without changing production config."""
    source = PRODUCTION_INPUTS['nav2_params'].read_bytes()
    marker = b'\nglobal_costmap:\n'
    if source.count(marker) != 1:
        raise ValueError('G005 source global costmap identity drift')
    prefix, global_section = source.split(marker, 1)
    plugins = b'plugins: ["static_layer", "obstacle_layer", "inflation_layer"]'
    filters = b'      filters: ["keepout_filter"]\n'
    if (global_section.count(plugins) != 1 or
            global_section.count(filters) != 1):
        raise ValueError('G005 source global costmap plugin drift')
    global_section = global_section.replace(
        plugins, b'plugins: ["static_layer", "inflation_layer"]', 1)
    global_section = global_section.replace(filters, b'', 1)
    return prefix + marker + global_section


def validate_assets(root: Path, expected_mode: str) -> dict:
    """Rehash the exact generated tree and validate every asset contract."""
    if (not root.is_absolute() or not root.is_dir() or root.is_symlink() or
            expected_mode not in ('smoke', 'full')):
        raise ValueError('non-canonical G005 asset root')
    actual = sorted(path.name for path in root.iterdir())
    seeds = (11,) if expected_mode == 'smoke' else LAYOUT_SEEDS
    expected_assets = [item for seed in seeds for item in (
        f'layout_{seed}.world', f'layout_{seed}_gt.json')]
    expected_assets.extend(
        (EVALUATION_NAV2_PARAMS_NAME, EVALUATION_ROBOT_NAME))
    expected = sorted(expected_assets + ['asset_manifest.json'])
    if actual != expected or any(path.is_symlink() or not path.is_file()
                                 for path in root.iterdir()):
        raise ValueError('G005 asset tree inventory drift')
    manifest = strict_json_load(root / 'asset_manifest.json')
    keys = {'schema_version', 'mode', 'layout_seeds', 'full_plan',
            'policies', 'layout_contract', 'layouts', 'production_inputs',
            'asset_files', 'asset_tree_sha256', 'generator_source',
            'storage_limit_bytes'}
    if (type(manifest) is not dict or set(manifest) != keys or
            manifest['schema_version'] != 1 or
            manifest['mode'] != expected_mode or
            manifest['layout_seeds'] != list(seeds) or
            manifest['policies'] != list(POLICIES) or
            manifest['full_plan'] != [
                {'policy': policy, 'layout_seed': seed}
                for policy, seed in FULL_PLAN] or
            manifest['storage_limit_bytes'] != ARTIFACT_LIMIT_BYTES):
        raise ValueError('G005 asset manifest schema drift')
    expected_layout_contract = {
        'resolution_m_per_cell': 0.25,
        'footprint_clearance_m': FOOTPRINT_CLEARANCE_M,
        'frames': {'gazebo_world': 'g005_frontier', 'map': 'map',
                   'relationship': 'IDENTITY'},
        'lidar': {'rate_hz': LIDAR_RATE_HZ, 'beam_count': LIDAR_BEAMS,
                  'min_range_m': LIDAR_MIN_RANGE_M,
                  'range_m': LIDAR_RANGE_M,
                  'min_angle_rad': LIDAR_MIN_ANGLE_RAD,
                  'max_angle_rad': LIDAR_MAX_ANGLE_RAD,
                  'base_yaw_rad': LIDAR_BASE_YAW_RAD},
        'reveal': {'monotonic': True, 'dropout_per_mille': 20,
                   'delay_scan_values': [0, 1, 2],
                   'noise_key': ['layout_seed', 'sensor_origin_cell',
                                 'beam_bin', 'target_cell']}}
    if (manifest['layout_contract'] != expected_layout_contract or
            type(manifest['layouts']) is not dict or
            set(manifest['layouts']) != {str(seed) for seed in seeds}):
        raise ValueError('G005 layout contract drift')
    records, tree_sha = _tree_records(root, expected_assets)
    if (manifest['asset_files'] != records or
            manifest['asset_tree_sha256'] != tree_sha):
        raise ValueError('G005 asset tree hash drift')
    if manifest['generator_source'] != file_identity(Path(__file__).resolve()):
        raise ValueError('G005 generator source drift')
    expected_production = {name: file_identity(path)
                           for name, path in PRODUCTION_INPUTS.items()}
    if manifest['production_inputs'] != expected_production:
        raise ValueError('G005 production input drift')
    if (root / EVALUATION_ROBOT_NAME).read_bytes() != (
            _evaluation_robot_urdf()):
        raise ValueError('G005 evaluation robot profile drift')
    if (root / EVALUATION_NAV2_PARAMS_NAME).read_bytes() != (
            _evaluation_nav2_params()):
        raise ValueError('G005 evaluation Nav2 profile drift')
    for seed in seeds:
        layout = strict_json_load(root / f'layout_{seed}_gt.json')
        expected_layout = build_layout(seed)
        if layout != expected_layout:
            raise ValueError('G005 GT occupancy drift')
        if (root / f'layout_{seed}.world').read_bytes() != _sdf(
                expected_layout):
            raise ValueError('G005 SDF geometry drift')
        record = manifest['layouts'][str(seed)]
        if (type(record) is not dict or set(record) != {
                'gazebo_seed', 'start_cell_index',
                'reachable_denominator_cells', 'reachable_cell_indices',
                'gt_occupancy_sha256', 'sdf_sha256'} or
                record['gazebo_seed'] != seed or
                record['start_cell_index'] != (
                    layout['start_cell'][1] * layout['width'] +
                    layout['start_cell'][0]) or
                record['gt_occupancy_sha256'] != sha256_file(
                    root / f'layout_{seed}_gt.json') or
                record['sdf_sha256'] != sha256_file(
                    root / f'layout_{seed}.world') or
                record['reachable_cell_indices'] !=
                connected_reachable_cells(layout) or
                record['reachable_denominator_cells'] !=
                len(record['reachable_cell_indices'])):
            raise ValueError('G005 reachable denominator drift')
    if sum(path.stat().st_size for path in root.iterdir()) > (
            ARTIFACT_LIMIT_BYTES):
        raise ValueError('G005 asset storage cap exceeded')
    return manifest


def generate(output_root: Path, mode: str) -> dict:
    """Atomically create smoke or full preregistered layout assets."""
    if (output_root.exists() or output_root.is_symlink() or
            not output_root.is_absolute()):
        raise ValueError('G005 output root must be absent and absolute')
    if mode not in ('smoke', 'full'):
        raise ValueError('unknown G005 generation mode')
    seeds = (11,) if mode == 'smoke' else LAYOUT_SEEDS
    with tempfile.TemporaryDirectory(
            prefix='.g005-frontier-assets-', dir=output_root.parent) as temp:
        stage = Path(temp) / 'assets'
        stage.mkdir()
        layouts = {}
        names = [EVALUATION_NAV2_PARAMS_NAME, EVALUATION_ROBOT_NAME]
        (stage / EVALUATION_NAV2_PARAMS_NAME).write_bytes(
            _evaluation_nav2_params())
        (stage / EVALUATION_ROBOT_NAME).write_bytes(
            _evaluation_robot_urdf())
        for seed in seeds:
            layout = build_layout(seed)
            gt_name = f'layout_{seed}_gt.json'
            world_name = f'layout_{seed}.world'
            (stage / gt_name).write_bytes(canonical_json_bytes(layout))
            (stage / world_name).write_bytes(_sdf(layout))
            names.extend((gt_name, world_name))
            reachable = connected_reachable_cells(layout)
            start_index = (layout['start_cell'][1] * layout['width'] +
                           layout['start_cell'][0])
            layouts[str(seed)] = {
                'gazebo_seed': seed, 'start_cell_index': start_index,
                'reachable_denominator_cells': len(reachable),
                'reachable_cell_indices': reachable,
                'gt_occupancy_sha256': sha256_file(stage / gt_name),
                'sdf_sha256': sha256_file(stage / world_name)}
        records, tree_sha = _tree_records(stage, names)
        manifest = {
            'schema_version': 1, 'mode': mode,
            'layout_seeds': list(seeds),
            'full_plan': [{'policy': policy, 'layout_seed': seed}
                          for policy, seed in FULL_PLAN],
            'policies': list(POLICIES),
            'layout_contract': {
                'resolution_m_per_cell': 0.25,
                'footprint_clearance_m': FOOTPRINT_CLEARANCE_M,
                'frames': {'gazebo_world': 'g005_frontier', 'map': 'map',
                           'relationship': 'IDENTITY'},
                'lidar': {
                    'rate_hz': LIDAR_RATE_HZ,
                    'beam_count': LIDAR_BEAMS,
                    'min_range_m': LIDAR_MIN_RANGE_M,
                    'range_m': LIDAR_RANGE_M,
                    'min_angle_rad': LIDAR_MIN_ANGLE_RAD,
                    'max_angle_rad': LIDAR_MAX_ANGLE_RAD,
                    'base_yaw_rad': LIDAR_BASE_YAW_RAD,
                },
                'reveal': {'monotonic': True, 'dropout_per_mille': 20,
                           'delay_scan_values': [0, 1, 2],
                           'noise_key': ['layout_seed', 'sensor_origin_cell',
                                         'beam_bin', 'target_cell']}},
            'layouts': layouts,
            'production_inputs': {name: file_identity(path)
                                  for name, path in PRODUCTION_INPUTS.items()},
            'asset_files': records, 'asset_tree_sha256': tree_sha,
            'generator_source': file_identity(Path(__file__).resolve()),
            'storage_limit_bytes': ARTIFACT_LIMIT_BYTES}
        (stage / 'asset_manifest.json').write_bytes(
            canonical_json_bytes(manifest))
        validate_assets(stage, mode)
        stage.rename(output_root)
    try:
        return validate_assets(output_root, mode)
    except Exception:
        if output_root.is_dir() and not output_root.is_symlink():
            shutil.rmtree(output_root)
        raise


def main() -> int:
    """Generate one exact G005 asset root."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--mode', choices=('smoke', 'full'), required=True)
    args = parser.parse_args()
    generate(args.output_root, args.mode)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
