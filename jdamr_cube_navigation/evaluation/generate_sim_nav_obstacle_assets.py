#!/usr/bin/env python3
"""Generate immutable evaluation world and map from the static corridor."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path  # noqa: I100
from statistics import NormalDist  # noqa: I100

from sim_nav_obstacle_contract import SEEDS

import yaml


ROOT = Path(__file__).resolve().parents[2]
SOURCE_WORLD = ROOT / 'jdamr_cube_gazebo' / 'worlds' / 'slam_corridor.world'
SOURCE_URDF = (ROOT / 'jdamr_cube_description' / 'urdf'
               / 'jdamr_cube.urdf')
REAL_SCAN_REFERENCE = (
    Path(__file__).resolve().parent / 'real_scan_reference_g4.json')
RESOLUTION_M = 0.05
ORIGIN_X_M = -11.0
ORIGIN_Y_M = -2.2
WIDTH_CELLS = 440
HEIGHT_CELLS = 88
LIDAR_VISIBILITY_MASK = 4
ROBOT_VISIBILITY_FLAGS = 11
WALL_BEAM_FAMILYWISE_ALPHA = 0.01
DRIVE_WHEEL_LINKS = ('left_wheel_link', 'right_wheel_link')
DRIVE_WHEEL_RADIUS_M = 0.075
DRIVE_WHEEL_NORMAL_FORCE_N = 70.0


def sha256_file(path: Path) -> str:
    """Return a file SHA-256 digest."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def derive_wall_beam_multiple_comparison_contract(
        noise_stddev_m: float, direction_count: int,
        replicate_seeds: tuple[int, ...],
        familywise_alpha: float = WALL_BEAM_FAMILYWISE_ALPHA
        ) -> dict[str, float | int | bool | str]:
    """Derive the two-sided Bonferroni gate for direction mean bias."""
    if not 0.0 < familywise_alpha < 1.0:
        raise ValueError('familywise alpha must be between zero and one')
    replicate_count = len(replicate_seeds)
    if (direction_count <= 0 or replicate_count <= 0
            or len(set(replicate_seeds)) != replicate_count
            or noise_stddev_m <= 0.0):
        raise ValueError(
            'direction, replicate, and noise counts must be positive')
    critical_z = NormalDist().inv_cdf(
        1.0 - familywise_alpha / (2.0 * direction_count))
    standard_error_m = noise_stddev_m / math.sqrt(replicate_count)
    return {
        'protocol_version': 2,
        'unit_of_analysis': 'direction_mean_bias',
        'familywise_alpha': familywise_alpha,
        'comparison_count': direction_count,
        'direction_count': direction_count,
        'replicate_count': replicate_count,
        'replicate_seeds': list(replicate_seeds),
        'two_sided': True,
        'correction': 'bonferroni',
        'noise_sigma_m': noise_stddev_m,
        'standard_error_m': standard_error_m,
        'critical_z': critical_z,
        'wall_beam_mean_tolerance_m': critical_z * standard_error_m,
        'formula': 'NormalDist.inv_cdf(1-alpha/(2*m))*sigma/sqrt(n)',
    }


def _pose_xyz(element: ET.Element | None) -> tuple[float, float, float]:
    if element is None or not element.text:
        return 0.0, 0.0, 0.0
    values = [float(value) for value in element.text.split()]
    values.extend([0.0] * (3 - len(values)))
    return values[0], values[1], values[2]


def derive_level_caster_geometry(root: ET.Element) -> dict[str, float]:
    """Derive the caster joint height that matches the drive-wheel bottom."""
    wheel_joint = root.find("./joint[@name='left_wheel_joint']/origin")
    wheel = root.find(
        "./link[@name='left_wheel_link']/collision/geometry/cylinder")
    caster_joint = root.find("./joint[@name='caster_front_joint']/origin")
    caster = root.find(
        "./link[@name='caster_link_front']/collision/geometry/sphere")
    if None in (wheel_joint, wheel, caster_joint, caster):
        raise ValueError('wheel or caster support geometry is incomplete')
    wheel_joint_z_m = float(wheel_joint.attrib['xyz'].split()[2])
    wheel_radius_m = float(wheel.attrib['radius'])
    caster_joint_z_m = float(caster_joint.attrib['xyz'].split()[2])
    caster_radius_m = float(caster.attrib['radius'])
    wheel_bottom_z_m = wheel_joint_z_m - wheel_radius_m
    caster_bottom_z_m = caster_joint_z_m - caster_radius_m
    return {
        'wheel_bottom_from_base_m': wheel_bottom_z_m,
        'caster_bottom_from_base_m': caster_bottom_z_m,
        'current_support_height_delta_m': (
            caster_bottom_z_m - wheel_bottom_z_m),
        'level_caster_joint_z_m': wheel_bottom_z_m + caster_radius_m,
    }


def derive_front_observation_probe() -> dict[str, float | bool]:
    """Derive a noncontact probe centered in the required distance band."""
    start_x_m = -8.0
    observation_distance_min_m = 1.0
    observation_distance_max_m = 2.0
    center_distance_m = (
        observation_distance_min_m + observation_distance_max_m) / 2.0
    length_m = 0.5
    footprint_front_x_m = 0.23
    near_surface_distance_m = center_distance_m - length_m / 2.0
    return {
        'center_x_m': start_x_m + center_distance_m,
        'center_y_m': 0.0,
        'length_m': length_m,
        'width_m': 0.4,
        'start_to_center_m': center_distance_m,
        'start_to_near_surface_m': near_surface_distance_m,
        'footprint_to_near_surface_m': (
            near_surface_distance_m - footprint_front_x_m),
        'contact_expected': False,
    }


def derive_sub_minimum_range_probe(
        range_min_m: float) -> dict[str, float | bool]:
    """Derive a noncontact target inside the reported minimum range."""
    footprint_half_width_m = 0.20
    target_thickness_m = 0.02
    near_surface_m = (footprint_half_width_m + range_min_m) / 2.0
    center_y_m = near_surface_m + target_thickness_m / 2.0
    return {
        'center_x_m': -8.0,
        'center_y_m': center_y_m,
        'length_m': target_thickness_m,
        'width_m': target_thickness_m,
        'near_surface_m': near_surface_m,
        'footprint_clearance_m': near_surface_m - footprint_half_width_m,
        'scan_return_expected': False,
        'contact_expected': False,
    }


def derive_preloaded_obstacles(range_min_m: float) -> dict[str, dict]:
    """Derive the parked models used by every evaluation world."""
    front = derive_front_observation_probe()
    sub_minimum = derive_sub_minimum_range_probe(range_min_m)
    definitions = (
        ('route', 'g003_preloaded_route_obstacle', 0.50, 0.40, -1.0, 0.0),
        ('full_block', 'g003_preloaded_full_block', 0.50, 2.30, -1.0, 0.0),
        ('goal_occupied', 'g003_preloaded_goal_occupied',
         1.70, 1.70, 6.0, 0.0),
        ('sub_minimum_probe', 'g003_preloaded_sub_minimum_probe',
         sub_minimum['length_m'], sub_minimum['width_m'],
         sub_minimum['center_x_m'], sub_minimum['center_y_m']),
        ('front_observation_probe', 'g003_preloaded_front_observation_probe',
         front['length_m'], front['width_m'],
         front['center_x_m'], front['center_y_m']),
        ('contact_control', 'g003_preloaded_contact_control',
         0.14, 0.12, -7.80, 0.0),
    )
    park_start_y_m = 20.0
    park_spacing_y_m = 5.0
    return {
        role: {
            'role': role,
            'name': name,
            'static': True,
            'length_m': length_m,
            'width_m': width_m,
            'height_m': 0.40 if role == 'contact_control' else 1.0,
            'active_pose_m': [
                active_x_m, active_y_m,
                0.3 if role == 'contact_control' else 0.5],
            'park_pose_m': [
                0.0, park_start_y_m + index * park_spacing_y_m,
                0.20 if role == 'contact_control' else 0.5],
            'visual_visibility_flags': LIDAR_VISIBILITY_MASK,
            'contact_collision_ref': 'collision',
        }
        for index, (role, name, length_m, width_m,
                    active_x_m, active_y_m) in enumerate(definitions)
    }


def _preloaded_model(spec: dict) -> ET.Element:
    model = ET.Element('model', {'name': spec['name']})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = ' '.join(
        str(value) for value in (*spec['park_pose_m'], 0.0, 0.0, 0.0))
    link = ET.SubElement(model, 'link', {'name': 'body'})
    size = ' '.join(str(spec[key]) for key in (
        'length_m', 'width_m', 'height_m'))
    collision = ET.SubElement(link, 'collision', {'name': 'collision'})
    collision_box = ET.SubElement(
        ET.SubElement(collision, 'geometry'), 'box')
    ET.SubElement(collision_box, 'size').text = size
    visual = ET.SubElement(link, 'visual', {'name': 'visual'})
    visual_box = ET.SubElement(ET.SubElement(visual, 'geometry'), 'box')
    ET.SubElement(visual_box, 'size').text = size
    ET.SubElement(visual, 'visibility_flags').text = str(
        spec['visual_visibility_flags'])
    sensor = ET.SubElement(
        link, 'sensor', {'name': 'contact_sensor', 'type': 'contact'})
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '50'
    contact = ET.SubElement(sensor, 'contact')
    ET.SubElement(contact, 'collision').text = spec[
        'contact_collision_ref']
    return model


def _static_boxes(
        world: ET.Element) -> list[tuple[float, float, float, float]]:
    boxes = []
    for model in world.findall('model'):
        if model.attrib.get('name') == 'ground_plane':
            continue
        if (model.findtext('static') or '').strip().lower() != 'true':
            continue
        model_x_m, model_y_m, _ = _pose_xyz(model.find('pose'))
        for link in model.findall('link'):
            link_x_m, link_y_m, _ = _pose_xyz(link.find('pose'))
            for collision in link.findall('collision'):
                box = collision.find('./geometry/box/size')
                if box is None or not box.text:
                    continue
                size = [float(value) for value in box.text.split()]
                collision_x_m, collision_y_m, _ = _pose_xyz(
                    collision.find('pose'))
                boxes.append((
                    model_x_m + link_x_m + collision_x_m,
                    model_y_m + link_y_m + collision_y_m,
                    size[0], size[1]))
    return boxes


def _expected_wall_beams(world: ET.Element) -> dict[str, float]:
    def inner_distance(model_name: str, axis: int, start_m: float) -> float:
        model = world.find(f"model[@name='{model_name}']")
        if model is None:
            raise ValueError(f'missing wall model: {model_name}')
        center_m = float(model.findtext('pose').split()[axis])
        size_m = float(model.findtext(
            'link/collision/geometry/box/size').split()[axis])
        inner_m = center_m - math.copysign(size_m / 2.0, center_m)
        return abs(inner_m - start_m)

    return {
        'south': inner_distance('wall_south', 1, 0.0),
        'west': inner_distance('wall_west', 0, -8.0),
        'north': inner_distance('wall_north', 1, 0.0),
    }


def _rasterize(boxes: list[tuple[float, float, float, float]]) -> bytes:
    pixels = bytearray([254] * (WIDTH_CELLS * HEIGHT_CELLS))
    for center_x_m, center_y_m, length_m, width_m in boxes:
        min_x = math.floor(
            (center_x_m - length_m / 2.0 - ORIGIN_X_M) / RESOLUTION_M)
        max_x = math.ceil(
            (center_x_m + length_m / 2.0 - ORIGIN_X_M) / RESOLUTION_M)
        min_y = math.floor(
            (center_y_m - width_m / 2.0 - ORIGIN_Y_M) / RESOLUTION_M)
        max_y = math.ceil(
            (center_y_m + width_m / 2.0 - ORIGIN_Y_M) / RESOLUTION_M)
        for map_y in range(max(0, min_y), min(HEIGHT_CELLS, max_y)):
            pgm_y = HEIGHT_CELLS - 1 - map_y
            offset = pgm_y * WIDTH_CELLS
            for map_x in range(max(0, min_x), min(WIDTH_CELLS, max_x)):
                pixels[offset + map_x] = 0
    header = f'P5\n{WIDTH_CELLS} {HEIGHT_CELLS}\n255\n'.encode('ascii')
    return header + bytes(pixels)


def _evaluation_world(source: Path) -> bytes:
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('source SDF has no world')
    if not any(plugin.attrib.get('filename') == 'gz-sim-contact-system'
               for plugin in world.findall('plugin')):
        contact = ET.Element('plugin', {
            'filename': 'gz-sim-contact-system',
            'name': 'gz::sim::systems::Contact',
        })
        user_commands_index = next(
            (index for index, child in enumerate(world)
             if child.tag == 'plugin'
             and child.attrib.get('filename')
             == 'gz-sim-user-commands-system'), 0)
        world.insert(user_commands_index + 1, contact)
    real_scan_reference = json.loads(
        REAL_SCAN_REFERENCE.read_text(encoding='utf-8'))
    preloaded = derive_preloaded_obstacles(
        real_scan_reference['range_min_m'])
    for spec in preloaded.values():
        world.append(_preloaded_model(spec))
    ET.indent(tree, space='  ')
    return ET.tostring(tree.getroot(), encoding='utf-8', xml_declaration=True)


def _element_sha256(element: ET.Element) -> str:
    def normalized(node: ET.Element) -> dict:
        return {
            'tag': node.tag,
            'attributes': dict(sorted(node.attrib.items())),
            'text': (node.text or '').strip(),
            'children': [normalized(child) for child in node],
        }

    payload = json.dumps(
        normalized(element), sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()


def _collision_records(root: ET.Element) -> list[dict[str, object]]:
    records = []
    for link in root.findall('link'):
        for index, collision in enumerate(link.findall('collision')):
            records.append({
                'link': link.attrib['name'],
                'index': index,
                'name': collision.attrib.get('name', '<unnamed>'),
                'xml_sha256': _element_sha256(collision),
            })
    return records


def _joint_records(root: ET.Element) -> list[dict[str, object]]:
    return [{
        'name': joint.attrib['name'],
        'type': joint.attrib['type'],
        'xml_sha256': _element_sha256(joint),
    } for joint in root.findall('joint')]


def _inertial_records(root: ET.Element) -> list[dict[str, object]]:
    return [{
        'link': link.attrib['name'],
        'xml_sha256': _element_sha256(inertial),
    } for link in root.findall('link')
        for inertial in link.findall('inertial')]


def _visual_records(root: ET.Element) -> list[dict[str, object]]:
    records = []
    for link in root.findall('link'):
        for index, visual in enumerate(link.findall('visual')):
            records.append({
                'link': link.attrib['name'],
                'index': index,
                'name': visual.attrib.get('name', '<unnamed>'),
                'xml_sha256': _element_sha256(visual),
            })
    return records


def _lidar_sensor_record(root: ET.Element) -> dict[str, object]:
    sensor = root.find(
        "./gazebo[@reference='laser_link']/sensor[@name='laser_sensor']")
    if sensor is None:
        raise ValueError('missing laser sensor')
    visibility_mask = sensor.findtext('lidar/visibility_mask')
    return {
        'sensor_pose': sensor.findtext('pose'),
        'frame_id': sensor.findtext('gz_frame_id'),
        'topic': sensor.findtext('topic'),
        'sample_count': int(sensor.findtext(
            'lidar/scan/horizontal/samples')),
        'horizontal_resolution': float(sensor.findtext(
            'lidar/scan/horizontal/resolution')),
        'min_angle_rad': float(sensor.findtext(
            'lidar/scan/horizontal/min_angle')),
        'max_angle_rad': float(sensor.findtext(
            'lidar/scan/horizontal/max_angle')),
        'range_min_m': float(sensor.findtext('lidar/range/min')),
        'range_max_m': float(sensor.findtext('lidar/range/max')),
        'range_resolution_m': float(sensor.findtext(
            'lidar/range/resolution')),
        'noise_type': sensor.findtext('lidar/noise/type'),
        'noise_mean_m': float(sensor.findtext('lidar/noise/mean')),
        'noise_stddev_m': float(sensor.findtext('lidar/noise/stddev')),
        'update_rate_hz': float(sensor.findtext('update_rate')),
        'visibility_mask': (
            int(visibility_mask) if visibility_mask is not None else None),
    }


def _convert_sdf(urdf_path: Path) -> tuple[ET.Element, str]:
    environment = os.environ.copy()
    if not environment.get('GZ_CONFIG_PATH'):
        config_paths = sorted(
            str(path) for path in Path('/opt/ros').glob('*/opt/*/share/gz'))
        if config_paths:
            environment['GZ_CONFIG_PATH'] = os.pathsep.join(config_paths)
    if not environment.get('LD_LIBRARY_PATH'):
        library_paths = sorted(
            str(path) for path in Path('/opt/ros').glob('*/opt/*/lib'))
        library_paths.extend(
            str(path) for path in Path('/opt/ros').glob('*/lib*')
            if path.is_dir())
        if library_paths:
            environment['LD_LIBRARY_PATH'] = os.pathsep.join(library_paths)
    result = subprocess.run(
        ['gz', 'sdf', '-p', str(urdf_path.resolve())],
        env=environment, capture_output=True, text=True, timeout=20.0,
        check=False)
    if result.returncode != 0:
        raise RuntimeError(
            f'gz sdf conversion failed ({result.returncode}): '
            f'{result.stderr.strip()}')
    return ET.fromstring(result.stdout), result.stdout


def _sdf_collision_pose_z(root: ET.Element, name_fragment: str) -> float:
    collision = next(
        item for item in root.findall('.//collision')
        if name_fragment in item.attrib.get('name', ''))
    pose = collision.findtext('pose', default='0 0 0 0 0 0').split()
    return float(pose[2])


def _sdf_named_records(root: ET.Element, tag: str) -> list[dict[str, str]]:
    return [{
        'name': element.attrib.get('name', '<unnamed>'),
        'xml_sha256': _element_sha256(element),
    } for element in root.findall(f'.//{tag}')]


def _sdf_collision_semantic_records(
        root: ET.Element) -> list[dict[str, str]]:
    records = []
    for collision in root.findall('.//collision'):
        preserved = ET.fromstring(ET.tostring(collision, encoding='utf-8'))
        surface = preserved.find('surface')
        empty_default_surface = (
            surface is not None
            and not any(node.attrib or (node.text or '').strip()
                        for node in surface.iter())
            and [node.tag for node in surface.iter()] == [
                'surface', 'contact', 'ode', 'friction', 'ode'])
        if empty_default_surface:
            preserved.remove(surface)
        records.append({
            'name': collision.attrib.get('name', '<unnamed>'),
            'xml_sha256': _element_sha256(preserved),
        })
    return records


def _sdf_visual_semantic_records(
        root: ET.Element,
        caster_pose_z_adjustment_m: float = 0.0) -> list[dict[str, str]]:
    records = []
    for visual in root.findall('.//visual'):
        preserved = ET.fromstring(ET.tostring(visual, encoding='utf-8'))
        for visibility in preserved.findall('visibility_flags'):
            preserved.remove(visibility)
        pose = preserved.find('pose')
        if pose is not None and pose.text:
            values = [float(value) for value in pose.text.split()]
            if 'caster_link_' in visual.attrib.get('name', ''):
                values[2] += caster_pose_z_adjustment_m
            pose.text = ' '.join(format(value, '.12g') for value in values)
        elif 'caster_link_' in visual.attrib.get('name', ''):
            raise ValueError('caster visual is missing its converted pose')
        records.append({
            'name': visual.attrib.get('name', '<unnamed>'),
            'xml_sha256': _element_sha256(preserved),
        })
    return records


def _parallel_axis(position: list[float]) -> list[list[float]]:
    x_m, y_m, z_m = position
    return [
        [y_m * y_m + z_m * z_m, -x_m * y_m, -x_m * z_m],
        [-x_m * y_m, x_m * x_m + z_m * z_m, -y_m * z_m],
        [-x_m * z_m, -y_m * z_m, x_m * x_m + y_m * y_m],
    ]


def _matrix_add(first, second, scale=1.0):
    return [[first[row][column] + scale * second[row][column]
             for column in range(3)] for row in range(3)]


def _sdf_inertial(link: ET.Element) -> tuple[float, list[float], list]:
    inertial = link.find('inertial')
    if inertial is None:
        raise ValueError(
            f'missing inertial for SDF link {link.attrib["name"]}')
    pose = [float(value) for value in inertial.findtext(
        'pose', default='0 0 0 0 0 0').split()[:3]]
    tensor = inertial.find('inertia')
    if tensor is None:
        raise ValueError(f'missing inertia for SDF link {link.attrib["name"]}')
    values = {name: float(tensor.findtext(name)) for name in (
        'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
    matrix = [
        [values['ixx'], values['ixy'], values['ixz']],
        [values['ixy'], values['iyy'], values['iyz']],
        [values['ixz'], values['iyz'], values['izz']],
    ]
    return float(inertial.findtext('mass')), pose, matrix


def _validate_converted_sdf(
        source_urdf: Path, generated_urdf: Path) -> dict[str, object]:
    source_root, source_text = _convert_sdf(source_urdf)
    generated_root, generated_text = _convert_sdf(generated_urdf)
    source_collisions = source_root.findall('.//collision')
    generated_collisions = generated_root.findall('.//collision')
    source_arm_count = sum(
        'arm_' in item.attrib.get('name', '') for item in source_collisions)
    generated_arm_count = sum(
        'arm_' in item.attrib.get('name', '') for item in generated_collisions)
    caster_deltas = {}
    for name in ('caster_link_front_collision',
                 'caster_link_rear_collision'):
        caster_deltas[name] = (
            _sdf_collision_pose_z(generated_root, name)
            - _sdf_collision_pose_z(source_root, name))
    source_collision_records = [
        item for item in _sdf_collision_semantic_records(source_root)
        if 'caster_link_' not in item['name']]
    generated_collision_records = [
        item for item in _sdf_collision_semantic_records(generated_root)
        if 'caster_link_' not in item['name']]
    source_visual_records = _sdf_visual_semantic_records(source_root)
    generated_visual_records = _sdf_visual_semantic_records(
        generated_root, -caster_deltas['caster_link_front_collision'])
    generated_visuals = generated_root.findall('.//visual')
    generated_visual_flags = [
        visual.findall('visibility_flags') for visual in generated_visuals]
    lidar_visibility_masks = generated_root.findall(
        ".//sensor[@name='laser_sensor']/lidar/visibility_mask")
    source_links = {
        link.attrib['name']: link
        for link in source_root.findall('.//model/link')}
    generated_links = {
        link.attrib['name']: link
        for link in generated_root.findall('.//model/link')}
    source_total_mass_kg = sum(
        float(link.findtext('inertial/mass', default='0'))
        for link in source_links.values())
    generated_total_mass_kg = sum(
        float(link.findtext('inertial/mass', default='0'))
        for link in generated_links.values())
    source_mass_kg, source_com_m, source_tensor = _sdf_inertial(
        source_links['base_footprint'])
    generated_mass_kg, generated_com_m, generated_tensor = _sdf_inertial(
        generated_links['base_footprint'])
    caster_mass_kg = 0.03
    caster_delta_z_m = -0.048
    expected_com_z_m = (
        source_com_m[2]
        + 2.0 * caster_mass_kg * caster_delta_z_m / source_mass_kg)
    source_origin_tensor = _matrix_add(
        source_tensor, _parallel_axis(source_com_m), source_mass_kg)
    expected_origin_tensor = source_origin_tensor
    for x_m in (0.2, -0.2):
        expected_origin_tensor = _matrix_add(
            expected_origin_tensor,
            _parallel_axis([x_m, 0.0, 0.022]), caster_mass_kg)
        expected_origin_tensor = _matrix_add(
            expected_origin_tensor,
            _parallel_axis([x_m, 0.0, 0.070]), -caster_mass_kg)
    expected_tensor = _matrix_add(
        expected_origin_tensor, _parallel_axis(generated_com_m),
        -generated_mass_kg)
    tensor_valid = all(math.isclose(
        expected_tensor[row][column], generated_tensor[row][column],
        abs_tol=1e-12) for row in range(3) for column in range(3))
    if (len(source_collisions) != len(generated_collisions)
            or source_arm_count != 25 or generated_arm_count != 25
            or source_collision_records != generated_collision_records
            or source_visual_records != generated_visual_records
            or any(len(flags) != 1 or int(flags[0].text)
                   != ROBOT_VISIBILITY_FLAGS
                   for flags in generated_visual_flags)
            or len(lidar_visibility_masks) != 1
            or int(lidar_visibility_masks[0].text) != LIDAR_VISIBILITY_MASK
            or not math.isclose(
                source_total_mass_kg, generated_total_mass_kg, abs_tol=1e-12)
            or not math.isclose(
                source_mass_kg, generated_mass_kg, abs_tol=1e-12)
            or not math.isclose(
                expected_com_z_m, generated_com_m[2], abs_tol=1e-12)
            or not tensor_valid
            or any(not math.isclose(delta_m, -0.048, abs_tol=1e-12)
                   for delta_m in caster_deltas.values())):
        raise ValueError('converted SDF lost the caster/collision contract')
    return {
        'status': 'PASS',
        'source_sdf_sha256': hashlib.sha256(
            source_text.encode('utf-8')).hexdigest(),
        'generated_sdf_sha256': hashlib.sha256(
            generated_text.encode('utf-8')).hexdigest(),
        'source_collision_count': len(source_collisions),
        'generated_collision_count': len(generated_collisions),
        'source_arm_collision_count': source_arm_count,
        'generated_arm_collision_count': generated_arm_count,
        'caster_collision_center_delta_z_m': caster_deltas,
        'non_caster_collision_elements_equal': True,
        'converter_default_collision_surfaces_normalized': True,
        'visual_semantics_equal_except_visibility_flags': True,
        'visual_count': len(generated_visuals),
        'visual_visibility_flags': ROBOT_VISIBILITY_FLAGS,
        'lidar_visibility_mask': LIDAR_VISIBILITY_MASK,
        'source_total_mass_kg': source_total_mass_kg,
        'generated_total_mass_kg': generated_total_mass_kg,
        'fixed_lump_com_delta_z_m': generated_com_m[2] - source_com_m[2],
        'fixed_lump_expected_com_z_m': expected_com_z_m,
        'fixed_lump_parallel_axis_tensor_valid': tensor_valid,
    }


def _evaluation_urdf(source: Path) -> tuple[bytes, dict]:
    tree = ET.parse(source)
    root = tree.getroot()
    source_collisions = _collision_records(root)
    geometry = derive_level_caster_geometry(root)
    changed_joints = []
    for joint_name in ('caster_front_joint', 'caster_rear_joint'):
        origin = root.find(f"./joint[@name='{joint_name}']/origin")
        if origin is None:
            raise ValueError(f'missing caster joint: {joint_name}')
        xyz = origin.attrib['xyz'].split()
        before_z_m = float(xyz[2])
        xyz[2] = str(geometry['level_caster_joint_z_m'])
        origin.attrib['xyz'] = ' '.join(xyz)
        changed_joints.append({
            'name': joint_name,
            'before_z_m': before_z_m,
            'after_z_m': geometry['level_caster_joint_z_m'],
        })
    horizontal = root.find(
        "./gazebo[@reference='laser_link']/sensor[@name='laser_sensor']"
        '/lidar/scan/horizontal')
    if horizontal is None:
        raise ValueError('missing lidar horizontal scan configuration')
    sample_count = int(horizontal.findtext('samples'))
    angle_increment_rad = 2.0 * math.pi / sample_count
    min_angle = horizontal.find('min_angle')
    max_angle = horizontal.find('max_angle')
    if min_angle is None or max_angle is None:
        raise ValueError('missing lidar horizontal scan endpoints')
    source_min_angle_rad = float(min_angle.text)
    source_max_angle_rad = float(max_angle.text)
    min_angle.text = str(-math.pi)
    max_angle.text = str(-math.pi + (sample_count - 1) * angle_increment_rad)
    range_min = root.find(
        "./gazebo[@reference='laser_link']/sensor[@name='laser_sensor']"
        '/lidar/range/min')
    if range_min is None:
        raise ValueError('missing lidar minimum range')
    source_range_min_m = float(range_min.text)
    real_scan_reference = json.loads(
        REAL_SCAN_REFERENCE.read_text(encoding='utf-8'))
    range_min.text = str(real_scan_reference['range_min_m'])
    lidar = root.find(
        "./gazebo[@reference='laser_link']/sensor[@name='laser_sensor']"
        '/lidar')
    if lidar is None:
        raise ValueError('missing lidar configuration')
    visibility_mask = lidar.find('visibility_mask')
    if visibility_mask is None:
        visibility_mask = ET.Element('visibility_mask')
        lidar.insert(0, visibility_mask)
    visibility_mask.text = str(LIDAR_VISIBILITY_MASK)
    wheel_slip = ET.Element('gazebo')
    plugin = ET.SubElement(wheel_slip, 'plugin', {
        'filename': 'gz-sim-wheel-slip-system',
        'name': 'gz::sim::systems::WheelSlip',
    })
    for link_name in DRIVE_WHEEL_LINKS:
        wheel = ET.SubElement(plugin, 'wheel', {'link_name': link_name})
        ET.SubElement(wheel, 'wheel_radius').text = str(
            DRIVE_WHEEL_RADIUS_M)
        ET.SubElement(wheel, 'slip_compliance_lateral').text = '0.0'
        ET.SubElement(wheel, 'slip_compliance_longitudinal').text = '0.0'
        ET.SubElement(wheel, 'wheel_normal_force').text = str(
            DRIVE_WHEEL_NORMAL_FORCE_N)
    root.append(wheel_slip)
    visual_links = [
        link.attrib['name'] for link in root.findall('link')
        if link.findall('visual')]
    for link_name in visual_links:
        gazebo = ET.Element('gazebo', {'reference': link_name})
        visual = ET.SubElement(gazebo, 'visual')
        ET.SubElement(visual, 'visibility_flags').text = str(
            ROBOT_VISIBILITY_FLAGS)
        root.append(gazebo)
    generated_collisions = _collision_records(root)
    if generated_collisions != source_collisions:
        raise ValueError('evaluation URDF changed collision geometry')
    ET.indent(tree, space='  ')
    metadata = {
        'type': 'evaluation_support_plane_and_full_azimuth_correction',
        'scope': 'evaluation_only',
        'geometry': geometry,
        'changed_joints': changed_joints,
        'lidar_horizontal_angle_deltas': {
            'sample_count': sample_count,
            'angle_increment_rad': angle_increment_rad,
            'min_angle_before_rad': source_min_angle_rad,
            'min_angle_after_rad': float(min_angle.text),
            'max_angle_before_rad': source_max_angle_rad,
            'max_angle_after_rad': float(max_angle.text),
        },
        'lidar_range_min_delta': {
            'before_m': source_range_min_m,
            'after_m': float(range_min.text),
            'source': REAL_SCAN_REFERENCE.name,
        },
        'lidar_visibility': {
            'mask': LIDAR_VISIBILITY_MASK,
            'robot_visual_flags': ROBOT_VISIBILITY_FLAGS,
            'visual_link_count': len(visual_links),
            'visual_links': visual_links,
            'selection': 'all source URDF links containing visual elements',
        },
        'wheel_slip_runtime_fault': {
            'plugin': 'gz-sim-wheel-slip-system',
            'wheel_links': list(DRIVE_WHEEL_LINKS),
            'wheel_radius_m': DRIVE_WHEEL_RADIUS_M,
            'wheel_normal_force_n': DRIVE_WHEEL_NORMAL_FORCE_N,
            'nominal_slip_compliance_lateral': 0.0,
            'nominal_slip_compliance_longitudinal': 0.0,
        },
    }
    return (ET.tostring(
        root, encoding='utf-8', xml_declaration=True), metadata)


def generate_assets(output_dir: Path) -> dict:
    """Write deterministic assets and return a hash manifest."""
    output_dir.mkdir(parents=True, exist_ok=True)
    source_tree = ET.parse(SOURCE_WORLD)
    source_world = source_tree.getroot().find('world')
    if source_world is None:
        raise ValueError('source SDF has no world')

    world_path = output_dir / 'slam_corridor_contact.world'
    pgm_path = output_dir / 'slam_corridor_eval.pgm'
    yaml_path = output_dir / 'slam_corridor_eval.yaml'
    urdf_path = output_dir / 'jdamr_cube_nav_eval.urdf'
    world_path.write_bytes(_evaluation_world(SOURCE_WORLD))
    urdf_bytes, support_correction = _evaluation_urdf(SOURCE_URDF)
    urdf_path.write_bytes(urdf_bytes)
    converted_sdf = _validate_converted_sdf(SOURCE_URDF, urdf_path)
    pgm_path.write_bytes(_rasterize(_static_boxes(source_world)))
    yaml_path.write_text(yaml.safe_dump({
        'image': pgm_path.name,
        'mode': 'trinary',
        'resolution': RESOLUTION_M,
        'origin': [ORIGIN_X_M, ORIGIN_Y_M, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }, sort_keys=False), encoding='utf-8')

    content_hashes = {
        path.name: sha256_file(path)
        for path in (world_path, pgm_path, yaml_path, urdf_path)
    }
    source_robot_root = ET.parse(SOURCE_URDF).getroot()
    generated_robot_root = ET.parse(urdf_path).getroot()
    source_collisions = _collision_records(source_robot_root)
    generated_collisions = _collision_records(generated_robot_root)
    source_joints = _joint_records(source_robot_root)
    generated_joints = _joint_records(generated_robot_root)
    source_inertials = _inertial_records(source_robot_root)
    generated_inertials = _inertial_records(generated_robot_root)
    source_visuals = _visual_records(source_robot_root)
    generated_visuals = _visual_records(generated_robot_root)
    source_lidar = _lidar_sensor_record(source_robot_root)
    generated_lidar = _lidar_sensor_record(generated_robot_root)
    unchanged_lidar_keys = set(source_lidar) - {
        'min_angle_rad', 'max_angle_rad', 'range_min_m', 'visibility_mask'}
    allowed_joint_names = {'caster_front_joint', 'caster_rear_joint'}
    unchanged_source_joints = [
        item for item in source_joints
        if item['name'] not in allowed_joint_names]
    unchanged_generated_joints = [
        item for item in generated_joints
        if item['name'] not in allowed_joint_names]
    if (generated_collisions != source_collisions
            or generated_visuals != source_visuals
            or unchanged_generated_joints != unchanged_source_joints
            or generated_inertials != source_inertials
            or any(source_lidar[key] != generated_lidar[key]
                   for key in unchanged_lidar_keys)):
        raise ValueError('generated URDF changed physical model elements')
    lidar_noise_stddev_m = float(source_robot_root.findtext(
        "./gazebo[@reference='laser_link']/sensor[@name='laser_sensor']"
        '/lidar/noise/stddev'))
    laser_joint = source_robot_root.find("./joint[@name='laser_joint']/origin")
    if laser_joint is None:
        raise ValueError('missing laser joint origin')
    laser_yaw_in_base_rad = float(laser_joint.attrib['rpy'].split()[2])
    real_scan_reference = json.loads(
        REAL_SCAN_REFERENCE.read_text(encoding='utf-8'))
    preloaded_obstacles = derive_preloaded_obstacles(
        real_scan_reference['range_min_m'])
    generated_world = ET.parse(world_path).getroot().find('world')
    if generated_world is None:
        raise ValueError('generated evaluation world has no world')
    generated_preloaded_names = {
        model.attrib['name'] for model in generated_world.findall('model')
        if model.attrib.get('name', '').startswith('g003_preloaded_')}
    expected_preloaded_names = {
        spec['name'] for spec in preloaded_obstacles.values()}
    park_aabbs = []
    for spec in preloaded_obstacles.values():
        model = generated_world.find(f"model[@name='{spec['name']}']")
        if model is None:
            raise ValueError(f"missing preloaded model: {spec['name']}")
        actual_size = [float(value) for value in model.findtext(
            'link/collision/geometry/box/size').split()]
        visual_flags = int(model.findtext('link/visual/visibility_flags'))
        contact_ref = model.findtext('link/sensor/contact/collision')
        actual_pose = [float(value) for value in model.findtext(
            'pose').split()[:3]]
        if (model.findtext('static') != 'true'
                or actual_size != [spec['length_m'], spec['width_m'],
                                   spec['height_m']]
                or actual_pose != spec['park_pose_m']
                or visual_flags != LIDAR_VISIBILITY_MASK
                or contact_ref != spec['contact_collision_ref']):
            raise ValueError(f"invalid preloaded model: {spec['name']}")
        park_aabbs.append((
            spec['name'], actual_pose[0], actual_pose[1],
            spec['length_m'], spec['width_m']))
    if generated_preloaded_names != expected_preloaded_names:
        raise ValueError('preloaded obstacle inventory mismatch')
    park_clearances_m = []
    for index, first in enumerate(park_aabbs):
        for second in park_aabbs[index + 1:]:
            x_clearance_m = abs(first[1] - second[1]) - (
                first[3] + second[3]) / 2.0
            y_clearance_m = abs(first[2] - second[2]) - (
                first[4] + second[4]) / 2.0
            clearance_m = max(x_clearance_m, y_clearance_m)
            if clearance_m <= 0.0:
                raise ValueError('preloaded park models overlap')
            park_clearances_m.append(clearance_m)
    manifest = {
        'schema_version': 1,
        'source_world': {
            'path': str(SOURCE_WORLD.resolve()),
            'sha256': sha256_file(SOURCE_WORLD),
        },
        'preloaded_obstacles': {
            'count': len(preloaded_obstacles),
            'models': preloaded_obstacles,
            'minimum_pairwise_aabb_clearance_m': min(park_clearances_m),
            'runtime_create_delete_forbidden': True,
        },
        'source_robot': {
            'path': str(SOURCE_URDF.resolve()),
            'sha256': sha256_file(SOURCE_URDF),
            'collision_elements': source_collisions,
            'removed_collision_elements': [],
            'joint_elements': source_joints,
            'inertial_elements': source_inertials,
            'visual_elements': source_visuals,
            'allowed_joint_deltas': support_correction['changed_joints'],
            'allowed_sensor_deltas': {
                'laser_horizontal_min_angle_rad':
                    support_correction['lidar_horizontal_angle_deltas'][
                        'min_angle_after_rad'],
                'laser_horizontal_max_angle_rad':
                    support_correction['lidar_horizontal_angle_deltas'][
                        'max_angle_after_rad'],
                'laser_range_min_m':
                    support_correction['lidar_range_min_delta']['after_m'],
                'laser_visibility_mask': LIDAR_VISIBILITY_MASK,
            },
            'source_lidar_sensor': source_lidar,
            'generated_lidar_sensor': generated_lidar,
            'unchanged_lidar_sensor_fields': sorted(unchanged_lidar_keys),
            'canonical_equivalence': {
                'collision_geometry_and_origin': True,
                'visuals': True,
                'inertials': True,
                'unchanged_joints': True,
                'all_joints': False,
            },
            'required_collision_counts': {
                'arm': sum(1 for record in source_collisions
                           if record['link'].startswith('arm_')),
                'base_wheel_caster': sum(
                    1 for record in source_collisions
                    if record['link'] in {
                        'base_link', 'left_wheel_link', 'right_wheel_link',
                        'caster_link_front', 'caster_link_rear'}),
            },
        },
        'simulation_support_plane_correction': {
            'support_plane_correction': support_correction,
            'converted_sdf_validation': converted_sdf,
            'scan_preflight_contract': {
                'expected_wall_beams_m': _expected_wall_beams(source_world),
                'lidar_noise_stddev_m': lidar_noise_stddev_m,
                'laser_yaw_in_base_rad': laser_yaw_in_base_rad,
                'simulation_update_rate_hz': generated_lidar[
                    'update_rate_hz'],
                'wall_beam_multiple_comparison':
                    derive_wall_beam_multiple_comparison_contract(
                        lidar_noise_stddev_m,
                        len(_expected_wall_beams(source_world)), SEEDS),
                'full_azimuth': {
                    'sample_count':
                        support_correction['lidar_horizontal_angle_deltas'][
                            'sample_count'],
                    'angle_increment_rad':
                        support_correction['lidar_horizontal_angle_deltas'][
                            'angle_increment_rad'],
                    'seam_duplicate_allowed': False,
                },
                'real_scan_reference': real_scan_reference,
                'real_scan_reference_sha256': sha256_file(
                    REAL_SCAN_REFERENCE),
                'observed_full_fov_near_return_envelope_m': {
                    'minimum': 0.07522211223840714,
                    'maximum': 0.164383664727211,
                    'sample_count': 25,
                },
                'front_observation_probe': derive_front_observation_probe(),
                'sub_minimum_range_probe': derive_sub_minimum_range_probe(
                    real_scan_reference['range_min_m']),
            },
            'contact_scope': {
                'measured_geometry': (
                    'full_evaluation_robot_collision_set_against_'
                    'preloaded_obstacle'),
                'collision_geometry_preserved': True,
                'excluded_collision_elements': [],
                'simulation_only': True,
                'evaluation_geometry_delta': (
                    'caster_support_plane_full_azimuth_and_lidar_visibility'),
                'arm_probe': {
                    'park_pose_source': (
                        'preloaded_obstacles.contact_control'),
                    'overlap_pose_world_m': [-7.80, 0.0, 0.30],
                    'size_source': 'preloaded_obstacles.contact_control',
                    'robot_relative_x_extent_m': [0.13, 0.27],
                    'classification_source': (
                        'evaluation_urdf_arm_link_inventory'),
                    'unexpected_robot_collision_policy': 'fail_closed',
                    'excluded_collision_suffix': '__base_link_collision',
                    'sequence': [
                        'scan_data_ready', 'preloaded_entity_ready',
                        'contact_topic_ready', 'set_overlap_pose',
                        'nonempty_arm_contact', 'deactivate_ack',
                        'strictly_newer_scan', 'contact_stream_quiet'],
                },
                'claim_exclusions': [
                    'moving_people_or_objects',
                    'human_safety',
                    'physical_robot_collision_safety',
                ],
            },
            'sensor_model_claim': real_scan_reference['claim'],
            'sensor_model_claim_exclusions': [
                'objects_inside_0.28m',
                'emergency_stop_behavior',
                'moving_people_or_objects',
                'real_sensor_10hz_rate_equivalence',
            ],
            'stale_geometry_assumptions': [{
                'assumption': 'lidar_x_m_equals_0.3',
                'actual_laser_joint_x_m': 0.0,
                'used_for_evaluation': False,
            }],
        },
        'map': {
            'resolution_m': RESOLUTION_M,
            'origin': [ORIGIN_X_M, ORIGIN_Y_M, 0.0],
            'width_cells': WIDTH_CELLS,
            'height_cells': HEIGHT_CELLS,
        },
        'content_hashes': content_hashes,
    }
    (output_dir / 'asset_manifest.json').write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    return manifest


def main() -> int:
    """Generate assets at the requested path."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(generate_assets(args.output), indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
