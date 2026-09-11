#!/usr/bin/env python3
"""Build a Gazebo 2.5D world from the real navigation occupancy map."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np  # noqa: I201
import yaml  # noqa: I201


CAMERA_TOPIC = '/actual_map_scene/image_raw'
CAMERA_RATE_HZ = 15.0
SCENE_OBJECTS = (
    {
        'name': 'recorded_box_outbound_1', 'kind': 'cardboard_box',
        'pose': [20.00, -2.10, 0.275], 'size': [0.65, 0.45, 0.55],
        'source': 'IMG_7861.mov_box_avoidance',
        'position_basis': 'route_phase_adjusted_between_waypoints_5_and_6',
    },
    {
        'name': 'recorded_box_outbound_2', 'kind': 'cardboard_box',
        'pose': [32.00, -3.10, 0.275], 'size': [0.65, 0.45, 0.55],
        'source': 'IMG_7862.mov_box_avoidance',
        'position_basis': 'route_phase_adjusted_between_waypoints_8_and_9',
    },
    {
        'name': 'recorded_box_person_scene', 'kind': 'cardboard_box',
        'pose': [10.30, -0.10, 0.275], 'size': [0.65, 0.45, 0.55],
        'source': 'IMG_7862.mov_box_with_person_stop',
        'initially_parked': True,
    },
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _rectangles(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    """Cover occupied pixels exactly with vertically merged horizontal runs."""
    if mask.ndim != 2:
        raise ValueError('occupancy mask must be two-dimensional')
    active: dict[tuple[int, int], tuple[int, int, int, int]] = {}
    rectangles = []
    for row_index, row in enumerate(mask):
        columns = np.flatnonzero(row)
        runs = []
        if len(columns):
            breaks = np.flatnonzero(np.diff(columns) > 1)
            starts = np.r_[columns[0], columns[breaks + 1]]
            ends = np.r_[columns[breaks], columns[-1]]
            runs = list(zip(starts.tolist(), ends.tolist()))
        current = {}
        for run in runs:
            if run in active:
                x_min, x_max, y_min, _ = active[run]
                current[run] = (x_min, x_max, y_min, row_index)
            else:
                current[run] = (run[0], run[1], row_index, row_index)
        rectangles.extend(value for key, value in active.items()
                          if key not in current)
        active = current
    rectangles.extend(active.values())
    return rectangles


def _write_obj(path: Path, rectangles: list[tuple[int, int, int, int]],
               *, width: int, height: int, resolution_m: float,
               origin_x_m: float, origin_y_m: float,
               wall_height_m: float) -> None:
    """Write all exact occupied rectangles as one efficient prism mesh."""
    if width <= 0 or height <= 0:
        raise ValueError('occupancy dimensions must be positive')
    normals = (
        (0, 0, -1), (0, 0, 1), (0, -1, 0),
        (1, 0, 0), (0, 1, 0), (-1, 0, 0),
    )
    lines = ['# occupancy-grid extrusion']
    lines.extend(f'vn {x} {y} {z}' for x, y, z in normals)
    vertex_index = 1
    faces = (
        ((1, 3, 2), 1), ((1, 4, 3), 1),
        ((5, 6, 7), 2), ((5, 7, 8), 2),
        ((1, 2, 6), 3), ((1, 6, 5), 3),
        ((2, 3, 7), 4), ((2, 7, 6), 4),
        ((3, 4, 8), 5), ((3, 8, 7), 5),
        ((4, 1, 5), 6), ((4, 5, 8), 6),
    )
    for x_min, x_max, row_min, row_max in rectangles:
        x0 = origin_x_m + x_min * resolution_m
        x1 = origin_x_m + (x_max + 1) * resolution_m
        y0 = origin_y_m + (height - row_max - 1) * resolution_m
        y1 = origin_y_m + (height - row_min) * resolution_m
        vertices = (
            (x0, y0, 0.0), (x1, y0, 0.0),
            (x1, y1, 0.0), (x0, y1, 0.0),
            (x0, y0, wall_height_m), (x1, y0, wall_height_m),
            (x1, y1, wall_height_m), (x0, y1, wall_height_m),
        )
        lines.extend(f'v {x:.6f} {y:.6f} {z:.6f}'
                     for x, y, z in vertices)
        lines.extend(
            'f ' + ' '.join(
                f'{vertex_index + item - 1}//{normal_index}'
                for item in face)
            for face, normal_index in faces)
        vertex_index += 8
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _write_floor_mask_obj(
        path: Path, rectangles: list[tuple[int, int, int, int]], *,
        height: int, resolution_m: float, origin_x_m: float,
        origin_y_m: float) -> None:
    """Write an exact flat mesh for the effective keepout-mask cells."""
    lines = ['# exact keepout-mask overlay', 'vn 0 0 1']
    vertex_index = 1
    for x_min, x_max, row_min, row_max in rectangles:
        x0 = origin_x_m + x_min * resolution_m
        x1 = origin_x_m + (x_max + 1) * resolution_m
        y0 = origin_y_m + (height - row_max - 1) * resolution_m
        y1 = origin_y_m + (height - row_min) * resolution_m
        lines.extend((
            f'v {x0:.6f} {y0:.6f} 0.004',
            f'v {x1:.6f} {y0:.6f} 0.004',
            f'v {x1:.6f} {y1:.6f} 0.004',
            f'v {x0:.6f} {y1:.6f} 0.004',
            f'f {vertex_index}//1 {vertex_index + 1}//1 '
            f'{vertex_index + 2}//1',
            f'f {vertex_index}//1 {vertex_index + 2}//1 '
            f'{vertex_index + 3}//1',
        ))
        vertex_index += 4
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _remove_so101(root: ET.Element) -> list[str]:
    """Remove the arm subtree while retaining the independent RGB-D mast."""
    removed = []
    for child in list(root):
        name = child.get('name', '')
        reference = child.get('reference', '')
        arm_control = child.tag == 'ros2_control' and name == 'jdamr_cube_arm'
        arm_plugin = (
            child.tag == 'gazebo'
            and child.find("./plugin[@filename='gz_ros2_control-system']")
            is not None)
        if (name.startswith('arm_') or reference.startswith('arm_')
                or name.startswith('so101_') or arm_control or arm_plugin):
            removed.append(f'{child.tag}:{name or reference}')
            root.remove(child)
    for material in root.findall(".//material[@name='so101_sts3215']"):
        material.set('name', 'black')
        removed.append('material_reference:so101_sts3215')
    return removed


def _add_cardboard_box(world: ET.Element, spec: dict) -> None:
    model = ET.SubElement(world, 'model', {'name': spec['name']})
    ET.SubElement(model, 'static').text = 'true'
    pose = list(spec['pose'])
    if spec.get('initially_parked'):
        pose[1] = 50.0
    ET.SubElement(model, 'pose').text = (
        ' '.join(map(str, pose)) + ' 0 0 0')
    link = ET.SubElement(model, 'link', {'name': 'body'})
    for tag in ('collision', 'visual'):
        element = ET.SubElement(link, tag, {'name': tag})
        box = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
        ET.SubElement(box, 'size').text = ' '.join(map(str, spec['size']))
        if tag == 'visual':
            ET.SubElement(element, 'visibility_flags').text = '5'
            material = ET.SubElement(element, 'material')
            ET.SubElement(material, 'diffuse').text = '0.62 0.36 0.14 1'
            ET.SubElement(material, 'ambient').text = '0.44 0.24 0.08 1'


def _add_person(world: ET.Element, park_x_m: float) -> None:
    model = ET.SubElement(world, 'model', {'name': 'crossing_person'})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{park_x_m} 50 0 0 0 0'
    link = ET.SubElement(model, 'link', {'name': 'body'})
    collision = ET.SubElement(link, 'collision', {'name': 'collision'})
    ET.SubElement(collision, 'pose').text = '0 0 0.85 0 0 0'
    cylinder = ET.SubElement(ET.SubElement(collision, 'geometry'), 'cylinder')
    ET.SubElement(cylinder, 'radius').text = '0.24'
    ET.SubElement(cylinder, 'length').text = '1.70'
    shapes = (
        ('torso', 'cylinder', '0 0 1.08 0 0 0', ('0.22', '0.72'),
         '0.12 0.34 0.78 1'),
        ('head', 'sphere', '0 0 1.62 0 0 0', ('0.18',),
         '0.92 0.70 0.52 1'),
        ('left_leg', 'cylinder', '0 0.10 0.42 0 0 0', ('0.08', '0.72'),
         '0.10 0.12 0.18 1'),
        ('right_leg', 'cylinder', '0 -0.10 0.42 0 0 0', ('0.08', '0.72'),
         '0.10 0.12 0.18 1'),
    )
    for name, geometry_kind, pose, dimensions, color in shapes:
        visual = ET.SubElement(link, 'visual', {'name': name})
        ET.SubElement(visual, 'pose').text = pose
        ET.SubElement(visual, 'visibility_flags').text = '5'
        geometry = ET.SubElement(visual, 'geometry')
        shape = ET.SubElement(geometry, geometry_kind)
        if geometry_kind == 'sphere':
            ET.SubElement(shape, 'radius').text = dimensions[0]
        else:
            ET.SubElement(shape, 'radius').text = dimensions[0]
            ET.SubElement(shape, 'length').text = dimensions[1]
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'diffuse').text = color


def _world(world_path: Path, mesh_path: Path, keepout_mesh_path: Path,
           metadata: dict,
           image_shape: tuple[int, int], wall_height_m: float,
           traction_zone: dict) -> None:
    """Create the physical world, parked obstacle, and fixed camera."""
    height, width = image_shape
    resolution_m = float(metadata['resolution'])
    origin_x_m, origin_y_m = map(float, metadata['origin'][:2])
    width_m = width * resolution_m
    height_m = height * resolution_m
    center_x_m = origin_x_m + width_m / 2.0
    center_y_m = origin_y_m + height_m / 2.0
    sdf = ET.Element('sdf', {'version': '1.8'})
    world = ET.SubElement(sdf, 'world', {'name': 'slam_corridor'})
    physics = ET.SubElement(world, 'physics', {
        'name': '1ms', 'type': 'ignored'})
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_factor').text = '1.0'
    for filename, name in (
            ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
            ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
            ('gz-sim-scene-broadcaster-system',
             'gz::sim::systems::SceneBroadcaster'),
            ('gz-sim-imu-system', 'gz::sim::systems::Imu')):
        ET.SubElement(world, 'plugin', {'filename': filename, 'name': name})
    sensors = ET.SubElement(world, 'plugin', {
        'filename': 'gz-sim-sensors-system',
        'name': 'gz::sim::systems::Sensors'})
    ET.SubElement(sensors, 'render_engine').text = 'ogre2'
    light = ET.SubElement(world, 'light', {
        'type': 'directional', 'name': 'sun'})
    ET.SubElement(light, 'pose').text = f'{center_x_m} {center_y_m} 35 0 0 0'
    ET.SubElement(light, 'diffuse').text = '0.85 0.85 0.85 1'
    ET.SubElement(light, 'specular').text = '0.2 0.2 0.2 1'
    ET.SubElement(light, 'direction').text = '-0.4 0.2 -0.9'

    floor_x_min, floor_x_max = origin_x_m, origin_x_m + width_m
    floor_y_min, floor_y_max = origin_y_m, origin_y_m + height_m
    zone_x_min = traction_zone['center_x_m'] - traction_zone['length_m'] / 2
    zone_x_max = traction_zone['center_x_m'] + traction_zone['length_m'] / 2
    zone_y_min = traction_zone['center_y_m'] - traction_zone['width_m'] / 2
    zone_y_max = traction_zone['center_y_m'] + traction_zone['width_m'] / 2
    if not (floor_x_min < zone_x_min < zone_x_max < floor_x_max
            and floor_y_min < zone_y_min < zone_y_max < floor_y_max):
        raise ValueError('actual-map traction zone must be inside the floor')

    def floor_box(name: str, x_min: float, x_max: float,
                  y_min: float, y_max: float, *, traction: bool = False
                  ) -> None:
        model = ET.SubElement(world, 'model', {'name': name})
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = (
            f'{(x_min + x_max) / 2} {(y_min + y_max) / 2} '
            '-0.025 0 0 0')
        link = ET.SubElement(model, 'link', {'name': 'surface'})
        collision = ET.SubElement(
            link, 'collision', {'name': 'collision'})
        box = ET.SubElement(ET.SubElement(collision, 'geometry'), 'box')
        ET.SubElement(box, 'size').text = (
            f'{x_max - x_min} {y_max - y_min} 0.05')
        if traction:
            surface = ET.SubElement(collision, 'surface')
            friction = ET.SubElement(surface, 'friction')
            ode = ET.SubElement(friction, 'ode')
            ET.SubElement(ode, 'mu').text = str(traction_zone['mu'])
            ET.SubElement(ode, 'mu2').text = str(traction_zone['mu2'])
        visual = ET.SubElement(link, 'visual', {'name': 'visual'})
        visual_box = ET.SubElement(
            ET.SubElement(visual, 'geometry'), 'box')
        ET.SubElement(visual_box, 'size').text = (
            f'{x_max - x_min} {y_max - y_min} 0.05')
        material = ET.SubElement(visual, 'material')
        color = '0.95 0.55 0.08 1' if traction else '0.72 0.72 0.68 1'
        ET.SubElement(material, 'diffuse').text = color

    floor_box('ground_plane', floor_x_min, zone_x_min,
              floor_y_min, floor_y_max)
    floor_box('restaurant_floor_right', zone_x_max, floor_x_max,
              floor_y_min, floor_y_max)
    floor_box('restaurant_floor_south', zone_x_min, zone_x_max,
              floor_y_min, zone_y_min)
    floor_box('restaurant_floor_north', zone_x_min, zone_x_max,
              zone_y_max, floor_y_max)
    floor_box('restaurant_low_traction_zone', zone_x_min, zone_x_max,
              zone_y_min, zone_y_max, traction=True)

    walls = ET.SubElement(world, 'model', {'name': 'actual_map_walls'})
    ET.SubElement(walls, 'static').text = 'true'
    wall_link = ET.SubElement(walls, 'link', {'name': 'walls'})
    for tag in ('collision', 'visual'):
        element = ET.SubElement(wall_link, tag, {'name': tag})
        mesh = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
        ET.SubElement(mesh, 'uri').text = mesh_path.resolve().as_uri()
        if tag == 'visual':
            ET.SubElement(element, 'visibility_flags').text = '4'
            material = ET.SubElement(element, 'material')
            ET.SubElement(material, 'diffuse').text = '0.34 0.39 0.45 1'

    keepout = ET.SubElement(world, 'model', {'name': 'keepout_mask_overlay'})
    ET.SubElement(keepout, 'static').text = 'true'
    keepout_link = ET.SubElement(keepout, 'link', {'name': 'mask'})
    keepout_visual = ET.SubElement(
        keepout_link, 'visual', {'name': 'orange_keepout'})
    mask_mesh = ET.SubElement(
        ET.SubElement(keepout_visual, 'geometry'), 'mesh')
    ET.SubElement(mask_mesh, 'uri').text = keepout_mesh_path.resolve().as_uri()
    ET.SubElement(keepout_visual, 'visibility_flags').text = '1'
    mask_material = ET.SubElement(keepout_visual, 'material')
    ET.SubElement(mask_material, 'diffuse').text = '1.0 0.22 0.02 0.82'
    ET.SubElement(mask_material, 'ambient').text = '0.72 0.10 0.01 0.82'

    for spec in SCENE_OBJECTS:
        _add_cardboard_box(world, spec)
    _add_person(world, center_x_m)

    camera_model = ET.SubElement(
        world, 'model', {'name': 'actual_map_scene_camera'})
    ET.SubElement(camera_model, 'static').text = 'true'
    ET.SubElement(camera_model, 'pose').text = (
        f'{center_x_m} {center_y_m - 20.0} 18 0 0 0')
    camera_link = ET.SubElement(camera_model, 'link', {'name': 'mount'})
    camera = ET.SubElement(
        camera_link, 'sensor', {'name': 'actual_map_camera', 'type': 'camera'})
    ET.SubElement(camera, 'pose').text = (
        '0 0 0 0 0.733 1.57079632679')
    ET.SubElement(camera, 'always_on').text = 'true'
    ET.SubElement(camera, 'update_rate').text = str(CAMERA_RATE_HZ)
    ET.SubElement(camera, 'topic').text = CAMERA_TOPIC
    camera_spec = ET.SubElement(camera, 'camera')
    ET.SubElement(camera_spec, 'horizontal_fov').text = '1.45'
    image = ET.SubElement(camera_spec, 'image')
    ET.SubElement(image, 'width').text = '1280'
    ET.SubElement(image, 'height').text = '720'
    ET.SubElement(image, 'format').text = 'R8G8B8'
    clip = ET.SubElement(camera_spec, 'clip')
    ET.SubElement(clip, 'near').text = '0.1'
    ET.SubElement(clip, 'far').text = '80'
    tree = ET.ElementTree(sdf)
    ET.indent(tree, space='  ')
    tree.write(world_path, encoding='utf-8', xml_declaration=True)


def prepare(args: argparse.Namespace) -> dict:
    """Copy immutable inputs and produce an actual-map replay contract."""
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=False)
    base = json.loads(args.base_contract.read_text(encoding='utf-8'))
    route = yaml.safe_load(args.route.read_text(encoding='utf-8'))
    if len(route.get('waypoints', [])) != 20:
        raise ValueError('actual-map replay requires 20 route waypoints')
    map_metadata = yaml.safe_load(args.map_yaml.read_text(encoding='utf-8'))
    map_image_source = (args.map_yaml.parent / map_metadata['image']).resolve()
    keepout_metadata = yaml.safe_load(
        args.keepout_yaml.read_text(encoding='utf-8'))
    keepout_image_source = (
        args.keepout_yaml.parent / keepout_metadata['image']).resolve()
    robot_root = ET.parse(args.robot_urdf).getroot()
    lidar_minimums = robot_root.findall(
        ".//sensor[@type='gpu_lidar']/lidar/range/min")
    lidar_rates = robot_root.findall(
        ".//sensor[@type='gpu_lidar']/update_rate")
    if (len(lidar_minimums) != 1 or lidar_minimums[0].text is None
            or len(lidar_rates) != 1 or lidar_rates[0].text is None):
        raise ValueError(
            'robot URDF must contain one GPU lidar range and update rate')
    lidar_min_range_m = float(lidar_minimums[0].text)
    lidar_update_rate_hz = float(lidar_rates[0].text)
    image = cv2.imread(str(map_image_source), cv2.IMREAD_GRAYSCALE)
    keepout_image = cv2.imread(
        str(keepout_image_source), cv2.IMREAD_GRAYSCALE)
    if image is None or keepout_image is None:
        raise ValueError('occupancy or keepout image is unreadable')
    if image.shape != keepout_image.shape:
        raise ValueError('keepout mask dimensions differ from the map')
    if (float(keepout_metadata['resolution'])
            != float(map_metadata['resolution'])
            or list(keepout_metadata['origin'])
            != list(map_metadata['origin'])):
        raise ValueError('keepout mask metadata differs from the map')
    occupied_limit = round(
        (1.0 - float(map_metadata['occupied_thresh'])) * 255)
    occupied = image < occupied_limit
    rectangles = _rectangles(occupied)
    if sum((x1 - x0 + 1) * (y1 - y0 + 1)
           for x0, x1, y0, y1 in rectangles) != int(occupied.sum()):
        raise ValueError('rectangle cover changed occupied-cell count')
    wall_obj = output / 'actual_map_walls.obj'
    _write_obj(
        wall_obj, rectangles, width=image.shape[1], height=image.shape[0],
        resolution_m=float(map_metadata['resolution']),
        origin_x_m=float(map_metadata['origin'][0]),
        origin_y_m=float(map_metadata['origin'][1]),
        wall_height_m=args.wall_height_m)
    keepout_limit = round(
        (1.0 - float(keepout_metadata['occupied_thresh'])) * 255)
    keepout_cells = keepout_image < keepout_limit
    keepout_rectangles = _rectangles(keepout_cells)
    keepout_obj = output / 'actual_keepout_mask.obj'
    _write_floor_mask_obj(
        keepout_obj, keepout_rectangles, height=image.shape[0],
        resolution_m=float(map_metadata['resolution']),
        origin_x_m=float(map_metadata['origin'][0]),
        origin_y_m=float(map_metadata['origin'][1]))
    world = output / 'actual_map_restaurant.world'
    previous = route['waypoints'][3]
    target = route['waypoints'][4]
    inherited_zone = base['traction_fault']['zone']
    traction_zone = {
        'center_x_m': (float(previous['x']) + float(target['x'])) / 2,
        'center_y_m': (float(previous['y']) + float(target['y'])) / 2,
        'length_m': max(2.0, float(inherited_zone['length_m'])),
        'width_m': max(1.2, float(inherited_zone['width_m'])),
        'mu': float(inherited_zone['mu']),
        'mu2': float(inherited_zone['mu2']),
        'provenance': 'synthetic_stress_not_real_measurement',
    }
    _world(
        world, wall_obj, keepout_obj, map_metadata, image.shape,
        args.wall_height_m, traction_zone)
    map_pgm = output / map_image_source.name
    map_yaml = output / args.map_yaml.name
    keepout_pgm = output / keepout_image_source.name
    keepout_yaml = output / args.keepout_yaml.name
    robot_urdf = output / args.robot_urdf.name
    bridge = output / 'bridge_guarded.yaml'
    shutil.copy2(map_image_source, map_pgm)
    local_metadata = {**map_metadata, 'image': map_pgm.name}
    map_yaml.write_text(
        yaml.safe_dump(local_metadata, sort_keys=False), encoding='utf-8')
    shutil.copy2(keepout_image_source, keepout_pgm)
    local_keepout_metadata = {
        **keepout_metadata, 'image': keepout_pgm.name}
    keepout_yaml.write_text(
        yaml.safe_dump(local_keepout_metadata, sort_keys=False),
        encoding='utf-8')
    robot_tree = ET.parse(args.robot_urdf)
    removed_so101_elements = _remove_so101(robot_tree.getroot())
    if not removed_so101_elements:
        raise ValueError('source robot URDF did not contain an SO-101 subtree')
    ET.indent(robot_tree, space='  ')
    robot_tree.write(robot_urdf, encoding='utf-8', xml_declaration=True)
    base_bridge = Path(base['traction_fault']['guarded_bridge']['path'])
    shutil.copy2(base_bridge, bridge)
    base['scenario_id'] = 'restaurant_actual_map_2_5d_replay'
    base['route']['source_waypoint_count'] = len(route['waypoints'])
    base['route']['simulation_waypoint_count'] = len(route['waypoints'])
    base['route']['longitudinal_scale'] = 1.0
    base['route']['waypoints'] = [
        {'id': item['id'], 'x_m': float(item['x']), 'y_m': float(item['y'])}
        for item in route['waypoints']]
    base['route']['transform'] = {
        'kind': 'identity_actual_map_coordinates', 'scale': 1.0}
    base['route']['start_pose'] = route['start_pose']
    base['navigation_map'] = {
        'yaml': {'path': str(map_yaml.resolve()), 'sha256': _sha256(map_yaml)},
        'pgm': {'path': str(map_pgm.resolve()), 'sha256': _sha256(map_pgm)},
        'source_yaml_sha256': _sha256(args.map_yaml),
        'source_pgm_sha256': _sha256(map_image_source),
        'resolution_m': float(map_metadata['resolution']),
        'origin': map_metadata['origin'],
        'width_cells': int(image.shape[1]),
        'height_cells': int(image.shape[0]),
        'keepout_yaml': {
            'path': str(keepout_yaml.resolve()),
            'sha256': _sha256(keepout_yaml)},
        'keepout_pgm': {
            'path': str(keepout_pgm.resolve()),
            'sha256': _sha256(keepout_pgm)},
        'source_keepout_pgm_sha256': _sha256(keepout_image_source),
        'keepout_cell_count': int(keepout_cells.sum()),
    }
    base['traction_fault']['world'] = {
        'path': str(world.resolve()), 'sha256': _sha256(world)}
    base['traction_fault']['robot_urdf'] = {
        'path': str(robot_urdf.resolve()), 'sha256': _sha256(robot_urdf)}
    base['traction_fault']['guarded_bridge'] = {
        'path': str(bridge.resolve()), 'sha256': _sha256(bridge)}
    base['traction_fault']['zone'] = traction_zone
    base['traction_fault']['guard_overrides'] = {
        'recovery_grace_s': 15.0,
        'anomaly_dwell_s': 0.20,
        'minimum_odom_progress_m': 0.05,
        'reason': 'low_speed_exit_from_actual_route_traction_zone',
    }
    obstacle_injection = {
        'obstacle_ahead_m': 0.75,
        'obstacle_length_m': 0.5,
        'nominal_nearest_face_from_base_m': 0.50,
        'lidar_min_range_m': lidar_min_range_m,
        'stop_zone_front_m': 0.65,
        'sensor_update_rate_hz': lidar_update_rate_hz,
        'contact_intent': 'noncontact_emergency_stop',
        'dynamic_entity': 'crossing_person',
    }
    if not (obstacle_injection['lidar_min_range_m']
            < obstacle_injection['nominal_nearest_face_from_base_m']
            < obstacle_injection['stop_zone_front_m']):
        raise ValueError(
            'obstacle must be visible inside the noncontact stop zone')
    base['obstacle_interventions']['injection'] = obstacle_injection
    base['obstacle_interventions']['simulation_required_count'] = 3
    base['obstacle_interventions']['scenes'] = [
        {
            'scene_id': 'box_avoidance_1', 'kind': 'static_avoidance',
            'trigger_waypoint_index': 6,
            'object_entity': 'recorded_box_outbound_1',
            'object_pose_m': SCENE_OBJECTS[0]['pose'][:2],
            'source_event_ids': [1, 2],
            'source_media': 'IMG_7861.mov',
            'required_transition': [
                'APPROACH', 'LOCAL_PLAN_STEERING', 'SAME_GOAL_CONTINUE'],
        },
        {
            'scene_id': 'box_avoidance_2', 'kind': 'static_avoidance',
            'trigger_waypoint_index': 9,
            'object_entity': 'recorded_box_outbound_2',
            'object_pose_m': SCENE_OBJECTS[1]['pose'][:2],
            'source_event_ids': [3, 4],
            'source_media': 'IMG_7862.mov',
            'required_transition': [
                'APPROACH', 'LOCAL_PLAN_STEERING', 'SAME_GOAL_CONTINUE'],
        },
        {
            'scene_id': 'person_crossing_stop',
            'kind': 'person_crossing_emergency_stop',
            'trigger_waypoint_index': 18,
            'object_entity': 'crossing_person',
            'supporting_box_entity': 'recorded_box_person_scene',
            'supporting_box_pose_m': SCENE_OBJECTS[2]['pose'],
            'source_event_ids': [5, 6],
            'source_media': 'IMG_7862.mov',
            'required_transition': [
                'PERSON_CROSSING', 'PROTECTIVE_STOP',
                'PERSON_CLEAR', 'SAME_GOAL_RESUME'],
        },
    ]
    base['simulator_camera'] = {
        'fps': CAMERA_RATE_HZ, 'source': 'gazebo_camera_sensor',
        'views': [
            {'name': 'actual_map_wide', 'topic': CAMERA_TOPIC,
             'width': 1280, 'height': 720},
            {'name': 'robot_front', 'topic': '/rgbd_camera/image',
             'width': 640, 'height': 480},
        ],
    }
    base['world_reconstruction'] = {
        'kind': 'exact_occupied_cell_2_5d_extrusion',
        'occupied_cell_count': int(occupied.sum()),
        'rectangle_count': len(rectangles),
        'wall_height_m': args.wall_height_m,
        'wall_height_provenance': 'explicit_visualization_assumption',
        'wall_mesh': {'path': str(wall_obj.resolve()),
                      'sha256': _sha256(wall_obj)},
        'keepout_mask_mesh': {
            'path': str(keepout_obj.resolve()),
            'sha256': _sha256(keepout_obj),
            'cell_count': int(keepout_cells.sum()),
            'color': 'orange',
        },
        'scene_objects': list(SCENE_OBJECTS),
        'dynamic_person': {
            'entity': 'crossing_person',
            'source': 'IMG_7862.mov_user_identified_person_stop',
            'representation': 'articulated-looking_cylinder_sphere_proxy',
        },
        'robot_configuration': {
            'base_only': True,
            'removed_so101_elements': removed_so101_elements,
            'retained_sensor_mast': 'rgbd_mast_link',
        },
    }
    base['fidelity'] = {
        'implementation_status': {
            'prepared': [
                'actual occupancy-grid collision and visual mesh',
                'identity-coordinate 20-waypoint route',
                'two fixed-box detours and one person-crossing stop',
                'physical low-traction patch and recovery supervisor',
                'simultaneous overhead and robot-front Gazebo cameras',
            ],
            'runtime_validation_required': [
                '20-waypoint Nav2 completion',
                'two same-goal box detours and one same-goal person resume',
                'one low-traction stop, relocalize, and recovery sequence',
            ],
        },
        'preserved': [
            'source occupancy cells, resolution, origin, and metric extent',
            'actual-map waypoint coordinates without scale or warp',
            'six source records grouped into three video-grounded scenes',
            'same-goal resume requirement',
            'Nav2 target decision stack contract',
        ],
        'not_identical': [
            'unmeasured wall heights, materials, doors, and furniture',
            'unmeasured obstacle dimensions and semantic class',
            'unmeasured real floor friction',
            '610-second wall-clock timing',
            'real sensor and network noise realization',
        ],
        'claim': (
            'actual_occupancy_extrusion_functional_replay_'
            'not_3d_digital_twin'),
    }
    contract = output / 'scenario_contract.json'
    contract.write_text(
        json.dumps(base, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8')
    return base


def main() -> int:
    """Build actual-map simulation assets from command-line inputs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-contract', required=True, type=Path)
    parser.add_argument('--route', required=True, type=Path)
    parser.add_argument('--map-yaml', required=True, type=Path)
    parser.add_argument('--keepout-yaml', required=True, type=Path)
    parser.add_argument('--robot-urdf', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--wall-height-m', type=float, default=2.4)
    args = parser.parse_args()
    if args.wall_height_m <= 0.0:
        parser.error('--wall-height-m must be positive')
    for path in (args.base_contract, args.route, args.map_yaml,
                 args.keepout_yaml,
                 args.robot_urdf):
        if not path.is_file():
            parser.error(f'input does not exist: {path}')
    result = prepare(args)
    print(json.dumps({
        'status': 'prepared', 'scenario_id': result['scenario_id'],
        'map': result['navigation_map'],
        'world_reconstruction': result['world_reconstruction'],
    }, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
