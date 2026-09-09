#!/usr/bin/env python3
"""Create fixed-view capture assets with explicitly recorded doorway changes."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


CAMERA_TOPIC = '/portfolio_scene/image_raw'
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_RATE_HZ = 30.0
CAMERA_VISIBILITY_MASK = 1
LIDAR_VISIBILITY_MASK = 4
PEDESTRIAN_DOOR_CENTER_X_M = 1.5
PEDESTRIAN_DOOR_WIDTH_M = 1.8
PEDESTRIAN_DOORWAY_EDGE_Y_M = 1.5
MARKER_NAMES = (
    'portfolio_architectural_shell',)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def _collision_digest(
        world: ET.Element, excluded_models: tuple[str, ...] = ()) -> str:
    digest = hashlib.sha256()
    for model in world.findall('model'):
        if model.attrib.get('name') in excluded_models:
            continue
        for collision in model.findall('.//collision'):
            digest.update(ET.tostring(collision, encoding='utf-8'))
    return digest.hexdigest()


def _elements_digest(root: ET.Element, query: str) -> str:
    digest = hashlib.sha256()
    for element in root.findall(query):
        digest.update(ET.tostring(element, encoding='utf-8'))
    return digest.hexdigest()


def _fixed_world_camera() -> ET.Element:
    """Build an oblique world camera that covers the complete route."""
    model = ET.Element('model', {'name': 'portfolio_scene_camera'})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = '-0.5 -11 5.6 0 0 0'
    link = ET.SubElement(model, 'link', {'name': 'camera_mount'})
    sensor = ET.SubElement(
        link, 'sensor', {'name': 'portfolio_camera', 'type': 'camera'})
    ET.SubElement(sensor, 'pose').text = '0 0 0 0 0.43 1.5708'
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = str(CAMERA_RATE_HZ)
    ET.SubElement(sensor, 'topic').text = CAMERA_TOPIC
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = '1.18'
    ET.SubElement(camera, 'visibility_mask').text = str(
        CAMERA_VISIBILITY_MASK)
    image = ET.SubElement(camera, 'image')
    ET.SubElement(image, 'width').text = str(CAMERA_WIDTH)
    ET.SubElement(image, 'height').text = str(CAMERA_HEIGHT)
    ET.SubElement(image, 'format').text = 'R8G8B8'
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.1'
    ET.SubElement(clip, 'far').text = '100'
    return model


def _split_corridor_walls_for_doors(world: ET.Element) -> None:
    """Open aligned physical doorways on both pedestrian crossing edges."""
    wall_min_x_m = -10.0
    wall_max_x_m = 10.0
    door_min_x_m = (
        PEDESTRIAN_DOOR_CENTER_X_M - PEDESTRIAN_DOOR_WIDTH_M / 2.0)
    door_max_x_m = (
        PEDESTRIAN_DOOR_CENTER_X_M + PEDESTRIAN_DOOR_WIDTH_M / 2.0)
    segments = (
        ('west', wall_min_x_m, door_min_x_m),
        ('east', door_max_x_m, wall_max_x_m),
    )
    for wall_name in ('wall_north', 'wall_south'):
        model = world.find(f"./model[@name='{wall_name}']")
        if model is None:
            raise ValueError(f'{wall_name} is missing')
        link = model.find("./link[@name='link']")
        if link is None:
            raise ValueError(f'{wall_name} link is missing')
        collisions = link.findall('collision')
        visuals = link.findall('visual')
        if len(collisions) != 1 or len(visuals) != 1:
            raise ValueError(f'{wall_name} geometry drifted')
        for element in (*collisions, *visuals):
            link.remove(element)
        for side, start_x_m, end_x_m in segments:
            length_m = end_x_m - start_x_m
            center_x_m = (start_x_m + end_x_m) / 2.0
            pose = f'{center_x_m} 0 0 0 0 0'
            size = f'{length_m} 0.1 1'
            collision = ET.SubElement(
                link, 'collision', {'name': f'door_{side}_collision'})
            ET.SubElement(collision, 'pose').text = pose
            box = ET.SubElement(
                ET.SubElement(collision, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = size
            visual = ET.SubElement(
                link, 'visual', {'name': f'door_{side}_visual'})
            ET.SubElement(visual, 'pose').text = pose
            box = ET.SubElement(ET.SubElement(visual, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = size


def _marker(name: str, pose: str, geometry_tag: str,
            geometry: dict[str, str], color: str) -> ET.Element:
    model = ET.Element('model', {'name': name})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = pose
    link = ET.SubElement(model, 'link', {'name': 'link'})
    visual = ET.SubElement(link, 'visual', {'name': 'visual'})
    ET.SubElement(visual, 'cast_shadows').text = 'false'
    ET.SubElement(visual, 'visibility_flags').text = str(
        CAMERA_VISIBILITY_MASK)
    shape = ET.SubElement(ET.SubElement(visual, 'geometry'), geometry_tag)
    for key, value in geometry.items():
        ET.SubElement(shape, key).text = value
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = color
    ET.SubElement(material, 'diffuse').text = color
    return model


def _add_visual(link: ET.Element, name: str, pose: str,
                geometry_tag: str, geometry: dict[str, str],
                color: str, *, emissive: str | None = None,
                cast_shadows: bool = True) -> None:
    if len(pose.split()) != 6:
        raise ValueError(f'visual pose must contain xyz and rpy: {name}')
    visual = ET.SubElement(link, 'visual', {'name': name})
    ET.SubElement(visual, 'pose').text = pose
    shape = ET.SubElement(ET.SubElement(visual, 'geometry'), geometry_tag)
    for key, value in geometry.items():
        ET.SubElement(shape, key).text = value
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = color
    ET.SubElement(material, 'diffuse').text = color
    if emissive is not None:
        ET.SubElement(material, 'emissive').text = emissive
    ET.SubElement(visual, 'cast_shadows').text = str(cast_shadows).lower()
    ET.SubElement(visual, 'visibility_flags').text = str(
        CAMERA_VISIBILITY_MASK)


def _person_model(name: str, pose: str, shirt: str) -> ET.Element:
    """Build a visual-only pedestrian used to show crossing complexity."""
    model = ET.Element('model', {'name': name})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = pose
    link = ET.SubElement(model, 'link', {'name': 'body'})
    _add_person_visuals(link, shirt)
    return model


def _add_person_visuals(link: ET.Element, shirt: str) -> None:
    """Add a readable human silhouette without altering collision geometry."""
    skin = '0.70 0.47 0.31 1'
    trousers = '0.07 0.10 0.14 1'
    shoes = '0.025 0.03 0.04 1'
    _add_visual(
        link, 'person_torso', '0 0 0.31 0 0 0', 'cylinder',
        {'radius': '0.17', 'length': '0.62'}, shirt)
    _add_visual(
        link, 'person_shoulder', '0 0 0.55 1.5708 0 0', 'cylinder',
        {'radius': '0.08', 'length': '0.43'}, shirt)
    _add_visual(
        link, 'person_head', '0 0 0.78 0 0 0', 'sphere',
        {'radius': '0.14'}, skin)
    _add_visual(
        link, 'person_hair', '-0.025 0 0.87 0 0 0', 'sphere',
        {'radius': '0.115'}, '0.035 0.025 0.02 1')
    _add_visual(
        link, 'person_left_arm', '0 0.22 0.28 -0.20 0.08 0', 'cylinder',
        {'radius': '0.055', 'length': '0.55'}, shirt)
    _add_visual(
        link, 'person_right_arm', '0 -0.22 0.28 0.20 -0.08 0', 'cylinder',
        {'radius': '0.055', 'length': '0.55'}, shirt)
    _add_visual(
        link, 'person_left_hand', '0 0.27 0.01 0 0 0', 'sphere',
        {'radius': '0.06'}, skin)
    _add_visual(
        link, 'person_right_hand', '0 -0.27 0.01 0 0 0', 'sphere',
        {'radius': '0.06'}, skin)
    _add_visual(
        link, 'person_left_leg', '0 0.09 -0.26 -0.08 0 0', 'cylinder',
        {'radius': '0.075', 'length': '0.70'}, trousers)
    _add_visual(
        link, 'person_right_leg', '0 -0.09 -0.26 0.08 0 0', 'cylinder',
        {'radius': '0.075', 'length': '0.70'}, trousers)
    _add_visual(
        link, 'person_left_shoe', '0.055 0.09 -0.60 0 0 0', 'box',
        {'size': '0.23 0.12 0.09'}, shoes)
    _add_visual(
        link, 'person_right_shoe', '0.055 -0.09 -0.60 0 0 0', 'box',
        {'size': '0.23 0.12 0.09'}, shoes)


def _style_detected_person(world: ET.Element) -> None:
    """Replace the camera-visible box with a person while retaining LiDAR."""
    link = world.find(
        "./model[@name='g003_preloaded_front_observation_probe']/link")
    if link is None:
        raise ValueError('front observation probe is missing')
    box = link.find("visual[@name='visual']")
    if box is None:
        raise ValueError('front observation probe visual is missing')
    flags = box.find('visibility_flags')
    if flags is None or flags.text != str(LIDAR_VISIBILITY_MASK):
        raise ValueError('front observation probe LiDAR mask drifted')
    # 원래 박스는 LiDAR 전용 프록시로 유지한다. 촬영 카메라 레이어에는 사람
    # 비주얼만 보이므로 검증 형상과 촬영 표현이 서로 영향을 주지 않는다.
    box.attrib['name'] = 'lidar_proxy'
    _add_person_visuals(link, '0.72 0.16 0.10 1')


def _style_route_crate(world: ET.Element) -> None:
    """Render the persistent route obstacle as a wooden transport crate."""
    link = world.find(
        "./model[@name='g003_preloaded_route_obstacle']/link")
    if link is None:
        raise ValueError('route obstacle is missing')
    core = link.find("visual[@name='visual']")
    if core is None:
        raise ValueError('route obstacle visual is missing')
    core.attrib['name'] = 'lidar_proxy'
    flags = core.find('visibility_flags')
    if flags is None:
        flags = ET.SubElement(core, 'visibility_flags')
    flags.text = str(LIDAR_VISIBILITY_MASK)
    wood = '0.46 0.24 0.09 1'
    light_wood = '0.68 0.39 0.15 1'
    _add_visual(
        link, 'crate_body', '0 0 0 0 0 0', 'box',
        {'size': '0.49 0.39 0.98'}, wood)
    for index, z_m in enumerate((-0.36, 0.0, 0.36)):
        _add_visual(
            link, f'crate_band_{index}', f'0.255 0 {z_m} 0 0 0', 'box',
            {'size': '0.025 0.43 0.075'}, light_wood)
    for index, y_m in enumerate((-0.16, 0.16)):
        _add_visual(
            link, f'crate_post_{index}', f'0.255 {y_m} 0 0 0 0', 'box',
            {'size': '0.025 0.055 0.92'}, light_wood)
    _add_visual(
        link, 'crate_hazard', '0.269 0 0.18 0 0 0', 'box',
        {'size': '0.008 0.20 0.20'}, '0.92 0.50 0.04 1',
        emissive='0.20 0.08 0.0 1', cast_shadows=False)


def _style_capture_environment(world: ET.Element) -> None:
    """Keep canonical geometry on LiDAR and hide it from the camera layer."""
    palette = {
        'ground_plane': '0.16 0.19 0.23 1',
        'wall_north': '0.31 0.38 0.46 1',
        'wall_south': '0.31 0.38 0.46 1',
        'wall_west': '0.24 0.30 0.37 1',
        'wall_east': '0.24 0.30 0.37 1',
        'feature_north_west': '0.12 0.55 0.72 1',
        'feature_south_mid': '0.88 0.47 0.12 1',
        'feature_north_east': '0.12 0.55 0.72 1',
        'feature_south_east': '0.88 0.47 0.12 1',
    }
    for model_name, color in palette.items():
        model = world.find(f"./model[@name='{model_name}']")
        if model is None:
            continue
        for visual in model.findall('.//visual'):
            material = visual.find('material')
            if material is None:
                material = ET.SubElement(visual, 'material')
            for tag in ('ambient', 'diffuse'):
                channel = material.find(tag)
                if channel is None:
                    channel = ET.SubElement(material, tag)
                channel.text = color
            flags = visual.find('visibility_flags')
            if flags is None:
                flags = ET.SubElement(visual, 'visibility_flags')
            flags.text = str(
                LIDAR_VISIBILITY_MASK | CAMERA_VISIBILITY_MASK
                if model_name.startswith('feature_')
                else LIDAR_VISIBILITY_MASK)


def _architectural_shell() -> ET.Element:
    """Build the cutaway camera shell around the matching physical doorways."""
    model = ET.Element('model', {'name': 'portfolio_architectural_shell'})
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', {'name': 'interior'})
    _add_visual(
        link, 'floor', '0 0 0.005 0 0 0', 'box',
        {'size': '20 2.28 0.01'}, '0.36 0.39 0.42 1')
    # 관제 카메라는 절개도 시점이다. 남쪽 벽과 천장의 카메라 비주얼만
    # 생략하고, 원본 충돌 형상과 LiDAR 벽은 유지한다.
    door_min_x_m = (
        PEDESTRIAN_DOOR_CENTER_X_M - PEDESTRIAN_DOOR_WIDTH_M / 2.0)
    door_max_x_m = (
        PEDESTRIAN_DOOR_CENTER_X_M + PEDESTRIAN_DOOR_WIDTH_M / 2.0)
    wall_segments = (
        ('west', -10.0, door_min_x_m),
        ('east', door_max_x_m, 10.0),
    )
    for side, start_x_m, end_x_m in wall_segments:
        length_m = end_x_m - start_x_m
        center_x_m = (start_x_m + end_x_m) / 2.0
        _add_visual(
            link, f'north_wall_{side}',
            f'{center_x_m} 1.145 1.31 0 0 0', 'box',
            {'size': f'{length_m} 0.025 2.62'}, '0.73 0.75 0.76 1')
        _add_visual(
            link, f'north_baseboard_{side}',
            f'{center_x_m} 1.127 0.08 0 0 0', 'box',
            {'size': f'{length_m} 0.035 0.16'}, '0.13 0.16 0.18 1')
    _add_visual(
        link, 'north_wall_band', '0 1.127 2.22 0 0 0', 'box',
        {'size': '20 0.03 0.07'}, '0.12 0.35 0.42 1')
    for seam, x_m in enumerate(range(-9, 10)):
        _add_visual(
            link, f'floor_seam_x_{seam}', f'{x_m} 0 0.012 0 0 0',
            'box', {'size': '0.012 2.26 0.004'},
            '0.21 0.23 0.25 1', cast_shadows=False)
    for seam, y_m in enumerate((-0.57, 0.0, 0.57)):
        _add_visual(
            link, f'floor_seam_y_{seam}', f'0 {y_m} 0.012 0 0 0',
            'box', {'size': '19.98 0.012 0.004'},
            '0.21 0.23 0.25 1', cast_shadows=False)
    door_positions = (
        -7.5, -5.0, -2.5, PEDESTRIAN_DOOR_CENTER_X_M, 5.0, 7.5)
    for side, y_m, yaw in (('north', 1.125, 0.0),):
        for index, x_m in enumerate(door_positions):
            crossing_door = x_m == PEDESTRIAN_DOOR_CENTER_X_M
            door_color = (
                '0.19 0.23 0.25 1' if index % 3
                else '0.16 0.31 0.34 1')
            if not crossing_door:
                _add_visual(
                    link, f'{side}_door_{index}',
                    f'{x_m} {y_m} 1.08 0 0 {yaw}', 'box',
                    {'size': '0.86 0.025 2.12'}, door_color)
            frame_y_m = y_m - 0.019 if y_m > 0 else y_m + 0.019
            frame_offset_x_m = (
                PEDESTRIAN_DOOR_WIDTH_M / 2.0 if crossing_door else 0.48)
            for edge, offset_x_m in (
                    ('left', -frame_offset_x_m),
                    ('right', frame_offset_x_m)):
                _add_visual(
                    link, f'{side}_door_{index}_{edge}',
                    f'{x_m + offset_x_m} {frame_y_m} 1.10 0 0 0', 'box',
                    {'size': '0.065 0.045 2.20'}, '0.08 0.10 0.11 1')
            _add_visual(
                link, f'{side}_door_{index}_top',
                f'{x_m} {frame_y_m} 2.18 0 0 0', 'box',
                {'size': (
                    f'{PEDESTRIAN_DOOR_WIDTH_M + 0.06} 0.045 0.065'
                    if crossing_door else '1.02 0.045 0.065')},
                '0.08 0.10 0.11 1')
            if not crossing_door:
                handle_y_m = y_m - 0.035 if y_m > 0 else y_m + 0.035
                _add_visual(
                    link, f'{side}_door_{index}_handle',
                    f'{x_m + 0.30} {handle_y_m} 1.03 1.5708 0 0',
                    'cylinder', {'radius': '0.018', 'length': '0.09'},
                    '0.65 0.67 0.68 1')
    for side, y_m in (('north', 1.095),):
        _add_visual(
            link, f'{side}_exit_sign', f'7.5 {y_m} 2.34 0 0 0', 'box',
            {'size': '0.46 0.025 0.20'}, '0.03 0.42 0.18 1',
            emissive='0.02 0.20 0.08 1', cast_shadows=False)
    return model


def _capture_lights() -> tuple[ET.Element, ...]:
    lights = []
    for index, x_m in enumerate((-7.5, -4.5, -1.5, 1.5, 4.5, 7.5)):
        light = ET.Element('light', {
            'name': f'portfolio_ceiling_light_{index}', 'type': 'point'})
        ET.SubElement(light, 'pose').text = f'{x_m} 0 2.42 0 0 0'
        ET.SubElement(light, 'cast_shadows').text = 'false'
        ET.SubElement(light, 'diffuse').text = '0.82 0.86 0.88 1'
        ET.SubElement(light, 'specular').text = '0.18 0.20 0.22 1'
        ET.SubElement(light, 'intensity').text = '0.55'
        attenuation = ET.SubElement(light, 'attenuation')
        ET.SubElement(attenuation, 'range').text = '5.0'
        ET.SubElement(attenuation, 'constant').text = '0.45'
        ET.SubElement(attenuation, 'linear').text = '0.12'
        ET.SubElement(attenuation, 'quadratic').text = '0.025'
        lights.append(light)
    return tuple(lights)


def _style_scene(world: ET.Element) -> None:
    scene = world.find('scene')
    if scene is None:
        scene = ET.Element('scene')
        first_model = world.find('model')
        if first_model is None:
            world.append(scene)
        else:
            world.insert(list(world).index(first_model), scene)
    for tag, value in (
            ('ambient', '0.24 0.27 0.30 1'),
            ('background', '0.06 0.075 0.09 1'),
            ('shadows', 'true'), ('grid', 'false')):
        child = scene.find(tag)
        if child is None:
            child = ET.SubElement(scene, tag)
        child.text = value


def _add_urdf_visual(link: ET.Element, name: str, xyz: str, rpy: str,
                     geometry_tag: str, geometry: dict[str, str],
                     color: str) -> None:
    visual = ET.SubElement(link, 'visual', {'name': name})
    ET.SubElement(visual, 'origin', {'xyz': xyz, 'rpy': rpy})
    shape = ET.SubElement(ET.SubElement(visual, 'geometry'), geometry_tag)
    for key, value in geometry.items():
        shape.set(key, value)
    material = ET.SubElement(visual, 'material', {'name': f'{name}_material'})
    ET.SubElement(material, 'color', {'rgba': color})


def _style_robot_visuals(root: ET.Element) -> int:
    """Replace test primitives with a camera-only industrial AMR shell."""
    styled_links = ('base_link', 'left_wheel_link', 'right_wheel_link',
                    'caster_link_front', 'caster_link_rear', 'laser_link')
    links = {link.attrib.get('name'): link for link in root.findall('link')}
    for name in styled_links:
        link = links.get(name)
        if link is None:
            raise ValueError(f'robot visual link is missing: {name}')
        for visual in tuple(link.findall('visual')):
            link.remove(visual)

    base = links['base_link']
    _add_urdf_visual(
        base, 'amr_lower_chassis', '0 0 -0.005', '0 0 0', 'box',
        {'size': '0.44 0.36 0.085'}, '0.035 0.045 0.055 1')
    _add_urdf_visual(
        base, 'amr_body', '-0.015 0 0.072', '0 0 0', 'box',
        {'size': '0.36 0.30 0.105'}, '0.09 0.27 0.34 1')
    _add_urdf_visual(
        base, 'amr_top_deck', '-0.025 0 0.142', '0 0 0', 'box',
        {'size': '0.29 0.24 0.035'}, '0.03 0.055 0.068 1')
    _add_urdf_visual(
        base, 'amr_front_bumper', '0.221 0 -0.004', '0 0 0', 'box',
        {'size': '0.035 0.33 0.07'}, '0.015 0.02 0.025 1')
    _add_urdf_visual(
        base, 'amr_rear_bumper', '-0.221 0 -0.004', '0 0 0', 'box',
        {'size': '0.035 0.33 0.07'}, '0.015 0.02 0.025 1')
    for side, y_m in (('left', 0.105), ('right', -0.105)):
        _add_urdf_visual(
            base, f'amr_headlight_{side}', f'0.241 {y_m} 0.052',
            '0 0 0', 'box', {'size': '0.012 0.065 0.027'},
            '0.45 0.93 1 1')
        _add_urdf_visual(
            base, f'amr_tail_light_{side}', f'-0.241 {y_m} 0.052',
            '0 0 0', 'box', {'size': '0.012 0.055 0.025'},
            '0.95 0.08 0.035 1')
    _add_urdf_visual(
        base, 'amr_status_bar', '0.05 0 0.129', '0 0 0', 'box',
        {'size': '0.16 0.245 0.012'}, '0.02 0.72 0.76 1')

    for name in ('left_wheel_link', 'right_wheel_link'):
        wheel = links[name]
        _add_urdf_visual(
            wheel, f'{name}_tire', '0 0 0', '1.570795 0 0', 'cylinder',
            {'length': '0.055', 'radius': '0.078'}, '0.018 0.022 0.026 1')
        _add_urdf_visual(
            wheel, f'{name}_hub', '0 0 0', '1.570795 0 0', 'cylinder',
            {'length': '0.058', 'radius': '0.037'}, '0.26 0.29 0.31 1')
    for name in ('caster_link_front', 'caster_link_rear'):
        _add_urdf_visual(
            links[name], f'{name}_visual', '0 0 0', '0 0 0', 'sphere',
            {'radius': '0.031'}, '0.025 0.03 0.035 1')
    laser = links['laser_link']
    _add_urdf_visual(
        laser, 'lidar_body', '0 0 0', '0 0 0', 'cylinder',
        {'length': '0.055', 'radius': '0.043'}, '0.025 0.035 0.045 1')
    _add_urdf_visual(
        laser, 'lidar_scan_ring', '0 0 0.018', '0 0 0', 'cylinder',
        {'length': '0.018', 'radius': '0.045'}, '0.02 0.62 0.75 1')
    _add_urdf_visual(
        laser, 'lidar_cap', '0 0 0.042', '0 0 0', 'cylinder',
        {'length': '0.012', 'radius': '0.038'}, '0.035 0.05 0.06 1')
    return len(styled_links)


def _capture_models() -> tuple[ET.Element, ...]:
    return (_architectural_shell(), _fixed_world_camera())


def build_capture_world(source: Path, output: Path,
                        keepout_polygon_m=None) -> dict:
    """Add a fixed view and doorways, proving other collisions are unchanged."""
    if not source.is_file() or source.is_symlink():
        raise ValueError(f'canonical world must be a regular file: {source}')
    if output.exists():
        raise ValueError(f'capture world already exists: {output}')
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('canonical SDF has no world')
    reserved = {'portfolio_scene_camera', *MARKER_NAMES}
    existing = {model.attrib.get('name') for model in world.findall('model')}
    if reserved & existing:
        raise ValueError('canonical world already contains capture visuals')
    collision_before = _collision_digest(world)
    doorway_walls = ('wall_north', 'wall_south')
    other_collisions_before = _collision_digest(
        world, excluded_models=doorway_walls)

    _split_corridor_walls_for_doors(world)
    _style_capture_environment(world)
    _style_scene(world)
    _style_detected_person(world)
    _style_route_crate(world)
    for model in _capture_models():
        world.append(model)
    if keepout_polygon_m:
        marker = ET.SubElement(
            world, 'model', {'name': 'portfolio_keepout_zone'})
        ET.SubElement(marker, 'static').text = 'true'
        link = ET.SubElement(marker, 'link', {'name': 'zone_boundary'})
        for index, start in enumerate(keepout_polygon_m):
            end = keepout_polygon_m[(index + 1) % len(keepout_polygon_m)]
            dx_m, dy_m = end[0] - start[0], end[1] - start[1]
            x_m, y_m = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
            _add_visual(
                link, f'edge_{index}',
                f'{x_m} {y_m} 0.025 0 0 {math.atan2(dy_m, dx_m)}',
                'box', {'size': f'{math.hypot(dx_m, dy_m)} 0.06 0.012'},
                '0.85 0.08 0.42 1', emissive='0.3 0.02 0.1 1',
                cast_shadows=False)
    for light in _capture_lights():
        world.append(light)
    if _collision_digest(world) == collision_before:
        raise RuntimeError(
            'capture doorways did not change wall collisions')
    if (_collision_digest(world, excluded_models=doorway_walls)
            != other_collisions_before):
        raise RuntimeError('capture doorway changed unrelated collisions')

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return {
        'schema_version': 1,
        'source': {'path': str(source.resolve()), 'sha256': _sha256(source)},
        'output': {'path': str(output.resolve()), 'sha256': _sha256(output)},
        'visual_only_markers': list(MARKER_NAMES),
        'detected_obstacle_visual': 'camera_person_lidar_box_proxy',
        'collision_xml_unchanged': False,
        'collision_change_scope': 'corridor_edge_pedestrian_doorways',
        'pedestrian_doorway': {
            'center_x_m': PEDESTRIAN_DOOR_CENTER_X_M,
            'width_m': PEDESTRIAN_DOOR_WIDTH_M,
            'crossing_edge_y_m': PEDESTRIAN_DOORWAY_EDGE_Y_M,
            'walls': list(doorway_walls),
            'supported_entry_sides': ['left', 'right'],
            'camera_and_lidar_geometry_aligned': True,
        },
        'camera': {
            'source': 'gazebo_world_camera_sensor', 'topic': CAMERA_TOPIC,
            'width': CAMERA_WIDTH, 'height': CAMERA_HEIGHT,
            'rate_hz': CAMERA_RATE_HZ,
            'view': 'fixed_world_oblique_full_route'},
        'keepout_authored_polygon_m': keepout_polygon_m,
    }


def build_capture_urdf(source: Path, output: Path) -> dict:
    """Hide upper visuals and retain collision, joints, and inertia exactly."""
    if not source.is_file() or source.is_symlink():
        raise ValueError(f'canonical URDF must be a regular file: {source}')
    if output.exists():
        raise ValueError(f'capture URDF already exists: {output}')
    tree = ET.parse(source)
    root = tree.getroot()
    collision_before = _elements_digest(root, './/collision')
    inertial_before = _elements_digest(root, './/inertial')
    joints_before = _elements_digest(root, './/joint')
    hidden = []
    for link in root.findall('link'):
        name = link.attrib.get('name', '')
        if not name.startswith(('arm_', 'rgbd_')):
            continue
        visuals = link.findall('visual')
        if visuals:
            hidden.append(name)
        for visual in visuals:
            link.remove(visual)
    if not hidden:
        raise ValueError('non-navigation upper visual links were not found')
    if (_elements_digest(root, './/collision') != collision_before
            or _elements_digest(root, './/inertial') != inertial_before
            or _elements_digest(root, './/joint') != joints_before):
        raise RuntimeError('capture URDF changed physics or kinematics')
    styled_link_count = _style_robot_visuals(root)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return {
        'schema_version': 1,
        'source': {'path': str(source.resolve()), 'sha256': _sha256(source)},
        'output': {'path': str(output.resolve()), 'sha256': _sha256(output)},
        'hidden_visual_links': hidden,
        'hidden_visual_count': len(hidden),
        'styled_navigation_link_count': styled_link_count,
        'collision_xml_unchanged': True,
        'inertial_xml_unchanged': True,
        'joint_xml_unchanged': True,
    }


def main() -> int:
    """Write one capture-only world and its JSON provenance."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--manifest', required=True, type=Path)
    args = parser.parse_args()
    if args.manifest.exists():
        parser.error(f'manifest already exists: {args.manifest}')
    report = build_capture_world(args.source, args.output)
    args.manifest.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
