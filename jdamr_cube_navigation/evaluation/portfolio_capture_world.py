#!/usr/bin/env python3
"""Create 3D portfolio capture assets without changing evaluation physics."""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path


CAMERA_TOPIC = '/portfolio_scene/image_raw'
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_RATE_HZ = 30.0
CAMERA_VISIBILITY_MASK = 1
LIDAR_VISIBILITY_MASK = 4
MARKER_NAMES = (
    'portfolio_route_marker', 'portfolio_start_marker',
    'portfolio_goal_marker', 'portfolio_pedestrian_left',
    'portfolio_pedestrian_right')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def _collision_digest(world: ET.Element) -> str:
    digest = hashlib.sha256()
    for collision in world.findall('.//collision'):
        digest.update(ET.tostring(collision, encoding='utf-8'))
    return digest.hexdigest()


def _elements_digest(root: ET.Element, query: str) -> str:
    digest = hashlib.sha256()
    for element in root.findall(query):
        digest.update(ET.tostring(element, encoding='utf-8'))
    return digest.hexdigest()


def _follow_camera() -> ET.Element:
    """Build a massless chase camera attached to the navigation base."""
    gazebo = ET.Element('gazebo', {'reference': 'base_footprint'})
    sensor = ET.SubElement(
        gazebo, 'sensor', {'name': 'portfolio_camera', 'type': 'camera'})
    # 차체의 뒤·옆에서 따라가는 사선 시점이다. 로봇과 복도의
    # 높이·전후 관계를 유지하면서 주행 전체에 차체가 화면에 남는다.
    ET.SubElement(sensor, 'pose').text = '-1.1 -0.42 0.85 0 0.48 0.20'
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = str(CAMERA_RATE_HZ)
    ET.SubElement(sensor, 'topic').text = CAMERA_TOPIC
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = '1.42'
    ET.SubElement(camera, 'visibility_mask').text = str(
        CAMERA_VISIBILITY_MASK)
    image = ET.SubElement(camera, 'image')
    ET.SubElement(image, 'width').text = str(CAMERA_WIDTH)
    ET.SubElement(image, 'height').text = str(CAMERA_HEIGHT)
    ET.SubElement(image, 'format').text = 'R8G8B8'
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.1'
    ET.SubElement(clip, 'far').text = '100'
    return gazebo


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
                color: str) -> None:
    visual = ET.SubElement(link, 'visual', {'name': name})
    ET.SubElement(visual, 'pose').text = pose
    shape = ET.SubElement(ET.SubElement(visual, 'geometry'), geometry_tag)
    for key, value in geometry.items():
        ET.SubElement(shape, key).text = value
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = color
    ET.SubElement(material, 'diffuse').text = color
    ET.SubElement(visual, 'visibility_flags').text = str(
        CAMERA_VISIBILITY_MASK)


def _person_model(name: str, pose: str, shirt: str) -> ET.Element:
    """Build a visual-only pedestrian used to show crossing complexity."""
    model = ET.Element('model', {'name': name})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = pose
    link = ET.SubElement(model, 'link', {'name': 'body'})
    _add_visual(
        link, 'torso', '0 0 0.14 0 0 0', 'cylinder',
        {'radius': '0.16', 'length': '0.58'}, shirt)
    _add_visual(
        link, 'head', '0 0 0.57 0 0 0', 'sphere',
        {'radius': '0.13'}, '0.78 0.58 0.42 1')
    _add_visual(
        link, 'left_leg', '0 0.08 -0.27 0 0 0', 'box',
        {'size': '0.11 0.11 0.48'}, '0.10 0.14 0.18 1')
    _add_visual(
        link, 'right_leg', '0 -0.08 -0.27 0 0 0', 'box',
        {'size': '0.11 0.11 0.48'}, '0.10 0.14 0.18 1')
    return model


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
    _add_visual(
        link, 'person_torso', '0 0 0.14 0 0 0', 'cylinder',
        {'radius': '0.16', 'length': '0.58'}, '0.92 0.18 0.12 1')
    _add_visual(
        link, 'person_head', '0 0 0.57 0 0 0', 'sphere',
        {'radius': '0.13'}, '0.82 0.62 0.45 1')
    _add_visual(
        link, 'person_left_leg', '0 0.08 -0.27 0 0 0', 'box',
        {'size': '0.11 0.11 0.48'}, '0.12 0.16 0.21 1')
    _add_visual(
        link, 'person_right_leg', '0 -0.08 -0.27 0 0 0', 'box',
        {'size': '0.11 0.11 0.48'}, '0.12 0.16 0.21 1')


def _style_capture_environment(world: ET.Element) -> None:
    """Give the capture copy readable industrial colors."""
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


def _capture_models() -> tuple[ET.Element, ...]:
    return (
        _marker(
            'portfolio_route_marker', '-1 0 0.003 0 0 0', 'box',
            {'size': '14 0.06 0.004'}, '0.05 0.45 0.8 1'),
        _marker(
            'portfolio_start_marker', '-8 0 0.007 0 0 0', 'cylinder',
            {'radius': '0.28', 'length': '0.01'}, '0.1 0.35 0.95 1'),
        _marker(
            'portfolio_goal_marker', '6 0 0.007 0 0 0', 'cylinder',
            {'radius': '0.28', 'length': '0.01'}, '0.1 0.8 0.35 1'),
        _person_model(
            'portfolio_pedestrian_left', '-2 18 0.5 0 0 0',
            '0.16 0.52 0.82 1'),
        _person_model(
            'portfolio_pedestrian_right', '2 22 0.5 0 0 0',
            '0.90 0.62 0.12 1'),
    )


def build_capture_world(source: Path, output: Path) -> dict:
    """Add camera-only visuals while proving collision XML is unchanged."""
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

    _style_capture_environment(world)
    _style_detected_person(world)
    for model in _capture_models():
        world.append(model)
    if _collision_digest(world) != collision_before:
        raise RuntimeError('capture visuals changed collision XML')

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return {
        'schema_version': 1,
        'source': {'path': str(source.resolve()), 'sha256': _sha256(source)},
        'output': {'path': str(output.resolve()), 'sha256': _sha256(output)},
        'visual_only_markers': list(MARKER_NAMES),
        'detected_obstacle_visual': 'camera_person_lidar_box_proxy',
        'collision_xml_unchanged': True,
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
    root.append(_follow_camera())
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return {
        'schema_version': 1,
        'source': {'path': str(source.resolve()), 'sha256': _sha256(source)},
        'output': {'path': str(output.resolve()), 'sha256': _sha256(output)},
        'hidden_visual_links': hidden,
        'hidden_visual_count': len(hidden),
        'camera': {
            'source': 'gazebo_camera_sensor', 'topic': CAMERA_TOPIC,
            'width': CAMERA_WIDTH, 'height': CAMERA_HEIGHT,
            'rate_hz': CAMERA_RATE_HZ,
            'view': 'robot_follow_oblique_3d_corridor'},
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
