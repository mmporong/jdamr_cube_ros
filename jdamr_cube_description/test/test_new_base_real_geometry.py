"""Real bringup model must match the measured lower-base contract."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'urdf/new_base_real.urdf'
CONTRACT = ROOT / 'config/new_base_geometry.yaml'


def _xyz(joint):
    return tuple(float(value) for value in joint.find('origin').attrib['xyz'].split())


def test_new_base_footprint_lidar_and_wheels():
    geometry = yaml.safe_load(CONTRACT.read_text(encoding='utf-8'))
    root = ET.parse(MODEL).getroot()
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    links = {link.attrib['name']: link for link in root.findall('link')}
    rail_half = float(links['lower_front_rail_link'].find(
        'collision/geometry/box').attrib['size'].split()[0]) / 2.0
    wheel_half = float(links['left_wheel_link'].find(
        'collision/geometry/cylinder').attrib['length']) / 2.0
    front = _xyz(joints['lower_front_rail_joint'])[0] + rail_half
    rear = _xyz(joints['lower_rear_rail_joint'])[0] - rail_half
    left = _xyz(joints['left_wheel_joint'])[1] + wheel_half
    right = _xyz(joints['right_wheel_joint'])[1] - wheel_half
    assert abs(front - geometry['front_to_wheel_axis']['value']) < 1e-9
    assert abs(front - rear - geometry['frame_length']['value']) < 1e-9
    assert abs(left - right - geometry['wheel_outer_width']['value']) < 1e-9
    assert abs(_xyz(joints['left_wheel_joint'])[1] -
               _xyz(joints['right_wheel_joint'])[1] -
               geometry['wheel_center_separation']['value']) < 1e-9
    wheel = links['left_wheel_link'].find('collision/geometry/cylinder')
    assert float(wheel.attrib['radius']) == geometry['wheel_radius_initial']['value']
    assert float(wheel.attrib['length']) == geometry['wheel_visual_width']['value']
    assert _xyz(joints['laser_joint']) == tuple(geometry['laser_translation']['value'])
    assert float(joints['laser_joint'].find('origin').attrib['rpy'].split()[2]) == \
        geometry['laser_yaw']['value']
    assert 'arm_base_link' not in links
    assert 'caster_link_front' not in links
    assert {'rear_left_caster_contact_projection',
            'rear_right_caster_contact_projection'} <= links.keys()
