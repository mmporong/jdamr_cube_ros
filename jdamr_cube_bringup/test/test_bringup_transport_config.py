"""Regression tests for physical-robot DDS transport configuration."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / 'launch' / 'real_bringup.launch.py')
ALIAS_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / 'launch' / 'jdamr_cube_bringup.launch.py')
URDF_PATH = (
    Path(__file__).resolve().parents[2]
    / 'jdamr_cube_description' / 'urdf' / 'jdamr_cube.urdf')


def _call_name(call):
    function = call.func
    if isinstance(function, ast.Name):
        return function.id
    if isinstance(function, ast.Attribute):
        return function.attr
    return ''


def test_real_bringup_forces_udp_before_starting_nodes():
    syntax = ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'))
    description = next(
        node.value for node in ast.walk(syntax)
        if isinstance(node, ast.Return)
        and isinstance(node.value, ast.Call)
        and _call_name(node.value) == 'LaunchDescription'
    )
    first_action = description.args[0].elts[0]

    assert _call_name(first_action) == 'SetEnvironmentVariable'
    assert ast.literal_eval(first_action.args[0]) == \
        'FASTDDS_BUILTIN_TRANSPORTS'
    assert ast.literal_eval(first_action.args[1]) == 'UDPv4'


def test_real_bringup_uses_subnet_discovery_for_sensor_delivery():
    syntax = ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'))
    environment = {
        ast.literal_eval(node.args[0]): ast.literal_eval(node.args[1])
        for node in ast.walk(syntax)
        if isinstance(node, ast.Call)
        and _call_name(node) == 'SetEnvironmentVariable'
    }

    assert environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SUBNET'


def test_real_bringup_uses_measured_base_geometry():
    for path in (LAUNCH_PATH, ALIAS_LAUNCH_PATH):
        source = path.read_text(encoding='utf-8')
        assert "'wheel_separation', default_value='0.510'" in source or \
            "'wheel_separation': '0.510'" in source

    root = ET.parse(URDF_PATH).getroot()
    laser_joint = next(
        joint for joint in root.findall('joint')
        if joint.attrib['name'] == 'laser_joint')
    origin = laser_joint.find('origin')

    assert origin.attrib['xyz'] == '-0.010 0 0.075'
    assert origin.attrib['rpy'] == '0 0 3.141592653589793'


def test_real_bringup_keeps_odom_fast_but_limits_dynamic_tf_to_20_hz():
    syntax = ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'))
    base_node = next(
        node for node in ast.walk(syntax)
        if isinstance(node, ast.Call) and _call_name(node) == 'Node'
        and any(
            keyword.arg == 'package'
            and ast.literal_eval(keyword.value) == 'jdamr_base_driver'
            for keyword in node.keywords))
    parameters = next(
        keyword.value for keyword in base_node.keywords
        if keyword.arg == 'parameters')
    parameter_dict = parameters.elts[0]
    values = {
        ast.literal_eval(key): value
        for key, value in zip(parameter_dict.keys, parameter_dict.values)}

    assert ast.literal_eval(values['tf_publish_hz']) == 20.0
