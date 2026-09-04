"""Regression tests for physical-robot DDS transport configuration."""

import ast
from pathlib import Path


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / 'launch' / 'real_bringup.launch.py')


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


def test_real_bringup_keeps_dds_off_the_wifi_interface():
    syntax = ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'))
    environment = {
        ast.literal_eval(node.args[0]): ast.literal_eval(node.args[1])
        for node in ast.walk(syntax)
        if isinstance(node, ast.Call)
        and _call_name(node) == 'SetEnvironmentVariable'
    }

    assert environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'


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
