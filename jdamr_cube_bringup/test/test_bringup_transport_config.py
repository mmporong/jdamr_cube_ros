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
