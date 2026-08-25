"""Regression tests for physical SLAM DDS transport configuration."""

import ast
from pathlib import Path
import re


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'cartographer_real.launch.py'
RESET_PATH = PACKAGE_ROOT / 'scripts' / 'reset_map.sh'
PHYSICAL_CONFIG_PATHS = (
    PACKAGE_ROOT / 'config' / 'jdamr_cube_2d_real.lua',
    PACKAGE_ROOT / 'config' / 'jdamr_cube_2d_corridor.lua',
)


def _call_name(call):
    function = call.func
    if isinstance(function, ast.Name):
        return function.id
    if isinstance(function, ast.Attribute):
        return function.attr
    return ''


def test_real_slam_forces_udp_before_starting_nodes():
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


def test_map_reset_defaults_to_udp_transport():
    source = RESET_PATH.read_text(encoding='utf-8')
    export = (
        'export FASTDDS_BUILTIN_TRANSPORTS='
        '"${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"')

    assert export in source
    assert source.index(export) < source.index('ros2 pkg prefix')


def test_physical_slam_limits_pose_tf_to_twenty_hertz():
    """A 200 Hz map-to-odom stream starves the Python safety monitor on Pi."""
    for path in PHYSICAL_CONFIG_PATHS:
        source = path.read_text(encoding='utf-8')
        match = re.search(
            r'^\s*pose_publish_period_sec\s*=\s*([^,]+),',
            source, flags=re.MULTILINE)

        assert match, path
        assert float(match.group(1)) == 0.05, path
