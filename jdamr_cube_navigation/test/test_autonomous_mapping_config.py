"""Static regression tests for the physical-robot mapping safety envelope."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMS_PATH = PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'autonomous_mapping.launch.py'
NAVIGATION_LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
BT_PATH = (
    PACKAGE_ROOT / 'behavior_trees' / 'navigate_to_pose_safe_mapping.xml')
SETUP_PATH = PACKAGE_ROOT / 'setup.py'
EXPLORER_PATH = (
    PACKAGE_ROOT / 'jdamr_cube_navigation' / 'frontier_explorer.py')
EXPECTED_FOOTPRINT = [
    [0.33, 0.25], [0.33, -0.25], [-0.25, -0.25], [-0.25, 0.25],
]


def _params():
    with PARAMS_PATH.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _node_params(config, node_name, nested=False):
    node = config[node_name]
    if nested:
        node = node[node_name]
    return node['ros__parameters']


def _literal(value):
    return ast.literal_eval(value) if isinstance(value, str) else value


def _call_name(call):
    function = call.func
    if isinstance(function, ast.Name):
        return function.id
    if isinstance(function, ast.Attribute):
        return function.attr
    return ''


def _assert_udp_transport_call(call):
    assert _call_name(call) == 'SetEnvironmentVariable'
    assert ast.literal_eval(call.args[0]) == 'FASTDDS_BUILTIN_TRANSPORTS'
    assert ast.literal_eval(call.args[1]) == 'UDPv4'


def test_physical_navigation_launches_force_fastdds_udp_transport():
    autonomous_syntax = ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'))
    description = next(
        node.value for node in ast.walk(autonomous_syntax)
        if isinstance(node, ast.Return)
        and isinstance(node.value, ast.Call)
        and _call_name(node.value) == 'LaunchDescription'
    )
    _assert_udp_transport_call(description.args[0].elts[0])

    navigation_syntax = ast.parse(
        NAVIGATION_LAUNCH_PATH.read_text(encoding='utf-8'))
    add_actions = [
        node.value for node in navigation_syntax.body
        if isinstance(node, ast.FunctionDef)
        and node.name == 'generate_launch_description'
        for node in node.body
        if isinstance(node, ast.Expr)
        and isinstance(node.value, ast.Call)
        and _call_name(node.value) == 'add_action'
    ]
    assert add_actions
    _assert_udp_transport_call(add_actions[0].args[0])


def test_controller_uses_forward_only_collision_aware_rpp():
    controller = _node_params(_params(), 'controller_server')['FollowPath']

    assert controller['plugin'].endswith('RegulatedPurePursuitController')
    assert controller['desired_linear_vel'] <= 0.18
    assert controller['allow_reversing'] is False
    assert controller['use_collision_detection'] is True
    assert controller['rotate_to_heading_angular_vel'] <= 0.7


def test_velocity_smoother_clamps_forward_and_angular_velocity():
    smoother = _node_params(_params(), 'velocity_smoother')

    assert smoother['min_velocity'][0] == 0.0, (
        'physical mapping must not command reverse linear velocity')
    assert smoother['max_velocity'][0] <= 0.18
    assert abs(smoother['min_velocity'][2]) <= 0.7
    assert smoother['max_velocity'][2] <= 0.7


def test_navfn_refuses_paths_through_unknown_space():
    planner = _node_params(_params(), 'planner_server')['GridBased']

    assert planner['plugin'].endswith('NavfnPlanner')
    assert planner['allow_unknown'] is False


def test_costmaps_use_exact_robot_footprint_and_scan_obstacle_layers():
    config = _params()
    for name in ('local_costmap', 'global_costmap'):
        costmap = _node_params(config, name, nested=True)
        obstacle = costmap['obstacle_layer']

        assert _literal(costmap['footprint']) == EXPECTED_FOOTPRINT, name
        assert 'obstacle_layer' in costmap['plugins'], name
        assert obstacle['plugin'].endswith('ObstacleLayer'), name
        assert obstacle['observation_sources'] == 'scan', name
        assert obstacle['scan']['topic'] == '/scan', name
        assert obstacle['scan']['data_type'] == 'LaserScan', name
        assert costmap['inflation_layer']['inflation_radius'] >= 0.55, name


def test_collision_monitor_is_final_velocity_owner_with_fresh_scan():
    monitor = _node_params(_params(), 'collision_monitor')

    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == 'cmd_vel'
    assert monitor['source_timeout'] <= 0.5
    assert monitor['observation_sources'] == ['scan']
    assert monitor['scan']['type'] == 'scan'
    assert monitor['scan']['topic'] == '/scan'


def test_collision_monitor_has_stop_slowdown_and_two_second_approach():
    monitor = _node_params(_params(), 'collision_monitor')
    actions = {
        monitor[name]['action_type']: monitor[name]
        for name in monitor['polygons']
    }

    assert {'stop', 'slowdown', 'approach'} <= actions.keys()
    assert actions['approach']['time_before_collision'] >= 2.0


def test_mapping_behavior_tree_is_forward_only_but_keeps_safe_recoveries():
    tree = ET.parse(BT_PATH)
    element_names = {element.tag for element in tree.iter()}

    assert 'BackUp' not in element_names
    assert 'Spin' in element_names
    assert 'Wait' in element_names


def test_mapping_launch_uses_navigation_only_with_safe_defaults():
    source = LAUNCH_PATH.read_text(encoding='utf-8')
    syntax = ast.parse(source)

    assert 'bringup_launch.py' not in source, (
        'mapping must not start AMCL/map_server through bringup_launch.py')
    assert 'navigation_launch.py' in source
    assert "executable='frontier_explorer'" in source
    assert "executable='map_saver_server'" in source
    assert "name='lifecycle_manager_map_saver'" in source
    assert 'default_nav_to_pose_bt_xml' in source

    declarations = [
        node for node in ast.walk(syntax)
        if isinstance(node, ast.Call) and _call_name(node) ==
        'DeclareLaunchArgument'
    ]
    sim_time = next(
        call for call in declarations
        if call.args and isinstance(call.args[0], ast.Constant) and
        call.args[0].value == 'use_sim_time')
    defaults = {
        keyword.arg: keyword.value for keyword in sim_time.keywords
    }
    assert ast.literal_eval(defaults['default_value']) == 'false'


def test_include_launch_arguments_never_receive_parameter_file_objects():
    syntax = ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'))
    includes = [
        node for node in ast.walk(syntax)
        if isinstance(node, ast.Call) and
        _call_name(node) == 'IncludeLaunchDescription'
    ]

    assert includes, 'expected a Nav2 IncludeLaunchDescription'
    for include in includes:
        argument = next(
            keyword.value for keyword in include.keywords
            if keyword.arg == 'launch_arguments')
        dictionary = argument.func.value if (
            isinstance(argument, ast.Call) and
            isinstance(argument.func, ast.Attribute) and
            argument.func.attr == 'items') else argument
        assert isinstance(dictionary, ast.Dict)
        for value in dictionary.values:
            assert not (
                isinstance(value, ast.Call) and
                _call_name(value) == 'ParameterFile'), (
                    'Include launch arguments accept substitutions, not '
                    'ParameterFile objects')
            assert not (
                isinstance(value, ast.Name) and
                value.id == 'configured_params'), (
                    'configured_params is a ParameterFile and cannot be '
                    'passed to IncludeLaunchDescription')


def test_docking_velocity_is_remapped_before_navigation_launch():
    source = LAUNCH_PATH.read_text(encoding='utf-8')
    remap = "SetRemap(src='docking_server:cmd_vel', dst='cmd_vel_nav')"

    if 'navigation_launch.py' in source:
        assert remap in source, (
            'Jazzy docking_server must not bypass collision_monitor')
        group_start = source.index('GroupAction(actions=[')
        remap_position = source.index(remap, group_start)
        navigation_position = source.index('navigation,', group_start)
        assert remap_position < navigation_position


def test_explorer_exit_handler_precedes_process_and_shuts_down_launch():
    """Even an immediate explorer crash must stop the whole mapping launch."""
    source = LAUNCH_PATH.read_text(encoding='utf-8')
    assert 'RegisterEventHandler(OnProcessExit(' in source
    assert 'target_action=explorer' in source
    assert 'on_exit=[Shutdown(' in source
    action_list = source[source.index('return LaunchDescription(['):]
    assert action_list.index('explorer_exit_shutdown,') < action_list.index(
        'explorer,')


def test_package_installs_behavior_tree_and_frontier_entrypoint():
    source = SETUP_PATH.read_text(encoding='utf-8')

    assert "glob('behavior_trees/*.xml')" in source
    assert (
        'frontier_explorer = '
        'jdamr_cube_navigation.frontier_explorer:main') in source


def test_frontier_explorer_never_publishes_twist_directly():
    source = EXPLORER_PATH.read_text(encoding='utf-8')
    syntax = ast.parse(source)
    imported_names = {
        alias.name
        for node in ast.walk(syntax)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }

    assert 'Twist' not in imported_names
    for call in ast.walk(syntax):
        if not isinstance(call, ast.Call) or _call_name(call) != \
                'create_publisher':
            continue
        assert not any(
            isinstance(argument, ast.Name) and argument.id == 'Twist'
            for argument in call.args), (
                'frontier_explorer must delegate all motion to Nav2')
