"""Static regression tests for the physical-robot mapping safety envelope."""

import ast
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMS_PATH = PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
NEW_BASE_PARAMS_PATH = PACKAGE_ROOT / 'config' / 'new_base_nav2_params.yaml'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'autonomous_mapping.launch.py'
NAVIGATION_LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
URDF_PATH = (
    PACKAGE_ROOT.parent / 'jdamr_cube_description' / 'urdf' /
    'jdamr_cube.urdf')
BT_PATH = (
    PACKAGE_ROOT / 'behavior_trees' / 'navigate_to_pose_safe_mapping.xml')
SETUP_PATH = PACKAGE_ROOT / 'setup.py'
EXPLORER_PATH = (
    PACKAGE_ROOT / 'jdamr_cube_navigation' / 'frontier_explorer.py')


def _urdf_chassis_footprint():
    root = ET.parse(URDF_PATH).getroot()
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    links = {link.attrib['name']: link for link in root.findall('link')}

    def origin(joint_name):
        return tuple(float(value) for value in
                     joints[joint_name].find('origin').attrib['xyz'].split())

    caster_radius = float(
        links['caster_link_front'].find(
            'collision/geometry/sphere').attrib['radius'])
    wheel_width = float(
        links['left_wheel_link'].find(
            'collision/geometry/cylinder').attrib['length'])
    front = origin('caster_front_joint')[0] + caster_radius
    rear = origin('caster_rear_joint')[0] - caster_radius
    left = origin('left_wheel_joint')[1] + wheel_width / 2.0
    right = origin('right_wheel_joint')[1] - wheel_width / 2.0
    return [
        [round(front, 6), round(left, 6)],
        [round(front, 6), round(right, 6)],
        [round(rear, 6), round(right, 6)],
        [round(rear, 6), round(left, 6)],
    ]


EXPECTED_FOOTPRINT = _urdf_chassis_footprint()


def _expanded_footprint(margin):
    front = max(point[0] for point in EXPECTED_FOOTPRINT) + margin
    rear = min(point[0] for point in EXPECTED_FOOTPRINT) - margin
    left = max(point[1] for point in EXPECTED_FOOTPRINT) + margin
    right = min(point[1] for point in EXPECTED_FOOTPRINT) - margin
    return [
        [round(front, 6), round(left, 6)],
        [round(front, 6), round(right, 6)],
        [round(rear, 6), round(right, 6)],
        [round(rear, 6), round(left, 6)],
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


def test_autonomous_mapping_uses_the_proven_sensor_transport_scope():
    spec = importlib.util.spec_from_file_location(
        'autonomous_mapping_launch', LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch = pytest.MonkeyPatch()
    monkeypatch.setattr(
        module, 'get_package_share_directory',
        lambda _name: str(PACKAGE_ROOT))
    try:
        entities = module.generate_launch_description().entities
    finally:
        monkeypatch.undo()
    discovery = next(
        entity for entity in entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == 'discovery_range')
    assert discovery.default_value[0].text == 'SUBNET'
    environment = next(
        entity for entity in entities
        if isinstance(entity, SetEnvironmentVariable)
        and perform_substitutions(LaunchContext(), entity.name)
        == 'ROS_AUTOMATIC_DISCOVERY_RANGE')
    assert entities.index(discovery) < entities.index(environment)
    context = LaunchContext()
    discovery.execute(context)
    environment.execute(context)
    assert context.environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SUBNET'
    context = LaunchContext()
    context.launch_configurations['discovery_range'] = 'LOCALHOST'
    discovery.execute(context)
    environment.execute(context)
    assert context.environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'


def test_autonomous_mapping_defaults_to_new_base_geometry():
    source = LAUNCH_PATH.read_text(encoding='utf-8')

    assert "package_share, 'config', 'new_base_nav2_params.yaml'" in source


def test_controller_uses_forward_only_collision_aware_rpp():
    controller = _node_params(_params(), 'controller_server')['FollowPath']

    assert controller['plugin'].endswith('RegulatedPurePursuitController')
    assert controller['desired_linear_vel'] <= 0.18
    assert controller['allow_reversing'] is False
    assert controller['use_collision_detection'] is True
    assert controller['rotate_to_heading_angular_vel'] <= 0.7


def test_velocity_smoother_disables_reverse_recovery_and_limits_turn_rate():
    config = yaml.safe_load(NEW_BASE_PARAMS_PATH.read_text(encoding='utf-8'))
    smoother = _node_params(config, 'velocity_smoother')

    assert smoother['min_velocity'][0] == 0.0
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
        # 2026-09-03: lowered from 0.55 to 0.30.  The corridor narrows to
        # 1.20~1.50 m, so 0.55 m of inflation on both sides left only
        # 0.1~0.4 m of passable width for a 0.40 m robot and made planning
        # fail at the boundary.  It must still exceed the 0.20 m inscribed
        # radius so obstacles keep a real cost gradient.
        inflation = costmap['inflation_layer']['inflation_radius']
        assert 0.25 <= inflation <= 0.35, name


def test_collision_monitor_is_final_velocity_owner_with_fresh_scan():
    monitor = _node_params(_params(), 'collision_monitor')

    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == 'cmd_vel'
    assert 1.0 <= monitor['source_timeout'] <= 2.0
    assert monitor['transform_tolerance'] >= 0.5
    assert monitor['observation_sources'] == ['scan']
    assert monitor['scan']['type'] == 'scan'
    assert monitor['scan']['topic'] == '/scan'


def test_progress_checker_allows_slowdown_zone_heading_alignment():
    controller = _node_params(_params(), 'controller_server')
    progress = controller['progress_checker']

    assert progress['plugin'] == 'nav2_controller::PoseProgressChecker'
    assert progress['required_movement_radius'] == 0.05
    assert progress['required_movement_angle'] == 0.05
    assert progress['movement_time_allowance'] <= 10.0
    assert controller['FollowPath']['transform_tolerance'] >= 0.5


def test_controller_tolerates_measured_wifi_tf_jitter_without_goal_abort():
    controller = _node_params(_params(), 'controller_server')

    assert controller['controller_frequency'] <= 10.0
    assert controller['costmap_update_timeout'] >= 1.0
    assert controller['failure_tolerance'] >= 2.0


def test_collision_monitor_has_stop_slowdown_and_two_second_approach():
    monitor = _node_params(_params(), 'collision_monitor')
    actions = {
        monitor[name]['action_type']: monitor[name]
        for name in monitor['polygons']
    }

    assert {'stop', 'slowdown', 'approach'} <= actions.keys()
    assert actions['approach']['time_before_collision'] >= 2.0
    assert ast.literal_eval(actions['stop']['points']) == \
        _expanded_footprint(0.05)
    assert ast.literal_eval(actions['slowdown']['points']) == \
        _expanded_footprint(0.15)
    assert actions['slowdown']['slowdown_ratio'] >= 0.60


def test_mapping_behavior_tree_recovery_does_not_move_the_robot():
    tree = ET.parse(BT_PATH)
    element_names = {element.tag for element in tree.iter()}
    selector = tree.find('.//GoalCheckerSelector')
    follow_path = tree.find('.//FollowPath')

    assert tree.getroot().find('.//RecoveryNode').attrib[
        'number_of_retries'] == '1'
    assert tree.find('.//BackUp') is None
    assert tree.findall('.//Spin') == []
    assert 'Wait' in element_names
    assert selector is not None
    assert selector.attrib['default_goal_checker'] == 'general_goal_checker'
    assert follow_path is not None
    assert follow_path.attrib['goal_checker_id'] == '{selected_goal_checker}'


def test_mapping_launch_uses_navigation_only_with_safe_defaults():
    source = LAUNCH_PATH.read_text(encoding='utf-8')
    syntax = ast.parse(source)

    assert 'bringup_launch.py' not in source, (
        'mapping must not start AMCL/map_server through bringup_launch.py')
    for package in (
            'nav2_controller', 'nav2_planner', 'nav2_behaviors',
            'nav2_velocity_smoother', 'nav2_collision_monitor',
            'nav2_bt_navigator'):
        assert f"'{package}'" in source
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


def test_mapping_physical_custom_params_use_new_base_contract(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'autonomous_mapping_launch', LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(
        PACKAGE_ROOT if name == 'jdamr_cube_navigation'
        else PACKAGE_ROOT.parent / 'jdamr_cube_description'))
    context = LaunchContext()
    context.launch_configurations.update({
        'params_file': str(PARAMS_PATH),
        'use_sim_time': 'false',
    })

    with pytest.raises(RuntimeError, match='footprint is smaller'):
        module._validate_physical_params(context)


def test_mapping_simulation_allows_legacy_params(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'autonomous_mapping_launch', LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, 'validate_new_base_params',
        lambda _params, _geometry: pytest.fail('simulation invoked validator'))
    context = LaunchContext()
    context.launch_configurations.update({
        'params_file': '/missing/legacy-sim.yaml',
        'use_sim_time': 'true',
    })

    assert module._validate_physical_params(context) == []


def test_mapping_validation_action_precedes_every_node(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'autonomous_mapping_launch', LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(
        PACKAGE_ROOT if name == 'jdamr_cube_navigation'
        else PACKAGE_ROOT.parent / 'jdamr_cube_description'))

    entities = module.generate_launch_description().entities
    validator_index = next(
        index for index, entity in enumerate(entities)
        if isinstance(entity, OpaqueFunction)
        and entity._OpaqueFunction__function is
        module._validate_physical_params)
    first_node_index = next(
        index for index, entity in enumerate(entities)
        if isinstance(entity, Node))
    assert validator_index < first_node_index


def test_nav2_uses_an_isolated_component_container_for_physical_reliability():
    source = LAUNCH_PATH.read_text(encoding='utf-8')

    assert "executable='component_container_isolated'" in source
    assert "name='nav2_mapping_container'" in source
    assert source.count("'bond_timeout': 0.0") == 2
    assert "executable='nav2_liveness_guard'" in source
    assert "package='nav2_collision_monitor'" in source
    assert "executable='collision_monitor'" in source


def test_collision_monitor_is_not_composed_with_the_planner():
    syntax = ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'))
    composed_plugins = [
        ast.literal_eval(node.args[1])
        for node in ast.walk(syntax)
        if isinstance(node, ast.Call)
        and _call_name(node) == 'ComposableNode'
        and len(node.args) >= 2
        and isinstance(node.args[1], ast.Constant)
    ]
    assert 'nav2_collision_monitor::CollisionMonitor' not in composed_plugins


def test_composed_nodes_receive_the_rewritten_parameter_file():
    syntax = ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'))
    components = [
        node for node in ast.walk(syntax)
        if isinstance(node, ast.Call) and
        _call_name(node) == 'ComposableNode'
    ]

    assert components
    factory = next(
        node for node in ast.walk(syntax)
        if isinstance(node, ast.FunctionDef) and node.name == 'component')
    parameters = next(
        keyword.value for node in ast.walk(factory)
        if isinstance(node, ast.Call) and _call_name(node) == 'ComposableNode'
        for keyword in node.keywords if keyword.arg == 'parameters')
    assert any(
        isinstance(item, ast.Name) and item.id == 'configured_params'
        for item in parameters.elts)


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
