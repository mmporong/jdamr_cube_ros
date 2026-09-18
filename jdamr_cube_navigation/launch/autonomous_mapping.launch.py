"""Launch conservative Nav2 navigation over a live Cartographer map."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.new_base_contract import validate_new_base_params

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile

from nav2_common.launch import RewrittenYaml
import yaml


def _validate_physical_params(context):
    """Reject custom physical parameters that bypass the new-base contract."""
    if LaunchConfiguration('use_sim_time').perform(context).lower() in (
            '1', 'true', 'yes', 'on'):
        return []
    params_path = Path(os.path.expanduser(
        LaunchConfiguration('params_file').perform(context)))
    params = yaml.safe_load(params_path.read_text(encoding='utf-8'))
    geometry_path = (
        Path(get_package_share_directory('jdamr_cube_description'))
        / 'config' / 'new_base_geometry.yaml')
    geometry = yaml.safe_load(geometry_path.read_text(encoding='utf-8'))
    validate_new_base_params(params, geometry)
    return []


def generate_launch_description():
    """Build navigation-only mapping launch; the explorer starts in IDLE."""
    package_share = get_package_share_directory('jdamr_cube_navigation')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    safe_bt = os.path.join(
        package_share, 'behavior_trees', 'navigate_to_pose_safe_mapping.xml')

    rewritten_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': safe_bt,
            'use_sim_time': use_sim_time,
        },
        convert_types=True,
    )
    configured_params = ParameterFile(
        rewritten_params,
        allow_substs=True,
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def component(package, plugin, name, *, cmd_vel=False):
        component_remappings = list(remappings)
        if cmd_vel:
            component_remappings.append(('cmd_vel', 'cmd_vel_nav'))
        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=component_remappings,
        )

    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
    ]
    navigation_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_mapping_container',
        namespace='',
        # Costmaps are child nodes of controller/planner. Supplying the
        # rewritten file at process level keeps their footprint and obstacle
        # layers identical to the parent components.
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
        composable_node_descriptions=[
            component(
                'nav2_controller', 'nav2_controller::ControllerServer',
                'controller_server', cmd_vel=True),
            component(
                'nav2_planner', 'nav2_planner::PlannerServer',
                'planner_server'),
            component(
                'nav2_behaviors', 'behavior_server::BehaviorServer',
                'behavior_server', cmd_vel=True),
            component(
                'nav2_velocity_smoother',
                'nav2_velocity_smoother::VelocitySmoother',
                'velocity_smoother', cmd_vel=True),
            component(
                'nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator',
                'bt_navigator'),
        ],
        output='screen',
    )
    # Keep scan processing out of the planner/controller process so planner
    # load cannot delay the monitor callback and cause an invalid-source stop.
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )
    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': autostart,
            'node_names': navigation_nodes,
            'use_sim_time': use_sim_time,
            # The separate graph guard owns liveness. Nav2 bond heartbeats
            # added DDS traffic and falsely reset healthy components on this
            # Pi; zero disables only the bond timer, not lifecycle control.
            'bond_timeout': 0.0,
        }],
    )
    liveness_guard = Node(
        package='jdamr_cube_navigation',
        executable='nav2_liveness_guard',
        name='nav2_mapping_liveness_guard',
        output='screen',
        arguments=['--required', ','.join(navigation_nodes)],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[configured_params],
    )
    map_saver_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[{
            'autostart': autostart,
            'node_names': ['map_saver'],
            'use_sim_time': use_sim_time,
            'bond_timeout': 0.0,
        }],
    )
    explorer = Node(
        package='jdamr_cube_navigation',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    explorer_exit_shutdown = RegisterEventHandler(OnProcessExit(
        target_action=explorer,
        on_exit=[Shutdown(
            reason='frontier explorer exited; stopping autonomous mapping')],
    ))
    required_exit_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[Shutdown(reason=(
                'required autonomous-mapping process exited'))],
        ))
        for process in (
            navigation_container, collision_monitor,
            navigation_lifecycle, liveness_guard)
    ]

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share, 'config', 'new_base_nav2_params.yaml'),
            description='Absolute path to the autonomous mapping Nav2 params'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock (false on the physical robot)'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Activate Nav2 and map_saver lifecycle nodes'),
        OpaqueFunction(function=_validate_physical_params),
        *required_exit_handlers,
        navigation_container,
        collision_monitor,
        navigation_lifecycle,
        liveness_guard,
        map_saver,
        map_saver_lifecycle,
        explorer_exit_shutdown,
        explorer,
    ])
