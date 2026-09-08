"""Launch the selected onboard navigation profile in a composed container."""

# Run the corridor Nav2 subset in a composed container and keep graph-loss
# detection separate.  Composition correlated with lower load in earlier
# runs, but it does not prove intra-process delivery: rclcpp defaults that
# option to false and this launch does not override it.  Run-specific evidence
# and remaining causal uncertainty live in evaluation/20260904_HANDOFF.md.

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.keepout_mask import validate_mask
from jdamr_cube_navigation.mobile_manipulator_protection import (
    load_mobile_manipulator_protection,
)
from jdamr_cube_navigation.nav2_liveness_guard import DEFAULT_REQUIRED
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def _validate_keepout(context):
    """Reject missing or misaligned physical map inputs before Nav2 starts."""
    map_path = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('map').perform(context))))
    mask_path = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('keepout_mask').perform(context))))
    if not map_path.is_file():
        raise RuntimeError(f'saved map YAML not found: {map_path}')
    if not mask_path.is_file():
        raise RuntimeError(f'keepout mask YAML not found: {mask_path}')
    validate_mask(mask_path, map_path)
    return []


def _shutdown_unless_already_stopping(reason):
    """Fail closed on a required process exit without duplicate shutdowns."""
    def handler(_event, context):
        if context.is_shutdown:
            return []
        return [Shutdown(reason=reason)]

    return handler


def _launch_navigation(context):
    """Resolve the profile before selecting its BT and required components."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    profile = LaunchConfiguration('navigation_profile').perform(context)
    behavior_tree = {
        'corridor': 'navigate_to_pose_corridor_fail_fast.xml',
        'obstacle_candidate': 'navigate_to_pose_dynamic_obstacle_eval.xml',
    }[profile]
    map_yaml = LaunchConfiguration('map')
    keepout_mask = LaunchConfiguration('keepout_mask')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    selected_bt = os.path.join(package_share, 'behavior_trees', behavior_tree)
    protection = None
    if profile == 'obstacle_candidate':
        protection = load_mobile_manipulator_protection(Path(
            package_share) / 'config' / 'mobile_manipulator_protection.yaml')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'default_nav_to_pose_bt_xml': selected_bt,
                'yaml_filename': map_yaml,
                ('local_costmap.local_costmap.ros__parameters.'
                 'keepout_filter.enabled'): 'true',
                ('global_costmap.global_costmap.ros__parameters.'
                 'keepout_filter.enabled'): 'true',
                'use_sim_time': use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_bond = {
        # Allow lifecycle service and bond handling to tolerate scheduler
        # jitter.  The executable threshold is covered by launch tests.
        'bond_timeout': 10.0,
        'bond_respawn_max_duration': 20.0,
    }

    def composable(package, plugin, name, parameters, remap=True):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[*parameters, {'use_sim_time': use_sim_time}],
            remappings=remappings if remap else [],
        )

    keepout_components = [
        composable('nav2_map_server', 'nav2_map_server::MapServer',
                   'keepout_filter_mask_server', [{
                       'use_sim_time': use_sim_time,
                       'yaml_filename': keepout_mask,
                       'topic_name': '/keepout_filter_mask',
                       'frame_id': 'map',
                   }], remap=False),
        composable('nav2_map_server',
                   'nav2_map_server::CostmapFilterInfoServer',
                   'keepout_costmap_filter_info_server', [{
                       'use_sim_time': use_sim_time,
                       'type': 0,
                       'filter_info_topic': '/keepout_costmap_filter_info',
                       'mask_topic': '/keepout_filter_mask',
                       'base': 0.0,
                       'multiplier': 1.0,
                   }], remap=False),
    ]
    collision_monitor_parameters = [configured_params]
    if protection is not None:
        collision_monitor_parameters.append(
            protection['collision_monitor_overrides'])
    nav2_components = [
        composable('nav2_map_server', 'nav2_map_server::MapServer',
                   'map_server', [configured_params,
                                  {'yaml_filename': map_yaml}]),
        composable('nav2_amcl', 'nav2_amcl::AmclNode', 'amcl',
                   [configured_params]),
        ComposableNode(
            package='nav2_controller',
            plugin='nav2_controller::ControllerServer',
            name='controller_server',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        composable('nav2_planner', 'nav2_planner::PlannerServer',
                   'planner_server', [configured_params]),
        ComposableNode(
            package='nav2_velocity_smoother',
            plugin='nav2_velocity_smoother::VelocitySmoother',
            name='velocity_smoother',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        composable('nav2_collision_monitor',
                   'nav2_collision_monitor::CollisionMonitor',
                   'collision_monitor', collision_monitor_parameters),
        composable('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator',
                   'bt_navigator', [configured_params]),
    ]
    navigation_nodes = [
        'controller_server', 'planner_server', 'velocity_smoother',
        'collision_monitor', 'bt_navigator',
    ]
    required_nodes = list(DEFAULT_REQUIRED)
    if profile == 'obstacle_candidate':
        # The candidate BT calls Wait during bounded recovery.  Load only
        # that plugin; selecting this profile must not enable spin or backup.
        nav2_components.insert(-1, ComposableNode(
            package='nav2_behaviors',
            plugin='behavior_server::BehaviorServer',
            name='behavior_server',
            parameters=[configured_params, {
                'behavior_plugins': ['wait'], 'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ))
        navigation_nodes.insert(-1, 'behavior_server')
        required_nodes.append('behavior_server')

    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        namespace='',
        # Costmaps are child nodes created inside planner/controller, not
        # LoadNode targets.  They inherit process-level ROS arguments, so
        # component parameters alone silently leave them on Nav2 defaults.
        # RewrittenYaml only substitutes existing leaf keys.  The production
        # YAML omits use_sim_time, so pass it explicitly to components and
        # their child nodes instead of relying on that rewrite alone.
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
        composable_node_descriptions=keepout_components + nav2_components,
        output='screen',
    )

    keepout_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'keepout_filter_mask_server',
                'keepout_costmap_filter_info_server',
            ],
            **lifecycle_bond,
        }],
    )
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
            **lifecycle_bond,
        }],
    )
    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': navigation_nodes,
            **lifecycle_bond,
        }],
    )

    # The container process surviving is not evidence that the nodes inside it
    # are alive, so the graph-level guard keeps that failure observable while
    # Nav2 remains composed.
    liveness_guard = Node(
        package='jdamr_cube_navigation',
        executable='nav2_liveness_guard',
        name='nav2_liveness_guard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--required', ','.join(required_nodes)],
    )

    required_processes = [
        container,
        keepout_lifecycle,
        localization_lifecycle,
        navigation_lifecycle,
        liveness_guard,
    ]
    required_exit_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=_shutdown_unless_already_stopping(
                'required Nav2 process exited; stopping navigation')))
        for process in required_processes
    ]

    return [*required_exit_handlers, *required_processes]


def generate_launch_description():
    """Keep the proven corridor profile as the default onboard launch."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908.yaml')),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908_keepout_multi.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'navigation_profile', default_value='corridor',
            choices=['corridor', 'obstacle_candidate'],
            description='Candidate enables online replanning and Wait recovery'),
        OpaqueFunction(function=_validate_keepout),
        OpaqueFunction(function=_launch_navigation),
    ])
