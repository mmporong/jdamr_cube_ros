"""Launch only the Nav2 components required by the onboard corridor run."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.keepout_mask import validate_mask
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
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


def generate_launch_description():
    """Build the headless saved-map corridor navigation process set."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    map_yaml = LaunchConfiguration('map')
    keepout_mask = LaunchConfiguration('keepout_mask')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    corridor_bt = os.path.join(
        package_share, 'behavior_trees',
        'navigate_to_pose_corridor_fail_fast.xml')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'default_nav_to_pose_bt_xml': corridor_bt,
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
    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = [
        'controller_server',
        'planner_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
    ]

    keepout_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': keepout_mask,
            'topic_name': '/keepout_filter_mask',
            'frame_id': 'map',
        }],
    )
    keepout_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_costmap_filter_info_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'type': 0,
            'filter_info_topic': '/keepout_costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )
    keepout_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            # The Pi answers lifecycle services slowly under Nav2 load.  On
            # 2026-09-03 the default 4 s bond timeout reported a healthy
            # keepout server as failed and aborted its bringup.
            'bond_timeout': 10.0,
            'bond_respawn_max_duration': 20.0,
            'node_names': [
                'keepout_filter_mask_server',
                'keepout_costmap_filter_info_server',
            ],
        }],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params, {'yaml_filename': map_yaml}],
        remappings=remappings,
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
    )
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
    )
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'bond_timeout': 10.0,
            'bond_respawn_max_duration': 20.0,
            'node_names': localization_nodes,
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
            'bond_timeout': 10.0,
            'bond_respawn_max_duration': 20.0,
            'node_names': navigation_nodes,
        }],
    )
    required_nav2_processes = [
        map_server,
        amcl,
        controller_server,
        planner_server,
        velocity_smoother,
        collision_monitor,
        bt_navigator,
        localization_lifecycle,
        navigation_lifecycle,
    ]
    required_exit_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=_shutdown_unless_already_stopping(
                'required Nav2 process exited; stopping navigation')))
        for process in required_nav2_processes
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
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
        OpaqueFunction(function=_validate_keepout),
        RegisterEventHandler(OnProcessExit(
            target_action=keepout_mask_server,
            on_exit=_shutdown_unless_already_stopping(
                'keepout mask server exited; stopping navigation'))),
        RegisterEventHandler(OnProcessExit(
            target_action=keepout_info_server,
            on_exit=_shutdown_unless_already_stopping(
                'keepout info server exited; stopping navigation'))),
        *required_exit_handlers,
        keepout_mask_server,
        keepout_info_server,
        keepout_lifecycle,
        *required_nav2_processes,
    ])
