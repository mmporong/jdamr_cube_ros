"""Launch saved-map Nav2 localization with an optional keepout mask."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.keepout_mask import validate_mask
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def _enabled(context, argument_name):
    value = LaunchConfiguration(argument_name).perform(context)
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _validate_keepout(context):
    if not _enabled(context, 'use_keepout'):
        return []
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


def generate_launch_description():
    """Build saved-map navigation and fail closed on keepout failure."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    keepout_mask = LaunchConfiguration('keepout_mask')
    params_file = LaunchConfiguration('params_file')
    use_keepout = LaunchConfiguration('use_keepout')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    safe_bt = os.path.join(
        package_share, 'behavior_trees', 'navigate_to_pose_safe_mapping.xml')

    rewritten_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            # waypoint_follower delegates each pose to NavigateToPose without
            # supplying a tree path.  A blank default makes every waypoint
            # fail with BehaviorTreeEngine "Empty Tree" before any movement.
            'default_nav_to_pose_bt_xml': safe_bt,
            'yaml_filename': map_yaml,
            'local_costmap.local_costmap.ros__parameters.'
            'keepout_filter.enabled': use_keepout,
            'global_costmap.global_costmap.ros__parameters.'
            'keepout_filter.enabled': use_keepout,
            'use_sim_time': use_sim_time,
        },
        convert_types=True,
    )

    keepout_mask_server = Node(
        condition=IfCondition(use_keepout),
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
        condition=IfCondition(use_keepout),
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
        condition=IfCondition(use_keepout),
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
        }],
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': rewritten_params,
            'autostart': autostart,
            'use_composition': use_composition,
            'slam': 'False',
            'use_localization': 'True',
        }.items(),
    )

    shutdown_on_mask_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=keepout_mask_server,
            on_exit=[Shutdown(
                reason='keepout mask server exited; stopping navigation')],
        ),
        condition=IfCondition(use_keepout),
    )
    shutdown_on_info_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=keepout_info_server,
            on_exit=[Shutdown(
                reason='keepout info server exited; stopping navigation')],
        ),
        condition=IfCondition(use_keepout),
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable(
        'FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'))
    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/maps/jdamr_cube_room.yaml'),
        description='Saved map YAML used by map_server and AMCL'))
    ld.add_action(DeclareLaunchArgument(
        'keepout_mask', default_value='',
        description='Aligned keepout mask YAML; required when enabled'))
    ld.add_action(DeclareLaunchArgument(
        'use_keepout', default_value='false',
        description='Enable fail-closed keepout servers and costmap filters'))
    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            package_share, 'config', 'nav2_params.yaml'),
        description='JD-AMR Nav2 parameter file'))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock; physical wrapper passes false'))
    ld.add_action(DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Activate localization, navigation, and filter nodes'))
    ld.add_action(DeclareLaunchArgument(
        'use_composition', default_value='false',
        description=(
            'Compose Nav2 on the Pi to bound CPU load; keep false on laptop')))
    ld.add_action(OpaqueFunction(function=_validate_keepout))
    ld.add_action(shutdown_on_mask_exit)
    ld.add_action(shutdown_on_info_exit)
    ld.add_action(keepout_mask_server)
    ld.add_action(keepout_info_server)
    ld.add_action(keepout_lifecycle)
    ld.add_action(GroupAction(actions=[
        SetRemap(src='docking_server:cmd_vel', dst='cmd_vel_nav'),
        bringup,
    ]))
    return ld
