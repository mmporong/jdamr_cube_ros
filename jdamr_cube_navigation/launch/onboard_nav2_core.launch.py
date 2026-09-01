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
            'node_names': [
                'keepout_filter_mask_server',
                'keepout_costmap_filter_info_server',
            ],
        }],
    )

    nav2_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        namespace='',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params, {'yaml_filename': map_yaml}],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': localization_nodes,
                }],
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': navigation_nodes,
                }],
            ),
        ],
    )

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
        RegisterEventHandler(OnProcessExit(
            target_action=nav2_container,
            on_exit=_shutdown_unless_already_stopping(
                'minimal Nav2 container exited; stopping navigation'))),
        keepout_mask_server,
        keepout_info_server,
        keepout_lifecycle,
        nav2_container,
    ])
