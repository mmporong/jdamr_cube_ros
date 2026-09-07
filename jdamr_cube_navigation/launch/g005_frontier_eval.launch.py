"""Launch the evaluation-only Nav2 graph for G005 frontier runs."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Build navigation without AMCL, SLAM, map server, or production edits."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    safe_bt = os.path.join(
        package_share, 'behavior_trees', 'navigate_to_pose_safe_mapping.xml')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'default_nav_to_pose_bt_xml': safe_bt,
                'use_sim_time': use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
    ]

    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[configured_params],
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')])
    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[configured_params], remappings=tf_remaps)
    behaviors = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[configured_params],
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')])
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[configured_params],
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')])
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[configured_params], remappings=tf_remaps)
    navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[configured_params], remappings=tf_remaps)
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
            'bond_timeout': 10.0,
        }])
    localization = Node(
        package='jdamr_cube_navigation',
        executable='g005_ground_truth_localization',
        name='ground_truth_localization', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'ground_truth_frame': 'g005_frontier',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }])

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        DeclareLaunchArgument(
            'params_file',
            description='Generated and hash-sealed G005 Nav2 params'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        localization,
        controller,
        planner,
        behaviors,
        velocity_smoother,
        collision_monitor,
        navigator,
        lifecycle,
    ])
