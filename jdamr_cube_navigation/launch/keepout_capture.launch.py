"""Show a saved map in RViz and capture multiple keepout polygons."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch only map visualization and polygon capture, never navigation."""
    nav2_share = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    output = LaunchConfiguration('output')
    zone_id = LaunchConfiguration('zone_id')
    point_count = LaunchConfiguration('point_count')
    margin = LaunchConfiguration('margin')
    use_rviz = LaunchConfiguration('use_rviz')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_capture_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': map_yaml,
            'topic_name': '/map',
            'frame_id': 'map',
        }],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout_capture',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['keepout_capture_map_server'],
        }],
    )
    capture = Node(
        package='jdamr_cube_navigation',
        executable='keepout_zone_capture',
        name='keepout_zone_capture',
        output='screen',
        arguments=[
            '--map', map_yaml,
            '--output', output,
            '--zone-id', zone_id,
            '--points', point_count,
            '--margin', margin,
        ],
    )
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='keepout_capture_rviz',
        output='screen',
        arguments=[
            '-d', os.path.join(
                nav2_share, 'rviz', 'nav2_default_view.rviz'),
        ],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908.yaml')),
        DeclareLaunchArgument(
            'output',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908_keepout_zones.yaml')),
        DeclareLaunchArgument('zone_id', default_value='keepout_area'),
        DeclareLaunchArgument('point_count', default_value='4'),
        DeclareLaunchArgument('margin', default_value='0.55'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        RegisterEventHandler(OnProcessExit(
            target_action=map_server,
            on_exit=[Shutdown(reason='capture map server exited')],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=capture,
            on_exit=[Shutdown(reason='keepout polygon capture stopped')],
        )),
        map_server,
        lifecycle_manager,
        capture,
        rviz,
    ])
