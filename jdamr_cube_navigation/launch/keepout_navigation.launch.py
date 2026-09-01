"""Launch physical saved-map navigation with keepout protection required."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Require a valid keepout mask for the physical navigation entrypoint."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    navigation_launch = os.path.join(
        package_share, 'launch', 'navigation.launch.py')
    rviz_config = os.path.join(
        package_share, 'rviz', 'keepout_navigation.rviz')

    map_yaml = LaunchConfiguration('map')
    keepout_mask = LaunchConfiguration('keepout_mask')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'map': map_yaml,
            'keepout_mask': keepout_mask,
            'use_keepout': 'true',
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_composition': use_composition,
        }.items(),
    )
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='keepout_navigation_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908.yaml'),
            description='Protocol map for supervised corridor navigation'),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908_keepout.yaml'),
            description='Required aligned mask; launch fails if invalid'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'autostart', default_value='true'),
        DeclareLaunchArgument(
            'use_composition', default_value='false',
            description='Compose Nav2 only for the Pi onboard wrapper'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start the keepout-aware operator RViz view'),
        navigation,
        rviz,
    ])
