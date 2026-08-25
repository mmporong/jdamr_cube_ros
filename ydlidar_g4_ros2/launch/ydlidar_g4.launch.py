# Copyright 2026 Lim
# SPDX-License-Identifier: MIT

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('port', default_value='/dev/ydlidar_g4'),
        DeclareLaunchArgument('frame_id', default_value='laser_link'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('frequency', default_value='10.0'),
        DeclareLaunchArgument('sample_rate', default_value='9.0'),
    ]
    node = Node(
        package='ydlidar_g4_ros2',
        executable='ydlidar_g4_node',
        name='ydlidar_g4_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'frequency': LaunchConfiguration('frequency'),
            'sample_rate': LaunchConfiguration('sample_rate'),
        }],
    )
    return LaunchDescription(arguments + [node])
