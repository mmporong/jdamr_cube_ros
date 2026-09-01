"""Open the low-bandwidth keepout operator view without starting Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Keep the laptop as a display/client, not the motion-loop owner."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    rviz_config = os.path.join(
        package_share, 'rviz', 'keepout_navigation.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='keepout_operator_view',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
