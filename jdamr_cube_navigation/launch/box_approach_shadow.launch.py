"""Start perception and approach preview only; never launch a motion stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Share existing observer settings and keep both nodes non-actuating."""
    share = get_package_share_directory('jdamr_cube_navigation')
    return LaunchDescription([
        Node(package='jdamr_cube_navigation', executable='depth_box_parking',
             name='jdamr_depth_box_parking', output='screen',
             parameters=[os.path.join(share, 'config/depth_box_parking.yaml')]),
        Node(package='jdamr_cube_navigation', executable='box_approach_shadow',
             name='box_approach_shadow', output='screen'),
    ])
