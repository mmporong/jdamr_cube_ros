"""Odom + gyro EKF (robot_localization). Not part of any bringup yet.

Run only with the base driver started with publish_tf:=false, so that odom->base_footprint
comes from the EKF alone. The board's z axis points down (gyro z anti-correlated with the
wheel yaw rate, accel z -9.4 at rest), so imu_link is base_link turned 180 deg about x;
only the z axis is used.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('jdamr_cube_navigation'),
                          'config', 'ekf_odom_imu.yaml')
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='imu_link_static_tf',
             arguments=['--roll', '3.141592653589793', '--frame-id', 'base_link',
                        '--child-frame-id', 'imu_link'],
             parameters=[sim]),
        Node(package='jdamr_cube_navigation', executable='imu_bias_relay',
             name='imu_bias_relay', output='screen', parameters=[sim]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen', parameters=[config, sim]),
    ])
