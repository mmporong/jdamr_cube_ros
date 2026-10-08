"""
실기 SLAM — 파이에서 헤드리스로 돌린다.

전제: real_bringup.launch.py 가 이미 떠서 /scan·/odom·TF 가 살아 있다.
시뮬용 cartographer.launch.py 와 분리한 이유: use_sim_time 과 설정 lua 가
다르고, 파이4 에는 RViz 를 띄우지 않는다.

시각 검토:
  기록 종료 뒤 bag을 노트북의 격리 domain에서 재생한다.
지도 저장:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/room_$(date +%m%d)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """실기 Cartographer와 occupancy grid 노드를 구성한다."""
    config_dir = os.path.join(
        get_package_share_directory('jdamr_cube_cartographer'), 'config')

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-configuration_directory', config_dir,
                       '-configuration_basename', 'jdamr_cube_2d_real.lua'],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        ),
    ])
