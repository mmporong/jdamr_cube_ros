"""실기 SLAM — 파이에서 헤드리스로 돌린다 (RViz 는 노트북에서 별도 실행).

전제: real_bringup.launch.py 가 이미 떠서 /scan·/odom·TF 가 살아 있다.
시뮬용 cartographer.launch.py 와 분리한 이유: use_sim_time 과 설정 lua 가
다르고, 파이4 에는 RViz 를 띄우지 않는다.

노트북에서 보기:
  rviz2  (Fixed Frame=map, LaserScan·Map·TF 추가. DOMAIN_ID 12 로 맞출 것)
지도 저장:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/room_$(date +%m%d)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('jdamr_cube_cartographer'), 'config')

    return LaunchDescription([
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
