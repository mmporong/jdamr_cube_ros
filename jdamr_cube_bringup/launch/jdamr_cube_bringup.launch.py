import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 라이다 런치 경로 (사용 중인 라이다 패키지에 맞춰 수정)
    # 예: RPLidar A1/A2 기준
    lidar_launch_path = os.path.join(
        get_package_share_directory('ldlidar_sl_ros2'), 'launch', 'ld14.launch.py')

    return LaunchDescription([
        # A. 기존 jdamr_cube_node 패키지의 노드 실행
        Node(
            package='jdamr_cube_node', # 기존 패키지 이름
            executable='jdamr_cube_node', # setup.py의 entry_points에 등록된 이름
            name='jdamr_cube_node',
            output='screen',
            parameters=[{'port': '/dev/ttyAMA0', 'baudrate': 115200}]
        ),

        # B. 라이다 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)
        ),

        # C. 고정 TF 발행 (Robot State Publisher 대체용 단순 TF)
        # base_link에서 각 센서까지의 거리(xyz)와 회전(rpy) 정의
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'base_scan']
        ),
    ])