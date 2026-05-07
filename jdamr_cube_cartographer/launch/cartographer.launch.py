import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('jdamr_cube_cartographer')
    
    # 설정 파일 경로 [cite: 58]
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    configuration_basename = 'jdamr_cube_2d.lua'
    # 2. RViz2 설정 파일 경로 (rviz 폴더 안에 jdamr_cube_cartographer.rviz 파일이 있다고 가정) 
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'jdamr_cube_cartographer.rviz')

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}], # 시뮬레이션 시 True [cite: 7]
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            remappings=[('scan', '/scan')] # 토픽 이름 확인 필요
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),

        # RViz2 노드 추가 [cite: 23, 84]
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir], # 설정된 rviz 파일 로드 
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])