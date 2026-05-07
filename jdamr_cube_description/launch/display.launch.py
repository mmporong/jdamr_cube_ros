import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 및 파일 경로 설정 [cite: 22, 36]
    pkg_name = 'jdamr_cube_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # URDF 파일 경로 (urdf 폴더 내 jdamr_cube.urdf 파일 가정) 
    urdf_file = os.path.join(pkg_share, 'urdf', 'jdamr_cube.urdf')
    
    # 2. URDF 파일 내용 읽기 (Xacro 미사용 시 직접 로드)
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # 3. 노드 구성
    # 로봇의 TF 정보를 발행하는 노드 [cite: 9, 28]
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 관절 움직임을 테스트할 수 있는 GUI 노드 [cite: 46]
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # RViz2 실행 노드 [cite: 10, 47]
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])