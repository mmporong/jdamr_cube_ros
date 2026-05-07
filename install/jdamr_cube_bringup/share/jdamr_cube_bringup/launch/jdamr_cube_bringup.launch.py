import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 경로 설정
    # jdamr_cube_description 패키지에서 URDF 파일 위치만 가져옵니다.
    pkg_description_dir = get_package_share_directory('jdamr_cube_description')
    urdf_file = os.path.join(pkg_description_dir, 'urdf', 'jdamr_cube.urdf')

    # 라이다 런치 경로
    lidar_launch_path = os.path.join(
        get_package_share_directory('ldlidar_sl_ros2'), 'launch', 'ld14.launch.py')

    # 2. 실행 인자 및 설정
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # URDF 파일을 읽어 문자열로 변환 (robot_state_publisher 파라미터용)
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # A. Robot State Publisher 직접 실행
        # 외부 런치를 Include하지 않고, 여기서 직접 URDF를 로드하여 TF를 발행합니다.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': use_sim_time
            }]
        ),

        # B. 하드웨어 제어 노드 (모터 및 오도메트리)
        Node(
            package='jdamr_cube_node',
            executable='jdamr_cube_node',
            name='jdamr_cube_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyAMA0', 
                'baudrate': 115200,
                'use_sim_time': use_sim_time
            }]
        ),

        # C. 라이다 센서 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)
        ),
    ])