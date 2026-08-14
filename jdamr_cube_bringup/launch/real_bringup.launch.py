"""실기 브링업 — robot_state_publisher + C++ 베이스 드라이버 + LD14.

시뮬용 jdamr_cube_bringup.launch.py 와 별개 파일이다 (실기는 use_sim_time
불가·포트·드라이버가 다르다). TF 지오메트리의 단일 출처는 URDF:
강사원본 ld14.launch.py 의 static_transform_publisher(0,0,0.18)와 URDF
laser_joint 가 서로 다른 값으로 이중 발행되던 것(조사기록 E7)을,
라이다 노드를 직접 띄우고 frame_id 를 URDF 링크(laser_link)로 맞춰 없앤다.

바퀴 제원은 실측 후 런치 인자로 넘긴다 (안 넘기면 드라이버가 경고):
  ros2 launch jdamr_cube_bringup real_bringup.launch.py \
      wheel_radius:=0.0XX wheel_separation:=0.3XX

연결 구성 (2026-08-14 실물 확정):
  ESP32 ↔ 파이 = 40핀 헤더 UART(/dev/ttyS0) — USB 케이블 불필요.
  보드의 UART 가 헤더에 배선돼 있고 50Hz 스트림 실측으로 확인했다.
  전제: cmdline.txt 에서 console=serial0 제거 + serial-getty mask +
  udev 규칙(ttyS0 → dialout). USB(ttyUSB0)는 라이다 전용이 된다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('jdamr_cube_description'), 'urdf', 'jdamr_cube.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    base_port = LaunchConfiguration('base_port')
    lidar_port = LaunchConfiguration('lidar_port')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')

    return LaunchDescription([
        DeclareLaunchArgument('base_port', default_value='/dev/ttyS0',
                              description='ESP32 시리얼 — 40핀 헤더 UART (실물 확정)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',
                              description='LD14P 시리얼 — USB 는 라이다 전용 (어댑터 직결)'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0329',
                              description='바퀴 반지름 [m] — 2026-08-14 주행 캘리브레이션 확정(자 실측 지름 65mm 와 일치)'),
        DeclareLaunchArgument('wheel_separation', default_value='0.1836',
                              description='유효 트레드 [m] — 주행 캘리브레이션 확정. 기하 중심거리는 0.20, 접지면 효과로 유효값이 작다'),

        # URDF 가 모든 고정 TF(base_footprint→base_link→laser_link…)의 단일 출처
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # 가동 관절(바퀴·팔) 기본값 0 발행 — 실기엔 관절 상태 소스가 없어
        # RobotModel TF 가 비는 것을 막는다. 팔 구동 시 실제 퍼블리셔로 교체.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # C++ 베이스 드라이버 — cmd_vel↔펌웨어 v2, odom→base_footprint TF 발행
        Node(
            package='jdamr_base_driver',
            executable='base_driver_node',
            name='jdamr_base_driver',
            output='screen',
            parameters=[{
                'port': base_port,
                'wheel_radius': wheel_radius,
                'wheel_separation': wheel_separation,
                'base_frame': 'base_footprint',
                'imu_frame': 'base_link',   # 보드가 base_link 에 장착 — 전용 imu_link 추가 전까지
            }],
        ),

        # LD14 — frame_id 를 URDF 링크로. 정적 TF 는 여기서 절대 만들지 않는다 (E7)
        Node(
            package='ldlidar_sl_ros2',
            executable='ldlidar_sl_ros2_node',
            name='ldlidar_node',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD14P'},   # 실물 확정 — 230400 에서만 프레임 수신
                {'laser_scan_topic_name': 'scan'},
                {'point_cloud_2d_topic_name': 'pointcloud2d'},
                {'frame_id': 'laser_link'},
                {'port_name': lidar_port},
                {'serial_baudrate': 230400},
                {'laser_scan_dir': False},  # 2026-08-14 실측: +90° 회전에 스캔 -98° 이동(거울반전) → 반전
                {'enable_angle_crop_func': False},
            ],
        ),
    ])
