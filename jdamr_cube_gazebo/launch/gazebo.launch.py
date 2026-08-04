import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_dir = get_package_share_directory('jdamr_cube_gazebo')
    pkg_ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    pkg_description_dir = get_package_share_directory('jdamr_cube_description')

    urdf_file = os.path.join(pkg_description_dir, 'urdf', 'jdamr_cube.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # gz_ros2_control-system 플러그인은 <parameters> 안의 package:// URI를 스스로 해석하지
    # 못하고 그대로 --params-file 인자로 넘겨 gz-sim이 죽는다. 실제 설치 경로로 치환한다.
    so101_controllers_yaml = os.path.join(pkg_description_dir, 'config', 'so101_controllers.yaml')
    robot_description_content = robot_description_content.replace(
        'package://jdamr_cube_description/config/so101_controllers.yaml',
        so101_controllers_yaml)

    bridge_config_file = os.path.join(pkg_gazebo_dir, 'params', 'bridge.yaml')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gazebo_dir, 'worlds', 'empty.world'),
        description='Full path to the world file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='false면 Gazebo GUI 없이 서버만 실행 (시각화는 RViz2 사용)')

    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0', description='Initial x position of the robot')

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0', description='Initial y position of the robot')

    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.01', description='Initial z position of the robot')

    # URDF의 package:// 메쉬 URI는 스폰 시 model://로 변환되는데, gz 서버가 이를
    # 해석하려면 GZ_SIM_RESOURCE_PATH에 share 디렉터리가 있어야 한다. 없으면 SO-101
    # 팔의 STL 메쉬(비주얼+충돌)가 조용히 빠져 그리퍼가 물체를 통과한다(충돌 없음).
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_description_dir))

    # (네이티브 리눅스) WSLg용 d3d12 강제는 제거하고, 헤드리스 EGL이 NVIDIA를
    # 쓰도록 벤더를 지정한다. 이게 없으면 EGL이 NVIDIA PCI 장치(10de:2dd8)를
    # 고른 뒤 Mesa 드라이버로 열려다 실패하고("egl: failed to create dri2 screen")
    # llvmpipe(CPU)로 떨어진다. 그러면 카메라 프레임이 아예 발행되지 않는다.
    # GUI 모드는 GLX를 쓰므로 걸지 않는다 — 그쪽은 실행 시 오프로드
    # (__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia)로 처리한다.
    set_egl_vendor = SetEnvironmentVariable(
        '__EGL_VENDOR_LIBRARY_FILENAMES',
        '/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
        condition=UnlessCondition(gui))
    set_prime_offload = SetEnvironmentVariable(
        '__NV_PRIME_RENDER_OFFLOAD', '1', condition=UnlessCondition(gui))

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v2 ', world]}.items(),
        condition=IfCondition(gui))

    gazebo_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v2 ', world]}.items(),
        condition=UnlessCondition(gui))

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }])

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'jdamr_cube',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen')

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_file}'],
        output='screen')

    wrist_camera_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['wrist_camera/image_raw'],
        output='screen')

    rgbd_camera_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['rgbd_camera/image', 'rgbd_camera/depth_image'],
        output='screen')

    # LeRobot 시연 모사 관측 카메라 (room.world의 demo_cam_up/side)
    demo_camera_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['demo_up/image_raw', 'demo_side/image_raw'],
        output='screen')

    # so101 팔은 gz_ros2_control(URDF의 <ros2_control>/<gazebo><plugin gz_ros2_control-system>)로
    # 노출되는데, 스폰 전에는 controller_manager 서비스가 없어 스포너가 실패한다.
    # spawn_entity_node가 끝난 뒤 joint_state_broadcaster -> arm_controller -> gripper_controller
    # 순서로 로드/activate 한다.
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen')

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen')

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    ld.add_action(set_resource_path)
    ld.add_action(set_egl_vendor)
    ld.add_action(set_prime_offload)
    ld.add_action(gazebo_sim)
    ld.add_action(gazebo_sim_headless)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(bridge_node)
    ld.add_action(wrist_camera_bridge_node)
    ld.add_action(rgbd_camera_bridge_node)
    ld.add_action(demo_camera_bridge_node)
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity_node, on_exit=[load_joint_state_broadcaster])))
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_arm_controller])))
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=load_arm_controller, on_exit=[load_gripper_controller])))
    return ld
