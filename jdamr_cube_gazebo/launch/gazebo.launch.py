import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
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

    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0', description='Initial x position of the robot')

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0', description='Initial y position of the robot')

    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.01', description='Initial z position of the robot')

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v2 ', world]}.items())

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
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    ld.add_action(gazebo_sim)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(bridge_node)
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity_node, on_exit=[load_joint_state_broadcaster])))
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_arm_controller])))
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=load_arm_controller, on_exit=[load_gripper_controller])))
    return ld
