import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_description = get_package_share_directory('jdamr_cube_description')
    urdf_file = os.path.join(pkg_description, 'urdf', 'jdamr_cube.urdf')

    # MoveItConfigsBuilder가 jdamr_cube_moveit_config/config/ 아래의
    # jdamr_cube.srdf, kinematics.yaml, joint_limits.yaml, *_controllers.yaml,
    # *_planning.yaml을 규약대로 찾아서 이 MoveIt 버전(2.12.4)이 기대하는
    # 파라미터 스키마(예: ompl.planning_plugins, ompl.request_adapters 등)로 조립해준다.
    # 로봇 URDF만 jdamr_cube_description 패키지에 있으므로 절대경로로 지정한다.
    moveit_config = (
        MoveItConfigsBuilder('jdamr_cube', package_name='jdamr_cube_moveit_config')
        .robot_description(file_path=urdf_file)
        # pilz/chomp/stomp 파이프라인은 설정하지 않았으므로(pilz_cartesian_limits.yaml 등이 없음)
        # 우리가 config/ompl_planning.yaml로 준비한 ompl 파이프라인만 로드한다.
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(move_group_node)
    return ld
