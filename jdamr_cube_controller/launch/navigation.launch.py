import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_controller_dir = get_package_share_directory('jdamr_cube_controller')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map = os.path.expanduser('~/maps/jdamr_cube_room.yaml')
    default_params_file = os.path.join(pkg_controller_dir, 'config', 'nav2_params.yaml')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='scripts/wsl_save_map.sh (또는 save_map.bat)로 저장한 지도(.yaml) 경로')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='jdamr_cube 로봇 사양(footprint, 속도 제한 등)에 맞춘 nav2 파라미터 파일')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Nav2 lifecycle 노드를 자동으로 activate할지 여부')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items())

    ld = LaunchDescription()
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(bringup_cmd)
    return ld
