"""Launch conservative Nav2 navigation over a live Cartographer map."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Build navigation-only mapping launch; the explorer starts in IDLE."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    safe_bt = os.path.join(
        package_share, 'behavior_trees', 'navigate_to_pose_safe_mapping.xml')

    rewritten_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': safe_bt,
            'use_sim_time': use_sim_time,
        },
        convert_types=True,
    )
    configured_params = ParameterFile(
        rewritten_params,
        allow_substs=True,
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # Include launch arguments accept substitutions, not ParameterFile.
            'params_file': rewritten_params,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[configured_params],
    )
    map_saver_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[{
            'autostart': autostart,
            'node_names': ['map_saver'],
            'use_sim_time': use_sim_time,
        }],
    )
    explorer = Node(
        package='jdamr_cube_navigation',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    explorer_exit_shutdown = RegisterEventHandler(OnProcessExit(
        target_action=explorer,
        on_exit=[Shutdown(
            reason='frontier explorer exited; stopping autonomous mapping')],
    ))

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share, 'config', 'nav2_params.yaml'),
            description='Absolute path to the autonomous mapping Nav2 params'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock (false on the physical robot)'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Activate Nav2 and map_saver lifecycle nodes'),
        GroupAction(actions=[
            # Jazzy navigation_launch.py starts docking_server without a
            # cmd_vel remap. Keep it behind the smoother/collision pipeline so
            # collision_monitor remains the only final /cmd_vel publisher.
            SetRemap(src='docking_server:cmd_vel', dst='cmd_vel_nav'),
            navigation,
        ]),
        map_saver,
        map_saver_lifecycle,
        explorer_exit_shutdown,
        explorer,
    ])
