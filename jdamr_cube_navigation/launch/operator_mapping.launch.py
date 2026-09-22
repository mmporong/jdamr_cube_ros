"""Run operator-driven Cartographer mapping with unrestricted manual drive."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from jdamr_cube_navigation.new_base_contract import validate_new_base_params

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.actions import SetEnvironmentVariable, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml

import yaml


def _validate_physical_params(context):
    params_path = Path(os.path.expanduser(
        LaunchConfiguration('params_file').perform(context)))
    params = yaml.safe_load(params_path.read_text(encoding='utf-8'))
    geometry_path = (
        Path(get_package_share_directory('jdamr_cube_description'))
        / 'config' / 'new_base_geometry.yaml')
    geometry = yaml.safe_load(geometry_path.read_text(encoding='utf-8'))
    validate_new_base_params(params, geometry)
    return []


def generate_launch_description():
    """Build a lean manual-mapping stack without autonomous drive gates."""
    navigation_share = get_package_share_directory('jdamr_cube_navigation')
    cartographer_share = get_package_share_directory(
        'jdamr_cube_cartographer')

    params_file = LaunchConfiguration('params_file')
    controller_port = LaunchConfiguration('controller_port')
    rewritten_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': 'false'},
        convert_types=True,
    )
    configured_params = ParameterFile(rewritten_params, allow_substs=True)
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            cartographer_share, 'launch', 'cartographer_real.launch.py')),
    )
    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[configured_params, {'use_sim_time': False}],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_operator_mapping',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['map_saver'],
            'use_sim_time': False,
        }],
    )
    controller = Node(
        package='jdamr_cube_node',
        executable='web_teleop',
        name='operator_mapping_controller',
        output='screen',
        parameters=[{
            'output_topic': 'cmd_vel',
            'port': controller_port,
            'profile_label': 'Cartographer 수동 매핑 · 주행 차단 없음',
            'require_preflight': False,
        }],
    )

    required_nodes = (map_saver, lifecycle_manager, controller)
    shutdown_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[Shutdown(reason=(
                'required operator-mapping process exited'))],
        ))
        for node in required_nodes
    ]

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                navigation_share, 'config', 'new_base_nav2_params.yaml')),
        DeclareLaunchArgument('controller_port', default_value='8080'),
        OpaqueFunction(function=_validate_physical_params),
        *shutdown_handlers,
        cartographer,
        map_saver,
        lifecycle_manager,
        controller,
    ])
