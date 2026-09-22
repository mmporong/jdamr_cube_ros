"""Run operator-driven Cartographer mapping through the protected drive chain."""

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
    """Build a lean manual-mapping stack with one protected command path."""
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
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            cartographer_share, 'launch', 'cartographer_real.launch.py')),
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params, {'use_sim_time': False}],
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
    )
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[configured_params, {'use_sim_time': False}],
        remappings=tf_remaps,
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
            'node_names': [
                'velocity_smoother', 'collision_monitor', 'map_saver'],
            'use_sim_time': False,
        }],
    )
    controller = Node(
        package='jdamr_cube_node',
        executable='web_teleop',
        name='operator_mapping_controller',
        output='screen',
        parameters=[{
            'output_topic': 'cmd_vel_nav',
            'port': controller_port,
            'profile_label': (
                'Cartographer 수동 매핑 · 속도 완화 · 충돌 감시 적용'),
        }],
    )

    required_nodes = (
        velocity_smoother, collision_monitor, map_saver,
        lifecycle_manager, controller)
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
        velocity_smoother,
        collision_monitor,
        map_saver,
        lifecycle_manager,
        controller,
    ])
