"""Prepare a dedicated bounded approach chain; startup never arms motion."""

import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.box_approach_shadow import camera_geometry, lower_base_geometry
from jdamr_cube_navigation.new_base_contract import validate_new_base_params
from jdamr_cube_navigation.parking import load_parking_contract
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
import yaml


def validate_configuration(paths):
    """Resolve measured geometry and reuse the existing protection contract."""
    camera_geometry(paths['camera_mount_file'])
    lower_base_geometry(paths['geometry_file'])
    load_parking_contract(Path(paths['parking_contract_file']))
    params = yaml.safe_load(Path(paths['nav_params_file']).read_text())
    geometry = yaml.safe_load(Path(paths['geometry_file']).read_text())
    validate_new_base_params(params, geometry)
    return params


def startup_graph_ready(publishers, base_subscribers, names):
    """Require positive base discovery and reject command/node conflicts."""
    commands = ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel')
    conflicts = {topic: publishers.get(topic) for topic in commands if publishers.get(topic)}
    required_names = {'jdamr_depth_box_parking', 'box_approach_execution',
                      'velocity_smoother', 'collision_monitor'}
    if required_names.intersection(names):
        conflicts['nodes'] = sorted(required_names.intersection(names))
    if conflicts:
        raise RuntimeError(f'existing_motion_stack_must_be_stopped: {conflicts}')
    return (all(publishers.get(topic) for topic in ('/odom', '/scan', '/battery_state'))
            and base_subscribers.count(('jdamr_base_driver', '/')) == 1)


def reject_existing_motion_stack():
    """Do not introduce zero heartbeats into another controller's command chain."""
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('box_approach_startup_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        # Boot-time discovery may precede the base's serial/scan initialization.
        # Proceed as soon as it is stable; this is a deadline, not a fixed delay.
        deadline = time.monotonic() + 15.0
        stable_since = None
        previous = None
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
            topics = ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel',
                      '/odom', '/scan', '/battery_state')
            publishers = {
                topic: sorted((item.node_name, item.node_namespace)
                              for item in node.get_publishers_info_by_topic(topic))
                for topic in topics}
            subscribers = sorted((item.node_name, item.node_namespace)
                                 for item in node.get_subscriptions_info_by_topic('/cmd_vel'))
            names = sorted(node.get_node_names())
            snapshot = publishers, subscribers, names
            ready = startup_graph_ready(*snapshot)
            now = time.monotonic()
            if not ready or snapshot != previous:
                stable_since = now if ready else None
            if stable_since is not None and now - stable_since >= 1.0:
                return
            previous = snapshot
        raise RuntimeError('base_graph_not_discovered_or_unstable_before_startup')
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def launch_chain(context):
    """All validation precedes starting any actuator-path publisher."""
    keys = ('camera_mount_file', 'geometry_file', 'parking_contract_file', 'nav_params_file')
    paths = {key: LaunchConfiguration(key).perform(context) for key in keys}
    validate_configuration(paths)
    reject_existing_motion_stack()
    observer = Node(
        package='jdamr_cube_navigation', executable='depth_box_parking',
        name='jdamr_depth_box_parking', output='screen',
        parameters=[os.path.join(
            get_package_share_directory('jdamr_cube_navigation'),
            'config/depth_box_parking.yaml')])
    executor = Node(
        package='jdamr_cube_navigation', executable='box_approach_execution',
        name='box_approach_execution', output='screen',
        parameters=[paths, {
            'physical_validation_file': ParameterValue(
                LaunchConfiguration('physical_validation_file'), value_type=str),
            'charger_unplugged_confirmed': ParameterValue(
                LaunchConfiguration('charger_unplugged_confirmed'), value_type=bool),
        }])
    smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[paths['nav_params_file'], {'use_sim_time': False}],
        remappings=[('cmd_vel', 'cmd_vel_nav')])
    monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[paths['nav_params_file'], {'use_sim_time': False}])
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_box_approach', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'node_names': ['velocity_smoother', 'collision_monitor']}])
    required = (observer, executor, smoother, monitor, lifecycle)
    return [
        *[RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[Shutdown(reason='required box approach process exited')]))
          for node in required],
        *required,
    ]


def generate_launch_description():
    """Require a separate start service after preparation; never auto-resume."""
    share = get_package_share_directory('jdamr_cube_navigation')
    defaults = {
        'camera_mount_file': os.path.join(
            get_package_share_directory('jdamr_cube_vslam'), 'config/camera_mount.yaml'),
        'geometry_file': os.path.join(
            get_package_share_directory('jdamr_cube_description'),
            'config/new_base_geometry.yaml'),
        'parking_contract_file': os.path.join(share, 'config/parking_contract.yaml'),
        'nav_params_file': os.path.join(share, 'config/new_base_nav2_params.yaml'),
        'physical_validation_file': '',
        'charger_unplugged_confirmed': 'false',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        OpaqueFunction(function=launch_chain),
    ])
