"""Launch opt-in registered-depth obstacle perception and Nav2 integration."""

import math
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory

from jdamr_cube_navigation.depth_navigation_config import (
    DEPTH_FILTER_DEFAULTS,
    load_depth_navigation_params,
)
from jdamr_cube_navigation.new_base_contract import validate_new_base_params

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import yaml


def _true(value):
    return value.lower() in ('1', 'true', 'yes', 'on')


def _camera_mount(context):
    """Load and validate the measured camera-link mounting transform."""
    mount_path = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('camera_mount_file').perform(context))))
    document = yaml.safe_load(mount_path.read_text(encoding='utf-8'))
    try:
        mount = document['camera_mount']
        transform = mount['transform']
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            'camera mount file must define a measured transform') from error
    fields = ('x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad')
    if (mount.get('status') != 'measured'
            or mount.get('parent_frame') != 'base_link'
            or mount.get('child_frame') != 'camera_link'
            or any(type(transform.get(field)) not in (int, float)
                   or not math.isfinite(transform[field])
                   for field in fields)):
        raise RuntimeError(
            'camera mount must be measured finite base_link to camera_link')
    return document, mount, transform


def _validate_camera_gate(context, mode):
    """Require measured, explicitly approved fusion for physical Nav2 modes."""
    if mode == 'observe' or _true(
            LaunchConfiguration('use_sim_time').perform(context)):
        return
    document, _mount, _transform = _camera_mount(context)
    if document.get('usage_gate', {}).get('lidar_rgbd_fusion') != 'allowed':
        raise RuntimeError(
            'physical depth navigation requires measured camera mounting and '
            'usage_gate.lidar_rgbd_fusion=allowed')
    if not _true(LaunchConfiguration(
            'publish_camera_mount').perform(context)):
        raise RuntimeError(
            'physical depth navigation requires publish_camera_mount=true; '
            'stop any other camera mount TF broadcaster first')


def _validate_simulation_isolation(context, mode):
    """Allow simulated fusion only in an explicitly isolated DDS domain."""
    if mode == 'observe' or not _true(
            LaunchConfiguration('use_sim_time').perform(context)):
        return
    raw_domain = os.environ.get('ROS_DOMAIN_ID')
    try:
        domain_id = int(raw_domain)
    except (TypeError, ValueError) as error:
        raise RuntimeError(
            'simulated depth fusion requires an explicit ROS_DOMAIN_ID') \
            from error
    if not 100 <= domain_id <= 232:
        raise RuntimeError(
            'simulated depth fusion requires ROS_DOMAIN_ID in [100, 232]')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError(
            'simulated depth fusion requires LOCALHOST discovery')
    if os.environ.get('ROS_STATIC_PEERS', ''):
        raise RuntimeError(
            'simulated depth fusion requires empty ROS_STATIC_PEERS')
    if LaunchConfiguration('discovery_range').perform(context) != 'LOCALHOST':
        raise RuntimeError(
            'simulated depth fusion requires LOCALHOST discovery')


def _camera_mount_actions(context):
    """Optionally publish the measured mounting TF as a required process."""
    if not _true(LaunchConfiguration(
            'publish_camera_mount').perform(context)):
        return [LogInfo(msg=(
            'WARNING: camera mount TF is not published by this launch; '
            'an external base_link to camera_link transform is required'))]
    _document, mount, transform = _camera_mount(context)
    arguments = []
    for option, field in (
            ('--x', 'x_m'), ('--y', 'y_m'), ('--z', 'z_m'),
            ('--roll', 'roll_rad'), ('--pitch', 'pitch_rad'),
            ('--yaw', 'yaw_rad')):
        arguments.extend((option, str(transform[field])))
    arguments.extend((
        '--frame-id', mount['parent_frame'],
        '--child-frame-id', mount['child_frame']))
    publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_static_transform',
        output='screen',
        arguments=arguments,
    )
    required_exit = RegisterEventHandler(OnProcessExit(
        target_action=publisher,
        on_exit=[Shutdown(
            reason='camera mount TF publisher exited; stopping depth mode')],
    ))
    return [required_exit, publisher]


def _validate_depth_filter_config(context, mode):
    """Pin fused modes to the filter contract consumed by the Nav2 overlay."""
    if mode == 'observe':
        return
    config_path = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('depth_config').perform(context))))
    document = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    try:
        parameters = document['depth_obstacle_filter']['ros__parameters']
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            'fused depth configuration must define depth_obstacle_filter') \
            from error
    if parameters != DEPTH_FILTER_DEFAULTS:
        raise RuntimeError(
            'fused depth configuration differs from the approved contract')


def _write_nav2_overlay(context):
    base_path = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('params_file').perform(context))))
    params = load_depth_navigation_params(base_path)
    geometry_path = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('geometry_file').perform(context))))
    geometry = yaml.safe_load(geometry_path.read_text(encoding='utf-8'))
    validate_new_base_params(params, geometry)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix='jdamr_depth_nav2_', suffix='.yaml')
    with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
    return temporary_path


def _cleanup_file(path):
    def cleanup(_event, _context):
        try:
            Path(path).unlink()
        except FileNotFoundError:
            pass
        return []

    return cleanup


def _launch_selected_mode(context):
    mode = LaunchConfiguration('mode').perform(context)
    _validate_camera_gate(context, mode)
    _validate_simulation_isolation(context, mode)
    _validate_depth_filter_config(context, mode)
    package_share = Path(get_package_share_directory(
        'jdamr_cube_navigation'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    filter_node = Node(
        package='jdamr_cube_navigation',
        executable='depth_obstacle_filter',
        name='depth_obstacle_filter',
        output='screen',
        parameters=[
            LaunchConfiguration('depth_config'),
            {
                'geometry_file': LaunchConfiguration('geometry_file'),
                'use_sim_time': use_sim_time,
            },
        ],
    )
    filter_exit = RegisterEventHandler(OnProcessExit(
        target_action=filter_node,
        on_exit=[Shutdown(
            reason='depth obstacle filter exited; stopping depth navigation')],
    ))
    actions = [filter_exit, *_camera_mount_actions(context), filter_node]
    if mode == 'observe':
        return actions

    overlay_path = _write_nav2_overlay(context)
    cleanup = RegisterEventHandler(OnShutdown(
        on_shutdown=_cleanup_file(overlay_path)))
    if mode == 'navigation':
        included_launch = (
            package_share / 'launch' / 'onboard_nav2_core.launch.py')
        arguments = {
            'map': LaunchConfiguration('map'),
            'keepout_mask': LaunchConfiguration('keepout_mask'),
            'params_file': overlay_path,
            'use_sim_time': use_sim_time,
            'autostart': LaunchConfiguration('autostart'),
            'navigation_profile': 'new_base_candidate',
            'discovery_range': LaunchConfiguration('discovery_range'),
        }
    else:
        included_launch = (
            package_share / 'launch' / 'autonomous_mapping.launch.py')
        arguments = {
            'params_file': overlay_path,
            'use_sim_time': use_sim_time,
            'autostart': LaunchConfiguration('autostart'),
            'discovery_range': LaunchConfiguration('discovery_range'),
        }
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(included_launch)),
        launch_arguments=arguments.items(),
    )
    return [cleanup, *actions, nav2]


def generate_launch_description():
    """Expose perception-only, saved-map navigation, and mapping modes."""
    navigation_share = Path(get_package_share_directory(
        'jdamr_cube_navigation'))
    description_share = Path(get_package_share_directory(
        'jdamr_cube_description'))
    vslam_share = Path(get_package_share_directory('jdamr_cube_vslam'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='observe',
            choices=['observe', 'navigation', 'mapping']),
        DeclareLaunchArgument(
            'depth_config', default_value=str(
                navigation_share / 'config' / 'depth_obstacles.yaml')),
        DeclareLaunchArgument(
            'params_file', default_value=str(
                navigation_share / 'config' / 'new_base_nav2_params.yaml')),
        DeclareLaunchArgument(
            'geometry_file', default_value=str(
                description_share / 'config' / 'new_base_geometry.yaml')),
        DeclareLaunchArgument(
            'camera_mount_file', default_value=str(
                vslam_share / 'config' / 'camera_mount.yaml')),
        DeclareLaunchArgument(
            'publish_camera_mount', default_value='false',
            choices=['true', 'false'],
            description=(
                'Publish measured base_link to camera_link TF; false requires '
                'an existing authoritative broadcaster')),
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser(
                '~/maps/new_base_navigation.yaml')),
        DeclareLaunchArgument(
            'keepout_mask', default_value=os.path.expanduser(
                '~/maps/new_base_navigation_keepout.yaml')),
        DeclareLaunchArgument(
            'discovery_range', default_value='LOCALHOST',
            choices=['LOCALHOST', 'SUBNET']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=_launch_selected_mode),
    ])
