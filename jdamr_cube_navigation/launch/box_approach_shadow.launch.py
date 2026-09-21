"""Start perception and approach preview only; never launch a motion stack."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.box_approach_shadow import camera_geometry, lower_base_geometry
from jdamr_cube_navigation.parking import load_parking_contract
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_inputs(context):
    """Validate every geometry input before either observation process starts."""
    camera_geometry(LaunchConfiguration('camera_mount_file').perform(context))
    lower_base_geometry(LaunchConfiguration('geometry_file').perform(context))
    load_parking_contract(Path(LaunchConfiguration('parking_contract_file').perform(context)))
    return []


def generate_launch_description():
    """Share existing observer settings and keep both nodes non-actuating."""
    share = get_package_share_directory('jdamr_cube_navigation')
    vslam = get_package_share_directory('jdamr_cube_vslam')
    description = get_package_share_directory('jdamr_cube_description')
    paths = {
        'camera_mount_file': os.path.join(vslam, 'config/camera_mount.yaml'),
        'geometry_file': os.path.join(description, 'config/new_base_geometry.yaml'),
        'parking_contract_file': os.path.join(share, 'config/parking_contract.yaml'),
    }
    observer = Node(
        package='jdamr_cube_navigation', executable='depth_box_parking',
        name='jdamr_depth_box_parking', output='screen',
        parameters=[os.path.join(share, 'config/depth_box_parking.yaml')])
    shadow = Node(package='jdamr_cube_navigation', executable='box_approach_shadow',
                  name='box_approach_shadow', output='screen',
                  parameters=[{key: LaunchConfiguration(key) for key in paths}])
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in paths.items()],
        OpaqueFunction(function=validate_inputs),
        *[RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[Shutdown(reason='required box approach process exited')]))
          for node in (observer, shadow)],
        observer, shadow,
    ])
