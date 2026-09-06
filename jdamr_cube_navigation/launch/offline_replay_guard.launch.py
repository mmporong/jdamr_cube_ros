"""Replay only SLAM sensor inputs through a localization-TF guard."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _preflight(context):
    bag = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('bag').perform(context)))).resolve()
    expected_domain = LaunchConfiguration(
        'offline_domain_id').perform(context).strip()
    actual_domain = os.environ.get('ROS_DOMAIN_ID', '0').strip()
    rate = float(LaunchConfiguration('rate').perform(context))
    delay = float(LaunchConfiguration('delay').perform(context))
    duration = float(LaunchConfiguration('duration').perform(context))
    if not bag.exists():
        raise RuntimeError(f'bag path not found: {bag}')
    if actual_domain == '12' or expected_domain == '12':
        raise RuntimeError('offline replay is forbidden on physical domain 12')
    if actual_domain != expected_domain:
        raise RuntimeError(
            f'offline replay requires ROS_DOMAIN_ID={expected_domain}; '
            f'current value is {actual_domain}')
    if not 0.0 < rate <= 1.0:
        raise RuntimeError('offline replay rate must be greater than 0 and <= 1')
    if delay <= 0.0:
        raise RuntimeError('offline replay delay must be positive')
    if duration != -1.0 and duration <= 0.0:
        raise RuntimeError('duration must be -1 for full replay or positive')
    return []


def generate_launch_description():
    """Keep recorded map authority and motion commands out of replay."""
    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')
    delay = LaunchConfiguration('delay')
    duration = LaunchConfiguration('duration')
    stats_path = LaunchConfiguration('stats_path')

    tf_filter = Node(
        package='jdamr_cube_navigation',
        executable='tf_replay_filter',
        name='tf_replay_filter',
        output='screen',
        parameters=[{'use_sim_time': True, 'stats_path': stats_path}],
    )
    playback = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag,
            '--clock', '100',
            '--rate', rate,
            '--delay', delay,
            '--playback-duration', duration,
            '--disable-keyboard-controls',
            '--topics',
            '/scan', '/odom', '/tf', '/tf_static',
            '/imu/data_raw', '/joint_states',
            '--remap',
            '/tf:=/tf_recorded',
            '/tf_static:=/tf_static_recorded',
            '/odom:=/odom_recorded',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag', description='MCAP bag directory or file to replay'),
        DeclareLaunchArgument(
            'rate', default_value='1.0',
            description='Replay at real time or slower to avoid TF loss'),
        DeclareLaunchArgument('delay', default_value='3.0'),
        DeclareLaunchArgument(
            'duration', default_value='-1.0',
            description='Replay seconds; -1 means the full bag'),
        DeclareLaunchArgument(
            'offline_domain_id', default_value='199',
            description='Must match an isolated ROS_DOMAIN_ID; 12 is blocked'),
        DeclareLaunchArgument(
            'stats_path', default_value='',
            description='Optional replay forwarding evidence JSON path'),
        OpaqueFunction(function=_preflight),
        RegisterEventHandler(OnProcessExit(
            target_action=tf_filter,
            on_exit=[Shutdown(reason='offline TF guard exited')],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=playback,
            on_exit=[Shutdown(reason='offline bag replay finished')],
        )),
        tf_filter,
        playback,
    ])
