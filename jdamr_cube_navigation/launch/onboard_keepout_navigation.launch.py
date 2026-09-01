"""Run keepout Nav2 and protocol recording together on the robot computer."""

from datetime import datetime
import os
from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Keep only mapping, localization, command, and safety evidence.  RViz-only
# state such as joint_states stays off the Pi recorder by default.
RECORDED_TOPICS = [
    '/scan',
    '/odom',
    '/tf',
    '/tf_static',
    '/imu/data_raw',
    '/cmd_vel',
    '/cmd_vel_nav',
    '/amcl_pose',
    '/battery_state',
    '/plan',
    '/collision_monitor_state',
]


def _enabled(context, argument_name):
    value = LaunchConfiguration(argument_name).perform(context)
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _prepare_bag_output(context):
    """Fail before navigation if the onboard evidence path is not writable."""
    if not _enabled(context, 'record_bag'):
        return []
    for executable in ('nice', 'ionice'):
        if shutil.which(executable) is None:
            raise RuntimeError(
                f'{executable} is required for low-priority onboard recording')
    output = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('bag_output').perform(context))))
    if not output.is_absolute():
        raise RuntimeError('bag_output must be an absolute path')
    if output.exists():
        raise RuntimeError(f'bag_output already exists: {output}')
    output.parent.mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    """Keep the motion loop and essential recording off the Wi-Fi link."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    keepout_launch = os.path.join(
        package_share, 'launch', 'keepout_navigation.launch.py')
    writer_options = os.path.join(
        package_share, 'evaluation', 'mcap_writer_options.yaml')
    qos_overrides = os.path.join(
        package_share, 'evaluation', 'qos_overrides.yaml')
    run_id = datetime.now().strftime(
        'corridor_keepout_onboard_%Y%m%dT%H%M%S')

    map_yaml = LaunchConfiguration('map')
    keepout_mask = LaunchConfiguration('keepout_mask')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    record_bag = LaunchConfiguration('record_bag')
    bag_output = LaunchConfiguration('bag_output')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(keepout_launch),
        launch_arguments={
            'map': map_yaml,
            'keepout_mask': keepout_mask,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            # Pi 4 static-soak evidence: composition reduced load from about
            # 13 to 3.53.  Process-group shutdown remains the required stop.
            'use_composition': 'true',
            # This wrapper runs headless on the Pi.  The laptop may open the
            # low-bandwidth RViz profile without owning the control loop.
            'use_rviz': 'false',
        }.items(),
    )
    recorder = ExecuteProcess(
        condition=IfCondition(record_bag),
        cmd=[
            # Navigation wins CPU and storage contention.  A recorder that
            # cannot keep up is rejected by the post-run loss gate instead of
            # delaying the safety/control loop.
            'ionice', '--class', 'best-effort', '--classdata', '7',
            'nice', '--adjustment', '10',
            'ros2', 'bag', 'record',
            '--storage', 'mcap',
            '--storage-config-file', writer_options,
            '--qos-profile-overrides-path', qos_overrides,
            '--output', bag_output,
            '--topics', *RECORDED_TOPICS,
        ],
        output='screen',
    )
    stop_if_recorder_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder,
            on_exit=[Shutdown(
                reason='onboard recorder exited; stopping navigation')],
        ),
        condition=IfCondition(record_bag),
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908.yaml')),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908_keepout_multi.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'record_bag', default_value='true',
            description='Record essential full-rate evidence on the robot'),
        DeclareLaunchArgument(
            'bag_output',
            default_value=os.path.expanduser(
                f'~/jdamr_artifacts/{run_id}'),
            description='Absolute, not-yet-existing output directory on Pi'),
        OpaqueFunction(function=_prepare_bag_output),
        stop_if_recorder_exits,
        recorder,
        navigation,
    ])
