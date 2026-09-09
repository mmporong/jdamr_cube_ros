"""Run keepout Nav2 and protocol recording together on the robot computer."""

from datetime import datetime
import os
from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.onboard_recording import RECORDED_TOPICS
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# The recorded topic set and its QoS depths are one contract shared with the
# evaluation overrides, so importing keeps them from drifting apart.


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


def _shutdown_on_recorder_exit(_event, context):
    """Stop Nav2 only when the recorder exit did not follow shutdown."""
    if context.is_shutdown:
        return []
    return [Shutdown(reason='onboard recorder exited; stopping navigation')]


def generate_launch_description():
    """Keep the motion loop and essential recording off the Wi-Fi link."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    onboard_core_launch = os.path.join(
        package_share, 'launch', 'onboard_nav2_core.launch.py')
    writer_options = os.path.join(
        package_share, 'evaluation', 'mcap_writer_options.yaml')
    qos_overrides = os.path.join(
        package_share, 'evaluation', 'qos_overrides.yaml')
    run_id = datetime.now().strftime(
        'corridor_keepout_onboard_%Y%m%dT%H%M%S')

    map_yaml = LaunchConfiguration('map')
    keepout_mask = LaunchConfiguration('keepout_mask')
    params_file = LaunchConfiguration('params_file')
    navigation_profile = LaunchConfiguration('navigation_profile')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    record_bag = LaunchConfiguration('record_bag')
    bag_output = LaunchConfiguration('bag_output')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(onboard_core_launch),
        launch_arguments={
            'map': map_yaml,
            'keepout_mask': keepout_mask,
            'params_file': params_file,
            'navigation_profile': navigation_profile,
            'discovery_range': 'SUBNET',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
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
            # No terminal owns the recorder under launch, so the keyboard
            # control thread only adds a polling thread to a loaded Pi.
            '--disable-keyboard-controls',
            '--include-hidden-topics',
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
            on_exit=_shutdown_on_recorder_exit,
        ),
        condition=IfCondition(record_bag),
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        # Sensor publishers remain LOCALHOST-only.  On the current Jazzy /
        # Fast DDS build, a SUBNET participant on the same host receives their
        # user data in both directions while a LOCALHOST participant only
        # discovers endpoint names.  The field observation belongs in the
        # evaluation report; this comment pins the asymmetric boundary.
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
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
            'navigation_profile', default_value='corridor',
            choices=[
                'corridor', 'obstacle_candidate',
                'obstacle_base_candidate'],
            description='Select the same navigation profile for the route runner'),
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
        navigation,
        recorder,
    ])
