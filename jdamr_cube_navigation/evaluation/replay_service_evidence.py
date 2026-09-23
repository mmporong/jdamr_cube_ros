#!/usr/bin/env python3
"""Replay a motion-safe topic subset into an isolated portfolio RViz session."""

import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
import yaml


REPLAY_TOPICS = (
    '/map',
    '/tf',
    '/tf_static',
    '/scan',
    '/odom',
    '/amcl_pose',
    '/plan',
    '/service_visualization/destinations',
    '/service_visualization/saved_keepout',
)
ISOLATED_ENVIRONMENT = {
    'ROS_DOMAIN_ID': '78',
    'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    'ROS_STATIC_PEERS': '',
    'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4',
}


def default_rviz_config():
    """Return the installed read-only live configuration used as a template."""
    package = Path(get_package_share_directory('jdamr_cube_navigation'))
    return package / 'rviz/restaurant_service.rviz'


def _display_name(display):
    name = str(display.get('Name', '')).strip()
    return name if name.startswith('Recorded ') else f'Recorded {name}'


def recorded_rviz_document(source):
    """Create a bag-only RViz document without changing the live config."""
    document = yaml.safe_load(Path(source).read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError('RViz config must contain a mapping')
    result = deepcopy(document)
    manager = result.get('Visualization Manager')
    if not isinstance(manager, dict) or not isinstance(
            manager.get('Displays'), list):
        raise ValueError('RViz config must contain Visualization Manager displays')
    manager.setdefault('Global Options', {})['Use Sim Time'] = True
    for display in manager['Displays']:
        if not isinstance(display, dict):
            raise ValueError('RViz display entry must be a mapping')
        display['Name'] = _display_name(display)
        if display.get('Class') == 'rviz_default_plugins/RobotModel':
            display['Enabled'] = False
    manager['Displays'].extend([
        {
            'Class': 'rviz_default_plugins/Axes',
            'Name': 'Recorded Base Footprint Axes',
            'Enabled': True,
            'Reference Frame': 'base_footprint',
            'Length': 0.5,
            'Radius': 0.03,
        },
        {
            'Class': 'rviz_default_plugins/Odometry',
            'Name': 'Recorded Odometry',
            'Enabled': True,
            'Keep': 100,
            'Topic': {
                'Value': '/odom',
                'Reliability Policy': 'Best Effort',
            },
        },
    ])
    return result


def replay_commands(bag, rviz_config, rate):
    """Build the two child argv arrays; never include command topics."""
    bag = str(Path(bag).resolve())
    rviz_config = str(Path(rviz_config).resolve())
    return {
        'rviz': [
            'ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config,
            '--ros-args', '-p', 'use_sim_time:=true',
        ],
        'player': [
            'ros2', 'bag', 'play', bag,
            '--clock', '30', '--delay', '3', '--rate', str(rate),
            '--topics', *REPLAY_TOPICS,
        ],
    }


def _positive_rate(value):
    try:
        rate = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('rate must be a number') from error
    if not math.isfinite(rate) or rate <= 0.0:
        raise argparse.ArgumentTypeError('rate must be finite and positive')
    return rate


def parse_args(argv=None):
    """Parse replay inputs without sourcing or mutating a ROS environment."""
    parser = argparse.ArgumentParser(
        description='Replay non-command restaurant evidence in isolated RViz')
    parser.add_argument('--bag', required=True, type=Path,
                        help='Absolute rosbag2 directory')
    parser.add_argument('--rviz-config', type=Path, default=None,
                        help='Live RViz template; the source is never modified')
    parser.add_argument('--rate', type=_positive_rate, default=1.0)
    parser.add_argument(
        '--print-command', action='store_true',
        help='Print isolated environment, argv and generated RViz YAML as JSON')
    return parser.parse_args(argv)


def _validated_paths(args):
    bag = args.bag.expanduser()
    if not bag.is_absolute() or not bag.is_dir():
        raise ValueError(f'bag must be an existing absolute directory: {bag}')
    template = (args.rviz_config or default_rviz_config()).expanduser()
    if not template.is_absolute() or not template.is_file():
        raise ValueError(
            f'RViz config must be an existing absolute file: {template}')
    return bag.resolve(), template.resolve()


def _stop_child(process, timeout_s=5.0):
    """Stop only the process group created for one replay child."""
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout_s)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=timeout_s)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def _run_children(commands, environment):
    """Keep RViz open after playback; close children on window close or Ctrl-C."""
    rviz = None
    player = None
    try:
        rviz = subprocess.Popen(
            commands['rviz'], env=environment, start_new_session=True)
        player = subprocess.Popen(
            commands['player'], env=environment, start_new_session=True)
        while rviz.poll() is None:
            time.sleep(0.2)
        return rviz.returncode or 0
    except KeyboardInterrupt:
        return 130
    finally:
        _stop_child(player)
        _stop_child(rviz)


def main(argv=None):
    """Generate a temporary recorded-data view and optionally launch children."""
    args = parse_args(argv)
    try:
        bag, template = _validated_paths(args)
        document = recorded_rviz_document(template)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f'error: {error}', file=os.sys.stderr)
        return 2

    descriptor, generated_name = tempfile.mkstemp(
        prefix='jdamr_recorded_service_', suffix='.rviz')
    generated = Path(generated_name)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(document, stream, sort_keys=False,
                           allow_unicode=True)
        commands = replay_commands(bag, generated, args.rate)
        if args.print_command:
            print(json.dumps({
                'allowenv': ISOLATED_ENVIRONMENT,
                'argv': commands,
                'rviz_config': document,
            }, ensure_ascii=False, sort_keys=True))
            return 0
        environment = os.environ.copy()
        environment.update(ISOLATED_ENVIRONMENT)
        return _run_children(commands, environment)
    finally:
        generated.unlink(missing_ok=True)


if __name__ == '__main__':
    raise SystemExit(main())
