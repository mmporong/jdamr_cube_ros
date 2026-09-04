#!/usr/bin/env python3
"""Run one deterministic Gazebo SLAM experiment and compute ATE/RPE."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory

from evaluate_sim_slam import analyse


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def backend_command(
        backend: str,
        slam_params: Path | None,
        cartographer_config_dir: Path | None = None,
        cartographer_config: str = 'jdamr_cube_2d_real.lua') -> list[str]:
    """Return the selected mapping backend command without RViz."""
    if backend == 'cartographer':
        config_dir = (
            cartographer_config_dir
            if cartographer_config_dir is not None
            else Path(get_package_share_directory(
                'jdamr_cube_cartographer')) / 'config')
        return [
            'ros2', 'run', 'cartographer_ros', 'cartographer_node',
            '-configuration_directory', str(config_dir),
            '-configuration_basename', cartographer_config,
            '--ros-args', '-p', 'use_sim_time:=true',
        ]
    if backend == 'slam_toolbox':
        if slam_params is None:
            raise ValueError('slam_toolbox requires --slam-params')
        return [
            'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            'use_sim_time:=true', f'slam_params_file:={slam_params}',
        ]
    raise ValueError(f'unsupported backend: {backend}')


def _start(command: list[str], log_path: Path,
           environment: dict[str, str]):
    log_stream = log_path.open('wb')
    process = subprocess.Popen(
        command,
        env=environment,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        start_new_session=True)
    return process, log_stream


def _stop(process: subprocess.Popen | None, timeout_s: float = 12.0):
    if process is None or process.poll() is not None:
        return None if process is None else process.returncode
    os.killpg(process.pid, signal.SIGINT)
    try:
        return process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            return process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            return process.wait(timeout=5.0)


def _wait_for_topics(
        required: set[str],
        environment: dict[str, str],
        *,
        timeout_s: float = 40.0) -> None:
    deadline = time.monotonic() + timeout_s
    last_topics: set[str] = set()
    while time.monotonic() < deadline:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            env=environment,
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False)
        if result.returncode == 0:
            last_topics = set(result.stdout.splitlines())
            if required <= last_topics:
                return
        time.sleep(0.5)
    missing = sorted(required - last_topics)
    raise RuntimeError(f'simulation topics did not appear: {missing}')


def _one_mcap(bag_dir: Path) -> Path:
    mcaps = sorted(bag_dir.glob('*.mcap'))
    if len(mcaps) != 1:
        raise RuntimeError(
            f'expected one finalized MCAP, found {len(mcaps)}')
    metadata = bag_dir / 'metadata.yaml'
    if not metadata.is_file() or metadata.stat().st_size == 0:
        raise RuntimeError('rosbag metadata is missing or empty')
    return mcaps[0]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('cartographer', 'slam_toolbox'),
                        required=True)
    parser.add_argument('--profile-urdf', type=Path, required=True)
    parser.add_argument('--profile-label', required=True)
    parser.add_argument('--slam-params', type=Path)
    parser.add_argument('--cartographer-config',
                        default='jdamr_cube_2d_real.lua')
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--out-root', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--route', choices=('square', 'corridor'),
                        default='square')
    parser.add_argument('--spawn-x', dest='spawn_x_m', type=float,
                        default=0.0)
    parser.add_argument('--spawn-y', dest='spawn_y_m', type=float,
                        default=0.0)
    parser.add_argument('--corridor-distance-m', type=float, default=14.0)
    args = parser.parse_args(argv)
    for name, path in (
            ('--profile-urdf', args.profile_urdf),
            ('--world', args.world)):
        if not path.is_file():
            parser.error(f'{name} must be an existing file')
    if args.slam_params is not None and not args.slam_params.is_file():
        parser.error('--slam-params must be an existing file')
    if args.backend == 'slam_toolbox' and args.slam_params is None:
        parser.error('slam_toolbox requires --slam-params')

    run_label = f'{args.profile_label}__{args.backend}'
    run_dir = args.out_root / run_label
    if run_dir.exists():
        parser.error(f'run output already exists: {run_dir}')
    run_dir.mkdir(parents=True)
    bag_dir = run_dir / 'bag'
    environment = os.environ.copy()
    environment.update({
        'ROS_DOMAIN_ID': '199',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    })
    gazebo_share = Path(get_package_share_directory('jdamr_cube_gazebo'))
    gazebo_command = [
        'ros2', 'launch', 'jdamr_cube_gazebo', 'gazebo.launch.py',
        'gui:=false', f'seed:={args.seed}',
        f'world:={args.world.resolve()}',
        f'urdf_file:={args.profile_urdf.resolve()}',
        f'x_pose:={args.spawn_x_m}', f'y_pose:={args.spawn_y_m}',
    ]
    cartographer_config_dir = Path(get_package_share_directory(
        'jdamr_cube_cartographer')) / 'config'
    cartographer_config_path = (
        cartographer_config_dir / args.cartographer_config)
    if (args.backend == 'cartographer'
            and not cartographer_config_path.is_file()):
        parser.error('--cartographer-config does not exist')
    selected_backend_command = backend_command(
        args.backend,
        args.slam_params.resolve() if args.slam_params else None,
        cartographer_config_dir,
        args.cartographer_config)
    recorder_command = [
        'ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_dir),
        '--topics', '/tf', '/odom', '/ground_truth_pose', '/scan', '/clock',
    ]
    route_command = [
        'ros2', 'run', 'jdamr_cube_navigation', 'sim_slam_route',
        '--route', args.route,
        '--distance-m', str(args.corridor_distance_m),
        '--ros-args', '-p', 'use_sim_time:=true',
    ]
    if args.route == 'corridor':
        ros_index = route_command.index('--ros-args')
        route_command[ros_index:ros_index] = [
            '--linear-mps', '0.35', '--angular-radps', '0.5']
    commands = {
        'gazebo': gazebo_command,
        'backend': selected_backend_command,
        'recorder': recorder_command,
        'route': route_command,
    }
    processes: dict[str, subprocess.Popen | None] = {
        'gazebo': None,
        'backend': None,
        'recorder': None,
    }
    streams = []
    exit_codes: dict[str, int | None] = {}
    status = 'failed'
    failure = None
    try:
        processes['gazebo'], stream = _start(
            gazebo_command, run_dir / 'gazebo.log', environment)
        streams.append(stream)
        _wait_for_topics(
            {'/ground_truth_pose', '/odom', '/scan'}, environment)
        processes['backend'], stream = _start(
            selected_backend_command, run_dir / 'backend.log', environment)
        streams.append(stream)
        time.sleep(3.0)
        processes['recorder'], stream = _start(
            recorder_command, run_dir / 'recorder.log', environment)
        streams.append(stream)
        time.sleep(1.5)
        with (run_dir / 'route.log').open('wb') as route_log:
            route_result = subprocess.run(
                route_command,
                env=environment,
                stdout=route_log,
                stderr=subprocess.STDOUT,
                timeout=150.0,
                check=False)
        exit_codes['route'] = route_result.returncode
        if route_result.returncode != 0:
            raise RuntimeError(
                f'route exited with {route_result.returncode}')
        time.sleep(3.0)
        exit_codes['recorder'] = _stop(processes['recorder'])
        processes['recorder'] = None
        if exit_codes['recorder'] != 0:
            raise RuntimeError(
                f'recorder exited with {exit_codes["recorder"]}')
        mcap = _one_mcap(bag_dir)
        metrics = analyse(mcap, args.backend)
        (run_dir / 'metrics.json').write_text(
            json.dumps(metrics, ensure_ascii=False, indent=2),
            encoding='utf-8')
        status = 'complete'
    except Exception as error:
        failure = f'{type(error).__name__}: {error}'
        raise
    finally:
        for name in ('recorder', 'backend', 'gazebo'):
            if processes[name] is not None:
                exit_codes[name] = _stop(processes[name])
                processes[name] = None
        for stream in streams:
            stream.close()
        manifest: dict[str, Any] = {
            'schema_version': 1,
            'status': status,
            'failure': failure,
            'run_label': run_label,
            'backend': args.backend,
            'seed': args.seed,
            'route': args.route,
            'spawn_xy_m': [args.spawn_x_m, args.spawn_y_m],
            'corridor_distance_m': args.corridor_distance_m,
            'ros_domain_id': 199,
            'discovery_range': 'LOCALHOST',
            'profile': {
                'label': args.profile_label,
                'urdf': str(args.profile_urdf.resolve()),
                'sha256': _sha256(args.profile_urdf),
            },
            'world': {
                'path': str(args.world.resolve()),
                'sha256': _sha256(args.world),
            },
            'slam_params': (
                {
                    'path': str(args.slam_params.resolve()),
                    'sha256': _sha256(args.slam_params),
                }
                if args.slam_params is not None else None),
            'cartographer_config': (
                {
                    'path': str(cartographer_config_path.resolve()),
                    'sha256': _sha256(cartographer_config_path),
                }
                if args.backend == 'cartographer' else None),
            'commands': commands,
            'exit_codes': exit_codes,
            'gazebo_share': str(gazebo_share),
        }
        (run_dir / 'run_manifest.json').write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding='utf-8')
    print(json.dumps({
        'status': status,
        'run_dir': str(run_dir),
    }, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
