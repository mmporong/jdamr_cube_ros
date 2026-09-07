"""Launch one isolated G005 Gazebo, oracle-map, and Nav2 evaluation run."""

import importlib.util
import json
import math
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


VALID_LAYOUT_SEEDS = {11, 23, 42, 67, 89}
VALID_ASSET_MODES = {'smoke', 'full'}
ASSET_FAILURE_SIGNAL = 'G005_INVALID_ASSETS'
SPAWN_FAILURE_SIGNAL = 'G005_INVALID_ROBOT_SPAWN'


def _load_asset_generator():
    """Load the sealed evaluator without relying on ambient PYTHONPATH."""
    evaluation = Path(get_package_share_directory(
        'jdamr_cube_navigation')) / 'evaluation'
    contract_path = (evaluation / 'frontier_policy_contract.py').resolve(
        strict=True)
    generator_path = (
        evaluation / 'generate_frontier_policy_assets.py').resolve(strict=True)

    def load(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        if spec is None or spec.loader is None:
            raise ValueError('G005 evaluation module loader drift')
        loaded = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(loaded)
        if Path(loaded.__file__).resolve(strict=True) != path:
            raise ValueError('G005 evaluation module identity drift')
        return loaded

    contract = load('_g005_runtime_frontier_contract', contract_path)
    previous_contract = sys.modules.get('frontier_policy_contract')
    sys.modules['frontier_policy_contract'] = contract
    try:
        generator = load('_g005_runtime_asset_generator', generator_path)
    finally:
        if previous_contract is None:
            sys.modules.pop('frontier_policy_contract', None)
        else:
            sys.modules['frontier_policy_contract'] = previous_contract
    if not callable(getattr(generator, 'validate_assets', None)):
        raise ValueError('G005 asset validator entry point drift')
    return generator


def _asset_paths(asset_root: str, layout_seed: str, asset_mode: str) -> dict:
    """Resolve the generated files and start pose for one fixed layout."""
    root = Path(asset_root)
    if not root.is_absolute() or not root.is_dir() or root.is_symlink():
        raise ValueError('G005 asset root must be an absolute directory')
    try:
        seed = int(layout_seed)
    except ValueError as error:
        raise ValueError('G005 layout seed must be an integer') from error
    if seed not in VALID_LAYOUT_SEEDS or str(seed) != layout_seed:
        raise ValueError('G005 layout seed is not preregistered')
    if asset_mode not in VALID_ASSET_MODES:
        raise ValueError('G005 asset mode is not preregistered')
    try:
        manifest = _load_asset_generator().validate_assets(root, asset_mode)
    except Exception as error:
        raise ValueError(f'{ASSET_FAILURE_SIGNAL}: {error}') from error
    if seed not in manifest['layout_seeds']:
        raise ValueError('G005 layout seed is absent from sealed assets')
    paths = {
        'layout': root / f'layout_{seed}_gt.json',
        'world': root / f'layout_{seed}.world',
        'robot': root / 'g005_robot.urdf',
        'params': root / 'g005_nav2_params.yaml',
        'manifest': root / 'asset_manifest.json',
    }
    if any(path.is_symlink() or not path.is_file() for path in paths.values()):
        raise ValueError('G005 generated runtime asset is missing')
    layout = json.loads(paths['layout'].read_text(encoding='utf-8'))
    if (type(layout) is not dict or layout.get('layout_seed') != seed or
            type(layout.get('start_cell')) is not list or
            len(layout['start_cell']) != 2 or
            type(layout.get('origin_m_rad')) is not list or
            len(layout['origin_m_rad']) != 3 or
            type(layout.get('resolution_m_per_cell')) not in (int, float)):
        raise ValueError('G005 generated layout schema drift')
    start_x, start_y = layout['start_cell']
    resolution = float(layout['resolution_m_per_cell'])
    origin_x, origin_y, origin_yaw = [
        float(value) for value in layout['origin_m_rad']]
    local_x = (start_x + 0.5) * resolution
    local_y = (start_y + 0.5) * resolution
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    return {
        **{name: str(path) for name, path in paths.items()},
        'seed': str(seed),
        'start_x': str(origin_x + cosine * local_x - sine * local_y),
        'start_y': str(origin_y + sine * local_x + cosine * local_y),
        'start_yaw': str(origin_yaw),
    }


def _spawn_exit_actions(returncode: int, observer, navigation):
    """Start downstream nodes only after a successful robot spawn."""
    if type(returncode) is not int or returncode != 0:
        raise RuntimeError(
            f'{SPAWN_FAILURE_SIGNAL}: create_exit_code={returncode!r}')
    return [observer, navigation]


def _spawn_exit_handler(observer, navigation):
    """Bind the spawn result to the guarded downstream action set."""
    def handle(event, context):
        del context
        return _spawn_exit_actions(event.returncode, observer, navigation)

    return handle


def _runtime_actions(context):
    package_share = get_package_share_directory('jdamr_cube_navigation')
    gazebo_share = get_package_share_directory('jdamr_cube_gazebo')
    description_share = get_package_share_directory('jdamr_cube_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    resolved = _asset_paths(
        LaunchConfiguration('asset_root').perform(context),
        LaunchConfiguration('layout_seed').perform(context),
        LaunchConfiguration('asset_mode').perform(context))
    robot_description = Path(resolved['robot']).read_text(encoding='utf-8')
    robot_description = robot_description.replace(
        'package://jdamr_cube_description/config/so101_controllers.yaml',
        os.path.join(
            description_share, 'config', 'so101_controllers.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [
            '-r -s -v2 --seed ', resolved['seed'], ' ', resolved['world'],
        ]}.items())
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }])
    spawn = Node(
        package='ros_gz_sim', executable='create', name='g005_spawn_robot',
        output='screen', arguments=[
            '-topic', 'robot_description', '-name', 'jdamr_cube',
            '-x', resolved['start_x'], '-y', resolved['start_y'],
            '-z', '0.01', '-Y', resolved['start_yaw'],
        ])
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='g005_runtime_bridge', output='screen',
        arguments=['--ros-args', '-p', [
            'config_file:=',
            os.path.join(gazebo_share, 'params', 'bridge.yaml'),
        ]])
    contact_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='g005_contact_bridge', output='screen', arguments=[
            '/g005_contacts@ros_gz_interfaces/msg/Contacts'
            '[gz.msgs.Contacts',
        ])
    observer = Node(
        package='jdamr_cube_navigation',
        executable='g005_frontier_observer',
        name='g005_frontier_observer', output='screen', parameters=[{
            'use_sim_time': use_sim_time,
            'asset_root': str(Path(resolved['layout']).parent),
            'layout_seed': int(resolved['seed']),
        }])
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_share, 'launch', 'g005_frontier_eval.launch.py')),
        launch_arguments={
            'params_file': resolved['params'],
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items())

    after_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=_spawn_exit_handler(observer, navigation)))
    return [gazebo, robot_state_publisher, bridge, contact_bridge,
            spawn, after_spawn]


def generate_launch_description():
    """Build one runtime whose inputs are supplied by the sealed assets."""
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.dirname(get_package_share_directory(
                'jdamr_cube_description'))),
        DeclareLaunchArgument(
            'asset_root', description='Absolute generated G005 asset root'),
        DeclareLaunchArgument(
            'layout_seed', description='Preregistered G005 layout seed'),
        DeclareLaunchArgument(
            'asset_mode', choices=['smoke', 'full'],
            description='Sealed G005 asset generation mode'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_runtime_actions),
    ])
