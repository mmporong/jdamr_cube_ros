"""Regression tests for opt-in registered-depth Nav2 integration."""

from copy import deepcopy
import importlib.util
from pathlib import Path

from jdamr_cube_navigation.depth_navigation_config import (
    build_depth_navigation_params,
    DEPTH_FILTER_DEFAULTS,
)
from jdamr_cube_navigation.new_base_contract import validate_new_base_params

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

from launch_ros.actions import Node

import pytest

import yaml


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'jdamr_cube_navigation'
PARAMS = PACKAGE / 'config' / 'new_base_nav2_params.yaml'
DEPTH_CONFIG = PACKAGE / 'config' / 'depth_obstacles.yaml'
GEOMETRY = ROOT / 'jdamr_cube_description/config/new_base_geometry.yaml'
CAMERA_MOUNT = ROOT / 'jdamr_cube_vslam/config/camera_mount.yaml'
LAUNCH = PACKAGE / 'launch/depth_obstacle_navigation.launch.py'


def _load_yaml(path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _load_launch(monkeypatch):
    spec = importlib.util.spec_from_file_location('depth_nav_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    shares = {
        'jdamr_cube_navigation': PACKAGE,
        'jdamr_cube_description': ROOT / 'jdamr_cube_description',
        'jdamr_cube_vslam': ROOT / 'jdamr_cube_vslam',
    }
    monkeypatch.setattr(
        module, 'get_package_share_directory',
        lambda name: str(shares[name]))
    return module


def _context(**overrides):
    values = {
        'mode': 'observe',
        'depth_config': str(DEPTH_CONFIG),
        'params_file': str(PARAMS),
        'geometry_file': str(GEOMETRY),
        'camera_mount_file': str(CAMERA_MOUNT),
        'publish_camera_mount': 'false',
        'map': '/tmp/new_base_map.yaml',
        'keepout_mask': '/tmp/new_base_map_keepout.yaml',
        'discovery_range': 'LOCALHOST',
        'use_sim_time': 'false',
        'autostart': 'true',
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def _set_simulation_environment(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '198')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_STATIC_PEERS', raising=False)


def _allowed_mount(tmp_path):
    document = _load_yaml(CAMERA_MOUNT)
    document['usage_gate']['lidar_rgbd_fusion'] = 'allowed'
    path = tmp_path / 'allowed_camera_mount.yaml'
    path.write_text(yaml.safe_dump(document), encoding='utf-8')
    return path


def test_filter_defaults_match_registered_depth_contract():
    """The installed filter defaults match the measured registered stream."""
    params = _load_yaml(DEPTH_CONFIG)['depth_obstacle_filter'][
        'ros__parameters']
    assert params == {
        'depth_topic': '/camera/depth/image_raw',
        'camera_info_topic': '/camera/depth/camera_info',
        'obstacle_topic': '/depth_navigation/obstacles',
        'ray_topic': '/depth_navigation/rays',
        'status_topic': '/depth_navigation/status',
        'optical_frame': 'camera_color_optical_frame',
        'base_frame': 'base_footprint',
        'calibration_model': 'rectified_projection',
        'min_depth_m': 0.4,
        'max_depth_m': 2.5,
        'min_height_m': 0.05,
        'max_height_m': 1.5,
        'max_forward_m': 2.5,
        'max_lateral_m': 1.5,
        'pixel_stride': 4,
        'voxel_size_m': 0.04,
        'min_valid_fraction': 0.05,
        'max_points': 20000,
        'max_image_age_s': 0.5,
        'max_processing_hz': 5.0,
        'transform_timeout_s': 0.05,
    }
    assert params == DEPTH_FILTER_DEFAULTS


def test_factory_deep_copies_and_adds_approved_sources():
    """The opt-in overlay preserves the base and adds both depth roles."""
    base = _load_yaml(PARAMS)
    original = deepcopy(base)
    depth = build_depth_navigation_params(base)
    assert base == original
    assert depth is not base

    for name in ('local_costmap', 'global_costmap'):
        costmap = depth[name][name]['ros__parameters']
        original_costmap = original[name][name]['ros__parameters']
        assert costmap['obstacle_layer'] == original_costmap['obstacle_layer']
        inflation_index = costmap['plugins'].index('inflation_layer')
        assert costmap['plugins'][inflation_index - 1] == (
            'depth_obstacle_layer')
        layer = costmap['depth_obstacle_layer']
        assert layer['plugin'] == 'nav2_costmap_2d::VoxelLayer'
        assert layer['observation_sources'] == 'depth_marks depth_rays'
        assert layer['z_voxels'] == 16
        assert layer['z_resolution'] == 0.1
        assert layer['depth_marks']['sensor_frame'] == (
            'camera_color_optical_frame')
        assert layer['depth_marks']['marking'] is True
        assert layer['depth_marks']['clearing'] is False
        assert layer['depth_rays']['marking'] is False
        assert layer['depth_rays']['clearing'] is True
        assert layer['depth_marks']['obstacle_min_range'] == 0.4
        assert layer['depth_rays']['min_obstacle_height'] == -0.05
        assert layer['depth_rays']['raytrace_min_range'] == 0.4

    monitor = depth['collision_monitor']['ros__parameters']
    assert monitor['observation_sources'] == ['scan', 'depth_obstacles']
    assert monitor['depth_obstacles'] == {
        'type': 'pointcloud',
        'topic': '/depth_navigation/obstacles',
        'enabled': True,
        'source_timeout': 1.0,
        'min_height': 0.05,
        'max_height': 1.5,
    }
    validate_new_base_params(depth, _load_yaml(GEOMETRY))


@pytest.mark.parametrize('mutation', [
    lambda params: params['local_costmap']['local_costmap'][
        'ros__parameters']['depth_obstacle_layer'][
            'depth_marks'].__setitem__(
            'sensor_frame', 'base_footprint'),
    lambda params: params['global_costmap']['global_costmap'][
        'ros__parameters']['depth_obstacle_layer'].__setitem__(
            'z_voxels', 8),
    lambda params: params['global_costmap']['global_costmap'][
        'ros__parameters'].pop('depth_obstacle_layer'),
    lambda params: params['collision_monitor']['ros__parameters'].update(
        observation_sources=['scan']),
    lambda params: params['collision_monitor']['ros__parameters'][
        'depth_obstacles'].__setitem__('source_timeout', 2.0),
    lambda params: params['collision_monitor']['ros__parameters'][
        'StopZone'].__setitem__('sources_names', ['scan']),
])
def test_validator_rejects_unapproved_depth_source_changes(mutation):
    """Depth source frames, timeout, and polygon coverage stay fail-closed."""
    params = build_depth_navigation_params(_load_yaml(PARAMS))
    mutation(params)
    with pytest.raises(RuntimeError):
        validate_new_base_params(params, _load_yaml(GEOMETRY))


def test_observe_mode_starts_filter_only(monkeypatch):
    """Observe mode cannot start Nav2 or a motor command path."""
    module = _load_launch(monkeypatch)
    context = _context()
    actions = module._launch_selected_mode(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(nodes) == 1
    assert nodes[0].node_executable == 'depth_obstacle_filter'
    assert not any(isinstance(action, IncludeLaunchDescription)
                   for action in actions)
    assert any(isinstance(action, RegisterEventHandler)
               for action in actions)
    assert any(isinstance(action, LogInfo) for action in actions)


def test_physical_navigation_fails_closed_on_blocked_fusion_gate(monkeypatch):
    """The current unapproved physical fusion gate blocks navigation."""
    module = _load_launch(monkeypatch)
    with pytest.raises(RuntimeError, match='lidar_rgbd_fusion=allowed'):
        module._launch_selected_mode(_context(mode='navigation'))


def test_fused_mode_rejects_custom_filter_contract(monkeypatch, tmp_path):
    """Navigation cannot pair its fixed overlay with divergent filter data."""
    module = _load_launch(monkeypatch)
    custom = _load_yaml(DEPTH_CONFIG)
    custom['depth_obstacle_filter']['ros__parameters']['max_depth_m'] = 3.0
    path = tmp_path / 'custom_depth.yaml'
    path.write_text(yaml.safe_dump(custom), encoding='utf-8')
    _set_simulation_environment(monkeypatch)
    context = _context(
        mode='navigation', use_sim_time='true', depth_config=str(path))
    with pytest.raises(RuntimeError, match='approved contract'):
        module._launch_selected_mode(context)


@pytest.mark.parametrize('environment,value,message', [
    ('ROS_DOMAIN_ID', None, 'explicit ROS_DOMAIN_ID'),
    ('ROS_DOMAIN_ID', '12', r'ROS_DOMAIN_ID in \[100, 232\]'),
    ('ROS_DOMAIN_ID', '99', r'ROS_DOMAIN_ID in \[100, 232\]'),
    ('ROS_DOMAIN_ID', '233', r'ROS_DOMAIN_ID in \[100, 232\]'),
    ('ROS_DOMAIN_ID', 'invalid', 'explicit ROS_DOMAIN_ID'),
    ('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET', 'LOCALHOST discovery'),
    ('ROS_STATIC_PEERS', '192.0.2.10', 'empty ROS_STATIC_PEERS'),
])
def test_simulated_fusion_requires_isolated_dds_environment(
        monkeypatch, environment, value, message):
    """Simulation cannot bypass physical gates on a shared DDS graph."""
    module = _load_launch(monkeypatch)
    _set_simulation_environment(monkeypatch)
    if value is None:
        monkeypatch.delenv(environment)
    else:
        monkeypatch.setenv(environment, value)
    with pytest.raises(RuntimeError, match=message):
        module._launch_selected_mode(
            _context(mode='navigation', use_sim_time='true'))


def test_simulated_fusion_rejects_subnet_launch_override(monkeypatch):
    """A safe ambient environment cannot be widened by a launch argument."""
    module = _load_launch(monkeypatch)
    _set_simulation_environment(monkeypatch)
    with pytest.raises(RuntimeError, match='LOCALHOST discovery'):
        module._launch_selected_mode(_context(
            mode='mapping', use_sim_time='true', discovery_range='SUBNET'))


def test_physical_fusion_requires_owned_camera_mount_publisher(
        monkeypatch, tmp_path):
    """Approved physical fusion fails if its authoritative TF is absent."""
    module = _load_launch(monkeypatch)
    mount = _allowed_mount(tmp_path)
    with pytest.raises(RuntimeError, match='publish_camera_mount=true'):
        module._launch_selected_mode(_context(
            mode='navigation', camera_mount_file=str(mount)))


def test_measured_camera_mount_publisher_is_required_process(
        monkeypatch, tmp_path):
    """The opt-in measured TF publisher uses exact frames and measurements."""
    module = _load_launch(monkeypatch)
    mount = _allowed_mount(tmp_path)
    context = _context(
        mode='navigation', camera_mount_file=str(mount),
        publish_camera_mount='true')
    before = set(Path('/tmp').glob('jdamr_depth_nav2_*.yaml'))
    actions = module._launch_selected_mode(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    publisher = next(
        node for node in nodes
        if node.node_executable == 'static_transform_publisher')
    arguments = [
        perform_substitutions(context, part)
        for part in publisher.cmd[1:-1]
    ]
    assert arguments[:16] == [
        '--x', '0.065', '--y', '0.0', '--z', '0.215',
        '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
        '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
    ]
    exit_handlers = [
        action.event_handler for action in actions
        if isinstance(action, RegisterEventHandler)
        and isinstance(action.event_handler, OnProcessExit)]
    assert len(exit_handlers) == len(nodes)
    temporary_files = (
        set(Path('/tmp').glob('jdamr_depth_nav2_*.yaml')) - before)
    assert len(temporary_files) == 1
    module._cleanup_file(temporary_files.pop())(None, context)


@pytest.mark.parametrize('field,value', [
    ('parent_frame', 'base_footprint'),
    ('x_m', float('nan')),
])
def test_camera_mount_publisher_rejects_invalid_measurement(
        monkeypatch, tmp_path, field, value):
    """Static TF accepts only the measured finite frame contract."""
    module = _load_launch(monkeypatch)
    document = _load_yaml(CAMERA_MOUNT)
    target = (document['camera_mount'] if field == 'parent_frame'
              else document['camera_mount']['transform'])
    target[field] = value
    path = tmp_path / 'invalid_camera_mount.yaml'
    path.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(RuntimeError, match='measured finite'):
        module._launch_selected_mode(_context(
            camera_mount_file=str(path), publish_camera_mount='true'))


@pytest.mark.parametrize('mode, launch_name', [
    ('navigation', 'onboard_nav2_core.launch.py'),
    ('mapping', 'autonomous_mapping.launch.py'),
])
def test_simulation_modes_include_existing_guarded_launch_and_cleanup(
        monkeypatch, mode, launch_name):
    """Simulation composes guarded launches and a removable Nav2 overlay."""
    module = _load_launch(monkeypatch)
    _set_simulation_environment(monkeypatch)
    context = _context(mode=mode, use_sim_time='true')
    before = set(Path('/tmp').glob('jdamr_depth_nav2_*.yaml'))
    actions = module._launch_selected_mode(context)
    includes = [action for action in actions
                if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 1
    source = includes[0]._IncludeLaunchDescription__launch_description_source
    location = perform_substitutions(
        context, source._LaunchDescriptionSource__location)
    assert Path(location).name == launch_name
    cleanup_handlers = [action for action in actions
                        if isinstance(action, RegisterEventHandler)]
    assert len(cleanup_handlers) >= 2
    assert any(isinstance(action.event_handler, OnShutdown)
               for action in cleanup_handlers)

    temporary_files = (
        set(Path('/tmp').glob('jdamr_depth_nav2_*.yaml')) - before)
    assert len(temporary_files) == 1
    temporary_file = temporary_files.pop()
    arguments = {
        name: perform_substitutions(
            context, normalize_to_list_of_substitutions(value))
        for name, value in includes[0].launch_arguments
    }
    assert Path(arguments['params_file']) == temporary_file
    if mode == 'navigation':
        assert arguments['navigation_profile'] == 'new_base_candidate'
        assert arguments['discovery_range'] == 'LOCALHOST'
    else:
        assert 'map' not in arguments
        assert 'keepout_mask' not in arguments
        assert arguments['discovery_range'] == 'LOCALHOST'
    overlay = _load_yaml(temporary_file)
    assert overlay['collision_monitor']['ros__parameters'][
        'observation_sources'] == ['scan', 'depth_obstacles']
    module._cleanup_file(temporary_file)(None, context)
    assert not temporary_file.exists()
