"""Validate the dedicated chain before any of its processes are started."""

import importlib.util
from pathlib import Path

import pytest
import yaml


def launch_module():
    """Load the launch validation without launching ROS processes."""
    path = Path(__file__).resolve().parents[1] / 'launch/box_approach_execution.launch.py'
    spec = importlib.util.spec_from_file_location('box_execution_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def paths():
    """Use the repository's versioned geometry and protection parameters."""
    nav = Path(__file__).resolve().parents[1]
    root = nav.parent
    return {
        'camera_mount_file': str(root / 'jdamr_cube_vslam/config/camera_mount.yaml'),
        'geometry_file': str(root / 'jdamr_cube_description/config/new_base_geometry.yaml'),
        'parking_contract_file': str(nav / 'config/parking_contract.yaml'),
        'nav_params_file': str(nav / 'config/new_base_nav2_params.yaml'),
    }


def test_measured_snapshot_keeps_protected_chain():
    """Both protection components consume the existing validated configuration."""
    params = launch_module().validate_configuration(paths())
    cm = params['collision_monitor']['ros__parameters']
    assert cm['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert cm['cmd_vel_out_topic'] == 'cmd_vel'
    assert cm['FootprintApproach']['enabled']
    assert params['velocity_smoother']['ros__parameters']['velocity_timeout'] <= .5


def test_unmeasured_underlay_fails_before_motion_probe(tmp_path):
    """The old Pi null-calibration failure is caught before starting children."""
    selected = paths()
    camera = tmp_path / 'camera.yaml'
    camera.write_text(yaml.safe_dump({'camera_mount': {
        'status': 'unmeasured', 'transform': {'roll_rad': None}}}))
    selected['camera_mount_file'] = str(camera)
    with pytest.raises(ValueError, match='camera_mount_unmeasured_or_invalid'):
        launch_module().validate_configuration(selected)


def test_protection_cannot_be_silently_disabled(tmp_path):
    """A stale or relaxed stop-zone configuration does not reach startup."""
    selected = paths()
    config = yaml.safe_load(Path(selected['nav_params_file']).read_text())
    config['collision_monitor']['ros__parameters']['StopZone']['enabled'] = False
    path = tmp_path / 'nav.yaml'
    path.write_text(yaml.safe_dump(config))
    selected['nav_params_file'] = str(path)
    with pytest.raises(RuntimeError):
        launch_module().validate_configuration(selected)


def test_empty_discovery_is_not_proof_of_clear_command_ownership():
    """Absent base graph must block, even if no motion publishers are seen."""
    assert not launch_module().startup_graph_ready({}, [], [])


def test_discovered_base_and_late_motion_conflict():
    """Repeated discovery rejects a publisher that appears after the base."""
    module = launch_module()
    publishers = {topic: [('sensor', '/')] for topic in ('/odom', '/scan', '/battery_state')}
    subscribers = [('jdamr_base_driver', '/')]
    assert module.startup_graph_ready(publishers, subscribers, [])
    assert module.startup_graph_ready(
        publishers, subscribers + [('ammr_dashboard_monitor', '/')], [])
    assert not module.startup_graph_ready(publishers, subscribers * 2, [])
    with pytest.raises(RuntimeError, match='existing_motion_stack'):
        module.startup_graph_ready(publishers, subscribers, ['jdamr_depth_box_parking'])
    publishers['/cmd_vel_nav'] = [('controller_server', '/')]
    with pytest.raises(RuntimeError, match='existing_motion_stack'):
        module.startup_graph_ready(publishers, subscribers, [])
