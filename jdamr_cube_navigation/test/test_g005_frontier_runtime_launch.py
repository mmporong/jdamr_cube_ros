"""Contract tests for the isolated G005 Gazebo and Nav2 runtime launch."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import pytest


LAUNCH = (
    Path(__file__).parents[1] / 'launch' / 'g005_frontier_runtime.launch.py')
NAVIGATION_LAUNCH = (
    Path(__file__).parents[1] / 'launch' / 'g005_frontier_eval.launch.py')
PACKAGE_XML = Path(__file__).parents[1] / 'package.xml'


def _module():
    spec = importlib.util.spec_from_file_location('g005_runtime_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _assets(tmp_path, mode='smoke'):
    module = _module()
    root = tmp_path / 'assets'
    root.parent.mkdir(parents=True, exist_ok=True)
    module._load_asset_generator().generate(root, mode)
    return root


def test_asset_paths_resolve_exact_seed_and_rotated_start(tmp_path):
    module = _module()
    root = _assets(tmp_path)
    resolved = module._asset_paths(str(root), '11', 'smoke')
    assert float(resolved['start_x']) == pytest.approx(-4.625)
    assert float(resolved['start_y']) == pytest.approx(0.125)
    assert resolved['seed'] == '11'
    assert resolved['robot'].endswith('/g005_robot.urdf')


def test_asset_paths_accept_full_manifest_seed_and_reject_smoke_absence(
        tmp_path):
    module = _module()
    full_root = _assets(tmp_path / 'full', 'full')
    assert module._asset_paths(full_root.as_posix(), '89', 'full')[
        'seed'] == '89'
    smoke_root = _assets(tmp_path / 'smoke', 'smoke')
    with pytest.raises(ValueError, match='absent'):
        module._asset_paths(smoke_root.as_posix(), '89', 'smoke')


@pytest.mark.parametrize('seed', ['11.0', '12', 'x'])
def test_asset_paths_reject_noncanonical_or_unregistered_seed(tmp_path, seed):
    module = _module()
    root = _assets(tmp_path)
    with pytest.raises(ValueError, match='seed'):
        module._asset_paths(str(root), seed, 'smoke')


def test_asset_paths_reject_missing_or_symlinked_runtime_input(tmp_path):
    module = _module()
    root = _assets(tmp_path)
    (root / 'layout_11.world').unlink()
    with pytest.raises(ValueError, match='inventory|missing'):
        module._asset_paths(str(root), '11', 'smoke')
    (root / 'layout_11.world').symlink_to(root / 'g005_robot.urdf')
    with pytest.raises(ValueError, match='inventory|missing'):
        module._asset_paths(str(root), '11', 'smoke')


def test_asset_paths_reject_wrong_mode_or_tampered_asset(tmp_path):
    module = _module()
    root = _assets(tmp_path)
    with pytest.raises(ValueError, match='manifest|tree|mode'):
        module._asset_paths(str(root), '11', 'full')
    with (root / 'layout_11.world').open('ab') as stream:
        stream.write(b'\n<!-- tampered -->\n')
    with pytest.raises(ValueError, match='hash|geometry|tree'):
        module._asset_paths(str(root), '11', 'smoke')


def test_asset_paths_reject_duplicate_manifest_key(tmp_path):
    module = _module()
    root = _assets(tmp_path)
    manifest = root / 'asset_manifest.json'
    manifest.write_text('{"schema_version":1,"schema_version":1}\n',
                        encoding='utf-8')
    with pytest.raises(ValueError, match='duplicate JSON key'):
        module._asset_paths(str(root), '11', 'smoke')


def test_spawn_exit_starts_downstream_only_for_exact_zero():
    module = _module()
    observer = object()
    navigation = object()
    assert module._spawn_exit_actions(0, observer, navigation) == [
        observer, navigation]
    for returncode in (1, -9, None, False):
        with pytest.raises(RuntimeError, match=(
                'G005_INVALID_ROBOT_SPAWN.*create_exit_code')):
            module._spawn_exit_actions(returncode, observer, navigation)


def test_process_exit_callback_is_bound_to_spawn_result():
    module = _module()
    observer = object()
    navigation = object()
    handler = module._spawn_exit_handler(observer, navigation)
    assert handler(SimpleNamespace(returncode=0), object()) == [
        observer, navigation]
    assert module.SPAWN_FAILURE_SIGNAL == 'G005_INVALID_ROBOT_SPAWN'


def test_nonzero_spawn_callback_terminates_launch_with_failure():
    module = _module()
    spawn = ExecuteProcess(cmd=[
        sys.executable, '-c', 'raise SystemExit(7)'])
    launch_service = LaunchService()
    launch_service.include_launch_description(LaunchDescription([
        spawn,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=module._spawn_exit_handler(object(), object()))),
    ]))
    assert launch_service.run() == 1


def test_runtime_launch_binds_minimal_graph_and_scoped_contact_bridge():
    source = LAUNCH.read_text(encoding='utf-8')
    for required in (
            "executable='g005_frontier_observer'",
            "'g005_frontier_eval.launch.py'",
            "'-r -s -v2 --seed '",
            "'/g005_contacts@ros_gz_interfaces/msg/Contacts'",
            "name='g005_runtime_bridge'",
            "name='g005_contact_bridge'",
            "name='g005_spawn_robot'",
            "'ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'",
            "'robot_state_publisher'",
            'OnProcessExit',
            "LaunchConfiguration('asset_mode')",
            'ASSET_FAILURE_SIGNAL',
            'SPAWN_FAILURE_SIGNAL'):
        assert required in source
    for forbidden in ('joint_state_broadcaster', 'arm_controller',
                      'gripper_controller', 'rviz2'):
        assert forbidden not in source


def test_every_nav2_process_receives_sim_time_even_when_yaml_omits_it():
    source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')
    assert source.count(
        "parameters=[configured_params, {'use_sim_time': use_sim_time}]") == 6


def test_runtime_launch_dependencies_are_declared():
    package = PACKAGE_XML.read_text(encoding='utf-8')
    for dependency in (
            'jdamr_cube_description', 'jdamr_cube_gazebo',
            'robot_state_publisher', 'ros_gz_bridge'):
        assert f'<exec_depend>{dependency}</exec_depend>' in package
