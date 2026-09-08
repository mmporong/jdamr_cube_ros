"""Exercise opt-in parking wiring without starting ROS or moving a robot."""

import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from jdamr_cube_navigation.corridor_route import (
    CorridorRoute, parse_args, _quaternion_yaw, validate_parking_route,
)
from jdamr_cube_navigation.parking import (
    load_parking_contract, parking_controller_overrides,
)
import pytest
from rclpy.parameter import Parameter
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / 'evaluation'))
from prepare_parking_params import prepare  # noqa: E402


def _contract():
    return load_parking_contract(PACKAGE_ROOT / 'config/parking_contract.yaml')


def _document():
    return yaml.safe_load((PACKAGE_ROOT / 'config/nav2_params.yaml').read_text())


def test_generated_params_only_add_opt_in_controller_content(tmp_path):
    """Preserve costmaps, monitor, AMCL and every existing plugin exactly."""
    source = PACKAGE_ROOT / 'config/nav2_params.yaml'
    before = source.read_bytes()
    output = tmp_path / 'parking.yaml'
    evidence = prepare(source, PACKAGE_ROOT / 'config/parking_contract.yaml',
                       output)
    actual = yaml.safe_load(output.read_text())
    original = _document()
    modified = actual['controller_server']['ros__parameters']
    modified['goal_checker_plugins'].remove('parking_goal_checker')
    modified['controller_plugins'].remove('Parking')
    del modified['Parking'], modified['parking_goal_checker']
    assert actual == original
    assert source.read_bytes() == before
    assert evidence['physical_accuracy'] == 'NOT_MEASURED'
    with pytest.raises(FileExistsError):
        prepare(source, PACKAGE_ROOT / 'config/parking_contract.yaml', output)


def test_parking_bt_fixes_controller_and_checker_without_selector():
    """External selector messages cannot silently drop final orientation."""
    root = ET.parse(PACKAGE_ROOT / 'behavior_trees/navigate_to_pose_parking.xml')
    follow = root.find('.//FollowPath')
    assert follow.attrib['controller_id'] == 'Parking'
    assert follow.attrib['goal_checker_id'] == 'parking_goal_checker'
    for forbidden in ('GoalCheckerSelector', 'ControllerSelector',
                      'Spin', 'BackUp', 'Wait'):
        assert root.find('.//' + forbidden) is None


@pytest.mark.parametrize('final', [{'x': 0.0, 'y': 0.0},
                                  {'x': 0.0, 'y': 0.0, 'yaw': math.nan}])
def test_parking_requires_explicit_finite_final_yaw(final):
    """Do not silently park facing the default world x axis."""
    with pytest.raises(ValueError, match='yaw'):
        validate_parking_route({'waypoints': [final]}, _contract())


def test_parking_rejects_other_reference_frame():
    """Never compare an odom-frame target to map-frame measurements."""
    with pytest.raises(ValueError, match='frame'):
        validate_parking_route(
            {'frame_id': 'odom', 'waypoints': [{'yaw': 0.0}]}, _contract())


def test_cli_is_opt_in_and_planning_only_by_default():
    """Adding parking must not imply motion or alter existing invocations."""
    args = parse_args(['route', '--route', '/tmp/test.yaml'])
    assert not args.park_final and not args.execute
    assert args.navigation_profile == 'corridor'
    parked = parse_args(['route', '--route', '/tmp/test.yaml', '--park-final'])
    assert parked.park_final and not parked.execute


def _completed_future(value):
    return SimpleNamespace(done=lambda: True, result=lambda: value,
                           exception=lambda: None)


def _route(parking=True, confirmed=True):
    route = object.__new__(CorridorRoute)
    route.stop_requested = False
    route.waypoints = [
        {'id': 'approach', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'id': 'park', 'x': 0.8, 'y': 0.0, 'yaw': 0.5},
    ]
    route.behavior_tree = '/existing.xml'
    route.parking_behavior_tree = '/parking.xml'
    route.parking_contract = _contract() if parking else None
    route.get_logger = lambda: Mock()
    route._navigation_ready = lambda **_: True
    route._pose = lambda *_: PoseStamped()
    route._route_event = Mock()
    route._verify_parking_stop = Mock(return_value=confirmed)
    result = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(error_code=0))
    handle = SimpleNamespace(accepted=True,
                             get_result_async=lambda: _completed_future(result))
    route.navigate = Mock()
    route.navigate.send_goal_async.side_effect = (
        lambda *args, **kwargs: _completed_future(handle))
    return route


def test_only_final_waypoint_uses_parking_bt_and_confirmation():
    """Keep transit navigation unchanged and verify the final pose afterward."""
    route = _route()
    assert route.execute()
    goals = [call.args[0] for call in route.navigate.send_goal_async.call_args_list]
    assert [goal.behavior_tree for goal in goals] == [
        '/existing.xml', '/parking.xml']
    assert route._verify_parking_stop.call_count == 1
    assert route._verify_parking_stop.call_args.args[0] == 1


def test_nav2_success_is_not_parking_success_without_stationary_confirmation():
    """Fail the route when the final pose or stopped window is not confirmed."""
    assert not _route(confirmed=False).execute()


def test_original_route_never_runs_parking_verification():
    """Preserve the default position-only corridor behavior."""
    route = _route(parking=False)
    assert route.execute()
    assert not route._verify_parking_stop.called
    assert all(call.args[0].behavior_tree == '/existing.xml'
               for call in route.navigate.send_goal_async.call_args_list)


@pytest.mark.parametrize('mismatch', [None, 'Parking.stateful',
                                     'parking_goal_checker.xy_goal_tolerance',
                                     'Parking.use_collision_detection',
                                     'controller_plugins'])
def test_runtime_parameter_check_rejects_missing_or_relaxed_configuration(
        mismatch):
    """Validate effective runtime parameters before the first route goal."""
    route = _route()
    values = parking_controller_overrides(_document(), _contract())
    flat = {}
    for name, value in values.items():
        if isinstance(value, dict):
            flat.update({f'{name}.{key}': item for key, item in value.items()})
        else:
            flat[name] = value
    flat = copy.deepcopy(flat)
    if mismatch == 'controller_plugins':
        flat[mismatch] = ['FollowPath']
    elif mismatch:
        flat[mismatch] = None
    route.parking_parameters = Mock()
    route.parking_parameters.get_parameters.side_effect = lambda names: (
        _completed_future(SimpleNamespace(values=[
            Parameter(name, value=flat.get(name)).get_parameter_value()
            for name in names])))
    assert route._parking_parameters_ready() is (mismatch is None)


def test_quaternion_conversion_validates_input_and_preserves_yaw():
    """Handle the ROS quaternion boundary without accepting invalid rotations."""
    yaw_rad = 0.7
    assert _quaternion_yaw(Quaternion(
        z=math.sin(yaw_rad / 2.0), w=math.cos(yaw_rad / 2.0))) == pytest.approx(
            yaw_rad)
    with pytest.raises(ValueError):
        _quaternion_yaw(Quaternion(w=0.0))
    with pytest.raises(ValueError):
        _quaternion_yaw(Quaternion(w=math.nan))
