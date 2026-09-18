"""Exercise service routing with ROS messages and in-memory action peers."""

import importlib.util
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from jdamr_cube_navigation.parking import load_parking_contract
from jdamr_cube_navigation.restaurant_service import (
    BLOCKED_PLAN_CODES, load_service_contract, parse_args, select_destination, ServiceRoute,
)
from nav2_msgs.action import ComputePathThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid
import pytest
from rclpy.parameter import Parameter
from rclpy.task import Future
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def done(value):
    """Return a real completed rclpy future."""
    future = Future()
    future.set_result(value)
    return future


def route():
    """Create the service adapter without touching DDS or robot hardware."""
    node = object.__new__(ServiceRoute)
    node.registry = {'frame_id': 'map'}
    node.parking_contract = load_parking_contract(PACKAGE / 'config/parking_contract.yaml')
    node.get_logger = lambda: Mock()
    node.table_id, node.pose_id = 'table_01', 'main'
    node.result_stream = io.StringIO()
    node.stop_requested = False
    node.active_handle = None
    node.pending_goal = None
    node.navigation_result = None
    node.navigation_uuid = None
    node.amcl_yaw_covariance_rad2 = 0.01
    node.amcl_covariance = (0.001, 0.001)
    node.service_contract = load_service_contract(
        PACKAGE / 'config/restaurant_service_contract.yaml')
    node.live_grids = {}
    node.expected_grids = {}
    node.map_mismatch = None
    node.confirmation = None
    node.start_index = 0
    node.navigation_profile = 'obstacle_base_candidate'
    node.behavior_tree, node.parking_behavior_tree = 'transit.xml', 'parking.xml'
    node.config = {'waypoints': [
        {'id': 'approach', 'x': 0.5, 'y': 0.0, 'yaw': 0.2},
        {'id': 'main', 'x': 1.0, 'y': 0.0, 'yaw': 0.2}]}
    node.waypoints = node.config['waypoints']
    node._pose = lambda *_: PoseStamped()
    node._navigation_ready = lambda **_: True
    node._guard_failure = lambda *_: None
    return node


def handle(status=GoalStatus.STATUS_SUCCEEDED, error_code=0, future=None):
    """Build one accepted Nav2 response with an identifiable goal."""
    result = NavigateToPose.Result()
    result.error_code = error_code
    wrapped = SimpleNamespace(status=status, result=result)
    value = SimpleNamespace(
        accepted=True, goal_id=SimpleNamespace(uuid=bytes(range(16))),
        get_result_async=lambda: future if future is not None else done(wrapped),
        cancel_goal_async=Mock(return_value=done(SimpleNamespace(goals_canceling=[]))))
    return value


@pytest.mark.parametrize('failure,expected_calls', [
    (ComputePathThroughPoses.Result.GOAL_OCCUPIED, 2),
    (ComputePathThroughPoses.Result.NO_VALID_PATH, 2),
    (ComputePathThroughPoses.Result.TF_ERROR, 1),
    (ComputePathThroughPoses.Result.START_OCCUPIED, 1),
    (ComputePathThroughPoses.Result.TIMEOUT, 1),
    (None, 1),
])
def test_alternate_only_for_destination_path_obstruction(failure, expected_calls):
    """Sensor, TF or start failures must not dispatch alternate targets."""
    poses = [{'id': 'main'}, {'id': 'alternate'}, {'id': 'not_allowed'}]
    planner = Mock(side_effect=[{'ok': False, 'error_code': failure}, {'ok': True}])
    chosen, attempts = select_destination(poses, planner)
    assert len(attempts) == planner.call_count == expected_calls
    assert chosen == (poses[1] if expected_calls == 2 else None)


def test_two_blocked_poses_end_without_retry():
    """Bound retries to the registered primary and one alternate."""
    planner = Mock(return_value={'ok': False, 'error_code': next(iter(BLOCKED_PLAN_CODES))})
    chosen, attempts = select_destination([{'id': 'a'}, {'id': 'b'}], planner)
    assert chosen is None and len(attempts) == 2


def test_plan_only_is_default_and_teaching_requires_named_pose():
    """Parsing a goal does not silently authorize movement."""
    args = parse_args(['service', 'go', '--registry', '/tmp/r.yaml',
                       '--table-id', 'table_02', '--log', '/tmp/r.jsonl'])
    assert not args.execute
    assert args.table_id == 'table_02'


@pytest.mark.parametrize('confirmed', [False, True])
def test_final_action_uses_parking_and_requires_confirmation(confirmed):
    """Two Nav2 successes alone cannot confirm service arrival."""
    node = route()
    node.navigate = Mock()
    node.navigate.send_goal_async.side_effect = lambda *_, **__: done(handle())
    node._verify_parking_stop = Mock(return_value=confirmed)
    assert node.execute() is confirmed
    goals = [call.args[0] for call in node.navigate.send_goal_async.call_args_list]
    assert [goal.behavior_tree for goal in goals] == ['transit.xml', 'parking.xml']
    assert node._verify_parking_stop.call_count == 1


def test_action_failure_does_not_send_parking_goal():
    """Transit failure ends execution without crossing to the final waypoint."""
    node = route()
    node.navigate = Mock()
    node.navigate.send_goal_async.return_value = done(handle(GoalStatus.STATUS_ABORTED))
    node._verify_parking_stop = Mock()
    assert not node.execute()
    assert node.navigate.send_goal_async.call_count == 1
    node._verify_parking_stop.assert_not_called()


def test_stop_during_action_cancels_and_waits_for_terminal(monkeypatch):
    """A cancel acknowledgment is not the terminal action result."""
    node = route()
    pending = Future()
    accepted = handle(future=pending)
    node.navigate = Mock()
    node.navigate.send_goal_async.return_value = done(accepted)

    def spin(_node, timeout_sec):
        node.stop_requested = True
        if accepted.cancel_goal_async.called:
            pending.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))

    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    assert not node.execute()
    accepted.cancel_goal_async.assert_called_once()
    assert node.active_handle is None
    assert 'navigation_terminal' in node.result_stream.getvalue()


def test_late_goal_acceptance_is_canceled(monkeypatch):
    """Shutdown drains a pending send and cancels the accepted goal."""
    node = route()
    node.stop_requested = True
    node.pending_goal = Future()
    result = Future()
    accepted = handle(future=result)

    def spin(_node, timeout_sec):
        if node.pending_goal is not None and not node.pending_goal.done():
            node.pending_goal.set_result(accepted)
        elif accepted.cancel_goal_async.called:
            result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))

    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    assert node.finish_navigation()
    accepted.cancel_goal_async.assert_called_once()
    assert node.active_handle is None


def test_uuid_cancellation_confirms_our_goal_without_acceptance(monkeypatch):
    """Use an exact goal UUID, never a cancel-all request, on lost acceptance."""
    node = route()
    node.navigation_uuid = NavigateToPose.Impl.SendGoalService.Request().goal_id
    node.navigation_uuid.uuid = list(range(16))
    node.cancel_navigation = Mock()
    node.query_navigation = Mock()
    node.cancel_navigation.call_async.return_value = done(SimpleNamespace())
    response = NavigateToPose.Impl.GetResultService.Response()
    response.status = GoalStatus.STATUS_CANCELED
    node.query_navigation.call_async.return_value = done(response)
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.rclpy.spin_once', lambda *_, **__: None)
    assert node._cancel_navigation_uuid()
    request = node.cancel_navigation.call_async.call_args.args[0]
    assert bytes(request.goal_info.goal_id.uuid) == bytes(range(16))
    assert request.goal_info.stamp.sec == 0
    assert 'navigation_terminal' in node.result_stream.getvalue()


def test_amcl_stale_after_motion_is_blocked(monkeypatch):
    """Service mode retains the existing moving-base localization timeout."""
    node = route()
    del node._guard_failure
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.CorridorRoute._guard_failure',
        lambda *_: None)
    node.amcl_covariance = (0.01, 0.01)
    node.amcl_seen = 0.0
    node.amcl_motion_distance_m = 0.3
    node.amcl_motion_rotation_rad = 0.0
    node.revisit_motion_min_distance_m = 0.25
    node.revisit_motion_min_rotation_rad = 0.5
    node.revisit_motion_amcl_freshness_s = 25.0
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.time.monotonic', lambda: 26.0)
    assert node._guard_failure(False) == 'AMCL stale after motion'
    node.amcl_seen = 20.0
    assert node._guard_failure(False) is None
    node.amcl_yaw_covariance_rad2 = math.nan
    assert node._guard_failure(False) == 'AMCL yaw covariance invalid'
    node.amcl_yaw_covariance_rad2 = 1.0
    assert node._guard_failure(False) == 'AMCL yaw covariance high'


def test_planner_cancellation_without_error_is_not_success():
    """Even a nonempty path needs a successful terminal action status."""
    node = route()
    result = ComputePathThroughPoses.Result()
    result.path.poses = [PoseStamped()]
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_CANCELED, result=result)
    node.compute = Mock()
    node.compute.send_goal_async.return_value = done(SimpleNamespace(
        accepted=True, get_result_async=lambda: done(wrapped)))
    planned = node.plan_pose({'id': 'main', 'x_m': 1.0, 'y_m': 2.0,
                             'yaw_rad': 0.4, 'approach_offset_m': 0.5})
    assert not planned['ok']
    goal = node.compute.send_goal_async.call_args.args[0]
    assert len(goal.goals) == 2 and goal.planner_id == 'GridBased'


def test_wrong_live_map_prevents_any_goal(monkeypatch):
    """Check live server assets before planning or movement."""
    node = route()
    node.registry = {'map': {'yaml_path': 'expected'}, 'keepout': {'yaml_path': 'mask'}}
    node.live_grids = {'map': 'expected', 'keepout': 'mask'}
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.map_grid_signature', lambda x: x)
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.validate_registry', lambda _: None)
    client = Mock()
    client.get_parameters.return_value = done(SimpleNamespace(values=[
        Parameter('yaml_filename', value='/wrong/map.yaml').get_parameter_value()]))
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.AsyncParameterClient',
                        lambda *_: client)
    verify = Mock(side_effect=ValueError('map hash mismatch'))
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.verify_identity', verify)
    with pytest.raises(ValueError, match='hash mismatch'):
        node.verify_live_maps()
    assert verify.call_args.args[1] == '/wrong/map.yaml'


def test_live_map_reload_is_detected_even_with_unchanged_parameters():
    """A changed OccupancyGrid invalidates the session independently of YAML names."""
    node = route()
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 2
    grid.info.resolution = 0.05
    grid.info.origin.orientation.w = 1.0
    grid.data = [0, 100, -1, 0]
    node._map_callback('map', grid)
    node.expected_grids['map'] = node.live_grids['map']
    grid.header.stamp.sec = 20
    node._map_callback('map', grid)
    assert not node.stop_requested
    grid.data = [100, 100, -1, 0]
    node._map_callback('map', grid)
    assert node.stop_requested
    assert 'live map changed' in node.map_mismatch


def test_localization_limits_use_squared_si_units():
    """The candidate confidence thresholds have explicit independent units."""
    contract = load_service_contract(PACKAGE / 'config/restaurant_service_contract.yaml')
    assert contract['max_x_covariance_m2'] == pytest.approx(0.1 ** 2)
    assert contract['max_y_covariance_m2'] == pytest.approx(0.1 ** 2)
    assert contract['max_yaw_covariance_rad2'] == pytest.approx(math.radians(10.0) ** 2)


@pytest.mark.parametrize('sequence,confirmed', [('steady', True), ('moving', False)])
def test_stationary_teaching_uses_fresh_unique_samples(monkeypatch, sequence, confirmed):
    """Exercise real hold logic instead of replacing its verdict."""
    node = route()
    clock = {'now': 10.0, 'step': 0}
    node.parking_motion_revision = 0
    node.parking_command = None
    node.parking_odom = None
    node.amcl_covariance = (0.01, 0.01)
    node._guard_failure = lambda **_: None
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=round(clock['now'] * 1e9)))
    transform = TransformStamped()
    transform.transform.translation.x = 2.0
    transform.transform.translation.y = 1.0
    transform.transform.rotation.w = 1.0
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = transform

    def spin(_node, timeout_sec):
        clock['step'] += 1
        clock['now'] = 10.0 + clock['step'] * 0.1
        stamp = transform.header.stamp
        stamp.sec = int(clock['now'])
        stamp.nanosec = round((clock['now'] - stamp.sec) * 1e9)
        node.parking_odom = (clock['now'], stamp, 0.0 if sequence == 'steady' else 0.2, 0.0)

    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.time.monotonic', lambda: clock['now'])
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    if confirmed:
        pose, evidence = node.capture_stationary_pose(timeout_s=2.0)
        assert pose == (2.0, 1.0, 0.0)
        assert evidence['hold_s'] >= 1.0
        assert evidence['command_observed'] is False
        assert evidence['physical_accuracy'] == 'NOT_MEASURED'
    else:
        with pytest.raises(RuntimeError, match='stationary teaching unavailable'):
            node.capture_stationary_pose(timeout_s=2.0)


def test_arrival_evidence_retains_final_errors():
    """Persist the actual parking verdict in a parseable JSONL event."""
    node = route()
    node._route_event('parking_estimate_confirmed', 1, handle(),
                      confirmed=True, position_error_m=0.034,
                      yaw_error_rad=math.radians(2.0), hold_s=1.1)
    event = json.loads(node.result_stream.getvalue())
    assert event['table_id'] == 'table_01'
    assert event['position_error_m'] == 0.034
    assert event['physical_accuracy'] == 'NOT_MEASURED'
    assert node.confirmation['confirmed']


def test_service_launch_adds_parking_without_changing_costmaps(monkeypatch):
    """Construct launch parameters without starting nodes or moving a robot."""
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location(
        'service_launch', PACKAGE / 'launch/restaurant_service.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(PACKAGE))
    monkeypatch.setattr(module, 'load_registry', lambda _: {
        'map': {'yaml_path': '/maps/new_base_room.yaml'},
        'keepout': {'yaml_path': '/maps/new_base_mask.yaml'}})
    context = LaunchContext()
    source = PACKAGE / 'config/new_base_nav2_params.yaml'
    context.launch_configurations.update({
        'registry': '/registry.yaml', 'params_file': str(source),
        'navigation_profile': 'new_base_candidate', 'use_sim_time': 'false'})
    original = yaml.safe_load(source.read_text())
    actions = module._configure(context)
    arguments = dict(actions[1].launch_arguments)
    generated = Path(arguments['params_file'])
    try:
        output = yaml.safe_load(generated.read_text())
        controller = output['controller_server']['ros__parameters']
        assert controller['Parking']['desired_linear_vel'] == 0.08
        assert controller['Parking']['use_collision_detection']
        assert controller['parking_goal_checker']['xy_goal_tolerance'] == 0.05
        controller['controller_plugins'].remove('Parking')
        controller['goal_checker_plugins'].remove('parking_goal_checker')
        del controller['Parking'], controller['parking_goal_checker']
        assert output == original
        assert arguments['map'] == '/maps/new_base_room.yaml'
    finally:
        generated.unlink()
