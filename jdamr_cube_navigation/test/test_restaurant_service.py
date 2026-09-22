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
    BLOCKED_PLAN_CODES, BoxDwell, load_service_contract, parse_args,
    select_destination, ServiceRoute,
)
from jdamr_cube_navigation.service_destinations import route_config, taught_pose
from nav2_msgs.action import ComputePathThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid
import pytest
from rclpy.parameter import Parameter
from rclpy.task import Future
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('override,expected', [(None, 'LOCALHOST'), ('SUBNET', 'SUBNET')])
def test_core_launch_resolves_discovery_default_before_environment(override, expected):
    """Execute startup declarations in order, without starting ROS processes."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
    spec = importlib.util.spec_from_file_location(
        'core_launch_defaults', PACKAGE / 'launch/onboard_nav2_core.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    if override is not None:
        context.launch_configurations['discovery_range'] = override
    for action in module.generate_launch_description().entities:
        if isinstance(action, (DeclareLaunchArgument, SetEnvironmentVariable)):
            action.execute(context)
    assert context.environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == expected


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


def test_teach_parses_gap_without_enabling_motion():
    """Manual gap provenance is a teaching option, not a driving command."""
    args = parse_args(['service', 'teach', '--registry', '/tmp/r.yaml',
                       '--table-id', 'table_02', '--pose-id', 'main', '--log', '/tmp/t.jsonl',
                       '--target-front-gap-m', '0.05', '--measured-front-gap-m', '0.06',
                       '--gap-measurement-note', 'ruler'])
    assert args.target_front_gap_m == 0.05
    assert args.measured_front_gap_m == 0.06
    assert not hasattr(args, 'execute')


def test_roundtrip_defaults_to_plan_only_and_twenty_second_box_dwell():
    """Planning is non-moving by default and retains the agreed station wait."""
    args = parse_args([
        'service', 'roundtrip', '--registry', '/tmp/r.yaml',
        '--table-id', 'table_02', '--log', '/tmp/r.jsonl'])
    assert not args.execute
    assert args.table_ids == ['table_02']
    assert args.dwell_s == 20.0
    assert args.box_timeout_s == 45.0


def test_teach_home_uses_named_dock_without_table_id():
    """Home teaching is explicit and cannot be confused with a table pose."""
    args = parse_args([
        'service', 'teach-home', '--registry', '/tmp/r.yaml',
        '--log', '/tmp/home.jsonl'])
    assert args.pose_id == 'home_dock'
    assert args.approach_offset_m == 0.7
    assert not hasattr(args, 'table_id')
    assert not hasattr(args, 'execute')


@pytest.mark.parametrize(('dwell', 'timeout'), [
    ('0', '45'), ('nan', '45'), ('20', '20'), ('20', '10'),
])
def test_roundtrip_rejects_invalid_box_wait(dwell, timeout):
    """A box wait must be positive and finish before its timeout."""
    with pytest.raises(SystemExit):
        parse_args([
            'service', 'roundtrip', '--registry', '/tmp/r.yaml',
            '--table-id', 'table_02', '--log', '/tmp/r.jsonl',
            '--dwell-s', dwell, '--box-timeout-s', timeout])


def test_roundtrip_rejects_duplicate_station_ids():
    """An accidental duplicate cannot trigger the same station twice."""
    with pytest.raises(SystemExit):
        parse_args([
            'service', 'roundtrip', '--registry', '/tmp/r.yaml',
            '--table-id', 'table_02', '--table-id', 'table_02',
            '--log', '/tmp/r.jsonl'])


def stable_box():
    return {
        'detected': True, 'stable': True, 'surface_kind': 'front',
        'front_distance_m': 0.5, 'lateral_error_m': 0.01,
        'edge_angle_deg': 1.0,
    }


def test_box_dwell_requires_continuous_fresh_front_detection():
    """A stale or lost frame resets the full dwell instead of counting gaps."""
    gate = BoxDwell(2.0, maximum_age_s=0.5)
    assert gate.observe(1.0, 1.0, stable_box())['hold_s'] == 0.0
    assert not gate.observe(2.0, 2.0, stable_box())['confirmed']
    assert gate.observe(3.0, 3.0, stable_box())['confirmed']
    assert not gate.observe(4.0, 3.0, stable_box())['confirmed']
    assert gate.observe(5.0, 5.0, stable_box())['hold_s'] == 0.0


def test_box_dwell_rejects_top_plane_and_nonfinite_geometry():
    """The mission waits for a finite front face rather than any depth plane."""
    gate = BoxDwell(1.0)
    top = {**stable_box(), 'surface_kind': 'top'}
    invalid = {**stable_box(), 'edge_angle_deg': math.nan}
    assert gate.observe(1.0, 1.0, top)['reason'] == 'box_not_stable'
    assert gate.observe(2.0, 2.0, invalid)['reason'] == 'box_observation_invalid'


def test_roundtrip_orders_destination_box_wait_and_home():
    """Home is attempted only after arrival and the complete box dwell."""
    node = route()
    node.registry['home'] = taught_pose('home_dock', (0, 0, 0), {})
    node.visit = Mock(return_value=True)
    node.wait_for_box = Mock(return_value=True)
    node.go_home = Mock(return_value=True)
    node.emit = Mock()
    assert node.roundtrip('table_01', execute=True, dwell_s=20, box_timeout_s=45)
    node.visit.assert_called_once_with('table_01', execute=True)
    node.wait_for_box.assert_called_once_with(20, 45)
    node.go_home.assert_called_once_with(execute=True)
    assert node.emit.call_args.args[0] == 'roundtrip_complete'


def test_roundtrip_visits_multiple_stations_before_one_home_return():
    """Each selected station completes its box dwell before the next goal."""
    node = route()
    node.registry['home'] = taught_pose('home_dock', (0, 0, 0), {})
    node.visit = Mock(return_value=True)
    node.wait_for_box = Mock(return_value=True)
    node.go_home = Mock(return_value=True)
    node.emit = Mock()

    assert node.roundtrip(
        ['kitchen_station', 'table_01'], execute=True,
        dwell_s=20, box_timeout_s=45)

    assert [call.args[0] for call in node.visit.call_args_list] == [
        'kitchen_station', 'table_01']
    assert node.wait_for_box.call_count == 2
    node.go_home.assert_called_once_with(execute=True)
    completed = [
        call.kwargs['station_id'] for call in node.emit.call_args_list
        if call.args[0] == 'station_complete']
    assert completed == ['kitchen_station', 'table_01']


def test_roundtrip_does_not_leave_destination_after_box_failure():
    """Missing box evidence prevents an unsupported successful return claim."""
    node = route()
    node.registry['home'] = taught_pose('home_dock', (0, 0, 0), {})
    node.visit = Mock(return_value=True)
    node.wait_for_box = Mock(return_value=False)
    node.go_home = Mock()
    node.emit = Mock()
    assert not node.roundtrip('table_01', execute=True)
    node.go_home.assert_not_called()
    assert node.emit.call_args.kwargs['phase'] == 'box_wait'


def test_go_home_plans_taught_pose_and_preserves_yaw_target():
    """A plan-only home check uses the taught pose and never starts motion."""
    node = route()
    home = taught_pose('home_dock', (-0.5, 0.4, -1.2), {},
                       approach_offset_m=0.7)
    node.registry['home'] = home
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node.plan_pose = Mock(return_value={'ok': True, 'error_code': 0})
    node.execute = Mock()
    node.emit = Mock()

    assert node.go_home(execute=False)

    node.plan_pose.assert_called_once_with(home)
    node.execute.assert_not_called()
    selected = next(
        call for call in node.emit.call_args_list
        if call.args[0] == 'home_selected')
    assert selected.kwargs['target_pose'] == [-0.5, 0.4, -1.2]
    assert node.emit.call_args.args[0] == 'home_planned_only'


def serving_route():
    """Prepare an explicitly taught home and one table without ROS hardware."""
    node = route()
    node.registry['home'] = taught_pose('home_dock', (0, 0, 0), {})
    node.registry['tables'] = [{
        'table_id': 'table_01', 'enabled': True,
        'service_poses': [taught_pose('table_01_main', (1, 0, 1.57), {})],
    }]
    return node


@pytest.mark.parametrize('failed_step', [None, 0, 1, 2, 3, 4])
def test_serving_sequence_stops_at_each_failed_phase(failed_step):
    """No later goal is dispatched after failed parking or incomplete dwell."""
    node = serving_route()
    calls = []

    def step(name):
        calls.append(name)
        return len(calls) - 1 != failed_step

    node.go_home = lambda execute: step('home')
    node.visit = lambda table_id, execute: step(table_id)
    node.wait_parked = lambda seconds: step(('wait', seconds))
    node.emit = Mock()
    assert node.serve('table_01', execute=True) is (failed_step is None)
    expected = ['home', ('wait', 5.0), 'table_01', ('wait', 5.0), 'home']
    assert calls == (expected if failed_step is None else expected[:failed_step + 1])
    assert node.emit.call_args.args[0] == (
        'serving_complete' if failed_step is None else 'serving_failed')


def test_serving_rejects_unknown_table_before_initial_home_motion():
    """A wrong destination cannot move the robot toward home first."""
    node = serving_route()
    node.go_home = Mock()
    with pytest.raises(ValueError, match='unknown table'):
        node.serve('absent', execute=True)
    node.go_home.assert_not_called()


def test_serving_plan_skips_dwell_and_never_enables_execution():
    """Preview checks endpoints and never claims a physical completed task."""
    node = serving_route()
    node.go_home = Mock(return_value=True)
    node.visit = Mock(return_value=True)
    node.wait_parked = Mock()
    node.emit = Mock()
    assert node.serve('table_01')
    assert all(call.kwargs == {'execute': False}
               for call in node.go_home.call_args_list)
    node.visit.assert_called_once_with('table_01', execute=False)
    node.wait_parked.assert_not_called()
    assert node.emit.call_args.args[0] == 'serving_plan_checks_complete'


def test_serving_operator_stop_prevents_next_goal():
    """An operator stop during dwell never launches the table transit."""
    node = serving_route()
    node.go_home = Mock(return_value=True)
    node.visit = Mock()

    def hold(_seconds):
        node.stop_requested = True
        return True

    node.wait_parked = hold
    assert not node.serve('table_01', execute=True)
    node.visit.assert_not_called()


def test_serve_cli_selects_one_table_and_defaults_to_no_motion():
    """The new workflow does not require a kitchen or pouring argument."""
    args = parse_args(['service', 'serve', '--registry', '/tmp/r.yaml',
                       '--table-id', 'table_01', '--log', '/tmp/s.jsonl'])
    assert args.command == 'serve' and not args.execute
    assert args.table_id == 'table_01'


def test_wait_parked_uses_selected_pose_and_clears_previous_deadline():
    """The dwell checks the taught yaw and does not inherit transit timeout."""
    node = serving_route()
    node.selected_pose = node.registry['tables'][0]['service_poses'][0]
    node.run_deadline_s = 1.0
    node._verify_parking_stop = Mock(return_value=True)
    assert node.wait_parked(5.0)
    assert node.run_deadline_s is None
    node._verify_parking_stop.assert_called_once_with(
        1, {'x': 1.0, 'y': 0.0, 'yaw': 1.57}, None, hold_s=5.0)


@pytest.mark.parametrize('fresh', [True, False])
def test_dwell_real_verifier_and_event_writer_without_action_handle(monkeypatch, fresh):
    """Both dwell outcomes must log without inventing a completed goal handle."""
    node = serving_route()
    node.selected_pose = node.registry['tables'][0]['service_poses'][0]
    node.parking_motion_revision = 0
    node.parking_odom = None
    node.parking_observation_diagnostics = None
    clock = {'tick': 0}
    monkeypatch.setattr(
        'jdamr_cube_navigation.corridor_route.time.monotonic',
        lambda: clock['tick'] * 0.25)

    def spin_once(_node, timeout_sec):
        if timeout_sec:
            clock['tick'] += 1
            node.parking_odom = (
                clock['tick'] * 0.25,
                SimpleNamespace(sec=clock['tick'], nanosec=0), 0.0, 0.0)

    monkeypatch.setattr(
        'jdamr_cube_navigation.corridor_route.rclpy.spin_once', spin_once)
    node._parking_observation = lambda: {
        'actual_pose': (1.0, 0.0, 1.57),
        'linear_mps': 0.0, 'angular_radps': 0.0,
        'cmd_linear_mps': 0.0, 'cmd_angular_radps': 0.0,
        'sample_age_s': 0.01 if fresh else 1.0,
    }
    assert node.wait_parked(5.0) is fresh
    records = [json.loads(line) for line in node.result_stream.getvalue().splitlines()]
    assert [record['event'] for record in records] == [
        'parked_dwell_started', 'parked_dwell_observation',
        'parked_dwell_complete' if fresh else 'parked_dwell_failed',
    ]
    assert records[1]['goal_uuid'] is None
    assert records[1]['confirmed'] is fresh
    assert records[2]['confirmation']['confirmed'] is fresh
    assert clock['tick'] * 0.25 >= 5.0


@pytest.mark.parametrize('options', [
    ['--measured-front-gap-m', '0.05'], ['--gap-measurement-note', 'ruler'],
    ['--target-front-gap-m', 'nan'], ['--target-front-gap-m', '-1'],
])
def test_invalid_gap_options_fail_before_ros_startup(options):
    """Invalid measurements must not wait for the robot to connect."""
    with pytest.raises(SystemExit) as failure:
        parse_args(['service', 'teach', '--registry', '/tmp/r.yaml',
                    '--table-id', 'table_02', '--pose-id', 'main',
                    '--log', '/tmp/t.jsonl', *options])
    assert failure.value.code == 2


def test_navigation_terminal_failure_is_propagated():
    """Do not let an unresolved action fall through to the next waypoint."""
    node = route()
    node.navigate = Mock()
    node.navigate.send_goal_async.return_value = done(handle(GoalStatus.STATUS_ABORTED))
    node.finish_navigation = Mock(return_value=False)
    with pytest.raises(RuntimeError, match='cancellation unconfirmed'):
        node.execute()
    assert node.navigate.send_goal_async.call_count == 1
    assert '"event": "arrived"' not in node.result_stream.getvalue()


@pytest.mark.parametrize('execute', [False, True])
def test_alternate_selection_retains_waypoints_and_gap_evidence(execute):
    """Plan and report the selected table's two poses without reusing the primary."""
    node = route()
    primary = taught_pose('main', (1, 2, 0), {})
    alternate = taught_pose('other', (3, 4, math.pi / 2), {}, priority=2,
                            target_front_gap_m=0.05, measured_front_gap_m=0.06,
                            gap_measurement_note='ruler')
    node.registry['tables'] = [{'table_id': 'table_01', 'enabled': True,
                                'service_poses': [primary, alternate]}]
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    failed = ComputePathThroughPoses.Result()
    failed.error_code = ComputePathThroughPoses.Result.GOAL_OCCUPIED
    successful = ComputePathThroughPoses.Result()
    successful.path.poses = [PoseStamped()]
    node.compute = Mock()
    node.compute.send_goal_async.side_effect = [
        done(SimpleNamespace(accepted=True, get_result_async=lambda: done(
            SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=failed)))),
        done(SimpleNamespace(accepted=True, get_result_async=lambda: done(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=successful)))),
    ]
    expected = route_config(node.registry, alternate)['waypoints']

    def dispatch():
        assert node.waypoints == expected
        assert node.pose_id == 'other'
        return True

    node.execute = Mock(side_effect=dispatch)
    assert node.visit('table_01', execute=execute)
    assert node.execute.call_count == int(execute)
    records = [json.loads(line) for line in node.result_stream.getvalue().splitlines()]
    selected = next(record for record in records if record['event'] == 'selected')
    assert selected['waypoints'] == expected
    assert selected['front_gap']['teaching_measured_m'] == 0.06
    assert records[-1]['event'] == ('arrived' if execute else 'planned_only')
    assert records[-1]['front_gap']['arrival_verification'] == 'NOT_MEASURED'


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


@pytest.mark.parametrize('discovery_range', ['SUBNET', 'LOCALHOST'])
def test_service_launch_adds_parking_without_changing_costmaps(monkeypatch, discovery_range):
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
        'navigation_profile': 'new_base_candidate', 'use_sim_time': 'false',
        'discovery_range': discovery_range})
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
        assert arguments['discovery_range'].perform(context) == discovery_range
    finally:
        generated.unlink()


def test_service_launch_defaults_to_physical_sensor_discovery(monkeypatch):
    """Use the same discovery scope as the physical keepout wrapper."""
    from launch.actions import DeclareLaunchArgument
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location(
        'service_launch_defaults', PACKAGE / 'launch/restaurant_service.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(PACKAGE))
    description = module.generate_launch_description()
    declaration = next(action for action in description.entities
                       if isinstance(action, DeclareLaunchArgument)
                       and action.name == 'discovery_range')
    context = LaunchContext()
    declaration.execute(context)
    assert context.launch_configurations['discovery_range'] == 'SUBNET'


def test_service_launch_keeps_box_observer_explicit():
    """Ordinary table navigation stays light; round trips opt into RGB-D."""
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location(
        'service_launch_box_observer',
        PACKAGE / 'launch/restaurant_service.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    declaration = next(
        action for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
        and action.name == 'use_box_observer')
    assert declaration.default_value[0].text == 'false'
