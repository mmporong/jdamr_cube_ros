"""Verify bounded table transit and face-based parking without hardware."""

import contextlib
from copy import deepcopy
import inspect
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from jdamr_cube_navigation import box_service
from jdamr_cube_navigation.box_service import BoxServiceRoute, parse_args
from jdamr_cube_navigation.corridor_route import CorridorRoute
from jdamr_cube_navigation.docking_stop_profile import apply_docking_stop_profile
from nav2_msgs.action import ComputePathThroughPoses
from nav_msgs.msg import Path as RosPath
import pytest
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.task import Future
from rclpy.time import Time
import yaml


ROOT = Path(__file__).resolve().parents[2]
PARAMS = ROOT / 'jdamr_cube_navigation/config/new_base_nav2_params.yaml'
GEOMETRY = ROOT / 'jdamr_cube_description/config/new_base_geometry.yaml'
BOX_CONTRACT = ROOT / 'jdamr_cube_navigation/config/box_parking_contract.yaml'
CAMERA_MOUNT = ROOT / 'jdamr_cube_navigation/config/camera_mount_derived_20261012.yaml'
TABLE_EDGE_ALIGNMENT_FIXTURE = (
    ROOT / 'jdamr_cube_navigation/test/fixtures/table_edge_alignment_20261013.json')


@pytest.fixture
def mission(monkeypatch):
    """Provide synthetic action peers and a map-bound observation route."""
    route = {'frame_id': 'map', 'map_yaml': 'map', 'keepout_mask_yaml': 'mask',
             'start_pose': {'x': 0.0, 'y': 0.0}, 'max_route_start_distance_m': 0.3,
             'waypoints': [{'id': 'exit'}, {'id': 'observation'}]}
    monkeypatch.setattr(box_service, 'load_route', lambda _: route)
    monkeypatch.setattr(box_service, 'verify_identity', Mock())
    node = object.__new__(BoxServiceRoute)
    node.registry = {'map': {}, 'keepout': {}}
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node._operator_link_ready = Mock(return_value=True)
    for method in ('verify_live_maps', 'emit'):
        setattr(node, method, Mock())
    for method in ('wait_until_ready', '_parking_parameters_ready', 'preflight', 'execute'):
        setattr(node, method, Mock(return_value=True))
    node._precision_collision_ready = Mock(return_value=True)
    # The straight odom approach is exercised on its own; here it succeeds and
    # the gap is judged on the map as before.
    node._straight_final_approach = Mock(return_value=(True, None))
    node.capture_stationary_pose = Mock(side_effect=[((0, 0, 0), {}), ((0.885, 0, 0), {})])
    node.plan_pose = Mock(return_value={'ok': True})
    node.search_rotation = Mock(return_value=True)
    node.confirmation = {'confirmed': True, 'physical_accuracy': 'NOT_MEASURED'}
    node.parking_contract = {'yaw_tolerance_rad': math.radians(1)}
    target = {'x_m': 0.885, 'y_m': 0.0, 'yaw_rad': 0.0,
              'face_center_map_xy_m': (1.0, 0.0), 'outward_normal_map_xy': (-1.0, 0.0)}
    node.observe_target = Mock(return_value=target)
    return node


def invoke(node, execute=True, *, search=False):
    """Run the table attempt with explicit execution semantics."""
    return node.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540}},
        'table_01', (1, 0), 0.6, execute=execute,
        candidate_trial=execute, search=search)


def test_plan_only_does_not_observe_or_move(mission):
    assert invoke(mission, execute=False)
    mission.execute.assert_not_called()
    mission.observe_target.assert_not_called()
    mission._precision_collision_ready.assert_not_called()


def test_transit_alignment_final_sequence(mission):
    assert invoke(mission)
    # The final approach is the straight odom leg, not a planned execute.
    assert mission.execute.call_count == 2
    assert [call.kwargs for call in mission.execute.call_args_list] == [
        {'final_parking': False}, {'final_parking': False, 'alignment': True, 'face': True}]
    mission._straight_final_approach.assert_called_once()
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [0.45, 0.05]
    # The final plan only proves the way is free: one 5 cm costmap cell + 1 cm.
    assert [call.kwargs.get('end_tolerance_m') for call in mission.plan_pose.call_args_list] == [
        0.05, 0.06]
    assert mission.emit.call_args.args == ('box_approach_finished',)
    assert mission.emit.call_args.kwargs['estimated_front_gap_m'] == pytest.approx(0.05)
    assert mission.emit.call_args.kwargs['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'
    assert mission.emit.call_args.kwargs['final_depth_measurement'] is False


def _table_edge_final_target(face_x):
    """Return a map-frame table edge used by final measurement tests."""
    return {
        'x_m': face_x - 0.115,
        'y_m': 0.0,
        'yaw_rad': 0.0,
        'face_center_map_xy_m': (face_x, 0.0),
        'outward_normal_map_xy': (-1.0, 0.0),
    }


def _run_table_edge_final_measurement(mission, fresh_target):
    """Run table-edge parking with a distinct post-motion observation."""
    mission.surface_mode = 'table_edge'
    before = _table_edge_final_target(0.97)
    mission.observe_target.side_effect = [before, before, fresh_target]

    def stale_odom_transform(*_args):
        raise AssertionError('post-motion table edge reused frozen odom transform')

    mission._straight_final_approach.return_value = (True, stale_odom_transform)
    return invoke(mission)


def test_table_edge_final_depth_rejects_odom_success_at_eight_centimetres(mission):
    assert not _run_table_edge_final_measurement(
        mission, _table_edge_final_target(1.03))

    result = mission.emit.call_args
    assert result.args == ('box_gap_not_confirmed',)
    assert result.kwargs['estimated_nearest_front_gap_m'] == pytest.approx(0.08)
    assert result.kwargs['final_depth_measurement'] is True
    assert result.kwargs['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'
    assert mission.observe_target.call_count == 3


def test_table_edge_final_depth_confirms_fresh_five_centimetres_in_map(mission):
    assert _run_table_edge_final_measurement(
        mission, _table_edge_final_target(1.0))

    result = mission.emit.call_args
    assert result.args == ('box_approach_finished',)
    assert result.kwargs['estimated_nearest_front_gap_m'] == pytest.approx(0.05)
    assert result.kwargs['final_depth_measurement'] is True
    assert mission.selected_pose == {
        'id': 'final_approach', 'x_m': 0.885, 'y_m': 0.0, 'yaw_rad': 0.0}
    assert mission.last_box_face == {
        'face_center_map_xy_m': [1.0, 0.0],
        'outward_normal_map_xy': [-1.0, 0.0],
    }
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [
        0.45, 0.05, 0.05]


def test_table_edge_final_depth_observation_failure_has_no_success(mission):
    mission.surface_mode = 'table_edge'
    before = _table_edge_final_target(0.97)
    mission.observe_target.side_effect = [
        before,
        before,
        box_service.BoxObservationUnavailable(
            'no_stable_table_edge', retryable=True),
    ]

    assert not invoke(mission)

    result = mission.emit.call_args
    assert result.args == ('failed',)
    assert result.kwargs == {
        'phase': 'final_depth_measurement',
        'reason': 'no_stable_table_edge',
        'final_depth_measurement': False,
    }
    assert not any(call.args == ('box_approach_finished',)
                   for call in mission.emit.call_args_list)
    assert mission.observe_target.call_count == 3


def test_one_centimetre_table_uses_frozen_face_when_fresh_depth_is_occluded(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.observer_stable_frames = 8
    target = _table_edge_final_target(1.0)
    mission.observe_target.side_effect = [
        target,
        target,
        box_service.BoxObservationUnavailable(
            'stable detected table_edge surface is required', retryable=True,
            fresh_frames=8, required_fresh_frames=8,
            fresh_rejection_reasons={'no_table_edge_candidate'}),
    ]
    mission._straight_final_approach.return_value = (
        True, lambda x, y, yaw: (x, y, yaw))
    mission.capture_stationary_pose.side_effect = [
        ((0.0, 0.0, 0.0), {}), ((0.925, 0.0, 0.0), {})]
    mission._odom_pose = Mock(return_value=(0.925, 0.0, 0.0))

    assert invoke(mission)

    result = mission.emit.call_args
    assert result.args == ('box_approach_finished',)
    assert result.kwargs['confirmation_basis'] == 'frozen_table_edge_odom_stop'
    assert result.kwargs['internal_estimate_only'] is True
    assert result.kwargs['final_depth_measurement'] is False
    assert result.kwargs['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'
    assert mission.last_box_face == {
        'reference_frame': 'odom',
        'face_center_xy_m': [1.0, 0.0],
        'outward_normal_xy': [-1.0, 0.0],
    }


def test_occluded_five_millimetre_correction_reuses_original_odom_face(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    target = _table_edge_final_target(1.0)

    def occluded():
        return box_service.BoxObservationUnavailable(
            'stable detected table_edge surface is required', retryable=True,
            fresh_frames=8, required_fresh_frames=8,
            fresh_rejection_reasons={'no_table_edge_candidate'})
    mission.observe_target.side_effect = [target, target, occluded(), occluded()]
    turn = math.radians(30.0)

    def original_to_odom(x, y, yaw):
        return (2.0 + math.cos(turn) * x - math.sin(turn) * y,
                -1.0 + math.sin(turn) * x + math.cos(turn) * y,
                yaw + turn)

    mission._straight_final_approach.return_value = (True, original_to_odom)
    mission.capture_stationary_pose.side_effect = [
        ((0.0, 0.0, 0.0), {}),
        ((20.0, -10.0, math.radians(25.0)), {}),
        ((20.02, -10.01, math.radians(17.0)), {}),
        ((19.98, -9.99, math.radians(12.0)), {}),
    ]
    face_x, face_y, _ = original_to_odom(1.0, 0.0, 0.0)
    outward = (-math.cos(turn), -math.sin(turn))

    def odom_pose(gap_m):
        centre_distance_m = 0.065 + gap_m
        return (face_x + outward[0] * centre_distance_m,
                face_y + outward[1] * centre_distance_m, turn)

    mission._odom_pose = Mock(side_effect=[
        odom_pose(0.015), odom_pose(0.015), odom_pose(0.010)])
    mission._drive_zero_turn = Mock(return_value=0.005)
    mission._frozen_in_odom = Mock(
        side_effect=AssertionError('correction re-froze the stale map face'))

    assert invoke(mission)

    mission._straight_final_approach.assert_called_once()
    mission._frozen_in_odom.assert_not_called()
    mission._drive_zero_turn.assert_called_once_with(
        pytest.approx(0.005), precision=True, hold_heading=True)
    result = mission.emit.call_args
    assert result.args == ('box_approach_finished',)
    assert result.kwargs['estimated_nearest_front_gap_m'] == pytest.approx(0.01)
    assert result.kwargs['confirmation_basis'] == 'frozen_table_edge_odom_stop'


@pytest.mark.parametrize(
    'kwargs', [
        {'latency_limited': True, 'fresh_frames': 8,
         'fresh_rejection_reasons': {'no_table_edge_candidate'}},
        {'fresh_frames': 7, 'fresh_rejection_reasons': {'no_table_edge_candidate'}},
        {'fresh_frames': 8, 'fresh_rejection_reasons': {'no_table_edge_candidate'},
         'latency_reasons': {'observer_silent'}},
        {'fresh_frames': 8, 'fresh_rejection_reasons': {'stale_or_future_depth'}},
    ])
def test_one_centimetre_table_never_uses_odom_for_unhealthy_depth(mission, kwargs):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.observer_stable_frames = 8
    target = _table_edge_final_target(1.0)
    mission.observe_target.side_effect = [
        target,
        target,
        box_service.BoxObservationUnavailable(
            'stable detected table_edge surface is required', retryable=True,
            required_fresh_frames=8, **kwargs),
    ]
    mission._straight_final_approach.return_value = (
        True, lambda x, y, yaw: (x, y, yaw))

    assert not invoke(mission)
    assert mission.emit.call_args.args == ('failed',)
    assert mission.emit.call_args.kwargs['phase'] == 'final_depth_measurement'


def test_transit_failure_never_observes_or_dispatches_alignment(mission):
    mission.execute.return_value = False
    assert not invoke(mission)
    mission.observe_target.assert_not_called()


def test_resume_observation_skips_transit_but_keeps_both_parking_phases(mission):
    mission.config = {}
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 1.0), {}), ((0.885, 0, 0), {})]
    original = box_service.load_route('route')
    original['waypoints'][-1].update(x=0, y=0)
    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540}},
        'table_01', (1, 0), 0.6, execute=True,
        candidate_trial=True, resume_at_observation=True)
    assert mission.execute.call_count == 1
    mission._straight_final_approach.assert_called_once()
    mission.preflight.assert_not_called()
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [0.45, 0.05]
    mission.verify_live_maps.assert_called()
    mission._precision_collision_ready.assert_called_once()


def test_resume_final_approach_skips_transit_and_face_alignment(mission):
    mission.surface_mode = 'table_edge'
    original = box_service.load_route('route')
    original['waypoints'][-1].update(x=0, y=0)
    before = _table_edge_final_target(0.97)
    fresh = _table_edge_final_target(1.0)
    mission.observe_target.side_effect = [before, fresh]

    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540}},
        'table_01', (1, 0), 0.6, execute=True, candidate_trial=True,
        resume_at_observation=True, resume_at_final_approach=True)

    mission.preflight.assert_not_called()
    mission.execute.assert_not_called()
    mission._straight_final_approach.assert_called_once_with(before, 0.065)
    assert mission.observe_target.call_count == 2
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [0.05, 0.05]
    mission.plan_pose.assert_called_once_with(
        {**before, 'id': 'final_approach', 'priority': 1, 'approach_offset_m': 0.5},
        single=True, end_tolerance_m=box_service.FINAL_PLAN_END_TOLERANCE_M)


def test_resume_final_approach_defaults_off(mission):
    assert inspect.signature(BoxServiceRoute.visit_observed_box).parameters[
        'resume_at_final_approach'].default is False
    assert invoke(mission)
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [0.45, 0.05]


def test_resume_observation_rejects_robot_outside_observation_region(mission):
    original = box_service.load_route('route')
    original['waypoints'][-1].update(x=1, y=0)
    with pytest.raises(RuntimeError, match='not at the reached observation'):
        mission.visit_observed_box(
            'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                          'wheel_outer_width': {'value': 0.540}},
            'table_01', (1, 0), 0.6, execute=True,
            candidate_trial=True, resume_at_observation=True)
    mission.execute.assert_not_called()
    mission.observe_target.assert_not_called()


def test_missing_face_has_no_parking_dispatch(mission):
    mission.observe_target.side_effect = RuntimeError('no stable face')
    with pytest.raises(RuntimeError, match='no stable face'):
        invoke(mission)
    assert mission.execute.call_count == 1
    mission.plan_pose.assert_not_called()


def test_opt_in_search_rotates_once_then_resumes_parking(mission):
    target = mission.observe_target.return_value
    mission.observe_target.side_effect = [
        box_service.BoxObservationUnavailable('no_stable_box', retryable=True),
        target,
        target,
    ]
    assert invoke(mission, search=True)
    mission.search_rotation.assert_called_once_with(math.radians(30.0))
    assert mission.execute.call_count == 2
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [
        0.45, 0.45, 0.05]


def test_search_budget_stops_after_one_full_turn(mission):
    mission.observe_target.side_effect = lambda *_args, **_kwargs: (
        _raise_observation('no_stable_box', retryable=True))
    with pytest.raises(RuntimeError, match='search exhausted after 12 rotations'):
        invoke(mission, search=True)
    assert mission.search_rotation.call_count == 12
    assert sum(call.args[0] for call in mission.search_rotation.call_args_list) == (
        pytest.approx(math.tau))
    exhausted = [call for call in mission.emit.call_args_list
                 if call.args == ('box_search_exhausted',)]
    assert len(exhausted) == 1
    assert exhausted[0].kwargs['search_steps_used'] == 12
    assert exhausted[0].kwargs['search_cumulative_yaw_rad'] == pytest.approx(
        math.tau)


def _raise_observation(reason, *, retryable):
    raise box_service.BoxObservationUnavailable(reason, retryable=retryable)


def test_search_does_not_rotate_for_sensor_or_geometry_failure(mission):
    mission.observe_target.side_effect = (
        box_service.BoxObservationUnavailable(
            'LiDAR scan is stale or future-dated', retryable=False))
    with pytest.raises(RuntimeError, match='LiDAR scan is stale'):
        invoke(mission, search=True)
    mission.search_rotation.assert_not_called()
    failure = [call for call in mission.emit.call_args_list
               if call.args == ('box_observation_failed',)][0]
    assert failure.kwargs['retryable'] is False


def test_final_approach_observation_loss_never_rotates_near_box(mission):
    target = mission.observe_target.return_value
    mission.observe_target.side_effect = [
        target,
        box_service.BoxObservationUnavailable('no_stable_box', retryable=True),
    ]
    with pytest.raises(RuntimeError, match='no_stable_box'):
        invoke(mission, search=True)
    mission.search_rotation.assert_not_called()
    assert mission.execute.call_count == 2


def test_failed_search_rotation_stops_without_repeating(mission):
    mission.observe_target.side_effect = (
        box_service.BoxObservationUnavailable('no_stable_box', retryable=True))
    mission.search_rotation.return_value = False
    mission.last_search_failure_reason = 'nav2_spin_failed'
    with pytest.raises(RuntimeError, match='search rotation failed'):
        invoke(mission, search=True)
    mission.search_rotation.assert_called_once_with(math.radians(30.0))
    assert mission.observe_target.call_count == 1
    failure = [call for call in mission.emit.call_args_list
               if call.args == ('failed',)][-1]
    assert failure.kwargs['reason'] == 'nav2_spin_failed'


def test_failed_search_rotation_reports_observation_reason(mission):
    mission.observe_target.side_effect = (
        box_service.BoxObservationUnavailable('no_stable_box', retryable=True))
    mission.search_rotation.return_value = False
    mission.last_search_failure_reason = 'rotation_observation_not_confirmed'

    with pytest.raises(RuntimeError, match='search rotation failed'):
        invoke(mission, search=True)

    failure = [call for call in mission.emit.call_args_list
               if call.args == ('failed',)][-1]
    assert failure.kwargs['reason'] == 'rotation_observation_not_confirmed'


def test_collision_blocked_search_can_reposition_once_and_continue(mission):
    target = mission.observe_target.return_value
    mission.observe_target.side_effect = [
        box_service.BoxObservationUnavailable('no_stable_box', retryable=True), target, target]
    mission.search_rotation.return_value = False
    mission._reposition_search = Mock(return_value=True)
    assert invoke(mission, search=True)
    mission._reposition_search.assert_called_once()


@pytest.mark.parametrize('error_name', ['COLLISION_AHEAD', 'TIMEOUT'])
def test_search_reposition_requires_recoverable_result_and_valid_path(mission, error_name):
    from nav2_msgs.action import Spin
    mission.last_search_error_code = Spin.Result.TF_ERROR
    assert not mission._reposition_search()
    mission.plan_pose.assert_not_called()
    mission.last_search_error_code = getattr(Spin.Result, error_name)
    mission.plan_pose.side_effect = [{'ok': False}, {'ok': True}]
    assert mission._reposition_search()
    assert mission.plan_pose.call_count == 2
    mission.execute.assert_called_once_with(final_parking=False, alignment=True, face=True)


def test_failed_alignment_never_dispatches_final_approach(mission):
    mission.execute.side_effect = [True, False]
    assert not invoke(mission)
    assert mission.observe_target.call_count == 1


def test_goal_success_with_wrong_gap_is_not_parking_success(mission):
    mission.capture_stationary_pose.side_effect = [((0, 0, 0), {}), ((0.84, 0, 0), {})]
    assert not invoke(mission)
    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)


def test_center_gap_cannot_hide_rotated_front_corner(mission):
    yaw = math.radians(3)
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.95 - 0.065 * math.cos(yaw), 0, yaw), {})]
    assert not invoke(mission)
    result = mission.emit.call_args.kwargs
    assert result['estimated_front_gap_m'] == pytest.approx(0.05)
    assert min(result['estimated_front_corner_gaps_m']) < 0.04


def test_corner_gaps_inside_yaw_tolerance_are_logged_not_gated(mission):
    """2026-09-30 table_02: 2.74 deg, centre 5.7 cm, corners 4.4/7.0 cm was a good park."""
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}
    yaw = math.radians(2.74)
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.943 - 0.065 * math.cos(yaw), 0, yaw), {})]
    assert invoke(mission)
    result = mission.emit.call_args.kwargs
    assert mission.emit.call_args.args == ('box_approach_finished',)
    assert result['estimated_front_gap_m'] == pytest.approx(0.057)
    assert max(result['estimated_front_corner_gaps_m']) > 0.06
    assert result['gap_confirmation_basis'] == 'nearest_front_corner'
    assert result['allowed_face_yaw_error_rad'] == pytest.approx(math.radians(3))


def _table_edge_pose_for_corner_gaps(
        left_gap_m, right_gap_m, front_extent_m=0.065, frame_width_m=0.45):
    """Pose facing a face at x=1 with the requested two front-corner gaps."""
    # box_service orders corners as robot-right, then robot-left.
    yaw = -math.asin((right_gap_m - left_gap_m) / frame_width_m)
    centre_gap_m = (left_gap_m + right_gap_m) / 2.0
    return (1.0 - centre_gap_m - front_extent_m * math.cos(yaw), 0.0, yaw)


def test_table_edge_rejects_synthetic_five_and_six_point_five_centimetre_tilt(mission):
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}

    result = _final_judged(
        mission, _table_edge_pose_for_corner_gaps(0.05, 0.065)).kwargs

    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)
    assert result['estimated_front_corner_gaps_m'] == pytest.approx([0.065, 0.05])
    assert result['estimated_face_yaw_error_rad'] == pytest.approx(math.radians(1.910213))
    assert result['gap_confirmation_basis'] == 'both_front_corners'
    assert result['allowed_face_yaw_error_rad'] == pytest.approx(math.radians(1))


def test_table_edge_rejects_far_corner_over_six_centimetres_below_one_degree(mission):
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}

    result = _final_judged(
        mission, _table_edge_pose_for_corner_gaps(0.055, 0.061)).kwargs

    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)
    assert result['estimated_face_yaw_error_rad'] < math.radians(1)
    assert result['estimated_nearest_front_gap_m'] == pytest.approx(0.055)
    assert max(result['estimated_front_corner_gaps_m']) == pytest.approx(0.061)


def test_table_edge_rejects_yaw_over_one_degree_with_both_gaps_in_range(mission):
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}

    result = _final_judged(
        mission, _table_edge_pose_for_corner_gaps(0.045, 0.055)).kwargs

    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)
    assert result['estimated_front_corner_gaps_m'] == pytest.approx([0.055, 0.045])
    assert result['estimated_face_yaw_error_rad'] > math.radians(1)
    assert result['allowed_face_yaw_error_rad'] == pytest.approx(math.radians(1))


def test_table_edge_preserves_contract_yaw_tighter_than_one_degree(mission):
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(0.5)}

    result = _final_judged(
        mission, _table_edge_pose_for_corner_gaps(0.047, 0.053)).kwargs

    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)
    assert result['estimated_front_corner_gaps_m'] == pytest.approx([0.053, 0.047])
    assert result['estimated_face_yaw_error_rad'] > math.radians(0.5)
    assert result['allowed_face_yaw_error_rad'] == pytest.approx(math.radians(0.5))


def test_table_edge_accepts_both_corners_within_gap_and_one_degree(mission):
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}

    result = _final_judged(
        mission, _table_edge_pose_for_corner_gaps(0.052, 0.058)).kwargs

    assert mission.emit.call_args.args == ('box_approach_finished',)
    assert result['estimated_front_corner_gaps_m'] == pytest.approx([0.058, 0.052])
    assert result['estimated_face_yaw_error_rad'] < math.radians(1)


def _replay_table_edge_arrival(mission, camera_mount, sample):
    """Judge one saved observation at its stationary base-frame origin."""
    observed = sample['observation']
    target = box_service.compute_table_edge_docking_target(
        observed,
        camera_mount,
        {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        requested_gap_m=0.05,
        front_extent_m=0.065,
        now_s=observed['stamp_s'],
    )
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}
    mission.observe_target.side_effect = None
    mission.observe_target.return_value = target
    return _final_judged(mission, (0.0, 0.0, 0.0))


def test_saved_table_edges_with_provisional_yaw_do_not_confirm_arrival(mission):
    with CAMERA_MOUNT.open(encoding='utf-8') as stream:
        camera_mount = yaml.safe_load(stream)
    with TABLE_EDGE_ALIGNMENT_FIXTURE.open(encoding='utf-8') as stream:
        fixture = json.load(stream)

    for sample in fixture['samples']:
        event = _replay_table_edge_arrival(mission, camera_mount, sample)
        result = event.kwargs
        expected_gaps = [
            sample['measured_right_gap_m'], sample['measured_left_gap_m']]
        physical_heading_error_rad = math.asin(
            (sample['measured_right_gap_m'] - sample['measured_left_gap_m'])
            / fixture['frame_width_m'])

        assert event.args == ('box_gap_not_confirmed',)
        assert all(
            abs(projected - measured) < 0.005
            for projected, measured in zip(
                result['estimated_front_corner_gaps_m'], expected_gaps))
        assert abs(
            result['estimated_face_yaw_error_rad'] - physical_heading_error_rad
        ) < math.radians(0.32)


def test_latest_saved_table_edge_with_zero_yaw_still_passes_new_gate(mission):
    with CAMERA_MOUNT.open(encoding='utf-8') as stream:
        camera_mount = yaml.safe_load(stream)
    with TABLE_EDGE_ALIGNMENT_FIXTURE.open(encoding='utf-8') as stream:
        sample = json.load(stream)['samples'][-1]
    baseline_mount = deepcopy(camera_mount)
    baseline_mount['camera_mount']['transform']['yaw_rad'] = 0.0

    event = _replay_table_edge_arrival(mission, baseline_mount, sample)

    assert event.args == ('box_approach_finished',)
    assert event.kwargs['estimated_face_yaw_error_rad'] < math.radians(1)


def test_wrong_start_never_dispatches_transit(mission):
    mission.capture_stationary_pose.side_effect = [((1, 1, 0), {})]
    with pytest.raises(RuntimeError, match='starting place'):
        invoke(mission)
    mission.execute.assert_not_called()


def test_parser_does_not_enable_execution_by_default():
    args = parse_args([
        '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
        '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
        '--table-id', 'table_01', '--region-xy', '1.896', '0.303'])
    assert args.execute is False
    assert args.candidate_trial is False


def test_parser_requires_explicit_candidate_trial_for_execution():
    base = [
        '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
        '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
        '--table-id', 'table_01', '--region-xy', '1.896', '0.303',
        '--execute',
    ]
    with pytest.raises(SystemExit):
        parse_args(base)
    assert parse_args([*base, '--candidate-trial']).candidate_trial is True


def test_final_gap_and_dwell_options_are_bounded():
    base = [
        '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
        '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
        '--table-id', 'table_01', '--region-xy', '1.896', '0.303',
    ]
    args = parse_args(base)
    assert args.final_gap_m is None and args.dwell_s == box_service.STOP_DWELL_S
    assert args.surface_mode == 'front'
    assert parse_args([*base, '--surface-mode', 'table_edge']).surface_mode == 'table_edge'
    args = parse_args([*base, '--final-gap-m', '0.02', '--dwell-s', '30'])
    assert (args.final_gap_m, args.dwell_s) == (0.02, 30.0)
    assert parse_args([*base, '--final-gap-m', '0.01']).final_gap_m == 0.01
    for extra in (['--final-gap-m', '0'], ['--final-gap-m', '0.005'], ['--final-gap-m', '0.2'],
                  ['--final-gap-m', 'nan'], ['--dwell-s', '0'], ['--dwell-s', '500']):
        with pytest.raises(SystemExit):
            parse_args([*base, *extra])


def test_final_gap_comes_from_the_option_then_the_contract():
    fake = SimpleNamespace(final_gap_override_m=None,
                           parking_contract={'target_front_gap_m': 0.015})
    assert BoxServiceRoute._final_gap_m(fake) == pytest.approx(0.015)
    fake.final_gap_override_m = 0.02
    assert BoxServiceRoute._final_gap_m(fake) == pytest.approx(0.02)
    older = SimpleNamespace(parking_contract={})
    assert BoxServiceRoute._final_gap_m(older) == pytest.approx(0.05)
    table = SimpleNamespace(final_gap_override_m=None, surface_mode='table_edge',
                            parking_contract={'target_front_gap_m': 0.01})
    assert BoxServiceRoute._final_gap_m(table) == pytest.approx(0.05)
    table.final_gap_override_m = 0.02
    assert BoxServiceRoute._final_gap_m(table) == pytest.approx(0.02)


@pytest.mark.parametrize('flag', ['--search', '--resume-at-observation'])
def test_search_and_resume_require_explicit_execution(flag):
    base = [
        '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
        '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
        '--table-id', 'table_01', '--region-xy', '1.896', '0.303',
    ]
    with pytest.raises(SystemExit):
        parse_args([*base, flag])
    args = parse_args([*base, '--execute', '--candidate-trial', flag])
    assert getattr(args, flag.removeprefix('--').replace('-', '_')) is True


def test_final_approach_resume_requires_complete_table_edge_resume_flags():
    base = [
        '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
        '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
        '--table-id', 'table_01', '--region-xy', '1.896', '0.303',
    ]
    execute = [*base, '--execute', '--candidate-trial']
    valid = parse_args([
        *execute, '--resume-at-observation', '--resume-at-final-approach',
        '--surface-mode', 'table_edge'])
    assert valid.resume_at_final_approach is True
    for invalid in (
            [*base, '--resume-at-final-approach'],
            [*execute, '--resume-at-final-approach'],
            [*execute, '--resume-at-observation', '--resume-at-final-approach']):
        with pytest.raises(SystemExit):
            parse_args(invalid)


def test_direct_execution_requires_candidate_before_runtime_or_motion(mission):
    with pytest.raises(RuntimeError, match='explicit nominal-camera candidate'):
        mission.visit_observed_box(
            'route', {}, {'front_to_wheel_axis': {'value': 0.065}},
            'table_01', (1, 0), 0.6, execute=True,
            candidate_trial=False)
    mission._precision_collision_ready.assert_not_called()
    mission.execute.assert_not_called()


def test_bad_runtime_precision_profile_blocks_before_execute(mission):
    mission._precision_collision_ready.return_value = False
    with pytest.raises(RuntimeError, match='precision collision-monitor'):
        invoke(mission)
    mission.execute.assert_not_called()


def _stamp(seconds):
    whole = int(seconds)
    return SimpleNamespace(sec=whole, nanosec=int((seconds - whole) * 1e9))


def _observer_node(monkeypatch, *, scan=True, observation_stamp=10.1):
    node = object.__new__(BoxServiceRoute)
    node.capture_stationary_pose = Mock(return_value=((0.0, 0.0, 0.0), {}))
    node.parking_motion_revision = 0
    node.candidate_trial = True
    node.surface_mode = 'front'
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node.emit = Mock()
    node._configure_observer = Mock(return_value=True)
    node._observer_stable_frame_quorum = Mock(return_value=8)
    node.box_geometry = yaml.safe_load(GEOMETRY.read_text())
    node.box_status = {
        'frame_id': 'camera_color_optical_frame',
        'stamp_s': observation_stamp,
        'detected': True,
        'stable': True,
        'surface_kind': 'front',
        'front_distance_m': 0.5,
        'lateral_error_m': 0.0,
        'plane_normal': [0.0, 0.0, -1.0],
        'confidence': 0.9,
        'control_ready': False,
    }
    node.last_scan = (SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(10.1), frame_id='laser_link'),
        ranges=[0.5] * 10, angle_min=-0.1, angle_increment=0.02,
        range_min=0.05, range_max=12.0) if scan else None)
    clock_values = iter((10.0, 10.2))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(next(clock_values) * 1e9)))
    monkeypatch.setattr(box_service.rclpy, 'spin_once', lambda *_a, **_k: None)
    monkeypatch.setattr(box_service, 'compute_box_docking_target', Mock(
        return_value={
            'x_m': 0.385, 'y_m': 0.0, 'yaw_rad': 0.0,
            'estimate_gap_m': 0.05,
            'face_center_map_xy_m': (0.5, 0.0),
            'outward_normal_map_xy': (-1.0, 0.0),
            'provenance': 'NOMINAL_CAMERA_MOUNT_ESTIMATE',
        }))
    monkeypatch.setattr(box_service, 'witness_box_face_with_lidar', Mock(
        return_value={
            'fused_face_center_map_xy_m': (0.51, 0.01),
            'outward_normal_map_xy': (-1.0, 0.0),
            'residual_rms_m': 0.005,
            'distance_difference_m': -0.01,
            'normal_yaw_difference_rad': 0.01,
            'support_count': 8,
            'tangent_spread_m': 0.12,
            'provenance': 'LIDAR_DEPTH_FACE_AGREEMENT_NOT_EXTERNAL_ACCURACY',
        }))
    return node


def test_lidar_witness_rejection_records_replayable_scan_once(monkeypatch):
    node = _observer_node(monkeypatch)
    node.last_scan.ranges = [0.5, math.inf, math.nan, 0.6]

    def reject(*_args, diagnostics, **_kwargs):
        diagnostics.update({
            'valid_scan_points': 2,
            'support_count': 1,
            'plane_band_only_count': 1,
            'tangent_band_only_count': 2,
            'closest_normal_distance_m': 0.07,
        })
        raise ValueError('LiDAR face support is below five points')

    monkeypatch.setattr(box_service, 'witness_box_face_with_lidar', reject)
    spin_count = 0

    def spin(*_args, **_kwargs):
        nonlocal spin_count
        spin_count += 1
        if spin_count == 2:
            node.stop_requested = True

    monkeypatch.setattr(box_service.rclpy, 'spin_once', spin)
    clock_values = iter((10.0, 10.2, 10.2))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(
            nanoseconds=int(next(clock_values) * 1e9)))
    with pytest.raises(RuntimeError, match='below five'):
        node.observe_target({'height_m': 0.215}, 0.065, 0.05, (0.5, 0.0), 0.6)
    rejected = [call for call in node.emit.call_args_list
                if call.args == ('box_lidar_witness_rejected',)]
    assert len(rejected) == 1
    evidence = rejected[0].kwargs
    assert evidence['scan_frame'] == 'laser_link'
    assert evidence['scan_ranges'] == [0.5, None, None, 0.6]
    assert evidence['scan_ranges_truncated'] is False
    assert evidence['diagnostics']['support_count'] == 1
    assert evidence['observation']['stamp_s'] == 10.1
    assert evidence['target']['face_center_map_xy_m'] == [0.5, 0.0]
    assert evidence['robot_pose'] == {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    assert evidence['camera_mount'] == {'height_m': 0.215}
    assert evidence['box_geometry'] == BoxServiceRoute._json_evidence(
        node.box_geometry)
    assert evidence['reason'] == 'LiDAR face support is below five points'


def test_wrong_lidar_frame_is_rejected_before_witness(monkeypatch):
    node = _observer_node(monkeypatch)
    node.last_scan.header.frame_id = 'base_link'
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='measured laser_link'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.witness_box_face_with_lidar.assert_not_called()
    rejected = [call for call in node.emit.call_args_list
                if call.args == ('box_lidar_witness_rejected',)]
    assert rejected
    assert rejected[-1].kwargs['scan_frame'] == 'base_link'


def test_nonfinite_scan_scalar_keeps_original_rejection_reason(monkeypatch):
    node = _observer_node(monkeypatch)
    node.last_scan.angle_min = math.nan
    box_service.witness_box_face_with_lidar.side_effect = ValueError(
        'angle_min must be finite')
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(box_service.BoxObservationUnavailable,
                       match='angle_min must be finite') as caught:
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    assert caught.value.retryable is False
    rejected = [call for call in node.emit.call_args_list
                if call.args == ('box_lidar_witness_rejected',)]
    assert len(rejected) == 1
    evidence = rejected[0].kwargs
    assert evidence['scan_angle_min'] is None
    assert evidence['reason'] == 'angle_min must be finite'
    json.dumps(evidence, allow_nan=False)


def test_scan_evidence_is_bounded_to_nine_thousand_ranges():
    values, truncated = BoxServiceRoute._json_scan_ranges(
        (float(index) for index in range(9001)))
    assert len(values) == 9000
    assert values[0] == 0.0
    assert values[-1] == 8999.0
    assert truncated is True


def test_fused_lidar_face_recomputes_final_goal_and_preserves_provenance(
        monkeypatch):
    node = _observer_node(monkeypatch)
    result = node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    assert result['x_m'] == pytest.approx(0.395)
    assert result['y_m'] == pytest.approx(0.01)
    assert result['yaw_rad'] == pytest.approx(0.0)
    assert result['face_center_map_xy_m'] == (0.51, 0.01)
    assert result['depth_target_provenance'] == 'NOMINAL_CAMERA_MOUNT_ESTIMATE'
    assert result['lidar_witness']['support_count'] == 8
    assert result['candidate_trial'] is True
    assert result['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'
    assert [call.args for call in node.emit.call_args_list].count(
        ('box_target_observed',)) == 1


def test_table_edge_target_succeeds_without_lidar_face(monkeypatch):
    node = _observer_node(monkeypatch, scan=False)
    node.surface_mode = 'table_edge'
    node.box_status['surface_kind'] = 'table_edge'
    expected = {
        'x_m': 0.385, 'y_m': 0.0, 'yaw_rad': 0.0,
        'estimate_gap_m': 0.05,
        'face_center_map_xy_m': (0.5, 0.0),
        'outward_normal_map_xy': (-1.0, 0.0),
        'surface_kind': 'table_edge', 'table_height_m': 0.75,
        'edge_basis_map_xy': (0.0, 1.0), 'edge_rms_m': 0.003,
        'provenance': 'TABLE_EDGE_CAMERA_MOUNT_ESTIMATE',
    }
    monkeypatch.setattr(
        box_service, 'compute_table_edge_docking_target', Mock(return_value=expected))
    result = node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    assert result is expected
    assert result['front_extent_m'] == pytest.approx(0.065)
    assert result['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'
    box_service.compute_table_edge_docking_target.assert_called_once()
    box_service.witness_box_face_with_lidar.assert_not_called()
    observed = [call for call in node.emit.call_args_list
                if call.args == ('box_target_observed',)]
    assert observed[-1].kwargs['target_basis'] == 'tabletop_front_edge'
    assert observed[-1].kwargs['lidar_face_fusion'] is False


def test_table_edge_observer_reports_fresh_close_occlusion_frames(monkeypatch):
    node = _observer_node(monkeypatch, scan=False)
    node.surface_mode = 'table_edge'
    node.observer_stable_frames = 8
    node.box_status.update(
        detected=False, stable=False, surface_kind=None,
        reason='no_table_edge_candidate')
    monkeypatch.setattr(
        box_service, 'compute_table_edge_docking_target',
        Mock(side_effect=ValueError(
            'stable detected table_edge surface is required')))
    frames = iter(range(8))

    def spin(*_args, **_kwargs):
        try:
            frame = next(frames)
        except StopIteration:
            node.stop_requested = True
            return
        node.box_status['stamp_s'] = 10.1 + frame * 0.01

    monkeypatch.setattr(box_service.rclpy, 'spin_once', spin)
    clock_values = iter([10.0] + [10.2] * 8)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(next(clock_values) * 1e9)))

    with pytest.raises(box_service.BoxObservationUnavailable) as caught:
        node.observe_target({}, 0.065, 0.01, (0.5, 0.0), 0.6)

    error = caught.value
    assert error.reason == 'stable detected table_edge surface is required'
    assert error.fresh_frames == 8
    assert error.required_fresh_frames == 8
    assert error.fresh_rejection_reasons == {'no_table_edge_candidate'}
    assert not error.latency_reasons


@pytest.mark.parametrize(
    'yaml_frames,runtime_frames,fresh_frames,accepted',
    [(8, 4, 4, True), (4, 8, 4, False)])
def test_close_occlusion_uses_runtime_observer_quorum(
        monkeypatch, yaml_frames, runtime_frames, fresh_frames, accepted):
    client = Mock()
    client.wait_for_services.return_value = True
    client.get_parameters.return_value = 'future'
    node = object.__new__(BoxServiceRoute)
    node.observer_stable_frames = yaml_frames
    node.observer_parameters = client
    node._wait = Mock(return_value=SimpleNamespace(values=[runtime_frames]))
    monkeypatch.setattr(box_service, 'parameter_value_to_python', lambda value: value)

    quorum = node._observer_stable_frame_quorum()
    error = box_service.BoxObservationUnavailable(
        'stable detected table_edge surface is required', retryable=True,
        fresh_frames=fresh_frames, required_fresh_frames=quorum,
        fresh_rejection_reasons={'no_table_edge_candidate'})

    assert node._fresh_table_edge_occlusion(error) is accepted
    client.get_parameters.assert_called_once_with(['stable_frames'])


def test_close_occlusion_is_blocked_when_observer_parameter_service_is_down():
    client = Mock()
    client.wait_for_services.return_value = False
    node = object.__new__(BoxServiceRoute)
    node.observer_parameters = client
    error = box_service.BoxObservationUnavailable(
        'stable detected table_edge surface is required', retryable=True,
        fresh_frames=8, required_fresh_frames=node._observer_stable_frame_quorum(),
        fresh_rejection_reasons={'no_table_edge_candidate'})

    assert not node._fresh_table_edge_occlusion(error)
    client.get_parameters.assert_not_called()


def test_table_edge_observer_parameter_failure_blocks_before_consuming_status(monkeypatch):
    node = _observer_node(monkeypatch, scan=False)
    node.surface_mode = 'table_edge'
    node._configure_observer.return_value = False
    monkeypatch.setattr(box_service, 'compute_table_edge_docking_target', Mock())
    with pytest.raises(RuntimeError, match='observer surface and camera parameters'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.compute_table_edge_docking_target.assert_not_called()
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_front_mode_switch_failure_also_blocks_a_cli_attempt(monkeypatch):
    node = _observer_node(monkeypatch)
    node.surface_mode = 'front'
    node.require_observer_configuration = True
    node._configure_observer.return_value = False
    with pytest.raises(RuntimeError, match='observer surface and camera parameters'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.compute_box_docking_target.assert_not_called()
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_stale_table_edge_never_reaches_target_geometry(monkeypatch):
    node = _observer_node(monkeypatch, scan=False, observation_stamp=9.9)
    node.surface_mode = 'table_edge'
    node.box_status['surface_kind'] = 'table_edge'
    monkeypatch.setattr(box_service, 'compute_table_edge_docking_target', Mock())
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='no_stable_box'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.compute_table_edge_docking_target.assert_not_called()
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_observation_consumer_requires_explicit_candidate(monkeypatch):
    node = _observer_node(monkeypatch)
    node.candidate_trial = False
    with pytest.raises(RuntimeError, match='explicit candidate trial'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    node.capture_stationary_pose.assert_not_called()
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_missing_scan_never_returns_final_target(monkeypatch):
    node = _observer_node(monkeypatch, scan=False)
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='fresh LiDAR scan'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)


def test_stale_scan_never_returns_final_target(monkeypatch):
    node = _observer_node(monkeypatch)
    node.last_scan.header.stamp = _stamp(9.0)
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='LiDAR scan is stale'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)


def test_observer_false_control_flag_requires_separate_candidate_gate(monkeypatch):
    node = _observer_node(monkeypatch)
    node.box_status['control_ready'] = True
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='perception-only'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_motion_during_sensor_witness_discards_fused_target(monkeypatch):
    node = _observer_node(monkeypatch)

    def witness(*_args, **_kwargs):
        node.parking_motion_revision += 1
        return {
            'fused_face_center_map_xy_m': (0.51, 0.01),
            'outward_normal_map_xy': (-1.0, 0.0),
            'support_count': 8,
            'provenance': 'LIDAR_DEPTH_FACE_AGREEMENT_NOT_EXTERNAL_ACCURACY',
        }

    monkeypatch.setattr(box_service, 'witness_box_face_with_lidar', witness)
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='moved during box sensor witness'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)


def test_stale_depth_observation_never_reaches_fusion(monkeypatch):
    node = _observer_node(monkeypatch, observation_stamp=9.9)
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='no_stable_box'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_fresh_no_box_status_is_retryable_by_heading_search(monkeypatch):
    node = _observer_node(monkeypatch)
    node.box_status.update(
        detected=False, stable=False, surface_kind=None,
        reason='no_box_surface_candidate')
    box_service.compute_box_docking_target.side_effect = ValueError(
        'stable detected front surface is required')
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(box_service.BoxObservationUnavailable) as caught:
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    assert caught.value.retryable is True
    box_service.witness_box_face_with_lidar.assert_not_called()


@pytest.mark.parametrize('reason', [
    'angle_min must be finite', 'angle_increment must be nonzero',
    'laser range bounds are invalid', 'ranges must not be empty',
    'scan has no valid ranges', 'geometry is missing laser mounting data',
])
def test_invalid_witness_never_requests_search_rotation(monkeypatch, reason):
    node = _observer_node(monkeypatch)
    node.search_rotation = Mock()
    box_service.witness_box_face_with_lidar.side_effect = ValueError(reason)
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(box_service.BoxObservationUnavailable) as caught:
        node._observe_with_search(
            {}, 0.065, 0.45, (0.5, 0.0), 0.6,
            phase='face_alignment', search_enabled=True,
            search_budget=box_service.BoxSearchBudget())
    assert caught.value.retryable is False
    node.search_rotation.assert_not_called()


@pytest.mark.parametrize('stamp_s', [10.01, 11.0, math.nan])
def test_stale_or_future_no_box_does_not_enable_search(monkeypatch, stamp_s):
    node = _observer_node(monkeypatch, observation_stamp=stamp_s)
    node.box_status.update(detected=False, stable=False, surface_kind=None)
    clock_values = iter((10.0, 10.7))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(next(clock_values) * 1e9)))
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(box_service.BoxObservationUnavailable) as caught:
        node.observe_target({}, 0.065, 0.45, (0.5, 0.0), 0.6)
    assert caught.value.retryable is False
    box_service.compute_box_docking_target.assert_not_called()


def test_scan_callback_preserves_parent_freshness_and_full_message():
    node = object.__new__(BoxServiceRoute)
    node.samples = {'scan': None}
    node.last_scan = None
    message = SimpleNamespace()
    node._scan_callback(message)
    assert node.samples['scan'] is not None
    assert node.last_scan is message


@pytest.mark.parametrize('polygon_name', [
    'StopZone.translation_forward.points',
    'StopZone.translation_backward.points',
])
def test_live_precision_profile_matches_generated_candidate(monkeypatch, polygon_name):
    baseline = yaml.safe_load(PARAMS.read_text())
    geometry = yaml.safe_load(GEOMETRY.read_text())
    candidate = apply_docking_stop_profile(baseline, geometry)
    monitor = candidate['collision_monitor']['ros__parameters']

    def nested(name):
        value = monitor
        for component in name.split('.'):
            value = value[component]
        return deepcopy(value)

    client = Mock()
    client.wait_for_services.return_value = True
    client.get_parameters.side_effect = lambda names: list(names)
    node = object.__new__(BoxServiceRoute)
    node.precision_parameters = client
    node._wait = lambda names, _timeout: SimpleNamespace(
        values=[nested(name) for name in names])
    monkeypatch.setattr(box_service, 'parameter_value_to_python', lambda value: value)
    monkeypatch.setattr(
        box_service, 'get_package_share_directory',
        lambda _name: str(ROOT / 'jdamr_cube_navigation'))
    assert node._precision_collision_ready(geometry)

    original_wait = node._wait
    node._wait = lambda names, timeout: SimpleNamespace(values=[
        ('[[0.135, 0.29], [0.135, -0.29], [-0.295, -0.29], '
         '[-0.295, 0.29]]') if name == polygon_name
        else nested(name) for name in names])
    assert not node._precision_collision_ready(geometry)
    node._wait = original_wait


def _done(value):
    future = Future()
    future.set_result(value)
    return future


def _transit_planner(node, end_xy):
    """Answer route preflight with one successful plan that ends at end_xy."""
    result = ComputePathThroughPoses.Result()
    start, finish = PoseStamped(), PoseStamped()
    finish.pose.position.x, finish.pose.position.y = end_xy
    result.path.poses = [start, finish]
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=result)
    node.compute = Mock()
    node.compute.wait_for_server.return_value = True
    node.compute.send_goal_async.return_value = _done(SimpleNamespace(
        accepted=True, get_result_async=lambda: _done(wrapped)))
    node.navigation_profile = 'obstacle_base_candidate'
    node.get_logger = lambda: Mock()
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=10 ** 12))
    route = box_service.load_route('route')
    route['waypoints'] = [
        {'id': 'home_exit', 'x': -0.255, 'y': 0.2, 'yaw': 1.61},
        {'id': 'table_01_observation', 'x': 1.45, 'y': 0.25, 'yaw': 0.0}]
    del node.preflight
    return route


def test_t17_box_preflight_rejects_transit_end_shortfall(mission):
    """Reject a transit plan that ends 0.26 m short of the observation pose."""
    _transit_planner(mission, (1.19, 0.25))
    result = invoke(mission, execute=False)
    failed = [call.kwargs for call in mission.emit.call_args_list
              if call.args == ('failed',) and call.kwargs.get('phase') == 'preflight']
    assert result is False and failed, (
        f'HEADFAIL[T17]: 0.26 m transit end shortfall passed preflight ({result})')
    assert failed[0]['end_error_m'] == pytest.approx(0.26)
    mission.execute.assert_not_called()


def test_t17_corridor_preflight_keeps_same_end_shortfall(mission):
    """Keep the base corridor preflight verdict for the same short plan."""
    route = _transit_planner(mission, (1.19, 0.25))
    mission.config, mission.waypoints = route, route['waypoints']
    assert CorridorRoute.preflight(mission) is True


def _latency_limited(reason='stale_or_future_depth'):
    error = box_service.BoxObservationUnavailable(reason, retryable=False)
    error.latency_limited = True
    return error


def _search(node):
    return node._observe_with_search(
        {}, 0.065, 0.45, (1, 0), 0.6, phase='face_alignment', search_enabled=True,
        search_budget=box_service.BoxSearchBudget())


@pytest.mark.parametrize('case', ['ok', 'twice', 'stop'])
def test_t20_latency_reobserve(mission, case):
    """Re-observe one latency-limited window in place; never after a stop."""
    target = mission.observe_target.return_value
    if case == 'ok':
        mission.observe_target.side_effect = [_latency_limited(), target]
        try:
            result = _search(mission)
        except box_service.BoxObservationUnavailable as error:
            raise AssertionError(
                f'HEADFAIL[T20]: latency-limited window was not re-observed: '
                f'{error.reason}') from error
        assert result is target
        assert mission.observe_target.call_count == 2
        assert [call.args for call in mission.emit.call_args_list].count(
            ('box_observation_latency_reobserve',)) == 1
    elif case == 'twice':
        mission.observe_target.side_effect = [_latency_limited(), _latency_limited()]
        with pytest.raises(box_service.BoxObservationUnavailable) as caught:
            _search(mission)
        assert mission.observe_target.call_count == 2, (
            f'HEADFAIL[T20]: observations={mission.observe_target.call_count}')
        assert caught.value.retryable is False
    else:
        mission.stop_requested = True
        mission.observe_target.side_effect = [
            box_service.BoxObservationUnavailable('no_stable_box', retryable=True), target]
        try:
            _search(mission)
            raised = None
        except box_service.BoxObservationUnavailable as error:
            raised = error
        assert (mission.search_rotation.call_count == 0
                and mission.observe_target.call_count == 1), (
            f'HEADFAIL[T20]: stop request rotated={mission.search_rotation.call_count} '
            f'observed={mission.observe_target.call_count}')
        assert raised is not None
        failed = [call.kwargs for call in mission.emit.call_args_list
                  if call.args == ('box_observation_failed',)]
        assert failed and failed[-1]['stop_requested'] is True
    mission.search_rotation.assert_not_called()


RETURN_ARGS = [
    '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
    '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
    '--table-id', 'table_01', '--region-xy', '1.896', '0.303',
]


def _parse_new(tag, argv):
    try:
        return parse_args(argv)
    except SystemExit as error:
        raise AssertionError(f'NEW[{tag}]: parser rejected {argv[-4:]}') from error


def test_t28_return_cli_requires_explicit_timeouts():
    """Require --execute and an explicit return budget for the home return."""
    execute = [*RETURN_ARGS, '--execute', '--candidate-trial']
    args = _parse_new('T28', [*execute, '--return-home', '--return-timeout-s', '500'])
    assert args.return_home is True and args.return_timeout_s == 500.0
    assert args.task_timeout_s == 240.0
    assert _parse_new('T28', [*execute, '--task-timeout-s', '300']).task_timeout_s == 300.0
    for invalid in ([*execute, '--return-home'],
                    [*RETURN_ARGS, '--return-home', '--return-timeout-s', '500'],
                    [*execute, '--return-home', '--return-timeout-s', 'nan'],
                    [*execute, '--task-timeout-s', '0']):
        with pytest.raises(SystemExit):
            parse_args(invalid)


def test_t28_failed_visit_never_returns_home(monkeypatch, tmp_path):
    """Dwell and return only after a confirmed approach; exit 0 needs every step."""
    for name in ('registry', 'mount', 'geometry', 'route'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_01',
            '--region-xy', '1.896', '0.303', '--execute', '--candidate-trial',
            '--return-home', '--return-timeout-s', '500']
    _parse_new('T28', [*argv, '--log', str(tmp_path / 'probe.jsonl')])
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    created = []

    class Route:
        visit_ok = False

        def __init__(self, *args, **kwargs):
            self.kwargs = kwargs
            self.visit_observed_box = Mock(return_value=Route.visit_ok)
            self.dwell_and_return_home = Mock(return_value=True)
            self.go_home = Mock(return_value=True)
            self._camera = Mock(return_value=True)
            self.finish_navigation = Mock(return_value=True)
            self.destroy_node, self.emit, self.request_stop = Mock(), Mock(), Mock()
            self.call_operator = Mock(return_value='CRITICAL')
            self.battery_return_due = Mock(return_value=False)
            created.append(self)

    monkeypatch.setattr(box_service, 'BoxServiceRoute', Route)
    for visit_ok, expected in ((False, 1), (True, 0)):
        Route.visit_ok = visit_ok
        log = tmp_path / f'visit_{visit_ok}.jsonl'
        assert box_service.main([*argv, '--log', str(log)]) == expected
    failed, succeeded = created
    failed.dwell_and_return_home.assert_not_called()
    failed.go_home.assert_not_called()
    # Every attempt ends with the box camera off, the failed one included.
    for node in created:
        assert node.camera_on_demand is True
        node._camera.assert_called_with(False)
    # A cycle that stopped short calls the operator once; a finished one does not.
    failed.call_operator.assert_called_once_with()
    succeeded.call_operator.assert_not_called()
    succeeded.dwell_and_return_home.assert_called_once_with(2.0, 500.0)
    assert succeeded.visit_observed_box.call_args.kwargs['task_timeout_s'] == 240.0
    assert succeeded.kwargs['home_contract']['xy_tolerance_m'] == 0.05


def test_t28_final_execute_releases_task_deadline(mission):
    """Release the task deadline after the final execute, before final capture."""
    assert 'task_timeout_s' in inspect.signature(
        BoxServiceRoute.visit_observed_box).parameters, 'NEW[T28]: no task_timeout_s'
    seen = []
    mission.execute.side_effect = lambda **_kwargs: seen.append(
        ('execute', mission.run_deadline_s)) or True
    poses = iter([((0, 0, 0), {}), ((0.885, 0, 0), {})])

    def capture(*_args, **_kwargs):
        seen.append(('capture', getattr(mission, 'run_deadline_s', None)))
        return next(poses)

    mission.capture_stationary_pose = Mock(side_effect=capture)
    started_s = time.monotonic()
    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540}},
        'table_01', (1, 0), 0.6, execute=True, candidate_trial=True,
        task_timeout_s=300.0)
    assert [kind for kind, _deadline in seen] == [
        'capture', 'execute', 'execute', 'capture']
    deadlines = [deadline for kind, deadline in seen if kind == 'execute']
    assert deadlines[0] - started_s == pytest.approx(300.0, abs=5.0)
    assert seen[-1] == ('capture', None)
    assert mission.selected_pose['id'] == 'final_approach'


def test_l9_home_contract_load_failure_uses_failure_path(monkeypatch, tmp_path, capsys):
    """Report an invalid dock contract through main's JSON failure path."""
    for name in ('registry', 'mount', 'geometry', 'route'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    share = tmp_path / 'share'
    (share / 'config').mkdir(parents=True)
    (share / 'config/parking_contract.yaml').write_text('{}\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_01',
            '--region-xy', '1.896', '0.303', '--log', str(tmp_path / 'run.jsonl')]
    monkeypatch.setattr(box_service, 'get_package_share_directory', lambda _name: str(share))
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    created = []
    monkeypatch.setattr(box_service, 'BoxServiceRoute',
                        lambda *args, **kwargs: created.append(kwargs))
    try:
        code = box_service.main(argv)
    except ValueError as error:
        raise AssertionError(f'REVIEW[L9]: dock contract error escaped main: {error}') from error
    assert code == 1
    lines = capsys.readouterr().out.strip().splitlines()
    assert json.loads(lines[-1])['failed'].startswith('parking contract keys invalid')
    assert created == []


VIA_ARGS = ['--via-id', 'water_station', '--via-route', '/w',
            '--via-region-xy', '-0.14', '-1.53']


def test_t36_via_cli_is_all_or_nothing_and_ends_at_the_dock():
    """A first stop needs its route and region, execution and the dock return."""
    execute = [*RETURN_ARGS, '--execute', '--candidate-trial',
               '--return-home', '--return-timeout-s', '500']
    args = _parse_new('T36', [*execute, *VIA_ARGS, '--via-region-radius-m', '0.35'])
    assert (args.via_id, str(args.via_route), args.via_region_xy,
            args.via_region_radius_m) == ('water_station', '/w', [-0.14, -1.53], 0.35)
    assert _parse_new('T36', execute).via_id is None
    for invalid in ([*execute, *VIA_ARGS[:2]],
                    [*execute, *VIA_ARGS[:4]],
                    [*RETURN_ARGS, '--execute', '--candidate-trial', *VIA_ARGS],
                    [*execute, *VIA_ARGS, '--via-region-radius-m', '0.7']):
        with pytest.raises(SystemExit):
            parse_args(invalid)


def _via_main(monkeypatch, tmp_path, via_ok=True, leave_ok=True, battery_low=False):
    for name in ('registry', 'mount', 'geometry', 'route', 'water'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_01',
            '--region-xy', '1.5', '-1.49', '--execute', '--candidate-trial', '--search',
            '--return-home', '--return-timeout-s', '500',
            '--via-id', 'water_station', '--via-route', str(tmp_path / 'water.yaml'),
            '--via-region-xy', '-0.14', '-1.53']
    _parse_new('T36', [*argv, '--log', str(tmp_path / 'probe.jsonl')])
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    calls = []

    class Route:
        def __init__(self, *args, **kwargs):
            self.visit_observed_box = Mock(side_effect=lambda route, *a, **k: calls.append(
                ('visit', Path(route).name, a[2], list(a[3]), k['search'])) or (
                    via_ok if Path(route).name == 'water.yaml' else True))
            self.dwell_and_leave = Mock(side_effect=lambda dwell: calls.append(
                ('dwell_and_leave', dwell)) or leave_ok)
            self.dwell_and_return_home = Mock(side_effect=lambda dwell, timeout: calls.append(
                ('dwell_and_return_home', dwell, timeout)) or True)
            self._camera = Mock(return_value=True)
            self.finish_navigation = Mock(return_value=True)
            self.destroy_node, self.emit, self.request_stop = Mock(), Mock(), Mock()
            self.call_operator = Mock(side_effect=lambda: calls.append(('call_operator',))
                                      or 'URGENT')
            self.battery_return_due = Mock(return_value=battery_low)
            self.go_home = Mock(side_effect=lambda **k: calls.append(('go_home',)) or True)

    monkeypatch.setattr(box_service, 'BoxServiceRoute', Route)
    code = box_service.main([*argv, '--log', str(tmp_path / f'run_{via_ok}_{leave_ok}.jsonl')])
    return code, calls


def test_t36_water_station_then_table_then_dock(monkeypatch, tmp_path):
    """The call runs the water stop, its dwell and escape, the table, then the dock."""
    code, calls = _via_main(monkeypatch, tmp_path)
    assert code == 0
    assert calls == [('visit', 'water.yaml', 'water_station', [-0.14, -1.53], True),
                     ('dwell_and_leave', 2.0),
                     ('visit', 'route.yaml', 'table_01', [1.5, -1.49], True),
                     ('dwell_and_return_home', 2.0, 500.0)]


def test_low_battery_after_the_water_stop_docks_and_calls_the_operator(monkeypatch, tmp_path):
    """Below the departure reserve between stops: no table, straight to the dock, then a call."""
    code, calls = _via_main(monkeypatch, tmp_path, battery_low=True)
    assert code == 1
    assert calls == [('visit', 'water.yaml', 'water_station', [-0.14, -1.53], True),
                     ('dwell_and_leave', 2.0), ('go_home',), ('call_operator',)]


@pytest.mark.parametrize('via_ok,leave_ok,expected', [
    (False, True, [('visit', 'water.yaml')]),
    (True, False, [('visit', 'water.yaml'), ('dwell_and_leave',)]),
])
def test_t36_failed_water_stop_never_goes_to_the_table(
        monkeypatch, tmp_path, via_ok, leave_ok, expected):
    """A failed water approach or escape ends the run before any table motion."""
    code, calls = _via_main(monkeypatch, tmp_path, via_ok=via_ok, leave_ok=leave_ok)
    assert code == 1
    assert [call[:len(step)] for call, step in zip(calls, expected)] == expected
    # Nothing moves after the failed step; the stopped cycle calls the operator.
    assert calls[len(expected):] == [('call_operator',)]


@pytest.mark.parametrize('parked,left,expected', [
    (False, True, False), (True, False, False), (True, True, True)])
def test_t36_dwell_and_leave_holds_then_escapes(parked, left, expected):
    """Hold the verified stop, then reverse straight away; never escape unverified."""
    route = BoxServiceRoute.__new__(BoxServiceRoute)
    route.wait_parked = Mock(return_value=parked)
    route._leave_parked_pose = Mock(return_value=left)
    route.emit = Mock()
    assert hasattr(BoxServiceRoute, 'dwell_and_leave'), 'NEW[T36]: no dwell_and_leave'
    assert route.dwell_and_leave(5.0) is expected
    route.wait_parked.assert_called_once_with(5.0)
    assert route._leave_parked_pose.call_count == (1 if parked else 0)
    events = [call.args[0] for call in route.emit.call_args_list]
    assert ('via_stop_finished' in events) is expected


def test_t37_live_precision_profile_requires_slowdown_disabled(monkeypatch):
    """A live collision monitor that still slows near the box is not the profile."""
    baseline = yaml.safe_load(PARAMS.read_text())
    geometry = yaml.safe_load(GEOMETRY.read_text())
    monitor = apply_docking_stop_profile(baseline, geometry)[
        'collision_monitor']['ros__parameters']

    def nested(name):
        value = monitor
        for component in name.split('.'):
            value = value[component]
        return deepcopy(value)

    client = Mock()
    client.wait_for_services.return_value = True
    client.get_parameters.side_effect = lambda names: list(names)
    node = object.__new__(BoxServiceRoute)
    node.precision_parameters = client
    monkeypatch.setattr(box_service, 'parameter_value_to_python', lambda value: value)
    monkeypatch.setattr(
        box_service, 'get_package_share_directory',
        lambda _name: str(ROOT / 'jdamr_cube_navigation'))
    node._wait = lambda names, _timeout: SimpleNamespace(values=[
        True if name == 'SlowdownZone.enabled' else nested(name) for name in names])
    assert node._precision_collision_ready(geometry) is False


@pytest.mark.parametrize('toward_deg,expected_deg', [
    (-24.5, -24.5),   # 2026-09-30 water station: region 24 deg clockwise, sweep turned CCW
    (-60.0, -30.0),   # far off: one bounded step toward the region
    (16.0, 16.0),
    (4.0, 30.0),      # centred region: the failure is not a heading problem
    (None, 30.0),     # no diagnostic: the fixed counter-clockwise sweep
])
def test_t42_search_turns_toward_the_region_first(toward_deg, expected_deg):
    """A known region bearing sets the step direction and size; the budget stays absolute."""
    budget = box_service.BoxSearchBudget()
    toward = None if toward_deg is None else math.radians(toward_deg)
    step = budget.next_rotation(toward_rad=toward)
    assert step == pytest.approx(math.radians(expected_deg))
    assert budget.cumulative_yaw_rad == pytest.approx(abs(math.radians(expected_deg)))


def test_t42_observation_search_uses_the_diagnostic_bearing(mission):
    """The first search rotation follows the recorded region bearing."""
    failure = box_service.BoxObservationUnavailable('no_box_surface_candidate', retryable=True)
    target = mission.observe_target.return_value

    def observe(*_args, **_kwargs):
        mission.last_region_bearing_error_rad = math.radians(-24.5)
        if mission.search_rotation.call_count == 0:
            raise failure
        return target

    mission.observe_target = Mock(side_effect=observe)
    assert invoke(mission, search=True)
    first = mission.search_rotation.call_args_list[0].args[0]
    assert first == pytest.approx(math.radians(-24.5))


def test_t43_resume_options_parse():
    """Resume at the via observation, or from a parked stop recorded in a log."""
    execute = [*RETURN_ARGS, '--execute', '--candidate-trial',
               '--return-home', '--return-timeout-s', '500']
    assert _parse_new('T43', [*execute, *VIA_ARGS, '--resume-at-observation']).via_id
    args = _parse_new('T43', [*execute, *VIA_ARGS, '--resume-parked-from-log', '/log'])
    assert str(args.resume_parked_from_log) == '/log'
    for invalid in ([*RETURN_ARGS, '--execute', '--candidate-trial',
                     '--resume-parked-from-log', '/log'],
                    [*execute, '--resume-parked-from-log', '/log', '--resume-at-observation']):
        with pytest.raises(SystemExit):
            parse_args(invalid)


def _resume_main(monkeypatch, tmp_path, extra, at_table_observation=False):
    for name in ('registry', 'mount', 'geometry', 'route', 'water'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    log = tmp_path / 'previous.jsonl'
    log.write_text(json.dumps({'event': 'box_target_observed', 'target': {
        'face_center_map_xy_m': [0.02, -1.2], 'outward_normal_map_xy': [1.0, 0.0]}}) + '\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_02',
            '--region-xy', '0.7', '-2.6', '--execute', '--candidate-trial', '--search',
            '--return-home', '--return-timeout-s', '500',
            *[str(log) if item == 'LOG' else item for item in extra]]
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    calls = []

    class Route:
        def __init__(self, *args, **kwargs):
            self.visit_observed_box = Mock(side_effect=lambda route, *a, **k: calls.append(
                ('visit', Path(route).name, a[2], k.get('resume_at_observation'))) or True)
            self.resume_parked = Mock(side_effect=lambda face, dwell: calls.append(
                ('resume_parked', face['face_center_map_xy_m'], dwell)) or True)
            self.dwell_and_leave = Mock(side_effect=lambda dwell: calls.append(
                ('dwell_and_leave', dwell)) or True)
            self.dwell_and_return_home = Mock(side_effect=lambda dwell, timeout: calls.append(
                ('dwell_and_return_home', dwell)) or True)
            self.go_home = Mock(side_effect=lambda **k: calls.append(('go_home',)) or True)
            self.at_observation = Mock(side_effect=lambda route: calls.append(
                ('at_observation', Path(route).name)) or at_table_observation)
            self.verify_live_maps = Mock()
            self.wait_until_ready = Mock(return_value=True)
            self._camera = Mock(return_value=True)
            self.finish_navigation = Mock(return_value=True)
            self.destroy_node, self.emit, self.request_stop = Mock(), Mock(), Mock()
            self.call_operator = Mock(return_value='CRITICAL')
            self.battery_return_due = Mock(return_value=False)

    monkeypatch.setattr(box_service, 'BoxServiceRoute', Route)
    code = box_service.main([*argv, '--log', str(tmp_path / 'run.jsonl')])
    return code, calls


def test_t43_resume_at_via_observation_applies_to_the_water_stop_only(monkeypatch, tmp_path):
    code, calls = _resume_main(monkeypatch, tmp_path,
                               [*VIA_ARGS[:2], '--via-route', str(tmp_path / 'water.yaml'),
                                *VIA_ARGS[4:], '--resume-at-observation'])
    assert code == 0
    assert calls == [('at_observation', 'route.yaml'),
                     ('visit', 'water.yaml', 'water_station', True), ('dwell_and_leave', 2.0),
                     ('visit', 'route.yaml', 'table_02', False), ('dwell_and_return_home', 2.0)]


def test_resume_at_the_table_observation_skips_the_done_water_stop(monkeypatch, tmp_path):
    """2026-10-01 15:49: a resume at table_01 must not demand the water station."""
    code, calls = _resume_main(monkeypatch, tmp_path,
                               [*VIA_ARGS[:2], '--via-route', str(tmp_path / 'water.yaml'),
                                *VIA_ARGS[4:], '--resume-at-observation'],
                               at_table_observation=True)
    assert code == 0
    assert calls == [('at_observation', 'route.yaml'),
                     ('visit', 'route.yaml', 'table_02', True), ('dwell_and_return_home', 2.0)]


@pytest.mark.parametrize('pose, expected', [((0.80, -1.86, 0.2), True),
                                            ((0.40, -1.82, 0.0), False)])
def test_at_observation_compares_the_still_pose_with_the_route_end(
        monkeypatch, pose, expected):
    route = BoxServiceRoute.__new__(BoxServiceRoute)
    monkeypatch.setattr(box_service, 'load_route', lambda _path: {
        'max_route_start_distance_m': 0.3,
        'waypoints': [{'x': 0.438, 'y': -1.817}, {'x': 0.858, 'y': -1.817, 'yaw': 0.0}]})
    route.verify_live_maps = Mock()
    route.wait_until_ready = Mock(return_value=True)
    route._localization_bound = lambda _relaxed: contextlib.nullcontext()
    route.capture_stationary_pose = Mock(return_value=(pose, {}))
    assert route.at_observation('table_01_route.yaml') is expected


def test_t43_parked_water_stop_resumes_with_logged_face_then_table(monkeypatch, tmp_path):
    code, calls = _resume_main(monkeypatch, tmp_path,
                               [*VIA_ARGS[:2], '--via-route', str(tmp_path / 'water.yaml'),
                                *VIA_ARGS[4:], '--resume-parked-from-log', 'LOG'])
    assert code == 0
    assert calls == [('resume_parked', [0.02, -1.2], 2.0),
                     ('visit', 'route.yaml', 'table_02', False), ('dwell_and_return_home', 2.0)]


def test_t43_parked_table_resumes_with_logged_face_then_docks(monkeypatch, tmp_path):
    code, calls = _resume_main(monkeypatch, tmp_path, ['--resume-parked-from-log', 'LOG'])
    assert code == 0
    assert calls == [('resume_parked', [0.02, -1.2], 2.0), ('go_home',)]


def test_t43_last_logged_face_requires_an_observed_face(tmp_path):
    path = tmp_path / 'log.jsonl'
    path.write_text('{"event": "accepted"}\nnot json\n')
    with pytest.raises(ValueError, match='no observed box face'):
        box_service.last_logged_face(path)
    path.write_text('\n'.join(json.dumps({'event': 'box_target_observed', 'target': {
        'face_center_map_xy_m': [x, 0.0], 'outward_normal_map_xy': [1.0, 0.0]}})
        for x in (1.0, 2.0)))
    assert box_service.last_logged_face(path)['face_center_map_xy_m'] == [2.0, 0.0]


@pytest.mark.parametrize('moved,left,expected', [
    (0.0, True, True), (0.05, True, False), (0.0, False, False)])
def test_t43_resume_parked_holds_still_then_escapes(monkeypatch, moved, left, expected):
    """The resumed dwell needs a stationary robot; the escape uses the logged face."""
    monkeypatch.setattr(box_service.time, 'sleep', lambda _s: None)
    route = BoxServiceRoute.__new__(BoxServiceRoute)
    route.capture_stationary_pose = Mock(side_effect=[((0.1, -1.2, math.pi), {}),
                                                      ((0.1 + moved, -1.2, math.pi), {})])
    route._leave_parked_pose = Mock(return_value=left)
    route.emit = Mock()
    face = {'face_center_map_xy_m': [0.0, -1.2], 'outward_normal_map_xy': [1.0, 0.0]}
    assert route.resume_parked(face, 5.0) is expected
    assert route._leave_parked_pose.call_count == (0 if moved else 1)
    if moved == 0.0:
        assert route.last_box_face == face


def test_resident_executor_runs_requests_in_one_context(tmp_path, monkeypatch):
    """One rclpy context for every attempt; results land beside the requests."""
    import signal
    calls = []
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kw: calls.append('init'))
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda: calls.append('shutdown'))
    monkeypatch.setattr(box_service, 'parse_args', lambda argv: SimpleNamespace(argv=argv))

    def attempt(args, active):
        calls.append(tuple(args.argv))
        assert active == {'node': None}
        if args.argv == ['b']:
            signal.raise_signal(signal.SIGTERM)   # leave after this attempt
        return 0 if args.argv == ['a'] else 1

    monkeypatch.setattr(box_service, 'run_attempt', attempt)
    (tmp_path / '1.request').write_text(json.dumps({'argv': ['a']}))
    (tmp_path / '2.request').write_text(json.dumps({'argv': ['b']}))
    assert box_service.serve(tmp_path) == 0
    assert calls == ['init', ('a',), ('b',), 'shutdown']
    assert json.loads((tmp_path / '1.result').read_text()) == {'code': 0}
    assert json.loads((tmp_path / '2.result').read_text()) == {'code': 1}
    assert not list(tmp_path.glob('*.running'))


class _ParameterClientStub:
    """Answers set_parameters with the given success, recording the request."""

    def __init__(self, successful=True, available=True):
        self.successful, self.available, self.requests = successful, available, []

    def wait_for_services(self, timeout_sec=None):
        return self.available

    def set_parameters_atomically(self, parameters):
        self.requests.append([(p.name, p.value) for p in parameters])
        return SetParametersAtomically.Response(result=SetParametersResult(
            successful=self.successful, reason='' if self.successful else 'rejected'))


@pytest.mark.parametrize('distance_m, radius_m, expected_m', [
    (0.746, 0.4, 1.146),     # table_01 15:37: the 1.33 m wall is left out
    (1.9, 0.4, 2.0),         # never wider than the configured observer range
    (None, 0.4, 2.0),        # no region bearing: the full range
])
def test_observation_atomically_configures_mode_mount_and_depth(
        distance_m, radius_m, expected_m):
    node = object.__new__(BoxServiceRoute)
    events = []
    node.emit = lambda event, **fields: events.append((event, fields))
    node._wait = lambda future, timeout_s: future
    node.observer_parameters = _ParameterClientStub()
    node.last_region_distance_m = distance_m
    node.surface_mode = 'table_edge'
    mount = yaml.safe_load(CAMERA_MOUNT.read_text())
    assert node._configure_observer(mount, radius_m) is True
    requested = dict(node.observer_parameters.requests[0])
    transform = mount['camera_mount']['transform']
    assert requested.pop('surface_mode') == 'table_edge'
    assert requested == pytest.approx({
        'maximum_depth_m': expected_m,
        'camera_height_m': transform['z_m'],
        'camera_roll_rad': transform['roll_rad'],
        'camera_pitch_rad': transform['pitch_rad'],
    })
    assert events[-1][0] == 'box_observer_configuration'
    assert events[-1][1]['applied'] is True


@pytest.mark.parametrize('client', [_ParameterClientStub(successful=False),
                                    _ParameterClientStub(available=False)])
def test_depth_window_failure_is_recorded_and_does_not_stop_the_observation(client):
    node = object.__new__(BoxServiceRoute)
    events = []
    node.emit = lambda event, **fields: events.append((event, fields))
    node._wait = lambda future, timeout_s: future
    node.observer_parameters = client
    node.last_region_distance_m = 0.7
    node.surface_mode = 'front'
    assert node._configure_observer(
        yaml.safe_load(CAMERA_MOUNT.read_text()), 0.4) is False
    assert events[-1][1]['applied'] is False and events[-1][1]['reason']


def test_operator_sent_stop_is_not_called_back_to_the_operator():
    """`jdamr_depart.py stop` ends the attempt quietly; any other short stop calls."""
    node = SimpleNamespace(call_operator=Mock())
    assert box_service._attempt_code(node, True) == 0
    node.call_operator.assert_not_called()
    assert box_service._attempt_code(node, False) == 1
    node.call_operator.assert_called_once_with()
    node.call_operator.reset_mock()
    node.operator_stopped = True
    assert box_service._attempt_code(node, False) == 1
    node.call_operator.assert_not_called()


def test_rpp_transit_flag_selects_rpp_before_the_cycle(monkeypatch, tmp_path):
    for name in ('registry', 'mount', 'geometry', 'route'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_01',
            '--region-xy', '1.896', '0.303', '--execute', '--candidate-trial']
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    created = []

    class Route:
        def __init__(self, *args, **kwargs):
            calls = []
            self.use_rpp_transit = Mock(side_effect=lambda: calls.append('rpp'))
            self.visit_observed_box = Mock(side_effect=lambda *a, **k: calls.append('visit'))
            self._camera = Mock(return_value=True)
            self.finish_navigation = Mock(return_value=True)
            self.destroy_node, self.emit, self.request_stop = Mock(), Mock(), Mock()
            self.call_operator = Mock(return_value='CRITICAL')
            self.battery_return_due = Mock(return_value=False)
            self.calls = calls
            created.append(self)

    monkeypatch.setattr(box_service, 'BoxServiceRoute', Route)
    box_service.main([*argv, '--log', str(tmp_path / 'rpp.jsonl')])
    box_service.main([*argv, '--rpp-transit', '--log', str(tmp_path / 'mppi.jsonl')])
    default, rpp = created
    assert default.calls == ['visit']
    assert rpp.calls == ['rpp', 'visit']


def test_final_gap_and_dwell_reach_the_attempt(monkeypatch, tmp_path):
    for name in ('registry', 'mount', 'geometry', 'route'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_01',
            '--region-xy', '1.896', '0.303', '--execute', '--candidate-trial', '--return-home',
            '--return-timeout-s', '600']
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    created = []

    class Route:
        def __init__(self, *args, **kwargs):
            self.visit_observed_box = Mock(return_value=True)
            self.dwell_and_return_home = Mock(return_value=True)
            self.dwell_and_leave = Mock(return_value=True)
            self._camera = Mock(return_value=True)
            self.finish_navigation = Mock(return_value=True)
            self.destroy_node, self.emit, self.request_stop = Mock(), Mock(), Mock()
            self.call_operator = Mock(return_value='CRITICAL')
            self.battery_return_due = Mock(return_value=False)
            created.append(self)

    monkeypatch.setattr(box_service, 'BoxServiceRoute', Route)
    box_service.main([*argv, '--log', str(tmp_path / 'default.jsonl')])
    box_service.main([*argv, '--final-gap-m', '0.02', '--dwell-s', '30',
                      '--via-id', 'water_station', '--via-route', str(tmp_path / 'route.yaml'),
                      '--via-region-xy', '0.06', '-1.27', '--log', str(tmp_path / 'trial.jsonl')])
    box_service.main([*argv, '--resume-at-observation', '--resume-at-final-approach',
                      '--surface-mode', 'table_edge',
                      '--log', str(tmp_path / 'resume_final.jsonl')])
    default, trial, resume_final = created
    assert default.final_gap_override_m is None
    assert default.dwell_and_return_home.call_args.args[0] == box_service.STOP_DWELL_S
    assert trial.final_gap_override_m == 0.02
    assert trial.dwell_and_return_home.call_args.args[0] == 30.0
    # The water stop holds as long as the table.
    assert trial.dwell_and_leave.call_args.args[0] == 30.0
    assert resume_final.visit_observed_box.call_args.kwargs[
        'resume_at_observation'] is True
    assert resume_final.visit_observed_box.call_args.kwargs[
        'resume_at_final_approach'] is True


def _final_judged(mission, final_pose, frame_width_m=0.45):
    """Run one visit whose straight approach ended at final_pose; return the last event."""
    mission.capture_stationary_pose = Mock(side_effect=[((0, 0, 0), {}), (final_pose, {})])
    mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540},
                      'frame_width': {'value': frame_width_m}},
        'table_01', (1, 0), 0.6, execute=True, candidate_trial=True)
    return mission.emit.call_args


def test_box_contract_judges_the_nearest_front_corner_at_its_target(mission):
    """Face at x 1.0 facing -x; the contract target is met at the nearest frame corner."""
    from jdamr_cube_navigation.parking import load_parking_contract
    mission.parking_contract = load_parking_contract(BOX_CONTRACT)
    gap = mission.parking_contract['target_front_gap_m']
    finished = _final_judged(mission, (1.0 - 0.065 - gap, 0.0, 0.0))
    assert finished.args == ('box_approach_finished',)
    assert finished.kwargs['desired_front_gap_m'] == pytest.approx(gap)
    assert finished.kwargs['estimated_nearest_front_gap_m'] == pytest.approx(gap)
    # The old 5.5 cm stop is now far from the target.
    assert _final_judged(mission, (1.0 - 0.065 - 0.055, 0.0, 0.0)).args == (
        'box_gap_not_confirmed',)
    # 3 deg inside the contract, centre 0.6 cm: a corner is 0.58 cm inside the face.
    tilt = math.radians(3.0)
    centre_x = 1.0 - 0.006 - 0.065 * math.cos(tilt)
    tilted = _final_judged(mission, (centre_x, 0.0, tilt))
    assert tilted.args == ('box_gap_not_confirmed',)
    assert min(tilted.kwargs['estimated_front_corner_gaps_m']) == pytest.approx(
        0.006 - 0.225 * math.sin(tilt))
    # 2.69 deg with the nearest corner at the target is a park.
    tilt = math.radians(2.69)
    corner_x = 1.0 - gap - box_service.front_lead_m(0.065, 0.225, tilt)
    assert _final_judged(mission, (corner_x, 0.0, tilt)).args == ('box_approach_finished',)


def test_final_gap_option_overrides_the_contract_in_the_judgement(mission):
    from jdamr_cube_navigation.parking import load_parking_contract
    mission.parking_contract = load_parking_contract(BOX_CONTRACT)
    mission.final_gap_override_m = 0.05
    finished = _final_judged(mission, (1.0 - 0.065 - 0.05, 0.0, 0.0))
    assert finished.args == ('box_approach_finished',)
    assert finished.kwargs['desired_front_gap_m'] == pytest.approx(0.05)


def test_table_edge_alignment_pose_still_closes_from_45cm_to_default_5cm():
    node = object.__new__(BoxServiceRoute)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = None
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    node.capture_stationary_pose = Mock(return_value=((0.5, 0.0, 0.0), {}))
    node._pose = Mock(side_effect=lambda _index, pose: SimpleNamespace(
        header=SimpleNamespace(), pose=pose))
    node._frozen_in_odom = Mock(return_value=(
        SimpleNamespace(), lambda x, y, yaw: (x, y, yaw)))
    node._execute_reverse_path = Mock(return_value=True)
    node.emit = Mock()
    node.config = {'frame_id': 'map', 'waypoints': []}
    node.waypoints = []
    target = {
        'face_center_map_xy_m': (1.0, 0.0),
        'outward_normal_map_xy': (-1.0, 0.0),
    }
    reached, _to_odom = node._parking_final_approach(target, 0.065)
    assert reached is True
    straight = next(call for call in node.emit.call_args_list
                    if call.args == ('final_approach_straight',))
    # The robot starts with its front 43.5 cm from the edge and advances toward 5 cm.
    assert straight.kwargs['travel_m'] == pytest.approx(0.39)
    node._execute_reverse_path.assert_called_once()


def test_final_approach_constants_keep_their_measured_basis():
    """0.5 cm: 38 Parking stops ended 0.51 cm short of 5 cm (09-30 to 10-02)."""
    assert box_service.FINAL_STOP_SHORT_M == pytest.approx(0.005)
    assert box_service.FINAL_GAP_TOLERANCE_M == pytest.approx(0.01)
    assert box_service.front_lead_m(0.065, 0.225, 0.0) == pytest.approx(0.065)
    assert box_service.front_lead_m(0.065, 0.225, math.radians(-2.7)) == pytest.approx(
        0.065 * math.cos(math.radians(2.7)) + 0.225 * math.sin(math.radians(2.7)))


def _camera_node():
    node = object.__new__(BoxServiceRoute)
    node.camera_on_demand = True
    node.stop_requested = False
    node.events = []
    node.emit = lambda event, **fields: node.events.append((event, fields))
    return node


def test_box_camera_starts_and_stops_through_systemd_only_on_demand(monkeypatch):
    """2026-10-06: camera and observer held the Pi at 80 deg C; they run for box stops."""
    calls = []
    monkeypatch.setattr(box_service.subprocess, 'run', lambda command, **_kw: calls.append(
        command) or SimpleNamespace(returncode=0, stderr=''))
    node = _camera_node()
    assert node._camera(True) and node._camera(False)
    assert calls == [['sudo', '-n', 'systemctl', 'start', *box_service.CAMERA_UNITS],
                     ['sudo', '-n', 'systemctl', 'stop', *box_service.CAMERA_UNITS]]
    assert [fields['action'] for event, fields in node.events if event == 'box_camera'] == [
        'start', 'stop']
    node.camera_on_demand = False                    # --camera-always-on
    calls.clear()
    assert node._camera(False) is True and calls == []


def test_box_camera_failure_is_reported(monkeypatch):
    node = _camera_node()
    monkeypatch.setattr(box_service.subprocess, 'run', lambda *_a, **_k: SimpleNamespace(
        returncode=1, stderr='Unit not found'))
    assert node._camera(True) is False
    assert node.events[-1][1]['reason'] == 'Unit not found'


def test_box_camera_ready_needs_an_observer_status_after_the_start(monkeypatch):
    node = _camera_node()
    node._camera_requested_s = 100.0
    clock = iter([100.0 + 0.1 * k for k in range(1000)])
    monkeypatch.setattr(box_service.time, 'monotonic', lambda: next(clock))
    statuses = iter([None, 99.0, 99.0, 111.3])         # an old status does not count

    def spin(*_a, **_k):
        node.box_status_received_s = next(statuses, 111.3)
    monkeypatch.setattr(box_service.rclpy, 'spin_once', spin)
    node.box_status_received_s = None
    assert node._camera_ready(timeout_s=30.0) is True
    node = _camera_node()
    node._camera_requested_s = 100.0
    node.box_status_received_s = 50.0
    monkeypatch.setattr(box_service.rclpy, 'spin_once', lambda *_a, **_k: None)
    assert node._camera_ready(timeout_s=1.0) is False
    assert node.events[-1] == ('box_camera_ready', {'ready': False, 'timeout_s': 1.0})


def test_transit_option_selects_the_planner_and_controller_trees():
    base = ['--registry', 'r', '--approach-route', 'a', '--camera-mount', 'm', '--geometry',
            'g', '--log', 'l', '--parking-contract', 'p', '--table-id', 'table_02',
            '--region-xy', '0', '0']
    args = parse_args([*base, '--transit', 'navfn-mppi'])
    assert args.transit == 'navfn-mppi' and args.camera_always_on is False
    with pytest.raises(SystemExit):
        parse_args([*base, '--transit', 'dwb'])


@pytest.mark.parametrize('final_gap_m', [0.010, 0.012])
def test_one_centimetre_table_parking_closes_remaining_gap_in_same_visit(mission, final_gap_m):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.905, 0, 0), {}),
        ((1.0 - 0.065 - final_gap_m, 0, 0), {})]
    deadlines = []

    def approach(*_args):
        deadlines.append(mission.run_deadline_s)
        return True, None

    mission._straight_final_approach.side_effect = approach
    assert invoke(mission)
    assert mission._straight_final_approach.call_count == 2
    assert mission.observe_target.call_count == 4
    assert all(deadline is not None for deadline in deadlines)
    assert mission.run_deadline_s is None
    assert mission.emit.call_args.args == ('box_approach_finished',)
    assert mission.emit.call_args.kwargs['estimated_front_gap_m'] == pytest.approx(final_gap_m)


def test_one_centimetre_table_parking_does_not_accept_two_centimetres(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.915, 0, 0), {}), ((0.925, 0, 0), {})]
    assert invoke(mission)
    assert mission._straight_final_approach.call_count == 2


def test_one_centimetre_table_correction_stops_when_gap_does_not_improve(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.905, 0, 0), {}), ((0.905, 0, 0), {})]
    assert not invoke(mission)
    assert mission._straight_final_approach.call_count == 2
    assert mission.observe_target.call_count == 4
    assert any(call.kwargs.get('reason') == 'final_gap_no_progress'
               for call in mission.emit.call_args_list)


def test_one_centimetre_table_correction_never_reuses_face_after_observation_loss(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    target = _table_edge_final_target(1.0)
    mission.observe_target.side_effect = [
        target, target, target, RuntimeError('depth unavailable')]
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.905, 0, 0), {})]
    assert not invoke(mission)
    assert mission._straight_final_approach.call_count == 2
    assert mission.emit.call_args.kwargs['final_depth_measurement'] is False


def test_one_centimetre_table_correction_never_advances_after_overshoot(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.933, 0, 0), {})]
    assert not invoke(mission)
    mission._straight_final_approach.assert_called_once()
    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)


def test_one_centimetre_table_correction_has_a_bounded_iteration_count(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), *[((x, 0, 0), {}) for x in [0.855, 0.865, 0.875, 0.885]]]
    assert not invoke(mission)
    assert mission._straight_final_approach.call_count == 4
    assert mission.emit.call_args.kwargs['reason'] == 'final_gap_correction_limit'


def test_fine_trim_corrects_small_overshoot_without_restarting_the_visit():
    node = object.__new__(BoxServiceRoute)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    node.capture_stationary_pose = Mock(side_effect=[
        ((0.7, 0.0, 0.0), {}), ((0.7, 0.0, 0.0249), {})])
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node._operator_link_ready = Mock(return_value=True)
    node._rotate_in_place = Mock(side_effect=[0.031, -0.0061])
    node._pose = Mock(return_value=PoseStamped())
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._drive_zero_turn = Mock(return_value=0.3)
    node.emit = Mock()
    reached, _ = node._zero_turn_final_approach({
        'face_center_map_xy_m': (1.0, 0.0),
        'outward_normal_map_xy': (-math.cos(0.025), -math.sin(0.025)),
    }, 0.065)
    assert reached
    assert node._rotate_in_place.call_count == 2
    assert node._rotate_in_place.call_args.args[0] == pytest.approx(-0.006)
    assert node._drive_zero_turn.call_args.kwargs == {'precision': True, 'hold_heading': True}


def test_one_centimetre_table_correction_requires_current_operator_link(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission._operator_link_ready.return_value = False
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.905, 0, 0), {})]
    assert not invoke(mission)
    mission._straight_final_approach.assert_called_once()
    assert mission.emit.call_args.kwargs['reason'] == 'navigation_not_ready'


@pytest.mark.parametrize('remaining_deg, reached_expected', [(0.5, True), (1.1, False)])
def test_fine_trim_residual_is_judged_by_final_heading_contract(remaining_deg, reached_expected):
    node = object.__new__(BoxServiceRoute)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node._operator_link_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(side_effect=[
        ((0.7, 0.0, 0.0), {}), ((0.7, 0.0, math.radians(2.0 - remaining_deg)), {})])
    node._rotate_in_place = Mock(side_effect=[math.radians(2.0 - remaining_deg), 0.0, 0.0])
    node._pose = Mock(return_value=PoseStamped())
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._drive_zero_turn = Mock(return_value=0.3)
    node.emit = Mock()
    heading_rad = math.radians(2.0)
    reached, _ = node._zero_turn_final_approach({
        'face_center_map_xy_m': (1.0, 0.0),
        'outward_normal_map_xy': (-math.cos(heading_rad), -math.sin(heading_rad)),
    }, 0.065)
    assert reached is reached_expected
    assert node._rotate_in_place.call_count == 3
    assert node._drive_zero_turn.call_count == int(reached_expected)


@pytest.mark.parametrize('start_x, stages', [(0.50, [0.05, None]), (0.90, [None])])
def test_table_fine_approach_reobserves_at_visible_stage_without_retreat(start_x, stages):
    node = object.__new__(BoxServiceRoute)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.box_front_half_width_m = 0.225
    node._table_observation_context = ({}, (1.0, 0.0), 0.6)
    node.capture_stationary_pose = Mock(return_value=((start_x, 0.0, 0.0), {}))
    fresh = _table_edge_final_target(1.01)
    node.observe_target = Mock(return_value=fresh)
    calls = []

    def approach(target, extent, **kwargs):
        calls.append((dict(target), extent, kwargs))
        return True, 'odom'

    node._zero_turn_final_approach = Mock(side_effect=approach)
    node.emit = Mock()
    target = _table_edge_final_target(1.0)
    assert node._table_fine_approach(target, 0.065) == (True, 'odom')
    assert [item[2].get('gap_m') for item in calls] == stages
    assert node.observe_target.call_count == len(stages) - 1
    if len(stages) > 1:
        assert calls[-1][0]['face_center_map_xy_m'] == (1.01, 0.0)
        assert callable(calls[0][2]['reobserve'])
    assert calls[-1][2] == {'allow_trim': False}


def test_default_parking_routes_one_centimetre_table_to_precision_approach():
    node = object.__new__(BoxServiceRoute)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.zero_turn_final = False
    node._table_fine_approach = Mock(return_value=(True, 'odom'))
    node._parking_final_approach = Mock()
    target = _table_edge_final_target(1.0)

    assert node._straight_final_approach(target, 0.065) == (True, 'odom')
    node._table_fine_approach.assert_called_once_with(target, 0.065)
    node._parking_final_approach.assert_not_called()


def test_final_table_leg_freezes_face_before_computing_distance():
    node = object.__new__(BoxServiceRoute)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node._operator_link_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=((10.0, 0.0, 0.0), {}))
    node._odom_pose = Mock(return_value=(0.924, 0.0, 0.0))
    node._pose = Mock(return_value=PoseStamped())
    node._frozen_in_odom = Mock(return_value=(RosPath(), lambda x, y, yaw: (x - 10.0, y, yaw)))
    node._drive_zero_turn = Mock(return_value=0.001)
    node.emit = Mock()

    reached, to_odom = node._zero_turn_final_approach(
        _table_edge_final_target(11.0), 0.065, allow_trim=False)

    assert reached
    assert callable(to_odom)
    node._frozen_in_odom.assert_called_once()
    node._drive_zero_turn.assert_called_once_with(
        pytest.approx(0.001), precision=True, hold_heading=True)


def test_occluded_table_escape_keeps_frozen_odom_face_after_amcl_shift():
    node = object.__new__(BoxServiceRoute)
    node.last_box_face = {
        'reference_frame': 'odom',
        'face_center_xy_m': [1.0, 0.0],
        'outward_normal_xy': [-1.0, 0.0],
    }
    node.stop_requested = False
    node.config = {'frame_id': 'map', 'waypoints': []}
    node.waypoints = node.config['waypoints']
    node._parking_parameters_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(side_effect=[
        ((0.980, 0.0, 0.0), {}),
        ((0.555, 0.0, 0.0), {}),
    ])
    node._odom_pose = Mock(side_effect=[
        (0.920, 0.0, 0.0),
        (0.495, 0.0, 0.0),
    ])
    node._make_reverse_path = Mock(return_value=RosPath())
    node._reverse_path_valid = Mock(return_value=True)

    def stale_map_face_transform(*_args):
        raise AssertionError('escape transformed the frozen odom face through current map')

    node._frozen_in_odom = Mock(
        return_value=(RosPath(), stale_map_face_transform))
    node._execute_reverse_path = Mock(return_value=True)
    node.emit = Mock()

    assert not node._leave_parked_pose()

    start, target = node._make_reverse_path.call_args.args
    assert start == pytest.approx((0.980, 0.0, 0.0))
    assert target == pytest.approx((0.495, 0.0, 0.0))
    assert start[0] - target[0] == pytest.approx(0.485)
    result = node.emit.call_args
    assert result.args == ('failed',)
    assert result.kwargs['reason'] == 'box_escape_not_confirmed'
    assert result.kwargs['face_distance_m'] == pytest.approx(0.505)
    assert result.kwargs['face_distance_m'] < 0.515
    assert node.last_box_face['reference_frame'] == 'odom'


def test_table_fine_approach_never_closes_final_leg_after_stage_observation_loss():
    node = object.__new__(BoxServiceRoute)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.box_front_half_width_m = 0.225
    node._table_observation_context = ({}, (1.0, 0.0), 0.6)
    node.capture_stationary_pose = Mock(return_value=((0.5, 0.0, 0.0), {}))
    node.observe_target = Mock(side_effect=RuntimeError('depth unavailable'))
    node._zero_turn_final_approach = Mock(return_value=(True, 'odom'))
    node.emit = Mock()
    assert node._table_fine_approach(_table_edge_final_target(1.0), 0.065) == (False, None)
    node._zero_turn_final_approach.assert_called_once()
    assert node.emit.call_args.kwargs['reason'] == 'table_alignment_observation_lost'


def test_zero_turn_remeasures_face_after_rotation_before_computing_final_distance():
    node = object.__new__(BoxServiceRoute)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    yaw = math.radians(2.0)
    node.capture_stationary_pose = Mock(side_effect=[
        ((0.0, 0.0, 0.0), {}), ((0.0, 0.0, yaw), {}), ((0.0, 0.0, yaw), {})])
    node._rotate_in_place = Mock(return_value=yaw)
    node._pose = Mock(return_value=PoseStamped())
    node._frozen_in_odom = Mock(return_value=(None, 'odom'))
    node._drive_zero_turn = Mock(return_value=0.125)
    node.emit = Mock()
    outward = (-math.cos(yaw), -math.sin(yaw))
    target = {'face_center_map_xy_m': (0.30 * math.cos(yaw), 0.30 * math.sin(yaw)),
              'outward_normal_map_xy': outward}
    fresh = {'face_center_map_xy_m': (0.20 * math.cos(yaw), 0.20 * math.sin(yaw)),
             'outward_normal_map_xy': outward}
    reobserve = Mock(return_value=fresh)
    assert node._zero_turn_final_approach(target, 0.065, reobserve=reobserve) == (True, 'odom')
    reobserve.assert_called_once()
    assert target == fresh
    node._drive_zero_turn.assert_called_once_with(
        pytest.approx(0.125), precision=True, hold_heading=True)


def test_zero_turn_never_advances_using_pre_turn_face_when_new_observation_fails():
    node = object.__new__(BoxServiceRoute)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    yaw = math.radians(2.0)
    node.capture_stationary_pose = Mock(side_effect=[
        ((0.0, 0.0, 0.0), {}), ((0.0, 0.0, yaw), {})])
    node._rotate_in_place = Mock(return_value=yaw)
    node._drive_zero_turn = Mock()
    node.emit = Mock()
    target = {'face_center_map_xy_m': (0.3, 0.0),
              'outward_normal_map_xy': (-math.cos(yaw), -math.sin(yaw))}
    assert node._zero_turn_final_approach(
        target, 0.065, reobserve=Mock(side_effect=RuntimeError('depth lost'))) == (False, None)
    node._drive_zero_turn.assert_not_called()
    assert node.emit.call_args.kwargs['reason'] == 'table_post_trim_observation_lost'


def test_table_fine_approach_does_not_dispatch_final_leg_when_link_lost_during_observation():
    node = object.__new__(BoxServiceRoute)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.box_front_half_width_m = 0.225
    node._table_observation_context = ({}, (1.0, 0.0), 0.6)
    node.capture_stationary_pose = Mock(return_value=((0.5, 0.0, 0.0), {}))
    target = _table_edge_final_target(1.0)

    def observe(*_args):
        node._operator_link_ready.return_value = False
        return target

    node.observe_target = Mock(side_effect=observe)
    node._zero_turn_final_approach = Mock(return_value=(True, 'odom'))
    node.emit = Mock()
    assert node._table_fine_approach(target, 0.065) == (False, None)
    node._zero_turn_final_approach.assert_called_once()
    assert node.emit.call_args.kwargs['reason'] == 'final_trim_interrupted'
