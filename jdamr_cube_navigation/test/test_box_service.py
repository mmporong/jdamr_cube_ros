"""Verify bounded table transit and face-based parking without hardware."""

from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from jdamr_cube_navigation import box_service
from jdamr_cube_navigation.box_service import BoxServiceRoute, parse_args
from jdamr_cube_navigation.docking_stop_profile import apply_docking_stop_profile
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
PARAMS = ROOT / 'jdamr_cube_navigation/config/new_base_nav2_params.yaml'
GEOMETRY = ROOT / 'jdamr_cube_description/config/new_base_geometry.yaml'


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
    for method in ('verify_live_maps', 'emit'):
        setattr(node, method, Mock())
    for method in ('wait_until_ready', '_parking_parameters_ready', 'preflight', 'execute'):
        setattr(node, method, Mock(return_value=True))
    node._precision_collision_ready = Mock(return_value=True)
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
    assert mission.execute.call_count == 3
    assert [call.kwargs for call in mission.execute.call_args_list] == [
        {'final_parking': False}, {'final_parking': False, 'alignment': True},
        {'final_parking': True, 'alignment': False}]
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [0.45, 0.05]
    assert mission.emit.call_args.args == ('box_approach_finished',)
    assert mission.emit.call_args.kwargs['estimated_front_gap_m'] == pytest.approx(0.05)
    assert mission.emit.call_args.kwargs['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'


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
    assert mission.execute.call_count == 2
    mission.preflight.assert_not_called()
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [0.45, 0.05]
    mission.verify_live_maps.assert_called()
    mission._precision_collision_ready.assert_called_once()


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
    assert mission.execute.call_count == 3
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
    with pytest.raises(RuntimeError, match='search rotation failed'):
        invoke(mission, search=True)
    mission.search_rotation.assert_called_once_with(math.radians(30.0))
    assert mission.observe_target.call_count == 1


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
    mission.execute.assert_called_once_with(final_parking=False, alignment=True)


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


def test_corner_gap_is_checked_even_inside_yaw_tolerance(mission):
    yaw = math.radians(0.9)
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.959 - 0.065 * math.cos(yaw), 0, yaw), {})]
    assert not invoke(mission)
    assert min(mission.emit.call_args.kwargs['estimated_front_corner_gaps_m']) < 0.04


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
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node.emit = Mock()
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
    assert node.emit.call_args.args == ('box_lidar_witness_rejected',)
    assert node.emit.call_args.kwargs['scan_frame'] == 'base_link'


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
    assert node.emit.call_count == 1
    evidence = node.emit.call_args.kwargs
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
    node.emit.assert_called_once()


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
