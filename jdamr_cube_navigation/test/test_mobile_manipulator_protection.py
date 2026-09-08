"""Tests for the fixed-stow mobile-manipulator protection contract."""

import ast
import math
from pathlib import Path
import time

from jdamr_cube_navigation.corridor_route import CorridorRoute
from jdamr_cube_navigation.mobile_manipulator_protection import (
    evaluate_travel_pose,
    load_mobile_manipulator_protection,
)

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROTECTION_PATH = (
    PACKAGE_ROOT / 'config' / 'mobile_manipulator_protection.yaml')
ONBOARD_CORE_LAUNCH = (
    PACKAGE_ROOT / 'launch' / 'onboard_nav2_core.launch.py')


def _contract():
    return load_mobile_manipulator_protection(PROTECTION_PATH)


def _ready_route(contract):
    route = object.__new__(CorridorRoute)
    now = time.monotonic()
    route.samples = {
        name: now for name in ('battery', 'odom', 'scan', 'joint_states')}
    route.freshness_s = 2.5
    route.battery_freshness_s = 30.0
    route.battery_voltage = 12.0
    route.minimum_battery_v = 10.5
    route.mobile_manipulator_protection = contract
    route.travel_pose_status = evaluate_travel_pose(
        list(contract['travel_pose']['joints_rad']),
        list(contract['travel_pose']['joints_rad'].values()), contract)
    route.amcl_seen = now
    route.amcl_covariance = (0.25, 0.25)
    route.amcl_position = (0.0, 0.0)
    route.max_amcl_covariance = (50.0, 50.0)
    route.amcl_freshness_s = 60.0
    route.start_check_pending = False
    return route


def test_runtime_contract_has_the_derived_rectangles_and_fixed_pose():
    """Load the installed values with explicit metric geometry."""
    contract = _contract()

    assert contract['claim_scope'] == (
        'fixed_stow_pose_candidate_not_safety_certification')
    assert contract['validated_geometry_m']['stop_zone'] == {
        'front_m': 0.4, 'rear_m': -0.28, 'half_width_m': 0.25}
    assert contract['validated_geometry_m']['slowdown_zone'] == {
        'front_m': 0.5, 'rear_m': -0.38, 'half_width_m': 0.35}
    assert contract['travel_pose']['position_tolerance_rad'] == 0.03


def test_travel_pose_evaluation_is_fail_closed():
    """Reject missing, duplicate, or out-of-tolerance joint samples."""
    contract = _contract()
    joints = contract['travel_pose']['joints_rad']

    passed = evaluate_travel_pose(
        list(joints), list(joints.values()), contract)
    assert passed['status'] == 'PASS'
    assert passed['maximum_error_rad'] == 0.0

    positions = list(joints.values())
    positions[1] += 0.031
    failed = evaluate_travel_pose(list(joints), positions, contract)
    assert failed['status'] == 'FAIL'
    assert failed['maximum_error_rad'] == pytest.approx(0.031)

    with pytest.raises(ValueError, match='duplicate joint names'):
        evaluate_travel_pose(['arm_elbow_flex', 'arm_elbow_flex'],
                             [1.5, 1.5], contract)


def test_obstacle_profile_applies_protection_only_to_collision_monitor():
    """Keep the expanded polygons scoped to the candidate monitor."""
    source = ONBOARD_CORE_LAUNCH.read_text(encoding='utf-8')

    ast.parse(source)
    assert "if profile == 'obstacle_candidate':" in source
    assert "protection['collision_monitor_overrides']" in source
    assert ("'collision_monitor', collision_monitor_parameters" in source)


def test_corridor_route_blocks_stale_or_non_stowed_arm_state():
    """Cancel or block candidate navigation when the arm contract is lost."""
    contract = _contract()
    route = _ready_route(contract)

    assert route._guard_failure() is None

    route.travel_pose_status = {
        'status': 'FAIL', 'maximum_error_rad': 0.031}
    assert route._guard_failure().startswith('travel pose invalid:')

    route = _ready_route(contract)
    route.samples['joint_states'] = time.monotonic() - 2.6
    assert route._guard_failure().startswith('joint_states stale:')


@pytest.mark.parametrize('bad_position', [math.nan, math.inf, -math.inf])
def test_travel_pose_rejects_non_finite_measurements(bad_position):
    """Treat NaN and infinity as invalid travel-pose evidence."""
    contract = _contract()
    joints = contract['travel_pose']['joints_rad']
    positions = list(joints.values())
    positions[0] = bad_position

    result = evaluate_travel_pose(list(joints), positions, contract)

    assert result['status'] == 'FAIL'
    assert result['maximum_error_rad'] is None
