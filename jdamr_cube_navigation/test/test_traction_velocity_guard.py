"""Regression tests for bounded simulation traction recovery."""

from collections import deque

from jdamr_cube_navigation.traction_velocity_guard import (
    FAULT_LATCHED,
    GuardConfig,
    LOW_SPEED_RESUME,
    NAVIGATING,
    PROTECTIVE_STOP,
    RECOVERED,
    RELOCALIZE,
    TractionRecoveryState,
    motion_ratio,
    traction_observation_allowed,
)

from nav2_msgs.msg import CollisionMonitorState


def test_one_fault_stops_relocalizes_and_resumes_at_reduced_speed():
    """A sustained first mismatch receives exactly one bounded recovery."""
    policy = TractionRecoveryState(GuardConfig())

    assert policy.update(
        0.0, anomaly=True, localization_stable=False) == NAVIGATING
    assert policy.update(
        0.4, anomaly=True, localization_stable=False) == PROTECTIVE_STOP
    assert policy.velocity_scale() == 0.0
    assert policy.update(
        1.16, anomaly=False, localization_stable=False) == RELOCALIZE
    assert policy.update(
        1.20, anomaly=False, localization_stable=True) == RELOCALIZE
    assert policy.update(
        1.96, anomaly=False,
        localization_stable=True) == LOW_SPEED_RESUME
    assert policy.velocity_scale() == 0.80
    assert policy.update(
        6.96, anomaly=False, localization_stable=True) == RECOVERED
    assert policy.recovery_count == 1


def test_second_sustained_fault_latches_without_another_resume():
    """A recurrence after the one allowed recovery cannot command motion."""
    policy = TractionRecoveryState(GuardConfig())
    policy.update(0.0, anomaly=True, localization_stable=False)
    policy.update(0.4, anomaly=True, localization_stable=False)
    policy.update(1.16, anomaly=False, localization_stable=False)
    policy.update(1.20, anomaly=False, localization_stable=True)
    policy.update(1.96, anomaly=False, localization_stable=True)
    policy.update(4.96, anomaly=False, localization_stable=True)

    assert policy.update(
        7.0, anomaly=True, localization_stable=False) == RECOVERED
    assert policy.update(
        12.01, anomaly=True, localization_stable=False) == RECOVERED
    assert policy.update(
        12.42, anomaly=True, localization_stable=False) == FAULT_LATCHED
    assert policy.velocity_scale() == 0.0
    assert policy.recovery_count == 1


def test_relocalization_timeout_latches_fail_closed():
    """An unstable localization never advances to low-speed motion."""
    policy = TractionRecoveryState(GuardConfig())
    policy.update(0.0, anomaly=True, localization_stable=False)
    policy.update(0.4, anomaly=True, localization_stable=False)
    policy.update(1.16, anomaly=False, localization_stable=False)

    assert policy.update(
        4.16, anomaly=False,
        localization_stable=False) == FAULT_LATCHED
    assert policy.velocity_scale() == 0.0


def test_motion_ratio_uses_independent_endpoint_progress():
    """Wheel motion can exceed actual displacement under synthetic slip."""
    odom = deque(((0.0, 0.0, 0.0), (1.0, 0.20, 0.0)))
    truth = deque(((0.0, 0.0, 0.0), (1.0, 0.08, 0.0)))

    ratio, odom_m, truth_m = motion_ratio(odom, truth)

    assert round(ratio, 6) == 0.4
    assert odom_m == 0.20
    assert truth_m == 0.08


def test_obstacle_stop_is_not_reclassified_as_traction_loss():
    """Collision Monitor owns STOP and a fresh window after it clears."""
    assert not traction_observation_allowed(
        CollisionMonitorState.STOP, 4.0, 4.0, 1.0)
    assert not traction_observation_allowed(
        CollisionMonitorState.DO_NOTHING, 4.5, 4.0, 1.0)
    assert traction_observation_allowed(
        CollisionMonitorState.DO_NOTHING, 5.0, 4.0, 1.0)
