"""Offline differential-drive closure and fault tests, not hardware trials."""

from dataclasses import replace
import math
from pathlib import Path

from jdamr_cube_navigation.box_approach import (
    ApproachConfig, BoxApproach, relative_target, wrap,
)
from jdamr_cube_navigation.parking import load_parking_contract
import pytest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = load_parking_contract(ROOT / 'config/parking_contract.yaml')
# Synthetic fixture geometry; physical geometry is read by the ROS adapter.
CAMERA = (0.065, 0., 0.)
BODY = (-0.275, 0.065, -0.27, 0.27)


def observation(pose=(0., 0., 0.), face=(1.2, 0., 0.), stamp_s=1.):
    """Render an ideal planar face into an offset camera's optical frame."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = face[0] - x, face[1] - y
    return {'stamp_s': stamp_s, 'detected': True, 'stable': True,
            'confidence': .95, 'surface_kind': 'front',
            'front_distance_m': c * dx + s * dy - CAMERA[0],
            'lateral_error_m': s * dx - c * dy,
            'edge_angle_deg': math.degrees(wrap(face[2] - yaw))}


def controller():
    """Create an isolated policy with no ROS graph or motion output."""
    return BoxApproach(CONTRACT, CAMERA, BODY)


@pytest.mark.parametrize('initial', [
    (0., 0., 0.), (0., .15, 0.), (0., -.15, 0.),
    (0., .10, .25), (0., -.10, -.25),
])
@pytest.mark.parametrize('face_yaw', [-.15, 0., .15])
def test_closed_loop_converges_from_offset_and_skew(initial, face_yaw):
    """Integrate bounded commands and feed newly observed geometry back."""
    policy = controller()
    pose, last = initial, (0., 0.)
    dt_s = .05
    for step in range(1800):
        now = 1. + step * dt_s
        sample = observation(pose, face=(1.2, 0., face_yaw), stamp_s=now)
        result = policy.step(sample, pose, now_s=now, odom_stamp_s=now,
                             linear_mps=last[0], angular_radps=last[1],
                             cmd_linear_mps=last[0], cmd_angular_radps=last[1])
        v, w = result['linear_mps'], result['angular_radps']
        assert 0 <= v <= policy.config.max_linear_mps
        assert abs(w) <= policy.config.max_angular_radps
        assert result['state'] != 'ABORTED', result
        if result['state'] == 'SUCCEEDED':
            break
        x, y, yaw = pose
        pose = (x + v * math.cos(yaw) * dt_s,
                y + v * math.sin(yaw) * dt_s, wrap(yaw + w * dt_s))
        last = v, w
    assert result['state'] == 'SUCCEEDED', (result, pose)
    goal, clearance = relative_target(sample, CAMERA, BODY, policy.config)
    assert math.hypot(*goal[:2]) <= .02
    assert abs(goal[2]) <= CONTRACT['yaw_tolerance_rad']
    assert clearance >= policy.config.minimum_clearance_m
    assert v == w == 0.


@pytest.mark.parametrize('blocker', [
    'charging_or_unknown', 'scan_stale', 'collision_stop',
    'command_owner_conflict', 'perception_not_approved',
])
def test_interlock_stops_and_latches_after_acquisition(blocker):
    """Safety faults cannot automatically resume a partly completed approach."""
    policy = controller()
    assert policy.step(observation(), (0., 0., 0.), now_s=1.,
                       odom_stamp_s=1.)['linear_mps'] > 0
    result = policy.step(observation(), (0., 0., 0.), now_s=1.1,
                         odom_stamp_s=1.1, blockers=[blocker])
    assert result['state'] == 'ABORTED'
    assert result['linear_mps'] == result['angular_radps'] == 0
    assert policy.step(observation(), (0., 0., 0.), now_s=1.2,
                       odom_stamp_s=1.2)['state'] == 'ABORTED'
    policy.reset()
    assert policy.goal is None


@pytest.mark.parametrize('patch', [
    {'stamp_s': .1}, {'stamp_s': 2.}, {'front_distance_m': math.nan},
    {'detected': False}, {'surface_kind': 'top'}, {'confidence': .1},
    {'front_distance_m': .30},
])
def test_unusable_observations_never_produce_motion(patch):
    """Missing, invalid, too-near and stale observations fail closed."""
    sample = observation()
    sample.update(patch)
    result = controller().step(sample, (0., 0., 0.), now_s=1., odom_stamp_s=1.)
    assert result['linear_mps'] == result['angular_radps'] == 0


def test_target_switch_and_manual_relocation_abort():
    """A different plane or a jumped odometry pose invalidates the old target."""
    for sample, pose in [(observation(face=(1.6, 0., 0.)), (0., 0., 0.)),
                         (observation(), (.5, 0., 0.))]:
        policy = controller()
        policy.step(observation(), (0., 0., 0.), now_s=1., odom_stamp_s=1.)
        result = policy.step(sample, pose, now_s=1.1, odom_stamp_s=1.1)
        assert result['state'] == 'ABORTED'


def test_repeated_sample_cannot_complete_hold():
    """A frozen perception frame cannot count as consecutive observations."""
    policy = controller()
    sample = observation(face=(.515, 0., 0.))
    for index in range(20):
        result = policy.step(sample, (0., 0., 0.), now_s=1. + index * .02,
                             odom_stamp_s=1. + index * .02)
        assert result['state'] != 'SUCCEEDED'


def test_configuration_rejects_blind_five_centimetre_target():
    """The initial controller cannot be configured for a blind close finish."""
    with pytest.raises(ValueError):
        replace(ApproachConfig(), standoff_m=.05)


@pytest.mark.parametrize('changes', [
    {'standoff_m': .05, 'minimum_clearance_m': .01, 'position_tolerance_m': .01},
    {'max_linear_mps': 10.}, {'max_angular_radps': 10.}, {'max_age_s': 30.},
    {'target_jump_m': 5.}, {'min_confidence': .1}, {'minimum_clearance_m': .1},
])
def test_parameters_cannot_weaken_the_tested_envelope(changes):
    """Multiple parameter overrides cannot bypass the operating bounds."""
    with pytest.raises(ValueError):
        replace(ApproachConfig(), **changes)


def test_offset_and_normal_have_correct_signs():
    """A right-hand face produces a right-hand goal, not reversed steering."""
    sample = observation(face=(1.2, -.2, -.1))
    goal, _ = relative_target(sample, CAMERA, BODY, ApproachConfig())
    assert goal[1] < 0 and goal[2] < 0
