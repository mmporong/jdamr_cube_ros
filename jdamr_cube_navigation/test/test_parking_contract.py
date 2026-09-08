"""Tests for the opt-in precision parking contract and pure observer."""

import math
from copy import deepcopy  # noqa: I100
from pathlib import Path

from jdamr_cube_navigation.parking import (  # noqa: I101
    load_parking_contract, parking_controller_overrides, ParkingHold,
    pose_errors,
)

import pytest

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / 'config/parking_contract.yaml'
NAV2_PATH = ROOT / 'config/nav2_params.yaml'


def _contract():
    return load_parking_contract(CONTRACT_PATH)


def _nav2():
    return yaml.safe_load(NAV2_PATH.read_text(encoding='utf-8'))


def test_contract_is_user_target_and_not_physical_measurement():
    """Keep the requested values separate from physical validation."""
    contract = _contract()

    assert contract['schema_version'] == 1
    assert contract['reference_frame'] == 'map'
    assert contract['robot_base_frame'] == 'base_link'
    assert contract['target_basis'] == 'USER_ACCEPTED_TARGET'
    assert contract['physical_validation'] == 'NOT_PHYSICALLY_VALIDATED'
    assert contract['yaw_tolerance_rad'] == pytest.approx(math.radians(3.0))


@pytest.mark.parametrize('field,value', [
    ('schema_version', True),
    ('xy_tolerance_m', float('nan')),
    ('hold_s', 0.0),
    ('reference_frame', 'odom'),
    ('physical_validation', 'VALIDATED'),
])
def test_contract_rejects_invalid_or_overclaimed_values(
        tmp_path, field, value):
    """Reject malformed values and unsupported validation claims."""
    document = yaml.safe_load(CONTRACT_PATH.read_text(encoding='utf-8'))
    document[field] = value
    path = tmp_path / 'parking.yaml'
    path.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(ValueError):
        load_parking_contract(path)


def test_contract_requires_timeout_longer_than_hold(tmp_path):
    """Leave time to observe the complete stationary hold."""
    document = yaml.safe_load(CONTRACT_PATH.read_text(encoding='utf-8'))
    document['observation_timeout_s'] = document['hold_s']
    path = tmp_path / 'parking.yaml'
    path.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(ValueError, match='timeout must exceed hold'):
        load_parking_contract(path)


def test_overrides_only_append_parking_plugins_and_deep_copy_follow_path():
    """Preserve production controller settings while adding parking."""
    nav2 = _nav2()
    before = deepcopy(nav2)
    original = nav2['controller_server']['ros__parameters']

    configured = parking_controller_overrides(nav2, _contract())

    assert nav2 == before
    assert configured['controller_plugins'] == (
        original['controller_plugins'] + ['Parking'])
    assert configured['goal_checker_plugins'] == (
        original['goal_checker_plugins'] + ['parking_goal_checker'])
    unchanged = set(original) - {
        'controller_plugins', 'goal_checker_plugins', 'FollowPath'}
    assert all(configured[key] == original[key] for key in unchanged)
    assert configured['FollowPath'] == original['FollowPath']
    assert configured['Parking']['plugin'] == original['FollowPath']['plugin']
    assert configured['Parking']['desired_linear_vel'] == 0.08
    assert configured['Parking']['min_approach_linear_velocity'] == 0.02
    assert configured['Parking']['regulated_linear_scaling_min_speed'] == 0.02
    assert configured['Parking']['rotate_to_heading_angular_vel'] == 0.2
    assert configured['Parking']['use_rotate_to_heading'] is True
    assert configured['Parking']['allow_reversing'] is False
    assert configured['Parking']['stateful'] is False
    checker = configured['parking_goal_checker']
    assert checker == {
        'plugin': 'nav2_controller::SimpleGoalChecker',
        'stateful': False,
        'xy_goal_tolerance': 0.05,
        'yaw_goal_tolerance': pytest.approx(math.radians(3.0)),
    }
    assert all(value.get('plugin') != 'nav2_controller::StoppedGoalChecker'
               for value in configured.values() if isinstance(value, dict))


@pytest.mark.parametrize('field,value', [
    ('plugin', 'another_controller'),
    ('use_collision_detection', False),
])
def test_override_rejects_non_rpp_or_collision_disabled_source(field, value):
    """Never derive the parking controller from a weaker safety profile."""
    nav2 = _nav2()
    nav2['controller_server']['ros__parameters']['FollowPath'][field] = value

    with pytest.raises(ValueError, match='collision-enabled'):
        parking_controller_overrides(nav2, _contract())


def test_pose_errors_wrap_yaw_and_reject_nonfinite_values():
    """Use shortest wrapped yaw distance and finite poses only."""
    distance, yaw = pose_errors(
        (0.0, 0.0, math.radians(179.0)),
        (0.03, 0.04, math.radians(-179.0)))
    assert distance == pytest.approx(0.05)
    assert yaw == pytest.approx(math.radians(2.0))

    with pytest.raises(ValueError):
        pose_errors((0.0, 0.0, 0.0), (math.inf, 0.0, 0.0))
    with pytest.raises(ValueError):
        pose_errors(None, (0.0, 0.0, 0.0))


def _observe(hold, now, **overrides):
    values = {
        'target_pose': (1.0, 2.0, 0.0),
        'actual_pose': (1.01, 1.99, math.radians(1.0)),
        'linear_mps': 0.0,
        'angular_radps': 0.0,
        'cmd_linear_mps': 0.0,
        'cmd_angular_radps': 0.0,
        'sample_age_s': 0.05,
    }
    values.update(overrides)
    return hold.observe(now, **values)


def test_hold_requires_every_condition_continuously_for_one_second():
    """Confirm only fresh, stopped, zero-command, accurate observations."""
    hold = ParkingHold(_contract())

    assert _observe(hold, 10.0)['reason'] == 'holding'
    assert _observe(hold, 10.5)['confirmed'] is False
    result = _observe(hold, 11.0)
    assert result['confirmed'] is True
    assert result['hold_s'] == pytest.approx(1.0)
    assert result['physical_accuracy'] == 'NOT_MEASURED'


@pytest.mark.parametrize('overrides,reason', [
    ({'actual_pose': (1.06, 2.0, 0.0)}, 'position_out_of_tolerance'),
    ({'actual_pose': (1.0, 2.0, math.radians(4.0))},
     'yaw_out_of_tolerance'),
    ({'linear_mps': 0.011}, 'odometry_not_stopped'),
    ({'angular_radps': 0.021}, 'odometry_not_stopped'),
    ({'cmd_linear_mps': 1e-6}, 'command_not_zero'),
    ({'sample_age_s': 0.51}, 'stale_sample'),
])
def test_hold_resets_on_each_failed_condition(overrides, reason):
    """Report the failed condition and restart continuous hold timing."""
    hold = ParkingHold(_contract())
    _observe(hold, 1.0)
    result = _observe(hold, 1.2, **overrides)
    assert result['confirmed'] is False
    assert result['reason'] == reason
    assert result['hold_s'] == 0.0
    assert _observe(hold, 1.3)['hold_s'] == 0.0


def test_hold_resets_on_gap_time_regression_and_nonfinite_observation():
    """Disallow discontinuous or malformed observation timelines."""
    hold = ParkingHold(_contract())
    _observe(hold, 2.0)
    assert _observe(hold, 2.51)['reason'] == 'observation_gap'
    assert _observe(hold, 2.4)['reason'] == 'time_regression'
    result = _observe(hold, 2.5, angular_radps=float('nan'))
    assert result['reason'] == 'non_finite_or_invalid_observation'
    assert result['position_error_m'] is None
    assert result['physical_accuracy'] == 'NOT_MEASURED'


def test_hold_distinguishes_observation_timeout_from_short_gap():
    """Report a full observer timeout separately from a continuity gap."""
    hold = ParkingHold(_contract())
    _observe(hold, 1.0)

    result = _observe(hold, 6.01)

    assert result['reason'] == 'observation_timeout'
    assert result['hold_s'] == 0.0
