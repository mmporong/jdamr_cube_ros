"""Pure helpers for an opt-in, not-yet-validated precision parking target."""

from __future__ import annotations

import math
from copy import deepcopy  # noqa: I100
from pathlib import Path
from typing import Any, Sequence

import yaml


REQUIRED_CONTRACT = {
    'schema_version', 'reference_frame', 'robot_base_frame',
    'xy_tolerance_m', 'yaw_tolerance_deg', 'stopped_linear_mps',
    'stopped_angular_radps', 'hold_s', 'observation_timeout_s',
    'sample_max_age_s', 'desired_linear_mps',
    'min_approach_linear_mps', 'rotate_angular_radps', 'target_basis',
    'physical_validation',
}
POSITIVE_FIELDS = {
    'xy_tolerance_m', 'yaw_tolerance_deg', 'stopped_linear_mps',
    'stopped_angular_radps', 'hold_s', 'observation_timeout_s',
    'sample_max_age_s', 'desired_linear_mps',
    'min_approach_linear_mps', 'rotate_angular_radps',
}
COMMAND_ZERO_EPSILON = 1e-9


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f'{name} must be a finite number')
    return number


def load_parking_contract(path: Path) -> dict:
    """Load and strictly validate the user-accepted parking target."""
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError('parking contract must be a mapping')
    missing = REQUIRED_CONTRACT - document.keys()
    unknown = document.keys() - REQUIRED_CONTRACT
    if missing or unknown:
        raise ValueError(
            f'parking contract keys invalid: missing={sorted(missing)}, '
            f'unknown={sorted(unknown)}')
    if (isinstance(document['schema_version'], bool)
            or not isinstance(document['schema_version'], int)
            or document['schema_version'] != 1):
        raise ValueError('parking contract schema_version must be 1')
    if document['reference_frame'] != 'map':
        raise ValueError('parking reference_frame must be map')
    if document['robot_base_frame'] != 'base_link':
        raise ValueError('parking robot_base_frame must be base_link')
    if document['target_basis'] != 'USER_ACCEPTED_TARGET':
        raise ValueError('parking values must be user-accepted targets')
    if document['physical_validation'] != 'NOT_PHYSICALLY_VALIDATED':
        raise ValueError('parking values must remain physically unvalidated')
    for name in POSITIVE_FIELDS:
        document[name] = _finite_number(document[name], name)
        if document[name] <= 0.0:
            raise ValueError(f'{name} must be positive')
    if document['min_approach_linear_mps'] > document['desired_linear_mps']:
        raise ValueError('minimum approach speed exceeds desired speed')
    if document['observation_timeout_s'] <= document['hold_s']:
        raise ValueError('observation timeout must exceed hold duration')
    if document['yaw_tolerance_deg'] > 180.0:
        raise ValueError('yaw_tolerance_deg must not exceed 180')
    document['yaw_tolerance_rad'] = math.radians(
        document['yaw_tolerance_deg'])
    return document


def parking_controller_overrides(
        nav2_document: dict, contract: dict) -> dict:
    """Return an opt-in controller parameter copy with parking plugins."""
    try:
        original = nav2_document['controller_server']['ros__parameters']
        goal_checkers = original['goal_checker_plugins']
        controllers = original['controller_plugins']
        follow_path = original['FollowPath']
    except (KeyError, TypeError) as error:
        raise ValueError('invalid Nav2 controller document') from error
    if not all(isinstance(value, list)
               for value in (goal_checkers, controllers)):
        raise ValueError('Nav2 plugin inventories must be lists')
    if not isinstance(follow_path, dict):
        raise ValueError('FollowPath configuration must be a mapping')
    if (follow_path.get('plugin') != (
            'nav2_regulated_pure_pursuit_controller::'
            'RegulatedPurePursuitController')
            or follow_path.get('use_collision_detection') is not True):
        raise ValueError(
            'parking requires collision-enabled Regulated Pure Pursuit')
    if ('Parking' in controllers or 'Parking' in original
            or 'parking_goal_checker' in goal_checkers
            or 'parking_goal_checker' in original):
        raise ValueError('parking plugins already exist')

    configured = deepcopy(original)
    configured['controller_plugins'].append('Parking')
    configured['goal_checker_plugins'].append('parking_goal_checker')
    parking = deepcopy(configured['FollowPath'])
    parking.update({
        'desired_linear_vel': contract['desired_linear_mps'],
        'min_approach_linear_velocity': contract[
            'min_approach_linear_mps'],
        'regulated_linear_scaling_min_speed': contract[
            'min_approach_linear_mps'],
        'rotate_to_heading_angular_vel': contract['rotate_angular_radps'],
        'use_rotate_to_heading': True,
        'allow_reversing': False,
        'stateful': False,
    })
    configured['Parking'] = parking
    configured['parking_goal_checker'] = {
        'plugin': 'nav2_controller::SimpleGoalChecker',
        'stateful': False,
        'xy_goal_tolerance': contract['xy_tolerance_m'],
        'yaw_goal_tolerance': contract['yaw_tolerance_rad'],
    }
    return configured


def pose_errors(target_xyz: Sequence[float],
                actual_xyz: Sequence[float]) -> tuple[float, float]:
    """Return planar distance and shortest absolute yaw error."""
    try:
        sizes = len(target_xyz), len(actual_xyz)
    except TypeError as error:
        raise ValueError('parking poses must be finite sequences') from error
    if sizes != (3, 3):
        raise ValueError('parking poses must contain x, y, and yaw')
    target = tuple(_finite_number(value, 'target_pose')
                   for value in target_xyz)
    actual = tuple(_finite_number(value, 'actual_pose')
                   for value in actual_xyz)
    distance_m = math.hypot(actual[0] - target[0], actual[1] - target[1])
    yaw_delta_rad = actual[2] - target[2]
    yaw_error_rad = abs(math.atan2(
        math.sin(yaw_delta_rad), math.cos(yaw_delta_rad)))
    return distance_m, yaw_error_rad


class ParkingHold:
    """Confirm fresh, stationary in-tolerance observations for a full hold."""

    def __init__(self, contract: dict):
        """Create an empty hold timeline from a validated contract."""
        self.contract = deepcopy(contract)
        self._hold_started_s: float | None = None
        self._last_observation_s: float | None = None

    def _result(self, reason: str, position_error_m: float | None = None,
                yaw_error_rad: float | None = None, hold_s: float = 0.0,
                confirmed: bool = False) -> dict:
        return {
            'confirmed': confirmed,
            'reason': reason,
            'position_error_m': position_error_m,
            'yaw_error_rad': yaw_error_rad,
            'hold_s': hold_s,
            'physical_accuracy': 'NOT_MEASURED',
        }

    def _reset(self) -> None:
        self._hold_started_s = None

    def observe(
            self, now_s: float, target_pose: Sequence[float],
            actual_pose: Sequence[float], linear_mps: float,
            angular_radps: float, cmd_linear_mps: float,
            cmd_angular_radps: float, sample_age_s: float) -> dict:
        """Update the hold using one caller-guaranteed unique sample."""
        try:
            now = _finite_number(now_s, 'now_s')
            sample_age = _finite_number(sample_age_s, 'sample_age_s')
            motion = tuple(_finite_number(value, 'velocity') for value in (
                linear_mps, angular_radps, cmd_linear_mps,
                cmd_angular_radps))
            position_error_m, yaw_error_rad = pose_errors(
                target_pose, actual_pose)
        except (TypeError, ValueError):
            self._reset()
            self._last_observation_s = None
            return self._result('non_finite_or_invalid_observation')

        if (self._last_observation_s is not None
                and now < self._last_observation_s):
            self._reset()
            self._last_observation_s = now
            return self._result(
                'time_regression', position_error_m, yaw_error_rad)
        if (self._last_observation_s is not None
                and now - self._last_observation_s
                > self.contract['observation_timeout_s']):
            self._reset()
            self._last_observation_s = now
            return self._result(
                'observation_timeout', position_error_m, yaw_error_rad)
        if (self._last_observation_s is not None
                and now - self._last_observation_s
                > self.contract['sample_max_age_s']):
            self._reset()
            self._last_observation_s = now
            return self._result(
                'observation_gap', position_error_m, yaw_error_rad)
        self._last_observation_s = now
        if sample_age < 0.0 or sample_age > self.contract['sample_max_age_s']:
            self._reset()
            return self._result(
                'stale_sample', position_error_m, yaw_error_rad)

        conditions = (
            ('position_out_of_tolerance',
             position_error_m <= self.contract['xy_tolerance_m']),
            ('yaw_out_of_tolerance',
             yaw_error_rad <= self.contract['yaw_tolerance_rad']),
            ('odometry_not_stopped',
             abs(motion[0]) <= self.contract['stopped_linear_mps']
             and abs(motion[1]) <= self.contract['stopped_angular_radps']),
            ('command_not_zero',
             abs(motion[2]) <= COMMAND_ZERO_EPSILON
             and abs(motion[3]) <= COMMAND_ZERO_EPSILON),
        )
        for reason, satisfied in conditions:
            if not satisfied:
                self._reset()
                return self._result(reason, position_error_m, yaw_error_rad)

        if self._hold_started_s is None:
            self._hold_started_s = now
        held = max(0.0, now - self._hold_started_s)
        confirmed = held >= self.contract['hold_s']
        return self._result(
            'confirmed' if confirmed else 'holding', position_error_m,
            yaw_error_rad, held, confirmed)
