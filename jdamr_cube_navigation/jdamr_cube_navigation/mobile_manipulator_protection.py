"""Load and evaluate the fixed travel-pose protection contract."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import yaml


OVERRIDE_KEYS = {'StopZone.points', 'SlowdownZone.points'}


def _rectangle(points_text: str, label: str) -> dict[str, float]:
    points = json.loads(points_text)
    if (not isinstance(points, list) or len(points) != 4
            or any(not isinstance(point, list) or len(point) != 2
                   for point in points)
            or any(not math.isfinite(float(value))
                   for point in points for value in point)):
        raise ValueError(f'{label} must be a finite rectangle')
    front_m = max(float(point[0]) for point in points)
    rear_m = min(float(point[0]) for point in points)
    half_width_m = max(abs(float(point[1])) for point in points)
    expected = {
        (front_m, half_width_m), (front_m, -half_width_m),
        (rear_m, -half_width_m), (rear_m, half_width_m),
    }
    if {(float(point[0]), float(point[1])) for point in points} != expected:
        raise ValueError(f'{label} must be an axis-aligned rectangle')
    return {'front_m': front_m, 'rear_m': rear_m,
            'half_width_m': half_width_m}


def load_mobile_manipulator_protection(path: Path) -> dict[str, Any]:
    """Load a strict fixed-pose and Collision Monitor override contract."""
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError(
            'mobile manipulator protection schema_version must be 1')
    travel_pose = document.get('travel_pose')
    if not isinstance(travel_pose, dict):
        raise ValueError('travel_pose must be a mapping')
    source_topic = travel_pose.get('source_topic')
    if (not isinstance(source_topic, str)
            or not source_topic.startswith('/')):
        raise ValueError('travel pose source_topic must be an absolute topic')
    tolerance_rad = travel_pose.get('position_tolerance_rad')
    joints = travel_pose.get('joints_rad')
    if (not isinstance(tolerance_rad, (int, float))
            or not math.isfinite(float(tolerance_rad))
            or not 0.0 < float(tolerance_rad) <= 0.1):
        raise ValueError('travel pose tolerance must be in (0, 0.1] rad')
    if (not isinstance(joints, dict) or not joints
            or any(not str(name).startswith('arm_') for name in joints)
            or any(not isinstance(value, (int, float))
                   or not math.isfinite(float(value))
                   for value in joints.values())):
        raise ValueError('travel pose joints must be finite arm positions')
    overrides = document.get('collision_monitor_overrides')
    if not isinstance(overrides, dict) or set(overrides) != OVERRIDE_KEYS:
        raise ValueError('Collision Monitor override keys drifted')
    stop = _rectangle(overrides['StopZone.points'], 'StopZone')
    slowdown = _rectangle(overrides['SlowdownZone.points'], 'SlowdownZone')
    if slowdown['front_m'] <= stop['front_m']:
        raise ValueError('SlowdownZone must extend beyond StopZone')
    document['validated_geometry_m'] = {
        'stop_zone': stop, 'slowdown_zone': slowdown}
    return document


def load_base_obstacle_protection(path: Path) -> dict[str, Any]:
    """Load a base-only Collision Monitor override contract."""
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('base obstacle protection schema_version must be 1')
    if document.get('claim_scope') != (
            'base_only_obstacle_candidate_not_safety_certification'):
        raise ValueError('base obstacle protection claim_scope drifted')
    if 'travel_pose' in document:
        raise ValueError('base-only protection must not declare travel_pose')
    overrides = document.get('collision_monitor_overrides')
    if not isinstance(overrides, dict) or set(overrides) != OVERRIDE_KEYS:
        raise ValueError('Collision Monitor override keys drifted')
    stop = _rectangle(overrides['StopZone.points'], 'StopZone')
    slowdown = _rectangle(overrides['SlowdownZone.points'], 'SlowdownZone')
    if slowdown['front_m'] <= stop['front_m']:
        raise ValueError('SlowdownZone must extend beyond StopZone')
    document['validated_geometry_m'] = {
        'stop_zone': stop, 'slowdown_zone': slowdown}
    return document


def evaluate_travel_pose(
        names: list[str], positions: list[float], contract: dict[str, Any],
) -> dict[str, Any]:
    """Return a fail-closed comparison of one JointState sample."""
    if len(names) != len(positions):
        raise ValueError('JointState name and position lengths differ')
    if len(set(names)) != len(names):
        raise ValueError('JointState contains duplicate joint names')
    measured = dict(zip(names, positions))
    expected = contract['travel_pose']['joints_rad']
    missing = sorted(set(expected) - set(measured))
    errors = {
        name: abs(float(measured[name]) - float(target))
        for name, target in expected.items() if name in measured}
    finite = all(math.isfinite(error) for error in errors.values())
    tolerance_rad = float(contract['travel_pose']['position_tolerance_rad'])
    passed = (
        not missing and bool(errors) and finite
        and max(errors.values()) <= tolerance_rad)
    return {
        'status': 'PASS' if passed else 'FAIL',
        'missing_joints': missing,
        'absolute_errors_rad': errors,
        'maximum_error_rad': (
            max(errors.values()) if errors and finite else None),
        'tolerance_rad': tolerance_rad,
    }
