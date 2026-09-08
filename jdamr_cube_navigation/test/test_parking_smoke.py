"""Tests for the bounded parking simulation harness."""

import math
from pathlib import Path
import sys

import pytest
import yaml


EVALUATION_ROOT = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(EVALUATION_ROOT))

from navigation_mcap_reader import TOPIC_TYPES  # noqa: E402,I100
from run_parking_smoke import (  # noqa: E402
    _angle_error_rad, _write_route, APPROACH_X_M, GOAL_X_M, GOAL_YAW_RAD,
    START_X_M,
)


def test_parking_angle_error_wraps_across_pi():
    """Keep the evaluator from reporting a full turn at the wrap boundary."""
    assert math.degrees(_angle_error_rad(
        math.radians(-179.0), math.radians(179.0))) == pytest.approx(2.0)


def test_route_has_explicit_final_yaw_and_binds_mask_hash(tmp_path):
    """Create a self-contained route contract without accepting hash drift."""
    image = tmp_path / 'mask.pgm'
    image.write_bytes(b'P5\n1 1\n255\n\x00')
    mask = tmp_path / 'mask.yaml'
    mask.write_text(yaml.safe_dump({'image': image.name}), encoding='utf-8')
    route = tmp_path / 'route.yaml'
    _write_route(route, {'mask': mask})
    document = yaml.safe_load(route.read_text(encoding='utf-8'))
    assert document['waypoints'][-1] == {
        'id': 'parking_target', 'x': GOAL_X_M, 'y': 0.0,
        'yaw': GOAL_YAW_RAD,
    }
    assert document['start_pose']['x'] == START_X_M
    assert document['waypoints'][0]['x'] == APPROACH_X_M
    assert len(document['expected_mask_sha256']) == 64


def test_compact_reader_supports_parking_ground_truth_and_static_tf():
    """Decode every additional topic used by the parking evaluator."""
    assert TOPIC_TYPES['/ground_truth_pose'] == (
        'geometry_msgs/msg/PoseStamped')
    assert TOPIC_TYPES['/tf_static'] == 'tf2_msgs/msg/TFMessage'


def test_simulation_route_uses_the_same_clock_as_sensor_headers():
    """Prevent wall time from making every simulated observation stale."""
    source = (EVALUATION_ROOT / 'run_parking_smoke.py').read_text()
    assert "'--ros-args', '-p', 'use_sim_time:=true'" in source
