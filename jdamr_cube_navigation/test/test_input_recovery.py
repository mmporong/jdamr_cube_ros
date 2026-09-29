"""Bounded input recovery without sending commands to robot hardware."""

from unittest.mock import Mock

from jdamr_cube_navigation import corridor_route
from jdamr_cube_navigation.corridor_route import CorridorRoute
import pytest


@pytest.mark.parametrize('reason,expected', [
    ('scan stale: age=0.6s limit=0.5s', True),
    ('odom stale: age=0.6s limit=0.5s', True),
    ('AMCL stale after motion', True),
    ('battery low: voltage=10.4V', False),
    ('AMCL x covariance high: value=1.0', False),
    ('live map changed', False),
    ('operator interrupt', False),
])
def test_only_temporary_input_gaps_are_recoverable(reason, expected):
    assert CorridorRoute._input_gap_recoverable(reason) is expected


def test_recovery_requires_new_stopped_odometry_and_never_dispatches(monkeypatch):
    node = object.__new__(CorridorRoute)
    node.stop_requested = False
    node._guard_failure = Mock(return_value=None)
    node.get_logger = lambda: Mock()
    clock = [0.0]
    node.latest_motion = (-1.0, 0.0, 0.0)
    monkeypatch.setattr(corridor_route.time, 'monotonic', lambda: clock[0])

    def spin(_node, timeout_sec):
        clock[0] += 0.1
        node.latest_motion = (clock[0], 0.02 if clock[0] < 0.3 else 0.0, 0.0)

    monkeypatch.setattr(corridor_route.rclpy, 'spin_once', spin)
    assert node._wait_for_input_recovery('scan stale: age=1s')
    assert clock[0] >= 0.3


def test_operator_stop_never_resumes(monkeypatch):
    node = object.__new__(CorridorRoute)
    node.stop_requested = True
    node.get_logger = lambda: Mock()
    node._guard_failure = Mock(return_value=None)
    spin = Mock()
    monkeypatch.setattr(corridor_route.rclpy, 'spin_once', spin)
    assert not node._wait_for_input_recovery('scan stale: age=1s')
    spin.assert_not_called()


def test_recovery_retries_only_remaining_waypoint_once():
    node = object.__new__(CorridorRoute)
    node.stop_requested = False
    node._wait_for_input_recovery = Mock(return_value=True)
    calls = []

    def attempt():
        calls.append(node._resume_waypoint_index)
        node._resume_waypoint_index = 1
        node._retry_guard_reason = 'scan stale: age=1s'
        return False

    assert not node._run_with_input_recovery(attempt)
    assert calls == [0, 1]
    node._wait_for_input_recovery.assert_called_once()
