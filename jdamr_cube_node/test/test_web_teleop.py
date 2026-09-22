"""Tests for ordering browser teleoperation commands."""

import threading
import time

from jdamr_cube_node.web_teleop import (
    CommandOrder,
    TeleopNode,
    scaled_command,
)

import pytest


def test_rejects_older_sequence_from_same_browser():
    """A delayed request cannot overwrite a newer held-key command."""
    order = CommandOrder()
    assert order.accept('browser-a', 1, 1.0)
    assert order.accept('browser-a', 3, 1.1)
    assert not order.accept('browser-a', 2, 1.2)


def test_active_browser_cannot_be_overridden_until_command_is_stale():
    """Only one browser owns the controller within the freshness window."""
    order = CommandOrder()
    assert order.accept('browser-a', 1, 1.0)
    assert not order.accept('browser-b', 1, 1.1)
    assert order.accept('browser-b', 1, 1.0 + 0.36)


def test_legacy_command_remains_compatible():
    """Existing clients without ordering metadata remain accepted."""
    order = CommandOrder()
    assert order.accept('', 0, 1.0)


def test_scaled_command_clamps_normalized_input():
    """Browser values outside the unit interval remain bounded."""
    assert scaled_command(2.0, -2.0, 0.15, 1.2) == (0.15, -1.2)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_scaled_command_rejects_nonfinite_values(value):
    """Nonfinite values never reach the motor command topic."""
    with pytest.raises(ValueError, match='finite'):
        scaled_command(value, 0.0, 0.15, 1.2)


def test_page_exposes_fail_closed_preflight_state():
    """The browser disables movement until live readiness passes."""
    from jdamr_cube_node.web_teleop import PAGE
    assert 'state.drive_ready === true' in PAGE
    assert '출발 조건 확인 중 · 방향 입력 차단' in PAGE
    assert 'button:disabled' in PAGE


def test_nonzero_command_is_rejected_when_preflight_is_not_ready():
    """A browser cannot bypass the ROS-side readiness gate."""
    node = object.__new__(TeleopNode)
    node.require_preflight = True
    node.preflight_stale_sec = 1.5
    node._lock = threading.RLock()
    node._preflight = {'ready': False, 'blockers': ['라이다 입력 지연']}
    node._preflight_stamp = time.monotonic()
    assert node.set_cmd(0.5, 0.0, 'browser-a', 1) == (
        False, 'preflight_not_ready')


def test_stale_preflight_cannot_authorize_motion():
    """A formerly healthy graph cannot leave movement latched as ready."""
    node = object.__new__(TeleopNode)
    node.require_preflight = True
    node.preflight_stale_sec = 1.5
    node._lock = threading.RLock()
    node._preflight = {'ready': True, 'blockers': []}
    node._preflight_stamp = time.monotonic() - 2.0
    assert not node._drive_ready()


def test_blocked_direction_cannot_authorize_motion():
    """A direction blocked by current LiDAR points remains rejected."""
    node = object.__new__(TeleopNode)
    node.require_preflight = True
    node.preflight_stale_sec = 1.5
    node._lock = threading.RLock()
    node._preflight = {
        'ready': True,
        'blockers': [],
        'checks': {'directions': {
            'forward': {'clear': False, 'point_count': 3},
            'backward': {'clear': True, 'point_count': 0},
            'left': {'clear': True, 'point_count': 0},
            'right': {'clear': True, 'point_count': 0},
        }},
    }
    node._preflight_stamp = time.monotonic()
    assert node.set_cmd(0.5, 0.0, 'browser-a', 1) == (
        False, 'direction_blocked:forward')
