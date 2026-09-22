"""Tests for ordering browser teleoperation commands."""

from jdamr_cube_node.web_teleop import CommandOrder, scaled_command

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
