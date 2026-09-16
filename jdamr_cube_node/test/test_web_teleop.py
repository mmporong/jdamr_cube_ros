"""Tests for ordering browser teleoperation commands."""

from jdamr_cube_node.web_teleop import CommandOrder


def test_rejects_older_sequence_from_same_browser():
    order = CommandOrder()
    assert order.accept('browser-a', 1, 1.0)
    assert order.accept('browser-a', 3, 1.1)
    assert not order.accept('browser-a', 2, 1.2)


def test_active_browser_cannot_be_overridden_until_command_is_stale():
    order = CommandOrder()
    assert order.accept('browser-a', 1, 1.0)
    assert not order.accept('browser-b', 1, 1.1)
    assert order.accept('browser-b', 1, 1.0 + 0.36)


def test_legacy_command_remains_compatible():
    order = CommandOrder()
    assert order.accept('', 0, 1.0)
