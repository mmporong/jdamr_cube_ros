"""Tests for the deterministic simulation-only SLAM route."""

import math

from jdamr_cube_navigation.sim_slam_route import (
    corridor_segments,
    segment_at,
    square_segments,
)
import pytest


def test_square_route_has_four_equal_sides_and_turns():
    """The planned command sequence must close one square."""
    segments = square_segments(
        side_m=0.8, linear_mps=-0.2, angular_radps=0.65)

    sides = [segment for segment in segments
             if segment.name.startswith('side_')]
    turns = [segment for segment in segments
             if segment.name.startswith('turn_')]
    assert len(sides) == 4
    assert len(turns) == 4
    assert all(segment.duration_s == pytest.approx(4.0)
               for segment in sides)
    assert all(
        abs(segment.angular_radps) * segment.duration_s
        == pytest.approx(math.pi / 2.0)
        for segment in turns)


def test_route_starts_and_ends_stationary():
    """Settle and stop commands bracket every moving segment."""
    segments = square_segments()
    total_duration_s = sum(segment.duration_s for segment in segments)

    assert segment_at(segments, 0.0).name == 'settle'
    assert segment_at(segments, 2.0).name == 'side_1'
    assert segment_at(segments, total_duration_s - 0.01).name == 'stop'
    assert segment_at(segments, total_duration_s) is None
    assert segment_at(segments, -0.1) is None


def test_corridor_route_drives_out_turns_pi_and_returns():
    """The long route must command equal outbound and return distances."""
    segments = corridor_segments(
        distance_m=14.0, linear_mps=0.35, angular_radps=0.5)

    assert [segment.name for segment in segments] == [
        'settle', 'outbound', 'turnaround', 'return', 'stop']
    assert segments[1].linear_mps * segments[1].duration_s == 14.0
    assert segments[3].linear_mps * segments[3].duration_s == 14.0
    assert (segments[2].angular_radps * segments[2].duration_s
            == pytest.approx(math.pi))
