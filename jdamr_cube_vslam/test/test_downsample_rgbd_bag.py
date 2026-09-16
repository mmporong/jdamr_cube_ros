"""Tests for synchronized RGB-D processing-bag frame selection."""

from jdamr_cube_vslam.downsample_rgbd_bag import (
    ImageFrame,
    select_synchronized_frames,
)
import pytest


def test_selects_rate_limited_one_to_one_pairs():
    color = [ImageFrame(index, index * 50_000_000) for index in range(7)]
    depth = [
        ImageFrame(index, index * 50_000_000 + 10_000_000)
        for index in range(7)
    ]
    selected_color, selected_depth, deltas = select_synchronized_frames(
        color, depth, image_fps=10.0, max_pair_delta_ms=30.0)
    assert selected_color == {0, 2, 4, 6}
    assert selected_depth == {0, 2, 4, 6}
    assert deltas == [10_000_000] * 4


def test_rejects_pairs_outside_sync_window():
    color = [ImageFrame(0, 100_000_000)]
    depth = [ImageFrame(0, 150_000_000)]
    selected_color, selected_depth, deltas = select_synchronized_frames(
        color, depth, image_fps=10.0, max_pair_delta_ms=30.0)
    assert not selected_color
    assert not selected_depth
    assert not deltas


def test_sensor_timestamps_can_arrive_out_of_recording_order():
    color = [
        ImageFrame(2, 200_000_000),
        ImageFrame(0, 0),
        ImageFrame(1, 100_000_000),
    ]
    depth = [
        ImageFrame(1, 110_000_000),
        ImageFrame(2, 210_000_000),
        ImageFrame(0, 10_000_000),
    ]
    selected_color, selected_depth, deltas = select_synchronized_frames(
        color, depth, image_fps=10.0, max_pair_delta_ms=30.0)
    assert selected_color == {0, 1, 2}
    assert selected_depth == {0, 1, 2}
    assert deltas == [10_000_000] * 3


@pytest.mark.parametrize('fps,delta_ms', [(0.0, 30.0), (10.0, 0.0)])
def test_rejects_non_positive_limits(fps, delta_ms):
    with pytest.raises(ValueError):
        select_synchronized_frames([], [], fps, delta_ms)
