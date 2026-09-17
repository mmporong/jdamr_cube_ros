"""Tests for perception-only depth box-top detection."""

from dataclasses import replace
import math
from pathlib import Path

from jdamr_cube_navigation.box_top_detection import (
    CameraIntrinsics,
    detect_box_top,
    DetectionStability,
)
import numpy as np
import pytest


INTRINSICS = CameraIntrinsics(
    width=320,
    height=240,
    focal_x_px=285.1711,
    focal_y_px=285.1711,
    center_x_px=159.5,
    center_y_px=119.5,
)


def _synthetic_box_depth(
        center_x_m=0.0, top_y_m=-0.20,
        front_m=0.80, back_m=1.20, width_m=0.50):
    """Project a horizontal box top over a distant vertical background."""
    depth = np.full((INTRINSICS.height, INTRINSICS.width), 3000,
                    dtype=np.uint16)
    rows, columns = np.mgrid[0:INTRINSICS.height, 0:INTRINSICS.width]
    denominator = rows - INTRINSICS.center_y_px
    with np.errstate(divide='ignore', invalid='ignore'):
        z_m = top_y_m * INTRINSICS.focal_y_px / denominator
        x_m = ((columns - INTRINSICS.center_x_px) * z_m
               / INTRINSICS.focal_x_px)
    inside = np.isfinite(z_m)
    inside &= z_m >= front_m
    inside &= z_m <= back_m
    inside &= np.abs(x_m - center_x_m) <= width_m / 2.0
    depth[inside] = np.rint(z_m[inside] * 1000.0).astype(np.uint16)
    return depth


def test_detects_metric_box_top_without_a_tag():
    """A horizontal near-field rectangle yields a metric observation."""
    detection = detect_box_top(_synthetic_box_depth(), INTRINSICS)
    assert detection is not None
    assert detection.front_distance_m == pytest.approx(0.80, abs=0.05)
    assert detection.center_x_m == pytest.approx(0.0, abs=0.03)
    assert detection.width_m == pytest.approx(0.50, abs=0.08)
    assert math.degrees(detection.edge_angle_rad) == pytest.approx(
        0.0, abs=2.0)
    assert detection.confidence > 0.6


def test_rejects_a_distant_vertical_wall():
    """A wall alone cannot masquerade as a nearby box top."""
    depth = np.full((240, 320), 1500, dtype=np.uint16)
    assert detect_box_top(depth, INTRINSICS) is None


def test_rejects_floor_below_the_camera():
    """A horizontal plane with positive optical Y is treated as floor."""
    floor = _synthetic_box_depth(top_y_m=0.20)
    assert detect_box_top(floor, INTRINSICS) is None


def test_detection_stability_requires_consecutive_consistency():
    """Detection readiness resets after a missing or jumping observation."""
    detection = detect_box_top(_synthetic_box_depth(), INTRINSICS)
    assert detection is not None
    tracker = DetectionStability(required_frames=3)
    assert tracker.update(detection) is False
    assert tracker.update(detection) is False
    assert tracker.update(detection) is True
    shifted = replace(detection, front_distance_m=1.0)
    assert tracker.update(shifted) is False
    assert tracker.frame_count == 1
    assert tracker.update(None) is False
    assert tracker.frame_count == 0


def test_ros_observer_is_perception_only():
    """The first-stage observer must not own a velocity publisher."""
    source = (
        Path(__file__).resolve().parents[1]
        / 'jdamr_cube_navigation' / 'depth_box_parking.py'
    ).read_text(encoding='utf-8')
    assert 'create_publisher(Twist' not in source
    assert "'/cmd_vel'" not in source
