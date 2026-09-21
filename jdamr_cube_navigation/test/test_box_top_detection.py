"""Tests for perception-only depth box-top detection."""

from dataclasses import replace
import math
from pathlib import Path

from jdamr_cube_navigation.box_top_detection import (
    CameraIntrinsics,
    detect_box_front,
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


def _synthetic_box_front_depth():
    depth = np.full((INTRINSICS.height, INTRINSICS.width), 3000,
                    dtype=np.uint16)
    rows, columns = np.mgrid[0:INTRINSICS.height, 0:INTRINSICS.width]
    z_m = 0.80 + 0.20 * (
        (columns - INTRINSICS.center_x_px) / INTRINSICS.focal_x_px)
    x_m = ((columns - INTRINSICS.center_x_px) * z_m
           / INTRINSICS.focal_x_px)
    y_m = ((rows - INTRINSICS.center_y_px) * z_m
           / INTRINSICS.focal_y_px)
    inside = np.abs(x_m - 0.05) <= 0.25
    inside &= y_m >= 0.10
    inside &= y_m <= 0.35
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


def test_detects_metric_box_front_and_yaw_without_a_tag():
    """A bounded vertical face yields distance, center, size and yaw."""
    detection = detect_box_front(_synthetic_box_front_depth(), INTRINSICS)
    assert detection is not None
    assert detection.surface_kind == 'front'
    assert detection.front_distance_m == pytest.approx(0.80, abs=0.05)
    assert detection.center_x_m == pytest.approx(0.05, abs=0.04)
    assert detection.width_m == pytest.approx(0.50, abs=0.08)
    assert detection.height_m == pytest.approx(0.25, abs=0.06)
    assert math.degrees(detection.edge_angle_rad) == pytest.approx(
        13.8, abs=2.0)


@pytest.mark.parametrize(('detector', 'image', 'expected'), [
    (detect_box_top, _synthetic_box_depth(), (
        0.0015271533475867643, -0.200011501866774,
        0.8210000000000001, 0.4488172188556273,
        0.33100000000000007, 0.030090524134165585,
        -6.112095682314795e-07, 0.9999999702582258,
        -0.00024389172618421995,
        5.568407104132557e-05, 817, 817, 0.9987749504370909,
        0.00017533333496980164)),
    (detect_box_front, _synthetic_box_front_depth(), (
        0.029713740277328246, 0.21797089536772835,
        0.807, 0.44574818416031636,
        0.10899999999999999, 0.24037592698272192,
        0.23806774436721712, 0.00038243198637888387,
        -0.9712484763631196,
        0.0013863980572212243, 3632, 3632, 0.9694992427411331,
        0.21415309966542886)),
])
def test_seeded_detection_regression(detector, image, expected):
    """Lock all reported metric and quality values before optimization."""
    detection = detector(image, INTRINSICS)
    assert detection is not None
    actual = (
        detection.center_x_m, detection.center_y_m,
        detection.front_distance_m, detection.width_m,
        detection.depth_extent_m, detection.edge_angle_rad,
        *detection.plane_normal,
        detection.plane_rms_m, detection.inlier_count,
        detection.candidate_count, detection.confidence,
        detection.height_m,
    )
    assert actual == pytest.approx(expected, rel=1e-12, abs=1e-12)


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
