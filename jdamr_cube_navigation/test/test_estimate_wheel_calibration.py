"""Tests for offline wheel-calibration candidate estimation."""

from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from estimate_wheel_calibration import (  # noqa: E402,I100
    estimate_candidates,
    pair_nearest,
    PoseSample,
    wrapped_delta,
)


def _sample(stamp, x, y, yaw):
    return PoseSample(stamp, x, y, yaw)


def test_pair_nearest_rejects_samples_outside_time_gate():
    reference = [_sample(100_000_000, 0.0, 0.0, 0.0)]
    odometry = [_sample(250_000_000, 0.0, 0.0, 0.0)]

    assert pair_nearest(reference, odometry, 0.1) == []


def test_pair_nearest_selects_the_closest_header_time():
    reference = [_sample(150_000_000, 0.0, 0.0, 0.0)]
    odometry = [
        _sample(100_000_000, 1.0, 0.0, 0.0),
        _sample(140_000_000, 2.0, 0.0, 0.0),
        _sample(220_000_000, 3.0, 0.0, 0.0),
    ]

    pairs = pair_nearest(reference, odometry, 0.1)

    assert pairs == [(reference[0], odometry[1])]


def test_wrapped_delta_crosses_pi_without_full_turn_jump():
    assert wrapped_delta(3.1, -3.1) == pytest.approx(0.0831853)


def test_estimate_separates_distance_scale_and_straight_heading_drift():
    reference = []
    odometry = []
    for index in range(6):
        reference.append(_sample(index, index * 0.1, 0.0, 0.0))
        odometry.append(_sample(
            index, index * 0.104, 0.0, index * -0.0024))
    for index in range(5):
        stamp = 6 + index
        reference.append(_sample(stamp, 0.5, 0.0, index * 0.2))
        odometry.append(_sample(
            stamp, 0.52, 0.0, -0.012 + index * 0.204))

    result = estimate_candidates(
        list(zip(reference, odometry)), 0.0329, 0.510)

    assert result['straight_intervals'] == 5
    assert result['turn_intervals'] == 4
    assert result['candidate']['wheel_radius_m'] == pytest.approx(
        0.031635, abs=1e-6)
    assert result['candidate'][
        'wheel_radius_ratio_right_over_left'] == pytest.approx(
            1.01224, abs=1e-6)
    assert result['turn_yaw_scale'] == pytest.approx(1.02)
    assert result['candidate'][
        'turn_based_wheel_separation_m'] == pytest.approx(
            0.500192, abs=1e-6)


def test_estimate_requires_both_motion_classes():
    paired = [
        (_sample(i, i * 0.1, 0.0, 0.0),
         _sample(i, i * 0.1, 0.0, 0.0))
        for i in range(5)
    ]

    with pytest.raises(ValueError, match='turning'):
        estimate_candidates(paired, 0.0329, 0.510)


def test_estimate_applies_drift_correction_to_current_ratio():
    reference = []
    odometry = []
    for index in range(5):
        reference.append(_sample(index, index * 0.1, 0.0, 0.0))
        odometry.append(_sample(
            index, index * 0.1, 0.0, index * -0.002))
    for index in range(4):
        stamp = 5 + index
        reference.append(_sample(stamp, 0.4, 0.0, index * 0.2))
        odometry.append(_sample(stamp, 0.4, 0.0, -0.008 + index * 0.2))

    result = estimate_candidates(
        list(zip(reference, odometry)), 0.0329, 0.5,
        current_radius_ratio=1.02)

    assert result['candidate'][
        'wheel_radius_ratio_right_over_left'] == pytest.approx(1.03)


@pytest.mark.parametrize('field', ('radius', 'separation', 'ratio'))
def test_estimate_rejects_non_finite_geometry(field):
    values = {
        'radius': 0.0329,
        'separation': 0.510,
        'ratio': 1.0,
    }
    values[field] = float('nan')

    with pytest.raises(ValueError, match='finite and positive'):
        estimate_candidates(
            [], values['radius'], values['separation'], values['ratio'])
