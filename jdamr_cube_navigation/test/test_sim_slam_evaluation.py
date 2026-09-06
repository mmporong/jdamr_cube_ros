"""Tests for simulated SLAM ground-truth metrics."""

import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from evaluate_sim_slam import (  # noqa: E402,I100
    absolute_trajectory_error,
    analyse,
    metric_groups_are_finite,
    relative_pose_error,
    time_match,
)


def _pose(timestamp_s, x_m, y_m=0.0, yaw_rad=0.0):
    return (timestamp_s, x_m, y_m, yaw_rad)


def test_time_match_rejects_stale_ground_truth():
    """Missing truth must not be silently paired across a long gap."""
    estimated = [_pose(1.0, 0.0), _pose(2.0, 1.0)]
    truth = [_pose(1.02, 0.0), _pose(2.2, 1.0)]

    pairs = time_match(estimated, truth, max_offset_s=0.075)

    assert pairs == [(estimated[0], truth[0])]


def test_ate_removes_only_global_se2_frame_offset():
    """A different map origin is not trajectory error."""
    truth = [_pose(float(i), float(i)) for i in range(4)]
    estimated = [
        _pose(float(i), 10.0, float(i), math.pi / 2.0)
        for i in range(4)
    ]

    result = absolute_trajectory_error(list(zip(estimated, truth)))

    assert result['translation_rms_m'] == pytest.approx(0.0, abs=1e-12)
    assert result['yaw_rms_rad'] == pytest.approx(0.0, abs=1e-12)


def test_ate_reports_scale_drift_without_rescaling():
    """Rigid alignment must not hide metric scale drift."""
    truth = [_pose(float(i), float(i)) for i in range(4)]
    estimated = [_pose(float(i), float(i) * 1.1) for i in range(4)]

    result = absolute_trajectory_error(list(zip(estimated, truth)))

    assert result['translation_rms_m'] > 0.1


def test_rpe_measures_fixed_one_second_motion_error():
    """Ten-percent local translation drift remains visible in RPE."""
    truth = [_pose(i * 0.5, i * 0.5) for i in range(7)]
    estimated = [_pose(i * 0.5, i * 0.55) for i in range(7)]

    result = relative_pose_error(list(zip(estimated, truth)), delta_s=1.0)

    assert result['samples'] == 5
    assert result['translation_rms_m'] == pytest.approx(0.1)
    assert result['yaw_rms_rad'] == pytest.approx(0.0)


def test_rpe_wraps_heading_error_at_pi_boundary():
    """Equivalent wrapped headings must not produce a 2-pi error."""
    truth = [_pose(0.0, 0.0, yaw_rad=math.pi - 0.01),
             _pose(1.0, 1.0, yaw_rad=-math.pi + 0.01)]
    estimated = [_pose(0.0, 0.0, yaw_rad=-math.pi - 0.01),
                 _pose(1.0, 1.0, yaw_rad=-math.pi + 0.01)]

    result = relative_pose_error(list(zip(estimated, truth)))

    assert result['yaw_max_rad'] < 0.03


def test_analyse_rejects_nonpositive_commanded_path(tmp_path):
    """Completion ratios must never be computed from invalid commands."""
    with pytest.raises(ValueError):
        analyse(tmp_path / 'missing.mcap', 'cartographer', 0.0)


def test_metric_validity_rejects_nonfinite_ate_or_rpe():
    """A finite ATE cannot hide a non-finite RPE result."""
    assert metric_groups_are_finite(
        {'translation_rms_m': 0.1}, {'translation_rms_m': 0.02})
    assert not metric_groups_are_finite(
        {'translation_rms_m': 0.1}, {'translation_rms_m': math.inf})
