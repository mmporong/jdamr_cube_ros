"""Regression tests for recorded sensor and timing profiles."""

from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


EVALUATION_ROOT = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(EVALUATION_ROOT))

pytest.importorskip('mcap_ros2',
                    reason='evaluation deps are installed separately')

from profile_sensor_streams import (  # noqa: E402,I100
    distribution,
    nearest_offsets_s,
    robust_sigma,
    summarize_imu,
    summarize_odometry,
    summarize_scan,
    timing_profile,
)


def test_distribution_ignores_nonfinite_values():
    """Non-finite values must not poison an exported quantile."""
    result = distribution([1.0, 2.0, float('nan'), float('inf')])

    assert result['samples'] == 2
    assert result['p50'] == pytest.approx(1.5)
    assert distribution([]) == {'samples': 0}


def test_robust_sigma_resists_a_single_motion_spike():
    """One bump must not become the stationary IMU noise parameter."""
    quiet_with_spike = [-0.01, -0.005, 0.0, 0.005, 0.01, 5.0]

    assert robust_sigma(quiet_with_spike) == pytest.approx(
        0.0111195, abs=1e-7)


def test_timing_profile_keeps_jitter_and_latency_separate():
    """Transport delay is not the same quantity as period jitter."""
    header_ns = [1_000_000_000, 1_100_000_000, 1_205_000_000]
    recorder_ns = [1_010_000_000, 1_112_000_000, 1_220_000_000]

    result, _series = timing_profile(recorder_ns, header_ns)

    assert result['messages'] == 3
    assert result['observed_rate_hz'] == pytest.approx(9.524, abs=0.001)
    assert result['header_interval_s']['p50'] == pytest.approx(0.1025)
    assert result['header_abs_jitter_from_median_s']['max'] == pytest.approx(
        0.0025)
    assert result['recorder_minus_header_s']['max'] == pytest.approx(0.015)
    yaml.safe_dump(result)


def test_nearest_offsets_preserve_which_stream_was_late():
    """Signed offsets must retain target-minus-reference direction."""
    offsets_s = nearest_offsets_s(
        [1_000_000_000, 2_000_000_000],
        [1_010_000_000, 1_980_000_000])

    assert offsets_s.tolist() == pytest.approx([0.01, -0.02])


def test_scan_profile_separates_invalid_reasons():
    """A dropout is different from a finite out-of-range return."""
    frames = [{
        'ranges_m': [0.2, 1.0, float('inf'), 9.0],
        'range_min_m': 0.3,
        'range_max_m': 8.0,
        'scan_time_s': 0.1,
        'time_increment_s': 0.001,
    }]

    result, _series = summarize_scan(frames)

    assert result['points_total'] == 4
    assert result['points_valid'] == 1
    assert result['valid_fraction'] == pytest.approx(0.25)
    assert result['nonfinite_fraction'] == pytest.approx(0.25)
    assert result['invalid_reason_counts'] == {
        'nan': 0,
        'positive_infinity': 1,
        'negative_infinity': 0,
        'below_range_min': 1,
        'above_range_max': 1,
    }


def test_odometry_wraps_yaw_and_separates_stationary_increments():
    """Crossing the angle boundary must not look like a full rotation."""
    samples = [
        {'header_stamp_ns': 1, 'x_m': 0.0, 'y_m': 0.0,
         'yaw_rad': np.deg2rad(179.0), 'linear_mps': 0.0,
         'angular_radps': 0.0},
        {'header_stamp_ns': 2, 'x_m': 0.001, 'y_m': 0.0,
         'yaw_rad': np.deg2rad(-179.0), 'linear_mps': 0.0,
         'angular_radps': 0.0},
        {'header_stamp_ns': 3, 'x_m': 0.101, 'y_m': 0.0,
         'yaw_rad': np.deg2rad(-170.0), 'linear_mps': 0.2,
         'angular_radps': 0.1},
    ]

    result, series = summarize_odometry(
        samples, stationary_linear_mps=0.01,
        stationary_angular_radps=0.02)

    assert result['stationary_classification']['stationary_samples'] == 2
    assert result['stationary_translation_increment_m']['samples'] == 1
    assert result['stationary_translation_increment_m']['p50'] == pytest.approx(
        0.001)
    assert series['abs_yaw_increment_rad'][0] == pytest.approx(
        np.deg2rad(2.0))


def test_imu_profile_inherits_nearest_odom_motion_class():
    """IMU noise summaries must use the matching odometry motion segment."""
    odom_series = {
        'header_stamps_ns': [100, 200, 300],
        'stationary': [True, True, False],
    }
    samples = [
        {'header_stamp_ns': 110, 'x_radps': 0.0, 'y_radps': 0.0,
         'z_radps': 0.01},
        {'header_stamp_ns': 290, 'x_radps': 0.0, 'y_radps': 0.0,
         'z_radps': 0.5},
    ]

    result, series = summarize_imu(samples, odom_series)

    assert result['stationary']['samples'] == 1
    assert result['moving']['samples'] == 1
    assert series['stationary_z_radps'].tolist() == pytest.approx([0.01])
    assert series['moving_z_radps'].tolist() == pytest.approx([0.5])


def test_imu_profile_leaves_distant_odometry_unclassified():
    """A stale odometry state must not contaminate stationary IMU noise."""
    odom_series = {
        'header_stamps_ns': [100_000_000],
        'stationary': [True],
    }
    samples = [
        {'header_stamp_ns': 300_000_000, 'x_radps': 0.0, 'y_radps': 0.0,
         'z_radps': 0.01},
    ]

    result, series = summarize_imu(
        samples, odom_series, max_alignment_s=0.05)

    assert result['odom_motion_alignment']['classified_samples'] == 0
    assert result['odom_motion_alignment']['unclassified_samples'] == 1
    assert result['stationary']['samples'] == 0
    assert result['moving']['samples'] == 0
    assert series['stationary_z_radps'].size == 0
