"""Regression tests for real protective-stop latency extraction."""

import sys
from pathlib import Path


EVALUATION_ROOT = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(EVALUATION_ROOT))

from evaluate_real_stop_latency import (  # noqa: E402,I100
    _percentile,
    first_sustained_standstill,
)


def _sample(stamp_ns, linear, angular=0.0):
    return {
        'stamp_ns': stamp_ns,
        'linear_speed_mps': linear,
        'angular_speed_radps': angular,
    }


def test_standstill_requires_a_continuous_dwell():
    """One zero sample between moving samples is not a physical stop proxy."""
    samples = [
        _sample(0, 0.2),
        _sample(20_000_000, 0.0),
        _sample(40_000_000, 0.1),
        _sample(100_000_000, 0.0),
        _sample(150_000_000, 0.0),
        _sample(200_000_000, 0.0),
    ]

    assert first_sustained_standstill(samples, 0, 200_000_000) == 100_000_000


def test_standstill_rejects_angular_motion():
    """Rotating in place is not a standstill."""
    samples = [
        _sample(0, 0.0, 0.1),
        _sample(100_000_000, 0.0, 0.1),
        _sample(200_000_000, 0.0, 0.0),
        _sample(300_000_000, 0.0, 0.0),
    ]

    assert first_sustained_standstill(
        samples, 0, 300_000_000) == 200_000_000


def test_nearest_rank_percentile_is_conservative_for_small_samples():
    """A six-event p95 reports the worst event rather than interpolation."""
    assert _percentile([1.0, 2.0, 3.0, 4.0, 5.0, 100.0], 0.95) == 100.0
