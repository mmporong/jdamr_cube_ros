"""Tests for deterministic simulation-only fault injection."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from jdamr_cube_navigation.sim_fault_injector import (  # noqa: E402,I100
    FaultModel,
    FaultProfile,
)


def test_fault_profile_rejects_unknown_or_unsafe_values():
    """A malformed matrix must fail before any simulation process starts."""
    with pytest.raises(ValueError, match='unsupported'):
        FaultProfile.from_mapping({'unknown_fault': 1.0})
    with pytest.raises(ValueError, match=r'\[0, 1\)'):
        FaultProfile.from_mapping({
            'scan_message_dropout_probability': 1.0,
        })
    with pytest.raises(ValueError, match='positive'):
        FaultProfile.from_mapping({'wheel_translation_scale': 0.0})
    with pytest.raises(ValueError, match='finite'):
        FaultProfile.from_mapping({'scan_transport_delay_s': math.inf})


def test_fault_model_is_seed_reproducible_and_preserves_no_returns():
    """The same seed must reproduce message loss, jitter, and range noise."""
    profile = FaultProfile.from_mapping({
        'scan_message_dropout_probability': 0.2,
        'scan_stamp_jitter_max_s': 0.002164,
        'scan_range_noise_stddev_m': 0.01,
    })
    first = FaultModel(profile, 42)
    second = FaultModel(profile, 42)

    assert [first.drop_scan() for _ in range(20)] == [
        second.drop_scan() for _ in range(20)]
    assert [first.scan_stamp_jitter_s() for _ in range(10)] == [
        second.scan_stamp_jitter_s() for _ in range(10)]
    assert first.noisy_range_m(math.inf, 0.05, 8.0) == math.inf
    assert [first.noisy_range_m(2.0, 0.05, 8.0) for _ in range(10)] == [
        second.noisy_range_m(2.0, 0.05, 8.0) for _ in range(10)]


def test_wheel_slip_is_incremental_and_never_reverses_translation():
    """Large sampled slip may stop an increment but cannot invert motion."""
    model = FaultModel(FaultProfile.from_mapping({
        'wheel_translation_scale': 0.986,
        'wheel_slip_stddev_fraction': 0.8,
    }), 7)

    scales = [model.wheel_increment_scale() for _ in range(100)]

    assert min(scales) >= 0.0
    assert any(scale == 0.0 for scale in scales)
