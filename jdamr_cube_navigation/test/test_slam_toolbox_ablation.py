"""Tests for reproducible SLAM Toolbox parameter ablations."""

from copy import deepcopy
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from make_slam_toolbox_ablation import apply_overrides  # noqa: E402,I100


def _document():
    return {
        'slam_toolbox': {
            'ros__parameters': {
                'max_laser_range': 8.0,
                'minimum_travel_distance': 0.5,
                'minimum_travel_heading': 0.5,
                'do_loop_closing': True,
            },
        },
    }


def test_ablation_changes_only_named_parameters():
    """One experiment must not silently change unrelated settings."""
    original = _document()
    derived = apply_overrides(
        deepcopy(original),
        {'max_laser_range': 3.5})

    expected = deepcopy(original)
    expected['slam_toolbox']['ros__parameters']['max_laser_range'] = 3.5
    assert derived == expected


def test_ablation_rejects_unknown_parameter_names():
    """Typographical parameter names must fail before a long replay."""
    with pytest.raises(ValueError, match='unsupported overrides'):
        apply_overrides(_document(), {'unrelated': 1})


def test_ablation_requires_parameter_to_exist_in_base():
    """The derived file must remain anchored to the reviewed base config."""
    document = _document()
    del document['slam_toolbox']['ros__parameters']['max_laser_range']

    with pytest.raises(ValueError, match='base parameter is missing'):
        apply_overrides(document, {'max_laser_range': 3.5})
