"""Tests for candidate Keepout mask reprojection."""

import math
from pathlib import Path
import sys

import numpy as np
import pytest


pytest.importorskip('cv2')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from reproject_keepout_mask import (  # noqa: E402,I100
    grid_indices,
    inverse_transform_points,
    reproject_mask,
    transform_points,
    validate_source_grids,
)


def _metadata():
    return {
        'resolution': 0.1,
        'origin': [-0.5, -0.5, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }


def test_transform_and_inverse_are_consistent():
    points = np.asarray([[0.0, 0.0], [1.0, -2.0], [-0.5, 0.8]])
    transformed = transform_points(
        points, math.radians(7.0), (0.3, -0.2))

    restored = inverse_transform_points(
        transformed, math.radians(7.0), (0.3, -0.2))

    assert restored == pytest.approx(points)


def test_positive_rotation_moves_source_x_toward_target_y():
    transformed = transform_points(
        np.asarray([[1.0, 0.0]]), math.pi / 2.0, (0.0, 0.0))

    assert transformed == pytest.approx(np.asarray([[0.0, 1.0]]), abs=1e-9)


def test_identity_reprojection_preserves_mask_cells():
    source = np.full((10, 10), 254, dtype=np.uint8)
    source[2:5, 6:9] = 0

    projected = reproject_mask(
        _metadata(), source, _metadata(), source.shape, 0.0, (0.0, 0.0))

    assert np.array_equal(projected, source)


def test_translation_moves_keepout_in_map_coordinates():
    source = np.full((10, 10), 254, dtype=np.uint8)
    source[5, 5] = 0

    projected = reproject_mask(
        _metadata(), source, _metadata(), source.shape, 0.0, (0.1, 0.0))

    assert projected[5, 6] == 0
    assert np.count_nonzero(projected == 0) == 1


def test_target_origin_change_samples_the_same_world_cell():
    source = np.full((10, 10), 254, dtype=np.uint8)
    source[5, 5] = 0
    target_metadata = _metadata()
    target_metadata['origin'] = [-0.4, -0.5, 0.0]

    projected = reproject_mask(
        _metadata(), source, target_metadata, (8, 8), 0.0, (0.0, 0.0))

    assert projected[3, 4] == 0
    assert np.count_nonzero(projected == 0) == 1


def test_grid_indices_drop_points_outside_the_target():
    points = np.asarray([[0.05, 0.05], [10.0, 10.0]])

    rows, columns = grid_indices(points, _metadata(), (10, 10))

    assert rows.tolist() == [4]
    assert columns.tolist() == [5]


def test_source_map_and_mask_must_share_one_grid():
    metadata = _metadata()
    moved = _metadata()
    moved['origin'] = [-0.4, -0.5, 0.0]

    with pytest.raises(ValueError, match='grids differ'):
        validate_source_grids(metadata, (10, 10), moved, (10, 10))
