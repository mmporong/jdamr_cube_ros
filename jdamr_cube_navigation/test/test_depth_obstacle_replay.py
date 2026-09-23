"""Tests for bounded offline depth-obstacle replay helpers."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import pytest

import yaml


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / 'evaluation' / 'replay_depth_obstacles.py'
)
SPEC = importlib.util.spec_from_file_location(
    'replay_depth_obstacles', SCRIPT)
REPLAY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPLAY)


def _tf(parent, child, x=0.0, rotation=None):
    """Build the subset of TransformStamped used by the evaluator."""
    if rotation is None:
        rotation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=x, y=0.0, z=0.0),
        rotation=rotation,
    )
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=parent),
        child_frame_id=child,
        transform=transform,
    )


def _transform_matrix(transform):
    """Provide a small test substitute for the node TF helper."""
    result = np.eye(4)
    result[:3, 3] = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return result


def test_resolves_only_an_invariant_recorded_camera_chain():
    """The sensor-to-camera path composes child-to-parent transforms."""
    edges = {
        'optical': [('color', np.eye(4), '/tf')] * 2,
        'color': [('camera_link', np.array([
            [1.0, 0.0, 0.0, 0.2],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]), '/tf')],
    }
    matrix, frames, topics = REPLAY._resolve_invariant_chain(
        edges, 'optical', 'camera_link')
    assert frames == ['optical', 'color', 'camera_link']
    assert topics == ['/tf']
    assert matrix[0, 3] == pytest.approx(0.2)


def test_rejects_missing_or_changing_recorded_transform():
    """Replay never fabricates a missing or moving camera transform."""
    with pytest.raises(ValueError, match='missing recorded transform'):
        REPLAY._resolve_invariant_chain({}, 'optical', 'camera_link')
    changed = np.eye(4)
    changed[0, 3] = 0.1
    edges = {'optical': [
        ('camera_link', np.eye(4), '/tf'),
        ('camera_link', changed, '/tf'),
    ]}
    with pytest.raises(ValueError, match='not static'):
        REPLAY._resolve_invariant_chain(
            edges, 'optical', 'camera_link')


def test_accepts_only_equivalent_multiple_recorded_paths():
    """A duplicate-parent driver tree is safe only when paths agree."""
    identity = np.eye(4)
    edges = {
        'optical': [('color', identity, '/tf')],
        'color': [
            ('camera_link', identity, '/tf'),
            ('depth', identity, '/tf'),
        ],
        'depth': [('camera_link', identity, '/tf')],
    }
    _, frames, _ = REPLAY._resolve_invariant_chain(
        edges, 'optical', 'camera_link')
    assert frames == ['optical', 'color', 'camera_link']
    shifted = np.eye(4)
    shifted[1, 3] = 0.01
    edges['depth'] = [('camera_link', shifted, '/tf')]
    with pytest.raises(ValueError, match='ambiguous recorded transform paths'):
        REPLAY._resolve_invariant_chain(
            edges, 'optical', 'camera_link')


def test_uniform_cap_spans_the_full_recording():
    """A bounded selection retains frames near both recording ends."""
    selected = REPLAY._selected_depth_indices(3038, 15, 100)
    assert len(selected) == 100
    assert min(selected) == 0
    assert max(selected) == 3030


def test_loads_only_measured_base_camera_mount(tmp_path):
    """Mount loading validates provenance state and frame direction."""
    path = tmp_path / 'mount.yaml'
    path.write_text(yaml.safe_dump({
        'camera_mount': {
            'status': 'measured',
            'parent_frame': 'base_link',
            'child_frame': 'camera_link',
            'transform': {
                'x_m': 0.065, 'y_m': 0.0, 'z_m': 0.215,
                'roll_rad': 0.0, 'pitch_rad': 0.0, 'yaw_rad': 0.0,
            },
        },
    }))
    runtime = SimpleNamespace(transform_matrix=_transform_matrix)
    matrix, document = REPLAY._load_measured_mount(path, runtime)
    np.testing.assert_allclose(matrix[:3, 3], [0.065, 0.0, 0.215])
    assert document['camera_mount']['status'] == 'measured'


def test_statistics_are_json_safe_for_empty_and_populated_series():
    """Aggregate output has stable keys and native JSON numbers."""
    assert REPLAY._statistics([]) == {
        'mean': None, 'p95': None, 'max': None}
    values = REPLAY._statistics([1.0, 2.0, 3.0])
    assert values['mean'] == pytest.approx(2.0)
    assert values['p95'] == pytest.approx(2.9)
    assert isinstance(values['max'], float)
