"""Tests for constrained offline box-alignment candidate fitting."""

import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

import pytest


SCRIPT = (Path(__file__).resolve().parents[1]
          / 'evaluation' / 'fit_box_alignment_candidate.py')
SPEC = importlib.util.spec_from_file_location(
    'box_alignment_candidate', SCRIPT)
ALIGNMENT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ALIGNMENT)


def _synthetic_face(yaw=0.08, pitch=-0.05, candidate_x=0.82,
                    y_values=None, z_values=None):
    """Build optical points whose corrected pose lies on a vertical face."""
    if y_values is None:
        y_values = np.linspace(-0.30, 0.30, 15)
    if z_values is None:
        z_values = np.linspace(0.10, 0.80, 12)
    slope = 0.12
    intercept = 1.20
    nominal = np.eye(4)
    nominal[:3, 3] = [0.65, 0.04, 0.25]
    candidate = nominal.copy()
    candidate[:3, :3] = ALIGNMENT.rotation_yaw_pitch(yaw, pitch)
    candidate[0, 3] = candidate_x
    yy, zz = np.meshgrid(y_values, z_values)
    corrected = np.column_stack((
        slope * yy.ravel() + intercept, yy.ravel(), zz.ravel()))
    optical = ((corrected - candidate[:3, 3])
               @ candidate[:3, :3])
    laser_y = np.linspace(-0.35, 0.35, 30)
    laser = np.column_stack((
        slope * laser_y + intercept,
        laser_y,
        np.zeros_like(laser_y),
    ))
    return optical, laser, nominal, candidate


def _write_capture(path):
    height = width = 8
    angles = np.linspace(-0.15, 0.15, 21)
    optical_to_base = np.eye(4)
    optical_to_base[:3, :3] = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])
    np.savez(
        path,
        depth_raw_uint16=np.full((height, width), 1000, dtype=np.uint16),
        intrinsics_fx_fy_cx_cy=np.array([20.0, 20.0, 3.5, 3.5]),
        tf_optical_to_base=optical_to_base,
        scan_ranges=1.0 / np.cos(angles),
        scan_angles=angles,
        scan_range_min_m=np.array(0.1),
        scan_range_max_m=np.array(10.0),
        tf_laser_to_base=np.eye(4),
    )


def _selection():
    return {
        'depth_roi_inclusive_uv': [0, 7, 0, 7],
        'depth_range_m': [0.9, 1.1],
        'lidar_base_x_range_m': [0.9, 1.1],
    }


def _main_args(train, validation, selection, output):
    return [
        str(SCRIPT),
        '--train', str(train),
        '--validation', str(validation),
        '--selection', str(selection),
        '--output-dir', str(output),
    ]


def test_transform_points_applies_rotation_then_translation():
    """Transform points with rotation before adding translation."""
    transform = np.eye(4)
    transform[:3, :3] = ALIGNMENT.rotation_yaw_pitch(np.pi / 2, 0.0)
    transform[:3, 3] = [1.0, 2.0, 3.0]

    result = ALIGNMENT.transform_points([[1.0, 0.0, 0.0]], transform)

    np.testing.assert_allclose(result, [[1.0, 3.0, 3.0]], atol=1e-12)


def test_plane_fit_recovers_vertical_face_coefficients():
    """Recover all coefficients of a non-degenerate face patch."""
    y, z = np.meshgrid(np.linspace(-0.3, 0.3, 8),
                       np.linspace(0.1, 0.8, 7))
    points = np.column_stack((
        0.2 * y.ravel() - 0.1 * z.ravel() + 1.1,
        y.ravel(), z.ravel()))

    fit = ALIGNMENT.plane_fit(points)

    np.testing.assert_allclose(fit, [0.2, -0.1, 1.1], atol=1e-12)


def test_scan_line_recovers_laser_face_slice():
    """Recover slope and intercept from a valid horizontal scan slice."""
    y = np.linspace(-0.4, 0.4, 20)
    points = np.column_stack((0.15 * y + 1.2, y, np.zeros_like(y)))

    fit = ALIGNMENT.scan_line(points)

    np.testing.assert_allclose(fit, [0.15, 1.2], atol=1e-12)


def test_rotation_yaw_pitch_uses_rz_ry_order():
    """Compose the constrained correction in documented yaw-pitch order."""
    rotation = ALIGNMENT.rotation_yaw_pitch(np.pi / 2, np.pi / 2)

    np.testing.assert_allclose(rotation @ [1.0, 0.0, 0.0],
                               [0.0, 0.0, -1.0], atol=1e-12)


def test_fit_candidate_recovers_constrained_known_pose_and_reduces_residual():
    """Recover known observable pose terms and reduce face residual."""
    optical, laser, nominal, expected = _synthetic_face()

    candidate, parameters = ALIGNMENT.fit_candidate(
        optical, laser, nominal)
    before = ALIGNMENT.evaluate(optical, laser, nominal)
    after = ALIGNMENT.evaluate(optical, laser, candidate)

    np.testing.assert_allclose(candidate, expected, atol=1e-12)
    assert parameters['yaw_correction_rad'] == pytest.approx(0.08)
    assert parameters['pitch_correction_rad'] == pytest.approx(-0.05)
    assert after['face_to_lidar_plane_abs_residual_p50_p95_m'][1] < 1e-12
    assert after['face_to_lidar_plane_abs_residual_p50_p95_m'][1] < (
        before['face_to_lidar_plane_abs_residual_p50_p95_m'][1])


def test_training_candidate_generalizes_to_temporal_holdout_face_points():
    """Apply a training fit to independently sampled same-pose points."""
    training = _synthetic_face()
    validation = _synthetic_face(
        y_values=np.linspace(-0.27, 0.28, 13),
        z_values=np.linspace(0.14, 0.74, 11),
    )
    candidate, _ = ALIGNMENT.fit_candidate(*training[:3])

    result = ALIGNMENT.evaluate(validation[0], validation[1], candidate)

    assert result['face_to_lidar_plane_abs_residual_p50_p95_m'][1] < 1e-12


def test_single_plane_keeps_height_unobservable_with_rank_three():
    """Keep residual unchanged by height while exposing only rank three."""
    optical, laser, _, candidate = _synthetic_face()
    raised = candidate.copy()
    raised[2, 3] += 1.7

    baseline = ALIGNMENT.evaluate(optical, laser, candidate)
    changed_height = ALIGNMENT.evaluate(optical, laser, raised)

    assert baseline['point_to_plane_pose_jacobian_rank'] == 3
    assert changed_height['point_to_plane_pose_jacobian_rank'] == 3
    np.testing.assert_allclose(
        changed_height['face_to_lidar_plane_abs_residual_p50_p95_m'],
        baseline['face_to_lidar_plane_abs_residual_p50_p95_m'],
        atol=1e-15,
    )
    np.testing.assert_allclose(
        np.subtract(
            changed_height['nominal_or_candidate_height_min_max_m'],
            baseline['nominal_or_candidate_height_min_max_m']),
        [1.7, 1.7],
        atol=1e-12,
    )


@pytest.mark.parametrize('points', [
    np.zeros((29, 3)),
    np.column_stack((np.ones(40), np.linspace(0.0, 0.2, 40),
                     np.zeros(40))),
    np.full((40, 3), np.nan),
])
def test_plane_fit_rejects_invalid_or_degenerate_face(points):
    """Reject face samples that cannot define the required 2-D patch."""
    with pytest.raises(ValueError):
        ALIGNMENT.plane_fit(points)


@pytest.mark.parametrize('points', [
    np.zeros((9, 3)),
    np.column_stack((np.ones(20), np.zeros(20), np.zeros(20))),
    np.full((20, 3), np.inf),
])
def test_scan_line_rejects_invalid_or_degenerate_slice(points):
    """Reject scan samples that cannot define a horizontal line."""
    with pytest.raises(ValueError):
        ALIGNMENT.scan_line(points)


def test_load_sample_selects_valid_depth_patch_and_laser_slice(tmp_path):
    """Decode selected depth and laser regions from a valid fixture."""
    capture = tmp_path / 'capture.npz'
    _write_capture(capture)

    optical, laser, nominal = ALIGNMENT.load_sample(capture, _selection())

    assert optical.shape == (64, 3)
    assert laser.shape == (21, 3)
    np.testing.assert_allclose(nominal[:3, :3], [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])


def test_main_writes_diagnostic_only_report(tmp_path, monkeypatch):
    """Write only non-deploying diagnostic artifacts for valid inputs."""
    train = tmp_path / 'train.npz'
    validation = tmp_path / 'validation.npz'
    selection = tmp_path / 'selection.json'
    output = tmp_path / 'result'
    _write_capture(train)
    _write_capture(validation)
    selection.write_text(json.dumps(_selection()))
    monkeypatch.setattr(sys, 'argv', _main_args(
        train, validation, selection, output))

    ALIGNMENT.main()

    assert sorted(path.name for path in output.iterdir()) == [
        'candidate.json', 'comparison.png']
    report = json.loads((output / 'candidate.json').read_text())
    assert report['status'] == 'offline_candidate_not_deployed'
    assert report['fusion_approved'] is False
    assert report['validation_scope'].startswith('temporal holdout')


def test_main_refuses_to_overwrite_existing_output(tmp_path, monkeypatch):
    """Preserve an existing output directory instead of overwriting it."""
    train = tmp_path / 'train.npz'
    validation = tmp_path / 'validation.npz'
    selection = tmp_path / 'selection.json'
    output = tmp_path / 'existing'
    _write_capture(train)
    _write_capture(validation)
    selection.write_text(json.dumps(_selection()))
    output.mkdir()
    marker = output / 'keep.txt'
    marker.write_text('preserve')
    monkeypatch.setattr(sys, 'argv', _main_args(
        train, validation, selection, output))

    with pytest.raises(FileExistsError):
        ALIGNMENT.main()

    assert marker.read_text() == 'preserve'


def test_ground_reference_keeps_invalid_floor_separate_from_zero_height(tmp_path):
    """Recover synthetic edge pitch without interpreting missing depth as floor."""
    capture = tmp_path / 'ground.npz'
    depth = np.zeros((12, 12), dtype=np.uint16)
    depth[10, :] = 1000
    nominal = np.eye(4)
    nominal[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    nominal[2, 3] = 0.215
    np.savez(capture, depth_raw_uint16=depth,
             intrinsics_fx_fy_cx_cy=[40, 40, 5.5, 5.5],
             tf_optical_to_base=nominal)
    reference = {'edge_roi_inclusive_uv': [0, 11, 10, 10],
                 'edge_depth_range_m': [.5, 1.5],
                 'floor_roi_inclusive_uv': [0, 11, 11, 11],
                 'camera_height_m': .215}
    result = ALIGNMENT.ground_diagnostic(
        capture, reference, {'nominal': nominal})
    elevation_rad = np.radians(result['edge_implied_elevation_deg_p05_p50_p95'][1])
    corrected = nominal.copy()
    corrected[:3, :3] = ALIGNMENT.rotation_yaw_pitch(
        0, -elevation_rad) @ nominal[:3, :3]
    checked = ALIGNMENT.ground_diagnostic(
        capture, reference, {'corrected': corrected})
    assert result['edge_sample_count'] == 12
    assert result['visible_floor_roi_nonzero_depth_fraction'] == 0
    np.testing.assert_allclose(
        checked['edge_height_p05_p50_p95_m']['corrected'], 0, atol=1e-12)


def test_ground_reference_rejects_missing_edge(tmp_path):
    """Do not estimate ground alignment when selected depth is missing."""
    capture = tmp_path / 'missing.npz'
    np.savez(capture, depth_raw_uint16=np.zeros((12, 12), dtype=np.uint16),
             intrinsics_fx_fy_cx_cy=[40, 40, 5.5, 5.5],
             tf_optical_to_base=np.eye(4))
    reference = {'edge_roi_inclusive_uv': [0, 11, 10, 10],
                 'edge_depth_range_m': [.5, 1.5],
                 'floor_roi_inclusive_uv': [0, 11, 11, 11],
                 'camera_height_m': .215}
    with pytest.raises(ValueError, match='insufficient_bottom_edge_samples'):
        ALIGNMENT.ground_diagnostic(capture, reference, {'nominal': np.eye(4)})
