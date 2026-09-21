#!/usr/bin/env python3
"""
Fit a constrained, offline camera-pose candidate from a vertical box face.

One plane cannot calibrate a full rigid transform. No ROS or configuration
writer is provided: all results are conditional diagnostic artifacts.
"""

import argparse
import json
from pathlib import Path

import numpy as np


def transform_points(points, transform):
    """Apply a homogeneous transform to finite Cartesian points."""
    points = np.asarray(points, dtype=float)
    transform = np.asarray(transform, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3
            or transform.shape != (4, 4)
            or not np.isfinite(points).all()
            or not np.isfinite(transform).all()):
        raise ValueError('invalid_points_or_transform')
    return points @ transform[:3, :3].T + transform[:3, 3]


def plane_fit(points):
    """Fit x = a*y + b*z + c, requiring a two-dimensional face patch."""
    points = np.asarray(points, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 30
            or not np.isfinite(points).all()):
        raise ValueError('insufficient_finite_face_points')
    design = np.column_stack((points[:, 1:], np.ones(len(points))))
    fit, _, rank, _ = np.linalg.lstsq(design, points[:, 0], rcond=None)
    if rank != 3 or np.min(np.ptp(points[:, 1:], axis=0)) < 0.03:
        raise ValueError('degenerate_face_patch')
    return fit


def scan_line(points):
    """Fit the observed horizontal laser slice x = a*y + b."""
    points = np.asarray(points, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 10
            or not np.isfinite(points).all()
            or np.ptp(points[:, 1]) < 0.03):
        raise ValueError('degenerate_laser_slice')
    return np.polyfit(points[:, 1], points[:, 0], 1)


def rotation_yaw_pitch(yaw_rad, pitch_rad):
    """Return Rz(yaw) @ Ry(pitch), with roll fixed at zero."""
    cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
    cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array(
        [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])


def fit_candidate(optical_points, laser_points, nominal_tf):
    """
    Fit pitch/yaw/x only under a vertical-target assumption.

    Translation y/z and roll stay nominal. These constraints select one
    solution; they are not measurements of the unobservable parameters.
    """
    nominal = transform_points(optical_points, nominal_tf)
    face = plane_fit(nominal)
    line = scan_line(laser_points)
    pitch_rad = -np.arctan(face[1])
    yaw_rad = -np.arctan(line[0]) + np.arctan(
        face[0] / np.sqrt(1 + face[1]**2))
    rotation = rotation_yaw_pitch(yaw_rad, pitch_rad)
    local = nominal - nominal_tf[:3, 3]
    rotated = local @ rotation.T
    candidate = np.array(nominal_tf, copy=True)
    candidate[:3, :3] = rotation @ nominal_tf[:3, :3]
    candidate[0, 3] = (line[1] + line[0]*candidate[1, 3]
                       - np.median(rotated[:, 0]-line[0]*rotated[:, 1]))
    return candidate, {
        'pitch_correction_rad': float(pitch_rad),
        'yaw_correction_rad': float(yaw_rad),
        'translation_candidate_m': candidate[:3, 3].tolist(),
        'translation_change_m': (
            candidate[:3, 3]-nominal_tf[:3, 3]).tolist(),
        'assumptions': ['target face is vertical',
                        'nominal roll is correct',
                        'nominal lateral position and height are retained'],
    }


def evaluate(optical_points, laser_points, transform):
    """Measure a face-plane residual, not a full alignment accuracy."""
    points = transform_points(optical_points, transform)
    line = scan_line(laser_points)
    residual_m = np.abs(points[:, 0]-line[0]*points[:, 1]-line[1])
    residual_m /= np.sqrt(1+line[0]**2)
    normal = np.array([1., -line[0], 0.])
    normal /= np.linalg.norm(normal)
    jacobian = np.column_stack((np.tile(normal, (len(points), 1)),
                                np.cross(points, normal)))
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    return {
        'face_to_lidar_plane_abs_residual_p50_p95_m': np.percentile(
            residual_m, [50, 95]).tolist(),
        'point_to_plane_pose_jacobian_rank': int(np.linalg.matrix_rank(jacobian)),
        'point_to_plane_pose_jacobian_singular_values': singular_values.tolist(),
        'nominal_or_candidate_height_min_max_m': [
            float(points[:, 2].min()), float(points[:, 2].max())],
    }


def load_sample(path, selection):
    """Select a manually identified box patch using external provenance."""
    with np.load(path, allow_pickle=False) as data:
        depth_m = data['depth_raw_uint16'] * .001
        v, u = np.indices(depth_m.shape)
        fx, fy, cx, cy = data['intrinsics_fx_fy_cx_cy']
        optical = np.stack(((u-cx)*depth_m/fx, (v-cy)*depth_m/fy, depth_m), -1)
        u0, u1, v0, v1 = selection['depth_roi_inclusive_uv']
        low_m, high_m = selection['depth_range_m']
        mask = ((u >= u0) & (u <= u1) & (v >= v0) & (v <= v1)
                & (depth_m > low_m) & (depth_m < high_m))
        optical_points = optical[mask]
        nominal_tf = data['tf_optical_to_base']
        nominal_points = transform_points(optical_points, nominal_tf)
        plane_fit(nominal_points)
        ranges, angles = data['scan_ranges'], data['scan_angles']
        valid = (np.isfinite(ranges) & (ranges >= data['scan_range_min_m'])
                 & (ranges <= data['scan_range_max_m']))
        r, t = ranges[valid], angles[valid]
        laser = transform_points(np.column_stack((r*np.cos(t), r*np.sin(t),
                                                  np.zeros(len(r)))),
                                 data['tf_laser_to_base'])
        low_m, high_m = selection['lidar_base_x_range_m']
        mask = ((laser[:, 0] > low_m) & (laser[:, 0] < high_m)
                & (laser[:, 1] >= nominal_points[:, 1].min())
                & (laser[:, 1] <= nominal_points[:, 1].max()))
        laser = laser[mask]
        scan_line(laser)
        return optical_points, laser, nominal_tf.copy()


def render_comparison(path, laser_points, hypotheses, output_path):
    """Render held-out projections without changing the recorded images."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    with np.load(path, allow_pickle=False) as data:
        depth_m = data['depth_raw_uint16'] * .001
        fx, fy, cx, cy = data['intrinsics_fx_fy_cx_cy']
    fig, axes = plt.subplots(1, len(hypotheses), figsize=(15, 5))
    projection_rows = {}
    for axis, (name, transform) in zip(axes, hypotheses.items()):
        optical = transform_points(laser_points, np.linalg.inv(transform))
        in_front = optical[:, 2] > 0
        optical = optical[in_front]
        pixels_u = fx*optical[:, 0]/optical[:, 2]+cx
        pixels_v = fy*optical[:, 1]/optical[:, 2]+cy
        axis.imshow(np.where(depth_m > 0, depth_m, np.nan),
                    cmap='turbo_r', vmin=.4, vmax=2.5)
        axis.scatter(pixels_u, pixels_v, s=14, c='lime', edgecolors='black',
                     linewidths=.3, label='LiDAR projected to depth')
        axis.set_title(name.replace('_', ' '))
        axis.set(xlim=(100, 265), ylim=(230, 110), xlabel='u (pixels)',
                 ylabel='v (pixels)')
        axis.legend(fontsize=8, loc='lower left')
        projection_rows[name] = float(np.median(pixels_v))
    fig.suptitle('Same-pose temporal holdout: conditional vertical-box fit\n'
                 'NOT full calibration / height unobservable / NOT deployed')
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)
    return projection_rows


def ground_diagnostic(path, ground_reference, hypotheses):
    """Check the independent height prior using a manual bottom-edge proxy."""
    with np.load(path, allow_pickle=False) as data:
        depth_m = data['depth_raw_uint16'] * .001
        v, u = np.indices(depth_m.shape)
        fx, fy, cx, cy = data['intrinsics_fx_fy_cx_cy']
        nominal_tf = data['tf_optical_to_base']
    u0, u1, v0, v1 = ground_reference['edge_roi_inclusive_uv']
    low_m, high_m = ground_reference['edge_depth_range_m']
    mask = ((u >= u0) & (u <= u1) & (v >= v0) & (v <= v1)
            & (depth_m > low_m) & (depth_m < high_m))
    optical = np.column_stack(((u[mask]-cx)*depth_m[mask]/fx,
                               (v[mask]-cy)*depth_m[mask]/fy, depth_m[mask]))
    if len(optical) < 10:
        raise ValueError('insufficient_bottom_edge_samples')
    local = optical @ nominal_tf[:3, :3].T
    height_m = ground_reference['camera_height_m']
    radius_m = np.hypot(local[:, 0], local[:, 2])
    if not 0 < height_m < radius_m.min():
        raise ValueError('incompatible_height_prior')
    elevation_rad = np.arcsin(-height_m/radius_m) - np.arctan2(
        local[:, 2], local[:, 0])
    u0, u1, v0, v1 = ground_reference['floor_roi_inclusive_uv']
    floor_roi = depth_m[v0:v1+1, u0:u1+1]
    return {
        'provenance': ground_reference,
        'edge_sample_count': len(optical),
        'edge_implied_elevation_deg_p05_p50_p95': np.percentile(
            np.degrees(elevation_rad), [5, 50, 95]).tolist(),
        'edge_height_p05_p50_p95_m': {
            name: np.percentile(transform_points(optical, transform)[:, 2],
                                [5, 50, 95]).tolist()
            for name, transform in hypotheses.items()},
        'visible_floor_roi_nonzero_depth_fraction': float(np.mean(floor_roi > 0)),
        'caveat': 'bottom-edge proxy is not a measured floor plane; no mount update',
    }


def main():
    """Write diagnostic-only candidates; refuse to overwrite an output run."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train', type=Path, required=True)
    parser.add_argument('--validation', type=Path, required=True)
    parser.add_argument('--selection', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    selection = json.loads(args.selection.read_text())
    training = load_sample(args.train, selection)
    validation = load_sample(args.validation, selection)
    if not np.allclose(training[2], validation[2], atol=1e-9, rtol=0):
        raise ValueError('nominal_mount_changed_between_samples')
    candidate, parameters = fit_candidate(*training)
    nominal = training[2]
    rotation_only = candidate.copy()
    rotation_only[:3, 3] = nominal[:3, 3]
    hypotheses = {'nominal': nominal, 'rotation_only': rotation_only,
                  'constrained_candidate': candidate}
    report = {
        'status': 'offline_candidate_not_deployed',
        'fusion_approved': False,
        'training_capture': str(args.train.resolve()),
        'validation_capture': str(args.validation.resolve()),
        'validation_scope': ('temporal holdout at same pose, '
                             'not independent calibration validation'),
        'selection_provenance': selection,
        'parameters': parameters,
        'candidate_optical_to_base': candidate.tolist(),
        'results': {name: {
            'training': evaluate(*training[:2], transform),
            'validation': evaluate(*validation[:2], transform),
        } for name, transform in hypotheses.items()},
        'unresolved': ['single plane leaves three pose degrees unobservable',
                       'target tilt versus camera pitch is not independently measured',
                       'height requires a ground reference; invalid depth is not a floor'],
    }
    if 'ground_reference' in selection:
        report['ground_reference_check'] = ground_diagnostic(
            args.validation, selection['ground_reference'], hypotheses)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    report['holdout_projected_lidar_row_median'] = render_comparison(
        args.validation, validation[1], hypotheses,
        args.output_dir / 'comparison.png')
    (args.output_dir / 'candidate.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
