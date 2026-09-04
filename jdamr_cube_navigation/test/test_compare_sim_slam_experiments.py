"""Tests for controlled simulated SLAM cross-run comparison."""

from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from compare_sim_slam_experiments import (  # noqa: E402,I100
    BACKENDS,
    render_markdown,
    summarize,
)


def _run(profile, backend, ate_m, rpe_m, yaw_scale=1.0):
    return {
        'manifest_sha256': f'manifest-{profile}-{backend}',
        'metrics_sha256': f'metrics-{profile}-{backend}',
        'profile_manifest_sha256': f'profile-{profile}',
        'manifest': {
            'status': 'complete',
            'exit_codes': {
                'route': 0,
                'recorder': 0,
                'backend': 0,
                'gazebo': 0,
            },
            'backend': backend,
            'seed': 42,
            'route': 'corridor',
            'spawn_xy_m': [-8.0, 0.0],
            'corridor_distance_m': 14.0,
            'world': {'sha256': 'world'},
            'profile': {'label': profile, 'sha256': f'urdf-{profile}'},
        },
        'metrics': {
            'backend': backend,
            'ground_truth': {'path_length_m': 28.0},
            'slam': {'path_length_m': 27.0},
            'ate': {
                'translation_rms_m': ate_m,
                'yaw_rms_rad': ate_m * yaw_scale,
            },
            'rpe': {
                'translation_rms_m': rpe_m,
                'yaw_rms_rad': rpe_m * yaw_scale,
            },
            'wheel_odometry_reference': {'path_length_m': 27.9},
        },
        'profile_manifest': {
            'changes': {
                'lidar_update_rate_hz': {
                    'from': 2.0,
                    'to': 10.0,
                    'unit': 'Hz',
                },
                'lidar_noise_stddev_m': {
                    'from': 0.01,
                    'to': 0.01 if profile == 'baseline_10hz' else 0.05,
                    'unit': 'm',
                },
            },
        },
    }


def _matrix():
    return [
        _run('baseline_10hz', 'cartographer', 0.5, 0.05),
        _run('baseline_10hz', 'slam_toolbox', 4.0, 0.3),
        _run('lidar_noise_5x', 'cartographer', 0.75, 0.08),
        _run('lidar_noise_5x', 'slam_toolbox', 3.8, 0.28),
    ]


def test_summary_selects_only_backend_that_dominates_every_error_metric():
    summary = summarize(_matrix())

    assert summary['selection'] == {
        'backend': 'cartographer',
        'criterion': (
            'lower translation/yaw ATE and RPE in every tested profile'),
        'passed': True,
    }
    assert summary['backend_comparison']['baseline_10hz'][
        'slam_toolbox_to_cartographer_ate_translation_ratio'
    ] == pytest.approx(8.0)
    assert summary['noise_sensitivity']['cartographer'][
        'ate_translation_rms_m_change_percent'] == pytest.approx(50.0)


def test_summary_rejects_non_controlled_seed():
    runs = _matrix()
    runs[-1]['manifest']['seed'] = 7

    with pytest.raises(ValueError, match='seed'):
        summarize(runs)


def test_summary_rejects_incomplete_experiment_matrix():
    runs = _matrix()[:-1]

    with pytest.raises(ValueError, match='matrix'):
        summarize(runs)


def test_report_preserves_single_seed_noise_caveat():
    report = render_markdown(summarize(_matrix()))

    assert '노이즈 개선 효과로 해석하지 않는다' in report
    assert '실차 절대 정확도로 확대하지 않는다' in report
    assert BACKENDS == ('cartographer', 'slam_toolbox')
