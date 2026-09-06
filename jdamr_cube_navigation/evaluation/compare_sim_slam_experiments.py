#!/usr/bin/env python3
"""Compare controlled Gazebo SLAM runs and render evidence artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable


PROFILES = ('baseline_10hz', 'lidar_noise_5x')
BACKENDS = ('cartographer', 'slam_toolbox')
ERROR_FIELDS = (
    ('ate', 'translation_rms_m'),
    ('ate', 'yaw_rms_rad'),
    ('rpe', 'translation_rms_m'),
    ('rpe', 'yaw_rms_rad'),
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def load_runs(results_root: Path) -> list[dict[str, Any]]:
    """Load the required run manifests, metrics, and sensor provenance."""
    runs = []
    for profile in PROFILES:
        for backend in BACKENDS:
            run_dir = results_root / f'{profile}__{backend}'
            manifest_path = run_dir / 'execution_manifest.json'
            if not manifest_path.is_file():
                manifest_path = run_dir / 'run_manifest.json'
            metrics_path = run_dir / 'metrics.json'
            if not manifest_path.is_file() or not metrics_path.is_file():
                raise ValueError(f'incomplete run directory: {run_dir}')
            manifest = _load_json(manifest_path)
            metrics = _load_json(metrics_path)
            profile_manifest_path = Path(
                manifest['profile']['urdf']).with_suffix('.manifest.json')
            if not profile_manifest_path.is_file():
                raise ValueError(
                    f'sensor profile manifest is missing: '
                    f'{profile_manifest_path}')
            profile_manifest = _load_json(profile_manifest_path)
            if (profile_manifest['output']['sha256']
                    != manifest['profile']['sha256']):
                raise ValueError(
                    f'sensor profile hash mismatch: {profile_manifest_path}')
            runs.append({
                'run_dir': str(run_dir.resolve()),
                'manifest_path': str(manifest_path.resolve()),
                'manifest_sha256': _sha256(manifest_path),
                'metrics_path': str(metrics_path.resolve()),
                'metrics_sha256': _sha256(metrics_path),
                'profile_manifest_path': str(profile_manifest_path.resolve()),
                'profile_manifest_sha256': _sha256(profile_manifest_path),
                'manifest': manifest,
                'metrics': metrics,
                'profile_manifest': profile_manifest,
            })
    return runs


def _common_value(runs: Iterable[dict[str, Any]], name: str,
                  getter) -> Any:
    values = [getter(run['manifest']) for run in runs]
    first = values[0]
    if any(value != first for value in values[1:]):
        raise ValueError(f'controlled field differs across runs: {name}')
    return first


def _percent_change(value: float, baseline: float) -> float:
    return (value / baseline - 1.0) * 100.0


def _lower_percent(value: float, reference: float) -> float:
    return (1.0 - value / reference) * 100.0


def summarize(runs: list[dict[str, Any]]) -> dict[str, Any]:
    """Validate the experiment matrix and calculate cross-run comparisons."""
    expected = {(profile, backend)
                for profile in PROFILES for backend in BACKENDS}
    indexed = {}
    for run in runs:
        manifest = run['manifest']
        metrics = run['metrics']
        key = (manifest['profile']['label'], manifest['backend'])
        if key in indexed:
            raise ValueError(f'duplicate run: {key}')
        if manifest['status'] != 'complete':
            raise ValueError(f'run is not complete: {key}')
        if any(code != 0 for code in manifest['exit_codes'].values()):
            raise ValueError(f'run has a non-zero exit code: {key}')
        if metrics['backend'] != manifest['backend']:
            raise ValueError(f'backend mismatch: {key}')
        indexed[key] = run
    if set(indexed) != expected:
        raise ValueError(
            f'experiment matrix mismatch: expected {sorted(expected)}, '
            f'found {sorted(indexed)}')

    seed = _common_value(runs, 'seed', lambda item: item['seed'])
    route = _common_value(runs, 'route', lambda item: item['route'])
    world_sha256 = _common_value(
        runs, 'world.sha256', lambda item: item['world']['sha256'])
    spawn_xy_m = _common_value(
        runs, 'spawn_xy_m',
        lambda item: item.get('spawn_xy_m', item.get('spawn_xy')))
    corridor_distance_m = _common_value(
        runs, 'corridor_distance_m',
        lambda item: item['corridor_distance_m'])

    results = []
    for profile in PROFILES:
        for backend in BACKENDS:
            run = indexed[(profile, backend)]
            metrics = run['metrics']
            truth_path_m = metrics['ground_truth']['path_length_m']
            slam_path_m = metrics['slam']['path_length_m']
            results.append({
                'profile': profile,
                'backend': backend,
                'profile_sha256': run['manifest']['profile']['sha256'],
                'profile_manifest_sha256': run[
                    'profile_manifest_sha256'],
                'run_manifest_sha256': run['manifest_sha256'],
                'metrics_sha256': run['metrics_sha256'],
                'ground_truth_path_m': truth_path_m,
                'slam_path_m': slam_path_m,
                'slam_path_error_percent': _percent_change(
                    slam_path_m, truth_path_m),
                'normalized_ate_percent': (
                    metrics['ate']['translation_rms_m']
                    / truth_path_m * 100.0),
                'ate': metrics['ate'],
                'rpe': metrics['rpe'],
                'wheel_odometry_reference': metrics[
                    'wheel_odometry_reference'],
            })

    backend_comparison = {}
    for profile in PROFILES:
        cartographer = indexed[(profile, 'cartographer')]['metrics']
        toolbox = indexed[(profile, 'slam_toolbox')]['metrics']
        backend_comparison[profile] = {
            'cartographer_ate_translation_lower_percent': _lower_percent(
                cartographer['ate']['translation_rms_m'],
                toolbox['ate']['translation_rms_m']),
            'slam_toolbox_to_cartographer_ate_translation_ratio': (
                toolbox['ate']['translation_rms_m']
                / cartographer['ate']['translation_rms_m']),
            'cartographer_rpe_translation_lower_percent': _lower_percent(
                cartographer['rpe']['translation_rms_m'],
                toolbox['rpe']['translation_rms_m']),
            'slam_toolbox_to_cartographer_rpe_translation_ratio': (
                toolbox['rpe']['translation_rms_m']
                / cartographer['rpe']['translation_rms_m']),
        }

    noise_sensitivity = {}
    for backend in BACKENDS:
        baseline = indexed[('baseline_10hz', backend)]['metrics']
        stressed = indexed[('lidar_noise_5x', backend)]['metrics']
        noise_sensitivity[backend] = {
            f'{group}_{field}_change_percent': _percent_change(
                stressed[group][field], baseline[group][field])
            for group, field in ERROR_FIELDS
        }

    cartographer_dominates = all(
        indexed[(profile, 'cartographer')]['metrics'][group][field]
        < indexed[(profile, 'slam_toolbox')]['metrics'][group][field]
        for profile in PROFILES
        for group, field in ERROR_FIELDS
    )
    truth_paths_m = [result['ground_truth_path_m'] for result in results]
    return {
        'schema_version': 1,
        'experiment': {
            'seed': seed,
            'route': route,
            'spawn_xy_m': spawn_xy_m,
            'corridor_distance_m': corridor_distance_m,
            'world_sha256': world_sha256,
            'ground_truth_path_range_m': [
                min(truth_paths_m), max(truth_paths_m)],
            'alignment': 'SE(2) rigid transform without scale',
            'rpe_delta_s': 1.0,
        },
        'sensor_profiles': {
            profile: indexed[(profile, BACKENDS[0])][
                'profile_manifest']['changes']
            for profile in PROFILES
        },
        'results': results,
        'backend_comparison': backend_comparison,
        'noise_sensitivity': noise_sensitivity,
        'selection': {
            'backend': (
                'cartographer' if cartographer_dominates else None),
            'criterion': (
                'lower translation/yaw ATE and RPE in every tested profile'),
            'passed': cartographer_dominates,
        },
        'limitations': [
            'one random seed and one out-and-back route',
            'synthetic Gaussian LiDAR stress, not measured sensor noise',
            'straight corridor proxy, not a metric reconstruction of the site',
            'simulated wheel odometry is cleaner than the physical platform',
        ],
    }


def render_markdown(summary: dict[str, Any]) -> str:
    """Render the comparison as a concise Korean engineering report."""
    sensor_profiles = summary['sensor_profiles']
    profile_labels = {
        profile: (
            f"{sensor_profiles[profile]['lidar_update_rate_hz']['to']:g} Hz / "
            f"σ {sensor_profiles[profile]['lidar_noise_stddev_m']['to']:g} m")
        for profile in PROFILES
    }
    lines = [
        '# 시뮬레이션 SLAM 정답 궤적 비교',
        '',
        '> 판정: 두 센서 조건의 이동·회전 ATE/RPE가 모두 낮은 '
        '**Cartographer를 복도용 기본 백엔드로 유지한다.**',
        '',
        '## 측정 결과',
        '',
        '| 센서 조건 | 백엔드 | GT 경로 (m) | SLAM 경로 (m) | '
        'ATE RMS (m) | 정규화 ATE | 1초 RPE RMS (m) | yaw ATE RMS (rad) |',
        '|---|---|---:|---:|---:|---:|---:|---:|',
    ]
    labels = {
        **profile_labels,
        'cartographer': 'Cartographer',
        'slam_toolbox': 'SLAM Toolbox',
    }
    for result in summary['results']:
        lines.append(
            f"| {labels[result['profile']]} | {labels[result['backend']]} | "
            f"{result['ground_truth_path_m']:.3f} | "
            f"{result['slam_path_m']:.3f} | "
            f"{result['ate']['translation_rms_m']:.3f} | "
            f"{result['normalized_ate_percent']:.2f}% | "
            f"{result['rpe']['translation_rms_m']:.4f} | "
            f"{result['ate']['yaw_rms_rad']:.4f} |")
    baseline = summary['backend_comparison']['baseline_10hz']
    stressed = summary['backend_comparison']['lidar_noise_5x']
    cartographer_noise = summary['noise_sensitivity']['cartographer']
    toolbox_noise = summary['noise_sensitivity']['slam_toolbox']
    lines.extend([
        '',
        '기준 조건에서 SLAM Toolbox의 이동 ATE는 Cartographer의 '
        f"{baseline['slam_toolbox_to_cartographer_ate_translation_ratio']:.2f}"
        '배, '
        f"{summary['experiment']['rpe_delta_s']:g}초 이동 RPE는 "
        f"{baseline['slam_toolbox_to_cartographer_rpe_translation_ratio']:.2f}"
        '배였다. '
        '노이즈 조건에서는 각각 '
        f"{stressed['slam_toolbox_to_cartographer_ate_translation_ratio']:.2f}"
        '배와 '
        f"{stressed['slam_toolbox_to_cartographer_rpe_translation_ratio']:.2f}"
        '배였다.',
        '',
        '## 센서 노이즈 민감도',
        '',
        'LiDAR 가우시안 표준편차를 '
        f"{sensor_profiles['baseline_10hz']['lidar_noise_stddev_m']['to']:g}"
        'm에서 '
        f"{sensor_profiles['lidar_noise_5x']['lidar_noise_stddev_m']['to']:g}"
        'm로 '
        '높였을 때 '
        'Cartographer의 이동 ATE는 '
        f"{cartographer_noise['ate_translation_rms_m_change_percent']:+.1f}%, "
        '1초 이동 RPE는 '
        f"{cartographer_noise['rpe_translation_rms_m_change_percent']:+.1f}% "
        '변했다. 센서 열화가 전역 오차보다 단기 상대 오차에 더 크게 나타났다.',
        '',
        'SLAM Toolbox의 이동 ATE와 RPE는 각각 '
        f"{toolbox_noise['ate_translation_rms_m_change_percent']:+.1f}%와 "
        f"{toolbox_noise['rpe_translation_rms_m_change_percent']:+.1f}%였지만, "
        'yaw ATE와 RPE는 각각 '
        f"{toolbox_noise['ate_yaw_rms_rad_change_percent']:+.1f}%와 "
        f"{toolbox_noise['rpe_yaw_rms_rad_change_percent']:+.1f}% 악화됐다. "
        '단일 시드에서 이동 오차가 줄어든 값을 노이즈 개선 효과로 해석하지 않는다.',
        '',
        '## 실험 계약과 해석 범위',
        '',
        f"- Gazebo seed {summary['experiment']['seed']}, 동일 world hash와 "
        '동일 왕복 명령을 사용했다.',
        '- `/ground_truth_pose`는 Gazebo 모델 pose이며 wheel odom과 SLAM TF에서 '
        '독립적으로 기록했다.',
        '- ATE는 축척을 바꾸지 않는 SE(2) 강체 정렬 후 계산했고, RPE는 '
        f"{summary['experiment']['rpe_delta_s']:g}초 간격의 상대 운동 오차다.",
        '- 실제 평면도에서는 긴 복도·반복 벽·양 끝 회차라는 정성적 구조만 '
        '가져왔다. 축척이나 실제 방 위치를 복원한 지도가 아니다.',
        '- 이 결과는 한 시드·한 경로의 백엔드 선택 근거다. 반복 성공률이나 '
        '실차 절대 정확도로 확대하지 않는다.',
        '',
        '## 논문과 구현 연결',
        '',
        '- Kümmerle et al., [On Measuring the Accuracy of SLAM Algorithms]'
        '(https://doi.org/10.1007/s10514-009-9155-6): 전역 원점보다 '
        '상대 pose 관계를 이용한 SLAM 비교의 필요성을 제시한다.',
        '- Sturm et al., [A Benchmark for the Evaluation of RGB-D SLAM '
        'Systems]'
        '(https://cvg.cit.tum.de/_media/spezial/bib/sturm12iros.pdf): '
        'timestamp 연계, 정답 궤적 정렬, ATE/RPE 계산 절차를 참고해 '
        '2D SE(2)로 적용했다.',
        '- Hess et al., [Real-Time Loop Closure in 2D LIDAR SLAM]'
        '(https://research.google/pubs/'
        'real-time-loop-closure-in-2d-lidar-slam/): '
        'Cartographer의 scan-to-submap 제약과 실시간 loop closure 설계 근거다.',
        '- Macenski and Jambrecic, [SLAM Toolbox: SLAM for the dynamic world]'
        '(https://joss.theoj.org/papers/10.21105/joss.02783): '
        '비교 대상 백엔드의 ROS 2 pose-graph 및 lifelong mapping 설계를 설명한다.',
        '',
        '원시 MCAP은 저장소 밖 `$HOME/jdamr_artifacts/`에 보존하고, 이 디렉터리의 '
        'JSON과 manifest가 입력 해시를 연결한다.',
        '',
    ])
    return '\n'.join(lines)


def render_plot(summary: dict[str, Any], output: Path) -> None:
    """Render translation ATE and one-second RPE without loading raw bags."""
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    indexed = {
        (result['profile'], result['backend']): result
        for result in summary['results']
    }
    sensor_profiles = summary['sensor_profiles']
    profile_labels = [
        (
            f"{'Baseline' if profile == PROFILES[0] else 'Noise stress'}\n"
            f"{sensor_profiles[profile]['lidar_update_rate_hz']['to']:g} Hz / "
            f"σ {sensor_profiles[profile]['lidar_noise_stddev_m']['to']:g} m")
        for profile in PROFILES
    ]
    backend_labels = ['Cartographer', 'SLAM Toolbox']
    colors = ['#087F8C', '#D95D39']
    x = np.arange(len(PROFILES), dtype=float)
    width = 0.34
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.8))
    plots = (
        ('ATE translation RMS [m]', 'ate', 'translation_rms_m'),
        (f'RPE translation RMS [m] '
         f"(Δt = {summary['experiment']['rpe_delta_s']:g} s)",
         'rpe', 'translation_rms_m'),
    )
    for axis, (title, group, field) in zip(axes, plots):
        for backend_index, backend in enumerate(BACKENDS):
            values = [indexed[(profile, backend)][group][field]
                      for profile in PROFILES]
            offset = (backend_index - 0.5) * width
            bars = axis.bar(
                x + offset, values, width, label=backend_labels[backend_index],
                color=colors[backend_index])
            axis.bar_label(bars, fmt='%.3f', padding=3, fontsize=8)
        axis.set_title(title)
        axis.set_xticks(x, profile_labels)
        axis.grid(axis='y', alpha=0.25)
        axis.set_axisbelow(True)
    axes[0].set_ylabel('Lower is better')
    axes[1].legend(frameon=False, loc='upper left')
    fig.suptitle(
        'Controlled corridor SLAM — Gazebo ground truth, '
        f"seed {summary['experiment']['seed']}",
        fontweight='bold')
    fig.text(
        0.5, 0.01,
        'One seed / one route; synthetic Gaussian LiDAR stress. '
        'No scale alignment.',
        ha='center', fontsize=9, color='#444444')
    fig.tight_layout(rect=(0, 0.05, 1, 0.94))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches='tight')
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    """Compare validated simulation runs and write report artifacts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    if not args.results_root.is_dir():
        parser.error('--results-root must be an existing directory')

    runs = load_runs(args.results_root)
    summary = summarize(runs)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / 'sim_slam_robustness.json'
    report_path = args.output_dir / 'sim_slam_robustness.md'
    plot_path = args.output_dir / 'sim_slam_robustness.png'
    manifest_path = args.output_dir / 'media_manifest.json'
    json_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    report_path.write_text(render_markdown(summary), encoding='utf-8')
    render_plot(summary, plot_path)
    media_manifest = {
        'schema_version': 1,
        'sources': [
            {
                'run': run['manifest']['run_label'],
                'run_manifest': {
                    'path': run['manifest_path'],
                    'sha256': run['manifest_sha256'],
                },
                'metrics': {
                    'path': run['metrics_path'],
                    'sha256': run['metrics_sha256'],
                },
                'sensor_profile_manifest': {
                    'path': run['profile_manifest_path'],
                    'sha256': run['profile_manifest_sha256'],
                },
            }
            for run in runs
        ],
        'artifacts': {
            path.name: {'sha256': _sha256(path)}
            for path in (json_path, report_path, plot_path)
        },
    }
    manifest_path.write_text(
        json.dumps(media_manifest, ensure_ascii=False, indent=2),
        encoding='utf-8')
    print(json.dumps({
        'selection': summary['selection'],
        'output_dir': str(args.output_dir.resolve()),
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
