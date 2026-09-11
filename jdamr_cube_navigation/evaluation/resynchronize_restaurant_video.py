#!/usr/bin/env python3
"""Rebuild an already sealed restaurant replay video on capture time."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from run_restaurant_replay_sim import (
    PLAYBACK_SPEED,
    _sha256,
    camera_sim_timing,
    encode_camera_video,
    resample_camera_on_sim_clock,
    scene_video_annotations,
    select_video_encoder,
)


def is_recoverable_video_failure(summary: dict) -> bool:
    """Accept only a sealed core PASS whose derived encode was interrupted."""
    return (
        summary.get('status') == 'FAIL'
        and summary.get('gate_failures') == ['runner_failure']
        and str(summary.get('failure', '')).startswith(
            'camera_encode_CalledProcessError:')
        and summary.get('scenario', {}).get('status') == 'PASS'
        and summary.get('traction_guard', {}).get('final_state') == 'RECOVERED'
        and summary.get('bag') is not None
        and not summary.get('teardown', {}).get('remaining_process_groups')
        and not summary.get('teardown', {}).get('identity_survivors'))


def verify_evidence_record(record: dict, label: str) -> Path:
    """Verify sealed bytes and digest before deriving media."""
    path = Path(record['path'])
    if not path.is_file():
        raise ValueError(f'{label} is missing: {path}')
    if path.stat().st_size != int(record['bytes']):
        raise ValueError(f'{label} byte count changed: {path}')
    if _sha256(path) != record['sha256']:
        raise ValueError(f'{label} digest changed: {path}')
    return path


def main() -> int:
    """Replace only the derived video record while preserving raw evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument(
        '--encoder', choices=('h264_nvenc', 'libx264'), default=None)
    parser.add_argument(
        '--output-name', default='gazebo_actual_map_4x_synced.mp4')
    args = parser.parse_args()
    summary_path = args.run_dir / 'summary.json'
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    recoverable_encode_failure = is_recoverable_video_failure(summary)
    if summary.get('status') != 'PASS' and not recoverable_encode_failure:
        parser.error('run is not PASS or a derived-video-only failure')
    video = summary.get('simulator_video')
    views = video.get('views', []) if video else []
    if len(views) != 2:
        parser.error('exactly two captured views are required')
    try:
        raw_paths = [
            verify_evidence_record(view['raw'], f'camera view {view["name"]}')
            for view in views]
        verify_evidence_record(summary['bag'], 'MCAP')
    except (KeyError, TypeError, ValueError) as error:
        parser.error(str(error))
    fps = float(views[0]['capture']['requested_fps'])
    scales, offsets, _duration_s, common_start_s = camera_sim_timing(
        views, fps)
    common_end_s = min(
        float(view['capture']['frame_timestamps'][-1]['sim_ns']) / 1e9
        for view in views)
    annotations = scene_video_annotations(
        summary.get('scenario'), summary.get('traction_guard'),
        common_start_s)
    output = args.run_dir / args.output_name
    if output.name != args.output_name or output.suffix.lower() != '.mp4':
        parser.error('--output-name must be a plain MP4 filename')
    if output.exists():
        parser.error(f'output already exists: {output}')
    encoder = args.encoder or select_video_encoder()
    aligned_paths = []
    for view, raw_path in zip(views, raw_paths):
        aligned = args.run_dir / f'gazebo_{view["name"]}_clock_aligned.mp4'
        if aligned.exists():
            parser.error(f'aligned view already exists: {aligned}')
        resample_camera_on_sim_clock(
            raw_path, view['capture'], aligned, fps,
            common_start_s, common_end_s)
        aligned_paths.append(aligned)
    encode_camera_video(
        aligned_paths, output, fps, encoder, [1.0, 1.0], [0.0, 0.0],
        annotations)
    video['encoder'] = encoder
    video['playback_speed'] = PLAYBACK_SPEED
    video['timing_alignment'] = {
        'basis': 'per_frame_gazebo_timestamp_resampling',
        'prior_linear_input_pts_scales': scales,
        'prior_linear_input_start_offsets_s': offsets,
        'common_start_sim_s': common_start_s,
        'common_end_sim_s': common_end_s,
    }
    for view, aligned in zip(views, aligned_paths):
        view['clock_aligned'] = {
            'path': str(aligned.resolve()), 'sha256': _sha256(aligned),
            'bytes': aligned.stat().st_size,
        }
    video['annotations'] = annotations
    video['video_4x'] = {
        'path': str(output.resolve()), 'sha256': _sha256(output),
        'bytes': output.stat().st_size,
    }
    if recoverable_encode_failure:
        video['recovery'] = {
            'scope': 'derived_video_only',
            'prior_failure': summary['failure'],
            'raw_evidence_unchanged': True,
        }
        summary['status'] = 'PASS'
        summary['gate_failures'] = []
        summary['failure'] = None
    temporary = summary_path.with_suffix('.json.tmp')
    temporary.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    temporary.replace(summary_path)
    print(json.dumps({
        'status': 'PASS', 'video': video['video_4x'],
        'timing_alignment': video['timing_alignment'],
    }, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
