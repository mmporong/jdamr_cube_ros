#!/usr/bin/env python3
"""Rebuild an already sealed restaurant replay video on capture time."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from run_restaurant_replay_sim import (
    _sha256,
    encode_camera_video,
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
    scales = [
        float(view['capture']['capture_duration_s']) * fps
        / max(1, int(view['capture']['frames']))
        for view in views]
    output = args.run_dir / 'gazebo_actual_map_2x_synced.mp4'
    if output.exists():
        parser.error(f'output already exists: {output}')
    encoder = args.encoder or select_video_encoder()
    encode_camera_video(
        raw_paths, output, fps, encoder, scales)
    video['encoder'] = encoder
    video['timing_alignment'] = {
        'basis': 'capture_steady_duration_per_frame',
        'input_pts_scales': scales,
    }
    video['video_2x'] = {
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
        'status': 'PASS', 'video': video['video_2x'],
        'timing_alignment': video['timing_alignment'],
    }, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
