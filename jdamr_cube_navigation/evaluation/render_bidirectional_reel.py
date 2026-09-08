#!/usr/bin/env python3
"""Join two verified pedestrian-entry runs into one portfolio reel."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from typing import Any


HIGHLIGHT_NAME = 'gazebo_dynamic_obstacle_highlight.mp4'


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _load_directional_source(manifest_path: Path) -> dict[str, Any]:
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('status') != 'PASS':
        raise RuntimeError(f'non-passing media manifest: {manifest_path}')
    direction = manifest.get('verified_metrics', {}).get('entry_side')
    if direction not in {'left', 'right'}:
        raise RuntimeError(f'unverified entry direction: {manifest_path}')
    output = next(
        (item for item in manifest.get('outputs', [])
         if Path(item.get('path', '')).name == HIGHLIGHT_NAME),
        None)
    if output is None:
        raise RuntimeError(f'highlight missing: {manifest_path}')
    video_path = Path(output['path'])
    if not video_path.is_file() or _sha256(video_path) != output['sha256']:
        raise RuntimeError(f'highlight hash mismatch: {video_path}')
    return {
        'direction': direction,
        'manifest': str(manifest_path.resolve()),
        'manifest_sha256': _sha256(manifest_path),
        'video': str(video_path.resolve()),
        'video_sha256': output['sha256'],
        'verified_metrics': manifest['verified_metrics'],
    }


def main() -> int:
    """Validate both directions and render their combined public video."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--left-manifest', required=True, type=Path)
    parser.add_argument('--right-manifest', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error('--output-dir must be new or empty')
    args.output_dir.mkdir(parents=True, exist_ok=True)

    left = _load_directional_source(args.left_manifest)
    right = _load_directional_source(args.right_manifest)
    if left['direction'] != 'left' or right['direction'] != 'right':
        raise RuntimeError(
            'left/right manifests were supplied in the wrong order')

    reel_path = args.output_dir / 'gazebo_pedestrian_bidirectional_reel.mp4'
    subprocess.run([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-i', str(left['video']), '-i', str(right['video']),
        '-filter_complex', '[0:v][1:v]concat=n=2:v=1:a=0[v]',
        '-map', '[v]', '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(reel_path),
    ], check=True)
    manifest = {
        'schema_version': 1,
        'status': 'PASS',
        'claim_scope': 'TWO_INDEPENDENT_SIM_INTEGRATION_RUNS',
        'interpretation': (
            'Two independently verified single-pedestrian runs concatenated; '
            'not a simultaneous multi-person safety claim.'),
        'sequence': [left, right],
        'output': {
            'path': str(reel_path.resolve()),
            'sha256': _sha256(reel_path),
            'size_bytes': reel_path.stat().st_size,
        },
    }
    manifest_path = args.output_dir / 'bidirectional_reel_manifest.json'
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True)
        + '\n', encoding='utf-8')
    print(json.dumps(manifest, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
