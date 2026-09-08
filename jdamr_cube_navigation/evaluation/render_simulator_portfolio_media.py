#!/usr/bin/env python3
"""Turn a passing Gazebo sensor recording into traceable portfolio media."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np  # noqa: I201
from PIL import Image, ImageDraw, ImageFont  # noqa: I100,I201


REGULAR_FONT = Path(
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
BOLD_FONT = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _font(path: Path, size: int) -> ImageFont.FreeTypeFont:
    if not path.is_file():
        raise RuntimeError(f'Korean font unavailable: {path}')
    return ImageFont.truetype(str(path), size=size)


def _ffmpeg_writer(path: Path, width: int, height: int, fps: float):
    command = [
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-f', 'rawvideo', '-pix_fmt', 'bgr24',
        '-s', f'{width}x{height}', '-r', f'{fps:g}', '-i', '-',
        '-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path),
    ]
    return subprocess.Popen(command, stdin=subprocess.PIPE)


def _event_frame_interval(
        evidence: dict, metadata: dict, total_frames: int,
) -> tuple[int, int]:
    """Map verified steady-clock events onto retained camera frames."""
    events = {
        item.get('name'): item for item in evidence.get('events', [])
        if isinstance(item, dict)}
    first_ns = metadata.get('first_frame_steady_ns')
    last_ns = metadata.get('last_frame_steady_ns')
    start_ns = events.get('obstacle_crossing_started', {}).get('steady_ns')
    end_ns = events.get('clear_set_pose_requested', {}).get('steady_ns')
    values = (first_ns, last_ns, start_ns, end_ns, total_frames)
    if (not all(isinstance(value, int) for value in values)
            or total_frames < 20
            or not first_ns < start_ns < end_ns < last_ns):
        raise RuntimeError('verified pedestrian event timing is incomplete')
    scale = (total_frames - 1) / (last_ns - first_ns)
    start = round((start_ns - first_ns) * scale)
    end = round((end_ns - first_ns) * scale)
    if end - start + 1 < 3:
        raise RuntimeError('verified pedestrian visibility is too short')
    return start, end


def _pedestrian_bounds(frame: np.ndarray):
    """Find the largest dark-red person torso above the robot body."""
    height = frame.shape[0]
    region = frame[:round(height * 0.55)]
    blue, green, red = cv2.split(region)
    red16 = red.astype(np.int16)
    mask = ((red > 70) & (red16 > green.astype(np.int16) + 25)
            & (red16 > blue.astype(np.int16) + 25)).astype(np.uint8)
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    candidates = [stats[index] for index in range(1, count)
                  if stats[index, cv2.CC_STAT_AREA] >= 200]
    if not candidates:
        return None
    left, top, width, box_height, _ = max(
        candidates, key=lambda item: item[cv2.CC_STAT_AREA])
    return int(left), int(top), int(left + width), int(top + box_height)


def _box(draw: ImageDraw.ImageDraw, xy, fill, radius=16, outline=None,
         width=1) -> None:
    draw.rounded_rectangle(
        xy, radius=radius, fill=fill, outline=outline, width=width)


def _state(index: int, obstacle_start: int, obstacle_end: int,
           arrival_start: int) -> tuple[str, tuple[int, int, int, int]]:
    if index < obstacle_start:
        return '자율주행', (35, 125, 225, 235)
    if index <= obstacle_end:
        return '장애물 감지 · 정지', (215, 55, 45, 240)
    if index < arrival_start:
        return '동일 목표로 재개', (25, 155, 105, 235)
    return '목표 도착', (30, 165, 90, 240)


def _annotate(
        frame: np.ndarray, index: int, total_frames: int,
        obstacle_start: int, obstacle_end: int, arrival_start: int,
        metrics: dict, compressed: bool = False) -> np.ndarray:
    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).convert(
        'RGBA')
    overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    title_font = _font(BOLD_FONT, 31)
    subtitle_font = _font(REGULAR_FONT, 18)
    state_font = _font(BOLD_FONT, 21)
    label_font = _font(REGULAR_FONT, 15)
    value_font = _font(BOLD_FONT, 24)
    small_font = _font(REGULAR_FONT, 13)

    _box(draw, (28, 26, 1252, 142), (13, 20, 31, 224), radius=18)
    draw.text((52, 45), 'JDAMR · 보행자 횡단 정지 후 동일 목표 재개',
              font=title_font, fill=(245, 248, 252, 255))
    entry_label = {
        'left': '좌측→경로', 'right': '우측→경로',
    }.get(metrics.get('entry_side'), '진입 방향 기록')
    draw.text((53, 91),
              '실제 Gazebo 카메라 센서 · Nav2 Collision Monitor · '
              f'{entry_label} · 동시 MCAP 검증',
              font=subtitle_font, fill=(168, 186, 207, 255))

    state_text, state_color = _state(
        index, obstacle_start, obstacle_end, arrival_start)
    state_left = 978 if len(state_text) <= 5 else 930
    _box(draw, (state_left, 52, 1224, 110), state_color, radius=18)
    draw.text((state_left + 18, 67), state_text, font=state_font,
              fill=(255, 255, 255, 255))
    if compressed:
        _box(draw, (1115, 116, 1224, 137), (245, 166, 35, 235), radius=9)
        draw.text((1131, 117), '주행 8×', font=small_font,
                  fill=(20, 25, 30, 255))

    if obstacle_start <= index <= obstacle_end:
        bounds = _pedestrian_bounds(frame)
        if bounds is not None:
            left, top, right, bottom = bounds
            left, top = left - 12, top - 12
            right, bottom = right + 12, bottom + 12
            draw.rounded_rectangle(
                (left, top, right, bottom), radius=8,
                outline=(255, 74, 62, 255), width=4)
            _box(draw, (left, top - 34, left + 176, top - 5),
                 (215, 55, 45, 240), radius=9)
            draw.text((left + 10, top - 32), '보행자 횡단 · LiDAR 감지',
                      font=label_font, fill=(255, 255, 255, 255))

    _box(draw, (28, 554, 1252, 696), (13, 20, 31, 224), radius=18)
    cells = [
        ('목표 전송 / 취소',
         f"{metrics['goal_send_count']}회 / {metrics['goal_cancel_count']}회"),
        ('물리 접촉', f"{metrics['contact_count']}회"),
        ('보호영역 최소 이격',
         f"{metrics['protected_clearance_m']:.3f} m"),
        ('scan 수신 → 정지명령', f"{metrics['command_latency_s']:.3f} s"),
    ]
    for cell, (label, value) in enumerate(cells):
        x = 52 + cell * 300
        draw.text((x, 574), label, font=label_font,
                  fill=(157, 178, 202, 255))
        draw.text((x, 600), value, font=value_font,
                  fill=(247, 249, 252, 255))
        if cell < len(cells) - 1:
            draw.line((x + 270, 574, x + 270, 640),
                      fill=(75, 91, 110, 220), width=1)
    draw.text((52, 650),
              '파랑: 시작점  ·  초록: 목표점  ·  빨강: 횡단 보행자  '
              '·  결과: 접촉 없이 도착 PASS',
              font=small_font, fill=(170, 189, 210, 255))
    progress_left, progress_right, progress_y = 52, 1228, 681
    draw.line((progress_left, progress_y, progress_right, progress_y),
              fill=(72, 90, 111, 255), width=4)
    current_x = progress_left + int(
        (progress_right - progress_left) * index / max(1, total_frames - 1))
    draw.line((progress_left, progress_y, current_x, progress_y),
              fill=(48, 164, 235, 255), width=4)
    draw.ellipse((current_x - 6, progress_y - 6,
                  current_x + 6, progress_y + 6),
                 fill=(245, 248, 252, 255))

    composed = Image.alpha_composite(image, overlay).convert('RGB')
    return cv2.cvtColor(np.asarray(composed), cv2.COLOR_RGB2BGR)


def _write_gif(source: Path, output: Path) -> None:
    subprocess.run([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-i', str(source), '-filter_complex',
        ('fps=8,scale=800:-1:flags=lanczos,split[s0][s1];'
         '[s0]palettegen=max_colors=128[p];'
         '[s1][p]paletteuse=dither=bayer:bayer_scale=3'),
        str(output),
    ], check=True)


def main() -> int:
    """Validate one passing run and render its public media bundle."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-root', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error('--output-dir must be new or empty')
    args.output_dir.mkdir(parents=True, exist_ok=True)

    summary_path = args.run_root / 'summary.json'
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    if summary.get('status') != 'PASS' or len(summary.get('results', [])) != 1:
        raise RuntimeError('portfolio media requires one passing run')
    result = summary['results'][0]
    if result.get('status') != 'PASS' or 'simulator_video' not in result:
        raise RuntimeError('passing Gazebo sensor video evidence is missing')
    raw_video = Path(result['simulator_video']['raw_video']['path'])
    if _sha256(raw_video) != result['simulator_video']['raw_video']['sha256']:
        raise RuntimeError('raw Gazebo video hash mismatch')
    evidence_path = args.run_root / result['case'] / 'scenario.json'
    evidence = json.loads(evidence_path.read_text(encoding='utf-8'))
    scenario = result['scenario']
    metrics = {
        'goal_send_count': scenario['goal_send_count'],
        'goal_cancel_count': scenario['goal_cancel_count'],
        'contact_count': scenario['contact_count'],
        'protected_clearance_m':
            scenario['protected_envelope_minimum_clearance_m'],
        'command_latency_s': scenario['observer_scan_to_zero_command_s'],
        'entry_side': evidence.get('obstacle_entry_side'),
    }
    capture = cv2.VideoCapture(str(raw_video))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if (width, height) != (1280, 720) or fps <= 0:
        raise RuntimeError('unexpected Gazebo source video geometry')
    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    metadata_path = Path(
        result['simulator_video']['capture_metadata']['path'])
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    if metadata.get('frames') != total_frames:
        raise RuntimeError('camera metadata frame count mismatch')
    obstacle_start, obstacle_end = _event_frame_interval(
        evidence, metadata, total_frames)
    arrival_start = max(obstacle_end + 1, total_frames - round(2.0 * fps))
    full_path = args.output_dir / 'gazebo_dynamic_obstacle_full.mp4'
    highlight_path = args.output_dir / 'gazebo_dynamic_obstacle_highlight.mp4'
    full_writer = _ffmpeg_writer(full_path, width, height, fps)
    highlight_writer = _ffmpeg_writer(highlight_path, width, height, fps)
    if full_writer.stdin is None or highlight_writer.stdin is None:
        raise RuntimeError('ffmpeg stdin unavailable')
    highlight_start = max(0, obstacle_start - round(2.0 * fps))
    normal_until = min(
        arrival_start - 1, obstacle_end + round(4.0 * fps))
    compressed_stride = 8
    posters = {
        obstacle_start + (obstacle_end - obstacle_start) // 2:
            args.output_dir / 'gazebo_obstacle_stop.png',
        total_frames - 1: args.output_dir / 'gazebo_goal_arrival.png',
    }
    highlight_frames = 0
    index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        compressed = (
            normal_until < index < arrival_start
            and (index - normal_until) % compressed_stride == 0)
        annotated = _annotate(
            frame, index, total_frames, obstacle_start, obstacle_end,
            arrival_start, metrics, compressed=compressed)
        full_writer.stdin.write(annotated.tobytes())
        selected = (
            highlight_start <= index <= normal_until
            or compressed or index >= arrival_start)
        if selected:
            highlight_writer.stdin.write(annotated.tobytes())
            highlight_frames += 1
        if index in posters:
            if not cv2.imwrite(str(posters[index]), annotated):
                raise RuntimeError(f'could not write poster: {posters[index]}')
        index += 1
    capture.release()
    full_writer.stdin.close()
    highlight_writer.stdin.close()
    if full_writer.wait() != 0 or highlight_writer.wait() != 0:
        raise RuntimeError('ffmpeg video encoding failed')
    if index != total_frames:
        raise RuntimeError('source frame count changed during rendering')

    gif_path = args.output_dir / 'gazebo_dynamic_obstacle_highlight.gif'
    _write_gif(highlight_path, gif_path)
    outputs = [full_path, highlight_path, gif_path, *posters.values()]
    manifest = {
        'schema_version': 1,
        'status': 'PASS',
        'claim_scope': result['claim_scope'],
        'source': {
            'type': 'gazebo_camera_sensor_video',
            'path': str(raw_video.resolve()),
            'sha256': _sha256(raw_video),
            'summary': str(summary_path.resolve()),
            'summary_sha256': _sha256(summary_path),
            'overlays_applied_to_source': False,
        },
        'detected_visual_events': {
            'pedestrian_first_frame': obstacle_start,
            'pedestrian_last_frame': obstacle_end,
            'arrival_overlay_first_frame': arrival_start,
        },
        'verified_metrics': metrics,
        'highlight': {
            'frames': highlight_frames,
            'fps': fps,
            'middle_cruise_stride': compressed_stride,
            'middle_cruise_label': '8x',
        },
        'outputs': [{
            'path': str(path.resolve()),
            'size_bytes': path.stat().st_size,
            'sha256': _sha256(path),
        } for path in outputs],
    }
    manifest_path = args.output_dir / 'portfolio_media_manifest.json'
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2,
                   sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(manifest, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
