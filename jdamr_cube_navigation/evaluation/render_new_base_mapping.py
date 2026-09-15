#!/usr/bin/env python3
"""Render recorded Cartographer map updates, not a simulated camera view."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw, ImageFont


MAP_TYPE = 'nav_msgs/msg/OccupancyGrid'
TF_TYPE = 'tf2_msgs/msg/TFMessage'
CANVAS = (960, 720)
MAP_BOX = (48, 104, 912, 640)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _input_files(path: Path) -> list[dict]:
    """Inventory a bag directory without treating it as one opaque file."""
    if not path.exists():
        raise FileNotFoundError(path)
    files = sorted(path.rglob('*')) if path.is_dir() else [path]
    files = [item for item in files if item.is_file()]
    if not files:
        raise ValueError(f'input contains no files: {path}')
    return [{'path': str(item.resolve()), 'sha256': _sha256(item),
             'size_bytes': item.stat().st_size} for item in files]


def _decode_map(data: bytes, schema, factory):
    if schema.data:
        decoder = factory.decoder_for('cdr', schema)
        if decoder is None:
            raise ValueError('unsupported embedded OccupancyGrid schema')
        return decoder(data)
    from nav_msgs.msg import OccupancyGrid
    from rclpy.serialization import deserialize_message

    return deserialize_message(data, OccupancyGrid)


def _read_maps(path: Path, max_snapshots: int) -> tuple[list[dict], int]:
    """Keep bounded, changed occupancy snapshots and count observed /tf."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    snapshots = []
    tf_messages = 0
    prior_fingerprint = None
    factory = DecoderFactory()
    with path.open('rb') as stream:
        reader = make_reader(stream, validate_crcs=True)
        for schema, channel, message in reader.iter_messages(
                topics=['/map', '/tf'], log_time_order=True):
            expected = MAP_TYPE if channel.topic == '/map' else TF_TYPE
            if (schema is None or schema.name != expected or
                    channel.message_encoding != 'cdr'):
                raise ValueError(f'unexpected ROS type on {channel.topic}')
            if channel.topic == '/tf':
                tf_messages += 1
                continue
            grid = _decode_map(message.data, schema, factory)
            width, height = int(grid.info.width), int(grid.info.height)
            resolution = float(grid.info.resolution)
            if (width <= 0 or height <= 0 or
                    not math.isfinite(resolution) or resolution <= 0 or
                    len(grid.data) != width * height):
                raise ValueError('invalid OccupancyGrid dimensions or data')
            if grid.header.frame_id != 'map':
                raise ValueError('OccupancyGrid must be in map frame')
            origin = grid.info.origin
            if (not all(math.isfinite(value) for value in
                        [origin.position.x, origin.position.y,
                         origin.orientation.x, origin.orientation.y,
                         origin.orientation.z, origin.orientation.w]) or
                    abs(origin.orientation.x) > 1e-6 or
                    abs(origin.orientation.y) > 1e-6 or
                    abs(origin.orientation.z) > 1e-6 or
                    abs(abs(origin.orientation.w) - 1) > 1e-6):
                raise ValueError('only axis-aligned map origins are supported')
            values = bytes((int(value) + 256) % 256
                           for value in grid.data)
            fingerprint = hashlib.sha256(
                values + f'{width},{height},{resolution},'
                f'{origin.position.x},{origin.position.y}'.encode()).hexdigest()
            if fingerprint == prior_fingerprint:
                continue
            prior_fingerprint = fingerprint
            unknown = values.count(255)
            occupied = sum(value >= 50 and value != 255 for value in values)
            snapshots.append({
                'time_ns': message.log_time, 'width': width,
                'height': height, 'resolution_m': resolution,
                'origin_x_m': float(origin.position.x),
                'origin_y_m': float(origin.position.y),
                'unknown_cells': unknown,
                'known_cells': width * height - unknown,
                'occupied_cells': occupied, 'data': values,
            })
            if len(snapshots) > max_snapshots:
                latest = snapshots[-1]
                snapshots = snapshots[::2]
                if snapshots[-1] is not latest:
                    snapshots.append(latest)
    if not snapshots:
        raise ValueError('MCAP has no changing /map OccupancyGrid messages')
    resolutions = {item['resolution_m'] for item in snapshots}
    if len(resolutions) != 1:
        raise ValueError('map resolution changed during recording')
    return snapshots, tf_messages


def _world_bounds(snapshots: list[dict]) -> tuple[float, float, int, int]:
    resolution = snapshots[0]['resolution_m']
    min_x = min(item['origin_x_m'] for item in snapshots)
    min_y = min(item['origin_y_m'] for item in snapshots)
    max_x = max(item['origin_x_m'] + item['width'] * resolution
                for item in snapshots)
    max_y = max(item['origin_y_m'] + item['height'] * resolution
                for item in snapshots)
    world_w = math.ceil((max_x - min_x) / resolution - 1e-7)
    world_h = math.ceil((max_y - min_y) / resolution - 1e-7)
    return min_x, min_y, world_w, world_h


def _map_image(sample: dict, bounds: tuple) -> Image.Image:
    min_x, min_y, world_w, world_h = bounds
    resolution = sample['resolution_m']
    ox = round((sample['origin_x_m'] - min_x) / resolution)
    oy = round((sample['origin_y_m'] - min_y) / resolution)
    if (abs(min_x + ox * resolution - sample['origin_x_m']) >
            resolution * 0.01 or
            abs(min_y + oy * resolution - sample['origin_y_m']) >
            resolution * 0.01):
        raise ValueError('map origin is not aligned to resolution')
    width, height = sample['width'], sample['height']
    pixels = bytes(88 if value == 255 else 30 if value >= 50 else 224
                   for value in sample['data'])
    tile = Image.frombytes('L', (width, height), pixels)
    tile = tile.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
    world = Image.new('L', (world_w, world_h), 88)
    world.paste(tile, (ox, world_h - oy - height))
    return world


def _frame(world: Image.Image, sample: dict, index: int, total: int,
           first_time_ns: int) -> Image.Image:
    frame = Image.new('RGB', CANVAS, '#101923')
    draw = ImageDraw.Draw(frame)
    font = ImageFont.load_default()
    draw.text((48, 32), 'CARTOGRAPHER / 2D LIDAR MAP RECORD',
              fill='#eff4f8', font=font)
    draw.text((48, 60), 'Recorded /map OccupancyGrid  |  not camera footage',
              fill='#a7bbc9', font=font)
    left, top, right, bottom = MAP_BOX
    avail_w, avail_h = right - left, bottom - top
    factor = min(avail_w / world.width, avail_h / world.height)
    size = (max(1, round(world.width * factor)),
            max(1, round(world.height * factor)))
    displayed = world.resize(size, Image.Resampling.NEAREST)
    x = left + (avail_w - size[0]) // 2
    y = top + (avail_h - size[1]) // 2
    frame.paste(displayed.convert('RGB'), (x, y))
    draw.rectangle((x, y, x + size[0] - 1, y + size[1] - 1),
                   outline='#7aa6be', width=1)
    elapsed = (sample['time_ns'] - first_time_ns) / 1e9
    draw.text((48, 664),
              f'actual map t+{elapsed:.1f}s  |  snapshot {index}/{total}'
              f'  |  known cells {sample["known_cells"]:,}',
              fill='#eff4f8', font=font)
    draw.text((48, 687),
              'Gray unknown  /  light free  /  dark occupied'
              '  |  map frame: map', fill='#a7bbc9', font=font)
    return frame


def render(map_mcap: Path, source_bag: Path, output_dir: Path,
           run_log: Path | None = None, fps: int = 12,
           duration_s: float = 24.0, max_snapshots: int = 240) -> dict:
    """Create a map-growth reel and the last recorded online-map PNG."""
    if fps < 1 or duration_s <= 0 or max_snapshots < 2:
        raise ValueError('fps, duration and max_snapshots must be positive')
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError('output directory must be new or empty')
    if not map_mcap.is_file():
        raise FileNotFoundError(map_mcap)
    if map_mcap.resolve() == source_bag.resolve():
        raise ValueError('source bag must differ from Cartographer output')
    inputs = {'cartographer_map_mcap': _input_files(map_mcap),
              'source_bag': _input_files(source_bag)}
    if run_log is not None:
        inputs['run_log'] = _input_files(run_log)
    snapshots, tf_messages = _read_maps(map_mcap, max_snapshots)
    bounds = _world_bounds(snapshots)
    output_dir.mkdir(parents=True, exist_ok=True)
    final = snapshots[-1]
    # The authoritative optimized final map is converted from the pbstream
    # after replay shutdown; this image is only the last recorded /map update.
    final_png = output_dir / 'new_base_cartographer_last_recorded_map.png'
    final_image = _map_image(final, bounds)
    _frame(final_image, final, len(snapshots), len(snapshots),
           snapshots[0]['time_ns']).save(final_png)
    video = output_dir / 'new_base_cartographer_mapping.mp4'
    process = subprocess.Popen([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size',
        f'{CANVAS[0]}x{CANVAS[1]}', '-framerate', str(fps), '-i', '-',
        '-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video),
    ], stdin=subprocess.PIPE)
    frame_times = []
    total_frames = max(1, round(duration_s * fps))
    first, last = snapshots[0]['time_ns'], snapshots[-1]['time_ns']
    index = 0
    rendered_index = -1
    frame_bytes = b''
    try:
        for frame_number in range(total_frames):
            fraction = (frame_number / (total_frames - 1)
                        if total_frames > 1 else 1.0)
            target_ns = first + (last - first) * fraction
            while (index + 1 < len(snapshots) and
                   snapshots[index + 1]['time_ns'] <= target_ns):
                index += 1
            sample = snapshots[index]
            if index != rendered_index:
                world = _map_image(sample, bounds)
                frame = _frame(world, sample, index + 1,
                               len(snapshots), first)
                frame_bytes = frame.tobytes()
                rendered_index = index
            process.stdin.write(frame_bytes)
            frame_times.append({
                'video_frame': frame_number,
                'video_elapsed_s': round(frame_number / fps, 6),
                'map_mcap_log_time_ns': sample['time_ns'],
                'known_cells': sample['known_cells'],
            })
    finally:
        process.stdin.close()
    if process.wait() != 0:
        raise RuntimeError('ffmpeg video encoding failed')
    manifest = {
        'schema_version': 1,
        'claim_scope': 'RECORDED_CARTOGRAPHER_2D_OCCUPANCY_GRID_ONLY',
        'not_claimed': ['physical camera video', '3D map',
                        'automatic human/obstacle classification'],
        'inputs': inputs,
        'observed_topics': {'/map_changed_snapshots': len(snapshots),
                            '/tf_messages': tf_messages},
        'map': {
            'frame_id': 'map', 'last_recorded_width_cells': final['width'],
            'last_recorded_height_cells': final['height'],
            'resolution_m_per_cell': final['resolution_m'],
            'last_recorded_known_cells': final['known_cells'],
            'last_recorded_unknown_cells': final['unknown_cells'],
            'last_recorded_occupied_cells': final['occupied_cells'],
            'world_canvas_width_cells': bounds[2],
            'world_canvas_height_cells': bounds[3],
            'first_map_log_time_ns': first,
            'last_map_log_time_ns': last,
        },
        'timing': {'video_fps': fps, 'video_duration_s': duration_s,
                   'frame_times': frame_times},
        'outputs': {
            'png': {'path': str(final_png.resolve()),
                    'sha256': _sha256(final_png)},
            'mp4': {'path': str(video.resolve()),
                    'sha256': _sha256(video)},
        },
    }
    sidecar = output_dir / 'new_base_mapping_provenance.json'
    sidecar.write_text(json.dumps(manifest, ensure_ascii=False, indent=2,
                                  sort_keys=True) + '\n', encoding='utf-8')
    return manifest


def main() -> int:
    """Run the offline renderer after the map-output MCAP exists."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-mcap', type=Path, required=True)
    parser.add_argument('--source-bag', type=Path, required=True)
    parser.add_argument('--run-log', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--fps', type=int, default=12)
    parser.add_argument('--duration-s', type=float, default=24.0)
    args = parser.parse_args()
    result = render(args.map_mcap, args.source_bag, args.output_dir,
                    args.run_log, args.fps, args.duration_s)
    print(json.dumps({'outputs': result['outputs'], 'map': result['map']},
                     ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
