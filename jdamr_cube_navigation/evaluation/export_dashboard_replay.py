#!/usr/bin/env python3
"""Export causal ROS samples on the uncompressed camera wall-time axis."""

import argparse
import bisect
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2

from navigation_dashboard import LiveState
from navigation_mcap_reader import read_navigation_messages  # noqa: I201
from render_simulator_portfolio_media import (  # noqa: I201
    _read_telemetry, _saved_map_points, _sha256)


def trimmed_start_ns(start_ns: int, end_ns: int, offset_s: float) -> int:
    """Keep the trim inside the captured interval on the original clock."""
    if not math.isfinite(offset_s) or offset_s < 0:
        raise ValueError('start offset must be finite and nonnegative')
    shifted_ns = start_ns + round(offset_s * 1e9)
    if shifted_ns >= end_ns:
        raise ValueError('start offset must precede capture end')
    return shifted_ns


def prepare_scene_video(capture: Path, output: Path, offset_s: float = 0.0) -> None:
    """Resample raw camera frames at their receipt times, without overlays."""
    raw = capture.parent / 'gazebo_sensor_raw.mp4'
    metadata = json.loads(capture.read_text(encoding='utf-8'))
    stamps = [frame['wall_ns'] for frame in metadata['frame_timestamps']]
    if not stamps or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('camera frame timestamps must increase')
    start_ns = trimmed_start_ns(stamps[0], stamps[-1], offset_s)
    if output.exists() or output.resolve() == raw.resolve():
        raise ValueError('scene output must be a new file')
    reader = cv2.VideoCapture(str(raw))
    writer = None
    fps = 30.0
    try:
        ok, frame = reader.read()
        if not ok or int(reader.get(cv2.CAP_PROP_FRAME_COUNT)) != len(stamps):
            raise ValueError('raw frame count differs from capture metadata')
        height, width = frame.shape[:2]
        writer = subprocess.Popen([
            'ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'rawvideo',
            '-pixel_format', 'bgr24', '-video_size', f'{width}x{height}',
            '-framerate', str(fps), '-i', '-', '-an', '-c:v', 'libx264',
            '-threads', '2', '-preset', 'fast', '-crf', '22',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)],
            stdin=subprocess.PIPE)
        count = math.ceil((stamps[-1] - start_ns) / 1e9 * fps) + 1
        source_index = 0
        for index in range(count):
            stamp_ns = start_ns + round(index / fps * 1e9)
            while source_index + 1 < len(stamps) and stamps[source_index + 1] <= stamp_ns:
                ok, frame = reader.read()
                if not ok:
                    raise ValueError('raw camera decode ended early')
                source_index += 1
            writer.stdin.write(frame.tobytes())
        writer.stdin.close()
        if writer.wait(timeout=30) != 0:
            raise RuntimeError('scene video encoding failed')
        evidence = {'video_sha256': _sha256(output), 'raw_sha256': _sha256(raw),
                    'capture_sha256': _sha256(capture), 'fps': fps,
                    'duration_s': count / fps, 'frames': count,
                    'start_offset_s': offset_s,
                    'method': 'causal_camera_frame_hold_on_wall_time'}
        output.with_suffix('.timing.json').write_text(
            json.dumps(evidence, indent=2), encoding='utf-8')
    finally:
        reader.release()
        if writer is not None and writer.poll() is None:
            writer.terminate()
            try:
                writer.wait(timeout=5)
            except subprocess.TimeoutExpired:
                writer.kill()
                writer.wait(timeout=5)


def validate_video(video: Path, capture: Path, duration_s: float,
                   offset_s: float = 0.0) -> dict:
    """Require generated timing provenance, source identity and wall duration."""
    timing = json.loads(video.with_suffix('.timing.json').read_text(encoding='utf-8'))
    if (timing.get('start_offset_s', 0.0) != offset_s
            or timing.get('method') != 'causal_camera_frame_hold_on_wall_time'
            or timing.get('video_sha256') != _sha256(video)
            or timing.get('capture_sha256') != _sha256(capture)
            or timing.get('raw_sha256') != _sha256(capture.parent / 'gazebo_sensor_raw.mp4')):
        raise ValueError('scene video timing provenance mismatch')
    result = subprocess.run([
        'ffprobe', '-v', 'error', '-show_entries', 'format=duration',
        '-of', 'json', str(video)], check=True, capture_output=True, text=True)
    encoded_s = float(json.loads(result.stdout)['format']['duration'])
    if not math.isfinite(encoded_s) or abs(encoded_s - duration_s) > 0.1:
        raise ValueError('video duration differs from camera wall time')
    return timing


def export_replay(mcap: Path, capture: Path, offset_s: float = 0.0) -> dict:
    """Preserve missing observations and never substitute future samples."""
    metadata = json.loads(capture.read_text(encoding='utf-8'))
    urdf = capture.parent / 'jdamr_cube_capture.urdf'
    origin = ET.parse(urdf).getroot().find("./joint[@name='laser_joint']/origin")
    rpy = [float(value) for value in origin.attrib['rpy'].split()]
    if rpy[:2] != [0.0, 0.0]:
        raise ValueError('replay requires a planar laser transform')
    frames = metadata['frame_timestamps']
    start_ns, end_ns = frames[0]['wall_ns'], frames[-1]['wall_ns']
    start_ns = trimmed_start_ns(start_ns, end_ns, offset_s)
    telemetry = _read_telemetry(mcap)
    capture_view = 'unknown'
    for name in ('gazebo_capture_urdf.json', 'gazebo_capture_world.json'):
        report_path = capture.parent / name
        if report_path.is_file():
            report = json.loads(report_path.read_text(encoding='utf-8'))
            capture_view = report.get('camera', {}).get('view', capture_view)
    telemetry['monitor'] = []
    telemetry['navigation'] = []
    for item in read_navigation_messages(mcap, topics=[
            '/collision_monitor_state', '/navigate_to_pose/_action/status']):
        message = item.ros_msg
        if item.channel.topic == '/collision_monitor_state':
            telemetry['monitor'].append((
                item.log_time_ns, int(message.action_type)))
        elif message.status_list:
            telemetry['navigation'].append((
                item.log_time_ns, int(message.status_list[-1].status)))
    times = {key: [row[0] for row in rows]
             for key, rows in telemetry.items()}

    def latest(key, stamp_ns):
        index = bisect.bisect_right(times[key], stamp_ns) - 1
        return telemetry[key][index] if index >= 0 else None

    samples = []
    for stamp_ns in range(start_ns, end_ns + 1, 200_000_000):
        sample = {'time_s': (stamp_ns - start_ns) / 1e9,
                  'source': 'recorded_ros', 'ages_s': {}}
        scan = latest('scans', stamp_ns)
        if scan:
            state = LiveState()
            # Use the captured joint transform so front means base-forward.
            state.update_scan([float(value) for value in scan[5]],
                              scan[1] + rpy[2], scan[2], scan[3], scan[4])
            sample['scan'] = state.scan
            sample['scan']['points'] = [
                [(angle + 180.0) % 360.0 - 180.0, distance]
                for angle, distance in state.scan['points']]
            angle = state.scan['closest_angle_deg']
            if angle is not None:
                sample['scan']['closest_angle_deg'] = (angle + 180.0) % 360.0 - 180.0
        for key, output in [('cmd', 'command'), ('odom', 'pose')]:
            value = latest(key, stamp_ns)
            sample[output] = ({'linear_mps': value[1], 'angular_rps': value[2]}
                              if value else {})
        if 'poses' in telemetry:
            pose = latest('poses', stamp_ns)
            sample['map_pose'] = list(pose[1:]) if pose else None
            scan_pose = latest('poses', scan[0]) if scan else None
            sample['scan_pose'] = list(scan_pose[1:]) if scan_pose else None
        plan = latest('plans', stamp_ns)
        sample['plan'] = plan[1] if plan else []
        monitor = latest('monitor', stamp_ns)
        navigation = latest('navigation', stamp_ns)
        sample['monitor'] = {'action': (
            {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH',
             4: 'LIMIT'}.get(monitor[1], 'UNKNOWN') if monitor else 'NO_DATA')}
        sample['navigation'] = {'status': (
            {1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED',
             5: 'CANCELED', 6: 'ABORTED'}.get(navigation[1], 'UNKNOWN')
            if navigation else 'NO_DATA')}
        for key in ('scans', 'cmd', 'odom', 'monitor', 'navigation', 'plans'):
            value = latest(key, stamp_ns)
            sample['ages_s'][key] = (
                (stamp_ns - value[0]) / 1e9 if value else None)
        samples.append(sample)
    return {
        'schema_version': 1, 'source': 'recorded_ros', 'samples': samples,
        'duration_s': (end_ns - start_ns) / 1e9,
        'start_offset_s': offset_s,
        'synchronization': 'camera wall time / MCAP receipt time; no speedup',
        'bt_ticks_available': False,
        'capture_view': capture_view,
        'sources': {'mcap_sha256': _sha256(mcap),
                    'capture_sha256': _sha256(capture),
                    'urdf_sha256': _sha256(urdf)},
    }


def main():
    """Write a bounded browser replay without launching ROS or a GPU."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mcap', required=True, type=Path)
    parser.add_argument('--capture', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--video', required=True, type=Path,
                        help='Unsped wall-time video, not a highlight reel')
    parser.add_argument('--prepare-scene', action='store_true',
                        help='Create a new CPU-only wall-time video from raw frames')
    parser.add_argument('--start-offset-s', type=float, default=0.0,
                        help='Trim the same leading interval from video and ROS replay')
    args = parser.parse_args()
    result = export_replay(args.mcap, args.capture, args.start_offset_s)
    if args.prepare_scene:
        prepare_scene_video(args.capture, args.video, args.start_offset_s)
    result['video_timing'] = validate_video(
        args.video, args.capture, result['duration_s'], args.start_offset_s)
    result['video_sha256'] = _sha256(args.video)
    map_path = Path(__file__).parent / 'assets/nav_obstacle/slam_corridor_eval.yaml'
    keepout_path = args.capture.parent.parent / 'assets/sim_keepout_mask.yaml'
    result['map_points'] = _saved_map_points(map_path)
    result['keepout_points'] = (
        _saved_map_points(keepout_path) if keepout_path.is_file() else [])
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, allow_nan=False), encoding='utf-8')


if __name__ == '__main__':
    main()
