#!/usr/bin/env python3
"""Run one integrated restaurant obstacle and traction replay in Gazebo."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

import cv2

import numpy as np  # noqa: I201

from prepare_sim_nav_obstacle_run import prepare as prepare_navigation

from run_sim_nav_obstacle_eval import (
    _cleanup_identity,
    _environment,
    _group_members,
    _sha256,
    _start,
    _stop,
    _wait_lifecycle_active,
    _wait_tf_available,
    _wait_topics,
)


ROOT = Path(__file__).resolve().parents[2]
ASSETS = (ROOT / 'jdamr_cube_navigation' / 'evaluation' / 'assets'
          / 'nav_obstacle')
BT = (ROOT / 'jdamr_cube_navigation' / 'behavior_trees'
      / 'navigate_to_pose_dynamic_obstacle_eval.xml')
CAMERA_RECORDER = (ROOT / 'jdamr_cube_navigation' / 'evaluation'
                   / 'record_simulator_camera.py')
RECORDED_TOPICS = (
    '/tf', '/tf_static', '/scan', '/odom', '/ground_truth_pose',
    '/amcl_pose', '/plan', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel',
    '/guarded_cmd_vel', '/collision_monitor_state',
    '/sim/traction_fault_active',
    '/navigate_to_pose/_action/status', '/clock',
)


def one_mcap(bag_dir: Path) -> Path | None:
    """Return the only finalized MCAP, or None for incomplete evidence."""
    files = sorted(bag_dir.glob('*.mcap')) if bag_dir.is_dir() else []
    metadata = bag_dir / 'metadata.yaml'
    return files[0] if len(files) == 1 and metadata.is_file() else None


def classify_result(
        scenario: dict[str, Any] | None, guard: dict[str, Any] | None,
        mcap: Path | None,
        survivors: list[dict[str, Any]]) -> tuple[str, list[str]]:
    """Apply every terminal gate without laundering partial execution."""
    failures = []
    if scenario is None:
        failures.append('scenario_evidence_missing')
    else:
        if scenario.get('status') != 'PASS':
            failures.append('scenario_failed')
        if scenario.get('successful_goal_count') != 20:
            failures.append('waypoint_gate_failed')
        if scenario.get('successful_intervention_count') != 3:
            failures.append('obstacle_intervention_gate_failed')
    if guard is None:
        failures.append('guard_evidence_missing')
    else:
        if guard.get('recovery_count') != 1:
            failures.append('traction_recovery_count_failed')
        if guard.get('final_state') == 'FAULT_LATCHED':
            failures.append('traction_guard_latched')
    if mcap is None:
        failures.append('mcap_missing_or_unfinalized')
    if survivors:
        failures.append('process_survivors_present')
    return ('PASS' if not failures else 'FAIL', failures)


def load_if_present(path: Path) -> dict[str, Any] | None:
    """Load one JSON document only when the producer finalized it."""
    if not path.is_file() or path.stat().st_size == 0:
        return None
    return json.loads(path.read_text(encoding='utf-8'))


def wait_file_ready(process: subprocess.Popen, path: Path,
                    timeout_s: float = 30.0) -> None:
    """Require a producer's first-frame marker before starting the route."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.is_file() and path.stat().st_size > 0:
            return
        if process.poll() is not None:
            raise RuntimeError(
                f'camera recorder exited before ready: {path}')
        time.sleep(0.1)
    raise TimeoutError(f'camera recorder did not become ready: {path}')


def select_video_encoder() -> str:
    """Use the locally verified hardware path when NVIDIA is available."""
    return ('h264_nvenc' if Path('/dev/nvidia0').exists()
            and shutil.which('nvidia-smi') else 'libx264')


def camera_sim_timing(
        records: list[dict[str, Any]], fps: float
        ) -> tuple[list[float], list[float], float, float]:
    """Map fixed-rate recorder PTS back onto the Gazebo simulation clock."""
    first_sim_s = [
        float(item['capture']['frame_timestamps'][0]['sim_ns']) / 1e9
        for item in records]
    last_sim_s = [
        float(item['capture']['frame_timestamps'][-1]['sim_ns']) / 1e9
        for item in records]
    common_start_s = min(first_sim_s)
    scales = []
    for item, first_s, last_s in zip(records, first_sim_s, last_sim_s):
        frames = int(item['capture']['frames'])
        nominal_span_s = max(1, frames - 1) / fps
        scales.append((last_s - first_s) / nominal_span_s)
    offsets = [value - common_start_s for value in first_sim_s]
    duration_s = max(last_sim_s) - common_start_s
    return scales, offsets, duration_s, common_start_s


def uniform_sim_frame_indices(
        frame_timestamps: list[dict[str, int]], common_start_s: float,
        common_end_s: float, fps: float) -> list[int | None]:
    """Choose the latest captured frame for each uniform simulation tick."""
    if not frame_timestamps or fps <= 0.0 or common_end_s < common_start_s:
        raise ValueError('invalid simulation frame timeline')
    source_times = [item['sim_ns'] / 1e9 for item in frame_timestamps]
    frame_count = int((common_end_s - common_start_s) * fps) + 1
    indices: list[int | None] = []
    source_index = -1
    for output_index in range(frame_count):
        target_s = common_start_s + output_index / fps
        while (source_index + 1 < len(source_times)
               and source_times[source_index + 1] <= target_s):
            source_index += 1
        indices.append(source_index if source_index >= 0 else None)
    return indices


def resample_camera_on_sim_clock(
        raw: Path, capture: dict[str, Any], output: Path, fps: float,
        common_start_s: float, common_end_s: float) -> None:
    """Write a fixed-rate video selected by per-frame Gazebo timestamps."""
    indices = uniform_sim_frame_indices(
        capture['frame_timestamps'], common_start_s, common_end_s, fps)
    source = cv2.VideoCapture(str(raw))
    writer = cv2.VideoWriter(
        str(output), cv2.VideoWriter_fourcc(*'mp4v'), fps,
        (int(capture['width']), int(capture['height'])))
    if not source.isOpened() or not writer.isOpened():
        source.release()
        writer.release()
        raise RuntimeError(f'camera resampling could not open {raw}')
    latest = np.zeros(
        (int(capture['height']), int(capture['width']), 3),
        dtype=np.uint8)
    decoded_index = -1
    try:
        for selected_index in indices:
            while (selected_index is not None
                   and decoded_index < selected_index):
                ok, frame = source.read()
                if not ok:
                    raise RuntimeError(
                        f'camera frame {decoded_index + 1} is missing: {raw}')
                latest = frame
                decoded_index += 1
            writer.write(latest)
    finally:
        source.release()
        writer.release()


def scene_video_annotations(
        scenario: dict[str, Any] | None,
        guard: dict[str, Any] | None,
        common_start_s: float) -> list[dict[str, Any]]:
    """Build concise 2x-video state banners from sealed simulation times."""
    annotations = []

    def append(text: str, start_sim_s: float, end_sim_s: float,
               color: str) -> None:
        start_s = max(0.0, (start_sim_s - common_start_s) / 2.0)
        end_s = max(start_s + 0.6, (end_sim_s - common_start_s) / 2.0)
        annotations.append({
            'text': text, 'start_s': start_s, 'end_s': end_s,
            'color': color})

    for scene in (scenario or {}).get('interventions', []):
        if scene.get('kind') == 'static_avoidance':
            append(
                'STATIC BOX - LOCAL PLAN DETOUR',
                float(scene['trigger_sim_s']), float(scene['result_sim_s']),
                'orange')
        elif scene.get('kind') == 'person_crossing_emergency_stop':
            append(
                'PERSON CROSSING DETECTED',
                float(scene['trigger_sim_s']), float(scene['stop_sim_s']),
                'yellow')
            append(
                'EMERGENCY STOP - PERSON IN STOP ZONE',
                float(scene['stop_sim_s']), float(scene['clear_sim_s']),
                'red')
            append(
                'PERSON CLEAR - SAME GOAL RESUME',
                float(scene['clear_sim_s']),
                float(scene['resume_sim_s']) + 2.0,
                'lime')
    events = (guard or {}).get('events', [])
    for event, following in zip(events, events[1:] + [None]):
        labels = {
            'PROTECTIVE_STOP': ('LOW TRACTION - PROTECTIVE STOP', 'red'),
            'RELOCALIZE': ('LOW TRACTION - RELOCALIZING', 'yellow'),
            'LOW_SPEED_RESUME': (
                'LOW TRACTION - LIMITED SPEED RECOVERY', 'lime'),
            'RECOVERED': ('TRACTION RECOVERED', 'lime'),
        }
        if event['to'] in labels:
            label, color = labels[event['to']]
            start_sim_s = float(event.get('sim_s', event['at_s']))
            end_sim_s = (
                float(following.get('sim_s', following['at_s']))
                if following is not None else start_sim_s + 2.0)
            append(label, start_sim_s, end_sim_s, color)
    return annotations


def encode_camera_video(raws: list[Path], output: Path, fps: float,
                        encoder: str = 'libx264',
                        timing_scales: list[float] | None = None,
                        timing_offsets_s: list[float] | None = None,
                        canvas_duration_s: float | None = None,
                        annotations: list[dict[str, Any]] | None = None
                        ) -> None:
    """Encode one or two Gazebo views as a 2x browser-compatible MP4."""
    if len(raws) not in (1, 2):
        raise ValueError('camera encoding requires one or two views')
    scales = timing_scales or [1.0] * len(raws)
    offsets = timing_offsets_s or [0.0] * len(raws)
    if len(scales) != len(raws) or any(scale <= 0.0 for scale in scales):
        raise ValueError('camera timing scales must match every input')
    if len(offsets) != len(raws) or any(offset < 0.0 for offset in offsets):
        raise ValueError('camera timing offsets must match every input')
    command = ['ffmpeg', '-y', '-loglevel', 'error']
    for raw in raws:
        command.extend(['-i', str(raw)])
    if len(raws) == 1:
        command.extend([
            '-vf', f'setpts={0.5 * scales[0]:.9f}*PTS,fps={fps:g}'])
    else:
        command.extend([
            '-filter_complex',
            (f'[0:v]setpts={scales[0]:.9f}*PTS+'
             f'{offsets[0]:.9f}/TB,scale=854:480,'
             'drawtext=text=ACTUAL MAP 2.5D  ORANGE KEEPOUT  AMBER TRACTION:'
             'x=20:y=20:fontsize=20:fontcolor=white:'
             'box=1:boxcolor=black@0.55[wide];'
             f'[1:v]setpts={scales[1]:.9f}*PTS+'
             f'{offsets[1]:.9f}/TB,scale=426:320,'
             'drawtext=text=BASE FRONT CAMERA  SO-101 REMOVED:'
             'x=14:y=14:fontsize=18:fontcolor=white:'
             'box=1:boxcolor=black@0.55[front];'
             f'color=c=black:s=426x480:d={canvas_duration_s or 1.0:.9f}'
             '[right];'
             '[right][front]overlay=0:80:eof_action=pass:repeatlast=0'
             '[right_view];'
             '[wide][right_view]hstack=inputs=2:shortest=1,'
             'setpts=0.5*PTS[base]'),
        ])
        filter_graph = command[-1]
        current = 'base'
        for index, annotation in enumerate(annotations or []):
            target = f'ann{index}'
            safe_text = str(annotation['text']).replace("'", '')
            safe_text = safe_text.replace('%', ' percent')
            filter_graph += (
                f';[{current}]drawtext=text={safe_text}:'
                'x=(w-text_w)/2:y=h-52:fontsize=25:'
                f'fontcolor={annotation["color"]}:box=1:'
                'boxcolor=black@0.78:'
                f"enable='between(t,{float(annotation['start_s']):.3f},"
                f"{float(annotation['end_s']):.3f})'[{target}]")
            current = target
        filter_graph += f';[{current}]fps={fps:g}[out]'
        command[-1] = filter_graph
        command.extend([
            '-map', '[out]', '-shortest',
        ])
    command.extend(['-an', '-c:v', encoder])
    if encoder == 'h264_nvenc':
        command.extend(['-preset', 'p1', '-cq', '23'])
    elif encoder == 'libx264':
        command.extend(['-preset', 'ultrafast', '-crf', '20'])
    else:
        raise ValueError(f'unsupported video encoder: {encoder}')
    command.extend([
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)])
    subprocess.run(command, check=True)


def main(argv: list[str] | None = None) -> int:
    """Launch the isolated stack, run the replay, and seal its evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario-contract', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--domain-id', type=int, default=198)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--timeout-s', type=float, default=900.0)
    args = parser.parse_args(argv)
    if args.domain_id == 12:
        parser.error('physical robot domain 12 is forbidden')
    if args.output_dir.exists():
        parser.error('--output-dir must not already exist')
    if not args.scenario_contract.is_file():
        parser.error('--scenario-contract must exist')
    contract = json.loads(
        args.scenario_contract.read_text(encoding='utf-8'))
    world = Path(contract['traction_fault']['world']['path'])
    robot_urdf = Path(contract['traction_fault']['robot_urdf']['path'])
    bridge_record = contract['traction_fault'].get('guarded_bridge')
    guard_overrides = contract['traction_fault'].get('guard_overrides', {})
    bridge = Path(bridge_record['path']) if bridge_record else None
    map_record = contract.get('navigation_map', {}).get('yaml')
    navigation_map = (
        Path(map_record['path']) if map_record
        else ASSETS / 'slam_corridor_eval.yaml')
    keepout_record = contract.get('navigation_map', {}).get('keepout_yaml')
    keepout_mask = Path(keepout_record['path']) if keepout_record else None
    start_pose = contract['route'].get(
        'start_pose', {'x': -8.0, 'y': 0.0})
    camera = contract.get('simulator_camera')
    obstacle_injection = contract['obstacle_interventions'].get(
        'injection', {})
    for label, path in (
            ('traction world', world), ('guarded bridge', bridge),
            ('evaluation URDF', robot_urdf),
            ('evaluation map', navigation_map),
            ('keepout mask', keepout_mask),
            ('behavior tree', BT)):
        if ((label == 'keepout mask' and path is None)
                or path is None or not path.is_file()):
            parser.error(f'{label} does not exist: {path}')

    run_dir = args.output_dir
    run_dir.mkdir(parents=True)
    nav_inputs = prepare_navigation(
        run_dir / 'navigation_inputs', movement_time_allowance_s=25.0,
        initial_pose_xy=(float(start_pose['x']), float(start_pose['y'])),
        stop_zone_front_m=obstacle_injection.get('stop_zone_front_m'))
    scenario_path = run_dir / 'scenario_evidence.json'
    guard_path = run_dir / 'traction_guard_evidence.json'
    bag_dir = run_dir / 'bag'
    camera_views = (camera.get('views', []) if camera is not None else [])
    if camera is not None and not camera_views:
        camera_views = [{'name': 'scene', 'topic': camera['topic']}]
    camera_outputs = [{
        **view,
        'raw': run_dir / f'gazebo_{view["name"]}_raw.mp4',
        'metadata': run_dir / f'gazebo_{view["name"]}_capture.json',
        'ready': run_dir / f'.gazebo_{view["name"]}_ready',
    } for view in camera_views]
    camera_video = run_dir / 'gazebo_actual_map_2x.mp4'
    camera_encoder = select_video_encoder() if camera is not None else None
    environment = _environment(args.run_id, args.domain_id)
    processes: list[tuple[str, subprocess.Popen, Any]] = []
    failure: str | None = None
    scenario_returncode: int | None = None

    def launch(name: str, command: list[str]) -> subprocess.Popen:
        process, stream = _start(
            command, run_dir / f'{name}.log', environment)
        processes.append((name, process, stream))
        return process

    try:
        launch('gazebo', [
            'ros2', 'launch', 'jdamr_cube_gazebo', 'gazebo.launch.py',
            f'world:={world}',
            f'urdf_file:={robot_urdf}',
            f'bridge_config:={bridge}', 'gui:=false',
            'enable_image_bridges:=false',
            'enable_arm_controllers:=false', f'seed:={args.seed}',
            f'x_pose:={float(start_pose["x"])}',
            f'y_pose:={float(start_pose["y"])}', 'z_pose:=0.01',
        ])
        _wait_topics(
            {'/scan', '/odom', '/ground_truth_pose'}, environment, 60.0,
            run_dir / 'gazebo.log')
        if camera is not None:
            for view in camera_outputs:
                launch(f'camera_bridge_{view["name"]}', [
                    'ros2', 'run', 'ros_gz_image', 'image_bridge',
                    str(view['topic']).lstrip('/'),
                ])
                _wait_topics({str(view['topic'])}, environment, 30.0)
                recorder = launch(f'camera_recorder_{view["name"]}', [
                    'python3', str(CAMERA_RECORDER),
                    '--topic', str(view['topic']),
                    '--output', str(view['raw']),
                    '--metadata', str(view['metadata']),
                    '--ready-file', str(view['ready']),
                    '--fps', f'{float(camera["fps"]):g}',
                ])
                wait_file_ready(recorder, view['ready'])
        launch('navigation', [
            'ros2', 'launch', 'jdamr_cube_navigation',
            'navigation.launch.py',
            f'map:={navigation_map}',
            f'params_file:={nav_inputs["params"]}',
            'use_keepout:=true', f'keepout_mask:={keepout_mask}',
            'use_sim_time:=true',
            'use_composition:=True',
        ])
        _wait_topics({
            '/cmd_vel', '/plan', '/collision_monitor_state', '/amcl_pose',
        }, environment, 90.0, run_dir / 'navigation.log')
        _wait_lifecycle_active((
            '/amcl', '/controller_server', '/planner_server',
            '/bt_navigator', '/velocity_smoother', '/collision_monitor',
        ), environment, 90.0)
        _wait_tf_available(
            'map', 'base_footprint', environment, 60.0,
            run_dir / 'tf_readiness.log')
        launch('traction_guard', [
            'ros2', 'run', 'jdamr_cube_navigation',
            'traction_velocity_guard', '--output', str(guard_path),
            '--recovery-grace-s',
            str(guard_overrides.get('recovery_grace_s', 5.0)),
            '--anomaly-dwell-s',
            str(guard_overrides.get('anomaly_dwell_s', 0.4)),
            '--minimum-odom-progress-m',
            str(guard_overrides.get('minimum_odom_progress_m', 0.08)),
            '--ros-args', '-p', 'use_sim_time:=true',
        ])
        _wait_topics({'/guarded_cmd_vel'}, environment, 30.0)
        launch('recorder', [
            'ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_dir),
            '--topics', *RECORDED_TOPICS,
        ])
        scenario = launch('scenario', [
            'ros2', 'run', 'jdamr_cube_navigation',
            'sim_restaurant_replay_scenario',
            '--contract', str(args.scenario_contract.resolve()),
            '--behavior-tree', str(BT), '--output', str(scenario_path),
            '--obstacle-entity', str(obstacle_injection.get(
                'dynamic_entity', 'crossing_person')),
            '--obstacle-ahead-m',
            str(obstacle_injection.get('obstacle_ahead_m', 0.55)),
            '--goal-timeout-s', '120.0',
            '--ros-args', '-p', 'use_sim_time:=true',
        ])
        scenario_returncode = scenario.wait(timeout=args.timeout_s)
        if scenario_returncode != 0:
            failure = f'scenario_returncode:{scenario_returncode}'
    except Exception as error:
        failure = f'{type(error).__name__}: {error}'
    finally:
        launched_groups = [process.pid for _, process, _ in processes]
        for _, process, _ in reversed(processes):
            _stop(process)
        for _, _, stream in processes:
            stream.close()
        survivors = _cleanup_identity(args.run_id, args.domain_id)
        remaining_groups = [
            group for group in launched_groups if _group_members(group)]

    scenario_evidence = load_if_present(scenario_path)
    guard_evidence = load_if_present(guard_path)
    camera_records = [{
        **view, 'capture': load_if_present(view['metadata'])}
        for view in camera_outputs]
    camera_complete = bool(camera_records) and all(
        item['raw'].is_file() and item['capture'] is not None
        and item['capture'].get('frames', 0) > 0
        for item in camera_records)
    timing_scales: list[float] = []
    timing_offsets_s: list[float] = []
    canvas_duration_s = 0.0
    common_start_s = 0.0
    common_end_s = 0.0
    if camera is not None and camera_complete:
        (timing_scales, timing_offsets_s, canvas_duration_s,
         common_start_s) = camera_sim_timing(
            camera_records, float(camera['fps']))
        common_end_s = min(
            float(item['capture']['frame_timestamps'][-1]['sim_ns']) / 1e9
            for item in camera_records)
        canvas_duration_s = common_end_s - common_start_s
    annotations = scene_video_annotations(
        scenario_evidence, guard_evidence, common_start_s)
    if camera is not None and camera_complete:
        try:
            for item in camera_records:
                item['clock_aligned'] = (
                    run_dir / f'gazebo_{item["name"]}_clock_aligned.mp4')
                resample_camera_on_sim_clock(
                    item['raw'], item['capture'], item['clock_aligned'],
                    float(camera['fps']), common_start_s, common_end_s)
            encode_camera_video(
                [item['clock_aligned'] for item in camera_records],
                camera_video, float(camera['fps']), str(camera_encoder),
                [1.0] * len(camera_records),
                [0.0] * len(camera_records), canvas_duration_s, annotations)
        except Exception as error:
            failure = f'camera_encode_{type(error).__name__}: {error}'
    mcap = one_mcap(bag_dir)
    outcome, gate_failures = classify_result(
        scenario_evidence, guard_evidence, mcap, survivors)
    if remaining_groups:
        outcome = 'FAIL'
        gate_failures.append('launched_process_groups_not_empty')
    if failure is not None:
        outcome = 'FAIL'
        gate_failures.append('runner_failure')
    if camera is not None and not camera_video.is_file():
        outcome = 'FAIL'
        gate_failures.append('simulator_camera_video_missing')
    summary = {
        'schema_version': 1,
        'run_id': args.run_id,
        'status': outcome,
        'failure': failure,
        'gate_failures': gate_failures,
        'scenario_returncode': scenario_returncode,
        'source': {
            'scenario_contract': {
                'path': str(args.scenario_contract.resolve()),
                'sha256': _sha256(args.scenario_contract),
            },
            'world': {'path': str(world), 'sha256': _sha256(world)},
            'robot_urdf': {
                'path': str(robot_urdf), 'sha256': _sha256(robot_urdf)},
            'bridge': {'path': str(bridge), 'sha256': _sha256(bridge)},
            'navigation_map': {
                'path': str(navigation_map),
                'sha256': _sha256(navigation_map)},
        },
        'scenario': scenario_evidence,
        'traction_guard': guard_evidence,
        'bag': (
            {'path': str(mcap.resolve()), 'sha256': _sha256(mcap),
             'bytes': mcap.stat().st_size}
            if mcap is not None else None),
        'simulator_video': (
            {
                'source': 'gazebo_camera_sensor',
                'encoder': camera_encoder,
                'timing_alignment': {
                    'basis': 'per_frame_gazebo_timestamp_resampling',
                    'prior_linear_input_pts_scales': timing_scales,
                    'prior_linear_input_start_offsets_s': timing_offsets_s,
                    'common_start_sim_s': common_start_s,
                    'common_end_sim_s': common_end_s,
                },
                'annotations': annotations,
                'views': [{
                    'name': item['name'], 'topic': item['topic'],
                    'raw': {'path': str(item['raw'].resolve()),
                            'sha256': _sha256(item['raw']),
                            'bytes': item['raw'].stat().st_size},
                    'clock_aligned': {
                        'path': str(item['clock_aligned'].resolve()),
                        'sha256': _sha256(item['clock_aligned']),
                        'bytes': item['clock_aligned'].stat().st_size},
                    'capture': item['capture'],
                } for item in camera_records],
                'video_2x': {'path': str(camera_video.resolve()),
                             'sha256': _sha256(camera_video),
                             'bytes': camera_video.stat().st_size},
            }
            if camera is not None and camera_video.is_file() else None),
        'teardown': {
            'launched_process_groups': launched_groups,
            'remaining_process_groups': remaining_groups,
            'identity_survivors': survivors,
        },
    }
    summary_path = run_dir / 'summary.json'
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(json.dumps({
        'status': outcome, 'run_dir': str(run_dir.resolve()),
        'gate_failures': gate_failures,
    }, ensure_ascii=False))
    return 0 if outcome == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
