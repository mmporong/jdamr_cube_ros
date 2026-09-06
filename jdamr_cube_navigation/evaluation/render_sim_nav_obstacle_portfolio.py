#!/usr/bin/env python3
"""Render reproducible portfolio data and media from G003 evidence bags."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use('Agg')
import matplotlib.animation as animation  # noqa: E402, I100
import matplotlib.pyplot as plt  # noqa: E402, I100

from action_msgs.msg import GoalStatus  # noqa: E402, I100
from action_msgs.msg import GoalStatusArray  # noqa: E402, I100
from mcap.reader import make_reader  # noqa: E402, I100, I201
from mcap_ros2.reader import read_ros2_messages  # noqa: E402, I201
import numpy as np  # noqa: E402, I201
from PIL import Image  # noqa: E402, I100, I201

from rclpy.serialization import deserialize_message  # noqa: E402, I100

from tf2_msgs.msg import TFMessage  # noqa: E402, I100


SCENARIOS = (
    'baseline', 'detour', 'event_driven_removal',
    'full_block', 'goal_occupied')
EVALUATION_SEEDS = (11, 23, 42, 67, 89)
REQUIRED_TOPICS = {
    '/ground_truth_pose', '/plan', '/global_costmap/costmap_raw',
    '/local_costmap/costmap_raw', '/scan', '/cmd_vel',
    '/tf', '/navigate_to_pose/_action/status',
    '/navigate_to_pose/_action/feedback'}
BLOCKING_COST_VALUES = (253, 254)
TIMELINE_LABELS = {
    'activate_ack', 'global_marked', 'local_marked', 'deactivate_ack',
    'global_cleared', 'local_cleared', 'post_clear_new_plan',
    'succeeded', 'aborted'}
REQUIRED_DIRECT_SOURCE_PATHS = {
    'contract.json', 'assets/slam_corridor_eval.pgm'}


def expected_run_ids() -> list[str]:
    """Return the exact G003 scenario-seed run identity matrix."""
    return [f'{scenario}__seed_{seed}'
            for scenario in SCENARIOS for seed in EVALUATION_SEEDS]


def expected_source_paths() -> set[str]:
    """Return every source-evidence path required by this renderer."""
    paths = {
        'aggregate.json', 'contract.json',
        'assets/slam_corridor_eval.pgm',
        'evidence/scan_ab_preflight.json',
    }
    paths.update(f'evidence/{run_id}.json' for run_id in expected_run_ids())
    paths.update(
        f'portfolio_media/representative_evidence/{scenario}__seed_11.json'
        for scenario in SCENARIOS)
    return paths


def expected_generated_paths() -> set[str]:
    """Return the exact output files, excluding manifest.json itself."""
    paths = {
        'audit/previous_media_status.json',
        'data/causal_event_timelines.json',
        'data/representative_bag_inventory.json',
        'data/scenario_seed_metrics.csv',
        'data/scenario_seed_metrics.json',
        'visuals/scenario_outcomes_summary.gif',
    }
    for scenario in SCENARIOS:
        paths.add(f'visuals/{scenario}_seed11_overlay.png')
        paths.add(f'visuals/{scenario}_seed11_story.mp4')
    return paths


def require_exact_record_paths(
        name: str, records: list[dict[str, Any]], expected: set[str]) -> None:
    """Reject missing, duplicate, or extra manifest record paths."""
    paths = [record.get('relative_path') for record in records]
    if len(paths) != len(expected) or set(paths) != expected:
        raise ValueError(f'manifest {name} record set mismatch')


def sha256_file(path: Path) -> str:
    """Return the SHA-256 digest of one file."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def require_empty_output(path: Path) -> None:
    """Refuse to overwrite an existing portfolio package."""
    if path.exists() and any(path.iterdir()):
        raise FileExistsError(f'output is not empty: {path}')
    path.mkdir(parents=True, exist_ok=True)


def blocking_points(
        costmap: Any,
        map_from_frame: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[np.ndarray, np.ndarray]:
    """Return map-frame centers of 253/254 costmap cells."""
    data = np.frombuffer(costmap.data, dtype=np.uint8).reshape(
        costmap.metadata.size_y, costmap.metadata.size_x)
    rows, columns = np.where(np.isin(data, BLOCKING_COST_VALUES))
    resolution_m = float(costmap.metadata.resolution)
    origin = costmap.metadata.origin.position
    x_values = origin.x + (columns + 0.5) * resolution_m
    y_values = origin.y + (rows + 0.5) * resolution_m
    tx_m, ty_m, yaw_rad = map_from_frame
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return (
        tx_m + cosine * x_values - sine * y_values,
        ty_m + sine * x_values + cosine * y_values,
    )


def message_log_time_s(message: Any) -> float:
    """Convert the MCAP reader's integer log timestamp to seconds."""
    return message.log_time_ns / 1_000_000_000


def stamp_ns(message: Any) -> int:
    """Convert a ROS builtin time message to integer nanoseconds."""
    return message.sec * 1_000_000_000 + message.nanosec


def quaternion_yaw(rotation: Any) -> float:
    """Return planar yaw from one geometry quaternion."""
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))


def interpolate_map_transform(
        samples: list[tuple[int, float, float, float]],
        target_stamp_ns: int,
) -> tuple[float, float, float]:
    """Interpolate map<-frame at the requested stamp; never use latest."""
    ordered = sorted(samples)
    exact = [sample for sample in ordered if sample[0] == target_stamp_ns]
    if exact:
        return exact[-1][1:]
    before = [sample for sample in ordered if sample[0] < target_stamp_ns]
    after = [sample for sample in ordered if sample[0] > target_stamp_ns]
    if not before or not after:
        raise ValueError(f'missing exact-stamp TF: {target_stamp_ns}')
    lower = before[-1]
    upper = after[0]
    fraction = ((target_stamp_ns - lower[0])
                / (upper[0] - lower[0]))
    yaw_delta = math.atan2(
        math.sin(upper[3] - lower[3]), math.cos(upper[3] - lower[3]))
    return (
        lower[1] + fraction * (upper[1] - lower[1]),
        lower[2] + fraction * (upper[2] - lower[2]),
        lower[3] + fraction * yaw_delta,
    )


def _raw_messages(path: Path, topics: list[str], message_type: Any):
    """Decode raw CDR even when an MCAP schema definition is empty."""
    with path.open('rb') as stream:
        reader = make_reader(stream)
        for _, channel, message in reader.iter_messages(topics=topics):
            yield channel.topic, message, deserialize_message(
                message.data, message_type)


def read_map_transforms(path: Path) -> dict[str, list[tuple]]:
    """Read direct map<-child transforms from dynamic and static TF."""
    transforms: dict[str, list[tuple]] = {}
    for _, _, message in _raw_messages(path, ['/tf', '/tf_static'], TFMessage):
        for transform in message.transforms:
            if transform.header.frame_id != 'map':
                continue
            translation = transform.transform.translation
            transforms.setdefault(transform.child_frame_id, []).append((
                stamp_ns(transform.header.stamp), translation.x,
                translation.y, quaternion_yaw(transform.transform.rotation)))
    return transforms


def map_transform_for_costmap(
        costmap: Any, transforms: dict[str, list[tuple]],
) -> tuple[float, float, float]:
    """Resolve the costmap header frame in map at its exact header stamp."""
    frame = costmap.header.frame_id
    if frame == 'map':
        return 0.0, 0.0, 0.0
    if frame not in transforms:
        raise ValueError(f'missing map TF frame: {frame}')
    return interpolate_map_transform(
        transforms[frame], stamp_ns(costmap.header.stamp))


def bind_action_status(path: Path, evidence: dict[str, Any]) -> dict[str, Any]:
    """Bind one evidence run to one bag goal UUID and terminal status."""
    records = []
    for _, raw, message in _raw_messages(
            path, ['/navigate_to_pose/_action/status'], GoalStatusArray):
        for status in message.status_list:
            records.append({
                'uuid': bytes(status.goal_info.goal_id.uuid).hex(),
                'status': status.status,
                'log_time_ns': raw.log_time,
            })
    executing = [item for item in records
                 if item['status'] == GoalStatus.STATUS_EXECUTING]
    expected_terminal = {
        'succeeded': GoalStatus.STATUS_SUCCEEDED,
        'aborted': GoalStatus.STATUS_ABORTED,
    }[evidence['action_terminal']]
    candidates = []
    for start in executing:
        terminals = [item for item in records
                     if item['uuid'] == start['uuid']
                     and item['status'] == expected_terminal
                     and item['log_time_ns'] > start['log_time_ns']]
        if len(terminals) == 1:
            candidates.append((start, terminals[0]))
    if len(candidates) != 1:
        raise ValueError('bag does not contain one matching action UUID')
    start, terminal = candidates[0]
    evidence_uuid = evidence.get('goal_uuid')
    if evidence_uuid is not None and evidence_uuid != start['uuid']:
        raise ValueError('evidence and bag goal UUID mismatch')
    goal_event = [event for event in evidence['events']
                  if event['name'] == 'goal_accepted']
    terminal_event = [event for event in evidence['events']
                      if event['name'] == evidence['action_terminal']]
    if len(goal_event) != 1 or len(terminal_event) != 1:
        raise ValueError('evidence action events are not unique')
    evidence_duration_s = (
        terminal_event[0]['elapsed_s'] - goal_event[0]['elapsed_s'])
    bag_duration_s = (
        terminal['log_time_ns'] - start['log_time_ns']) / 1_000_000_000
    tolerance_s = evidence['contract']['controller_period_s']
    if abs(evidence_duration_s - bag_duration_s) > tolerance_s:
        raise ValueError('evidence and bag terminal timing mismatch')
    return {
        'goal_uuid': start['uuid'],
        'executing_log_time_ns': start['log_time_ns'],
        'terminal_log_time_ns': terminal['log_time_ns'],
        'terminal_status': terminal['status'],
        'evidence_goal_accepted_elapsed_s': goal_event[0]['elapsed_s'],
        'evidence_terminal_elapsed_s': terminal_event[0]['elapsed_s'],
        'bag_action_duration_s': bag_duration_s,
        'evidence_action_duration_s': evidence_duration_s,
        'duration_error_s': bag_duration_s - evidence_duration_s,
    }


def evidence_elapsed_at_bag_time(
        binding: dict[str, Any], bag_time_s: float) -> float:
    """Map an MCAP log time onto the evidence monotonic elapsed axis."""
    executing_s = binding['executing_log_time_ns'] / 1_000_000_000
    return (binding['evidence_goal_accepted_elapsed_s']
            + bag_time_s - executing_s)


def obstacle_is_active(
        scenario: str, events: list[dict[str, Any]], elapsed_s: float) -> bool:
    """Return whether the scenario obstacle exists at a timeline instant."""
    if scenario != 'event_driven_removal':
        return scenario != 'baseline'
    deactivation = next(
        (event['elapsed_s'] for event in events
         if event['name'] == 'deactivate_ack'), math.inf)
    return elapsed_s < deactivation


def validate_representative_identity(
        scenario: str, evidence_path: Path, bag_path: Path,
        evidence: dict[str, Any]) -> None:
    """Bind filenames and document identity to one seed-11 scenario."""
    expected_run_id = f'{scenario}__seed_11'
    expected_bag_prefix = f'{scenario}_seed11'
    if (evidence_path.name != f'{expected_run_id}.json'
            or (evidence.get('run_id'), evidence.get('scenario'),
                evidence.get('seed')) != (expected_run_id, scenario, 11)
            or not bag_path.parent.name.startswith(expected_bag_prefix)):
        raise ValueError(
            f'representative evidence identity mismatch: {scenario}')


def bag_inventory(
        path: Path, scenario: str, input_root: Path) -> dict[str, Any]:
    """Read message counts and schemas from a finalized MCAP."""
    with path.open('rb') as stream:
        summary = make_reader(stream).get_summary()
    counts = summary.statistics.channel_message_counts
    topics = []
    for channel_id, channel in sorted(
            summary.channels.items(), key=lambda item: item[1].topic):
        schema = summary.schemas.get(channel.schema_id)
        topics.append({
            'topic': channel.topic,
            'type': schema.name if schema else None,
            'message_count': counts.get(channel_id, 0),
        })
    observed = {item['topic'] for item in topics if item['message_count'] > 0}
    return {
        'scenario': scenario,
        'relative_path': str(path.relative_to(input_root)),
        'size_bytes': path.stat().st_size,
        'sha256': sha256_file(path),
        'message_count': summary.statistics.message_count,
        'topics': topics,
        'required_topics_complete': REQUIRED_TOPICS <= observed,
        'missing_required_topics': sorted(REQUIRED_TOPICS - observed),
        'requested_action_result_service_message_count': 0,
        'action_result_source': 'representative_evidence.action_terminal',
    }


def _evidence_rows(
        input_root: Path,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    rows = []
    timelines = []
    for path in sorted((input_root / 'evidence').glob('*__seed_*.json')):
        document = json.loads(path.read_text())
        rows.append({
            'run_id': document['run_id'],
            'scenario': document['scenario'],
            'seed': document['seed'],
            'action_terminal': document['action_terminal'],
            'action_terminal_elapsed_s': document[
                'action_terminal_elapsed_s'],
            'minimum_clearance_m': document[
                'minimum_robot_obstacle_aabb_distance_m'],
            'plan_message_count': document['plan_message_count'],
            'scan_message_count': document['scan_message_count'],
            'global_blocking': document['global_blocking'],
            'local_blocking': document['local_blocking'],
            'global_cleared': document['global_cleared'],
            'local_cleared': document['local_cleared'],
            'clear_mode': document['clear_mode'],
            'contact_count': document['contact_count'],
            'final_cmd_vel_zero': document['final_cmd_vel_zero'],
            'final_zero_hold_s': document['final_zero_hold_s'],
            'final_pose_x_m': document['final_robot_pose_xy_yaw'][0],
            'final_pose_y_m': document['final_robot_pose_xy_yaw'][1],
            'survivor_count': len(document['teardown'][
                'identity_survivors']),
        })
        timelines.append({
            'run_id': document['run_id'],
            'scenario': document['scenario'],
            'seed': document['seed'],
            'events': document['events'],
        })
    if len(rows) != 25:
        raise ValueError(
            f'exactly 25 run evidence files required: {len(rows)}')
    return rows, timelines


def write_tables(input_root: Path, output: Path) -> None:
    """Write scenario-seed metrics and causal timelines."""
    rows, timelines = _evidence_rows(input_root)
    data = output / 'data'
    data.mkdir()
    (data / 'scenario_seed_metrics.json').write_text(
        json.dumps(rows, indent=2, sort_keys=True) + '\n')
    with (data / 'scenario_seed_metrics.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (data / 'causal_event_timelines.json').write_text(
        json.dumps(timelines, indent=2, sort_keys=True) + '\n')


def write_media_lineage(output: Path) -> None:
    """Mark earlier renderers as audit-only without mutating their bytes."""
    audit = output / 'audit'
    audit.mkdir()
    (audit / 'previous_media_status.json').write_text(json.dumps({
        'recommended_package': 'this_package',
        'audit_only_packages': [
            {
                'relative_path': '../portfolio_media',
                'reason': 'local_costmap_without_exact_stamp_map_tf',
            },
            {
                'relative_path': '../portfolio_media_repro_v01',
                'reason': 'normalized_event_time_and_unbound_action_uuid',
            },
        ],
    }, indent=2, sort_keys=True) + '\n')


def _read_story(path: Path, binding: dict[str, Any]) -> dict[str, Any]:
    topics = [
        '/ground_truth_pose', '/plan', '/global_costmap/costmap_raw',
        '/local_costmap/costmap_raw', '/cmd_vel']
    story = {'gt': [], 'plans': [], 'global': [], 'local': [], 'cmd': []}
    transforms = read_map_transforms(path)
    local_frames = set()
    for item in read_ros2_messages(path, topics=topics):
        timestamp_s = message_log_time_s(item)
        message = item.ros_msg
        if (item.channel.topic.endswith('/costmap_raw')
                and item.log_time_ns < binding['executing_log_time_ns']):
            continue
        if item.channel.topic == '/ground_truth_pose':
            story['gt'].append((timestamp_s, message.pose.position.x,
                                message.pose.position.y))
        elif item.channel.topic == '/plan':
            story['plans'].append((timestamp_s, [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in message.poses]))
        elif item.channel.topic == '/global_costmap/costmap_raw':
            story['global'].append((
                timestamp_s,
                blocking_points(
                    message, map_transform_for_costmap(message, transforms))))
        elif item.channel.topic == '/local_costmap/costmap_raw':
            local_frames.add(message.header.frame_id)
            story['local'].append((
                timestamp_s,
                blocking_points(
                    message, map_transform_for_costmap(message, transforms))))
        elif item.channel.topic == '/cmd_vel':
            story['cmd'].append((timestamp_s, math.hypot(
                message.linear.x, message.linear.y), message.angular.z))
    if any(not story[key] for key in ('gt', 'plans', 'global', 'local')):
        raise ValueError('representative MCAP lacks a required media topic')
    moving = [sample[0] for sample in story['cmd']
              if sample[1] > 0.01 or abs(sample[2]) > 0.01]
    story['start_s'] = min(moving) - 2.0
    story['end_s'] = max(moving) + 3.0
    story['transform_evidence'] = {
        'policy': 'exact_header_stamp_interpolation_no_latest_fallback',
        'local_costmap_count': len(story['local']),
        'local_costmap_frames': sorted(local_frames),
        'map_tf_child_frames': sorted(transforms),
    }
    return story


def _latest(samples: list[Any], timestamp_s: float) -> Any:
    candidates = [sample for sample in samples if sample[0] <= timestamp_s]
    return candidates[-1] if candidates else samples[0]


def render_story(
        scenario: str, bag: Path, event_evidence: Path,
        map_image_path: Path, contract: dict[str, Any], output: Path,
        binding: dict[str, Any]) -> dict[str, Any]:
    """Render one 720p temporal path/costmap/event story."""
    story = _read_story(bag, binding)
    document = json.loads(event_evidence.read_text())
    events = document['events']
    times = np.linspace(story['start_s'], story['end_s'], 56)
    map_image = np.asarray(Image.open(map_image_path))
    figure, (axes, timeline) = plt.subplots(
        2, 1, figsize=(12.8, 7.2), dpi=100,
        gridspec_kw={'height_ratios': [5, 1]})
    figure.patch.set_facecolor('#0f172a')
    obstacle = contract['scenarios'][scenario]['obstacle']

    def draw(frame: int) -> None:
        timestamp_s = times[frame]
        axes.clear()
        timeline.clear()
        axes.imshow(map_image, cmap='gray', origin='lower',
                    extent=(-11.0, 11.0, -2.2, 2.2), alpha=0.88)
        trajectory = [sample for sample in story['gt']
                      if story['start_s'] <= sample[0] <= timestamp_s]
        if trajectory:
            axes.plot([item[1] for item in trajectory],
                      [item[2] for item in trajectory], color='#38bdf8',
                      linewidth=3, label='ground truth')
        plan = _latest(story['plans'], timestamp_s)[1]
        axes.plot([point[0] for point in plan], [point[1] for point in plan],
                  color='#facc15', linewidth=2, label='Nav2 global plan')
        for key, color, label in (
                ('global', '#fb7185', 'global blocking cells'),
                ('local', '#c084fc', 'local blocking cells')):
            x_values, y_values = _latest(story[key], timestamp_s)[1]
            axes.scatter(x_values, y_values, s=4, color=color, alpha=0.4,
                         label=label)
        terminal_s = max(event['elapsed_s'] for event in events)
        progress_s = evidence_elapsed_at_bag_time(binding, timestamp_s)
        if (obstacle is not None
                and obstacle_is_active(scenario, events, progress_s)):
            from matplotlib.patches import Rectangle
            axes.add_patch(Rectangle(
                (obstacle['x_m'] - obstacle['length_m'] / 2,
                 obstacle['y_m'] - obstacle['width_m'] / 2),
                obstacle['length_m'], obstacle['width_m'],
                color='#ef4444', alpha=0.65, label='obstacle'))
        axes.set(xlim=(-9, 7.2), ylim=(-1.45, 1.45),
                 title=scenario.replace('_', ' ').title())
        axes.set_aspect('equal')
        axes.legend(loc='upper center', ncol=3, fontsize=8)
        timeline.hlines(0, 0, terminal_s, color='#64748b', linewidth=3)
        labelled = set()
        for event in events:
            color = (
                '#22c55e'
                if event['elapsed_s'] <= progress_s else '#475569')
            timeline.scatter(event['elapsed_s'], 0, color=color, s=24)
            name = event['name']
            if name in TIMELINE_LABELS and name not in labelled:
                timeline.text(
                    event['elapsed_s'], 0.08, name.replace('_', '\n'),
                    rotation=55, fontsize=6, color='#e2e8f0')
                labelled.add(name)
        timeline.axvline(progress_s, color='#38bdf8', linewidth=2)
        timeline.set(xlim=(0, terminal_s), ylim=(-0.35, 0.5), yticks=[],
                     xlabel='causal event elapsed time (s)')
        figure.tight_layout()

    movie = animation.FuncAnimation(
        figure, draw, frames=len(times), interval=125)
    movie.save(output / f'{scenario}_seed11_story.mp4',
               writer=animation.FFMpegWriter(
                   fps=8, bitrate=1800, codec='libx264'))
    draw(len(times) - 1)
    figure.savefig(output / f'{scenario}_seed11_overlay.png', dpi=100)
    plt.close(figure)
    return story['transform_evidence']


def render_summary_gif(output: Path) -> None:
    """Create a compact five-scenario outcome GIF from final overlays."""
    frames = [Image.open(output / f'{scenario}_seed11_overlay.png')
              .convert('RGB').resize((960, 540)) for scenario in SCENARIOS]
    path = output / 'scenario_outcomes_summary.gif'
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=1400, loop=0, optimize=True)
    for frame in frames:
        frame.close()
    if path.stat().st_size > 12 * 1024 * 1024:
        raise ValueError('summary GIF exceeds 12 MiB')


def write_manifest(
        input_root: Path, output: Path,
        inventories: list[dict[str, Any]],
        representative_evidence: list[Path]) -> Path:
    """Hash every generated file and all authoritative input evidence."""
    generated = []
    for path in sorted(item for item in output.rglob('*') if item.is_file()):
        if path.name == 'manifest.json':
            continue
        generated.append({
            'relative_path': str(path.relative_to(output)),
            'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path),
        })
    source = []
    for path in [
            input_root / 'aggregate.json',
            input_root / 'contract.json',
            input_root / 'assets/slam_corridor_eval.pgm',
            *sorted(
            (input_root / 'evidence').glob('*.json')),
            *representative_evidence]:
        source.append({
            'relative_path': str(path.relative_to(input_root)),
            'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path),
        })
    manifest = output / 'manifest.json'
    manifest.write_text(json.dumps({
        'schema_version': 1,
        'generator': {
            'path': str(Path(__file__).resolve()),
            'sha256': sha256_file(Path(__file__).resolve()),
        },
        'claim_scope': (
            'Gazebo simulation, idealized 2D LiDAR, fixed obstacles only; '
            'no moving-person or real-robot safety claim.'),
        'aggregate_sha256': sha256_file(input_root / 'aggregate.json'),
        'bag_inventories': inventories,
        'source_evidence': source,
        'generated_files': generated,
    }, indent=2, sort_keys=True) + '\n')
    return manifest


def verify_manifest(input_root: Path, output: Path) -> None:
    """Fail closed when any source or generated artifact changed."""
    document = json.loads((output / 'manifest.json').read_text())
    generator_sha256 = sha256_file(Path(__file__).resolve())
    if generator_sha256 != document['generator']['sha256']:
        raise ValueError('manifest mismatch: generator source')
    require_exact_record_paths(
        'source_evidence', document['source_evidence'],
        expected_source_paths())
    require_exact_record_paths(
        'generated_files', document['generated_files'],
        expected_generated_paths())
    actual_generated = {
        str(path.relative_to(output))
        for path in output.rglob('*') if path.is_file()
        and path.name != 'manifest.json'}
    if actual_generated != expected_generated_paths():
        raise ValueError('generated output file set mismatch')
    inventories = document['bag_inventories']
    if (len(inventories) != len(SCENARIOS)
            or {item.get('scenario') for item in inventories}
            != set(SCENARIOS)):
        raise ValueError('manifest bag inventory set mismatch')
    for inventory in inventories:
        scenario = inventory['scenario']
        prefix = f'portfolio_media/bags/{scenario}_seed11'
        expected_run_id = f'{scenario}__seed_11'
        if (inventory.get('seed') != 11
                or inventory.get('run_id') != expected_run_id
                or not inventory.get('relative_path', '').startswith(prefix)
                or inventory.get('evidence_relative_path') != (
                    'portfolio_media/representative_evidence/'
                    f'{expected_run_id}.json')):
            raise ValueError('manifest bag inventory identity mismatch')
    bag_paths = [item['relative_path'] for item in inventories]
    if len(set(bag_paths)) != len(SCENARIOS):
        raise ValueError('manifest bag inventory path multiplicity mismatch')
    for group, root in (
            ('source_evidence', input_root), ('generated_files', output)):
        for record in document[group]:
            path = root / record['relative_path']
            if (not path.is_file()
                    or path.stat().st_size != record['size_bytes']
                    or sha256_file(path) != record['sha256']):
                raise ValueError(f'manifest mismatch: {path}')
    for record in document['bag_inventories']:
        path = input_root / record['relative_path']
        if (not path.is_file() or path.stat().st_size != record['size_bytes']
                or sha256_file(path) != record['sha256']):
            raise ValueError(f'manifest mismatch: {path}')


def main() -> int:
    """Generate or verify one immutable portfolio media package."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-root', type=Path, required=True)
    parser.add_argument('--representative-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--verify-existing', action='store_true')
    args = parser.parse_args()
    if args.verify_existing:
        verify_manifest(args.input_root, args.output_dir)
        return 0
    require_empty_output(args.output_dir)
    write_tables(args.input_root, args.output_dir)
    write_media_lineage(args.output_dir)
    visuals = args.output_dir / 'visuals'
    visuals.mkdir()
    contract = json.loads((args.input_root / 'contract.json').read_text())
    inventories = []
    representative_evidence_paths = []
    for scenario in SCENARIOS:
        bag_paths = list((args.representative_root / 'bags').glob(
            f'{scenario}_seed11*/*.mcap'))
        if len(bag_paths) != 1:
            raise ValueError(f'exactly one representative bag: {scenario}')
        representative_evidence = (
            args.representative_root / 'representative_evidence'
            / f'{scenario}__seed_11.json')
        if not representative_evidence.is_file():
            raise ValueError(
                f'missing representative evidence: {representative_evidence}')
        representative_evidence_paths.append(representative_evidence)
        evidence = json.loads(representative_evidence.read_text())
        validate_representative_identity(
            scenario, representative_evidence, bag_paths[0], evidence)
        binding = bind_action_status(bag_paths[0], evidence)
        inventory = bag_inventory(
            bag_paths[0], scenario, args.input_root)
        if not inventory['required_topics_complete']:
            raise ValueError(f'missing topics: {scenario}')
        inventory.update({
            'run_id': evidence['run_id'],
            'seed': evidence['seed'],
            'evidence_relative_path': str(
                representative_evidence.relative_to(args.input_root)),
            'evidence_sha256': sha256_file(representative_evidence),
            'action_binding': binding,
        })
        transform_evidence = render_story(
            scenario, bag_paths[0], representative_evidence,
            args.input_root / 'assets/slam_corridor_eval.pgm',
            contract, visuals, binding)
        inventory['transform_evidence'] = transform_evidence
        inventories.append(inventory)
    render_summary_gif(visuals)
    (args.output_dir / 'data/representative_bag_inventory.json').write_text(
        json.dumps(inventories, indent=2, sort_keys=True) + '\n')
    manifest = write_manifest(
        args.input_root, args.output_dir, inventories,
        representative_evidence_paths)
    verify_manifest(args.input_root, args.output_dir)
    print(json.dumps({'status': 'PASS', 'manifest': str(manifest),
                      'sha256': sha256_file(manifest)}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
