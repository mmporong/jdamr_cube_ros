#!/usr/bin/env python3
"""Measure protective-stop timing from one recorded real navigation run."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from corridor_run_media import (  # noqa: I201
    _command_state,
    _plan_observation,
    parse_route_log,
)
from navigation_mcap_reader import read_navigation_messages  # noqa: I201
from same_goal_resume_evidence import (  # noqa: I201
    goal_status_snapshot,
    summarize_same_goal_resume,
)


STOP_ACTION = 1
STOP_POLYGON = 'StopZone'
LINEAR_STANDSTILL_MPS = 0.01
ANGULAR_STANDSTILL_RADPS = 0.02
STANDSTILL_DWELL_NS = 100_000_000
MAX_SCAN_BOUND_NS = 250_000_000
PLAN_SEARCH_NS = 5_000_000_000


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _latest_at_or_before(items: list[dict[str, Any]], stamp_ns: int):
    candidates = [item for item in items if item['stamp_ns'] <= stamp_ns]
    return candidates[-1] if candidates else None


def first_sustained_standstill(
        samples: list[dict[str, Any]], start_ns: int, end_ns: int,
        dwell_ns: int = STANDSTILL_DWELL_NS) -> int | None:
    """Return the first wheel-odom standstill held for the requested dwell."""
    window = [
        sample for sample in samples if start_ns <= sample['stamp_ns'] <= end_ns
    ]
    for index, sample in enumerate(window):
        if (sample['linear_speed_mps'] > LINEAR_STANDSTILL_MPS or
                sample['angular_speed_radps'] > ANGULAR_STANDSTILL_RADPS):
            continue
        dwell_end_ns = sample['stamp_ns'] + dwell_ns
        endpoint = next((
            later_index for later_index, later in enumerate(
                window[index:], start=index)
            if later['stamp_ns'] >= dwell_end_ns
        ), None)
        if endpoint is None:
            continue
        observed = window[index:endpoint + 1]
        if any(later['linear_speed_mps'] > LINEAR_STANDSTILL_MPS or
               later['angular_speed_radps'] > ANGULAR_STANDSTILL_RADPS
               for later in observed):
            continue
        return sample['stamp_ns']
    return None


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
    return round(ordered[position], 3)


def _summary(records: list[dict[str, Any]], field: str) -> dict[str, Any]:
    values = [float(row[field]) for row in records if row[field] is not None]
    return {
        'measured': len(values),
        'p50_ms': _percentile(values, 0.50),
        'p95_ms': _percentile(values, 0.95),
        'max_ms': round(max(values), 3) if values else None,
    }


def analyse(run_dir: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Read the MCAP once and return a conservative stop-latency report."""
    run_dir = run_dir.expanduser().resolve()
    bags = sorted(run_dir.glob('*.mcap'))
    if len(bags) != 1:
        raise ValueError(f'expected one MCAP, found {len(bags)}')
    bag = bags[0]
    route_log = run_dir.parent / f'{run_dir.name}.route.log'
    route = parse_route_log(route_log.read_text(encoding='utf-8'))
    start_ns = route['start_stamp_ns']
    end_ns = route['end_stamp_ns']

    scans = []
    collisions = []
    commands = []
    odom = []
    plans = []
    statuses = []
    pre_drive_status = None
    topics = [
        '/scan', '/collision_monitor_state', '/cmd_vel', '/odom', '/plan',
        '/navigate_to_pose/_action/status',
    ]
    for message in read_navigation_messages(bag, topics=topics):
        topic = message.channel.topic
        stamp_ns = message.log_time_ns
        if topic == '/navigate_to_pose/_action/status':
            snapshot = goal_status_snapshot(stamp_ns, message.ros_msg)
            if stamp_ns < start_ns:
                pre_drive_status = snapshot
            elif stamp_ns <= end_ns + 5_000_000_000:
                statuses.append(snapshot)
            continue
        if not start_ns <= stamp_ns <= end_ns:
            continue
        if topic == '/scan':
            scans.append({'stamp_ns': stamp_ns})
        elif topic == '/collision_monitor_state':
            state = message.ros_msg
            collisions.append((
                stamp_ns, int(state.action_type), str(state.polygon_name)))
        elif topic == '/cmd_vel':
            commands.append((stamp_ns, _command_state(message.ros_msg)))
        elif topic == '/odom':
            twist = message.ros_msg.twist.twist
            odom.append({
                'stamp_ns': stamp_ns,
                'linear_speed_mps': math.hypot(
                    float(twist.linear.x), float(twist.linear.y)),
                'angular_speed_radps': abs(float(twist.angular.z)),
            })
        elif topic == '/plan':
            plans.append(_plan_observation(stamp_ns, message.ros_msg))
    if pre_drive_status is not None:
        statuses.insert(0, pre_drive_status)

    resume = summarize_same_goal_resume(
        collisions, commands, statuses, start_ns, end_ns,
        end_ns + 5_000_000_000)
    episodes = {
        item['stop_stamp_ns']: item for item in resume['episodes']
    }
    records = []
    previous_action = None
    for stamp_ns, action, polygon in collisions:
        is_entry = (
            action == STOP_ACTION and polygon == STOP_POLYGON and
            previous_action != (action, polygon)
        )
        previous_action = (action, polygon)
        if not is_entry:
            continue
        episode = episodes.get(stamp_ns)
        clear_ns = episode.get('clear_stamp_ns') if episode else None
        zero_ns = episode.get('zero_command_stamp_ns') if episode else None
        resume_ns = episode.get('resume_command_stamp_ns') if episode else None
        last_scan = _latest_at_or_before(scans, stamp_ns)
        scan_bound_ns = (
            stamp_ns - last_scan['stamp_ns'] if last_scan is not None else None
        )
        if scan_bound_ns is not None and scan_bound_ns > MAX_SCAN_BOUND_NS:
            scan_bound_ns = None
        standstill_end_ns = clear_ns or min(end_ns, stamp_ns + PLAN_SEARCH_NS)
        standstill_ns = first_sustained_standstill(
            odom, stamp_ns, standstill_end_ns)
        baseline_plan = _latest_at_or_before(plans, stamp_ns)
        changed_plan = next((
            plan for plan in plans
            if (plan['stamp_ns'] > stamp_ns and
                plan['stamp_ns'] <= min(end_ns, stamp_ns + PLAN_SEARCH_NS) and
                baseline_plan is not None and
                plan['frame_and_geometry_sha256'] !=
                baseline_plan['frame_and_geometry_sha256'])
        ), None)
        records.append({
            'event': len(records) + 1,
            'stop_elapsed_s': round((stamp_ns - start_ns) / 1e9, 3),
            'last_scan_to_stop_interval_ms': (
                round(scan_bound_ns / 1e6, 3)
                if scan_bound_ns is not None else None),
            'stop_to_zero_command_ms': (
                round((zero_ns - stamp_ns) / 1e6, 3)
                if zero_ns is not None else None),
            'stop_to_wheel_odom_standstill_ms': (
                round((standstill_ns - stamp_ns) / 1e6, 3)
                if standstill_ns is not None else None),
            'zero_command_to_wheel_odom_standstill_ms': (
                round((standstill_ns - zero_ns) / 1e6, 3)
                if standstill_ns is not None and zero_ns is not None else None),
            'last_scan_to_wheel_odom_standstill_interval_ms': (
                round((standstill_ns - last_scan['stamp_ns']) / 1e6, 3)
                if standstill_ns is not None and scan_bound_ns is not None
                else None),
            'stop_to_clear_ms': (
                round((clear_ns - stamp_ns) / 1e6, 3)
                if clear_ns is not None else None),
            'clear_to_resume_command_ms': (
                round((resume_ns - clear_ns) / 1e6, 3)
                if resume_ns is not None and clear_ns is not None else None),
            'stop_to_first_changed_plan_ms': (
                round((changed_plan['stamp_ns'] - stamp_ns) / 1e6, 3)
                if changed_plan is not None else None),
            'same_goal_resumed': (
                episode.get('same_goal_resumed') if episode else False),
            'terminal_succeeded': (
                episode.get('terminal_succeeded') if episode else False),
        })

    report = {
        'schema_version': 1,
        'run_id': run_dir.name,
        'source': {
            'mcap': str(bag),
            'mcap_sha256': _sha256(bag),
            'route_log': str(route_log),
            'route_log_sha256': _sha256(route_log),
        },
        'time_basis': 'MCAP recorder log time',
        'event_count': len(records),
        'same_goal_resume_verdict': resume['verdict'],
        'thresholds': {
            'linear_standstill_mps': LINEAR_STANDSTILL_MPS,
            'angular_standstill_radps': ANGULAR_STANDSTILL_RADPS,
            'standstill_dwell_ms': STANDSTILL_DWELL_NS / 1e6,
            'maximum_scan_association_gap_ms': MAX_SCAN_BOUND_NS / 1e6,
        },
        'summaries': {
            field: _summary(records, field)
            for field in (
                'last_scan_to_stop_interval_ms',
                'stop_to_zero_command_ms',
                'stop_to_wheel_odom_standstill_ms',
                'zero_command_to_wheel_odom_standstill_ms',
                'last_scan_to_wheel_odom_standstill_interval_ms',
                'clear_to_resume_command_ms',
                'stop_to_first_changed_plan_ms',
            )
        },
        'limitations': [
            'Last scan to STOP is only a recorder-order interval, not lidar '
            'acquisition-to-decision latency; no causal trace ID was recorded.',
            'Wheel odometry below threshold is not external proof that the '
            'physical base stopped, especially under wheel slip.',
            'A changed /plan after STOP is temporal association and does not '
            'by itself prove that the obstacle caused the route change.',
            'Obstacle class was not measured in the MCAP.',
        ],
    }
    return report, records


def write_outputs(report: dict[str, Any], records: list[dict[str, Any]],
                  output_dir: Path) -> None:
    """Write deterministic JSON and CSV evidence files."""
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'stop_latency.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    fields = list(records[0]) if records else [
        'event', 'stop_elapsed_s', 'last_scan_to_stop_interval_ms',
        'stop_to_zero_command_ms', 'stop_to_wheel_odom_standstill_ms',
        'zero_command_to_wheel_odom_standstill_ms', 'stop_to_clear_ms',
        'last_scan_to_wheel_odom_standstill_interval_ms',
        'clear_to_resume_command_ms', 'stop_to_first_changed_plan_ms',
        'same_goal_resumed', 'terminal_succeeded',
    ]
    with (output_dir / 'stop_latency.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        writer.writerows(records)


def main(argv: list[str] | None = None) -> int:
    """Run the real-stop analysis CLI."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    report, records = analyse(args.run_dir)
    write_outputs(report, records, args.output_dir)
    print(json.dumps(report['summaries'], ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
