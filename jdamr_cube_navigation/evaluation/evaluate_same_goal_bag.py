#!/usr/bin/env python3
"""Extract same-goal command-resume evidence without a route log or ROS graph."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from corridor_run_media import _command_state
from navigation_mcap_reader import read_navigation_messages
from same_goal_resume_evidence import (
    goal_status_snapshot, summarize_same_goal_resume)


TOPICS = ('/navigate_to_pose/_action/status', '/collision_monitor_state',
          '/cmd_vel')


def evaluate(mcap: Path) -> dict:
    """Read one existing recording and preserve its source identity."""
    collisions, commands, snapshots, stamps = [], [], [], []
    for message in read_navigation_messages(mcap, topics=list(TOPICS)):
        stamp_ns = message.log_time_ns
        stamps.append(stamp_ns)
        if message.channel.topic == TOPICS[0]:
            snapshots.append(goal_status_snapshot(stamp_ns, message.ros_msg))
        elif message.channel.topic == TOPICS[1]:
            state = message.ros_msg
            collisions.append((stamp_ns, int(state.action_type),
                               str(state.polygon_name)))
        else:
            commands.append((stamp_ns, _command_state(message.ros_msg)))
    start_ns, end_ns = (min(stamps), max(stamps)) if stamps else (0, 0)
    with mcap.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    return {
        'schema_version': 1,
        'source': {'mcap': str(mcap.resolve()), 'sha256': digest,
                   'size_bytes': mcap.stat().st_size},
        'time_basis': 'recorder_log_time_ns',
        'window': 'entire_recording',
        'claim_scope': 'COMMAND_SPACE_ONLY_NOT_PHYSICAL_STOP_OR_SAFETY',
        'counts': {'action_status': len(snapshots),
                   'collision_monitor_state': len(collisions),
                   'cmd_vel': len(commands)},
        'evidence': summarize_same_goal_resume(
            collisions, commands, snapshots, start_ns, end_ns, end_ns),
    }


def main(argv=None) -> int:
    """Write a new compact evidence file, never overwrite a previous result."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mcap', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if not args.mcap.is_file():
        parser.error('--mcap must name an existing recording')
    if args.output.exists():
        parser.error('--output already exists')
    result = evaluate(args.mcap)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({
        'output': str(args.output.resolve()),
        'verdict': result['evidence']['verdict'],
        'scope': result['claim_scope'],
        'episodes': len(result['evidence']['episodes']),
        'counts': result['counts'],
    }, ensure_ascii=False))
    # An unconfirmed episode is a data verdict, not a tool execution error.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
