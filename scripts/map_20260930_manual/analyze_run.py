"""Check a box-service run against the 2026-09-30 analysis, from its own records.

    python3 analyze_run.py <cycle_events.jsonl> [--bag <PC mcap bag dir>] [--json out.json]

Event part (always): phase durations and Nav2 results, final approach controller,
gap and heading, staging heading and turn, escape, dock confirmation.
Bag part (--bag, the PC relay bag from jdamr_depart display-start): per phase the
number of /plan messages (replans), odom yaw-rate direction reversals, and map->odom
jumps (AMCL corrections). Events need wall_time_s (added 2026-09-30 evening) to be
lined up with the bag; older runs get the event part only.

Baselines to compare (2026-09-30, internal estimates): staging leg 118.4 s by events
(run table_02_20260930_184711), with 113 "Passing new path" lines in the Nav2 journal over
18:48:54-18:51:01; final approach tilted 11-23 deg via planner+RPP vs 0.87 deg via odom
straight; AMCL heading off by about 15 deg near the dock.
"""
import argparse
import bisect
import json
import math
from pathlib import Path

YAW_RATE_MIN_RADPS = 0.05     # below this the base is not turning
MAP_ODOM_JUMP_M = 0.03
MAP_ODOM_JUMP_RAD = math.radians(2.0)


def load_events(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def phases(events):
    """Pair accepted/result per waypoint and keep the times both clocks give."""
    out, open_ = [], {}
    for event in events:
        kind, wp = event.get('event'), event.get('waypoint_id')
        if kind == 'accepted' and wp:
            open_[wp] = event
        elif kind == 'result' and wp in open_:
            start = open_.pop(wp)
            row = {'waypoint': wp, 'status': event.get('terminal_status_code'),
                   'nav2_error_code': event.get('nav2_error_code'),
                   'duration_s': round(event['monotonic_s'] - start['monotonic_s'], 2)}
            if 'wall_time_s' in start and 'wall_time_s' in event:
                row['wall'] = (start['wall_time_s'], event['wall_time_s'])
            out.append(row)
    return out


def pick(events, name, *keys):
    return [{k: e.get(k) for k in keys if k in e} for e in events if e.get('event') == name]


def event_summary(events):
    staging = [p for p in phases(events) if p['waypoint'] == 'home_dock_approach']
    return {
        'phases': phases(events),
        'final_approach': pick(events, 'final_approach_straight',
                               'travel_m', 'face_heading_cos', 'controller_id'),
        'box_gap': pick(events, 'box_approach_finished', 'estimated_front_gap_m',
                        'estimated_front_corner_gaps_m', 'estimated_face_yaw_error_rad')
        + pick(events, 'box_gap_not_confirmed', 'estimated_front_gap_m',
               'estimated_front_corner_gaps_m', 'estimated_face_yaw_error_rad'),
        'parking_confirmations': pick(events, 'parking_estimate_confirmed', 'waypoint_id',
                                      'position_error_m', 'yaw_error_rad'),
        'staging_heading': pick(events, 'staging_heading_measured', 'delta_yaw_rad'),
        'staging_turn': pick(events, 'staging_turn_observed', 'requested_yaw_rad',
                             'observed_yaw_rad', 'confirmed'),
        'staging_legs_s': [p['duration_s'] for p in staging],
        'escape': pick(events, 'box_escape_finished', 'face_distance_m', 'nav2_goal_reached')
        + pick(events, 'box_escape_skipped', 'face_distance_m', 'reason'),
        'dock_leg': pick(events, 'dock_leg_frozen_in_odom', 'controller_id'),
        'home_arrived': sum(1 for e in events if e.get('event') == 'home_arrived'),
        'failed': pick(events, 'failed', 'phase', 'reason'),
    }


def read_bag(bag_dir):
    """Return plan stamps, odom->base yaw samples and map->odom samples (Pi clock)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    plans, odom_yaw, map_odom = [], [], []
    while reader.has_next():
        topic, raw, _t = reader.read_next()
        if topic not in ('/plan', '/tf'):
            continue
        msg = deserialize_message(raw, get_message(types[topic]))
        if topic == '/plan':
            plans.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            continue
        for tf in msg.transforms:
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            pair = (tf.header.frame_id, tf.child_frame_id)
            if pair == ('odom', 'base_footprint'):
                odom_yaw.append((stamp, yaw_of(tf.transform.rotation)))
            elif pair == ('map', 'odom'):
                t = tf.transform.translation
                map_odom.append((stamp, t.x, t.y, yaw_of(tf.transform.rotation)))
    return sorted(plans), sorted(odom_yaw), sorted(map_odom)


def window(samples, start, end):
    keys = [s[0] if isinstance(s, tuple) else s for s in samples]
    return samples[bisect.bisect_left(keys, start):bisect.bisect_right(keys, end)]


def reversals(yaw_samples):
    """Count turning-direction changes (ignoring rates below YAW_RATE_MIN_RADPS)."""
    signs = []
    for (t0, y0), (t1, y1) in zip(yaw_samples, yaw_samples[1:]):
        if t1 - t0 <= 0:
            continue
        rate = wrap(y1 - y0) / (t1 - t0)
        if abs(rate) >= YAW_RATE_MIN_RADPS:
            sign = 1 if rate > 0 else -1
            if not signs or signs[-1] != sign:
                signs.append(sign)
    total = sum(wrap(b[1] - a[1]) for a, b in zip(yaw_samples, yaw_samples[1:]))
    return max(0, len(signs) - 1), math.degrees(total)


def jumps(map_odom):
    big, largest_m, largest_deg = 0, 0.0, 0.0
    for a, b in zip(map_odom, map_odom[1:]):
        d = math.hypot(b[1] - a[1], b[2] - a[2])
        r = abs(wrap(b[3] - a[3]))
        largest_m, largest_deg = max(largest_m, d), max(largest_deg, math.degrees(r))
        big += d >= MAP_ODOM_JUMP_M or r >= MAP_ODOM_JUMP_RAD
    return {'jumps': big, 'max_m': round(largest_m, 3), 'max_deg': round(largest_deg, 1)}


def bag_summary(summary, bag_dir):
    plans, odom_yaw, map_odom = read_bag(bag_dir)
    rows = []
    for phase in summary['phases']:
        if 'wall' not in phase:
            continue
        start, end = phase['wall']
        turns, turned_deg = reversals(window(odom_yaw, start, end))
        rows.append({'waypoint': phase['waypoint'], 'duration_s': phase['duration_s'],
                     'plans': len(window(plans, start, end)),
                     'turn_reversals': turns, 'odom_turn_deg': round(turned_deg, 1),
                     'map_odom': jumps(window(map_odom, start, end))})
    return {'per_phase': rows, 'whole_bag_map_odom': jumps(map_odom),
            'samples': {'plan': len(plans), 'odom_yaw': len(odom_yaw),
                        'map_odom': len(map_odom)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('events')
    parser.add_argument('--bag')
    parser.add_argument('--json')
    args = parser.parse_args()
    summary = event_summary(load_events(args.events))
    if args.bag:
        summary['bag'] = bag_summary(summary, args.bag)
    text = json.dumps(summary, indent=1, ensure_ascii=False, default=str)
    if args.json:
        Path(args.json).write_text(text)
    print(text)


if __name__ == '__main__':
    main()
