"""Compare box-service cycles on the same terms: phase times, staging turn, docking, parks.

    python3 compare_runs.py <run_dir> [<run_dir> ...] [--json out.json]

Reads each run's cycle_events.jsonl and, when present, onboard_bag (/odom) and
run_config.json (planner settings recorded beside the run). One table row per run;
times are from the first accepted Nav2 goal. Values are robot-internal estimates.
"""
import json
import math
import sys
from pathlib import Path


def events(run):
    return [json.loads(line) for line in (run / 'cycle_events.jsonl').read_text().splitlines()
            if line.strip()]


def legs(ev, waypoint):
    """(accepted, result) wall times of every goal for one waypoint id."""
    out, start = [], None
    for e in ev:
        if e.get('waypoint_id') != waypoint:
            continue
        if e['event'] == 'accepted':
            start = e['wall_time_s']
        elif e['event'] == 'result' and start is not None:
            out.append((start, e['wall_time_s'], e.get('nav2_error_code')))
            start = None
    return out


def odom_turn(run, t0, t1):
    """Odom left/right rotation (deg) and travel (m) between two wall times."""
    bag = run / 'onboard_bag'
    if not (bag / 'metadata.yaml').exists():
        return None
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/odom']))
    rows = []
    while reader.has_next():
        _topic, raw, _t = reader.read_next()
        m = deserialize_message(raw, Odometry)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t0 <= t <= t1:
            q = m.pose.pose.orientation
            rows.append((m.pose.pose.position.x, m.pose.pose.position.y,
                         math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    left = right = travel = 0.0
    for a, b in zip(rows, rows[1:]):
        d = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        left, right = left + max(d, 0.0), right + max(-d, 0.0)
        travel += math.dist(a[:2], b[:2])
    return round(math.degrees(left)), round(math.degrees(right)), round(travel, 2)


def summary(run):
    ev = events(run)
    accepted = [e for e in ev if e['event'] == 'accepted']
    t0 = accepted[0]['wall_time_s']
    home = [e for e in ev if e['event'] == 'home_arrived']
    out = {'run': run.name, 'cycle_s': round(home[-1]['wall_time_s'] - t0, 1) if home else None,
           'home': bool(home)}
    config = run / 'run_config.json'
    if config.exists():
        out['staging_planner'] = json.loads(config.read_text()).get('staging_planner')
    out['parks'] = [(e['table_id'], round(e['estimated_front_gap_m'] * 100, 2),
                     round(math.degrees(e['estimated_face_yaw_error_rad']), 2))
                    for e in ev if e['event'] == 'box_approach_finished']
    out['face_alignment_s'] = [round(b - a, 1) for a, b, _c in legs(ev, 'face_alignment')]
    out['searches'] = sum(e['event'] == 'box_search_rotation_requested' for e in ev)
    staging = legs(ev, 'home_dock_approach')
    dock = legs(ev, 'home_dock')
    escapes = legs(ev, 'box_escape')
    if staging:
        out['staging_leg_s'] = round(staging[-1][1] - staging[-1][0], 1)
    out['staging_heading_deg'] = [round(math.degrees(e['delta_yaw_rad']), 1)
                                  for e in ev if e['event'] == 'staging_heading_measured']
    out['staging_spins_deg'] = [round(math.degrees(e['observed_yaw_rad']), 1)
                                for e in ev if e['event'] == 'staging_turn_observed']
    if staging and dock:
        out['arrival_to_dock_leg_s'] = round(dock[-1][0] - staging[-1][1], 1)
    if dock:
        out['dock_leg_s'] = round(dock[-1][1] - dock[-1][0], 1)
        out['dock_code'] = dock[-1][2]
    out['dock_heading_deg'] = [round(math.degrees(e['delta_yaw_rad']), 2)
                               for e in ev if e['event'] == 'dock_heading_measured']
    confirm = [e for e in ev if e.get('waypoint_id') == 'home_dock' and 'position_error_m' in e
               and e['event'] in ('parking_estimate_confirmed', 'parked_dwell_observation')]
    if confirm:
        out['dock_confirm'] = (round(confirm[-1]['position_error_m'] * 100, 2),
                               round(math.degrees(confirm[-1]['yaw_error_rad']), 2))
    if escapes and dock:
        last_escape = escapes[-1]
        out['escape_to_dock_leg_s'] = round(dock[-1][0] - last_escape[1], 1)
        turn = odom_turn(run, last_escape[1], dock[-1][0])
        if turn:
            out['escape_to_dock_turn_lr_deg_travel_m'] = turn
    return out


def main(argv):
    target = None
    if '--json' in argv:
        target = Path(argv[argv.index('--json') + 1])
        argv = [a for a in argv if a not in ('--json', str(target))]
    rows = [summary(Path(a)) for a in argv]
    for row in rows:
        print(json.dumps(row, ensure_ascii=False))
    if target:
        target.write_text(json.dumps(rows, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main(sys.argv[1:])
