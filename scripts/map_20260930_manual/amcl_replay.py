"""Replay a recorded drive through AMCL with different parameter files (PC only).

    python3 amcl_replay.py prepare <bag> <map.yaml> <out_dir>
    python3 amcl_replay.py run <out_dir> <tag> <nav2_params.yaml> [--rate 1.0]
    python3 amcl_replay.py compare <out_dir> <tag> [<tag> ...]

prepare: writes <out_dir>/play (the bag's /scan, /tf without map->odom, /tf_static from
the first recorded AMCL pose on), the recorded pose series, and a reference pose for every
stationary window (odom moved < 5 mm and 0.5 deg for >= 2 s) from scan_match.py against the
map. The reference is an independent scan-to-map fit, not AMCL.
run: map_server + amcl + lifecycle manager in ROS domain 88, localhost only, inside a
systemd scope with memory/CPU/time limits; plays the bag with sim time, seeds AMCL with the
recorded pose at the start, and records the map->odom it publishes.
compare: pose error of the recorded (Pi) AMCL and of each replay against the references.

Inputs: any bag with /scan, /tf (odom->base and map->odom) and /tf_static, such as the PC
relay bag from jdamr_depart display-start or an onboard Nav2 bag. Nothing is sent to the
robot; the replay domain is not the robot's (12).
"""
import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
REPLAY_DOMAIN = '88'
STATIONARY_MIN_S = 2.0
STATIONARY_XY_M = 0.005
STATIONARY_YAW_RAD = math.radians(0.5)
# Same acceptance as the jdamr_depart init match (MATCH_MIN_INLIER): unmapped boxes near
# the dock put consistent matches at 0.48-0.49 of beams within 5 cm, and mean distances
# run 0.08-0.14 m there, so the mean distance is reported with every row, not gated.
REFERENCE_MIN_INLIER = 0.45
INIT_COVARIANCE = (0.0025, 0.0025, math.radians(5.0) ** 2)   # jdamr_depart init


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    x, y, t = a
    return (x + math.cos(t) * b[0] - math.sin(t) * b[1],
            y + math.sin(t) * b[0] + math.cos(t) * b[1], wrap(t + b[2]))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def latest_before(samples, t):
    """Last (stamp, ...) sample with stamp <= t, or None."""
    i = bisect.bisect_right([s[0] for s in samples], t)
    return samples[i - 1] if i else None


def nearest(samples, t):
    keys = [s[0] for s in samples]
    i = bisect.bisect_left(keys, t)
    best = [j for j in (i - 1, i) if 0 <= j < len(samples)]
    return min((samples[j] for j in best), key=lambda s: abs(s[0] - t)) if best else None


def stationary_windows(odom):
    """[(t0, t1)] where odom->base stays within 5 mm / 0.5 deg for >= 2 s."""
    windows, start = [], 0
    for i in range(1, len(odom) + 1):
        moved = i == len(odom) or (
            math.hypot(odom[i][1] - odom[start][1], odom[i][2] - odom[start][2]) > STATIONARY_XY_M
            or abs(wrap(odom[i][3] - odom[start][3])) > STATIONARY_YAW_RAD)
        if moved:
            if odom[i - 1][0] - odom[start][0] >= STATIONARY_MIN_S:
                windows.append((odom[start][0], odom[i - 1][0]))
            start = i
    return windows


# ---------------------------------------------------------------- prepare
def prepare(bag, map_yaml, out):
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    sys.path.insert(0, str(HERE))
    import scan_match

    out.mkdir(parents=True, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    storage = 'mcap' if any(p.suffix == '.mcap' for p in Path(bag).iterdir()) else 'sqlite3'
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    metas = {m.name: m for m in reader.get_all_topics_and_types()}
    for need in ('/scan', '/tf', '/tf_static'):
        if need not in metas:
            raise SystemExit(f'{bag}: no {need}')
    TFMessage = get_message('tf2_msgs/msg/TFMessage')
    LaserScan = get_message(metas['/scan'].type)
    rows, odom, map_odom, scans, static = [], [], [], [], []
    base = None
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in ('/scan', '/tf', '/tf_static'):
            continue
        if topic == '/scan':
            msg = deserialize_message(raw, LaserScan)
            scans.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg))
            rows.append((topic, raw, t_ns))
            continue
        msg = deserialize_message(raw, TFMessage)
        if topic == '/tf_static':
            static.extend(msg.transforms)
            rows.append((topic, raw, t_ns))
            continue
        kept = []
        for tf in msg.transforms:
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            tr, q = tf.transform.translation, tf.transform.rotation
            pair = (tf.header.frame_id.lstrip('/'), tf.child_frame_id.lstrip('/'))
            if pair == ('map', 'odom'):
                map_odom.append((stamp, tr.x, tr.y, yaw_of(q)))
                continue
            if pair[0] == 'odom':
                base = base or pair[1]
                odom.append((stamp, tr.x, tr.y, yaw_of(q)))
            kept.append(tf)
        if kept:
            msg.transforms = kept
            rows.append((topic, serialize_message(msg), t_ns))
    odom.sort()
    map_odom.sort()
    if not odom or not map_odom:
        raise SystemExit('bag needs odom->base and map->odom transforms')

    # Seed: the recorded AMCL estimate at its first map->odom.
    t_seed = map_odom[0][0]
    o = nearest(odom, t_seed)
    seed = compose(map_odom[0][1:], o[1:])
    t_begin_ns = int((t_seed - 0.5) * 1e9)

    writer = rosbag2_py.SequentialWriter()
    play = out / 'play'
    if play.exists():
        raise SystemExit(f'{play} exists; use a new out_dir')
    writer.open(rosbag2_py.StorageOptions(uri=str(play), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for name in ('/scan', '/tf', '/tf_static'):
        writer.create_topic(metas[name])
    written = 0
    for topic, raw, t_ns in rows:
        if topic == '/tf_static':
            writer.write(topic, raw, max(t_ns, t_begin_ns))
            written += 1
        elif t_ns >= t_begin_ns:
            writer.write(topic, raw, t_ns)
            written += 1
    del writer

    # Stationary references from scan-to-map fitting.
    static_rows = [{'child': tf.child_frame_id,
                    't': [tf.transform.translation.x, tf.transform.translation.y],
                    'q': [tf.transform.rotation.x, tf.transform.rotation.y,
                          tf.transform.rotation.z, tf.transform.rotation.w]} for tf in static]
    _img, occupied, _free, res, origin = scan_match.load_map(str(map_yaml))
    score, _dist = scan_match.make_score(occupied, res, origin)
    windows = []
    for t0, t1 in stationary_windows(odom):
        t0 = max(t0, t_seed)
        if t1 - t0 < STATIONARY_MIN_S:
            continue
        in_window = [m for s, m in scans if t0 <= s <= t1]
        if len(in_window) < 5:
            continue
        mid = 0.5 * (t0 + t1)
        rec = latest_before(map_odom, mid)
        o = nearest(odom, mid)
        recorded = compose(rec[1:], o[1:])
        scan_json = out / 'windows' / f'scan_{t0:.1f}.json'
        scan_json.parent.mkdir(exist_ok=True)
        scan_json.write_text(json.dumps({'tf_static': static_rows, 'scans': [
            {'frame': m.header.frame_id, 'angle_min': m.angle_min,
             'angle_increment': m.angle_increment, 'range_min': m.range_min,
             'range_max': m.range_max,
             'ranges': [r if math.isfinite(r) else None for r in m.ranges]}
            for m in in_window]}))
        pts = scan_match.laser_points(str(scan_json))[0]
        mean_dist, ref = scan_match.search(score, pts, recorded)
        _s, vals = score(pts, *ref)
        inlier = float((vals < 0.05).mean())
        windows.append({
            't0': t0, 't1': t1, 'scans': len(in_window), 'odom_at_mid': list(o[1:]),
            'reference': list(ref), 'mean_dist_m': mean_dist, 'inlier_5cm': inlier,
            'recorded': list(recorded)})
        print(f'window {t0 - t_seed:7.1f}-{t1 - t_seed:7.1f}s ref ({ref[0]:.3f}, {ref[1]:.3f}, '
              f'{math.degrees(ref[2]):.1f}) fit {mean_dist:.4f} m inl {inlier:.2f} | recorded '
              f'd {math.hypot(ref[0] - recorded[0], ref[1] - recorded[1]) * 100:.1f} cm '
              f'{math.degrees(wrap(recorded[2] - ref[2])):+.1f} deg', flush=True)
    summary = {
        'bag': str(bag), 'map_yaml': str(map_yaml),
        'map_sha256': {p: sha256(Path(map_yaml).parent / p) for p in
                       (Path(map_yaml).name, _map_image(map_yaml))},
        'base_frame': base, 't_seed': t_seed, 'seed': list(seed),
        'play_messages': written, 'odom': odom, 'recorded_map_odom': map_odom,
        'windows': windows}
    (out / 'prepare.json').write_text(json.dumps(summary))
    print(f'seed ({seed[0]:.3f}, {seed[1]:.3f}, {math.degrees(seed[2]):.1f} deg); '
          f'{len(windows)} stationary windows, '
          f'{sum(w["inlier_5cm"] >= REFERENCE_MIN_INLIER for w in windows)} with a reference')


def _map_image(map_yaml):
    import yaml
    return yaml.safe_load(Path(map_yaml).read_text())['image']


# ---------------------------------------------------------------- run
def run(out, tag, params, rate):
    """Re-exec inside a bounded systemd scope; the scope holds every child process."""
    unit = f'amcl-replay-{tag}-{int(time.time())}'
    cmd = ['systemd-run', '--user', '--scope', '--quiet', f'--unit={unit}',
           '-p', 'MemoryMax=3G', '-p', 'CPUQuota=300%', '-p', 'TimeoutStopSec=15',
           'timeout', '-s', 'INT', '-k', '20', '3600',
           sys.executable, str(Path(__file__).resolve()), '_run', str(out), tag,
           str(Path(params).resolve()), str(rate)]
    code = subprocess.call(cmd)
    subprocess.call(['systemctl', '--user', 'stop', f'{unit}.scope'],
                    stderr=subprocess.DEVNULL)
    return code


def _ros_env(log_dir):
    env = {k: v for k, v in os.environ.items() if k not in (
        'ROS_STATIC_PEERS', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
        'CYCLONEDDS_URI', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY')}
    env.update(ROS_DOMAIN_ID=REPLAY_DOMAIN, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_LOG_DIR=str(log_dir), RCUTILS_COLORIZED_OUTPUT='0')
    return env


def _run(out, tag, params, rate):
    prep = json.loads((out / 'prepare.json').read_text())
    run_dir = out / 'runs' / tag
    run_dir.mkdir(parents=True, exist_ok=True)
    env = _ros_env(run_dir / 'roslog')
    os.environ.update(env)
    for key in ('ROS_STATIC_PEERS', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI', 'RMW_IMPLEMENTATION',
                'ROS_LOCALHOST_ONLY'):
        os.environ.pop(key, None)
    (run_dir / 'params.yaml').write_bytes(Path(params).read_bytes())
    launch = subprocess.Popen(
        ['ros2', 'launch', str(HERE / 'amcl_replay.launch.py'), f'params:={params}',
         f'map:={prep["map_yaml"]}'],
        stdout=open(run_dir / 'launch.log', 'w'), stderr=subprocess.STDOUT, env=env)
    player = None
    try:
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from lifecycle_msgs.srv import GetState
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_system_default
        from tf2_msgs.msg import TFMessage

        rclpy.init()
        node = rclpy.create_node('amcl_replay_recorder',
                                 parameter_overrides=[Parameter(
                                     'use_sim_time', Parameter.Type.BOOL, True)])
        state = {'map_odom': [], 'odom_seen': None, 'poses': 0}

        def on_tf(msg):
            for tf in msg.transforms:
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                pair = (tf.header.frame_id.lstrip('/'), tf.child_frame_id.lstrip('/'))
                if pair == ('map', 'odom'):
                    t = tf.transform
                    state['map_odom'].append((stamp, t.translation.x, t.translation.y,
                                              yaw_of(t.rotation)))
                elif pair[0] == 'odom' and stamp >= prep['t_seed']:
                    state['odom_seen'] = state['odom_seen'] or stamp

        node.create_subscription(TFMessage, '/tf', on_tf, 200)
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 lambda _m: state.__setitem__('poses', state['poses'] + 1), 10)
        initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose',
                                        qos_profile_system_default)
        client = node.create_client(GetState, '/amcl/get_state')
        deadline = time.monotonic() + 90
        active = False
        while time.monotonic() < deadline and not active:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                active = bool(future.result()) and future.result().current_state.id == 3
            if launch.poll() is not None:
                raise RuntimeError('launch exited early; see launch.log')
        if not active:
            raise RuntimeError('amcl did not become active')
        while initial.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        player = subprocess.Popen(
            ['ros2', 'bag', 'play', str(out / 'play'), '--clock', '50', '--rate', str(rate),
             '--disable-keyboard-controls'],
            stdout=open(run_dir / 'play.log', 'w'), stderr=subprocess.STDOUT, env=env)
        sent = False
        while player.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not sent and state['odom_seen']:
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp.sec = int(prep['t_seed'])
                msg.header.stamp.nanosec = int((prep['t_seed'] % 1.0) * 1e9)
                x, y, yaw = prep['seed']
                msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
                msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
                msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
                cov = [0.0] * 36
                cov[0], cov[7], cov[35] = INIT_COVARIANCE
                msg.pose.covariance = cov
                initial.publish(msg)
                sent = True
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        result = {'tag': tag, 'params_sha256': sha256(params), 'rate': rate,
                  'initial_pose_sent': sent, 'amcl_pose_count': state['poses'],
                  'play_exit': player.returncode, 'map_odom': sorted(state['map_odom'])}
        (run_dir / 'result.json').write_text(json.dumps(result))
        print(f'{tag}: {len(state["map_odom"])} map->odom, {state["poses"]} amcl_pose, '
              f'play exit {player.returncode}', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    finally:
        for proc in (player, launch):
            if proc and proc.poll() is None:
                proc.send_signal(2)
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    proc.kill()


# ---------------------------------------------------------------- compare
def _transform_tolerance(params_yaml):
    import yaml
    doc = yaml.safe_load(Path(params_yaml).read_text())
    return float(doc['amcl']['ros__parameters'].get('transform_tolerance', 1.0))


def _pose_at(map_odom, tolerance, odom_pose, t):
    """map->base at t; AMCL stamps map->odom transform_tolerance into the future."""
    shifted = [(s - tolerance,) + tuple(v) for s, *v in map_odom]
    rec = latest_before(shifted, t)
    return compose(rec[1:], odom_pose) if rec else None


def _stats(errors):
    if not errors:
        return {}
    xy = sorted(e[0] for e in errors)
    yaw = sorted(abs(e[1]) for e in errors)
    mid = len(xy) // 2
    return {'n': len(errors), 'xy_cm_median': round(xy[mid] * 100, 1),
            'xy_cm_max': round(xy[-1] * 100, 1), 'yaw_deg_median': round(math.degrees(yaw[mid]), 2),
            'yaw_deg_max': round(math.degrees(yaw[-1]), 2),
            'over_5cm': sum(e[0] > 0.05 for e in errors),
            'over_3deg': sum(abs(e[1]) > math.radians(3.0) for e in errors)}


def compare(out, tags):
    prep = json.loads((out / 'prepare.json').read_text())
    series = {'recorded': (prep['recorded_map_odom'], None)}
    for tag in tags:
        result = json.loads((out / 'runs' / tag / 'result.json').read_text())
        series[tag] = (result['map_odom'],
                       _transform_tolerance(out / 'runs' / tag / 'params.yaml'))
    rec_tol = next((tol for _m, tol in series.values() if tol is not None), 1.0)
    rows, errors = [], {name: [] for name in series}
    for w in prep['windows']:
        if w['inlier_5cm'] < REFERENCE_MIN_INLIER:
            continue
        mid = 0.5 * (w['t0'] + w['t1'])
        ref = w['reference']
        row = {'t_s': round(w['t0'] - prep['t_seed'], 1), 'reference': [round(v, 4) for v in ref],
               'fit_mean_dist_m': round(w['mean_dist_m'], 3), 'inlier_5cm': round(w['inlier_5cm'], 2)}
        for name, (map_odom, tol) in series.items():
            pose = _pose_at(map_odom, rec_tol if tol is None else tol, w['odom_at_mid'], mid)
            if pose is None:
                continue
            err = (math.hypot(pose[0] - ref[0], pose[1] - ref[1]), wrap(pose[2] - ref[2]))
            errors[name].append(err)
            row[name] = {'xy_cm': round(err[0] * 100, 1), 'yaw_deg': round(math.degrees(err[1]), 2)}
        rows.append(row)
    summary = {'windows_trusted': len(rows),
               'windows_total': len(prep['windows']),
               'stats': {name: _stats(e) for name, e in errors.items()}, 'rows': rows}
    (out / f'compare_{"_".join(tags)}.json').write_text(json.dumps(summary, indent=1))
    names = list(series)
    lines = ['| t (s) | ' + ' | '.join(names) + ' |', '|---|' + '---|' * len(names)]
    for row in rows:
        cells = [f'{row[n]["xy_cm"]} cm {row[n]["yaw_deg"]:+}°' if n in row else '-'
                 for n in names]
        lines.append(f'| {row["t_s"]} | ' + ' | '.join(cells) + ' |')
    lines += ['', '| series | n | xy median/max (cm) | yaw median/max (deg) | >5 cm | >3° |',
              '|---|---|---|---|---|---|']
    for name in names:
        s = summary['stats'][name]
        if s:
            lines.append(f'| {name} | {s["n"]} | {s["xy_cm_median"]} / {s["xy_cm_max"]} | '
                         f'{s["yaw_deg_median"]} / {s["yaw_deg_max"]} | {s["over_5cm"]} | '
                         f'{s["over_3deg"]} |')
    text = '\n'.join(lines)
    (out / f'compare_{"_".join(tags)}.md').write_text(text + '\n')
    print(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('prepare')
    p.add_argument('bag', type=Path)
    p.add_argument('map_yaml', type=Path)
    p.add_argument('out', type=Path)
    r = sub.add_parser('run')
    r.add_argument('out', type=Path)
    r.add_argument('tag')
    r.add_argument('params', type=Path)
    r.add_argument('--rate', type=float, default=1.0)
    inner = sub.add_parser('_run')
    for name in ('out', 'tag', 'params', 'rate'):
        inner.add_argument(name)
    c = sub.add_parser('compare')
    c.add_argument('out', type=Path)
    c.add_argument('tags', nargs='+')
    args = parser.parse_args()
    if args.command == 'prepare':
        prepare(args.bag.resolve(), args.map_yaml.resolve(), args.out.resolve())
    elif args.command == 'run':
        sys.exit(run(args.out.resolve(), args.tag, args.params, args.rate))
    elif args.command == '_run':
        _run(Path(args.out), args.tag, args.params, float(args.rate))
    else:
        compare(args.out.resolve(), args.tags)


if __name__ == '__main__':
    main()
