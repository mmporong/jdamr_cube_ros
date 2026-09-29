"""정지 상태 센서 지연을 읽기 전용으로 측정한다."""
import argparse
from bisect import bisect_left
from collections import defaultdict, deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time


def percentile(values, fraction):
    """Return the nearest-rank percentile without interpolation."""
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def summarize(samples):
    """수신 간격과 header age를 분리해 요약한다."""
    if not samples:
        return {'count': 0}
    received = [sample['recv_offset_s'] for sample in samples]
    ages = [sample['age_s'] for sample in samples]
    gaps = [b - a for a, b in zip(received, received[1:])]
    span = received[-1] - received[0]
    return {
        'count': len(samples),
        'hz': (len(samples) - 1) / span if span > 0 else None,
        'age_min_s': min(ages),
        'age_p50_s': percentile(ages, .50),
        'age_p95_s': percentile(ages, .95),
        'age_max_s': max(ages),
        'gap_p50_s': percentile(gaps, .50),
        'gap_p95_s': percentile(gaps, .95),
        'gap_max_s': max(gaps, default=None),
        'gap_over_0_2_s': sum(gap > .2 for gap in gaps),
        'gap_over_0_5_s': sum(gap > .5 for gap in gaps),
        'gap_over_1_0_s': sum(gap > 1.0 for gap in gaps),
    }


def stream_report(samples, received_count):
    """Describe retained raw evidence and any bounded-buffer truncation."""
    values = list(samples)
    return {
        'summary': summarize(values),
        'received_count': received_count,
        'retained_count': len(values),
        'dropped_count': received_count - len(values),
        'retained_window_s': (
            values[-1]['recv_offset_s'] - values[0]['recv_offset_s']
            if len(values) > 1 else None),
        'raw_samples': values,
    }


def system_sample(offset_s, sysfs_root=Path('/sys/devices/system/cpu'),
                  previous_cpu_ticks=None,
                  proc_stat_path=Path('/proc/stat')):
    """Read load and CPU frequencies; unavailable sysfs values stay null."""
    frequencies = []
    try:
        for path in sysfs_root.glob('cpu[0-9]*/cpufreq/scaling_cur_freq'):
            frequencies.append(int(path.read_text(encoding='utf-8').strip()))
    except (OSError, ValueError):
        frequencies = []
    try:
        load_1m, load_5m, load_15m = Path('/proc/loadavg').read_text(
            encoding='utf-8').split()[:3]
        loads = [float(load_1m), float(load_5m), float(load_15m)]
    except (OSError, ValueError):
        loads = [None, None, None]
    total_ticks = idle_ticks = None
    try:
        fields = proc_stat_path.read_text(encoding='utf-8').splitlines()[
            0].split()[1:]
        ticks = [int(value) for value in fields]
        total_ticks = sum(ticks)
        idle_ticks = ticks[3] + (ticks[4] if len(ticks) > 4 else 0)
    except (OSError, ValueError, IndexError):
        pass
    used_percent = None
    if previous_cpu_ticks is not None and total_ticks is not None:
        total_delta = total_ticks - previous_cpu_ticks[0]
        idle_delta = idle_ticks - previous_cpu_ticks[1]
        if total_delta > 0:
            used_percent = 100.0 * (total_delta - idle_delta) / total_delta
    return {
        'recv_offset_s': offset_s,
        'cpu_frequency_khz': frequencies or None,
        'load_1m': loads[0],
        'load_5m': loads[1],
        'load_15m': loads[2],
        'cpu_total_ticks': total_ticks,
        'cpu_idle_ticks': idle_ticks,
        'cpu_used_percent': used_percent,
    }


def stationary_report(odom_samples, max_speed, max_yaw_rate, max_displacement):
    """Allow bounded odometry noise while rejecting actual base motion."""
    thresholds = {
        'max_speed_mps': max_speed,
        'max_yaw_rate_rps': max_yaw_rate,
        'max_displacement_m': max_displacement,
    }
    if not odom_samples:
        return {'valid_stationary': False, 'reason': 'no_odom_samples',
                'thresholds': thresholds}
    origin = odom_samples[0]
    displacement = max(math.hypot(
        sample['x_m'] - origin['x_m'], sample['y_m'] - origin['y_m'])
                       for sample in odom_samples)
    observed_speed = max(abs(sample['speed_mps']) for sample in odom_samples)
    observed_yaw_rate = max(
        abs(sample['yaw_rate_rps']) for sample in odom_samples)
    return {
        'valid_stationary': (
            observed_speed <= max_speed
            and observed_yaw_rate <= max_yaw_rate
            and displacement <= max_displacement),
        'noise_tolerance_note': (
            '임계값 이하 odometry 속도·변위는 정지 측정 노이즈로 허용'),
        'thresholds': thresholds,
        'observed': {
            'max_speed_mps': observed_speed,
            'max_yaw_rate_rps': observed_yaw_rate,
            'max_displacement_m': displacement,
        },
    }


def alignment_report(scan_stamps, transform_samples, tolerance_s,
                     epsilon_s, raw_limit):
    """Compare scan stamps with map->odom stamp minus transform tolerance."""
    scans = sorted(scan_stamps)
    comparisons = deque(maxlen=raw_limit)
    if scans:
        for sample in transform_samples:
            adjusted = sample['header_stamp_s'] - tolerance_s
            index = bisect_left(scans, adjusted)
            candidates = scans[max(0, index - 1):index + 1]
            if not candidates:
                continue
            nearest = min(candidates, key=lambda stamp: abs(stamp - adjusted))
            delta = adjusted - nearest
            comparisons.append({
                'map_odom_header_stamp_s': sample['header_stamp_s'],
                'adjusted_stamp_s': adjusted,
                'nearest_scan_stamp_s': nearest,
                'delta_s': delta,
                'matched': abs(delta) <= epsilon_s,
            })
    values = list(comparisons)
    deltas = [abs(item['delta_s']) for item in values]
    return {
        'transform_tolerance_s': tolerance_s,
        'match_epsilon_s': epsilon_s,
        'count': len(values),
        'matched_count': sum(item['matched'] for item in values),
        'abs_delta_p50_s': percentile(deltas, .50),
        'abs_delta_p95_s': percentile(deltas, .95),
        'abs_delta_max_s': max(deltas, default=None),
        'raw_samples': values,
    }


def stamp_seconds(stamp):
    """Convert a ROS stamp to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def utc_now():
    """Return an explicit UTC timestamp."""
    return datetime.now(timezone.utc).isoformat()


def parse_args():
    """Parse bounded measurement-only options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=15)
    parser.add_argument('--warmup-s', type=float, default=3)
    parser.add_argument('--output', type=Path,
                        help='새 JSON 기록 경로; 기존 파일은 덮어쓰지 않는다')
    parser.add_argument('--label', default='unlabeled')
    parser.add_argument('--max-raw-samples', type=int, default=8192)
    parser.add_argument('--map-odom-transform-tolerance-s', type=float,
                        default=1.0)
    parser.add_argument('--stamp-match-epsilon-s', type=float, default=.05)
    parser.add_argument('--stationary-max-speed-mps', type=float, default=.02)
    parser.add_argument(
        '--stationary-max-yaw-rate-rps', type=float, default=.05)
    parser.add_argument('--stationary-max-displacement-m', type=float,
                        default=.03)
    args = parser.parse_args()
    bounded = (
        math.isfinite(args.duration_s) and 0 < args.duration_s <= 60
        and math.isfinite(args.warmup_s) and 0 <= args.warmup_s <= 15
        and 1 <= args.max_raw_samples <= 20000
        and math.isfinite(args.map_odom_transform_tolerance_s)
        and 0 <= args.map_odom_transform_tolerance_s <= 5
        and math.isfinite(args.stamp_match_epsilon_s)
        and 0 <= args.stamp_match_epsilon_s <= 1
        and math.isfinite(args.stationary_max_speed_mps)
        and 0 <= args.stationary_max_speed_mps <= 1
        and math.isfinite(args.stationary_max_yaw_rate_rps)
        and 0 <= args.stationary_max_yaw_rate_rps <= 2
        and math.isfinite(args.stationary_max_displacement_m)
        and 0 <= args.stationary_max_displacement_m <= 1)
    if not bounded:
        parser.error(
            'duration, warmup, sample limits and tolerances must be bounded')
    return args


def amcl_pose_record(message):
    """Serialize the last latched AMCL prior without asserting freshness."""
    pose = message.pose.pose
    return {
        'header_stamp_s': stamp_seconds(message.header.stamp),
        'frame_id': message.header.frame_id,
        'position': {'x': pose.position.x, 'y': pose.position.y,
                     'z': pose.position.z},
        'orientation': {'x': pose.orientation.x, 'y': pose.orientation.y,
                        'z': pose.orientation.z, 'w': pose.orientation.w},
        'covariance': list(message.pose.covariance),
        'purpose': 'performance-only restart prior; not localization proof',
    }


def main():
    """Collect bounded read-only ROS and host timing evidence."""
    args = parse_args()
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import (DurabilityPolicy, QoSProfile,
                           ReliabilityPolicy)
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node('jdamr_readonly_latency_probe')
    best_effort = QoSProfile(
        depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(
        depth=100, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    samples = defaultdict(lambda: deque(maxlen=args.max_raw_samples))
    seen_counts = defaultdict(int)
    scan_stamps = deque(maxlen=args.max_raw_samples)
    odom_motion = deque(maxlen=args.max_raw_samples)
    nonzero_commands = deque(maxlen=args.max_raw_samples)
    system_samples = []
    last_amcl_pose = [None]
    warmup_started_at = utc_now()
    warmup_end = time.monotonic() + args.warmup_s
    measurement_start = None

    def record(name, stamp):
        received = time.monotonic()
        if measurement_start is None or received < measurement_start:
            return
        header_stamp = stamp_seconds(stamp)
        seen_counts[name] += 1
        samples[name].append({
            'recv_offset_s': received - measurement_start,
            'header_stamp_s': header_stamp,
            'age_s': node.get_clock().now().nanoseconds * 1e-9 - header_stamp,
        })

    def scan(message):
        record('scan', message.header.stamp)
        if measurement_start is not None:
            scan_stamps.append(stamp_seconds(message.header.stamp))

    def odom(message):
        record('odom', message.header.stamp)
        if measurement_start is not None:
            odom_motion.append({
                'x_m': message.pose.pose.position.x,
                'y_m': message.pose.pose.position.y,
                'speed_mps': message.twist.twist.linear.x,
                'yaw_rate_rps': message.twist.twist.angular.z,
            })

    def transforms(qos_name, message):
        for transform in message.transforms:
            edge = (transform.header.frame_id + '->'
                    + transform.child_frame_id)
            record('tf_' + qos_name + ':' + edge, transform.header.stamp)

    def commands(message):
        if measurement_start is None:
            return
        if abs(message.linear.x) > .001 or abs(message.angular.z) > .001:
            nonzero_commands.append({
                'recv_offset_s': time.monotonic() - measurement_start,
                'linear_x_mps': message.linear.x,
                'angular_z_rps': message.angular.z,
            })

    node.create_subscription(LaserScan, '/scan', scan, best_effort)
    node.create_subscription(Odometry, '/odom', odom, best_effort)
    node.create_subscription(
        TFMessage, '/tf', lambda msg: transforms('best_effort', msg),
        best_effort)
    node.create_subscription(
        TFMessage, '/tf', lambda msg: transforms('reliable', msg), reliable)
    node.create_subscription(Twist, '/cmd_vel', commands, best_effort)
    node.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose',
        lambda msg: last_amcl_pose.__setitem__(0, amcl_pose_record(msg)),
        latched)
    try:
        while time.monotonic() < warmup_end:
            rclpy.spin_once(node, timeout_sec=.02)
        measurement_started_at = utc_now()
        measurement_start = time.monotonic()
        measurement_end = measurement_start + args.duration_s
        next_system_sample = measurement_start
        previous_cpu_ticks = None
        while time.monotonic() < measurement_end:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if now >= next_system_sample:
                host_sample = system_sample(
                    now - measurement_start,
                    previous_cpu_ticks=previous_cpu_ticks)
                system_samples.append(host_sample)
                if host_sample['cpu_total_ticks'] is not None:
                    previous_cpu_ticks = (
                        host_sample['cpu_total_ticks'],
                        host_sample['cpu_idle_ticks'])
                next_system_sample += 1.0
        measurement_ended_at = utc_now()

        endpoints = {}
        topics = ('/scan', '/odom', '/tf', '/amcl_pose',
                  '/camera/color/image_raw', '/camera/depth/image_raw')
        for topic in topics:
            endpoints[topic] = {}
            queries = (('publishers', node.get_publishers_info_by_topic),
                       ('subscribers', node.get_subscriptions_info_by_topic))
            for kind, query in queries:
                endpoints[topic][kind] = [{
                    'node': item.node_namespace.rstrip('/') + '/'
                    + item.node_name,
                    'reliability': str(item.qos_profile.reliability),
                    'depth': item.qos_profile.depth,
                } for item in query(topic)]

        stream_values = {name: list(values)
                         for name, values in samples.items()}
        alignment = {}
        for qos_name in ('best_effort', 'reliable'):
            key = 'tf_' + qos_name + ':map->odom'
            alignment[qos_name] = alignment_report(
                list(scan_stamps), stream_values.get(key, []),
                args.map_odom_transform_tolerance_s,
                args.stamp_match_epsilon_s, args.max_raw_samples)
        stationary = stationary_report(
            list(odom_motion), args.stationary_max_speed_mps,
            args.stationary_max_yaw_rate_rps,
            args.stationary_max_displacement_m)
        if seen_counts['odom'] > len(odom_motion):
            stationary.update(valid_stationary=False,
                              reason='odom_motion_evidence_truncated')
        payload = {
            'label': args.label,
            'warmup_s': args.warmup_s,
            'duration_s': args.duration_s,
            'warmup_started_at': warmup_started_at,
            'measurement_started_at': measurement_started_at,
            'measurement_ended_at': measurement_ended_at,
            'streams': {
                name: stream_report(values, seen_counts[name])
                for name, values in stream_values.items()
            },
            'map_odom_scan_alignment': alignment,
            'stationary': stationary,
            'amcl_pose': last_amcl_pose[0],
            'nonzero_commands': list(nonzero_commands),
            'system_samples': system_samples,
            'endpoints': endpoints,
        }
        serialized = json.dumps(payload, ensure_ascii=False)
        if args.output is not None:
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(serialized + '\n')
        print(serialized, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
