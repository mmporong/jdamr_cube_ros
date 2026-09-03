"""Attribute onboard soak load to individual processes."""

# The 2026-09-01 independent-process soak recorded load1 4.13 to 9.85 but only
# as a system total, so no evidence said which Nav2 process spent the CPU.
# This sampler reads /proc directly, needs no ROS dependency, and writes one
# TSV row per process per sample so the next soak can name the cost.

import argparse
import os
from pathlib import Path
import re
import sys
import time


# Every process the onboard corridor run is allowed to start.  Anything else on
# the Pi is reported under the ``other`` label so an unexpected consumer is not
# silently folded into the Nav2 total.
PROCESS_PATTERNS = (
    ('map_server', r'nav2_map_server|map_server'),
    ('amcl', r'nav2_amcl|/amcl\b'),
    ('controller_server', r'controller_server'),
    ('planner_server', r'planner_server'),
    ('velocity_smoother', r'velocity_smoother'),
    ('collision_monitor', r'collision_monitor'),
    ('bt_navigator', r'bt_navigator'),
    ('lifecycle_manager', r'lifecycle_manager'),
    ('costmap_filter_info', r'costmap_filter_info_server'),
    ('recorder', r'ros2 bag record|rosbag2_transport|_ros2_bag'),
    ('corridor_route', r'corridor_route'),
    ('bringup', r'jdamr_cube_bringup|ydlidar|rplidar|micro_ros'),
)

TSV_HEADER = (
    'sample\ttime\tlabel\tpid\tcpu_pct\tthreads\trss_mb\t'
    'load1\tload5\tload15\ttemp_c\tthrottled'
)


def classify(cmdline):
    """Return the contract label for a process command line."""
    for label, pattern in PROCESS_PATTERNS:
        if re.search(pattern, cmdline):
            return label
    return 'other'


def parse_proc_stat(text):
    """Return (utime+stime jiffies, threads) from a /proc/<pid>/stat body."""
    # The executable name may contain spaces and parentheses, so the split
    # starts after the final ')' rather than at the first space.
    tail = text[text.rindex(')') + 2:].split()
    utime, stime = int(tail[11]), int(tail[12])
    threads = int(tail[17])
    return utime + stime, threads


def parse_proc_status_rss_kb(text):
    """Return resident set size in kB from a /proc/<pid>/status body."""
    for line in text.splitlines():
        if line.startswith('VmRSS:'):
            return int(line.split()[1])
    return 0


def cpu_percent(delta_jiffies, elapsed_s, clock_ticks):
    """Convert a jiffy delta to percent of one CPU core."""
    if elapsed_s <= 0:
        return 0.0
    return 100.0 * delta_jiffies / clock_ticks / elapsed_s


def read_loadavg(text):
    """Return the three load averages from a /proc/loadavg body."""
    fields = text.split()
    return fields[0], fields[1], fields[2]


def _read(path):
    try:
        return Path(path).read_text(encoding='utf-8', errors='replace')
    except (OSError, ValueError):
        return None


def _read_temperature():
    text = _read('/sys/class/thermal/thermal_zone0/temp')
    if text is None:
        return ''
    try:
        return f'{int(text.strip()) / 1000.0:.1f}'
    except ValueError:
        return ''


def _read_throttled():
    for path in ('/sys/devices/platform/soc/soc:firmware/get_throttled',):
        text = _read(path)
        if text is not None:
            return text.strip()
    return ''


def sample_processes(clock_ticks, previous, now):
    """Return per-process rows and the jiffy snapshot for the next sample."""
    snapshot = {}
    rows = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        cmdline_raw = _read(entry / 'cmdline')
        stat_raw = _read(entry / 'stat')
        if not cmdline_raw or not stat_raw:
            continue
        cmdline = cmdline_raw.replace('\0', ' ').strip()
        if not cmdline:
            continue
        label = classify(cmdline)
        if label == 'other':
            continue
        try:
            jiffies, threads = parse_proc_stat(stat_raw)
        except (ValueError, IndexError):
            continue
        snapshot[pid] = (jiffies, now)
        if pid not in previous:
            continue
        last_jiffies, last_time = previous[pid]
        rss_kb = parse_proc_status_rss_kb(_read(entry / 'status') or '')
        rows.append({
            'label': label,
            'pid': pid,
            'cpu_pct': cpu_percent(
                jiffies - last_jiffies, now - last_time, clock_ticks),
            'threads': threads,
            'rss_mb': rss_kb / 1024.0,
        })
    return rows, snapshot


def format_row(index, timestamp, row, load, temp, throttled):
    """Render one TSV line for a sampled process."""
    fields = (
        index, timestamp, row['label'], row['pid'],
        f'{row["cpu_pct"]:.1f}', row['threads'], f'{row["rss_mb"]:.1f}',
        load[0], load[1], load[2], temp, throttled,
    )
    return '\t'.join(str(field) for field in fields)


def summarize(rows):
    """Aggregate mean CPU and peak threads per label, hottest first."""
    totals = {}
    for row in rows:
        entry = totals.setdefault(
            row['label'], {'cpu_sum': 0.0, 'samples': 0, 'threads': 0})
        entry['cpu_sum'] += row['cpu_pct']
        entry['samples'] += 1
        entry['threads'] = max(entry['threads'], row['threads'])
    summary = [
        {
            'label': label,
            'mean_cpu_pct': entry['cpu_sum'] / entry['samples'],
            'max_threads': entry['threads'],
        }
        for label, entry in totals.items()
    ]
    summary.sort(key=lambda item: item['mean_cpu_pct'], reverse=True)
    return summary


def main(argv=None):
    """Sample the onboard process set for the requested duration."""
    parser = argparse.ArgumentParser(
        description='Record per-process CPU during an onboard soak')
    parser.add_argument('--output', required=True,
                        help='absolute TSV path for the per-process samples')
    parser.add_argument('--duration', type=float, default=330.0,
                        help='total sampling seconds')
    parser.add_argument('--interval', type=float, default=5.0,
                        help='seconds between samples')
    args = parser.parse_args(argv)

    output = Path(args.output).expanduser()
    if not output.is_absolute():
        parser.error('--output must be an absolute path')
    output.parent.mkdir(parents=True, exist_ok=True)

    clock_ticks = os.sysconf('SC_CLK_TCK')
    deadline = time.monotonic() + args.duration
    previous = {}
    collected = []
    index = 0
    with output.open('w', encoding='utf-8') as stream:
        stream.write(TSV_HEADER + '\n')
        while True:
            now = time.monotonic()
            rows, previous = sample_processes(clock_ticks, previous, now)
            if rows:
                index += 1
                load = read_loadavg(_read('/proc/loadavg') or '0 0 0')
                temp = _read_temperature()
                throttled = _read_throttled()
                stamp = time.strftime('%Y-%m-%dT%H:%M:%S%z')
                for row in sorted(rows, key=lambda item: -item['cpu_pct']):
                    stream.write(format_row(
                        index, stamp, row, load, temp, throttled) + '\n')
                    collected.append(row)
                stream.flush()
            if time.monotonic() >= deadline:
                break
            time.sleep(args.interval)

    for entry in summarize(collected):
        print(
            f'{entry["label"]:22} mean_cpu={entry["mean_cpu_pct"]:6.1f}%'
            f'  max_threads={entry["max_threads"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
