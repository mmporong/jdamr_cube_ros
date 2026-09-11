"""Attribute onboard soak load to individual processes."""

# System load alone cannot attribute cost to a process.  This sampler reads
# /proc directly and records the executable command with every row so a label
# can be audited after the run.  Historical measurements live in evaluation
# reports, not in runtime comments.

import argparse
import os
from pathlib import Path
import shlex
import sys
import time


# Match executable tokens, not arbitrary command-line substrings.  The old
# regexes classified ``ros2 bag record ... /collision_monitor_state`` as the
# Collision Monitor and did not classify ``component_container_isolated`` at
# all.  That made a recorder look like a Nav2 server while omitting the process
# that held every composed server.
EXECUTABLE_LABELS = {
    'map_server': 'map_server',
    'costmap_filter_info_server': 'costmap_filter_info',
    'amcl': 'amcl',
    'controller_server': 'controller_server',
    'planner_server': 'planner_server',
    'velocity_smoother': 'velocity_smoother',
    'collision_monitor': 'collision_monitor',
    'bt_navigator': 'bt_navigator',
    'lifecycle_manager': 'lifecycle_manager',
    '_ros2_bag': 'recorder',
    'corridor_route': 'corridor_route',
    'nav2_liveness_guard': 'nav2_liveness_guard',
    'base_driver_node': 'bringup',
    'ydlidar_g4_node': 'bringup',
    'rplidar_composition': 'bringup',
    'micro_ros_agent': 'bringup',
}

COMPONENT_CONTAINER_EXECUTABLES = {
    'component_container',
    'component_container_isolated',
    'component_container_mt',
}

SPLIT_NAV2_REQUIRED = {
    'map_server',
    'amcl',
    'controller_server',
    'planner_server',
    'collision_monitor',
    'bt_navigator',
}
MIN_STEADY_COVERAGE_SAMPLES = 2
MIN_STEADY_COVERAGE_S = 60.0
MAX_STEADY_SAMPLE_GAP_S = 10.0

TSV_HEADER = (
    'sample\ttime\telapsed_s\tsystem_cpu_pct\t'
    'label\tpid\tcpu_pct\tthreads\trss_mb\t'
    'load1\tload5\tload15\ttemp_c\tthrottled\tcommand'
)


def classify(cmdline):
    """Return the contract label for a process command line."""
    try:
        tokens = shlex.split(cmdline)
    except ValueError:
        tokens = cmdline.split()
    names = [Path(token).name for token in tokens]

    # Recorder detection must happen before topic arguments are inspected.
    # A recorded topic can have exactly the same name as a Nav2 executable.
    for index in range(len(names) - 2):
        if names[index:index + 3] == ['ros2', 'bag', 'record']:
            return 'recorder'
    if '_ros2_bag' in names:
        return 'recorder'

    # ``ros2 launch`` is a real process too.  Keep it visible so a future
    # report can audit the entire process set rather than only its children.
    for index in range(len(names) - 3):
        if names[index:index + 2] == ['ros2', 'launch']:
            target = names[index + 3]
            if target in {
                    'onboard_keepout_navigation.launch.py',
                    'onboard_nav2_core.launch.py'}:
                return 'nav2_launch'

    for index in range(len(names) - 3):
        if names[index:index + 2] == ['ros2', 'run']:
            label = EXECUTABLE_LABELS.get(names[index + 3])
            if label is not None:
                return label

    executable_names = names[:1]
    if names and names[0].startswith('python') and len(names) > 1:
        executable_names.append(names[1])
    for name in executable_names:
        if name in COMPONENT_CONTAINER_EXECUTABLES:
            if '__node:=nav2_container' in tokens:
                return 'nav2_container'
            return 'other'
        label = EXECUTABLE_LABELS.get(name)
        if label is not None:
            return label
    return 'other'


def process_coverage(rows):
    """Return simultaneous, continuous process coverage after warmup."""
    # A label is evidence only when the executable command that produced it is
    # retained.  This intentionally rejects legacy TSVs whose classification
    # cannot be audited after the fact.
    samples = {}
    for row in rows:
        sample = str(row.get('sample', '0'))
        entry = samples.setdefault(sample, {
            'labels': set(),
            'elapsed_s': row.get('elapsed_s'),
        })
        if row.get('command'):
            entry['labels'].add(row['label'])
        if entry['elapsed_s'] is None and row.get('elapsed_s') is not None:
            entry['elapsed_s'] = row['elapsed_s']

    ordered = list(samples.items())
    for start, (_sample, entry) in enumerate(ordered):
        labels = entry['labels']
        if {'nav2_container', 'recorder'} <= labels:
            mode = 'composed'
            required = {'nav2_container', 'recorder'}
            break
        if SPLIT_NAV2_REQUIRED | {'recorder'} <= labels:
            mode = 'split'
            required = SPLIT_NAV2_REQUIRED | {'recorder'}
            break
    else:
        return {
            'mode': 'incomplete', 'complete': 0, 'checked': 0,
            'duration_s': 0.0, 'max_gap_s': None,
            'cadence_valid': False, 'sample_ids': (),
        }

    steady = ordered[start:]
    complete = sum(
        required <= entry['labels'] for _sample, entry in steady)
    elapsed_values_s = [entry['elapsed_s'] for _sample, entry in steady]
    timing_complete = all(value is not None for value in elapsed_values_s)
    elapsed_values_s = (
        [float(value) for value in elapsed_values_s]
        if timing_complete else [])
    gaps_s = [
        current_s - previous_s
        for previous_s, current_s in zip(
            elapsed_values_s, elapsed_values_s[1:])
    ]
    duration_s = (
        max(0.0, elapsed_values_s[-1] - elapsed_values_s[0])
        if elapsed_values_s else 0.0)
    max_gap_s = max(gaps_s, default=None)
    cadence_valid = (
        timing_complete and
        bool(gaps_s) and
        all(0.0 < gap_s <= MAX_STEADY_SAMPLE_GAP_S for gap_s in gaps_s)
    )
    valid = (
        len(steady) >= MIN_STEADY_COVERAGE_SAMPLES and
        duration_s >= MIN_STEADY_COVERAGE_S and
        cadence_valid and
        complete == len(steady)
    )
    return {
        'mode': mode if valid else 'incomplete',
        'complete': complete,
        'checked': len(steady),
        'duration_s': duration_s,
        'max_gap_s': max_gap_s,
        'cadence_valid': cadence_valid,
        'sample_ids': tuple(sample for sample, _entry in steady),
    }


def process_mode(rows):
    """Return the measured Nav2 deployment mode, or ``None`` if incomplete."""
    mode = process_coverage(rows)['mode']
    return None if mode == 'incomplete' else mode


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


def parse_system_cpu_stat(text):
    """Return Linux system-wide busy and total CPU jiffies."""
    fields = text.splitlines()[0].split()
    if not fields or fields[0] != 'cpu' or len(fields) < 6:
        raise ValueError('invalid /proc/stat cpu line')
    values = [int(value) for value in fields[1:9]]
    total_jiffies = sum(values)
    idle_jiffies = values[3] + values[4]
    return total_jiffies - idle_jiffies, total_jiffies


def system_cpu_percent(previous, current, core_count):
    """Convert system CPU jiffy deltas to the process-percent scale."""
    busy_delta_jiffies = current[0] - previous[0]
    total_delta_jiffies = current[1] - previous[1]
    if (
            total_delta_jiffies <= 0 or
            busy_delta_jiffies < 0 or
            busy_delta_jiffies > total_delta_jiffies):
        return None
    return 100.0 * core_count * busy_delta_jiffies / total_delta_jiffies


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
            'command': cmdline.replace('\t', ' ').replace('\n', ' '),
        })
    return rows, snapshot


def _decimal_or_empty(value, places=1):
    """Format an optional numeric value without inventing a measurement."""
    if value in ('', None):
        return ''
    return f'{float(value):.{places}f}'


def format_row(index, timestamp, row, load, temp, throttled):
    """Render one TSV line for a sampled process."""
    fields = (
        index, timestamp,
        _decimal_or_empty(row.get('elapsed_s'), places=3),
        _decimal_or_empty(row.get('system_cpu_pct')),
        row['label'], row['pid'],
        f'{row["cpu_pct"]:.1f}', row['threads'], f'{row["rss_mb"]:.1f}',
        load[0], load[1], load[2], temp, throttled, row.get('command', ''),
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


def percentile(values, fraction):
    """Return the value at *fraction* of a sorted sample list."""
    if not values:
        return 0.0
    ordered = sorted(values)
    position = min(len(ordered) - 1, int(len(ordered) * fraction))
    return ordered[position]


# Linux load average includes tasks waiting on I/O and locks, so it is retained
# as diagnostic context rather than used as the drive gate.  The executable
# contract below evaluates measured CPU headroom, thermal state, and process
# coverage.  Run-specific values live in evaluation reports.
CPU_BUDGET_FRACTION = 0.75

# Keep an operating margin below the platform's thermal throttle point.
MAX_TEMPERATURE_C = 75.0


def evaluate_resource_gate(rows, samples, core_count,
                           budget_fraction=CPU_BUDGET_FRACTION,
                           max_temperature_c=MAX_TEMPERATURE_C):
    """Return the onboard resource verdict measured from a soak."""
    budget_pct = 100.0 * core_count * budget_fraction
    coverage = process_coverage(rows)
    steady_sample_ids = set(coverage['sample_ids'])
    steady_samples = [
        sample for sample in samples
        if not steady_sample_ids or str(sample.get('sample')) in steady_sample_ids
    ]
    system_cpu_values = [
        sample['system_cpu_pct'] for sample in steady_samples
        if sample.get('system_cpu_pct') is not None
    ]
    peak_cpu = max(system_cpu_values, default=0.0)
    mean_cpu = (
        sum(system_cpu_values) / len(system_cpu_values)
        if system_cpu_values else 0.0)
    # Startup is transient, so the gate uses a sustained percentile and keeps
    # the peak as diagnostic output.  The percentile behavior is executable in
    # test_onboard_load.py rather than duplicated as a numeric comment.
    sustained_cpu = percentile(
        system_cpu_values, 0.90)
    temperatures = [sample['temp_c'] for sample in steady_samples
                    if sample['temp_c'] is not None]
    peak_temp = max(temperatures, default=0.0)
    throttled = [sample['throttled'] for sample in steady_samples
                 if sample['throttled'] not in ('', None)]
    throttle_clean = (
        bool(throttled) and
        all(value in ('0', '0x0') for value in throttled)
    )
    steady_rows = [
        row for row in rows
        if not steady_sample_ids or str(row.get('sample')) in steady_sample_ids
    ]
    peak_threads = max((row['threads'] for row in steady_rows), default=0)

    mode = coverage['mode']
    gates = {
        'process_coverage': 'PASS' if mode != 'incomplete' else 'FAIL',
        'cpu_headroom': (
            'PASS' if system_cpu_values and sustained_cpu <= budget_pct
            else ('FAIL' if system_cpu_values else 'UNKNOWN')),
        'thermal_throttle': (
            'PASS' if throttle_clean
            else ('FAIL' if throttled else 'UNKNOWN')),
        'temperature': (
            'PASS' if temperatures and peak_temp <= max_temperature_c
            else ('FAIL' if temperatures else 'UNKNOWN')),
    }
    return {
        'gates': gates,
        'verdict': (
            'FAIL' if 'FAIL' in gates.values()
            else ('UNKNOWN' if 'UNKNOWN' in gates.values() else 'PASS')),
        'core_count': core_count,
        'cpu_budget_pct': budget_pct,
        'peak_cpu_pct': peak_cpu,
        'sustained_cpu_pct': sustained_cpu,
        'mean_cpu_pct': mean_cpu,
        'peak_temperature_c': peak_temp,
        'peak_threads': peak_threads,
        'process_mode': mode,
        'coverage_complete_samples': coverage['complete'],
        'coverage_checked_samples': coverage['checked'],
        'coverage_duration_s': coverage['duration_s'],
        'coverage_max_gap_s': coverage['max_gap_s'],
    }


def read_samples(path):
    """Aggregate a per-process TSV into one record per sample index."""
    rows = []
    totals = {}
    with Path(path).open(encoding='utf-8') as stream:
        header = stream.readline().rstrip('\n').split('\t')
        index_of = {name: position for position, name in enumerate(header)}
        for line in stream:
            fields = line.rstrip('\n').split('\t')
            if len(fields) < len(header):
                continue
            sample = fields[index_of['sample']]
            cpu = float(fields[index_of['cpu_pct']])
            threads = int(fields[index_of['threads']])
            elapsed_s = (
                float(fields[index_of['elapsed_s']])
                if 'elapsed_s' in index_of and fields[index_of['elapsed_s']]
                else None
            )
            system_cpu_pct = (
                float(fields[index_of['system_cpu_pct']])
                if ('system_cpu_pct' in index_of and
                    fields[index_of['system_cpu_pct']])
                else None
            )
            rows.append({
                'sample': sample,
                'elapsed_s': elapsed_s,
                'label': fields[index_of['label']],
                'cpu_pct': cpu,
                'threads': threads,
                'command': (
                    fields[index_of['command']]
                    if 'command' in index_of else ''),
            })
            entry = totals.setdefault(sample, {
                'sample': sample,
                'elapsed_s': elapsed_s,
                'system_cpu_pct': system_cpu_pct,
                'cpu_pct': 0.0,
                'load1': fields[index_of['load1']],
                'temp_c': None,
                'throttled': fields[index_of['throttled']],
            })
            entry['cpu_pct'] += cpu
            if entry['elapsed_s'] is None and elapsed_s is not None:
                entry['elapsed_s'] = elapsed_s
            if entry['system_cpu_pct'] is None and system_cpu_pct is not None:
                entry['system_cpu_pct'] = system_cpu_pct
            raw_temp = fields[index_of['temp_c']]
            if raw_temp:
                entry['temp_c'] = float(raw_temp)
    return rows, list(totals.values())


def select_recent_window(rows, samples, window_seconds):
    """Keep only the newest measured window without inventing samples."""
    if window_seconds is None:
        return rows, samples
    elapsed = [
        float(sample['elapsed_s']) for sample in samples
        if sample.get('elapsed_s') is not None
    ]
    if not elapsed:
        return rows, samples
    cutoff_s = max(elapsed) - window_seconds
    measured = [
        sample for sample in samples if sample.get('elapsed_s') is not None
    ]
    # Include the sample immediately before the cutoff. With a 5 s sampler,
    # filtering only timestamps >= cutoff can leave 57--59 s after cadence
    # jitter and falsely reject an otherwise complete 60 s observation.
    start = 0
    for index, sample in enumerate(measured):
        if float(sample['elapsed_s']) <= cutoff_s:
            start = index
        else:
            break
    selected_samples = measured[start:]
    sample_ids = {str(sample['sample']) for sample in selected_samples}
    selected_rows = [
        row for row in rows if str(row.get('sample')) in sample_ids
    ]
    return selected_rows, selected_samples


def main(argv=None):
    """Sample the onboard process set for the requested duration."""
    parser = argparse.ArgumentParser(
        description='Record per-process CPU during an onboard soak')
    parser.add_argument('--output',
                        help='absolute TSV path for the per-process samples')
    parser.add_argument('--duration', type=float, default=330.0,
                        help='total sampling seconds')
    parser.add_argument('--interval', type=float, default=5.0,
                        help='seconds between samples')
    parser.add_argument('--evaluate',
                        help='score an existing TSV instead of sampling')
    parser.add_argument('--window-seconds', type=float, default=None,
                        help='with --evaluate, score only the newest window')
    parser.add_argument('--cores', type=int, default=None,
                        help='core count of the machine that produced the '
                             'samples; defaults to this machine')
    args = parser.parse_args(argv)
    if args.cores is not None and args.cores <= 0:
        parser.error('--cores must be positive')
    if args.duration < 0:
        parser.error('--duration must be non-negative')
    if args.interval <= 0:
        parser.error('--interval must be positive')
    if args.window_seconds is not None and args.window_seconds <= 0:
        parser.error('--window-seconds must be positive')
    if args.window_seconds is not None and not args.evaluate:
        parser.error('--window-seconds requires --evaluate')

    if args.evaluate:
        rows, samples = read_samples(args.evaluate)
        rows, samples = select_recent_window(
            rows, samples, args.window_seconds)
        return _report(rows, samples, cores=args.cores)

    output = Path(args.output or '.').expanduser()
    if not args.output:
        parser.error('--output is required unless --evaluate is given')
    if not output.is_absolute():
        parser.error('--output must be an absolute path')
    output.parent.mkdir(parents=True, exist_ok=True)

    clock_ticks = os.sysconf('SC_CLK_TCK')
    started_at_s = time.monotonic()
    deadline = started_at_s + args.duration
    sampling_cores = args.cores or os.cpu_count() or 1
    previous = {}
    previous_system_cpu_jiffies = None
    index = 0
    with output.open('w', encoding='utf-8') as stream:
        stream.write(TSV_HEADER + '\n')
        while True:
            now = time.monotonic()
            rows, previous = sample_processes(clock_ticks, previous, now)
            current_system_cpu_jiffies = parse_system_cpu_stat(
                _read('/proc/stat') or '')
            measured_system_cpu_pct = (
                system_cpu_percent(
                    previous_system_cpu_jiffies,
                    current_system_cpu_jiffies,
                    sampling_cores)
                if previous_system_cpu_jiffies is not None else None
            )
            previous_system_cpu_jiffies = current_system_cpu_jiffies
            index += 1
            load = read_loadavg(_read('/proc/loadavg') or '0 0 0')
            temp = _read_temperature()
            throttled = _read_throttled()
            stamp = time.strftime('%Y-%m-%dT%H:%M:%S%z')
            elapsed_s = now - started_at_s
            if not rows:
                # Preserve empty intervals.  Without this sentinel, a complete
                # process set followed by total process loss looked continuous
                # because the missing tick never reached the TSV.
                rows = [{
                    'label': 'sample_sentinel',
                    'pid': 0,
                    'cpu_pct': 0.0,
                    'threads': 0,
                    'rss_mb': 0.0,
                    'command': 'soak_metrics internal sample sentinel',
                }]
            for row in sorted(rows, key=lambda item: -item['cpu_pct']):
                row = {
                    **row,
                    'elapsed_s': elapsed_s,
                    'system_cpu_pct': measured_system_cpu_pct,
                }
                stream.write(format_row(
                    index, stamp, row, load, temp, throttled) + '\n')
            stream.flush()
            if time.monotonic() >= deadline:
                break
            time.sleep(args.interval)

    return _report(*read_samples(output), cores=sampling_cores)


def _report(rows, samples, cores=None):
    """Print the per-process ranking and the resource verdict."""
    for entry in summarize(rows):
        print(
            f'{entry["label"]:22} mean_cpu={entry["mean_cpu_pct"]:6.1f}%'
            f'  max_threads={entry["max_threads"]}')
    verdict = evaluate_resource_gate(
        rows, samples, cores or os.cpu_count() or 1)
    print()
    print(f'mode={verdict["process_mode"]} '
          f'coverage={verdict["coverage_complete_samples"]}/'
          f'{verdict["coverage_checked_samples"]} '
          f'duration={verdict["coverage_duration_s"]:.1f}s '
          f'max_gap={verdict["coverage_max_gap_s"] or 0.0:.1f}s '
          f'cores={verdict["core_count"]} '
          f'budget={verdict["cpu_budget_pct"]:.0f}% '
          f'system_p90={verdict["sustained_cpu_pct"]:.0f}% '
          f'system_peak={verdict["peak_cpu_pct"]:.0f}% '
          f'system_mean={verdict["mean_cpu_pct"]:.0f}% '
          f'peak_temp={verdict["peak_temperature_c"]:.1f}C '
          f'threads={verdict["peak_threads"]}')
    for name, value in verdict['gates'].items():
        print(f'  {name:20} {value}')
    print(f'resource gate: {verdict["verdict"]}')
    return 0 if verdict['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
