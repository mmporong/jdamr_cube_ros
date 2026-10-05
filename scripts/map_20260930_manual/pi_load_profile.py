"""Log the Pi's CPU, memory and temperature per process while a run drives (read-only).

    python3 pi_load_profile.py OUT.jsonl [PERIOD_S]

Every PERIOD_S (default 2) appends one JSON line: system CPU busy % of all cores,
MemAvailable, load average, SoC temperature, the firmware throttle flags, and the
processes using CPU in that interval (% of one core, RSS MB), so a run shows whether
the Pi 4 keeps up and which node takes the time (2026-10-05: MPPI missed 10 Hz while
the system ran at 75-85 %, and only controller_server had been sampled).
"""
import json
import os
import subprocess
import sys
import time

TICK = os.sysconf('SC_CLK_TCK')
PAGE_MB = os.sysconf('SC_PAGE_SIZE') / 1e6


def process_name(pid):
    try:
        cmd = open(f'/proc/{pid}/cmdline').read().replace('\0', ' ')
    except OSError:
        return None
    for key in ('__node:=', '-m jdamr_cube_navigation.', 'lib/'):
        if key in cmd:
            start = cmd.index(key) + len(key)
            return cmd[start:start + 40].split()[0]
    return cmd[:40] or None


def process_ticks():
    ticks = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            fields = open(f'/proc/{entry}/stat').read().rsplit(')', 1)[1].split()
            rss_pages = int(open(f'/proc/{entry}/statm').read().split()[1])
        except (OSError, IndexError, ValueError):
            continue
        ticks[int(entry)] = (int(fields[11]) + int(fields[12]), rss_pages * PAGE_MB)
    return ticks


def system_ticks():
    values = [int(v) for v in open('/proc/stat').readline().split()[1:]]
    return sum(values) - values[3] - values[4], sum(values)


def mem_available_mb():
    for line in open('/proc/meminfo'):
        if line.startswith('MemAvailable:'):
            return int(line.split()[1]) / 1024
    return None


def temperature_c():
    try:
        return int(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000
    except (OSError, ValueError):
        return None


def throttled():
    try:
        out = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True,
                             timeout=2).stdout.strip()
        return out.split('=')[-1] or None
    except (OSError, subprocess.SubprocessError):
        return None


def busy_processes(before, after, wall_s, names):
    rows = {}
    for pid, (ticks, rss) in after.items():
        if pid not in before:
            continue
        cpu = 100.0 * (ticks - before[pid][0]) / TICK / wall_s
        if cpu < 0.5:
            continue
        name = names.get(pid) or process_name(pid)
        if name is None:
            continue
        names[pid] = name
        total = rows.setdefault(name, [0.0, 0.0])
        total[0] += cpu
        total[1] += rss
    return {name: [round(cpu, 1), round(rss, 1)] for name, (cpu, rss) in
            sorted(rows.items(), key=lambda item: -item[1][0])[:15]}


def main():
    out, period_s = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    names = {}
    before, sys_before, t_before = process_ticks(), system_ticks(), time.monotonic()
    while True:
        time.sleep(period_s)
        after, sys_after, t_after = process_ticks(), system_ticks(), time.monotonic()
        busy = 100.0 * (sys_after[0] - sys_before[0]) / max(1, sys_after[1] - sys_before[1])
        line = {'t': round(time.time(), 2), 'cpu_busy_pct': round(busy, 1),
                'mem_available_mb': round(mem_available_mb() or 0.0),
                'loadavg_1m': float(open('/proc/loadavg').read().split()[0]),
                'temp_c': temperature_c(), 'throttled': throttled(),
                'processes': busy_processes(before, after, t_after - t_before, names)}
        with open(out, 'a') as stream:
            stream.write(json.dumps(line) + '\n')
        before, sys_before, t_before = after, sys_after, t_after


if __name__ == '__main__':
    main()
