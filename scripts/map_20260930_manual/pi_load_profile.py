"""Log the Pi's CPU, memory and temperature per process while a run drives (read-only).

    python3 pi_load_profile.py OUT.jsonl [PERIOD_S]

Every PERIOD_S (default 2) appends one JSON line: system CPU busy % of all cores,
MemAvailable (MB = 1e6 bytes, null when unreadable), load average, SoC temperature, the firmware throttle flags, and the
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
PAGE_MB = os.sysconf('SC_PAGE_SIZE') / 1e6   # MB = 1e6 bytes (RSS)


INTERPRETERS = ('python3', 'python', 'bash', 'sh')


def name_from_cmdline(raw):
    """Short process name from a /proc cmdline string ('' when there is none)."""
    args = [a for a in raw.split('\0') if a]
    cmd = ' '.join(args)
    for key in ('__node:=', 'lib/'):
        if key in cmd:
            words = cmd[cmd.index(key) + len(key):][:40].split()
            if words:
                return words[0]
    if not args:
        return ''
    base = os.path.basename(args[0])
    if base.startswith('python') or base in INTERPRETERS:
        rest = args[1:]
        if rest and rest[0] == '-m' and len(rest) > 1:
            return rest[1]
        rest = [a for a in rest if not a.startswith('-')]
        if not rest:
            return base
        base, args = os.path.basename(rest[0]), rest
    if base == 'ros2' and len(args) > 1:
        sub = [a for a in args[1:] if not a.startswith('-')]
        return ' '.join(['ros2', *sub[:3 if sub[:1] == ['run'] else 1]])
    return base or cmd[:40]


def process_name(pid):
    try:
        raw = open(f'/proc/{pid}/cmdline').read()
    except OSError:
        return None
    return name_from_cmdline(raw) or None


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
        # fields[19] is starttime: (pid, starttime) identifies one process across pid reuse.
        ticks[int(entry)] = (int(fields[11]) + int(fields[12]), rss_pages * PAGE_MB,
                             int(fields[19]))
    return ticks


def system_ticks():
    values = [int(v) for v in open('/proc/stat').readline().split()[1:]]
    return sum(values) - values[3] - values[4], sum(values)


def mem_available_mb():
    """Return MemAvailable in MB (1e6 bytes, like RSS), None when unreadable."""
    try:
        for line in open('/proc/meminfo'):
            if line.startswith('MemAvailable:'):
                return int(line.split()[1]) * 1024 / 1e6
    except (OSError, ValueError, IndexError):
        pass
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
    for pid, (ticks, rss, started) in after.items():
        try:
            if pid not in before or before[pid][2] != started:
                continue
            cpu = 100.0 * (ticks - before[pid][0]) / TICK / wall_s
            if cpu < 0.5:
                continue
            name = names.get((pid, started))
            if name is None:
                name = names[(pid, started)] = process_name(pid)
            if name is None:
                continue
        except Exception:   # one odd process must not stop the profiler
            continue
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
        mem = mem_available_mb()
        mem_available = None if mem is None else round(mem)
        busy = 100.0 * (sys_after[0] - sys_before[0]) / max(1, sys_after[1] - sys_before[1])
        line = {'t': round(time.time(), 2), 'cpu_busy_pct': round(busy, 1),
                'mem_available_mb': mem_available,
                'loadavg_1m': float(open('/proc/loadavg').read().split()[0]),
                'temp_c': temperature_c(), 'throttled': throttled(),
                'processes': busy_processes(before, after, t_after - t_before, names)}
        with open(out, 'a') as stream:
            stream.write(json.dumps(line) + '\n')
        before, sys_before, t_before = after, sys_after, t_after


if __name__ == '__main__':
    main()
