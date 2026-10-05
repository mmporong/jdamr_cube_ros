"""Log UDP receive drops per process on the Pi while a run drives (read-only).

    python3 udp_drop_monitor.py OUT.jsonl [PERIOD_S]

Every PERIOD_S (default 5) appends {"t": unix, "RcvbufErrors": n, "drops": {process: n}}
for sockets that dropped anything. /proc/net/udp counts drops per socket only while
the socket exists, so a process that ends takes its count with it; sampling during
the run keeps the attribution (2026-10-01: 64k drops whose owners had already exited).
"""
import json
import os
import re
import sys
import time


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
        return f'pid{pid}'
    return name_from_cmdline(raw) or f'pid{pid}'


def sample():
    owners = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            for fd in os.listdir(f'/proc/{pid}/fd'):
                match = re.match(r'socket:\[(\d+)\]', os.readlink(f'/proc/{pid}/fd/{fd}'))
                if match:
                    owners[match.group(1)] = pid
        except OSError:
            continue
    drops = {}
    for line in open('/proc/net/udp').read().splitlines()[1:]:
        fields = line.split()
        if int(fields[12]):
            name = process_name(owners.get(fields[9], '?'))
            drops[name] = drops.get(name, 0) + int(fields[12])
    udp = [x for x in open('/proc/net/snmp') if x.startswith('Udp:')][1].split()
    return {'t': round(time.time(), 1), 'RcvbufErrors': int(udp[5]), 'drops': drops}


def main():
    out = sys.argv[1]
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    while True:
        with open(out, 'a') as stream:
            stream.write(json.dumps(sample()) + '\n')
        time.sleep(period)


if __name__ == '__main__':
    main()
