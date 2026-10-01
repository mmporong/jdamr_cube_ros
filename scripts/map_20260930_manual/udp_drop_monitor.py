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


def process_name(pid):
    try:
        cmd = open(f'/proc/{pid}/cmdline').read().replace('\0', ' ')
    except OSError:
        return f'pid{pid}'
    for key in ('__node:=', '-m jdamr_cube_navigation.', 'lib/'):
        if key in cmd:
            start = cmd.index(key) + len(key)
            return cmd[start:start + 40].split()[0]
    return cmd[:40]


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
