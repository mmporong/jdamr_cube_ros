#!/usr/bin/env python3
"""Find processes that carry one offline replay's identity environment."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any


def _nul_fields(path: Path) -> list[str]:
    return [
        field.decode(errors='replace')
        for field in path.read_bytes().split(b'\0') if field]


def scan_processes(
        proc_root: Path, run_id: str, domain_id: int,
        excluded_pids: set[int] | None = None) -> list[dict[str, Any]]:
    """Return matching processes sorted by PID."""
    excluded = excluded_pids or set()
    matches = []
    required_environment = {
        f'JDAMR_REPLAY_RUN_ID={run_id}',
        f'ROS_DOMAIN_ID={domain_id}',
    }
    for process_dir in proc_root.glob('[0-9]*'):
        try:
            pid = int(process_dir.name)
            if pid in excluded:
                continue
            environment = set(_nul_fields(process_dir / 'environ'))
            if not required_environment.issubset(environment):
                continue
            raw_stat = (process_dir / 'stat').read_text(encoding='utf-8')
            fields = raw_stat[raw_stat.rfind(')') + 2:].split()
            matches.append({
                'pid': pid,
                'ppid': int(fields[1]),
                'process_group': int(fields[2]),
                'session': int(fields[3]),
                'start_ticks': int(fields[19]),
                'command': _nul_fields(process_dir / 'cmdline'),
            })
        except (FileNotFoundError, PermissionError, IndexError, ValueError):
            continue
    return sorted(matches, key=lambda item: item['pid'])


def main() -> int:
    """Print matching process identities or their unique process groups."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument('--proc-root', type=Path, default=Path('/proc'))
    parser.add_argument('--format', choices=('json', 'pgids'), default='json')
    args = parser.parse_args()
    matches = scan_processes(
        args.proc_root, args.run_id, args.domain_id,
        {os.getpid(), os.getppid()})
    if args.format == 'pgids':
        for process_group in sorted({
                item['process_group'] for item in matches}):
            print(process_group)
    else:
        print(json.dumps(matches, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
