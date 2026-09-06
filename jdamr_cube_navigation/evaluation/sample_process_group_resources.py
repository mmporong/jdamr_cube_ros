#!/usr/bin/env python3
"""Sample one Linux process group's CPU-time deltas and resident memory."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time
from typing import Any


def process_group_totals(process_group: int) -> tuple[float, int, int]:
    """Return CPU seconds, RSS bytes, and member count from procfs."""
    ticks_per_s = os.sysconf('SC_CLK_TCK')
    page_size = os.sysconf('SC_PAGE_SIZE')
    cpu_s = 0.0
    rss_bytes = 0
    members = 0
    for stat_path in Path('/proc').glob('[0-9]*/stat'):
        try:
            raw = stat_path.read_text(encoding='utf-8')
            fields = raw[raw.rfind(')') + 2:].split()
            if int(fields[2]) != process_group:
                continue
            cpu_s += (int(fields[11]) + int(fields[12])) / ticks_per_s
            rss_bytes += int(fields[21]) * page_size
            members += 1
        except (OSError, ValueError, IndexError):
            continue
    return cpu_s, rss_bytes, members


def sample_row(
        process_group: int,
        previous: tuple[float, float] | None) -> tuple[
            dict[str, Any], tuple[float, float]]:
    """Return one resource row and state for the next CPU delta."""
    monotonic_s = time.monotonic()
    cpu_s, rss_bytes, members = process_group_totals(process_group)
    cpu_pct_one_core = None
    if previous is not None and monotonic_s > previous[0]:
        cpu_pct_one_core = max(
            0.0,
            (cpu_s - previous[1]) / (monotonic_s - previous[0]) * 100.0,
        )
    return ({
        'monotonic_s': monotonic_s,
        'cpu_total_s': cpu_s,
        'cpu_pct_one_core': cpu_pct_one_core,
        'rss_mb': rss_bytes / (1024.0 * 1024.0),
        'process_count': members,
    }, (monotonic_s, cpu_s))


def main() -> int:
    """Sample until the process group disappears."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--process-group', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--interval-s', type=float, default=0.5)
    args = parser.parse_args()
    if args.process_group <= 1 or args.interval_s <= 0.0:
        parser.error('process group and interval must be positive')
    previous = None
    with args.output.open('w', encoding='utf-8') as stream:
        while True:
            row, previous = sample_row(args.process_group, previous)
            stream.write(json.dumps(row) + '\n')
            stream.flush()
            if row['process_count'] == 0:
                break
            time.sleep(args.interval_s)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
