"""
Summarise a run's Pi load profile (load_profile.jsonl from pi_load_profile.py).

Usage: python3 analyze_load.py RUN_DIR
"""
import json
from pathlib import Path
import statistics
import sys


def summarise(lines):
    busy = [line['cpu_busy_pct'] for line in lines]
    per_process = {}
    for line in lines:
        for name, (cpu, rss) in line['processes'].items():
            entry = per_process.setdefault(name, {'cpu': [], 'rss': []})
            entry['cpu'].append(cpu)
            entry['rss'].append(rss)
    processes = sorted(
        ((name, sum(v['cpu']) / len(lines), max(v['cpu']), max(v['rss']))
         for name, v in per_process.items()), key=lambda row: -row[1])
    temps = [line['temp_c'] for line in lines if line.get('temp_c') is not None]
    return {
        'samples': len(lines),
        'cpu_busy_pct_mean': round(statistics.mean(busy), 1),
        'cpu_busy_pct_max': max(busy),
        'mem_available_mb_min': min(line['mem_available_mb'] for line in lines),
        'temp_c_max': max(temps) if temps else None,
        'throttled': sorted({line['throttled'] for line in lines if line.get('throttled')}),
        'top_processes': [{'name': name, 'cpu_mean_pct': round(mean, 1), 'cpu_max_pct': peak,
                           'rss_max_mb': rss} for name, mean, peak, rss in processes[:12]],
    }


def main():
    path = Path(sys.argv[1]) / 'load_profile.jsonl'
    lines = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    print(json.dumps(summarise(lines), ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
