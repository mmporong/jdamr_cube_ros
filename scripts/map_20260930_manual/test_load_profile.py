"""Pi load profiler and its summary (2026-10-05)."""
import analyze_load
import pi_load_profile as profiler


def test_busy_processes_reports_cpu_of_one_core_and_rss_per_name(monkeypatch):
    monkeypatch.setattr(profiler, 'TICK', 100)
    names = {1: 'controller_server', 2: 'controller_server', 3: 'amcl', 4: 'idle'}
    before = {1: (1000, 50.0), 2: (500, 20.0), 3: (100, 30.0), 4: (10, 5.0)}
    after = {1: (1040, 51.0), 2: (520, 20.0), 3: (110, 30.0), 4: (10, 5.0), 5: (999, 1.0)}
    rows = profiler.busy_processes(before, after, 2.0, names)
    assert rows['controller_server'] == [30.0, 71.0]      # (40 + 20) ticks / 100 / 2 s
    assert rows['amcl'] == [5.0, 30.0]
    assert 'idle' not in rows and len(rows) == 2           # below 0.5 % and new pids skipped


def test_summary_ranks_processes_and_keeps_the_worst_case():
    lines = [
        {'cpu_busy_pct': 60.0, 'mem_available_mb': 900, 'temp_c': 61.0, 'throttled': '0x0',
         'processes': {'controller_server': [20.0, 70.0], 'amcl': [5.0, 30.0]}},
        {'cpu_busy_pct': 85.0, 'mem_available_mb': 700, 'temp_c': 66.5, 'throttled': '0x20000',
         'processes': {'controller_server': [40.0, 72.0]}},
    ]
    summary = analyze_load.summarise(lines)
    assert summary['cpu_busy_pct_mean'] == 72.5 and summary['cpu_busy_pct_max'] == 85.0
    assert summary['mem_available_mb_min'] == 700 and summary['temp_c_max'] == 66.5
    assert summary['throttled'] == ['0x0', '0x20000']
    top = summary['top_processes'][0]
    assert top == {'name': 'controller_server', 'cpu_mean_pct': 30.0, 'cpu_max_pct': 40.0,
                   'rss_max_mb': 72.0}
    assert summary['top_processes'][1]['cpu_mean_pct'] == 2.5   # absent samples count as 0
