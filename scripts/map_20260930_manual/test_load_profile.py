"""Pi load profiler and its summary (2026-10-05)."""
import sys

import analyze_load
import pi_load_profile as profiler
import pytest
import udp_drop_monitor


def test_busy_processes_reports_cpu_of_one_core_and_rss_per_name(monkeypatch):
    monkeypatch.setattr(profiler, 'TICK', 100)
    names = {(1, 7): 'controller_server', (2, 7): 'controller_server', (3, 7): 'amcl',
             (4, 7): 'idle'}
    before = {1: (1000, 50.0, 7), 2: (500, 20.0, 7), 3: (100, 30.0, 7), 4: (10, 5.0, 7)}
    after = {1: (1040, 51.0, 7), 2: (520, 20.0, 7), 3: (110, 30.0, 7), 4: (10, 5.0, 7),
             5: (999, 1.0, 7)}
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


@pytest.mark.parametrize('cmdline,expected', [
    ('/usr/bin/python3\0/home/lim/tools/udp_drop_monitor.py\0out\0', 'udp_drop_monitor.py'),
    ('python3\0-u\0run_it.py\0', 'run_it.py'),
    ('/usr/bin/python3\0-m\0jdamr_cube_navigation.box_service\0--x\0',
     'jdamr_cube_navigation.box_service'),
    ('/usr/bin/python3\0/opt/ros/jazzy/bin/ros2\0bag\0record\0-o\0b\0', 'ros2 bag'),
    ('ros2\0run\0demo_nodes_cpp\0talker\0--ros-args\0', 'ros2 run demo_nodes_cpp talker'),
    ('/opt/ros/jazzy/lib/nav2_amcl/amcl\0--ros-args\0', 'nav2_amcl/amcl'),
    ('node\0--ros-args\0__node:=planner\0', 'planner'),
    ('/bin/bash\0-c\0', 'bash'),
    ('du\0-sh\0/usr/lib/\0', 'du'),          # 'lib/' at the very end used to raise IndexError
    ('', ''),
])
def test_process_names_use_the_script_module_or_ros2_subcommand(cmdline, expected):
    assert profiler.name_from_cmdline(cmdline) == expected
    assert udp_drop_monitor.name_from_cmdline(cmdline) == expected


def test_a_bad_process_cannot_stop_the_profiler_and_pid_reuse_renames(monkeypatch):
    monkeypatch.setattr(profiler, 'TICK', 100)
    calls = []

    def name(pid):
        calls.append(pid)
        if pid == 9:
            raise ValueError('odd process')
        return f'proc{len(calls)}'
    monkeypatch.setattr(profiler, 'process_name', name)
    names = {}
    before = {1: (0, 1.0, 5), 9: (0, 1.0, 5)}
    rows = profiler.busy_processes(before, {1: (100, 1.0, 5), 9: (100, 1.0, 5)}, 1.0, names)
    assert list(rows) == ['proc1']                       # pid 9 skipped, pid 1 reported
    reused = profiler.busy_processes({1: (0, 1.0, 6)}, {1: (100, 1.0, 6)}, 1.0, names)
    assert list(reused) == ['proc3']                     # new starttime: name looked up again


def test_unreadable_memavailable_is_none(monkeypatch):
    def unreadable(*_args):
        raise OSError
    monkeypatch.setattr('builtins.open', unreadable)
    assert profiler.mem_available_mb() is None


def test_analyze_skips_a_cut_off_line_and_tolerates_missing_memory(capsys, monkeypatch, tmp_path):
    good = '{"cpu_busy_pct": 10.0, "mem_available_mb": null, "processes": {}}'
    lines, skipped = analyze_load.read_lines(good + '\n' + good[:30])
    assert (len(lines), skipped) == (1, 1)
    assert analyze_load.summarise(lines)['mem_available_mb_min'] is None
    (tmp_path / 'load_profile.jsonl').write_text('')
    monkeypatch.setattr(sys, 'argv', ['analyze_load.py', str(tmp_path)])
    analyze_load.main()
    assert 'no load samples' in capsys.readouterr().out


def test_summary_skips_lines_that_are_not_samples_and_a_missing_file(tmp_path, capsys):
    import analyze_load
    text = '123\n{"other": 1}\n{"cpu_busy_pct": 5, "processes": {}}\n'
    lines, skipped = analyze_load.read_lines(text)
    assert len(lines) == 1 and skipped == 2
    import sys
    argv = sys.argv
    sys.argv = ['analyze_load.py', str(tmp_path)]
    try:
        analyze_load.main()
    finally:
        sys.argv = argv
    assert 'no load profile' in capsys.readouterr().out
