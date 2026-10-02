"""Mocked flow checks for jdamr_depart.py: no ssh, no robot, temporary state file.

PC-only like the tool itself (it reads the map data at import):
    python3 -m pytest -q scripts/map_20260930_manual/test_jdamr_depart_flow.py
"""
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parent
if not (Path.home() / 'jdamr_data/map_20260930_manual/aligned/destinations.json').exists():
    pytest.skip('map_20260930_manual data is not on this machine', allow_module_level=True)
# dock_survey and scan_match are the Phase 2 tools the departure tool imports.
sys.path[:0] = [str(HERE), str(HERE.parent / 'phase2_20260930')]
import jdamr_depart as d  # noqa: E402


class Pi:
    """Scripted Pi: answers by command substring, records every command."""

    def __init__(self, busy='', linger='yes', session='active', restart_moves=True):
        self.calls, self.busy, self.linger, self.session = [], busy, linger, session
        self.restart_moves = restart_moves
        self.start = {u: 100 for u in d.SENSOR_UNITS}

    def __call__(self, command, timeout=120, check=True, stdin=None):
        self.calls.append(command)
        out = ''
        if 'jdamr-table-cycle-*' in command:
            out = self.busy
        elif 'loginctl show-user' in command:
            out = self.linger
        elif 'is-active jdamr-restaurant-navigation' in command:
            out = self.session
        elif command.startswith('systemctl show'):
            unit = command.split()[-1]
            out = f'ActiveState=active\nActiveEnterTimestampMonotonic={self.start[unit]}\n'
        elif command.startswith('sudo -n systemctl restart'):
            unit = command.split()[-1]
            if self.restart_moves:
                self.start[unit] += 1
        return SimpleNamespace(returncode=0, stdout=out)


@pytest.fixture
def env(tmp_path, monkeypatch):
    monkeypatch.setattr(d, 'STATE', tmp_path / 'state.json')
    monkeypatch.setattr(d, 'P2', tmp_path)
    monkeypatch.setattr(d, 'CLICK', tmp_path / 'click.json')
    monkeypatch.setattr(d.time, 'sleep', lambda _s: None)
    monkeypatch.setattr(d, 'sh', lambda *a, **k: SimpleNamespace(returncode=0, stdout='active'))
    regions = {k: {'xy': [0.0, 0.0], 'radius_m': 0.4} for k in ('water_station', 'table_02')}
    d.save_state({'regions': regions, 'pose': [0, 0, 0], 'localized': True,
                  'init_local_only': False})
    return tmp_path


def go_args(**extra):
    base = dict(table_id='table_02', route=None, region=None, via_route=None, skip_via=False,
                resume_at_observation=False, resume_parked_log=None, dock_only=False, graceful_final=False,
                mppi_transit=False)
    base.update(extra)
    return SimpleNamespace(**base)


def test_go_refuses_without_a_completed_init(env, monkeypatch):
    state = d.load_state()
    state['localized'] = False
    d.save_state(state)
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    with pytest.raises(SystemExit):
        d.cmd_go(go_args())
    assert pi.calls == []


def test_go_during_a_cycle_stops_nothing(env, monkeypatch):
    pi = Pi(busy='jdamr-table-cycle-x.service loaded active running')
    monkeypatch.setattr(d, 'pi', pi)
    runs = []
    monkeypatch.setattr(d, 'run_cycle', lambda *a: runs.append(a) or [])
    with pytest.raises(SystemExit):
        d.cmd_go(go_args())
    assert runs == []
    assert not any('stop' in c or 'restart' in c for c in pi.calls)


@pytest.mark.parametrize('action', ['recover', 'session-stop'])
def test_recover_and_session_stop_refuse_during_a_cycle(env, monkeypatch, action):
    pi = Pi(busy='jdamr-table-cycle-x.service loaded active running')
    monkeypatch.setattr(d, 'pi', pi)
    handler = d.cmd_recover if action == 'recover' else d.cmd_session_stop
    with pytest.raises(SystemExit):
        handler(SimpleNamespace(seed=None, local_only=False))
    assert not any('restart' in c or 'session.sh stop' in c for c in pi.calls)
    assert d.load_state()['localized'] is True


MAP_FAIL = [{'event': 'failed', 'reason': 'live map data unavailable: map'}]


def test_map_wait_failure_recovers_once_and_departs_again(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    runs, recovered = [], []
    results = iter([MAP_FAIL, [{'event': 'home_arrived', 'confirmation': {'confirmed': True}}]])
    monkeypatch.setattr(d, 'run_cycle', lambda *a: runs.append(a) or next(results))
    monkeypatch.setattr(d, 'cmd_recover', lambda a: recovered.append(
        (a.seed, a.local_only)))
    d.cmd_go(go_args())
    assert len(runs) == 2 and recovered == [(None, False)]


def test_later_failures_are_not_retried(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    runs = []
    moved = [{'event': 'observation_route_selected'},
             {'event': 'failed', 'reason': 'live map data unavailable: map'}]
    monkeypatch.setattr(d, 'run_cycle', lambda *a: runs.append(a) or moved)
    monkeypatch.setattr(d, 'cmd_recover', lambda a: pytest.fail('recovered'))
    # A cycle that did not end docked exits non-zero, without a retry.
    with pytest.raises(SystemExit):
        d.cmd_go(go_args())
    assert len(runs) == 1


def test_recover_local_only_needs_a_seed(env, monkeypatch):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    with pytest.raises(SystemExit):
        d.cmd_recover(SimpleNamespace(seed=None, local_only=True))
    assert pi.calls == []


@pytest.mark.parametrize('extra', [{'skip_via': True}, {'resume_at_observation': True},
                                   {'resume_parked_log': '/x y/log.jsonl'}, 'local_init'])
def test_resumes_and_local_inits_get_guidance_not_auto_recovery(env, monkeypatch, extra):
    if extra == 'local_init':
        state = d.load_state()
        state['init_local_only'] = True
        d.save_state(state)
        extra = {}
    monkeypatch.setattr(d, 'pi', Pi())
    monkeypatch.setattr(d, 'run_cycle', lambda *a: MAP_FAIL)
    monkeypatch.setattr(d, 'cmd_recover', lambda a: pytest.fail('recovered'))
    with pytest.raises(SystemExit):
        d.cmd_go(go_args(**extra))


def test_recover_requires_linger(env, monkeypatch):
    pi = Pi(linger='no')
    monkeypatch.setattr(d, 'pi', pi)
    with pytest.raises(SystemExit):
        d.cmd_recover(SimpleNamespace(seed=None, local_only=False))
    assert not any('restart' in c for c in pi.calls)


def test_restart_must_show_a_new_start_time(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi(restart_moves=False))
    with pytest.raises(SystemExit):
        d.restart_sensor_services()


def test_restart_refuses_an_inactive_sensor_service(env, monkeypatch):
    pi = Pi()
    real = pi.__call__

    def inactive_observer(command, **kw):
        result = real(command, **kw)
        if command.startswith('systemctl show') and command.endswith('jdamr-box-observer.service'):
            result.stdout = 'ActiveState=inactive\nActiveEnterTimestampMonotonic=0\n'
        return result
    monkeypatch.setattr(d, 'pi', inactive_observer)
    with pytest.raises(SystemExit):
        d.restart_sensor_services()


def test_restart_covers_all_sensor_services(env, monkeypatch):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    d.restart_sensor_services()
    restarted = [c.split()[-1] for c in pi.calls if c.startswith('sudo -n systemctl restart')]
    assert restarted == list(d.SENSOR_UNITS)


def test_recover_seed_does_not_imply_local_only(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    for name in ('cmd_display_stop', 'cmd_session_stop', 'restart_sensor_services',
                 'cmd_display_start', 'cmd_session_start'):
        monkeypatch.setattr(d, name, lambda *a: None)
    seen = []
    monkeypatch.setattr(d, 'cmd_init', lambda a: seen.append(
        (a.keep_home, a.seed_from_state, a.local_only, a.click)))
    monkeypatch.setattr(d, 'dds_health', lambda: ('ok', 'PROBE health ok {} 1.0 s'))
    d.cmd_recover(SimpleNamespace(seed=[1.0, 2.0, 90.0], local_only=False))
    assert seen == [(True, False, False, True)]
    assert json.loads(d.CLICK.read_text())['x_m'] == 1.0


def test_init_seeds_from_the_registered_dock_and_keeps_it(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    monkeypatch.setattr(d, 'pull_registry', lambda: {'home': {
        'x_m': -0.204, 'y_m': 0.184, 'yaw_rad': 0.0}})
    monkeypatch.setattr(d, 'capture_scan', lambda _path: None)
    seen = []

    def refine(_scan, click):
        seen.append(click)
        raise SystemExit('stop after seeding')
    monkeypatch.setattr(d, 'refine_pose', refine)
    args = SimpleNamespace(seed_from_state=False, click=False, local_only=False, keep_home=False)
    with pytest.raises(SystemExit):
        d.cmd_init(args)
    assert (seen[0]['x_m'], seen[0]['y_m'], seen[0]['source']) == (-0.204, 0.184, 'registered_dock')
    assert args.keep_home is True
    assert not d.CLICK.exists()


def test_failed_init_leaves_go_refused(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi(session='inactive'))
    with pytest.raises(SystemExit):
        d.cmd_init(SimpleNamespace(seed_from_state=True, local_only=False, keep_home=True))
    assert d.load_state()['localized'] is False


@pytest.mark.parametrize('stdout,expected', [
    ("PROBE health ok {'map': 1} 1.2 s\n", 'ok'),
    ("PROBE health MISSING {'map': 0} 30.0 s\n", 'missing'),
    ('ssh: connect to host: No route to host\n', 'error'),
])
def test_probe_result_is_classified(env, monkeypatch, stdout, expected):
    monkeypatch.setattr(d, 'pi', lambda *a, **k: SimpleNamespace(returncode=0, stdout=stdout))
    assert d.dds_health()[0] == expected


def test_probe_timeout_is_an_error(env, monkeypatch):
    def slow(*_a, **_k):
        raise subprocess.TimeoutExpired('ssh', 60)
    monkeypatch.setattr(d, 'pi', slow)
    assert d.dds_health()[0] == 'error'


def test_session_restart_revokes_localization(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', lambda c, **k: SimpleNamespace(
        returncode=0, stdout='' if 'jdamr-table-cycle-*' in c else 'STARTED'))
    d.cmd_session_start(None)
    assert d.load_state()['localized'] is False


def test_session_reuse_keeps_localization(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', lambda c, **k: SimpleNamespace(
        returncode=0, stdout='' if 'jdamr-table-cycle-*' in c else 'REUSED'))
    d.cmd_session_start(None)
    assert d.load_state()['localized'] is True


def test_display_start_starts_only_missing_units(env, monkeypatch):
    started = []

    def sh(command, **_k):
        if command.startswith('systemd-run'):
            started.append(command.split('--unit=')[1].split()[0])
        if 'is-active' in command:
            unit = command.split()[-1]
            active = unit != 'jdamr-p2-rviz' or 'jdamr-p2-rviz' in started
            return SimpleNamespace(returncode=0, stdout='active' if active else 'inactive')
        return SimpleNamespace(returncode=0, stdout='')
    monkeypatch.setattr(d, 'sh', sh)
    d.cmd_display_start(None)
    assert started == ['jdamr-p2-rviz']


def test_stop_during_the_map_wait_prevents_recovery(env, monkeypatch):
    """A stop makes the executor fail with the same map message; do not re-depart."""
    monkeypatch.setattr(d, 'pi', Pi())

    def stopped_run(*_a):
        d.cmd_stop(None)
        return MAP_FAIL
    monkeypatch.setattr(d, 'run_cycle', stopped_run)
    monkeypatch.setattr(d, 'cmd_recover', lambda a: pytest.fail('recovered'))
    with pytest.raises(SystemExit):
        d.cmd_go(go_args())


def test_stop_during_recovery_prevents_the_second_departure(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    runs = []
    monkeypatch.setattr(d, 'run_cycle', lambda *a: runs.append(a) or MAP_FAIL)
    monkeypatch.setattr(d, 'cmd_recover', lambda a: d.cmd_stop(None))
    with pytest.raises(SystemExit):
        d.cmd_go(go_args())
    assert len(runs) == 1


class ExecutorPi(Pi):
    """The resident executor is up; requests finish at once."""

    def __init__(self):
        super().__init__()
        self.stdins = []

    def __call__(self, command, timeout=120, check=True, stdin=None):
        self.stdins.append(stdin)
        if command == f'systemctl is-active {d.EXECUTOR_UNIT}':
            self.calls.append(command)
            return SimpleNamespace(returncode=0, stdout='active')
        return super().__call__(command, timeout, check, stdin)


def test_go_hands_the_run_to_the_resident_executor(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
    events = d.run_cycle(go_args(), d.load_state(), 'table_02')
    assert events == [{'event': 'home_arrived'}]
    request = next(s for s in pi.stdins if s)
    argv = json.loads(request)['argv']
    assert argv[argv.index('--table-id') + 1] == 'table_02'
    assert not any('systemd-run --unit=jdamr-table-cycle' in c for c in pi.calls)
    assert d.load_state()['last_unit'].startswith('executor:')
    d.cmd_stop(SimpleNamespace())
    assert f'sudo -n systemctl kill --signal=SIGINT {d.EXECUTOR_UNIT}' in pi.calls


def test_onboard_bag_brackets_the_run_and_is_copied(env, monkeypatch):
    pi = ExecutorPi()
    copies = []
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'sh', lambda command, **k: copies.append(command) or
                        SimpleNamespace(returncode=0, stdout=''))
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
    d.run_cycle(go_args(), d.load_state(), 'table_02')
    start = next(i for i, c in enumerate(pi.calls) if 'systemd-run --unit=jdamr-onboard-bag-' in c)
    request = next(i for i, c in enumerate(pi.calls) if c.endswith('.request'))
    assert start < request
    assert all(topic in pi.calls[start] for topic in ('/odom', '/imu/data_raw', '/scan'))
    assert 'RuntimeMaxSec=' in pi.calls[start] and 'MemoryMax=' in pi.calls[start]
    unit = pi.calls[start].split('--unit=')[1].split()[0]
    stop = pi.calls.index(f'sudo -n systemctl stop {unit}')
    assert stop > request
    assert any(c.startswith(f'rsync -a --partial {d.HOST}:') for c in copies)
    assert 'onboard_bag_unit' not in d.load_state()
    d.cmd_stop(SimpleNamespace())
    assert pi.calls.count(f'sudo -n systemctl stop {unit}') == 1


def test_onboard_bag_is_closed_when_the_run_is_interrupted(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)

    def interrupted(_path):
        raise KeyboardInterrupt
    monkeypatch.setattr(d, 'read_events', interrupted)
    with pytest.raises(KeyboardInterrupt):
        d.run_cycle(go_args(), d.load_state(), 'table_02')
    assert any(c.startswith('sudo -n systemctl stop jdamr-onboard-bag-') for c in pi.calls)
    assert 'onboard_bag_unit' not in d.load_state()


HOME_EVENTS = [{'event': 'box_approach_finished'}, {'event': 'parked_dwell_complete'},
               {'event': 'box_escape_finished'},
               {'event': 'home_arrived', 'confirmation': {'confirmed': True}}]


def test_sequence_runs_each_table_from_the_dock_with_an_init_between(env, monkeypatch):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    order = []
    monkeypatch.setattr(d, 'run_cycle', lambda args, state, table: order.append(table) or HOME_EVENTS)
    monkeypatch.setattr(d, 'cmd_init', lambda init_args: order.append(
        ('init', init_args.keep_home, init_args.click, init_args.seed_from_state)))
    d.cmd_go(go_args(table_ids=['table_01', 'table_02'], dock_wait_s=0.0))
    assert order == ['table_01', ('init', True, False, False), 'table_02']


def test_sequence_stops_at_the_first_cycle_that_does_not_dock(env, monkeypatch):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    order = []
    failed = [{'event': 'failed'}]
    monkeypatch.setattr(d, 'run_cycle', lambda args, state, table: order.append(table) or failed)
    monkeypatch.setattr(d, 'cmd_init', lambda init_args: order.append('init'))
    with pytest.raises(SystemExit):
        d.cmd_go(go_args(table_ids=['table_02', 'table_01'], dock_wait_s=0.0))
    assert order == ['table_02']


@pytest.mark.parametrize('extra', [{'skip_via': True}, {'resume_at_observation': True},
                                   {'dock_only': True}, {'resume_parked_log': '/log'}])
def test_sequence_refuses_single_cycle_options(env, monkeypatch, extra):
    monkeypatch.setattr(d, 'pi', Pi())
    monkeypatch.setattr(d, 'run_cycle', lambda *a: pytest.fail('must not depart'))
    with pytest.raises(SystemExit):
        d.cmd_go(go_args(table_ids=['table_01', 'table_02'], **extra))


def test_poll_pi_tells_a_failed_link_from_a_finished_cycle(monkeypatch):
    """ssh exit 255 or a hung ssh is 'unknown', never 'the cycle ended'."""
    import subprocess
    answers = iter([SimpleNamespace(returncode=255, stdout=''),
                    subprocess.TimeoutExpired('ssh', 30),
                    SimpleNamespace(returncode=0, stdout='running\n'),
                    SimpleNamespace(returncode=0, stdout='')])

    def pi(*_args, **_kwargs):
        answer = next(answers)
        if isinstance(answer, BaseException):
            raise answer
        return answer

    monkeypatch.setattr(d, 'pi', pi)
    assert d.poll_pi('test -e result || echo running') is None
    assert d.poll_pi('test -e result || echo running') is None
    assert d.poll_pi('test -e result || echo running').stdout.strip() == 'running'
    assert d.poll_pi('test -e result || echo running').stdout.strip() == ''


def test_operator_call_event_alerts_the_pc(monkeypatch):
    """An operator_call rings, notifies the desktop, and uses the optional extra channel."""
    sent = []
    monkeypatch.setattr(d, 'sh', lambda command, **kwargs: sent.append(command)
                        or SimpleNamespace(returncode=0, stdout=''))
    monkeypatch.setattr(d, 'log', lambda message: sent.append(('log', message)))
    monkeypatch.delenv('JDAMR_OPERATOR_NOTIFY_CMD', raising=False)
    d.show_event({'event': 'interrupted', 'reason': 'scan stale'})
    assert not any(isinstance(item, str) for item in sent)
    d.show_event({'event': 'operator_call', 'level': 'FATAL', 'category': 'emergency_stop',
                  'reason': 'interrupted: emergency stop engaged'})
    assert any(isinstance(item, str) and item.startswith('notify-send') for item in sent)
    assert ('log', 'OPERATOR CALL FATAL emergency_stop: interrupted: emergency stop engaged') \
        in sent
    monkeypatch.setenv('JDAMR_OPERATOR_NOTIFY_CMD', 'echo')
    sent.clear()
    d.alert_operator({'level': 'URGENT', 'category': 'battery', 'reason': 'low'})
    assert any(isinstance(item, str) and item.startswith('echo ') for item in sent)


def test_go_passes_the_mppi_transit_switch_only_when_asked(env, monkeypatch):
    seen = []
    for second, extra in enumerate(({}, {'mppi_transit': True})):
        # One run directory per second; keep the two runs apart.
        monkeypatch.setattr(d.time, 'strftime', lambda _fmt, s=second: f'20261002_11000{s}')
        pi = ExecutorPi()
        monkeypatch.setattr(d, 'pi', pi)
        monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
        d.run_cycle(go_args(**extra), d.load_state(), 'table_02')
        seen.append(json.loads(next(s for s in pi.stdins if s))['argv'])
    assert '--mppi-transit' not in seen[0]
    assert '--mppi-transit' in seen[1]


def test_operator_alerts_can_be_switched_off_for_test_runs(env, monkeypatch):
    sent = []
    monkeypatch.setattr(d, 'sh', lambda command, **k: sent.append(command) or
                        SimpleNamespace(returncode=0, stdout=''))
    event = {'level': 'CRITICAL', 'category': 'path_blocked', 'reason': 'give up'}
    d.cmd_alerts(SimpleNamespace(mode='off'))
    d.alert_operator(event)
    assert sent == []
    d.cmd_alerts(SimpleNamespace(mode='on'))
    d.alert_operator(event)
    assert any(command.startswith('notify-send') for command in sent)
