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
                resume_at_observation=False, resume_parked_log=None, dock_only=False, rpp_final=False)
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
        (a.keep_home, a.seed_from_state, a.local_only)))
    monkeypatch.setattr(d, 'dds_health', lambda: ('ok', 'PROBE health ok {} 1.0 s'))
    d.cmd_recover(SimpleNamespace(seed=[1.0, 2.0, 90.0], local_only=False))
    assert seen == [(True, False, False)]
    assert json.loads(d.CLICK.read_text())['x_m'] == 1.0


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
