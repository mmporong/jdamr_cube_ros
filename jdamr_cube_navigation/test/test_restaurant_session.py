"""Shell-mocked contract tests for the Nav2-only restaurant session wrapper."""

import os
from pathlib import Path
import subprocess

import pytest


PACKAGE = Path(__file__).parents[1]
SCRIPT = PACKAGE / 'scripts/restaurant_session.sh'


def _executable(path, body):
    path.write_text('#!/usr/bin/env bash\n' + body, encoding='utf-8')
    path.chmod(0o755)


@pytest.fixture
def session_env(tmp_path):
    home = tmp_path / 'home'
    workspace = home / 'jdamr_ws'
    bin_dir = tmp_path / 'bin'
    install = workspace / 'install'
    share = install / 'jdamr_cube_navigation/share/jdamr_cube_navigation'
    registry = home / 'registry.yaml'
    ros_setup = tmp_path / 'ros_setup.bash'
    log = tmp_path / 'commands.log'
    bin_dir.mkdir()
    (share / 'config').mkdir(parents=True)
    registry.parent.mkdir(parents=True, exist_ok=True)
    registry.write_text('schema_version: 1\n', encoding='utf-8')
    params = share / 'config/new_base_nav2_params.yaml'
    params.write_text('controller_server: {}\n', encoding='utf-8')
    ros_setup.write_text(
        'test -z "$MOCK_SETUP_UNDEFINED"\nexport MOCK_ROS_SETUP=1\n',
        encoding='utf-8',
    )
    install.mkdir(exist_ok=True)
    (install / 'setup.bash').write_text(
        'export MOCK_OVERLAY_SETUP=1\n', encoding='utf-8')

    _executable(bin_dir / 'systemctl', r"""
printf 'systemctl %s\n' "$*" >> "$MOCK_LOG"
if [ "$1" = status ]; then exit "${MOCK_STATUS_RC:-0}"; fi
if [ "$1" = stop ]; then exit 0; fi
if [ "$1" = is-active ]; then
  service="${@: -1}"
  case " ${MOCK_ACTIVE_SERVICES:-jdamr-base.service} " in
    *" $service "*) exit 0 ;;
    *) exit 3 ;;
  esac
fi
exit 0
""")
    _executable(bin_dir / 'ros2', r"""
printf 'ros2 %s\n' "$*" >> "$MOCK_LOG"
if [ "$1 $2 $3" = "node list --no-daemon" ]; then
  printf '%b' "${MOCK_NODES:-/robot_state_publisher\\n/base_driver\\n}"
elif [ "$1 $2 $3" = "topic list --no-daemon" ]; then
  printf '%b' "${MOCK_TOPICS:-/scan\\n/odom\\n/tf\\n}"
elif [ "$1" = launch ]; then
  printf 'launch-env %s %s %s %s\n' "$ROS_DOMAIN_ID" \
    "$ROS_AUTOMATIC_DISCOVERY_RANGE" "$FASTDDS_BUILTIN_TRANSPORTS" \
    "${MOCK_OVERLAY_SETUP:-}" >> "$MOCK_LOG"
fi
""")
    _executable(bin_dir / 'python3', r"""
printf 'python3 %s\n' "$*" >> "$MOCK_LOG"
exit "${MOCK_REGISTRY_RC:-0}"
""")
    _executable(bin_dir / 'systemd-run', r"""
printf 'systemd-run %s\n' "$*" >> "$MOCK_LOG"
exit "${MOCK_SYSTEMD_RUN_RC:-0}"
""")
    _executable(bin_dir / 'sudo', r"""
printf 'sudo %s\n' "$*" >> "$MOCK_LOG"
[ "$1" = -n ] && shift
exec "$@"
""")

    env = os.environ.copy()
    env.update({
        'HOME': str(home),
        'PATH': f'{bin_dir}:{env["PATH"]}',
        'JDAMR_ROS_SETUP': str(ros_setup),
        'MOCK_LOG': str(log),
    })
    return {
        'env': env, 'home': home, 'workspace': workspace,
        'registry': registry, 'params': params, 'log': log,
    }


def _run(session_env, *args, **updates):
    env = session_env['env'].copy()
    env.update(updates)
    return subprocess.run(
        ['/bin/bash', str(SCRIPT), *map(str, args)], env=env,
        text=True, capture_output=True, check=False,
    )


def _start(session_env, **updates):
    return _run(
        session_env, 'start', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], **updates,
    )


def test_help_states_nav2_only_and_readiness_boundary(session_env):
    result = _run(session_env, '--help')
    assert result.returncode == 0
    assert 'Nav2 서버만 시작' in result.stdout
    assert '초기 pose, NavigateToPose, FollowPath' in result.stdout
    assert '준비 완료를 뜻하지 않는다' in result.stdout
    assert not session_env['log'].exists()


def test_start_uses_singleton_unit_and_non_composed_nav2(session_env):
    result = _start(session_env)
    assert result.returncode == 0, result.stderr
    assert '기동 요청을 수락했다' in result.stdout
    assert '주행 준비 완료가 아니다' in result.stdout
    commands = session_env['log'].read_text(encoding='utf-8')
    assert '--unit=jdamr-restaurant-navigation.service --collect' in commands
    assert '__run' in commands
    assert '--workspace ' + str(session_env['workspace']) in commands
    assert 'NavigateToPose' not in commands
    assert 'FollowPath' not in commands


def test_internal_run_sources_overlay_and_launches_servers_without_action(session_env):
    result = _run(
        session_env, '__run', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], JDAMR_RESTAURANT_INTERNAL='1',
    )
    assert result.returncode == 0, result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'ros2 launch jdamr_cube_navigation restaurant_service.launch.py' in commands
    assert 'use_composition:=false' in commands
    assert 'coordinated_startup:=true' in commands
    assert 'use_box_observer:=false' in commands
    assert 'discovery_range:=SUBNET' in commands
    assert 'launch-env 12 SUBNET UDPv4 1' in commands
    assert 'initial_pose' not in commands


@pytest.mark.parametrize(
    ('updates', 'message'), [
        ({'MOCK_ACTIVE_SERVICES':
          'jdamr-base.service jdamr-cartographer-session.service'},
         'jdamr-cartographer-session.service 실행 중'),
        ({'MOCK_NODES': '/robot_state_publisher\\n/amcl\\n'},
         '기존 localization/Cartographer/Nav2 노드'),
        ({'MOCK_TOPICS': '/scan\\n/tf\\n'},
         '센서 진단 실패'),
    ],
)
def test_start_fails_closed_without_stopping_conflicts(
        session_env, updates, message):
    result = _start(session_env, **updates)
    assert result.returncode == 4
    assert message in result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemctl stop jdamr-cartographer-session.service' not in commands
    assert 'systemd-run ' not in commands


def test_start_requires_active_base_service(session_env):
    result = _start(session_env, MOCK_ACTIVE_SERVICES='none.service')
    assert result.returncode == 4
    assert 'jdamr-base.service가 active가 아니다' in result.stderr


def test_start_rejects_existing_singleton_unit(session_env):
    result = _start(
        session_env,
        MOCK_ACTIVE_SERVICES=(
            'jdamr-base.service jdamr-restaurant-navigation.service'),
    )
    assert result.returncode == 4
    assert 'singleton 세션을 중복 시작하지 않는다' in result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands


def test_registry_validation_failure_prevents_start(session_env):
    result = _start(session_env, MOCK_REGISTRY_RC='1')
    assert result.returncode == 4
    assert (
        'registry 스키마·지도·keepout 경로 또는 해시 검증 실패'
        in result.stderr
    )
    assert 'systemd-run ' not in session_env['log'].read_text(encoding='utf-8')


def test_overlay_is_required_and_checked_before_ros_queries(session_env):
    (session_env['workspace'] / 'install/setup.bash').unlink()
    result = _start(session_env)
    assert result.returncode == 4
    assert 'workspace overlay를 읽을 수 없다' in result.stderr
    assert not session_env['log'].exists()


def test_overlay_source_failure_is_reported(session_env):
    (session_env['workspace'] / 'install/setup.bash').write_text(
        'return 1\n', encoding='utf-8')
    result = _start(session_env)
    assert result.returncode == 4
    assert 'workspace overlay 적용 실패' in result.stderr
    assert not session_env['log'].exists()


def test_status_and_stop_target_only_navigation_unit(session_env):
    status = _run(session_env, 'status')
    stop = _run(session_env, 'stop')
    assert status.returncode == 0
    assert '준비 완료 판정이 아니다' in status.stdout
    assert stop.returncode == 0
    assert '베이스·센서 서비스는 변경하지 않았다' in stop.stdout
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemctl status --no-pager jdamr-restaurant-navigation.service' in commands
    assert 'systemctl stop jdamr-restaurant-navigation.service' in commands
    assert 'systemctl stop jdamr-base.service' not in commands
