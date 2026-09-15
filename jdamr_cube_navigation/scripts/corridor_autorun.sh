#!/usr/bin/env bash
# 로봇 안에서 복도 왕복을 처음부터 끝까지 혼자 수행한다.
#
# 복도에서는 무선 연결이 끊길 수 있으므로 시작·기록·정리를 파이에서 수행한다.
# 기록 자체는 파이의 SD 카드에 쌓이므로 데이터는 무선과 무관하다.
# 무선이 필요한 것은 시작·정지·회수뿐이다. 시작을 미리 로봇 안에 넣어두면
# 주행 중 연결이 끊겨도 상관없어진다.
#
# 사용법 (실내에서 예약하고 복도로 가져간다):
#   setsid nohup corridor_autorun.sh --delay 180 > /dev/null 2>&1 &
#
# --delay 동안 로봇을 출발 지점에 놓고 물러선다. 게이트가 하나라도 실패하면
# 주행하지 않는다. 비상 정지는 물리 전원 차단이다.
set -o pipefail

DELAY_S=180
STOP_GRACE_S=25
ROUTE_STOP_GRACE_S=7
METRICS_DURATION_S=2400
METRICS_INTERVAL_S=5
RESOURCE_WINDOW_S=60
METRICS_MAX_AGE_S=12
PREFLIGHT_TIMEOUT_S=180
ROUTE_TIMEOUT_S=1800
EXECUTE=1
WAIT_FOR_START=0
NAVIGATION_PROFILE=corridor
RUN_ID="corridor_autorun_$(date +%Y%m%dT%H%M%S)"
while [ $# -gt 0 ]; do
  case "$1" in
    --delay)
      [ "$#" -ge 2 ] || { echo "--delay 값이 필요하다" >&2; exit 2; }
      case "$2" in --*) echo "--delay 값이 필요하다" >&2; exit 2 ;; esac
      DELAY_S="$2"; shift 2 ;;
    --run-id)
      [ "$#" -ge 2 ] || { echo "--run-id 값이 필요하다" >&2; exit 2; }
      case "$2" in --*) echo "--run-id 값이 필요하다" >&2; exit 2 ;; esac
      RUN_ID="$2"; shift 2 ;;
    --no-execute) EXECUTE=0; shift ;;
    --wait-for-start) WAIT_FOR_START=1; shift ;;
    --navigation-profile)
      [ "$#" -ge 2 ] || { echo "--navigation-profile 값이 필요하다" >&2; exit 2; }
      case "$2" in
        corridor|obstacle_candidate|obstacle_base_candidate)
          NAVIGATION_PROFILE="$2"; shift 2 ;;
        *) echo "주행 프로필이 올바르지 않다" >&2; exit 2 ;;
      esac ;;
    *) echo "알 수 없는 인자: $1" >&2; exit 2 ;;
  esac
done
case "$DELAY_S" in
  ''|*[!0-9]*|?????*)
    echo "--delay는 0~3600 범위의 정수여야 한다" >&2
    exit 2 ;;
esac
if [ "$DELAY_S" -gt 3600 ]; then
  echo "--delay는 0~3600 범위의 정수여야 한다" >&2
  exit 2
fi
case "$RUN_ID" in
  ''|[!A-Za-z0-9]*|*[!A-Za-z0-9._-]*)
    echo "--run-id는 영문·숫자로 시작하고 영문·숫자·점·밑줄·하이픈만 허용한다" >&2
    exit 2 ;;
esac

# 이 실행기는 구차체의 고정 복도 경로·보호영역만 사용한다.
# 새 차체 모델이 설치된 파이에서는 로그·예약·주행을 시작하기 전에 거부한다.
if [ -f "$HOME/jdamr_ws/install/jdamr_cube_description/share/jdamr_cube_description/urdf/new_base_real.urdf" ]; then
  echo "새 차체에서는 기존 복도 자동 실행기를 사용할 수 없다" >&2
  exit 2
fi

A="$HOME/jdamr_artifacts"
mkdir -p "$A"
LOG="$A/$RUN_ID.autorun.log"
ABORT_FILE="$HOME/jdamr_abort"
START_FILE="$HOME/jdamr_start"
say() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$1" | tee -a "$LOG"; }
say "자동 실행기 bootstrap 시작"

WIFI_PID=""
METRICS_PID=""
NAV_PID=""
ROUTE_PID=""
STOPPING=0

source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=12
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
# bringup의 원시 센서 publisher는 LOCALHOST로 유지한다. 이 호스트의 Nav2와
# 기록기만 SUBNET participant로 만들어 현재 Fast DDS의 local user-data 경로를
# 사용한다. 이전 discovery 범위의 CLI daemon은 재사용하지 않는다.
ros2 daemon stop >/dev/null 2>&1 || true
SHARE="$(ros2 pkg prefix jdamr_cube_navigation)/share/jdamr_cube_navigation"
ROUTE="$SHARE/config/corridor_roundtrip.autonomous_20260826.yaml"

group_alive() {
  local pgid="$1"
  ps -eo pgid=,stat= | awk -v pgid="$pgid" '
    $1 == pgid && $2 !~ /^Z/ { found = 1 }
    END { exit(found ? 0 : 1) }
  '
}

stop_group() {
  local pid="$1" label="$2" signal="${3:-TERM}" grace_s="${4:-$STOP_GRACE_S}"
  [ -n "$pid" ] || return 0
  if ! group_alive "$pid"; then
    wait "$pid" 2>/dev/null
    return 0
  fi
  say "$label 종료 요청 (pid=$pid, signal=$signal)"
  kill -"$signal" -- "-$pid" 2>/dev/null || kill -"$signal" "$pid" 2>/dev/null
  for _ in $(seq 1 "$grace_s"); do
    group_alive "$pid" || {
      wait "$pid" 2>/dev/null
      return 0
    }
    sleep 1
  done
  say "$label 정상 종료 시간 초과 — 해당 프로세스 그룹만 강제 종료"
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
  wait "$pid" 2>/dev/null
}

stop_stack() {
  [ "$STOPPING" -eq 0 ] || return 0
  STOPPING=1
  say "정리 시작"
  stop_group "$ROUTE_PID" "경로 실행기" INT "$ROUTE_STOP_GRACE_S"
  # Nav2와 recorder는 같은 launch 프로세스 그룹이다. 부모가 먼저 끝나도
  # 그룹 전체가 사라질 때까지 기다려 recorder에 파일 마감 시간을 준다.
  stop_group "$NAV_PID" "Nav2·기록기"
  stop_group "$METRICS_PID" "자원 계측기"
  if [ -n "$WIFI_PID" ] && kill -0 "$WIFI_PID" 2>/dev/null; then
    kill -TERM "$WIFI_PID" 2>/dev/null
    wait "$WIFI_PID" 2>/dev/null
  fi
  say "정리 완료"
}
trap stop_stack EXIT
trap 'exit 130' INT TERM

say "run_id=$RUN_ID delay=${DELAY_S}s execute=$EXECUTE profile=$NAVIGATION_PROFILE"
say "${DELAY_S}초 안에 로봇을 출발 지점에 놓고 물러설 것. 이후에는 옮기지 말 것."
say "취소하려면 $ABORT_FILE 을 만들거나 전원을 차단할 것."
remaining_s="$DELAY_S"
while [ "$remaining_s" -gt 0 ]; do
  if [ -e "$ABORT_FILE" ]; then
    say "중단 파일 발견. 기동하지 않는다."
    exit 6
  fi
  [ $(( remaining_s % 30 )) -eq 0 ] && say "기동까지 ${remaining_s}초"
  sleep 5
  remaining_s=$(( remaining_s - 5 ))
done
if [ -e "$ABORT_FILE" ]; then
  say "중단 파일 발견. 기동하지 않는다."
  exit 6
fi
if [ "$WAIT_FOR_START" -eq 1 ]; then
  say "출발 신호 대기: $START_FILE"
  while [ ! -e "$START_FILE" ]; do
    if [ -e "$ABORT_FILE" ]; then
      say "중단 파일 발견. 기동하지 않는다."
      exit 6
    fi
    sleep 1
  done
  rm -f -- "$START_FILE"
  say "출발 신호 확인"
fi
if [ -e "$ABORT_FILE" ]; then
  say "중단 파일 발견. 기동하지 않는다."
  exit 6
fi
say "배치 완료로 간주하고 현 위치에서 계측·위치추정·경로 검증을 시작한다."

# 무선 세기를 계속 남긴다. 끊겨도 파일은 로봇 안에 남으므로 복도 어디가
# 약한지 나중에 확인할 수 있다.
( while true; do
    printf '%s\t%s\n' "$(date +%H:%M:%S)" \
      "$(awk 'NR==3{print $3, $4}' /proc/net/wireless 2>/dev/null)"
    sleep 2
  done ) > "$A/$RUN_ID.wifi.tsv" 2>/dev/null &
WIFI_PID=$!

setsid nohup ros2 run jdamr_cube_navigation soak_metrics \
  --output "$A/$RUN_ID.per_process.tsv" \
  --duration "$METRICS_DURATION_S" --interval "$METRICS_INTERVAL_S" \
  --cores "$(nproc)" \
  > "$A/$RUN_ID.metrics.log" 2>&1 &
METRICS_PID=$!

say "Nav2 와 기록 기동"
setsid nohup ros2 launch jdamr_cube_navigation onboard_keepout_navigation.launch.py \
  navigation_profile:="$NAVIGATION_PROFILE" \
  bag_output:="$A/$RUN_ID" > "$A/$RUN_ID.launch.log" 2>&1 &
NAV_PID=$!

say "lifecycle 활성화 대기 (최대 180초)"
for _ in $(seq 1 36); do
  sleep 5
  n=$(grep -c 'Managed nodes are active' "$A/$RUN_ID.launch.log" 2>/dev/null) || n=0
  [ "$n" -ge 3 ] && break
done
if [ "${n:-0}" -lt 3 ]; then
  say "실패: lifecycle 매니저 $n/3 만 활성화. 주행하지 않는다."
  exit 3
fi
say "lifecycle 3/3 활성화"

say "사전점검"
if bash "$SHARE/scripts/corridor_preflight.sh" >> "$LOG" 2>&1; then
  say "사전점검 PASS"
else
  say "실패: 사전점검 FAIL. 주행하지 않는다."
  exit 4
fi

say "전체 경로 계획 확인 (이동 없음)"
if timeout "$PREFLIGHT_TIMEOUT_S" ros2 run jdamr_cube_navigation \
  corridor_route --route "$ROUTE" \
  --navigation-profile "$NAVIGATION_PROFILE" >> "$LOG" 2>&1; then
  say "계획 PASS"
else
  say "실패: 전체 경로 계획 실패. 주행하지 않는다."
  exit 5
fi

if ! group_alive "$METRICS_PID"; then
  say "실패: 자원 계측기가 종료됨. 주행하지 않는다."
  exit 7
fi
METRICS_MTIME=$(stat -c %Y "$A/$RUN_ID.per_process.tsv" 2>/dev/null) || {
  say "실패: 자원 계측 파일이 없음. 주행하지 않는다."
  exit 7
}
METRICS_AGE_S=$(( $(date +%s) - METRICS_MTIME ))
if [ "$METRICS_AGE_S" -lt 0 ] || [ "$METRICS_AGE_S" -gt "$METRICS_MAX_AGE_S" ]; then
  say "실패: 자원 계측 표본이 ${METRICS_AGE_S}초 전 값임. 주행하지 않는다."
  exit 7
fi
say "기동 중 누적한 최신 ${RESOURCE_WINDOW_S}초 자원 계측 확인"
if ros2 run jdamr_cube_navigation soak_metrics \
  --evaluate "$A/$RUN_ID.per_process.tsv" \
  --window-seconds "$RESOURCE_WINDOW_S" \
  --cores "$(nproc)" >> "$LOG" 2>&1; then
  say "자원 게이트 PASS"
else
  say "실패: 자원 게이트 FAIL. 주행하지 않는다."
  exit 7
fi

if [ "$EXECUTE" -eq 0 ]; then
  say "--no-execute 지정. 여기서 멈춘다."
  exit 0
fi

if [ -e "$ABORT_FILE" ]; then
  say "중단 파일 발견. 주행하지 않는다."
  exit 6
fi
say "출발"
setsid timeout --kill-after="${ROUTE_STOP_GRACE_S}s" "$ROUTE_TIMEOUT_S" \
  ros2 run jdamr_cube_navigation corridor_route \
  --route "$ROUTE" --navigation-profile "$NAVIGATION_PROFILE" \
  --execute >> "$A/$RUN_ID.route.log" 2>&1 &
ROUTE_PID=$!
while group_alive "$ROUTE_PID"; do
  if [ -e "$ABORT_FILE" ]; then
    say "주행 중 중단 파일 발견"
    stop_group "$ROUTE_PID" "경로 실행기" INT "$ROUTE_STOP_GRACE_S"
    ROUTE_PID=""
    RC=130
    break
  fi
  sleep 1
done
if [ -n "$ROUTE_PID" ]; then
  wait "$ROUTE_PID"
  RC=$?
fi
ROUTE_PID=""
say "주행 종료코드=$RC"
say "기록: $A/$RUN_ID"
exit "$RC"
