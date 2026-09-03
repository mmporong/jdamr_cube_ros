#!/usr/bin/env bash
# 로봇 안에서 복도 왕복을 처음부터 끝까지 혼자 수행한다.
#
# 2026-09-03 에 복도에서 무선이 끊겨 출발 명령이 로봇에 도달하지 못했다.
# 기록 자체는 원래 파이의 SD 카드에 쌓이므로 데이터는 무선과 무관하다.
# 무선이 필요한 것은 시작·정지·회수뿐이다. 시작을 미리 로봇 안에 넣어두면
# 주행 중 연결이 끊겨도 상관없어진다.
#
# 사용법 (실내에서 예약하고 복도로 가져간다):
#   setsid nohup corridor_autorun.sh --delay 180 > /dev/null 2>&1 &
#
# --delay 동안 로봇을 출발 지점에 놓고 물러선다. 게이트가 하나라도 실패하면
# 주행하지 않는다. 비상 정지는 물리 전원 차단이다.
set -uo pipefail

DELAY=180
EXECUTE=1
RUN_ID="corridor_autorun_$(date +%Y%m%dT%H%M%S)"
while [ $# -gt 0 ]; do
  case "$1" in
    --delay)      DELAY="$2"; shift 2 ;;
    --run-id)     RUN_ID="$2"; shift 2 ;;
    --no-execute) EXECUTE=0; shift ;;
    *) echo "알 수 없는 인자: $1" >&2; exit 2 ;;
  esac
done

A="$HOME/jdamr_artifacts"
mkdir -p "$A"
LOG="$A/$RUN_ID.autorun.log"
ABORT_FILE="$HOME/jdamr_abort"
say() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$1" | tee -a "$LOG"; }

source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_ws/install/setup.bash"
export ROS_DOMAIN_ID=12
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
SHARE="$(ros2 pkg prefix jdamr_cube_navigation)/share/jdamr_cube_navigation"
ROUTE="$SHARE/config/corridor_roundtrip.autonomous_20260826.yaml"

stop_stack() {
  say "정리 시작"
  pkill -INT -f '[c]orridor_route' 2>/dev/null
  sleep 3
  pkill -INT -f 'onboard_keepout_navigation' 2>/dev/null
  sleep 10
  for pat in 'lib/nav2_' 'bag record' '[s]oak_metrics.py' '[w]ifi_log'; do
    pkill -f "$pat" 2>/dev/null
  done
  sleep 3
  say "잔류 프로세스: $(pgrep -cf 'lib/nav2_|bag record' 2>/dev/null || echo 0)"
}
trap stop_stack EXIT

say "run_id=$RUN_ID delay=${DELAY}s execute=$EXECUTE"

# 무선 세기를 계속 남긴다. 끊겨도 파일은 로봇 안에 남으므로 복도 어디가
# 약한지 나중에 확인할 수 있다.
( while true; do
    printf '%s\t%s\n' "$(date +%H:%M:%S)" \
      "$(awk 'NR==3{print $3, $4}' /proc/net/wireless 2>/dev/null)"
    sleep 2
  done ) > "$A/$RUN_ID.wifi.tsv" 2>/dev/null &
WIFI_PID=$!

setsid nohup python3 \
  "$HOME/jdamr_ws/src/jdamr_cube_ros/jdamr_cube_navigation/jdamr_cube_navigation/soak_metrics.py" \
  --output "$A/$RUN_ID.per_process.tsv" --duration 2400 --interval 5 \
  > "$A/$RUN_ID.metrics.log" 2>&1 &

say "Nav2 와 기록 기동"
setsid nohup ros2 launch jdamr_cube_navigation onboard_keepout_navigation.launch.py \
  bag_output:="$A/$RUN_ID" > "$A/$RUN_ID.launch.log" 2>&1 &

say "lifecycle 활성화 대기 (최대 180초)"
for _ in $(seq 1 36); do
  sleep 5
  n=$(grep -c 'Managed nodes are active' "$A/$RUN_ID.launch.log" 2>/dev/null || echo 0)
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
if timeout 180 ros2 run jdamr_cube_navigation corridor_route --route "$ROUTE" >> "$LOG" 2>&1; then
  say "계획 PASS"
else
  say "실패: 전체 경로 계획 실패. 주행하지 않는다."
  exit 5
fi

if [ "$EXECUTE" -eq 0 ]; then
  say "--no-execute 지정. 여기서 멈춘다."
  exit 0
fi

say "${DELAY}초 뒤 출발한다. 로봇을 출발 지점에 놓고 물러설 것."
say "취소하려면 $ABORT_FILE 을 만들거나 전원을 차단할 것."
remaining="$DELAY"
while [ "$remaining" -gt 0 ]; do
  if [ -e "$ABORT_FILE" ]; then
    say "중단 파일 발견. 주행하지 않는다."
    exit 6
  fi
  [ $(( remaining % 30 )) -eq 0 ] && say "출발까지 ${remaining}초"
  sleep 5
  remaining=$(( remaining - 5 ))
done

say "출발"
timeout 1800 ros2 run jdamr_cube_navigation corridor_route --route "$ROUTE" --execute \
  >> "$A/$RUN_ID.route.log" 2>&1
RC=$?
say "주행 종료코드=$RC"
say "기록: $A/$RUN_ID"
exit "$RC"
