#!/bin/bash
# 병렬 시연 수집 — 격리된 시뮬 N개를 동시에 돌려 처리량을 N배로.
#
# gz-sim 물리는 단일 스레드라 인스턴스 하나가 실시간(RTF 1.0)이 상한이고
# (물리 스텝 4ms 이상은 접촉이 발산해 물체가 1m 튕긴다 — 실측), 코어는 16개
# 남는다. 그래서 배속 대신 인스턴스 수로 처리량을 얻는다.
#
# 격리: GZ_PARTITION(gz transport) + ROS_DOMAIN_ID(DDS) 두 축을 워커마다 달리한다.
#       하나만 바꾸면 토픽이 섞여 로봇이 남의 명령을 받는다.
#
# 사용: parallel_collect.sh [총_에피소드=20] [워커수=4]
set -o pipefail
TOTAL=${1:-20}
WORKERS=${2:-4}
TOOLS="$(cd "$(dirname "$0")" && pwd)"
LOGD="$TOOLS/logs/parallel"
mkdir -p "$LOGD"

PER=$(( (TOTAL + WORKERS - 1) / WORKERS ))
echo "병렬 수집: 총 ${TOTAL}편 / 워커 ${WORKERS}개 (워커당 ~${PER}편)"

for w in $(seq 0 $((WORKERS - 1))); do
  DOMAIN=$((40 + w))
  PART="capstone-w${w}"
  START=$((w * PER))
  END=$((START + PER - 1))
  [ $END -ge $TOTAL ] && END=$((TOTAL - 1))
  [ $START -gt $END ] && continue

  # 워커별 시뮬 (헤드리스). systemd 유닛으로 띄워 세션과 무관하게 산다.
  systemctl --user stop "capstone-sim-w${w}" 2>/dev/null
  systemd-run --user --collect \
    --setenv=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    --setenv=GZ_PARTITION="$PART" --setenv=ROS_DOMAIN_ID="$DOMAIN" \
    --unit="capstone-sim-w${w}" bash -c \
    'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && exec ros2 launch jdamr_cube_gazebo gazebo.launch.py gui:=false world:=$HOME/jdamr_cube_ws/install/jdamr_cube_gazebo/share/jdamr_cube_gazebo/worlds/room.world' \
    >/dev/null 2>&1
  echo "  워커 $w: 시뮬 기동 (partition=$PART domain=$DOMAIN, ep ${START}~${END})"
done

# 컨트롤러 3개가 모두 뜰 때까지 워커별로 대기
for w in $(seq 0 $((WORKERS - 1))); do
  DOMAIN=$((40 + w))
  for i in $(seq 1 40); do
    N=$(ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION="capstone-w${w}" ROS_DOMAIN_ID=$DOMAIN \
        bash -c 'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; source ~/jdamr_cube_ws/install/setup.bash >/dev/null 2>&1; timeout 10 ros2 control list_controllers 2>/dev/null | grep -c active')
    [ "${N:-0}" = "3" ] && break
    sleep 5
  done
  echo "  워커 $w: 준비 완료 (컨트롤러 ${N:-0}개)"
done

# 수집 실행 (워커별 백그라운드)
PIDS=()
for w in $(seq 0 $((WORKERS - 1))); do
  DOMAIN=$((40 + w))
  START=$((w * PER))
  END=$((START + PER - 1))
  [ $END -ge $TOTAL ] && END=$((TOTAL - 1))
  [ $START -gt $END ] && continue
  (
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export GZ_PARTITION="capstone-w${w}" ROS_DOMAIN_ID=$DOMAIN
    source /opt/ros/jazzy/setup.bash
    source ~/jdamr_cube_ws/install/setup.bash
    python3 -u "$TOOLS/rule_collect.py" --episodes $((END + 1)) --start $START
  ) > "$LOGD/w${w}.log" 2>&1 &
  PIDS+=($!)
done

echo "수집 시작 — 진행: tail -f $LOGD/w*.log"
FAIL=0
for p in "${PIDS[@]}"; do
  wait "$p" || FAIL=1
done

for w in $(seq 0 $((WORKERS - 1))); do
  systemctl --user stop "capstone-sim-w${w}" 2>/dev/null
done
echo "== 병렬 수집 종료 $(date +%H:%M:%S) (시뮬 전부 정지) =="
grep -hE "^\[ep" "$LOGD"/w*.log | sort -t p -k2 -n
exit $FAIL
