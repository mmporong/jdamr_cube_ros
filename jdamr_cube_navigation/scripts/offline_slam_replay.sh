#!/usr/bin/env bash
# 기록한 bag 을 격리 도메인에서 재생해 2D SLAM 백엔드 하나를 돌린다.
#
# 재생에서 저장 지도, 이동 명령, 기록된 AMCL map->odom 을 제외한다. 그래야 새
# backend 하나만 TF 권한자가 되고, 결과가 기존 지도의 복사본이 되지 않는다.
# 제외는 offline_replay_guard.launch.py 가 담당하며 물리 도메인 12 를 거부한다.
set -uo pipefail

BAG=""; BACKEND=""; OUT=""; RATE="1.0"; DURATION="-1.0"
while [ $# -gt 0 ]; do
  case "$1" in
    --bag)      BAG="$2"; shift 2 ;;
    --backend)  BACKEND="$2"; shift 2 ;;
    --out)      OUT="$2"; shift 2 ;;
    --rate)     RATE="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    *) echo "알 수 없는 인자: $1" >&2; exit 2 ;;
  esac
done
[ -n "$BAG" ] && [ -n "$BACKEND" ] && [ -n "$OUT" ] || {
  echo "사용법: $0 --bag DIR --backend {cartographer|slam_toolbox} --out DIR" >&2
  exit 2; }
[ -d "$BAG" ] || { echo "bag 디렉터리가 없다: $BAG" >&2; exit 2; }

export ROS_DOMAIN_ID="${OFFLINE_DOMAIN_ID:-199}"
export ROS_LOCALHOST_ONLY=1
[ "$ROS_DOMAIN_ID" = "12" ] && { echo "물리 도메인 12 에서는 재생하지 않는다" >&2; exit 2; }
mkdir -p "$OUT"
RUN_ID="$(basename "$BAG")__${BACKEND}"
LOG="$OUT/${RUN_ID}.log"

cleanup() {
  for pat in cartographer_node cartographer_occupancy async_slam_toolbox \
             tf_replay_filter 'bag play' 'bag record'; do
    pkill -f "$pat" 2>/dev/null
  done
  sleep 2
}
trap cleanup EXIT

echo "run=$RUN_ID domain=$ROS_DOMAIN_ID rate=$RATE" | tee "$LOG"
START=$(date +%s.%N)

case "$BACKEND" in
  cartographer)
    CFG="$(ros2 pkg prefix jdamr_cube_cartographer)/share/jdamr_cube_cartographer/config"
    # gflags 인자는 --ros-args 앞에 와야 한다. 뒤에 두면 cartographer_node 가
    # -configuration_directory is missing 으로 즉시 죽는다.
    setsid nohup ros2 run cartographer_ros cartographer_node \
      -configuration_directory "$CFG" \
      -configuration_basename jdamr_cube_2d_corridor.lua \
      --ros-args -p use_sim_time:=true >> "$LOG" 2>&1 &
    setsid nohup ros2 run cartographer_ros cartographer_occupancy_grid_node \
      -resolution 0.05 -publish_period_sec 1.0 \
      --ros-args -p use_sim_time:=true >> "$LOG" 2>&1 &
    ;;
  slam_toolbox)
    # Jazzy 의 async_slam_toolbox_node 는 lifecycle 노드다. ros2 run 으로
    # 띄우면 unconfigured 상태로 남아 스캔을 처리하지 않고 /map 도 내지
    # 않는다. 2026-09-03 첫 실행이 정확히 그렇게 조용히 실패했다.
    # upstream launch 가 configure/activate 를 수행하므로 그대로 쓴다.
    PARAMS="$(ros2 pkg prefix jdamr_cube_navigation)/share/jdamr_cube_navigation/config/slam_toolbox_corridor.yaml"
    setsid nohup ros2 launch slam_toolbox online_async_launch.py \
      use_sim_time:=true slam_params_file:="$PARAMS" \
      >> "$LOG" 2>&1 &
    ;;
  *) echo "알 수 없는 backend: $BACKEND" >&2; exit 2 ;;
esac
sleep 10
case "$BACKEND" in
  cartographer)  probe='cartographer_node' ;;
  slam_toolbox)  probe='async_slam_toolbox_node' ;;
esac
if ! pgrep -f "$probe" >/dev/null; then
  echo "backend($probe) 기동 실패 - 로그 확인" | tee -a "$LOG"
  tail -20 "$LOG"
  exit 3
fi
echo "backend 기동 확인: $probe" | tee -a "$LOG"

# backend 의 추정 궤적과 지도를 남긴다. 원본 센서는 이미 입력 bag 에 있다.
setsid nohup ros2 bag record --disable-keyboard-controls --storage mcap \
  --output "$OUT/${RUN_ID}_result" \
  --topics /tf /tf_static /map >> "$LOG" 2>&1 &
sleep 4

# 재생을 배경으로 돌리고 진행 증거를 감시한다.
#
# 2026-09-03 에 같은 모양의 실패가 세 번 났다. slam_toolbox 는 303초 내내
# 아무것도 만들지 않았고, cartographer 는 15분짜리 재생 도중 죽었으며, 둘 다
# 끝난 뒤 로그를 읽고서야 알았다. 증거 없이 도는 시간을 없앤다.
setsid nohup ros2 launch jdamr_cube_navigation offline_replay_guard.launch.py \
  bag:="$BAG" rate:="$RATE" duration:="$DURATION" \
  offline_domain_id:="$ROS_DOMAIN_ID" >> "$LOG" 2>&1 &
REPLAY_PID=$!

RESULT_DIR="$OUT/${RUN_ID}_result"
# slam_toolbox 는 첫 지도까지 실측 약 3분이 걸린다. 120초 제한은 정상 런을
# 죽인다. 진짜 죽음은 아래 두 검사(프로세스 생존, 기록 정체)가 잡으므로 이
# 제한은 "영원히 안 나오는" 경우만 걸러내도록 넉넉히 둔다.
FIRST_MAP_DEADLINE=$(( $(date +%s) + 420 ))
STALL_LIMIT=90
last_size=0
last_progress=$(date +%s)
saw_map=0
RC=0

while kill -0 "$REPLAY_PID" 2>/dev/null; do
  sleep 10
  if ! pgrep -f "$probe" >/dev/null; then
    echo "감시: backend($probe) 가 죽었다 - 재생을 중단한다" | tee -a "$LOG"
    RC=4; break
  fi
  size=$(du -sb "$RESULT_DIR" 2>/dev/null | cut -f1)
  size=${size:-0}
  now=$(date +%s)
  if [ "$size" -gt "$last_size" ]; then
    last_size="$size"; last_progress="$now"
  elif [ $(( now - last_progress )) -gt "$STALL_LIMIT" ]; then
    echo "감시: 결과 기록이 ${STALL_LIMIT}초 동안 늘지 않았다 - 중단한다" \
      | tee -a "$LOG"
    RC=5; break
  fi
  if [ "$saw_map" -eq 0 ]; then
    if timeout 8 ros2 topic info /map 2>/dev/null \
         | grep -q 'Publisher count: [1-9]'; then
      saw_map=1
      echo "감시: /map 발행 확인 ($(date +%H:%M:%S))" | tee -a "$LOG"
    elif [ "$now" -gt "$FIRST_MAP_DEADLINE" ]; then
      echo "감시: 420초 안에 /map 이 나오지 않았다 - backend 가 입력을 처리하지 못한다" \
        | tee -a "$LOG"
      RC=6; break
    fi
  fi
done

if [ "$RC" -eq 0 ]; then
  wait "$REPLAY_PID"; RC=$?
else
  kill -INT "$REPLAY_PID" 2>/dev/null
  sleep 5
fi

sleep 6
ros2 run nav2_map_server map_saver_cli -f "$OUT/${RUN_ID}_map" >> "$LOG" 2>&1
END=$(date +%s.%N)
printf 'wall_seconds=%.1f\n' "$(echo "$END - $START" | bc)" | tee -a "$LOG"
echo "replay_exit=$RC" | tee -a "$LOG"
ls -la "$OUT" | tee -a "$LOG"
exit "$RC"
