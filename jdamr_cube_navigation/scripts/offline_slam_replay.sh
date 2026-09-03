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
    setsid nohup ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
      -p use_sim_time:=true -p base_frame:=base_footprint -p odom_frame:=odom \
      -p map_frame:=map -p scan_topic:=/scan -p mode:=mapping \
      -p resolution:=0.05 -p max_laser_range:=8.0 \
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

ros2 launch jdamr_cube_navigation offline_replay_guard.launch.py \
  bag:="$BAG" rate:="$RATE" duration:="$DURATION" \
  offline_domain_id:="$ROS_DOMAIN_ID" >> "$LOG" 2>&1
RC=$?

sleep 6
ros2 run nav2_map_server map_saver_cli -f "$OUT/${RUN_ID}_map" >> "$LOG" 2>&1
END=$(date +%s.%N)
printf 'wall_seconds=%.1f\n' "$(echo "$END - $START" | bc)" | tee -a "$LOG"
echo "replay_exit=$RC" | tee -a "$LOG"
ls -la "$OUT" | tee -a "$LOG"
exit "$RC"
