#!/usr/bin/env bash
# 기록한 bag 을 격리 도메인에서 재생해 2D SLAM 백엔드 하나를 돌린다.
#
# 재생에서 저장 지도, 이동 명령, 기록된 AMCL map->odom 을 제외한다. 그래야 새
# backend 하나만 TF 권한자가 되고, 결과가 기존 지도의 복사본이 되지 않는다.
# 제외는 offline_replay_guard.launch.py 가 담당하며 물리 도메인 12 를 거부한다.
set -uo pipefail

BAG=""; BACKEND=""; OUT=""; RATE="1.0"; DURATION="-1.0"
RUN_LABEL=""; SLAM_PARAMS=""
CARTOGRAPHER_CONFIG=""
STALL_LIMIT="${STALL_LIMIT:-240}"
CARTOGRAPHER_SHUTDOWN_TIMEOUT="${CARTOGRAPHER_SHUTDOWN_TIMEOUT:-90}"
while [ $# -gt 0 ]; do
  case "$1" in
    --bag)      BAG="$2"; shift 2 ;;
    --backend)  BACKEND="$2"; shift 2 ;;
    --out)      OUT="$2"; shift 2 ;;
    --rate)     RATE="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --label)    RUN_LABEL="$2"; shift 2 ;;
    --slam-params) SLAM_PARAMS="$2"; shift 2 ;;
    --cartographer-config) CARTOGRAPHER_CONFIG="$2"; shift 2 ;;
    *) echo "알 수 없는 인자: $1" >&2; exit 2 ;;
  esac
done
[ -n "$BAG" ] && [ -n "$BACKEND" ] && [ -n "$OUT" ] || {
  echo "사용법: $0 --bag DIR --backend {cartographer|slam_toolbox} --out DIR" >&2
  exit 2; }
[ -d "$BAG" ] || { echo "bag 디렉터리가 없다: $BAG" >&2; exit 2; }
[[ "$STALL_LIMIT" =~ ^[1-9][0-9]*$ ]] || {
  echo "STALL_LIMIT은 양의 정수여야 한다: $STALL_LIMIT" >&2; exit 2; }
[[ "$CARTOGRAPHER_SHUTDOWN_TIMEOUT" =~ ^[1-9][0-9]*$ ]] || {
  echo "CARTOGRAPHER_SHUTDOWN_TIMEOUT은 양의 정수여야 한다" >&2; exit 2; }
[ -n "$RUN_LABEL" ] || RUN_LABEL="$BACKEND"
[[ "$RUN_LABEL" =~ ^[a-z0-9_]+$ ]] || {
  echo "label은 소문자·숫자·밑줄만 허용한다: $RUN_LABEL" >&2; exit 2; }
if [ -n "$SLAM_PARAMS" ] && [ "$BACKEND" != "slam_toolbox" ]; then
  echo "--slam-params는 slam_toolbox에서만 사용한다" >&2
  exit 2
fi
if [ -n "$CARTOGRAPHER_CONFIG" ] && [ "$BACKEND" != "cartographer" ]; then
  echo "--cartographer-config는 cartographer에서만 사용한다" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${OFFLINE_DOMAIN_ID:-199}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
[ "$ROS_DOMAIN_ID" = "12" ] && { echo "물리 도메인 12 에서는 재생하지 않는다" >&2; exit 2; }
mkdir -p "$OUT"
RUN_ID="$(basename "$BAG")__${RUN_LABEL}"
LOG="$OUT/${RUN_ID}.log"
RESULT_DIR="$OUT/${RUN_ID}_result"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export JDAMR_REPLAY_RUN_ID="$RUN_ID"
WRITER_CONFIG="$(ros2 pkg prefix jdamr_cube_navigation)/share/jdamr_cube_navigation/evaluation/mcap_writer_options.yaml"
[ -f "$WRITER_CONFIG" ] || {
  echo "MCAP writer 설정이 없다: $WRITER_CONFIG" >&2; exit 2; }
[ ! -e "$RESULT_DIR" ] || {
  echo "결과 디렉터리가 이미 있다: $RESULT_DIR" >&2; exit 2; }

PROCESS_GROUPS=()
SAMPLE_PID=""
register_group() {
  local candidate="$1" process_group
  for process_group in "${PROCESS_GROUPS[@]}"; do
    [ "$process_group" = "$candidate" ] && return
  done
  PROCESS_GROUPS+=("$candidate")
}
stop_group() {
  local process_group="$1"
  kill -TERM -- "-$process_group" 2>/dev/null || true
}
wait_pid_bounded() {
  local pid="$1" timeout_s="$2" deadline state
  deadline=$(( $(date +%s) + timeout_s ))
  while kill -0 "$pid" 2>/dev/null; do
    state=$(ps -o stat= -p "$pid" 2>/dev/null | xargs)
    [[ "$state" = Z* ]] && break
    [ "$(date +%s)" -ge "$deadline" ] && return 1
    sleep 1
  done
  wait "$pid"
}
refresh_owned_groups() {
  local process_group
  while read -r process_group; do
    [ -n "$process_group" ] && register_group "$process_group"
  done < <(python3 "$SCRIPT_DIR/../evaluation/scan_replay_processes.py" \
    --run-id "$RUN_ID" --domain-id "$ROS_DOMAIN_ID" --format pgids)
}
cleanup() {
  local process_group deadline groups_present
  if [ -n "$SAMPLE_PID" ]; then
    kill "$SAMPLE_PID" 2>/dev/null || true
    wait "$SAMPLE_PID" 2>/dev/null || true
  fi
  refresh_owned_groups
  for process_group in "${PROCESS_GROUPS[@]}"; do
    stop_group "$process_group"
  done
  deadline=$(( $(date +%s) + 5 ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    if ! ps -eo pgid= | awk -v groups="${PROCESS_GROUPS[*]}" '
        BEGIN {split(groups, wanted, " ")}
        {for (index in wanted) if ($1 == wanted[index]) found=1}
        END {exit found ? 0 : 1}'; then
      return
    fi
    sleep 1
  done
  for process_group in "${PROCESS_GROUPS[@]}"; do
    kill -KILL -- "-$process_group" 2>/dev/null || true
  done
  deadline=$(( $(date +%s) + 5 ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    groups_present=0
    for process_group in "${PROCESS_GROUPS[@]}"; do
      if ps -eo pgid= 2>/dev/null |
          awk -v target="$process_group" \
          '$1 == target {found=1} END {exit !found}'; then
        groups_present=1
        break
      fi
    done
    [ "$groups_present" -eq 0 ] && return
    sleep 1
  done
}
trap cleanup EXIT

echo "run=$RUN_ID domain=$ROS_DOMAIN_ID rate=$RATE" | tee "$LOG"
START=$(date +%s.%N)

case "$BACKEND" in
  cartographer)
    PBSTREAM="$OUT/${RUN_ID}_map.pbstream"
    if [ -n "$CARTOGRAPHER_CONFIG" ]; then
      [ -f "$CARTOGRAPHER_CONFIG" ] || {
        echo "Cartographer 설정이 없다: $CARTOGRAPHER_CONFIG" >&2; exit 2; }
      CFG="$(dirname "$(realpath "$CARTOGRAPHER_CONFIG")")"
      CFG_BASENAME="$(basename "$(realpath "$CARTOGRAPHER_CONFIG")")"
    else
      CFG="$(ros2 pkg prefix jdamr_cube_cartographer)/share/jdamr_cube_cartographer/config"
      CFG_BASENAME="jdamr_cube_2d_corridor.lua"
    fi
    echo "cartographer_config=$CFG/$CFG_BASENAME" | tee -a "$LOG"
    # gflags 인자는 --ros-args 앞에 와야 한다. 뒤에 두면 cartographer_node 가
    # -configuration_directory is missing 으로 즉시 죽는다.
    setsid nohup ros2 run cartographer_ros cartographer_node \
      -configuration_directory "$CFG" \
      -configuration_basename "$CFG_BASENAME" \
      -save_state_filename "$PBSTREAM" \
      --ros-args -p use_sim_time:=true >> "$LOG" 2>&1 &
    BACKEND_PID=$!; register_group "$BACKEND_PID"
    setsid nohup ros2 run cartographer_ros cartographer_occupancy_grid_node \
      -resolution 0.05 -publish_period_sec 1.0 \
      --ros-args -p use_sim_time:=true >> "$LOG" 2>&1 &
    OCCUPANCY_PID=$!; register_group "$OCCUPANCY_PID"
    ;;
  slam_toolbox)
    # Jazzy 의 async_slam_toolbox_node 는 lifecycle 노드다. ros2 run 으로
    # 띄우면 unconfigured 상태로 남아 스캔을 처리하지 않고 /map 도 내지
    # 않는다.
    # upstream launch 가 configure/activate 를 수행하므로 그대로 쓴다.
    if [ -n "$SLAM_PARAMS" ]; then
      PARAMS="$SLAM_PARAMS"
    else
      PARAMS="$(ros2 pkg prefix jdamr_cube_navigation)/share/jdamr_cube_navigation/config/slam_toolbox_corridor.yaml"
    fi
    [ -f "$PARAMS" ] || {
      echo "SLAM Toolbox 설정이 없다: $PARAMS" >&2; exit 2; }
    echo "slam_params=$(realpath "$PARAMS")" | tee -a "$LOG"
    setsid nohup ros2 launch slam_toolbox online_async_launch.py \
      use_sim_time:=true slam_params_file:="$PARAMS" \
      >> "$LOG" 2>&1 &
    BACKEND_PID=$!; register_group "$BACKEND_PID"
    ;;
  *) echo "알 수 없는 backend: $BACKEND" >&2; exit 2 ;;
esac

RESOURCE_SAMPLES="$OUT/${RUN_ID}_resources.jsonl"
setsid nohup python3 \
  "$SCRIPT_DIR/../evaluation/sample_process_group_resources.py" \
  --process-group "$BACKEND_PID" --output "$RESOURCE_SAMPLES" \
  >> "$LOG" 2>&1 &
SAMPLE_PID=$!
register_group "$SAMPLE_PID"
sleep 10
case "$BACKEND" in
  cartographer)  probe='cartographer_node' ;;
  slam_toolbox)  probe='async_slam_toolbox_node' ;;
esac
if ! kill -0 "$BACKEND_PID" 2>/dev/null; then
  echo "backend($probe) 기동 실패 - 로그 확인" | tee -a "$LOG"
  tail -20 "$LOG"
  exit 3
fi
echo "backend 기동 확인: $probe" | tee -a "$LOG"
if [ "$BACKEND" = "cartographer" ]; then
  FINALIZER_TRIGGER="$OUT/${RUN_ID}_finalize.trigger"
  FINALIZER_SIDECAR="$OUT/${RUN_ID}_finalizer.json"
  rm -f "$FINALIZER_TRIGGER"
  setsid nohup python3 \
    "$SCRIPT_DIR/../evaluation/finalize_cartographer_replay.py" \
    --trigger "$FINALIZER_TRIGGER" \
    --sidecar "$FINALIZER_SIDECAR" --timeout-s 30 \
    >> "$LOG" 2>&1 &
  FINALIZER_PID=$!; register_group "$FINALIZER_PID"
fi

# backend 의 추정 궤적과 지도를 남긴다. 원본 센서는 이미 입력 bag 에 있다.
setsid nohup ros2 bag record --disable-keyboard-controls --storage mcap \
  --storage-config-file "$WRITER_CONFIG" \
  --output "$OUT/${RUN_ID}_result" \
  --topics /tf /tf_static /map >> "$LOG" 2>&1 &
RECORDER_PID=$!; register_group "$RECORDER_PID"
sleep 4
if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
  echo "결과 recorder 기동 실패 - 로그 확인" | tee -a "$LOG"
  exit 5
fi

# 재생을 배경으로 돌리고 진행 증거를 감시한다.
#
# backend 생존과 결과 기록 증가를 함께 감시해, 출력 없이 재생만 계속되는
# 실패를 실행 중에 검출한다.
setsid nohup ros2 launch jdamr_cube_navigation offline_replay_guard.launch.py \
  bag:="$BAG" rate:="$RATE" duration:="$DURATION" \
  stats_path:="$OUT/${RUN_ID}_replay_guard.json" \
  offline_domain_id:="$ROS_DOMAIN_ID" >> "$LOG" 2>&1 &
REPLAY_PID=$!
register_group "$REPLAY_PID"

# 감시는 신뢰할 수 있는 신호만 쓴다. 앞선 판은 /map 토픽 조회를
# 진행 증거로 삼았는데, 부하 상태의 디스커버리가 제한 시간 안에 끝나지 않아
# 정상 런을 중단할 수 있었다. 조회가 실패할 수 있는 검사는 감시에 쓰지 않는다.
# 프로세스 생존과 기록
# 증가만으로 판정한다.
last_size=0
last_progress=$(date +%s)
RC=0

while kill -0 "$REPLAY_PID" 2>/dev/null; do
  sleep 10
  if ! kill -0 "$BACKEND_PID" 2>/dev/null; then
    echo "감시: backend($probe) 가 죽었다 - 재생을 중단한다" | tee -a "$LOG"
    RC=4; break
  fi
  if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
    echo "감시: 결과 recorder가 죽었다 - 재생을 중단한다" | tee -a "$LOG"
    RC=5; break
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
done

if [ "$RC" -eq 0 ]; then
  wait "$REPLAY_PID"; RC=$?
else
  stop_group "$REPLAY_PID"
  sleep 5
fi

sleep 6
if [ "$RC" -eq 0 ] && [ "$BACKEND" = "cartographer" ]; then
  touch "$FINALIZER_TRIGGER"
  if ! wait_pid_bounded "$FINALIZER_PID" 35 || \
      [ ! -s "$FINALIZER_SIDECAR" ] || \
      ! grep -q '"status": "PASS"' "$FINALIZER_SIDECAR"; then
    echo "Cartographer persistent finalizer 실패" | tee -a "$LOG"
    RC=8
  else
    echo "evidence_stage=finish_trajectory_ok" | tee -a "$LOG"
  fi
fi
stop_group "$RECORDER_PID"
wait "$RECORDER_PID" 2>/dev/null || true
if [ ! -s "$RESULT_DIR/metadata.yaml" ]; then
  echo "결과 MCAP metadata 마감 실패" | tee -a "$LOG"
  RC=7
fi
if [ "$RC" -eq 0 ] && [ "$BACKEND" = "cartographer" ]; then
  stop_group "$OCCUPANCY_PID"
  wait "$OCCUPANCY_PID" 2>/dev/null || true
  kill -INT -- "-$BACKEND_PID" 2>/dev/null || true
  if ! wait_pid_bounded \
      "$BACKEND_PID" "$CARTOGRAPHER_SHUTDOWN_TIMEOUT"; then
    echo "Cartographer graceful shutdown 제한 시간 초과" | tee -a "$LOG"
    RC=8
  elif [ ! -s "$PBSTREAM" ] || \
      ! grep -q 'Running final trajectory optimization' "$LOG"; then
    echo "Cartographer 최종 최적화 또는 pbstream 증거 누락" | tee -a "$LOG"
    RC=8
  else
    echo "evidence_stage=shutdown_final_optimization_ok" | tee -a "$LOG"
  fi
fi
if [ "$RC" -eq 0 ] && [ "$BACKEND" = "cartographer" ]; then
  if ! timeout 90 ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
      -pbstream_filename "$PBSTREAM" \
      -map_filestem "$OUT/${RUN_ID}_map" -resolution 0.05 \
      >> "$LOG" 2>&1; then
    echo "pbstream 최종 지도 변환 실패" | tee -a "$LOG"
    RC=6
  else
    echo "evidence_stage=pbstream_to_ros_map_ok" | tee -a "$LOG"
  fi
elif [ "$RC" -eq 0 ] && ! timeout 30 ros2 run nav2_map_server map_saver_cli \
    -f "$OUT/${RUN_ID}_map" >> "$LOG" 2>&1; then
  echo "최종 지도 저장 실패" | tee -a "$LOG"
  RC=6
fi
END=$(date +%s.%N)
printf 'wall_seconds=%.1f\n' "$(echo "$END - $START" | bc)" | tee -a "$LOG"
echo "replay_exit=$RC" | tee -a "$LOG"
ls -la "$OUT" | tee -a "$LOG"
if [ -n "$SAMPLE_PID" ]; then
  kill "$SAMPLE_PID" 2>/dev/null || true
  wait "$SAMPLE_PID" 2>/dev/null || true
  SAMPLE_PID=""
fi
cleanup
trap - EXIT
TEARDOWN_FILE="$OUT/${RUN_ID}_teardown.json"
IDENTITY_SURVIVORS_JSON="$(python3 \
  "$SCRIPT_DIR/../evaluation/scan_replay_processes.py" \
  --run-id "$RUN_ID" --domain-id "$ROS_DOMAIN_ID" --format json)"
remaining_groups=()
for process_group in "${PROCESS_GROUPS[@]}"; do
  if ps -eo pgid= 2>/dev/null |
      awk -v target="$process_group" '$1 == target {found=1} END {exit !found}'; then
    remaining_groups+=("$process_group")
  fi
done
if [ "${#remaining_groups[@]}" -eq 0 ]; then
  printf '{"remaining_process_groups":[],"identity_survivors":%s}\n' \
    "$IDENTITY_SURVIVORS_JSON" > "$TEARDOWN_FILE"
else
  printf '{"remaining_process_groups":[%s],"identity_survivors":%s}\n' \
    "$(IFS=,; echo "${remaining_groups[*]}")" \
    "$IDENTITY_SURVIVORS_JSON" > "$TEARDOWN_FILE"
  RC=9
fi
[ "$IDENTITY_SURVIVORS_JSON" = "[]" ] || RC=9
exit "$RC"
