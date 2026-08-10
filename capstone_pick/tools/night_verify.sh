#!/bin/bash
# 야간 체인: 규칙 기반 검증 → (성공률이 나오면) 4cm 큐브로 재수집.
#
# 오늘 파지 단계의 차체 주행을 없앴다(팔이 내려간 상태에서 차체를 밀면 큐브를
# 쳐낸다). 그 효과를 먼저 여러 번 재고, 쓸 만하면 학습용 데이터를 새로 모은다.
# 수집물이 3cm 기준인데 월드가 4cm로 돌아왔으므로 데이터를 다시 맞춰야 한다.
set -o pipefail
TOOLS="$(cd "$(dirname "$0")" && pwd)"
LOG="$TOOLS/logs/night_verify.log"
TRIALS=${1:-10}
GATE=${2:-50}          # 재수집으로 넘어가는 성공률 하한(%)
say() { echo "[$(date +%H:%M)] $*" | tee -a "$LOG"; }

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone
unset ROS_DOMAIN_ID
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash

reset_scene() {
  for req in 'name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}' \
             'name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}' \
             'name: "pick_object_blue", position: {x: 0.45, y: -0.26, z: 0.02}' \
             'name: "pick_object_orange", position: {x: 0.52, y: 0.00, z: 0.02}'; do
    gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
      --timeout 4000 --req "$req" >/dev/null 2>&1
  done
}

say "=== 야간 검증 시작 (${TRIALS}회) ==="
systemctl --user stop capstone-sim 2>/dev/null; sleep 4
systemd-run --user --collect --setenv=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --setenv=GZ_PARTITION=lim-capstone --unit=capstone-sim bash -c \
  'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && exec ros2 launch jdamr_cube_gazebo gazebo.launch.py gui:=false world:=$HOME/jdamr_cube_ws/install/jdamr_cube_gazebo/share/jdamr_cube_gazebo/worlds/room.world' >/dev/null 2>&1
for i in $(seq 1 40); do
  N=$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c active)
  [ "${N:-0}" = "3" ] && break; sleep 5
done
say "시뮬 준비 (컨트롤러 ${N:-0}/3)"
[ "${N:-0}" = "3" ] || { say "시뮬 기동 실패 — 중단"; exit 1; }

OK=0
for t in $(seq 1 "$TRIALS"); do
  reset_scene; sleep 2
  timeout 300 ros2 run capstone_pick pick --ros-args \
    -p target_color:=green -p place_target:=trash -p speed_scale:=1.0 -p detector:=hsv \
    > "$TOOLS/logs/nv_${t}.log" 2>&1
  if grep -q "PICK_SUCCESS" "$TOOLS/logs/nv_${t}.log"; then
    OK=$((OK + 1)); R="성공"
  else
    R=$(grep -oE "파지 실패 \([^)]*\)|반복 소진|접근 실패" "$TOOLS/logs/nv_${t}.log" | tail -1)
    R="실패 ${R:-원인불명}"
  fi
  say "  [$t/$TRIALS] $R  (누적 ${OK}/${t})"
done
RATE=$((OK * 100 / TRIALS))
say "=== 검증 종료: ${OK}/${TRIALS} (${RATE}%) ==="

if [ "$RATE" -ge "$GATE" ]; then
  say "성공률 ${RATE}% ≥ ${GATE}% — 4cm 큐브로 재수집 시작 (50편)"
  systemctl --user stop capstone-sim 2>/dev/null; sleep 3
  COLLECT_OUT=rule_4cm COLLECT_YAW_MAX=0.785 COLLECT_EP_TIMEOUT=300 \
    bash "$TOOLS/collect_yaw45.sh" >> "$LOG" 2>&1
  say "재수집 종료: $(ls -d "$TOOLS/logs/rule_4cm"/ep* 2>/dev/null | wc -l)편"
else
  say "성공률 ${RATE}% < ${GATE}% — 재수집 보류. 실패 로그: logs/nv_*.log"
fi
systemctl --user stop capstone-sim 2>/dev/null
say "=== 야간 체인 종료 (시뮬 정지) ==="
python3 "$HOME/PKM/05_AI_Config/notify_telegram.py" "<b>야간 검증 완료</b>
규칙 기반 ${OK}/${TRIALS} (${RATE}%)

- 파지 중 차체 주행 제거 효과 측정
- 재수집: $([ "$RATE" -ge "$GATE" ] && echo "진행 $(ls -d "$TOOLS/logs/rule_4cm"/ep* 2>/dev/null | wc -l)편" || echo "보류")" >/dev/null 2>&1
