#!/bin/bash
# 특정 조건만 골라 재판정한다. 사용: gate_subset.sh "색:x:yaw" "색:x:yaw" ...
#   예) gate_subset.sh red:0.34:0.20 green:0.28:-0.25 red:0.29:-0.35
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
OUT=~/capstone_tools/logs
mkdir -p "$OUT"
RESULT="$OUT/gate_subset_result.txt"
: > "$RESULT"

pose_of() {
  for t in 1 2 3; do
    P=$(gz model -m "$1" -p 2>/dev/null | grep -A2 Pose | sed -n 2p | tr -d '[]' | tr -s ' ')
    if [ -n "$P" ]; then echo "$P"; return; fi
    sleep 1
  done
  echo ""
}

n=0
for spec in "$@"; do
  C=$(echo "$spec" | cut -d: -f1)
  X=$(echo "$spec" | cut -d: -f2)
  YAW=$(echo "$spec" | cut -d: -f3)
  python3 ~/capstone_tools/reset_and_stage.py tricolor_stage.py > /dev/null 2>&1 || { echo "무대 배치 실패 — 중단"; exit 1; }
  sleep 1
  gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 5000 --req 'name: "jdamr_cube" position {x: 0 y: 0 z: 0.05} orientation {w: 1}' > /dev/null 2>&1
  sleep 2
  HW=$(python3 -c "import math; print(math.cos(${YAW}/2))")
  HZ=$(python3 -c "import math; print(math.sin(${YAW}/2))")
  TX=$(python3 -c "print(0.3+${X})")
  gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 5000 --req "name: \"pick_${C}\" position {x: ${TX} y: 0 z: 0.015} orientation {w: ${HW} z: ${HZ}}" > /dev/null 2>&1
  sleep 2
  P0=$(pose_of pick_${C})
  timeout 420 ros2 run capstone_pick pick --ros-args -p speed_scale:=4.0 -p target_color:=${C} > "$OUT/subset_${n}.log" 2>&1
  P1=$(pose_of pick_${C})
  ANG=$(grep -oE '파지 검증: 그리퍼 각도=[-0-9.]+' "$OUT/subset_${n}.log" | tail -1 | grep -oE '[-0-9.]+$')
  HOLD=$(grep -oE '파지 확인[^→]*→ (HOLDING|DROPPED)' "$OUT/subset_${n}.log" | tail -1 | grep -oE '(HOLDING|DROPPED)')
  RES=$(grep -oE 'PICK_(SUCCESS|FAIL)' "$OUT/subset_${n}.log" | tail -1)
  CONV=$(grep -oE '정렬 수렴\([0-9]+회\): .*' "$OUT/subset_${n}.log" | tail -1)
  if [ -z "$P1" ] || [ -z "$P0" ]; then
    V="측정불가"
  else
    V=$(python3 -c "
import math
p0='${P0}'.split(); p1='${P1}'.split()
d=math.dist([float(p0[0]),float(p0[1])],[float(p1[0]),float(p1[1])])
print(('옮김 %.0fmm'%(d*1000)) if d>0.05 else ('제자리 %.0fmm'%(d*1000)))
")
  fi
  echo "[${n}] ${C} x=${X} yaw=${YAW} 물림각=${ANG} 판정=${HOLD} 결과=${RES} 실제=${V}" >> "$RESULT"
  echo "     ${CONV:-정렬수렴 로그없음}" >> "$RESULT"
  n=$((n+1))
done
echo DONE >> "$RESULT"
