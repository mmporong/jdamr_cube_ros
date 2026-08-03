#!/bin/bash
# 게이트 1 판정: 3색 x 위치·기울기 무작위 10회.
# 판정은 로그가 아니라 큐브가 실제로 옮겨졌는지(변위 50mm 초과)로 한다.
#
# 산출물을 ~/capstone_tools/logs 에 둔다 — /tmp는 WSL 재시작마다 비워져
# 한 시간짜리 테스트 결과가 통째로 사라진다(실측).
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
OUT=~/capstone_tools/logs
mkdir -p "$OUT"
RESULT="$OUT/gate10_result.txt"
: > "$RESULT"

pose_of() {
  for t in 1 2 3; do
    P=$(gz model -m "$1" -p 2>/dev/null | grep -A2 Pose | sed -n 2p | tr -d '[]' | tr -s ' ')
    if [ -n "$P" ]; then echo "$P"; return; fi
    sleep 1
  done
  echo ""
}

COLORS=(blue red green blue red green blue red green blue)
XS=(0.30 0.34 0.28 0.32 0.36 0.30 0.33 0.29 0.35 0.31)
YAWS=(0.0 0.20 -0.25 0.35 0.0 -0.15 0.30 -0.35 0.10 -0.20)
PASS=0
for i in $(seq 0 9); do
  C=${COLORS[$i]}
  X=${XS[$i]}
  YAW=${YAWS[$i]}
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
  timeout 420 ros2 run capstone_pick pick --ros-args -p speed_scale:=4.0 -p target_color:=${C} > "$OUT/gate10_${i}.log" 2>&1
  P1=$(pose_of pick_${C})
  ANG=$(grep -oE '파지 검증: 그리퍼 각도=[-0-9.]+' "$OUT/gate10_${i}.log" | tail -1 | grep -oE '[-0-9.]+$')
  RES=$(grep -oE 'PICK_(SUCCESS|FAIL)' "$OUT/gate10_${i}.log" | tail -1)
  if [ -z "$P1" ] || [ -z "$P0" ]; then
    VERDICT="측정불가"
  else
    VERDICT=$(python3 -c "
import math
p0='${P0}'.split(); p1='${P1}'.split()
d=math.dist([float(p0[0]),float(p0[1])],[float(p1[0]),float(p1[1])])
print(('옮김 %.0fmm'%(d*1000)) if d>0.05 else ('제자리 %.0fmm'%(d*1000)))
")
  fi
  case "$VERDICT" in 옮김*) PASS=$((PASS+1));; esac
  echo "[$i] ${C} x=${X} yaw=${YAW} 물림각=${ANG} 로그=${RES} 실제=${VERDICT}" >> "$RESULT"
done
echo "판정: ${PASS}/10 실제 옮김 (게이트 기준 8)" >> "$RESULT"
echo DONE >> "$RESULT"
