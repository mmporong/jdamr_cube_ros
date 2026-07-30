#!/bin/bash
# 게이트 2 판정: 손목캠 낙하 판정이 두 경우를 실제로 갈라내는가.
#   A) 정상 파지 → HOLDING이어야 하고 PICK_SUCCESS로 끝나야 한다
#   B) 파지 성립 후 큐브를 인위적으로 치움 → DROPPED로 잡아내야 한다(가짜 성공 차단)
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
OUT=~/capstone_tools/logs
mkdir -p "$OUT"
RESULT="$OUT/gate2_result.txt"
: > "$RESULT"

pose_of() {
  for t in 1 2 3; do
    P=$(gz model -m "$1" -p 2>/dev/null | grep -A2 Pose | sed -n 2p | tr -d '[]' | tr -s ' ')
    if [ -n "$P" ]; then echo "$P"; return; fi
    sleep 1
  done
  echo ""
}

stage() {
  python3 ~/capstone_tools/reset_and_stage.py tricolor_stage.py > /dev/null 2>&1
  sleep 1
  gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 5000 --req 'name: "jdamr_cube" position {x: 0 y: 0 z: 0.05} orientation {w: 1}' > /dev/null 2>&1
  sleep 2
  gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 5000 --req 'name: "pick_blue" position {x: 0.681 y: 0 z: 0.015} orientation {w: 1}' > /dev/null 2>&1
  sleep 2
}

# A) 정상 파지
stage
timeout 420 ros2 run capstone_pick pick --ros-args -p speed_scale:=4.0 > "$OUT/gate2_normal.log" 2>&1
P=$(pose_of pick_blue)
A=$(grep -oE '파지 확인: 손목캠 면적=[0-9]+' "$OUT/gate2_normal.log" | tail -1 | grep -oE '[0-9]+$')
H=$(grep -oE '→ (HOLDING|DROPPED)' "$OUT/gate2_normal.log" | tail -1)
R=$(grep -oE 'PICK_(SUCCESS|FAIL)' "$OUT/gate2_normal.log" | tail -1)
echo "A) 정상 파지: 면적=${A} 판정=${H} 결과=${R} 큐브=${P}" >> "$RESULT"

# B) 파지 후 큐브를 없애 낙하를 모사한다.
#    set_pose로 옮기는 방식은 안 된다 — 쥐고 있는 물체는 그리퍼와 접촉이 유지돼
#    물리 엔진이 다시 끌고 온다(실측: 옮겼는데 면적 20675로 HOLDING).
#    엔티티를 삭제하면 손목캠에서 완전히 사라지므로 면적이 0이 되어야 한다.
stage
timeout 420 ros2 run capstone_pick pick --ros-args -p speed_scale:=4.0 > "$OUT/gate2_steal.log" 2>&1 &
PICK_PID=$!
sleep 100
gz service -s /world/room/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'name: "pick_blue" type: MODEL' > /dev/null 2>&1
echo "  (100초 시점에 큐브 삭제)" >> "$RESULT"
wait $PICK_PID
A2=$(grep -oE '파지 확인: 손목캠 면적=[0-9]+' "$OUT/gate2_steal.log" | tail -1 | grep -oE '[0-9]+$')
H2=$(grep -oE '→ (HOLDING|DROPPED)' "$OUT/gate2_steal.log" | tail -1)
R2=$(grep -oE 'PICK_(SUCCESS|FAIL)' "$OUT/gate2_steal.log" | tail -1)
echo "B) 파지 후 큐브 삭제: 면적=${A2:-0} 판정=${H2} 결과=${R2}" >> "$RESULT"
echo "   기대: 면적 0 → DROPPED → PICK_FAIL (각도 판정으로는 못 잡는 경우)" >> "$RESULT"
echo DONE >> "$RESULT"
