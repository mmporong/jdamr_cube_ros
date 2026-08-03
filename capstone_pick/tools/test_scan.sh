#!/bin/bash
# 쓰레기통 탐색 검증: 통을 로봇 뒤쪽에 두어 회전 탐색을 강제한다.
# 확인 대상은 "명령한 각도만큼 기체가 실제로 도는가"이고, 오도메트리 실측이 로그에 남는다.
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
OUT=~/capstone_tools/logs
mkdir -p "$OUT"

python3 ~/capstone_tools/reset_and_stage.py trash_demo_stage.py > /dev/null 2>&1 || { echo "무대 배치 실패 — 중단"; exit 1; }
sleep 1
# 로봇은 원점 정면, 통은 로봇 뒤쪽(-y)에 둔다 → 파지 후 통이 시야 밖이라 탐색이 발동한다
gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'name: "jdamr_cube" position {x: 0 y: 0 z: 0.05} orientation {w: 1}' > /dev/null 2>&1
sleep 2
gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'name: "trash_can" position {x: 0.30 y: -1.10 z: 0.0} orientation {w: 1}' > /dev/null 2>&1
sleep 1
gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'name: "pick_blue" position {x: 0.681 y: 0 z: 0.015} orientation {w: 1}' > /dev/null 2>&1
sleep 2

timeout 500 ros2 run capstone_pick pick --ros-args \
  -p speed_scale:=4.0 -p place_target:=trash > "$OUT/scan_test.log" 2>&1

echo "=== 탐색 로그 ==="
grep -E '미검출|회전 명령|자리 이동|통 후보|쓰레기통:|통 접근|근접 락|PICK_' "$OUT/scan_test.log" \
  | sed 's/.*capstone_pick.: //' | tail -24
echo "=== 투입 판정 (로그가 아니라 좌표로) ==="
pose_of() {   # gz 조회는 간헐적으로 빈 값을 준다 — 3회 재시도
  for t in 1 2 3; do
    P=$(gz model -m "$1" -p 2>/dev/null | grep -A2 Pose | sed -n 2p | tr -d '[]' | tr -s ' ')
    if [ -n "$P" ]; then echo "$P"; return; fi
    sleep 1
  done
  echo ""
}
CUBE=$(pose_of pick_blue)
CAN=$(pose_of trash_can)
python3 -c "
import math
c = '${CUBE}'.split()
t = '${CAN}'.split()
if len(c) < 3 or len(t) < 3:
    print('좌표 조회 실패')
else:
    cx, cy, cz = map(float, c)
    tx, ty, _ = map(float, t)
    d = math.dist([cx, cy], [tx, ty])
    # 개구부 13.6cm(반폭 6.8cm), 통 바닥판 z=0.027 — 둘 다 만족해야 투입이다
    ok = d < 0.068 and cz > 0.02
    print(f'큐브 ({cx:.3f}, {cy:.3f}, {cz:.3f})  통 ({tx:.3f}, {ty:.3f})')
    print(f'통 중심에서 {d*1000:.0f}mm, 높이 {cz:.3f}  →  {\"투입 성공\" if ok else \"통 밖(실패)\"}')
"
