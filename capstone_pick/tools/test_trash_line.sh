#!/bin/bash
# 일직선 배치 쓰레기통 투입 (smoke_test [3]과 같은 조건)을 로그를 남기며 재현한다.
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
OUT=~/capstone_tools/logs
mkdir -p "$OUT"

python3 ~/capstone_tools/trash_demo_stage.py > /dev/null 2>&1
sleep 2
timeout 560 ros2 run capstone_pick pick --ros-args \
  -p speed_scale:=4.0 -p place_target:=trash > "$OUT/trash_line.log" 2>&1

echo "=== 운반·투입 구간 ==="
grep -E '파지 확인|들기 후|통 후보|쓰레기통:|통 접근|근접 락|락 좌표|미검출|자리 이동|통 중심|열기|PICK' \
  "$OUT/trash_line.log" | sed 's/.*capstone_pick.: //' | tail -22

echo "=== 투입 판정 ==="
pose_of() {
  for t in 1 2 3; do
    P=$(gz model -m "$1" -p 2>/dev/null | grep -A2 Pose | sed -n 2p | tr -d '[]' | tr -s ' ')
    if [ -n "$P" ]; then echo "$P"; return; fi
    sleep 1
  done
  echo ""
}
CUBE=$(pose_of pick_blue)
python3 -c "
import math
c = '${CUBE}'.split()
if len(c) < 3:
    print('좌표 조회 실패')
else:
    cx, cy, cz = map(float, c)
    tx, ty = 0.34, 1.0
    d = math.dist([cx, cy], [tx, ty])
    ok = d < 0.068 and cz > 0.02
    print(f'큐브 ({cx:.3f}, {cy:.3f}, {cz:.3f})  통 ({tx}, {ty})')
    print(f'통 중심에서 {d*1000:.0f}mm, 높이 {cz:.3f}  →  {\"투입 성공\" if ok else \"실패\"}')
"
