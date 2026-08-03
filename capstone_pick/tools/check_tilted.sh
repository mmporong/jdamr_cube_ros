#!/bin/bash
source /opt/ros/jazzy/setup.bash
L=~/capstone_tools/logs/rec_demo_tilted_pick.log
for i in $(seq 1 24); do
  sleep 20
  if grep -qE 'PICK_(SUCCESS|FAIL)' "$L" 2>/dev/null; then break; fi
done
grep -E '롤 먼저|파지 확인|파지 검증|PICK_' "$L" | sed 's/.*capstone_pick.: //' | tail -6
echo "=== 큐브 위치 ==="
gz model -m pick_blue -p 2>/dev/null | grep -A2 Pose | sed -n 2p
echo "=== GIF ==="
ls -la ~/gazebo-so101-capstone/assets/demo_tilted_pick.gif 2>/dev/null || echo "GIF 없음(실패라 미생성)"
