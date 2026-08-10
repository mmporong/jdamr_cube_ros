#!/bin/bash
# 멈춘 상태(skip_approach)에서 색상별 파지가 되는지 검증.
# 주행 단계를 분리해 나머지 기능이 정상인지 먼저 확인한다.
TOOLS="$(cd "$(dirname "$0")" && pwd)"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone
export COLLECT_YAW_MAX=0.785 COLLECT_EP_TIMEOUT=300
source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash
for c in blue red green; do
  echo "===== $c ====="
  COLLECT_OUT="verify_$c" python3 -u "$TOOLS/rule_collect.py" --episodes 3 --color "$c" 2>&1 \
    | grep -E "^\[ep|수집 완료"
done
echo "===== 집계 ====="
for c in blue red green; do
  N=$(ls -d "$TOOLS/logs/verify_$c"/ep* 2>/dev/null | wc -l)
  echo "  $c: ${N}/3편 성공"
done
