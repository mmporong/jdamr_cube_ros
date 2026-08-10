#!/bin/bash
# 휴지통 투입 검증: 파지는 멈춘 상태(주행 없음), 운반만 주행.
# 접근 주행이 실패하는 것과 별개로, 운반 주행은 되는지 가른다.
TOOLS="$(cd "$(dirname "$0")" && pwd)"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone
export COLLECT_YAW_MAX=0.785 COLLECT_EP_TIMEOUT=400
export COLLECT_PLACE=trash COLLECT_OUT=verify_trash
source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash
python3 -u "$TOOLS/rule_collect.py" --episodes 3 --color blue 2>&1 | grep -E "^\[ep|수집 완료"
echo "저장: $(ls -d "$TOOLS/logs/verify_trash"/ep* 2>/dev/null | wc -l)/3편"
