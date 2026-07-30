#!/bin/bash
# 스택이 실제로 응답할 때까지 대기 후 진단. start_stack.sh 다음에 돌린다.
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
for i in $(seq 1 18); do
  sleep 5
  if timeout 10 ros2 control list_controllers 2>/dev/null | grep -q 'gripper_controller.*active'; then
    echo "컨트롤러 active ($((i*5))초)"
    break
  fi
done
bash ~/capstone_tools/doctor.sh 2>&1 | tail -14
