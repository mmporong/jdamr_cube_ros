#!/bin/bash
# 이전에 남아있는 jdamr_cube 시뮬레이션/SLAM 프로세스를 정리한다.
# (run_gazebo_slam.bat을 다시 실행했을 때 gazebo/cartographer/rviz2가 중복 기동되는 것을 방지)
PATTERNS=(
  "gazebo.launch.py"
  "cartographer.launch.py"
  "jdamr_cube_teleop"
  "cartographer_node"
  "cartographer_occupancy_grid_node"
  "rviz2"
  "gz sim"
  "parameter_bridge"
  "ros_gz_sim create"
  "navigation.launch.py"
  "component_container_isolated"
  "lifecycle_manager"
)

for p in "${PATTERNS[@]}"; do
  pkill -9 -f "$p" >/dev/null 2>&1
done

sleep 1
exit 0
