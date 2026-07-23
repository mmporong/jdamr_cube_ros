#!/bin/bash
# navigation.launch.py가 이미 떠 있는 상태에서 목표 좌표로 이동시킨다.
# usage: wsl_goto_pose.sh X Y [--yaw YAW]
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
exec ros2 run jdamr_cube_controller goto_pose "$@"
