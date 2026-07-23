#!/bin/bash
# gazebo.launch.py가 이미 떠 있는 상태에서 SO-101 팔 관절을 목표값으로 이동시킨다.
# usage: wsl_arm_control.sh [--shoulder-pan V] [--shoulder-lift V] [--elbow-flex V]
#                            [--wrist-flex V] [--wrist-roll V] [--gripper V] [--duration SEC]
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
exec ros2 run jdamr_cube_so101_arm joint_control "$@"
