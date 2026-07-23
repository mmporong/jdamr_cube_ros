#!/bin/bash
# jdamr_cube_controller의 Nav2 스택(map_server + amcl + planner/controller 등)을 띄운다.
# 기본 맵 경로는 navigation.launch.py의 기본값(~/maps/jdamr_cube_room.yaml)을 그대로 쓰며,
# 필요하면 추가 launch 인자를 그대로 전달할 수 있다. 예) wsl_run_navigation.sh map:=/path/to/map.yaml
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
exec ros2 launch jdamr_cube_controller navigation.launch.py "$@"
