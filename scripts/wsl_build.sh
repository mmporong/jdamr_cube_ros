#!/bin/bash
# Windows 저장소(REPO_WSL)를 WSL 네이티브 워크스페이스(~/jdamr_cube_ws)로 rsync한 뒤 빌드한다.
# 저장소에 커밋된 build/install/log(Humble 기준 산출물)는 건드리지 않는다.
set -e

REPO_WSL="$1"
if [ -z "$REPO_WSL" ]; then
  echo "usage: wsl_build.sh <repo-path-in-wsl>" >&2
  exit 1
fi

WS="$HOME/jdamr_cube_ws"
PACKAGES="jdamr_cube_bringup jdamr_cube_cartographer jdamr_cube_description jdamr_cube_gazebo jdamr_cube_navigation jdamr_cube_node jdamr_cube_teleop ldlidar_sl_ros2"

mkdir -p "$WS/src"
for p in $PACKAGES; do
  mkdir -p "$WS/src/$p"
  rsync -a --delete "$REPO_WSL/$p/" "$WS/src/$p/"
done

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths "$WS/src" --ignore-src -y --rosdistro jazzy

cd "$WS"
colcon build --symlink-install
