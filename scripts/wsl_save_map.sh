#!/bin/bash
# 현재 cartographer가 발행 중인 /map 토픽을 pgm/yaml로 저장하고,
# Windows에서도 바로 보이도록 저장소의 maps/ 폴더에도 복사한다.
# usage: wsl_save_map.sh <repo-path-in-wsl> [map-name]
set -e

REPO_WSL="$1"
MAP_NAME="${2:-jdamr_cube_room}"

if [ -z "$REPO_WSL" ]; then
  echo "usage: wsl_save_map.sh <repo-path-in-wsl> [map-name]" >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"

mkdir -p "$HOME/maps"
ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/$MAP_NAME" --ros-args -p save_map_timeout:=10.0

mkdir -p "$REPO_WSL/maps"
cp "$HOME/maps/$MAP_NAME.pgm" "$REPO_WSL/maps/$MAP_NAME.pgm"
cp "$HOME/maps/$MAP_NAME.yaml" "$REPO_WSL/maps/$MAP_NAME.yaml"

echo "Saved: $HOME/maps/$MAP_NAME.pgm / .yaml"
echo "Copied to: $REPO_WSL/maps/$MAP_NAME.pgm / .yaml"
