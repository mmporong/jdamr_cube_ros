#!/usr/bin/env bash

configuration_basename="${1:-jdamr_cube_2d_real.lua}"
workspace_setup="${HOME}/jdamr_ws/install/setup.bash"

if [[ ! "${configuration_basename}" =~ ^[A-Za-z0-9_.-]+\.lua$ ]]; then
  echo "잘못된 설정 파일 이름: ${configuration_basename}" >&2
  exit 2
fi

if [[ ! -f "${workspace_setup}" ]]; then
  echo "ROS 워크스페이스를 찾을 수 없습니다: ${workspace_setup}" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
source "${workspace_setup}"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

configuration_directory="$(ros2 pkg prefix jdamr_cube_cartographer)/share/jdamr_cube_cartographer/config"
configuration_path="${configuration_directory}/${configuration_basename}"
log_stem="${configuration_basename%.lua}"

if [[ ! -f "${configuration_path}" ]]; then
  echo "Cartographer 설정을 찾을 수 없습니다: ${configuration_path}" >&2
  exit 2
fi

mkdir -p "${HOME}/maps"
echo "[1/3] 현재 지도 저장..."
if ! ros2 run nav2_map_server map_saver_cli \
    -f "${HOME}/maps/$(date +%m%d_%H%M)_prereset" \
    --ros-args -p save_map_timeout:=15.0 >/dev/null 2>&1; then
  echo "저장할 지도가 없어 지도 저장을 건너뜁니다."
fi

echo "[2/3] 기존 Cartographer 종료..."
for executable in cartographer_node cartographer_occupancy_grid_node; do
  mapfile -t process_ids < <(
    pgrep -f "^/opt/ros/.*/lib/cartographer_ros/${executable} " || true
  )
  if (( ${#process_ids[@]} > 0 )); then
    kill "${process_ids[@]}"
  fi
done
sleep 3

echo "[3/3] Cartographer 재시작 (${configuration_basename})..."
setsid ros2 run cartographer_ros cartographer_node \
  -configuration_directory "${configuration_directory}" \
  -configuration_basename "${configuration_basename}" \
  --ros-args -p use_sim_time:=false \
  > "${HOME}/carto_${log_stem}.log" 2>&1 < /dev/null &
setsid ros2 run cartographer_ros cartographer_occupancy_grid_node \
  -resolution 0.05 -publish_period_sec 1.0 \
  --ros-args -p use_sim_time:=false \
  > "${HOME}/carto_occ.log" 2>&1 < /dev/null &

sleep 5
if ! pgrep -f "^/opt/ros/.*/lib/cartographer_ros/cartographer_node " >/dev/null; then
  echo "Cartographer 시작에 실패했습니다. ${HOME}/carto_${log_stem}.log 를 확인하세요." >&2
  exit 1
fi

echo "완료: ${configuration_basename}"
