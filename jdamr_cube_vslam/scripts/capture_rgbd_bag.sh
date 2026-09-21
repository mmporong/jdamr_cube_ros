#!/usr/bin/env bash
set -eo pipefail

usage() {
  echo "usage: $0 [--duration SEC] [--output DIR] [--low-bandwidth] [--record-navigation]"
}

duration_s=0
output_root="${HOME}/jdamr_data/vslam"
color_width=640
color_height=480
record_navigation=false
while (($#)); do
  case "$1" in
    --duration)
      duration_s="$2"
      shift 2
      ;;
    --output)
      output_root="$2"
      shift 2
      ;;
    --low-bandwidth)
      color_width=320
      color_height=240
      shift
      ;;
    --record-navigation)
      record_navigation=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

source /opt/ros/jazzy/setup.bash
source "${HOME}/astra_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

run_id="rgbd_$(date +%Y%m%dT%H%M%S)"
bag_dir="${output_root}/${run_id}"
camera_log="${bag_dir}/astra_rgbd.log"
recorder_log="${bag_dir}/rosbag_record.log"
mkdir -p "$bag_dir"

camera_pid=''
recorder_pid=''
cleanup() {
  set +e
  if [[ -n "$recorder_pid" ]]; then
    kill -TERM -- "-$recorder_pid" 2>/dev/null
    wait "$recorder_pid" 2>/dev/null
  fi
  if [[ -n "$camera_pid" ]]; then
    kill -INT -- "-$camera_pid" 2>/dev/null
    for _ in 1 2 3 4 5; do
      kill -0 "$camera_pid" 2>/dev/null || break
      sleep 1
    done
    kill -TERM -- "-$camera_pid" 2>/dev/null
  fi
  sudo -n systemctl start jdamr-astra-camera.service
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

sudo -n systemctl stop jdamr-astra-camera.service
setsid nice -n 5 bash -lc "source /opt/ros/jazzy/setup.bash; \
  source '${HOME}/astra_ws/install/setup.bash'; \
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'; \
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; \
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; \
  exec ros2 launch astra_camera astra.launch.xml \
    enable_color:=true enable_depth:=true enable_ir:=false \
    enable_point_cloud:=false enable_colored_point_cloud:=false \
    tf_publish_rate:=0.0 \
    depth_registration:=true color_depth_synchronization:=true \
    depth_width:=320 depth_height:=240 depth_fps:=30 \
    color_width:=${color_width} color_height:=${color_height} color_fps:=30" \
  >"$camera_log" 2>&1 &
camera_pid=$!

sleep 6
for image_topic in /camera/color/image_raw /camera/depth/image_raw; do
  if ! timeout 20s ros2 topic echo --no-daemon --spin-time 4 \
      --once --field header "$image_topic" \
      >/dev/null 2>&1; then
    echo "No frame received from ${image_topic}; see ${camera_log}" >&2
    exit 1
  fi
done

for reference_topic in /odom /scan; do
  if ! timeout 10s ros2 topic echo --no-daemon --spin-time 4 \
      --once --field header "$reference_topic" \
      >/dev/null 2>&1; then
    echo "No sample received from ${reference_topic}; refusing an incomplete SLAM capture" >&2
    exit 1
  fi
done

metadata="${bag_dir}/capture.yaml"
{
  echo "run_id: ${run_id}"
  echo "started_at: $(date --iso-8601=seconds)"
  echo "camera_model: Orbbec Astra S"
  echo "camera_serial: '17120813010'"
  echo "depth_registered_to_color: true"
  echo "color_depth_synchronization: true"
  echo "color_resolution: '${color_width}x${color_height}'"
  echo "depth_resolution: '320x240'"
  echo "navigation_telemetry: ${record_navigation}"
  echo "duration_s: ${duration_s}"
} >"$metadata"

package_prefix="$(ros2 pkg prefix jdamr_cube_vslam 2>/dev/null || true)"
qos_overrides="${package_prefix}/share/jdamr_cube_vslam/config/rosbag_qos_overrides.yaml"
if [[ -z "$package_prefix" || ! -f "$qos_overrides" ]]; then
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  qos_overrides=''
  for candidate in \
      "${script_dir}/../config/rosbag_qos_overrides.yaml" \
      "${script_dir}/../../../share/jdamr_cube_vslam/config/rosbag_qos_overrides.yaml"; do
    if [[ -f "$candidate" ]]; then
      qos_overrides="$(readlink -f "$candidate")"
      break
    fi
  done
fi
if [[ -z "$qos_overrides" || ! -f "$qos_overrides" ]]; then
  echo "rosbag QoS overrides not found" >&2
  exit 1
fi

record_topics=(
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/depth/image_raw
  /camera/depth/camera_info
  /tf
  /tf_static
  /odom
  /scan
)
if "$record_navigation"; then
  record_topics+=(
    /cmd_vel
    /cmd_vel_smoothed
    /collision_monitor_state
  )
fi
record_command=(
  ros2 bag record
  --storage mcap
  --storage-preset-profile zstd_fast
  --qos-profile-overrides-path "$qos_overrides"
  --output "${bag_dir}/bag"
  --topics
  "${record_topics[@]}"
)

echo "RGB-D capture: ${bag_dir}"
if ((duration_s > 0)); then
  setsid nice -n 10 ionice -c 2 -n 7 stdbuf -oL -eL \
    "${record_command[@]}" >"$recorder_log" 2>&1 &
  recorder_pid=$!
  subscribed=false
  for _ in $(seq 1 30); do
    if grep -q 'All requested topics are subscribed' "$recorder_log"; then
      subscribed=true
      break
    fi
    if ! kill -0 "$recorder_pid" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  if ! "$subscribed"; then
    cat "$recorder_log" >&2
    echo "rosbag did not subscribe to every required topic" >&2
    exit 1
  fi
  sleep "$duration_s"
  kill -TERM -- "-$recorder_pid"
  recorder_status=0
  wait "$recorder_pid" || recorder_status=$?
  recorder_pid=''
  cat "$recorder_log"
  if [[ "$recorder_status" != 0 && "$recorder_status" != 130 ]]; then
    exit "$recorder_status"
  fi
else
  "${record_command[@]}"
fi

echo "RGB-D capture complete: ${bag_dir}"
