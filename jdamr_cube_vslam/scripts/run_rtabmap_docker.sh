#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 BAG_DIR OUTPUT_DIR [--allow-static] [--profile baseline|low-texture] [--odom-guess-frame FRAME] [--camera-mount YAML] [--image IMAGE]"
}

if (($# < 2)); then
  usage >&2
  exit 2
fi

bag_dir="$(realpath "$1")"
output_dir="$(realpath -m "$2")"
shift 2
image="introlab3it/rtabmap_ros:jazzy"
require_assembled_map=1
odom_guess_frame_id=''
rtabmap_profile='baseline'
camera_mount_config=''
while (($#)); do
  case "$1" in
    --allow-static)
      require_assembled_map=0
      shift
      ;;
    --odom-guess-frame)
      odom_guess_frame_id="$2"
      shift 2
      ;;
    --profile)
      rtabmap_profile="$2"
      shift 2
      ;;
    --camera-mount)
      camera_mount_config="$(realpath "$2")"
      shift 2
      ;;
    --image)
      image="$2"
      shift 2
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -d "$bag_dir" ]]; then
  echo "bag directory not found: ${bag_dir}" >&2
  exit 1
fi
mkdir -p "$output_dir"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
if [[ -z "$camera_mount_config" ]]; then
  camera_mount_config="${package_dir}/config/camera_mount.yaml"
fi

camera_mount_env=()
if [[ -n "$odom_guess_frame_id" ]]; then
  if [[ ! -f "$camera_mount_config" ]]; then
    echo "camera mount config not found: ${camera_mount_config}" >&2
    exit 1
  fi
  mapfile -t camera_mount_values < <(
    python3 - "$camera_mount_config" <<'PY'
import math
import sys

import yaml


path = sys.argv[1]
with open(path, encoding='utf-8') as stream:
    config = yaml.safe_load(stream)

mount = config.get('camera_mount', {})
gate = config.get('usage_gate', {})
if mount.get('status') != 'measured':
    raise SystemExit(
        'wheel odometry guess is blocked: camera mount status must be measured')
if gate.get('wheel_odom_fusion') != 'allowed':
    raise SystemExit(
        'wheel odometry guess is blocked by usage_gate.wheel_odom_fusion')

transform = mount.get('transform', {})
keys = ('x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad')
values = []
for key in keys:
    value = transform.get(key)
    if type(value) not in (int, float) or not math.isfinite(value):
        raise SystemExit(f'camera mount transform {key} must be finite')
    values.append(value)

parent = mount.get('parent_frame')
child = mount.get('child_frame')
if not isinstance(parent, str) or not parent:
    raise SystemExit('camera mount parent_frame is required')
if not isinstance(child, str) or not child:
    raise SystemExit('camera mount child_frame is required')

for value in (parent, child, *values):
    print(value)
PY
  )
  if [[ ${#camera_mount_values[@]} -ne 8 ]]; then
    echo "invalid camera mount config: ${camera_mount_config}" >&2
    exit 1
  fi
  camera_mount_env=(
    -e CAMERA_MOUNT_PARENT="${camera_mount_values[0]}"
    -e CAMERA_MOUNT_CHILD="${camera_mount_values[1]}"
    -e CAMERA_MOUNT_X="${camera_mount_values[2]}"
    -e CAMERA_MOUNT_Y="${camera_mount_values[3]}"
    -e CAMERA_MOUNT_Z="${camera_mount_values[4]}"
    -e CAMERA_MOUNT_ROLL="${camera_mount_values[5]}"
    -e CAMERA_MOUNT_PITCH="${camera_mount_values[6]}"
    -e CAMERA_MOUNT_YAW="${camera_mount_values[7]}"
  )
fi

docker run --rm \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=72 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e REQUIRE_ASSEMBLED_MAP="$require_assembled_map" \
  -e ODOM_GUESS_FRAME_ID="$odom_guess_frame_id" \
  -e RTABMAP_PROFILE="$rtabmap_profile" \
  "${camera_mount_env[@]}" \
  -v "${bag_dir}:/data:ro" \
  -v "${output_dir}:/output" \
  -v "${package_dir}:/workspace/jdamr_cube_vslam:ro" \
  "$image" \
  bash /workspace/jdamr_cube_vslam/scripts/process_rgbd_bag_in_container.sh

echo "RTAB-Map output: ${output_dir}"
