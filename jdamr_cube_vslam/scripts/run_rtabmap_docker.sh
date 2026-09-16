#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 BAG_DIR OUTPUT_DIR [--allow-static] [--image IMAGE]"
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
while (($#)); do
  case "$1" in
    --allow-static)
      require_assembled_map=0
      shift
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

docker run --rm \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=72 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e REQUIRE_ASSEMBLED_MAP="$require_assembled_map" \
  -v "${bag_dir}:/data:ro" \
  -v "${output_dir}:/output" \
  -v "${package_dir}:/workspace/jdamr_cube_vslam:ro" \
  "$image" \
  bash /workspace/jdamr_cube_vslam/scripts/process_rgbd_bag_in_container.sh

echo "RTAB-Map output: ${output_dir}"
