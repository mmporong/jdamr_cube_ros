#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 DATABASE OUTPUT_DIR [preview|portfolio] [--image IMAGE]"
}

if (($# < 2)); then
  usage >&2
  exit 2
fi

database="$(realpath "$1")"
output_dir="$(realpath -m "$2")"
quality="${3:-preview}"
if (($# >= 3)); then
  shift 3
else
  shift 2
fi
image="introlab3it/rtabmap_ros:jazzy"
if (($#)); then
  if [[ "$1" != '--image' || $# -ne 2 ]]; then
    usage >&2
    exit 2
  fi
  image="$2"
fi

if [[ ! -f "$database" ]]; then
  echo "database not found: ${database}" >&2
  exit 1
fi
case "$quality" in
  preview|portfolio) ;;
  *)
    usage >&2
    exit 2
    ;;
esac
mkdir -p "$output_dir"

database_dir="$(dirname "$database")"
database_name="$(basename "$database")"

if [[ "$quality" == preview ]]; then
  export_command=(
    rtabmap-export
    --cloud
    --opt 2
    --poses
    --poses_camera
    --poses_format 10
    --decimation 2
    --voxel 0.02
    --noise_radius 0.05
    --noise_k 5
    --output jdamr_rgbd_preview
    --output_dir /output
    "/database/${database_name}"
  )
else
  export_command=(
    rtabmap-export
    --cloud
    --mesh
    --texture
    --opt 2
    --poses
    --poses_camera
    --poses_format 10
    --decimation 1
    --voxel 0.01
    --noise_radius 0.04
    --noise_k 5
    --texture_size 4096
    --max_polygons 500000
    --output jdamr_rgbd_portfolio
    --output_dir /output
    "/database/${database_name}"
  )
fi

docker run --rm \
  -v "${database_dir}:/database:ro" \
  -v "${output_dir}:/output" \
  "$image" \
  "${export_command[@]}"

echo "3D assets: ${output_dir}"
