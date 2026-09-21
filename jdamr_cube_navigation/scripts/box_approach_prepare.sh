#!/usr/bin/env bash
# Prepare the pinned, disarmed approach chain. This script never calls start.
set -eo pipefail

release_dir="${JDAMR_APPROACH_RELEASE:?JDAMR_APPROACH_RELEASE must name a verified release}"
cd "$release_dir"
sha256sum --check --strict --quiet manifest.sha256
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_ws/install/setup.bash"
source "$release_dir/install/local_setup.bash"
exec ros2 launch jdamr_cube_navigation box_approach_execution.launch.py \
  camera_mount_file:="$release_dir/config/camera_mount.yaml" \
  geometry_file:="$release_dir/config/new_base_geometry.yaml" \
  parking_contract_file:="$release_dir/config/parking_contract.yaml" \
  nav_params_file:="$release_dir/config/new_base_nav2_params.yaml" \
  physical_validation_file:="$release_dir/config/physical_validation.yaml"
