#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

database_path=/output/jdamr_rgbd.db
rtabmap_log=/output/rtabmap.log
trajectory_log=/output/trajectory_recorder.log

cleanup() {
  set +e
  for pid in "${snapshot_pid:-}" "${recorder_pid:-}" "${launch_pid:-}"; do
    if [[ -n "$pid" ]]; then
      kill -INT -- "-$pid" 2>/dev/null
    fi
  done
  wait "${recorder_pid:-}" 2>/dev/null
  wait "${snapshot_pid:-}" 2>/dev/null
  wait "${launch_pid:-}" 2>/dev/null
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  args:="-d --Mem/IncrementalMemory true --Mem/InitWMWithAllNodes false \
    --Reg/Force3DoF true \
    --Vis/MinInliers 15 --RGBD/NeighborLinkRefining true \
    --RGBD/ProximityBySpace true --Grid/Sensor 1 \
    --Grid/3D true --Grid/RangeMax 5.0 --Rtabmap/DetectionRate 2.0" \
  database_path:="$database_path" \
  frame_id:=camera_link \
  map_frame_id:=map \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  depth:=true visual_odometry:=true icp_odometry:=false \
  subscribe_scan:=false \
  rgbd_sync:=true approx_rgbd_sync:=true approx_sync:=true \
  approx_sync_max_interval:=0.03 \
  qos:=2 topic_queue_size:=30 sync_queue_size:=30 \
  odom_always_process_most_recent_frame:=false \
  wait_for_transform:=0.3 \
  rtabmap_viz:=false rviz:=false \
  >"$rtabmap_log" 2>&1 &
launch_pid=$!

setsid python3 \
  /workspace/jdamr_cube_vslam/jdamr_cube_vslam/trajectory_csv_recorder.py \
  --output-dir /output \
  --visual-topic /rtabmap/odom \
  --reference-topic /odom \
  >"$trajectory_log" 2>&1 &
recorder_pid=$!

setsid python3 \
  /workspace/jdamr_cube_vslam/jdamr_cube_vslam/rgbd_snapshot_ply.py \
  --output-ply /output/rgbd_snapshot.ply \
  --output-json /output/rgbd_snapshot.json \
  >/output/rgbd_snapshot.log 2>&1 &
snapshot_pid=$!

sleep 5
ros2 bag play /data --clock --read-ahead-queue-size 2000
sleep 5

cleanup
trap - EXIT

if [[ ! -s "$database_path" ]]; then
  echo "RTAB-Map database was not created; see ${rtabmap_log}" >&2
  exit 1
fi

if [[ $(wc -l </output/visual_trajectory.csv) -gt 1 ]] \
    && command -v rtabmap-export >/dev/null 2>&1; then
  set +e
  rtabmap-export --cloud --poses --poses_format 10 \
    --voxel 0.02 --noise_radius 0.05 --noise_k 5 \
    --output jdamr_rgbd --output_dir /output "$database_path" \
    >/output/rtabmap_export.log 2>&1
  export_status=$?
  set -e
  if [[ $export_status -ne 0 ]]; then
    echo "Moving trajectory is required for assembled 3D export." \
      >>/output/rtabmap_export.log
    if [[ "${REQUIRE_ASSEMBLED_MAP:-1}" == 1 ]]; then
      exit "$export_status"
    fi
  fi
else
  echo "No visual odometry poses; database retained but 3D export skipped." \
    >/output/rtabmap_export.log
  if [[ "${REQUIRE_ASSEMBLED_MAP:-1}" == 1 ]]; then
    exit 1
  fi
fi

if [[ ! -s /output/rgbd_snapshot.ply ]]; then
  echo "Metric RGB-D snapshot was not created." >&2
  exit 1
fi
