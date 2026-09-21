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
rtabmap_profile="${RTABMAP_PROFILE:-baseline}"
odom_source_mode="${ODOM_SOURCE_MODE:-visual}"
if [[ -n "${ODOM_GUESS_FRAME_ID:-}" ]]; then
  echo "wheel-guess replay blocked: independent visual/reference TF trees are not implemented" >&2
  exit 2
fi
rtabmap_extra_args=''
rtabmap_odom_extra_args=''
case "$rtabmap_profile" in
  baseline) ;;
  low-texture)
    rtabmap_extra_args='--Vis/MaxFeatures 2000 --Vis/MinInliers 10 --Vis/GridRows 3 --Vis/GridCols 4 --GFTT/QualityLevel 0.0001 --GFTT/MinDistance 3'
    rtabmap_odom_extra_args='--Odom/ResetCountdown 5 --OdomF2M/MaxSize 3000'
    ;;
  *)
    echo "unknown RTAB-Map profile: ${rtabmap_profile}" >&2
    exit 2
    ;;
esac
odom_extra_launch_args=()
if [[ -n "$rtabmap_odom_extra_args" ]]; then
  odom_extra_launch_args=("odom_args:=${rtabmap_odom_extra_args}")
fi
odom_guess_launch_arg=()
wait_for_transform_s=0.3
bag_play_rate=0.5
rtabmap_frame_id=camera_link
visual_odometry=true
# The replay already owns odom -> base -> camera_link. A second parent for
# camera_link would mix the recorded wheel tree with visual estimation.
publish_tf_odom=false
rtabmap_odom_topic=/rtabmap/odom
playback_bag=/data
if [[ "$odom_source_mode" == external ]]; then
  rtabmap_frame_id=base_link
  visual_odometry=false
  rtabmap_odom_topic=/odom
elif [[ "$odom_source_mode" != visual ]]; then
  echo "unknown odometry source mode: ${odom_source_mode}" >&2
  exit 2
fi

if [[ "$odom_source_mode" == visual && -z "${ODOM_GUESS_FRAME_ID:-}" ]]; then
  # Retain camera-internal TF and wheel odometry messages for comparison,
  # but let visual odometry be the only TF parent of camera_link.
  python3 /workspace/jdamr_cube_vslam/scripts/prepare_visual_replay.py \
    --input /data --output /output/visual_input \
    >/output/visual_replay_preparation.log
  playback_bag=/output/visual_input
  publish_tf_odom=true
fi
if [[ -n "${ODOM_GUESS_FRAME_ID:-}" || "$odom_source_mode" == external ]]; then
  wait_for_transform_s=1.5
  bag_play_rate=0.5
fi
if [[ -n "${ODOM_GUESS_FRAME_ID:-}" ]]; then
  odom_guess_launch_arg=(
    "odom_guess_frame_id:=${ODOM_GUESS_FRAME_ID}"
    "odom_guess_min_translation:=${ODOM_GUESS_MIN_TRANSLATION:-0.005}"
    "odom_guess_min_rotation:=${ODOM_GUESS_MIN_ROTATION:-0.005}"
  )
fi

cleanup() {
  set +e
  for pid in "${bag_pid:-}" "${snapshot_pid:-}" "${recorder_pid:-}" \
    "${launch_pid:-}" \
    "${camera_tf_pid:-}"; do
    if [[ -n "$pid" ]]; then
      kill -INT -- "-$pid" 2>/dev/null
    fi
  done
  wait "${recorder_pid:-}" 2>/dev/null
  wait "${snapshot_pid:-}" 2>/dev/null
  wait "${launch_pid:-}" 2>/dev/null
  wait "${camera_tf_pid:-}" 2>/dev/null
  wait "${bag_pid:-}" 2>/dev/null
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ -n "${ODOM_GUESS_FRAME_ID:-}" || "$odom_source_mode" == external ]]; then
  for variable in CAMERA_MOUNT_PARENT CAMERA_MOUNT_CHILD CAMERA_MOUNT_X \
    CAMERA_MOUNT_Y CAMERA_MOUNT_Z CAMERA_MOUNT_ROLL CAMERA_MOUNT_PITCH \
    CAMERA_MOUNT_YAW; do
    if [[ -z "${!variable:-}" ]]; then
      echo "missing measured camera mount value: ${variable}" >&2
      exit 1
    fi
  done
  setsid ros2 run tf2_ros static_transform_publisher \
    --x "$CAMERA_MOUNT_X" --y "$CAMERA_MOUNT_Y" --z "$CAMERA_MOUNT_Z" \
    --roll "$CAMERA_MOUNT_ROLL" --pitch "$CAMERA_MOUNT_PITCH" \
    --yaw "$CAMERA_MOUNT_YAW" \
    --frame-id "$CAMERA_MOUNT_PARENT" \
    --child-frame-id "$CAMERA_MOUNT_CHILD" \
    >/output/camera_mount_tf.log 2>&1 &
  camera_tf_pid=$!
fi

setsid ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  args:="-d --Mem/IncrementalMemory true --Mem/InitWMWithAllNodes false \
    --Reg/Force3DoF true \
    --Vis/MinInliers 15 --RGBD/NeighborLinkRefining true \
    --RGBD/ProximityBySpace true --Grid/Sensor 1 \
    --Grid/3D true --Grid/RangeMax 5.0 --Rtabmap/DetectionRate 2.0 \
    ${rtabmap_extra_args}" \
  "${odom_extra_launch_args[@]}" \
  database_path:="$database_path" \
  frame_id:="$rtabmap_frame_id" \
  "${odom_guess_launch_arg[@]}" \
  map_frame_id:=map \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  odom_topic:="$rtabmap_odom_topic" \
  vo_frame_id:=vslam_odom \
  depth:=true visual_odometry:="$visual_odometry" icp_odometry:=false \
  publish_tf_odom:="$publish_tf_odom" \
  subscribe_scan:=false \
  rgbd_sync:=true approx_rgbd_sync:=true approx_sync:=true \
  approx_sync_max_interval:=0.03 \
  qos:=2 topic_queue_size:=30 sync_queue_size:=30 \
  odom_always_process_most_recent_frame:=false \
  wait_for_transform:="$wait_for_transform_s" \
  rtabmap_viz:=false rviz:=false \
  >"$rtabmap_log" 2>&1 &
launch_pid=$!

if [[ "$odom_source_mode" == visual ]]; then
  setsid python3 \
    /workspace/jdamr_cube_vslam/jdamr_cube_vslam/trajectory_csv_recorder.py \
    --output-dir /output \
    --visual-topic /rtabmap/odom \
    --reference-topic /odom \
    >"$trajectory_log" 2>&1 &
  recorder_pid=$!
fi

setsid python3 \
  /workspace/jdamr_cube_vslam/jdamr_cube_vslam/rgbd_snapshot_ply.py \
  --output-ply /output/rgbd_snapshot.ply \
  --output-json /output/rgbd_snapshot.json \
  >/output/rgbd_snapshot.log 2>&1 &
snapshot_pid=$!

sleep 5
if ! kill -0 "$launch_pid" 2>/dev/null; then
  echo "RTAB-Map launch exited before replay; see ${rtabmap_log}" >&2
  exit 1
fi
setsid ros2 bag play "$playback_bag" --clock --rate "$bag_play_rate" \
  --read-ahead-queue-size 2000 \
  >/output/bag_play.log 2>&1 &
bag_pid=$!

if [[ -n "${ODOM_GUESS_FRAME_ID:-}" || "$odom_source_mode" == external ]]; then
  if ! kill -0 "$camera_tf_pid" 2>/dev/null; then
    echo "camera mount static TF publisher exited early" >&2
    exit 1
  fi
  tf_check_from_frame="${ODOM_GUESS_FRAME_ID:-$CAMERA_MOUNT_PARENT}"
  if ! python3 /workspace/jdamr_cube_vslam/scripts/wait_for_tf.py \
      --from-frame "$tf_check_from_frame" \
      --to-frame "$CAMERA_MOUNT_CHILD" \
      --timeout 10 \
      --use-sim-time \
      >/output/camera_mount_tf_check.log; then
    cat /output/camera_mount_tf_check.log >&2
    exit 1
  fi
fi

wait "$bag_pid"
bag_pid=''
sleep 5

cleanup
trap - EXIT

if grep -Eq 'ParameterNotDeclaredException|\[ERROR\].*process has died.*rtabmap' \
    "$rtabmap_log"; then
  echo "RTAB-Map node failed; see ${rtabmap_log}" >&2
  exit 1
fi

if [[ -n "${ODOM_GUESS_FRAME_ID:-}" ]] \
    && ! grep -Fq \
      "guess_frame_id         = ${ODOM_GUESS_FRAME_ID}" "$rtabmap_log"; then
  echo "RTAB-Map did not activate the requested odometry guess frame" >&2
  exit 1
fi

if [[ -n "${ODOM_GUESS_FRAME_ID:-}" ]]; then
  for expected in \
    "odom_frame_id          = vslam_odom" \
    "guess_min_translation  = ${ODOM_GUESS_MIN_TRANSLATION:-0.005}" \
    "guess_min_rotation     = ${ODOM_GUESS_MIN_ROTATION:-0.005}"; do
    if ! grep -Fq "$expected" "$rtabmap_log"; then
      echo "RTAB-Map did not activate requested odometry guess threshold: ${expected}" >&2
      exit 1
    fi
  done
fi

if [[ ! -s "$database_path" ]]; then
  echo "RTAB-Map database was not created; see ${rtabmap_log}" >&2
  exit 1
fi

trajectory_ready=false
if [[ "$odom_source_mode" == external ]]; then
  trajectory_ready=true
elif [[ -f /output/visual_trajectory.csv ]] \
    && [[ $(wc -l </output/visual_trajectory.csv) -gt 1 ]]; then
  trajectory_ready=true
fi

if [[ "$trajectory_ready" == true ]] \
    && command -v rtabmap-export >/dev/null 2>&1; then
  set +e
  rtabmap-export --cloud --poses --poses_format 10 \
    --decimation 2 --voxel 0.02 --noise_radius 0.05 --noise_k 5 \
    --output jdamr_rgbd --output_dir /output "$database_path" \
    >/output/rtabmap_export.log 2>&1
  export_status=$?
  if [[ $export_status -ne 0 ]]; then
    printf '\nFiltered export failed; retrying without radius-noise filtering.\n' \
      >>/output/rtabmap_export.log
    rm -f /output/jdamr_rgbd_cloud.ply /output/jdamr_rgbd_poses.txt
    rtabmap-export --cloud --poses --poses_format 10 \
      --decimation 2 --voxel 0.02 \
      --output jdamr_rgbd --output_dir /output "$database_path" \
      >>/output/rtabmap_export.log 2>&1
    export_status=$?
  fi
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
