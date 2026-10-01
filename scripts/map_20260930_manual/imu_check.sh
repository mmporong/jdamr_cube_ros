#!/bin/bash
# After a go: board axes, offline odom+IMU EKF, and odom/gyro/EKF rotations against scans
# for one run's onboard bag. PC only (ROS domain 89 localhost via ekf_replay.sh).
# Usage: imu_check.sh <run_dir>   -> <run_dir>/imu_check/{axes.jsonl,rotation.txt}
set -eu
RUN=$1
BAG=$RUN/onboard_bag
OUT=$RUN/imu_check
HERE=$(cd "$(dirname "$0")" && pwd)
test -e "$BAG/metadata.yaml" || { echo "no onboard bag in $RUN" >&2; exit 2; }
mkdir -p "$OUT"
set +u; source /opt/ros/jazzy/setup.bash; set -u
timeout 600 python3 "$HERE/imu_axes.py" "$BAG" > "$OUT/axes.jsonl"
rm -rf "$OUT/ekf"
timeout 900 bash "$HERE/ekf_replay.sh" "$BAG" "$OUT/ekf" > /dev/null
timeout 900 python3 "$HERE/rotation_truth.py" --ekf="$OUT/ekf/filtered" "$BAG" > "$OUT/rotation.txt" 2>/dev/null
cat "$OUT/axes.jsonl" "$OUT/rotation.txt"
