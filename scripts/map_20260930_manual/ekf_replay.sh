#!/bin/bash
# Offline EKF replay: bag /odom + /imu/data_raw -> imu_bias_relay -> robot_localization,
# records /odometry/filtered. PC only, ROS domain 89 localhost. Usage: ekf_replay.sh <bag> <out>
set -u
BAG=$1; OUT=$2
REPO=$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_navigation
set +u; source /opt/ros/jazzy/setup.bash; set -u
unset ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=89 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=$OUT/roslog RCUTILS_COLORIZED_OUTPUT=0
export PYTHONPATH=$REPO:$PYTHONPATH
mkdir -p $OUT
# Bags before 2026-10-01 have no imu_link in /tf_static; same rotation as the URDF imu_joint.
ros2 run tf2_ros static_transform_publisher --roll 3.141592653589793 --yaw 1.5707963267948966 --frame-id base_link --child-frame-id imu_link --ros-args -p use_sim_time:=true > $OUT/static.log 2>&1 &
P1=$!
python3 -m jdamr_cube_navigation.imu_bias_relay --ros-args -p use_sim_time:=true > $OUT/relay.log 2>&1 &
P2=$!
ros2 run robot_localization ekf_node --ros-args --params-file $REPO/config/ekf_odom_imu.yaml -r __node:=ekf_filter_node -p use_sim_time:=true > $OUT/ekf.log 2>&1 &
P3=$!
sleep 3
ros2 bag record -s mcap -o $OUT/filtered /odometry/filtered /imu/data > $OUT/record.log 2>&1 &
P4=$!
sleep 2
ros2 bag play $BAG --clock 50 --topics /odom /imu/data_raw /tf_static --disable-keyboard-controls > $OUT/play.log 2>&1
sleep 2
kill -TERM $P4; wait $P4
kill -TERM $P3 $P2 $P1; wait $P3 $P2 $P1 2>/dev/null
echo done
