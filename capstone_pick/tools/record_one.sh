#!/bin/bash
# 데모 하나만 다시 녹화한다. 사용: record_one.sh <이름> <색> <yaw> [pick 추가인자...]
#   예) record_one.sh demo_tilted_pick blue 0.35
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
ASSETS=~/gazebo-so101-capstone/assets
OUT=~/capstone_tools/logs
NAME=$1; COLOR=$2; YAW=$3; shift 3
POSE="0.34 1.45 0.75 0 0.42 -1.571"

python3 ~/capstone_tools/reset_and_stage.py tricolor_stage.py > /dev/null 2>&1 || { echo "무대 배치 실패 — 중단"; exit 1; }
gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'name: "jdamr_cube" position {x: 0 y: 0 z: 0.05} orientation {w: 1}' > /dev/null 2>&1
sleep 2
HW=$(python3 -c "import math; print(math.cos(${YAW}/2))")
HZ=$(python3 -c "import math; print(math.sin(${YAW}/2))")
gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 5000 --req "name: \"pick_${COLOR}\" position {x: 0.681 y: 0 z: 0.015} orientation {w: ${HW} z: ${HZ}}" > /dev/null 2>&1
sleep 2

UNIT="capstone-rec-${NAME}"
systemctl --user stop "$UNIT" 2>/dev/null
systemctl --user reset-failed "$UNIT" 2>/dev/null
rm -f /tmp/frames/f_*.png 2>/dev/null
sleep 1
systemd-run --user --unit="$UNIT" --setenv=REC_POSE="$POSE" --setenv=REC_SEC=190 \
  bash -lc "source /opt/ros/jazzy/setup.bash; source ~/jdamr_cube_ws/install/setup.bash; python3 ~/capstone_tools/recorder.py" > /dev/null 2>&1
sleep 6
timeout 420 ros2 run capstone_pick pick --ros-args -p speed_scale:=4.0 -p target_color:=${COLOR} "$@" > "$OUT/rec_${NAME}.log" 2>&1
sleep 3
systemctl --user stop "$UNIT" 2>/dev/null
sleep 2
N=$(ls /tmp/frames/f_*.png 2>/dev/null | wc -l)
RES=$(grep -oE 'PICK_(SUCCESS|FAIL)' "$OUT/rec_${NAME}.log" | tail -1)
ANG=$(grep -oE '파지 검증: 그리퍼 각도=[-0-9.]+' "$OUT/rec_${NAME}.log" | tail -1 | grep -oE '[-0-9.]+$')
POS=$(gz model -m "pick_${COLOR}" -p 2>/dev/null | grep -A2 Pose | sed -n 2p | tr -d '[]' | tr -s ' ')
if [ "$RES" = "PICK_SUCCESS" ]; then
  python3 ~/capstone_tools/make_gif.py "$ASSETS/${NAME}.gif" 6 > /dev/null 2>&1
  SZ=$(du -h "$ASSETS/${NAME}.gif" | cut -f1)
  echo "${NAME}: ${RES} 물림각=${ANG} 프레임=${N} 크기=${SZ} 큐브=${POS}"
else
  echo "${NAME}: ${RES} 물림각=${ANG} — 실패라 GIF를 만들지 않았다. 큐브=${POS}"
fi
