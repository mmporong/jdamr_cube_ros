#!/bin/bash
# 작동 검증된 기능들을 GIF로 녹화한다(포트폴리오용).
# 카메라 pose는 무대마다 다르므로 REC_POSE로 넘긴다 — 안 맞으면 로봇이 화면 밖으로 난다.
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
ASSETS=~/gazebo-so101-capstone/assets
OUT=~/capstone_tools/logs
mkdir -p "$ASSETS" "$OUT"
LOG="$OUT/record_demos.txt"
: > "$LOG"

reset_robot() {
  gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 5000 --req 'name: "jdamr_cube" position {x: 0 y: 0 z: 0.05} orientation {w: 1}' > /dev/null 2>&1
  sleep 2
}

place_cube() {   # $1=색 $2=x $3=yaw
  HW=$(python3 -c "import math; print(math.cos($3/2))")
  HZ=$(python3 -c "import math; print(math.sin($3/2))")
  gz service -s /world/room/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 5000 --req "name: \"pick_$1\" position {x: $2 y: 0 z: 0.015} orientation {w: ${HW} z: ${HZ}}" > /dev/null 2>&1
  sleep 2
}

cube_pos() {
  gz model -m "pick_$1" -p 2>/dev/null | grep -A2 Pose | sed -n 2p | tr -d '[]' | tr -s ' '
}

record() {   # $1=이름 $2=REC_POSE $3=녹화초 $4=GIF스텝 $5.. = pick 인자
  NAME=$1; POSE=$2; SEC=$3; STEP=$4; shift 4
  # 유닛 이름을 데모마다 다르게 둔다 — 같은 이름으로 다시 띄우면 systemd-run이
  # "already exists"로 실패하고, 그러면 직전 녹화 프레임이 그대로 남아 GIF가
  # 세 개 모두 같은 파일이 된다(실측: 313프레임/263889바이트 동일).
  UNIT="capstone-rec-${NAME}"
  systemctl --user stop "$UNIT" 2>/dev/null
  systemctl --user reset-failed "$UNIT" 2>/dev/null
  rm -f /tmp/frames/f_*.png 2>/dev/null
  sleep 1
  systemd-run --user --unit="$UNIT" \
    --setenv=REC_POSE="$POSE" --setenv=REC_SEC="$SEC" \
    bash -lc "source /opt/ros/jazzy/setup.bash; source ~/jdamr_cube_ws/install/setup.bash; python3 ~/capstone_tools/recorder.py" > /dev/null 2>&1
  sleep 6
  timeout 420 ros2 run capstone_pick pick --ros-args -p speed_scale:=4.0 "$@" > "$OUT/rec_${NAME}.log" 2>&1
  sleep 3
  systemctl --user stop "$UNIT" 2>/dev/null
  sleep 2
  N=$(ls /tmp/frames/f_*.png 2>/dev/null | wc -l)
  python3 ~/capstone_tools/make_gif.py "$ASSETS/${NAME}.gif" "$STEP" > /dev/null 2>&1
  SZ=$(du -h "$ASSETS/${NAME}.gif" 2>/dev/null | cut -f1)
  RES=$(grep -oE 'PICK_(SUCCESS|FAIL)' "$OUT/rec_${NAME}.log" | tail -1)
  ANG=$(grep -oE '파지 검증: 그리퍼 각도=[-0-9.]+' "$OUT/rec_${NAME}.log" | tail -1 | grep -oE '[-0-9.]+$')
  echo "${NAME}: ${RES} 물림각=${ANG} 프레임=${N} 크기=${SZ}" >> "$LOG"
}

# 1) 바닥 파지 → 들기 → 옮겨 놓기 (게이트1에서 10/10 검증된 핵심 동작)
#    place가 y+ 쪽이라 카메라를 y+에 둬서 내려놓는 순간이 가려지지 않게 한다.
python3 ~/capstone_tools/reset_and_stage.py tricolor_stage.py > /dev/null 2>&1 || { echo "무대 배치 실패 — 중단"; exit 1; }
reset_robot
place_cube blue 0.681 0.0
echo "1) 바닥 파지 시작 (큐브 $(cube_pos blue))" >> "$LOG"
record demo_floor_pick "0.34 1.45 0.75 0 0.42 -1.571" 190 6

# 2) 기울어진 큐브 (손목 롤 정렬) — yaw 0.35rad = 20도
python3 ~/capstone_tools/reset_and_stage.py tricolor_stage.py > /dev/null 2>&1 || { echo "무대 배치 실패 — 중단"; exit 1; }
reset_robot
place_cube blue 0.681 0.35
echo "2) 기울기 20도 파지 시작 (큐브 $(cube_pos blue))" >> "$LOG"
record demo_tilted_pick "0.34 1.45 0.75 0 0.42 -1.571" 190 6

# 3) 색 지정 — 3색 무대에서 초록만 집기
python3 ~/capstone_tools/reset_and_stage.py tricolor_stage.py > /dev/null 2>&1 || { echo "무대 배치 실패 — 중단"; exit 1; }
reset_robot
place_cube green 0.681 -0.2
echo "3) 색 지정(초록) 파지 시작 (큐브 $(cube_pos green))" >> "$LOG"
record demo_color_pick "0.34 1.45 0.75 0 0.42 -1.571" 190 6 -p target_color:=green

echo DONE >> "$LOG"
cat "$LOG"
