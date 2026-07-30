#!/bin/bash
# 환경 점검·자동 복구. 스택이 죽거나 이상할 때 이것부터 돌린다.
#   ~/capstone_tools/doctor.sh          점검만
#   ~/capstone_tools/doctor.sh --fix    문제를 발견하면 복구까지
FIX=0
[ "${1:-}" = "--fix" ] && FIX=1
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/jdamr_cube_ws/install/setup.bash 2>/dev/null
BAD=0

say() { printf '%-34s %s\n' "$1" "$2"; }

# 1) systemd linger — 켜져 있지 않으면 세션이 닫힐 때 스택이 통째로 죽는다
LINGER=$(loginctl show-user "$USER" 2>/dev/null | grep -c 'Linger=yes')
if [ "$LINGER" = "1" ]; then
  say "systemd linger" "OK"
else
  say "systemd linger" "없음 (세션 종료 시 스택이 죽는다)"
  BAD=1
  [ $FIX = 1 ] && loginctl enable-linger "$USER" && say "  → linger 활성화" "완료"
fi

# 2) 유닛 상태
for u in capstone-sim capstone-gui capstone-ui; do
  ST=$(systemctl --user is-active "$u" 2>/dev/null)
  if [ "$ST" = "active" ]; then say "$u" "active"; else say "$u" "$ST"; BAD=1; fi
done

# 3) Gazebo 응답 (유닛이 살아 있어도 서버가 먹통일 수 있다)
if timeout 15 gz model --list >/dev/null 2>&1; then
  say "Gazebo 서버 응답" "OK"
else
  say "Gazebo 서버 응답" "무응답"
  BAD=1
fi

# 4) 컨트롤러
CTRL=$(timeout 15 ros2 control list_controllers 2>/dev/null | grep -c 'active')
if [ "${CTRL:-0}" -ge 3 ]; then say "ros2_control 컨트롤러" "$CTRL개 active"; else
  say "ros2_control 컨트롤러" "${CTRL:-0}개 (3개여야 함)"; BAD=1; fi

# 5) DDS 공유메모리 잔재 — 누적되면 디스커버리가 깨진다
SHM=$(ls /dev/shm 2>/dev/null | grep -c 'fastrtps\|fastdds')
# 정상 동작 중에도 60~80개를 쓴다(실측) — 임계를 낮게 잡으면 오탐이 난다
if [ "$SHM" -lt 150 ]; then say "DDS 공유메모리 잔재" "${SHM}개"; else
  say "DDS 공유메모리 잔재" "${SHM}개 (과다)"
  BAD=1
  [ $FIX = 1 ] && rm -rf /dev/shm/fastrtps* /dev/shm/sem.fastrtps* /dev/shm/fastdds* 2>/dev/null && \
    say "  → 잔재 정리" "완료"
fi

# 6) 부하 (GUI를 켠 채 두면 누적된다)
LOAD=$(awk '{printf "%.1f", $1}' /proc/loadavg)
if (( $(echo "$LOAD < 8" | bc -l) )); then say "부하(load avg)" "$LOAD"; else
  say "부하(load avg)" "$LOAD (과부하 — GUI를 닫아볼 것)"; BAD=1; fi

# 7) YOLO 모델·맵 등 자산
[ -f ~/capstone_tools/yolo_cubes.pt ] && say "YOLO 모델" "OK" || { say "YOLO 모델" "없음"; BAD=1; }
[ -f ~/maps/jdamr_cube_room.yaml ] && say "SLAM 맵" "OK" || { say "SLAM 맵" "없음"; BAD=1; }

# 8) numpy ABI — pip가 2.x로 올려놓으면 cv_bridge가 전부 깨진다
if python3 -c "import numpy,sys; sys.exit(0 if numpy.__version__.startswith('1.') else 1)" 2>/dev/null; then
  say "numpy 버전" "$(python3 -c 'import numpy; print(numpy.__version__)')"
else
  say "numpy 버전" "2.x — cv_bridge가 깨진다 (pip install 'numpy<2')"
  BAD=1
  [ $FIX = 1 ] && python3 -m pip install --user --break-system-packages 'numpy<2' -q && \
    say "  → numpy 되돌림" "완료"
fi

echo
if [ $BAD = 0 ]; then
  echo "진단: 정상"
else
  echo "진단: 문제 발견"
  if [ $FIX = 1 ]; then
    echo "스택을 다시 올린다..."
    ~/capstone_tools/start_stack.sh >/dev/null 2>&1
    for i in $(seq 1 16); do
      sleep 5
      if timeout 10 ros2 control list_controllers 2>/dev/null | grep -q 'gripper_controller.*active'; then
        echo "복구 완료 (컨트롤러 active)"; exit 0
      fi
    done
    echo "복구 실패 — WSL 자체를 의심할 것(Windows에서 wsl 클라이언트 프로세스 정리 후 재시도)"
    exit 1
  else
    echo "복구하려면: ~/capstone_tools/doctor.sh --fix"
  fi
fi
