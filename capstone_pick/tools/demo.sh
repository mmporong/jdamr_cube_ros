#!/bin/bash
# 데모/테스트 실행 — 시뮬 화면 + 카메라 + 조작 패널을 한 번에 띄운다.
#
#   ~/capstone_tools/demo.sh          기동
#   ~/capstone_tools/stop_stack.sh    정지 (팬 안전 — 작업 후 반드시)
#
# 조작 패널에서 색·놓을 곳을 고르고 [실행]을 누르면 파이프라인이 돈다.
# 실행할 때마다 로봇·큐브가 초기 배치로 되돌아가므로 결과를 비교할 수 있다.
#
# 카메라 화면은 Gazebo 창 안에 도킹된다(~/.gz/sim/8/gui.config의 ImageDisplay).
# 별도 rqt 창을 여러 개 띄우면 중복 실행되기 쉬워 하나의 GUI로 모았다.
#
# DDS 격리(LOCALHOST + GZ_PARTITION)는 강의실 LAN 오염 차단용 필수 설정이다.
set -o pipefail
TOOLS="$(cd "$(dirname "$0")" && pwd)"
WORLD="$HOME/jdamr_cube_ws/install/jdamr_cube_gazebo/share/jdamr_cube_gazebo/worlds/room.world"
ISOLATE=(--setenv=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST --setenv=GZ_PARTITION=lim-capstone)
export DISPLAY="${DISPLAY:-:1}"

say() { echo "[$(date +%H:%M)] $*"; }

# 남아 있는 노드부터 정리한다. 유닛만 멈추면 launch가 띄운 자식들이 systemd로
# 재부모화돼 살아남고, 특히 robot_state_publisher가 둘이 되면 /tf에 서로 다른
# 변환이 동시에 실려 물체를 봐도 좌표 변환이 안 된다(실측: 검증 4회 중 2회 전멸).
# stop_stack.sh가 대상 전부를 잡고 잔여 0을 확인해 준다.
say "이전 실행 정리"
bash "$TOOLS/stop_stack.sh"

say "시뮬 기동 (화면 + 카메라 2대)"
systemd-run --user --collect "${ISOLATE[@]}" --setenv=DISPLAY="$DISPLAY" --unit=capstone-sim bash -c \
  "source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && \
   exec ros2 launch jdamr_cube_gazebo gazebo.launch.py gui:=true world:=$WORLD" >/dev/null 2>&1

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source ~/jdamr_cube_ws/install/setup.bash >/dev/null 2>&1
N=0
for i in $(seq 1 40); do
  N=$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c active)
  [ "${N:-0}" = "3" ] && break
  sleep 5
done
say "컨트롤러 ${N:-0}/3"
if [ "${N:-0}" != "3" ]; then
  say "시뮬 기동 실패 — journalctl --user -u capstone-sim 확인"
  exit 1
fi

say "조작 패널 기동"
setsid nohup python3 "$TOOLS/capstone_gui.py" >/dev/null 2>&1 < /dev/null &
sleep 3

cat <<'EOF'

  준비 완료. 화면에 창 2개가 떠 있습니다.

    Gazebo Sim      로봇·큐브 3색·휴지통, 좌측에 전방/손목 카메라 도킹
    캡스톤 조작 패널  색 선택 → 놓을 곳 선택 → [실행]

  테스트 요령
    · "주행 생략" 체크  : 큐브가 이미 팔 앞에 있는 상태에서 파지만 시험
    · 체크 해제         : 로봇이 큐브를 찾아가는 전체 흐름
    · 로그창 초록=성공/파지확인, 빨강=실패
    · [중지]는 프로세스 그룹째 끊으므로 확실히 멈춥니다

  끝나면  ~/capstone_tools/stop_stack.sh   (팬 안전)

EOF
