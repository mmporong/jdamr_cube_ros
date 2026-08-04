#!/bin/bash
# LeRobot 시연 50 에피소드 일괄 수집. 사용: lerobot_collect_all.sh [시작=0] [끝=49]
# 종료 시 스택 자동 정지 (팬 폭주 방지).
# set -u는 ROS setup.bash(AMENT_TRACE_SETUP_FILES 미정의)와 충돌
S=${1:-0}; E=${2:-49}
TOOLS="$(cd "$(dirname "$0")" && pwd)"
LOG="$TOOLS/logs/collect/batch.log"
mkdir -p "$TOOLS/logs/collect"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
# 시뮬이 없으면 기동 (단일 검증 유닛의 종료 stop_stack이 본 배치의 시뮬을 끄는
# 사고 실측 — 배치는 자기 시뮬을 스스로 보장한다)
if [ "$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c active)" != "3" ]; then
  echo "시뮬 기동 중..." | tee -a "$LOG"
  systemd-run --user --collect --setenv=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST --setenv=GZ_PARTITION=lim-capstone --unit=capstone-sim bash -c 'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && exec ros2 launch jdamr_cube_gazebo gazebo.launch.py gui:=false world:=$HOME/jdamr_cube_ws/install/jdamr_cube_gazebo/share/jdamr_cube_gazebo/worlds/room.world' 2>/dev/null
  for i in $(seq 1 30); do
    sleep 5
    [ "$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c active)" = "3" ] && break
  done
fi
for ep in $(seq $S $E); do
  if [ -f "$TOOLS/logs/collect/ep$ep/meta.json" ]; then
    echo "[ep$ep] 이미 수집됨 — 건너뜀" | tee -a "$LOG"; continue
  fi
  if [ ! -f "$TOOLS/logs/ep${ep}_rad.npy" ]; then
    ~/miniforge3/envs/lerobot/bin/python "$TOOLS/lerobot_extract.py" $ep >> "$LOG" 2>&1 || { echo "[ep$ep] 추출 실패" | tee -a "$LOG"; continue; }
  fi
  echo "[ep$ep] 수집 시작 $(date +%H:%M:%S)" | tee -a "$LOG"
  timeout 400 python3 -u "$TOOLS/lerobot_replay.py" --ep $ep --auto --record >> "$LOG" 2>&1
  tail -2 "$LOG" | grep -E "수집|레고 최종" || true
done
echo "== 일괄 수집 종료 $(date +%H:%M:%S) ==" | tee -a "$LOG"
bash "$TOOLS/stop_stack.sh"
