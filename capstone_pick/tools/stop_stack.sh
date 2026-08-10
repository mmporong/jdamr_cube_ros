#!/bin/bash
# 캡스톤 시뮬 스택 정지 — 작업이 끝나면 반드시 실행할 것.
# systemd 유저 유닛은 터미널·에이전트 세션을 닫아도 계속 돌며, 헤드리스 시뮬은
# 화면에 안 보이는 채 CPU 1.3코어+를 상시 소모한다(실사례: 팬 폭주, 13시간 방치).
#
# 종전 판은 'gz sim'만 봤다. 그래서 launch가 띄운 나머지 — robot_state_publisher,
# 브리지 3종, 스포너 — 가 systemd(ppid 1)로 재부모화돼 살아남았고, 다음 실행에서
# **두 번째 robot_state_publisher**가 되어 /tf에 서로 다른 변환을 동시에 뿌렸다.
# 실측(2026-08-07): rsp 3개 동시 기동, /tf 퍼블리셔 2개 → "TF 대기 실패 → 물체
# 미검출"이 검증 4회 중 2회를 통째로 날렸다. 이 스크립트가 남기는 것이 없어야 한다.
#
# pkill -f는 쓰지 않는다 — 패턴이 자기 명령줄에 걸려 셸까지 죽인 적이 있다(실측).
# ps로 PID를 뽑아 개별 kill 한다.
set -o pipefail

# launch가 띄우는 것 전부. robot_state_publisher는 명령줄에 패키지명이 없어
# 'jdamr' 패턴에 안 걸리므로 실행 파일명으로 따로 잡아야 한다.
PATTERN='gz sim|gz-sim|ros_gz_bridge|ros_gz_image|parameter_bridge|image_bridge'
PATTERN+='|robot_state_publisher|controller_manager/spawner|jdamr_cube_gazebo'
PATTERN+='|capstone_gui\.py|run capstone_pick|ros_gz_sim/create'

# 자기 자신과 조상 프로세스는 절대 건드리지 않는다. 호출한 셸의 명령줄에 'run
# capstone_pick' 같은 문자열이 들어 있으면 패턴에 걸려 자기를 죽인다 — 실제로
# 그렇게 셸이 exit 144로 죽었다. 이름 패턴 하나로 거르는 방식의 고질적 함정이라
# (pkill -f를 안 쓰는 이유와 같다) PID 계보로 확실히 뺀다.
SELF_CHAIN=" "
_p=$$
while [ -n "$_p" ] && [ "$_p" -gt 1 ] 2>/dev/null; do
  SELF_CHAIN="$SELF_CHAIN$_p "
  _p=$(ps -o ppid= -p "$_p" 2>/dev/null | tr -d ' ')
done

sim_pids() {
  ps -eo pid,args --no-headers \
    | awk -v pat="$PATTERN" -v skip="$SELF_CHAIN" \
      '$0 ~ pat && $0 !~ /awk|stop_stack/ && index(skip, " " $1 " ") == 0 {print $1}'
}

systemctl --user stop capstone-sim capstone-gui capstone-ui capstone-night capstone-scan 2>/dev/null
systemctl --user reset-failed capstone-sim capstone-gui capstone-ui capstone-night capstone-scan 2>/dev/null
sleep 2

for sig in TERM TERM KILL; do
  pids=$(sim_pids)
  [ -z "$pids" ] && break
  for p in $pids; do kill -"$sig" "$p" 2>/dev/null; done
  sleep 3
done

left=$(sim_pids | wc -l)
if [ "$left" -eq 0 ]; then
  echo "스택 정지 완료 — 잔여 프로세스 없음"
else
  echo "경고: ${left}개가 남았다"
  ps -eo pid,args --no-headers | awk -v pat="$PATTERN" '$0 ~ pat && $0 !~ /awk|stop_stack/ {print "  " $0}' | cut -c1-110
fi
