#!/bin/bash
# 캡스톤 시뮬 스택 정지 — 작업이 끝나면 반드시 실행할 것.
# systemd 유저 유닛은 터미널·에이전트 세션을 닫아도 계속 돌며, 헤드리스 시뮬은
# 화면에 안 보이는 채 CPU 1.3코어+를 상시 소모한다(실사례: 팬 폭주, 재부팅으로야 발견).
systemctl --user stop capstone-sim capstone-gui capstone-ui 2>/dev/null
systemctl --user reset-failed capstone-sim capstone-gui capstone-ui 2>/dev/null
sleep 2
LEFT=$(pgrep -c -f "gz sim" 2>/dev/null || echo 0)
if [ "${LEFT:-0}" -gt 0 ]; then
  pkill -f "gz sim"
  echo "잔여 gz 프로세스 ${LEFT}개 강제 종료"
fi
echo "스택 정지 완료 (확인: pgrep -af 'gz sim')"
