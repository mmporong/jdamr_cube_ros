#!/bin/bash
# 수집 성공률 감시. 떨어지면 즉시 멈춘다 — 편수보다 정확도가 우선이다.
#
# 실측: 단일 워커 5/5(100%), 4워커 6/13(46%). 병렬 부하가 커지면 카메라
# 프레임이 밀려 비주얼 서보잉이 흔들린다. 편수를 벌자고 쓰레기를 쌓지 않는다.
TOOLS="$(cd "$(dirname "$0")" && pwd)"
FLOOR=80          # 최근 10편 성공률 하한(%)
while systemctl --user is-active capstone-collect45b >/dev/null 2>&1; do
  sleep 120
  LINES=$(grep -hE "^\[ep" "$TOOLS"/logs/parallel/w[01].log 2>/dev/null | tail -10)
  N=$(echo "$LINES" | grep -c "ep")
  [ "$N" -lt 10 ] && continue
  OK=$(echo "$LINES" | grep -c "성공")
  RATE=$((OK * 10))
  echo "[$(date +%H:%M)] 최근 10편 성공률 ${RATE}% (${OK}/10)"
  if [ "$RATE" -lt "$FLOOR" ]; then
    echo "[$(date +%H:%M)] ⚠ 성공률 ${RATE}% < ${FLOOR}% — 수집 중단 (워커 축소 필요)"
    systemctl --user stop capstone-collect45b capstone-sim-w0 capstone-sim-w1 2>/dev/null
    break
  fi
done
echo "[$(date +%H:%M)] 품질 감시 종료"
