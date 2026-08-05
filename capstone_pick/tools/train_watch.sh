#!/bin/bash
# 학습 감시 — 13분마다 loss를 기록하고, 정체되면 즉시 중단한다.
#
# 오늘 세 번의 학습을 100분씩 다 채우고 나서야 실패를 알았다. 2,500스텝
# (13분) 체크포인트만 봤어도 알 수 있던 것이다. 이 스크립트가 그 확인을
# 자동으로 한다: 개선이 멈추면 GPU를 더 태우지 않고 끊는다.
#
# 사용: train_watch.sh <태그> [정체_허용횟수=3]
TAG=${1:-v3}
PATIENCE=${2:-3}
TOOLS="$(cd "$(dirname "$0")" && pwd)"
TRAINLOG="$TOOLS/logs/act_rule_${TAG}_train.log"
WATCH="$TOOLS/logs/train_watch_${TAG}.log"

say() { echo "[$(date +%H:%M)] $*" | tee -a "$WATCH"; }
loss_now() { grep -oE "loss:[0-9.]+" "$TRAINLOG" 2>/dev/null | tail -1 | cut -d: -f2; }

say "감시 시작 (tag=$TAG, 정체 허용 ${PATIENCE}회)"
BEST=99; STALL=0
while systemctl --user is-active capstone-train >/dev/null 2>&1; do
  sleep 780          # 13분 = 체크포인트 주기(2500스텝)
  L=$(loss_now)
  [ -z "$L" ] && continue
  STEP=$(grep -oE "step:[0-9K]+" "$TRAINLOG" | tail -1 | cut -d: -f2)
  IMPROVED=$(python3 -c "print(1 if $L < $BEST - 0.005 else 0)")
  if [ "$IMPROVED" = "1" ]; then
    say "step $STEP  loss $L  (개선, 이전 최고 $BEST)"
    BEST=$L; STALL=0
  else
    STALL=$((STALL + 1))
    say "step $STEP  loss $L  (정체 ${STALL}/${PATIENCE}, 최고 $BEST)"
    if [ $STALL -ge $PATIENCE ]; then
      say "정체 ${PATIENCE}회 연속 — 학습 중단 (GPU 낭비 방지)"
      systemctl --user stop capstone-train
      break
    fi
  fi
done
say "감시 종료 (최종 최고 loss $BEST)"
