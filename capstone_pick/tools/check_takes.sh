#!/bin/bash
# 각 데모 테이크가 한 번에 끝났는지(재시도·재물림·미검출 없이) 확인한다.
for n in demo_floor_pick demo_tilted_pick demo_color_pick; do
  L=~/capstone_tools/logs/rec_${n}.log
  [ -e "$L" ] || { echo "$n: 로그 없음"; continue; }
  CYC=$(grep -c '비전 접근 주행 (사이클' "$L")
  MISS=$(grep -c '물체 미검출' "$L")
  REGRIP=$(grep -c '재물림' "$L")
  FAILAP=$(grep -c '접근 실패' "$L")
  RES=$(grep -oE 'PICK_(SUCCESS|FAIL)' "$L" | tail -1)
  ANG=$(grep -oE '파지 검증: 그리퍼 각도=[-0-9.]+' "$L" | tail -1 | grep -oE '[-0-9.]+$')
  echo "$n: $RES 물림각=$ANG | 접근사이클=$CYC 미검출=$MISS 재물림=$REGRIP 접근실패=$FAILAP"
done
