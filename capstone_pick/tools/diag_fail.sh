#!/bin/bash
# 실패 회차의 정렬 수렴 상태와 놓친 지점을 본다.
for i in "$@"; do
  L=~/capstone_tools/logs/gate10_${i}.log
  [ -e "$L" ] || { echo "[$i] 로그 없음"; continue; }
  echo "===== [$i] ====="
  grep -E '손목캠 정렬|정렬 수렴|롤 먼저|밀어 넣기|파지 검증|파지 확인|들기 후|PICK_' "$L" \
    | sed 's/.*capstone_pick.: //' | tail -16
done
