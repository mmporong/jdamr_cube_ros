#!/bin/bash
# ±45° 랜덤 회전 큐브 수집. 목표 50편.
#
# 4워커 병렬은 성공률 46%로 떨어졌다(단일 워커 시험은 5/5). 시뮬 부하로
# 카메라 프레임이 지연되면 비주얼 서보잉이 흔들린다. 2워커로 낮춘다.
# 타임아웃도 올린다 — 192mm 옮겨 성공해놓고 3분 상한에 잘린 편이 있었다.
TOOLS="$(cd "$(dirname "$0")" && pwd)"
TARGET=50
ATTEMPTS=90                          # 성공률 감안한 시도 횟수
export COLLECT_OUT=rule_yaw45
export COLLECT_YAW_MAX=0.785
export COLLECT_EP_TIMEOUT=300        # 시뮬 5분
OUT="$TOOLS/logs/$COLLECT_OUT"
say() { echo "[$(date +%H:%M)] $*"; }

say "2워커로 ${ATTEMPTS}회 시도 (목표 ${TARGET}편, 현재 $(ls -d "$OUT"/ep* 2>/dev/null | wc -l)편)"
bash "$TOOLS/parallel_collect.sh" $ATTEMPTS 2

N=$(ls -d "$OUT"/ep* 2>/dev/null | wc -l)
say "수집 결과: ${N}편"
python3 - << 'PY'
import json, glob, os, numpy as np
T = os.path.expanduser('~/jdamr_cube_ws/src/jdamr_cube_ros/capstone_pick/tools')
ms = [json.load(open(m)) for m in sorted(glob.glob(f'{T}/logs/rule_yaw45/ep*/meta.json'))]
if ms:
    y = [abs(np.degrees(m.get('spawn_yaw', 0))) for m in ms]
    mv = [m['moved_mm'] for m in ms]; fr = [m['frames'] for m in ms]
    print(f'  |yaw| {min(y):.0f}~{max(y):.0f}°  이동 {min(mv):.0f}~{max(mv):.0f}mm  '
          f'길이 {min(fr)/20:.1f}~{max(fr)/20:.1f}초  ({len(ms)}편)')
PY
