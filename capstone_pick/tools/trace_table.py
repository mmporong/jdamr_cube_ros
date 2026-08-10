"""추적 기록(trace.jsonl)을 시행 비교 표로 편다.

로그를 길게 찍으면 한 번의 실패는 읽을 수 있어도, 5회를 나란히 놓고 "어느 단계에서
갈렸나"는 볼 수 없다. 단계마다 남긴 한 줄을 표로 펴서 그걸 본다.

핵심은 마지막 열이다 — **코드의 판정과 실좌표가 어긋난 줄**을 표시한다.
그런 줄이 있으면 성공률 숫자 자체를 믿으면 안 된다(실제로 그런 적이 있다:
로그는 전 구간 HOLDING인데 큐브는 시작 위치 바닥 그대로였다).

사용:
    python3 trace_table.py [trace.jsonl]     기본 ~/capstone_tools/logs/trace.jsonl
"""
import json
import math
import os
import sys

TRASH = (0.15, 0.62)      # 휴지통 중심 [m, world]
TRASH_R = 0.08            # 안쪽 반경 — 이 안이면 들어간 것
FLOOR_Z = 0.06            # 이보다 낮으면 바닥에 있다


def load(path):
    runs, cur = [], []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            if r.get('stage') == '시작' and cur:
                runs.append(cur)
                cur = []
            cur.append(r)
    if cur:
        runs.append(cur)
    return runs


def truth_of(r):
    """실좌표가 말하는 큐브 상태."""
    c = r.get('cube_gz')
    if not c:
        return '?'
    if math.hypot(c[0] - TRASH[0], c[1] - TRASH[1]) < TRASH_R:
        return '통 안'
    return '바닥' if c[2] < FLOOR_Z else f'들림 z={c[2]:.2f}'


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.expanduser('~/capstone_tools/logs/trace.jsonl')
    if not os.path.exists(path):
        print(f'기록 없음: {path}')
        return
    runs = load(path)
    print(f'{path} — 시행 {len(runs)}회\n')
    bad = 0
    for i, run in enumerate(runs, 1):
        last = run[-1]
        claim = last.get('코드주장', '?')
        final = truth_of(last)
        mismatch = (claim == 'PICK_SUCCESS') != (final == '통 안')
        bad += mismatch
        head = f'── 시행 {i}: 코드주장={claim}  실제={final}'
        print(head + ('   ※ 어긋남' if mismatch else ''))
        print(f'   {"t[s]":>6} {"단계":<12} {"판정":<10} {"죠 x/z":>14} '
              f'{"부하":>7} {"큐브 실좌표":>22}  상태')
        for r in run:
            jaw = r.get('jaw')
            jaw_s = '—' if not jaw else f'{jaw[0]:+.3f}/{jaw[2]:+.3f}'
            load_v = r.get('grip_load')
            load_s = '—' if load_v is None else f'{load_v:+.2f}'
            c = r.get('cube_gz')
            c_s = '—' if not c else f'({c[0]:+.3f},{c[1]:+.3f},{c[2]:+.3f})'
            verdict = r.get('판정') or r.get('코드주장') or ''
            print(f'   {r.get("t", 0):6.1f} {r.get("stage", ""):<12} {verdict:<10} '
                  f'{jaw_s:>14} {load_s:>7} {c_s:>22}  {truth_of(r)}')
        print()
    ok = sum(1 for r in runs if truth_of(r[-1]) == '통 안')
    print(f'=== 실좌표 기준 {ok}/{len(runs)} ===')
    if bad:
        print(f'=== 코드 판정과 실제가 어긋난 시행 {bad}건 — 판정 신호를 먼저 고칠 것 ===')


if __name__ == '__main__':
    main()
