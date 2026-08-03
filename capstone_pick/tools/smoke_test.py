"""capstone_pick 회귀 테스트: 변경 후 이것만 돌리면 파지 성능 저하를 즉시 잡는다.

배경: 파지력·마찰·패드 같은 물리 파라미터를 바꿨을 때 파지가 완전히 망가졌는데
알아채기까지 열 번 넘게 시도했다. 각 단계를 시뮬 참값(물체 좌표)으로 검증해
어느 단계에서 깨졌는지 바로 보이게 한다.

사용: python3 smoke_test.py [--quick]
  전체: 파지 → 들기 → 옆에 놓기 → 쓰레기통 투입 (약 6분)
  quick: 파지 → 들기 까지만 (약 2분)
"""
import math
import os
import subprocess
import sys
import time

QUICK = '--quick' in sys.argv
TOOLS = os.path.dirname(os.path.abspath(__file__))  # 사용자명 하드코딩 금지
TRASH_CENTER = (0.34, 1.0)
TRASH_HALF_OPEN = 0.068


def sh(cmd, timeout=700):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                       timeout=timeout, executable='/bin/bash')
    return r.stdout + r.stderr


def pose(name):
    out = sh(f'gz model -m {name} -p', timeout=30)
    for ln in out.splitlines():
        ln = ln.strip()
        if ln.startswith('[') and ln.count(' ') >= 2:
            return [float(v) for v in ln.strip('[]').split()]
    return None


def stage(script, arg=''):
    # 배치 실패는 이후 판정을 전부 오염시키므로 즉시 죽는다 (sh()는 returncode를 버림)
    r = subprocess.run(
        f'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && '
        f'python3 {TOOLS}/{script} {arg}', shell=True, capture_output=True, text=True,
        timeout=120, executable='/bin/bash')
    if r.returncode != 0:
        print(f'무대 배치 실패({script}): {(r.stderr or r.stdout)[-200:]}')
        sys.exit(1)


def run_pick(extra=''):
    return sh('source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && '
              f'ros2 run capstone_pick pick --ros-args -p speed_scale:=4.0 {extra}')


results = []


def check(name, ok, detail):
    results.append((name, ok, detail))
    print(f'  [{"PASS" if ok else "FAIL"}] {name}: {detail}', flush=True)


print('=' * 64)
print('capstone_pick 회귀 테스트')
print('=' * 64)

# --- 1) 바닥 파지 + 들어올리기 ---------------------------------------------
print('\n[1] 바닥 파지 · 들어올리기 (물체가 실제로 바닥을 떠나는지)')
stage('reset_and_stage.py', 'floor_stage.py 0.3')
before = pose('pick_blue')
out = run_pick('-p skip_approach:=true -p floor:=true')
angle = None
for ln in out.splitlines():
    if '파지 검증' in ln and '각도=' in ln:
        try:
            angle = float(ln.split('각도=')[1].split()[0])
        except (ValueError, IndexError):
            pass
lifted = 'PICK_SUCCESS' in out
after = pose('pick_blue')
check('파지 성립', angle is not None and angle > 0.05,
      f'그리퍼 각도 {angle}' if angle is not None else '각도 로그 없음')
check('물체 이동', bool(before and after and
                    math.hypot(after[0] - before[0], after[1] - before[1]) > 0.05),
      f'{before[:2] if before else None} → {after[:2] if after else None}')
check('사이클 완주', lifted, 'PICK_SUCCESS' if lifted else 'PICK_FAIL')

if QUICK:
    print()
else:
    # --- 2) 비전 접근 포함 E2E (옆에 놓기) --------------------------------
    print('\n[2] 비전 접근 → 파지 → 옆에 놓기 (전체 자율 사이클)')
    stage('reset_and_stage.py', 'floor_stage.py 0.3')
    before = pose('pick_blue')
    out = run_pick()
    after = pose('pick_blue')
    moved = bool(before and after and
                 math.hypot(after[0] - before[0], after[1] - before[1]) > 0.05)
    check('자율 사이클', 'PICK_SUCCESS' in out and moved,
          f'이동 {math.hypot(after[0] - before[0], after[1] - before[1]) * 1000:.0f}mm'
          if moved else '물체가 제자리')

    # --- 3) 쓰레기통 투입 ------------------------------------------------
    print('\n[3] 쓰레기통 투입 (통 개구부 안에 들어갔는지)')
    stage('trash_demo_stage.py')
    out = run_pick('-p place_target:=trash')
    cube = pose('pick_blue')
    if cube:
        dx, dy = cube[0] - TRASH_CENTER[0], cube[1] - TRASH_CENTER[1]
        inside = abs(dx) < TRASH_HALF_OPEN and abs(dy) < TRASH_HALF_OPEN and cube[2] > 0.02
        check('통 안 착지', inside,
              f'({cube[0]:.3f}, {cube[1]:.3f}, {cube[2]:.3f}) '
              f'통 중심에서 {math.hypot(dx, dy) * 1000:.0f}mm')
    else:
        check('통 안 착지', False, '물체 조회 실패')

print('\n' + '=' * 64)
npass = sum(1 for _, ok, _ in results if ok)
print(f'결과: {npass}/{len(results)} 통과')
for name, ok, detail in results:
    if not ok:
        print(f'  실패 → {name}: {detail}')
print('=' * 64)
sys.exit(0 if npass == len(results) else 1)
