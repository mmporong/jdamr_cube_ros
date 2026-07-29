"""Nav2 자율주행 + 비전 픽앤플레이스 통합 데모.

역할 분담이 핵심이다.
  Nav2  : 맵 기반으로 작업 구역까지 자율 이동 (도착 오차 수십 cm 수준)
  비전  : 물체 검출 후 mm 단위 정밀 접근·파지 (기존 capstone_pick)
빈 사각형 방은 라이다 특징이 부족해 AMCL 추정이 드리프트하므로, Nav2에 정밀도를
요구하지 않고 "물체가 카메라에 들어오는 범위까지"만 맡긴다.

사용: python3 nav_pick_demo.py [작업구역x] [작업구역y] [yaw_deg]
"""
import math
import subprocess
import sys

WORK_X = float(sys.argv[1]) if len(sys.argv) > 1 else 0.33
WORK_Y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
WORK_YAW = float(sys.argv[3]) if len(sys.argv) > 3 else 90.0


def run(cmd, timeout=400):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                       timeout=timeout, executable='/bin/bash')
    return r.stdout + r.stderr


def pose_of(name):
    out = run(f'gz model -m {name} -p', timeout=30)
    for ln in out.splitlines():
        ln = ln.strip()
        if ln.startswith('[') and ln.count(' ') >= 2:
            return [float(v) for v in ln.strip('[]').split()]
    return None


print('=' * 60)
print('1단계: Nav2로 작업 구역까지 자율 주행')
print('=' * 60)
out = run(f'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && '
          f'python3 ~/capstone_tools/nav_goto.py {WORK_X} {WORK_Y} {WORK_YAW}')
for ln in out.splitlines():
    if any(k in ln for k in ('AMCL 추정', '목표 전송', '주행 결과', '최종 위치')):
        print('  ' + ln.split(']: ')[-1])

rp = pose_of('jdamr_cube')
if rp:
    print(f'  주행 후 실제 위치: ({rp[0]:.3f}, {rp[1]:.3f})')

print()
print('=' * 60)
print('2단계: 비전 검출 → 정밀 접근 → 파지 → 쓰레기통 투입')
print('=' * 60)
out = run('source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && '
          'ros2 run capstone_pick pick --ros-args -p speed_scale:=4.0 -p place_target:=trash',
          timeout=700)
for ln in out.splitlines():
    if any(k in ln for k in ('검출 근거', '파지 검증', '들기 후', '통 접근: r=0.3',
                             '팔 뻗기', '쓰레기통 위', 'PICK_SUCCESS', 'PICK_FAIL')):
        print('  ' + ln.split(']: ')[-1])

cube = pose_of('pick_blue')
if cube:
    dx, dy = cube[0] - 0.34, cube[1] - 1.0        # 통 중심
    inside = abs(dx) < 0.068 and abs(dy) < 0.068 and cube[2] > 0.02
    print()
    print(f'최종 큐브 위치: ({cube[0]:.3f}, {cube[1]:.3f}, {cube[2]:.3f})')
    print(f'판정: {"쓰레기통 안" if inside else "통 밖"} '
          f'(통 중심에서 {math.hypot(dx, dy) * 1000:.0f}mm)')
