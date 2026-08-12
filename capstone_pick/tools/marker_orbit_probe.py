"""통 **주위 어느 방향**에서 마커가 보이는가 — 한 바퀴 돌며 잰다.

`marker_range_probe.py`는 거리만 쟀고, 그 결과 "0.5~2.0m 전 구간에서 잡힌다"였다.
그런데 파이프라인에서 로봇이 제자리 360도를 돌아도(`_seek_marker`) 마커를 못 찾는다.
빠진 축이 하나 있다 — **로봇이 통의 어느 쪽에 서 있는가.**

월드의 마커는 단면이다. 판(0.004×0.133×0.133)의 +x 면에만 무늬 셀이 붙어 있고
(`pose x=0.00275`), 모델 yaw가 -90도라 무늬가 -y 방향을 향한다. 통은 (0.15,0.62),
마커는 (0.15,0.71)이므로 **마커는 통 뒤에서 원점 쪽을 보고 서 있다.**

그러면 로봇이 통 반대편이나 옆으로 접근하면 무늬를 못 본다. 제자리에서 아무리
돌아도 소용이 없다 — 돌아야 할 것은 로봇의 방향이 아니라 **위치**다.

여기서 그 가시 범위를 잰다. 통을 중심으로 각도 θ만큼 떨어진 자리에 로봇을 놓고
통을 향하게 한 뒤 마커를 찾는다. θ=0은 원점 쪽(마커 정면)이다.

사용 (ROS 환경 source 후): python3 marker_orbit_probe.py [거리]
"""
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import (PickNode, POSE_CARRY_SCAN,  # noqa: E402
                                     POSE_FOLDED)

TRASH = (0.15, 0.62)
ORBIT = list(range(0, 360, 30))       # 통 주위 방향 [도] — 0은 원점 쪽(마커 정면)


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def model_xy(name):
    try:
        out = subprocess.run(['gz', 'model', '-m', name, '-p'],
                             capture_output=True, text=True, timeout=15).stdout
    except subprocess.TimeoutExpired:
        return None
    L = out.splitlines()
    for i, ln in enumerate(L):
        if '- Pose' in ln and i + 1 < len(L):
            try:
                v = [float(t) for t in L[i + 1].strip().strip('[]').split()]
                if len(v) >= 2:
                    return v[0], v[1]
            except ValueError:
                continue
    return None


def place(r, theta_deg):
    """통 주위 theta 방향, 거리 r에 로봇을 놓고 통을 향하게 한다. 배치 확인까지."""
    base = math.atan2(TRASH[1], TRASH[0])          # 원점 → 통 방향
    a = base + math.radians(theta_deg)             # 통에서 로봇을 보는 방향
    rx, ry = TRASH[0] - r * math.cos(a), TRASH[1] - r * math.sin(a)
    if not svc(f'name: "jdamr_cube", position: {{x: {rx:.4f}, y: {ry:.4f}, z: 0.05}}, '
               f'orientation: {{z: {math.sin(a / 2):.6f}, w: {math.cos(a / 2):.6f}}}'):
        return False
    time.sleep(1.0)
    got = model_xy('jdamr_cube')
    return got is not None and math.dist((rx, ry), got) < 0.05


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def main():
    r = float(sys.argv[1]) if len(sys.argv) > 1 else 0.80
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=blue',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.move_gripper(0.5)
    n.move_arm(POSE_FOLDED, 2.5)
    n.move_arm(POSE_CARRY_SCAN, 3.0)
    spin(n, 1.0)

    print(f'통 주위 방향별 마커 가시성 (거리 {r:.2f}m, 통을 향한 자세)')
    print('θ=0은 원점 쪽 — 마커 무늬가 향하는 방향이다\n')
    print(f'{"θ[도]":>6} {"검출":>6} {"거리오차":>10}')
    print('-' * 26)
    seen = []
    for th in ORBIT:
        if not place(r, th):
            print(f'{th:6d} {"배치실패":>6}')
            continue
        n.move_arm(POSE_CARRY_SCAN, 1.5)
        spin(n, 1.2)
        m = n._locate_trash_aruco()
        if m is None:
            print(f'{th:6d} {"✗":>6}')
            continue
        mx, my = (m[0], m[1]) if isinstance(m, (tuple, list)) else (m.x, m.y)
        print(f'{th:6d} {"○":>6} {(math.hypot(mx, my) - r) * 1000:9.0f}mm')
        seen.append(th)

    print('\n--- 판정 ---')
    if not seen:
        print('  어느 방향에서도 미검출 — 거리·자세를 먼저 의심하라')
    else:
        print(f'  보이는 방향: {seen}')
        print(f'  가시 범위 {len(seen)}/{len(ORBIT)} = 통 주위의 {len(seen) / len(ORBIT) * 100:.0f}%')
        if len(seen) < len(ORBIT) * 0.6:
            print('\n  마커가 단면이라 반쪽 이하에서만 보인다. 제자리 회전으로는 못 찾는다 —')
            print('  로봇이 마커 앞쪽으로 **이동**해야 한다. 통 위치를 모르는 상태에서는')
            print('  그 이동을 계획할 수 없으므로, 마커를 여러 면에 두거나(실물에서도')
            print('  흔한 조치) 통 검출 자체를 개선하는 편이 맞다.')
    place(0.638, 0.0)
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
