"""ArUco 마커가 **몇 미터까지** 잡히는가 — 로봇을 옮겨 가며 사거리를 잰다.

통 인식이 파이프라인의 마지막 병목이다. 실측(2026-08-11) 네 번 모두 `marker: False`,
즉 마커로 확인한 적이 한 번도 없이 HSV 회색 덩어리만 믿고 투입했다. 그래서 착지가
통에서 60mm(우연히 성공)와 397mm(실패) 사이를 오간다.

순환이 있다.

    HSV가 엉뚱한 덩어리를 통으로 봄 → 그쪽으로 가서 멈춤(진짜 통 앞이 아님)
    → 마커가 시야에 없음 → HSV에 계속 의존 ┘

끊으려면 **접근 초기부터** 마커를 봐야 한다. 그런데 기존 `aruco_probe.py`는 통까지
0.638m 이하만 쟀다 — 그보다 먼 거리에서 잡히는지는 아무도 모른다. 여기서 그걸 잰다.

통은 건드리지 않고 **로봇을 옮긴다.** 처음엔 통을 옮겼는데, 그 판이 예외로 죽으면서
통이 엉뚱한 자리(0.658,0.239)에 남았고 이후 측정이 통째로 무의미해졌다 — 무대를
바꾸는 도구는 반드시 원복까지 보장해야 한다. 로봇만 옮기면 그 위험이 없다.
자세는 운반 중과 같은 `POSE_CARRY_SCAN` — 실제로 통을 찾는 그 자세여야 의미가 있다.

사용 (ROS 환경 source 후): python3 marker_range_probe.py
"""
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import (PickNode, POSE_CARRY_SCAN,  # noqa: E402
                                     POSE_FOLDED, TRASH_POCKET_X)

# 통까지 이 거리들에서 본다 [m]
RANGES = [0.50, 0.70, 0.90, 1.10, 1.30, 1.60, 2.00]
# 방위도 함께 본다 — 정면에서 벗어나면 언제 놓치는지
BEARINGS = [0.0, 10.0, 20.0]
# **통은 원래 자리에 두고 로봇을 옮긴다.** 통을 옮기면 마커가 함께 돌아야 하는데
# (마커는 통 뒤에서 로봇을 향해 서 있다) 그 방향을 맞추다 틀리면 "안 보인다"는
# 결과가 마커 자세 탓인지 사거리 탓인지 갈리지 않는다. 로봇을 옮기면 그 위험이 없다.
TRASH_HOME = (0.15, 0.62)


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def model_xyz(name):
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
                if len(v) >= 3:
                    return v[0], v[1], v[2]
            except ValueError:
                continue
    return None


def place_robot(r, bearing_deg):
    """통에서 r만큼 떨어진 곳에 로봇을 놓고 통 쪽을 보게 한다(방위만큼 틀어서).

    **놓은 뒤 실제로 그 자리에 있는지 확인한다.** set_pose는 성공을 반환해도
    물리가 튕겨 내거나 미끄러뜨릴 수 있는데, 그러면 "마커가 안 보인다"가 사거리
    탓으로 오독된다. 배치를 확인하지 않아 측정을 통째로 버린 일이 오늘만 세 번이다.
    실패하면 (False, 어긋난 거리)를 돌려준다.
    """
    tx, ty = TRASH_HOME
    base = math.atan2(ty, tx)                 # 원점에서 통을 보는 방향
    rx, ry = tx - r * math.cos(base), ty - r * math.sin(base)
    yaw = base + math.radians(bearing_deg)    # 통을 이만큼 빗겨 본다
    if not svc(f'name: "jdamr_cube", position: {{x: {rx:.4f}, y: {ry:.4f}, z: 0.05}}, '
               f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}'):
        return False, -1.0
    time.sleep(1.0)                            # 순간이동 뒤 물리가 가라앉기를 기다린다
    got = model_xyz('jdamr_cube')
    if got is None:
        return False, -1.0
    err = math.dist((rx, ry), got[:2])
    return err < 0.05, err


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=blue',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)

    n.move_gripper(0.5)
    n.move_arm(POSE_FOLDED, 2.5)
    n.move_arm(POSE_CARRY_SCAN, 3.0)
    spin(n, 1.0)

    print('ArUco 마커 사거리 — 통은 그대로 두고 로봇을 옮겨 가며 검출 여부를 본다')
    print(f'자세: POSE_CARRY_SCAN (운반 중 통을 찾는 그 자세) · 정지 거리 {TRASH_POCKET_X}m\n')
    print(f'{"통까지":>8} | ' + ' '.join(f'{f"{b:.0f}도":>10}' for b in BEARINGS))
    print('-' * 46)
    reach = {b: 0.0 for b in BEARINGS}
    for r in RANGES:
        cells = []
        for bd in BEARINGS:
            ok, err = place_robot(r, bd)
            if not ok:
                cells.append(f'배치{err * 1000:.0f}mm')
                continue
            # 순간이동으로 팔 자세가 흐트러질 수 있으니 다시 명령한다
            n.move_arm(POSE_CARRY_SCAN, 1.5)
            spin(n, 1.2)
            m = n._locate_trash_aruco()
            if m is None:
                cells.append('✗')
                continue
            mx, my = (m[0], m[1]) if isinstance(m, (tuple, list)) else (m.x, m.y)
            got = math.hypot(mx, my)
            cells.append(f'○ {(got - r) * 1000:+.0f}mm')
            reach[bd] = max(reach[bd], r)
        print(f'{r:7.2f}m | ' + ' '.join(f'{c:>10}' for c in cells))

    print('\n--- 판정 ---')
    for bd in BEARINGS:
        v = reach[bd]
        print(f'  방위 {bd:4.0f}도 → 최대 {v:.2f}m' if v else f'  방위 {bd:4.0f}도 → 어느 거리도 미검출')
    best = reach[0.0]
    if best >= 1.0:
        print(f'\n  정면 {best:.2f}m까지 잡힌다 — 접근 **초기부터** 마커를 쓸 수 있다.')
        print('  HSV 방위에 의존하는 구간을 그만큼 줄이면 순환이 끊긴다.')
    elif best:
        print(f'\n  정면 {best:.2f}m까지만 잡힌다 — 그 밖에서는 HSV로 갈 수밖에 없다.')
        print('  그렇다면 HSV 정확도를 올리거나, 통을 찾는 전용 탐색이 필요하다.')
    else:
        print('\n  어느 거리에서도 미검출 — 자세·가림·마커 크기를 먼저 봐야 한다.')

    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
