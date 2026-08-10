"""죠의 실제 파지점이 어디인가 — TF로 재서 확정한다.

IK 파지가 큐브를 죠에 **끼웠다**(화면 확인). 물림각이 0.296~0.349로 나왔는데
4cm 큐브를 죠 중앙에 제대로 물면 0.408~0.445다. 값이 작다는 건 죠가 더 닫혔다는
뜻이고, 큐브가 죠 사이 깊숙이가 아니라 끝이나 모서리에 걸렸다는 신호다.

원인 가설: 파지 목표를 **TCP** 기준으로 잡았는데 TCP가 죠 중앙이 아니다.
URDF에서 움직이는 죠의 회전축은 아랫턱 기준 z=-0.0234, TCP는 z=-0.0981이다.
즉 TCP는 죠 **끝**이고, 손가락이 벌어지는 구간은 그 위쪽이다.

여기서는 추론하지 않고 잰다. 파지 자세에서 아랫턱·움직이는 죠·TCP의 TF를 읽고,
큐브 실좌표와 대조해 "큐브 중심이 어디 있을 때 제대로 물리는가"를 확정한다.

사용 (ROS 환경 source 후): python3 jaw_frame_probe.py
"""
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick import kinematics as K  # noqa: E402
from capstone_pick.pick_node import (ARM_JOINTS, GRIPPER_CLOSED,  # noqa: E402
                                     PickNode, POSE_FOLDED, POSE_GRASP_FLOOR)

FRAMES = ['arm_gripper_link', 'arm_moving_jaw_link', 'arm_gripper_frame_link']
# GRASP_BACK 후보 — 현재 값 0.0698을 가운데 두고 앞뒤로
BACKS = [0.025, 0.032, 0.040, 0.045]   # 50~70mm는 전부 0.245~0.251(끼임) — 더 줄인다


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def cube_pose():
    for _ in range(3):
        try:
            out = subprocess.run(['gz', 'model', '-m', 'pick_object_green', '-p'],
                                 capture_output=True, text=True, timeout=25).stdout
        except subprocess.TimeoutExpired:
            continue
        L = out.splitlines()
        for i, ln in enumerate(L):
            if '- Pose' in ln and i + 1 < len(L):
                try:
                    v = [float(t) for t in L[i + 1].strip().strip('[]').split()]
                    if len(v) >= 3:
                        return tuple(v[:3])
                except ValueError:
                    continue
    return None


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def tf_xyz(n, frame):
    for _ in range(20):
        try:
            t = n.tf_buffer.lookup_transform('base_footprint', frame, rclpy.time.Time())
            v = t.transform.translation
            return np.array([v.x, v.y, v.z])
        except Exception:
            spin(n, 0.2)
    return np.array([float('nan')] * 3)


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True

    # 1) 죠가 열린 파지 자세에서 세 프레임의 실제 위치
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    svc('name: "pick_object_green", position: {x: 1.5, y: 1.5, z: 0.02}')   # 큐브는 치운다
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    n.move_arm(dict(zip(ARM_JOINTS, K.grasp_q(0.405, 0.0))), 3.5)
    spin(n, 1.0)
    print('파지 자세(큐브 x=0.405 목표, 죠 열림)에서 실제 프레임 위치 [base]\n')
    P = {}
    for f in FRAMES:
        P[f] = tf_xyz(n, f)
        print(f'  {f:26s} {np.round(P[f], 4)}')
    mid = (P['arm_gripper_link'] + P['arm_moving_jaw_link']) / 2.0
    print(f'  {"두 죠 원점의 중점":26s} {np.round(mid, 4)}')
    print(f'\n  FK가 계산한 TCP            {np.round(K.fk_pos(K.grasp_q(0.405, 0.0)), 4)}')
    print(f'  → TCP는 아랫턱에서 {np.linalg.norm(P["arm_gripper_frame_link"] - P["arm_gripper_link"]) * 1000:.0f}mm, '
          f'두 죠 중점에서 {np.linalg.norm(P["arm_gripper_frame_link"] - mid) * 1000:.0f}mm 떨어져 있다')

    # 2) GRASP_BACK을 쓸어 물림 품질을 본다. 4cm 큐브의 정상 물림각은 0.408~0.445.
    print(f'\n=== GRASP_BACK 스윕 (큐브 x=0.405 고정) ===')
    print(f'{"back[mm]":>9} {"물림각":>8} {"부하":>8} {"들림z":>8} {"밀림mm":>8}  판정')
    print('-' * 60)
    best = None
    for back in BACKS:
        K.GRASP_BACK = back                       # 모듈 상수를 갈아 끼우며 시험
        q_path = K.descend_path(0.405, 0.0)
        if q_path is None:
            print(f'{back * 1000:9.0f} {"IK 해 없음":>34}')
            continue
        n.move_gripper(1.2)
        n.move_arm(POSE_FOLDED, 2.5)
        svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
        svc('name: "pick_object_green", position: {x: 0.405, y: 0.0, z: 0.02}, '
            'orientation: {w: 1}')
        spin(n, 1.2)
        n.move_gripper(1.2)
        before = cube_pose()
        n.move_arm(dict(zip(ARM_JOINTS, q_path[0])), 3.0)
        for q in q_path[1:]:
            n.move_arm(dict(zip(ARM_JOINTS, q)), 1.2)
        spin(n, 0.5)
        after = cube_pose()
        n.move_gripper(GRIPPER_CLOSED)
        spin(n, 2.0)
        ang = getattr(n, 'gripper_angle', None)
        eff = getattr(n, 'gripper_effort', None)
        up = K.grasp_q(0.405, 0.0, up=0.10)
        if up is not None:
            n.move_arm(dict(zip(ARM_JOINTS, up)), 2.5)
        spin(n, 1.2)
        lifted = cube_pose()
        ok = None not in (before, after, lifted)
        push = None if not ok else math.dist(before[:2], after[:2])
        held = ok and lifted[2] > 0.05
        # 4cm 큐브를 죠 중앙에 물면 0.408~0.445 (종전 실측). 그보다 작으면 끼임/모서리.
        if not ok:
            v = '측정불가'
        elif not held:
            v = '실패'
        elif ang is not None and ang < 0.38:
            v = '물림(얕거나 끼임 — 각도 작음)'
        elif ang is not None and ang > 0.50:
            v = '물림(얕은 걸침 의심)'
        else:
            v = '물림(정상 폭)'
        f = lambda v_, w=8, p=3: (' ' * (w - 1) + '-') if v_ is None else format(v_, f'{w}.{p}f')
        print(f'{back * 1000:9.0f} {f(ang)} {f(eff)} '
              f'{f(None if not ok else lifted[2])} '
              f'{f(None if push is None else push * 1000, 8, 1)}  {v}')
        if held and ang is not None and 0.38 <= ang <= 0.50:
            best = back if best is None else best

    print('\n--- 판정 ---')
    if best is not None:
        print(f'정상 폭으로 물리는 GRASP_BACK = {best * 1000:.0f}mm')
        print('  → kinematics.GRASP_BACK 을 이 값으로 고칠 것')
    else:
        print('정상 폭(0.38~0.50)으로 물린 back이 없다.')
        print('  물림각이 전부 작으면 죠가 큐브를 지나쳐 안쪽에서 닫히는 것 —')
        print('  GRASP_DOWN(높이)도 함께 재야 한다.')
    svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}, orientation: {w: 1}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
