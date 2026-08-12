"""들어올리는 중 **어느 계단에서** 큐브가 빠지는가 — 팔 단독, 실좌표로 추적한다.

파이프라인 실측(2026-08-11 11:10)에서 이런 로그가 남았다.

    닫힘 신호: 각도=0.249 부하=-10.0 → HOLDING     제대로 물었다
    == 4. 들어올리기 ==
    파지 확인(각도): -0.170 → DROPPED               들고 나니 빈손

들기 전후만 찍혀서 "들다가 놓쳤다"까지만 알 뿐, 네 계단(up 0.02→0.14) 중
어디서 빠졌는지 알 수 없었다. 원인을 못 좁히면 처방이 정반대가 된다.

  · 계단 초반에 무너진다        → 바닥을 떠나는 순간의 하중. 더 잘게 올려야 한다
  · 계단은 멀쩡한데 유지력 직후 → effort=30의 순간 힘이 큐브를 튕긴 것
  · 운반 자세 전환에서 무너진다 → 자세 급변. 경유점을 넣어야 한다

그래서 계단마다 **큐브 실좌표**와 물림 신호를 함께 잰다. 물림각만 보면
큐브가 이미 빠졌는데 죠가 아직 안 닫힌 구간을 놓친다.

차체는 고정한다 — 주행 오차를 섞으면 들기 성능과 구분되지 않는다.

사용 (ROS 환경 source 후): python3 lift_probe.py [시행수]
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
from capstone_pick.pick_node import (ARM_JOINTS, CARRY_UP, CARRY_X,  # noqa: E402
                                     CUBE_CZ, GRIP_HOLD_MARGIN, GRIPPER_CLOSED,
                                     PickNode, POSE_FOLDED)

SPOTS = [(0.36, 0.0), (0.38, 0.0), (0.36, 0.04), (0.36, -0.04)]
UPS = (0.02, 0.05, 0.09, 0.14)
HELD_DZ = 0.010          # 큐브가 이만큼 올라왔으면 딸려 오는 중


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def cube_model():
    """월드에 실제로 있는 픽 대상 모델 이름.

    무대에 따라 다르다 — 월드 원본은 `pick_object_green`인데 `reset_and_stage.py`를
    돌리면 `pick_blue`만 남는다. 하드코딩하면 set_pose가 조용히 실패하고, 큐브가
    없는 상태를 측정 결과로 기록한다(실측: 시야 측정이 전 항목 ✗로 나와 자세 문제로
    오진할 뻔했다). 그래서 실제 목록에서 찾는다.
    """
    try:
        out = subprocess.run(['gz', 'model', '--list'], capture_output=True,
                             text=True, timeout=10).stdout
        for ln in out.splitlines():
            n = ln.strip().lstrip('-').strip()
            if n.startswith('pick'):
                return n
    except Exception:
        pass
    return 'pick_object_green'


CUBE = cube_model()
COLOR = CUBE.split('_')[-1]      # HSV 검출이 색에 걸려 있어 무대와 맞춰야 한다


def cube_xyz():
    """큐브 world (x, y, z). 실패하면 None."""
    for _ in range(3):
        try:
            out = subprocess.run(['gz', 'model', '-m', CUBE, '-p'],
                                 capture_output=True, text=True, timeout=20).stdout
        except subprocess.TimeoutExpired:
            continue
        L = out.splitlines()
        for i, ln in enumerate(L):
            if '- Pose' in ln and i + 1 < len(L):
                try:
                    p = [float(t) for t in L[i + 1].strip().strip('[]').split()]
                    if len(p) >= 3:
                        return p[0], p[1], p[2]
                except ValueError:
                    continue
    return None


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def grip(n, timeout=1.5):
    """(물림각, 부하). 못 얻으면 (None, None)."""
    n.gripper_angle = n.gripper_effort = None
    n.spin_until(lambda: n.gripper_angle is not None, timeout)
    return n.gripper_angle, n.gripper_effort


def jaw_local(n, cube):
    """큐브를 아랫턱 로컬 좌표로. 죠 사이에 있는지 보는 용도."""
    for _ in range(10):
        try:
            t = n.tf_buffer.lookup_transform('base_footprint', 'arm_gripper_link',
                                             rclpy.time.Time())
            q, v = t.transform.rotation, t.transform.translation
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ])
            M = np.eye(4)
            M[:3, :3] = R
            M[:3, 3] = [v.x, v.y, v.z]
            return (np.linalg.inv(M) @ np.append(np.array(cube), 1.0))[:3]
        except Exception:
            spin(n, 0.2)
    return None


def place(cx, cy):
    """큐브를 먼저 치운 뒤 배치한다 — 죠에 물린 채 set_pose하면 물리가 꼬인다."""
    svc(f'name: "{CUBE}", position: {{x: 1.5, y: 1.5, z: 0.0155}}, orientation: {{w: 1}}')
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    return svc(f'name: "{CUBE}", position: {{x: {cx:.4f}, y: {cy:.4f}, '
               f'z: {CUBE_CZ:.4f}}}, orientation: {{w: 1}}')


def one(n, cx, cy, legacy=False):
    """한 번의 파지 + 들기. 계단별 기록 목록을 돌려준다.

    legacy=True면 유지 목표를 완전 닫힘(-0.17)으로 둔다 — 옛 동작 재현용 대조군.
    """
    rows = []
    path = K.descend_path(cx, cy, CUBE_CZ, 0.0)
    if path is None:
        return None, 'IK 해 없음'

    svc(f'name: "{CUBE}", position: {{x: 1.5, y: 1.5, z: 0.0155}}, orientation: {{w: 1}}')
    spin(n, 0.4)
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    if not place(cx, cy):
        return None, '배치 실패'
    spin(n, 1.0)

    # --- 파지: 파이프라인(_grasp_ik)과 같은 절차 ---
    n.move_gripper(1.2)
    n.move_arm(dict(zip(ARM_JOINTS, path[0])), 3.0)
    for q in path[1:]:
        n.move_arm(dict(zip(ARM_JOINTS, q)), 1.0)
    n.move_gripper(GRIPPER_CLOSED, effort=4.0)          # 살살 닫기
    n.move_gripper(GRIPPER_CLOSED, wait=False, effort=30.0)
    spin(n, 0.5)
    a, e = grip(n)
    # 유지 목표를 접촉각 안쪽으로 묶는다(_grasp_ik와 같은 처방). legacy면 옛 동작.
    hold = GRIPPER_CLOSED if (legacy or a is None) else max(GRIPPER_CLOSED, a - GRIP_HOLD_MARGIN)
    n.move_gripper(hold, wait=False, effort=30.0)
    spin(n, 0.4)
    c0 = cube_xyz()
    if c0 is None:
        return None, '큐브 좌표 실패'
    rows.append(('파지 직후', a, e, c0[2], 0.0, jaw_local(n, c0)))

    # --- 들기: _lift_ik과 같은 계단 ---
    for up in UPS:
        q = K.grasp_q(cx, cy, CUBE_CZ, up=up)
        if q is None:
            break
        n.move_arm(dict(zip(ARM_JOINTS, q)), 1.5)
        a, e = grip(n)
        c = cube_xyz()
        if c is None:
            continue
        rows.append((f'up={up:.2f}', a, e, c[2], c[2] - c0[2], jaw_local(n, c)))

    # --- 유지력 → 운반 자세 (여기가 종전 의심 구간) ---
    n.move_gripper(hold, wait=False, effort=30.0)
    spin(n, 0.6)
    a, e = grip(n)
    c = cube_xyz()
    if c:
        rows.append(('유지력 직후', a, e, c[2], c[2] - c0[2], jaw_local(n, c)))
    carry = K.grasp_q(CARRY_X, 0.0, CUBE_CZ, up=CARRY_UP)
    if carry is not None:
        n.move_arm(dict(zip(ARM_JOINTS, carry)), 2.5)
    spin(n, 0.8)
    a, e = grip(n)
    c = cube_xyz()
    if c:
        rows.append(('운반 자세', a, e, c[2], c[2] - c0[2], jaw_local(n, c)))
    return rows, None


def main():
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else len(SPOTS)
    legacy = 'old' in sys.argv[1:]
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', f'target_color:={COLOR}',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True
    n.target_color = COLOR

    print('들기 계단별 추적 — 큐브가 어디서 빠지는지 실좌표로 본다')
    print(f'큐브 크기 {CUBE_CZ * 2 * 1000:.0f}mm · 빈손 물림각 기준 -0.10 이하')
    print(f'유지 목표: {"완전 닫힘 -0.17 (옛 동작, 대조군)" if legacy else f"접촉각 안쪽 {GRIP_HOLD_MARGIN}"}\n')
    lost_at = {}
    held_all = 0
    for t in range(trials):
        cx, cy = SPOTS[t % len(SPOTS)]
        print(f'--- 시행 {t + 1}/{trials}  큐브 ({cx:.3f}, {cy:+.3f}) ---')
        rows, err = one(n, cx, cy, legacy=legacy)
        if rows is None:
            print(f'    건너뜀: {err}\n')
            continue
        print(f'    {"단계":<12} {"물림각":>8} {"부하":>8} {"큐브z":>8} {"Δz":>8}  {"죠 로컬 x/z":>14}')
        lost = None
        for name, a, e, z, dz, jl in rows:
            jls = '—' if jl is None else f'{jl[0]:+.3f}/{jl[2]:+.3f}'
            flag = ''
            if name != '파지 직후' and dz < HELD_DZ:
                flag = '  ← 안 딸려옴'
                lost = lost or name
            print(f'    {name:<12} {"?" if a is None else f"{a:8.3f}"} '
                  f'{"?" if e is None else f"{e:8.2f}"} {z:8.3f} {dz:+8.3f}  {jls:>14}{flag}')
        if lost:
            lost_at[lost] = lost_at.get(lost, 0) + 1
            print(f'    → 놓친 지점: {lost}')
        else:
            held_all += 1
            print('    → 끝까지 유지')
        print()

    print('=== 요약 ===')
    print(f'  유지 목표 {"완전 닫힘(옛 동작)" if legacy else "접촉각 안쪽"} '
          f'→ 끝까지 유지 {held_all}/{trials}')
    if not lost_at:
        print('  전 시행 유지 — 들기 자체는 문제가 아니다. 다음 의심은 운반 주행이다.')
    else:
        for k, v in sorted(lost_at.items(), key=lambda kv: -kv[1]):
            print(f'  {k}에서 {v}회 놓침')
        print('\n  계단 초반이면 하중, 유지력 직후면 순간 힘, 운반 자세면 자세 급변이다.')
    svc(f'name: "{CUBE}", position: {{x: 0.45, y: 0.26, z: 0.0155}}, orientation: {{w: 1}}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
