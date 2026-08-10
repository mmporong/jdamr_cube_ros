"""IK 파지를 팔 단독으로 검증한다 — 차체는 고정, 큐브만 옮긴다.

종전 구조에서는 파지 성립 구간이 전후 40mm뿐이라 접근 잔차(±45mm)가 그 안에
안 들어갔다. IK로는 작업공간이 전후 225mm·좌우 340mm로 계산된다(kinematics 분석).
그 계산이 실제로 성립하는지, 그리고 죠 오프셋 상수 세 개가 맞는지 여기서 잰다.

차체를 고정하는 이유: 주행 오차를 섞으면 IK 성능과 주행 성능이 구분되지 않는다.
전체 파이프라인 연결은 이 단계가 통과한 뒤에 한다.

판정은 Gazebo 실좌표로 한다. 로그가 아니다.

사용 (ROS 환경 source 후):
    python3 ik_grasp_probe.py            위치 격자 (yaw 0)
    python3 ik_grasp_probe.py yaw        큐브 기울기별 (roll 부호 확정)
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
                                     PickNode, POSE_FOLDED)

# 들었다고 파지가 아니다. 큐브가 **윗턱에 끼어** 딸려 올라가도 높이는 올라간다.
# 그래서 들린 뒤 큐브가 죠 사이에 있는지를 아랫턱 로컬 좌표로 확인한다.
#   정상: x가 파지 중앙(GRASP 상수에서 역산) 근처, z가 손가락 구간
# 이 검사를 안 넣어 "성공 33%"를 보고했는데 그 안에 끼임이 섞여 있었다.
SEAT_X_TOL = 0.010      # 파지 중앙에서 이만큼 벗어나면 죠 사이가 아니다
SEAT_Z_TOL = 0.012

# 위치 격자 — 종전 성립 구간(0.385~0.425)을 훨씬 넘겨 잡는다
GRID_X = [0.29, 0.32, 0.35, 0.38, 0.41]   # 20mm 큐브 기준 도달 범위
GRID_Y = [0.0, -0.05, 0.05]   # 정면 위주 — 큰 좌우는 차체 회전이 맡는다
YAWS = [0, 15, 30, 45, -30]


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def cube_pose():
    """(x, y, z, yaw). 실패하면 None."""
    for _ in range(3):
        try:
            out = subprocess.run(['gz', 'model', '-m', 'pick_object_green', '-p'],
                                 capture_output=True, text=True, timeout=25).stdout
        except subprocess.TimeoutExpired:
            continue
        lines = out.splitlines()
        for i, ln in enumerate(lines):
            if '- Pose' in ln and i + 2 < len(lines):
                try:
                    p = [float(t) for t in lines[i + 1].strip().strip('[]').split()]
                    r = [float(t) for t in lines[i + 2].strip().strip('[]').split()]
                    if len(p) >= 3 and len(r) >= 3:
                        return p[0], p[1], p[2], r[2]
                except ValueError:
                    continue
    return None


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def jaw_local(n, cube):
    """큐브를 아랫턱 로컬 좌표로. TF를 못 얻으면 None."""
    for _ in range(20):
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
            return (np.linalg.inv(M) @ np.append(np.array(cube[:3]), 1.0))[:3]
        except Exception:
            spin(n, 0.2)
    return None


def place(cx, cy, yaw=0.0):
    # **큐브를 먼저 치운다.** 앞 시행이 성공했으면 큐브가 죠에 물린 채 끝나는데,
    # 그 상태에서 set_pose로 옮기면 물리가 꼬여 다음 시행이 실패한다. 그래서
    # 성공·실패가 정확히 번갈아 나왔고 "성공률 33%"로 보였다(실제로는 6/6).
    svc('name: "pick_object_green", position: {x: 1.5, y: 1.5, z: 0.0125}, '
        'orientation: {w: 1}')
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    return svc(f'name: "pick_object_green", position: {{x: {cx:.4f}, y: {cy:.4f}, z: 0.0125}}, '
               f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')


def try_grasp(n, cx, cy, yaw=0.0, roll_sign=1.0):
    """IK만으로 파지한다. 실좌표로 판정한 결과 dict."""
    path = K.descend_path(cx, cy, 0.0125, yaw, roll_sign=roll_sign)
    if path is None:
        return dict(reason='IK 해 없음', held=False, ang=None, measured=True)

    # 치우기 → 팔 정리 → 배치 순서를 지킨다(위 place 주석 참조)
    svc('name: "pick_object_green", position: {x: 1.5, y: 1.5, z: 0.0125}, '
        'orientation: {w: 1}')
    spin(n, 0.5)
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    if not place(cx, cy, yaw):
        return dict(reason='배치 실패', held=False, ang=None, measured=False)
    spin(n, 1.2)
    n.move_gripper(1.2)

    before = cube_pose()
    # 프리그래스프로 한 번에 간 뒤, 나머지는 직교 직선으로 내린다
    n.move_arm(dict(zip(ARM_JOINTS, path[0])), 3.0)
    for q in path[1:]:
        n.move_arm(dict(zip(ARM_JOINTS, q)), 1.2)
    spin(n, 0.5)
    after = cube_pose()

    n.move_gripper(GRIPPER_CLOSED)
    spin(n, 2.0)
    ang = getattr(n, 'gripper_angle', None)
    eff = getattr(n, 'gripper_effort', None)
    # 들기 — 파지 자세에서 10cm 위로 (IK로 생성)
    up = K.grasp_q(cx, cy, 0.0125, yaw, up=0.10, roll_sign=roll_sign)
    if up is not None:
        n.move_arm(dict(zip(ARM_JOINTS, up)), 2.5)
    spin(n, 1.2)
    lifted = cube_pose()

    ok = None not in (before, after, lifted)
    push = None if not ok else math.dist(before[:2], after[:2])
    # **들었다고 파지가 아니다.** 윗턱에 끼어 딸려 올라가도 높이는 올라간다.
    # 들린 뒤 큐브가 죠 사이(파지 중앙 근처)에 있는지 로컬 좌표로 확인한다.
    loc = jaw_local(n, lifted) if ok else None
    seat_x = K.GRASP_BACK - 0.0081          # 파지 중앙의 아랫턱 로컬 x
    seat_z = 0.0125 - 0.1001                # 큐브 중심의 아랫턱 로컬 z (25mm 기준)
    seated = (loc is not None
              and abs(loc[0] - seat_x) < SEAT_X_TOL
              and abs(loc[2] - seat_z) < SEAT_Z_TOL)
    lifted_ok = ok and lifted[2] > 0.05
    reason = ''
    if lifted_ok and not seated:
        reason = '끼임(들렸으나 죠 사이가 아님)'
    return dict(reason=reason, held=(lifted_ok and seated), ang=ang, eff=eff,
                push=push, measured=ok, loc=loc,
                z=None if not ok else lifted[2])


def run_grid(n):
    print('차체 고정. 큐브만 옮기며 IK로만 파지한다. 판정은 실좌표.\n')
    print(f'{"큐브(x,y)":>16} {"하강밀림mm":>11} {"물림각":>8} {"부하":>7} {"들림z":>8}  판정')
    print('-' * 66)
    rows = []
    for cx in GRID_X:
        for cy in GRID_Y:
            r = try_grasp(n, cx, cy)
            r.update(cx=cx, cy=cy)
            rows.append(r)
            f = lambda v, w=8, p=3: (' ' * (w - 1) + '-') if v is None else format(v, f'{w}.{p}f')
            v = (r['reason'] or ('물림' if r['held'] else
                                 ('허공' if (r['ang'] or 0) < 0 else '얕은 걸침')))
            lc = r.get('loc')
            ls = '' if lc is None else f'  [로컬 x{lc[0]:+.3f} z{lc[2]:+.3f}]'
            print(f'{f"({cx:.2f},{cy:+.2f})":>16} '
                  f'{f(None if r.get("push") is None else r["push"] * 1000, 11, 1)} '
                  f'{f(r["ang"])} {f(r.get("eff"), 7, 2)} {f(r.get("z"))}  {v}{ls}')
    return rows


def run_yaw(n):
    print('큐브를 (0.40, 0)에 놓고 yaw만 바꾼다. roll 부호 규약을 확정한다.\n')
    for sign in (1.0, -1.0):
        print(f'--- roll_sign = {sign:+.0f} ---')
        print(f'{"yaw":>6} {"물림각":>8} {"들림z":>8}  판정')
        ok = 0
        for yd in YAWS:
            r = try_grasp(n, 0.40, 0.0, math.radians(yd), roll_sign=sign)
            f = lambda v, w=8, p=3: (' ' * (w - 1) + '-') if v is None else format(v, f'{w}.{p}f')
            v = (r['reason'] or ('물림' if r['held'] else
                                 ('허공' if (r['ang'] or 0) < 0 else '얕은 걸침')))
            ok += 1 if r['held'] else 0
            print(f'{yd:5d}도 {f(r["ang"])} {f(r.get("z"))}  {v}')
        print(f'  → {ok}/{len(YAWS)} 물림\n')


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True
    mode = sys.argv[1] if len(sys.argv) > 1 else 'grid'
    if mode == 'yaw':
        run_yaw(n)
    else:
        rows = run_grid(n)
        valid = [r for r in rows if r['measured']]
        held = [r for r in valid if r['held']]
        print(f'\n--- 결과 ---')
        if not valid:
            print('측정 실패 — gz 무응답. 결론을 내지 않는다.')
        else:
            print(f'파지 성공 {len(held)}/{len(valid)}  ({100 * len(held) / len(valid):.0f}%)')
            if held:
                xs = sorted({r['cx'] for r in held})
                ys = sorted({r['cy'] for r in held})
                print(f'  성공한 전후: {xs}')
                print(f'  성공한 좌우: {ys}')
            fail = [r for r in valid if not r['held']]
            if fail:
                print('  실패 지점:')
                for r in fail:
                    print(f'    ({r["cx"]:.2f},{r["cy"]:+.2f}) '
                          f'{r["reason"] or f"물림각 {r['ang']}"}')
    svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}, orientation: {w: 1}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
