"""손목캠 하강 기준점(GRASP_REF)을 **파이프라인과 같은 경로**로 다시 잰다.

종전 `pocket_scan.py`는 `POSE_GRASP`로 한 번에 내리는 관절 보간 경로에서 쟀다.
그런데 파이프라인은 `_descend_vertical`(직교 직선 하강 + 단계별 리치 보정)로 내린다.
두 경로의 최종 자세가 다르다 — 실측:

    관절 보간(명목) : 아랫턱 x = 0.344
    직교 하강(실제) : 아랫턱 x = 0.368      24mm 차이

그래서 손목캠이 "전후 −2mm, 정확함"이라 판정해도 실제로는 얕게 걸린다(물림각 0.587).
기준점을 쓰는 자세와 재는 자세가 같아야 한다. 여기서 그걸 맞춘다.

같은 스윕에서 세 가지가 한꺼번에 나온다.
  · 파지가 성립하는 큐브 x 구간 (들림을 실좌표로 확인)
  · 그 구간 한가운데에서 보이는 blob 좌표 = 새 GRASP_REF
  · x 대 blob x 회귀 기울기 = 새 GRASP_PX_PER_M

사용 (ROS 환경 source 후): python3 grasp_ref_scan.py
"""
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import (GRIPPER_CLOSED, PickNode,  # noqa: E402
                                     POSE_FOLDED, POSE_GRASP_FLOOR, POSE_PRE_FLOOR)

XS = [0.345, 0.365, 0.385, 0.405, 0.425]   # 큐브 x 스윕 [m]


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    if r.returncode != 0 or 'true' not in r.stdout.lower():
        print(f'  [경고] set_pose 실패: {req[:44]}...')
        return False
    return True


def cube_pose():
    for _ in range(3):
        try:
            out = subprocess.run(['gz', 'model', '-m', 'pick_object_green', '-p'],
                                 capture_output=True, text=True, timeout=25).stdout
        except subprocess.TimeoutExpired:
            continue
        lines = out.splitlines()
        for i, ln in enumerate(lines):
            if '- Pose' in ln and i + 1 < len(lines):
                try:
                    v = [float(t) for t in lines[i + 1].strip().strip('[]').split()]
                    if len(v) >= 3:
                        return tuple(v[:3])
                except ValueError:
                    continue
    return None


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def trial(n, x):
    n._reach_applied = 0.0
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    svc(f'name: "pick_object_green", position: {{x: {x}, y: 0.0, z: 0.02}}, '
        f'orientation: {{w: 1}}')
    spin(n, 1.5)
    n.move_gripper(1.2)
    n.move_arm(dict(POSE_PRE_FLOOR), 3.0)
    spin(n, 0.8)
    hover = n._wrist_blob(frames=5)
    before = cube_pose()

    # 파이프라인과 같은 경로로 내린다
    n._descend_vertical(dict(POSE_PRE_FLOOR), dict(POSE_GRASP_FLOOR))
    spin(n, 0.8)
    grasp = n._wrist_blob(frames=5)
    after = cube_pose()

    n.move_gripper(GRIPPER_CLOSED)
    spin(n, 2.0)
    ang = getattr(n, 'gripper_angle', None)
    n.move_arm({**POSE_GRASP_FLOOR, 'arm_shoulder_lift': 0.5,
                'arm_elbow_flex': 0.4, 'arm_wrist_flex': 0.6}, 2.5)
    spin(n, 1.2)
    lifted = cube_pose()

    ok_meas = None not in (before, after, lifted)
    push = (None if not ok_meas else math.dist(before[:2], after[:2]))
    return dict(x=x, hover=hover, grasp=grasp, ang=ang, measured=ok_meas, push=push,
                held=(ok_meas and lifted[2] > 0.05))


def fb(b):
    return '   미검출   ' if b is None else f'({b[0]:5.0f},{b[1]:4.0f})'


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True
    print('파이프라인과 같은 직교 하강 경로에서 잰다 (pocket_scan은 관절 보간이었다).\n')
    print(f'{"큐브 x":>7} | {"상공 blob":^13} | {"하강 blob":^13} | {"물림각":>7} '
          f'{"밀림mm":>7} {"들림":>4}  판정')
    print('-' * 82)
    rows = []
    for x in XS:
        r = trial(n, x)
        rows.append(r)
        if not r['measured']:
            v = '측정불가'
        elif r['held']:
            v = '물림'
        elif r['ang'] is not None and r['ang'] < 0.0:
            v = '허공'
        else:
            v = '얕은 걸침'
        s_ang = '      -' if r['ang'] is None else format(r['ang'], '7.3f')
        s_push = '      -' if r['push'] is None else format(r['push'] * 1000, '7.1f')
        s_held = 'O' if r['held'] else 'X'
        print(f'{x:7.3f} | {fb(r["hover"])} | {fb(r["grasp"])} | '
              f'{s_ang} {s_push} {s_held:>4}  {v}')

    print('\n--- 결과 ---')
    valid = [r for r in rows if r['measured']]
    if not valid:
        print('측정 실패 — gz 무응답. 결론을 내지 않는다.')
        n.destroy_node(); rclpy.shutdown(); return
    held = [r for r in valid if r['held'] and r['grasp'] is not None]
    if not held:
        print('어느 x에서도 안 물린다 — 스윕 범위나 하강 자세부터 재검토해야 한다.')
        n.destroy_node(); rclpy.shutdown(); return

    lo, hi = min(r['x'] for r in held), max(r['x'] for r in held)
    mid = (lo + hi) / 2.0
    print(f'파지 성립 구간: {lo:.3f} ~ {hi:.3f} m  (폭 {(hi - lo) * 1000:.0f}mm, 중앙 {mid:.3f})')

    # 중앙에 가장 가까운 시행의 blob = 새 기준점
    ref = min(held, key=lambda r: abs(r['x'] - mid))
    print(f'\nPOCKET_FLOOR = ({mid:.3f}, 0.000)')
    print(f'GRASP_REF    = ({ref["grasp"][0]:.1f}, {ref["grasp"][1]:.1f})    '
          f'← 큐브 x={ref["x"]:.3f}에서')
    hv = [r for r in valid if r['hover'] is not None]
    if hv:
        rh = min(hv, key=lambda r: abs(r['x'] - mid))
        print(f'WRIST_REF    = ({rh["hover"][0]:.1f}, {rh["hover"][1]:.1f})')

    # px/m 회귀 — blob x가 큐브 x에 따라 얼마나 움직이나
    for name, key in (('GRASP_PX_PER_M', 'grasp'), ('WRIST_PX_PER_M', 'hover')):
        pts = [(r['x'], r[key][0]) for r in valid if r[key] is not None]
        if len(pts) >= 3:
            xs = np.array([p[0] for p in pts]); ys = np.array([p[1] for p in pts])
            slope = float(np.polyfit(xs, ys, 1)[0])
            print(f'{name} = {abs(slope):.0f}    ({len(pts)}점 회귀, 기울기 {slope:+.0f} px/m)')
    print('\n※ 이 값들을 pick_node.py 상수에 반영하고 docs/18 §3 표도 함께 고칠 것.')
    svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
