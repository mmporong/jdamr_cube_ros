"""손목캠으로 IK 목표를 보정하기 위한 기준점과 감도를 잰다.

전방캠은 파지 거리에서 큐브를 볼 수 없다 — 팔 접힘 자세의 최고점이 z=0.381로
카메라(z=0.350)보다 높아, 카메라를 올려도 팔이 시선을 가린다(계산 확인).
손목캠은 팔에 달려 있어 팔이 가릴 수 없고 파지 지점을 항상 본다. Stretch 같은
상용 모바일 매니퓰레이터가 손목캠을 두는 이유가 이것이다.

종전 `wrist_align`도 손목캠을 썼지만 픽셀 기준점이 파지 자세에 얽혀 있었다
(WRIST_REF·GRASP_REF·px/m). 하나만 어긋나면 전부 무효가 됐다. IK에서는 오차를
**IK 목표에 더하고 다시 보는 반복 보정**이라 계수가 대략만 맞아도 수렴한다.

여기서 재는 것 둘:
  · 기준점  — 큐브가 IK 목표대로 죠 앞에 있을 때 손목캠에서의 blob 위치
  · 감도    — 큐브를 1m 옮기면 blob이 몇 px 움직이나 (전후·좌우 각각)

프리그래스프 자세(파지 지점 위 PREGRASP_UP)에서 잰다. 그 높이에서 보정한 뒤
내려가야 죠가 큐브를 건드리지 않는다.

사용 (ROS 환경 source 후): python3 wrist_servo_probe.py
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
from capstone_pick.pick_node import ARM_JOINTS, PickNode, POSE_FOLDED  # noqa: E402

BASE_X, BASE_Y, CZ = 0.36, 0.0, 0.0125
DX = [-0.03, -0.015, 0.0, 0.015, 0.03]     # 전후로 흔들어 감도를 잰다
DY = [-0.03, -0.015, 0.0, 0.015, 0.03]     # 좌우


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def look(n, cube_x, cube_y):
    """팔은 기준 자세(BASE)에 두고 큐브만 옮겨 blob을 본다. (x, y, area) 또는 None."""
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    svc(f'name: "pick_object_green", position: {{x: {cube_x:.4f}, y: {cube_y:.4f}, '
        f'z: {CZ}}}, orientation: {{w: 1}}')
    spin(n, 1.0)
    return n._wrist_blob(frames=5)


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True

    # 팔을 프리그래스프 자세로 고정한다 — 이 자세에서 보정하고 내려간다
    q = K.grasp_q(BASE_X, BASE_Y, CZ, up=K.PREGRASP_UP)
    if q is None:
        print('기준 자세 IK 해 없음'); return
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    n.move_arm(dict(zip(ARM_JOINTS, q)), 3.0)
    spin(n, 1.0)
    print(f'팔을 큐브({BASE_X},{BASE_Y}) 프리그래스프 자세로 고정하고 큐브만 옮긴다.')
    print(f'손목캠 blob이 큐브 위치에 따라 어떻게 움직이는지 잰다.\n')

    ref = look(n, BASE_X, BASE_Y)
    if ref is None:
        print('기준 위치에서 큐브가 손목캠에 안 보인다 — 자세·크기를 확인하라'); return
    print(f'기준점 (큐브가 목표대로 있을 때): blob = ({ref[0]:.1f}, {ref[1]:.1f})  면적 {ref[2]:.0f}\n')

    print(f'{"큐브 이동":>12} {"blob x":>9} {"blob y":>9} {"Δx":>8} {"Δy":>8}')
    print('-' * 50)
    rows_x, rows_y = [], []
    for d in DX:
        b = look(n, BASE_X + d, BASE_Y)
        if b is None:
            print(f'{f"전후 {d*1000:+.0f}mm":>12} {"미검출":>9}'); continue
        rows_x.append((d, b[0], b[1]))
        print(f'{f"전후 {d*1000:+.0f}mm":>12} {b[0]:9.1f} {b[1]:9.1f} '
              f'{b[0]-ref[0]:8.1f} {b[1]-ref[1]:8.1f}')
    for d in DY:
        b = look(n, BASE_X, BASE_Y + d)
        if b is None:
            print(f'{f"좌우 {d*1000:+.0f}mm":>12} {"미검출":>9}'); continue
        rows_y.append((d, b[0], b[1]))
        print(f'{f"좌우 {d*1000:+.0f}mm":>12} {b[0]:9.1f} {b[1]:9.1f} '
              f'{b[0]-ref[0]:8.1f} {b[1]-ref[1]:8.1f}')

    print('\n--- 감도 (회귀) ---')
    def slope(rows, idx):
        if len(rows) < 3:
            return None
        a = np.array([r[0] for r in rows]); b = np.array([r[idx] for r in rows])
        return float(np.polyfit(a, b, 1)[0])
    sxx, sxy = slope(rows_x, 1), slope(rows_x, 2)
    syx, syy = slope(rows_y, 1), slope(rows_y, 2)
    for nm, v in (('전후 1m → blob x', sxx), ('전후 1m → blob y', sxy),
                  ('좌우 1m → blob x', syx), ('좌우 1m → blob y', syy)):
        print(f'  {nm:20s} {"—" if v is None else f"{v:+9.0f} px"}')
    if sxx and syy:
        print(f'\n보정식 (blob 오차 → 큐브 위치 보정):')
        print(f'  전후 [m] = (blob_x - {ref[0]:.1f}) / {sxx:+.0f}')
        print(f'  좌우 [m] = (blob_y - {ref[1]:.1f}) / {syy:+.0f}')
        print('  ※ 반복 보정이므로 계수가 20% 어긋나도 2~3회면 수렴한다.')
    svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.0125}, orientation: {w: 1}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
