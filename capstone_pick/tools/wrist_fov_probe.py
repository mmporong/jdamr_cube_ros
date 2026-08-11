"""손목캠이 높이별로 큐브를 얼마나 넓게 보는가 — 탐색 자세를 정한다.

손목캠 서보(`_servo_correct`)는 프리그래스프(큐브 위 8cm)에서 본다. 그 높이에서
큐브가 이미 시야 밖이면 보정이 시작조차 못 한다 — 실측에서 3사이클 중 2번이
`손목캠 미검출`로 중단됐다.

팔을 더 들면 시야가 넓어진다. 손목캠은 팔에 달려 있으므로 **팔 관절이 곧 팬틸트**다.
하드웨어를 더하지 않고 같은 효과를 낸다.

여기서는 높이별로 "큐브가 얼마나 벗어나도 보이는가"를 잰다. 가장 넓은 높이를
탐색 자세로 삼아 먼저 찾고, 그다음 프리그래스프로 내려와 정밀 보정한다.

사용 (ROS 환경 source 후): python3 wrist_fov_probe.py
"""
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick import kinematics as K  # noqa: E402
from capstone_pick.pick_node import ARM_JOINTS, PickNode, POSE_FOLDED  # noqa: E402

BASE_X, BASE_Y, CZ = 0.36, 0.0, 0.0125
UPS = [0.08, 0.14, 0.20, 0.26]                 # 팔을 큐브 위 이만큼
OFFSETS = [0.0, 0.02, 0.04, 0.06, 0.08]        # 큐브를 이만큼 벗어나게 놓는다


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def seen(n, cube_x, cube_y):
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    svc(f'name: "pick_object_green", position: {{x: {cube_x:.4f}, y: {cube_y:.4f}, '
        f'z: {CZ}}}, orientation: {{w: 1}}')
    spin(n, 0.9)
    return n._wrist_blob(frames=3) is not None


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    print('팔을 큐브 위 여러 높이에 두고, 큐브를 벗어나게 놓아 어디까지 보이는지 잰다.')
    print('손목캠은 팔에 달려 있으므로 팔 관절이 곧 팬틸트 역할을 한다.\n')
    print(f'{"팔 높이":>8} | ' + ' '.join(f'{f"전{o*1000:.0f}":>6}' for o in OFFSETS[1:])
          + ' | ' + ' '.join(f'{f"좌{o*1000:.0f}":>6}' for o in OFFSETS[1:]) + ' | 중앙')
    print('-' * 78)
    best = None
    for up in UPS:
        q0 = K.grasp_q(BASE_X, BASE_Y, CZ, up=up)
        if q0 is None:
            print(f'{up * 1000:7.0f}mm | IK 해 없음')
            continue
        n.move_arm(dict(zip(ARM_JOINTS, q0)), 2.5)
        spin(n, 0.6)
        row_f, row_l = [], []
        center = seen(n, BASE_X, BASE_Y)
        for o in OFFSETS[1:]:
            row_f.append('○' if seen(n, BASE_X + o, BASE_Y) else '✗')
        for o in OFFSETS[1:]:
            row_l.append('○' if seen(n, BASE_X, BASE_Y + o) else '✗')
        rng_f = max([o for o, v in zip(OFFSETS[1:], row_f) if v == '○'] or [0.0])
        rng_l = max([o for o, v in zip(OFFSETS[1:], row_l) if v == '○'] or [0.0])
        score = min(rng_f, rng_l) if center else -1
        if best is None or score > best[1]:
            best = (up, score, rng_f, rng_l)
        print(f'{up * 1000:7.0f}mm | ' + ' '.join(f'{v:>6}' for v in row_f)
              + ' | ' + ' '.join(f'{v:>6}' for v in row_l)
              + f' | {"○" if center else "✗"}')

    print('\n--- 판정 ---')
    if best is None or best[1] < 0:
        print('중앙에서도 안 보이는 높이뿐 — 자세나 큐브 크기를 재검토해야 한다')
    else:
        up, score, rf, rl = best
        print(f'가장 넓은 탐색 높이: 큐브 위 {up * 1000:.0f}mm')
        print(f'  전후 {rf * 1000:.0f}mm / 좌우 {rl * 1000:.0f}mm 까지 보인다')
        print(f'  → 접근 잔차가 이 안에 들면 서보가 시작될 수 있다')
    svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.0125}, orientation: {w: 1}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
