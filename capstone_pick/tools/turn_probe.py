"""차체가 실제로 도는가 — 자세·적재·플래그별로 잰다.

운반 중 통 접근에서 방위가 24.2도에 굳었다. 관측값(`앞면 base=(0.366,0.164,0.081)
면적=472`)이 71회 내내 **소수점까지 동일**했는데, 실측이면 노이즈가 있어야 한다.
즉 차체가 돌지 않았다는 뜻이고 코드의 스톨 감지도 그걸 잡았다.

여기서는 명령 대비 **실제 회전량**을 odom으로 잰다. 무엇이 회전을 막는지
한 번에 하나씩만 바꿔 가며 본다.

  · 팔 자세      접힘 대 운반 자세(무게가 앞으로 쏠린다)
  · `_carrying`  운반 플래그가 켜지면 drive가 사다리꼴 램프를 쓴다
  · 명령 크기    0.25 / 0.35 / 0.5 rad/s

사용 (ROS 환경 source 후): python3 turn_probe.py
"""
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import (PickNode, POSE_CARRY,  # noqa: E402
                                     POSE_FOLDED)

WZ_LIST = [0.25, 0.35, 0.50]


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def turn_test(n, wz, sec=2.0):
    """명령 대비 실제 회전량. (명령 rad, 실제 rad, 달성률)"""
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    spin(n, 1.2)
    n.spin_until(lambda: n.odom is not None, 3.0)
    a0 = n.odom[2]
    n.drive(0.0, wz, sec)
    spin(n, 0.8)
    a1 = n.odom[2]
    want = wz * sec
    got = wrap(a1 - a0)
    return want, got, (got / want if want else 0.0)


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: n.odom is not None, 20.0)
    print('명령한 회전량 대비 실제 회전량(odom). 달성률이 낮으면 그 조건이 회전을 막는다.\n')
    print(f'{"자세":>10} {"운반플래그":>10} {"wz":>6} | {"명령[도]":>9} {"실제[도]":>9} {"달성률":>8}')
    print('-' * 62)
    for pose_name, pose in (('접힘', POSE_FOLDED), ('운반', POSE_CARRY)):
        n.move_arm(pose, 3.0)
        spin(n, 0.8)
        for carrying in (False, True):
            n._carrying = carrying
            for wz in WZ_LIST:
                want, got, ratio = turn_test(n, wz)
                print(f'{pose_name:>10} {str(carrying):>10} {wz:6.2f} | '
                      f'{math.degrees(want):9.1f} {math.degrees(got):9.1f} {ratio * 100:7.0f}%')
    n._carrying = False
    print('\n※ 달성률이 자세·플래그와 무관하게 낮으면 drive의 속도 하한·램프 문제,')
    print('  운반 자세에서만 낮으면 무게 쏠림(정지 마찰) 문제다.')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
