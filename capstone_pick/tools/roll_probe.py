"""wrist_roll이 명령을 따라가는가 — 공중과 바닥 접촉 두 조건에서 잰다.

파지 실패의 로그가 한 관절을 반복해서 가리켰다:
`wrist_roll 목표-0.68 실제-0.08(+0.599)` — 재명령 네 번 모두 -0.08에 고착.
관절 한계는 -2.74~+2.84라 여유가 있으니 한계 문제가 아니다.

roll이 안 돌면 두 가지가 동시에 깨진다. 죠가 큐브 기울기에 못 맞춰지고,
손목캠이 함께 돌지 않아 GRASP_REF 기준이 통째로 어긋난다 — 실제로 손목캠은
"전후 +6mm 통과"라 했는데 닫으면 허공이었다.

가설은 두 개다.
  (A) 공중에서도 안 돈다      → 컨트롤러·관절 설정 문제
  (B) 공중에서는 도는데 하강 자세에서 안 돈다 → 죠가 바닥에 눌려 마찰로 막힘.
      그렇다면 순서가 문제다. roll은 하강 전에 끝나 있어야 한다.

사용 (ROS 환경 source 후): python3 roll_probe.py
"""
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import (PickNode, POSE_FOLDED,  # noqa: E402
                                     POSE_GRASP_FLOOR, POSE_PRE_FLOOR)

ROLLS = [-0.68, +0.41, -0.30, 0.0]


def svc(req):
    subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '4000', '--req', req], capture_output=True, text=True)


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def jaw_z(n):
    for _ in range(15):
        try:
            t = n.tf_buffer.lookup_transform('base_footprint', 'arm_gripper_link',
                                             rclpy.time.Time())
            return t.transform.translation.z
        except Exception:
            spin(n, 0.2)
    return float('nan')


def test(n, label, base_pose):
    print(f'\n=== {label} ===')
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    n.move_arm(POSE_PRE_FLOOR, 3.0)
    if base_pose is not POSE_PRE_FLOOR:
        n.move_arm(base_pose, 4.0)
    spin(n, 1.0)
    z = jaw_z(n)
    print(f'  아랫턱 높이 z={z:+.3f}m')
    print(f'  {"목표":>7} {"실제":>7} {"오차":>7}   판정')
    worst = 0.0
    for r in ROLLS:
        n.move_arm({**base_pose, 'arm_wrist_roll': r}, 2.5)
        spin(n, 1.5)
        act = (getattr(n, 'joint_pos', None) or {}).get('arm_wrist_roll', float('nan'))
        e = abs(act - r)
        # NaN(관절 미수신)은 '정상'으로 삼키지 않는다 — max(0.0, nan)이 0.0이라
        # 표에는 '막힘'을 찍고 결론은 '두 조건 모두 정상'을 내던 자리다.
        worst = float('nan') if (e != e or worst != worst) else max(worst, e)
        print(f'  {r:+7.2f} {act:+7.2f} {e:+7.3f}   {"OK" if e < 0.04 else "막힘"}')
    return worst


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 15.0)
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    svc('name: "pick_object_green", position: {x: 1.5, y: 1.5, z: 0.02}')   # 큐브는 치운다
    spin(n, 1.5)
    hi = test(n, '공중 (상공 자세)', POSE_PRE_FLOOR)
    lo = test(n, '바닥 접촉 (하강 자세)', POSE_GRASP_FLOOR)
    print('\n--- 판정 ---')
    if hi != hi or lo != lo:
        print('관절 상태를 못 받은 시행이 있다 — 측정 실패. 결론을 내지 않는다.')
        svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}')
        n.move_arm(POSE_FOLDED, 3.0)
        n.destroy_node()
        rclpy.shutdown()
        return
    if hi >= 0.04:
        print(f'공중에서도 최대 {hi:.3f}rad 어긋난다 → 컨트롤러·관절 설정 문제.')
    elif lo >= 0.04:
        print(f'공중은 정상({hi:.3f})인데 하강 자세에서 {lo:.3f}rad 막힌다 → 죠가 바닥에 눌렸다.')
        print('  → roll은 하강 전에 끝나 있어야 한다. 하강 자세 명령에 roll을 섞지 말 것.')
    else:
        print(f'두 조건 모두 정상(공중 {hi:.3f}, 하강 {lo:.3f}) — roll은 원인이 아니다.')
    svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
