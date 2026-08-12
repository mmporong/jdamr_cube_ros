"""손목캠 각도 측정이 roll과 함께 도는가 — 규약을 실측으로 확정한다.

코드 안에 정반대 주석이 둘 있다.

  pick_node.py:314, 1142  "카메라는 roll 관절 앞단이라 롤을 돌려도 측정 불변"
  pick_node.py:1373       "손목캠은 roll 관절과 **함께 돈다** → 측정값은 잔여 각도"

URDF는 첫 번째 편이다.

  arm_wrist_roll : parent=arm_wrist_link → child=arm_gripper_link
  손목캠         : parent=arm_wrist_link          ← roll의 부모와 같다

즉 roll을 돌리면 그리퍼만 돌고 카메라는 제자리다. 그렇다면 `_ready_to_close`가
쓰는 `d_ang = ROLL_SIGN * ang`은 절대 yaw를 잔여 각도로 오해한 것이고,
`- roll`이 빠져 있다. 잔여를 0으로 읽으면 대각선으로 문다 — 실측으로 물림각
0.604가 나왔는데, 이는 4cm 큐브의 대각선(5.7cm)에 해당한다.

여기서 그 규약만 잰다. 큐브 yaw를 고정하고 roll만 바꾼다.

  카메라가 **안 돌면** : 측정값은 roll과 무관하게 일정 (= 큐브 절대 yaw)
  카메라가 **함께 돌면**: 측정값 = 절대 yaw − roll (roll이 커지면 줄어든다)

사용 (ROS 환경 source 후): python3 roll_convention_probe.py
"""
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import (PickNode, POSE_FOLDED,  # noqa: E402
                                     POSE_GRASP_FLOOR, POSE_PRE_FLOOR)

CUBE_X = 0.405                 # 새 포켓 (grasp_ref_scan 실측)
CUBE_YAW = math.radians(30)    # 큐브를 이만큼 기울여 놓는다
ROLLS = [0.0, 0.25, 0.50, -0.25]


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def cube_model():
    """월드에 실제로 있는 픽 대상 이름. 하드코딩하면 set_pose가 조용히 실패한다."""
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
COLOR = CUBE.split('_')[-1]


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', f'target_color:={COLOR}',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True

    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    svc(f'name: "{CUBE}", position: {{x: {CUBE_X}, y: 0.0, z: 0.02}}, '
        f'orientation: {{z: {math.sin(CUBE_YAW / 2):.6f}, w: {math.cos(CUBE_YAW / 2):.6f}}}')
    time.sleep(1.5)
    # **놓았는지 확인한다.** 앞 시행이 큐브를 통 안에 두고 끝나면 그대로 남는데,
    # 확인 없이 진행하면 "큐브가 없는 화면"을 미검출로 기록한다(오늘 세 번 겪었다).
    try:
        out = subprocess.run(['gz', 'model', '-m', CUBE, '-p'],
                             capture_output=True, text=True, timeout=15).stdout
        L = out.splitlines()
        pos = None
        for i, ln in enumerate(L):
            if '- Pose' in ln and i + 1 < len(L):
                pos = [float(t) for t in L[i + 1].strip().strip('[]').split()]
                break
        if pos is None or math.hypot(pos[0] - CUBE_X, pos[1]) > 0.03:
            print(f'큐브 배치 실패 — 실제 {pos} (목표 {CUBE_X}, 0). 측정을 중단한다.')
            n.destroy_node(); rclpy.shutdown(); sys.exit(1)
        print(f'큐브 배치 확인: ({pos[0]:.3f}, {pos[1]:+.3f}, {pos[2]:.3f})')
    except Exception as e:
        print(f'큐브 배치 확인 실패: {e}'); n.destroy_node(); rclpy.shutdown(); sys.exit(1)
    spin(n, 1.5)
    def sweep(label, base_pose, descend):
        """한 자세에서 roll을 쓸며 측정값을 본다. (roll, ang) 목록."""
        print(f'\n=== {label} ===')
        print(f'{"roll[rad]":>10} {"측정 ang[도]":>13}   해석')
        print('-' * 44)
        rows = []
        for r in ROLLS:
            n.move_gripper(1.2)
            n.move_arm(dict(POSE_PRE_FLOOR), 2.5)
            if descend:
                # 파이프라인과 같은 경로로 내린 뒤 그 자세에서 잰다.
                # 하강 후에는 죠가 큐브를 감싸므로 마스크 형상이 달라질 수 있다.
                n._reach_applied = 0.0
                n._descend_vertical(dict(POSE_PRE_FLOOR),
                                    {**POSE_GRASP_FLOOR, 'arm_wrist_roll': r})
            else:
                n.move_arm({**base_pose, 'arm_wrist_roll': r}, 2.0)
            spin(n, 1.0)
            ang = n._wrist_cube_angle()
            if ang is None:
                print(f'{r:10.2f} {"미검출":>13}')
                continue
            rows.append((r, ang))
            print(f'{r:10.2f} {math.degrees(ang):13.1f}')
        return rows

    print(f'큐브 yaw = {math.degrees(CUBE_YAW):+.0f}도 고정. roll만 바꾸며 측정값을 본다.')
    print('측정값이 roll을 따라 변하면 그 자세에서는 "잔여 각도"로 읽어야 한다.')

    hi = sweep('상공 자세 (wrist_align이 재는 곳)', dict(POSE_PRE_FLOOR), False)
    lo = sweep('하강 자세 (_ready_to_close가 재는 곳)', dict(POSE_GRASP_FLOOR), True)

    def verdict(rows, where):
        if len(rows) < 3:
            print(f'{where}: 표본 부족 — 판정 불가')
            return None
        angs = [a for _, a in rows]
        comp = [a - r for r, a in rows]   # 잔여 가설이면 (ang - roll)이... 아래 설명 참조
        s_raw = max(angs) - min(angs)
        # 카메라가 함께 돌면 측정값 = 절대yaw - roll → (측정값 + roll)이 일정
        s_res = max(a + r for r, a in rows) - min(a + r for r, a in rows)
        print(f'\n{where}')
        print(f'  측정값 자체의 산포     : {math.degrees(s_raw):5.1f}도')
        print(f'  (측정값 + roll)의 산포 : {math.degrees(s_res):5.1f}도')
        if s_raw < s_res:
            print('  → roll과 무관하게 일정 = 측정값은 **절대 yaw**. 잔여 = ROLL_SIGN*ang − roll')
            return 'absolute'
        print('  → roll을 따라 변한다 = 측정값이 이미 **잔여 각도**. 잔여 = ROLL_SIGN*ang')
        return 'residual'

    print('\n--- 판정 ---')
    v_hi = verdict(hi, '상공 (wrist_align)')
    v_lo = verdict(lo, '하강 (_ready_to_close)')
    if v_hi and v_lo:
        if v_hi != v_lo:
            print(f'\n※ 두 자세의 규약이 다르다 (상공 {v_hi} / 하강 {v_lo}).')
            print('  같은 함수 `_wrist_cube_angle()`을 쓰지만 해석이 달라야 한다 —')
            print('  하강 후에는 죠가 큐브를 감싸 마스크 주축이 죠를 따라가기 때문이다.')
            print('  wrist_align은 절대 yaw로, _ready_to_close는 잔여로 읽는 것이 맞다.')
        else:
            print(f'\n두 자세 모두 {v_hi} — 같은 규약으로 읽으면 된다.')

    svc(f'name: "{CUBE}", position: {{x: 0.45, y: 0.26, z: 0.02}}, '
        'orientation: {w: 1}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
