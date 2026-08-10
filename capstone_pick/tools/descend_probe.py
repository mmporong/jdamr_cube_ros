"""하강 자세에 실제로 도달하는가 — 명령과 실제를 관절 단위로 대조한다.

파지 실패 3/3의 로그가 전부 같은 곳을 가리켰다: `자세 도달 미완 (최대 오차
0.265~0.268rad)`. 하강 명령이 끝나도 팔이 그 자세에 없다는 뜻인데, 그러면
손목캠 기준점(GRASP_REF)이 통째로 무효가 된다 — 그 기준은 '하강 자세에
도달한 상태'에서 잰 값이기 때문이다. 실제로 3사이클 모두 손목캠은
"전후 +6~+12mm, 통과"라고 했는데 닫으면 허공(각도 -0.17)이었다.

그래서 여기서는 파지를 보지 않는다. **하강 명령이 도달하는지만** 본다.

  · 큐브 없이 / 큐브 놓고 두 조건으로 각각 잰다.
    큐브가 없어도 미달이면 관절 한계나 컨트롤러 문제고,
    큐브가 있을 때만 미달이면 죠가 큐브·바닥에 눌린 충돌 문제다.
  · 관절별 목표 대 실제를 그대로 찍는다. 최대 오차 하나로는 원인을 못 짚는다.
  · 아랫턱(arm_gripper_link)과 TCP(arm_gripper_frame_link)의 높이를 함께 본다.
    사용자 기준은 "큐브가 아랫턱에 거의 닿고 죠 중점에" 이므로 아랫턱이 기준이다.

사용 (ROS 환경 source 후): python3 descend_probe.py
"""
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import (ARM_JOINTS, PickNode,  # noqa: E402
                                     POSE_FOLDED, POSE_GRASP_FLOOR, POSE_PRE_FLOOR)

CUBE_X = 0.385                    # 파지 성립 구간 한가운데 (pocket_scan 실측)
SETTLE = [2.0, 5.0, 10.0]         # 명령 후 이 시점들에서 재본다 — 느린 수렴과 고착을 구분


def svc(req):
    subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '4000', '--req', req], capture_output=True, text=True)


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def tf_xz(n, frame):
    """base_footprint 기준 (x, z). 조회 실패는 NaN으로 표에 남긴다."""
    for _ in range(20):
        try:
            t = n.tf_buffer.lookup_transform('base_footprint', frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.z
        except Exception:
            spin(n, 0.2)
    return float('nan'), float('nan')


def report(n, target, when):
    jp = getattr(n, 'joint_pos', None) or {}
    tcp_x, tcp_z = tf_xz(n, 'arm_gripper_frame_link')
    jaw_x, jaw_z = tf_xz(n, 'arm_gripper_link')
    errs = {j: jp.get(j, float('nan')) - target[j] for j in ARM_JOINTS}
    finite = [(abs(v), k) for k, v in errs.items() if v == v]
    if not finite:      # 관절 상태를 하나도 못 받았다 — 죽지 말고 그대로 남긴다
        print(f'  {when:5.1f}s | 관절 상태 미수신 — 측정불가')
        return float('nan')
    worst = max(finite)
    print(f'  {when:5.1f}s | ' + ' '.join(
        f'{j.replace("arm_", "")[:5]}{errs[j]:+.3f}' for j in ARM_JOINTS)
        + f' | 최대 {worst[0]:.3f}({worst[1].replace("arm_", "")})'
          f' | TCP z{tcp_z:+.3f} 아랫턱 x{jaw_x:+.3f} z{jaw_z:+.3f}')
    return worst[0]


def run(n, label, with_cube, vertical=False):
    """vertical=True면 파이프라인이 실제로 쓰는 직교 하강 경로로 내린다.

    관절 보간(`move_arm` 한 방)과 `_descend_vertical`은 다른 경로다. 후자는
    x를 유지하려고 단계마다 `_reach_arm`으로 lift·elbow·wrist에 오프셋을 얹으므로,
    끝난 자세가 명목 `POSE_GRASP_FLOOR`가 **아니다**. 명목 목표로만 재면
    "도달 오차 0.000"이 나오지만 그건 파이프라인이 겪는 상황이 아니다.
    그래서 여기서는 명목과 실제 명령(`_last_arm`) 두 기준으로 함께 찍는다.
    """
    print(f'\n=== {label} ===')
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    if with_cube:
        svc(f'name: "pick_object_green", position: {{x: {CUBE_X}, y: 0.0, z: 0.02}}')
    else:
        svc('name: "pick_object_green", position: {x: 1.5, y: 1.5, z: 0.02}')
    spin(n, 1.5)
    n.move_gripper(1.2)
    n.move_arm(POSE_PRE_FLOOR, 3.0)
    spin(n, 1.0)
    print('  상공 도달 확인')
    report(n, POSE_PRE_FLOOR, 0.0)
    print('  하강 명령: ' + ' '.join(
        f'{j.replace("arm_", "")[:5]}{POSE_GRASP_FLOOR[j]:+.2f}' for j in ARM_JOINTS)
        + ('   [직교 하강 — 파이프라인 경로]' if vertical else '   [관절 보간]'))
    if vertical:
        n.floor_mode = True
        n._reach_applied = 0.0
        n._descend_vertical(dict(POSE_PRE_FLOOR), dict(POSE_GRASP_FLOOR))
        print(f'  누적 리치 {n._reach_applied * 1000:+.0f}mm')
    else:
        n.move_arm(POSE_GRASP_FLOOR, 4.0)   # 내부 도달 대기(6초)까지 포함해 돌아온다
    # 반환 이후로도 계속 재본다. 늦게라도 수렴하면 대기 시간 문제이고,
    # 값이 굳어 있으면 물리적으로 막힌 것이다 — 둘은 손볼 곳이 다르다.
    last = None
    t0 = time.time()
    for w in SETTLE:
        spin(n, max(0.0, w - (time.time() - t0)))
        last = report(n, POSE_GRASP_FLOOR, w)
    if vertical and n._last_arm:
        # 같은 자세를 '실제로 명령한 값' 기준으로 한 번 더 잰다. 이쪽이 0에 가깝고
        # 명목 기준만 크다면, 미달의 정체는 추종 실패가 아니라 리치 오프셋이다.
        print('  ─ 실제 명령(_last_arm) 기준으로 다시 재면:')
        report(n, dict(n._last_arm), 0.0)
    return last


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 15.0)
    print('관절 오차 = 실제 - 목표 [rad]. 허용 오차는 코드와 같은 0.04.')
    empty = run(n, '큐브 없이 (팔만)', with_cube=False)
    cube = run(n, f'큐브 놓고 (x={CUBE_X})', with_cube=True)
    vert = run(n, f'큐브 놓고 · 직교 하강 (x={CUBE_X})', with_cube=True, vertical=True)
    print('\n--- 판정 ---')
    if vert is not None and vert == vert and vert >= 0.04:
        print(f'직교 하강 경로에서 명목 목표 대비 {vert:.3f}rad 미달.')
        print('  위 "_last_arm 기준" 줄이 작다면 추종 실패가 아니라 리치 오프셋이 원인이다')
        print('  — 그렇다면 도달 검사의 목표를 _last_arm으로 삼아야 한다.')
    if empty is not None and empty >= 0.04:
        print(f'큐브가 없어도 {empty:.3f}rad 미달 → 팔 자체가 그 자세에 못 간다.')
        print('  관절 한계·중력 처짐·컨트롤러 게인 문제이지 충돌이 아니다.')
        print('  → POSE_GRASP_FLOOR가 도달 가능한 자세인지부터 재검토해야 한다.')
    elif cube is not None and cube >= 0.04:
        print(f'빈 팔은 도달하는데 큐브가 있으면 {cube:.3f}rad 미달 → 죠가 큐브에 눌린다.')
        print('  → 하강 목표가 큐브를 파고드는 높이다. lift를 줄여야 한다.')
    else:
        print('두 조건 모두 도달 — 하강 자세는 문제가 아니다. 다른 곳을 봐야 한다.')
    svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
