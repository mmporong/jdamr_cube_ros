"""투입만 따로 검증한다 — 파지·들기까지 만든 뒤 통을 앞에 놓고 넣어 본다.

전체 파이프라인으로 재면 접근(잔차 ±45mm)과 운반(주행 오차)이 섞여, 투입이
실패해도 그게 투입 탓인지 앞 단계 탓인지 갈리지 않는다. 여기서는 차체를 고정하고
통만 원하는 거리·방위에 놓아 **투입 단계만** 본다.

기하는 tools/drop_path_probe.py가 시뮬 없이 이미 검증했다(도착 허용오차 전 범위
성립, 착지 오차 0.00mm). 여기서 보는 것은 그 기하가 **물리와 만났을 때**도
성립하는가다 — 죠가 통 테두리를 치지 않는지, 놓는 순간 큐브가 튀지 않는지.

판정은 실좌표다. 큐브가 통 개구부 안(반폭 68mm)이고 벽 상단(0.090)보다 낮게
내려앉았으면 들어간 것이다.

사용 (ROS 환경 source 후): python3 drop_probe.py [시행수]
"""
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick import kinematics as K  # noqa: E402
from capstone_pick.pick_node import (ARM_JOINTS, CARRY_UP, CARRY_X,  # noqa: E402
                                     CUBE_CZ, GRIP_HOLD_MARGIN, GRIPPER_CLOSED,
                                     PickNode, POSE_FOLDED, TRASH_OPEN_HALF,
                                     TRASH_WALL_TOP)

GRASP_AT = (0.375, 0.0)          # 파지 지점 (팔 단독으로 안정적인 곳)
# (통까지 거리, 방위[도]) — 도착 조건 |er|<25mm, |brg|<5.7도의 모서리를 훑는다
CASES = [(0.500, 0.0), (0.475, 0.0), (0.525, 0.0), (0.500, 4.0), (0.500, -4.0)]
TRASH_HOME = (0.15, 0.62)        # 월드 원래 위치 — 끝나고 되돌린다


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


def model_xyz(name):
    for _ in range(3):
        try:
            out = subprocess.run(['gz', 'model', '-m', name, '-p'],
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


def setup(n, cx, cy):
    """파지 → 들기까지. 성공하면 True."""
    path = K.descend_path(cx, cy, CUBE_CZ, 0.0)
    if path is None:
        return False
    svc(f'name: "{CUBE}", position: {{x: 1.5, y: 1.5, z: 0.0155}}, orientation: {{w: 1}}')
    spin(n, 0.4)
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    svc(f'name: "{CUBE}", position: {{x: {cx:.4f}, y: {cy:.4f}, '
        f'z: {CUBE_CZ:.4f}}}, orientation: {{w: 1}}')
    spin(n, 1.0)

    n.move_gripper(1.2)
    n.move_arm(dict(zip(ARM_JOINTS, path[0])), 3.0)
    for q in path[1:]:
        n.move_arm(dict(zip(ARM_JOINTS, q)), 1.0)
    n.move_gripper(GRIPPER_CLOSED, effort=4.0)
    n.move_gripper(GRIPPER_CLOSED, wait=False, effort=30.0)
    spin(n, 0.6)
    n.gripper_angle = None
    n.spin_until(lambda: n.gripper_angle is not None, 1.5)
    a = n.gripper_angle
    if a is None or a <= -0.15:
        return False
    # 파이프라인과 같은 유지 목표 — 완전 닫힘을 계속 명령하면 큐브를 짜낸다
    n.hold_target = max(GRIPPER_CLOSED, a - GRIP_HOLD_MARGIN)
    n._hold_grip()
    n._ik_grasped = True
    n._ik_cube = (cx, cy, CUBE_CZ)
    # 들기: _lift_ik과 같은 계단 → 운반 자세
    for up in (0.02, 0.05, 0.09, 0.14):
        q = K.grasp_q(cx, cy, CUBE_CZ, up=up)
        if q is not None:
            n.move_arm(dict(zip(ARM_JOINTS, q)), 1.5)
    n._hold_grip()
    carry = K.grasp_q(CARRY_X, 0.0, CUBE_CZ, up=CARRY_UP)
    if carry is not None:
        n.move_arm(dict(zip(ARM_JOINTS, carry)), 2.5)
    spin(n, 0.6)
    c = model_xyz(CUBE)
    return c is not None and c[2] > 0.10          # 들려 있으면 통과


def main():
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else len(CASES)
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', f'target_color:={COLOR}',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True
    n.target_color = COLOR

    print('투입 단계만 검증 — 차체 고정, 통을 원하는 거리·방위에 놓는다')
    print(f'통 개구부 반폭 {TRASH_OPEN_HALF * 1000:.0f}mm · 벽 상단 {TRASH_WALL_TOP:.3f}m\n')
    ok = 0
    done = 0
    for t in range(trials):
        r, bd = CASES[t % len(CASES)]
        b = math.radians(bd)
        tx, ty = r * math.cos(b), r * math.sin(b)
        print(f'--- 시행 {t + 1}/{trials}  통 r={r:.3f}m 방위 {bd:+.1f}도 → ({tx:.3f},{ty:+.3f}) ---')
        # **통을 먼저 치우고 파지한다.** 통 앞면이 x=0.42까지 오는데 파지 지점이
        # 0.375라, 통을 먼저 놓으면 내려가는 팔이 통에 부딪힌다 — 실측에서
        # `자세 도달 미완 1.029rad`로 4/5가 파지 단계에서 깨졌다.
        svc(f'name: "trash_can", position: {{x: {TRASH_HOME[0]}, y: {TRASH_HOME[1]}, z: 0}}, '
            f'orientation: {{w: 1}}')
        spin(n, 0.4)
        if not setup(n, *GRASP_AT):
            print('    파지·들기 실패 — 투입 판정 제외\n')
            continue
        done += 1
        # 큐브를 든 뒤에 통을 목표 위치로 옮긴다. 로봇은 원점·yaw 0에 고정되어 있다.
        svc(f'name: "trash_can", position: {{x: {tx:.4f}, y: {ty:.4f}, z: 0}}, orientation: {{w: 1}}')
        spin(n, 0.5)
        n._trash_r, n._trash_brg = r, b
        n._drop_ik()
        spin(n, 1.0)
        c = model_xyz(CUBE)
        can = model_xyz('trash_can')
        if c is None or can is None:
            print('    좌표 실패\n')
            continue
        dx, dy = c[0] - can[0], c[1] - can[1]
        inside = abs(dx) < TRASH_OPEN_HALF and abs(dy) < TRASH_OPEN_HALF and c[2] < TRASH_WALL_TOP
        ok += inside
        print(f'    큐브 ({c[0]:.3f},{c[1]:+.3f},{c[2]:.3f})  통 중심 대비 '
              f'({dx * 1000:+.0f},{dy * 1000:+.0f})mm  높이 {c[2] * 1000:.0f}mm')
        print(f'    → {"○ 통 안" if inside else "✗ 밖"}'
              + ('' if inside else f'  (개구부 반폭 {TRASH_OPEN_HALF * 1000:.0f}mm)'))
        print()

    print('=== 요약 ===')
    print(f'  투입 성공 {ok}/{done}' + (f' (파지 실패로 {trials - done}회 제외)' if done < trials else ''))
    svc(f'name: "trash_can", position: {{x: {TRASH_HOME[0]}, y: {TRASH_HOME[1]}, z: 0}}, '
        f'orientation: {{w: 1}}')
    svc(f'name: "{CUBE}", position: {{x: 0.45, y: 0.26, z: 0.0155}}, orientation: {{w: 1}}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
