"""팔만으로 줍고 놓는다 — 주행 없음. 성공률까지 잰다.

전체 파이프라인은 주행 접근과 통 인식에서 막혔다(A·B 6회씩, 최종 0/6). 두 병목이
모두 팔 바깥의 문제이고, 팔 자체는 이미 검증돼 있다 — 파지 10/11, 들기 4/4,
투입 5/5. 그래서 큐브와 통을 팔이 닿는 곳에 고정하고 **팔만으로** 완주시킨다.

무대는 `armonly_stage.py`가 만든다(큐브 base (0.360,0), 통 base (0.231,-0.276)).

무엇을 비전으로 하고 무엇을 고정하나:

  · **큐브는 손목캠으로 찾는다.** 전방캠은 못 쓴다 — 큐브가 base 0.36m로 가까워
    팔 접힘 자세가 시선을 가린다(주행 파이프라인은 멀리서 보며 접근했지만 여기는
    처음부터 가깝다). 손목캠은 팔에 달려 있어 가릴 수가 없다.
    대략 위치를 알고 손목캠으로 정밀 정렬하는 것은 산업 로봇의 표준 절차다
    (coarse-to-fine). 정렬은 기존 `_servo_correct`를 그대로 쓴다
  · **통은 고정 좌표를 쓴다.** 방위 -50도라 전방캠 시야(수평 반각 33도) 밖이고,
    차체를 돌리면 주행이 다시 들어온다. 실물에서도 투입 지점은 티칭으로 잡으므로
    이 절충이 이관에 불리하지 않다

투입은 `_drop_ik`도 `K.drop_path`도 쓰지 않는다. 전자는 남은 방위를 **차체 회전**으로
흡수하는데 여기서는 차체를 안 움직이고, 후자는 **끝점 피치로 고정**해 보간하는 탓에
크게 돌아가는 경로의 중간이 안 풀린다(계산: -95도 고정 시 t=0.3에서 해 없음).
대신 구간마다 `reach_q`로 피치를 다시 골라 두 구간으로 간다 — 높이를 유지한 채
pan을 돌려 통 위로 간 뒤 제자리 하강. 반경 0.36에서는 pan이 방위를 흡수할 여유가
있다(정면 0.5m에서는 어깨 오프셋 때문에 해가 사라졌지만 여기서는 풀린다).

판정은 실좌표다. 로그의 PICK_SUCCESS는 통을 잘못 인식한 채로도 찍힌다.

사용 (ROS 환경 source 후): python3 armonly_pick.py [시행수]
"""
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick import kinematics as K  # noqa: E402
from capstone_pick.pick_node import (ARM_JOINTS, CUBE_CZ, DROP_CZ,  # noqa: E402
                                     GRIP_HOLD_MARGIN, GRIPPER_CLOSED, PickNode,
                                     POSE_FOLDED, TRASH_OPEN_HALF, TRASH_WALL_TOP)

CUBE_BASE = (0.360, 0.000)
TRASH_BASE = (0.231, -0.276)
STAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'armonly_stage.py')


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def cube_model():
    try:
        out = subprocess.run(['gz', 'model', '--list'], capture_output=True,
                             text=True, timeout=10).stdout
        for ln in out.splitlines():
            n = ln.strip().lstrip('-').strip()
            if n.startswith('pick'):
                return n
    except Exception:
        pass
    return 'pick_blue'


CUBE = cube_model()
COLOR = CUBE.split('_')[-1]


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
                    v = [float(t) for t in L[i + 1].strip().strip('[]').split()]
                    if len(v) >= 3:
                        return v[0], v[1], v[2]
                except ValueError:
                    continue
    return None


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def stage(n=None, yaw_deg=0.0):
    """무대를 다시 세운다 — 앞 시행이 큐브를 어디에 뒀든 초기화.

    실패하면 **사유를 남긴다.** 종전에는 stdout을 버려서 "무대 배치 실패"만 찍혔고,
    그게 로봇 문제인지 배치 도구 문제인지 갈리지 않았다(8회 중 2회가 이랬다).
    """
    r = subprocess.run(['python3', STAGE, COLOR, f'{yaw_deg:.1f}'],
                       capture_output=True, text=True, timeout=90)
    if '배치 완료' in r.stdout:
        return True
    msg = (r.stdout or r.stderr).strip().splitlines()
    # 마지막 몇 줄이 아니라 **문제를 말하는 줄**을 고른다. 종전에는 tail만 잘라
    # 정작 필요한 오차 줄이 빠지고 "위 오차를 보라"만 남았다.
    key = [m.strip() for m in msg
           if ('오차' in m or '실패' in m) and '위 오차를 보라' not in m]
    tail = ' | '.join(key) or ' | '.join(m.strip() for m in msg[-3:] if m.strip())
    if n is not None:
        n.get_logger().warning(f'무대 배치 실패 — {tail[:200]}')
    else:
        print(f'    배치 실패 상세: {tail[:200]}')
    return False


def drop(n):
    """통 위로 팔을 옮겨 큐브를 떨어뜨린다. 차체는 건드리지 않는다.

    경로를 두 구간으로 나눈다.
      ① 운반 높이(큐브 위 0.18)를 유지한 채 pan을 돌려 통 위로
      ② 그 자리에서 투입 높이까지 내린다

    `K.drop_path`를 쓰지 않는 이유가 있다. 그 함수는 **끝점 피치로 고정**해
    보간하는데, 여기처럼 크게 돌아가는 경로에서는 중간점이 그 피치로 안 풀린다
    (계산: 피치 -95도 고정 시 t=0.3부터 해 없음). 구간마다 `reach_q`로 피치를
    다시 고르면 전 구간이 풀리고, 실제 변화도 -90 → -95도로 5도뿐이다.

    통 위로 먼저 간 뒤 내리는 순서는 물리적으로도 안전하다 — 비스듬히 내려가면
    죠가 통 테두리를 스칠 수 있다.
    """
    tx, ty = TRASH_BASE
    fx, fy = K.PAN_X + 0.17, 0.0          # 운반 자세의 큐브 위치
    n.get_logger().info(
        f'투입: 통 ({tx:.3f},{ty:+.3f}) 방위 '
        f'{math.degrees(math.atan2(ty, tx)):.0f}도 — 차체 고정, pan으로만 돌린다')

    # ① 높이 유지하며 통 위로
    for i in range(1, 7):
        t = i / 6
        q, _ = K.reach_q(fx + (tx - fx) * t, fy + (ty - fy) * t, CUBE_CZ, up=0.180)
        if q is None:
            n.get_logger().warning(f'투입 경로 IK 없음 (선회 t={t:.2f})')
            return False
        n.move_arm(dict(zip(ARM_JOINTS, q)), 1.2 if i == 1 else 0.7)
        if i % 3 == 1:
            n._hold_grip()
    # ② 제자리 하강
    for u in (0.160, 0.140, 0.120, DROP_CZ - CUBE_CZ):
        q, pitch = K.reach_q(tx, ty, CUBE_CZ, up=u)
        if q is None:
            n.get_logger().warning(f'투입 하강 IK 없음 (up={u:.3f})')
            return False
        n.move_arm(dict(zip(ARM_JOINTS, q)), 0.8)
    n._hold_grip()
    time.sleep(0.5)
    n.get_logger().info(f'개구부 위 (피치 {math.degrees(pitch):.0f}도) — 그리퍼 열기')
    n.move_gripper(0.8)
    time.sleep(1.0)
    n.move_arm(POSE_FOLDED, 3.0)
    return True


def one(n, yaw_deg=0.0):
    """한 번의 줍고 놓기. (성공, 사유)."""
    if not stage(n, yaw_deg):
        return False, '무대 배치 실패(사유는 위 경고)'
    spin(n, 1.0)
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    spin(n, 0.8)

    # --- 정렬: 대략 위치로 팔을 가져가 손목캠으로 다듬는다(coarse-to-fine) ---
    cx, cy = n._servo_correct(*CUBE_BASE, CUBE_CZ)
    err = math.dist((cx, cy), CUBE_BASE) * 1000
    # 큐브 기울기도 **상공에서** 잰다. 하강 자세의 각도 측정은 죠가 큐브를 가려
    # 죽어 있다 — 실측(roll_convention_probe): 큐브를 30도 기울여도 하강에서는
    # 전부 -0.0도, 상공에서는 20.2도로 값이 산다. roll은 이 값으로 정한다.
    ang = n._wrist_cube_angle()
    yaw = ang if ang is not None else 0.0
    n.get_logger().info(f'손목캠 정렬 → 큐브 ({cx:.3f},{cy:+.3f}) · 초기 대비 {err:.0f}mm · '
                        f'기울기 {math.degrees(yaw):+.0f}도'
                        + ('' if ang is not None else ' (미검출 → 0으로)'))

    # --- 파지: 큐브 밖에서 내린 뒤 수평 삽입, 유지 목표는 접촉각 안쪽 ---
    path = K.descend_path(cx, cy, CUBE_CZ, yaw)
    if path is None:
        return False, f'파지 IK 없음 ({cx:.3f},{cy:+.3f})'
    n.move_arm(dict(zip(ARM_JOINTS, path[0])), 3.0)
    for q in path[1:]:
        n.move_arm(dict(zip(ARM_JOINTS, q)), 1.0)
    n.move_gripper(GRIPPER_CLOSED, effort=4.0)          # 살살 닫기
    n.move_gripper(GRIPPER_CLOSED, wait=False, effort=30.0)
    spin(n, 0.8)
    n.gripper_angle = n.gripper_effort = None
    n.spin_until(lambda: n.gripper_angle is not None, 2.0)
    a, e = n.gripper_angle, n.gripper_effort
    n.get_logger().info(f'닫힘 신호: 각도={a if a is None else round(a, 3)} '
                        f'부하={e if e is None else round(e, 2)}')
    if a is None or a <= -0.15:
        return False, f'물지 못함 (각도 {a}, 부하 {e})'
    n.hold_target = max(GRIPPER_CLOSED, a - GRIP_HOLD_MARGIN)
    n._hold_grip()
    n._ik_grasped = True
    n._ik_cube = (cx, cy, CUBE_CZ)

    # --- 들기: 같은 좌표를 높이만 올려 푼다(자세 급변 없음) ---
    for up in (0.02, 0.05, 0.09, 0.14, 0.18):
        q = K.grasp_q(cx, cy, CUBE_CZ, up=up)
        if q is not None:
            n.move_arm(dict(zip(ARM_JOINTS, q)), 1.2)
    n._hold_grip()
    spin(n, 0.5)
    held = model_xyz(CUBE)
    if held is None or held[2] < 0.10:
        return False, '들기 실패 (큐브가 안 딸려옴)'

    # --- 투입 ---
    if not drop(n):
        return False, '투입 IK 없음'
    spin(n, 1.0)

    c = model_xyz(CUBE)
    t = model_xyz('trash_can')
    if c is None or t is None:
        return False, '좌표 확인 실패'
    dx, dy = c[0] - t[0], c[1] - t[1]
    inside = (abs(dx) < TRASH_OPEN_HALF and abs(dy) < TRASH_OPEN_HALF
              and c[2] < TRASH_WALL_TOP)
    return inside, (f'통 중심 대비 ({dx * 1000:+.0f},{dy * 1000:+.0f})mm '
                    f'높이 {c[2] * 1000:.0f}mm')


def main():
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    # 두 번째 인자로 큐브 기울기 목록을 준다: "0,15,30,45" — 시행마다 돌아가며 쓴다
    yaws = [float(v) for v in sys.argv[2].split(',')] if len(sys.argv) > 2 else [0.0]
    rclpy.init(args=['--ros-args', '-p', f'target_color:={COLOR}',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True
    n.target_color = COLOR

    print(f'팔 단독 줍고 놓기 — 주행 없음 · {trials}회')
    print(f'  큐브 base {CUBE_BASE} (비전으로 찾음) · 통 base {TRASH_BASE} (고정)')
    print(f'  개구부 반폭 {TRASH_OPEN_HALF * 1000:.0f}mm · 벽 상단 {TRASH_WALL_TOP:.3f}m\n')
    ok = 0
    by_yaw = {}
    for t in range(trials):
        yd = yaws[t % len(yaws)]
        print(f'--- 시행 {t + 1}/{trials} · 큐브 yaw {yd:+.0f}도 ---')
        good, why = one(n, yd)
        ok += good
        by_yaw.setdefault(yd, [0, 0])
        by_yaw[yd][0] += good
        by_yaw[yd][1] += 1
        print(f'    {"○ 통 안" if good else "✗"}  {why}\n')
    print(f'=== 팔 단독 성공률: {ok}/{trials} ===')
    if len(by_yaw) > 1:
        print('기울기별:')
        for yd in sorted(by_yaw):
            a, b = by_yaw[yd]
            print(f'  yaw {yd:+5.0f}도 : {a}/{b}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
