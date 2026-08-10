"""팔 pan을 돌린 상태에서도 파지가 성립하는가 — 방위별로 잰다.

`jaw_contact_probe.py`로 확인한 것: 큐브가 정면(pan 0) 포켓에 있으면 현재
설정으로 6/6 물린다. 높이도 죠 충돌도 문제가 아니다.

그런데 파이프라인은 실패한다. 실좌표를 보니 **거리는 맞았다** — 접근 종료 시
로봇→큐브가 0.383m, 0.379m로 포켓(0.385) 한가운데였는데도 닫으면 허공이었다.
남은 차이는 방위다. 그 시행들의 큐브는 y=0.26(옆)이라 팔이 pan -17.4°, -7.4°로
돌아간 상태에서 물어야 했다.

설계상 포켓은 반경으로 정의되고(r_target = hypot(...)), 잔여 방위는 팔 pan이
흡수하기로 되어 있다. 접근 수렴 임계도 그 전제로 |brg| < 0.30rad(17.2°)까지
허용한다. **그 전제가 실제로 성립하는지는 한 번도 재본 적이 없다.**

여기서 그것만 잰다. 큐브를 반경 0.385에 고정하고 방위만 바꿔 놓은 뒤, 파이프라인과
같은 방식으로 pan을 돌려 파지한다. 손목캠 정렬은 일부러 쓰지 않는다 — 정렬이
메워주는 양을 빼고 **기하 자체가 성립하는 범위**를 봐야 하기 때문이다.

사용 (ROS 환경 source 후): python3 pan_grasp_probe.py
"""
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import (GRIPPER_CLOSED, PAN_AXIS_X,  # noqa: E402
                                     PAN_BASE_BEARING, PickNode, POCKET_FLOOR,
                                     POSE_FOLDED, POSE_GRASP_FLOOR, POSE_PRE_FLOOR)

# 방위·반경은 **pan 회전축 기준**이다. base_footprint 원점 기준으로 놓으면
# 파이프라인이 재는 것과 다른 값을 재게 된다 — approach()는
# `r = hypot(xb - PAN_AXIS_X, yb)`, `brg = atan2(yb, xb - PAN_AXIS_X)`로 잰다.
POCKET_R = math.hypot(POCKET_FLOOR[0] - PAN_AXIS_X, POCKET_FLOOR[1])   # ≈0.226
BEARINGS = [0, 10, 17]     # 0=정면, 17=접근 수렴 임계(0.30rad)의 경계


def svc(req):
    """모델을 옮긴다. 실패하면 그 사실을 알린다 — 조용히 넘기면 배치가 안 된 채
    이전 시행의 자리에서 재게 되고, 그 값이 표에 정상값처럼 실린다."""
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    if r.returncode != 0 or 'true' not in r.stdout.lower():
        print(f'  [경고] set_pose 실패: {req[:48]}... → {r.stdout.strip()[:40]}')
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


def trial(n, brg_deg):
    brg = math.radians(brg_deg)
    # 큐브는 pan축 기준 (반경 POCKET_R, 방위 brg). base 좌표로 되돌려 놓는다.
    cx = PAN_AXIS_X + POCKET_R * math.cos(brg)
    cy = POCKET_R * math.sin(brg)
    # pan은 approach()가 돌려주는 값과 **같은 식**으로 구한다. 부호를 직접 쓰면
    # 반대로 넣기 쉽다 — arm_shoulder_pan 축은 URDF에서 아래쪽(0,0,-1)이라
    # +pan이 오른쪽(-y)이고, 그래서 파이프라인도 방위에 음수를 붙인다.
    # 여기서 부호를 틀리면 큐브 반대편을 무는 셈이라 모든 비영 방위가 실패하고,
    # "방위 한계는 0°"라는 잘못된 결론이 기준 문서로 들어간다.
    pan = -(brg - PAN_BASE_BEARING)
    hover = {**POSE_PRE_FLOOR, 'arm_shoulder_pan': pan}
    descended = {**POSE_GRASP_FLOOR, 'arm_shoulder_pan': pan}

    n._reach_applied = 0.0
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    svc(f'name: "pick_object_green", position: {{x: {cx:.4f}, y: {cy:.4f}, z: 0.02}}, '
        f'orientation: {{w: 1}}')
    spin(n, 1.5)
    n.move_gripper(1.2)
    n.move_arm(hover, 3.0)
    spin(n, 0.8)
    before = cube_pose()

    n._descend_vertical(hover, descended)
    spin(n, 1.0)
    after = cube_pose()
    n.move_gripper(GRIPPER_CLOSED)
    spin(n, 2.0)
    ang = getattr(n, 'gripper_angle', None)
    n.move_arm({**descended, 'arm_shoulder_lift': 0.5, 'arm_elbow_flex': 0.4,
                'arm_wrist_flex': 0.6}, 2.5)
    spin(n, 1.2)
    lifted = cube_pose()

    d_desc = (None if (before is None or after is None)
              else math.dist(before[:2], after[:2]))
    z_up = None if lifted is None else lifted[2]
    # 실좌표를 하나라도 못 얻었으면 이 시행은 '측정불가'다. 이걸 실패로 세면
    # gz가 무응답일 때 "어느 방위에서도 안 물린다"는 확신에 찬 오진이 나온다.
    ok_meas = None not in (before, after, lifted)
    return dict(brg=brg_deg, cx=cx, cy=cy, pan=pan, d_desc=d_desc, ang=ang, z_up=z_up,
                measured=ok_meas, held=(ok_meas and z_up > 0.05))


def fmt(v, w=7, p=3):
    return ' ' * (w - 1) + '-' if v is None or v != v else f'{v:{w}.{p}f}'


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True
    print(f'큐브를 pan축 기준 반경 {POCKET_R:.3f}m에 고정하고 방위만 바꾼다.')
    print('pan은 approach()와 같은 식으로 구한다: pan = -(방위 - PAN_BASE_BEARING)')
    print('접근 수렴 임계가 17.2°까지 허용하므로, 그 범위가 실제로 물리는지가 관건이다.\n')
    print(f'{"방위":>5} {"큐브(x,y)":>16} {"pan":>7} | {"하강밀림":>9} {"물림각":>7} '
          f'{"들림z":>7}  판정')
    print('-' * 76)
    rows = []
    for b in BEARINGS:
        r = trial(n, b)
        rows.append(r)
        if not r['measured']:
            v = '측정불가(gz 무응답)'
        elif r['held']:
            v = '물림'
        elif r['ang'] is not None and r['ang'] < 0.0:
            v = '허공'
        else:
            v = '얕은 걸침'
        print(f'{r["brg"]:4d}° ({r["cx"]:.3f},{r["cy"]:+.3f}) {r["pan"]:7.3f} | '
              f'{fmt(None if r["d_desc"] is None else r["d_desc"] * 1000, 9, 1)} '
              f'{fmt(r["ang"])} {fmt(r["z_up"])}  {v}')

    print('\n--- 판정 ---')
    valid = [r for r in rows if r['measured']]
    if not valid:
        print(f'측정 실패 — 유효 시행 0/{len(rows)}. gz가 실좌표를 안 돌려줬다.')
        print('  결론을 내지 않는다. 시뮬을 재기동하고 다시 재라.')
        svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}')
        n.move_arm(POSE_FOLDED, 3.0)
        n.destroy_node()
        rclpy.shutdown()
        return
    if len(valid) < len(rows):
        print(f'※ 측정불가 {len(rows) - len(valid)}건은 제외하고 판정한다.')
    ok = [r['brg'] for r in valid if r['held']]
    bad = [r['brg'] for r in valid if not r['held']]
    if not bad:
        print('모든 방위에서 물린다 — pan 회전은 원인이 아니다. 다른 곳을 봐야 한다.')
    elif not ok:
        print('어느 방위에서도 안 물린다 — 이 프로브의 전제(포켓 반경)부터 재확인해야 한다.')
    else:
        print(f'물리는 방위: {ok}   실패: {bad}')
        print(f'→ 파지가 성립하는 방위 한계는 {max(ok)}° 부근이다.')
        print(f'  접근 수렴 임계(17.2°)가 이보다 느슨하면, 접근은 "합격"인데 파지는'
              f' 실패하는 구간이 생긴다 — 임계를 {max(ok)}° 안쪽으로 좁혀야 한다.')
    svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
