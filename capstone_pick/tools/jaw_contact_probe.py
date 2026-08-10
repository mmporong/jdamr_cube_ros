"""아랫턱이 큐브 위로 떨어지는가 — 높이 문제인지 위치 문제인지 갈라 잰다.

사용자가 화면에서 같은 현상을 반복해 지적했다: 하강이 끝나면 아랫턱이 큐브
**위에** 있다. 나는 그동안 전후(x) 정렬만 파고들었고 높이는 `arm_gripper_link`
원점 좌표로만 봤는데, 링크 원점은 손가락 접촉면이 어디인지 말해주지 않는다.
URDF를 봐도 죠는 메시라 접촉면 높이를 계산으로 얻을 수 없다.

그래서 **큐브가 실제로 밀리는지**로 판정한다. 두 가설이 서로 다른 흔적을 남긴다.

  H1 높이 문제 — 죠가 큐브에 부딪힌다
      → 하강만 해도 큐브가 움직인다(밀림·회전). 닫기 전에 이미 어긋난다.
  H2 위치 문제 — 높이는 맞는데 죠 사이에 안 들어온다
      → 큐브는 가만히 있는데 닫으면 허공이거나 얕게 걸린다.

둘은 손볼 곳이 정반대다. 높이 문제면 하강 목표(lift)를, 위치 문제면 포켓과
손목캠 기준점을 고쳐야 한다. 그래서 이 프로브는 **하강 전후 큐브 이동량**을
1순위 지표로 찍는다.

lift를 쓸어가며 재므로 "어느 높이가 맞는가"의 답도 같은 표에서 나온다.
자세족 불변식(lift+elbow+wrist≈1.58)을 지켜 죠 평면을 수직으로 유지한다 —
lift만 바꾸면 죠가 기울어 높이 실험이 아니라 각도 실험이 되어 버린다.

큐브 yaw는 0으로 고정한다. 기울기가 있으면 roll 정렬이 끼어들어 높이·위치
변수와 섞인다.

사용 (ROS 환경 source 후): python3 jaw_contact_probe.py
"""
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import (GRIPPER_CLOSED,  # noqa: E402
                                     PickNode, POSE_FOLDED, POSE_PRE_FLOOR)

CUBE_X = 0.385                   # 파지 성립 구간 한가운데 (pocket_scan 실측)
ELBOW = 0.15
INVARIANT = 1.58                 # lift + elbow + wrist
LIFTS = [1.05, 1.10, 1.15, 1.15, 1.20, 1.25]  # 1.15가 현재 값 — 재현성 확인차 두 번
MOVED = 0.004                    # 이보다 움직였으면 죠가 큐브를 쳤다고 본다


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
    """큐브 world 좌표 (x, y, z). 실패하면 None — 표에 그대로 남긴다."""
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


def tf_xz(n, frame):
    for _ in range(20):
        try:
            t = n.tf_buffer.lookup_transform('base_footprint', frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.z
        except Exception:
            spin(n, 0.2)
    return float('nan'), float('nan')


def trial(n, lift):
    wrist = INVARIANT - lift - ELBOW
    hover = dict(POSE_PRE_FLOOR)
    descended = dict(zip(['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
                          'arm_wrist_flex', 'arm_wrist_roll'],
                         [0.0, lift, ELBOW, wrist, 0.0]))
    # 초기화: 로봇 원점, 큐브는 포켓 한가운데에 yaw 0으로
    # 리치 누적을 시행마다 0으로 되돌린다. 안 하면 `_descend_vertical`의 keep_x
    # 보정이 시행을 넘어 쌓여 상한(REACH_TOTAL_MAX)을 넘고, 그때부터 보정이
    # 통째로 중단되어 뒤쪽 시행이 조용히 오염된다(첫 측정에서 +107mm까지 쌓였다).
    # 파이프라인은 grasp() 진입 때 초기화하므로 이 문제가 없다.
    n._reach_applied = 0.0
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    svc(f'name: "pick_object_green", position: {{x: {CUBE_X}, y: 0.0, z: 0.02}}, '
        f'orientation: {{w: 1}}')
    spin(n, 1.5)
    n.move_gripper(1.2)
    n.move_arm(hover, 3.0)
    spin(n, 0.8)
    before = cube_pose()

    # 실제 파이프라인과 같은 경로로 내린다 — 직교 직선 하강
    n._descend_vertical(hover, descended)
    spin(n, 1.0)
    after = cube_pose()
    tcp_x, tcp_z = tf_xz(n, 'arm_gripper_frame_link')
    jaw_x, jaw_z = tf_xz(n, 'arm_gripper_link')

    # 닫고 들어서 실제로 물리는지
    n.move_gripper(GRIPPER_CLOSED)
    spin(n, 2.0)
    ang = getattr(n, 'gripper_angle', None)
    eff = getattr(n, 'gripper_effort', None)
    n.move_arm({**descended, 'arm_shoulder_lift': 0.5, 'arm_elbow_flex': 0.4,
                'arm_wrist_flex': 0.6}, 2.5)
    spin(n, 1.2)
    lifted = cube_pose()

    d_desc = (None if (before is None or after is None) else
              math.dist(before[:2], after[:2]))
    z_up = None if lifted is None else lifted[2]
    # 실좌표를 하나라도 못 얻은 시행은 '측정불가'다. 이걸 그냥 실패로 세면
    # gz 무응답일 때 "큐브도 안 밀렸고 물리지도 않았다 → 위치 문제"라는
    # 확신에 찬 오진이 나온다. 판정은 실좌표로만 한다는 규칙의 연장이다.
    ok_meas = None not in (before, after, lifted)
    return dict(lift=lift, wrist=wrist, before=before, after=after,
                d_desc=d_desc, tcp_z=tcp_z, jaw_x=jaw_x, jaw_z=jaw_z,
                ang=ang, eff=eff, z_up=z_up, measured=ok_meas,
                held=(ok_meas and z_up > 0.05))


def fmt(v, w=7, p=3):
    return ' ' * (w - 1) + '-' if v is None or v != v else f'{v:{w}.{p}f}'


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True
    print(f'큐브: x={CUBE_X}, 4cm (윗면 z=0.04, 중심 z=0.02), yaw 0')
    print(f'"하강밀림"이 {MOVED * 1000:.0f}mm를 넘으면 죠가 큐브를 친 것 = 높이 문제\n')
    print(f'{"lift":>5} {"wrist":>6} | {"하강밀림":>9} | {"TCP z":>7} {"아랫턱x":>8} {"아랫턱z":>8} '
          f'| {"물림각":>7} {"부하":>7} {"들림z":>7}  판정')
    print('-' * 104)
    rows = []
    for lift in LIFTS:
        r = trial(n, lift)
        rows.append(r)
        hit = r['d_desc'] is not None and r['d_desc'] > MOVED
        if not r['measured']:
            verdict = '측정불가(gz 무응답)'
        elif r['held']:
            verdict = '물림'
        elif hit:
            verdict = '죠가 큐브를 침(높이)'
        elif r['ang'] is not None and r['ang'] < 0.0:
            verdict = '허공(위치)'
        else:
            verdict = '얕은 걸침'
        print(f'{r["lift"]:5.2f} {r["wrist"]:6.2f} | '
              f'{fmt(None if r["d_desc"] is None else r["d_desc"] * 1000, 9, 1)} | '
              f'{fmt(r["tcp_z"])} {fmt(r["jaw_x"], 8)} {fmt(r["jaw_z"], 8)} | '
              f'{fmt(r["ang"])} {fmt(r["eff"], 7, 2)} {fmt(r["z_up"])}  {verdict}')

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
    hits = [r for r in valid if r['d_desc'] is not None and r['d_desc'] > MOVED]
    held = [r for r in valid if r['held']]
    if hits and not held:
        print(f'하강만으로 큐브가 밀린 시행 {len(hits)}/{len(rows)} — H1 높이 문제.')
        print('  죠가 큐브에 부딪힌다. 닫기 전에 이미 어긋나므로 위치 보정으로는 못 고친다.')
        print(f'  밀림이 가장 작은 lift: {min(hits, key=lambda r: r["d_desc"])["lift"]:.2f}')
    elif held:
        ls = ' '.join(f'{r["lift"]:.2f}' for r in held)
        best = held[len(held) // 2]
        print(f'물리는 lift: {ls}  (중앙 {best["lift"]:.2f})')
        print(f'→ POSE_GRASP_FLOOR lift={best["lift"]:.2f}, '
              f'wrist={INVARIANT - best["lift"] - ELBOW:.2f}')
        if hits:
            print(f'  단, {len(hits)}개 시행에서 하강 중 큐브가 밀렸다 — 여유가 좁다.')
    else:
        print('어느 높이에서도 안 물리는데 큐브도 안 밀렸다 — H2 위치 문제.')
        print('  죠가 큐브에 닿지도 못한다. 전후(포켓·GRASP_REF)를 다시 재야 한다.')
        for r in valid:
            if r['jaw_x'] == r['jaw_x']:
                print(f'  lift {r["lift"]:.2f}: 아랫턱 x={r["jaw_x"]:.3f} vs 큐브 x={CUBE_X} '
                      f'→ 전후 {(CUBE_X - r["jaw_x"]) * 1000:+.0f}mm')
    svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
