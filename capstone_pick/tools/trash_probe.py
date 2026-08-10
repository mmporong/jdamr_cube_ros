"""쓰레기통 검출을 실좌표와 대조한다.

집기는 되는데 투입이 빗나간다(실측: 큐브가 통에서 0.20~0.55m 떨어진 곳에 떨어짐).
보정 상수를 만지기 전에 **로봇이 통을 어디로 보고 있는지**부터 확인해야 한다.

pick_node의 locate_trash()를 그대로 호출한다 — 검출 로직을 베껴 쓰면 진짜 코드가
아니라 사본을 재는 것이 된다. 로봇만 여러 위치로 옮겨 놓고, 검출한 base 좌표를
실좌표에서 계산한 참값과 나란히 찍는다.

사용 (ROS 환경 source 후):
    python3 trash_probe.py
"""
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import cv2  # noqa: E402
import numpy as np  # noqa: E402
import capstone_pick.pick_node as pn  # noqa: E402
from capstone_pick.pick_node import (PickNode, POSE_CARRY_SCAN,  # noqa: E402
                                     TRASH_POCKET_X)

TRASH_WORLD = (0.15, 0.62)      # room.world의 trash_can pose
# 로봇을 통 쪽으로 향하게 세울 위치들 (world). 통까지 거리를 달리해 본다.
POSES = [
    (0.00, 0.00), (0.05, 0.15), (0.08, 0.25), (0.10, 0.35), (0.12, 0.45),
]


def svc(req):
    subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '4000', '--req', req], capture_output=True, text=True)


def place_robot(x, y, yaw):
    """로봇을 (x, y, yaw)에 세운다. 쿼터니언은 z축 회전만."""
    svc(f'name: "jdamr_cube", position: {{x: {x}, y: {y}, z: 0.05}}, '
        f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')


# 쓸어볼 게이트 조합. locate_trash가 모듈 전역을 읽으므로 전역을 갈아끼워 잰다 —
# 검출 코드를 베끼지 않고 진짜 경로 그대로 조건만 바꾸기 위해서다.
#   d_lo      뎁스 하한. 0.35는 접근 목표(0.344)보다 커서 도착 지점이 항상 사각이다.
#   min_area  최소 픽셀. 벽을 0.18→0.09로 낮춰 보이는 면적이 반이 됐다.
# 팔 자세도 같이 쓴다. POSE_CARRY(정면)는 팔이 화면 중앙을 가려 검출이 자기 팔을
# 잡는다(실측: 로봇 위치와 무관하게 (+0.311,-0.01) 고정 — 통이 아니라 팔이다).
# POSE_CARRY_SCAN은 그걸 피하려고 pan을 -1.0으로 뺀 자세인데, 정의만 되어 있고
# 어디서도 쓰이지 않았다.
# 오차가 거리에 따라 커진다(+156mm@0.64m → +293mm@0.38m). 상수 보정으로는 못 고치는
# 형태다. 통과 바닥이 같은 회색이라 한 덩어리로 붙고, 가까울수록 통 뒤 바닥이 더
# 많이 딸려 들어와 중심이 뒤로 끌리는 것으로 본다. 높이 하한을 올려 바닥을 떼어낸다.
SWEEPS = [
    ('현재', POSE_CARRY_SCAN, dict(TRASH_Z_LO=0.02, TRASH_D_LO=0.20, TRASH_MIN_AREA=150)),
]


def measure(n, label, pose):
    """POSES 각각에서 locate_trash()를 부르고 참값과 대조. (검출수, 오차들) 반환."""
    hits, errs = 0, []
    print(f'\n[{label}] Z_LO={pn.TRASH_Z_LO} D_LO={pn.TRASH_D_LO} MIN_AREA={pn.TRASH_MIN_AREA}')
    print(f'  {"로봇(world)":>14} {"거리":>6} | {"검출 base":>18} {"검출 r":>7} '
          f'| {"참값 r":>7} | {"오차":>8}')
    for x, y in POSES:
        yaw = math.atan2(TRASH_WORLD[1] - y, TRASH_WORLD[0] - x)   # 통을 정면으로
        place_robot(x, y, yaw)
        n.move_arm(pose, 2.5)      # 실제 운반 자세 그대로
        time.sleep(1.2)
        for _ in range(30):
            rclpy.spin_once(n, timeout_sec=0.05)
        dx, dy = TRASH_WORLD[0] - x, TRASH_WORLD[1] - y
        tr = math.hypot(dx, dy)
        dump(n, f'{label}_{x:.2f}_{y:.2f}')
        loc = n.locate_trash()
        if loc is None:
            print(f'  {f"({x:.2f},{y:.2f})":>14} {tr:6.3f} | {"미검출":>18} {"—":>7} '
                  f'| {tr:7.3f} | {"—":>8}')
            continue
        hits += 1
        fx, fy = loc[0], loc[1]
        cr = math.hypot(fx, fy) + pn.TRASH_HALF
        errs.append((cr - tr) * 1000)
        print(f'  {f"({x:.2f},{y:.2f})":>14} {tr:6.3f} | ({fx:+.3f},{fy:+.3f}) {cr:7.3f} '
              f'| {tr:7.3f} | {(cr - tr) * 1000:+6.0f}mm')
    return hits, errs


def dump(n, tag):
    """검출 마스크와 최대 성분을 그림으로 남긴다. 무엇을 '통'으로 보는지 눈으로 본다."""
    n.color = n.depth = None
    if not n.spin_until(lambda: n.color is not None and n.depth is not None, 4.0):
        return
    hsv = cv2.cvtColor(n.color, cv2.COLOR_BGR2HSV)
    dep = np.nan_to_num(np.asarray(n.depth), nan=0.0, posinf=0.0)
    mask = ((hsv[:, :, 1] < pn.TRASH_S_MAX) & (hsv[:, :, 2] > pn.TRASH_V_LO)
            & (hsv[:, :, 2] < pn.TRASH_V_HI) & (dep > pn.TRASH_D_LO)
            & (dep < pn.TRASH_D_HI)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    vis = n.color.copy()
    vis[mask > 0] = (0, 0, 255)          # 마스크 전체 = 빨강
    if num > 1:
        big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        vis[labels == big] = (0, 255, 0)  # 최대 성분 = 초록 (검출이 통이라 믿는 것)
        d = dep[labels == big]
        d = d[(d > pn.TRASH_D_LO) & (d < pn.TRASH_D_HI)]
        if d.size:
            cv2.putText(vis, f'area={stats[big, cv2.CC_STAT_AREA]} '
                             f'd={np.percentile(d, 15):.2f}~{np.percentile(d, 85):.2f}',
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    out = os.path.expanduser(f'~/capstone_tools/logs/trashmask_{tag}.png')
    cv2.imwrite(out, np.hstack([n.color, vis]))


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: n.cam_info is not None, 15.0)
    summary = []
    for label, pose, over in SWEEPS:
        for k, v in over.items():
            setattr(pn, k, v)
        summary.append((label, *measure(n, label, pose)))
    print(f'\n{"조합":<18} {"검출":>8} {"평균오차":>10} {"최대오차":>10}')
    for label, hits, errs in summary:
        m = f'{sum(errs) / len(errs):+.0f}mm' if errs else '—'
        w = f'{max(errs, key=abs):+.0f}mm' if errs else '—'
        print(f'{label:<18} {hits}/{len(POSES):>6} {m:>10} {w:>10}')
    print(f'\n운반 접근은 통 중심이 r={TRASH_POCKET_X}에 올 때까지 전진한다 — '
          f'그 거리에서 검출이 되어야 마지막을 추측으로 가지 않는다.')
    place_robot(0.0, 0.0, 0.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
