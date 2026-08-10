"""운반 자세에서 팔이 통 마커를 가리는가 — pan 각도별로 잰다.

운반 루프에서 ArUco가 매번 None이었다. 저장된 실패 프레임을 보니 답이 나왔다:
**통과 마커가 팔 뒤에 가려져 있다.** 로봇은 통 앞 0.5m에 제대로 도착했는데
화면에는 노란 그리퍼가 크게 자리하고, 마커의 흰 여백만 그 옆으로 삐져나왔다.
정지 단독 호출(`aruco_probe.py`)에서는 잡혔던 이유도 이것이다 — 그때는 팔 자세가 달랐다.

`POSE_CARRY_SCAN`이 pan을 -1.0rad 돌려 시야를 연다고 되어 있는데, 그 값이
충분한지 한 번도 재본 적이 없다. 여기서 그것만 잰다.

큐브는 쥐지 않는다. 가림은 팔 자세만의 함수라 빈 손으로도 같은 답이 나오고,
쥔 상태를 재현하려면 파지부터 성공시켜야 해서 측정이 비싸진다.

사용 (ROS 환경 source 후): python3 carry_pan_probe.py
"""
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import (PickNode, POSE_CARRY,  # noqa: E402
                                     TRASH_ARUCO_DICT, TRASH_ARUCO_ID)

TRASH_WORLD = (0.15, 0.62)          # 통 중심 (world)
STANDOFF = 0.50                     # 통 앞 이 거리에서 본다 (TRASH_POCKET_X와 같다)
PANS = [0.0, -0.6, -1.0, -1.4, -1.8, +1.0, +1.4]   # -1.0이 현재 값


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def detect(n, save_as=None):
    """마커가 보이나. (검출여부, 마커 픽셀 면적 비율, 화면 내 위치)"""
    n.color = None
    if not n.spin_until(lambda: n.color is not None and n.cam_info is not None, 4.0):
        return None
    frame = n.color
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    d = cv2.aruco.getPredefinedDictionary(TRASH_ARUCO_DICT)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(
            d, cv2.aruco.DetectorParameters()).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, d)
    if save_as:
        try:
            cv2.imwrite(save_as, frame)
        except Exception:
            pass
    if ids is None:
        return None
    flat = [int(v) for v in ids.flatten()]
    if TRASH_ARUCO_ID not in flat:
        return None
    c = corners[flat.index(TRASH_ARUCO_ID)].reshape(4, 2)
    area = cv2.contourArea(c.astype(np.float32))
    cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
    return area, cx, cy


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: n.cam_info is not None, 20.0)
    outdir = os.path.expanduser('~/capstone_tools/logs/carry_pan')
    os.makedirs(outdir, exist_ok=True)

    # 통을 향해 통 앞 STANDOFF에 세운다
    yaw = math.atan2(TRASH_WORLD[1], TRASH_WORLD[0])
    rx = TRASH_WORLD[0] - STANDOFF * math.cos(yaw)
    ry = TRASH_WORLD[1] - STANDOFF * math.sin(yaw)
    svc(f'name: "jdamr_cube", position: {{x: {rx:.4f}, y: {ry:.4f}, z: 0.05}}, '
        f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
    spin(n, 2.0)
    print(f'로봇을 통 앞 {STANDOFF}m에 통을 향해 세웠다 (world {rx:.3f},{ry:.3f})')
    print('운반 자세(POSE_CARRY)에서 pan만 바꾸며 마커가 보이는지 본다.\n')
    print(f'{"pan[rad]":>9} {"deg":>6} | {"검출":>5} {"마커면적px":>11} {"화면위치":>14}  판정')
    print('-' * 68)

    ok_pans = []
    for p in PANS:
        n.move_arm({**POSE_CARRY, 'arm_shoulder_pan': p}, 2.5)
        spin(n, 1.2)
        r = detect(n, os.path.join(outdir, f'pan{p:+.1f}.png'))
        if r is None:
            print(f'{p:9.2f} {math.degrees(p):6.0f} | {"✗":>5} {"—":>11} {"—":>14}  가려짐/미검출')
            continue
        area, cx, cy = r
        ok_pans.append((p, area))
        print(f'{p:9.2f} {math.degrees(p):6.0f} | {"○":>5} {area:11.0f} '
              f'{f"({cx:.0f},{cy:.0f})":>14}  보임')

    print('\n--- 판정 ---')
    if not ok_pans:
        print('어느 pan에서도 마커를 못 본다 — 가림이 아니라 다른 문제다')
        print(f'  저장된 프레임을 확인하라: {outdir}')
    else:
        best = max(ok_pans, key=lambda t: t[1])
        cur = dict(ok_pans).get(-1.0)
        print(f'보이는 pan: ' + ', '.join(f'{p:+.1f}' for p, _ in ok_pans))
        print(f'가장 크게 보이는 pan: {best[0]:+.2f} (면적 {best[1]:.0f}px)')
        if cur is None:
            print(f'→ 현재 값 -1.0에서는 **안 보인다.** POSE_CARRY_SCAN을 '
                  f'{best[0]:+.2f}로 바꿔야 한다.')
        else:
            print(f'→ 현재 값 -1.0에서도 보인다(면적 {cur:.0f}px). '
                  f'가림이 아닌 다른 원인을 봐야 한다.')
    print(f'\n프레임: {outdir}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
