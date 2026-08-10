"""ArUco 마커 기반 통 위치 추정의 정확도를 실좌표와 대조해 정량화한다.

지금 HSV 블롭 검출은 거리·자세에 따라 +20~237mm로 흔들리고 방향 정보가 없다.
피듀셜 마커는 solvePnP로 6-DoF 자세를 주고, 마커 한 변(0.10m)이 기지값이라
정확도의 근거가 명확하다 — 물류 로봇 도킹의 표준 방식이다.

여기서는 '쓸 만한가'를 숫자로 확정한다: 여러 거리에서 위치 오차[mm]와 방위
오차[deg]를 재고, HSV 방식과 같은 표에 놓는다.

사용 (ROS 환경 source 후): python3 aruco_probe.py
"""
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import (CAMERA_FRAME, PickNode,  # noqa: E402
                                     POSE_CARRY_SCAN)

MARKER_WORLD = (0.15, 0.71, 0.13)      # gen_aruco_sdf.py가 넣은 위치
TRASH_WORLD = (0.15, 0.62)             # 실제 투입 목표(통 중심)
MARKER_SIZE = 0.10
DICT = cv2.aruco.DICT_4X4_50
POSES = [(0.00, 0.00), (0.05, 0.15), (0.08, 0.25), (0.10, 0.35), (0.12, 0.45)]


def svc(req):
    subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '4000', '--req', req], capture_output=True, text=True)


def place_robot(x, y, yaw):
    svc(f'name: "jdamr_cube", position: {{x: {x}, y: {y}, z: 0.05}}, '
        f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')


def detect(n):
    """마커 중심의 base_footprint 좌표. 검출 실패 시 None.

    solvePnP는 카메라 광학 프레임 기준 자세를 준다. 마커 코너의 3D 좌표(마커 중심
    원점, 한 변 MARKER_SIZE)와 검출된 2D 코너를 짝지어 푼다 — 여기에 임계값이 없다.
    """
    n.color = None
    if not n.spin_until(lambda: n.color is not None and n.cam_info is not None, 4.0):
        return None
    gray = cv2.cvtColor(n.color, cv2.COLOR_BGR2GRAY)
    d = cv2.aruco.getPredefinedDictionary(DICT)
    if hasattr(cv2.aruco, 'ArucoDetector'):                      # OpenCV 4.7+
        corners, ids, _ = cv2.aruco.ArucoDetector(
            d, cv2.aruco.DetectorParameters()).detectMarkers(gray)
    else:                                                        # 4.6 이하
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, d, parameters=cv2.aruco.DetectorParameters_create())
    if ids is None or len(ids) == 0:
        return None
    k = np.array(n.cam_info.k, dtype=np.float64).reshape(3, 3)
    dist = np.array(n.cam_info.d, dtype=np.float64) if len(n.cam_info.d) else np.zeros(5)
    h = MARKER_SIZE / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, corners[0].reshape(4, 2).astype(np.float64),
                                  k, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    # solvePnP는 광학 프레임(X=우, Y=하, Z=전방) 기준이고 CAMERA_FRAME은 링크
    # 프레임(x=전방, y=좌, z=상)이다. 이 축변환을 빼먹으면 높이가 0.13m 대신
    # 1.2m로 나온다(실측) — pick_node._pixel_to_base와 같은 변환을 쓴다.
    ox, oy, oz = (float(tvec[i]) for i in range(3))
    p = PointStamped()
    p.header.frame_id = CAMERA_FRAME
    p.point.x, p.point.y, p.point.z = oz, -ox, -oy
    try:
        tf = n.tf_buffer.lookup_transform('base_footprint', CAMERA_FRAME, rclpy.time.Time())
    except Exception:
        return None
    q = do_transform_point(p, tf).point
    return q.x, q.y, q.z


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: n.cam_info is not None, 15.0)
    print(f'{"로봇(world)":>14} {"통까지":>7} | {"ArUco 통중심(base)":>22} {"거리":>7} '
          f'| {"참값":>7} | {"거리오차":>9} {"방위오차":>9}')
    print('-' * 96)
    errs, brgs, hits = [], [], 0
    for x, y in POSES:
        yaw = math.atan2(TRASH_WORLD[1] - y, TRASH_WORLD[0] - x)
        place_robot(x, y, yaw)
        n.move_arm(POSE_CARRY_SCAN, 2.5)
        time.sleep(1.2)
        for _ in range(30):
            rclpy.spin_once(n, timeout_sec=0.05)
        tr = math.hypot(TRASH_WORLD[0] - x, TRASH_WORLD[1] - y)
        m = detect(n)
        if m is None:
            print(f'{f"({x:.2f},{y:.2f})":>14} {tr:7.3f} | {"미검출":>22} {"—":>7} '
                  f'| {tr:7.3f} | {"—":>9} {"—":>9}')
            continue
        hits += 1
        # 마커 → 투입 목표는 고정 변환이다(설치 시 정해진 값): 통 중심은 마커보다
        # y로 -0.09m, z로 -0.11m. 로봇이 마커를 정면으로 보므로 base에서는 x -0.09.
        cx, cy = m[0] - (MARKER_WORLD[1] - TRASH_WORLD[1]), m[1]
        cr = math.hypot(cx, cy)
        errs.append((cr - tr) * 1000)
        brgs.append(math.degrees(math.atan2(cy, cx)))
        print(f'{f"({x:.2f},{y:.2f})":>14} {tr:7.3f} | ({cx:+.3f},{cy:+.3f},{m[2]:+.3f}) '
              f'{cr:7.3f} | {tr:7.3f} | {(cr - tr) * 1000:+7.0f}mm '
              f'{math.degrees(math.atan2(cy, cx)):+8.1f}°')
    print(f'\n검출 {hits}/{len(POSES)}')
    if errs:
        print(f'거리 오차: 평균 {sum(errs) / len(errs):+.0f}mm, 최대 {max(errs, key=abs):+.0f}mm')
        print(f'방위 오차: 최대 {max(brgs, key=abs):+.1f}°  (참값은 모두 0°)')
        print('\n대조 — HSV 블롭 방식 실측: 거리 오차 +20~237mm, 방위 정보 없음')
    place_robot(0.0, 0.0, 0.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
