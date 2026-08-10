"""손목캠(eye-in-hand)으로 근접 마커 검출이 되는지 잰다.

전방캠은 차체 고정이라 단독으로 겨냥할 수 없고, 통에 0.38m보다 가까워지면 마커가
프레임 위로 벗어나 미검출이다(실측 2/5). 접근 목표는 0.344m라 그대로면 마지막
구간을 오도메트리 추측으로 가야 하는데, 그 추측이 10cm 어긋나 투입이 빗나갔다.

손목캠은 팔이 곧 팬·틸트다. 팔 자세를 바꿔가며 마커를 프레임 안에 넣을 수 있는지,
넣었을 때 정확도가 쓸 만한지 확인한다.

사용 (ROS 환경 source 후): python3 wrist_aruco_probe.py
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
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick.pick_node import ARM_JOINTS, PickNode  # noqa: E402

WRIST_FRAME = 'arm_wrist_camera_link'
MARKER_WORLD = (0.15, 0.71, 0.13)
TRASH_WORLD = (0.15, 0.62)
MARKER_SIZE = 0.10
DICT = cv2.aruco.DICT_4X4_50
# 전방캠이 실패하는 근접 구간만 본다
POSES = [(0.08, 0.25), (0.10, 0.35), (0.12, 0.45)]
# 팔로 만드는 시선 방향. 운반 자세(0.15/0.15/1.28)를 기준으로 위아래·좌우를 훑는다.
# lift를 키우면 팔이 내려가고, wrist_flex를 줄이면 카메라가 위를 본다.
AIMS = [
    ('운반자세', [0.0, 0.15, 0.15, 1.28, 0.0]),
    ('약간 위', [0.0, 0.15, 0.15, 1.00, 0.0]),
    ('더 위', [0.0, 0.15, 0.15, 0.70, 0.0]),
    ('낮춰 전방', [0.0, 0.45, 0.15, 0.90, 0.0]),
    ('세워 전방', [0.0, -0.10, 0.30, 1.00, 0.0]),
]


def svc(req):
    subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '4000', '--req', req], capture_output=True, text=True)


class Wrist:
    """손목캠 영상·내부 파라미터 구독 (PickNode는 front만 info를 받는다)."""

    def __init__(self, node):
        self.img = self.info = None
        self.bridge = node.bridge
        node.create_subscription(Image, '/wrist_camera/image_raw',
                                 lambda m: setattr(self, 'img',
                                                   self.bridge.imgmsg_to_cv2(m, 'bgr8')), 1)
        node.create_subscription(CameraInfo, '/wrist_camera/camera_info',
                                 lambda m: setattr(self, 'info', m), 1)


def detect(n, w):
    """마커 중심의 base_footprint 좌표. 검출 실패 시 None."""
    w.img = None
    if not n.spin_until(lambda: w.img is not None and w.info is not None, 4.0):
        return None
    gray = cv2.cvtColor(w.img, cv2.COLOR_BGR2GRAY)
    d = cv2.aruco.getPredefinedDictionary(DICT)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(
            d, cv2.aruco.DetectorParameters()).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, d, parameters=cv2.aruco.DetectorParameters_create())
    if ids is None or len(ids) == 0:
        return None
    k = np.array(w.info.k, dtype=np.float64).reshape(3, 3)
    dist = np.array(w.info.d, dtype=np.float64) if len(w.info.d) else np.zeros(5)
    h = MARKER_SIZE / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    ok, _, tvec = cv2.solvePnP(obj, corners[0].reshape(4, 2).astype(np.float64),
                               k, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    # 광학(X우/Y하/Z전) → 링크(x전/y좌/z상)
    ox, oy, oz = (float(tvec[i]) for i in range(3))
    p = PointStamped()
    p.header.frame_id = WRIST_FRAME
    p.point.x, p.point.y, p.point.z = oz, -ox, -oy
    try:
        tf = n.tf_buffer.lookup_transform('base_footprint', WRIST_FRAME, rclpy.time.Time())
    except Exception:
        return None
    q = do_transform_point(p, tf).point
    return q.x, q.y, q.z


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'speed_scale:=1.0'])
    n = PickNode()
    w = Wrist(n)
    n.spin_until(lambda: w.info is not None, 15.0)
    print(f'{"로봇":>12} {"통까지":>7} {"팔 자세":<10} | {"추정 통중심(base)":>22} {"거리":>7} '
          f'| {"거리오차":>9} {"방위오차":>9}')
    print('-' * 92)
    best = {}
    for x, y in POSES:
        yaw = math.atan2(TRASH_WORLD[1] - y, TRASH_WORLD[0] - x)
        svc(f'name: "jdamr_cube", position: {{x: {x}, y: {y}, z: 0.05}}, '
            f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
        tr = math.hypot(TRASH_WORLD[0] - x, TRASH_WORLD[1] - y)
        for label, pose in AIMS:
            n.move_arm(dict(zip(ARM_JOINTS, pose)), 2.5)
            time.sleep(1.0)
            for _ in range(20):
                rclpy.spin_once(n, timeout_sec=0.05)
            m = detect(n, w)
            if m is None:
                print(f'{f"({x:.2f},{y:.2f})":>12} {tr:7.3f} {label:<10} | {"미검출":>22}')
                continue
            cx, cy = m[0] - (MARKER_WORLD[1] - TRASH_WORLD[1]), m[1]
            cr = math.hypot(cx, cy)
            e, b = (cr - tr) * 1000, math.degrees(math.atan2(cy, cx))
            best.setdefault(label, []).append(e)
            print(f'{f"({x:.2f},{y:.2f})":>12} {tr:7.3f} {label:<10} | '
                  f'({cx:+.3f},{cy:+.3f},{m[2]:+.3f}) {cr:7.3f} | {e:+7.0f}mm {b:+8.1f}°')
    print(f'\n{"팔 자세":<10} {"검출":>8} {"평균오차":>10}')
    for label, _ in AIMS:
        es = best.get(label, [])
        avg = f'{sum(es) / len(es):+.0f}mm' if es else '—'
        print(f'{label:<10} {len(es)}/{len(POSES):>6} {avg:>10}')
    print('\n대조 — 전방캠은 이 세 위치에서 모두 미검출이었다.')
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
