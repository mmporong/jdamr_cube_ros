"""YOLO 학습용 자동 라벨링 데이터셋 생성.

큐브를 무작위 배치(set_pose)하고 RGB 캡처. 시뮬 큐브는 단색이라 HSV 마스크가
곧 진리값 — 연결성분 bbox를 라벨로 쓴다(가림·화면밖 자동 처리).
로봇 자체의 빨간 데크는 크기·하단접촉 필터로 제외.
클래스: 0=blue_box 1=red_box 2=green_box
사용: python3 gen_dataset.py <장면수> [출력디렉토리]
"""
import math
import os
import random
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

N_SCENES = int(sys.argv[1]) if len(sys.argv) > 1 else 150
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser('~/yolo_cubes_ds')
RANGES = {
    0: [((100, 130, 100), (135, 255, 255))],                                  # blue
    1: [((0, 150, 100), (6, 255, 255)), ((174, 150, 100), (179, 255, 255))],  # red
    2: [((45, 80, 80), (75, 255, 255))],                                      # green
}


def svc(s_, t_, r_):
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


def set_pose(name, x, y, z, yaw=0.0):
    return svc('/world/room/set_pose', 'gz.msgs.Pose',
               f'name: "{name}" position {{x: {x} y: {y} z: {z}}} '
               f'orientation {{w: {math.cos(yaw / 2)} z: {math.sin(yaw / 2)}}}')


class Cap(Node):
    def __init__(self):
        super().__init__('ds_gen')
        self.bridge = CvBridge()
        self.img = None
        self.create_subscription(Image, '/rgbd_camera/image', self._i, 1)

    def _i(self, m):
        self.img = self.bridge.imgmsg_to_cv2(m, 'bgr8')

    def fresh(self, timeout=5.0):
        self.img = None
        t0 = time.time()
        while time.time() - t0 < timeout and self.img is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.img


def hsv_bboxes(img, cls):
    """해당 클래스 색의 연결성분 bbox 목록 (데크·노이즈 필터 포함)."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in RANGES[cls]:
        mask |= cv2.inRange(hsv, lo, hi)
    H, W = mask.shape
    num, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    out = []
    for i in range(1, num):
        a = stats[i, cv2.CC_STAT_AREA]
        x, y, w, h = (stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
                      stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
        if a < 25 or a > 8000:
            continue  # 노이즈 또는 데크급 대형
        if y + h >= H - 3 and w > 120:
            continue  # 하단 접촉 대형 = 자기 데크
        out.append((x, y, w, h))
    return out


def main():
    random.seed(7)
    for sub in ('images/train', 'images/val', 'labels/train', 'labels/val'):
        os.makedirs(os.path.join(OUT, sub), exist_ok=True)
    rclpy.init()
    n = Cap()
    svc('/world/room/remove', 'gz.msgs.Entity', 'name: "pick_table" type: MODEL')
    time.sleep(0.5)
    n_val = max(1, N_SCENES // 6)
    saved = 0
    for i in range(N_SCENES):
        split = 'val' if i < n_val else 'train'
        set_pose('jdamr_cube', 0.0, 0.0, 0.03, random.uniform(-0.25, 0.25))
        for name in ('pick_blue', 'pick_red', 'pick_green'):
            set_pose(name, random.uniform(0.40, 1.35), random.uniform(-0.55, 0.55), 0.015)
        time.sleep(0.9)
        img = n.fresh()
        if img is None:
            print(f'[{i}] 캡처 실패 — 건너뜀', flush=True)
            continue
        H, W = img.shape[:2]
        lines = []
        for cls in (0, 1, 2):
            for x, y, w, h in hsv_bboxes(img, cls):
                lines.append(f'{cls} {(x + w / 2) / W:.6f} {(y + h / 2) / H:.6f} {w / W:.6f} {h / H:.6f}')
        cv2.imwrite(os.path.join(OUT, f'images/{split}/{i:04d}.png'), img)
        with open(os.path.join(OUT, f'labels/{split}/{i:04d}.txt'), 'w') as f:
            f.write('\n'.join(lines))
        saved += 1
        if i % 20 == 0:
            print(f'[{i}/{N_SCENES}] {split} 라벨 {len(lines)}개', flush=True)
    with open(os.path.join(OUT, 'data.yaml'), 'w') as f:
        f.write(f'path: {OUT}\ntrain: images/train\nval: images/val\n'
                'names:\n  0: blue_box\n  1: red_box\n  2: green_box\n')
    print(f'데이터셋 생성 완료: {OUT} ({saved}장)', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
