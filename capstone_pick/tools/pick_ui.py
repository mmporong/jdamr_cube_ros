#!/usr/bin/env python3
"""캡스톤 픽 관제 UI.

버튼으로 무대 리셋·픽 실행을 하고, 비전 뎁스 카메라와 손목(팔) 카메라를
라이브로 보여준다. ROS 환경이 소싱된 셸에서 실행:
    python3 ~/capstone_tools/pick_ui.py
"""
import os
import queue
import subprocess
import threading
import tkinter as tk
from tkinter import scrolledtext, ttk

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

TOOLS = os.path.expanduser('~/capstone_tools')
VIEW_W, VIEW_H = 420, 315
DEPTH_NEAR, DEPTH_FAR = 0.15, 3.0


class CamNode(Node):
    """카메라 두 대의 최신 프레임만 보관."""

    def __init__(self):
        super().__init__('pick_ui_cams')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.depth = None
        self.wrist = None
        self.create_subscription(Image, '/rgbd_camera/depth_image', self._depth_cb, 1)
        self.create_subscription(Image, '/wrist_camera/image_raw', self._wrist_cb, 1)

    def _depth_cb(self, msg):
        d = self.bridge.imgmsg_to_cv2(msg)
        with self.lock:
            self.depth = d

    def _wrist_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.wrist = img


def depth_to_bgr(d):
    """뎁스(m)를 컬러맵으로: 가까울수록 붉게, 무효 픽셀은 검정."""
    v = np.nan_to_num(np.asarray(d, np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    valid = v > 0.05
    v = np.clip(v, DEPTH_NEAR, DEPTH_FAR)
    n = ((DEPTH_FAR - v) / (DEPTH_FAR - DEPTH_NEAR) * 255).astype(np.uint8)
    bgr = cv2.applyColorMap(n, cv2.COLORMAP_JET)
    bgr[~valid] = 0
    return bgr


def bgr_to_photo(bgr):
    """BGR ndarray → tk.PhotoImage (PPM 경유, PIL 불필요)."""
    bgr = cv2.resize(bgr, (VIEW_W, VIEW_H))
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])
    header = f'P6\n{VIEW_W} {VIEW_H}\n255\n'.encode()
    return tk.PhotoImage(data=header + rgb.tobytes())


class App:
    def __init__(self, root, cam):
        self.root = root
        self.cam = cam
        self.proc = None
        self.logq = queue.Queue()
        root.title('캡스톤 픽 관제')
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        cams = tk.Frame(root)
        cams.pack(side=tk.TOP, padx=6, pady=6)
        self.depth_label = self._cam_panel(cams, '비전 뎁스 카메라 (rgbd_camera/depth)', 0)
        self.wrist_label = self._cam_panel(cams, '손목 카메라 (wrist_camera)', 1)

        ctrl = tk.Frame(root)
        ctrl.pack(side=tk.TOP, fill=tk.X, padx=6)
        tk.Button(ctrl, text='무대 리셋 (파란 타깃)', command=lambda: self.run_stage('grip_stage.py')).pack(side=tk.LEFT, padx=2)
        tk.Button(ctrl, text='빨간 타깃 무대', command=lambda: self.run_stage('red_stage.py')).pack(side=tk.LEFT, padx=2)
        tk.Label(ctrl, text='   색:').pack(side=tk.LEFT)
        self.color = tk.StringVar(value='blue')
        ttk.Combobox(ctrl, textvariable=self.color, width=7, state='readonly',
                     values=('blue', 'red', 'green', 'orange', 'pink')).pack(side=tk.LEFT)
        tk.Label(ctrl, text=' 속도:').pack(side=tk.LEFT)
        self.speed = tk.StringVar(value='4.0')
        tk.Entry(ctrl, textvariable=self.speed, width=5).pack(side=tk.LEFT)
        self.grip_only = tk.BooleanVar(value=False)
        tk.Checkbutton(ctrl, text='그립만', variable=self.grip_only).pack(side=tk.LEFT, padx=4)
        self.run_btn = tk.Button(ctrl, text='픽 실행', bg='#cfe8cf', command=self.run_pick)
        self.run_btn.pack(side=tk.LEFT, padx=6)
        tk.Button(ctrl, text='중지', bg='#e8cfcf', command=self.stop_pick).pack(side=tk.LEFT)

        self.log = scrolledtext.ScrolledText(root, height=10, width=110, state=tk.DISABLED)
        self.log.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=6)

        self._photos = [None, None]
        self.tick()

    def _cam_panel(self, parent, title, col):
        f = tk.Frame(parent)
        f.grid(row=0, column=col, padx=4)
        tk.Label(f, text=title).pack()
        lbl = tk.Label(f, width=VIEW_W, height=VIEW_H, bg='#222')
        lbl.pack()
        return lbl

    # ---- 실행 ----
    def run_stage(self, script):
        self.append(f'== 무대: {script} ==')
        threading.Thread(target=self._stage_worker, args=(script,), daemon=True).start()

    def _stage_worker(self, script):
        r = subprocess.run(['python3', os.path.join(TOOLS, script)],
                           capture_output=True, text=True)
        self.logq.put(r.stdout.strip() or r.stderr.strip()[-300:])

    def run_pick(self):
        if self.proc and self.proc.poll() is None:
            self.append('이미 실행 중')
            return
        cmd = ['ros2', 'run', 'capstone_pick', 'pick', '--ros-args',
               '-p', f'speed_scale:={self.speed.get()}',
               '-p', f'target_color:={self.color.get()}']
        if self.grip_only.get():
            cmd += ['-p', 'skip_approach:=true']
        self.append('== 픽 실행: ' + ' '.join(cmd[4:]) + ' ==')
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True)
        self.run_btn.config(state=tk.DISABLED)
        threading.Thread(target=self._pick_reader, daemon=True).start()

    def _pick_reader(self):
        for line in self.proc.stdout:
            self.logq.put(line.rstrip())
        self.logq.put(f'== 종료 (코드 {self.proc.wait()}) ==')
        self.logq.put('__ENABLE_RUN__')

    def stop_pick(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            self.append('중지 요청')

    # ---- 표시 갱신 ----
    def append(self, text):
        self.log.config(state=tk.NORMAL)
        self.log.insert(tk.END, text + '\n')
        self.log.see(tk.END)
        self.log.config(state=tk.DISABLED)

    def tick(self):
        with self.cam.lock:
            depth = self.cam.depth
            wrist = self.cam.wrist
        if depth is not None:
            self._photos[0] = bgr_to_photo(depth_to_bgr(depth))
            self.depth_label.config(image=self._photos[0], width=VIEW_W, height=VIEW_H)
        if wrist is not None:
            self._photos[1] = bgr_to_photo(wrist)
            self.wrist_label.config(image=self._photos[1], width=VIEW_W, height=VIEW_H)
        while not self.logq.empty():
            m = self.logq.get()
            if m == '__ENABLE_RUN__':
                self.run_btn.config(state=tk.NORMAL)
            elif m:
                self.append(m)
        self.root.after(100, self.tick)

    def on_close(self):
        self.stop_pick()
        self.root.destroy()


def main():
    rclpy.init()
    cam = CamNode()
    spin = threading.Thread(target=rclpy.spin, args=(cam,), daemon=True)
    spin.start()
    root = tk.Tk()
    App(root, cam)
    root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
