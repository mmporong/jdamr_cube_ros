#!/usr/bin/env python3
"""캡스톤 픽 대시보드.

카메라 3뷰(비전 RGB·뎁스, 손목) + 주행 패드 + 팔 자세 + 그리퍼 + 무대/픽 실행 + 로그.
ROS 환경이 소싱된 셸에서 실행:
    python3 ~/capstone_tools/pick_ui.py
"""
import math
import os
import queue
import signal
import subprocess
import threading
import tkinter as tk
import tkinter.font as tkfont
from tkinter import scrolledtext, ttk

import cv2
import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TOOLS = os.path.expanduser('~/capstone_tools')
VIEW_W, VIEW_H = 340, 255
DEPTH_NEAR, DEPTH_FAR = 0.15, 3.0
ARM_JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
              'arm_wrist_flex', 'arm_wrist_roll']
ARM_POSES = {
    '홈': [0.0, 0.0, 0.0, 0.0, 0.0],       # 원본 리포 SRDF 'home'과 동일 (리셋 기준 자세)
    '접힘': [0.0, -0.4, 1.0, 0.2, 0.0],     # 주행 자세 (pick_node와 동일)
    '상공': [0.0, 0.15, 0.2, 0.9, 0.0],
    '파지': [0.0, 0.48, 0.2, 0.9, 0.0],
    '바닥파지': [0.0, 1.20, 0.15, 0.23, 0.0],
}
DRIVE_LIN, DRIVE_ANG = 0.20, 0.80  # 주행 패드 속도


class UiNode(Node):
    """카메라·상태 구독 + 주행/팔/그리퍼 명령 발행."""

    def __init__(self):
        super().__init__('pick_dashboard')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.rgb = self.depth = self.wrist = None
        self.odom_xy_yaw = None
        self.grip_angle = None
        self.create_subscription(Image, '/rgbd_camera/image', self._rgb_cb, 1)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self._depth_cb, 1)
        self.create_subscription(Image, '/wrist_camera/image_raw', self._wrist_cb, 1)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')

    def _rgb_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.rgb = img

    def _depth_cb(self, msg):
        d = self.bridge.imgmsg_to_cv2(msg)
        with self.lock:
            self.depth = d

    def _wrist_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.wrist = img

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.odom_xy_yaw = (p.x, p.y, math.degrees(yaw))

    def _joint_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n == 'gripper':
                with self.lock:
                    self.grip_angle = p

    # ---- 명령 ----
    def drive(self, vx, wz):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self.cmd_pub.publish(t)

    def arm_pose(self, positions, sec=2.5):
        jt = JointTrajectory()
        jt.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start.sec = int(sec)
        pt.time_from_start.nanosec = int((sec % 1) * 1e9)
        jt.points = [pt]
        self.traj_pub.publish(jt)

    def gripper(self, position):
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = 10.0
        self.grip_client.send_goal_async(g)  # 결과 대기 안 함 (UI 논블로킹)


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


def set_korean_font(root):
    fams = set(tkfont.families(root))
    for name in ('Malgun Gothic', 'NanumGothic', 'Noto Sans CJK KR'):
        if name in fams:
            for fn in ('TkDefaultFont', 'TkTextFont', 'TkHeadingFont',
                       'TkMenuFont', 'TkFixedFont'):
                tkfont.nametofont(fn).configure(family=name, size=10)
            root.option_add('*Font', (name, 10))
            return name
    return None


class App:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.proc = None
        self.logq = queue.Queue()
        self.driving = None  # (vx, wz) 누르는 동안 유지
        root.title('캡스톤 픽 대시보드')
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        # 카메라 3뷰
        cams = tk.Frame(root)
        cams.pack(side=tk.TOP, padx=6, pady=4)
        self.rgb_label = self._cam_panel(cams, '비전 카메라 RGB', 0)
        self.depth_label = self._cam_panel(cams, '비전 카메라 뎁스', 1)
        self.wrist_label = self._cam_panel(cams, '손목 카메라', 2)

        # 상태 표시줄
        self.status = tk.Label(root, text='상태 수신 대기…', anchor='w', fg='#004080')
        self.status.pack(side=tk.TOP, fill=tk.X, padx=8)

        body = tk.Frame(root)
        body.pack(side=tk.TOP, fill=tk.X, padx=6, pady=2)

        # 주행 패드 (누르는 동안 주행)
        pad = tk.LabelFrame(body, text='주행 (누르는 동안)')
        pad.pack(side=tk.LEFT, padx=4)
        self._drive_btn(pad, '▲', 0, 1, DRIVE_LIN, 0.0)
        self._drive_btn(pad, '◀', 1, 0, 0.0, DRIVE_ANG)
        tk.Button(pad, text='■', width=3, command=self.stop_drive).grid(row=1, column=1)
        self._drive_btn(pad, '▶', 1, 2, 0.0, -DRIVE_ANG)
        self._drive_btn(pad, '▼', 2, 1, -DRIVE_LIN, 0.0)

        # 팔 자세
        arm = tk.LabelFrame(body, text='팔 자세')
        arm.pack(side=tk.LEFT, padx=4)
        for name in ARM_POSES:
            tk.Button(arm, text=name, width=5,
                      command=lambda n=name: self.arm_pose(n)).pack(side=tk.LEFT, padx=1, pady=2)

        # 그리퍼
        grip = tk.LabelFrame(body, text='그리퍼')
        grip.pack(side=tk.LEFT, padx=4)
        tk.Button(grip, text='열기', width=5, command=lambda: self.gripper(0.8)).pack(side=tk.LEFT, padx=1, pady=2)
        tk.Button(grip, text='닫기', width=5, command=lambda: self.gripper(-0.17)).pack(side=tk.LEFT, padx=1, pady=2)

        # 무대
        stage = tk.LabelFrame(body, text='무대')
        stage.pack(side=tk.LEFT, padx=4)
        tk.Button(stage, text='파란 타깃', command=lambda: self.run_stage('grip_stage.py')).pack(side=tk.LEFT, padx=1, pady=2)
        tk.Button(stage, text='빨간 타깃', command=lambda: self.run_stage('red_stage.py')).pack(side=tk.LEFT, padx=1, pady=2)
        tk.Button(stage, text='바닥 타깃', command=lambda: self.run_stage('floor_stage.py', '-0.3')).pack(side=tk.LEFT, padx=1, pady=2)
        tk.Button(stage, text='리셋', bg='#f0e0c0', command=self.reset_robot).pack(side=tk.LEFT, padx=1, pady=2)

        # 픽 실행
        pick = tk.LabelFrame(body, text='자율 픽')
        pick.pack(side=tk.LEFT, padx=4)
        tk.Label(pick, text='색').pack(side=tk.LEFT)
        self.color = tk.StringVar(value='blue')
        ttk.Combobox(pick, textvariable=self.color, width=7, state='readonly',
                     values=('blue', 'red', 'green', 'orange', 'pink')).pack(side=tk.LEFT)
        tk.Label(pick, text=' 속도').pack(side=tk.LEFT)
        self.speed = tk.StringVar(value='4.0')
        tk.Entry(pick, textvariable=self.speed, width=4).pack(side=tk.LEFT)
        self.grip_only = tk.BooleanVar(value=False)
        tk.Checkbutton(pick, text='그립만', variable=self.grip_only).pack(side=tk.LEFT)
        self.run_btn = tk.Button(pick, text='실행', bg='#cfe8cf', command=self.run_pick)
        self.run_btn.pack(side=tk.LEFT, padx=3, pady=2)
        tk.Button(pick, text='중지', bg='#e8cfcf', command=self.stop_pick).pack(side=tk.LEFT, pady=2)

        self.log = scrolledtext.ScrolledText(root, height=9, state=tk.DISABLED)
        self.log.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=4)

        self._photos = [None, None, None]
        self.tick()

    def _cam_panel(self, parent, title, col):
        f = tk.Frame(parent)
        f.grid(row=0, column=col, padx=3)
        tk.Label(f, text=title).pack()
        lbl = tk.Label(f, width=VIEW_W, height=VIEW_H, bg='#222')
        lbl.pack()
        return lbl

    def _drive_btn(self, parent, text, r, c, vx, wz):
        b = tk.Button(parent, text=text, width=3)
        b.grid(row=r, column=c, padx=1, pady=1)
        b.bind('<ButtonPress-1>', lambda e: self.start_drive(vx, wz))
        b.bind('<ButtonRelease-1>', lambda e: self.stop_drive())

    # ---- 주행 ----
    def start_drive(self, vx, wz):
        self.driving = (vx, wz)

    def stop_drive(self):
        self.driving = None
        self.node.drive(0.0, 0.0)

    # ---- 팔·그리퍼 ----
    def arm_pose(self, name):
        self.append(f'팔 자세: {name}')
        self.node.arm_pose(ARM_POSES[name])

    def gripper(self, pos):
        self.append(f'그리퍼 → {pos}')
        self.node.gripper(pos)

    # ---- 무대·픽 ----
    def run_stage(self, script, *args):
        self.append(f'== 무대: {script} {" ".join(args)} ==')
        threading.Thread(target=self._stage_worker, args=(script, args), daemon=True).start()

    def _stage_worker(self, script, args=()):
        r = subprocess.run(['python3', os.path.join(TOOLS, script), *args],
                           capture_output=True, text=True)
        self.logq.put((r.stdout.strip() or r.stderr.strip())[-300:])

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
                                     stderr=subprocess.STDOUT, text=True,
                                     start_new_session=True)  # 프로세스 그룹으로 분리 (중지 시 자식까지)
        self.run_btn.config(state=tk.DISABLED)
        threading.Thread(target=self._pick_reader, daemon=True).start()

    def _pick_reader(self):
        for line in self.proc.stdout:
            self.logq.put(line.rstrip())
        self.logq.put(f'== 종료 (코드 {self.proc.wait()}) ==')
        self.logq.put('__ENABLE_RUN__')

    def stop_pick(self):
        if self.proc and self.proc.poll() is None:
            p = self.proc
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)  # ros2 run의 자식 노드까지 정리
            except ProcessLookupError:
                pass
            self.append('중지 요청')

            def force():
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            threading.Thread(target=force, daemon=True).start()
        self.node.drive(0.0, 0.0)

    def reset_robot(self):
        self.append('== 리셋: 픽 중지·팔 홈(원본 자세)·위치 복귀 ==')
        self.stop_pick()
        self.stop_drive()
        self.node.arm_pose(ARM_POSES['홈'])
        self.node.gripper(0.0)
        threading.Thread(target=self._reset_worker, daemon=True).start()

    def _reset_worker(self):
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/room/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req',
             'name: "jdamr_cube" position {x: 0.3 y: 0 z: 0.03} orientation {w: 1}'],
            capture_output=True, text=True)
        self.logq.put('로봇 위치 리셋: ' + (r.stdout.strip() or r.stderr.strip()[-100:]))

    # ---- 표시 갱신 ----
    def append(self, text):
        self.log.config(state=tk.NORMAL)
        self.log.insert(tk.END, text + '\n')
        self.log.see(tk.END)
        self.log.config(state=tk.DISABLED)

    def tick(self):
        if self.driving:
            self.node.drive(*self.driving)
        with self.node.lock:
            rgb, depth, wrist = self.node.rgb, self.node.depth, self.node.wrist
            oxy = self.node.odom_xy_yaw
            ga = self.node.grip_angle
        for i, (lbl, img) in enumerate(((self.rgb_label, rgb),
                                        (self.depth_label, None if depth is None else depth_to_bgr(depth)),
                                        (self.wrist_label, wrist))):
            if img is not None:
                self._photos[i] = bgr_to_photo(img)
                lbl.config(image=self._photos[i], width=VIEW_W, height=VIEW_H)
        parts = []
        if oxy:
            parts.append(f'위치 x={oxy[0]:.2f} y={oxy[1]:.2f} yaw={oxy[2]:.0f}°')
        if ga is not None:
            parts.append(f'그리퍼 각도={ga:.3f}')
        parts.append('픽: ' + ('실행 중' if self.proc and self.proc.poll() is None else '대기'))
        self.status.config(text='   |   '.join(parts))
        while not self.logq.empty():
            m = self.logq.get()
            if m == '__ENABLE_RUN__':
                self.run_btn.config(state=tk.NORMAL)
            elif m:
                self.append(m)
        self.root.after(100, self.tick)

    def on_close(self):
        self.stop_pick()
        self.stop_drive()
        self.root.destroy()


def main():
    rclpy.init()
    node = UiNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    root = tk.Tk()
    set_korean_font(root)
    App(root, node)
    root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
