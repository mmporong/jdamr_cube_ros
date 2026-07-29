#!/usr/bin/env python3
"""캡스톤 픽 대시보드 (콤팩트 v3).

좌: 카메라 2뷰(비전 뎁스, 손목) / 우: 제어 컬럼(3색 집기·무대·수동 조작) / 하: 상태줄+로그.
3색 무대(파랑=받침대, 빨강·초록=바닥)를 깔고 색 버튼을 눌러 그 색만 골라 집는다.
ROS 환경이 소싱된 셸에서: python3 ~/capstone_tools/pick_ui.py
"""
import math
import os
import queue
import signal
import subprocess
import threading
import tkinter as tk
import tkinter.font as tkfont
from tkinter import scrolledtext

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
VIEW_W, VIEW_H = 320, 240
DEPTH_NEAR, DEPTH_FAR = 0.15, 3.0
ARM_JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
              'arm_wrist_flex', 'arm_wrist_roll']
SPAWN_POSE = (0.0, 0.0, 0.03)               # 원본 launch 기본 스폰 위치 (x_pose=0, y_pose=0)
ARM_POSES = {
    # 홈 = 최밀착 접힘 (MoveIt 충돌 0 중 elbow 최대): lift 리밋(-1.745, 어퍼암 숄더 밀착)
    # + elbow 1.55(전완-어퍼암 밀착 최대) + wrist 0.90(손목 몸쪽, 더 접으면 숄더 관통)
    '홈': [0.0, -1.745, 1.55, 0.90, 0.0],
    '접힘': [0.0, -0.4, 1.0, 0.2, 0.0],     # 주행 자세 (pick_node와 동일)
    '상공': [0.0, 0.15, 0.2, 0.9, 0.0],
    '파지': [0.0, 0.48, 0.2, 0.9, 0.0],
    '바닥': [0.0, 1.20, 0.15, 0.23, 0.0],
}
DRIVE_LIN, DRIVE_ANG = 0.20, 0.80
PICK_COLORS = (('파랑', 'blue', '#2244cc'), ('빨강', 'red', '#cc2222'), ('초록', 'green', '#22aa33'))


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
            if n in ('gripper', 'arm_gripper'):
                with self.lock:
                    self.grip_angle = p

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
        self.grip_client.send_goal_async(g)


def depth_to_bgr(d):
    v = np.nan_to_num(np.asarray(d, np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    valid = v > 0.05
    v = np.clip(v, DEPTH_NEAR, DEPTH_FAR)
    n = ((DEPTH_FAR - v) / (DEPTH_FAR - DEPTH_NEAR) * 255).astype(np.uint8)
    bgr = cv2.applyColorMap(n, cv2.COLORMAP_JET)
    bgr[~valid] = 0
    return bgr


def bgr_to_photo(bgr):
    bgr = cv2.resize(bgr, (VIEW_W, VIEW_H))
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])
    header = f'P6\n{VIEW_W} {VIEW_H}\n255\n'.encode()
    return tk.PhotoImage(data=header + rgb.tobytes())


# 검출 시각화용 HSV 범위 (pick_node의 TARGET_COLOR_RANGES와 동일 값)
UI_RANGES = {
    'blue': ([((100, 130, 100), (135, 255, 255))], (220, 120, 40)),
    'red': ([((0, 150, 100), (6, 255, 255)), ((174, 150, 100), (179, 255, 255))], (40, 40, 230)),
    'green': ([((45, 80, 80), (75, 255, 255))], (40, 190, 40)),
}


def overlay_detections(bgr):
    """RGB 화면에 3색 인식 박스를 그려 무엇이 검출되는지 보여준다."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    out = bgr.copy()
    for name, (ranges, box) in UI_RANGES.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        num, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        for i in range(1, num):
            if stats[i, cv2.CC_STAT_AREA] < 60:
                continue
            x, y, w, h = (stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
                          stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
            cv2.rectangle(out, (x, y), (x + w, y + h), box, 2)
            cv2.putText(out, name, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box, 1)
    return out


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
        self.driving = None
        # WSLg 타이틀바는 한글 인코딩이 깨지므로 ASCII 제목 사용
        root.title('JDAMR Pick Dashboard')
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        main = tk.Frame(root)
        main.pack(side=tk.TOP, fill=tk.BOTH, padx=6, pady=4)

        # 2×2 격자: [RGB+인식박스][뎁스] / [손목][제어]
        cams = tk.Frame(main)
        cams.grid(row=0, column=0, sticky='n')
        self.rgb_label = self._cam_panel(cams, '비전 RGB (인식 박스)', 0, 0)
        self.depth_label = self._cam_panel(cams, '비전 뎁스', 0, 1)
        self.wrist_label = self._cam_panel(cams, '손목 카메라', 1, 0)

        # 제어 (격자 우하단)
        ctrl = tk.Frame(cams)
        ctrl.grid(row=1, column=1, sticky='n', padx=(8, 0))

        pickf = tk.LabelFrame(ctrl, text='집기 — 색을 눌러 선택')
        pickf.pack(fill=tk.X, pady=2)
        row = tk.Frame(pickf)
        row.pack(pady=2)
        for label, color, hexc in PICK_COLORS:
            tk.Button(row, text=label, width=6, bg=hexc, fg='white',
                      activebackground=hexc,
                      command=lambda c=color: self.run_pick(c)).pack(side=tk.LEFT, padx=2)
        opt = tk.Frame(pickf)
        opt.pack(pady=2)
        tk.Label(opt, text='속도').pack(side=tk.LEFT)
        self.speed = tk.StringVar(value='4.0')
        tk.Entry(opt, textvariable=self.speed, width=4).pack(side=tk.LEFT, padx=(0, 6))
        tk.Button(opt, text='중지', bg='#e8cfcf', command=self.stop_pick).pack(side=tk.LEFT)

        stagef = tk.LabelFrame(ctrl, text='무대')
        stagef.pack(fill=tk.X, pady=2)
        row = tk.Frame(stagef)
        row.pack(pady=2)
        tk.Button(row, text='3색 배치', command=lambda: self.run_stage('tricolor_stage.py')).pack(side=tk.LEFT, padx=2)

        manf = tk.LabelFrame(ctrl, text='수동 조작')
        manf.pack(fill=tk.X, pady=2)
        pad = tk.Frame(manf)
        pad.pack(side=tk.LEFT, padx=4, pady=2)
        self._drive_btn(pad, '▲', 0, 1, DRIVE_LIN, 0.0)
        self._drive_btn(pad, '◀', 1, 0, 0.0, DRIVE_ANG)
        tk.Button(pad, text='■', width=2, command=self.stop_drive).grid(row=1, column=1)
        self._drive_btn(pad, '▶', 1, 2, 0.0, -DRIVE_ANG)
        self._drive_btn(pad, '▼', 2, 1, -DRIVE_LIN, 0.0)
        armf = tk.Frame(manf)
        armf.pack(side=tk.LEFT, padx=4)
        r1 = tk.Frame(armf)
        r1.pack()
        for name in ('홈', '접힘', '상공'):
            tk.Button(r1, text=name, width=4,
                      command=lambda n=name: self.arm_pose(n)).pack(side=tk.LEFT, padx=1, pady=1)
        r2 = tk.Frame(armf)
        r2.pack()
        for name in ('파지', '바닥'):
            tk.Button(r2, text=name, width=4,
                      command=lambda n=name: self.arm_pose(n)).pack(side=tk.LEFT, padx=1, pady=1)
        tk.Button(r2, text='열기', width=4, command=lambda: self.gripper(0.8)).pack(side=tk.LEFT, padx=1)
        tk.Button(r2, text='닫기', width=4, command=lambda: self.gripper(-0.17)).pack(side=tk.LEFT, padx=1)

        self.status = tk.Label(root, text='상태 수신 대기…', anchor='w', fg='#004080')
        self.status.pack(side=tk.TOP, fill=tk.X, padx=8)
        self.log = scrolledtext.ScrolledText(root, height=6, state=tk.DISABLED)
        self.log.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=4)

        self._photos = [None, None, None]
        self.tick()

    def _cam_panel(self, parent, title, row, col):
        f = tk.Frame(parent)
        f.grid(row=row, column=col, padx=3, pady=2)
        tk.Label(f, text=title).pack()
        # width/height는 이미지 없는 Label에선 문자 단위로 해석돼 창이 거대해진다
        # — 픽셀 크기 자리표시 이미지를 깔아 처음부터 정확한 크기로 고정
        ph = tk.PhotoImage(width=VIEW_W, height=VIEW_H)
        lbl = tk.Label(f, image=ph, bg='#222')
        lbl._ph = ph  # GC 방지
        lbl.pack()
        return lbl

    def _drive_btn(self, parent, text, r, c, vx, wz):
        b = tk.Button(parent, text=text, width=2)
        b.grid(row=r, column=c, padx=1, pady=1)
        b.bind('<ButtonPress-1>', lambda e: self.start_drive(vx, wz))
        b.bind('<ButtonRelease-1>', lambda e: self.stop_drive())

    # ---- 주행/팔/그리퍼 ----
    def start_drive(self, vx, wz):
        self.driving = (vx, wz)

    def stop_drive(self):
        self.driving = None
        self.node.drive(0.0, 0.0)

    def arm_pose(self, name):
        if name == '홈':          # 홈 = 완전 초기화 (초기 위치 복귀 + 스폰 접힘 + 그리퍼 0)
            self.reset_robot()
            return
        self.append(f'팔 자세: {name}')
        self.node.arm_pose(ARM_POSES[name])

    def gripper(self, pos):
        self.append(f'그리퍼 → {pos}')
        self.node.gripper(pos)

    # ---- 무대/집기 ----
    def run_stage(self, script, *args):
        self.append(f'== 무대: {script} {" ".join(args)} ==')
        threading.Thread(target=self._stage_worker, args=(script, args), daemon=True).start()

    def _stage_worker(self, script, args=()):
        r = subprocess.run(['python3', os.path.join(TOOLS, script), *args],
                           capture_output=True, text=True)
        self.logq.put((r.stdout.strip() or r.stderr.strip())[-300:])

    def run_pick(self, color):
        if self.proc and self.proc.poll() is None:
            self.append('이미 실행 중 — 먼저 중지하세요')
            return
        try:
            # 정수 입력(예: 20)은 ROS 파라미터가 int로 파싱돼 double 선언과 충돌 — 실수로 정규화
            spd = max(0.5, min(10.0, float(self.speed.get())))
        except ValueError:
            self.append(f'속도 값이 숫자가 아님: {self.speed.get()!r}')
            return
        self.speed.set(f'{spd:g}')
        cmd = ['ros2', 'run', 'capstone_pick', 'pick', '--ros-args',
               '-p', f'speed_scale:={spd:.2f}',
               '-p', f'target_color:={color}']
        self.append(f'== {color} 집기 시작 ==')
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True,
                                     start_new_session=True)
        threading.Thread(target=self._pick_reader, daemon=True).start()

    def _pick_reader(self):
        for line in self.proc.stdout:
            self.logq.put(line.rstrip())
        self.logq.put(f'== 종료 (코드 {self.proc.wait()}) ==')

    def stop_pick(self):
        if self.proc and self.proc.poll() is None:
            p = self.proc
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
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
        self.append('== 홈: 픽 중지·초기 위치 복귀·스폰 접힘·그리퍼 0 ==')
        self.stop_pick()
        self.stop_drive()
        self.node.arm_pose(ARM_POSES['홈'])
        self.node.gripper(0.0)
        threading.Thread(target=self._reset_worker, daemon=True).start()

    def _reset_worker(self):
        x, y, z = SPAWN_POSE
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/room/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req',
             f'name: "jdamr_cube" position {{x: {x} y: {y} z: {z}}} orientation {{w: 1}}'],
            capture_output=True, text=True)
        self.logq.put('초기 위치 복귀: ' + (r.stdout.strip() or r.stderr.strip()[-100:]))

    # ---- 표시 ----
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
        for i, (lbl, img) in enumerate(((self.rgb_label, None if rgb is None else overlay_detections(rgb)),
                                        (self.depth_label, None if depth is None else depth_to_bgr(depth)),
                                        (self.wrist_label, wrist))):
            if img is not None:
                self._photos[i] = bgr_to_photo(img)
                lbl.config(image=self._photos[i])
        parts = []
        if oxy:
            parts.append(f'위치 x={oxy[0]:.2f} y={oxy[1]:.2f} yaw={oxy[2]:.0f}°')
        if ga is not None:
            parts.append(f'그리퍼 {ga:.3f}')
        parts.append('픽 ' + ('실행 중' if self.proc and self.proc.poll() is None else '대기'))
        self.status.config(text='   |   '.join(parts))
        while not self.logq.empty():
            m = self.logq.get()
            if m:
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
