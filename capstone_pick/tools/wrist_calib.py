"""손목캠 캘리브레이션 실측 — FOV 변경 시 재산출용.

측정 항목 (pick_node.py의 상수와 1:1 대응):
  WRIST_REF        정렬 완료 상태(포켓 정위치·pan 0)의 blob 중심 픽셀
  WRIST_PX_PER_M   전후 1m당 픽셀 — 두 포켓 거리(0.361/0.401)의 blob x 차이로 산출
  WRIST_PY_PER_PAN pan 1rad당 픽셀 — pan ±0.05rad에서의 blob y 차이로 산출

사용 (ROS 환경): python3 wrist_calib.py
"""
import math
import re
import subprocess
import time

import cv2
import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

AJ = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll']
PRE_FLOOR = [0.0, 0.85, 0.15, 0.58, 0.0]     # 호버(정렬) 자세 — 정렬은 여기서 한다
BLUE = ((100, 130, 100), (135, 255, 255))
BASE_X = 0.3
POSE_RE = re.compile(r'\[([-\d.eE+ ]+)\]')


def svc(s_, t_, r_):
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


def spawn_cube(pocket_x, y=0.0):
    """포켓 거리(로봇 기준 전방 x)에 큐브 배치."""
    svc('/world/room/remove', 'gz.msgs.Entity', 'name: "pick_blue" type: MODEL')
    for _ in range(10):
        out = subprocess.run('gz model --list', shell=True, capture_output=True, text=True).stdout
        if 'pick_blue' not in out:
            break
        time.sleep(0.4)
    c = ('<sdf version="1.6"><model name="pick_blue">'
         f'<pose>{BASE_X + pocket_x} {y} 0.015 0 0 0</pose>'
         '<link name="link"><inertial><mass>0.04</mass>'
         '<inertia><ixx>4e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>4e-6</iyy><iyz>0</iyz><izz>4e-6</izz></inertia></inertial>'
         '<collision name="c"><geometry><box><size>0.03 0.03 0.03</size></box></geometry></collision>'
         '<visual name="v"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
         '<material><ambient>0.1 0.2 0.9 1</ambient><diffuse>0.1 0.2 0.9 1</diffuse></material>'
         '</visual></link></model></sdf>')
    r = svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + c.replace('"', '\\"') + '"')
    if 'true' not in r:
        time.sleep(1)
        svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + c.replace('"', '\\"') + '"')
    time.sleep(1.5)


class Calib(Node):
    def __init__(self):
        super().__init__('wrist_calib')
        self.st = {}
        self.img = None
        self.bridge = CvBridge()
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.st.update(dict(zip(m.name, m.position))), 10)
        self.create_subscription(Image, '/wrist_camera/image_raw',
                                 lambda m: setattr(self, 'img', self.bridge.imgmsg_to_cv2(m, 'bgr8')), 1)
        self.traj = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move(self, pos, sec=2.5):
        jt = JointTrajectory()
        jt.joint_names = AJ
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in pos]
        pt.time_from_start.sec = int(sec)
        pt.time_from_start.nanosec = int((sec % 1) * 1e9)
        jt.points = [pt]
        self.traj.publish(jt)

    def settle(self, target, tol=0.03, timeout=8):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(abs(self.st.get(k, 9.0) - v) < tol for k, v in zip(AJ, target)):
                return True
        return False

    def gripper(self, pos):
        g = GripperCommand.Goal()
        g.command.position = float(pos)
        g.command.max_effort = 10.0
        self.grip.send_goal_async(g)
        rclpy.spin_once(self, timeout_sec=0.02)

    def blob(self, frames=7):
        """가장 큰 파랑 블롭 중심의 중앙값 (x, y, area)."""
        xs, ys, areas = [], [], []
        for _ in range(frames):
            self.img = None
            t0 = time.time()
            while time.time() - t0 < 3 and self.img is None:
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.img is None:
                continue
            hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, *BLUE)
            num, _, stats, cents = cv2.connectedComponentsWithStats(mask)
            best = None
            for i in range(1, num):
                a = stats[i, cv2.CC_STAT_AREA]
                if a > 100 and (best is None or a > best[2]):
                    best = (cents[i][0], cents[i][1], a)
            if best:
                xs.append(best[0]); ys.append(best[1]); areas.append(best[2])
        if len(xs) < 3:
            return None
        return float(np.median(xs)), float(np.median(ys)), float(np.median(areas))


def main():
    rclpy.init()
    n = Calib()
    n.spin(2)
    n.grip.wait_for_server(timeout_sec=15)
    svc('/world/room/set_pose', 'gz.msgs.Pose',
        f'name: "jdamr_cube" position {{x: {BASE_X} y: 0 z: 0.03}} orientation {{w: 1}}')
    n.gripper(1.2)
    time.sleep(1)

    print('=== 손목캠 캘리브레이션 실측 ===')

    # 1) WRIST_REF — 포켓 정위치(0.381)·pan 0
    spawn_cube(0.381)
    n.move(PRE_FLOOR, 3)
    n.settle(PRE_FLOOR)
    n.spin(1)
    ref = n.blob()
    if not ref:
        print('블롭 미검출 — 포켓 0.381에서 큐브가 안 보인다')
        rclpy.shutdown()
        return
    print(f'WRIST_REF = ({ref[0]:.1f}, {ref[1]:.1f})   [면적 {ref[2]:.0f}]')

    # 2) WRIST_PX_PER_M — 포켓 0.361 / 0.401 두 점
    pts = {}
    for pocket in (0.361, 0.401):
        spawn_cube(pocket)
        n.spin(1.5)
        b = n.blob()
        pts[pocket] = b
        print(f'  포켓 {pocket}: blob=({b[0]:.1f}, {b[1]:.1f}) 면적 {b[2]:.0f}' if b
              else f'  포켓 {pocket}: 미검출')
    if pts.get(0.361) and pts.get(0.401):
        dx = pts[0.361][0] - pts[0.401][0]     # 가까울수록 x가 크다(+)
        px_per_m = dx / 0.04
        print(f'WRIST_PX_PER_M = {px_per_m:.0f}   (dx {dx:.1f}px / 0.04m)')

    # 3) WRIST_PY_PER_PAN — pan ±0.05rad
    spawn_cube(0.381)
    ys = {}
    for pan in (-0.05, 0.05):
        pose = list(PRE_FLOOR)
        pose[0] = pan
        n.move(pose, 2)
        n.settle(pose)
        n.spin(1)
        b = n.blob()
        ys[pan] = b
        print(f'  pan {pan:+.2f}: blob=({b[0]:.1f}, {b[1]:.1f})' if b else f'  pan {pan:+.2f}: 미검출')
    if ys.get(-0.05) and ys.get(0.05):
        dy = ys[0.05][1] - ys[-0.05][1]
        py_per_pan = abs(dy / 0.1)
        print(f'WRIST_PY_PER_PAN = {py_per_pan:.0f}   (dy {dy:.1f}px / 0.1rad)')

    n.move([0.0, -0.4, 1.0, 0.2, 0.0], 2)
    n.spin(2)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
