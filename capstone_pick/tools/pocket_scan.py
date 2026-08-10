"""파지 포켓·손목캠 기준점 동시 실측.

세 상수가 서로 맞물려 있는데 각각 다른 날 다른 자세에서 재는 바람에 어긋났다.
    POCKET_FLOOR  접근 주행이 큐브를 데려다 놓을 목표 x [m, base_footprint]
    WRIST_REF     그때 상공(PRE_FLOOR)에서 보이는 블롭 위치 [px]
    GRASP_REF     그때 하강(GRASP_FLOOR)에서 보이는 블롭 위치 [px]
셋을 한 번에, 같은 자세족에서 잰다. 그래야 "상공 정렬 → 하강 → 파지"가 보정 없이
이어진다 — 중간에 후퇴량을 따로 얹는 것은 셋이 어긋났다는 증거였다.

차체는 원점에 고정하고 큐브만 옮긴다. 팔은 명목 자세(리치 0)로만 움직인다.

사용 (ROS 환경 source 후): python3 pocket_scan.py
"""
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

AJ = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
      'arm_wrist_flex', 'arm_wrist_roll']
POSE_FOLDED = [0.0, -0.4, 1.0, 0.2, 0.0]
POSE_PRE = [0.0, 0.85, 0.15, 0.58, 0.0]
POSE_GRASP = [0.0, 1.15, 0.15, 0.28, 0.0]
GRIP_OPEN, GRIP_CLOSE = 1.2, -0.17
# 4cm 큐브를 제대로 물면 죠가 그만큼 벌어진 채 멈춘다. 첫 스윕 실측에서 실제로
# 들어올려진 경우가 0.400/0.458/0.460이었다 — 3cm 시절 상한 0.30으로는 진짜
# 파지가 전부 '얕은 걸침'으로 버려진다.
HOLD_MIN, HOLD_MAX = 0.05, 0.55
GREEN = ((35, 80, 60), (85, 255, 255))       # HSV
XS = [0.365, 0.375, 0.385, 0.395, 0.405, 0.415, 0.425]


def svc(req):
    subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '4000', '--req', req], capture_output=True, text=True)


def cube_z():
    out = subprocess.run(['gz', 'model', '-m', 'pick_object_green', '-p'],
                         capture_output=True, text=True, timeout=20).stdout
    L = out.splitlines()
    for i, l in enumerate(L):
        if '- Pose' in l and i + 1 < len(L):
            try:
                return [float(v) for v in L[i + 1].strip().strip('[]').split()][2]
            except ValueError:
                continue
    return None


class Scanner(Node):
    def __init__(self):
        super().__init__('pocket_scan')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.angle = None
        self.wrist = None
        self.create_subscription(JointState, '/joint_states', self._joint, 10)
        self.create_subscription(Image, '/wrist_camera/image_raw', self._img, 10)
        self.grip.wait_for_server(timeout_sec=20)

    def _joint(self, m):
        # 바퀴만 실린 메시지가 섞여 오므로(퍼블리셔 2개) 있을 때만 갱신한다
        if 'arm_gripper' in m.name:
            self.angle = m.position[m.name.index('arm_gripper')]

    def _img(self, m):
        self.wrist = self.bridge.imgmsg_to_cv2(m, 'bgr8')

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def arm(self, pos, sec=3.0):
        jt = JointTrajectory()
        jt.joint_names = AJ
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in pos]
        pt.time_from_start.sec = int(sec)
        jt.points = [pt]
        self.pub.publish(jt)
        self.spin(sec + 1.5)

    def gripper(self, pos, effort=7.5):
        g = GripperCommand.Goal()
        g.command.position = float(pos)
        g.command.max_effort = effort
        self.grip.send_goal_async(g)
        self.spin(2.0)

    def blob(self, frames=5):
        """손목캠에서 가장 큰 초록 블롭의 중심(중앙값). 못 보면 None."""
        pts = []
        for _ in range(frames):
            self.wrist = None
            t0 = time.time()
            while self.wrist is None and time.time() - t0 < 2.0:
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.wrist is None:
                continue
            hsv = cv2.cvtColor(self.wrist, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array(GREEN[0]), np.array(GREEN[1]))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < 200:
                continue
            m = cv2.moments(c)
            if m['m00'] == 0:
                continue
            pts.append((m['m10'] / m['m00'], m['m01'] / m['m00'], cv2.contourArea(c)))
        if not pts:
            return None
        a = np.array(pts)
        return float(np.median(a[:, 0])), float(np.median(a[:, 1])), float(np.median(a[:, 2]))


def fmt(b):
    return '  미검출  ' if b is None else f'({b[0]:5.0f},{b[1]:4.0f})'


def main():
    rclpy.init()
    n = Scanner()
    n.spin(2.0)
    print(f'{"큐브 x":>7} | {"상공 blob":^13} | {"하강 blob":^13} | {"물림각":>7} {"들림":>4}  판정')
    print('-' * 74)
    rows = []
    for x in XS:
        n.gripper(GRIP_OPEN)      # 쥔 채 접으면 다음 배치가 큐브를 튕긴다
        n.arm(POSE_FOLDED, 2.5)
        svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
        svc(f'name: "pick_object_green", position: {{x: {x}, y: 0.0, z: 0.02}}')
        n.spin(1.5)
        n.gripper(GRIP_OPEN)
        n.arm(POSE_PRE)
        n.spin(0.5)
        hov = n.blob()
        n.arm(POSE_GRASP)          # 명목 하강 — 리치 보정 없음
        n.spin(0.5)
        des = n.blob()
        n.gripper(GRIP_CLOSE)
        n.spin(1.5)
        self_t0 = time.time()
        while n.angle is None and time.time() - self_t0 < 5.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        ang = n.angle
        n.arm([0.0, 0.5, 0.4, 0.6, 0.0], 2.5)     # 들어올려 실제로 물었는지 확인
        n.spin(1.0)
        z = cube_z()
        held = ang is not None and HOLD_MIN < ang < HOLD_MAX
        lifted = z is not None and z > 0.05
        mark = '성공' if (held and lifted) else ('물림만' if held else '실패')
        print(f'{x:7.3f} | {fmt(hov):^13} | {fmt(des):^13} | '
              f'{"  n/a" if ang is None else f"{ang:7.3f}"} {"O" if lifted else "X":>4}  {mark}')
        rows.append((x, hov, des, held and lifted))
    print()
    good = [r for r in rows if r[3]]
    if not good:
        print('성립 구간 없음 — 파지 자세(POSE_GRASP) 자체를 재검토해야 한다')
    else:
        c = good[len(good) // 2]
        print(f'파지 성립: {" ".join(f"{r[0]:.3f}" for r in good)}  (중앙 {c[0]:.3f})')
        print(f'  POCKET_FLOOR = ({c[0]:.3f}, 0.000)')
        if c[1]:
            print(f'  WRIST_REF    = ({c[1][0]:.1f}, {c[1][1]:.1f})')
        if c[2]:
            print(f'  GRASP_REF    = ({c[2][0]:.1f}, {c[2][1]:.1f})')
        # px/m 게인도 같은 스윕에서 뽑는다 — 인접 두 점의 blob x 차 / 거리 차
        for lbl, idx in (('WRIST_PX_PER_M', 1), ('GRASP_PX_PER_M', 2)):
            pts = [(r[0], r[idx][0]) for r in rows if r[idx]]
            if len(pts) >= 2:
                xs = np.array([p[0] for p in pts]); ys = np.array([p[1] for p in pts])
                print(f'  {lbl} = {np.polyfit(xs, ys, 1)[0]:.0f}   ({len(pts)}점 회귀)')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
