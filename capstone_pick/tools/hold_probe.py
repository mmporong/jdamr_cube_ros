"""게이트 2용 실측: 쥔 상태와 빈손 상태의 손목캠 블롭 면적 차이.

낙하 판정을 각도에서 손목캠으로 바꾸려면 임계값이 필요하다. 각도로는 낙하를
볼 수 없다 — hold_target(접촉각-0.01)을 명령해 두면 물체가 빠져도 그리퍼가 그
각도를 유지한다. 죠를 다시 건드리는 확인은 물체를 밀어내므로 쓸 수 없다.

쥐고 있으면 큐브가 렌즈 앞 몇 cm에 있어 화면을 크게 채우고, 빠지면 큐브가
바닥에 남아 시야에서 사라지거나 작게 보인다. 그 차이를 면적으로 가른다.

사용: python3 ~/capstone_tools/hold_probe.py
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

AJ = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll']
CARRY = [0.0, 0.15, 0.15, 1.28, 0.0]      # 운반 자세 = 낙하 판정을 하는 자세
BLUE = ((100, 130, 100), (135, 255, 255))


def svc(s_, t_, r_):
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


rclpy.init()
n = Node('hold_probe')
b = CvBridge()
st, img = {}, {}
tp = n.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
n.create_subscription(JointState, '/joint_states',
                      lambda m: st.update({k: v for k, v in zip(m.name, m.position)}), 10)
n.create_subscription(Image, '/wrist_camera/image_raw',
                      lambda m: img.__setitem__('w', b.imgmsg_to_cv2(m, 'bgr8')), 1)
cl = ActionClient(n, GripperCommand, '/gripper_controller/gripper_cmd')


def spin(s):
    t0 = time.time()
    while time.time() - t0 < s:
        rclpy.spin_once(n, timeout_sec=0.1)


def move(p, sec=3):
    jt = JointTrajectory()
    jt.joint_names = AJ
    pt = JointTrajectoryPoint()
    pt.positions = [float(v) for v in p]
    pt.time_from_start.sec = sec
    jt.points = [pt]
    tp.publish(jt)


def settle(target, timeout=12):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
        if all(abs(st.get(k, 9.0) - v) < 0.03 for k, v in zip(AJ, target)):
            return True
    return False


def grip(pos, effort=10.0):
    g = GripperCommand.Goal()
    g.command.position = float(pos)
    g.command.max_effort = effort
    f = cl.send_goal_async(g)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    h = f.result()
    if h is not None and h.accepted:
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=15)
    spin(1.0)
    return st.get('arm_gripper')


def blob(frames=7):
    """가장 큰 색 블롭의 면적·중심을 중앙값으로."""
    areas, ys = [], []
    for _ in range(frames):
        img.pop('w', None)
        t0 = time.time()
        while time.time() - t0 < 3 and 'w' not in img:
            rclpy.spin_once(n, timeout_sec=0.1)
        if 'w' not in img:
            continue
        hsv = cv2.cvtColor(img['w'], cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, *BLUE)
        num, _, stats, cents = cv2.connectedComponentsWithStats(mask)
        best = None
        for i in range(1, num):
            a = stats[i, cv2.CC_STAT_AREA]
            if a > 50 and (best is None or a > best[0]):
                best = (a, cents[i][1])
        areas.append(best[0] if best else 0)
        ys.append(best[1] if best else -1)
    if not areas:
        return None
    return float(np.median(areas)), float(np.median(ys))


spin(2)
cl.wait_for_server(timeout_sec=15)
print('운반 자세에서 손목캠 블롭 면적 — 쥔 상태 vs 빈손')
print('상태                     그리퍼각  블롭면적   블롭 y')

# 1) 빈손: 큐브를 멀리 치우고 운반 자세
svc('/world/room/set_pose', 'gz.msgs.Pose',
    'name: "pick_blue" position {x: 2.5 y: 2.5 z: 0.015} orientation {w: 1}')
time.sleep(1.5)
grip(0.5)
move(CARRY, 3)
settle(CARRY)
r = blob()
print(f'빈손(큐브 없음)          {st.get("arm_gripper", 0):+.3f}   '
      f'{r[0] if r else 0:8.0f} {r[1] if r else -1:8.1f}', flush=True)

# 2) 낙하 상황: 큐브가 로봇 앞 바닥에 있고 팔은 운반 자세
svc('/world/room/set_pose', 'gz.msgs.Pose',
    'name: "pick_blue" position {x: 0.68 y: 0 z: 0.015} orientation {w: 1}')
time.sleep(1.5)
r = blob()
print(f'낙하(큐브 바닥에 있음)    {st.get("arm_gripper", 0):+.3f}   '
      f'{r[0] if r else 0:8.0f} {r[1] if r else -1:8.1f}', flush=True)

# 3) 쥔 상태: 큐브를 죠 사이에 두고 닫은 뒤 운반 자세로
print('쥔 상태를 만들려면 실제 파지가 필요하다 — pick을 돌려 로그로 확인할 것')
move([0.0, -0.4, 1.0, 0.2, 0.0], 3)
spin(3)
rclpy.shutdown()
