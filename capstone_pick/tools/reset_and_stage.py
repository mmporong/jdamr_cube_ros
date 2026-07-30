"""팔을 먼저 접고(그리퍼 열기) 무대를 배치한다.

팔이 뻗은 상태로 무대를 깔면, 다음 실행의 접힘 동작이 물체를 쳐서 밀어낸다.
사용: python3 reset_and_stage.py [stage_script] [robot_x]
"""
import subprocess
import sys
import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

STAGE = sys.argv[1] if len(sys.argv) > 1 else 'floor_stage.py'
ROBOT_X = sys.argv[2] if len(sys.argv) > 2 else '0.3'
AJ = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll']

rclpy.init()
n = Node('reset_stage')
tp = n.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
gc = ActionClient(n, GripperCommand, 'gripper_controller/gripper_cmd')

jt = JointTrajectory()
jt.joint_names = AJ
pt = JointTrajectoryPoint()
pt.positions = [0.0, -0.4, 1.0, 0.2, 0.0]     # 주행 접힘
pt.time_from_start.sec = 3
jt.points = [pt]
tp.publish(jt)

if gc.wait_for_server(timeout_sec=5.0):
    g = GripperCommand.Goal()
    g.command.position = 0.5                   # 열어서 물체를 놓아준다
    g.command.max_effort = 5.0
    gc.send_goal_async(g)

t0 = time.time()
while time.time() - t0 < 5.0:
    rclpy.spin_once(n, timeout_sec=0.1)
rclpy.shutdown()


def svc(s_, t_, r_):
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


# 로봇도 되돌린다. 무대 스크립트는 ROBOT_X를 로봇 위치로 가정해 물체를 놓는데
# (0.3 + 포켓 0.381 = 물체 0.681), 로봇을 옮기지 않으면 직전 실행이 끝난 자리에서
# 시작해 무대 좌표가 통째로 밀린다. 연속 실행일수록 어긋남이 누적된다.
svc('/world/room/set_pose', 'gz.msgs.Pose',
    f'name: "jdamr_cube" position {{x: {ROBOT_X} y: 0 z: 0.05}} orientation {{w: 1}}')
time.sleep(1.5)
print(f'로봇 리셋 완료 (x={ROBOT_X}) → 무대 배치')
r = subprocess.run(['python3', f'/home/mmporong/capstone_tools/{STAGE}', ROBOT_X],
                   capture_output=True, text=True)
print((r.stdout or r.stderr)[-200:])


def pose_of(name):
    """gz에서 모델 위치 조회 (x, y, z)."""
    out = subprocess.run(f'gz model -m {name} -p', shell=True,
                         capture_output=True, text=True).stdout
    for ln in out.splitlines():
        ln = ln.strip()
        if ln.startswith('[') and ln.count(' ') >= 2:
            return [float(v) for v in ln.strip('[]').split()]
    return None


# 배치 검증: 물체가 의도한 곳에 없으면(직전 실행 잔재로 데크 위 등) 강제 재배치
TARGETS = {'floor_stage.py': ("pick_blue", 0.681, 0.0, 0.015)}
if STAGE in TARGETS:
    name, tx, ty, tz = TARGETS[STAGE]
    for attempt in range(3):
        p = pose_of(name)
        if p and abs(p[0] - tx) < 0.02 and abs(p[1] - ty) < 0.02 and abs(p[2] - tz) < 0.02:
            print(f'배치 검증 OK: {name} at ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})')
            break
        print(f'배치 어긋남 {p} → 강제 재배치')
        svc('/world/room/set_pose', 'gz.msgs.Pose',
            f'name: "{name}" position {{x: {tx} y: {ty} z: {tz}}} orientation {{w: 1}}')
        time.sleep(1.5)
    else:
        print('경고: 배치 검증 실패')
