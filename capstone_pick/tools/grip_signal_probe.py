"""파지 판정 신호 비교 — 표준 신호가 실제로 갈라지는지 잰다.

파지 성공 판정의 표준은 그리퍼 액션이 돌려주는 stall 정보다.
    control_msgs/GripperCommand.Result
        stalled       최대 힘을 쓰는데 죠가 안 움직인다 = 뭔가 물렸다
        reached_goal  명령한 각도까지 다 닫혔다 = 허공
    → 파지 성공 = stalled and not reached_goal
실물 SO-101은 여기에 서보 present load(= effort state interface)를 함께 본다.

지금 코드는 이 결과를 읽지 않고 손목캠 블롭 '면적'으로 판정하는데, 면적은
렌즈와의 거리를 재는 값이라 바닥에 놓인 큐브가 코앞에 있으면 쥔 것과 구별되지
않는다(실측: 바닥 큐브인데 면적 42000, 전 구간 HOLDING 오판).

같은 두 상황(큐브 있음 / 없음)에서 네 신호를 나란히 찍어 무엇이 실제로
갈라지는지 확인한다.

사용 (ROS 환경 source 후): python3 grip_signal_probe.py
"""
import subprocess
import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

AJ = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
      'arm_wrist_flex', 'arm_wrist_roll']
POSE_FOLDED = [0.0, -0.4, 1.0, 0.2, 0.0]
POSE_PRE = [0.0, 0.85, 0.15, 0.58, 0.0]
POSE_GRASP = [0.0, 1.15, 0.15, 0.28, 0.0]
GRIP_OPEN, GRIP_CLOSE = 1.2, -0.17
POCKET = 0.400          # 파지 성립 구간 한가운데


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


class Probe(Node):
    def __init__(self):
        super().__init__('grip_signal_probe')
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.pos = self.eff = None
        self.create_subscription(JointState, '/joint_states', self._joint, 10)
        self.grip.wait_for_server(timeout_sec=20)

    def _joint(self, m):
        if 'arm_gripper' not in m.name:
            return
        i = m.name.index('arm_gripper')
        self.pos = m.position[i]
        # effort state interface가 붙어 있으면 여기에 실린다
        self.eff = m.effort[i] if len(m.effort) > i else None

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def arm(self, p, sec=3.0):
        jt = JointTrajectory()
        jt.joint_names = AJ
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in p]
        pt.time_from_start.sec = int(sec)
        jt.points = [pt]
        self.pub.publish(jt)
        self.spin(sec + 1.5)

    def gripper(self, position, effort=7.5):
        """액션 결과를 그대로 돌려준다 — stalled/reached_goal이 표준 판정 신호다."""
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = float(effort)
        f = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        h = f.result()
        if h is None or not h.accepted:
            return None
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=10.0)
        r = rf.result()
        return r.result if r is not None else None


def run(n, with_cube):
    n.gripper(GRIP_OPEN)
    n.arm(POSE_FOLDED, 2.5)
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    if with_cube:
        svc(f'name: "pick_object_green", position: {{x: {POCKET}, y: 0.0, z: 0.02}}')
    else:
        svc('name: "pick_object_green", position: {x: 2.0, y: 2.0, z: 0.02}')   # 치워둔다
    n.spin(1.5)
    n.gripper(GRIP_OPEN)
    n.arm(POSE_PRE)
    n.arm(POSE_GRASP)
    n.spin(0.5)
    res = n.gripper(GRIP_CLOSE)
    n.spin(1.5)
    pos, eff = n.pos, n.eff
    n.arm([0.0, 0.5, 0.4, 0.6, 0.0], 2.5)     # 들어서 진짜 물었는지 확인
    n.spin(1.0)
    z = cube_z()
    return {
        'stalled': None if res is None else res.stalled,
        'reached': None if res is None else res.reached_goal,
        'res_eff': None if res is None else round(res.effort, 3),
        'pos': None if pos is None else round(pos, 3),
        'eff': None if eff is None else round(eff, 3),
        'lifted': (z is not None and z > 0.05) if with_cube else None,
    }


def main():
    rclpy.init()
    n = Probe()
    n.spin(2.0)
    print(f'{"상황":<10} {"stalled":>8} {"reached":>8} {"결과effort":>10} '
          f'{"관절각":>8} {"관절effort":>10} {"실제들림":>8}')
    print('-' * 72)
    rows = []
    for label, with_cube in (('큐브 있음', True), ('허공', False),
                             ('큐브 있음', True), ('허공', False)):
        r = run(n, with_cube)
        rows.append((label, r))
        print(f'{label:<10} {str(r["stalled"]):>8} {str(r["reached"]):>8} '
              f'{str(r["res_eff"]):>10} {str(r["pos"]):>8} {str(r["eff"]):>10} '
              f'{str(r["lifted"]):>8}')
    print()
    cube = [r for l, r in rows if l == '큐브 있음']
    air = [r for l, r in rows if l == '허공']
    for key, name in (('stalled', 'stalled'), ('reached', 'reached_goal'),
                      ('eff', '관절 effort'), ('pos', '관절각')):
        c = {r[key] for r in cube}
        a = {r[key] for r in air}
        verdict = '갈린다' if c.isdisjoint(a) and None not in c | a else '못 가른다'
        print(f'  {name:<14} 큐브={sorted(map(str, c))} 허공={sorted(map(str, a))}  → {verdict}')
    svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}')   # 제자리로
    rclpy.shutdown()


if __name__ == '__main__':
    main()
