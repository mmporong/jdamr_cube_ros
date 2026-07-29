"""Nav2 자율주행: AMCL 초기 위치 설정 + 목표 지점 이동.

사용: python3 nav_goto.py <x> <y> [yaw_deg]
초기 위치는 시뮬 참값(gz)으로 한 번만 주고, 이후 주행은 Nav2가 담당한다.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

GX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.34
GY = float(sys.argv[2]) if len(sys.argv) > 2 else 0.60
GYAW = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else math.pi / 2


def robot_pose():
    """시뮬 참값에서 로봇 현재 위치(x, y, yaw) — AMCL 초기화용."""
    out = subprocess.run('gz model -m jdamr_cube -p', shell=True,
                         capture_output=True, text=True).stdout
    lines = [ln.strip() for ln in out.splitlines() if ln.strip().startswith('[')]
    if len(lines) < 2:
        return None
    xyz = [float(v) for v in lines[0].strip('[]').split()]
    rpy = [float(v) for v in lines[1].strip('[]').split()]
    return xyz[0], xyz[1], rpy[2]


class Nav(Node):
    def __init__(self):
        super().__init__('nav_goto')
        # Nav2가 시뮬 시각으로 동작하므로 이 노드도 맞춰야 한다 — 실제 시각으로 찍은
        # 타임스탬프는 AMCL이 버려서 초기 위치가 반영되지 않는다(실측).
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, 10)
        self.amcl = None

    def _amcl(self, m):
        p = m.pose.pose.position
        self.amcl = (p.x, p.y)

    def wait_clock(self, sec=10.0):
        """시뮬 시각이 들어올 때까지 대기 (0이면 아직 /clock 미수신)."""
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.get_clock().now().nanoseconds > 0:
                return True
        return False

    def set_initial(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05
        m.pose.covariance[35] = 0.03
        for _ in range(5):          # AMCL이 놓치지 않게 몇 번 발행
            self.init_pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.3)
        self.get_logger().info(f'초기 위치 설정: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}deg)')

    def goto(self, x, y, yaw, timeout=180.0):
        if not self.cli.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('navigate_to_pose 액션 서버 없음')
            return False
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = x
        g.pose.pose.position.y = y
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info(f'목표 전송: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}deg)')
        f = self.cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=15.0)
        h = f.result()
        if h is None or not h.accepted:
            self.get_logger().error('목표 거부')
            return False
        rf = h.get_result_async()
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if rf.done():
                break
        if not rf.done():
            self.get_logger().error('주행 시간 초과')
            return False
        return True


rclpy.init()
n = Nav()
print('시뮬 시각 수신:', n.wait_clock())
p = robot_pose()
if p is None:
    print('로봇 위치 조회 실패')
else:
    n.set_initial(*p)
    for _ in range(30):                 # AMCL이 추정을 내기 시작할 때까지
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.amcl:
            break
    print('AMCL 추정:', None if not n.amcl else f'({n.amcl[0]:.3f}, {n.amcl[1]:.3f})',
          f'| 참값 ({p[0]:.3f}, {p[1]:.3f})')
    ok = n.goto(GX, GY, GYAW)
    time.sleep(2)
    q = robot_pose()
    print(f'주행 결과: {"도착" if ok else "실패"}')
    if q:
        err = math.hypot(q[0] - GX, q[1] - GY)
        print(f'최종 위치 ({q[0]:.3f}, {q[1]:.3f}) 목표 ({GX:.3f}, {GY:.3f}) 오차 {err * 1000:.0f}mm')
rclpy.shutdown()
