"""odom(바퀴 추정)이 실제 이동을 얼마나 정확히 반영하는지 측정한다.

접근 단계에서 물체가 카메라 사각지대(0.37m)에 들어가면 비전이 끊기고
추측항법으로 넘어간다. 그때 odom이 실제보다 크게 나오면 로봇은 "다 왔다"고
믿고 멈추는데 실제로는 멀리 있다 — 실측: 추정 0.406m / 참값 0.712m (306mm 오차).

시뮬 참값(gz)과 odom을 같은 구간에서 재서 그 배율을 구한다.

사용 (ROS 환경 source 후):
  python3 odom_check.py
"""
import math
import re
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.node import Node

MODEL = 'jdamr_cube'
TESTS = [(0.15, 4.0), (0.20, 3.0), (0.10, 5.0)]   # (전진속도 m/s, 시간 s)


def gz_pose():
    """시뮬이 아는 참값 [x, y]. SDF 경고가 섞여 나오므로 '- Pose' 다음 줄만 읽는다."""
    out = subprocess.run(['gz', 'model', '-m', MODEL, '-p'],
                         capture_output=True, text=True, timeout=20).stdout
    lines = out.splitlines()
    for i, l in enumerate(lines):
        if '- Pose' in l and i + 1 < len(lines):
            try:
                v = [float(x) for x in lines[i + 1].strip().strip('[]').split()]
                if len(v) >= 2:
                    return v[0], v[1]
            except ValueError:
                continue
    return None


def main():
    rclpy.init()
    n = Node('odom_check')
    pub = n.create_publisher(Twist, 'cmd_vel', 10)
    odom = {}
    n.create_subscription(Odometry, 'odom',
                          lambda m: odom.update(x=m.pose.pose.position.x,
                                                y=m.pose.pose.position.y), 10)
    for _ in range(50):
        rclpy.spin_once(n, timeout_sec=0.05)
    if not odom:
        return print('odom 수신 실패')

    print(f'{"명령":>14} {"참값(gz)":>10} {"odom":>10} {"비율":>7} {"오차":>9}')
    for vx, sec in TESTS:
        g0 = gz_pose()
        o0 = (odom['x'], odom['y'])
        t0 = time.time()
        while time.time() - t0 < sec:
            m = Twist()
            m.linear.x = vx
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.02)
        pub.publish(Twist())
        t0 = time.time()
        while time.time() - t0 < 1.5:
            rclpy.spin_once(n, timeout_sec=0.05)

        g1 = gz_pose()
        o1 = (odom['x'], odom['y'])
        if not (g0 and g1):
            print('  참값 조회 실패'); continue
        real = math.hypot(g1[0] - g0[0], g1[1] - g0[1])
        est = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
        ratio = est / real if real > 1e-6 else float('nan')
        print(f'{vx:5.2f}m/s×{sec:.0f}s {real:9.3f}m {est:9.3f}m {ratio:7.3f} '
              f'{(est - real) * 1000:+8.0f}mm')

    print('\n비율 1.0 = 정확. 1.0보다 크면 실제보다 많이 갔다고 착각한다')
    print('(그러면 목표 지점 앞에서 멈춘다 — 파지 실패의 원인)')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
