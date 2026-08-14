"""바퀴 제원 캘리브레이션 — 줄자 실측으로 오도메트리 스케일을 보정한다.

UMBmark 축소판. 두 단계이고 순서가 중요하다 (회전 보정식이 반지름 보정을
입력으로 쓴다):

  1단계 직진: 오도메트리가 --distance(기본 2m)를 보고할 때까지 전진 후 정지.
     줄자로 실주행 거리를 재서 입력하면 반지름 보정.
       r_new = r_old × (실측 / odom)
     직진에는 트레드가 안 끼므로 반지름만 분리 측정된다.

  2단계 제자리 회전: 오도메트리 yaw 가 --turns(기본 5바퀴)×2π 를 볼 때까지
     회전 후 정지. 테이프로 표시해 둔 실제 회전수를 입력하면 트레드 보정.
       yaw_odom = (dr-dl)/sep 이고 dr·dl 은 반지름에 비례하므로
       sep_new = sep_old × (yaw_odom / yaw_실측) × (r_new / r_old)

드라이버는 시험 중에도 옛 파라미터로 돌므로, 보정식이 그 어긋남을 흡수한다.
끝나면 real_bringup 에 붙여 넣을 인자를 출력한다.

사용 (드라이버·펌웨어가 떠 있는 상태에서, 3m 이상 직선 공간):
  ros2 run jdamr_cube_node wheel_calibration -- --radius 0.075 --separation 0.35
"""
import argparse
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

CMD_RATE_HZ = 20.0        # 펌웨어 워치독(400ms)보다 충분히 촘촘하게
LINEAR_SPEED = 0.10       # [m/s] 미끄러짐 최소화 저속
ANGULAR_SPEED = 0.50      # [rad/s]
TIMEOUT_FACTOR = 3.0      # 예상 소요의 3배 넘으면 이상으로 보고 중단


class OdomWatcher(Node):
    """/odom 을 받아 거리·yaw 누적을 계산한다 (yaw 는 ±π 랩 언랩)."""

    def __init__(self):
        super().__init__('wheel_calibration')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self._lock = threading.Lock()
        self._pose = None          # (x, y, yaw 언랩)
        self._yaw_acc = 0.0
        self._last_yaw = None

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            if self._last_yaw is not None:
                d = yaw - self._last_yaw
                d = math.atan2(math.sin(d), math.cos(d))   # 최단 경로 차분
                self._yaw_acc += d
            self._last_yaw = yaw
            self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                          self._yaw_acc)

    def snapshot(self):
        with self._lock:
            return self._pose

    def drive_until(self, vx, wz, done, label):
        """done(시작, 현재)이 True 될 때까지 지령 유지. 타임아웃 시 None."""
        while self.snapshot() is None:
            time.sleep(0.05)
        start = self.snapshot()
        t0 = time.monotonic()
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(vx), float(wz)
        period = 1.0 / CMD_RATE_HZ
        try:
            while True:
                cur = self.snapshot()
                if done(start, cur):
                    break
                if time.monotonic() - t0 > self._budget:
                    self.get_logger().error(f'{label}: 타임아웃 — 로봇·펌웨어 확인')
                    return None
                self.pub.publish(cmd)
                time.sleep(period)
        finally:
            stop = Twist()
            for _ in range(5):                     # 정지 확실히
                self.pub.publish(stop)
                time.sleep(period)
        time.sleep(0.5)                            # 관성 정지 대기
        return start, self.snapshot()


def ask_float(prompt):
    while True:
        try:
            return float(input(prompt).strip())
        except ValueError:
            print('숫자로 입력하세요.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--radius', type=float, default=0.075, help='현재 wheel_radius [m]')
    ap.add_argument('--separation', type=float, default=0.35, help='현재 wheel_separation [m]')
    ap.add_argument('--distance', type=float, default=2.0, help='직진 시험 거리 [m]')
    ap.add_argument('--turns', type=float, default=5.0, help='회전 시험 바퀴 수')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = OdomWatcher()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    try:
        # ── 1단계: 직진 → 반지름 ──
        input(f'\n[1/2] 직진 {args.distance}m. 출발점을 테이프로 표시하고 Enter → ')
        node._budget = TIMEOUT_FACTOR * args.distance / LINEAR_SPEED
        res = node.drive_until(
            LINEAR_SPEED, 0.0,
            lambda s, c: math.hypot(c[0] - s[0], c[1] - s[1]) >= args.distance,
            '직진')
        if res is None:
            return 1
        s, c = res
        odom_dist = math.hypot(c[0] - s[0], c[1] - s[1])
        real_dist = ask_float(f'  odom 이동 {odom_dist:.4f}m. 줄자 실측 거리 [m] = ')
        r_new = args.radius * real_dist / odom_dist
        print(f'  → wheel_radius {args.radius:.4f} → {r_new:.4f} '
              f'(배율 {real_dist / odom_dist:.4f})')

        # ── 2단계: 제자리 회전 → 트레드 ──
        target_yaw = args.turns * 2.0 * math.pi
        input(f'\n[2/2] 제자리 {args.turns:.0f}바퀴. 차체 앞 방향을 테이프로 표시하고 Enter → ')
        node._budget = TIMEOUT_FACTOR * target_yaw / ANGULAR_SPEED
        res = node.drive_until(
            0.0, ANGULAR_SPEED,
            lambda s, c: abs(c[2] - s[2]) >= target_yaw,
            '회전')
        if res is None:
            return 1
        s, c = res
        odom_yaw = abs(c[2] - s[2])
        real_turns = ask_float(
            f'  odom 회전 {odom_yaw / (2 * math.pi):.3f}바퀴. '
            '실제 회전수(표시 기준, 소수 포함) = ')
        real_yaw = real_turns * 2.0 * math.pi
        sep_new = args.separation * (odom_yaw / real_yaw) * (r_new / args.radius)
        print(f'  → wheel_separation {args.separation:.4f} → {sep_new:.4f}')

        print('\n═══ 결과 — real_bringup 인자로 붙여넣기 ═══')
        print(f'wheel_radius:={r_new:.4f} wheel_separation:={sep_new:.4f}')
        print('검산: 같은 시험을 새 파라미터로 재실행해 배율이 1.00±0.02 인지 확인')
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
