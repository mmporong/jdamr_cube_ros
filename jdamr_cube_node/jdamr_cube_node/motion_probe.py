"""자동 운동 프로브 — 로봇이 스스로 정해진 동작을 하고 odom↔SLAM 어긋남을 잰다.

한 바퀴 주행이 어려운 상황을 위한 축소판이다. 사람은 로봇을 놓기만 하고,
주행·측정·판정은 도구가 한다. 좁은 공간에서 되며 폐루프 성질을 유지한다.

측정 원리
---------
동작이 끝나면 로봇은 **출발 자세로 돌아와 있어야 한다**(제자리 회전이든 왕복
직진이든 설계상 원점 복귀). 그때 두 계층이 각각 얼마나 벌어졌는지 본다.

  odom 잔차 : 바퀴가 "돌아왔다"고 믿는 정도. 명령 기준으로 닫히므로 작게 나온다
  SLAM 잔차 : 스캔이 본 결과. **이 값이 실제 판정 대상이다**
  괴리      : 두 계층의 차이 = SLAM 이 오도메트리에 반대하는 정도

동작 종류
---------
  spin : 제자리 360° × N. 회전 중 스캔 왜곡(타임스탬프·장착 오프셋)이 가장
         크게 드러나는 동작이다. 공간이 로봇 크기면 된다
  back : 전진 D → 후진 D. 직진 구간의 정확도. 복도형 무특징 구간에서
         전진이 기각되는지도 여기서 보인다

판정 문턱은 실내 소형 AMR 경험칙이며 절대 기준이 아니다 — 같은 동작을 환경만
바꿔 재고 **숫자를 비교**하는 용도가 본령이다.
"""
import argparse
import json
import math
import os
import threading
import time

import rclpy
import signal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener

CMD_HZ = 15.0          # 펌웨어 워치독(400ms)보다 촘촘히


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Probe(Node):
    def __init__(self):
        super().__init__('motion_probe')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._odom, 10)
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)

        self._l = threading.Lock()
        self.o = None                # (x, y, yaw_raw)
        self.yaw_acc = 0.0           # 언랩 누적 (다회전 대응)
        self.dist = 0.0
        self._prev = None
        self._prev_yaw = None
        self.track = []
        self.aborted = None          # 중단 사유 (None 이면 진행 중)
        # 두 경로로 멈춘다: 폰/브라우저 STOP → /probe_abort, 터미널 Ctrl+C → SIGINT.
        # 어느 쪽이든 지령 발행을 끊고 정지 프레임을 보낸 뒤 부분 결과를 남긴다.
        self.create_subscription(Empty, 'probe_abort', self._abort, 10)
        self.create_timer(0.2, self._sample)

    def _abort(self, _msg):
        if self.aborted is None:
            self.aborted = '외부 STOP'
            self.get_logger().warn('중단 요청 수신 — 정지합니다')

    def _odom(self, m):
        p = m.pose.pose.position
        y = yaw_of(m.pose.pose.orientation)
        with self._l:
            if self._prev is not None:
                self.dist += math.hypot(p.x - self._prev[0], p.y - self._prev[1])
            if self._prev_yaw is not None:
                self.yaw_acc += wrap(y - self._prev_yaw)
            self._prev, self._prev_yaw = (p.x, p.y), y
            self.o = (p.x, p.y, y)

    def slam(self):
        try:
            t = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        v = t.transform.translation
        return (v.x, v.y, yaw_of(t.transform.rotation))

    def _sample(self):
        with self._l:
            o, ya = self.o, self.yaw_acc
        s = self.slam()
        if o:
            self.track.append([round(time.monotonic(), 2), o[0], o[1], ya,
                               s[0] if s else None, s[1] if s else None])

    def state(self):
        with self._l:
            return self.o, self.yaw_acc, self.dist

    def stop(self):
        for _ in range(6):
            self.pub.publish(Twist())
            time.sleep(1.0 / CMD_HZ)

    def drive(self, vx, wz, done, budget):
        """done(state) 이 참이 될 때까지 지령 유지. 예산 초과 시 False."""
        c = Twist()
        c.linear.x, c.angular.z = float(vx), float(wz)
        t0 = time.monotonic()
        while not done(self.state()):
            if self.aborted:
                self.stop()
                return False
            if time.monotonic() - t0 > budget:
                self.aborted = '시간 예산 초과'
                self.stop()
                return False
            self.pub.publish(c)
            time.sleep(1.0 / CMD_HZ)
        self.stop()
        time.sleep(1.0)          # 관성 정지 대기
        return True


def run_spin(n, turns, wz):
    target = turns * 2 * math.pi
    _, y0, _ = n.state()
    print(f'  제자리 {turns:g}바퀴 회전 ({wz:.2f} rad/s)...', flush=True)
    ok = n.drive(0.0, wz,
                 lambda st: abs(st[1] - y0) >= target,
                 budget=target / abs(wz) * 3 + 15)
    return ok


def run_back(n, dist_m, vx):
    o0, _, d0 = n.state()
    print(f'  전진 {dist_m:.1f} m ...', flush=True)
    ok = n.drive(vx, 0.0,
                 lambda st: (st[2] - d0) >= dist_m,
                 budget=dist_m / vx * 3 + 15)
    if not ok:
        return False
    time.sleep(1.0)
    _, _, d1 = n.state()
    print(f'  후진 {dist_m:.1f} m ...', flush=True)
    return n.drive(-vx, 0.0,
                   lambda st: (st[2] - d1) >= dist_m,
                   budget=dist_m / vx * 3 + 15)


def resid(a, b):
    if a is None or b is None:
        return None
    dx, dy = b[0] - a[0], b[1] - a[1]
    return {'dx': round(dx, 4), 'dy': round(dy, 4),
            'dist': round(math.hypot(dx, dy), 4),
            'dyaw_deg': round(math.degrees(wrap(b[2] - a[2])), 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('motion', choices=['spin', 'back'], help='동작 종류')
    ap.add_argument('--turns', type=float, default=2.0, help='spin: 회전수')
    ap.add_argument('--wz', type=float, default=0.5, help='spin: 각속도 rad/s')
    ap.add_argument('--dist', type=float, default=1.5, help='back: 편도 거리 m')
    ap.add_argument('--vx', type=float, default=0.08, help='back: 속도 m/s')
    ap.add_argument('--tag', default='')
    ap.add_argument('--out', default=os.path.expanduser('~/maps'))
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    n = Probe()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()

    def _sigint(_s, _f):
        if n.aborted is None:
            n.aborted = 'Ctrl+C'
    signal.signal(signal.SIGINT, _sigint)   # 예외 대신 플래그 → 정지·보고를 마치고 끝낸다

    print('\n=== 자동 운동 프로브 ===')
    print('  멈추려면: 폰/브라우저 STOP 버튼 또는 이 터미널에서 Ctrl+C')
    while n.state()[0] is None:
        print('  /odom 대기...'); time.sleep(1.0)
    if n.slam() is None:
        print('  ⚠ SLAM(map→base_footprint) 미연결 — odom 만 측정합니다')
    time.sleep(1.5)

    o0, y0, d0 = n.state()
    s0 = n.slam()
    ok = run_spin(n, args.turns, args.wz) if args.motion == 'spin' \
        else run_back(n, args.dist, args.vx)
    time.sleep(1.0)
    o1, y1, d1 = n.state()
    s1 = n.slam()

    if not ok:
        why = n.aborted or '알 수 없음'
        print(f'\n⚠ 동작 미완료 ({why}) — 아래 수치는 중단 시점까지의 부분 결과입니다')

    od = resid(o0, o1)
    sl = resid(s0, s1)
    turned = math.degrees(y1 - y0)
    print(f'\n주행거리 {d1-d0:.2f} m · odom 총 회전 {turned:+.1f}°')
    print(f'  odom 잔차 : {od["dist"]*100:5.1f} cm · {od["dyaw_deg"]:+.1f}°')
    if sl:
        print(f'  SLAM 잔차 : {sl["dist"]*100:5.1f} cm · {sl["dyaw_deg"]:+.1f}°')
        gap = math.hypot(sl['dx'] - od['dx'], sl['dy'] - od['dy'])
        gyaw = abs(sl['dyaw_deg'] - od['dyaw_deg'])
        print(f'  괴리      : {gap*100:5.1f} cm · {gyaw:.1f}°  (두 계층의 불일치)')
        # 중단된 실행은 판정하지 않는다. 동작이 원점으로 닫히지 않았으므로
        # "잔차가 작다"가 곧 "잘 맞았다"를 뜻하지 않는다 — 덜 움직였을 뿐일 수 있다.
        if not ok:
            v = f'판정 보류 — 동작 미완료({n.aborted or "사유 불명"}). 잔차는 참고값'
        elif sl['dist'] < 0.08 and gyaw < 5:
            v = '합격 — SLAM 이 출발 자세를 잘 잡고 있다'
        elif gap > 0.25 or gyaw > 15:
            v = '⚠ SLAM 이 오도메트리와 크게 어긋난다 — 정합 설정·왜곡 의심'
        else:
            v = '경계 — 같은 동작을 다른 환경에서 재서 비교할 것'
        print(f'\n판정: {v}')
    else:
        v = ('판정 보류 — 동작 미완료' if not ok else 'SLAM 미가동')
        print('\n판정: SLAM 미연결 — odom 만 기록')

    os.makedirs(args.out, exist_ok=True)
    tag = args.tag or args.motion
    fp = os.path.join(args.out, f'probe_{tag}_{time.strftime("%m%d_%H%M")}.json')
    with open(fp, 'w') as f:
        json.dump({'motion': args.motion, 'tag': tag, 'completed': ok,
                   'aborted_by': n.aborted,
                   'path_len_m': round(d1 - d0, 3), 'odom_turn_deg': round(turned, 2),
                   'odom_resid': od, 'slam_resid': sl, 'verdict': v,
                   'track': n.track}, f, ensure_ascii=False, indent=1)
    print(f'저장: {fp}')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
