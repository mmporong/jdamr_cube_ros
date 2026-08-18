"""폐루프 오차 측정 — 실기에서 ATE 를 대신하는 판정.

시뮬에는 `gz model -p` 참값이 있어 ATE(절대 궤적 오차)를 바로 잰다. 실기에는
참값이 없다. 대신 **출발점으로 물리적으로 되돌아오면 그 지점의 참값을 우리가
안다**(0,0,0)는 사실을 쓴다. 복귀 시점에 시스템이 보고하는 좌표가 곧 누적 오차다.

두 계층을 따로 읽는 것이 이 도구의 요점이다.

  odom 폐루프 오차   : odom→base_footprint. 스캔·루프클로저가 손대지 않은
                       순수 바퀴 적산 → 오도메트리(하드웨어·캘리브레이션)의 성적표
  SLAM 폐루프 오차   : map→base_footprint. 스캔 매칭과 루프 클로저가 개입한 결과
                       → SLAM 스택(설정·튜닝)의 성적표

두 값을 갈라 보면 책임 소재가 갈린다:
  odom 큼 · SLAM 작음  → 정상. SLAM 이 드리프트를 회수했다(설계대로)
  odom 작음 · SLAM 큼  → SLAM 이 멀쩡한 오도메트리를 망쳤다. 정합 설정 문제
  둘 다 큼             → 오도메트리가 무너졌고 SLAM 도 못 건졌다. 하드웨어부터
  둘 다 작음           → 합격

정규화: 오차를 주행 거리로 나눈 상대 오차(%)를 함께 낸다. 10m 돌고 10cm 는
1% 로 좋은 값이고, 1m 돌고 10cm 는 10% 로 나쁜 값이다. 절대값만 보면 짧게
돌수록 좋아 보이는 함정에 빠진다.

사용:
  ros2 run jdamr_cube_node loop_eval
    → 출발점 표시 후 Enter (기준 스냅샷)
    → 사용자가 한 바퀴 주행 (도구는 궤적을 계속 기록)
    → 같은 표시에 정확히 맞춰 세우고 Enter (종료 스냅샷)
    → 결과 출력 + JSON 저장

  --tag corridor_v2   결과 파일 이름에 붙일 실험 딱지
  --out ~/maps        결과 저장 위치
"""
import argparse
import json
import math
import os
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class LoopEval(Node):
    def __init__(self):
        super().__init__('loop_eval')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_subscription(Odometry, 'odom', self._odom, 10)

        self._lock = threading.Lock()
        self.odom = None            # (x, y, yaw)
        self.path_len = 0.0         # odom 기준 주행 거리
        self._prev = None
        self.track = []             # [(t, ox, oy, oyaw, mx, my, myaw)]
        self.t0 = time.monotonic()

        self.create_timer(0.2, self._sample)   # 5Hz 궤적 기록

    def _odom(self, msg):
        p = msg.pose.pose.position
        y = yaw_of(msg.pose.pose.orientation)
        with self._lock:
            if self._prev is not None:
                self.path_len += math.hypot(p.x - self._prev[0], p.y - self._prev[1])
            self._prev = (p.x, p.y)
            self.odom = (p.x, p.y, y)

    def map_pose(self):
        """map→base_footprint. SLAM 이 안 떠 있으면 None."""
        try:
            tf = self.buf.lookup_transform('map', 'base_footprint',
                                           rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y, yaw_of(tf.transform.rotation))

    def _sample(self):
        with self._lock:
            o = self.odom
        m = self.map_pose()
        if o:
            self.track.append((round(time.monotonic() - self.t0, 2),
                               o[0], o[1], o[2],
                               m[0] if m else None,
                               m[1] if m else None,
                               m[2] if m else None))

    def snapshot(self):
        with self._lock:
            o = self.odom
            L = self.path_len
        return o, self.map_pose(), L


def delta(a, b, label):
    """b - a. 없으면 None."""
    if a is None or b is None:
        return None
    dx, dy = b[0] - a[0], b[1] - a[1]
    return {
        'layer': label,
        'dx': round(dx, 4), 'dy': round(dy, 4),
        'dist': round(math.hypot(dx, dy), 4),
        'dyaw_deg': round(math.degrees(wrap(b[2] - a[2])), 2),
    }


def show(d, path_len):
    if d is None:
        print(f'  {"(측정 불가 — 해당 프레임 없음)":>34}')
        return
    rel = (100.0 * d['dist'] / path_len) if path_len > 0.05 else float('nan')
    print(f"  {d['layer']:<6} 위치오차 {d['dist']*100:6.1f} cm "
          f"(dx {d['dx']*100:+.1f}, dy {d['dy']*100:+.1f}) · "
          f"방위오차 {d['dyaw_deg']:+.1f}° · 주행거리 대비 {rel:.2f}%")


def verdict(od, sl):
    """책임 소재 판정. 문턱은 실내 소형 AMR 경험칙."""
    if od is None:
        return '판정 불가 — odom 미수신'
    if sl is None:
        return f'SLAM 미가동 — odom 단독 {od["dist"]*100:.1f} cm'
    o, s = od['dist'], sl['dist']
    if s < 0.10 and o < 0.30:
        return '합격 — 오도메트리·SLAM 둘 다 건전'
    if s < 0.10 <= o:
        return '정상 동작 — 오도메트리 드리프트를 SLAM 이 회수했다'
    if o < 0.15 <= s:
        return '⚠ SLAM 이 멀쩡한 오도메트리를 악화시켰다 → 정합 설정을 의심'
    return '⚠ 둘 다 큼 — 오도메트리(제원·미끄러짐)부터 재확인'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', default='run', help='실험 딱지 (파일명에 들어감)')
    ap.add_argument('--out', default=os.path.expanduser('~/maps'))
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = LoopEval()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print('\n=== 폐루프 오차 측정 ===')
    print('바닥에 십자와 앞방향 화살표를 표시하고, 로봇을 거기 맞춰 세우세요.')
    while node.snapshot()[0] is None:
        print('  /odom 대기...'); time.sleep(1.0)

    input('준비되면 Enter → ')
    o0, m0, L0 = node.snapshot()
    print(f'  기준 확보. SLAM {"연결됨" if m0 else "미가동(odom 만 측정)"}')
    print('\n한 바퀴 주행하고 같은 표시에 정확히 되돌아와 세우세요.')
    input('복귀 완료 후 Enter → ')
    o1, m1, L1 = node.snapshot()

    path_len = L1 - L0
    od = delta(o0, o1, 'odom')
    sl = delta(m0, m1, 'SLAM')

    print(f'\n주행 거리 {path_len:.2f} m · 궤적 표본 {len(node.track)}개')
    show(od, path_len)
    show(sl, path_len)
    print(f'\n판정: {verdict(od, sl)}')

    os.makedirs(args.out, exist_ok=True)
    stamp = time.strftime('%m%d_%H%M')
    fp = os.path.join(args.out, f'loop_{args.tag}_{stamp}.json')
    with open(fp, 'w') as f:
        json.dump({'tag': args.tag, 'path_len_m': round(path_len, 3),
                   'odom_error': od, 'slam_error': sl,
                   'verdict': verdict(od, sl), 'track': node.track}, f,
                  ensure_ascii=False, indent=1)
    print(f'저장: {fp}  (track 으로 궤적 그림을 그릴 수 있다)')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
