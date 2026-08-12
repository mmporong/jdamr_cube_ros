"""쓰레기통 투입 경로가 성립하는가 — 시뮬 없이 기하만으로 검증한다.

투입은 오랫동안 고정 자세(POSE_DROP)였다. 그 자세는 그리퍼를 크게 눕혀 물체가
**스스로 빠지는 것에 기대는** 방식인데, 빠지는 방향을 제어할 수 없어 착지가
통 앞 바닥으로 흩어졌다(실측: 통에서 0.20~0.55m). 운반 자세에서 관절이 한 번에
크게 튀는 것도 문제였다 — 들기에서 큐브를 놓친 것과 같은 원인이다.

여기서는 좌표를 이어 붙이는 경로(`K.drop_path`)가 통 도착 허용오차 전 범위에서
성립하는지 본다. ROS도 Gazebo도 필요 없다. 확인하는 것 셋:

  · 경로 전 구간에 IK 해가 있는가
  · 큐브가 통 벽(월드 실측 0.090m)에 걸리지 않는가
  · 착지가 개구부(반폭 0.068m) 안인가 — ArUco 오차 ±25mm를 견디는가

사용: python3 drop_path_probe.py     (capstone_pick 디렉토리에서)
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick import kinematics as K  # noqa: E402

CUBE_SIZE, CUBE_HALF = 0.030, 0.015
CUBE_CZ = CUBE_SIZE / 2.0
TRASH_WALL_TOP, TRASH_HALF_OUTER, TRASH_OPEN_HALF = 0.090, 0.080, 0.068
DROP_CZ = TRASH_WALL_TOP + 0.010 + CUBE_HALF        # 0.115
CARRY_X, CARRY_UP = 0.330, 0.180
TRASH_POCKET_X, ARRIVE_TOL, ARUCO_ERR = 0.500, 0.025, 0.025


def hits_wall(x, z, trash_x):
    """큐브가 통 몸통과 간섭하는가 — 통 위인데 벽 상단보다 낮으면 걸린다."""
    return abs(x - trash_x) <= TRASH_HALF_OUTER + CUBE_HALF and (z - CUBE_HALF) < TRASH_WALL_TOP


def check(trash_r):
    """통 중심이 trash_r에 있을 때 투입 경로가 성립하는가."""
    path, pitch = K.drop_path(trash_r, 0.0, CUBE_CZ, up=DROP_CZ - CUBE_CZ,
                              from_xy=(CARRY_X, 0.0), from_up=CARRY_UP, steps=6)
    if path is None:
        return None, None, None
    n = len(path) - 1
    # 큐브 중심 궤적은 drop_path의 보간과 같은 식으로 재현한다
    wall = any(hits_wall(CARRY_X + (trash_r - CARRY_X) * i / n,
                         CUBE_CZ + CARRY_UP + (DROP_CZ - CUBE_CZ - CARRY_UP) * i / n,
                         trash_r) for i in range(n + 1))
    # 착지 지점은 끝 자세의 TCP에서 역산한다 (grasp_q가 두는 관계의 역)
    tcp = K.fk_pos(path[-1])
    return pitch, wall, (tcp[0] + K.GRASP_BACK, tcp[2] + K.GRASP_DOWN)


def main():
    print('쓰레기통 투입 경로 검증 — 기하만, 시뮬 불필요\n')
    print(f'통(월드 실측): 벽 상단 {TRASH_WALL_TOP:.3f}m · 개구부 반폭 {TRASH_OPEN_HALF:.3f}m')
    print(f'운반 자세: 큐브 중심 ({CARRY_X:.3f}, {CUBE_CZ + CARRY_UP:.3f})')
    print(f'투입 높이: 큐브 중심 {DROP_CZ:.3f}m → 하단이 벽 위 '
          f'{(DROP_CZ - CUBE_HALF - TRASH_WALL_TOP) * 1000:.0f}mm\n')

    print(f'도착 허용오차 ±{ARRIVE_TOL * 1000:.0f}mm 범위에서 경로가 성립하는가')
    print(f'{"통까지 r":>9} {"IK":>5} {"피치":>8} {"눕힘":>7} {"벽간섭":>7} {"착지 오차":>10}')
    print('-' * 56)
    ok_all = True
    for i in range(11):
        r = TRASH_POCKET_X - ARRIVE_TOL + 2 * ARRIVE_TOL * i / 10
        pitch, wall, land = check(r)
        if pitch is None:
            ok_all = False
            print(f'{r:9.3f} {"✗":>5} {"해 없음":>8}')
            continue
        # 착지가 목표(통 중심)에서 얼마나 벗어나는가 — 기구학이 정확하면 0이어야 한다
        err = (land[0] - r) * 1000
        ok_all &= (not wall) and abs(err) < 1.0
        print(f'{r:9.3f} {"○":>5} {math.degrees(pitch):7.0f}도 '
              f'{math.degrees(pitch - K.GRASP_PITCH):6.0f}도 '
              f'{"걸림" if wall else "-":>7} {err:9.2f}mm')

    print('\n--- 방위(brg)를 팔에 실을 수 있는가 ---')
    print('도착 조건은 |brg| < 0.10rad(5.7도)다. 그대로 두면 착지가 좌우로 어긋나는데,')
    print('IK에 실으면 pan이 돌며 어깨 오프셋 때문에 도달 거리가 줄어 해가 사라진다.')
    print(f'{"brg[도]":>8} {"좌우 편차":>10} ' + ' '.join(f'{f"r={r:.3f}":>9}' for r in (0.475, 0.500, 0.525)))
    for bd in (0.0, 1.0, 3.0, 5.7):
        b = math.radians(bd)
        cells = []
        for r in (0.475, 0.500, 0.525):
            p, _ = K.drop_path(r * math.cos(b), r * math.sin(b), CUBE_CZ, up=DROP_CZ - CUBE_CZ,
                               from_xy=(CARRY_X, 0.0), from_up=CARRY_UP, steps=10)
            cells.append('해 있음' if p else '해 없음')
        print(f'{bd:8.1f} {TRASH_POCKET_X * math.sin(b) * 1000:8.0f}mm ' + ' '.join(f'{c:>9}' for c in cells))
    print('→ 그래서 방위는 팔이 아니라 **차체 제자리 회전**으로 흡수한다(거리 r은 보존된다).')

    print('\n--- 착지 여유 (개구부 중앙 목표, ArUco 오차만큼 빗나갈 때) ---')
    print(f'개구부 반폭 {TRASH_OPEN_HALF * 1000:.0f}mm, 큐브 반폭 {CUBE_HALF * 1000:.0f}mm '
          f'→ 중앙에서 {(TRASH_OPEN_HALF - CUBE_HALF) * 1000:.0f}mm까지 벗어나도 들어간다')
    print(f'ArUco 거리 오차 ±{ARUCO_ERR * 1000:.0f}mm → '
          f'여유 {((TRASH_OPEN_HALF - CUBE_HALF) - ARUCO_ERR) * 1000:+.0f}mm '
          + ('✓ 견딘다' if (TRASH_OPEN_HALF - CUBE_HALF) > ARUCO_ERR else '✗ 부족'))

    print('\n--- 종전 고정 자세(POSE_DROP)와 비교 ---')
    _, p_ik, _ = None, check(TRASH_POCKET_X)[0], None
    print(f'  IK 투입   : 피치 {math.degrees(p_ik):.0f}도 — 죠가 거의 수직, 착지 지점이 정해진다')
    print('  POSE_DROP : 리치 0.528을 위해 크게 눕힌 자세 — 물체가 스스로 빠지는 것에 기댄다')

    print(f'\n판정: {"○ 전 범위 성립" if ok_all else "✗ 성립하지 않는 구간 있음"}')
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
