"""기구학 단위 테스트 — **시뮬 없이** 돈다.

이 파일이 있는 이유가 곧 IK로 옮기는 이유다. 종전 구조에서는 상수 하나를 고칠
때마다 Gazebo를 띄우고 몇 분씩 돌려야 확인이 됐고, 그 사이에 다른 상수가 조용히
무효가 됐다. 기구학은 순수 기하라 여기서 초 단위로 전수 검증된다.

    python3 -m pytest capstone_pick/test/test_kinematics.py -q
    (pytest가 없으면) python3 capstone_pick/test/test_kinematics.py
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick import kinematics as K  # noqa: E402

URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    '..', '..', 'jdamr_cube_description', 'urdf', 'jdamr_cube.urdf')

# Gazebo TF로 실측한 값 (descend_probe.py, 2026-08-10). 소수점 3자리까지 기록돼
# 있으므로 허용 오차는 그 반올림(0.5mm)보다 넉넉히 잡는다.
MEASURED = [
    ([0.0, 0.85, 0.15, 0.58, 0.0], dict(tcp_z=0.050, jaw_x=0.369, jaw_z=0.148)),
    ([0.0, 1.15, 0.15, 0.28, 0.0], dict(tcp_z=-0.001, jaw_x=0.344, jaw_z=0.097)),
]


def test_fk_matches_measurement():
    """FK가 시뮬 실측과 맞는가 — 기구학 파라미터가 옳다는 근거."""
    for q, exp in MEASURED:
        tcp = K.fk_pos(q)
        jaw = K.fk_pos(q, upto='jaw')
        got = dict(tcp_x=tcp[0], tcp_z=tcp[2], jaw_x=jaw[0], jaw_z=jaw[2])
        for key, want in exp.items():
            err = abs(got[key] - want)
            assert err < 0.001, f'{q} {key}: FK {got[key]:.4f} vs 실측 {want:.4f}'


def test_fk_arm_stays_in_plane():
    """pan=0이면 팔이 평면 안에 있어야 한다 — IK 분해의 전제."""
    for q in ([0, 0.85, 0.15, 0.58, 0], [0, 0.4, 0.8, 0.2, 0], [0, 1.3, -0.3, 0.9, 0]):
        assert abs(K.fk_pos(q)[1]) < 1e-4, f'{q}에서 TCP y={K.fk_pos(q)[1]:.6f}'


def test_pan_is_pure_bearing():
    """pan은 반경·높이를 보존하고 방위만 바꾼다. 방위 = -pan."""
    base = K.fk_pos([0.0, 0.85, 0.15, 0.58, 0.0])
    r0 = math.hypot(base[0] - K.PAN_X, base[1])
    for pan in (0.3, -0.5, 1.0, -1.2):
        p = K.fk_pos([pan, 0.85, 0.15, 0.58, 0.0])
        r = math.hypot(p[0] - K.PAN_X, p[1])
        brg = math.atan2(p[1], p[0] - K.PAN_X)
        assert abs(r - r0) < 1e-6, f'pan={pan}에서 반경이 {(r-r0)*1000:+.3f}mm 변했다'
        assert abs(p[2] - base[2]) < 1e-6, f'pan={pan}에서 높이가 변했다'
        assert abs(brg + pan) < 1e-3, f'pan={pan}인데 방위가 {brg:.4f}'   # URDF rpy 반올림 여파


def test_ik_roundtrip():
    """FK(IK(p)) = p — 작업공간 격자 전수. 이게 IK 정확성의 핵심 근거다."""
    ok = bad = 0
    worst = 0.0
    for x in [0.28, 0.33, 0.38, 0.43, 0.48]:
        for y in [-0.15, -0.07, 0.0, 0.07, 0.15]:
            for z in [0.0, 0.05, 0.12, 0.20]:
                for pitch in [-math.pi / 2, -1.3, -1.66]:
                    q = K.ik_best(x, y, z, pitch=pitch)
                    if q is None:
                        bad += 1
                        continue
                    p = K.fk_pos(q)
                    err = math.dist(p, (x, y, z))
                    worst = max(worst, err)
                    assert err < 1e-6, (f'({x},{y},{z}) pitch={pitch}: '
                                        f'IK→FK 오차 {err * 1000:.3f}mm')
                    # 죠 피치도 지정한 대로 나와야 한다
                    assert abs(K.jaw_pitch(q) - pitch) < 1e-9
                    ok += 1
    assert ok > 100, f'유효 해가 너무 적다 ({ok}개) — 작업공간 설정을 확인하라'
    print(f'  왕복 {ok}점 통과 (해 없음 {bad}점), 최대 오차 {worst * 1e6:.3f}um')


def test_ik_respects_limits():
    """한계를 넘는 해는 돌려주지 않는다."""
    for x, y, z in [(1.5, 0.0, 0.1), (0.0, 0.0, 1.5), (0.05, 0.0, -0.3)]:
        q = K.ik_best(x, y, z)
        assert q is None or K.in_limits(q), f'({x},{y},{z})에서 한계 밖 해 반환'


def test_roll_moves_tcp_but_not_jaw_origin():
    """roll은 아랫턱 원점을 안 움직이지만 TCP는 움직인다.

    TCP가 roll 축에서 7.9mm 벗어나 있기 때문이다(TCP_OFFSET의 x). IK가 아랫턱
    원점을 목표로 푸는 근거가 이것이다 — TCP를 그냥 목표로 삼으면 roll에 따라
    최대 15.8mm 어긋난다.
    """
    q0 = [0.0, 0.85, 0.15, 0.58, 0.0]
    jaw0 = K.fk_pos(q0, upto='jaw')
    tcp0 = K.fk_pos(q0)
    moved = 0.0
    for roll in (0.3, -0.6, 1.2):
        q = [0.0, 0.85, 0.15, 0.58, roll]
        assert math.dist(K.fk_pos(q, upto='jaw'), jaw0) < 1e-9, \
            f'roll={roll}이 아랫턱 원점을 움직였다'
        moved = max(moved, math.dist(K.fk_pos(q), tcp0))
    assert moved > 0.003, 'TCP가 roll에 안 움직이면 이 보정 자체가 불필요하다'
    print(f'  roll에 따른 TCP 이동 최대 {moved * 1000:.1f}mm (아랫턱 원점은 0)')


def test_constants_match_urdf():
    """코드 상수가 URDF와 일치하는가 — URDF가 바뀌면 여기서 잡힌다."""
    root = ET.parse(URDF).getroot()
    joints = {j.get('name'): j for j in root.findall('joint')}

    def origin(name):
        o = joints[name].find('origin')
        xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
        rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
        return xyz, rpy

    names = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
             'arm_wrist_flex', 'arm_wrist_roll']
    for i, n in enumerate(names):
        xyz, rpy = origin(n)
        for a, b, lbl in ((xyz, K.JOINTS[i][0], 'xyz'), (rpy, K.JOINTS[i][1], 'rpy')):
            for va, vb in zip(a, b):
                assert abs(va - vb) < 1e-4, f'{n} {lbl}: URDF {va} vs 코드 {vb}'   # URDF는 5자리 반올림
        lim = joints[n].find('limit')
        lo, hi = float(lim.get('lower')), float(lim.get('upper'))
        assert abs(lo - K.LIMITS[i][0]) < 1e-6 and abs(hi - K.LIMITS[i][1]) < 1e-6, \
            f'{n} 한계 불일치'


def test_pan_axis_matches_pick_node():
    """IK가 유도한 pan축 x가 pick_node의 실측 상수와 맞는가 (교차 검증)."""
    assert abs(K.PAN_X - 0.159) < 0.001, f'PAN_X={K.PAN_X:.5f}'


def test_reach_q_prefers_vertical():
    """reach_q는 닿는 한도 안에서 **최소한만** 눕혀야 한다.

    눕힐수록 물체가 죠에서 흘러내리므로, 가까운 곳은 수직으로 잡고 멀어질수록
    필요한 만큼만 누워야 한다. 거리가 늘 때 피치가 단조로 커지는지(더 눕는지) 본다.
    """
    prev = None
    for x in [0.36 + 0.02 * i for i in range(8)]:
        q, p = K.reach_q(x, 0.0, 0.015, up=0.10)
        if q is None:
            continue
        assert K.in_limits(q), f'x={x}에서 한계 밖 해'
        assert K.GRASP_PITCH <= p <= K.DROP_PITCH_MAX + 1e-9, f'x={x}: 피치 {p} 범위 밖'
        if prev is not None:
            assert p >= prev - 1e-9, f'x={x}에서 피치가 되레 수직으로 갔다 ({prev}→{p})'
        prev = p
    assert prev is not None, 'reach_q가 어디서도 해를 못 냈다'


def test_drop_path_is_continuous_and_reaches():
    """투입 경로가 통 도착 허용오차 전 범위에서 성립하고 관절이 튀지 않는가.

    고정 자세(POSE_DROP)로 점프하던 것을 좌표 보간으로 바꾼 이유가 급변 제거이므로,
    '해가 있다'만으로는 부족하고 **인접 단계의 관절 변화가 작아야** 의미가 있다.
    """
    CUBE_CZ, DROP_CZ = 0.015, 0.115          # pick_node와 같은 값 (벽 0.090 + 여유 0.010 + 반폭)
    for i in range(11):
        r = 0.475 + 0.005 * i                # 도착 허용오차 ±25mm
        path, pitch = K.drop_path(r, 0.0, CUBE_CZ, up=DROP_CZ - CUBE_CZ,
                                  from_xy=(0.330, 0.0), from_up=0.180, steps=10)
        assert path is not None, f'통 r={r:.3f}에서 투입 경로 해 없음'
        assert all(K.in_limits(q) for q in path), f'r={r:.3f} 경로에 한계 밖 자세'
        # 끝점에서 **큐브 중심**이 통 중심 위 목표 높이에 오는가 — 이것이 착지 지점이다.
        # grasp_q는 TCP를 큐브 중심에서 GRASP_BACK 뒤·GRASP_DOWN 아래에 두므로 역산한다
        # (y=0이라 접근 방위가 0, 뒤로 뺀 방향이 그대로 -x다).
        tcp = K.fk_pos(path[-1])
        cube_x, cube_z = tcp[0] + K.GRASP_BACK, tcp[2] + K.GRASP_DOWN
        assert abs(cube_x - r) < 1e-3, f'r={r:.3f}: 착지 x {cube_x:.4f} — 개구부 중앙에서 벗어남'
        assert abs(cube_z - DROP_CZ) < 1e-3, f'r={r:.3f}: 착지 높이 {cube_z:.4f}'
        assert cube_z - 0.015 > 0.090, f'r={r:.3f}: 큐브 하단이 통 벽(0.090)보다 낮다'
        # 인접 단계 관절 변화. 관절거리로 균등 재샘플링하므로 고르게 나와야 한다 —
        # 계산상 최대 0.234rad(13도)이고, 0.30을 넘으면 재샘플링이 깨진 것이다.
        # (직교 등간격이던 시절 마지막 단계가 0.72rad였다. 그 회귀를 막는 것이 목적)
        for a, b in zip(path, path[1:]):
            jump = max(abs(x - y) for x, y in zip(a, b))
            assert jump < 0.30, f'r={r:.3f}: 단계 사이 관절이 {jump:.2f}rad 튀었다'


if __name__ == '__main__':
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith('test_') or not callable(fn):
            continue
        try:
            fn()
            print(f'PASS {name}')
        except AssertionError as e:
            fails += 1
            print(f'FAIL {name}\n     {e}')
    print(f'\n{"모두 통과" if not fails else f"{fails}건 실패"}')
    sys.exit(1 if fails else 0)
