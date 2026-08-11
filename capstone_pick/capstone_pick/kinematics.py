"""SO-101 팔의 순기구학·역기구학. ROS에 의존하지 않는다 — 순수 기하 계산이다.

왜 따로 두나. 지금까지는 역기구학을 풀지 않고 "팔은 고정 자세 몇 개만 쓰고 차체를
움직여 물체를 고정된 한 점(포켓)에 갖다 놓는" 구조였다. 그 대가로 주행 정밀도가
곧 파지 정밀도가 됐고, 부족분을 손목캠 픽셀 기준점으로 메우다 보니 서로 무효화하는
상수가 스무 개 넘게 얽혔다(포켓 좌표·WRIST_REF·GRASP_REF·px/m·리치 계수).
표준 구조는 그 반대다 — **차체는 대략 세우고 팔이 오차를 흡수한다.**

ROS를 안 쓰므로 시뮬 없이 단위 테스트로 검증된다. 그게 이 파일의 핵심 이점이다.

좌표 규약은 URDF와 같다: base_footprint 기준 x=전방, y=좌, z=상.

체인 (jdamr_cube.urdf에서 추출):
    base_footprint
      └ base_joint(고정)        xyz [0, 0, 0.075]
      └ arm_riser_joint(고정)   xyz [0.12, 0, 0.075]
      └ arm_mount_joint(고정)   xyz [0, 0, 0]
      └ arm_shoulder_pan        xyz [0.0388353, 0, 0.0624]   rpy [pi, 0, -pi]
      └ arm_shoulder_lift       xyz [-0.0303992, -0.0182778, -0.0542]  rpy [-pi/2, -pi/2, 0]
      └ arm_elbow_flex          xyz [-0.11257, -0.028, 0]    rpy [0, 0, pi/2]
      └ arm_wrist_flex          xyz [-0.1349, 0.0052, 0]     rpy [0, 0, -pi/2]
      └ arm_wrist_roll          xyz [0, -0.0611, 0.0181]     rpy [pi/2, 0.0486795, pi]
      └ arm_gripper_frame_joint(고정) xyz [-0.0079, -0.000218, -0.0981274] rpy [0, pi, 0]

`arm_wrist_roll`까지 적용한 프레임이 **아랫턱**(arm_gripper_link)이고, 거기서
고정 변환을 한 번 더 하면 **TCP**(arm_gripper_frame_link)다. 사용자 기준인
"큐브가 아랫턱에 거의 닿고 죠 중점에"를 확인하려면 두 프레임이 모두 필요하다.

상수는 URDF에서 뽑아 여기 박았다. URDF가 바뀌면 `test_kinematics.py`가 대조해
불일치를 잡는다 — 런타임에 URDF를 파싱하지 않으므로 이 파일은 의존성이 없다.
"""
import math

import numpy as np

# 관절 순서는 pick_node.ARM_JOINTS와 같다
JOINT_NAMES = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
               'arm_wrist_flex', 'arm_wrist_roll']

# (xyz, rpy) — 각 관절 프레임을 부모에 놓는 고정 변환. 회전은 그 뒤 axis(0,0,1)로.
LINKS = [
    ((0.0, 0.0, 0.075), (0.0, 0.0, 0.0)),                    # base_joint
    ((0.12, 0.0, 0.075), (0.0, 0.0, 0.0)),                   # arm_riser_joint
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),                      # arm_mount_joint
]
JOINTS = [
    ((0.0388353, 0.0, 0.0624), (math.pi, 0.0, -math.pi)),                  # pan
    ((-0.0303992, -0.0182778, -0.0542), (-math.pi / 2, -math.pi / 2, 0.0)),  # lift
    ((-0.11257, -0.028, 0.0), (0.0, 0.0, math.pi / 2)),                    # elbow
    ((-0.1349, 0.0052, 0.0), (0.0, 0.0, -math.pi / 2)),                    # wrist_flex
    ((0.0, -0.0611, 0.0181), (math.pi / 2, 0.0486795, math.pi)),           # wrist_roll
]
# arm_wrist_roll의 child(=아랫턱)에서 TCP까지
TCP_OFFSET = ((-0.0079, -0.000218121, -0.0981274), (0.0, math.pi, 0.0))

# 관절 한계 [rad] (URDF limit)
LIMITS = [(-1.91986, 1.91986), (-1.74533, 1.74533), (-1.69, 1.69),
          (-1.65806, 1.65806), (-2.74385, 2.84121)]


def rpy_to_mat(r, p, y):
    """고정축 RPY → 회전행렬. URDF 규약은 R = Rz(y) @ Ry(p) @ Rx(r)."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def xform(xyz, rpy):
    """4x4 동차변환."""
    t = np.eye(4)
    t[:3, :3] = rpy_to_mat(*rpy)
    t[:3, 3] = xyz
    return t


def rot_z(q):
    """관절 회전. 모든 관절의 axis가 (0,0,1)이라 z축 회전 하나로 끝난다."""
    c, s = math.cos(q), math.sin(q)
    t = np.eye(4)
    t[0, 0], t[0, 1] = c, -s
    t[1, 0], t[1, 1] = s, c
    return t


def fk(q, upto=None):
    """관절각 q(5개) → base_footprint 기준 4x4 자세.

    upto=None이면 TCP(arm_gripper_frame_link), upto='jaw'면 아랫턱
    (arm_gripper_link), upto=k(정수)면 k번째 관절 프레임까지.
    """
    t = np.eye(4)
    for xyz, rpy in LINKS:
        t = t @ xform(xyz, rpy)
    n = len(JOINTS) if upto in (None, 'jaw') else int(upto)
    for i in range(n):
        xyz, rpy = JOINTS[i]
        t = t @ xform(xyz, rpy) @ rot_z(q[i])
    if upto is None:
        t = t @ xform(*TCP_OFFSET)
    return t


def fk_pos(q, upto=None):
    """위치만 (x, y, z)."""
    return fk(q, upto)[:3, 3]


def in_limits(q, margin=0.0):
    """모든 관절이 한계 안인가."""
    return all(lo + margin <= v <= hi - margin for v, (lo, hi) in zip(q, LIMITS))


def clamp_to_limits(q):
    return [max(lo, min(hi, v)) for v, (lo, hi) in zip(q, LIMITS)]


# ---------------------------------------------------------------------------
# 역기구학
#
# 구조가 교과서적으로 분해된다(FK로 확인):
#   · lift·elbow·wrist_flex 축이 모두 [0,1,0]으로 **서로 평행** → 평면 2R+1
#   · pan=0에서 TCP y가 정확히 0 → 팔이 평면 안, 옆 오프셋 없음
#   · pan을 돌리면 반경·높이가 보존되고 방위만 바뀐다 (실측 오차 0.00mm)
#   · 방위 = -pan  (pan +1.0rad → 방위 -57.3도)
#
# 그래서 5축이 이렇게 나뉜다:
#   pan          → 방위 하나
#   lift·elbow·wrist_flex → 평면 안의 (반경, 높이, 죠 피치) 셋
#   roll         → 죠 회전 (큐브 yaw)
# 미지수 5개에 조건 5개라 정확히 결정된다. 위에서 내려잡는 파지에는 충분하다.
#
# 평면 파라미터는 하드코딩하지 않고 FK에서 유도한다 — URDF가 바뀌면 자동으로 따라온다.
# ---------------------------------------------------------------------------

def _planar_params():
    """평면 2R+1의 링크 길이와 오프셋 각을 FK에서 뽑는다.

    반환: (pan축 x, lift축 (r,z), (L1,b1), (L2,b2), (L3,b3), 죠피치 상수)
      Lk = 링크 길이, bk = q=0일 때 그 링크의 평면 각 [rad]
      관절이 +q 돌면 평면 각은 -q 만큼 돈다(부호가 반대다 — FK로 확인).
    """
    pan_x = float(fk([0.0] * 5, upto=1)[0, 3])
    def rz(q, upto):
        p = fk(q)[:3, 3] if upto == 'tcp' else fk(q, upto=upto)[:3, 3]
        return float(p[0]) - pan_x, float(p[2])
    z0 = [0.0] * 5
    # `fk(q, upto=i)`는 관절을 i개 적용한 프레임이다. 회전은 원점을 안 움직이므로
    # upto=k의 원점이 곧 **k번째 관절의 원점**이다 (upto=2 → lift, 3 → elbow …).
    # 하나씩 밀려 쓰면 링크 길이가 자세에 따라 변하는 것처럼 보인다(그렇게 틀렸었다).
    p_lift = rz(z0, 2)
    p_elb = rz(z0, 3)
    p_wf = rz(z0, 4)
    p_tcp = rz(z0, 'tcp')

    def seg(a, b):
        d = (b[0] - a[0], b[1] - a[1])
        return math.hypot(*d), math.atan2(d[1], d[0])

    # 작업평면은 pan축을 지나지 않는다 — lift·elbow·wrist 원점이 y로 18mm 벗어나 있다
    # (어깨 오프셋). 이걸 무시하고 r을 'pan축까지의 수평거리'로 잡으면 5mm씩 어긋난다.
    y_off = float(fk(z0, upto=2)[1, 3])

    l1, b1 = seg(p_lift, p_elb)
    l2, b2 = seg(p_elb, p_wf)
    l3, b3 = seg(p_wf, p_tcp)
    # IK는 **아랫턱 원점**을 목표로 푼다(roll 축 위라 roll과 무관). 그 세그먼트도 잰다.
    p_jaw = rz(z0, 'jaw')
    l3j, b3j = seg(p_wf, p_jaw)
    # 죠 피치 = -(lift+elbow+wrist) + pitch0 — 아랫턱→TCP 방향으로 정의
    j = fk(z0, upto='jaw')[:3, 3]
    t = fk(z0)[:3, 3]
    pitch0 = math.atan2(t[2] - j[2], t[0] - j[0])
    return pan_x, y_off, p_lift, (l1, b1), (l2, b2), (l3, b3), (l3j, b3j), pitch0


PAN_X, Y_OFF, LIFT_RZ, (L1, B1), (L2, B2), (L3, B3), (L3J, B3J), PITCH0 = _planar_params()
# 죠가 수직 아래를 향하는 (lift+elbow+wrist) 값. 종전 코드의 '자세족 불변식 1.58'이 이것이다.
SUM_VERTICAL = PITCH0 + math.pi / 2


def jaw_pitch(q):
    """관절각 → 죠 피치[rad] (평면 내 접근 방향, -pi/2가 수직 하향)."""
    return PITCH0 - (q[1] + q[2] + q[3])


def _jaw_rot(pan, s, roll):
    """아랫턱 프레임의 회전. lift·elbow·wrist 축이 평행이라 **합 s에만** 의존한다."""
    return fk([pan, s, 0.0, 0.0, roll], upto='jaw')[:3, :3]


def ik(x, y, z, pitch=-math.pi / 2, roll=0.0, elbow_up=True):
    """TCP 목표 자세 → 관절각 5개. 해가 없으면 None.

    x, y, z : base_footprint 기준 TCP 위치 [m]
    pitch   : 죠 피치 [rad]. 기본 -pi/2 = 수직 하향(위에서 내려잡기)
    roll    : 죠 회전 [rad]. 큐브 yaw를 여기에 넣는다
    elbow_up: 2R의 두 해 중 어느 쪽을 고를지

    평면 문제로 바꿔 코사인 법칙으로 푼다. 수치 반복이 없어 결정적이고 빠르다.

    **roll 보정이 필요한 이유**: TCP는 roll 축 위에 있지 않다(`TCP_OFFSET`의 x가
    -0.0079). 그래서 roll을 돌리면 TCP가 반경 7.9mm 원을 그린다 — 최대 15.8mm다.
    파지 정밀도가 mm 단위라 무시할 수 없다. 아랫턱 **원점**은 roll 축 위에 있어
    roll과 무관하므로, 목표 TCP에서 그 오프셋을 빼 아랫턱 목표로 바꾼 뒤 푼다.
    pan이 아랫턱 위치에 의존하고 그 위치가 다시 pan에 의존하는 순환이 있는데,
    오프셋이 작아 두 번 돌리면 마이크로미터 수준으로 수렴한다.
    """
    s = PITCH0 - pitch                      # 세 관절의 합은 죠 피치가 정한다
    tcp_off = np.array(TCP_OFFSET[0])
    wf_off = np.array(JOINTS[4][0])         # wrist_flex → 아랫턱(roll 원점) 고정 오프셋

    def solve_pan(px, py):
        """목표점이 작업평면 안에 오도록 pan을 정한다. (pan, 평면 내 반경).

        작업평면이 pan축에서 Y_OFF만큼 옆에 있어 단순 방위가 아니다 — 수평거리 R에
        대해 asin(Y_OFF/R)만큼 보정해야 평면이 목표점을 지난다(어깨 오프셋 처리).
        """
        R = math.hypot(px - PAN_X, py)
        if R < abs(Y_OFF) + 1e-9:
            return None, None
        brg = math.atan2(py, px - PAN_X)
        return -(brg - math.asin(Y_OFF / R)), math.sqrt(R * R - Y_OFF * Y_OFF)

    # 1) TCP 목표 → 아랫턱 원점 → **wrist_flex 원점**으로 두 번 역산한다.
    #    평면 안에 있는 것은 wrist_flex까지다. 아랫턱은 roll 관절의 오프셋 때문에
    #    평면 밖(y≈0)으로 나와 있어 2R의 목표로 쓸 수 없다.
    #    pan이 위치에 의존하고 위치가 다시 pan에 의존하는 순환은 두 번이면 수렴한다.
    pan = -math.atan2(y, x - PAN_X)
    r = wz = 0.0
    for _ in range(3):
        jaw = np.array([x, y, z]) - _jaw_rot(pan, s, roll) @ tcp_off
        r_wf = fk([pan, s, 0.0, 0.0, 0.0], upto=4)[:3, :3]
        wf = jaw - r_wf @ wf_off
        pan, r = solve_pan(float(wf[0]), float(wf[1]))
        if pan is None:
            return None
        wz = float(wf[2])

    # 2) lift축 → wrist_flex 를 2R로 푼다
    wr = r
    ur, uz = wr - LIFT_RZ[0], wz - LIFT_RZ[1]
    d = math.hypot(ur, uz)
    if d > L1 + L2 or d < abs(L1 - L2):
        return None                      # 작업공간 밖
    # 코사인 법칙으로 첫 링크의 평면 각만 구한다.
    base_a = math.atan2(uz, ur)
    cos_off = (d * d + L1 * L1 - L2 * L2) / (2 * d * L1)
    cos_off = max(-1.0, min(1.0, cos_off))
    sign = 1.0 if elbow_up else -1.0
    a1 = base_a + sign * math.acos(cos_off)      # 첫 링크의 평면 각
    # 둘째 링크 각은 공식으로 유도하지 않고 팔꿈치 위치에서 **직접** 잰다.
    # (pi - 내각)으로 유도하면 형상에 따라 부호가 뒤집혀 조용히 틀린 해가 나온다
    # — 격자 검증에서 148mm 오차로 잡혔다.
    er_ = LIFT_RZ[0] + L1 * math.cos(a1)
    ez_ = LIFT_RZ[1] + L1 * math.sin(a1)
    a2 = math.atan2(wz - ez_, wr - er_)          # 둘째 링크의 평면 각

    # 5) 평면 각 → 관절각. 링크 각 = 오프셋 - (그때까지의 관절 합)
    q_lift = B1 - a1
    q_elbow = B2 - a2 - q_lift
    q_wrist = s - q_lift - q_elbow
    q = [pan, q_lift, q_elbow, q_wrist, roll]
    if not in_limits(q):
        return None
    # **자체 검증.** 해석해의 분기(elbow_up)가 형상에 따라 반대쪽에 착지하는 경우가
    # 있다(격자 검증에서 100mm 오차로 잡혔다). 여기서 걸러 두면 ik_best가 다른
    # 분기로 넘어가고, 둘 다 실패하면 정직하게 None이 된다.
    if float(np.linalg.norm(fk_pos(q) - np.array([x, y, z]))) > 1e-6:
        return None
    return q


def ik_best(x, y, z, pitch=-math.pi / 2, roll=0.0):
    """두 팔꿈치 해 중 한계를 만족하는 쪽을 고른다. 둘 다 되면 위쪽."""
    for up in (True, False):
        q = ik(x, y, z, pitch, roll, elbow_up=up)
        if q is not None:
            return q
    return None


# ---------------------------------------------------------------------------
# 파지 자세 생성
#
# IK로 옮겨도 **죠 기하는 실측이 필요하다.** 죠가 큐브를 감쌀 때 TCP가 큐브 중심에서
# 어디에 있어야 하는지는 메시 형상의 문제라 URDF 수치로 계산되지 않는다.
# 다만 종전처럼 스무 개가 아니라 **세 개**로 줄었고, 서로 무효화하지 않는다.
#
# 근거(2026-08-10 실측): POSE_GRASP_FLOOR [0, 1.15, 0.15, 0.28, 0] 자세에서
# 큐브 x=0.385~0.425가 물렸다(grasp_ref_scan.py, 들림을 실좌표로 확인).
# 그 자세의 TCP는 FK로 (0.3352, 0, -0.0007), 큐브 중앙은 0.405, z=0.02이므로:
# ---------------------------------------------------------------------------

# 2026-08-10 재산정. 종전 0.0698은 "POSE_GRASP_FLOOR로 큐브 0.405가 물렸다"는
# 실측에서 역산한 값인데, 그 자세는 리치 보정이 섞인 상태였고 큐브도 죠에 **끼어**
# 있었다(화면 확인, 물림각 0.24~0.30 — 4cm 정상 물림은 0.55 부근).
#
# URDF 메시(wrist_roll_follower / moving_jaw STL)의 정점으로 정확히 계산했다.
# 큐브가 있는 높이대(아랫턱 로컬 z -0.097~-0.057)에서 두 죠의 간격은
#   열림 q=1.2 → 86.1mm(중앙 x=+0.0352) / q=0.6 → 42.5mm(+0.0134) / q=0 → 15.8mm(0)
# 이라 **닫히면서 파지 중앙이 이동한다.** 4cm 큐브가 물리는 q≈0.545에서 중앙은
# x=+0.0121이고, 큐브를 열림 기준 중앙에 두면 그 차이만큼 닫는 동안 밀려난다.
#   실측 대조(밀림): back 30/45/60/70mm → 10.4/24.9/40.1/49.7mm
#   모델 예측      →  9.8/24.8/39.7/49.7mm   (0.6mm 이내 일치)
# 큐브 로컬 x = GRASP_BACK - 0.0081 이므로 목표 x=+0.0121에 맞추면:
GRASP_BACK = 0.0161     # 큐브 중심에서 접근 방향 **반대로** 이만큼 뒤에 TCP를 둔다
# 30mm 큐브 기준(2026-08-11). 죠 끝(TCP)을 바닥 살짝 위(z=+0.002)에 두는 것이
# 한계다 — 더 내리면 죠가 바닥을 파고든다.
#
# **크기를 40→20mm로 줄인 이유**: 죠를 최대한 내려도 열린 윗턱의 아래끝은
# 아랫턱 로컬 z=-0.0643까지만 내려온다. 40mm 큐브의 윗면은 -0.0601이라
# 윗턱이 큐브 **위에** 걸리고, 닫으면 감싸는 대신 위에서 누른다(화면에서 반복 확인).
#   큐브 윗면 대비 윗턱 여유: 40mm -4.2mm / 30mm +5.8 / 25mm +10.8 / 20mm +15.8
# 받침대로 띄우면 윗면이 더 올라가 오히려 불리하다. 이 그리퍼로 바닥 물체를
# 위에서 잡으려면 작을수록 유리하다.
#
# 30mm가 물리는 각은 q≈0.25, 그때 파지 중앙은 아랫턱 로컬 x=+0.0080이다.
# 크기를 20 → 25 → 30mm로 올린 근거는 **물림각과 완전 닫힘(-0.17)의 거리**다.
#   20mm q≈0.05 (여유 0.22) / 25mm q≈0.15 (0.32) / 30mm q≈0.25 (0.42)
# 25mm는 실측 물림각이 0.099~0.108로 계산(0.15)보다 낮게 나왔고, 드는 순간
# 하중이 걸리자 그 여유가 사라져 놓쳤다. 윗턱 여유는 30mm에서도 +5.8mm로 남는다.
GRASP_DOWN = 0.0130     # 큐브 중심보다 이만큼 아래
GRASP_PITCH = -1.66035  # 죠 피치 [rad] — 실측으로 물린 자세의 값(수직보다 5도 숙임)
PREGRASP_UP = 0.08      # 프리그래스프 높이 [m]
# 하강은 큐브 **밖**에서 한다. 파지 위치(GRASP_BACK)는 큐브 반폭보다 작아서
# 그대로 내리면 죠가 큐브 윗면을 찌른다 — 30mm 큐브의 반폭이 15mm인데
# GRASP_BACK이 14.9mm라 TCP가 큐브 뒷면 바로 위에 놓인다(화면 확인).
# 그래서 이만큼 더 뒤에서 내린 다음, 바닥 높이에서 수평으로 밀어 넣는다.
APPROACH_BACK = 0.038   # 하강할 때의 back [m] — 큐브 반폭 + 여유


def grasp_q(cx, cy, cz=0.015, cube_yaw=0.0, up=0.0, pitch=GRASP_PITCH, roll_sign=1.0):
    """큐브 중심(base 좌표) → 파지 관절각. 해가 없으면 None.

    up: 이만큼 위에서 잡는 자세(프리그래스프용). 0이면 파지 높이.
    cube_yaw: 큐브의 yaw[rad]. 죠를 나란히 맞추도록 roll에 싣는다.
    roll_sign: roll 부호 규약 — 실측으로 확정한다(부호는 죠 형상에 달렸다).
    """
    brg = math.atan2(cy, cx - PAN_X)          # 죠가 향하는 방향
    tx = cx - GRASP_BACK * math.cos(brg)
    ty = cy - GRASP_BACK * math.sin(brg)
    tz = cz - GRASP_DOWN + up
    # 죠가 물체와 나란해지도록 roll에 싣는다. 부호는 **실측으로 확정**했다 —
    # 큐브 yaw=0을 (0.35,±0.05)에 놓고 세 조건을 비교한 결과:
    #   roll = +(cube_yaw+brg) → 둘 다 물림 / -(그것) → 둘 다 허공 / roll=0 → 둘 다 물림
    # 기하로도 맞다: pan 회전으로 죠 폭 방향이 brg만큼 돌아가므로 그만큼 실어야
    # 물체 면과 나란해진다.
    # 정육면체는 90도 대칭이라 ±45도 안으로 접어 넣는다(덜 돌수록 안전).
    rel = cube_yaw + brg
    rel = (rel + math.pi / 4) % (math.pi / 2) - math.pi / 4
    return ik_best(tx, ty, tz, pitch=pitch, roll=roll_sign * rel)


def jaw_dir(q):
    """죠가 향하는 수평 방향 [rad] — 아랫턱→TCP 벡터의 xy 성분."""
    j = fk_pos(q, upto='jaw')
    t = fk_pos(q)
    return math.atan2(t[1] - j[1], t[0] - j[0])


def descend_path(cx, cy, cz=0.015, cube_yaw=0.0, steps=4, insert=3, **kw):
    """큐브 밖에서 수직으로 내린 뒤 수평으로 밀어 넣는 경로의 관절각 목록.

    두 구간으로 나뉜다.
      1) 하강 — back=APPROACH_BACK(큐브 밖)을 유지하며 프리그래스프에서 파지 높이까지
      2) 삽입 — 같은 높이에서 back을 GRASP_BACK까지 줄여 죠 안으로 큐브를 넣는다

    한 번에 내리면 죠가 큐브 윗면을 찌른다(GRASP_BACK이 큐브 반폭보다 작아서).
    관절 보간의 호로 내리지 않는 이유는 그대로다 — 죠가 앞뒤로 쓸며 큐브를 쳐낸다.
    IK로는 두 구간 모두 직선을 직접 만들 수 있고, 보정이 없으니 쌓일 것도 없다.
    """
    global GRASP_BACK
    keep = GRASP_BACK
    out = []
    try:
        GRASP_BACK = APPROACH_BACK
        for i in range(steps + 1):                      # 1) 큐브 밖에서 수직 하강
            q = grasp_q(cx, cy, cz, cube_yaw,
                        up=PREGRASP_UP * (1.0 - i / steps), **kw)
            if q is None:
                return None
            out.append(q)
        for j in range(1, insert + 1):                  # 2) 같은 높이에서 수평 삽입
            GRASP_BACK = APPROACH_BACK + (keep - APPROACH_BACK) * (j / insert)
            q = grasp_q(cx, cy, cz, cube_yaw, up=0.0, **kw)
            if q is None:
                return None
            out.append(q)
    finally:
        GRASP_BACK = keep
    return out
