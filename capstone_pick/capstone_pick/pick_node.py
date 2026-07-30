"""capstone_pick: 비전 기반 물체 판단 + 캘리브레이션 파지 파이프라인.

아키텍처 (모두 계산 기반, 시뮬 참값 비의존):
  1) Perception  : RGBD 컬러 HSV 검출 → 뎁스+카메라 내부행렬 역투영(광학 프레임)
                   → 광학→링크 축 변환 → TF로 base_footprint 3D 좌표
  2) Approach    : 물체가 파지 포켓 좌표(캘리브레이션 상수)에 오도록 cmd_vel P제어 주행,
                   주행 중 주기적 재인식으로 오차 보정
  3) Grasp       : 접힘(주행) 자세 → 전개 → C자세(pan 조준) → 2단 닫기
  4) Verify      : 그리퍼 조인트 스톨 각도로 파지 성공 판정
                   (물체를 물면 완전 닫힘 각도 -0.17에 도달하지 못함)
  5) Lift        : 저속 2단 들어올리기

캘리브레이션 상수의 출처: 2026-07-28 Gazebo 계측 세션
  - 파지 포켓(base): (0.397, 0.004, z≈0.19) — 죠 스윕 회전행렬 실측
  - 핀치 축은 xy 대각 45°, 고정 손끝 (0.388,0.000) ↔ 움직 손끝 닫힘 (0.406,0.018)
  - 손가락 충돌은 URDF에 추가된 primitive box (트라이메시 관통 문제 해결)
"""
import math
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint

# ---- 캘리브레이션 상수 (실측 기반) ----
POCKET_BASE = (0.410, 0.000)  # 그리드 스캔으로 파지 확정된 포켓 (2026-07-28, lift0.48)      # 파지 포켓 xy [m, base_footprint]
PAN_AXIS_X = 0.159                # shoulder_pan 회전축 x [m]
PAN_BASE_BEARING = math.radians(-1.1)   # pan=0일 때 포켓 방위각
GRIPPER_OPEN = 0.5                # 3cm 물체 통과에 충분한 최소 열림
GRIPPER_STAGE1 = 0.25
GRIPPER_CLOSED = -0.17
GRIP_HOLD_THRESHOLD = -0.05        # 조임 후 각도가 이보다 크면(덜 닫힘) 접촉은 있었다는 뜻
# 물림 판정 구간. 아래로는 허공 닫힘(-0.17)·모서리 헛집기(-0.07)·미닫힘(0.0)을,
# 위로는 열린 상태(0.5+)를 걸러낸다.
# 상한을 0.30까지 좁혀 봤지만(얕은 걸침 0.34대가 들기에서 실패한 관찰 때문) 정상 파지를
# 기각할 위험이 커서 0.40으로 되돌렸다 — 그 실패의 주원인은 물림 깊이가 아니라
# 미세 주행이 죽어 정렬이 어긋난 것이었다(drive 속도 하한 참조).
GRIP_HOLD_MIN, GRIP_HOLD_MAX = 0.05, 0.30
# 쥐고 있는지를 손목캠 블롭 면적으로 판정하는 임계(px). 실측(운반 자세):
# 쥔 상태 36721 / 큐브가 바닥에 떨어진 상태 6079 / 빈손 0. 6배 차이라 중간값보다
# 낮게 잡아도 안전하다. 쥐고 있으면 큐브가 렌즈 앞 몇 cm에 고정되므로 팔 자세가
# 바뀌어도 이 값은 유지된다.
HOLD_AREA_MIN = 15000.0
# 실측 근거: 3cm 큐브 정상 파지는 항상 +0.07 이상, 모서리 헛집기는 -0.07 부근(가짜 성공 사례)
ARM_JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
              'arm_wrist_flex', 'arm_wrist_roll']
# 주행 자세: 추종오차 0.000rad 검증(2026-07-28), 그리퍼가 데크 위로 뜨는 단정한 접힘
POSE_FOLDED = dict(zip(ARM_JOINTS, [0.0, -0.4, 1.0, 0.2, 0.0]))
POSE_DEPLOY = dict(zip(ARM_JOINTS, [0.0, 0.55, 0.2, 0.7, 0.0]))
POSE_PRE = dict(zip(ARM_JOINTS, [0.0, 0.15, 0.2, 0.9, 0.0]))    # 상공 대기
POSE_WAY = dict(zip(ARM_JOINTS, [0.0, 0.28, 0.32, 0.9, 0.0]))   # 수직 하강 경유점(실측)
POSE_GRASP = dict(zip(ARM_JOINTS, [0.0, 0.48, 0.2, 0.9, 0.0]))   # 그립(스캔 파지 확정)
POSE_LIFT1 = {'arm_shoulder_lift': 0.15}
POSE_LIFT2 = {'arm_shoulder_lift': 0.05}
# 바닥 모드: 수직 하향 자세족(lift+elbow+wrist≈1.58) TF 스윕 + 파지 실측으로 확정(2026-07-29).
# 닫힘이 큐브를 고정 죠 쪽으로 쓸어담아 물리는 방식 — 포켓은 쓸림 거리까지 반영한 실측값.
POSE_PRE_FLOOR = dict(zip(ARM_JOINTS, [0.0, 0.85, 0.15, 0.58, 0.0]))    # 바닥 상공 대기
POSE_GRASP_FLOOR = dict(zip(ARM_JOINTS, [0.0, 1.20, 0.15, 0.23, 0.0]))  # 바닥 파지
# 포켓 스윕 실측(2026-07-29): 0.361은 하강이 큐브를 3.5mm 밀어내고, 0.381은 밀림 0에
# 물림각 +0.273으로 가장 깊다(0.401은 얕게 물림 +0.142, 0.421은 놓침).
POCKET_FLOOR = (0.381, 0.000)   # 바닥 모드 파지 포켓 [m, base_footprint]
FLOOR_Z_MAX = 0.08              # 검출 높이가 이보다 낮으면 바닥 모드
# ---- 쓰레기통 투입 (2026-07-29 실측) ----
# 통 = 16cm 정사각, 벽 높이 0.18m, 개구부 13.6cm. 회색이라 색상(H)은 무의미하고
# 무채색(S<45) + 어두움(45<V<115) + 뎁스·높이 게이트로 분리한다(바닥 V=196, 그림자는 z<0).
TRASH_S_MAX, TRASH_V_LO, TRASH_V_HI = 45, 45, 115
TRASH_D_LO, TRASH_D_HI = 0.35, 2.0   # 상한을 넓히면 원경 벽이 통과 한 덩어리로 붙는다(실측)
TRASH_Z_LO, TRASH_Z_HI = 0.02, 0.30
TRASH_MIN_AREA = 400
TRASH_HALF = 0.08               # 뎁스는 앞면 → 중심까지 반깊이 보정
# 운반·투하 자세 = 들어올리기가 끝나는 자세 그대로. 자세를 '전환'하면 팔꿈치가 펴지는
# 관성으로 물체가 빠진다(계단식·저속으로 나눠도 반복 실패). 전환을 없애는 것이 해법이고,
# pan 회전만 하는 것은 옆에 내려놓기에서 이미 검증된 동작이다.
# 실측: 그리퍼 x=0.353 z=0.300 → 큐브 하단 0.205 (통 벽 0.18 위 2.5cm).
# 여유 0.6cm(lift 0.15) 자세로는 주행 흔들림에 큐브가 통 벽에 걸렸다 — 실측 확인.
POSE_CARRY = dict(zip(ARM_JOINTS, [0.0, 0.15, 0.15, 1.28, 0.0]))
# 운반·탐색 중에는 pan을 옆으로 빼 카메라 시야를 연다(정면 자세는 화면 중앙을 가림 — 실측 확인).
POSE_CARRY_SCAN = dict(POSE_CARRY, arm_shoulder_pan=-1.0)
TRASH_POCKET_X = 0.364          # 통 중심이 이 거리에 오면 그리퍼가 개구부 바로 위
# 투하 자세: 운반 자세(x=0.364)로는 통 중심에 9.6cm 못 미쳐 통 앞 바닥에 떨어졌다(실측).
# 로봇 전면(0.275)과 통 벽 때문에 더 접근할 수 없으므로 팔을 뻗어 채운다. 이 자세는
# 그리퍼가 기울어 물체를 놓지만, 이미 통 개구부 위이므로 그대로 투입이 된다.
POSE_DROP = dict(zip(ARM_JOINTS, [0.0, 0.05, -0.30, 0.60, 0.0]))   # 리치 x=0.410
# 손목 카메라 최종 정렬(바닥 모드) — 접근 비전은 근접(<0.45m)에서 팔·시야각에 가려지므로
# 마지막 정렬은 손목 RGB로. 실측(2026-07-29, 바닥 호버): 손목캠은 90° 회전 장착이라
# px=전후거리(82px/cm), py=좌우(67px/cm). pan 1rad당 py -1740px(포켓 반경 0.26m×6700px/m와 일치).
# 아래 세 값은 포켓 0.381에서 직접 실측(2026-07-29). 이전 포켓(0.361) 값을 게인으로
# 환산해 썼더니 기준이 46px 어긋나 정렬이 큐브 모서리로 수렴했다 — 포켓을 옮기면 반드시 다시 잰다.
# 포켓 정위치 큐브의 블롭 중심. 좌우(y) 기준은 "큐브가 로봇 정면(y=0)에 있고 pan=0인
# 상태"를 5프레임 중앙값으로 직접 재서 정한 값이다 — 종전 320.2는 40px 어긋나 있어
# 정렬 루프가 매번 큐브 중심을 벗어난 곳으로 수렴했다(성공/실패가 갈린 주원인).
WRIST_REF = (412.9, 360.6)
WRIST_PX_PER_M = 6073.0         # 전후 1m당 px — 포켓 0.361/0.401 두 점으로 산출
# pan 게인: 호버 자세에서 pan을 -0.1~+0.1로 돌려 잰 값(423.7→168.2px / 0.2rad).
# 이전 값 1740은 과대라 보정이 매번 부족했고, 남은 약 4mm 오차가 죠 한쪽에 치우친
# 얕은 물림을 만들어 들어올리는 첫 순간 미끄러지는 원인이 됐다.
# pan 1rad당 py 변화. 실측하면 상수가 아니라 방향·크기에 따라 568~2905 px/rad로
# 비선형이다(pan -0.10/-0.05/+0.05/+0.10에서 568/757/2905/1736). 상수 하나로 맞출 수
# 없으므로 큰 쪽에 가깝게 잡아 보정을 보수적으로 만든다 — 작게 잡으면 오버슈트해
# 정렬이 진동한다(종전 1277에서 blob y가 215~448로 흔들렸다).
WRIST_PY_PER_PAN = 2000.0
# 큐브 기울기 정렬: 손목캠 minAreaRect 각도는 큐브 yaw와 1:1 반전(실측: +20도→rect 70).
# 카메라는 roll 관절 앞단이라 롤을 돌려도 측정 불변 — 측정·제어 분리.
WRIST_RECT_REF = 90.0           # 정렬 큐브의 rect 각 (실측)
ROLL_SIGN = 1.0                 # 롤 방향 부호 (파지 실험으로 확정)
# 대상 색 HSV 범위 목록 (OpenCV H 0-179) — 2026-07-28 카메라 실측 기반.
# 실측: 병(오렌지) H15-19/S163/V206, 갈색 장애물 H≈13/V≈106(실조명), 노랑 팔 H30-34, 빨간 데크 H0-4.
# 갈색↔orange는 H15+V160 이중 게이트로, 노랑 팔은 H로 분리. 파란 장애물 실린더(H≈107)는
# blue 범위 안 — 최근접 후보 선택·타깃 락·높이 게이트로 회피. red는 H 랩어라운드라 범위 2개,
# 빨간 데크(자기 몸)는 뎁스 게이트(MIN_OBJECT_DEPTH)가 거른다.
TARGET_COLOR_RANGES = {
    'blue': [((100, 130, 100), (135, 255, 255))],
    'red': [((0, 150, 100), (6, 255, 255)), ((174, 150, 100), (179, 255, 255))],
    'green': [((45, 80, 80), (75, 255, 255))],
    'orange': [((15, 120, 160), (22, 255, 255))],
    'pink': [((140, 80, 80), (170, 255, 255))],
}
MIN_OBJECT_DEPTH = 0.30   # 이보다 가까운 검출은 자기 몸(팔)으로 간주하고 제외
MIN_COMPONENT_AREA = 20
OBJECT_HALF_DEPTH = 0.015
APPROACH_STANDOFF = 0.10  # 팔을 먼저 내린 뒤 이 거리만큼 전진 삽입 (하강 충돌 방지)  # 뎁스는 물체 앞표면을 재므로 중심 보정용 반폭 [m]
CAMERA_FRAME = 'rgbd_camera_link'


class PickNode(Node):
    def __init__(self):
        super().__init__('capstone_pick')
        # 속도 배율 (조절: --ros-args -p speed_scale:=1.0 ~ 5.0)
        self.scale = float(self.declare_parameter('speed_scale', 3.0).value)
        self.skip_approach = bool(self.declare_parameter('skip_approach', False).value)
        target_color = str(self.declare_parameter('target_color', 'blue').value).strip().lower()
        if target_color not in TARGET_COLOR_RANGES:
            self.get_logger().error(
                f'미지원 색 "{target_color}" — 사용 가능: {sorted(TARGET_COLOR_RANGES)}')
            raise SystemExit(1)
        self.hsv_ranges = TARGET_COLOR_RANGES[target_color]
        # 바닥 모드: 기본은 비전 검출 높이로 자동 결정, skip_approach 시엔 -p floor:=true로 강제
        self.floor_mode = bool(self.declare_parameter('floor', False).value)
        # 하강 후 죠 안쪽으로 큐브를 넣는 전진량(m). 상공-파지 자세의 그리퍼 x 차이(40mm)가
        # 근거이고, 값은 스윕 실측으로 정했다 — 0/25/43/60mm 중 25·43만 큐브가 실제로 옮겨졌고
        # (0은 제자리, 60은 밀어내 실패), 들기 중 각도 변화가 25mm에서 가장 작았다(+0.030).
        self.creep = float(self.declare_parameter('creep', 0.025).value)
        # 놓을 곳: side(옆 바닥) | trash(쓰레기통 투입)
        self.place_target = str(self.declare_parameter('place_target', 'side').value).strip().lower()
        # 검출기: yolo(기본, 시뮬 자동라벨 파인튜닝 모델) / hsv(폴백·orange·pink용)
        self.detector = str(self.declare_parameter('detector', 'yolo').value).strip().lower()
        self.yolo = None
        if self.detector == 'yolo':
            if target_color not in ('blue', 'red', 'green'):
                self.get_logger().error(
                    f'YOLO 모델은 blue/red/green만 학습됨 — "{target_color}"는 -p detector:=hsv로 실행')
                raise SystemExit(1)
            from ultralytics import YOLO as _YOLO   # 지연 임포트 (hsv 모드에선 불필요)
            model_path = os.path.expanduser('~/capstone_tools/yolo_cubes.pt')
            self.yolo = _YOLO(model_path)
            self.yolo_cls = {v: k for k, v in self.yolo.names.items()}[f'{target_color}_box']
        self.get_logger().info(
            f'speed_scale={self.scale} target_color={target_color} detector={self.detector}')
        self.hold_target = GRIPPER_CLOSED  # 파지 시 최초 접촉각-0.03으로 갱신됨
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.cam_info = None
        self.odom = None
        self.gripper_angle = None
        self.wrist_img = None
        self.create_subscription(Image, 'wrist_camera/image_raw', self._wrist_cb, 1)
        self.create_subscription(Image, 'rgbd_camera/image', self._color_cb, 1)
        self.create_subscription(Image, 'rgbd_camera/depth_image', self._depth_cb, 1)
        self.create_subscription(CameraInfo, 'rgbd_camera/camera_info', self._info_cb, 1)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(JointState, 'joint_states', self._joint_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       'arm_controller/follow_joint_trajectory')
        self.grip_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # ---- 콜백 ----
    def _wrist_cb(self, m):
        self.wrist_img = self.bridge.imgmsg_to_cv2(m, desired_encoding='bgr8')

    def _color_cb(self, m):
        self.color = self.bridge.imgmsg_to_cv2(m, desired_encoding='bgr8')

    def _depth_cb(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m)

    def _info_cb(self, m):
        self.cam_info = m

    def _odom_cb(self, m):
        p = m.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                         1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        self.odom = (p.position.x, p.position.y, yaw)

    def _joint_cb(self, m):
        if 'arm_gripper' in m.name:
            self.gripper_angle = m.position[m.name.index('arm_gripper')]
        self.joint_pos = {n: p for n, p in zip(m.name, m.position) if n in ARM_JOINTS}

    def spin_until(self, pred, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pred():
                return True
        return False

    # ---- 1) Perception ----
    def locate_object(self, timeout=15.0):
        """물체의 base_footprint 3D 좌표를 카메라 계산으로 구한다. 실패 시 None."""
        self.color = self.depth = None
        if not self.spin_until(
                lambda: self.color is not None and self.depth is not None
                and self.cam_info is not None, timeout):
            self.get_logger().error('카메라 토픽 수신 실패')
            return None
        if getattr(self, 'yolo', None) is not None:
            # YOLO 검출 경로: 후보 수집 후 최근접 선택 + 타깃 락 (HSV 경로와 동일 정책)
            cands = self._yolo_candidates(self.depth)
            if not cands:
                return None
            best = min(cands, key=lambda c: c[2])
            prev = getattr(self, '_target_px', None)
            if prev is not None:
                near = [c for c in cands
                        if math.hypot(c[0] - prev[0], c[1] - prev[1]) < 120]
                if near:
                    best = min(near, key=lambda c: math.hypot(c[0] - prev[0], c[1] - prev[1]))
            self._target_px = (best[0], best[1])
            return self._pixel_to_base(*best)
        hsv = cv2.cvtColor(self.color, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.hsv_ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        depth_img = np.asarray(self.depth)
        # 연결 성분별로 뎁스 게이팅: 자기 몸(근접)·무효 뎁스 성분을 걸러내고
        # 남은 후보 중 가장 가까운 것을 대상으로 삼는다.
        num, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
        best = None
        for i in range(1, num):
            if stats[i, cv2.CC_STAT_AREA] < MIN_COMPONENT_AREA:
                continue
            comp = labels == i
            ds = depth_img[comp]
            ds = ds[np.isfinite(ds) & (ds > 0.05)]
            if len(ds) < 15:
                continue
            d_med = float(np.median(ds))
            if d_med < MIN_OBJECT_DEPTH:
                continue
            if best is None or d_med < best[2]:
                best = (cents[i][0], cents[i][1], d_med)
        if best is None:
            return None
        # 타깃 고정: 직전 타깃과 가까운 후보 우선 (프레임 간 타깃 전환 방지)
        prev = getattr(self, '_target_px', None)
        if prev is not None:
            cands = []
            for i in range(1, num):
                if stats[i, cv2.CC_STAT_AREA] < MIN_COMPONENT_AREA:
                    continue
                comp = labels == i
                ds = depth_img[comp]
                ds = ds[np.isfinite(ds) & (ds > 0.05)]
                if len(ds) < 15 or float(np.median(ds)) < MIN_OBJECT_DEPTH:
                    continue
                cands.append((cents[i][0], cents[i][1], float(np.median(ds))))
            near = [c for c in cands
                    if math.hypot(c[0] - prev[0], c[1] - prev[1]) < 120]
            if near:
                best = min(near, key=lambda c: math.hypot(c[0] - prev[0], c[1] - prev[1]))
        self._target_px = (best[0], best[1])
        # 검출 근거 로그: 선택 성분의 실제 픽셀 색 — 어떤 색이 검출을 만들었는지 추적 가능하게
        li = int(labels[int(best[1]), int(best[0])])
        if li > 0:
            hm = hsv[labels == li].mean(axis=0)
            self.get_logger().info(
                f'검출 근거: HSV평균=({hm[0]:.0f},{hm[1]:.0f},{hm[2]:.0f}) '
                f'면적={int(stats[li, cv2.CC_STAT_AREA])}px')
        return self._pixel_to_base(*best)

    def _pixel_to_base(self, u, v, d, z_gate=(-0.05, 0.20)):
        """픽셀+뎁스 → base_footprint 3D (역투영 + 축변환 + TF + 높이 게이트).

        z_gate는 대상별로 다르다 — 바닥 물체는 기본값, 쓰레기통처럼 높은 구조물은 넓혀 준다.
        """
        k = self.cam_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        # 광학 프레임: X=우, Y=하, Z=전방
        ox = (u - cx) * d / fx
        oy = (v - cy) * d / fy
        oz = d
        # 광학 → 링크 프레임(x=전방, y=좌, z=상) 축 변환 — 기존 리포의 좌표 버그 수정 지점
        p = PointStamped()
        p.header.frame_id = CAMERA_FRAME
        p.point.x, p.point.y, p.point.z = oz, -ox, -oy
        if not self.spin_until(
                lambda: self.tf_buffer.can_transform('base_footprint', CAMERA_FRAME,
                                                     rclpy.time.Time()), 5.0):
            self.get_logger().error('TF 대기 실패')
            return None
        tr = self.tf_buffer.lookup_transform('base_footprint', CAMERA_FRAME, rclpy.time.Time())
        out = do_transform_point(p, tr)
        # 물리적 타당성 검증: 대상이 있을 수 있는 높이 범위 안인지
        if not (z_gate[0] < out.point.z < z_gate[1]):
            self.get_logger().warning(
                f'높이 검증 실패 z={out.point.z:.3f} — 오검출로 판단, 무시')
            return None
        return out.point.x, out.point.y, out.point.z, (u, v, d)

    def _yolo_candidates(self, depth_img):
        """YOLO 추론 → 대상 클래스 박스들을 (중심u, 중심v, 뎁스중앙값) 후보로."""
        r = self.yolo.predict(self.color, conf=0.40, verbose=False)[0]
        cands = []
        for b in r.boxes:
            if int(b.cls) != self.yolo_cls:
                continue
            x1, y1, x2, y2 = (max(0, int(t)) for t in b.xyxy[0].tolist())
            region = np.asarray(depth_img)[y1:y2 + 1, x1:x2 + 1]
            ds = region[np.isfinite(region) & (region > 0.05)]
            if len(ds) < 10:
                continue
            d_med = float(np.median(ds))
            if d_med < MIN_OBJECT_DEPTH:
                continue
            self.get_logger().info(
                f'검출 근거(YOLO): {self.yolo.names[int(b.cls)]} conf={float(b.conf):.2f} d={d_med:.2f}')
            cands.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0, d_med))
        return cands

    # ---- 팔/그리퍼 프리미티브 ----
    def move_arm(self, targets, duration):
        duration = max(0.8, duration / self.scale)
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        base = {j: 0.0 for j in ARM_JOINTS}
        cur = getattr(self, '_last_arm', base)
        merged = {**cur, **targets}
        self._last_arm = merged
        pt.positions = [merged[j] for j in ARM_JOINTS]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1) * 1e9)
        goal.trajectory.points = [pt]
        f = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if h is None or not h.accepted:
            return False
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        # 액션 완료 ≠ 도달. 궤적 시간이 끝나면 컨트롤러는 못 따라온 채로도 종료하는데,
        # 그 상태에서 다음 명령이 겹치면 급가속이 생겨 쥔 물체가 빠진다(실측).
        # 관절이 실제로 목표에 닿을 때까지 기다린다.
        return self.wait_arm_settled(merged)

    def wait_arm_settled(self, target, tol=0.04, timeout=6.0):
        """팔 관절이 목표 각도에 실제 도달할 때까지 대기."""
        t0 = time.time()
        worst = None
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            jp = getattr(self, 'joint_pos', None)
            if not jp:
                continue
            worst = max(abs(jp.get(j, target[j]) - target[j]) for j in target)
            if worst < tol:
                return True
        self.get_logger().warning(f'자세 도달 미완 (최대 오차 {worst if worst else float("nan"):.3f}rad)')
        return False

    def gripped(self, angle):
        """물림 판정: 그리퍼 각도가 '물체 두께에 해당하는 구간'에 있는가.

        하한만 두면 아예 닫히지 않은 상태(각도 0 부근)까지 성공으로 새어 나가고
        (실측: 명령 0.0 / 각도 -0.000인데 HOLDING 오판), 명령 대비 잔여각으로 보면
        유지용 재조임이 목표에 도달하는 순간 0이 되어 오판한다. 구간 판정이 둘 다 피한다.
        실측: 3cm 큐브 물림 0.07~0.40 / 허공 완전닫힘 -0.17 / 모서리 헛집기 -0.07 / 열림 0.5+
        """
        return angle is not None and GRIP_HOLD_MIN < angle < GRIP_HOLD_MAX

    def move_gripper(self, position, wait=True, effort=10.0):
        self._last_grip_cmd = float(position)
        if not self.grip_client.wait_for_server(timeout_sec=10.0):
            return False
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = effort  # 파지력. 파고듦 방지는 목표각 캡이 담당
        f = self.grip_client.send_goal_async(g)
        if not wait:
            # 유지 명령은 결과를 기다릴 필요가 없다. 다만 전송 자체는 spin에서
            # 처리되므로 수락까지만 짧게 돌린다(파라미터가 선언만 되고 무시되던 버그).
            rclpy.spin_until_future_complete(self, f, timeout_sec=0.3)
            return True
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if h is None or not h.accepted:
            return False
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=8.0)
        return True

    def drive(self, vx, wz, sec):
        # 기준(1배) 이동량(vx*sec, wz*sec)을 불변으로 유지하며 speed_scale 적용.
        # 속도 상한(0.35, 1.5)에 걸리면 시간을 늘려 보상 — 고배속에서 회전량이 깎여
        # "탐색 로그만 찍히고 실제로는 안 도는" 문제의 근본 수정.
        # 이동량 보존 + 속도 하한. 두 가지 실패를 모두 피해야 한다:
        #   ① 고배속에서 펄스가 짧으면 가속하다 끝나 이동량이 손실된다(명령 50mm에 실제 7mm)
        #   ② 최소 펄스를 길게 두면 미세 조정이 초속 8mm의 너무 느린 명령이 되어
        #      정지 마찰을 못 이기고 아예 안 움직인다(정렬 실패 → 파지가 허공을 집음)
        # 그래서 시간을 고정하지 않고, 속도를 [하한, 상한] 안에 두고 시간으로 이동량을 맞춘다.
        s = min(self.scale, 4.0)          # 주행 배율 상한 (팔 배율은 제한하지 않는다)
        t = Twist()
        dur = sec / s
        if vx:
            v = min(0.35, max(0.06, abs(vx) * s))     # 하한 0.06m/s: 정지 마찰 극복
            dur = abs(vx) * sec / v
            t.linear.x = v if vx > 0 else -v
        if wz:
            w = min(1.5, max(0.25, abs(wz) * s))      # 하한 0.25rad/s
            dur = max(dur, abs(wz) * sec / w)
            t.angular.z = w if wz > 0 else -w
        dur *= 1.2                        # 가감속 손실 보상
        if vx:
            t.linear.x = vx * sec / dur   # 늘어난 시간에 맞춰 속도 재계산 (이동량 불변)
        if wz:
            t.angular.z = wz * sec / dur
        t0 = time.time()
        while time.time() - t0 < dur:
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)

    # ---- 2) Approach: 비전 재인식 폐루프 (odom 기반 스톨 감지 포함) ----
    def approach(self, max_iter=40):
        prev_odom = None
        stall_n = 0
        for it in range(max_iter):
            self.spin_until(lambda: self.odom is not None, 5.0)
            # 스톨 = 2회 연속 무이동일 때만 (짧은 회전은 관성으로 1회 무이동이 정상 — 오판 방지)
            if prev_odom is not None and self.odom is not None:
                moved = math.hypot(self.odom[0] - prev_odom[0], self.odom[1] - prev_odom[1]) \
                    + abs(self.odom[2] - prev_odom[2])
                stall_n = stall_n + 1 if moved < 0.005 else 0
                if stall_n >= 2:
                    self.get_logger().warning(f'[{it}] 스톨 감지(odom 변위 {moved * 1000:.1f}mm) — 후진 회복')
                    self.drive(-0.08, 0.3, 1.5)
                    stall_n = 0
            prev_odom = self.odom
            loc = self.locate_object()
            real = loc is not None
            if not real and getattr(self, '_obj_odom', None) is not None and self.odom:
                # 마지막 관측 위치 추적: odom 기준으로 기억한 물체 방향으로 계속 접근
                ox, oy = self._obj_odom
                x, y, yaw = self.odom
                dx, dy = ox - x, oy - y
                xb = math.cos(yaw) * dx + math.sin(yaw) * dy
                yb = -math.sin(yaw) * dx + math.cos(yaw) * dy
                zb, dbg = 0.0, (0, 0, 0)
                if not getattr(self, '_arm_aside', False):
                    # 접힌 팔이 화면 하단 중앙(근접 물체 위치)을 가림 — 물체 반대쪽으로 젖혀 시야 확보.
                    # 방향은 순간 방위(진동 튐)가 아니라 추정 좌표의 좌우 부호로.
                    self._arm_aside = True
                    aside = 0.6 if yb > 0 else -0.6
                    self.get_logger().info(f'근접 시야 확보: 팔을 물체 반대쪽으로 (pan {aside})')
                    self.move_arm({'arm_shoulder_pan': aside}, 1.2)
                    prev_odom = None  # 의도적 정지 — 다음 반복 스톨 오판 방지
                    continue
                self.get_logger().info(f'[{it}] 추정 추적: base=({xb:.3f},{yb:.3f})')
            elif not real:
                # 카메라는 전방 ~0.9m 바닥만 본다(pitch 0.9rad) — 회전과 전진을 섞어 탐색
                self._miss = getattr(self, '_miss', 0) + 1
                if self._miss == 1:
                    self.get_logger().warning(f'[{it}] 물체 미검출 — 한 발 후진(근접 사각 확인)')
                    self.drive(-0.14, 0.0, 2.5)
                    continue
                if self._miss % 14 == 0:
                    # 한 바퀴(13회전×약27°) 스윕을 마친 뒤에만 전진 — 교대·중간전진은 스윕을 상쇄시킨다
                    self.get_logger().warning(f'[{it}] 물체 미검출 — 전진 탐색(0.25m)')
                    self.drive(0.10, 0.0, 2.5)
                else:
                    self.get_logger().warning(f'[{it}] 물체 미검출 — 회전 탐색(한 바퀴 스윕)')
                    self.drive(0.0, 0.4, 1.2)
                continue
            else:
                self._miss = 0
                self._last_seen = it
                xb, yb, zb, dbg = loc
                if zb > 0.001 and not getattr(self, '_mode_locked', False):
                    # 첫 실검출의 높이로 받침대/바닥 모드 결정 (이후 고정)
                    self.floor_mode = zb < FLOOR_Z_MAX
                    self._mode_locked = True
                    if self.floor_mode:
                        self.get_logger().info(f'바닥 모드 진입 (검출 z={zb:.3f})')
                if self.odom:
                    x, y, yaw = self.odom
                    self._obj_odom = (x + math.cos(yaw) * xb - math.sin(yaw) * yb,
                                      y + math.sin(yaw) * xb + math.cos(yaw) * yb)
            pocket = POCKET_FLOOR if self.floor_mode else POCKET_BASE
            r_target = math.hypot(pocket[0] - PAN_AXIS_X, pocket[1])
            r = math.hypot(xb - PAN_AXIS_X, yb)
            brg = math.atan2(yb, xb - PAN_AXIS_X)
            if real:
                self._last_brg = brg  # 팔 젖힘 방향 결정용 (물체가 좌/우 어느 쪽인지)
            # 뎁스=앞표면 → 물체 중심이 포켓에 오도록 앞표면은 반폭만큼 안쪽에
            er = r - (r_target - OBJECT_HALF_DEPTH)  # 물체 중심이 포켓에 오도록 (삽입 없음)
            self.get_logger().info(
                f'[{it}] 비전: base=({xb:.3f},{yb:.3f},{zb:.3f}) px=({dbg[0]:.0f},{dbg[1]:.0f}) '
                f'd={dbg[2]:.2f} | er={er * 1000:.0f}mm brg={math.degrees(brg):.1f}deg')
            # 바닥 모드는 ±30mm면 합격 — 잔여 오차는 손목캠 정렬(pan+미세주행)이 마무리.
            # 좁은 공차로 범퍼 코앞에서 전후 왕복하다 물체를 미는 것 방지.
            tol = 0.030 if self.floor_mode else 0.012
            if abs(er) < tol and abs(brg) < 0.30 and (real or it - getattr(self, "_last_seen", -99) <= 8):
                if not real and not getattr(self, '_reacq_done', False):
                    # 추측항법만으로 수렴 — 정지 상태에서 한 번 재관측 (주행 중 비전 상실 드리프트 보정)
                    self._reacq_done = True
                    self.get_logger().info('수렴(추측항법) — 정지 재관측 시도')
                    time.sleep(1.0)
                    prev_odom = None  # 의도적 정지 — 다음 반복 스톨 오판 방지
                    continue
                self._anchor_odom = self.odom
                return -(brg - PAN_BASE_BEARING)
            if abs(brg) > 0.30:
                # 맹회전은 저속으로 (고속 회전 슬립이 yaw 추정을 무너뜨려 지그재그 발진)
                wz = 0.25 if real else 0.10
                self.drive(0.0, wz if brg > 0 else -wz, min(2.0, abs(brg) / wz))
            else:
                # 맹구간 직진 위주 — 잔여 방위는 파지 단계의 pan 회전과 손목캠 정렬이 흡수
                v = 0.08 if er > 0.2 else (0.04 if er > 0 else -0.04)
                wz = max(-0.15, min(0.15, brg * 0.8)) if real else 0.0
                self.drive(v, wz, min(3.0, abs(er) / abs(v) + 0.2))
        return None

    # ---- 손목 카메라 최종 정렬 (바닥 모드): 좌우=pan, 전후=미세 주행 ----
    def wrist_align(self, pan, pre):
        """호버 자세에서 손목 RGB로 물체 중간이 파지선(WRIST_REF)에 오도록 정렬한다.

        순서가 중요하다: 기울기(roll)를 먼저 맞추고 그 자세에서 위치를 맞춘다.
        위치를 먼저 맞추고 roll을 돌리면 그리퍼가 회전하면서 파지 중심이 함께 이동해
        맞춰둔 위치가 어긋난다(물체 중점을 벗어나 얕게 물림).
        손목캠은 roll 관절 앞단에 있어 롤을 돌려도 측정 기준이 변하지 않으므로,
        roll을 먼저 적용해도 이후 위치 측정은 그대로 유효하다(실측 확인).
        """
        roll = 0.0
        ang = self._wrist_cube_angle()
        if ang is not None and abs(ang) > 0.06:
            roll = max(-0.8, min(0.8, ROLL_SIGN * ang))
            self.get_logger().info(f'큐브 기울기 {math.degrees(ang):+.0f}도 → 롤 먼저 정렬 {roll:+.2f}rad')
            self.move_arm({**pre, 'arm_shoulder_pan': pan, 'arm_wrist_roll': roll}, 1.5)
            time.sleep(0.4)

        for i in range(6):     # 임계를 좁힌 만큼 반복 여유를 준다
            best = self._wrist_blob()
            if best is None:
                self.get_logger().info('손목캠: 물체 미검출 — 정렬 생략')
                break
            dr = (best[0] - WRIST_REF[0]) / WRIST_PX_PER_M     # 전후 오차 [m] (+=멂)
            # 게인이 실측으로 정확해진 뒤에는 감쇠를 줄여 빠르게 수렴시킨다(0.85)
            dpan = 0.85 * (best[1] - WRIST_REF[1]) / WRIST_PY_PER_PAN
            self.get_logger().info(
                f'손목캠 정렬[{i}]: blob=({best[0]:.0f},{best[1]:.0f}) '
                f'전후 {dr * 1000:+.0f}mm, pan 보정 {dpan:+.3f}rad')
            # 좌우 임계 0.006rad ≈ 포켓 반경 0.222m에서 1.3mm — 죠 중앙에 물리려면 이 수준이어야 한다
            if abs(dr) < 0.006 and abs(dpan) < 0.006:
                break
            if abs(dpan) >= 0.012:
                pan = max(-0.6, min(0.6, pan + max(-0.25, min(0.25, dpan))))
                # roll을 유지한 채 pan만 조정 (roll을 빼면 정렬 기준이 다시 어긋난다)
                self.move_arm({**pre, 'arm_shoulder_pan': pan, 'arm_wrist_roll': roll}, 1.2)
                time.sleep(0.3)
            if abs(dr) >= 0.006:
                step = max(-0.05, min(0.05, dr))
                self.drive(0.03 if step > 0 else -0.03, 0.0, abs(step) / 0.03)
        return pan, roll

    def _wrist_blob(self, frames=5):
        """손목캠에서 가장 큰 색 블롭의 중심을 여러 프레임의 중앙값으로 구한다.

        단일 프레임으로 재면 좌우 좌표가 ±100px 흔들린다(실측: 기준 320인데 215~413).
        그리퍼 손가락이 큐브를 부분적으로 가리고 마스크 경계가 프레임마다 달라지기
        때문이다. 그 노이즈는 pan 약 ±0.08rad에 해당해 정렬 루프가 수렴하지 못하고,
        마지막 반복에서 우연히 작은 값이 나오면 성공하고 크면 얕게 물었다.
        중앙값은 튀는 프레임을 버려 이 운을 없앤다.
        """
        xs, ys, areas = [], [], []
        for _ in range(frames):
            self.wrist_img = None
            if not self.spin_until(lambda: self.wrist_img is not None, 3.0):
                break
            hsv = cv2.cvtColor(self.wrist_img, cv2.COLOR_BGR2HSV)
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in self.hsv_ranges:
                mask |= cv2.inRange(hsv, lo, hi)
            num, _, stats, cents = cv2.connectedComponentsWithStats(mask)
            best = None
            for j in range(1, num):
                a = stats[j, cv2.CC_STAT_AREA]
                if a > 100 and (best is None or a > best[2]):
                    best = (cents[j][0], cents[j][1], a)
            if best is not None:
                xs.append(best[0])
                ys.append(best[1])
                areas.append(best[2])
        if len(xs) < 3:      # 과반이 안 잡히면 측정으로 쓰지 않는다
            return None
        return (float(np.median(xs)), float(np.median(ys)), float(np.median(areas)))

    def _wrist_cube_angle(self):
        """손목캠 마스크의 최소면적사각형으로 큐브 yaw[rad] 추정 (90도 대칭 랩)."""
        self.wrist_img = None
        if not self.spin_until(lambda: self.wrist_img is not None, 3.0):
            return None
        hsv = cv2.cvtColor(self.wrist_img, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.hsv_ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        best = None
        for i in range(1, num):
            a = stats[i, cv2.CC_STAT_AREA]
            if a > 300 and (best is None or a > best[1]):
                best = (i, a)
        if best is None:
            return None
        pts = np.column_stack(np.where(labels == best[0])[::-1]).astype(np.float32)
        rect_ang = cv2.minAreaRect(pts)[2]
        delta = ((rect_ang - WRIST_RECT_REF + 45.0) % 90.0) - 45.0  # [-45,45)
        return -math.radians(delta)  # 실측: 이미지 각 = 큐브 yaw의 반전

    # ---- 3~5) Grasp / Verify / Lift ----
    def grasp(self, pan):
        pre = POSE_PRE_FLOOR if self.floor_mode else POSE_PRE
        grasp_pose = POSE_GRASP_FLOOR if self.floor_mode else POSE_GRASP
        self.move_arm({**pre, 'arm_shoulder_pan': pan}, 3.0)
        self.get_logger().info('그리퍼 열기 (물체 근처 도착)')
        self.move_gripper(1.2)
        time.sleep(0.5)
        # 팔 스윙 반동으로 베이스가 밀리므로 odom 변위만큼 되돌린다
        if getattr(self, '_anchor_odom', None) and self.odom:
            x0, y0, yaw0 = self._anchor_odom
            self.spin_until(lambda: self.odom is not None, 3.0)
            x1, y1, _ = self.odom
            fwd = math.cos(yaw0) * (x1 - x0) + math.sin(yaw0) * (y1 - y0)
            self.get_logger().info(f'스윙 드리프트 보정: {fwd * 1000:.0f}mm 후진')
            if abs(fwd) > 0.005:
                v = -0.03 if fwd > 0 else 0.03
                self.drive(v, 0.0, min(2.5, abs(fwd) / 0.03 + 0.1))
        roll = 0.0
        if self.floor_mode:
            # 근접 비전 사각을 손목 RGB로 보완: 물체 중간이 파지선에 오도록 pan 정렬 + 기울기 롤 정렬
            pan, roll = self.wrist_align(pan, pre)
        self.get_logger().info('수직 하강' + (' (바닥 모드)' if self.floor_mode else ''))
        descended = {**grasp_pose, 'arm_shoulder_pan': pan, 'arm_wrist_roll': roll}
        self.move_arm(descended, 3.0)
        # 하강이 끝나기 전에 닫으면 큐브 상단을 스치며 얕게 물거나 밀어낸다.
        # 궤적 시간(3초)으로는 이 자세에 도달하지 못한다(실측: 3.8초에 lift 1.17/1.20).
        self.wait_arm_settled(descended, timeout=8.0)
        # 하강은 수직이 아니다: 상공 자세와 파지 자세의 그리퍼 x가 40mm 차이 난다
        # (실측 0.378 → 0.338). 상공에서 큐브 중심에 맞춰 놓아도 내려오면 그만큼 뒤로
        # 물러나 큐브가 죠 끝단에만 걸리고, 들다가 미끄러진다. 열린 죠로 그 차이만큼
        # 전진해 큐브를 죠 안쪽까지 넣는다. 파지 자세에서는 큐브가 손목캠 시야를
        # 벗어나므로(실측: 포켓 0.381 검출 불가) 여기서는 비전 대신 실측 거리를 쓴다.
        if self.floor_mode and self.creep > 0.001:
            self.get_logger().info(f'죠 안쪽으로 밀어 넣기: {self.creep * 1000:.0f}mm 전진')
            self.drive(0.03, 0.0, self.creep / 0.03)
            time.sleep(0.3)
        angle = None
        for attempt in range(3):
            self.get_logger().info(f'그리퍼 닫기 (시도 {attempt + 1})')
            self.move_gripper(GRIPPER_CLOSED)
            time.sleep(1.0)
            self.gripper_angle = None
            self.spin_until(lambda: self.gripper_angle is not None, 5.0)
            angle = self.gripper_angle
            if self.gripped(angle):
                # 파고듦 방지: 최초 접촉각-0.01을 절대 바닥으로 고정 (이후 재조임도 이 값만 사용)
                self.hold_target = max(GRIPPER_CLOSED, angle - 0.01)
                # 유지력은 닫을 때보다 높게. 닫는 순간 effort를 올리면 강체 큐브를
                # 튕겨내지만(과거 실측), 이미 문 뒤의 유지는 다르다 — effort 10으로는
                # 들기 하중에 죠가 밀려 벌어졌다(실측: 0.184로 물고 첫 단계에 0.287).
                self.move_gripper(self.hold_target, effort=30.0)
                time.sleep(0.3)
                break
            if angle is not None and angle >= GRIP_HOLD_MAX:
                # 얕게 걸친 상태 — 큐브가 죠 사이가 아니라 죠 앞에 있어 죠가 큐브 위로
                # 올라탄 것이다(실측: 0.347/0.349/0.350이 반복되고 전부 제자리).
                # 물러나면 더 멀어지고, 상공으로 올려 다시 내려오면 40mm 후퇴가
                # 되풀이된다. 파지 자세를 유지한 채 열고 더 밀어 넣는다.
                # 전진량은 1cm까지만: 2cm로 두 번 밀었더니 큐브가 4cm 밀려나
                # 허공을 물었다(실측 -0.170).
                self.get_logger().info(f'얕은 걸침(각도={angle:.3f}) — 더 밀어 넣어 재물림')
                self.move_gripper(1.0)
                time.sleep(0.4)
                self.drive(0.02, 0.0, 0.5)
                time.sleep(0.3)
                continue
            self.get_logger().info(f'얕은 물림(각도={angle}) — 재물림')
            self.move_gripper(0.5)
            time.sleep(0.6)
        held = self.gripped(angle)
        self.get_logger().info(
            f'파지 검증: 그리퍼 각도={angle if angle is not None else float("nan"):.3f} '
            f'(임계 {GRIP_HOLD_THRESHOLD}) → {"HOLDING" if held else "EMPTY"}')
        if held and not self.floor_mode:
            # 바닥 모드는 자세 급변(원-모션 상승) 없이 곧장 계단 들기로 (실검증 방식)
            self.get_logger().info('수직 상승')
            self.move_arm({**pre, 'arm_shoulder_pan': pan}, 2.5)
            time.sleep(0.3)
        return held

    # ---- 쓰레기통 검출·접근·투입 ----
    def locate_trash(self):
        """쓰레기통 중심의 base_footprint 좌표. 실패 시 None."""
        self.color = self.depth = None
        if not self.spin_until(
                lambda: self.color is not None and self.depth is not None
                and self.cam_info is not None, 5.0):
            return None
        hsv = cv2.cvtColor(self.color, cv2.COLOR_BGR2HSV)
        dep = np.nan_to_num(np.asarray(self.depth), nan=0.0, posinf=0.0)
        mask = ((hsv[:, :, 1] < TRASH_S_MAX) & (hsv[:, :, 2] > TRASH_V_LO)
                & (hsv[:, :, 2] < TRASH_V_HI) & (dep > TRASH_D_LO)
                & (dep < TRASH_D_HI)).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        num, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
        best, rejected = None, []
        for i in range(1, num):
            a = int(stats[i, cv2.CC_STAT_AREA])
            if a < TRASH_MIN_AREA:
                continue
            ds = dep[labels == i]
            ds = ds[(ds > TRASH_D_LO) & (ds < TRASH_D_HI)]
            if len(ds) < 50:
                rejected.append(f'면적{a}:뎁스부족')
                continue
            # 통은 높이 0.18m 구조물 — 물체용 기본 게이트(0.20)로는 상단이 걸린다
            loc = self._pixel_to_base(cents[i][0], cents[i][1], float(np.median(ds)),
                                      z_gate=(-0.10, 0.60))
            if loc is None:
                rejected.append(f'면적{a}:역투영실패')
                continue
            x, y, z = loc[0], loc[1], loc[2]
            if not (TRASH_Z_LO < z < TRASH_Z_HI):
                rejected.append(f'면적{a}:높이{z:.2f}')
                continue          # 바닥 그림자(z<0)·원경 구조물 배제
            if best is None or a > best[0]:
                best = (a, x, y, z)
        if best is None:
            if rejected:
                self.get_logger().info('통 후보 기각: ' + ', '.join(rejected[:4]))
            return None
        a, x, y, z = best
        # 뎁스는 앞면 → 중심 방향으로 반깊이만큼 밀어 통 중심을 추정
        r = math.hypot(x, y)
        k = (r + TRASH_HALF) / r if r > 1e-3 else 1.0
        self.get_logger().info(
            f'쓰레기통: 앞면 base=({x:.3f},{y:.3f},{z:.3f}) 면적={a} → 중심 r={r + TRASH_HALF:.3f}')
        return x * k, y * k

    def carry_to_trash(self, max_iter=45):
        """물체를 든 채 쓰레기통 앞까지 저속 주행. 성공 시 True."""
        # 팔은 들어올린 자세 그대로 손대지 않는다. 자세 전환은 물론 pan 회전만으로도
        # 얕게 물린 물체가 빠진다(실측 반복). 통은 큰 구조물이라 팔 위쪽 시야로 원거리에서
        # 검출되고, 근접해 가려지는 구간은 근접 락(추측 접근)이 담당한다.
        if not self.holding():
            self.get_logger().error('운반 시작 시 물체 없음')
            return False
        # 운반 구간은 배율을 1로 — 회전 관성만으로도 얕게 물린 물체가 빠진다(실측)
        carry_scale, self.scale = self.scale, 1.0
        try:
            return self._carry_loop(max_iter)
        finally:
            self.scale = carry_scale

    def _carry_loop(self, max_iter):
        miss = 0
        for it in range(max_iter):
            self.spin_until(lambda: self.odom is not None, 3.0)
            # 근접 락: 통이 화면을 채우면 마스크 중심이 한쪽 면으로 치우쳐 방위가 수렴하지 않는다
            # (실측: r=0.50에서 brg -14도 고착). 락 이후에는 기억한 좌표로만 추측 접근한다.
            near_lock = getattr(self, '_trash_lock', False)
            loc = None if near_lock else self.locate_trash()
            if loc is not None and self.odom:
                # 통은 고정물 — 실검출마다 오도메트리 좌표로 기억해 두고 근접 사각에서 재사용
                x, y, yaw = self.odom
                self._trash_odom = (x + math.cos(yaw) * loc[0] - math.sin(yaw) * loc[1],
                                    y + math.sin(yaw) * loc[0] + math.cos(yaw) * loc[1])
            elif loc is None and getattr(self, '_trash_odom', None) and self.odom:
                ox, oy = self._trash_odom
                x, y, yaw = self.odom
                dx, dy = ox - x, oy - y
                loc = (math.cos(yaw) * dx + math.sin(yaw) * dy,
                       -math.sin(yaw) * dx + math.cos(yaw) * dy)
                self.get_logger().info(f'[{it}] 통 추정 추적: base=({loc[0]:.3f},{loc[1]:.3f})')
            if loc is None:
                miss += 1
                if miss > 12:
                    self.get_logger().error('쓰레기통 미검출 — 탐색 실패')
                    return False
                self.get_logger().warning(f'[{it}] 쓰레기통 미검출 — 회전 탐색')
                self.drive(0.0, 0.30, 1.2)
                time.sleep(0.4)      # 회전 직후 흔들림이 가라앉은 뒤 관측
                continue
            miss = 0
            tx, ty = loc
            r = math.hypot(tx, ty)
            brg = math.atan2(ty, tx)
            er = r - TRASH_POCKET_X
            if not near_lock and r < 0.60 and getattr(self, '_trash_odom', None):
                self._trash_lock = True     # 이 시점의 좌표를 기준으로 고정
                self.get_logger().info(f'근접 락 (r={r:.3f}) — 이후 추측 접근')
            self.get_logger().info(
                f'[{it}] 통 접근: r={r:.3f} er={er * 1000:.0f}mm brg={math.degrees(brg):.1f}deg')
            # 개구부 13.6cm 기준 거리 0.4m에서 허용 방위는 약 9.6도 — 임계를 그 안쪽으로 둔다.
            # 좁게 잡으면 회전만 반복하다 전진을 못 한다(실측: 3.4도 임계에서 예산 소진).
            if abs(er) < 0.025 and abs(brg) < 0.10:
                return True
            if not self.holding():
                self.get_logger().error('운반 중 물체 놓침')
                return False
            if abs(brg) > 0.25:
                self.drive(0.0, 0.20 if brg > 0 else -0.20, min(2.0, abs(brg) / 0.20))
            else:
                v = 0.06 if er > 0.15 else (0.03 if er > 0 else -0.03)
                wz = max(-0.12, min(0.12, brg * 0.7))     # 전진하며 방위도 함께 좁힌다
                self.drive(v, wz, min(3.0, abs(er) / abs(v) + 0.2))
        self.get_logger().error('쓰레기통 접근 반복 소진')
        return False

    def to_pose_holding(self, target, steps=5):
        """물체를 쥔 채 목표 자세로 계단식 전환.

        한 번에 바꾸면 팔꿈치가 펴지는 관성으로 물체가 밀려 빠진다(실측). 들어올리기와
        같은 방식으로 잘게 나누고 단계마다 쥔 힘을 재주장한다.
        """
        cur = dict(getattr(self, '_last_arm', {j: 0.0 for j in ARM_JOINTS}))
        for i in range(1, steps + 1):
            t = i / steps
            pose = {j: cur.get(j, 0.0) + (target[j] - cur.get(j, 0.0)) * t for j in ARM_JOINTS}
            self.move_arm(pose, 1.0 * self.scale)     # 배율 무시: 실제 1초씩
            time.sleep(0.5)
            # 재주장하지 않는다 — 명령을 다시 보내면 죠가 움직여 물체가 덜렁거린다.
        return self.holding()

    def holding(self):
        """물체를 쥐고 있는지 손목캠으로 판정 — 죠를 다시 움직이지 않는다.

        각도로는 낙하를 볼 수 없다. hold_target(접촉각-0.01)을 명령해 두면 물체가
        빠져도 그리퍼가 그 각도를 유지하기 때문이다(가짜 성공의 원인). 살짝 더 조여
        보는 능동 확인은 그 동작 자체가 물체를 밀어내고 운반 중 죠를 떨게 만들었다.
        손목캠은 둘 다 피한다 — 쥐고 있으면 큐브가 렌즈 앞에 고정돼 화면을 크게 채우고,
        떨어지면 바닥으로 멀어져 면적이 6분의 1로 준다(실측 36721 대 6079).
        """
        self.gripper_angle = None
        self.spin_until(lambda: self.gripper_angle is not None, 3.0)
        ang = self.gripper_angle
        roll = float(getattr(self, '_last_arm', {}).get('arm_wrist_roll', 0.0) or 0.0)
        if abs(roll) >= 0.1:
            # 기울기 정렬로 손목이 돌아간 상태에서는 큐브가 손목캠 시야를 벗어난다
            # (실측: roll 0.33에서 면적 0인데 실제로는 쥐고 있었다 — 큐브 z=0.19).
            # 이때는 각도 구간으로 폴백한다. 각도는 낙하를 못 잡을 수 있으므로
            # 기울어진 물체에서는 판정 신뢰도가 낮다는 한계가 남는다.
            held = self.gripped(ang)
            self.get_logger().info(
                f'파지 확인(각도 폴백, roll={roll:+.2f}): '
                f'각도={ang if ang is None else round(ang, 3)} → {"HOLDING" if held else "DROPPED"}')
            return held
        b = self._wrist_blob(frames=3)
        area = b[2] if b else 0.0
        held = area >= HOLD_AREA_MIN
        self.get_logger().info(
            f'파지 확인: 손목캠 면적={area:.0f} (임계 {HOLD_AREA_MIN:.0f}) '
            f'각도={ang if ang is None else round(ang, 3)} → {"HOLDING" if held else "DROPPED"}')
        return held

    def drop_into_trash(self):
        """통 개구부 위에서 그리퍼를 열어 투입."""
        time.sleep(0.5)
        if not self.holding():
            self.get_logger().error('투입 직전 물체 없음')
            return False
        # 통 중심까지 팔을 뻗는다. 이 자세는 그리퍼가 기울어 물체가 스스로 빠질 수 있는데,
        # 이미 개구부 위이므로 그것도 투입이다. 그래서 파지 유지를 확인하지 않는다.
        self.get_logger().info('통 중심으로 팔 뻗기')
        self.move_arm(POSE_DROP, 2.0 * self.scale)
        time.sleep(0.8)
        self.get_logger().info('쓰레기통 위 — 그리퍼 열기')
        self.move_gripper(1.0)
        time.sleep(1.2)
        self.move_arm(POSE_FOLDED, 3.0)     # 팔을 접어 통에서 빠져나옴
        time.sleep(0.5)
        return True

    def place(self, pan_target=-0.7):
        """잡은 물체를 옆(pan_target 방향)에 내려놓고 팔을 접는다."""
        grasp_pose = POSE_GRASP_FLOOR if self.floor_mode else POSE_GRASP
        self.move_arm({'arm_shoulder_pan': pan_target}, 2.5)
        time.sleep(0.3)
        self.move_arm({**grasp_pose, 'arm_shoulder_pan': pan_target}, 2.5)
        time.sleep(0.3)
        self.move_gripper(0.8)
        time.sleep(0.6)
        self.move_arm({**POSE_PRE, 'arm_shoulder_pan': pan_target}, 2.0)
        self.move_arm(POSE_FOLDED, 2.5)
        return True

    def lift(self):
        # 계단식 들기 + wrist 보상(lift 감소분 = wrist 증가분): 그리퍼 절대 피치 유지.
        # 시작 자세는 모드의 파지 자세 — 받침대(0.48/0.9), 바닥(1.20/0.23) 공용.
        grasp_pose = POSE_GRASP_FLOOR if self.floor_mode else POSE_GRASP
        lift0, wrist0 = grasp_pose['arm_shoulder_lift'], grasp_pose['arm_wrist_flex']
        # 물체가 바닥을 떠나는 첫 순간에 하중이 걸려 가장 잘 미끄러진다 — 초반 두 단계를
        # 잘게 올려 대응한다. 그리퍼는 파지 때 정한 목표를 그대로 유지한다(재조임 없음).
        lf, k = lift0, 0
        while lf > 0.151:
            step = 0.02 if k < 2 else 0.07      # 초반만 미세하게
            lf = max(0.15, lf - step)
            wf = wrist0 + (lift0 - lf)
            self.move_arm({'arm_shoulder_lift': lf, 'arm_wrist_flex': wf}, 1.5)
            time.sleep(0.2)
            # 파지 때 정한 절대 목표를 그대로 다시 주장한다. 값이 같으므로 죠는
            # 움직이지 않고(덜렁거림 없음), 하중에 밀려 벌어졌을 때만 되돌아온다.
            # 과거 파고듦의 원인은 "현재각 기준"으로 다시 조이던 래칫이었고,
            # 절대 목표 반복은 그 문제가 없다.
            # 완료를 기다리지 않는다 — 계단 수만큼 액션 왕복을 기다리면 전체 실행이
            # 400초 예산을 넘겨 중단됐다(실측). 유지 명령은 도착만 하면 된다.
            self.move_gripper(self.hold_target, wait=False, effort=30.0)
            self.gripper_angle = None
            self.spin_until(lambda: self.gripper_angle is not None, 3.0)
            self.get_logger().info(f'  step lift={lf:.2f}: 그리퍼 각도={self.gripper_angle:.3f}')
            k += 1
        time.sleep(0.5)
        still = self.holding()
        self.get_logger().info(f'들기 후 재검증 → {"HOLDING" if still else "DROPPED"}')
        return still


def main(args=None):
    rclpy.init(args=args)
    n = PickNode()
    ok = False
    try:
        n.get_logger().info('== 1. 초기화: 접힘 자세 ==')
        n.move_gripper(0.0)
        n.move_arm(POSE_FOLDED, 3.0)
        # 파지 실패 시 자동 재시도: 후진해 비전 재획득 후 재접근 (맹주행 드리프트 회복)
        for cycle in range(3):
            if n.skip_approach and cycle == 0:
                n.get_logger().info('== 2. 접근 생략 (물체 앞 가정, pan=0) ==')
                pan = 0.0
                n.spin_until(lambda: n.odom is not None, 5.0)
                n._anchor_odom = n.odom
            else:
                n.get_logger().info(f'== 2. 비전 접근 주행 (사이클 {cycle + 1}) ==')
                pan = n.approach()
            if pan is None:
                n.get_logger().error('접근 실패')
                if cycle < 2:
                    for attr in ('_reacq_done', '_arm_aside', '_miss', '_obj_odom', '_target_px'):
                        if hasattr(n, attr):
                            delattr(n, attr)
                    n.move_arm(POSE_FOLDED, 2.5)
                    continue
                break
            n.get_logger().info(f'== 3. 파지 (pan={math.degrees(pan):.1f}deg) ==')
            held = n.grasp(pan)
            if held:
                n.get_logger().info('== 4. 들어올리기 ==')
                ok = n.lift()
                if ok and n.place_target == 'trash':
                    n.get_logger().info('== 5. 쓰레기통으로 운반 ==')
                    ok = n.carry_to_trash() and n.drop_into_trash()
                elif ok:
                    n.get_logger().info('== 5. 옮겨 놓기 (place) ==')
                    n.place()
                break
            n.get_logger().error('파지 실패 (그리퍼 완전 닫힘 = 허공)')
            if cycle < 2:
                n.get_logger().info('재시도: 그리퍼 열고 후진 → 재접근')
                n.move_gripper(0.5)
                n.move_arm(POSE_FOLDED, 2.5)
                n.drive(-0.12, 0.0, 3.0)
                for attr in ('_reacq_done', '_arm_aside', '_miss', '_obj_odom', '_target_px'):
                    if hasattr(n, attr):
                        delattr(n, attr)
        n.get_logger().info('=== PICK_SUCCESS ===' if ok else '=== PICK_FAIL ===')
    finally:
        n.destroy_node()
        if rclpy.ok():      # 중단 신호로 이미 내려간 뒤 다시 부르면 RCLError가 난다
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
