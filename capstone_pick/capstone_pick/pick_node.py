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
GRIP_HOLD_THRESHOLD = -0.05        # 조임 후 각도가 이보다 크면(덜 닫힘) 파지 성공
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
POCKET_FLOOR = (0.361, 0.000)   # 바닥 모드 파지 포켓 [m, base_footprint]
FLOOR_Z_MAX = 0.08              # 검출 높이가 이보다 낮으면 바닥 모드
# 손목 카메라 최종 정렬(바닥 모드) — 접근 비전은 근접(<0.45m)에서 팔·시야각에 가려지므로
# 마지막 정렬은 손목 RGB로. 실측(2026-07-29, 바닥 호버): 손목캠은 90° 회전 장착이라
# px=전후거리(82px/cm), py=좌우(67px/cm). pan 1rad당 py -1740px(포켓 반경 0.26m×6700px/m와 일치).
WRIST_REF = (296.0, 329.0)      # 포켓 정위치 큐브의 블롭 중심
WRIST_PX_PER_M = 8175.0         # 전후 1m당 px (멀수록 px 증가)
WRIST_PY_PER_PAN = 1740.0       # pan -1rad당 py 증가 (Δpan = py오차/1740)
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
        self.get_logger().info(f'speed_scale={self.scale} target_color={target_color}')
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
        u, v, d = best
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
        # 물리적 타당성 검증: 물체는 지면 근처 높이여야 한다
        if not (-0.05 < out.point.z < 0.20):
            self.get_logger().warning(
                f'높이 검증 실패 z={out.point.z:.3f} — 오검출로 판단, 무시')
            return None
        return out.point.x, out.point.y, out.point.z, (u, v, d)

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
        return True

    def move_gripper(self, position, wait=True):
        if not self.grip_client.wait_for_server(timeout_sec=10.0):
            return False
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = 10.0  # 파지력 복원 — 파고듦 방지는 목표각 캡(스톨각-0.03)이 담당
        f = self.grip_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if h is None or not h.accepted:
            return False
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=8.0)
        return True

    def drive(self, vx, wz, sec):
        # 기준(1배) 속도·시간에 speed_scale 적용: 속도 ×scale, 시간 ÷scale (이동량 불변)
        s = self.scale
        t = Twist()
        t.linear.x = max(-0.35, min(0.35, vx * s))
        t.angular.z = max(-1.5, min(1.5, wz * s))
        t0 = time.time()
        while time.time() - t0 < sec / s:
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
        """호버 자세에서 손목 RGB로 물체 중간이 파지선(WRIST_REF)에 오도록 정렬한다."""
        for i in range(4):
            self.wrist_img = None
            if not self.spin_until(lambda: self.wrist_img is not None, 3.0):
                break
            hsv = cv2.cvtColor(self.wrist_img, cv2.COLOR_BGR2HSV)
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in self.hsv_ranges:
                mask |= cv2.inRange(hsv, lo, hi)
            num, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
            best = None
            for j in range(1, num):
                a = stats[j, cv2.CC_STAT_AREA]
                if a > 100 and (best is None or a > best[2]):
                    best = (cents[j][0], cents[j][1], a)
            if best is None:
                self.get_logger().info('손목캠: 물체 미검출 — 정렬 생략')
                break
            dr = (best[0] - WRIST_REF[0]) / WRIST_PX_PER_M     # 전후 오차 [m] (+=멂)
            dpan = (best[1] - WRIST_REF[1]) / WRIST_PY_PER_PAN  # 좌우 오차 → pan 보정 [rad]
            self.get_logger().info(
                f'손목캠 정렬[{i}]: blob=({best[0]:.0f},{best[1]:.0f}) '
                f'전후 {dr * 1000:+.0f}mm, pan 보정 {dpan:+.3f}rad')
            if abs(dr) < 0.006 and abs(dpan) < 0.012:
                break
            if abs(dpan) >= 0.012:
                pan = max(-0.6, min(0.6, pan + max(-0.25, min(0.25, dpan))))
                self.move_arm({**pre, 'arm_shoulder_pan': pan}, 1.2)
                time.sleep(0.3)
            if abs(dr) >= 0.006:
                step = max(-0.05, min(0.05, dr))
                self.drive(0.03 if step > 0 else -0.03, 0.0, abs(step) / 0.03)
        return pan

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
        if self.floor_mode:
            # 근접 비전 사각을 손목 RGB로 보완: 물체 중간이 파지선에 오도록 pan 정렬
            pan = self.wrist_align(pan, pre)
        self.get_logger().info('수직 하강' + (' (바닥 모드)' if self.floor_mode else ''))
        self.move_arm({**grasp_pose, 'arm_shoulder_pan': pan}, 3.0)
        time.sleep(0.3)
        angle = None
        for attempt in range(3):
            self.get_logger().info(f'그리퍼 닫기 (시도 {attempt + 1})')
            self.move_gripper(GRIPPER_CLOSED)
            time.sleep(1.0)
            self.gripper_angle = None
            self.spin_until(lambda: self.gripper_angle is not None, 5.0)
            angle = self.gripper_angle
            if angle is not None and angle > -0.10:
                # 파고듦 방지: 최초 접촉각-0.01을 절대 바닥으로 고정 (이후 재조임도 이 값만 사용)
                self.hold_target = max(GRIPPER_CLOSED, angle - 0.01)
                self.move_gripper(self.hold_target)
                time.sleep(0.3)
                break
            self.get_logger().info(f'얕은 물림(각도={angle}) — 재물림')
            self.move_gripper(0.5)
            time.sleep(0.6)
        held = angle is not None and angle > GRIP_HOLD_THRESHOLD
        self.get_logger().info(
            f'파지 검증: 그리퍼 각도={angle if angle is not None else float("nan"):.3f} '
            f'(임계 {GRIP_HOLD_THRESHOLD}) → {"HOLDING" if held else "EMPTY"}')
        if held and not self.floor_mode:
            # 바닥 모드는 자세 급변(원-모션 상승) 없이 곧장 계단 들기로 (실검증 방식)
            self.get_logger().info('수직 상승')
            self.move_arm({**pre, 'arm_shoulder_pan': pan}, 2.5)
            time.sleep(0.3)
        return held

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
        lf, k = lift0, 0
        while lf > 0.151:
            lf = max(0.15, lf - 0.07)
            wf = wrist0 + (lift0 - lf)
            self.move_arm({'arm_shoulder_lift': lf, 'arm_wrist_flex': wf}, 1.5)
            time.sleep(0.2)
            self.gripper_angle = None
            self.spin_until(lambda: self.gripper_angle is not None, 3.0)
            self.get_logger().info(f'  step lift={lf:.2f}: 그리퍼 각도={self.gripper_angle:.3f}')
            if k % 2 == 0:
                # 재조임 = 최초 접촉각 기준 절대 목표 재주장 (현재각 기준 래칫 파고듦 방지)
                self.move_gripper(self.hold_target)
            k += 1
        time.sleep(0.5)
        self.gripper_angle = None
        self.spin_until(lambda: self.gripper_angle is not None, 5.0)
        angle = self.gripper_angle
        still = angle is not None and angle > GRIP_HOLD_THRESHOLD
        self.get_logger().info(
            f'들기 후 재검증: 각도={angle if angle is not None else float("nan"):.3f} '
            f'→ {"HOLDING" if still else "DROPPED"}')
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
                if ok:
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
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
