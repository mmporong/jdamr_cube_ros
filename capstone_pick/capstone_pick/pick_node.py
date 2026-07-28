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
GRIP_HOLD_THRESHOLD = -0.155       # 조임 후 각도가 이보다 크면(덜 닫힘) 파지 성공
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
# 대상 물체 HSV 범위 — 2026-07-28 카메라 실측: 병 H15-19/S163/V206.
# 빨간 데크(H0-4)·노랑 팔(H30-34)은 H로, 갈색 장애물(V≈106)은 V≥160으로 분리.
HSV_LOWER, HSV_UPPER = (100, 130, 100), (135, 255, 255)  # 파란 큐브
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
        self.get_logger().info(f'speed_scale={self.scale}')
        self.hold_target = GRIPPER_CLOSED  # 파지 시 최초 접촉각-0.03으로 갱신됨
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.cam_info = None
        self.odom = None
        self.gripper_angle = None
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
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
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
    def approach(self, max_iter=24):
        r_target = math.hypot(POCKET_BASE[0] - PAN_AXIS_X, POCKET_BASE[1])
        prev_odom = None
        for it in range(max_iter):
            self.spin_until(lambda: self.odom is not None, 5.0)
            if prev_odom is not None and self.odom is not None:
                moved = math.hypot(self.odom[0] - prev_odom[0], self.odom[1] - prev_odom[1]) \
                    + abs(self.odom[2] - prev_odom[2])
                if moved < 0.005:
                    self.get_logger().warning(f'[{it}] 스톨 감지(odom 변위 {moved * 1000:.1f}mm) — 후진 회복')
                    self.drive(-0.08, 0.3, 1.5)
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
                self.get_logger().info(f'[{it}] 추정 추적: base=({xb:.3f},{yb:.3f})')
            elif not real:
                # 카메라는 전방 ~0.9m 바닥만 본다(pitch 0.9rad) — 회전과 전진을 섞어 탐색
                self._miss = getattr(self, '_miss', 0) + 1
                if self._miss == 1:
                    self.get_logger().warning(f'[{it}] 물체 미검출 — 한 발 후진(근접 사각 확인)')
                    self.drive(-0.14, 0.0, 2.5)
                    continue
                if self._miss % 5 == 4:
                    self.get_logger().warning(f'[{it}] 물체 미검출 — 전진 탐색(0.25m)')
                    self.drive(0.10, 0.0, 2.5)
                else:
                    self.get_logger().warning(f'[{it}] 물체 미검출 — 회전 탐색')
                    self.drive(0.0, 0.4, 1.2)
                continue
            else:
                self._miss = 0
                self._last_seen = it
                xb, yb, zb, dbg = loc
                if self.odom:
                    x, y, yaw = self.odom
                    self._obj_odom = (x + math.cos(yaw) * xb - math.sin(yaw) * yb,
                                      y + math.sin(yaw) * xb + math.cos(yaw) * yb)
            r = math.hypot(xb - PAN_AXIS_X, yb)
            brg = math.atan2(yb, xb - PAN_AXIS_X)
            # 뎁스=앞표면 → 물체 중심이 포켓에 오도록 앞표면은 반폭만큼 안쪽에
            er = r - (r_target - OBJECT_HALF_DEPTH)  # 물체 중심이 포켓에 오도록 (삽입 없음)
            self.get_logger().info(
                f'[{it}] 비전: base=({xb:.3f},{yb:.3f},{zb:.3f}) px=({dbg[0]:.0f},{dbg[1]:.0f}) '
                f'd={dbg[2]:.2f} | er={er * 1000:.0f}mm brg={math.degrees(brg):.1f}deg')
            if abs(er) < 0.012 and abs(brg) < 0.30 and (real or it - getattr(self, "_last_seen", -99) <= 8):
                self._anchor_odom = self.odom
                return -(brg - PAN_BASE_BEARING)
            if abs(brg) > 0.30:
                self.drive(0.0, 0.25 if brg > 0 else -0.25, min(2.0, abs(brg) / 0.25))
            else:
                v = 0.08 if er > 0.2 else (0.04 if er > 0 else -0.04)
                self.drive(v, max(-0.15, min(0.15, brg * 0.8)),
                           min(3.0, abs(er) / abs(v) + 0.2))
        return None

    # ---- 3~5) Grasp / Verify / Lift ----
    def grasp(self, pan):
        self.move_arm({**POSE_PRE, 'arm_shoulder_pan': pan}, 3.0)
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
        self.get_logger().info('수직 하강')
        self.move_arm({**POSE_GRASP, 'arm_shoulder_pan': pan}, 3.0)
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
        if held:
            self.get_logger().info('수직 상승')
            self.move_arm({**POSE_PRE, 'arm_shoulder_pan': pan}, 2.5)
            time.sleep(0.3)
        return held

    def place(self, pan_target=-0.7):
        """잡은 물체를 옆(pan_target 방향)에 내려놓고 팔을 접는다."""
        self.move_arm({'arm_shoulder_pan': pan_target}, 2.5)
        time.sleep(0.3)
        self.move_arm({**POSE_GRASP, 'arm_shoulder_pan': pan_target}, 2.5)
        time.sleep(0.3)
        self.move_gripper(0.8)
        time.sleep(0.6)
        self.move_arm({**POSE_PRE, 'arm_shoulder_pan': pan_target}, 2.0)
        self.move_arm(POSE_FOLDED, 2.5)
        return True

    def lift(self):
        # 계단식 들기 + wrist 보상: 그리퍼 절대 피치를 유지해 핀치가 풀리지 않게
        for lf, wf in ((0.42, 0.96), (0.35, 1.03), (0.27, 1.11), (0.20, 1.18), (0.15, 1.23)):
            self.move_arm({'arm_shoulder_lift': lf, 'arm_wrist_flex': wf}, 1.5)
            time.sleep(0.2)
            self.gripper_angle = None
            self.spin_until(lambda: self.gripper_angle is not None, 3.0)
            self.get_logger().info(f'  step lift={lf}: 그리퍼 각도={self.gripper_angle:.3f}')
            if lf in (0.42, 0.27):
                # 재조임 = 최초 접촉각 기준 절대 목표 재주장 (현재각 기준 래칫 파고듦 방지)
                self.move_gripper(self.hold_target)
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
        if n.skip_approach:
            n.get_logger().info('== 2. 접근 생략 (물체 앞 가정, pan=0) ==')
            pan = 0.0
            n.spin_until(lambda: n.odom is not None, 5.0)
            n._anchor_odom = n.odom
        else:
            n.get_logger().info('== 2. 비전 접근 주행 ==')
            pan = n.approach()
        if pan is None:
            n.get_logger().error('접근 실패')
        else:
            n.get_logger().info(f'== 3. 파지 (pan={math.degrees(pan):.1f}deg) ==')
            held = n.grasp(pan)
            if held:
                n.get_logger().info('== 4. 들어올리기 ==')
                ok = n.lift()
                if ok:
                    n.get_logger().info('== 5. 옮겨 놓기 (place) ==')
                    n.place()
            else:
                n.get_logger().error('파지 실패 (그리퍼 완전 닫힘 = 허공)')
        n.get_logger().info('=== PICK_SUCCESS ===' if ok else '=== PICK_FAIL ===')
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
