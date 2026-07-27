import argparse
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    'arm_shoulder_pan',
    'arm_shoulder_lift',
    'arm_elbow_flex',
    'arm_wrist_flex',
    'arm_wrist_roll',
]
GRIPPER_JOINT = 'arm_gripper'

# 관절 가동범위 (jdamr_cube_description/urdf/jdamr_cube.urdf의 <limit lower/upper>와 동일).
# `ui` 명령의 슬라이더 범위로 쓴다.
JOINT_LIMITS = {
    'arm_shoulder_pan': (-1.91986, 1.91986),
    'arm_shoulder_lift': (-1.74533, 1.74533),
    'arm_elbow_flex': (-1.69, 1.69),
    'arm_wrist_flex': (-1.65806, 1.65806),
    'arm_wrist_roll': (-2.74385, 2.84121),
    GRIPPER_JOINT: (-0.174533, 1.74533),
}
UI_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
UI_TRACKBAR_STEPS = 1000   # 슬라이더는 정수만 다루므로 가동범위를 이 칸수로 나눠 쓴다
UI_MOVE_DURATION = 0.5     # 슬라이더를 움직였을 때 그 목표까지 가는 데 주는 시간 [sec]
UI_SEND_PERIOD = 0.15      # 슬라이더를 끌 때 목표를 다시 보내는 최소 간격 [sec]
UI_PANEL_WIDTH = 620
UI_PANEL_ROW_HEIGHT = 34
# run_gazebo_slam.bat이 시뮬레이터와 같이 띄우기 때문에, 컨트롤러가 올라올 때까지 넉넉히 기다린다.
UI_STARTUP_TIMEOUT = 30.0

CAMERA_TOPIC = 'wrist_camera/image_raw'
IMAGE_WIDTH = 640
IMAGE_CENTER_X = IMAGE_WIDTH / 2.0

# `view` 명령에서 --camera로 고를 수 있는 카메라 토픽 프리셋.
DEPTH_CAMERA_TOPIC = 'rgbd_camera/depth_image'
CAMERA_TOPICS = {
    'wrist': CAMERA_TOPIC,
    'rgbd': 'rgbd_camera/image',
    'depth': DEPTH_CAMERA_TOPIC,
}
DEPTH_VIEW_NEAR = 0.1  # 이 거리 이하는 가장 가까운 색(빨강)으로 표시
DEPTH_VIEW_FAR = 3.0   # 이 거리 이상/무한대(NaN)는 가장 먼 색(파랑)으로 표시

# --- MoveIt2 기반 "pick --at-pixel" 에서 사용하는 RGBD 카메라/좌표 상수 ---
RGBD_IMAGE_TOPIC = 'rgbd_camera/image'
RGBD_DEPTH_TOPIC = DEPTH_CAMERA_TOPIC
RGBD_CAMERA_INFO_TOPIC = 'rgbd_camera/camera_info'
RGBD_CAMERA_FRAME = 'rgbd_camera_link'
MOVEIT_PLANNING_FRAME = 'base_footprint'
MOVEIT_PLANNING_GROUP = 'arm'
MOVEIT_EE_LINK = 'arm_gripper_frame_link'
# 집기 전/후 목표 지점 위(z+)로 이만큼 띄운 자세를 먼저 거쳐간다 (충돌 회피 + 들어올리기).
PICK_APPROACH_HEIGHT = 0.08
# 뎁스로 잰 표면 위치보다 그리퍼가 살짝 더 파고들도록 하는 보정값 (물체를 완전히 감싸 집기 위함).
PICK_GRASP_Z_OFFSET = -0.01
# 내려놓을 때는 반대로 살짝 띄운 곳에서 손을 펴야 물체가 바닥/상판에 끼이지 않는다.
PLACE_RELEASE_Z_OFFSET = 0.01

# grip/place 동작을 마친 뒤 돌아갈 "원위치" (jdamr_cube.srdf의 'home' group_state와 동일).
# 이 자세는 MoveIt 충돌 검사도 통과한다: URDF collision 메쉬로 실측한 laser_link 원통까지
# 최소 거리가 팔 링크 전부 19mm 이상(라이다가 base_link 기준 x=0.25, z=0.1일 때 기준. 라이다
# 위치를 옮기면 이 여유도 달라지므로 다시 확인할 것). 반면 Gazebo 스폰 자세
# (shoulder_lift=-1.0, elbow_flex=1.5, wrist_flex=0.8)는 arm_gripper_link가 라이다 안으로
# 22mm 파고들어 있어, 그 자세에서 MoveIt에 플래닝을 시키면 시작 자세 충돌로 즉시 거부된다.
DEFAULT_HOME_JOINTS = {
    'arm_shoulder_pan': 0.0,
    'arm_shoulder_lift': 0.0,
    'arm_elbow_flex': 0.0,
    'arm_wrist_flex': 0.0,
    'arm_wrist_roll': 0.0,
}
HOME_DURATION = 3.0

# 홈 자세는 파일에서 읽어온다. 워크스페이스를 다시 빌드해도(소스를 ~/jdamr_cube_ws로 rsync)
# 값이 날아가지 않도록 설치 경로가 아니라 홈 디렉터리에 저장한다. 파일이 없으면
# DEFAULT_HOME_JOINTS를 쓰고, `ui` 창의 "Set current as HOME" 버튼으로 새로 만들 수 있다.
HOME_FILE_ENV = 'JDAMR_CUBE_ARM_HOME_FILE'
DEFAULT_HOME_FILE = os.path.join(
    os.path.expanduser('~'), '.config', 'jdamr_cube', 'arm_home.yaml')

# 손목 카메라로 물체를 찾는 "탐색 자세". 이 자세에서 shoulder_pan만 바꿔가며 화면 중앙에
# 물체가 오도록 정렬한다. jdamr_cube_gazebo/worlds/room.world의 pick_object 기준으로
# 시뮬레이션에서 실측/튜닝한 값.
SEARCH_JOINTS = {
    'arm_shoulder_lift': -0.5,
    'arm_elbow_flex': 0.9,
    'arm_wrist_flex': 0.6,
    'arm_wrist_roll': 0.0,
}
# SEARCH_JOINTS 자세에서 shoulder_pan을 바꿨을 때 화면 속 물체의 x 픽셀이 움직이는 비율.
# 실측: pan=0.0 -> cx=260.5px, pan=0.3 -> cx=232.8px (약 -92 px/rad)
PAN_PIXELS_PER_RAD = -92.0

# 물체를 향해 실제로 팔을 뻗어 집는 자세 (시뮬레이션에서 실측/튜닝, shoulder_pan은 정렬 결과로 채움).
APPROACH_JOINTS = {
    'arm_shoulder_lift': -1.2,
    'arm_elbow_flex': 1.5,
    'arm_wrist_flex': 0.5,
    'arm_wrist_roll': 0.0,
}
LIFT_SHOULDER_LIFT = -0.6  # 집거나 놓은 뒤 이 정도만 들어올려 팔을 뒤로 뺀다.

GRIPPER_OPEN = 1.7
GRIPPER_CLOSED = -0.17

DEFAULT_PLACE_PAN = -0.8  # pick 지점과 다른 곳에 내려놓기 위한 기본 pan 각도 [rad]

# 물체 색상 프리셋 (HSV lower/upper). 기본값은 room.world의 pick_object(핑크색 큐브) 기준.
COLOR_PRESETS = {
    'pink': ((140, 80, 80), (170, 255, 255)),
}


def detect_object_center(image_bgr, hsv_lower, hsv_upper, min_area=50):
    """HSV 색상 마스크의 무게중심을 물체 픽셀 좌표로 반환. 못 찾으면 None."""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lower), np.array(hsv_upper))
    moments = cv2.moments(mask)
    if moments['m00'] < min_area:
        return None
    return moments['m10'] / moments['m00'], moments['m01'] / moments['m00']


def home_file_path(path=None):
    """홈 자세 파일 경로. 인자 > 환경변수(JDAMR_CUBE_ARM_HOME_FILE) > 기본 경로 순."""
    return os.path.expanduser(path or os.environ.get(HOME_FILE_ENV) or DEFAULT_HOME_FILE)


def load_home_joints(path=None):
    """홈 자세를 파일에서 읽어 (관절 dict, 출처 설명) 을 반환한다.

    파일이 없거나 형식이 잘못됐으면 DEFAULT_HOME_JOINTS를 그대로 쓴다 (UI/CLI가 죽지 않도록).
    파일에 일부 관절만 적혀 있어도 되고, 값은 가동범위(JOINT_LIMITS) 안으로 잘라서 쓴다."""
    file_path = home_file_path(path)
    home = dict(DEFAULT_HOME_JOINTS)
    if not os.path.exists(file_path):
        return home, f'기본값(파일 없음: {file_path})'
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as exc:
        return home, f'기본값(파일을 읽지 못함: {file_path} — {exc})'

    joints = data.get('home_joints', data)
    if not isinstance(joints, dict):
        return home, f'기본값(형식이 잘못됨: {file_path})'
    for joint in ARM_JOINTS:
        if joint not in joints:
            continue
        try:
            value = float(joints[joint])
        except (TypeError, ValueError):
            continue
        lo, hi = JOINT_LIMITS[joint]
        home[joint] = min(max(value, lo), hi)
    return home, file_path


def save_home_joints(values, path=None):
    """현재 각도를 홈 자세로 파일에 저장하고 저장한 경로를 반환한다."""
    file_path = home_file_path(path)
    directory = os.path.dirname(file_path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    payload = {'home_joints': {j: round(float(values[j]), 4) for j in ARM_JOINTS if j in values}}
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write('# jdamr_cube SO-101 팔의 홈(원위치) 자세 [rad].\n')
        f.write("# 'joint_control ui' 창의 'Set current as HOME' 버튼(또는 's' 키)이 덮어씁니다.\n")
        yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)
    return file_path


def rad_to_slider(value, joint):
    """관절 각도[rad]를 0~UI_TRACKBAR_STEPS 슬라이더 눈금으로 바꾼다."""
    lo, hi = JOINT_LIMITS[joint]
    ratio = (min(max(value, lo), hi) - lo) / (hi - lo)
    return int(round(ratio * UI_TRACKBAR_STEPS))


def slider_to_rad(pos, joint):
    """슬라이더 눈금을 관절 각도[rad]로 되돌린다."""
    lo, hi = JOINT_LIMITS[joint]
    return lo + (hi - lo) * pos / UI_TRACKBAR_STEPS


def render_joint_panel(targets, actual, gripper_ready=True, status=None):
    """`ui` 창에 그릴 상태 표. 슬라이더로 준 목표값과 /joint_states의 실제값을 나란히 보여준다.

    OpenCV putText는 한글을 못 그리므로 이 표 안의 글자만 영문으로 쓴다."""
    rows = len(UI_JOINTS)
    height = UI_PANEL_ROW_HEIGHT * (rows + 4)
    img = np.full((height, UI_PANEL_WIDTH, 3), 32, np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX

    cv2.putText(img, 'joint', (12, 26), font, 0.5, (170, 170, 170), 1, cv2.LINE_AA)
    cv2.putText(img, 'target [rad] (deg)', (200, 26), font, 0.5, (170, 170, 170), 1, cv2.LINE_AA)
    cv2.putText(img, 'actual', (410, 26), font, 0.5, (170, 170, 170), 1, cv2.LINE_AA)
    cv2.putText(img, 'limits', (505, 26), font, 0.5, (170, 170, 170), 1, cv2.LINE_AA)

    for i, joint in enumerate(UI_JOINTS):
        y = UI_PANEL_ROW_HEIGHT * (i + 2)
        target = targets[joint]
        now = actual.get(joint)
        lo, hi = JOINT_LIMITS[joint]
        # 목표와 실제가 많이 벌어져 있으면(아직 이동 중이거나 막혀 있으면) 노란색으로.
        color = (255, 255, 255)
        if now is not None and abs(now - target) > 0.05:
            color = (0, 210, 255)
        if joint == GRIPPER_JOINT and not gripper_ready:
            color = (120, 120, 120)
        cv2.putText(img, joint.replace('arm_', ''), (12, y), font, 0.55, color, 1, cv2.LINE_AA)
        cv2.putText(img, f'{target:+.3f} ({math.degrees(target):+.1f})', (200, y),
                    font, 0.5, color, 1, cv2.LINE_AA)
        cv2.putText(img, ' --' if now is None else f'{now:+.3f}', (410, y),
                    font, 0.5, color, 1, cv2.LINE_AA)
        cv2.putText(img, f'{lo:+.2f}~{hi:+.2f}', (505, y), font, 0.45,
                    (140, 140, 140), 1, cv2.LINE_AA)

    cv2.putText(img, 'h: go HOME   s: set current as HOME   r: resync   q: quit',
                (12, height - 44), font, 0.5, (0, 220, 120), 1, cv2.LINE_AA)
    if status:
        # 경로가 길면 뒤쪽(파일 이름 쪽)이 잘리지 않도록 앞을 줄인다.
        text = status if len(status) <= 66 else '...' + status[-63:]
        cv2.putText(img, text, (12, height - 14), font, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    return img


def colorize_depth(depth, near=DEPTH_VIEW_NEAR, far=DEPTH_VIEW_FAR):
    """rgbd_camera의 32FC1 depth 원본을 화면에 보이는 8비트 이미지로 바꾼다.
    near~far 범위 안에 있는 픽셀은 흰색, 벗어나면(너무 가깝거나 멀거나 반사가 없는 NaN/inf)
    검정으로 그린다."""
    in_range = np.isfinite(depth) & (depth >= near) & (depth <= far)
    mask = np.where(in_range, np.uint8(255), np.uint8(0))
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


def deproject_pixel(depth, camera_info, u, v):
    """뎁스 이미지의 픽셀 (u, v)를 rgbd_camera_link 자신의 프레임 기준 3D 점으로 되돌린다.

    gz-sim의 카메라 센서는 (ROS 표준 광학 프레임이 아니라) 링크 자신의 로컬 +X축 방향을
    바라본다. room.world의 pick_object를 카메라 정면 광선 위에 정확히 놓고 픽셀 중심(320,240)과
    실측 깊이로 반복 검증해 확정한 변환식이다:
        x_body = z (원시 뎁스, 카메라가 보는 방향)
        y_body = -(u-cx)*z/fx
        z_body = -(v-cy)*z/fy
    """
    height, width = depth.shape[:2]
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < width and 0 <= vi < height):
        return None
    z = float(depth[vi, ui])
    if not np.isfinite(z) or z <= 0.0:
        return None
    fx, fy = camera_info.k[0], camera_info.k[4]
    cx, cy = camera_info.k[2], camera_info.k[5]
    x_body = z
    y_body = -(u - cx) * z / fx
    z_body = -(v - cy) * z / fy
    return x_body, y_body, z_body


class So101ArmControl(Node):

    def __init__(self, node_name='jdamr_cube_so101_arm_control', home_file=None):
        super().__init__(node_name)
        self.home_file = home_file
        self._home_joints = None
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(
            self, GripperCommand, 'gripper_controller/gripper_cmd')
        self._latest_arm_state = None
        self._latest_image = None
        self._latest_depth = None
        self._latest_camera_info = None
        self._moveit_py = None
        self._moveit_arm = None
        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(JointState, 'joint_states', self._joint_state_cb, 10)
        self.create_subscription(Image, CAMERA_TOPIC, self._image_cb, 1)
        self.create_subscription(Image, RGBD_DEPTH_TOPIC, self._depth_cb, 1)
        self.create_subscription(
            CameraInfo, RGBD_CAMERA_INFO_TOPIC, self._camera_info_cb, 1)

    def _joint_state_cb(self, msg):
        if all(j in msg.name for j in ARM_JOINTS):
            self._latest_arm_state = msg

    def _image_cb(self, msg):
        self._latest_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg):
        self._latest_depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _camera_info_cb(self, msg):
        self._latest_camera_info = msg

    def latest_arm_positions(self):
        """마지막으로 받은 /joint_states에서 팔+그리퍼 각도를 꺼낸다 (없으면 빈 dict)."""
        msg = self._latest_arm_state
        if msg is None:
            return {}
        return {j: msg.position[msg.name.index(j)] for j in UI_JOINTS if j in msg.name}

    def wait_for_current_arm_positions(self, timeout_sec=5.0):
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self._latest_arm_state is None:
            if self.get_clock().now() > deadline:
                return None
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.latest_arm_positions()

    def wait_for_camera_image(self, timeout_sec=5.0):
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        self._latest_image = None
        while rclpy.ok() and self._latest_image is None:
            if self.get_clock().now() > deadline:
                return None
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._latest_image

    def wait_for_rgbd_depth_and_info(self, timeout_sec=5.0):
        """rgbd_camera의 뎁스 이미지 + camera_info(내부 파라미터)를 함께 기다린다."""
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        self._latest_depth = None
        self._latest_camera_info = None
        while rclpy.ok() and (self._latest_depth is None or self._latest_camera_info is None):
            if self.get_clock().now() > deadline:
                return None, None
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._latest_depth, self._latest_camera_info

    def move_joints(self, overrides, duration_sec):
        """현재 각도를 읽어와 overrides로 준 관절만 바꾼 목표를 arm_controller로 보낸다."""
        current = self.wait_for_current_arm_positions()
        if current is None:
            self.get_logger().error(
                '/joint_states에서 현재 팔 각도를 5초 안에 받지 못했습니다. '
                'jdamr_cube_gazebo/gazebo.launch.py가 실행 중인지 확인하세요.')
            return False
        target = dict(current)
        target.update(overrides)
        return self.send_arm_goal(target, duration_sec)

    def make_arm_goal(self, target_positions, duration_sec):
        point = JointTrajectoryPoint()
        point.positions = [float(target_positions[j]) for j in ARM_JOINTS]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(ARM_JOINTS)
        goal_msg.trajectory.points = [point]
        return goal_msg

    def send_arm_goal_nowait(self, target_positions, duration_sec):
        """결과를 기다리지 않고 팔 목표만 던진다.

        `ui` 명령처럼 슬라이더를 끄는 동안 목표를 계속 갱신해야 할 때 쓴다. 결과를 기다리면
        이동이 끝날 때까지 창이 멈추기 때문이다. joint_trajectory_controller는 새 목표가
        들어오면 실행 중이던 목표를 취소하고 새 목표를 따라가므로 이렇게 보내도 안전하다."""
        self._arm_client.send_goal_async(self.make_arm_goal(target_positions, duration_sec))

    def send_gripper_goal_nowait(self, position, max_effort=5.0):
        """결과를 기다리지 않고 그리퍼 목표만 던진다 (`ui` 명령용)."""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = max_effort
        self._gripper_client.send_goal_async(goal_msg)

    def send_arm_goal(self, target_positions, duration_sec):
        if not self._arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "'arm_controller/follow_joint_trajectory' 액션 서버가 없습니다. "
                'jdamr_cube_gazebo/gazebo.launch.py가 실행 중인지 확인하세요.')
            return False

        goal_msg = self.make_arm_goal(target_positions, duration_sec)
        point = goal_msg.trajectory.points[0]

        self.get_logger().info(
            '팔 목표 전송: ' + ', '.join(f'{j}={v:.3f}' for j, v in zip(ARM_JOINTS, point.positions)))
        send_goal_future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('팔 목표가 거부되었습니다.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('팔 이동 완료.')
            return True
        self.get_logger().error(f'팔 이동 실패 (error_code={result.error_code}).')
        return False

    def send_gripper_goal(self, position, max_effort=5.0):
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "'gripper_controller/gripper_cmd' 액션 서버가 없습니다. "
                'jdamr_cube_gazebo/gazebo.launch.py가 실행 중인지 확인하세요.')
            return False

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(f'그리퍼 목표 전송: position={position:.3f}')
        send_goal_future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('그리퍼 목표가 거부되었습니다.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.reached_goal or result.stalled:
            self.get_logger().info(f'그리퍼 이동 완료 (position={result.position:.3f}).')
            return True
        self.get_logger().error(f'그리퍼 이동 실패 (position={result.position:.3f}).')
        return False

    def align_to_object(self, hsv_range, max_iterations=4, pixel_tolerance=15.0):
        """SEARCH_JOINTS 자세로 이동한 뒤 shoulder_pan을 조정해 화면 중앙에 물체를 맞춘다.
        정렬된 shoulder_pan 각도를 반환하고, 물체를 못 찾으면 None을 반환한다."""
        if not self.move_joints(SEARCH_JOINTS, 2.5):
            return None
        current = self.wait_for_current_arm_positions()
        pan = current.get('arm_shoulder_pan', 0.0) if current else 0.0

        for i in range(max_iterations):
            image = self.wait_for_camera_image()
            if image is None:
                self.get_logger().error(
                    f"'{CAMERA_TOPIC}' 카메라 이미지를 받지 못했습니다.")
                return None
            center = detect_object_center(image, hsv_range[0], hsv_range[1])
            if center is None:
                self.get_logger().error('카메라 화면에서 물체를 찾지 못했습니다 (색상/위치 확인).')
                return None
            cx, _cy = center
            error_px = IMAGE_CENTER_X - cx
            self.get_logger().info(
                f'[탐색 {i + 1}/{max_iterations}] 물체 픽셀 x={cx:.1f} '
                f'(중앙 오차 {error_px:+.1f}px), 현재 pan={pan:.3f}')
            if abs(error_px) < pixel_tolerance:
                self.get_logger().info('물체가 화면 중앙에 정렬되었습니다.')
                return pan
            pan = max(-1.9, min(1.9, pan + error_px / PAN_PIXELS_PER_RAD))
            if not self.move_joints({'arm_shoulder_pan': pan}, 1.5):
                return None

        self.get_logger().warn('정렬 반복 횟수를 초과했습니다. 마지막 pan 각도로 진행합니다.')
        return pan

    def pixel_to_planning_frame(self, u, v, target_frame=MOVEIT_PLANNING_FRAME, timeout_sec=3.0):
        """RGBD 카메라 픽셀 (u, v)을 target_frame(기본 base_footprint) 기준 3D 점으로 변환한다.
        뎁스가 없거나(NaN/0) TF를 못 찾으면 None을 반환한다."""
        depth, camera_info = self.wait_for_rgbd_depth_and_info()
        if depth is None or camera_info is None:
            self.get_logger().error(
                f"'{RGBD_DEPTH_TOPIC}' 또는 '{RGBD_CAMERA_INFO_TOPIC}'을 받지 못했습니다. "
                'jdamr_cube_gazebo/gazebo.launch.py가 실행 중인지 확인하세요.')
            return None

        point_camera = deproject_pixel(np.asarray(depth), camera_info, u, v)
        if point_camera is None:
            self.get_logger().error(f'픽셀 ({u:.0f}, {v:.0f})의 뎁스 값이 유효하지 않습니다.')
            return None

        stamped = PointStamped()
        stamped.header.frame_id = RGBD_CAMERA_FRAME
        stamped.header.stamp = rclpy.time.Time().to_msg()
        stamped.point.x, stamped.point.y, stamped.point.z = point_camera

        # lookup_transform의 timeout은 노드가 spin되지 않으면 TF 버퍼가 채워지지 않아
        # 그대로 실패한다. 직접 spin하면서 변환이 가능해질 때까지 기다린다.
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and not self._tf_buffer.can_transform(
                target_frame, RGBD_CAMERA_FRAME, rclpy.time.Time()):
            if self.get_clock().now() > deadline:
                self.get_logger().error(
                    f"'{RGBD_CAMERA_FRAME}' -> '{target_frame}' TF 변환을 "
                    f'{timeout_sec}초 안에 못 찾았습니다.')
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        transform = self._tf_buffer.lookup_transform(
            target_frame, RGBD_CAMERA_FRAME, rclpy.time.Time())

        transformed = do_transform_point(stamped, transform)
        point = transformed.point.x, transformed.point.y, transformed.point.z
        self.get_logger().info(
            f'픽셀 ({u:.0f}, {v:.0f}) -> {target_frame} 기준 '
            f'x={point[0]:.3f}, y={point[1]:.3f}, z={point[2]:.3f}')
        return point

    def _init_moveit(self):
        """moveit_py는 이 명령(pick --at-pixel)에서만 필요하므로 지연 임포트한다 — 설치되지
        않은 환경에서도 move/view/pick(색상)/place 등 다른 명령은 정상 동작해야 한다."""
        if getattr(self, '_moveit_py', None) is not None:
            return self._moveit_py, self._moveit_arm

        from ament_index_python.packages import get_package_share_directory
        from moveit.planning import MoveItPy
        from moveit_configs_utils import MoveItConfigsBuilder

        urdf_file = os.path.join(
            get_package_share_directory('jdamr_cube_description'), 'urdf', 'jdamr_cube.urdf')
        moveit_config = (
            MoveItConfigsBuilder('jdamr_cube', package_name='jdamr_cube_moveit_config')
            .robot_description(file_path=urdf_file)
            .planning_pipelines(pipelines=['ompl'])
            .to_moveit_configs()
        )
        # moveit_ros_move_group(런치 파일)과 moveit_py가 내부적으로 쓰는 MoveItCpp는
        # 파이프라인 이름 목록을 서로 다른 파라미터 이름으로 읽는다:
        #   move_group      -> 최상위 'planning_pipelines' (문자열 리스트)
        #   MoveItCpp(moveit_py) -> 'planning_pipelines.pipeline_names'
        # MoveItConfigsBuilder.planning_pipelines()는 move_group 형식만 채워주므로
        # MoveItCpp가 읽는 중첩 키를 여기서 추가해준다.
        config_dict = moveit_config.to_dict()
        config_dict['planning_pipelines'] = {'pipeline_names': list(config_dict['planning_pipelines'])}
        # MoveItCpp는 어느 파이프라인/플래너를 쓸지도 'plan_request_params.*' 파라미터로 읽는다
        # (안 주면 planning_pipeline이 빈 문자열로 기본값 처리돼 플래닝이 바로 실패한다).
        config_dict['plan_request_params'] = {
            'planning_pipeline': 'ompl',
            'planner_id': 'RRTConnectkConfigDefault',
            'planning_time': 5.0,
            'planning_attempts': 5,
            'max_velocity_scaling_factor': 0.3,
            'max_acceleration_scaling_factor': 0.3,
        }
        # Gazebo 시뮬레이션 클럭 기준으로 joint_states 타임스탬프를 해석해야
        # 궤적 실행 전 현재 상태 검증이 통과한다 (없으면 실행이 바로 실패).
        # 주의: use_sim_time만 주면 내부 노드가 /clock 구독의 qos_overrides 파라미터를
        # 선언하다 InvalidParameterValueException으로 죽는다(moveit_py 2.12.4 실측).
        # qos_overrides 기본값 4개를 함께 미리 넣어주면 회피된다.
        config_dict['use_sim_time'] = True
        config_dict['qos_overrides./clock.subscription.depth'] = 1
        config_dict['qos_overrides./clock.subscription.durability'] = 'volatile'
        config_dict['qos_overrides./clock.subscription.history'] = 'keep_last'
        config_dict['qos_overrides./clock.subscription.reliability'] = 'best_effort'

        self.get_logger().info('MoveItPy를 초기화합니다 (몇 초 걸릴 수 있습니다)...')
        self._moveit_py = MoveItPy(
            node_name='jdamr_cube_pick_moveit_py', config_dict=config_dict)
        self._moveit_arm = self._moveit_py.get_planning_component(MOVEIT_PLANNING_GROUP)
        return self._moveit_py, self._moveit_arm

    def _moveit_move_to(self, x, y, z, frame_id=MOVEIT_PLANNING_FRAME):
        """MoveIt2(OMPL)로 arm_gripper_frame_link를 (x, y, z) 위치로 플래닝/실행한다.

        주의: PoseStamped 목표를 쓰면 안 된다 — 목표 제약에 방향(orientation)까지 포함돼
        position_only_ik(KDL)가 찾은 '위치만 맞는' 해가 전부 기각되어 5-DOF 팔에서는
        플래닝이 항상 실패한다(실측). 위치 제약만 담은 Constraints를 직접 만들어 준다.
        """
        from moveit.core.kinematic_constraints import construct_link_constraint

        moveit_py, arm = self._init_moveit()

        # moveit_py의 첫 execute() 호출이 컨트롤러/실행매니저 초기화 레이스로 가끔 바로
        # 실패한다(실측). 매번 새로 plan한 뒤 실행하고, 실패하면 최대 2회까지 재시도한다.
        for attempt in range(1, 3):
            constraint = construct_link_constraint(
                link_name=MOVEIT_EE_LINK,
                source_frame=frame_id,
                cartesian_position=[x, y, z],
                cartesian_position_tolerance=0.01)
            arm.set_start_state_to_current_state()
            arm.set_goal_state(motion_plan_constraints=[constraint])
            plan_result = arm.plan()
            if not plan_result:
                self.get_logger().error(
                    f'MoveIt 플래닝 실패: 목표 x={x:.3f}, y={y:.3f}, z={z:.3f} ({frame_id}).')
                return False

            self.get_logger().info(
                f'MoveIt 플랜 실행: x={x:.3f}, y={y:.3f}, z={z:.3f} ({frame_id}).')
            if moveit_py.execute(plan_result.trajectory, controllers=[]):
                return True
            self.get_logger().warn(
                f'MoveIt 궤적 실행 실패 (시도 {attempt}/2). 다시 시도합니다.')

        self.get_logger().error('MoveIt 궤적 실행에 2회 실패했습니다.')
        return False

    def home_joints(self, reload=False):
        """홈 자세를 (필요하면 파일에서 읽어) 돌려준다. 한 번 읽으면 캐시한다."""
        if self._home_joints is None or reload:
            self._home_joints, source = load_home_joints(self.home_file)
            self.get_logger().info(f'홈 자세 출처: {source}')
        return dict(self._home_joints)

    def save_home_joints(self, values):
        """현재 각도를 홈 자세로 파일에 저장한다. 저장한 경로를 반환(실패 시 None)."""
        try:
            path = save_home_joints(values, self.home_file)
        except OSError as exc:
            self.get_logger().error(f'홈 자세를 저장하지 못했습니다: {exc}')
            return None
        self._home_joints = {j: values[j] for j in ARM_JOINTS if j in values}
        self.get_logger().info(
            f'현재 각도를 홈 자세로 저장했습니다 -> {path}: '
            + ', '.join(f'{j.replace("arm_", "")}={self._home_joints[j]:+.3f}'
                        for j in self._home_joints))
        return path

    def go_home(self, duration_sec=HOME_DURATION):
        """팔을 원위치(홈 자세 파일 또는 기본값)로 되돌린다."""
        home = self.home_joints()
        self.get_logger().info(
            '팔을 원위치로 되돌립니다: '
            + ', '.join(f'{j.replace("arm_", "")}={v:+.3f}' for j, v in home.items()))
        return self.move_joints(home, duration_sec)

    def prepare_for_planning(self, duration_sec=HOME_DURATION):
        """MoveIt 플래닝 전에 팔을 충돌 없는 원위치로 먼저 빼낸다.

        MoveIt은 시작 자세가 충돌 상태면(CheckStartStateCollision) 플래닝 자체를 거부한다
        ("2 contact(s) detected : arm_gripper_link - laser_link, ..."). 그런데 Gazebo 스폰
        자세나 직전 명령이 남긴 자세는 그리퍼가 라이다에 닿아 있을 수 있다. arm_controller로
        보내는 궤적은 충돌 검사를 거치지 않으므로, 이 경로로 먼저 원위치까지 빠져나온 뒤
        플래닝을 시작한다. 홈 자세를 파일에서 바꿔 놓았다면 그 자세가 충돌 없는지도
        확인해야 한다 (충돌 자세를 홈으로 저장하면 여기서 다시 플래닝이 거부된다)."""
        self.get_logger().info('MoveIt 플래닝 시작 자세(원위치)로 팔을 먼저 옮깁니다.')
        return self.move_joints(self.home_joints(), duration_sec)

    def grasp_at_point(self, x, y, z, approach_height=PICK_APPROACH_HEIGHT,
                       z_offset=PICK_GRASP_Z_OFFSET, frame_id=MOVEIT_PLANNING_FRAME):
        """(x, y, z) 지점을 MoveIt2로 집는다: 열기 → pre-grasp → 하강 → 닫기 → 들어올리기."""
        target_z = z + z_offset

        if not self.send_gripper_goal(GRIPPER_OPEN):
            return False

        self.get_logger().info('목표 위 지점(pre-grasp)으로 접근합니다.')
        if not self._moveit_move_to(x, y, target_z + approach_height, frame_id):
            return False

        self.get_logger().info('목표 지점까지 내려갑니다.')
        if not self._moveit_move_to(x, y, target_z, frame_id):
            return False

        self.get_logger().info('그리퍼를 닫아 집습니다.')
        if not self.send_gripper_goal(GRIPPER_CLOSED):
            return False

        self.get_logger().info('들어올립니다.')
        return self._moveit_move_to(x, y, target_z + approach_height, frame_id)

    def release_at_point(self, x, y, z, approach_height=PICK_APPROACH_HEIGHT,
                         z_offset=PLACE_RELEASE_Z_OFFSET, frame_id=MOVEIT_PLANNING_FRAME):
        """(x, y, z) 지점에 MoveIt2로 내려놓는다: pre-place → 하강 → 열기 → 다시 들어올리기."""
        target_z = z + z_offset

        self.get_logger().info('목표 위 지점(pre-place)으로 접근합니다.')
        if not self._moveit_move_to(x, y, target_z + approach_height, frame_id):
            return False

        self.get_logger().info('목표 지점까지 내려갑니다.')
        if not self._moveit_move_to(x, y, target_z, frame_id):
            return False

        self.get_logger().info('그리퍼를 열어 물체를 놓습니다.')
        if not self.send_gripper_goal(GRIPPER_OPEN):
            return False

        self.get_logger().info('팔을 들어올려 물러납니다.')
        return self._moveit_move_to(x, y, target_z + approach_height, frame_id)

    def grip_at_point(self, x, y, z, approach_height=PICK_APPROACH_HEIGHT,
                      z_offset=PICK_GRASP_Z_OFFSET, frame_id=MOVEIT_PLANNING_FRAME):
        """grip 명령: (x, y, z)로 가서 집은 뒤 원위치로 돌아온다."""
        self.get_logger().info(
            f'grip 목표: x={x:.3f}, y={y:.3f}, z={z:.3f} ({frame_id} 기준).')
        if not self.prepare_for_planning():
            return False
        if not self.grasp_at_point(x, y, z, approach_height, z_offset, frame_id):
            return False
        if not self.go_home():
            return False
        self.get_logger().info('grip 동작을 완료했습니다.')
        return True

    def place_at_point(self, x, y, z, approach_height=PICK_APPROACH_HEIGHT,
                       z_offset=PLACE_RELEASE_Z_OFFSET, frame_id=MOVEIT_PLANNING_FRAME):
        """place 명령: (x, y, z)로 가서 내려놓은 뒤 원위치로 돌아온다."""
        self.get_logger().info(
            f'place 목표: x={x:.3f}, y={y:.3f}, z={z:.3f} ({frame_id} 기준).')
        if not self.prepare_for_planning():
            return False
        if not self.release_at_point(x, y, z, approach_height, z_offset, frame_id):
            return False
        if not self.go_home():
            return False
        self.get_logger().info('place 동작을 완료했습니다.')
        return True

    def pick_at_pixel(self, u, v, approach_height=PICK_APPROACH_HEIGHT, z_offset=PICK_GRASP_Z_OFFSET):
        """뎁스카메라 픽셀 (u, v) 위치를 3D로 되돌려 MoveIt2(OMPL)로 그 지점을 집는다."""
        # 3D 좌표를 먼저 구한다: 팔을 옮기면 RGBD 카메라 시야를 가릴 수 있으므로
        # 목표 픽셀의 뎁스를 읽은 뒤에 팔을 움직인다.
        point = self.pixel_to_planning_frame(u, v)
        if point is None:
            return False
        if not self.prepare_for_planning():
            return False
        if not self.grasp_at_point(point[0], point[1], point[2], approach_height, z_offset):
            return False

        self.get_logger().info('pick --at-pixel 동작을 완료했습니다.')
        return True

    def pick(self, color='pink'):
        if color not in COLOR_PRESETS:
            self.get_logger().error(f"알 수 없는 색상 '{color}'. 사용 가능: {list(COLOR_PRESETS)}")
            return False
        hsv_range = COLOR_PRESETS[color]

        self.get_logger().info(f"카메라로 '{color}' 색 물체를 찾는 중...")
        pan = self.align_to_object(hsv_range)
        if pan is None:
            return False

        if not self.send_gripper_goal(GRIPPER_OPEN):
            return False

        self.get_logger().info('물체 쪽으로 접근합니다.')
        approach = dict(APPROACH_JOINTS)
        approach['arm_shoulder_pan'] = pan
        if not self.move_joints(approach, 2.5):
            return False

        self.get_logger().info('그리퍼를 닫아 집습니다.')
        if not self.send_gripper_goal(GRIPPER_CLOSED):
            return False

        self.get_logger().info('들어올립니다.')
        if not self.move_joints({'arm_shoulder_lift': LIFT_SHOULDER_LIFT}, 2.0):
            return False

        self.get_logger().info(
            'pick 동작을 완료했습니다. (참고: SO-101 팔은 별도 IK 없이 시뮬레이션에서 튜닝한 '
            '고정 자세로 접근합니다 — 물체 위치/크기가 바뀌면 실제로 못 집을 수 있으니 '
            'APPROACH_JOINTS 등 상수를 물체 위치에 맞게 조정하세요.)')
        return True

    def place(self, pan=DEFAULT_PLACE_PAN):
        self.get_logger().info(f'물체를 pan={pan:.2f} 위치에 내려놓습니다.')
        approach = dict(APPROACH_JOINTS)
        approach['arm_shoulder_pan'] = pan
        if not self.move_joints(approach, 2.5):
            return False

        self.get_logger().info('그리퍼를 열어 물체를 놓습니다.')
        if not self.send_gripper_goal(GRIPPER_OPEN):
            return False

        self.get_logger().info('팔을 들어올려 물러납니다.')
        if not self.move_joints({'arm_shoulder_lift': LIFT_SHOULDER_LIFT}, 2.0):
            return False

        self.get_logger().info('place 동작을 완료했습니다.')
        return True

    def control_ui(self, duration_sec=UI_MOVE_DURATION, init_home=True):
        """슬라이더로 관절을 직접 움직이는 수동 제어 창을 띄운다.

        창을 띄우기 전에 홈 자세 파일을 읽어 그 자세로 팔을 초기화한다(init_home=False면 생략).
        Gazebo 스폰 자세는 그리퍼가 라이다에 닿아 있어서, 시작할 때 홈 자세로 빼두면 이후
        grip/place의 MoveIt 플래닝도 바로 통한다.

        창의 슬라이더는 각 관절의 가동범위(JOINT_LIMITS)를 UI_TRACKBAR_STEPS 칸으로 나눈 것이고,
        초기화가 끝난 뒤의 실제 각도에 맞춰 놓는다. 슬라이더를 움직이면 UI_SEND_PERIOD 간격으로
        arm_controller/gripper_controller에 목표를 (결과를 기다리지 않고) 보낸다.

        창의 버튼(OpenCV가 Qt로 빌드된 경우)과 단축키로 홈 자세를 다룰 수 있다:
        'h'=홈으로 이동, 's'=현재 각도를 홈으로 저장(파일에 기록), 'r'=슬라이더를 실제 각도로
        다시 맞춤, 'q'=종료."""
        self.get_logger().info('컨트롤러와 /joint_states를 기다리는 중...')
        current = self.wait_for_current_arm_positions(timeout_sec=UI_STARTUP_TIMEOUT)
        if current is None:
            self.get_logger().error(
                f'/joint_states에서 현재 팔 각도를 {UI_STARTUP_TIMEOUT:.0f}초 안에 받지 '
                '못했습니다. jdamr_cube_gazebo/gazebo.launch.py가 실행 중인지 확인하세요.')
            return False
        if not self._arm_client.wait_for_server(timeout_sec=UI_STARTUP_TIMEOUT):
            self.get_logger().error(
                "'arm_controller/follow_joint_trajectory' 액션 서버가 없습니다. "
                'jdamr_cube_gazebo/gazebo.launch.py가 실행 중인지 확인하세요.')
            return False
        gripper_ready = self._gripper_client.wait_for_server(timeout_sec=5.0)
        if not gripper_ready:
            self.get_logger().warn(
                "'gripper_controller/gripper_cmd' 액션 서버가 없어 gripper 슬라이더는 동작하지 "
                '않습니다 (팔 관절은 그대로 제어됩니다).')

        # 시작 자세 초기화: 홈 자세 파일을 읽어 그 자세로 옮긴 뒤, 슬라이더를 그 결과에 맞춘다.
        home = self.home_joints()  # 출처(파일 경로 또는 기본값)를 로그로 남긴다
        if init_home:
            self.get_logger().info('시작 자세를 홈 자세로 초기화합니다...')
            if self.go_home():
                init_status = 'initialized to HOME'
                current = self.wait_for_current_arm_positions() or dict(current, **home)
            else:
                init_status = 'HOME init failed (see terminal log)'
                self.get_logger().warn(
                    '홈 자세로 초기화하지 못했습니다. 현재 자세 그대로 창을 띄웁니다.')
        else:
            init_status = 'HOME init skipped (--no-home-init)'

        window = 'jdamr_cube arm joints'
        bar_names = {j: j.replace('arm_', '') for j in UI_JOINTS}
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, UI_PANEL_WIDTH, UI_PANEL_ROW_HEIGHT * (len(UI_JOINTS) + 4))
        for joint in UI_JOINTS:
            cv2.createTrackbar(bar_names[joint], window,
                               rad_to_slider(current.get(joint, 0.0), joint),
                               UI_TRACKBAR_STEPS, lambda _pos: None)

        def read_targets():
            return {j: slider_to_rad(cv2.getTrackbarPos(bar_names[j], window), j)
                    for j in UI_JOINTS}

        def set_sliders(values):
            for joint in UI_JOINTS:
                if joint in values:
                    cv2.setTrackbarPos(bar_names[joint], window,
                                       rad_to_slider(values[joint], joint))

        # 버튼 콜백은 Qt 이벤트 루프에서 불리므로 여기서는 요청만 남기고,
        # 실제 처리(액션 전송/파일 저장)는 아래 메인 루프에서 한다.
        request = {'go_home': False, 'save_home': False}
        status = {'text': f'{init_status} | home file: {home_file_path(self.home_file)}'}

        def _request(what):
            request[what] = True

        try:
            cv2.createButton('Go HOME', lambda state, _: _request('go_home'),
                             None, cv2.QT_PUSH_BUTTON, 0)
            cv2.createButton('Set current as HOME', lambda state, _: _request('save_home'),
                             None, cv2.QT_PUSH_BUTTON, 0)
        except cv2.error:
            self.get_logger().warn(
                "이 OpenCV 빌드는 버튼(Qt)을 지원하지 않습니다. 'h'(홈으로 이동)/"
                "'s'(현재 각도를 홈으로 저장) 키를 쓰세요.")

        self.get_logger().info(
            '관절 수동 제어 창을 띄웠습니다. 슬라이더로 각 관절을 움직이세요 '
            "(창에서 'h'=홈으로 이동, 's'=현재 각도를 홈으로 저장, "
            "'r'=슬라이더를 현재 각도로 동기화, 'q'=종료).")

        sent = read_targets()
        last_send = 0.0
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)

                if request['go_home']:
                    request['go_home'] = False
                    home = self.home_joints()
                    set_sliders(home)
                    status['text'] = 'moving to HOME'
                    self.get_logger().info('홈 자세로 이동합니다.')
                if request['save_home']:
                    request['save_home'] = False
                    actual = self.latest_arm_positions()
                    if not actual:
                        status['text'] = 'save failed: no /joint_states'
                        self.get_logger().error(
                            '/joint_states를 아직 못 받아 홈 자세를 저장하지 못했습니다.')
                    else:
                        saved = self.save_home_joints(actual)
                        status['text'] = (f'HOME saved: {saved}' if saved
                                          else 'save failed (see terminal log)')

                targets = read_targets()
                if time.monotonic() - last_send >= UI_SEND_PERIOD:
                    arm_changed = any(abs(targets[j] - sent[j]) > 1e-3 for j in ARM_JOINTS)
                    grip_changed = abs(targets[GRIPPER_JOINT] - sent[GRIPPER_JOINT]) > 1e-3
                    if arm_changed:
                        self.send_arm_goal_nowait(targets, duration_sec)
                    if grip_changed and gripper_ready:
                        self.send_gripper_goal_nowait(targets[GRIPPER_JOINT])
                    if arm_changed or grip_changed:
                        summary = ', '.join(
                            f'{bar_names[j]}={targets[j]:+.3f}' for j in UI_JOINTS)
                        self.get_logger().info(
                            '목표 갱신: ' + summary, throttle_duration_sec=1.0)
                        sent = targets
                        last_send = time.monotonic()

                cv2.imshow(window, render_joint_panel(
                    targets, self.latest_arm_positions(), gripper_ready, status['text']))
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('h'):
                    _request('go_home')
                elif key == ord('s'):
                    _request('save_home')
                elif key == ord('r'):
                    actual = self.latest_arm_positions()
                    if actual:
                        set_sliders(actual)
                        sent = read_targets()
                        status['text'] = 'sliders resynced to actual'
                        self.get_logger().info('슬라이더를 현재 각도로 동기화했습니다.')
        finally:
            cv2.destroyAllWindows()
        self.get_logger().info('관절 수동 제어 창을 닫았습니다.')
        return True

    def view_camera(self, topic=None, near=DEPTH_VIEW_NEAR, far=DEPTH_VIEW_FAR):
        """지정한 카메라 화면을 창으로 띄운다. 창에서 'q'를 누르거나 Ctrl+C로 종료.
        depth 카메라일 때는 near~far 범위 안만 흰색으로 표시하며, 창의 Near-/Near+/Far-/Far+
        버튼(또는 '['/']'/','/'.' 키)으로 범위를 실시간으로 조절할 수 있다."""
        topic = topic or CAMERA_TOPIC
        is_depth = topic == DEPTH_CAMERA_TOPIC
        self.get_logger().info(
            f"'{topic}' 화면을 표시합니다. 창을 클릭한 뒤 'q'를 누르면 종료합니다 "
            '(Ctrl+C로도 종료 가능).')
        latest = {'image': None}
        depth_range = {'near': near, 'far': far}
        depth_step = 0.05

        def _adjust(field, delta):
            if field == 'near':
                depth_range['near'] = max(0.0, min(depth_range['far'], depth_range['near'] + delta))
            else:
                depth_range['far'] = max(depth_range['near'], depth_range['far'] + delta)
            self.get_logger().info(
                f"뎁스 표시 범위 변경: near={depth_range['near']:.2f}m, "
                f"far={depth_range['far']:.2f}m")

        def _cb(msg):
            if is_depth:
                raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                latest['image'] = colorize_depth(
                    np.asarray(raw), depth_range['near'], depth_range['far'])
            else:
                latest['image'] = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        sub = self.create_subscription(Image, topic, _cb, 1)
        window = f'jdamr_cube camera: {topic}'
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

        if is_depth:
            self.get_logger().info(
                f'뎁스 표시 범위: near={near:.2f}m, far={far:.2f}m '
                "(창의 Near-/Near+/Far-/Far+ 버튼 또는 '['/']'/','/'.'  키로 조절, 0.05m 단위)")
            try:
                cv2.createButton(
                    'Near -', lambda state, _: _adjust('near', -depth_step),
                    None, cv2.QT_PUSH_BUTTON, 0)
                cv2.createButton(
                    'Near +', lambda state, _: _adjust('near', depth_step),
                    None, cv2.QT_PUSH_BUTTON, 0)
                cv2.createButton(
                    'Far -', lambda state, _: _adjust('far', -depth_step),
                    None, cv2.QT_PUSH_BUTTON, 0)
                cv2.createButton(
                    'Far +', lambda state, _: _adjust('far', depth_step),
                    None, cv2.QT_PUSH_BUTTON, 0)
            except cv2.error:
                self.get_logger().warn(
                    '이 OpenCV 빌드는 버튼(Qt)을 지원하지 않습니다. '
                    "'['/']'/','/'.' 키로 조절하세요.")

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                if latest['image'] is not None:
                    cv2.imshow(window, latest['image'])
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if is_depth and key in (ord('['), ord(']'), ord(','), ord('.')):
                    if key == ord('['):
                        _adjust('near', -depth_step)
                    elif key == ord(']'):
                        _adjust('near', depth_step)
                    elif key == ord(','):
                        _adjust('far', -depth_step)
                    elif key == ord('.'):
                        _adjust('far', depth_step)
        finally:
            cv2.destroyAllWindows()
            self.destroy_subscription(sub)


def parse_view_args(argv):
    parser = argparse.ArgumentParser(
        prog='joint_control view',
        description='카메라 화면을 창으로 띄운다.')
    parser.add_argument('--camera', default='wrist', choices=list(CAMERA_TOPICS),
                        help='표시할 카메라 (wrist=손목 카메라, rgbd=상단 RGBD 컬러 카메라, '
                             'depth=상단 RGBD 뎁스 카메라, 기본값 wrist)')
    parser.add_argument('--topic', default=None,
                        help='--camera 대신 임의의 이미지 토픽을 직접 지정')
    parser.add_argument('--near', type=float, default=DEPTH_VIEW_NEAR,
                        help=f'depth 카메라: 흰색으로 표시할 최소 거리 [m] (기본값 {DEPTH_VIEW_NEAR})')
    parser.add_argument('--far', type=float, default=DEPTH_VIEW_FAR,
                        help=f'depth 카메라: 흰색으로 표시할 최대 거리 [m] (기본값 {DEPTH_VIEW_FAR})')
    return parser.parse_args(argv)


def add_home_file_arg(parser):
    parser.add_argument('--home-file', default=None,
                        help='홈(원위치) 자세를 읽고 저장할 YAML 파일 경로 '
                             f'(기본값 ${HOME_FILE_ENV} 또는 {DEFAULT_HOME_FILE})')


def parse_ui_args(argv):
    parser = argparse.ArgumentParser(
        prog='joint_control ui',
        description='슬라이더로 팔 관절과 그리퍼를 직접 움직이는 수동 제어 창을 띄운다. '
                    '시작할 때 홈 자세 파일을 읽어 그 자세로 팔을 초기화하고, '
                    "창의 'Go HOME'/'Set current as HOME' 버튼(또는 'h'/'s' 키)으로 "
                    '홈 자세로 이동하거나 현재 각도를 홈 자세 파일에 저장할 수 있다.')
    parser.add_argument('--duration', type=float, default=UI_MOVE_DURATION,
                        help='슬라이더를 움직였을 때 그 목표까지 가는 데 주는 시간 [sec] '
                             f'(작을수록 즉각 반응, 기본값 {UI_MOVE_DURATION})')
    parser.add_argument('--no-home-init', action='store_true',
                        help='시작할 때 홈 자세로 초기화하지 않고 현재 자세 그대로 창을 띄운다')
    add_home_file_arg(parser)
    return parser.parse_args(argv)


def parse_move_args(argv):
    parser = argparse.ArgumentParser(
        prog='joint_control',
        description='SO-101 팔 관절 각도를 지정해 목표 자세로 이동시킨다. '
                    '지정하지 않은 관절은 현재 각도를 그대로 유지한다.')
    parser.add_argument('--shoulder-pan', type=float, default=None, help='arm_shoulder_pan 목표각 [rad]')
    parser.add_argument('--shoulder-lift', type=float, default=None, help='arm_shoulder_lift 목표각 [rad]')
    parser.add_argument('--elbow-flex', type=float, default=None, help='arm_elbow_flex 목표각 [rad]')
    parser.add_argument('--wrist-flex', type=float, default=None, help='arm_wrist_flex 목표각 [rad]')
    parser.add_argument('--wrist-roll', type=float, default=None, help='arm_wrist_roll 목표각 [rad]')
    parser.add_argument('--gripper', type=float, default=None,
                        help='arm_gripper 목표 위치 (-0.17=닫힘 근처 ~ 1.75=열림 근처)')
    parser.add_argument('--duration', type=float, default=3.0,
                        help='팔 이동에 걸리는 시간 [sec] (기본값 3.0)')
    return parser.parse_args(argv)


def parse_pick_args(argv):
    parser = argparse.ArgumentParser(
        prog='joint_control pick',
        description='물체를 찾아 집는다. 기본은 손목 카메라 + OpenCV 색상 검출이고, '
                    '--at-pixel U V를 주면 상단 RGBD 뎁스카메라 픽셀 좌표를 3D로 되돌려 '
                    'MoveIt2(OMPL)로 그 지점을 집는다.')
    parser.add_argument('--color', default='pink', choices=list(COLOR_PRESETS),
                        help='(색상 검출 방식) 찾을 물체 색상 프리셋 (기본값 pink)')
    parser.add_argument('--at-pixel', type=float, nargs=2, default=None, metavar=('U', 'V'),
                        help="(MoveIt2 방식) 'rgbd' 카메라 화면의 픽셀 좌표 U V를 지정하면 "
                             '뎁스로 3D 위치를 구해 MoveIt2로 그 지점을 집는다. '
                             "먼저 'joint_control view --camera rgbd'로 좌표를 확인하세요.")
    add_home_file_arg(parser)
    return parser.parse_args(argv)


def parse_grip_args(argv):
    parser = argparse.ArgumentParser(
        prog='joint_control grip',
        description='좌표 X Y Z(기본 base_footprint 기준 [m])를 주면 MoveIt2로 그 위치의 물체를 '
                    '집은 뒤 원위치(홈 자세)로 돌아온다.')
    parser.add_argument('x', type=float, help='목표 x [m]')
    parser.add_argument('y', type=float, help='목표 y [m]')
    parser.add_argument('z', type=float, help='목표 z [m]')
    parser.add_argument('--frame', default=MOVEIT_PLANNING_FRAME,
                        help=f'좌표 기준 프레임 (기본값 {MOVEIT_PLANNING_FRAME})')
    parser.add_argument('--approach-height', type=float, default=PICK_APPROACH_HEIGHT,
                        help='집기 전/후에 목표 지점 위로 띄울 높이 [m] '
                             f'(기본값 {PICK_APPROACH_HEIGHT})')
    parser.add_argument('--z-offset', type=float, default=PICK_GRASP_Z_OFFSET,
                        help=f'목표 z에 더할 보정값 [m] (기본값 {PICK_GRASP_Z_OFFSET})')
    add_home_file_arg(parser)
    return parser.parse_args(argv)


def parse_place_args(argv):
    parser = argparse.ArgumentParser(
        prog='joint_control place',
        description='집은 물체를 내려놓는다. 좌표 X Y Z(기본 base_footprint 기준 [m])를 주면 '
                    'MoveIt2로 그 위치에 내려놓은 뒤 원위치(홈 자세)로 돌아오고, 좌표를 생략하면 '
                    '기존처럼 --pan 각도의 고정 자세로 내려놓는다.')
    parser.add_argument('xyz', type=float, nargs='*', metavar='X Y Z',
                        help='내려놓을 위치의 x y z [m] (3개를 모두 주거나 전부 생략)')
    parser.add_argument('--frame', default=MOVEIT_PLANNING_FRAME,
                        help=f'좌표 기준 프레임 (기본값 {MOVEIT_PLANNING_FRAME})')
    parser.add_argument('--approach-height', type=float, default=PICK_APPROACH_HEIGHT,
                        help='놓기 전/후에 목표 지점 위로 띄울 높이 [m] '
                             f'(기본값 {PICK_APPROACH_HEIGHT})')
    parser.add_argument('--z-offset', type=float, default=PLACE_RELEASE_Z_OFFSET,
                        help=f'목표 z에 더할 보정값 [m] (기본값 {PLACE_RELEASE_Z_OFFSET})')
    parser.add_argument('--pan', type=float, default=DEFAULT_PLACE_PAN,
                        help='(좌표를 생략했을 때) 내려놓을 위치의 shoulder_pan 각도 [rad]')
    add_home_file_arg(parser)
    parsed = parser.parse_args(argv)
    if len(parsed.xyz) not in (0, 3):
        parser.error('좌표는 X Y Z 3개를 모두 지정하거나 전부 생략해야 합니다.')
    return parsed


def main(args=None):
    argv = args if args is not None else sys.argv
    clean_argv = remove_ros_args(args=argv)[1:]

    if clean_argv and clean_argv[0] in ('view', 'ui', 'pick', 'grip', 'place'):
        command, rest = clean_argv[0], clean_argv[1:]
    else:
        command, rest = 'move', clean_argv

    if command == 'move':
        parsed = parse_move_args(rest)
        overrides = {
            'arm_shoulder_pan': parsed.shoulder_pan,
            'arm_shoulder_lift': parsed.shoulder_lift,
            'arm_elbow_flex': parsed.elbow_flex,
            'arm_wrist_flex': parsed.wrist_flex,
            'arm_wrist_roll': parsed.wrist_roll,
        }
        arm_requested = any(v is not None for v in overrides.values())
        if not arm_requested and parsed.gripper is None:
            print('오류: 관절 값을 최소 하나 이상 지정해야 합니다 (--help 참고).', file=sys.stderr)
            sys.exit(1)
    elif command == 'ui':
        parsed = parse_ui_args(rest)
    elif command == 'pick':
        parsed = parse_pick_args(rest)
    elif command == 'grip':
        parsed = parse_grip_args(rest)
    elif command == 'place':
        parsed = parse_place_args(rest)
    else:  # view
        parsed = parse_view_args(rest)

    node_name = 'jdamr_cube_so101_arm_control'
    view_topic = None
    if command == 'view':
        view_topic = parsed.topic or CAMERA_TOPICS[parsed.camera]
        # 카메라 창을 여러 개 동시에 띄울 때(예: wrist + rgbd) 노드 이름이 겹치지 않도록
        # 토픽 이름을 반영한 고유한 이름을 쓴다.
        node_name = 'jdamr_cube_camera_view_' + view_topic.replace('/', '_').replace('-', '_')
    elif command == 'ui':
        # 다른 arm 명령과 동시에 띄울 수 있으므로 노드 이름을 따로 쓴다.
        node_name = 'jdamr_cube_arm_joint_ui'

    rclpy.init(args=argv)
    node = So101ArmControl(node_name, home_file=getattr(parsed, 'home_file', None))
    ok = True

    if command == 'move':
        overrides = {j: v for j, v in overrides.items() if v is not None}
        if overrides:
            ok = node.move_joints(overrides, parsed.duration) and ok
        if parsed.gripper is not None:
            ok = node.send_gripper_goal(parsed.gripper) and ok
    elif command == 'view':
        node.view_camera(view_topic, parsed.near, parsed.far)
    elif command == 'ui':
        ok = node.control_ui(parsed.duration, init_home=not parsed.no_home_init)
    elif command == 'pick':
        if parsed.at_pixel is not None:
            ok = node.pick_at_pixel(parsed.at_pixel[0], parsed.at_pixel[1])
        else:
            ok = node.pick(parsed.color)
    elif command == 'grip':
        ok = node.grip_at_point(
            parsed.x, parsed.y, parsed.z,
            parsed.approach_height, parsed.z_offset, parsed.frame)
    elif command == 'place':
        if parsed.xyz:
            ok = node.place_at_point(
                parsed.xyz[0], parsed.xyz[1], parsed.xyz[2],
                parsed.approach_height, parsed.z_offset, parsed.frame)
        else:
            ok = node.place(parsed.pan)

    used_moveit = node._moveit_py is not None
    if used_moveit:
        node._moveit_py.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    if used_moveit:
        # moveit_py(2.12.4)의 C++ 소멸자가 프로세스 종료 시점에 segfault를 내는
        # 알려진 버그가 있다. 실제 작업은 이미 끝났으므로, 소멸자를 건너뛰고
        # 우리가 계산한 성공/실패 코드로 즉시 종료한다.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0 if ok else 1)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
