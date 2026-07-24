import argparse
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    'arm_shoulder_pan',
    'arm_shoulder_lift',
    'arm_elbow_flex',
    'arm_wrist_flex',
    'arm_wrist_roll',
]
GRIPPER_JOINT = 'arm_gripper'

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


def colorize_depth(depth, near=DEPTH_VIEW_NEAR, far=DEPTH_VIEW_FAR):
    """rgbd_camera의 32FC1 depth 원본을 화면에 보이는 8비트 이미지로 바꾼다.
    near~far 범위 안에 있는 픽셀은 흰색, 벗어나면(너무 가깝거나 멀거나 반사가 없는 NaN/inf)
    검정으로 그린다."""
    in_range = np.isfinite(depth) & (depth >= near) & (depth <= far)
    mask = np.where(in_range, np.uint8(255), np.uint8(0))
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


class So101ArmControl(Node):

    def __init__(self, node_name='jdamr_cube_so101_arm_control'):
        super().__init__(node_name)
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(
            self, GripperCommand, 'gripper_controller/gripper_cmd')
        self._latest_arm_state = None
        self._latest_image = None
        self._bridge = CvBridge()
        self.create_subscription(JointState, 'joint_states', self._joint_state_cb, 10)
        self.create_subscription(Image, CAMERA_TOPIC, self._image_cb, 1)

    def _joint_state_cb(self, msg):
        if all(j in msg.name for j in ARM_JOINTS):
            self._latest_arm_state = msg

    def _image_cb(self, msg):
        self._latest_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def wait_for_current_arm_positions(self, timeout_sec=5.0):
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self._latest_arm_state is None:
            if self.get_clock().now() > deadline:
                return None
            rclpy.spin_once(self, timeout_sec=0.2)
        msg = self._latest_arm_state
        return {j: msg.position[msg.name.index(j)] for j in ARM_JOINTS + [GRIPPER_JOINT]
                if j in msg.name}

    def wait_for_camera_image(self, timeout_sec=5.0):
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        self._latest_image = None
        while rclpy.ok() and self._latest_image is None:
            if self.get_clock().now() > deadline:
                return None
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._latest_image

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

    def send_arm_goal(self, target_positions, duration_sec):
        if not self._arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "'arm_controller/follow_joint_trajectory' 액션 서버가 없습니다. "
                'jdamr_cube_gazebo/gazebo.launch.py가 실행 중인지 확인하세요.')
            return False

        point = JointTrajectoryPoint()
        point.positions = [target_positions[j] for j in ARM_JOINTS]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(ARM_JOINTS)
        goal_msg.trajectory.points = [point]

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
        description='손목 카메라 + OpenCV 색상 검출로 물체를 찾아 집는다.')
    parser.add_argument('--color', default='pink', choices=list(COLOR_PRESETS),
                        help='찾을 물체 색상 프리셋 (기본값 pink)')
    return parser.parse_args(argv)


def parse_place_args(argv):
    parser = argparse.ArgumentParser(
        prog='joint_control place',
        description='pick으로 집은 물체를 내려놓는다.')
    parser.add_argument('--pan', type=float, default=DEFAULT_PLACE_PAN,
                        help='내려놓을 위치의 shoulder_pan 각도 [rad] (기본값 pick 지점과 다른 위치)')
    return parser.parse_args(argv)


def main(args=None):
    argv = args if args is not None else sys.argv
    clean_argv = remove_ros_args(args=argv)[1:]

    if clean_argv and clean_argv[0] in ('view', 'pick', 'place'):
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
    elif command == 'pick':
        parsed = parse_pick_args(rest)
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

    rclpy.init(args=argv)
    node = So101ArmControl(node_name)
    ok = True

    if command == 'move':
        overrides = {j: v for j, v in overrides.items() if v is not None}
        if overrides:
            ok = node.move_joints(overrides, parsed.duration) and ok
        if parsed.gripper is not None:
            ok = node.send_gripper_goal(parsed.gripper) and ok
    elif command == 'view':
        node.view_camera(view_topic, parsed.near, parsed.far)
    elif command == 'pick':
        ok = node.pick(parsed.color)
    elif command == 'place':
        ok = node.place(parsed.pan)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
