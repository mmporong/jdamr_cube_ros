import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GotoPose(Node):

    def __init__(self):
        super().__init__('jdamr_cube_goto_pose')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "'navigate_to_pose' action server not available. "
                'jdamr_cube_controller navigation.launch.py가 실행 중인지 확인하세요.')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f'목표 전송: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} (map frame)')
        send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('목표가 거부되었습니다.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('목적지에 도착했습니다.')
            return True

        self.get_logger().error(f'이동 실패 (status={status}).')
        return False

    def _feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'남은 거리: {remaining:.2f} m', throttle_duration_sec=2.0)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='저장된 맵(map frame) 기준 좌표로 jdamr_cube를 이동시킨다.')
    parser.add_argument('x', type=float, help='목표 x 좌표 [m]')
    parser.add_argument('y', type=float, help='목표 y 좌표 [m]')
    parser.add_argument('--yaw', type=float, default=0.0, help='목표 지점에서의 방향 [rad] (기본값 0)')
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main(args=None):
    argv = args if args is not None else sys.argv
    parsed = parse_args(argv)

    rclpy.init(args=argv)
    node = GotoPose()
    ok = node.send_goal(parsed.x, parsed.y, parsed.yaw)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
