"""PC display graph only: keep the latest RViz 2D Pose Estimate click in a JSON file.

The click never reaches the Pi here. The departure tool reads this file, refines the
pose by scan matching and passes it to activate_navigation --initial-pose.
"""
import json
import math
import os
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy


def main():
    out = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('rviz_initialpose_capture')

    def on_pose(message):
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        record = {'frame_id': message.header.frame_id,
                  'x_m': message.pose.pose.position.x, 'y_m': message.pose.pose.position.y,
                  'yaw_rad': yaw, 'received_unix_s': time.time(),
                  'source': 'rviz_2d_pose_estimate_on_pc_display_graph'}
        if record['frame_id'] != 'map' or not all(
                math.isfinite(record[k]) for k in ('x_m', 'y_m', 'yaw_rad')):
            node.get_logger().error(f'rejected click: {record}')
            return
        temporary = out + '.tmp'
        with open(temporary, 'w', encoding='utf-8') as stream:
            json.dump(record, stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, out)
        node.get_logger().info(f'initial pose click saved: {record}')

    node.create_subscription(PoseWithCovarianceStamped, '/initialpose', on_pose, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
