"""Remove recorded localization TF before an offline SLAM replay."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def _normalized_frame(frame_id):
    return frame_id.strip().lstrip('/')


def filter_tf_message(message, drop_parent='map', drop_child='odom'):
    """Return a TFMessage without the forbidden parent-child authority."""
    parent = _normalized_frame(drop_parent)
    child = _normalized_frame(drop_child)
    kept = [
        transform for transform in message.transforms
        if not (
            _normalized_frame(transform.header.frame_id) == parent and
            _normalized_frame(transform.child_frame_id) == child)
    ]
    return TFMessage(transforms=kept), len(message.transforms) - len(kept)


class TfReplayFilter(Node):
    """Bridge recorded TF topics while excluding AMCL map-to-odom."""

    def __init__(self):
        super().__init__('tf_replay_filter')
        self.declare_parameter('drop_parent', 'map')
        self.declare_parameter('drop_child', 'odom')
        self.drop_parent = self.get_parameter(
            'drop_parent').get_parameter_value().string_value
        self.drop_child = self.get_parameter(
            'drop_child').get_parameter_value().string_value
        self.dropped = 0

        dynamic_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.dynamic_publisher = self.create_publisher(
            TFMessage, '/tf', dynamic_qos)
        self.static_publisher = self.create_publisher(
            TFMessage, '/tf_static', static_qos)
        self.create_subscription(
            TFMessage, '/tf_recorded', self._dynamic_callback, dynamic_qos)
        self.create_subscription(
            TFMessage, '/tf_static_recorded', self._static_callback,
            static_qos)
        self.get_logger().info(
            f'offline TF guard active: dropping '
            f'{self.drop_parent} -> {self.drop_child}')

    def _filtered(self, message):
        filtered, dropped = filter_tf_message(
            message, self.drop_parent, self.drop_child)
        previous_total = self.dropped
        self.dropped += dropped
        crossed_report_boundary = (
            previous_total == 0 or
            self.dropped // 1000 > previous_total // 1000)
        if dropped and crossed_report_boundary:
            self.get_logger().warn(
                f'dropped {dropped} recorded localization transform(s); '
                f'total={self.dropped}')
        return filtered

    def _dynamic_callback(self, message):
        filtered = self._filtered(message)
        if filtered.transforms:
            self.dynamic_publisher.publish(filtered)

    def _static_callback(self, message):
        filtered = self._filtered(message)
        if filtered.transforms:
            self.static_publisher.publish(filtered)


def main():
    """Run the offline TF authority guard."""
    rclpy.init()
    node = TfReplayFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(
            f'offline TF guard stopped; total dropped={node.dropped}',
            flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
