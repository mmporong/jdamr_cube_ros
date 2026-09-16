"""Record visual and reference odometry topics as timestamped pose CSV files."""

import argparse
import csv
from pathlib import Path

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


CSV_FIELDS = (
    'timestamp_s',
    'x_m',
    'y_m',
    'z_m',
    'qx',
    'qy',
    'qz',
    'qw',
    'frame_id',
    'child_frame_id',
)


class _CsvSink:
    """Own one trajectory CSV and flush each row for crash-safe evidence."""

    def __init__(self, output_path: Path):
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = output_path.open('w', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._stream, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self.count = 0

    def write(self, message: Odometry) -> None:
        """Append one odometry pose."""
        stamp = message.header.stamp
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self._writer.writerow({
            'timestamp_s': stamp.sec + stamp.nanosec * 1e-9,
            'x_m': position.x,
            'y_m': position.y,
            'z_m': position.z,
            'qx': orientation.x,
            'qy': orientation.y,
            'qz': orientation.z,
            'qw': orientation.w,
            'frame_id': message.header.frame_id,
            'child_frame_id': message.child_frame_id,
        })
        self._stream.flush()
        self.count += 1

    def close(self) -> None:
        """Close the underlying stream."""
        self._stream.close()


class TrajectoryCsvRecorder(Node):
    """Subscribe to RTAB-Map and wheel odometry without changing either."""

    def __init__(self, output_dir: Path, visual_topic: str,
                 reference_topic: str):
        super().__init__('jdamr_vslam_trajectory_csv_recorder')
        self._visual = _CsvSink(output_dir / 'visual_trajectory.csv')
        self._reference = _CsvSink(output_dir / 'reference_trajectory.csv')
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.create_subscription(
            Odometry, visual_topic, self._visual.write, sensor_qos)
        self.create_subscription(
            Odometry, reference_topic, self._reference.write, sensor_qos)
        self.get_logger().info(
            f'recording {visual_topic} and {reference_topic} into {output_dir}')

    def close(self) -> None:
        """Flush and close both output files."""
        visual_count = self._visual.count
        reference_count = self._reference.count
        self._visual.close()
        self._reference.close()
        print(f'rows: visual={visual_count}, reference={reference_count}')


def _parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--visual-topic', default='/rtabmap/odom')
    parser.add_argument('--reference-topic', default='/odom')
    return parser.parse_known_args(argv)


def main(args=None):
    """Run the CSV recorder until ROS shutdown."""
    parsed, ros_args = _parse_args(args)
    rclpy.init(args=ros_args)
    node = TrajectoryCsvRecorder(
        parsed.output_dir, parsed.visual_topic, parsed.reference_topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
