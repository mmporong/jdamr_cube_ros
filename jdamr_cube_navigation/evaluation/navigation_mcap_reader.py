"""Read navigation MCAPs, including rosbag channels without message definitions."""

from pathlib import Path
from types import SimpleNamespace


TOPIC_TYPES = {
    '/amcl_pose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
    '/battery_state': 'sensor_msgs/msg/BatteryState',
    '/collision_monitor_state': 'nav2_msgs/msg/CollisionMonitorState',
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/cmd_vel_nav': 'geometry_msgs/msg/Twist',
    '/imu/data_raw': 'sensor_msgs/msg/Imu',
    '/navigate_to_pose/_action/status': 'action_msgs/msg/GoalStatusArray',
    '/odom': 'nav_msgs/msg/Odometry',
    '/plan': 'nav_msgs/msg/Path',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/tf': 'tf2_msgs/msg/TFMessage',
}


def _installed_decoder(type_name):
    """Use the installed ROS type only for an explicitly allowed empty schema."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    message_type = get_message(type_name)
    return lambda data: deserialize_message(data, message_type)


def read_navigation_messages(path, topics):
    """Yield decoded messages in recorder order without replaying the ROS graph."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    if any(topic not in TOPIC_TYPES for topic in topics):
        raise ValueError('unsupported navigation topic')
    factory = DecoderFactory()
    decoders = {}
    with Path(path).open('rb') as stream:
        reader = make_reader(stream, validate_crcs=True)
        for schema, channel, message in reader.iter_messages(
                topics=topics, log_time_order=True):
            if (channel.message_encoding != 'cdr' or schema is None
                    or schema.name != TOPIC_TYPES[channel.topic]):
                raise ValueError(f'unexpected message type: {channel.topic}')
            if schema.id not in decoders:
                if schema.data:
                    decoder = factory.decoder_for('cdr', schema)
                    if decoder is None:
                        raise ValueError(f'unsupported schema: {schema.name}')
                else:
                    decoder = _installed_decoder(TOPIC_TYPES[channel.topic])
                decoders[schema.id] = decoder
            yield SimpleNamespace(
                channel=channel, log_time_ns=message.log_time,
                ros_msg=decoders[schema.id](message.data),
            )
