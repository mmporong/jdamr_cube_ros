"""Camera-only replay must not import the wheel TF parent or motion topics."""

import importlib.util
from pathlib import Path

from geometry_msgs.msg import TransformStamped

from rclpy.serialization import deserialize_message, serialize_message

import rosbag2_py

from sensor_msgs.msg import Image

from tf2_msgs.msg import TFMessage


SCRIPT = (Path(__file__).resolve().parents[1]
          / 'scripts/prepare_visual_replay.py')
SPEC = importlib.util.spec_from_file_location('prepare_visual_replay', SCRIPT)
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)


def transform(parent, child):
    """Build a named edge without publishing it."""
    edge = TransformStamped()
    edge.header.frame_id = parent
    edge.child_frame_id = child
    return edge


def test_visual_tree_keeps_only_camera_descendants():
    """Preserve internal edges without changing the original message."""
    pairs = [('odom', 'base_link'), ('base_link', 'camera_link'),
             ('camera_link', 'camera_color_frame'),
             ('camera_color_frame', 'camera_color_optical_frame'),
             ('base_link', 'laser_link')]
    frames = replay.camera_descendants(pairs)
    message = TFMessage(transforms=[transform(*pair) for pair in pairs])
    filtered = replay.keep_camera_edges(message, frames)
    assert [(t.header.frame_id, t.child_frame_id)
            for t in filtered.transforms] == pairs[2:4]
    assert len(message.transforms) == 5


def test_root_cannot_acquire_an_incoming_parent():
    """Even malformed input cannot assign a parent to the camera root."""
    pairs = [('camera_link', 'optical'), ('optical', 'camera_link')]
    filtered = replay.keep_camera_edges(
        TFMessage(transforms=[transform(*pair) for pair in pairs]),
        replay.camera_descendants(pairs))
    assert len(filtered.transforms) == 1
    assert filtered.transforms[0].child_frame_id == 'optical'


def test_commands_are_not_replayed_and_wheel_odom_is_reference_only():
    """Exclude actuation while retaining wheel comparison data."""
    assert '/cmd_vel' not in replay.ALLOWED_TOPICS
    assert '/cmd_vel_smoothed' not in replay.ALLOWED_TOPICS
    assert '/odom' in replay.ALLOWED_TOPICS


def test_prepared_bag_preserves_image_bytes_and_excludes_wheel_tf(tmp_path):
    """Exercise actual MCAP serialization, filtering, and output metadata."""
    source, output = tmp_path / 'source', tmp_path / 'visual'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(source), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for name, message_type in [
            ('/tf_static', 'tf2_msgs/msg/TFMessage'),
            ('/camera/color/image_raw', 'sensor_msgs/msg/Image'),
            ('/cmd_vel', 'geometry_msgs/msg/Twist')]:
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=name, type=message_type, serialization_format='cdr'))
    tf = TFMessage(transforms=[transform('base_link', 'camera_link'),
                               transform('camera_link', 'camera_optical')])
    image = Image(height=1, width=1, encoding='rgb8', step=3,
                  data=[10, 20, 30])
    image.header.frame_id = 'camera_optical'
    raw_image = serialize_message(image)
    writer.write('/tf_static', serialize_message(tf), 1)
    writer.write('/camera/color/image_raw', raw_image, 2)
    del writer
    summary = replay.prepare(source, output)
    assert summary['removed_tf_edges'] == [['base_link', 'camera_link']]
    reader = replay.open_reader(output)
    names = [t.name for t in reader.get_all_topics_and_types()]
    assert '/cmd_vel' not in names
    messages = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        messages[topic] = (data, stamp)
    assert messages['/camera/color/image_raw'] == (raw_image, 2)
    edges = deserialize_message(messages['/tf_static'][0], TFMessage)
    assert len(edges.transforms) == 1
    assert edges.transforms[0].header.frame_id == 'camera_link'
