"""Exercise schema-less recordings and source-type rejection without ROS replay."""

from pathlib import Path
import sys

from mcap.writer import Writer
import pytest  # noqa: I201


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from navigation_mcap_reader import (  # noqa: E402,I100,I201
    read_navigation_messages,
)


def _write_bag(path, *, topic='/cmd_vel', type_name='geometry_msgs/msg/Twist',
               message_encoding='cdr', schema_encoding='', schema_data=b''):
    with path.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        schema_id = writer.register_schema(
            type_name, schema_encoding, schema_data)
        channel = writer.register_channel(topic, message_encoding, schema_id)
        writer.add_message(channel, 20, b'first', 10)
        writer.add_message(channel, 10, b'second', 5)
        writer.finish()


def test_empty_schema_uses_known_installed_type_in_recorder_order(
        tmp_path, monkeypatch):
    """Keep normal empty-schema rosbag messages instead of silently omitting them."""
    path = tmp_path / 'source.mcap'
    _write_bag(path)
    types = []

    def installed_decoder(type_name):
        types.append(type_name)
        return lambda data: data.decode()

    monkeypatch.setattr(
        'navigation_mcap_reader._installed_decoder', installed_decoder)
    messages = list(read_navigation_messages(path, ['/cmd_vel']))
    assert types == ['geometry_msgs/msg/Twist']
    assert [item.log_time_ns for item in messages] == [10, 20]
    assert [item.ros_msg for item in messages] == ['second', 'first']


@pytest.mark.parametrize('changes', [
    {'type_name': 'std_msgs/msg/String'},
    {'message_encoding': 'json'},
    {'schema_encoding': 'unknown', 'schema_data': b'not a ROS schema'},
])
def test_wrong_types_or_encodings_fail_closed(tmp_path, changes):
    """Never decode a differently typed channel as a valid velocity command."""
    path = tmp_path / 'source.mcap'
    _write_bag(path, **changes)
    with pytest.raises(ValueError):
        list(read_navigation_messages(path, ['/cmd_vel']))


def test_unknown_topic_is_rejected_before_reading(tmp_path):
    """Do not let a recording select arbitrary local message imports."""
    with pytest.raises(ValueError, match='unsupported navigation topic'):
        list(read_navigation_messages(tmp_path / 'absent.mcap', ['/arbitrary']))


def test_embedded_schema_does_not_require_installed_ros(tmp_path, monkeypatch):
    """Retain standalone decoding for the existing self-describing recordings."""
    path = tmp_path / 'source.mcap'
    _write_bag(path, schema_encoding='ros2msg', schema_data=b'float64 value')
    expected = object()
    monkeypatch.setattr(
        'mcap_ros2.decoder.DecoderFactory.decoder_for',
        lambda _self, _encoding, _schema: lambda _data: expected)

    def unexpected_import(_type):
        pytest.fail('installed ROS types must not be loaded for embedded schema')

    monkeypatch.setattr(
        'navigation_mcap_reader._installed_decoder', unexpected_import)
    assert all(item.ros_msg is expected
               for item in read_navigation_messages(path, ['/cmd_vel']))
