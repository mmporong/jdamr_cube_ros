"""One-way display protocol preserves original data and rejects control."""

import base64
import io
import json
from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped, Twist
from jdamr_cube_navigation.rviz_display_relay import (
    LATCHED, MAX_LINE_BYTES, parse_line, PREFIX, qos, ssh_command, TOPICS,
    wire_line,
)
import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def test_roundtrip_preserves_tf_stamp_and_translation_exactly():
    tf = TransformStamped()
    tf.header.stamp.sec, tf.header.stamp.nanosec = 5, 123456789
    tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
    tf.transform.translation.x = 1.125
    tf.transform.rotation.w = 1.0
    message = TFMessage(transforms=[tf])
    topic, actual = parse_line(wire_line('/tf', message))
    assert topic == '/tf'
    assert serialize_message(actual) == serialize_message(message)


def test_scan_nan_inf_and_original_stamp_are_preserved():
    message = LaserScan()
    message.header.stamp.sec = 1
    message.ranges = [float('nan'), float('inf'), 0.35]
    _, actual = parse_line(wire_line('/scan', message))
    assert serialize_message(actual) == serialize_message(message)


@pytest.mark.parametrize('topic', [
    '/cmd_vel', '/cmd_vel_nav', '/initialpose',
    '/navigate_to_pose/_action/status', '/map'])
def test_control_and_unlisted_topics_are_rejected(topic):
    with pytest.raises(ValueError):
        wire_line(topic, Twist())
    encoded = base64.b64encode(serialize_message(Twist())).decode('ascii')
    with pytest.raises(ValueError):
        parse_line(PREFIX + json.dumps([topic, encoded]).encode())


def test_incorrect_message_type_is_rejected():
    with pytest.raises(ValueError):
        wire_line('/scan', Twist())


@pytest.mark.parametrize('line', [
    PREFIX + b'not json', PREFIX + b'["/scan", "invalid!"]',
    PREFIX + b'{}', PREFIX + b'[null, ""]',
    b'x' * (MAX_LINE_BYTES + 1)])
def test_invalid_and_oversized_frames_fail_closed(line):
    with pytest.raises(ValueError):
        parse_line(line)


def test_ros_logs_are_not_parsed_as_display_data():
    assert parse_line(b'[INFO] some ROS logging\n') is None


def test_static_topics_keep_latched_qos_and_sensor_subscriptions_match():
    assert LATCHED == {'/tf_static', '/robot_description'}
    assert qos('/tf_static').durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos('/scan', subscriber=True).reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos('/scan', subscriber=True).depth == 1
    assert qos('/tf', subscriber=True).depth == 1
    assert all('cmd' not in topic and '_action' not in topic for topic in TOPICS)


def test_ssh_command_quotes_workspace_and_is_read_only_export():
    command = ssh_command('lim@192.168.0.159', '/tmp/space name')
    assert command[:2] == ['ssh', '-T']
    assert "source '/tmp/space name/install/setup.bash'" in command[-1]
    assert command[-1].endswith('rviz_display_relay export')
    assert 'exec nice -n 10 python3' in command[-1]


@pytest.mark.parametrize('host', ['-evil', 'lim@host; echo wrong', '$(echo wrong)'])
def test_ssh_host_rejects_shell_and_option_injection(host):
    with pytest.raises(ValueError):
        ssh_command(host, '/tmp/workspace')


def test_export_tf_coalesces_at_flush_and_keeps_last_original_stamp(monkeypatch):
    from jdamr_cube_navigation import rviz_display_relay as relay
    node = object.__new__(relay.DisplayExport)
    node.pending, node.last_emit, node.received_counts = {}, {}, {}
    node.transforms = {'/tf': {}, '/tf_static': {}}
    for stamp_s in range(1, 101):
        tf = TransformStamped()
        tf.child_frame_id = 'base_link'
        tf.header.frame_id = 'odom'
        tf.header.stamp.sec = stamp_s
        node.receive('/tf', TFMessage(transforms=[tf]))
    assert node.pending['/tf'] is None
    assert node.received_counts['/tf'] == 100
    output = io.BytesIO()
    monkeypatch.setattr(relay.sys, 'stdout', SimpleNamespace(buffer=output))
    node.flush()
    lines = output.getvalue().splitlines(keepends=True)
    assert len(lines) == 1
    topic, message = parse_line(lines[0])
    assert topic == '/tf' and message.transforms[0].header.stamp.sec == 100


def test_display_rates_keep_latest_tf_and_scan_without_retimestamping(monkeypatch):
    from jdamr_cube_navigation import rviz_display_relay as relay
    node = object.__new__(relay.DisplayExport)
    node.pending, node.last_emit, node.received_counts = {}, {}, {}
    node.transforms = {'/tf': {}, '/tf_static': {}}
    output = io.BytesIO()
    monkeypatch.setattr(relay.sys, 'stdout', SimpleNamespace(buffer=output))
    now = [10.0]
    monkeypatch.setattr(relay.time, 'monotonic', lambda: now[0])
    for index, offset in enumerate((0.0, .06, .11, .16, .21)):
        now[0] = 10.0 + offset
        tf = TransformStamped()
        tf.child_frame_id = 'base_link'
        tf.header.stamp.sec = index + 1
        scan = LaserScan()
        scan.header.stamp.sec = index + 1
        node.receive('/tf', TFMessage(transforms=[tf]))
        node.receive('/scan', scan)
        node.flush()
    decoded = [parse_line(line) for line in output.getvalue().splitlines(keepends=True)]
    assert [msg.transforms[0].header.stamp.sec for topic, msg in decoded
            if topic == '/tf'] == [1, 3, 5]
    assert [msg.header.stamp.sec for topic, msg in decoded if topic == '/scan'] == [1, 5]
