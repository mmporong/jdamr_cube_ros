"""One-way SSH display data, isolated from the robot's control graph."""

import argparse
import base64
import faulthandler
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


PREFIX = b'JDAMR_DISPLAY_V1 '
MAX_LINE_BYTES = 4 * 1024 * 1024
MAX_TF_FRAMES = 512
TOPICS = {
    '/tf': TFMessage,
    '/tf_static': TFMessage,
    '/robot_description': String,
    '/scan': LaserScan,
    '/plan': Path,
    '/amcl_pose': PoseWithCovarianceStamped,
}
LATCHED = {'/tf_static', '/robot_description'}


def wire_line(topic, message):
    """Serialize only the fixed display allowlist, with original ROS stamps."""
    if topic not in TOPICS or not isinstance(message, TOPICS[topic]):
        raise ValueError('topic/type is outside display allowlist')
    payload = base64.b64encode(serialize_message(message)).decode('ascii')
    line = PREFIX + json.dumps([topic, payload]).encode('ascii') + b'\n'
    if len(line) > MAX_LINE_BYTES:
        raise ValueError('display frame exceeds size limit')
    return line


def parse_line(line):
    """Ignore ROS log lines and reject malformed or unauthorized data."""
    if len(line) > MAX_LINE_BYTES:
        raise ValueError('display frame exceeds size limit')
    if not line.startswith(PREFIX):
        return None
    try:
        document = json.loads(line[len(PREFIX):])
        if (not isinstance(document, list) or len(document) != 2
                or not isinstance(document[0], str)
                or document[0] not in TOPICS
                or not isinstance(document[1], str)):
            raise ValueError('invalid display frame')
        topic, encoded = document
        payload = base64.b64decode(encoded, validate=True)
        message = deserialize_message(payload, TOPICS[topic])
    except Exception as error:
        raise ValueError('invalid display frame') from error
    return topic, message


def qos(topic, *, subscriber=False):
    return QoSProfile(
        depth=1 if subscriber or topic in LATCHED else 10,
        reliability=(ReliabilityPolicy.BEST_EFFORT
                     if subscriber and topic not in LATCHED
                     else ReliabilityPolicy.RELIABLE),
        durability=(DurabilityPolicy.TRANSIENT_LOCAL
                    if topic in LATCHED else DurabilityPolicy.VOLATILE))


def ssh_command(host, workspace):
    """Build one SSH argv without interpreting caller text as shell code."""
    if not re.fullmatch(r'[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+', host):
        raise ValueError('host must be user@host')
    if not workspace.startswith('/') or '\n' in workspace:
        raise ValueError('workspace must be an absolute path')
    setup = shlex.quote(workspace.rstrip('/') + '/install/setup.bash')
    remote = (
        'set +u; source /opt/ros/jazzy/setup.bash && source ' + setup + ' && '
        'export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=0 '
        'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST '
        'FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && '
        'exec nice -n 10 python3 -m jdamr_cube_navigation.rviz_display_relay export')
    return ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
            '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
            host, remote]


class DisplayExport(Node):
    """Subscribe on the Pi; never create any ROS publisher or action client."""

    def __init__(self):
        super().__init__('rviz_display_export')
        self.pending = {}
        self.transforms = {'/tf': {}, '/tf_static': {}}
        self.last_emit = {}
        self.received_counts = {}
        for topic, kind in TOPICS.items():
            self.create_subscription(
                kind, topic, lambda msg, name=topic: self.receive(name, msg),
                qos(topic, subscriber=True))
        self.create_timer(0.1, self.flush)
        self.create_timer(10.0, self.report)

    def receive(self, topic, message):
        self.received_counts[topic] = self.received_counts.get(topic, 0) + 1
        if topic in self.transforms:
            cache = self.transforms[topic]
            for transform in message.transforms:
                if (transform.child_frame_id not in cache
                        and len(cache) >= MAX_TF_FRAMES):
                    raise ValueError('display TF frame cache exceeds bound')
                cache[transform.child_frame_id] = transform
            # Keep callback work bounded; construct the aggregated TF message
            # only at the display send rate, not at every sensor callback.
            message = None
        self.pending[topic] = message

    def flush(self):
        now = time.monotonic()
        for topic in list(self.pending):
            interval = 0.1 if topic == '/tf' else 0.2
            if now - self.last_emit.get(topic, 0) < interval:
                continue
            message = self.pending.pop(topic)
            if topic in self.transforms:
                message = TFMessage(transforms=list(self.transforms[topic].values()))
            sys.stdout.buffer.write(wire_line(topic, message))
            self.last_emit[topic] = now
        sys.stdout.buffer.flush()

    def report(self):
        print('display export received=' + json.dumps(self.received_counts),
              file=sys.stderr, flush=True)


class DisplayImport(Node):
    """Publish display topics on the PC, never onto the Pi control context."""

    def __init__(self, process):
        super().__init__('rviz_display_import')
        self.process = process
        self.pending = {}
        self.lock = threading.Lock()
        self.finished = threading.Event()
        self.failure = None
        self.received_counts = {}
        self.publishers_by_topic = {
            topic: self.create_publisher(kind, topic, qos(topic))
            for topic, kind in TOPICS.items()}
        self.thread = threading.Thread(target=self.read_stream, daemon=True)
        self.thread.start()
        self.create_timer(0.1, self.flush)
        self.create_timer(10.0, self.report)

    def read_stream(self):
        try:
            while True:
                line = self.process.stdout.readline(MAX_LINE_BYTES + 1)
                if not line:
                    break
                result = parse_line(line)
                if result is not None:
                    topic, message = result
                    with self.lock:
                        self.pending[topic] = message
                        self.received_counts[topic] = (
                            self.received_counts.get(topic, 0) + 1)
        except (ValueError, OSError) as error:
            self.failure = str(error)
        finally:
            self.finished.set()

    def flush(self):
        with self.lock:
            pending, self.pending = self.pending, {}
        for topic, message in pending.items():
            self.publishers_by_topic[topic].publish(message)

    def report(self):
        with self.lock:
            counts = dict(self.received_counts)
        self.get_logger().info('display import received=' + json.dumps(counts))


def main(argv=None):
    faulthandler.register(signal.SIGUSR1, file=sys.stderr, all_threads=True)
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest='mode', required=True)
    modes.add_parser('export')
    remote = modes.add_parser('import')
    remote.add_argument('--host', required=True)
    remote.add_argument('--workspace', required=True)
    args = parser.parse_args(argv)
    process = None
    if args.mode == 'import':
        command = ssh_command(args.host, args.workspace)
        # The receiving graph is local-PC only; it cannot join Pi LOCALHOST DDS.
        os.environ.update(ROS_DOMAIN_ID='12', ROS_LOCALHOST_ONLY='0',
                          ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                          FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    rclpy.init(args=[])
    node = None
    try:
        if args.mode == 'export':
            node = DisplayExport()
            rclpy.spin(node)
        else:
            process = subprocess.Popen(command, stdout=subprocess.PIPE)
            node = DisplayImport(process)
            while rclpy.ok() and not node.finished.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
            node.flush()
            if node.failure:
                raise RuntimeError(node.failure)
            raise RuntimeError('Pi display connection ended')
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
