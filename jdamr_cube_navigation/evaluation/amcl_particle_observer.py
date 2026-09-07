#!/usr/bin/env python3
"""Observe one metrics-only AMCL deterministic replay run."""

from __future__ import annotations

import argparse
import hashlib
import math
import os
from pathlib import Path
import struct
import tempfile
import time

from amcl_fault_contract import canonical_json_bytes, G002_CLOCK_QUIET_NS
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE)
CLOUD_OBSERVATION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE)
POSE_OBSERVATION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)
PAIRING_CONTRACT = 'FIFO_PAIRED_NO_COMMON_UPDATE_ID'
OBSERVATION_QOS_CONTRACT = {
    'particle_cloud': 'BEST_EFFORT_VOLATILE_SENSOR_DATA_QOS_COMPATIBLE',
    'amcl_pose': 'RELIABLE_TRANSIENT_LOCAL_KEEP_LAST_1',
}


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def particle_payload(cloud: ParticleCloud) -> tuple[str, int]:
    """Return a deterministic digest excluding the cloud header stamp."""
    frame = cloud.header.frame_id.encode('utf-8')
    digest = hashlib.sha256()
    digest.update(struct.pack('<I', len(frame)))
    digest.update(frame)
    digest.update(struct.pack('<Q', len(cloud.particles)))
    for index, particle in enumerate(cloud.particles):
        pose = particle.pose
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w, particle.weight)
        if any(not math.isfinite(float(value)) for value in values):
            raise ValueError('particle cloud contains non-finite pose or weight')
        digest.update(struct.pack('<Q8d', index, *map(float, values)))
    return digest.hexdigest(), len(cloud.particles)


class ParticleObserver(Node):
    """Capture readiness, scan linkage, and ordered particle payloads."""

    def __init__(self, args):
        """Create the evaluation-only observer and its subscriptions."""
        super().__init__(
            'g002_amcl_particle_observer', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.args = args
        self.started_steady_ns = time.monotonic_ns()
        self.events = []
        self.map_count = 0
        self.odom_count = 0
        self.clock_count = 0
        self.clock_samples = []
        self.last_clock_arrival_steady_ns = None
        self.tf_count = 0
        self.map_odom_tf_count = 0
        self.tf_static_count = 0
        self.scan_count = 0
        self.pre_initial_scan_count = 0
        self.pre_initial_cloud_count = 0
        self.raw_cloud_received_count = 0
        self.amcl_pose_received_count = 0
        self.initialpose_count = 0
        self.clouds = []
        self.pending_clouds = []
        self.pending_poses = []
        self.callback_trace = []
        self.scan_stamps = []
        self.scan_arrival_steady_ns = {}
        self.scan_payload = hashlib.sha256()
        self.scan_header = hashlib.sha256()
        self.done = False
        self.failure = None
        self.last_publisher_match_counts = (-1, -1)
        self.last_snapshot_request = None
        self._event('observer_started')
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', MAP_QOS)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map, MAP_QOS)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom, SENSOR_QOS, raw=True)
        self.clock_sub = self.create_subscription(
            ClockMessage, '/clock', self._clock_raw, SENSOR_QOS, raw=True)
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self._tf, SENSOR_QOS, raw=True)
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._tf_static, MAP_QOS)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_raw, SENSOR_QOS, raw=True)
        self.cloud_sub = self.create_subscription(
            ParticleCloud, '/particle_cloud', self._cloud_raw,
            CLOUD_OBSERVATION_QOS, raw=True)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_raw,
            POSE_OBSERVATION_QOS, raw=True)
        self.timer = self.create_timer(
            0.02, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._write_state()

    def _event(self, name: str) -> None:
        self.events.append({
            'name': name,
            'steady_ns': time.monotonic_ns(),
            'ros_ns': self.get_clock().now().nanoseconds,
        })

    def _map(self, _message) -> None:
        self.map_count += 1
        if self.map_count == 1:
            self._event('map_received')
            self._write_state()

    def _odom(self, _serialized: bytes) -> None:
        self.odom_count += 1
        if self.odom_count == 1:
            self._event('first_odom_received')
            self._write_state()

    def _clock_raw(self, serialized: bytes, _message_info) -> None:
        message = deserialize_message(serialized, ClockMessage)
        ros_ns = _stamp_ns(message.clock)
        if self.clock_samples and ros_ns < self.clock_samples[-1]['ros_ns']:
            self.failure = 'observed /clock moved backwards'
            self.done = True
        arrival_steady_ns = time.monotonic_ns()
        self.clock_samples.append({
            'ros_ns': ros_ns, 'arrival_steady_ns': arrival_steady_ns})
        self.last_clock_arrival_steady_ns = arrival_steady_ns
        self.clock_count += 1
        if self.clock_count == 1:
            self._event('first_clock_received')
            self._write_state()

    def _tf(self, serialized: bytes) -> None:
        self.tf_count += 1
        message = deserialize_message(serialized, TFMessage)
        self.map_odom_tf_count += sum(
            transform.header.frame_id == 'map' and
            transform.child_frame_id == 'odom'
            for transform in message.transforms)
        if self.tf_count == 1:
            self._event('first_tf_received')
            self._write_state()

    def _tf_static(self, _message) -> None:
        self.tf_static_count += 1
        if self.tf_static_count == 1:
            self._event('tf_static_received')
            self._write_state()

    def _scan_raw(self, serialized: bytes) -> None:
        message = deserialize_message(serialized, LaserScan)
        stamp_ns = _stamp_ns(message.header.stamp)
        arrival_steady_ns = time.monotonic_ns()
        self.scan_count += 1
        self.scan_stamps.append(stamp_ns)
        self.scan_arrival_steady_ns.setdefault(stamp_ns, arrival_steady_ns)
        self.scan_header.update(f'{stamp_ns}\n'.encode())
        self.scan_payload.update(bytes(serialized))
        if self.initialpose_count == 0:
            self.pre_initial_scan_count += 1
        if self.scan_count == 1:
            self._event('first_scan_received')
        self._write_state()

    def _cloud_raw(self, serialized: bytes) -> None:
        try:
            self.raw_cloud_received_count += 1
            message = deserialize_message(serialized, ParticleCloud)
            if self.initialpose_count == 0:
                self.pre_initial_cloud_count += 1
                self._write_state()
                return
            if not self.scan_stamps:
                return
            digest, count = particle_payload(message)
            arrival_steady_ns = time.monotonic_ns()
            cloud_stream_index = self.raw_cloud_received_count - 1
            self.callback_trace.append({
                'kind': 'particle_cloud', 'stream_index': cloud_stream_index,
                'header_stamp_ns': _stamp_ns(message.header.stamp),
                'arrival_steady_ns': arrival_steady_ns})
            self.pending_clouds.append({
                'header_stamp_ns': _stamp_ns(message.header.stamp),
                'arrival_steady_ns': arrival_steady_ns,
                'callback_ros_ns': self.get_clock().now().nanoseconds,
                'frame_id': message.header.frame_id,
                'particle_count': count, 'payload_sha256': digest,
                'stream_index': cloud_stream_index,
            })
            self._drain_updates()
            self._write_state()
        except Exception as exc:
            self.failure = f'{type(exc).__name__}: {exc}'
            self.done = True
            self._write_state()

    def _pose_raw(self, serialized: bytes) -> None:
        try:
            self.amcl_pose_received_count += 1
            message = deserialize_message(
                serialized, PoseWithCovarianceStamped)
            if self.initialpose_count == 0:
                return
            values = (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
                *message.pose.covariance,
            )
            if any(not math.isfinite(float(value)) for value in values):
                raise ValueError('AMCL pose contains non-finite value')
            header_stamp_ns = _stamp_ns(message.header.stamp)
            if header_stamp_ns not in self.scan_stamps:
                return
            arrival_steady_ns = time.monotonic_ns()
            pose_stream_index = self.amcl_pose_received_count - 1
            self.callback_trace.append({
                'kind': 'amcl_pose', 'stream_index': pose_stream_index,
                'header_stamp_ns': header_stamp_ns,
                'arrival_steady_ns': arrival_steady_ns})
            self.pending_poses.append({
                'header_stamp_ns': header_stamp_ns,
                'arrival_steady_ns': arrival_steady_ns,
                'callback_ros_ns': self.get_clock().now().nanoseconds,
                'frame_id': message.header.frame_id,
                'stream_index': pose_stream_index,
                'pose': [float(value) for value in values[:7]],
                'covariance': [float(value) for value in
                               message.pose.covariance],
            })
            self._drain_updates()
            self._write_state()
        except Exception as exc:
            self.failure = f'{type(exc).__name__}: {exc}'
            self.done = True
            self._write_state()

    def _drain_updates(self) -> None:
        while (self.pending_clouds and self.pending_poses and
               len(self.clouds) < self.args.max_clouds):
            cloud = self.pending_clouds.pop(0)
            pose = self.pending_poses.pop(0)
            associated_scan_stamp_ns = pose['header_stamp_ns']
            if associated_scan_stamp_ns not in self.scan_stamps:
                raise ValueError('AMCL pose stamp is not an observed scan')
            scan_arrival_steady_ns = self.scan_arrival_steady_ns[
                associated_scan_stamp_ns]
            latency_ns = pose['arrival_steady_ns'] - scan_arrival_steady_ns
            if latency_ns < 0:
                raise ValueError('AMCL pose arrived before triggering scan')
            record = dict(cloud)
            cloud_stream_index = record.pop('stream_index')
            record.update({
                'index': len(self.clouds),
                'fifo_associated_pose_scan_header_stamp_ns':
                    associated_scan_stamp_ns,
                'pose_header_stamp_ns': associated_scan_stamp_ns,
                'pose_arrival_steady_ns': pose['arrival_steady_ns'],
                'pose_callback_ros_ns': pose['callback_ros_ns'],
                'pose_frame_id': pose['frame_id'],
                'pose': pose['pose'],
                'covariance': pose['covariance'],
                'scan_arrival_steady_ns': scan_arrival_steady_ns,
                'scan_to_pose_steady_ns': latency_ns,
                'cloud_stream_index': cloud_stream_index,
                'pose_stream_index': pose['stream_index'],
                'pair_arrival_delta_ns': abs(
                    cloud['arrival_steady_ns'] - pose['arrival_steady_ns']),
            })
            self.clouds.append(record)
            if len(self.clouds) == 1:
                self._event('first_post_scan_cloud_received')

    def _tick(self) -> None:
        self._service_snapshot_request()
        matched = (self.cloud_sub.get_publisher_count(),
                   self.pose_sub.get_publisher_count())
        if matched != self.last_publisher_match_counts:
            self.last_publisher_match_counts = matched
            self._write_state()
        if (self.args.initialpose_request.exists() and
                self.initialpose_count == 0):
            if not (self.map_count and self.odom_count and
                    self.tf_static_count):
                self.failure = 'initialpose requested before prerequisites'
                self.done = True
            else:
                message = PoseWithCovarianceStamped()
                message.header.frame_id = 'map'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose.position.x = self.args.initial_x_m
                message.pose.pose.position.y = self.args.initial_y_m
                message.pose.pose.orientation.z = math.sin(
                    self.args.initial_yaw_rad / 2.0)
                message.pose.pose.orientation.w = math.cos(
                    self.args.initial_yaw_rad / 2.0)
                message.pose.covariance[0] = 0.25
                message.pose.covariance[7] = 0.25
                message.pose.covariance[35] = (math.pi / 12.0) ** 2
                self.initialpose_pub.publish(message)
                self.initialpose_count = 1
                self._event('initialpose_published')
            self._write_state()
        if (len(self.clouds) >= self.args.max_clouds and
                self.map_odom_tf_count > 0 and not self.done):
            self._event('cloud_limit_reached')
            self.done = True
            self._write_state()

    def _snapshot(self) -> dict:
        publisher_matched = self.cloud_sub.get_publisher_count()
        tf_publisher_endpoints = []
        for endpoint in self.get_publishers_info_by_topic('/tf'):
            gid_bytes = [int(value) for value in endpoint.endpoint_gid]
            qos = endpoint.qos_profile
            tf_publisher_endpoints.append({
                'node_name': endpoint.node_name,
                'node_namespace': endpoint.node_namespace,
                'topic_type': endpoint.topic_type,
                'endpoint_gid_bytes': gid_bytes,
                'endpoint_gid_hex': bytes(gid_bytes).hex(),
                'qos': {
                    'reliability': int(qos.reliability),
                    'durability': int(qos.durability),
                    'history': int(qos.history),
                    'depth': int(qos.depth),
                },
            })
        tf_publisher_endpoints.sort(
            key=lambda item: (item['node_namespace'], item['node_name'],
                              item['endpoint_gid_hex']))
        return {
            'schema_version': 1,
            'run_id': self.args.run_id,
            'seed': self.args.seed,
            'max_clouds': self.args.max_clouds,
            'publisher_matched_count': publisher_matched,
            'pose_publisher_matched_count': self.pose_sub.get_publisher_count(),
            'events': self.events,
            'readiness': {
                'map_count': self.map_count,
                'clock_count': self.clock_count,
                'odom_count': self.odom_count,
                'tf_count': self.tf_count,
                'map_odom_tf_count': self.map_odom_tf_count,
                'tf_static_count': self.tf_static_count,
            },
            'tf_publisher_endpoints': tf_publisher_endpoints,
            'initialpose_count': self.initialpose_count,
            'pre_initial_scan_count': self.pre_initial_scan_count,
            'pre_initial_cloud_count': self.pre_initial_cloud_count,
            'raw_cloud_received_count': self.raw_cloud_received_count,
            'amcl_pose_received_count': self.amcl_pose_received_count,
            'pending_cloud_count': len(self.pending_clouds),
            'pending_pose_count': len(self.pending_poses),
            'scan_count': self.scan_count,
            'scan_header_stamps_ns': self.scan_stamps,
            'scan_header_stamp_sha256': self.scan_header.hexdigest(),
            'scan_payload_sha256': self.scan_payload.hexdigest(),
            'clock_samples': self.clock_samples,
            'callback_trace': self.callback_trace,
            'pairing_contract': PAIRING_CONTRACT,
            'observation_qos_contract': OBSERVATION_QOS_CONTRACT,
            'pose_causality': 'NOT_PROVEN',
            'clouds': self.clouds,
            'failure': self.failure,
            'done': self.done,
            'motion_command_applicability': 'NOT_APPLICABLE',
        }

    def _service_snapshot_request(self) -> None:
        request = self.args.snapshot_request
        if not request.is_file():
            return
        token = request.read_text(encoding='utf-8').strip()
        if not token or token == self.last_snapshot_request:
            return
        if token != 'prelude_complete':
            return
        now_ns = time.monotonic_ns()
        endpoint_count = len(self.get_publishers_info_by_topic('/clock'))
        if (endpoint_count != 0 or self.last_clock_arrival_steady_ns is None or
                now_ns - self.last_clock_arrival_steady_ns <
                G002_CLOCK_QUIET_NS):
            return
        payload = canonical_json_bytes(self._snapshot())
        self._write_state(payload)
        snapshot_path = self.args.snapshot_ack.parent / (
            f'prelude_snapshot.{token}.json')
        self._atomic_write_new(snapshot_path, payload)
        ack_steady_ns = time.monotonic_ns()
        acknowledgement = {
            'request_token': token,
            'snapshot_ref': snapshot_path.name,
            'snapshot_sha256': hashlib.sha256(payload).hexdigest(),
            'clock_count': self.clock_count,
            'last_clock_arrival_steady_ns':
                self.last_clock_arrival_steady_ns,
            'ack_steady_ns': ack_steady_ns,
            'observed_quiet_ns':
                ack_steady_ns - self.last_clock_arrival_steady_ns,
            'publisher_endpoint_count': endpoint_count}
        self._atomic_write_new(
            self.args.snapshot_ack, canonical_json_bytes(acknowledgement))
        self.last_snapshot_request = token

    @staticmethod
    def _atomic_write_new(path: Path, payload: bytes) -> None:
        with tempfile.NamedTemporaryFile(
                dir=path.parent, delete=False) as stream:
            temp_path = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temp_path, path)
        finally:
            temp_path.unlink(missing_ok=True)

    def _write_state(self, payload: bytes | None = None) -> str:
        self.args.state.parent.mkdir(parents=True, exist_ok=True)
        if payload is None:
            payload = canonical_json_bytes(self._snapshot())
        with tempfile.NamedTemporaryFile(
                dir=self.args.state.parent, delete=False) as stream:
            temp_path = Path(stream.name)
            stream.write(payload)
        temp_path.replace(self.args.state)
        return hashlib.sha256(payload).hexdigest()


def main() -> int:
    """Run the observer until its deterministic cloud or wall limit."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--seed', required=True, type=int)
    parser.add_argument('--max-clouds', type=int, default=30)
    parser.add_argument('--state', required=True, type=Path)
    parser.add_argument('--initialpose-request', required=True, type=Path)
    parser.add_argument('--snapshot-request', required=True, type=Path)
    parser.add_argument('--snapshot-ack', required=True, type=Path)
    parser.add_argument('--initial-x-m', type=float, default=0.0)
    parser.add_argument('--initial-y-m', type=float, default=0.0)
    parser.add_argument('--initial-yaw-rad', type=float, default=0.0)
    args = parser.parse_args()
    rclpy.init()
    node = ParticleObserver(args)
    deadline = time.monotonic() + 180.0
    try:
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.done:
            node.failure = 'observer wall timeout'
        node._event('observer_finalized')
        node._write_state()
        return 0 if node.failure is None and node.done else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
