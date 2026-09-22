"""Publish a fail-closed readiness contract for operator-driven mapping."""

from collections import deque
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time

from lifecycle_msgs.srv import GetState

from nav_msgs.msg import OccupancyGrid, Odometry

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import BatteryState, LaserScan

from std_msgs.msg import String

from tf2_ros import Buffer, TransformException, TransformListener

import yaml


REQUIRED_NODES = {
    '/cartographer_node',
    '/collision_monitor',
    '/jdamr_base_driver',
    '/map_saver',
    '/occupancy_grid_node',
    '/operator_mapping_controller',
    '/robot_state_publisher',
    '/velocity_smoother',
    '/ydlidar_g4_node',
}
FORBIDDEN_NODES = {
    '/controller_server',
    '/frontier_explorer',
    '/motion_probe',
    '/web_teleop',
}
TOPIC_CHAIN = {
    '/cmd_vel_nav': ({'/operator_mapping_controller'}, {'/velocity_smoother'}),
    '/cmd_vel_smoothed': ({'/velocity_smoother'}, {'/collision_monitor'}),
    '/cmd_vel': ({'/collision_monitor'}, {'/jdamr_base_driver'}),
}
LIFECYCLE_NODES = ('velocity_smoother', 'collision_monitor', 'map_saver')


def _full_name(name, namespace):
    namespace = namespace.rstrip('/')
    return f'{namespace}/{name}' if namespace else f'/{name}'


def endpoint_names(endpoints):
    """Return fully qualified node names for graph endpoint information."""
    return {
        _full_name(endpoint.node_name, endpoint.node_namespace)
        for endpoint in endpoints
    }


def sample_metrics(arrivals, now):
    """Return freshness, rate and maximum recent receive gap."""
    values = list(arrivals)
    if not values:
        return {'age_s': None, 'rate_hz': 0.0, 'max_gap_s': None}
    if len(values) == 1:
        return {
            'age_s': now - values[-1],
            'rate_hz': 0.0,
            'max_gap_s': None,
        }
    gaps = [end - start for start, end in zip(values, values[1:])]
    duration = values[-1] - values[0]
    return {
        'age_s': now - values[-1],
        'rate_hz': (len(values) - 1) / duration if duration > 0.0 else 0.0,
        'max_gap_s': max(gaps),
    }


def point_in_polygon(x, y, polygon):
    """Return whether a 2D point lies inside a simple polygon."""
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if ((y1 > y) != (y2 > y)
                and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
        previous = current
    return inside


def directional_counts(points, polygons):
    """Count scan points inside each Collision Monitor direction polygon."""
    return {
        direction: sum(
            point_in_polygon(x, y, polygon) for x, y in points)
        for direction, polygon in polygons.items()
    }


def readiness_blockers(checks):
    """Translate measured checks into an ordered Korean blocker list."""
    blockers = []
    missing = checks['nodes']['missing']
    duplicates = checks['nodes']['duplicates']
    forbidden = checks['nodes']['forbidden']
    if missing:
        blockers.append(f'필수 노드 없음: {", ".join(missing)}')
    if duplicates:
        blockers.append(f'중복 노드: {", ".join(duplicates)}')
    if forbidden:
        blockers.append(f'충돌 노드 실행 중: {", ".join(forbidden)}')
    for topic, result in checks['command_chain'].items():
        if not result['valid']:
            blockers.append(f'명령 경로 불일치: {topic}')
    for name, state in checks['lifecycle'].items():
        if state != 'active':
            blockers.append(f'{name} 상태: {state}')
    scan = checks['scan']
    if scan['age_s'] is None or scan['age_s'] > 0.5:
        blockers.append('라이다 입력 지연')
    elif scan['rate_hz'] < 7.0:
        blockers.append(f'라이다 주기 부족: {scan["rate_hz"]:.1f}Hz')
    elif scan['max_gap_s'] is None or scan['max_gap_s'] > 0.5:
        blockers.append('라이다 입력 공백')
    odom = checks['odom']
    if odom['age_s'] is None or odom['age_s'] > 0.3:
        blockers.append('오도메트리 입력 지연')
    elif odom['rate_hz'] < 30.0:
        blockers.append(f'오도메트리 주기 부족: {odom["rate_hz"]:.1f}Hz')
    elif odom['max_gap_s'] is None or odom['max_gap_s'] > 0.2:
        blockers.append('오도메트리 입력 공백')
    if not checks['stationary']:
        blockers.append('차체가 정지 상태가 아님')
    mapping = checks['map']
    if mapping['age_s'] is None or mapping['age_s'] > 2.5:
        blockers.append('지도 갱신 지연')
    elif (not mapping['valid_size']
          or abs(mapping['resolution_m'] - 0.05) > 1e-6):
        blockers.append('지도 크기 또는 해상도 불일치')
    battery = checks['battery']
    if battery['age_s'] is None or battery['age_s'] > 5.0:
        blockers.append('배터리 상태 입력 지연')
    elif battery['voltage_v'] < 10.5:
        blockers.append(f'저전압: {battery["voltage_v"]:.2f}V')
    if not checks['tf']['map_to_base']:
        blockers.append('map→base_footprint TF 없음')
    if not checks['tf']['base_to_laser']:
        blockers.append('base_footprint→laser_link TF 없음')
    system = checks['system']
    if system['temperature_c'] >= 80.0:
        blockers.append(f'파이 고온: {system["temperature_c"]:.1f}°C')
    if system['current_throttle_bits'] != 0:
        blockers.append('파이 현재 저전압·스로틀 상태')
    if system['free_bytes'] < 512 * 1024 * 1024:
        blockers.append('저장 공간 512MB 미만')
    return blockers


class OperatorMappingPreflight(Node):
    """Measure every prerequisite without publishing a velocity command."""

    def __init__(self):
        """Create graph, sensor, lifecycle and TF readiness observers."""
        super().__init__('operator_mapping_preflight')
        self.scan_arrivals = deque(maxlen=50)
        self.odom_arrivals = deque(maxlen=100)
        self.map_arrived_at = None
        self.battery_arrived_at = None
        self.map_message = None
        self.battery_message = None
        self.odom_message = None
        self.scan_message = None
        self.lifecycle_states = {name: 'unknown' for name in LIFECYCLE_NODES}
        self.lifecycle_futures = {}
        self.last_system_check = 0.0
        self.system_status = {
            'temperature_c': 0.0,
            'current_throttle_bits': 0,
            'historical_throttle_bits': 0,
            'free_bytes': 0,
            'load_1m': 0.0,
        }
        self.declare_parameter('params_file', '')
        params_file = str(self.get_parameter('params_file').value)
        if not params_file:
            raise RuntimeError('operator preflight requires params_file')
        params = yaml.safe_load(
            Path(params_file).read_text(encoding='utf-8'))
        stop_zone = params['collision_monitor']['ros__parameters']['StopZone']
        self.stop_min_points = int(stop_zone['min_points'])
        self.direction_polygons = {
            'forward': json.loads(stop_zone['translation_forward']['points']),
            'backward': json.loads(stop_zone['translation_backward']['points']),
            'left': json.loads(stop_zone['rotation']['points']),
            'right': json.loads(stop_zone['rotation_clockwise']['points']),
        }
        self.create_subscription(
            LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odom', self._odom, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, '/map', self._map, map_qos)
        self.create_subscription(
            BatteryState, '/battery_state', self._battery,
            qos_profile_sensor_data)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, '/operator_mapping/preflight', status_qos)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in LIFECYCLE_NODES
        }
        self.create_timer(1.0, self._request_lifecycle_states)
        self.create_timer(0.25, self._publish_status)

    def _scan(self, message):
        self.scan_arrivals.append(time.monotonic())
        self.scan_message = message

    def _odom(self, message):
        self.odom_arrivals.append(time.monotonic())
        self.odom_message = message

    def _map(self, message):
        self.map_arrived_at = time.monotonic()
        self.map_message = message

    def _battery(self, message):
        self.battery_arrived_at = time.monotonic()
        self.battery_message = message

    def _request_lifecycle_states(self):
        for name, client in self.lifecycle_clients.items():
            current = self.lifecycle_futures.get(name)
            if current is not None and not current.done():
                continue
            if not client.service_is_ready():
                self.lifecycle_states[name] = 'service_unavailable'
                continue
            future = client.call_async(GetState.Request())
            self.lifecycle_futures[name] = future
            future.add_done_callback(
                lambda completed, node_name=name:
                self._lifecycle_result(node_name, completed))

    def _lifecycle_result(self, name, future):
        try:
            self.lifecycle_states[name] = future.result().current_state.label
        except Exception as error:  # ROS future transports the exact failure.
            self.lifecycle_states[name] = f'query_failed:{type(error).__name__}'

    def _graph_checks(self):
        nodes = [
            _full_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        ]
        missing = sorted(REQUIRED_NODES - set(nodes))
        duplicates = sorted(
            name for name in REQUIRED_NODES if nodes.count(name) > 1)
        forbidden = sorted(FORBIDDEN_NODES & set(nodes))
        command_chain = {}
        for topic, (expected_publishers, expected_subscribers) in (
                TOPIC_CHAIN.items()):
            publishers = endpoint_names(
                self.get_publishers_info_by_topic(topic))
            subscribers = endpoint_names(
                self.get_subscriptions_info_by_topic(topic))
            command_chain[topic] = {
                'publishers': sorted(publishers),
                'subscribers': sorted(subscribers),
                'valid': (
                    publishers == expected_publishers
                    and subscribers == expected_subscribers),
            }
        return {
            'nodes': {
                'missing': missing,
                'duplicates': duplicates,
                'forbidden': forbidden,
            },
            'command_chain': command_chain,
        }

    def _tf_available(self, target, source):
        try:
            return self.tf_buffer.can_transform(
                target, source, Time(), timeout=Duration(seconds=0.05))
        except TransformException:
            return False

    def _direction_checks(self):
        message = self.scan_message
        if message is None:
            return {
                name: {'clear': False, 'point_count': None}
                for name in self.direction_polygons
            }
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_footprint', message.header.frame_id, Time())
        except TransformException:
            return {
                name: {'clear': False, 'point_count': None}
                for name in self.direction_polygons
            }
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
        )
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        points = []
        angle = message.angle_min
        for distance in message.ranges:
            if (math.isfinite(distance)
                    and message.range_min <= distance <= message.range_max):
                laser_x = distance * math.cos(angle)
                laser_y = distance * math.sin(angle)
                points.append((
                    translation.x + cosine * laser_x - sine * laser_y,
                    translation.y + sine * laser_x + cosine * laser_y,
                ))
            angle += message.angle_increment
        counts = directional_counts(points, self.direction_polygons)
        return {
            name: {
                'clear': count < self.stop_min_points,
                'point_count': count,
            }
            for name, count in counts.items()
        }

    def _read_system(self, now):
        if now - self.last_system_check < 2.0:
            return self.system_status
        self.last_system_check = now
        temperature_path = Path('/sys/class/thermal/thermal_zone0/temp')
        try:
            temperature_c = float(temperature_path.read_text().strip()) / 1000.0
        except (OSError, ValueError):
            temperature_c = math.inf
        current_bits = -1
        historical_bits = -1
        try:
            output = subprocess.run(
                ['vcgencmd', 'get_throttled'], check=True,
                capture_output=True, text=True, timeout=1.0).stdout.strip()
            value = int(output.split('=', 1)[1], 16)
            current_bits = value & 0xFFFF
            historical_bits = value >> 16
        except (OSError, ValueError, IndexError, subprocess.SubprocessError):
            pass
        disk = shutil.disk_usage(Path.home())
        self.system_status = {
            'temperature_c': temperature_c,
            'current_throttle_bits': current_bits,
            'historical_throttle_bits': historical_bits,
            'free_bytes': disk.free,
            'load_1m': os.getloadavg()[0],
        }
        return self.system_status

    def _publish_status(self):
        now = time.monotonic()
        graph = self._graph_checks()
        scan = sample_metrics(self.scan_arrivals, now)
        odom = sample_metrics(self.odom_arrivals, now)
        if self.odom_message is None:
            linear_x = math.inf
            angular_z = math.inf
        else:
            linear_x = self.odom_message.twist.twist.linear.x
            angular_z = self.odom_message.twist.twist.angular.z
        mapping = self.map_message
        battery = self.battery_message
        checks = {
            **graph,
            'lifecycle': dict(self.lifecycle_states),
            'scan': scan,
            'odom': odom,
            'stationary': (
                abs(linear_x) <= 0.01 and abs(angular_z) <= 0.02),
            'measured_velocity': {
                'linear_x': linear_x, 'angular_z': angular_z},
            'map': {
                'age_s': (
                    None if self.map_arrived_at is None
                    else now - self.map_arrived_at),
                'resolution_m': (
                    0.0 if mapping is None else mapping.info.resolution),
                'valid_size': (
                    mapping is not None
                    and mapping.info.width > 0 and mapping.info.height > 0),
            },
            'battery': {
                'age_s': (
                    None if self.battery_arrived_at is None
                    else now - self.battery_arrived_at),
                'voltage_v': (
                    0.0 if battery is None else battery.voltage),
            },
            'tf': {
                'map_to_base': self._tf_available(
                    'map', 'base_footprint'),
                'base_to_laser': self._tf_available(
                    'base_footprint', 'laser_link'),
            },
            'directions': self._direction_checks(),
            'system': self._read_system(now),
        }
        blockers = readiness_blockers(checks)
        document = {
            'ready': not blockers,
            'blockers': blockers,
            'checks': checks,
            'generated_monotonic_s': now,
        }
        self.status_publisher.publish(
            String(data=json.dumps(document, ensure_ascii=False)))


def main():
    """Run the non-driving operator mapping readiness observer."""
    rclpy.init()
    node = OperatorMappingPreflight()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
