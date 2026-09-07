#!/usr/bin/env python3
"""Publish the evaluation-only monotonic oracle map for G005."""

from __future__ import annotations

from collections import deque
import hashlib
import importlib
import json
import math
from pathlib import Path
import sys

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

from geometry_msgs.msg import PoseStamped

from jdamr_cube_navigation.g005_ground_truth_localization import (
    quaternion_yaw,
    stamp_ns,
)

from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


STATUS_SCHEMA_VERSION = 1
DEFAULT_SCAN_PERIOD_JITTER_S = 0.03
REJECTION_REASONS = (
    'non_monotonic_stamp',
    'oracle_contract',
    'pose_stale',
    'pose_unavailable',
    'scan_contract',
    'scan_period_jitter',
)


def _strict_json(path: Path):
    """Read strict JSON while rejecting duplicate keys and non-finite data."""
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f'duplicate JSON key: {key}')
            result[key] = value
        return result

    def reject(value):
        raise ValueError(f'non-finite JSON number: {value}')

    return json.loads(
        path.read_text(encoding='utf-8'), object_pairs_hook=pairs,
        parse_constant=reject)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def occupancy_payload_sha256(message: OccupancyGrid) -> str:
    """Hash the exact map fields consumed by the G005 coordinator."""
    orientation = message.info.origin.orientation
    yaw = quaternion_yaw(
        orientation.x, orientation.y, orientation.z, orientation.w)
    payload = {
        'frame_id': message.header.frame_id,
        'height': int(message.info.height),
        'origin_xy_yaw': [
            float(message.info.origin.position.x),
            float(message.info.origin.position.y),
            yaw,
        ],
        'resolution_m_per_cell': float(message.info.resolution),
        'width': int(message.info.width),
        'data': [int(value) for value in message.data],
    }
    encoded = (json.dumps(
        payload, sort_keys=True, separators=(',', ':'),
        ensure_ascii=False, allow_nan=False) + '\n').encode()
    return hashlib.sha256(encoded).hexdigest()


def _evaluation_directory() -> Path:
    """Resolve installed evaluation modules, with a source-tree fallback."""
    source = Path(__file__).resolve().parents[1] / 'evaluation'
    candidates = []
    try:
        installed = (
            Path(get_package_share_directory('jdamr_cube_navigation'))
            / 'evaluation')
        if (installed / 'g005_oracle.py').is_file():
            candidates.append(installed)
    except PackageNotFoundError:
        pass
    if (source / 'g005_oracle.py').is_file():
        candidates.append(source)
    loaded_directories = set()
    for module_name in ('frontier_policy_contract', 'g005_oracle'):
        module = sys.modules.get(module_name)
        if module is None:
            continue
        module_path = Path(getattr(module, '__file__', '')).absolute()
        if module_path.name != f'{module_name}.py':
            raise ValueError('G005 evaluation module identity drift')
        loaded_directories.add(module_path.parent)
    if len(loaded_directories) > 1:
        raise ValueError('G005 evaluation module directory drift')
    if loaded_directories:
        loaded = next(iter(loaded_directories))
        if any(loaded == candidate.absolute() for candidate in candidates):
            return loaded
        raise ValueError('G005 evaluation module source drift')
    if candidates:
        return candidates[0]
    raise ValueError('G005 evaluation modules are unavailable')


def load_evaluation_modules():
    """Import the data-installed G005 contract and oracle modules."""
    directory = _evaluation_directory()
    text = str(directory)
    inserted = text not in sys.path
    if inserted:
        sys.path.insert(0, text)
    try:
        contract = importlib.import_module('frontier_policy_contract')
        oracle = importlib.import_module('g005_oracle')
    finally:
        if inserted:
            sys.path.remove(text)
    if Path(contract.__file__).absolute().parent != directory.absolute():
        raise ValueError('G005 contract import identity drift')
    if Path(oracle.__file__).absolute().parent != directory.absolute():
        raise ValueError('G005 oracle import identity drift')
    return contract, oracle


def load_observer_contract(asset_root: Path, layout_seed: int,
                           contract_module) -> tuple[dict, dict]:
    """Load one generated layout and its exact observer-facing contract."""
    if (not asset_root.is_absolute()
            or not asset_root.is_dir()
            or asset_root.is_symlink()
            or layout_seed not in contract_module.LAYOUT_SEEDS):
        raise ValueError('invalid G005 observer asset selection')
    manifest_path = asset_root / 'asset_manifest.json'
    layout_path = asset_root / f'layout_{layout_seed}_gt.json'
    if (not manifest_path.is_file()
            or manifest_path.is_symlink()
            or not layout_path.is_file()
            or layout_path.is_symlink()):
        raise ValueError('G005 observer asset inventory is incomplete')
    manifest = _strict_json(manifest_path)
    if type(manifest) is not dict:
        raise ValueError('G005 observer manifest schema drift')
    try:
        frames = manifest['layout_contract']['frames']
        lidar = manifest['layout_contract']['lidar']
        record = manifest['layouts'][str(layout_seed)]
    except (KeyError, TypeError) as error:
        raise ValueError('G005 observer manifest schema drift') from error
    expected_frames = {
        'gazebo_world': 'g005_frontier',
        'map': 'map',
        'relationship': 'IDENTITY',
    }
    expected_lidar = {
        'rate_hz': contract_module.LIDAR_RATE_HZ,
        'beam_count': contract_module.LIDAR_BEAMS,
        'min_range_m': contract_module.LIDAR_MIN_RANGE_M,
        'range_m': contract_module.LIDAR_RANGE_M,
        'min_angle_rad': contract_module.LIDAR_MIN_ANGLE_RAD,
        'max_angle_rad': contract_module.LIDAR_MAX_ANGLE_RAD,
        'base_yaw_rad': contract_module.LIDAR_BASE_YAW_RAD,
    }
    if (type(record) is not dict
            or type(manifest.get('layout_seeds')) is not list
            or frames != expected_frames
            or lidar != expected_lidar
            or manifest.get('schema_version') != 1
            or layout_seed not in manifest.get('layout_seeds', [])
            or record.get('gt_occupancy_sha256') != _sha256(layout_path)):
        raise ValueError('G005 observer manifest contract drift')
    layout = _strict_json(layout_path)
    if layout != contract_module.build_layout(layout_seed):
        raise ValueError('G005 observer GT occupancy drift')
    return layout, {'frames': frames, 'lidar': lidar}


def validate_scan(message: LaserScan, lidar: dict) -> int:
    """Validate the actual scan timing, geometry, and payload shape."""
    stamp = stamp_ns(message)
    count = len(message.ranges)
    expected_increment = (
        (lidar['max_angle_rad'] - lidar['min_angle_rad']) /
        (lidar['beam_count'] - 1))
    geometry = (
        (message.angle_min, lidar['min_angle_rad']),
        (message.angle_max, lidar['max_angle_rad']),
        (message.angle_increment, expected_increment),
        (message.range_min, lidar['min_range_m']),
        (message.range_max, lidar['range_m']),
    )
    if (message.header.frame_id != 'laser_link'
            or stamp <= 0
            or count != lidar['beam_count']
            or any(not math.isfinite(actual)
                   or not math.isclose(actual, expected, rel_tol=0.0,
                                       abs_tol=1e-5)
                   for actual, expected in geometry)
            or any(math.isnan(value)
                   or value == -math.inf
                   or (math.isfinite(value)
                       and not message.range_min <= value <=
                       message.range_max)
                   for value in message.ranges)):
        raise ValueError('G005 scan contract drift')
    return stamp


def map_qos_profile() -> QoSProfile:
    """Return the latched reliable QoS required by Nav2 map consumers."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def build_health_status(*, accepted_scan_count: int,
                        rejected_scan_count: int,
                        rejection_counts: dict[str, int],
                        last_accepted_stamp_ns: int | None,
                        observed_max_gap_ns: int | None,
                        map_sequence: int,
                        map_payload_sha256: str | None,
                        observed_scan_count: int,
                        actual_mean_scan_rate_hz: float | None,
                        expected_scan_period_ns: int,
                        allowed_jitter_ns: int) -> str:
    """Build the deterministic, externally consumed observer status JSON."""
    valid_map_identity = (
        map_sequence == 0 and map_payload_sha256 is None
        or map_sequence > 0
        and type(map_payload_sha256) is str
        and len(map_payload_sha256) == 64
        and all(character in '0123456789abcdef'
                for character in map_payload_sha256)
    )
    if not valid_map_identity:
        raise ValueError('G005 observer map status identity drift')
    counts = {reason: int(rejection_counts.get(reason, 0))
              for reason in REJECTION_REASONS}
    payload = {
        'accepted_scan_count': int(accepted_scan_count),
        'actual_mean_scan_rate_hz': (
            None if actual_mean_scan_rate_hz is None
            else round(float(actual_mean_scan_rate_hz), 9)),
        'allowed_jitter_ns': int(allowed_jitter_ns),
        'expected_scan_period_ns': int(expected_scan_period_ns),
        'health': 'INVALID' if rejected_scan_count else 'VALID',
        'last_accepted_stamp_ns': last_accepted_stamp_ns,
        'map_payload_sha256': map_payload_sha256,
        'map_sequence': int(map_sequence),
        'observed_max_gap_ns': observed_max_gap_ns,
        'observed_scan_count': int(observed_scan_count),
        'rejected_scan_count': int(rejected_scan_count),
        'rejection_counts': counts,
        'schema_version': STATUS_SCHEMA_VERSION,
    }
    return json.dumps(payload, ensure_ascii=True, separators=(',', ':'),
                      sort_keys=True)


class FrontierObserver(Node):
    """Reveal and publish a monotonic map from validated GT pose and scans."""

    def __init__(self, *, parameter_overrides=None) -> None:
        """Create the sealed observer subscriptions and map publisher."""
        super().__init__(
            'g005_frontier_observer',
            parameter_overrides=parameter_overrides)
        self.declare_parameter('asset_root', '')
        self.declare_parameter('layout_seed', 11)
        self.declare_parameter('ground_truth_topic', '/ground_truth_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('status_topic', '/g005/observer_status')
        self.declare_parameter('max_pose_age_s', 0.05)
        self.declare_parameter('max_scan_gap_s', 0.25)
        self.declare_parameter(
            'scan_period_jitter_s', DEFAULT_SCAN_PERIOD_JITTER_S)
        asset_root = Path(str(self.get_parameter('asset_root').value))
        layout_seed = int(self.get_parameter('layout_seed').value)
        max_pose_age_s = float(self.get_parameter('max_pose_age_s').value)
        max_scan_gap_s = float(self.get_parameter('max_scan_gap_s').value)
        scan_period_jitter_s = float(
            self.get_parameter('scan_period_jitter_s').value)
        if (not math.isfinite(max_pose_age_s)
                or max_pose_age_s <= 0.0
                or not math.isfinite(max_scan_gap_s)
                or max_scan_gap_s <= 0.0
                or not math.isfinite(scan_period_jitter_s)
                or scan_period_jitter_s < 0.0):
            raise ValueError('invalid G005 observer timing parameters')
        contract_module, oracle_module = load_evaluation_modules()
        layout, observer_contract = load_observer_contract(
            asset_root, layout_seed, contract_module)
        self._frames = observer_contract['frames']
        self._lidar = observer_contract['lidar']
        self._oracle = oracle_module.OracleMapState(layout)
        self._metadata = oracle_module.occupancy_grid_metadata(
            layout, self._frames['map'])
        self._max_pose_age_ns = round(max_pose_age_s * 1_000_000_000)
        self._max_scan_gap_ns = round(max_scan_gap_s * 1_000_000_000)
        self._expected_scan_period_ns = round(
            1_000_000_000 / self._lidar['rate_hz'])
        self._allowed_jitter_ns = round(
            scan_period_jitter_s * 1_000_000_000)
        if (self._allowed_jitter_ns >= self._expected_scan_period_ns
                or self._max_scan_gap_ns < (
                    self._expected_scan_period_ns
                    + self._allowed_jitter_ns)):
            raise ValueError('invalid G005 observer timing parameters')
        self._poses = deque(maxlen=32)
        self._first_scan_stamp_ns = None
        self._last_scan_stamp_ns = None
        self._first_observed_scan_stamp_ns = None
        self._last_observed_scan_stamp_ns = None
        self._observed_scan_count = 0
        self._observed_max_gap_ns = None
        self._scan_count = 0
        self._rejected_scan_count = 0
        self._rejection_counts = {reason: 0 for reason in REJECTION_REASONS}
        self._map_sequence = 0
        self._map_payload_sha256 = None
        self._map_load_stamp = None
        self._publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            map_qos_profile())
        self._status_publisher = self.create_publisher(
            String, str(self.get_parameter('status_topic').value),
            map_qos_profile())
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('ground_truth_topic').value),
            self._on_ground_truth, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self._on_scan, qos_profile_sensor_data)
        self._publish_status()

    @property
    def scan_count(self) -> int:
        """Return the number of scans accepted into the oracle."""
        return self._scan_count

    @property
    def scan_rate_hz(self) -> float | None:
        """Return the accepted scan rate when at least two scans exist."""
        if self._scan_count < 2:
            return None
        duration_ns = self._last_scan_stamp_ns - self._first_scan_stamp_ns
        return ((self._scan_count - 1) * 1_000_000_000 / duration_ns)

    @property
    def actual_mean_scan_rate_hz(self) -> float | None:
        """Return the mean rate of structurally valid monotonic input scans."""
        if self._observed_scan_count < 2:
            return None
        duration_ns = (
            self._last_observed_scan_stamp_ns
            - self._first_observed_scan_stamp_ns)
        return ((self._observed_scan_count - 1) * 1_000_000_000
                / duration_ns)

    def _status_json(self) -> str:
        return build_health_status(
            accepted_scan_count=self._scan_count,
            rejected_scan_count=self._rejected_scan_count,
            rejection_counts=self._rejection_counts,
            last_accepted_stamp_ns=self._last_scan_stamp_ns,
            observed_max_gap_ns=self._observed_max_gap_ns,
            map_sequence=self._map_sequence,
            map_payload_sha256=self._map_payload_sha256,
            observed_scan_count=self._observed_scan_count,
            actual_mean_scan_rate_hz=self.actual_mean_scan_rate_hz,
            expected_scan_period_ns=self._expected_scan_period_ns,
            allowed_jitter_ns=self._allowed_jitter_ns)

    def _publish_status(self) -> None:
        message = String()
        message.data = self._status_json()
        self._status_publisher.publish(message)

    def _reject_scan(self, reason: str, detail: str) -> None:
        self._rejected_scan_count += 1
        self._rejection_counts[reason] += 1
        self.get_logger().error(detail)
        self._publish_status()

    def _observe_scan_stamp(self, scan_stamp: int) -> int | None:
        gap_ns = None
        if self._last_observed_scan_stamp_ns is not None:
            gap_ns = scan_stamp - self._last_observed_scan_stamp_ns
            if (self._observed_max_gap_ns is None
                    or gap_ns > self._observed_max_gap_ns):
                self._observed_max_gap_ns = gap_ns
        if self._first_observed_scan_stamp_ns is None:
            self._first_observed_scan_stamp_ns = scan_stamp
        self._last_observed_scan_stamp_ns = scan_stamp
        self._observed_scan_count += 1
        return gap_ns

    def _on_ground_truth(self, message: PoseStamped) -> None:
        if (message.header.frame_id != self._frames['gazebo_world']
                or stamp_ns(message) <= 0):
            self.get_logger().error('rejected ground-truth frame or stamp')
            return
        pose = message.pose
        try:
            yaw = quaternion_yaw(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        sample = (stamp_ns(message), float(pose.position.x),
                  float(pose.position.y), yaw)
        if any(not math.isfinite(value) for value in sample[1:]):
            self.get_logger().error('rejected non-finite ground-truth pose')
            return
        self._poses.append(sample)

    def _on_scan(self, message: LaserScan) -> None:
        try:
            scan_stamp = validate_scan(message, self._lidar)
        except ValueError as error:
            self._reject_scan('scan_contract', str(error))
            return
        if (self._last_observed_scan_stamp_ns is not None
                and scan_stamp <= self._last_observed_scan_stamp_ns):
            self._reject_scan(
                'non_monotonic_stamp', 'rejected non-monotonic scan stamp')
            return
        gap_ns = self._observe_scan_stamp(scan_stamp)
        if (gap_ns is not None
                and (gap_ns > self._max_scan_gap_ns
                     or abs(gap_ns - self._expected_scan_period_ns)
                     > self._allowed_jitter_ns)):
            self._reject_scan(
                'scan_period_jitter', 'rejected scan period jitter')
            return
        if not self._poses:
            # The synchronized measurement window starts with the first GT pose.
            self._first_observed_scan_stamp_ns = None
            self._last_observed_scan_stamp_ns = None
            self._observed_scan_count = 0
            self._observed_max_gap_ns = None
            return
        pose = min(self._poses, key=lambda item: abs(item[0] - scan_stamp))
        if abs(pose[0] - scan_stamp) > self._max_pose_age_ns:
            self._reject_scan('pose_stale', 'rejected scan with stale pose')
            return
        try:
            result = self._oracle.update_scan(
                pose[1], pose[2], pose[3], self._scan_count)
        except ValueError as error:
            self._reject_scan('oracle_contract', str(error))
            return
        self._last_scan_stamp_ns = scan_stamp
        if self._first_scan_stamp_ns is None:
            self._first_scan_stamp_ns = scan_stamp
        self._scan_count += 1
        if result['map_changed']:
            if self._map_load_stamp is None:
                self._map_load_stamp = message.header.stamp
            self._map_sequence += 1
            if result['map_sequence'] != self._map_sequence:
                self._reject_scan(
                    'oracle_contract', 'oracle map sequence drift')
                return
            occupancy = self._occupancy_grid(message.header.stamp)
            self._map_payload_sha256 = occupancy_payload_sha256(occupancy)
            self._publisher.publish(occupancy)
        self._publish_status()

    def _occupancy_grid(self, stamp) -> OccupancyGrid:
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self._metadata['frame_id']
        message.info.map_load_time = self._map_load_stamp
        message.info.resolution = self._metadata['resolution']
        message.info.width = self._metadata['width']
        message.info.height = self._metadata['height']
        origin = self._metadata['origin']
        message.info.origin.position.x = origin['position']['x']
        message.info.origin.position.y = origin['position']['y']
        message.info.origin.position.z = origin['position']['z']
        message.info.origin.orientation.x = origin['orientation']['x']
        message.info.origin.orientation.y = origin['orientation']['y']
        message.info.origin.orientation.z = origin['orientation']['z']
        message.info.origin.orientation.w = origin['orientation']['w']
        message.data = self._oracle.observed
        return message


def main(args=None) -> None:
    """Run the G005 evaluation-only frontier observer."""
    rclpy.init(args=args)
    node = None
    try:
        node = FrontierObserver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
