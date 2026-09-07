"""Hostile contract tests for the G005 frontier map observer."""

from copy import deepcopy
import json
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped

from jdamr_cube_navigation.g005_frontier_observer import (
    build_health_status,
    FrontierObserver,
    load_evaluation_modules,
    load_observer_contract,
    map_qos_profile,
    occupancy_payload_sha256,
    validate_scan,
)

import pytest

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from sensor_msgs.msg import LaserScan


EVALUATION_DIR = Path(__file__).resolve().parents[1] / 'evaluation'


class PublisherProbe:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(deepcopy(message))


class OrderedPublisherProbe:
    def __init__(self, name, events):
        self.name = name
        self.events = events

    def publish(self, message):
        self.events.append((self.name, deepcopy(message)))


def _stamp(message, stamp_ns):
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    return message


def _ground_truth(stamp_ns, layout, yaw_rad=0.0):
    message = _stamp(PoseStamped(), stamp_ns)
    message.header.frame_id = 'g005_frontier'
    x, y = layout['start_cell']
    resolution = layout['resolution_m_per_cell']
    message.pose.position.x = (
        layout['origin_m_rad'][0] + (x + 0.5) * resolution)
    message.pose.position.y = (
        layout['origin_m_rad'][1] + (y + 0.5) * resolution)
    message.pose.orientation.z = math.sin(yaw_rad / 2.0)
    message.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return message


def _scan(stamp_ns, contract):
    lidar = contract['lidar']
    message = _stamp(LaserScan(), stamp_ns)
    message.header.frame_id = 'laser_link'
    message.angle_min = lidar['min_angle_rad']
    message.angle_max = lidar['max_angle_rad']
    message.angle_increment = (
        (message.angle_max - message.angle_min)
        / (lidar['beam_count'] - 1))
    message.range_min = lidar['min_range_m']
    message.range_max = lidar['range_m']
    message.ranges = [math.inf] * lidar['beam_count']
    return message


@pytest.fixture
def generated_assets(tmp_path):
    contract, _ = load_evaluation_modules()
    import generate_frontier_policy_assets as generator
    root = tmp_path / 'assets'
    generator.generate(root, 'smoke')
    layout, observer_contract = load_observer_contract(root, 11, contract)
    return root, layout, observer_contract


@pytest.fixture
def observer_node(generated_assets):
    root, layout, observer_contract = generated_assets
    rclpy.init(args=[])
    node = FrontierObserver(parameter_overrides=[
        Parameter('asset_root', value=str(root)),
        Parameter('layout_seed', value=11),
    ])
    node._publisher = PublisherProbe()
    node._status_publisher = PublisherProbe()
    try:
        yield node, layout, observer_contract
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_generated_layout_and_frame_contract_are_loaded(generated_assets):
    root, layout, observer_contract = generated_assets
    contract, _ = load_evaluation_modules()
    assert layout == contract.build_layout(11)
    assert observer_contract['frames'] == {
        'gazebo_world': 'g005_frontier',
        'map': 'map',
        'relationship': 'IDENTITY',
    }
    manifest_path = root / 'asset_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest['layout_contract']['frames']['relationship'] = 'UNKNOWN'
    manifest_path.write_text(json.dumps(manifest), encoding='utf-8')
    with pytest.raises(ValueError, match='contract drift'):
        load_observer_contract(root, 11, contract)


def test_scan_validation_rejects_shape_geometry_and_payload(generated_assets):
    _, _, observer_contract = generated_assets
    message = _scan(1_000_000_000, observer_contract)
    assert validate_scan(message, observer_contract['lidar']) == 1_000_000_000
    bad_count = deepcopy(message)
    bad_count.ranges.pop()
    with pytest.raises(ValueError, match='contract drift'):
        validate_scan(bad_count, observer_contract['lidar'])
    bad_geometry = deepcopy(message)
    bad_geometry.angle_max += 0.1
    with pytest.raises(ValueError, match='contract drift'):
        validate_scan(bad_geometry, observer_contract['lidar'])
    bad_minimum = deepcopy(message)
    bad_minimum.range_min = 0.0
    with pytest.raises(ValueError, match='contract drift'):
        validate_scan(bad_minimum, observer_contract['lidar'])
    bad_payload = deepcopy(message)
    bad_payload.ranges[0] = math.nan
    with pytest.raises(ValueError, match='contract drift'):
        validate_scan(bad_payload, observer_contract['lidar'])
    bad_frame = deepcopy(message)
    bad_frame.header.frame_id = 'base_scan'
    with pytest.raises(ValueError, match='contract drift'):
        validate_scan(bad_frame, observer_contract['lidar'])


def test_map_qos_is_latched_and_reliable():
    qos = map_qos_profile()
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_health_status_json_is_deterministic_and_complete():
    arguments = {
        'accepted_scan_count': 7,
        'rejected_scan_count': 2,
        'rejection_counts': {
            'scan_period_jitter': 2,
        },
        'last_accepted_stamp_ns': 1_600_000_000,
        'map_payload_sha256': 'a' * 64,
        'observed_max_gap_ns': 250_000_000,
        'map_sequence': 3,
        'observed_scan_count': 9,
        'actual_mean_scan_rate_hz': 9.9999999996,
        'expected_scan_period_ns': 100_000_000,
        'allowed_jitter_ns': 30_000_000,
    }
    encoded = build_health_status(**arguments)
    assert encoded == build_health_status(**arguments)
    assert encoded == json.dumps(
        json.loads(encoded), ensure_ascii=True, separators=(',', ':'),
        sort_keys=True)
    payload = json.loads(encoded)
    assert payload == {
        'accepted_scan_count': 7,
        'actual_mean_scan_rate_hz': 10.0,
        'allowed_jitter_ns': 30_000_000,
        'expected_scan_period_ns': 100_000_000,
        'health': 'INVALID',
        'last_accepted_stamp_ns': 1_600_000_000,
        'map_payload_sha256': 'a' * 64,
        'map_sequence': 3,
        'observed_max_gap_ns': 250_000_000,
        'observed_scan_count': 9,
        'rejected_scan_count': 2,
        'rejection_counts': {
            'non_monotonic_stamp': 0,
            'oracle_contract': 0,
            'pose_stale': 0,
            'pose_unavailable': 0,
            'scan_contract': 0,
            'scan_period_jitter': 2,
        },
        'schema_version': 1,
    }
    invalid = dict(arguments)
    invalid['map_payload_sha256'] = None
    with pytest.raises(ValueError, match='map status identity drift'):
        build_health_status(**invalid)


def test_valid_pose_and_scans_publish_monotonic_map_only_on_change(
        observer_node):
    node, layout, observer_contract = observer_node
    previous = [-1] * len(layout['data'])
    for scan_index in range(8):
        stamp_ns = 1_000_000_000 + scan_index * 100_000_000
        node._on_ground_truth(_ground_truth(stamp_ns, layout, 0.3))
        node._on_scan(_scan(stamp_ns, observer_contract))
    messages = node._publisher.messages
    assert node.scan_count == 8
    assert node.scan_rate_hz == pytest.approx(10.0)
    assert 0 < len(messages) < node.scan_count
    for message in messages:
        assert message.header.frame_id == 'map'
        assert message.info.width == layout['width']
        assert message.info.height == layout['height']
        assert message.info.resolution == layout['resolution_m_per_cell']
        assert len(message.data) == len(layout['data'])
        assert all(old == -1 or old == new
                   for old, new in zip(previous, message.data))
        previous = list(message.data)
    assert messages[0].info.map_load_time == messages[0].header.stamp
    assert list(messages[-1].data) == node._oracle.observed
    status = json.loads(node._status_publisher.messages[-1].data)
    assert status['health'] == 'VALID'
    assert status['accepted_scan_count'] == 8
    assert status['rejected_scan_count'] == 0
    assert status['last_accepted_stamp_ns'] == 1_700_000_000
    assert status['observed_max_gap_ns'] == 100_000_000
    assert status['map_sequence'] == len(messages)
    assert status['map_payload_sha256'] == occupancy_payload_sha256(
        messages[-1])
    assert status['actual_mean_scan_rate_hz'] == pytest.approx(10.0)


def test_changed_map_is_published_before_matching_sequence_status(
        observer_node):
    node, layout, observer_contract = observer_node
    events = []
    node._publisher = OrderedPublisherProbe('map', events)
    node._status_publisher = OrderedPublisherProbe('status', events)
    node._on_ground_truth(_ground_truth(1_000_000_000, layout, 0.3))
    node._on_scan(_scan(1_000_000_000, observer_contract))
    assert [name for name, _ in events] == ['map', 'status']
    occupancy = events[0][1]
    status = json.loads(events[1][1].data)
    assert status['map_sequence'] == 1
    assert status['map_payload_sha256'] == occupancy_payload_sha256(
        occupancy)


@pytest.mark.parametrize(('period_ns', 'expected_rate_hz'), [
    (50_000_000, 20.0),
    (250_000_000, 4.0),
])
def test_off_contract_scan_rates_are_rejected_and_remain_invalid(
        observer_node, period_ns, expected_rate_hz):
    node, layout, observer_contract = observer_node
    for scan_index in range(5):
        stamp = 1_000_000_000 + scan_index * period_ns
        node._on_ground_truth(_ground_truth(stamp, layout))
        node._on_scan(_scan(stamp, observer_contract))
    status = json.loads(node._status_publisher.messages[-1].data)
    assert node.scan_count == 1
    assert status['health'] == 'INVALID'
    assert status['accepted_scan_count'] == 1
    assert status['rejected_scan_count'] == 4
    assert status['rejection_counts']['scan_period_jitter'] == 4
    assert status['observed_max_gap_ns'] == period_ns
    assert status['actual_mean_scan_rate_hz'] == pytest.approx(
        expected_rate_hz)

    valid_stamp = 1_000_000_000 + 4 * period_ns + 100_000_000
    node._on_ground_truth(_ground_truth(valid_stamp, layout))
    node._on_scan(_scan(valid_stamp, observer_contract))
    recovered = json.loads(node._status_publisher.messages[-1].data)
    assert recovered['accepted_scan_count'] == 2
    assert recovered['health'] == 'INVALID'
    assert recovered['rejected_scan_count'] == 4


def test_realistic_scan_jitter_is_accepted(observer_node):
    node, layout, observer_contract = observer_node
    stamps = [
        1_000_000_000,
        1_075_000_000,
        1_200_000_000,
        1_295_000_000,
        1_405_000_000,
    ]
    for stamp in stamps:
        node._on_ground_truth(_ground_truth(stamp, layout))
        node._on_scan(_scan(stamp, observer_contract))
    status = json.loads(node._status_publisher.messages[-1].data)
    assert node.scan_count == len(stamps)
    assert status['health'] == 'VALID'
    assert status['rejected_scan_count'] == 0
    assert status['observed_max_gap_ns'] == 125_000_000
    assert status['actual_mean_scan_rate_hz'] == pytest.approx(
        4e9 / 405_000_000)


def test_frame_stale_pose_and_scan_timing_fail_closed(observer_node):
    node, layout, observer_contract = observer_node
    wrong_frame = _ground_truth(1_000_000_000, layout)
    wrong_frame.header.frame_id = 'map'
    node._on_ground_truth(wrong_frame)
    node._on_scan(_scan(1_000_000_000, observer_contract))
    assert node.scan_count == 0
    startup = json.loads(node._status_json())
    assert startup['health'] == 'VALID'
    assert startup['observed_scan_count'] == 0
    assert startup['rejection_counts']['pose_unavailable'] == 0
    node._on_ground_truth(_ground_truth(1_000_000_000, layout))
    node._on_scan(_scan(1_100_000_000, observer_contract))
    assert node.scan_count == 0
    node._on_ground_truth(_ground_truth(1_200_000_000, layout))
    node._on_scan(_scan(1_200_000_000, observer_contract))
    assert node.scan_count == 1
    node._on_ground_truth(_ground_truth(1_210_000_000, layout))
    node._on_scan(_scan(1_210_000_000, observer_contract))
    assert node.scan_count == 1
    node._on_ground_truth(_ground_truth(1_500_000_000, layout))
    node._on_scan(_scan(1_500_000_000, observer_contract))
    assert node.scan_count == 1


def test_nonfinite_pose_never_changes_or_publishes(observer_node):
    node, layout, observer_contract = observer_node
    message = _ground_truth(1_000_000_000, layout)
    message.pose.position.x = math.nan
    node._on_ground_truth(message)
    node._on_scan(_scan(1_000_000_000, observer_contract))
    assert node.scan_count == 0
    assert node._publisher.messages == []
