"""Regression tests for fixed-grid LaserScan bag preprocessing."""

import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from normalize_scan_bag import (  # noqa: E402,I100
    fixed_angular_grid,
    nearest_circular_indices,
    normalize_bag,
    normalized_scan_values,
    validate_bag_rewrite,
)


def test_fixed_grid_matches_karto_full_circle_count_contract():
    """The exclusive full-circle metadata must make Karto expect N beams."""
    angle_min_rad, angle_max_rad, angle_increment_rad = fixed_angular_grid(8)

    assert angle_min_rad == pytest.approx(-math.pi)
    assert angle_max_rad == pytest.approx(math.pi)
    assert round((angle_max_rad - angle_min_rad) / angle_increment_rad) == 8


def test_negative_increment_scan_preserves_physical_directions():
    """Reordering clockwise samples must not mirror the measured scene."""
    values = normalized_scan_values(
        ranges_m=[10.0, 20.0, 30.0, 40.0],
        intensities=[1.0, 2.0, 3.0, 4.0],
        source_angle_min_rad=math.pi / 2.0,
        source_angle_increment_rad=-math.pi / 2.0,
        beam_count=4)

    assert values['ranges_m'] == [40.0, 30.0, 20.0, 10.0]
    assert values['intensities'] == [4.0, 3.0, 2.0, 1.0]


def test_wraparound_uses_nearest_real_beam_without_interpolation():
    """Targets near the seam must select the nearby beam across wrapping."""
    source_angles_rad = [-math.pi + 0.1, -1.0, 0.0, 1.0]

    indices = nearest_circular_indices(
        source_angles_rad, [math.pi - 0.05, -math.pi + 0.05])

    assert indices == [0, 0]


def test_nonfinite_ranges_are_preserved_deterministically():
    """Nearest-neighbour resampling must not invent finite measurements."""
    values = normalized_scan_values(
        ranges_m=[float('inf'), 2.0, float('nan'), 4.0],
        intensities=[],
        source_angle_min_rad=-math.pi,
        source_angle_increment_rad=math.pi / 2.0,
        beam_count=4)

    assert math.isinf(values['ranges_m'][0])
    assert values['ranges_m'][1] == 2.0
    assert math.isnan(values['ranges_m'][2])
    assert values['ranges_m'][3] == 4.0
    assert values['intensities'] == []


def test_mismatched_intensity_vector_is_rejected():
    """An ambiguous partial intensity array must fail before bag writing."""
    with pytest.raises(ValueError, match='intensities'):
        normalized_scan_values(
            ranges_m=[1.0, 2.0],
            intensities=[1.0],
            source_angle_min_rad=0.0,
            source_angle_increment_rad=math.pi,
            beam_count=2)


def test_bag_rewrite_preserves_other_topics_and_receive_timestamps(tmp_path):
    """Only `/scan` payloads may change during the bag rewrite."""
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import LaserScan

    converter = rosbag2_py.ConverterOptions('cdr', 'cdr')
    source_bag = tmp_path / 'source'
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(source_bag), storage_id='mcap'),
        converter)
    for name, type_name in (
            ('/scan', 'sensor_msgs/msg/LaserScan'),
            ('/odom', 'nav_msgs/msg/Odometry')):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0,
            name=name,
            type=type_name,
            serialization_format='cdr'))
    scan = LaserScan()
    scan.angle_min = math.pi / 2.0
    scan.angle_max = -math.pi
    scan.angle_increment = -math.pi / 2.0
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [1.0, 2.0, 3.0, 4.0]
    odom = Odometry()
    odom.pose.pose.position.x = 2.5
    writer.write('/scan', serialize_message(scan), 100)
    writer.write('/odom', serialize_message(odom), 200)
    writer.close()

    output_bag = tmp_path / 'normalized'
    manifest = normalize_bag(source_bag, output_bag, beam_count=8)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id='mcap'),
        converter)
    records = []
    while reader.has_next():
        records.append(reader.read_next())
    reader.close()

    assert [(topic, timestamp_ns) for topic, _, timestamp_ns in records] == [
        ('/scan', 100), ('/odom', 200)]
    normalized_scan = deserialize_message(records[0][1], LaserScan)
    copied_odom = deserialize_message(records[1][1], Odometry)
    assert len(normalized_scan.ranges) == 8
    assert normalized_scan.angle_increment == pytest.approx(math.tau / 8)
    assert copied_odom.pose.pose.position.x == pytest.approx(2.5)
    assert manifest['message_count'] == 2
    assert manifest['scan_count'] == 1
    assert manifest['source_point_counts'] == {'4': 1}
    assert manifest['validation']['passed'] is True
    assert (output_bag / 'scan_normalization_manifest.json').is_file()
    assert (output_bag / 'scan_normalization_validation.json').is_file()

    validation = validate_bag_rewrite(
        source_bag, output_bag, beam_count=8)
    assert validation == manifest['validation']
