#!/usr/bin/env python3
"""Rewrite a ROS 2 bag with every LaserScan on one fixed angular grid."""

from __future__ import annotations

import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Sequence


def fixed_angular_grid(beam_count: int) -> tuple[float, float, float]:
    """Return a full-circle, upper-bound-exclusive laser grid."""
    if beam_count < 2:
        raise ValueError('beam_count must be at least 2')
    angle_min_rad = -math.pi
    angle_max_rad = math.pi
    angle_increment_rad = math.tau / beam_count
    return angle_min_rad, angle_max_rad, angle_increment_rad


def _wrap_angle_rad(angle_rad: float) -> float:
    """Wrap an angle to the half-open interval [-pi, pi)."""
    return (angle_rad + math.pi) % math.tau - math.pi


def nearest_circular_indices(
        source_angles_rad: Sequence[float],
        target_angles_rad: Sequence[float]) -> list[int]:
    """Map target angles to the nearest source indices on a circle."""
    if not source_angles_rad:
        raise ValueError('source_angles_rad must not be empty')
    ordered = sorted(
        (_wrap_angle_rad(angle_rad), index)
        for index, angle_rad in enumerate(source_angles_rad))
    ordered_angles_rad = [item[0] for item in ordered]
    nearest_indices = []
    for target_angle_rad in target_angles_rad:
        wrapped_target_rad = _wrap_angle_rad(target_angle_rad)
        insertion = bisect_left(ordered_angles_rad, wrapped_target_rad)
        candidates = []
        for raw_index in (insertion - 1, insertion):
            ordered_index = raw_index % len(ordered)
            source_angle_rad, source_index = ordered[ordered_index]
            angular_error_rad = abs(_wrap_angle_rad(
                source_angle_rad - wrapped_target_rad))
            candidates.append((angular_error_rad, source_index))
        nearest_indices.append(min(candidates)[1])
    return nearest_indices


def normalized_scan_values(
        ranges_m: Sequence[float],
        intensities: Sequence[float],
        *,
        source_angle_min_rad: float,
        source_angle_increment_rad: float,
        beam_count: int) -> dict[str, Any]:
    """Resample scan values onto a fixed full-circle grid."""
    if not ranges_m:
        raise ValueError('ranges_m must not be empty')
    if intensities and len(intensities) != len(ranges_m):
        raise ValueError('intensities must be empty or match ranges_m')
    angle_min_rad, angle_max_rad, angle_increment_rad = fixed_angular_grid(
        beam_count)
    source_angles_rad = [
        source_angle_min_rad + index * source_angle_increment_rad
        for index in range(len(ranges_m))
    ]
    target_angles_rad = [
        angle_min_rad + index * angle_increment_rad
        for index in range(beam_count)
    ]
    source_indices = nearest_circular_indices(
        source_angles_rad, target_angles_rad)
    return {
        'angle_min_rad': angle_min_rad,
        'angle_max_rad': angle_max_rad,
        'angle_increment_rad': angle_increment_rad,
        'ranges_m': [ranges_m[index] for index in source_indices],
        'intensities': (
            [intensities[index] for index in source_indices]
            if intensities else []),
    }


def normalize_laser_scan(scan: Any, beam_count: int) -> Any:
    """Return a LaserScan with fixed geometry and preserved observations."""
    from sensor_msgs.msg import LaserScan

    values = normalized_scan_values(
        scan.ranges,
        scan.intensities,
        source_angle_min_rad=float(scan.angle_min),
        source_angle_increment_rad=float(scan.angle_increment),
        beam_count=beam_count)
    normalized = LaserScan()
    normalized.header = scan.header
    normalized.angle_min = values['angle_min_rad']
    normalized.angle_max = values['angle_max_rad']
    normalized.angle_increment = values['angle_increment_rad']
    normalized.scan_time = scan.scan_time
    if scan.scan_time > 0.0:
        normalized.time_increment = scan.scan_time / beam_count
    else:
        normalized.time_increment = (
            abs(scan.time_increment) * len(scan.ranges) / beam_count)
    normalized.range_min = scan.range_min
    normalized.range_max = scan.range_max
    normalized.ranges = values['ranges_m']
    normalized.intensities = values['intensities']
    return normalized


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _bag_files(bag_path: Path) -> list[Path]:
    if bag_path.is_file():
        return [bag_path]
    return sorted(bag_path.glob('*.mcap'))


def _same_float_sequence(left: Sequence[float],
                         right: Sequence[float]) -> bool:
    if len(left) != len(right):
        return False
    return all(
        left_value == right_value
        or (math.isnan(left_value) and math.isnan(right_value))
        for left_value, right_value in zip(left, right))


def _same_scan(left: Any, right: Any) -> bool:
    scalar_fields = (
        'angle_min', 'angle_max', 'angle_increment', 'time_increment',
        'scan_time', 'range_min', 'range_max')
    return all((
        left.header == right.header,
        all(getattr(left, field) == getattr(right, field)
            for field in scalar_fields),
        _same_float_sequence(left.ranges, right.ranges),
        _same_float_sequence(left.intensities, right.intensities),
    ))


def validate_bag_rewrite(
        source_bag: Path,
        output_bag: Path,
        *,
        beam_count: int) -> dict[str, Any]:
    """Prove that only normalized `/scan` payloads changed."""
    from rclpy.serialization import deserialize_message, serialize_message
    import rosbag2_py
    from sensor_msgs.msg import LaserScan

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    source_reader = rosbag2_py.SequentialReader()
    output_reader = rosbag2_py.SequentialReader()
    source_reader.open(
        rosbag2_py.StorageOptions(uri=str(source_bag), storage_id='mcap'),
        converter_options)
    output_reader.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id='mcap'),
        converter_options)

    message_count = 0
    scan_count = 0
    stream_order_and_timestamps_match = True
    non_scan_payloads_match = True
    normalized_scans_match = True
    while source_reader.has_next() and output_reader.has_next():
        source_topic, source_serialized, source_timestamp_ns = (
            source_reader.read_next())
        output_topic, output_serialized, output_timestamp_ns = (
            output_reader.read_next())
        if (source_topic != output_topic
                or source_timestamp_ns != output_timestamp_ns):
            stream_order_and_timestamps_match = False
        if source_topic == '/scan' and output_topic == '/scan':
            source_scan = deserialize_message(source_serialized, LaserScan)
            expected_scan = deserialize_message(
                serialize_message(normalize_laser_scan(
                    source_scan, beam_count)),
                LaserScan)
            output_scan = deserialize_message(output_serialized, LaserScan)
            if not _same_scan(expected_scan, output_scan):
                normalized_scans_match = False
            scan_count += 1
        elif source_serialized != output_serialized:
            non_scan_payloads_match = False
        message_count += 1
    streams_ended_together = (
        not source_reader.has_next() and not output_reader.has_next())
    source_reader.close()
    output_reader.close()
    passed = all((
        stream_order_and_timestamps_match,
        non_scan_payloads_match,
        normalized_scans_match,
        streams_ended_together,
        scan_count > 0,
    ))
    return {
        'passed': passed,
        'message_count': message_count,
        'scan_count': scan_count,
        'stream_order_and_timestamps_match': (
            stream_order_and_timestamps_match),
        'non_scan_payloads_match': non_scan_payloads_match,
        'normalized_scans_match': normalized_scans_match,
        'streams_ended_together': streams_ended_together,
    }


def normalize_bag(
        source_bag: Path,
        output_bag: Path,
        *,
        beam_count: int,
        storage_config_file: Path | None = None) -> dict[str, Any]:
    """Copy a bag while replacing `/scan` messages with normalized scans."""
    from rclpy.serialization import deserialize_message, serialize_message
    import rosbag2_py
    from sensor_msgs.msg import LaserScan

    source_bag = source_bag.resolve()
    output_bag = output_bag.resolve()
    if not source_bag.exists():
        raise FileNotFoundError(f'source bag does not exist: {source_bag}')
    if output_bag.exists():
        raise FileExistsError(f'output bag already exists: {output_bag}')
    source_files = _bag_files(source_bag)
    if not source_files:
        raise FileNotFoundError(f'no MCAP file found in: {source_bag}')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(source_bag), storage_id='mcap'),
        converter_options)

    writer_options = rosbag2_py.StorageOptions(
        uri=str(output_bag), storage_id='mcap')
    if storage_config_file is not None:
        writer_options.storage_config_uri = str(storage_config_file.resolve())
    writer = rosbag2_py.SequentialWriter()
    writer.open(writer_options, converter_options)

    topic_types = {}
    for topic in reader.get_all_topics_and_types():
        topic_types[topic.name] = topic.type
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0,
            name=topic.name,
            type=topic.type,
            serialization_format=topic.serialization_format,
            offered_qos_profiles=topic.offered_qos_profiles,
            type_description_hash=topic.type_description_hash))
    if topic_types.get('/scan') != 'sensor_msgs/msg/LaserScan':
        raise ValueError(
            'source bag does not contain sensor_msgs/msg/LaserScan on /scan')

    message_count = 0
    scan_count = 0
    source_point_counts: dict[int, int] = {}
    while reader.has_next():
        topic, serialized, received_timestamp_ns = reader.read_next()
        if topic == '/scan':
            scan = deserialize_message(serialized, LaserScan)
            source_count = len(scan.ranges)
            source_point_counts[source_count] = (
                source_point_counts.get(source_count, 0) + 1)
            serialized = serialize_message(normalize_laser_scan(
                scan, beam_count))
            scan_count += 1
        writer.write(topic, serialized, received_timestamp_ns)
        message_count += 1
    reader.close()
    writer.close()

    validation = validate_bag_rewrite(
        source_bag, output_bag, beam_count=beam_count)
    validation_path = output_bag / 'scan_normalization_validation.json'
    validation_path.write_text(
        json.dumps(validation, ensure_ascii=False, indent=2),
        encoding='utf-8')
    if not validation['passed']:
        raise RuntimeError(
            f'normalized bag validation failed: {validation_path}')

    output_files = _bag_files(output_bag)
    manifest = {
        'schema_version': 1,
        'method': 'nearest_angular_neighbour',
        'source_bag': str(source_bag),
        'source_files': [
            {'name': path.name, 'sha256': _file_sha256(path)}
            for path in source_files
        ],
        'output_bag': str(output_bag),
        'output_files': [
            {'name': path.name, 'sha256': _file_sha256(path)}
            for path in output_files
        ],
        'message_count': message_count,
        'scan_count': scan_count,
        'source_point_counts': {
            str(count): occurrences
            for count, occurrences in sorted(source_point_counts.items())
        },
        'target': {
            'beam_count': beam_count,
            'angle_min_rad': -math.pi,
            'angle_max_rad': math.pi,
            'angle_increment_rad': math.tau / beam_count,
            'upper_bound_exclusive': True,
        },
        'validation': validation,
        'limitations': [
            'nearest angular neighbour preserves measured values without '
            'interpolation',
            'header timestamps are preserved; per-ray acquisition phase is '
            'not reconstructed',
            'this preprocessing does not create external ground truth',
        ],
    }
    manifest_path = output_bag / 'scan_normalization_manifest.json'
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding='utf-8')
    return manifest


def main(argv: list[str] | None = None) -> int:
    """Normalize one bag and write its provenance manifest."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--beam-count', type=int, required=True)
    parser.add_argument('--storage-config-file', type=Path)
    args = parser.parse_args(argv)
    if args.beam_count < 2:
        parser.error('--beam-count must be at least 2')
    if (args.storage_config_file is not None
            and not args.storage_config_file.is_file()):
        parser.error('--storage-config-file must exist')
    manifest = normalize_bag(
        args.bag,
        args.output,
        beam_count=args.beam_count,
        storage_config_file=args.storage_config_file)
    print(json.dumps({
        'output_bag': manifest['output_bag'],
        'message_count': manifest['message_count'],
        'scan_count': manifest['scan_count'],
        'target': manifest['target'],
        'validation': manifest['validation'],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
