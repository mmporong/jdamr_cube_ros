#!/usr/bin/env python3
"""Generate one canonical G002 Axis B localization replay input."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shutil
import tempfile

from amcl_fault_contract import (
    canonical_json_bytes,
    sha256_file,
    strict_json_load,
)
from axis_b_kidnapped_driver import validate_kidnapped_trace
from g002_tf_sanitizer import _canonical_tf_serialization, _tf_record
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from tf2_msgs.msg import TFMessage


SCENARIOS = {
    'correct_init': {
        'true_start_pose': [-8.0, 0.0, 0.0],
        'initial_estimate_offset': [0.0, 0.0, 0.0],
        'teleport_pose': None, 'observation_rotation_rad': 0.0},
    'initial_offset': {
        'true_start_pose': [-8.0, 0.0, 0.0],
        'initial_estimate_offset': [0.5, 0.0, 0.2617993877991494],
        'teleport_pose': None, 'observation_rotation_rad': 0.0},
    'kidnapped': {
        'true_start_pose': [-8.0, 0.0, 0.0],
        'initial_estimate_offset': [0.0, 0.0, 0.0],
        'teleport_pose': [0.0, 0.0, 3.141592653589793],
        'observation_rotation_rad': 6.283185307179586},
}
SOURCE_TOPICS = {
    '/sim_raw/scan': '/scan', '/odom': '/odom', '/tf': '/tf',
    '/tf_static': '/tf_static', '/ground_truth_pose': '/ground_truth_pose',
    '/cmd_vel': '/cmd_vel',
    ('/world/slam_corridor/model/g003_preloaded_front_observation_probe/'
     'link/body/sensor/contact_sensor/contact'): '/contact',
}


def _identity(path: Path) -> dict:
    return {'path': str(path), 'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path)}


def _output_identity(path: Path) -> dict:
    return {'name': path.name, 'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path)}


def _generator_source_identity() -> dict:
    return _identity(Path(__file__).resolve())


def _generator_source_valid(record: dict) -> bool:
    return record == _generator_source_identity()


def _identity_matches(record: dict, path: Path, output: bool = False) -> bool:
    expected_path_key = 'name' if output else 'path'
    expected_path = path.name if output else str(path)
    return (type(record) is dict and set(record) == {
        expected_path_key, 'size_bytes', 'sha256'} and
        record[expected_path_key] == expected_path and
        type(record['size_bytes']) is int and
        record['size_bytes'] == path.stat().st_size and
        record['sha256'] == sha256_file(path))


def validate_input(output_root: Path) -> dict:
    """Reopen and independently validate one generated Axis B input."""
    if (not output_root.is_absolute() or not output_root.is_dir() or
            output_root.is_symlink()):
        raise ValueError('Axis B input root is not canonical')
    files = {path.name for path in output_root.iterdir()
             if path.is_file() and not path.is_symlink()}
    expected_files = {'bag_0.mcap', 'metadata.yaml',
                      'axis_b_input_manifest.json',
                      'sanitizer_manifest.json'}
    if (files != expected_files or any(
            path.is_symlink() or not path.is_file()
            for path in output_root.iterdir())):
        raise ValueError('Axis B input file inventory drift')
    manifest_path = output_root / 'axis_b_input_manifest.json'
    alias_path = output_root / 'sanitizer_manifest.json'
    if manifest_path.read_bytes() != alias_path.read_bytes():
        raise ValueError('Axis B manifest alias drift')
    manifest = strict_json_load(manifest_path)
    expected_keys = {
        'schema_version', 'scenario', 'scenario_contract', 'source_bag',
        'source_metadata', 'source_evidence', 'g004_contract',
        'trace_evidence', 'topic_mapping', 'topic_counts',
        'removed_map_to_odom_count', 'ordered_payload_sha256', 'output',
        'output_limit_bytes', 'generator_source'}
    if (type(manifest) is not dict or set(manifest) != expected_keys or
            manifest['schema_version'] != 1 or
            manifest['scenario'] not in SCENARIOS or
            manifest['scenario_contract'] != SCENARIOS[manifest['scenario']] or
            manifest['topic_mapping'] != SOURCE_TOPICS or
            not _generator_source_valid(manifest['generator_source']) or
            manifest['output_limit_bytes'] != 134217728):
        raise ValueError('Axis B input manifest schema drift')
    for key in ('source_bag', 'source_metadata', 'source_evidence',
                'g004_contract'):
        record = manifest[key]
        if (type(record) is not dict or set(record) != {
                'path', 'size_bytes', 'sha256'} or
                not _identity_matches(record, Path(record['path']))):
            raise ValueError(f'Axis B source identity drift: {key}')
    trace = manifest['trace_evidence']
    if manifest['scenario'] == 'kidnapped':
        if (type(trace) is not dict or set(trace) != {
                'path', 'size_bytes', 'sha256'} or
                not _identity_matches(trace, Path(trace['path']))):
            raise ValueError('kidnapped trace identity drift')
        validate_kidnapped_trace(strict_json_load(Path(trace['path'])))
    elif trace is not None:
        raise ValueError('unexpected non-kidnapped trace')
    mcap_path = output_root / 'bag_0.mcap'
    metadata_path = output_root / 'metadata.yaml'
    output = manifest['output']
    if (type(output) is not dict or set(output) != {'mcap', 'metadata'} or
            not _identity_matches(output['mcap'], mcap_path, output=True) or
            not _identity_matches(
                output['metadata'], metadata_path, output=True) or
            mcap_path.stat().st_size > manifest['output_limit_bytes']):
        raise ValueError('Axis B output identity drift')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(output_root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    counts = {name: 0 for name in SOURCE_TOPICS.values()}
    digest = hashlib.sha256()
    while reader.has_next():
        topic, serialized, storage_ns = reader.read_next()
        if topic not in counts:
            raise ValueError('unexpected Axis B output topic')
        payload = bytes(serialized)
        if topic == '/tf':
            message = deserialize_message(payload, TFMessage)
            if any(_tf_record(transform)['parent'].lstrip('/') == 'map' and
                   _tf_record(transform)['child'].lstrip('/') == 'odom'
                   for transform in message.transforms):
                raise ValueError('map-to-odom remained in Axis B input')
            if _canonical_tf_serialization(
                    bytes(serialize_message(message))) != payload:
                raise ValueError('Axis B TF serialization is not canonical')
        counts[topic] += 1
        digest.update(topic.encode() + b'\0')
        digest.update(str(storage_ns).encode() + b'\0' + payload)
    reader.close()
    if (counts != manifest['topic_counts'] or
            digest.hexdigest() != manifest['ordered_payload_sha256'] or
            type(manifest['removed_map_to_odom_count']) is not int or
            manifest['removed_map_to_odom_count'] < 0):
        raise ValueError('Axis B output payload parity drift')
    return manifest


def generate(source_root: Path, output_root: Path, scenario: str,
             source_evidence: Path, g004_contract: Path,
             trace_evidence: Path | None = None) -> dict:
    """Filter one validated simulation bag without recorded map-to-odom."""
    if scenario not in SCENARIOS:
        raise ValueError('unknown Axis B scenario')
    if scenario == 'kidnapped':
        if trace_evidence is None:
            raise ValueError('kidnapped input requires live trace evidence')
        validate_kidnapped_trace(strict_json_load(trace_evidence))
    elif trace_evidence is not None:
        raise ValueError('trace evidence is only valid for kidnapped input')
    if output_root.exists() or output_root.is_symlink() or not output_root.is_absolute():
        raise ValueError('output root must be absent and absolute')
    converter = rosbag2_py.ConverterOptions('cdr', 'cdr')
    with tempfile.TemporaryDirectory(
            prefix='.g002-axis-b-input-', dir=output_root.parent) as temp_name:
        stage = Path(temp_name) / 'bag'
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(
            uri=str(source_root), storage_id='mcap'), converter)
        source_meta = {item.name: item
                       for item in reader.get_all_topics_and_types()}
        if not set(SOURCE_TOPICS).issubset(source_meta):
            raise ValueError('G004 representative topic set drift')
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(
            uri=str(stage), storage_id='mcap'), converter)
        for source_name, output_name in SOURCE_TOPICS.items():
            item = source_meta[source_name]
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=0, name=output_name, type=item.type,
                serialization_format=item.serialization_format,
                offered_qos_profiles=item.offered_qos_profiles,
                type_description_hash=item.type_description_hash))
        counts = {name: 0 for name in SOURCE_TOPICS.values()}
        removed = 0
        digest = hashlib.sha256()
        while reader.has_next():
            topic, serialized, storage_ns = reader.read_next()
            if topic not in SOURCE_TOPICS:
                continue
            output_topic = SOURCE_TOPICS[topic]
            payload = bytes(serialized)
            if topic == '/tf':
                message = deserialize_message(payload, TFMessage)
                kept = []
                for transform in message.transforms:
                    record = _tf_record(transform)
                    if (record['parent'].lstrip('/') == 'map' and
                            record['child'].lstrip('/') == 'odom'):
                        removed += 1
                    else:
                        kept.append(transform)
                message.transforms = kept
                payload = _canonical_tf_serialization(
                    bytes(serialize_message(message)))
            writer.write(output_topic, payload, storage_ns)
            counts[output_topic] += 1
            digest.update(output_topic.encode() + b'\0')
            digest.update(str(storage_ns).encode() + b'\0' + payload)
        reader.close()
        writer.close()
        del reader
        del writer
        mcap = next(stage.glob('*.mcap'))
        metadata = stage / 'metadata.yaml'
        manifest = {
            'schema_version': 1, 'scenario': scenario,
            'scenario_contract': SCENARIOS[scenario],
            'source_bag': _identity(source_root / 'bag_0.mcap'),
            'source_metadata': _identity(source_root / 'metadata.yaml'),
            'source_evidence': _identity(source_evidence),
            'g004_contract': _identity(g004_contract),
            'trace_evidence': (_identity(trace_evidence)
                               if trace_evidence is not None else None),
            'generator_source': _generator_source_identity(),
            'topic_mapping': SOURCE_TOPICS,
            'topic_counts': counts,
            'removed_map_to_odom_count': removed,
            'ordered_payload_sha256': digest.hexdigest(),
            'output': {'mcap': _output_identity(mcap),
                       'metadata': _output_identity(metadata)},
            'output_limit_bytes': 134217728,
        }
        if (counts['/scan'] <= 0 or counts['/ground_truth_pose'] <= 0 or
                counts['/odom'] <= 0 or counts['/tf'] <= 0 or
                mcap.stat().st_size > manifest['output_limit_bytes']):
            raise ValueError('Axis B input parity or cap failed')
        (stage / 'axis_b_input_manifest.json').write_bytes(
            canonical_json_bytes(manifest))
        # Preflight runner consumes this stable alias without interpreting it.
        shutil.copyfile(stage / 'axis_b_input_manifest.json',
                        stage / 'sanitizer_manifest.json')
        validate_input(stage)
        stage.rename(output_root)
    return validate_input(output_root)


def main() -> int:
    """Parse canonical Axis B input generation arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-root', required=True, type=Path)
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--scenario', choices=tuple(SCENARIOS), required=True)
    parser.add_argument('--source-evidence', required=True, type=Path)
    parser.add_argument('--g004-contract', required=True, type=Path)
    parser.add_argument('--trace-evidence', type=Path)
    args = parser.parse_args()
    generate(**vars(args))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
