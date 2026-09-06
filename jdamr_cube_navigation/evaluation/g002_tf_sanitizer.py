#!/usr/bin/env python3
"""Create the one reusable G002 replay bag without recorded map-to-odom TF."""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import struct
import tempfile

from amcl_fault_contract import canonical_json_bytes
from amcl_fault_contract import PUBLISH_TOPICS
from amcl_fault_contract import REMOVED_MAP_ODOM_TRANSFORMS
from amcl_fault_contract import sha256_file
from amcl_fault_contract import strict_json_load


def _canonical_directory(path: Path, must_exist: bool) -> Path:
    path = path.expanduser()
    if not path.is_absolute():
        raise ValueError(f'path is not absolute: {path}')
    if must_exist:
        if path.is_symlink() or not path.is_dir() or path != path.resolve():
            raise ValueError(f'bag directory is not canonical: {path}')
    elif path.exists() or path.is_symlink() or path.parent != path.parent.resolve():
        raise ValueError(f'output path is not canonical and absent: {path}')
    return path


def _bag_files(root: Path) -> tuple[Path, Path]:
    children = list(root.iterdir())
    if any(child.is_symlink() or not child.is_file() for child in children):
        raise ValueError('bag root may contain regular files only')
    mcap = [child for child in children if child.suffix == '.mcap']
    metadata = [child for child in children if child.name == 'metadata.yaml']
    if len(mcap) != 1 or len(metadata) != 1 or len(children) != 2:
        raise ValueError('input must contain exactly one MCAP and metadata.yaml')
    return mcap[0], metadata[0]


def _remove_created_output(root: Path) -> None:
    """Remove only a canonical output tree created by this invocation."""
    if root.parent != root.parent.resolve():
        raise RuntimeError('refusing to remove a non-canonical output root')
    if root.is_symlink():
        root.unlink()
        return
    if not root.is_dir() or root != root.resolve():
        raise RuntimeError('refusing to remove a non-canonical output root')
    for path in sorted(root.rglob('*'), reverse=True):
        if not path.is_symlink() and not path.is_file() and not path.is_dir():
            raise RuntimeError('refusing to remove output with special entries')
        if path.is_symlink() or path.is_file():
            path.unlink()
        else:
            path.rmdir()
    root.rmdir()


def validate_sanitized_bag(root: Path) -> dict:
    """Reopen and hash-check a completed sanitizer output."""
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    root = _canonical_directory(root, True)
    children = list(root.iterdir())
    if any(child.is_symlink() or not child.is_file() for child in children):
        raise ValueError('sanitized root may contain regular files only')
    if {child.name for child in children} != {
            'bag_0.mcap', 'metadata.yaml', 'sanitizer_manifest.json'}:
        raise ValueError('sanitized output file inventory drift')
    manifest = strict_json_load(root / 'sanitizer_manifest.json')
    if set(manifest) != {
            'schema_version', 'source', 'output_topics',
            'input_topic_inventory', 'removed_map_to_odom_transforms',
            'tf_semantic_parity', 'topic_parity', 'output_limit_bytes',
            'output'}:
        raise ValueError('sanitizer manifest schema drift')
    if manifest['schema_version'] != 1:
        raise ValueError('unsupported sanitizer manifest schema')
    source_record = manifest['source']
    if set(source_record) != {
            'path', 'mcap_size_bytes', 'mcap_sha256', 'metadata_sha256'}:
        raise ValueError('sanitizer source identity schema drift')
    source_root = _canonical_directory(Path(source_record['path']), True)
    source_mcap, source_metadata = _bag_files(source_root)
    if (source_mcap.stat().st_size != source_record['mcap_size_bytes'] or
            sha256_file(source_mcap) != source_record['mcap_sha256'] or
            sha256_file(source_metadata) != source_record['metadata_sha256']):
        raise ValueError('sanitizer source hash or size mismatch')
    output = manifest['output']
    if set(output) != {
            'mcap_name', 'mcap_size_bytes', 'mcap_sha256',
            'metadata_sha256'} or output['mcap_name'] != 'bag_0.mcap':
        raise ValueError('sanitizer output identity schema drift')
    mcap = root / 'bag_0.mcap'
    metadata = root / 'metadata.yaml'
    if (mcap.stat().st_size != output['mcap_size_bytes'] or
            sha256_file(mcap) != output['mcap_sha256'] or
            sha256_file(metadata) != output['metadata_sha256']):
        raise ValueError('sanitized output hash or size mismatch')
    output_limit = manifest['output_limit_bytes']
    if (type(output_limit) is not int or output_limit <= 0 or
            mcap.stat().st_size > output_limit):
        raise ValueError('sanitized output exceeds recorded cap')
    with mcap.open('rb') as stream:
        prefix = stream.read(8)
        stream.seek(-8, 2)
        suffix = stream.read(8)
    if prefix != b'\x89MCAP0\r\n' or suffix != b'\x89MCAP0\r\n':
        raise ValueError('sanitized MCAP magic mismatch')
    message_types = {'/scan': LaserScan, '/odom': Odometry,
                     '/tf': TFMessage, '/tf_static': TFMessage}
    source_reader = rosbag2_py.SequentialReader()
    source_reader.open(rosbag2_py.StorageOptions(
        uri=str(source_root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    source_inventory = {item.name: 0 for item in
                        source_reader.get_all_topics_and_types()}
    source_stats = {}
    source_dynamic_messages = []
    source_static_messages = []
    removed = 0
    while source_reader.has_next():
        topic, serialized, timestamp_ns = source_reader.read_next()
        source_inventory[topic] += 1
        if topic not in PUBLISH_TOPICS:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic == '/tf':
            transforms = []
            for transform in message.transforms:
                record = _tf_record(transform)
                if (record['parent'].lstrip('/') == 'map' and
                        record['child'].lstrip('/') == 'odom'):
                    removed += 1
                else:
                    transforms.append(record)
            source_dynamic_messages.append({
                'storage_ns': timestamp_ns, 'transforms': transforms})
        elif topic == '/tf_static':
            source_static_messages.append({
                'storage_ns': timestamp_ns,
                'transforms': [_tf_record(value)
                               for value in message.transforms]})
        header_ns = (_stamp_ns(message.header.stamp)
                     if topic in ('/scan', '/odom') else None)
        _raw_stats_update(
            source_stats, topic, timestamp_ns, serialized, header_ns,
            include_payload=topic in ('/scan', '/odom'))
    source_reader.close()
    if (source_inventory != manifest['input_topic_inventory'] or
            removed != manifest['removed_map_to_odom_transforms']):
        raise ValueError('sanitizer source inventory or removal drift')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topic_types = {item.name: item.type for item in
                   reader.get_all_topics_and_types()}
    if set(topic_types) != set(PUBLISH_TOPICS):
        raise ValueError('sanitized output topic drift')
    counts = {topic: 0 for topic in topic_types}
    output_stats = {}
    output_dynamic_messages = []
    output_static_messages = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        counts[topic] += 1
        message = deserialize_message(serialized, message_types[topic])
        header_ns = (_stamp_ns(message.header.stamp)
                     if topic in ('/scan', '/odom') else None)
        _raw_stats_update(
            output_stats, topic, timestamp_ns, serialized, header_ns,
            include_payload=topic in ('/scan', '/odom'))
        if topic == '/tf':
            if _canonical_tf_serialization(bytes(serialized)) != bytes(serialized):
                raise ValueError('sanitized TF serialization is not canonical')
            transforms = []
            for transform in message.transforms:
                record = _tf_record(transform)
                if (record['parent'].lstrip('/') == 'map' and
                        record['child'].lstrip('/') == 'odom'):
                    raise ValueError('sanitized output contains map-to-odom')
                transforms.append(record)
            output_dynamic_messages.append({
                'storage_ns': timestamp_ns, 'transforms': transforms})
        elif topic == '/tf_static':
            output_static_messages.append({
                'storage_ns': timestamp_ns,
                'transforms': [_tf_record(value)
                               for value in message.transforms]})
    reader.close()
    expected_counts = {
        topic: manifest['topic_parity'][topic]['message_count']
        for topic in PUBLISH_TOPICS}
    if counts != expected_counts:
        raise ValueError('sanitized output message-count drift')
    if (_finish_stats(source_stats) != manifest['topic_parity'] or
            _finish_stats(output_stats) != manifest['topic_parity']):
        raise ValueError('sanitized topic payload or stamp parity drift')
    parity = manifest['tf_semantic_parity']
    if set(parity) != {'dynamic_non_map', 'static'}:
        raise ValueError('TF semantic parity schema drift')
    _validate_tf_parity(
        parity['dynamic_non_map'], source_dynamic_messages,
        output_dynamic_messages)
    _validate_tf_parity(
        parity['static'], source_static_messages, output_static_messages)
    return manifest


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _tf_record(transform) -> dict:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    values = [translation.x, translation.y, translation.z,
              rotation.x, rotation.y, rotation.z, rotation.w]
    if any(not math.isfinite(float(value)) for value in values):
        raise ValueError('non-finite TF payload')
    return {
        'stamp_ns': _stamp_ns(transform.header.stamp),
        'parent': transform.header.frame_id,
        'child': transform.child_frame_id,
        'translation': [float(value) for value in values[:3]],
        'rotation': [float(value) for value in values[3:]],
    }


def _canonical_tf_serialization(serialized: bytes) -> bytes:
    """Zero CDR alignment padding in one tf2_msgs/TFMessage payload."""
    payload = bytearray(serialized)
    if len(payload) < 8 or bytes(payload[:4]) != b'\x00\x01\x00\x00':
        raise ValueError('unsupported TF CDR encapsulation')
    endian = '<'

    def uint32(offset: int) -> int:
        if offset + 4 > len(payload):
            raise ValueError('truncated TF CDR payload')
        return struct.unpack_from(endian + 'I', payload, offset)[0]

    def align(offset: int, size: int) -> int:
        data_offset = offset - 4
        aligned = 4 + (data_offset + size - 1) // size * size
        if aligned > len(payload):
            raise ValueError('truncated TF CDR padding')
        payload[offset:aligned] = b'\x00' * (aligned - offset)
        return aligned

    offset = 4
    count = uint32(offset)
    offset += 4
    for _ in range(count):
        offset = align(offset, 4)
        offset += 8
        for _ in range(2):
            offset = align(offset, 4)
            length = uint32(offset)
            offset += 4
            if length < 1 or offset + length > len(payload):
                raise ValueError('invalid TF CDR string')
            if payload[offset + length - 1] != 0:
                raise ValueError('TF CDR string lacks terminator')
            offset += length
        offset = align(offset, 8)
        offset += 7 * 8
        if offset > len(payload):
            raise ValueError('truncated TF CDR transform')
    if offset != len(payload):
        raise ValueError('unexpected trailing TF CDR bytes')
    return bytes(payload)


def _canonical_tf_serialized(serialized: bytes) -> bytes:
    """Compatibility alias for the canonical TF serialization helper."""
    return _canonical_tf_serialization(serialized)


def _digest_records(records: list[dict]) -> str:
    digest = hashlib.sha256()
    for record in records:
        digest.update(canonical_json_bytes(record))
    return digest.hexdigest()


def _tf_parity_record(source_messages: list[dict],
                      output_messages: list[dict]) -> dict:
    return {
        'message_count': len(source_messages),
        'source_transform_count': sum(
            len(item['transforms']) for item in source_messages),
        'output_transform_count': sum(
            len(item['transforms']) for item in output_messages),
        'source_ordered_digest': _digest_records(source_messages),
        'output_ordered_digest': _digest_records(output_messages),
    }


def _validate_tf_parity(parity: dict, source_messages: list[dict],
                        output_messages: list[dict]) -> None:
    if set(parity) != {
            'message_count', 'source_transform_count',
            'output_transform_count', 'source_ordered_digest',
            'output_ordered_digest'}:
        raise ValueError('TF semantic parity record schema drift')
    expected = _tf_parity_record(source_messages, output_messages)
    if (source_messages != output_messages or parity != expected):
        raise ValueError('sanitized TF semantic parity drift')


def _raw_stats_update(stats: dict, topic: str, timestamp_ns: int,
                      serialized: bytes, header_stamp_ns: int | None,
                      include_payload: bool = True) -> None:
    item = stats.setdefault(topic, {
        'message_count': 0,
        'storage_stamp_sha256': hashlib.sha256(),
    })
    if include_payload and 'payload_sha256' not in item:
        item['header_stamp_sha256'] = hashlib.sha256()
        item['payload_sha256'] = hashlib.sha256()
    if include_payload != ('payload_sha256' in item):
        raise ValueError(f'inconsistent payload parity mode: {topic}')
    item['message_count'] += 1
    item['storage_stamp_sha256'].update(f'{timestamp_ns}\n'.encode())
    if include_payload and header_stamp_ns is not None:
        item['header_stamp_sha256'].update(f'{header_stamp_ns}\n'.encode())
    if include_payload:
        item['payload_sha256'].update(bytes(serialized))


def _finish_stats(stats: dict) -> dict:
    return {
        topic: {
            key: value.hexdigest() if hasattr(value, 'hexdigest') else value
            for key, value in item.items()
        }
        for topic, item in sorted(stats.items())
    }


def _write_filtered_bag(source_root: Path, stage: Path) -> dict:
    """Write one bag and destroy rosbag handles before returning."""
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    converter = rosbag2_py.ConverterOptions('cdr', 'cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(source_root), storage_id='mcap'), converter)
    source_topics = {topic.name: topic for topic in
                     reader.get_all_topics_and_types()}
    for topic in PUBLISH_TOPICS:
        if topic not in source_topics:
            raise ValueError(f'required input topic missing: {topic}')
    inventory = {topic: 0 for topic in source_topics}
    source_stats = {}
    output_stats = {}
    source_dynamic_messages = []
    output_dynamic_messages = []
    source_static_messages = []
    output_static_messages = []
    removed = 0
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(
        uri=str(stage), storage_id='mcap'), converter)
    for topic_name in PUBLISH_TOPICS:
        topic = source_topics[topic_name]
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=topic.name, type=topic.type,
            serialization_format=topic.serialization_format,
            offered_qos_profiles=topic.offered_qos_profiles,
            type_description_hash=topic.type_description_hash))
    message_types = {'/scan': LaserScan, '/odom': Odometry,
                     '/tf': TFMessage, '/tf_static': TFMessage}
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        inventory[topic] += 1
        if topic not in PUBLISH_TOPICS:
            continue
        message = deserialize_message(serialized, message_types[topic])
        output_serialized = serialized
        if topic == '/tf':
            kept = []
            transforms = []
            for transform in message.transforms:
                record = _tf_record(transform)
                if (record['parent'].lstrip('/') == 'map' and
                        record['child'].lstrip('/') == 'odom'):
                    removed += 1
                else:
                    transforms.append(record)
                    kept.append(transform)
            source_dynamic_messages.append({
                'storage_ns': timestamp_ns, 'transforms': transforms})
            message.transforms = kept
            output_serialized = _canonical_tf_serialization(
                bytes(serialize_message(message)))
            round_trip = deserialize_message(output_serialized, TFMessage)
            if ([_tf_record(value) for value in round_trip.transforms] !=
                    [_tf_record(value) for value in kept]):
                raise RuntimeError('canonical TF serialization changed semantics')
            output_dynamic_messages.append({
                'storage_ns': timestamp_ns,
                'transforms': [_tf_record(value) for value in kept]})
        elif topic == '/tf_static':
            semantic = {
                'storage_ns': timestamp_ns,
                'transforms': [_tf_record(value)
                               for value in message.transforms]}
            source_static_messages.append(semantic)
            output_static_messages.append(semantic)
        header_ns = (_stamp_ns(message.header.stamp)
                     if topic in ('/scan', '/odom') else None)
        include_payload = topic in ('/scan', '/odom')
        _raw_stats_update(source_stats, topic, timestamp_ns,
                          serialized, header_ns, include_payload)
        _raw_stats_update(output_stats, topic, timestamp_ns,
                          output_serialized, header_ns, include_payload)
        writer.write(topic, output_serialized, timestamp_ns)
    reader.close()
    writer.close()
    del reader
    del writer
    return {
        'inventory': inventory,
        'source_stats': _finish_stats(source_stats),
        'output_stats': _finish_stats(output_stats),
        'source_dynamic_messages': source_dynamic_messages,
        'output_dynamic_messages': output_dynamic_messages,
        'source_static_messages': source_static_messages,
        'output_static_messages': output_static_messages,
        'removed': removed,
    }


def sanitize_bag(source_root: Path, output_root: Path,
                 expected_input_sha256: str,
                 expected_removed_count: int = REMOVED_MAP_ODOM_TRANSFORMS,
                 output_limit_bytes: int = 1024 ** 3,
                 expected_input_inventory: dict | None = None) -> dict:
    """Filter only map-to-odom transforms and verify ordered parity."""
    source_root = _canonical_directory(source_root, True)
    output_root = _canonical_directory(output_root, False)
    if type(output_limit_bytes) is not int or output_limit_bytes <= 0:
        raise ValueError('output limit must be a positive exact integer')
    source_mcap, source_metadata = _bag_files(source_root)
    if sha256_file(source_mcap) != expected_input_sha256:
        raise ValueError('canonical input MCAP hash mismatch')
    source_before = {
        'mcap_sha256': sha256_file(source_mcap),
        'metadata_sha256': sha256_file(source_metadata),
    }
    output_root.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
            prefix='.g002-sanitize-', dir=output_root.parent) as temp_name:
        stage = Path(temp_name) / 'bag'
        result = _write_filtered_bag(source_root, stage)
        inventory = result['inventory']
        if expected_input_inventory is not None and inventory != expected_input_inventory:
            raise ValueError('input topic inventory drift')
        removed = result['removed']
        if type(expected_removed_count) is not int or removed != expected_removed_count:
            raise ValueError(
                f'map-to-odom removal count {removed} != {expected_removed_count}')
        source_final = result['source_stats']
        output_final = result['output_stats']
        if source_final != output_final:
            differing = sorted(
                topic for topic in set(source_final) | set(output_final)
                if source_final.get(topic) != output_final.get(topic))
            raise RuntimeError(f'topic parity drift: {differing}')
        dynamic_parity = _tf_parity_record(
            result['source_dynamic_messages'],
            result['output_dynamic_messages'])
        static_parity = _tf_parity_record(
            result['source_static_messages'], result['output_static_messages'])
        if (result['source_dynamic_messages'] !=
                result['output_dynamic_messages'] or
                result['source_static_messages'] !=
                result['output_static_messages']):
            raise RuntimeError('TF semantic parity drift')
        output_mcap, output_metadata = _bag_files(stage)
        if output_mcap.stat().st_size > output_limit_bytes:
            raise RuntimeError('sanitized MCAP exceeds scratch cap')
        with output_mcap.open('rb') as stream:
            prefix = stream.read(8)
            stream.seek(-8, 2)
            suffix = stream.read(8)
        if prefix != b'\x89MCAP0\r\n' or suffix != b'\x89MCAP0\r\n':
            raise RuntimeError('sanitized MCAP magic mismatch')
        manifest = {
            'schema_version': 1,
            'source': {'path': str(source_root),
                       'mcap_size_bytes': source_mcap.stat().st_size,
                       **source_before},
            'output_topics': list(PUBLISH_TOPICS),
            'input_topic_inventory': inventory,
            'removed_map_to_odom_transforms': removed,
            'output_limit_bytes': output_limit_bytes,
            'tf_semantic_parity': {
                'dynamic_non_map': dynamic_parity,
                'static': static_parity},
            'topic_parity': source_final,
            'output': {
                'mcap_name': output_mcap.name,
                'mcap_size_bytes': output_mcap.stat().st_size,
                'mcap_sha256': sha256_file(output_mcap),
                'metadata_sha256': sha256_file(output_metadata),
            },
        }
        (stage / 'sanitizer_manifest.json').write_bytes(
            canonical_json_bytes(manifest))
        if source_before != {
                'mcap_sha256': sha256_file(source_mcap),
                'metadata_sha256': sha256_file(source_metadata)}:
            raise RuntimeError('source bag changed during sanitization')
        validate_sanitized_bag(stage)
        stage.rename(output_root)
    try:
        return validate_sanitized_bag(output_root)
    except Exception:
        _remove_created_output(output_root)
        raise


def main() -> int:
    """Sanitize one canonical input bag from command-line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--input-sha256', required=True)
    parser.add_argument('--removed-count', type=int,
                        default=REMOVED_MAP_ODOM_TRANSFORMS)
    args = parser.parse_args()
    sanitize_bag(args.source_root, args.output_root, args.input_sha256,
                 args.removed_count)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
