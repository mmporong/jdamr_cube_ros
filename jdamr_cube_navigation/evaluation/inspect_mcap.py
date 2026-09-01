#!/usr/bin/env python3
"""Inspect an MCAP rosbag without starting ROS nodes or replaying messages."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from typing import Any

from mcap.reader import make_reader
from mcap.records import Chunk, DataEnd, Footer
from mcap.stream_reader import StreamReader


def sha256_file(path: Path) -> str:
    """Return the SHA-256 digest for *path*."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def inspect(path: Path) -> dict[str, Any]:
    """Return deterministic integrity and topic metadata for an MCAP file."""
    topic_counts: Counter[str] = Counter()

    with path.open('rb') as stream:
        reader = make_reader(stream, validate_crcs=True)
        summary = reader.get_summary()
        if summary is None or summary.statistics is None:
            raise ValueError('MCAP summary/statistics are required')
        for _schema, channel, _message in reader.iter_messages():
            topic_counts[channel.topic] += 1

    record_counts: Counter[str] = Counter()
    chunks_with_crc = 0
    chunk_count = 0
    data_section_crc = 0
    summary_crc = 0
    with path.open('rb') as stream:
        for record in StreamReader(
            stream,
            emit_chunks=True,
            validate_crcs=True,
        ).records:
            record_counts[type(record).__name__] += 1
            if isinstance(record, Chunk):
                chunk_count += 1
                if record.uncompressed_crc != 0:
                    chunks_with_crc += 1
            elif isinstance(record, DataEnd):
                data_section_crc = record.data_section_crc
            elif isinstance(record, Footer):
                summary_crc = record.summary_crc

    statistics = summary.statistics
    return {
        'path': str(path.resolve()),
        'size_bytes': path.stat().st_size,
        'sha256': sha256_file(path),
        'message_count': statistics.message_count,
        'duration_ns': statistics.message_end_time - statistics.message_start_time,
        'topic_counts': dict(sorted(topic_counts.items())),
        'integrity': {
            'validate_crcs': 'PASS',
            'chunk_count': chunk_count,
            'chunks_with_crc': chunks_with_crc,
            'data_section_crc': data_section_crc,
            'summary_crc': summary_crc,
            'chunk_index_count': record_counts['ChunkIndex'],
            'message_index_count': record_counts['MessageIndex'],
        },
    }


def main() -> None:
    """Parse arguments and print the inspection result as JSON."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='MCAP rosbag file to inspect')
    args = parser.parse_args()
    if not args.bag.is_file():
        parser.error(f'file not found: {args.bag}')
    print(json.dumps(inspect(args.bag), ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
