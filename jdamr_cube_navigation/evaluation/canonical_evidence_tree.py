#!/usr/bin/env python3
"""Create or verify a canonical evidence-directory hash manifest."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


ALGORITHM = {
    'name': 'sha256_sorted_relative_path_size_file_sha256_v1',
    'path_encoding': 'UTF-8 POSIX relative path',
    'entry_order': 'ascending bytewise UTF-8 relative path',
    'entry_encoding': '<file_sha256> <size_bytes> <relative_path>\\n',
    'file_types': 'regular files only',
    'symlink_policy': 'reject',
    'manifest_policy': 'output manifest must be explicitly excluded',
}


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def canonical_tree_manifest(
        root: Path, excluded_relative_paths: set[str]) -> dict[str, Any]:
    """Return deterministic entries and their aggregate digest."""
    root = root.resolve()
    if not root.is_dir():
        raise ValueError(f'evidence root is not a directory: {root}')
    entries = []
    for path in root.rglob('*'):
        relative = path.relative_to(root).as_posix()
        if relative in excluded_relative_paths:
            continue
        if path.is_symlink():
            raise ValueError(f'symlink is not canonical evidence: {relative}')
        if not path.is_file():
            continue
        if '\n' in relative or '\r' in relative:
            raise ValueError('evidence paths cannot contain line breaks')
        entries.append({
            'relative_path': relative,
            'size_bytes': path.stat().st_size,
            'sha256': _file_hash(path),
        })
    entries.sort(key=lambda item: item['relative_path'].encode('utf-8'))
    digest = hashlib.sha256()
    for entry in entries:
        digest.update(
            f"{entry['sha256']} {entry['size_bytes']} "
            f"{entry['relative_path']}\n".encode('utf-8'))
    return {
        'schema_version': 1,
        'root': str(root),
        'algorithm': ALGORITHM,
        'excluded_relative_paths': sorted(excluded_relative_paths),
        'file_count': len(entries),
        'total_bytes': sum(item['size_bytes'] for item in entries),
        'tree_sha256': digest.hexdigest(),
        'entries': entries,
    }


def verify_manifest(document: dict[str, Any]) -> bool:
    """Recompute a stored manifest using its declared root and exclusions."""
    current = canonical_tree_manifest(
        Path(document['root']), set(document['excluded_relative_paths']))
    return all((
        document.get('schema_version') == current['schema_version'],
        document.get('algorithm') == current['algorithm'],
        document.get('file_count') == current['file_count'],
        document.get('total_bytes') == current['total_bytes'],
        document.get('tree_sha256') == current['tree_sha256'],
        document.get('entries') == current['entries'],
    ))


def main() -> int:
    """Write a canonical manifest or independently verify one."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--exclude', action='append', default=[])
    parser.add_argument('--verify', type=Path)
    args = parser.parse_args()
    if args.verify is not None:
        document = json.loads(args.verify.read_text(encoding='utf-8'))
        if not verify_manifest(document):
            print('INVALID')
            return 2
        print('PASS')
        return 0
    if args.root is None or args.output is None:
        parser.error('--root and --output are required when not verifying')
    root = args.root.resolve()
    output = args.output.resolve()
    excluded = set(args.exclude)
    if output.is_relative_to(root):
        relative_output = output.relative_to(root).as_posix()
        if relative_output not in excluded:
            parser.error('output inside root must be explicitly excluded')
    document = canonical_tree_manifest(root, excluded)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(document['tree_sha256'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
