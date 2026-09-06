#!/usr/bin/env python3
"""Relocate an absolute map-image reference without altering other metadata."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
from typing import Any

from canonical_evidence_tree import canonical_tree_manifest

import yaml


TRANSFORM_VERSION = 'map_yaml_image_relocation_v1'
IMAGE_LINE = re.compile(r'^(?P<prefix>\s*image\s*:\s*).+$', re.MULTILINE)


def _record(path: Path) -> dict[str, Any]:
    data = path.read_bytes()
    return {
        'path': str(path.resolve()),
        'bytes': len(data),
        'sha256': hashlib.sha256(data).hexdigest(),
    }


def normalize_map_yaml(
        map_yaml: Path, colocated_pgm: Path, raw_yaml: Path,
        provenance_path: Path) -> dict[str, Any]:
    """Preserve raw bytes and replace only the parsed image field."""
    original_bytes = map_yaml.read_bytes()
    if raw_yaml.exists():
        if raw_yaml.read_bytes() != original_bytes:
            raise ValueError('existing raw YAML does not match input bytes')
    else:
        shutil.copyfile(map_yaml, raw_yaml)
    original_text = original_bytes.decode('utf-8')
    original_document = yaml.safe_load(original_text)
    original_target = Path(original_document['image'])
    if not original_target.is_absolute():
        raise ValueError('normalization requires an absolute source target')
    if original_target.name != colocated_pgm.name:
        raise ValueError('source and colocated PGM basenames differ')
    if not original_target.is_file():
        raise ValueError('original target PGM is unavailable for hash proof')
    if _record(original_target)['sha256'] != _record(colocated_pgm)['sha256']:
        raise ValueError('source and colocated PGM hashes differ')
    matches = list(IMAGE_LINE.finditer(original_text))
    if len(matches) != 1:
        raise ValueError('expected exactly one image metadata line')
    normalized_text = IMAGE_LINE.sub(
        rf'\g<prefix>{colocated_pgm.name}', original_text)
    normalized_document = yaml.safe_load(normalized_text)
    expected_document = dict(original_document)
    expected_document['image'] = colocated_pgm.name
    if normalized_document != expected_document:
        raise ValueError('normalization changed metadata beyond image')
    map_yaml.write_text(normalized_text, encoding='utf-8')
    provenance = {
        'schema_version': 1,
        'transform_version': TRANSFORM_VERSION,
        'parent_raw_yaml': _record(raw_yaml),
        'derived_yaml': _record(map_yaml),
        'original_target': str(original_target),
        'new_relative_target': colocated_pgm.name,
        'original_target_pgm': _record(original_target),
        'colocated_pgm': _record(colocated_pgm),
        'image_only_change_verified': True,
        'basename_equal': True,
        'pgm_sha256_equal': True,
    }
    provenance_path.write_text(
        json.dumps(provenance, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    return provenance


def verify_relocated_copy(
        source_root: Path, durable_root: Path) -> dict[str, Any]:
    """Prove a copied tree differs only by evidenced YAML relocation."""
    source_files = {
        path.relative_to(source_root).as_posix(): path
        for path in source_root.rglob('*') if path.is_file()}
    durable_files = {
        path.relative_to(durable_root).as_posix(): path
        for path in durable_root.rglob('*') if path.is_file()}
    expected_durable = set(source_files)
    relocations = []
    for relative, source_path in source_files.items():
        durable_path = durable_files.get(relative)
        if durable_path is None:
            raise ValueError(f'durable copy is missing: {relative}')
        provenance_relative = (
            relative.removesuffix('.yaml') + '.relocation.json')
        if not relative.endswith('.yaml') or (
                provenance_relative not in durable_files):
            if _record(source_path)['sha256'] != _record(durable_path)['sha256']:
                raise ValueError(f'copied file hash differs: {relative}')
            continue
        raw_relative = relative.removesuffix('.yaml') + '.raw.yaml'
        expected_durable.update((raw_relative, provenance_relative))
        raw_path = durable_files.get(raw_relative)
        provenance_path = durable_files.get(provenance_relative)
        if raw_path is None or provenance_path is None:
            raise ValueError('relocated map lacks raw parent or provenance')
        if source_path.read_bytes() != raw_path.read_bytes():
            raise ValueError('raw map YAML differs from source bytes')
        provenance = json.loads(provenance_path.read_text(encoding='utf-8'))
        derived_document = yaml.safe_load(durable_path.read_text())
        raw_document = yaml.safe_load(raw_path.read_text())
        expected_document = dict(raw_document)
        expected_document['image'] = provenance['new_relative_target']
        checks = (
            provenance['transform_version'] == TRANSFORM_VERSION,
            provenance['parent_raw_yaml']['sha256'] ==
            _record(raw_path)['sha256'],
            provenance['derived_yaml']['sha256'] ==
            _record(durable_path)['sha256'],
            derived_document == expected_document,
            provenance['image_only_change_verified'] is True,
            provenance['pgm_sha256_equal'] is True,
        )
        if not all(checks):
            raise ValueError('map relocation provenance verification failed')
        relocations.append({
            'source_relative_path': relative,
            'raw_parent_relative_path': raw_relative,
            'provenance_relative_path': provenance_relative,
            'transform_version': TRANSFORM_VERSION,
        })
    unexpected = set(durable_files) - expected_durable
    if unexpected:
        raise ValueError(f'unexpected durable files: {sorted(unexpected)}')
    return {
        'schema_version': 1,
        'source_root': str(source_root.resolve()),
        'durable_root': str(durable_root.resolve()),
        'semantic_equivalence_verified': True,
        'unchanged_files_hash_verified': True,
        'relocations': relocations,
        'source_tree': canonical_tree_manifest(source_root, set()),
        'durable_tree': canonical_tree_manifest(durable_root, set()),
    }


def main() -> int:
    """Normalize one copied map YAML and emit its transformation evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-yaml', type=Path)
    parser.add_argument('--colocated-pgm', type=Path)
    parser.add_argument('--raw-yaml', type=Path)
    parser.add_argument('--provenance', type=Path)
    parser.add_argument('--verify-source-root', type=Path)
    parser.add_argument('--verify-durable-root', type=Path)
    parser.add_argument('--copy-report', type=Path)
    args = parser.parse_args()
    if args.verify_source_root is not None:
        if args.verify_durable_root is None or args.copy_report is None:
            parser.error('copy verification requires both roots and report')
        report = verify_relocated_copy(
            args.verify_source_root, args.verify_durable_root)
        args.copy_report.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8')
        return 0
    if any(value is None for value in (
            args.map_yaml, args.colocated_pgm, args.raw_yaml,
            args.provenance)):
        parser.error('normalization requires YAML, PGM, raw, and provenance')
    normalize_map_yaml(
        args.map_yaml, args.colocated_pgm, args.raw_yaml, args.provenance)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
