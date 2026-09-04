#!/usr/bin/env python3
"""Create one traceable SLAM Toolbox parameter-ablation file."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml


PARAMETER_UNITS = {
    'max_laser_range': 'm',
    'minimum_travel_distance': 'm',
    'minimum_travel_heading': 'rad',
    'do_loop_closing': 'bool',
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def apply_overrides(document: dict[str, Any],
                    overrides: dict[str, Any]) -> dict[str, Any]:
    """Apply only the experiment parameters declared by this module."""
    unknown = set(overrides) - set(PARAMETER_UNITS)
    if unknown:
        raise ValueError(f'unsupported overrides: {sorted(unknown)}')
    try:
        parameters = document['slam_toolbox']['ros__parameters']
    except (KeyError, TypeError) as error:
        raise ValueError('invalid SLAM Toolbox parameter document') from error
    for name, value in overrides.items():
        if name not in parameters:
            raise ValueError(f'base parameter is missing: {name}')
        parameters[name] = value
    return document


def main(argv: list[str] | None = None) -> int:
    """Write the derived YAML and its provenance manifest."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--max-laser-range-m', type=float)
    parser.add_argument('--minimum-travel-distance-m', type=float)
    parser.add_argument('--minimum-travel-heading-rad', type=float)
    parser.add_argument(
        '--do-loop-closing', choices=('true', 'false'), default=None)
    args = parser.parse_args(argv)
    if not args.base.is_file():
        parser.error('--base must be an existing file')
    if args.output.exists():
        parser.error('--output must not already exist')
    numeric_values = (
        args.max_laser_range_m,
        args.minimum_travel_distance_m,
        args.minimum_travel_heading_rad,
    )
    if any(value is not None and value <= 0.0 for value in numeric_values):
        parser.error('numeric overrides must be positive')

    overrides = {
        name: value
        for name, value in (
            ('max_laser_range', args.max_laser_range_m),
            ('minimum_travel_distance', args.minimum_travel_distance_m),
            ('minimum_travel_heading', args.minimum_travel_heading_rad),
            ('do_loop_closing', (
                args.do_loop_closing == 'true'
                if args.do_loop_closing is not None else None)),
        )
        if value is not None
    }
    if not overrides:
        parser.error('at least one override is required')
    document = yaml.safe_load(args.base.read_text(encoding='utf-8'))
    derived = apply_overrides(document, overrides)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(derived, allow_unicode=True, sort_keys=False),
        encoding='utf-8')
    manifest = {
        'schema_version': 1,
        'label': args.label,
        'base': {
            'path': str(args.base.resolve()),
            'sha256': _sha256(args.base),
        },
        'output': {
            'path': str(args.output.resolve()),
            'sha256': _sha256(args.output),
        },
        'overrides': {
            name: {'value': value, 'unit': PARAMETER_UNITS[name]}
            for name, value in overrides.items()
        },
    }
    manifest_path = args.output.with_suffix('.manifest.json')
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding='utf-8')
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
