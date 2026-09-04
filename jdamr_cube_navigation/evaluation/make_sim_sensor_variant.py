#!/usr/bin/env python3
"""Create a traceable Gazebo URDF sensor-profile variant."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


UNITS = {
    'lidar_update_rate_hz': 'Hz',
    'lidar_noise_stddev_m': 'm',
    'imu_update_rate_hz': 'Hz',
    'imu_angular_noise_stddev_radps': 'rad/s',
    'imu_linear_noise_stddev_mps2': 'm/s^2',
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _sensor(root: ET.Element, reference: str, name: str) -> ET.Element:
    matches = [
        sensor
        for gazebo in root.findall('gazebo')
        if gazebo.get('reference') == reference
        for sensor in gazebo.findall('sensor')
        if sensor.get('name') == name
    ]
    if len(matches) != 1:
        raise ValueError(
            f'expected one {reference}/{name} sensor, found {len(matches)}')
    return matches[0]


def _require_element(parent: ET.Element, path: str) -> ET.Element:
    element = parent.find(path)
    if element is None:
        raise ValueError(f'URDF element is missing: {path}')
    return element


def apply_sensor_settings(
        root: ET.Element,
        settings: dict[str, float]) -> dict[str, dict[str, Any]]:
    """Apply supported Gazebo sensor settings and return their old values."""
    unknown = set(settings) - set(UNITS)
    if unknown:
        raise ValueError(f'unsupported settings: {sorted(unknown)}')
    laser = _sensor(root, 'laser_link', 'laser_sensor')
    imu = _sensor(root, 'base_link', 'imu_sensor')
    targets = {
        'lidar_update_rate_hz': [_require_element(laser, 'update_rate')],
        'lidar_noise_stddev_m': [
            _require_element(laser, 'lidar/noise/stddev')],
        'imu_update_rate_hz': [_require_element(imu, 'update_rate')],
        'imu_angular_noise_stddev_radps': [
            _require_element(imu, f'imu/angular_velocity/{axis}/noise/stddev')
            for axis in ('x', 'y', 'z')
        ],
        'imu_linear_noise_stddev_mps2': [
            _require_element(
                imu, f'imu/linear_acceleration/{axis}/noise/stddev')
            for axis in ('x', 'y', 'z')
        ],
    }
    changes = {}
    for name, value in settings.items():
        elements = targets[name]
        previous = [float(element.text) for element in elements]
        for element in elements:
            element.text = str(value)
        changes[name] = {
            'from': previous[0] if len(set(previous)) == 1 else previous,
            'to': value,
            'unit': UNITS[name],
        }
    return changes


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--profile-source', type=Path)
    parser.add_argument('--lidar-update-rate-hz', type=float)
    parser.add_argument('--lidar-noise-stddev-m', type=float)
    parser.add_argument('--imu-update-rate-hz', type=float)
    parser.add_argument('--imu-angular-noise-stddev-radps', type=float)
    parser.add_argument('--imu-linear-noise-stddev-mps2', type=float)
    args = parser.parse_args(argv)
    if not args.base.is_file():
        parser.error('--base must be an existing URDF')
    if args.output.exists():
        parser.error('--output must not already exist')
    if args.profile_source is not None and not args.profile_source.is_file():
        parser.error('--profile-source must exist')
    settings = {
        name: value
        for name, value in (
            ('lidar_update_rate_hz', args.lidar_update_rate_hz),
            ('lidar_noise_stddev_m', args.lidar_noise_stddev_m),
            ('imu_update_rate_hz', args.imu_update_rate_hz),
            ('imu_angular_noise_stddev_radps',
             args.imu_angular_noise_stddev_radps),
            ('imu_linear_noise_stddev_mps2',
             args.imu_linear_noise_stddev_mps2),
        )
        if value is not None
    }
    if not settings:
        parser.error('at least one sensor setting is required')
    if any(value <= 0.0 for value in settings.values()):
        parser.error('sensor settings must be positive')

    tree = ET.parse(args.base)
    changes = apply_sensor_settings(tree.getroot(), settings)
    ET.indent(tree, space='  ')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding='utf-8', xml_declaration=True)
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
        'changes': changes,
        'profile_source': (
            {
                'path': str(args.profile_source.resolve()),
                'sha256': _sha256(args.profile_source),
            }
            if args.profile_source is not None else None),
    }
    manifest_path = args.output.with_suffix('.manifest.json')
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding='utf-8')
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
