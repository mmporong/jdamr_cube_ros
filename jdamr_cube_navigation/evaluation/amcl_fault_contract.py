#!/usr/bin/env python3
"""Pure contracts and identity helpers for the G002 AMCL benchmark."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any

import yaml


UPSTREAM_COMMIT = '6be3614013ec586051b86c97b919b293281490fe'
UPSTREAM_ARCHIVE_URL = (
    'https://codeload.github.com/ros-navigation/navigation2/tar.gz/'
    + UPSTREAM_COMMIT)
UPSTREAM_ARCHIVE_SHA256 = (
    '87c3f71033bc51295f1733902acb96fc53dcee1cf25a23b4361955ee0cbf52fe')
UPSTREAM_ARCHIVE_SIZE_BYTES = 64748892
UPSTREAM_LICENSE_SHA256 = (
    'e71fcb65bd31cdbdcf50cf43ab87a0c600ea3ec407495c2e15d1e68bc9c3bfc1')
UPSTREAM_AMCL_TREE_FILE_COUNT = 39
UPSTREAM_AMCL_TREE_SHA256 = (
    '9571c0f6913717ed282f4084033b054ceefe6f3bb09a240d787cc6f8e7d7eb77')
AMCL_BASELINE_FILES = {
    'nav2_amcl/src/amcl_node.cpp':
        '15afd544e5cbbcb16c09090bb41ebff65cbaa47793d6deb1033a4e1ec61771b7',
    'nav2_amcl/include/nav2_amcl/amcl_node.hpp':
        '1afedba75bee4f1c0e9bc7a9a4d08d459b3304a3d402c747f5db731fc39fc348',
    'nav2_amcl/src/pf/pf.c':
        '6747b6f528bd1e3727ebab79eece7ea52379737127ad4431276901ac88d82b6e',
    'nav2_amcl/include/nav2_amcl/pf/pf.hpp':
        'e3a5f7c84e1ae290faa56ff386ffc151c5c071d52421279bebdec34f1296659a',
    'nav2_amcl/src/pf/pf_pdf.c':
        '89a98b1de0baf3e60af7086ce388b0d33bf8f53a8e51e2630cf5bc0720a75724',
    'nav2_amcl/include/nav2_amcl/pf/pf_pdf.hpp':
        '228e4a9bb6fe5ba3bd59879106110bc3721fdec3cab6a5f0e2776f60cb80542c',
}
OVERLAY_CHANGED_FILES = frozenset({
    'nav2_amcl/src/amcl_node.cpp',
    'nav2_amcl/include/nav2_amcl/amcl_node.hpp',
    'nav2_amcl/src/pf/pf_pdf.c',
    'nav2_amcl/include/nav2_amcl/pf/pf_pdf.hpp',
})
PF_C_PATH = 'nav2_amcl/src/pf/pf.c'
SEEDS = (11, 23, 42, 67, 89)
PROFILES = ('P0', 'P1', 'P2')
AXIS_B_SCENARIOS = ('correct_init', 'initial_offset', 'kidnapped')
PUBLISH_TOPICS = ('/scan', '/odom', '/tf', '/tf_static')
FORBIDDEN_OUTPUT_TOPICS = (
    '/clock', '/map', '/amcl_pose', '/cmd_vel', '/cmd_vel_nav',
    '/cmd_vel_smoothed', '/navigate_to_pose/_action/status')
REAL_BAG_SHA256 = (
    '65105177e43eca6153d2543f3cf8e9cc9f7c62b77f258975fdcdbd10b5e2e16a')
REAL_MAP_YAML_SHA256 = (
    '3ddadf69e8f29a2ac4ec17f2d1bfc71f56cda0e805d65c792ddb5f5d46e0d652')
REAL_MAP_PGM_SHA256 = (
    'ee9b0911f41a7da31a92eca67d261b2a96c286f6f49d65b3d6c884fb5937fc6e')
REMOVED_MAP_ODOM_TRANSFORMS = 7290
REAL_BAG_TOPIC_INVENTORY = {
    '/collision_monitor_state': 0,
    '/plan': 22,
    '/cmd_vel_nav': 5613,
    '/amcl_pose': 328,
    '/tf_static': 1,
    '/tf': 29971,
    '/odom': 37810,
    '/cmd_vel': 11182,
    '/imu/data_raw': 37811,
    '/battery_state': 756,
    '/scan': 7304,
}

GIB = 1024 ** 3
MIB = 1024 ** 2
STORAGE_LIMITS = {
    'minimum_start_free_bytes': 6 * GIB,
    'projected_combined_limit_bytes': 2 * GIB,
    'absolute_combined_limit_bytes': int(4.5 * GIB),
    'abort_free_floor_bytes': 4 * GIB,
    'durable_limit_bytes': 1 * GIB,
    'scratch_and_source_build_limit_bytes': 1 * GIB,
    'axis_b_bag_limit_bytes': 128 * MIB,
    'axis_b_total_bag_limit_bytes': 384 * MIB,
    'run_output_mcap_count': 0,
    'run_output_limit_bytes': 2 * MIB,
}


def sha256_file(path: Path) -> str:
    """Return the SHA-256 digest of one regular file."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json_bytes(value: Any) -> bytes:
    """Serialize JSON without path- or locale-dependent formatting."""
    return (json.dumps(
        value, ensure_ascii=False, sort_keys=True,
        separators=(',', ':'), allow_nan=False) + '\n').encode('utf-8')


def strict_json_loads(payload: str) -> Any:
    """Load JSON text while rejecting duplicate keys and non-finite numbers."""
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f'duplicate JSON key: {key}')
            result[key] = value
        return result

    def reject_constant(value):
        raise ValueError(f'non-finite JSON number: {value}')
    return json.loads(
        payload, object_pairs_hook=pairs, parse_constant=reject_constant)


def strict_json_load(path: Path) -> Any:
    """Load a UTF-8 JSON file through the strict decoder."""
    return strict_json_loads(path.read_text(encoding='utf-8'))


def exact_regular_file(path: Path, expected_sha256: str | None = None) -> dict:
    """Return identity after rejecting symlinks and non-canonical paths."""
    path = path.expanduser()
    if not path.is_absolute() or path != path.resolve():
        raise ValueError(f'path must be absolute and canonical: {path}')
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'not a regular non-symlink file: {path}')
    digest = sha256_file(path)
    if expected_sha256 is not None and digest != expected_sha256:
        raise ValueError(f'SHA-256 mismatch: {path}')
    return {'path': str(path), 'size_bytes': path.stat().st_size,
            'sha256': digest}


def map_identity(yaml_path: Path, expected_yaml_sha256: str,
                 expected_pgm_sha256: str) -> dict:
    """Bind map YAML semantics to its referenced PGM bytes."""
    yaml_record = exact_regular_file(yaml_path, expected_yaml_sha256)
    profile = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    if set(profile) != {
            'image', 'mode', 'resolution', 'origin', 'negate',
            'occupied_thresh', 'free_thresh'}:
        raise ValueError('map YAML schema drift')
    image_path = (yaml_path.parent / profile['image']).resolve()
    pgm_record = exact_regular_file(image_path, expected_pgm_sha256)
    resolution = profile['resolution']
    origin = profile['origin']
    if not isinstance(resolution, (int, float)) or isinstance(resolution, bool):
        raise ValueError('invalid map resolution')
    if not isinstance(origin, list) or len(origin) != 3:
        raise ValueError('invalid map origin')
    return {'yaml': yaml_record, 'pgm': pgm_record,
            'resolution_m_per_cell': float(resolution),
            'origin': [float(value) for value in origin]}


def validate_storage_budget(start_free_bytes: int, durable_bytes: int,
                            scratch_bytes: int) -> dict:
    """Apply the preregistered storage caps without hidden fallback."""
    values = (start_free_bytes, durable_bytes, scratch_bytes)
    if any(type(value) is not int or value < 0 for value in values):
        raise ValueError('storage values must be nonnegative exact integers')
    projected = durable_bytes + scratch_bytes
    remaining = start_free_bytes - projected
    failures = []
    if start_free_bytes < STORAGE_LIMITS['minimum_start_free_bytes']:
        failures.append('start_free_below_6_gib')
    if durable_bytes > STORAGE_LIMITS['durable_limit_bytes']:
        failures.append('durable_above_1_gib')
    if scratch_bytes > STORAGE_LIMITS['scratch_and_source_build_limit_bytes']:
        failures.append('scratch_above_1_gib')
    if projected > STORAGE_LIMITS['projected_combined_limit_bytes']:
        failures.append('projected_above_2_gib')
    if projected > STORAGE_LIMITS['absolute_combined_limit_bytes']:
        failures.append('absolute_above_4_5_gib')
    if remaining < STORAGE_LIMITS['abort_free_floor_bytes']:
        failures.append('remaining_below_4_gib')
    return {'pass': not failures, 'failures': failures,
            'start_free_bytes': start_free_bytes,
            'durable_bytes': durable_bytes, 'scratch_bytes': scratch_bytes,
            'projected_combined_bytes': projected,
            'projected_remaining_free_bytes': remaining,
            'limits': dict(STORAGE_LIMITS)}


def current_free_bytes(path: Path) -> int:
    """Return free bytes on the filesystem containing path."""
    return os.statvfs(path).f_bavail * os.statvfs(path).f_frsize
