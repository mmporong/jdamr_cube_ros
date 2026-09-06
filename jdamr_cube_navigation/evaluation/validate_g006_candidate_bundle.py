#!/usr/bin/env python3
"""Fail-closed validator for the immutable G006 candidate bundle."""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from build_g006_candidate_bundle import _hardware_boundary
from build_g006_candidate_bundle import _measurement_contract
from build_g006_candidate_bundle import _protocol_id
from build_g006_candidate_bundle import _scenario_contract, _topologies
from build_g006_candidate_bundle import HARDWARE_STUBS
from build_g006_candidate_bundle import HARNESS_SOURCES
from build_g006_candidate_bundle import PREDECESSORS, PROFILE_SOURCES
from g006_candidate_contract import _complete_readiness_valid
from g006_candidate_contract import BUNDLE_LIMIT_BYTES, CLAIMS_LOCK
from g006_candidate_contract import evidence_identity_shape_valid
from g006_candidate_contract import file_identity, sha256_file
from g006_candidate_contract import final_decision
from g006_candidate_contract import GATE_FAMILIES, SCENARIOS, SEED_PENDING
from g006_candidate_contract import (
    held_out_seed_lock,
    PREDECESSOR_KEYS,
    RUNTIME_ENTITIES,
    RUNTIME_ENTITY_FIELDS,
)
from g006_candidate_contract import strict_json_load, tree_inventory
from g006_candidate_contract import validate_cartographer_metrics
from g006_candidate_contract import validate_gate_matrix
from probe_g006_candidate_bundle import probe_bundle
import yaml


TOP_KEYS = {
    'schema_version', 'claim_scope', 'bundle_id', 'profiles', 'topologies',
    'harness_sources', 'hardware_boundary', 'runtime', 'predecessors',
    'selection', 'seed_lock',
    'scenario_contract', 'measurement_contract', 'gate_applicability',
    'external_hard_gates', 'pi_historical_saved_nav', 'pi_current_bundle',
    'pi_resource_gate', 'claims', 'readiness_evidence', 'decision',
    'payload_tree'}
SOURCE_KEYS = {
    'source_root', 'source_relative_path', 'relative_path', 'mode_octal',
    'size_bytes', 'sha256', 'bundle_sha256', 'bundle_size_bytes'}


def _exact_keys(value, keys) -> bool:
    return isinstance(value, dict) and set(value) == set(keys)


def _identity_valid(root: Path, record: dict) -> bool:
    if not _exact_keys(record, SOURCE_KEYS):
        return False
    if (not isinstance(record['relative_path'], str) or
            not isinstance(record['source_root'], str) or
            not isinstance(record['source_relative_path'], str) or
            not isinstance(record['mode_octal'], str) or
            len(record['mode_octal']) != 4 or
            any(character not in '01234567'
                for character in record['mode_octal']) or
            isinstance(record['size_bytes'], bool) or
            not isinstance(record['size_bytes'], int) or
            record['size_bytes'] < 0 or
            isinstance(record['bundle_size_bytes'], bool) or
            not isinstance(record['bundle_size_bytes'], int) or
            not isinstance(record['sha256'], str) or
            not isinstance(record['bundle_sha256'], str) or
            any(len(value) != 64 or any(
                character not in '0123456789abcdef' for character in value)
                for value in (record['sha256'], record['bundle_sha256']))):
        return False
    relative = Path(record['relative_path'])
    if relative.is_absolute() or '..' in relative.parts:
        return False
    path = root / 'payload' / relative
    try:
        actual = file_identity(path, relative.as_posix())
    except (OSError, ValueError, TypeError):
        return False
    if not all(actual[key] == record[key]
               for key in ('relative_path', 'mode_octal', 'size_bytes',
                           'sha256')):
        return False
    if (record['bundle_sha256'] != actual['sha256'] or
            record['bundle_size_bytes'] != actual['size_bytes']):
        return False
    return (record['source_relative_path'] == 'resolved_params.json'
            if record['source_root'] == 'generated' else True)


def _sealed_tree_valid(tree: dict) -> bool:
    """Validate a sealed tree inventory without accessing its source root."""
    if set(tree) != {
            'algorithm', 'file_count', 'total_bytes', 'tree_sha256', 'entries'}:
        return False
    entries = tree['entries']
    if (not isinstance(entries, list) or
            tree['algorithm'] != 'sha256_path_size_mode_file_sha256_v1' or
            isinstance(tree['file_count'], bool) or
            not isinstance(tree['file_count'], int) or
            tree['file_count'] != len(entries) or
            isinstance(tree['total_bytes'], bool) or
            not isinstance(tree['total_bytes'], int)):
        return False
    expected = {'relative_path', 'mode_octal', 'size_bytes', 'sha256'}
    if (any(not isinstance(item, dict) or set(item) != expected
            for item in entries) or
            [item['relative_path'] for item in entries] != sorted(
                item['relative_path'] for item in entries) or
            len({item['relative_path'] for item in entries}) != len(entries) or
            tree['total_bytes'] != sum(item['size_bytes'] for item in entries)):
        return False
    for item in entries:
        relative = item['relative_path']
        if (not isinstance(relative, str) or Path(relative).is_absolute() or
                '..' in Path(relative).parts or
                isinstance(item['size_bytes'], bool) or
                not isinstance(item['size_bytes'], int) or
                item['size_bytes'] < 0 or
                not isinstance(item['mode_octal'], str) or
                len(item['mode_octal']) != 4 or
                any(character not in '01234567'
                    for character in item['mode_octal']) or
                not isinstance(item['sha256'], str) or
                len(item['sha256']) != 64 or
                any(character not in '0123456789abcdef'
                    for character in item['sha256'])):
            return False
    payload = b''.join(
        (f"{item['sha256']} {item['size_bytes']} {item['mode_octal']} "
         f"{item['relative_path']}\n").encode() for item in entries)
    return hashlib.sha256(payload).hexdigest() == tree['tree_sha256']


def _portable_predecessor_record_valid(root: Path, name: str,
                                       record: dict) -> bool:
    """Validate a complete predecessor using only copied snapshot bytes."""
    keys = {'canonical_root', 'status', 'identities', 'artifact_tree',
            'verified_conclusion', 'snapshots'}
    if not _exact_keys(record, keys) or record['status'] != 'PASS':
        return False
    canonical = record['canonical_root']
    if (not isinstance(canonical, str) or not Path(canonical).is_absolute() or
            Path(canonical).as_posix() != canonical or
            not _sealed_tree_valid(record['artifact_tree'])):
        return False
    identities = record['identities']
    snapshots = record['snapshots']
    if (not isinstance(identities, list) or not identities or
            not isinstance(snapshots, list) or len(identities) != len(snapshots)):
        return False
    artifact_entries = record['artifact_tree'].get('entries', [])
    if not isinstance(artifact_entries, list):
        return False
    expected_files = [
        'benchmark_manifest.json', 'evidence/run_summary.json',
        'evidence_tree_manifest.json'] + [
            f'artifact/{item["relative_path"]}' for item in artifact_entries]
    if [item.get('relative_path') for item in identities] != expected_files:
        return False
    for identity, snapshot in zip(identities, snapshots):
        if (not _exact_keys(identity, {'relative_path', 'size_bytes', 'sha256'}) or
                not _exact_keys(snapshot, {
                    'source_relative_path', 'bundle_relative_path',
                    'size_bytes', 'sha256'}) or
                snapshot['source_relative_path'] != identity['relative_path'] or
                snapshot['size_bytes'] != identity['size_bytes'] or
                snapshot['sha256'] != identity['sha256']):
            return False
        relative = Path(snapshot['bundle_relative_path'])
        if (relative.is_absolute() or '..' in relative.parts or
                relative.parts[:2] != ('predecessors', name)):
            return False
        try:
            path = root / 'payload' / relative
            if (path.stat().st_size != identity['size_bytes'] or
                    sha256_file(path) != identity['sha256']):
                return False
        except (OSError, ValueError, TypeError):
            return False
    try:
        base = root / 'payload' / 'predecessors' / name
        benchmark = strict_json_load(base / 'benchmark_manifest.json')
        tree_manifest = strict_json_load(base / 'evidence_tree_manifest.json')
    except (OSError, TypeError, ValueError):
        return False
    if (not _exact_keys(tree_manifest, {'schema_version', 'artifact_tree'}) or
            tree_manifest['schema_version'] != 1 or
            tree_manifest['artifact_tree'] != record['artifact_tree'] or
            not (base / 'artifact').is_dir() or
            tree_inventory(base / 'artifact') != record['artifact_tree']):
        return False
    if name == 'G002':
        expected = {
            'schema_version': 1, 'objective': 'AMCL_FAULT_BENCHMARK',
            'final_status': 'PASS', 'axis_a_run_count': 15,
            'axis_b_run_count': 45, 'production_hashes_unchanged': True,
            'evidence_tree_sha256': record['artifact_tree']['tree_sha256']}
    else:
        expected = {
            'schema_version': 1,
            'objective': 'FRONTIER_POLICY_PAIRED_EVALUATION',
            'final_status': 'PASS', 'run_count': 15,
            'promotion_status': 'PASS',
            'production_hashes_unchanged': True,
            'evidence_tree_sha256': record['artifact_tree']['tree_sha256']}
    if not (benchmark == expected and
            record['verified_conclusion'] == {
                'status': 'PASS', 'schema': name,
                'manifest_sha256': sha256_file(
                    base / 'benchmark_manifest.json')}):
        return False
    try:
        if name == 'G002':
            from evaluate_amcl_axis_a import validate_axis_a_full
            from evaluate_amcl_axis_b import validate_manifest
            axis_a = validate_axis_a_full(base / 'artifact' / 'axis_a')
            axis_b = validate_manifest(
                base / 'artifact' / 'axis_b' / 'manifest.json', 'full')
            return (axis_a['mode'] == 'full' and axis_b['mode'] == 'full' and
                    len(axis_a['runs']) == 15 and len(axis_b['runs']) == 45)
        from evaluate_frontier_policy import validate_artifact
        frontier = validate_artifact(base / 'artifact', 'full')
        return (frontier['mode'] == 'full' and len(frontier['runs']) == 15 and
                frontier['promotion']['status'] == 'PASS')
    except (ImportError, KeyError, OSError, TypeError, ValueError):
        return False


def _predecessors_valid(root: Path, records: dict, complete: bool) -> bool:
    if not isinstance(records, dict) or set(records) != {
            'G002', 'G003', 'G004', 'G005', 'G008'}:
        return False
    for name in ('G002', 'G005'):
        if complete:
            if not _portable_predecessor_record_valid(root, name, records[name]):
                return False
        elif records[name] != {'status': 'NOT_EVALUATED',
                               'reason': 'FULL_RESULT_ABSENT'}:
            return False
    for name, (expected_root, expected_files, expected_status) in (
            PREDECESSORS.items()):
        record = records[name]
        if set(record) != {
                'canonical_root', 'status', 'identities', 'artifact_tree',
                'verified_conclusion', 'snapshots'}:
            return False
        canonical_root = record['canonical_root']
        if (not isinstance(canonical_root, str) or
                not Path(canonical_root).is_absolute() or
                Path(canonical_root).as_posix() != canonical_root or
                record['status'] != expected_status or
                [item.get('relative_path') for item in record['identities']] !=
                list(expected_files)):
            return False
        snapshots = record['snapshots']
        if (not isinstance(snapshots, list) or len(snapshots) !=
                len(expected_files)):
            return False
        for identity, snapshot in zip(record['identities'], snapshots):
            if set(identity) != {'relative_path', 'size_bytes', 'sha256'}:
                return False
            if set(snapshot) != {
                    'source_relative_path', 'bundle_relative_path',
                    'size_bytes', 'sha256'}:
                return False
            expected_bundle = f'predecessors/{name}/{identity["relative_path"]}'
            if (snapshot['source_relative_path'] != identity['relative_path'] or
                    snapshot['bundle_relative_path'] != expected_bundle or
                    snapshot['size_bytes'] != identity['size_bytes'] or
                    snapshot['sha256'] != identity['sha256']):
                return False
            path = root / 'payload' / expected_bundle
            try:
                if (identity['size_bytes'] != path.stat().st_size or
                        identity['sha256'] != sha256_file(path)):
                    return False
            except (OSError, ValueError):
                return False
        if name in ('G003', 'G004'):
            try:
                if not _sealed_tree_valid(record['artifact_tree']):
                    return False
                aggregate = strict_json_load(
                    root / 'payload' / f'predecessors/{name}/aggregate.json')
                expected_count = 25 if name == 'G003' else 15
                if record['verified_conclusion'] != {
                        'status': 'PASS', 'run_count': expected_count} or (
                        aggregate.get('status'), aggregate.get('run_count')) != (
                            'PASS', expected_count):
                    return False
            except (OSError, ValueError):
                return False
        else:
            try:
                storage = strict_json_load(
                    root / 'payload/predecessors/G008/storage_manifest.json')
                tree = strict_json_load(
                    root / 'payload/predecessors/G008/'
                    'evidence_tree_manifest.json')
                final = storage['final_summary']
                canonical = storage['canonical_tree']
                expected_tree = {
                    'algorithm': tree['algorithm']['name'],
                    'file_count': canonical['file_count'],
                    'total_bytes': canonical['total_bytes'],
                    'tree_sha256': canonical['tree_sha256'], 'entries': [],
                    'manifest_sha256': canonical['manifest_sha256'],
                    'final_status': final['final_status'],
                    'selected_subdivision': final['selected_subdivision']}
                if (record['artifact_tree'] != expected_tree or
                        final['final_status'] != 'RETAIN_PRODUCTION' or
                        final['selected_subdivision'] != 1 or
                        canonical['manifest_sha256'] !=
                        sha256_file(root / 'payload/predecessors/G008/'
                                    'evidence_tree_manifest.json') or
                        canonical['tree_sha256'] != tree['tree_sha256']):
                    return False
                if record['verified_conclusion'] != {
                        'final_status': 'RETAIN_PRODUCTION',
                        'selected_subdivision': 1}:
                    return False
            except (OSError, ValueError, KeyError, TypeError):
                return False
    return True


def _resolved_profile_valid(root: Path, profile: str, records: list) -> bool:
    """Derive the generated profile solely from copied parameter bytes."""
    generated = records[-1]
    try:
        actual = strict_json_load(root / 'payload' / generated['relative_path'])
        nav_record = next(item for item in records if item[
            'source_relative_path'] ==
            'jdamr_cube_navigation/config/nav2_params.yaml')
        nav_path = root / 'payload' / nav_record['relative_path']
        parameters = yaml.safe_load(nav_path.read_text(encoding='utf-8'))
    except (OSError, ValueError, KeyError, TypeError, StopIteration,
            yaml.YAMLError):
        return False
    common = {'use_sim_time': False, 'autostart': True,
              'hardware_substitution': True}
    if profile == 'A':
        expected = {
            **common, 'use_composition': False,
            'num_subdivisions_per_laser_scan': 1,
            'default_nav_to_pose_bt_xml':
                'navigate_to_pose_safe_mapping.xml',
            'nav2_parameter_source_sha256': nav_record['sha256']}
    else:
        expected = {
            **common, 'record_bag': True,
            'container_executable': 'component_container_isolated',
            'default_nav_to_pose_bt_xml':
                'navigate_to_pose_corridor_fail_fast.xml',
            'amcl_runtime': 'stock_production_binary_only',
            'stock_amcl_applicable_parameters': parameters['amcl'][
                'ros__parameters'],
            'keepout_enabled': True}
    return actual == expected


def _pending_readiness_valid(readiness: dict) -> bool:
    expected = {
        'status': 'PENDING',
        'blockers': [
            'G002_FULL_NOT_EVALUATED', 'G005_FULL_NOT_EVALUATED',
            'SCENARIO_ATTEMPTS_NOT_EXECUTED',
            'RUNTIME_EQUIVALENCE_NOT_EVALUATED',
            'GOLDEN_VECTORS_NOT_EVALUATED',
            'RUNTIME_BINARY_IDENTITY_NOT_EVALUATED',
            'SEALED_ROS2_MCAP_RUNTIME_GRAPH_VALIDATION_PENDING']}
    return readiness == expected


def _open_evidence(root: Path, identity: dict):
    """Open one bundle-local sealed JSON evidence record."""
    if not evidence_identity_shape_valid(identity):
        raise ValueError('invalid evidence identity')
    relative = Path(identity['bundle_relative_path'])
    if relative.parts[:1] != ('evidence',):
        raise ValueError('evidence must live below payload/evidence')
    path = root / 'payload' / relative
    if (path.is_symlink() or not path.is_file() or
            path.stat().st_size != identity['size_bytes'] or
            sha256_file(path) != identity['sha256']):
        raise ValueError('evidence identity mismatch')
    return strict_json_load(path)


def _finite_number(value, minimum=0.0) -> bool:
    return (not isinstance(value, bool) and isinstance(value, (int, float)) and
            math.isfinite(value) and value >= minimum)


def _raw_attempt_trace_valid(evidence: dict, profile: str,
                             profile_records: list) -> bool:
    """Recompute attempt summaries from the sealed raw time series."""
    raw = evidence.get('raw_trace')
    if not _exact_keys(raw, {
            'lifecycle', 'resource_samples', 'publishers', 'processes',
            'production_files', 'input_files', 'launch_graph',
            'motion_samples', 'events', 'contacts'}):
        return False
    expected_inputs = [{
        'path': row['relative_path'], 'size_bytes': row['bundle_size_bytes'],
        'sha256': row['bundle_sha256']} for row in profile_records]
    expected_production = [{
        'path': row['source_relative_path'],
        'before_sha256': row['sha256'], 'after_sha256': row['sha256']}
        for row in profile_records if row['source_root'] != 'generated']
    expected_graph = [dict(zip(RUNTIME_ENTITY_FIELDS, row))
                      for row in RUNTIME_ENTITIES if row[0] == profile]
    if (raw['input_files'] != expected_inputs or
            raw['production_files'] != expected_production or
            raw['launch_graph'] != expected_graph):
        return False
    expected_nodes = (['cartographer_node', 'collision_monitor'] if
                      profile == 'A' else ['map_server', 'amcl',
                                           'collision_monitor'])
    lifecycle = raw['lifecycle']
    if (not isinstance(lifecycle, list) or
            [row.get('node') for row in lifecycle] != expected_nodes or
            any(not _exact_keys(row, {'node', 'state', 'steady_ns'}) or
                row['state'] != 'active' or
                not _finite_number(row['steady_ns'], 1) for row in lifecycle)):
        return False
    samples = raw['resource_samples']
    if (not isinstance(samples, list) or len(samples) < 2 or
            any(not _exact_keys(row, {
                'steady_ns', 'scan_count', 'cpu_seconds', 'rss_bytes',
                'callback_gap_s', 'tf_gap_s'}) for row in samples)):
        return False
    for row in samples:
        if (not _finite_number(row['steady_ns'], 1) or
                isinstance(row['scan_count'], bool) or
                not isinstance(row['scan_count'], int) or
                row['scan_count'] < 0 or
                not _finite_number(row['cpu_seconds']) or
                isinstance(row['rss_bytes'], bool) or
                not isinstance(row['rss_bytes'], int) or row['rss_bytes'] < 0 or
                not _finite_number(row['callback_gap_s']) or
                not _finite_number(row['tf_gap_s'])):
            return False
    first, last = samples[0], samples[-1]
    scan_delta = last['scan_count'] - first['scan_count']
    if (scan_delta <= 0 or last['cpu_seconds'] < first['cpu_seconds'] or
            any(left['steady_ns'] >= right['steady_ns']
                for left, right in zip(samples, samples[1:]))):
        return False
    derived_resources = {
        'cpu_seconds_per_1000_scans':
            (last['cpu_seconds'] - first['cpu_seconds']) * 1000 / scan_delta,
        'max_rss_bytes': max(row['rss_bytes'] for row in samples),
        'callback_gap_p95_s': max(row['callback_gap_s'] for row in samples),
        'tf_gap_p95_s': max(row['tf_gap_s'] for row in samples)}
    if evidence['common']['resource_metrics'] != derived_resources:
        return False
    publishers = raw['publishers']
    if publishers != evidence['common']['authorities']:
        return False
    processes = raw['processes']
    if (not isinstance(processes, list) or not processes or
            any(not _exact_keys(row, {'name', 'pid', 'alive_after_cleanup'}) or
                not isinstance(row['name'], str) or not row['name'] or
                isinstance(row['pid'], bool) or not isinstance(row['pid'], int) or
                row['pid'] <= 0 or row['alive_after_cleanup'] is not False
                for row in processes)):
        return False
    production = raw['production_files']
    if (not isinstance(production, list) or not production or
            any(not _exact_keys(row, {'path', 'before_sha256', 'after_sha256'}) or
                not isinstance(row['path'], str) or not row['path'] or
                row['before_sha256'] != row['after_sha256'] or
                not isinstance(row['before_sha256'], str) or
                len(row['before_sha256']) != 64 for row in production)):
        return False
    motion = raw['motion_samples']
    if (not isinstance(motion, list) or len(motion) < 2 or
            any(not _exact_keys(row, {'steady_ns', 'linear_x', 'angular_z'}) or
                not all(_finite_number(abs(row[key])) for key in (
                    'linear_x', 'angular_z')) or
                not _finite_number(row['steady_ns'], 1) for row in motion) or
            (motion[-1]['linear_x'], motion[-1]['angular_z']) != (0.0, 0.0)):
        return False
    events = raw['events']
    if (not isinstance(events, list) or
            [row.get('event') for row in events] != [
                'input_ready', 'motion_started', 'final_zero_hold'] or
            any(not _exact_keys(row, {'event', 'steady_ns'}) or
                not _finite_number(row['steady_ns'], 1) for row in events) or
            events != evidence['motion']['causal_trace']):
        return False
    contacts = raw['contacts']
    return (isinstance(contacts, list) and
            evidence['motion']['contact_count'] == len(contacts) == 0 and
            evidence['motion']['final_zero_hold'] is True and
            evidence['common']['survivor_count'] == 0 and
            evidence['common']['production_hashes_unchanged'] is True)


def _resources_valid(resources: dict) -> bool:
    return (_exact_keys(resources, {
        'cpu_seconds_per_1000_scans', 'max_rss_bytes',
        'callback_gap_p95_s', 'tf_gap_p95_s'}) and
        _finite_number(resources['cpu_seconds_per_1000_scans']) and
        isinstance(resources['max_rss_bytes'], int) and
        not isinstance(resources['max_rss_bytes'], bool) and
        resources['max_rss_bytes'] >= 0 and
        _finite_number(resources['callback_gap_p95_s']) and
        _finite_number(resources['tf_gap_p95_s']))


def _authorities_valid(authorities: list, profile: str) -> bool:
    expected = ([('map_to_odom', 'cartographer_node'),
                 ('final_cmd_vel', 'collision_monitor')] if profile == 'A'
                else [('map_to_odom', 'amcl'),
                      ('final_cmd_vel', 'collision_monitor')])
    if not isinstance(authorities, list) or len(authorities) != 2:
        return False
    for record, (role, node) in zip(authorities, expected):
        if (record != {
                'role': role, 'publisher_node': node,
                'publisher_gid': record.get('publisher_gid'),
                'publisher_count': 1} or
                isinstance(record.get('publisher_count'), bool) or
                not isinstance(record.get('publisher_gid'), str) or
                not record['publisher_gid']):
            return False
    return True


def _payload_file_valid(root: Path, identity: dict) -> bool:
    if not evidence_identity_shape_valid(identity):
        return False
    relative = Path(identity['bundle_relative_path'])
    try:
        path = root / 'payload' / relative
        return (relative.parts[:1] in (('profiles',), ('evidence',)) and
                not path.is_symlink() and path.is_file() and
                path.stat().st_size == identity['size_bytes'] and
                sha256_file(path) == identity['sha256'])
    except (OSError, ValueError, TypeError):
        return False


def _map_files_valid(root: Path, yaml_identity: dict,
                     pgm_identity: dict) -> bool:
    if (not _payload_file_valid(root, yaml_identity) or
            not _payload_file_valid(root, pgm_identity)):
        return False
    try:
        yaml_path = root / 'payload' / yaml_identity['bundle_relative_path']
        pgm_path = root / 'payload' / pgm_identity['bundle_relative_path']
        config = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
        raw = pgm_path.read_bytes()
        tokens = raw.split(maxsplit=4)
        width = int(tokens[1])
        height = int(tokens[2])
        maximum = int(tokens[3])
        return (isinstance(config, dict) and
                config.get('image') == pgm_path.name and
                tokens[0] in (b'P2', b'P5') and width >= 10 and height >= 10 and
                maximum == 255 and len(raw) >= width * height)
    except (OSError, TypeError, UnicodeError, yaml.YAMLError):
        return False


def _pbstream_valid(root: Path, identity: dict) -> bool:
    """Reject placeholder text while retaining a portable byte-level gate."""
    if not _payload_file_valid(root, identity):
        return False
    try:
        raw = (root / 'payload' / identity['bundle_relative_path']).read_bytes()
    except OSError:
        return False
    return (len(raw) >= 1024 and len(set(raw)) >= 32 and
            not raw.lower().startswith(b'pbstream'))


def _rosbag_records(root: Path, bag: dict, required_topics: dict):
    """Read a sealed MCAP and return independently deserialized messages."""
    if not _exact_keys(bag, {
            'directory', 'metadata', 'mcap', 'topic_types'}):
        raise ValueError('bag schema')
    directory = Path(bag['directory'])
    if (directory.is_absolute() or '..' in directory.parts or
            directory.parts[:2] != ('evidence', 'bags') or
            bag['topic_types'] != required_topics or
            not _payload_file_valid(root, bag['metadata']) or
            not _payload_file_valid(root, bag['mcap'])):
        raise ValueError('bag identity')
    bag_root = root / 'payload' / directory
    metadata_path = root / 'payload' / bag['metadata']['bundle_relative_path']
    mcap_path = root / 'payload' / bag['mcap']['bundle_relative_path']
    if (bag_root.is_symlink() or not bag_root.is_dir() or
            metadata_path.parent != bag_root or mcap_path.parent != bag_root or
            mcap_path.suffix != '.mcap' or
            {path.name for path in bag_root.iterdir()} != {
                metadata_path.name, mcap_path.name} or
            any(path.is_symlink() or not path.is_file()
                for path in bag_root.iterdir())):
        raise ValueError('bag path')
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    info = metadata.get('rosbag2_bagfile_information', {})
    if (info.get('storage_identifier') != 'mcap' or
            info.get('relative_file_paths') != [mcap_path.name]):
        raise ValueError('metadata')
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(bag_root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    actual_types = {row.name: row.type
                    for row in reader.get_all_topics_and_types()}
    if actual_types != required_topics:
        raise ValueError('topic inventory')
    message_types = {topic: get_message(type_name)
                     for topic, type_name in required_topics.items()}
    records = {topic: [] for topic in required_topics}
    last_storage_ns = -1
    while reader.has_next():
        topic, serialized, storage_ns = reader.read_next()
        if storage_ns < last_storage_ns or topic not in records:
            raise ValueError('bag order or unexpected topic')
        last_storage_ns = storage_ns
        records[topic].append((storage_ns, deserialize_message(
            serialized, message_types[topic])))
    if any(not rows for rows in records.values()):
        raise ValueError('empty required topic')
    return records


def _scenario_events(scenario: str) -> list[str]:
    sequences = {
        'mapping_nominal_closed_loop': [
            'input_ready', 'motion_started', 'mapping_saved',
            'final_zero_hold'],
        'mapping_bounded_frontier': [
            'input_ready', 'motion_started', 'frontier_goal_1',
            'frontier_goal_2', 'frontier_goal_3', 'mapping_saved',
            'final_zero_hold'],
        'saved_fixed_obstacle_detour': [
            'input_ready', 'motion_started', 'goal_succeeded',
            'final_zero_hold'],
        'saved_sudden_obstacle_stop_resume': [
            'input_ready', 'motion_started', 'obstacle_activated', 'stop',
            'zero', 'clear', 'resume', 'goal_succeeded', 'final_zero_hold'],
        'saved_scan_timeout_stop_resume': [
            'input_ready', 'motion_started', 'scan_frozen',
            'invalid_source_stop', 'zero', 'scan_unfrozen', 'resume',
            'goal_succeeded', 'final_zero_hold'],
        'saved_amcl_offset_recovery_then_nav': [
            'input_ready', 'initial_offset', 'relocalized',
            'motion_started', 'goal_succeeded', 'final_zero_hold']}
    return sequences[scenario]


def _attempt_rosbag_valid(root: Path, evidence: dict, scenario: str) -> bool:
    required = {
        '/g006/events': 'std_msgs/msg/String',
        '/g006/contact_count': 'std_msgs/msg/UInt32',
        '/cmd_vel': 'geometry_msgs/msg/Twist',
        '/g006/trajectory': 'geometry_msgs/msg/PoseStamped',
        '/g006/metrics': 'std_msgs/msg/String'}
    try:
        records = _rosbag_records(root, evidence['rosbag'], required)
        events = [strict_json_text(message.data) for _, message in
                  records['/g006/events']]
        names = [row['event'] for row in events]
        event_times = [row['steady_ns'] for row in events]
        metrics = strict_json_text(records['/g006/metrics'][-1][1].data)
        trajectory = records['/g006/trajectory']
        final_twist = records['/cmd_vel'][-1][1]
    except (AttributeError, KeyError, OSError, RuntimeError, TypeError,
            ValueError, yaml.YAMLError):
        return False
    return (
        names == _scenario_events(scenario) and
        all(not isinstance(value, bool) and isinstance(value, int) and
            value > 0 for value in event_times) and
        all(left < right for left, right in zip(event_times, event_times[1:])) and
        all(row[1].data == 0 for row in records['/g006/contact_count']) and
        final_twist.linear.x == 0.0 and final_twist.angular.z == 0.0 and
        len(trajectory) >= 2 and metrics == {
            'resource_metrics': evidence['common']['resource_metrics'],
            'status': 'PASS'})


def strict_json_text(text: str):
    """Decode duplicate-free, finite JSON carried inside a ROS String."""
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate key')
            result[key] = value
        return result

    import json
    return json.loads(text, object_pairs_hook=pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(
                          ValueError(value)))


def _attempt_evidence_valid(root: Path, bundle_id: str, scenario: str,
                            seed: int, identity: dict,
                            profile_records: list) -> bool:
    try:
        evidence = _open_evidence(root, identity)
    except (OSError, ValueError, TypeError):
        return False
    keys = {'schema_version', 'bundle_id', 'scenario', 'seed', 'status',
            'profile', 'common', 'motion', 'mapping', 'saved_nav',
            'raw_trace', 'rosbag'}
    profile = 'A' if scenario.startswith('mapping_') else 'B'
    if (not _exact_keys(evidence, keys) or evidence['schema_version'] != 1 or
            evidence['bundle_id'] != bundle_id or
            evidence['scenario'] != scenario or evidence['seed'] != seed or
            evidence['status'] != 'PASS' or evidence['profile'] != profile):
        return False
    common = evidence['common']
    if (not _exact_keys(common, {
            'input_hash_parity', 'expected_launch_graph', 'lifecycle_active',
            'resource_metrics', 'authorities', 'survivor_count',
            'production_hashes_unchanged'}) or
            common['input_hash_parity'] is not True or
            common['expected_launch_graph'] is not True or
            common['lifecycle_active'] is not True or
            common['production_hashes_unchanged'] is not True or
            isinstance(common['survivor_count'], bool) or
            common['survivor_count'] != 0 or
            not _resources_valid(common['resource_metrics']) or
            not _authorities_valid(common['authorities'], profile) or
            not _raw_attempt_trace_valid(
                evidence, profile, profile_records) or
            not _attempt_rosbag_valid(root, evidence, scenario)):
        return False
    motion = evidence['motion']
    if not _exact_keys(motion, {
            'status', 'contact_count', 'final_zero_hold', 'causal_trace'}):
        return False
    trace = motion['causal_trace']
    if (motion['status'] != 'PASS' or
            isinstance(motion['contact_count'], bool) or
            motion['contact_count'] != 0 or
            motion['final_zero_hold'] is not True or
            not isinstance(trace, list) or
            [item.get('event') for item in trace] != [
                'input_ready', 'motion_started', 'final_zero_hold'] or
            any(not _exact_keys(item, {'event', 'steady_ns'}) or
                isinstance(item['steady_ns'], bool) or
                not isinstance(item['steady_ns'], int) or
                item['steady_ns'] <= 0 for item in trace) or
            not trace[0]['steady_ns'] < trace[1]['steady_ns'] < trace[2][
                'steady_ns']):
        return False
    if profile == 'A':
        mapping = evidence['mapping']
        if (not _exact_keys(mapping, {
                'status', 'pbstream', 'map_yaml', 'map_pgm', 'reload_status',
                'cartographer_metrics', 'start_end_closure_m',
                'raw_trajectory_poses', 'raw_metric_samples'}) or
                mapping['status'] != 'PASS' or
                not _pbstream_valid(root, mapping['pbstream']) or
                not _map_files_valid(
                    root, mapping['map_yaml'], mapping['map_pgm']) or
                mapping['reload_status'] != 'PASS' or
                validate_cartographer_metrics(
                    mapping['cartographer_metrics']) != 'PASS' or
                not _finite_number(mapping['start_end_closure_m']) or
                evidence['saved_nav'] != 'NOT_APPLICABLE'):
            return False
        poses = mapping['raw_trajectory_poses']
        metrics = mapping['raw_metric_samples']
        if (not isinstance(poses, list) or len(poses) < 2 or
                any(not isinstance(row, list) or len(row) != 3 or
                    any(not _finite_number(abs(value)) for value in row)
                    for row in poses) or
                not isinstance(metrics, list) or len(metrics) < 2 or
                any(validate_cartographer_metrics(row) != 'PASS'
                    for row in metrics) or
                metrics[-1] != mapping['cartographer_metrics'] or
                not math.isclose(mapping['start_end_closure_m'], math.hypot(
                    poses[-1][0] - poses[0][0],
                    poses[-1][1] - poses[0][1]), rel_tol=0.0,
                    abs_tol=1e-12)):
            return False
    else:
        saved = evidence['saved_nav']
        if (not _exact_keys(saved, {
                'status', 'map_yaml', 'map_pgm', 'mask_yaml', 'mask_pgm',
                'load_status', 'amcl_active', 'behavior_tree_match',
                'collision_monitor_active', 'raw_node_states'}) or
                saved['status'] != 'PASS' or
                not _map_files_valid(root, saved['map_yaml'], saved['map_pgm']) or
                not _map_files_valid(root, saved['mask_yaml'], saved['mask_pgm']) or
                saved['load_status'] != 'PASS' or
                saved['amcl_active'] is not True or
                saved['behavior_tree_match'] is not True or
                saved['collision_monitor_active'] is not True or
                evidence['mapping'] != 'NOT_APPLICABLE'):
            return False
        if saved['raw_node_states'] != [
                {'node': 'map_server', 'state': 'active'},
                {'node': 'keepout_filter_mask_server', 'state': 'active'},
                {'node': 'amcl', 'state': 'active'},
                {'node': 'bt_navigator', 'state': 'active'},
                {'node': 'collision_monitor', 'state': 'active'}]:
            return False
    return True


def _infra_evidence_valid(root: Path, bundle_id: str, scenario: str,
                          seed: int, identity: dict) -> bool:
    try:
        evidence = _open_evidence(root, identity)
    except (OSError, ValueError, TypeError):
        return False
    return evidence == {
        'schema_version': 1, 'bundle_id': bundle_id,
        'scenario': scenario, 'seed': seed, 'status': 'INFRA_RETRY',
        'classification': 'INFRASTRUCTURE_ONLY',
        'semantic_failure': False, 'survivor_count': 0}


def _runtime_binary_snapshots_valid(root: Path, bundle_id: str,
                                    readiness: dict) -> bool:
    """Validate exhaustive runtime identity evidence from its sealed file."""
    try:
        evidence = _open_evidence(
            root, readiness['runtime_binary_identity']['evidence'])
        if (not _exact_keys(evidence, {
                'schema_version', 'bundle_id', 'status', 'entities',
                'static_probe_used_as_runtime_equivalence', 'graph',
                'endpoint_snapshot'}) or
                evidence['schema_version'] != 1 or
                evidence['bundle_id'] != bundle_id or
                evidence['status'] != 'PASS' or
                evidence['static_probe_used_as_runtime_equivalence'] is not
                False):
            return False
        records = evidence['entities']
        if not isinstance(records, list) or len(records) != len(RUNTIME_ENTITIES):
            return False
        extra = {'absolute_path', 'mode_octal', 'size_bytes', 'sha256',
                 'binary_snapshot', 'pid', 'proc_maps', 'publisher_gids',
                 'proc_captures'}
        graph = evidence['graph']
        expected_graph = []
        for record, expected in zip(records, RUNTIME_ENTITIES):
            snapshot_path = root / 'payload' / record[
                'binary_snapshot']['bundle_relative_path']
            snapshot_mode = f'{snapshot_path.stat().st_mode & 0o777:04o}'
            if (not _exact_keys(record, set(RUNTIME_ENTITY_FIELDS) | extra) or
                    tuple(record[field] for field in
                          RUNTIME_ENTITY_FIELDS) != expected or
                    not Path(record['absolute_path']).is_absolute() or
                    not isinstance(record['mode_octal'], str) or
                    len(record['mode_octal']) != 4 or
                    any(value not in '01234567'
                        for value in record['mode_octal']) or
                    isinstance(record['size_bytes'], bool) or
                    not isinstance(record['size_bytes'], int) or
                    record['size_bytes'] <= 0 or
                    not isinstance(record['sha256'], str) or
                    len(record['sha256']) != 64 or
                    any(value not in '0123456789abcdef'
                        for value in record['sha256']) or
                    not _payload_file_valid(root, record['binary_snapshot']) or
                    record['binary_snapshot']['size_bytes'] !=
                    record['size_bytes'] or
                    record['binary_snapshot']['sha256'] != record['sha256'] or
                    snapshot_mode != record['mode_octal'] or
                    not _elf_snapshot_valid(snapshot_path) or
                    isinstance(record['pid'], bool) or
                    not isinstance(record['pid'], int) or record['pid'] <= 0 or
                    record['proc_maps'] != [{
                        'pid': record['pid'],
                        'absolute_path': record['absolute_path'],
                        'size_bytes': record['size_bytes'],
                        'sha256': record['sha256']}] or
                    not _proc_captures_valid(root, record) or
                    not isinstance(record['publisher_gids'], list)):
                return False
            expected_graph.append({
                field: record[field] for field in RUNTIME_ENTITY_FIELDS} | {
                    'pid': record['pid'],
                    'publisher_gids': record['publisher_gids']})
            if ((record['authority_role'] == 'none' and
                 record['publisher_gids']) or
                    (record['authority_role'] != 'none' and
                     (len(record['publisher_gids']) != 1 or
                      not isinstance(record['publisher_gids'][0], str) or
                      not record['publisher_gids'][0]))):
                return False
        container_pid = next(row['pid'] for row in records if
                             row['profile'] == 'B' and
                             row['process_class'] == 'component_container')
        independent = [row['pid'] for row in records if
                       row['process_class'] != 'composed_component']
        if (graph != expected_graph or len(set(independent)) != len(independent) or
                any(row['pid'] != container_pid for row in records if
                    row['process_class'] == 'composed_component')):
            return False
        endpoints = _open_evidence(root, evidence['endpoint_snapshot'])
        if endpoints != {
                'schema_version': 1, 'bundle_id': bundle_id,
                'publishers': [row for row in expected_graph
                               if row['publisher_gids']]}:
            return False
    except (KeyError, OSError, TypeError, ValueError):
        return False
    return True


def _elf_snapshot_valid(path: Path) -> bool:
    """Validate copied ELF identity beyond a short arbitrary byte blob."""
    try:
        raw = path.read_bytes()
    except OSError:
        return False
    if (len(raw) < 64 or raw[:4] != b'\x7fELF' or raw[4] not in (1, 2) or
            raw[5] not in (1, 2)):
        return False
    endian = 'little' if raw[5] == 1 else 'big'
    machine = int.from_bytes(raw[18:20], endian)
    return machine > 0 and b'GNU\x00' in raw


def _proc_captures_valid(root: Path, record: dict) -> bool:
    """Cross-bind copied raw proc captures to one runtime entity."""
    captures = record.get('proc_captures')
    if not _exact_keys(captures, {'exe', 'maps', 'stat', 'cmdline'}):
        return False
    if any(not _payload_file_valid(root, identity)
           for identity in captures.values()):
        return False
    try:
        paths = {name: root / 'payload' / identity['bundle_relative_path']
                 for name, identity in captures.items()}
        exe = paths['exe'].read_text(encoding='utf-8').rstrip('\n')
        maps = paths['maps'].read_text(encoding='utf-8')
        stat = paths['stat'].read_text(encoding='utf-8')
        cmdline = paths['cmdline'].read_bytes()
    except (OSError, UnicodeError):
        return False
    return (exe == record['absolute_path'] and
            record['absolute_path'] in maps and
            stat.startswith(f'{record["pid"]} (') and
            stat.count(' ') >= 20 and bool(cmdline.strip(b'\0')))


def _external_evidence_valid(root: Path, bundle_id: str,
                             references: dict) -> bool:
    """Validate no-motion and isolated real-bag evidence independently."""
    try:
        no_motion = _open_evidence(root, references['no_motion_exact_launch'])
        real_bag = _open_evidence(root, references['real_bag_isolated_replay'])
    except (KeyError, OSError, TypeError, ValueError):
        return False
    no_motion_bag = no_motion.get('rosbag') if isinstance(no_motion, dict) else None
    no_motion_without_bag = dict(no_motion) if isinstance(no_motion, dict) else {}
    no_motion_without_bag.pop('rosbag', None)
    if no_motion_without_bag != {
            'schema_version': 1, 'bundle_id': bundle_id, 'status': 'PASS',
            'claim_scope': 'NO_MOTION_EXACT_LAUNCH_HARDWARE_STUBS',
            'profiles_exact_launch': ['A', 'B'], 'lifecycle_active': True,
            'motion_publishers_started': 0, 'authority_checks': 'PASS',
            'survivor_count': 0, 'production_hashes_unchanged': True,
            'raw_trace': {
                'profile_launch_events': [
                    {'profile': 'A', 'event': 'started'},
                    {'profile': 'A', 'event': 'lifecycle_active'},
                    {'profile': 'A', 'event': 'stopped'},
                    {'profile': 'B', 'event': 'started'},
                    {'profile': 'B', 'event': 'lifecycle_active'},
                    {'profile': 'B', 'event': 'stopped'}],
                'motion_publisher_gids': [],
                'authority_publishers': [
                    {'profile': 'A', 'role': 'map_to_odom',
                     'node': 'cartographer_node', 'gid': 'A-map'},
                    {'profile': 'A', 'role': 'final_cmd_vel',
                     'node': 'collision_monitor', 'gid': 'A-cmd'},
                    {'profile': 'B', 'role': 'map_to_odom',
                     'node': 'amcl', 'gid': 'B-map'},
                    {'profile': 'B', 'role': 'final_cmd_vel',
                     'node': 'collision_monitor', 'gid': 'B-cmd'}],
                'surviving_pids': []}}:
        return False
    try:
        no_motion_records = _rosbag_records(root, no_motion_bag, {
            '/g006/no_motion_events': 'std_msgs/msg/String'})
        no_motion_events = [strict_json_text(message.data)['event']
                            for _, message in
                            no_motion_records['/g006/no_motion_events']]
    except (AttributeError, KeyError, OSError, RuntimeError, TypeError,
            ValueError, yaml.YAMLError):
        return False
    if no_motion_events != [
            'profile_a_started', 'profile_a_stopped',
            'profile_b_started', 'profile_b_stopped']:
        return False
    if (isinstance(no_motion['motion_publishers_started'], bool) or
            isinstance(no_motion['survivor_count'], bool)):
        return False
    if not _exact_keys(real_bag, {
            'schema_version', 'bundle_id', 'status', 'claim_scope',
            'input_hash_parity', 'recorded_clock_absent',
            'generated_amcl_pose_only', 'resource_metrics',
            'authority_checks', 'survivor_count',
            'production_hashes_unchanged', 'raw_trace', 'rosbag'}):
        return False
    raw = real_bag['raw_trace']
    try:
        replay_records = _rosbag_records(root, real_bag['rosbag'], {
            '/scan': 'sensor_msgs/msg/LaserScan',
            '/odom': 'nav_msgs/msg/Odometry',
            '/tf': 'tf2_msgs/msg/TFMessage',
            '/tf_static': 'tf2_msgs/msg/TFMessage',
            '/amcl_pose': 'geometry_msgs/msg/PoseWithCovarianceStamped'})
    except (OSError, RuntimeError, TypeError, ValueError, yaml.YAMLError):
        return False
    if len(replay_records['/amcl_pose']) != raw.get(
            'generated_amcl_pose_count'):
        return False
    if (not _exact_keys(raw, {
            'topics', 'recorded_clock_count', 'recorded_amcl_pose_count',
            'generated_amcl_pose_count', 'surviving_pids',
            'resource_samples', 'authority_publishers'}) or
            raw['recorded_clock_count'] != 0 or
            raw['recorded_amcl_pose_count'] != 0 or
            isinstance(raw['generated_amcl_pose_count'], bool) or
            not isinstance(raw['generated_amcl_pose_count'], int) or
            raw['generated_amcl_pose_count'] <= 0 or
            raw['surviving_pids'] != [] or
            raw['authority_publishers'] != [{
                'role': 'map_to_odom', 'node': 'amcl',
                'gid': 'real-bag-amcl'}] or
            raw['topics'] != ['/odom', '/scan', '/tf', '/tf_static'] or
            not isinstance(raw['resource_samples'], list) or
            len(raw['resource_samples']) < 2):
        return False
    samples = raw['resource_samples']
    if (any(not _exact_keys(row, {
            'steady_ns', 'scan_count', 'cpu_seconds', 'rss_bytes',
            'callback_gap_s', 'tf_gap_s'}) for row in samples) or
            any(not _finite_number(row['steady_ns'], 1) or
                isinstance(row['scan_count'], bool) or
                not isinstance(row['scan_count'], int) or
                not _finite_number(row['cpu_seconds']) or
                isinstance(row['rss_bytes'], bool) or
                not isinstance(row['rss_bytes'], int) or
                not _finite_number(row['callback_gap_s']) or
                not _finite_number(row['tf_gap_s']) for row in samples)):
        return False
    first, last = samples[0], samples[-1]
    scan_delta = last['scan_count'] - first['scan_count']
    if scan_delta <= 0:
        return False
    if real_bag['resource_metrics'] != {
            'cpu_seconds_per_1000_scans':
            (last['cpu_seconds'] - first['cpu_seconds']) * 1000 / scan_delta,
            'max_rss_bytes': max(row['rss_bytes'] for row in samples),
            'callback_gap_p95_s': max(
                row['callback_gap_s'] for row in samples),
            'tf_gap_p95_s': max(row['tf_gap_s'] for row in samples)}:
        return False
    return (real_bag['schema_version'] == 1 and
            real_bag['bundle_id'] == bundle_id and
            real_bag['status'] == 'PASS' and
            real_bag['claim_scope'] ==
            'ISOLATED_REAL_BAG_REPLAY_NO_GT_NO_MOTION_COMMAND' and
            real_bag['input_hash_parity'] is True and
            real_bag['recorded_clock_absent'] is True and
            real_bag['generated_amcl_pose_only'] is True and
            _resources_valid(real_bag['resource_metrics']) and
            real_bag['authority_checks'] == 'PASS' and
            not isinstance(real_bag['survivor_count'], bool) and
            real_bag['survivor_count'] == 0 and
            real_bag['production_hashes_unchanged'] is True)


def _recorded_runtime_valid(root: Path, runtime: dict, profiles: dict) -> bool:
    """Validate recorded host metadata against copied package manifests."""
    keys = {'ros_distro', 'python_version', 'os_system', 'os_release',
            'machine', 'packages'}
    if not _exact_keys(runtime, keys) or runtime['ros_distro'] != 'jazzy':
        return False
    if any(not isinstance(runtime[name], str) or not runtime[name]
           for name in ('python_version', 'os_system', 'os_release', 'machine')):
        return False
    package_records = {}
    for records in profiles.values():
        for record in records:
            if record['source_relative_path'].endswith('package.xml'):
                path = root / 'payload' / record['relative_path']
                try:
                    package = ET.parse(path).getroot()
                    name = package.findtext('name')
                    version = package.findtext('version')
                except (OSError, ET.ParseError):
                    return False
                if not name or not version:
                    return False
                package_records[name] = {'name': name, 'version': version}
    return runtime['packages'] == [package_records[name] for name in (
        'jdamr_cube_bringup', 'jdamr_cube_cartographer',
        'jdamr_cube_navigation', 'jdamr_cube_description', 'nav2_bringup')]


def _validate_bundle(root: Path) -> dict:
    """Recompute all durable identities and return a compact verdict."""
    root = root.absolute()
    failures = []
    if root.is_symlink() or not root.is_dir():
        return {'status': 'FAIL', 'failures': ['invalid_root']}
    if {path.name for path in root.iterdir()} != {
            'bundle_manifest.json', 'payload'}:
        failures.append('root_exact_set')
    try:
        manifest = strict_json_load(root / 'bundle_manifest.json')
    except (OSError, ValueError, TypeError):
        return {'status': 'FAIL', 'failures': ['strict_manifest']}
    if not _exact_keys(manifest, TOP_KEYS):
        failures.append('top_schema')
        return {'status': 'FAIL', 'failures': failures}
    if (manifest['schema_version'] != 1 or
            manifest['claim_scope'] !=
            'OFFLINE_IMMUTABLE_TOPOLOGY_PROXY_NO_MOTION'):
        failures.append('claim_schema')
    profiles = manifest['profiles']
    if not isinstance(profiles, dict) or tuple(profiles) != ('A', 'B'):
        failures.append('profile_schema')
    else:
        for profile, records in profiles.items():
            if (not isinstance(records, list) or not records or
                    not all(_identity_valid(root, record) for record in records)):
                failures.append(f'profile_identity:{profile}')
                continue
            expected_sources = list(PROFILE_SOURCES[profile]) + [
                ('generated', 'resolved_params.json')]
            if [(item['source_root'], item['source_relative_path'])
                    for item in records] != expected_sources:
                failures.append(f'profile_source_set:{profile}')
            if not _resolved_profile_valid(root, profile, records):
                failures.append(f'resolved_params:{profile}')
    try:
        if manifest['payload_tree'] != tree_inventory(root / 'payload'):
            failures.append('payload_tree')
    except (OSError, ValueError):
        failures.append('payload_tree')
    readiness = manifest['readiness_evidence']
    pending = _pending_readiness_valid(readiness)
    complete = _complete_readiness_valid(readiness)
    if not (pending or complete):
        failures.append('readiness_evidence')
    if complete and not _runtime_binary_snapshots_valid(
            root, manifest['bundle_id'], readiness):
        failures.append('runtime_binary_identity')
    if not _predecessors_valid(root, manifest['predecessors'], complete):
        failures.append('predecessors')
    if complete:
        for name, record in manifest['predecessors'].items():
            expected_hash = record['artifact_tree']['tree_sha256']
            if readiness['predecessors'][name][
                    'canonical_tree_sha256'] != expected_hash:
                failures.append(f'readiness_predecessor:{name}')
        if readiness['seed_lock']['protocol_salt'] != (
                f"G006-{manifest['bundle_id']}"):
            failures.append('readiness_seed_protocol')
        predecessor_hashes = {
            name: readiness['predecessors'][name]['canonical_tree_sha256']
            for name in PREDECESSOR_KEYS}
        if readiness['seed_lock'] != held_out_seed_lock(
                f"G006-{manifest['bundle_id']}", predecessor_hashes):
            failures.append('readiness_seed_recomputed')
        for scenario_name, records in readiness['scenario_attempts'].items():
            for record in records:
                valid = (_attempt_evidence_valid(
                    root, manifest['bundle_id'], scenario_name,
                    record['seed'], record['evidence'],
                    profiles['A' if scenario_name.startswith(
                        'mapping_') else 'B'])
                    if record['outcome'] == 'PASS' else
                    _infra_evidence_valid(
                        root, manifest['bundle_id'], scenario_name,
                        record['seed'], record['evidence']))
                if not valid:
                    failures.append(
                        f'attempt_evidence:{scenario_name}:{record["attempt"]}')
    harness_sources = manifest['harness_sources']
    if (not isinstance(harness_sources, list) or
            [(item.get('source_root'), item.get('source_relative_path'))
             for item in harness_sources] != [
                ('repository', relative) for relative in HARNESS_SOURCES] or
            not all(_identity_valid(root, item) for item in harness_sources)):
        failures.append('harness_sources')
    expected_bundle_id = _protocol_id(
        profiles, harness_sources, manifest['predecessors'],
        manifest['topologies'], manifest['hardware_boundary'],
        manifest['scenario_contract'], manifest['measurement_contract'])
    if manifest['bundle_id'] != expected_bundle_id:
        failures.append('bundle_id')
    boundary = manifest['hardware_boundary']
    if boundary != _hardware_boundary():
        failures.append('hardware_boundary')
    if manifest['topologies'] != _topologies():
        failures.append('topologies')
    try:
        if strict_json_load(root / 'payload/hardware_stub_topology.json') != {
                'mode': 'typed_endpoint_contract_only_no_publishers_started',
                'endpoints': list(HARDWARE_STUBS)}:
            failures.append('hardware_stub_topology')
        if strict_json_load(root / 'payload/offline_probe.json') != {
                'status': 'NOT_EVALUATED',
                'reason': 'BUILDER_DOES_NOT_LAUNCH_ROS_OR_HARDWARE'}:
            failures.append('offline_probe_record')
    except (OSError, ValueError, TypeError):
        failures.append('generated_payload')
    runtime = manifest['runtime']
    if not _recorded_runtime_valid(root, runtime, profiles):
        failures.append('runtime')
    selection = manifest['selection']
    pending_selection = {
            'amcl': 'stock_production_applicable_parameters',
            'patched_random_seed_binary_included': False,
            'frontier_policy': 'current',
            'frontier_reason': 'G005_FULL_NOT_EVALUATED',
            'g003_g004_runtime_equivalence': 'NOT_EVALUATED',
            'g005_golden_decision_vectors': 'NOT_EVALUATED',
            'runtime_binary_identity': 'NOT_EVALUATED',
            'cartographer_subdivision': 1,
            'cartographer_reason': 'G008_RETAIN_PRODUCTION_SUBDIVISION_1'}
    complete_selection = dict(pending_selection)
    complete_selection.update({
        'frontier_reason': 'G005_GOLDEN_VECTORS_VERIFIED',
        'g003_g004_runtime_equivalence': 'PASS',
        'g005_golden_decision_vectors': 'PASS',
        'runtime_binary_identity': 'PASS'})
    if selection != (complete_selection if complete else pending_selection):
        failures.append('selection')
    if pending and manifest['seed_lock'] != {
            'status': SEED_PENDING,
            'protocol_salt': f"G006-{manifest['bundle_id']}",
            'scenario_seeds': {}}:
        failures.append('seed_lock')
    if complete and manifest['seed_lock'] != readiness['seed_lock']:
        failures.append('seed_lock')
    matrix = manifest['gate_applicability']
    allowed_matrix = ({'PASS', 'NOT_APPLICABLE'} if complete else
                      {'NOT_EVALUATED', 'NOT_APPLICABLE'})
    if (not validate_gate_matrix(matrix) or set(matrix) != set(SCENARIOS) or
            any(value not in allowed_matrix
                for row in matrix.values() for value in row.values())):
        failures.append('gate_applicability')
    expected_external = manifest['external_hard_gates'] if complete else {
        'no_motion_exact_launch': 'NOT_EVALUATED',
        'real_bag_isolated_replay': 'NOT_EVALUATED'}
    if (complete and not _external_evidence_valid(
            root, manifest['bundle_id'], expected_external)) or (
            pending and manifest['external_hard_gates'] != expected_external):
        failures.append('external_gates')
    if (manifest['pi_historical_saved_nav'] != 'NOT_EVALUATED' or
            manifest['pi_current_bundle'] != 'NOT_RUN' or
            manifest['pi_resource_gate'] !=
            'NOT_EVALUATED_CURRENT_BUNDLE'):
        failures.append('decision')
    if manifest['claims'] != CLAIMS_LOCK:
        failures.append('claims')
    expected_decision = final_decision(matrix, expected_external, readiness)
    if manifest['decision'] != expected_decision:
        failures.append('decision')
    scenario = manifest['scenario_contract']
    expected_scenario = _scenario_contract()
    if complete:
        expected_scenario['attempts'] = readiness['scenario_attempts']
    if scenario != expected_scenario:
        failures.append('scenario_contract')
    if manifest['measurement_contract'] != _measurement_contract():
        failures.append('measurement_contract')
    if tuple(GATE_FAMILIES) != (
            'common', 'motion', 'mapping', 'saved_nav', 'real_bag', 'no_motion'):
        failures.append('internal_gate_contract')
    try:
        probe = probe_bundle(root)
        if probe['status'] != 'PASS':
            failures.append('offline_topology_probe')
    except (OSError, ValueError, KeyError, TypeError):
        failures.append('offline_topology_probe')
    actual_bytes = sum(path.stat().st_size for path in root.rglob('*')
                       if path.is_file() and not path.is_symlink())
    if actual_bytes > BUNDLE_LIMIT_BYTES:
        failures.append('bundle_cap')
    return {'status': 'PASS' if not failures else 'FAIL',
            'failures': failures, 'actual_bytes': actual_bytes,
            'decision': manifest['decision']}


def validate_bundle(root: Path) -> dict:
    """Return a fail-closed verdict for malformed nested input of any kind."""
    try:
        return _validate_bundle(root)
    except (AttributeError, KeyError, OSError, TypeError, ValueError):
        return {'status': 'FAIL', 'failures': ['invalid_nested_structure']}


def main() -> int:
    """Validate one bundle without modifying it."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle-root', type=Path, required=True)
    args = parser.parse_args()
    result = validate_bundle(args.bundle_root)
    print(result)
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
