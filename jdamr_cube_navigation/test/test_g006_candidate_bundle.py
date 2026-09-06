#!/usr/bin/env python3
"""Contract, hostile, and offline smoke tests for the G006 bundle."""

from copy import deepcopy
import json

import build_g006_candidate_bundle as builder
from g006_candidate_contract import (
    CARTOGRAPHER_METRICS,
    final_decision,
    FINAL_NOT_READY,
    FINAL_READY,
    gate_matrix,
    held_out_seed_lock,
    SCENARIOS,
    SEED_LOCKED,
    SEED_PENDING,
    strict_json_load,
    validate_cartographer_metrics,
    validate_gate_matrix,
)
from probe_g006_candidate_bundle import probe_bundle
import pytest
import validate_g006_candidate_bundle as validator
from validate_g006_candidate_bundle import validate_bundle


def _metric_values():
    result = {}
    for name, contract in CARTOGRAPHER_METRICS.items():
        if contract['type'] == 'nonnegative_int':
            result[name] = 1
        elif contract['type'] == 'finite_vector3':
            result[name] = [0.0, 0.0, 0.0]
        else:
            result[name] = 0.5
    return result


def _complete_readiness(salt='salt'):
    hashes = {name: character * 64 for name, character in zip(
        ('G002', 'G003', 'G004', 'G005', 'G008'), 'abcde')}
    seed_lock = held_out_seed_lock(salt, hashes)
    attempts = {}
    for scenario, seeds in seed_lock['scenario_seeds'].items():
        attempts[scenario] = [
            {'attempt': index, 'seed': seed, 'outcome': 'PASS',
             'semantic_final': True,
             'evidence': {
                 'bundle_relative_path':
                     f'evidence/attempts/{scenario}/{seed}.json',
                 'size_bytes': 1, 'sha256': 'f' * 64}}
            for index, seed in enumerate(seeds, 1)]
    return {
        'status': 'COMPLETE',
        'predecessors': {
            name: {'status': 'COMPLETE', 'verified': True,
                   'canonical_tree_sha256': digest}
            for name, digest in hashes.items()},
        'seed_lock': seed_lock, 'scenario_attempts': attempts,
        'runtime_equivalence': {'status': 'PASS', 'evidence': {
            'bundle_relative_path': 'evidence/runtime_identity.json',
            'size_bytes': 1, 'sha256': 'e' * 64}},
        'golden_vectors': 'PASS',
        'runtime_binary_identity': {'status': 'PASS', 'evidence': {
            'bundle_relative_path': 'evidence/runtime_identity.json',
            'size_bytes': 1, 'sha256': 'e' * 64}},
        'claims': {
            'real_motion_authorized': False, 'safety_certified': False,
            'production_qualified': False, 'map_provenance': 'INCOMPLETE',
            'lidar_extrinsic': 'UNVERIFIED'}}


def _promote_bundle_to_complete(root):
    contract = __import__('g006_candidate_contract')
    manifest_path = root / 'bundle_manifest.json'
    manifest = strict_json_load(manifest_path)
    for name in ('G002', 'G005'):
        base = root / 'payload' / 'predecessors' / name
        evidence = base / 'evidence'
        evidence.mkdir(parents=True)
        summary = evidence / 'run_summary.json'
        summary.write_bytes(contract.canonical_json_bytes({
            'status': 'PASS', 'run_count': 60 if name == 'G002' else 15}))
        artifact_tree = contract.tree_inventory(evidence)
        benchmark = ({
            'schema_version': 1, 'objective': 'AMCL_FAULT_BENCHMARK',
            'final_status': 'PASS', 'axis_a_run_count': 15,
            'axis_b_run_count': 45, 'production_hashes_unchanged': True,
            'evidence_tree_sha256': artifact_tree['tree_sha256']}
            if name == 'G002' else {
                'schema_version': 1,
                'objective': 'FRONTIER_POLICY_PAIRED_EVALUATION',
                'final_status': 'PASS', 'run_count': 15,
                'promotion_status': 'PASS',
                'production_hashes_unchanged': True,
                'evidence_tree_sha256': artifact_tree['tree_sha256']})
        (base / 'benchmark_manifest.json').write_bytes(
            contract.canonical_json_bytes(benchmark))
        (base / 'evidence_tree_manifest.json').write_bytes(
            contract.canonical_json_bytes({
                'schema_version': 1, 'artifact_tree': artifact_tree}))
        relative_paths = [
            'benchmark_manifest.json', 'evidence/run_summary.json',
            'evidence_tree_manifest.json']
        identities = []
        snapshots = []
        for relative_path in relative_paths:
            path = base / relative_path
            identity = {
                'relative_path': relative_path,
                'size_bytes': path.stat().st_size,
                'sha256': contract.sha256_file(path)}
            identities.append(identity)
            snapshots.append({
                'source_relative_path': relative_path,
                'bundle_relative_path': f'predecessors/{name}/{relative_path}',
                'size_bytes': identity['size_bytes'],
                'sha256': identity['sha256']})
        manifest['predecessors'][name] = {
            'canonical_root': f'/sealed/{name}', 'status': 'PASS',
            'identities': identities, 'snapshots': snapshots,
            'artifact_tree': artifact_tree,
            'verified_conclusion': {
                'status': 'PASS', 'schema': name,
                'manifest_sha256': contract.sha256_file(
                    base / 'benchmark_manifest.json')}}
    manifest['bundle_id'] = builder._protocol_id(
        manifest['profiles'], manifest['harness_sources'],
        manifest['predecessors'], manifest['topologies'],
        manifest['hardware_boundary'], manifest['scenario_contract'],
        manifest['measurement_contract'])
    hashes = {name: record['artifact_tree']['tree_sha256']
              for name, record in manifest['predecessors'].items()}
    seed_lock = held_out_seed_lock(f"G006-{manifest['bundle_id']}", hashes)

    def write_evidence(relative, value):
        path = root / 'payload' / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(contract.canonical_json_bytes(value))
        return {'bundle_relative_path': relative,
                'size_bytes': path.stat().st_size,
                'sha256': contract.sha256_file(path)}

    def copied_ref(source_relative):
        record = next(item for item in manifest['profiles']['B']
                      if item['source_relative_path'] == source_relative)
        return {'bundle_relative_path': record['relative_path'],
                'size_bytes': record['size_bytes'],
                'sha256': record['sha256']}

    generated = root / 'payload/evidence/generated'
    generated.mkdir(parents=True)
    (generated / 'map.pgm').write_bytes(
        b'P5\n10 10\n255\n' + bytes(range(100)))
    (generated / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n',
        encoding='utf-8')
    (generated / 'map.pbstream').write_bytes(bytes(range(256)) * 5)

    def generated_ref(name):
        path = generated / name
        return {'bundle_relative_path': f'evidence/generated/{name}',
                'size_bytes': path.stat().st_size,
                'sha256': contract.sha256_file(path)}

    map_yaml = copied_ref('autonomous_20260826T161908.yaml')
    map_pgm = copied_ref('autonomous_20260826T161908.pgm')
    mask_yaml = copied_ref(
        'autonomous_20260826T161908_keepout_multi.yaml')
    mask_pgm = copied_ref(
        'autonomous_20260826T161908_keepout_multi.pgm')
    resources = {'cpu_seconds_per_1000_scans': 1.0,
                 'max_rss_bytes': 1024,
                 'callback_gap_p95_s': 0.01, 'tf_gap_p95_s': 0.01}
    attempts = {}
    for scenario, seeds in seed_lock['scenario_seeds'].items():
        profile = 'A' if scenario.startswith('mapping_') else 'B'
        profile_records = manifest['profiles'][profile]
        authorities = [
            {'role': 'map_to_odom',
             'publisher_node': ('cartographer_node' if profile == 'A'
                                else 'amcl'),
             'publisher_gid': f'{profile}-map-gid', 'publisher_count': 1},
            {'role': 'final_cmd_vel', 'publisher_node': 'collision_monitor',
             'publisher_gid': f'{profile}-cmd-gid', 'publisher_count': 1}]
        records = []
        for index, seed in enumerate(seeds, 1):
            mapping_evidence = 'NOT_APPLICABLE'
            saved_nav_evidence = 'NOT_APPLICABLE'
            if profile == 'A':
                mapping_evidence = {
                    'status': 'PASS',
                    'pbstream': generated_ref('map.pbstream'),
                    'map_yaml': generated_ref('map.yaml'),
                    'map_pgm': generated_ref('map.pgm'),
                    'reload_status': 'PASS',
                    'cartographer_metrics': _metric_values(),
                    'start_end_closure_m': 0.1,
                    'raw_trajectory_poses': [[0.0, 0.0, 0.0],
                                             [0.1, 0.0, 0.0]],
                    'raw_metric_samples': [_metric_values(),
                                           _metric_values()]}
            else:
                saved_nav_evidence = {
                    'status': 'PASS', 'map_yaml': map_yaml,
                    'map_pgm': map_pgm, 'mask_yaml': mask_yaml,
                    'mask_pgm': mask_pgm, 'load_status': 'PASS',
                    'amcl_active': True, 'behavior_tree_match': True,
                    'collision_monitor_active': True,
                    'raw_node_states': [
                        {'node': 'map_server', 'state': 'active'},
                        {'node': 'keepout_filter_mask_server',
                         'state': 'active'},
                        {'node': 'amcl', 'state': 'active'},
                        {'node': 'bt_navigator', 'state': 'active'},
                        {'node': 'collision_monitor', 'state': 'active'}]}
            evidence = {
                'schema_version': 1, 'bundle_id': manifest['bundle_id'],
                'scenario': scenario, 'seed': seed, 'status': 'PASS',
                'profile': profile,
                'common': {
                    'input_hash_parity': True,
                    'expected_launch_graph': True, 'lifecycle_active': True,
                    'resource_metrics': resources, 'authorities': authorities,
                    'survivor_count': 0,
                    'production_hashes_unchanged': True},
                'motion': {
                    'status': 'PASS', 'contact_count': 0,
                    'final_zero_hold': True,
                    'causal_trace': [
                        {'event': 'input_ready', 'steady_ns': 1},
                        {'event': 'motion_started', 'steady_ns': 2},
                        {'event': 'final_zero_hold', 'steady_ns': 3}]},
                'raw_trace': {
                    'lifecycle': [
                        {'node': node, 'state': 'active',
                         'steady_ns': position + 1}
                        for position, node in enumerate(
                            (['cartographer_node', 'collision_monitor'] if
                             profile == 'A' else
                             ['map_server', 'amcl', 'collision_monitor']))],
                    'resource_samples': [
                        {'steady_ns': 10, 'scan_count': 0,
                         'cpu_seconds': 0.0, 'rss_bytes': 512,
                         'callback_gap_s': 0.0, 'tf_gap_s': 0.0},
                        {'steady_ns': 20, 'scan_count': 1000,
                         'cpu_seconds': 1.0, 'rss_bytes': 1024,
                         'callback_gap_s': 0.01, 'tf_gap_s': 0.01}],
                    'publishers': authorities,
                    'processes': [{
                        'name': f'{profile}-runtime', 'pid': 1000 + index,
                        'alive_after_cleanup': False}],
                    'production_files': [{
                        'path': row['source_relative_path'],
                        'before_sha256': row['sha256'],
                        'after_sha256': row['sha256']}
                        for row in profile_records
                        if row['source_root'] != 'generated'],
                    'input_files': [{
                        'path': row['relative_path'],
                        'size_bytes': row['bundle_size_bytes'],
                        'sha256': row['bundle_sha256']}
                        for row in profile_records],
                    'launch_graph': [dict(zip(
                        contract.RUNTIME_ENTITY_FIELDS, row))
                        for row in contract.RUNTIME_ENTITIES
                        if row[0] == profile],
                    'motion_samples': [
                        {'steady_ns': 2, 'linear_x': 0.1,
                         'angular_z': 0.0},
                        {'steady_ns': 3, 'linear_x': 0.0,
                         'angular_z': 0.0}],
                    'events': [
                        {'event': 'input_ready', 'steady_ns': 1},
                        {'event': 'motion_started', 'steady_ns': 2},
                        {'event': 'final_zero_hold', 'steady_ns': 3}],
                    'contacts': []},
                'mapping': mapping_evidence,
                'saved_nav': saved_nav_evidence}
            evidence_ref = write_evidence(
                f'evidence/attempts/{scenario}/{seed}.json', evidence)
            records.append({'attempt': index, 'seed': seed,
                            'outcome': 'PASS', 'semantic_final': True,
                            'evidence': evidence_ref})
        attempts[scenario] = records
    runtime_entities = []
    runtime_graph = []
    component_container_pid = 2017
    for index, fields in enumerate(contract.RUNTIME_ENTITIES):
        record = dict(zip(contract.RUNTIME_ENTITY_FIELDS, fields))
        binary = generated / f'runtime-{index}.bin'
        binary.write_bytes(f'runtime-binary-{index}\n'.encode())
        binary.chmod(0o755)
        binary_ref = generated_ref(binary.name)
        absolute_path = f'/opt/ros/jazzy/lib/sealed/runtime-{index}'
        pid = (component_container_pid if
               record['process_class'] == 'composed_component' else
               2000 + index)
        publisher_gids = ([] if record['authority_role'] == 'none'
                          else [f'gid-{index}'])
        record.update({
            'absolute_path': absolute_path,
            'mode_octal': '0755',
            'size_bytes': binary_ref['size_bytes'],
            'sha256': binary_ref['sha256'],
            'binary_snapshot': binary_ref, 'pid': pid,
            'proc_maps': [{
                'pid': pid, 'absolute_path': absolute_path,
                'size_bytes': binary_ref['size_bytes'],
                'sha256': binary_ref['sha256']}],
            'publisher_gids': publisher_gids})
        runtime_entities.append(record)
        runtime_graph.append({
            **{field: record[field] for field in
               contract.RUNTIME_ENTITY_FIELDS},
            'pid': pid, 'publisher_gids': publisher_gids})
    runtime_identity = {'status': 'PASS', 'evidence': write_evidence(
        'evidence/runtime_identity.json', {
            'schema_version': 1, 'bundle_id': manifest['bundle_id'],
            'status': 'PASS',
            'static_probe_used_as_runtime_equivalence': False,
            'entities': runtime_entities, 'graph': runtime_graph})}
    readiness = {
        'status': 'COMPLETE',
        'predecessors': {
            name: {'status': 'COMPLETE', 'verified': True,
                   'canonical_tree_sha256': digest}
            for name, digest in hashes.items()},
        'seed_lock': seed_lock, 'scenario_attempts': attempts,
        'runtime_equivalence': {
            'status': 'PASS',
            'evidence': dict(runtime_identity['evidence'])},
        'golden_vectors': 'PASS',
        'runtime_binary_identity': runtime_identity,
        'claims': manifest['claims']}
    manifest['readiness_evidence'] = readiness
    manifest['seed_lock'] = seed_lock
    manifest['scenario_contract']['attempts'] = attempts
    manifest['gate_applicability'] = gate_matrix('PASS')
    manifest['external_hard_gates'] = {
        'no_motion_exact_launch': write_evidence(
            'evidence/no_motion.json', {
                'schema_version': 1, 'bundle_id': manifest['bundle_id'],
                'status': 'PASS',
                'claim_scope': 'NO_MOTION_EXACT_LAUNCH_HARDWARE_STUBS',
                'profiles_exact_launch': ['A', 'B'],
                'lifecycle_active': True, 'motion_publishers_started': 0,
                'authority_checks': 'PASS', 'survivor_count': 0,
                'production_hashes_unchanged': True,
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
                    'surviving_pids': []}}),
        'real_bag_isolated_replay': write_evidence(
            'evidence/real_bag.json', {
                'schema_version': 1, 'bundle_id': manifest['bundle_id'],
                'status': 'PASS',
                'claim_scope':
                    'ISOLATED_REAL_BAG_REPLAY_NO_GT_NO_MOTION_COMMAND',
                'input_hash_parity': True, 'recorded_clock_absent': True,
                'generated_amcl_pose_only': True,
                'resource_metrics': resources, 'authority_checks': 'PASS',
                'survivor_count': 0,
                'production_hashes_unchanged': True,
                'raw_trace': {
                    'topics': ['/odom', '/scan', '/tf', '/tf_static'],
                    'recorded_clock_count': 0,
                    'recorded_amcl_pose_count': 0,
                    'generated_amcl_pose_count': 10,
                    'surviving_pids': [],
                    'authority_publishers': [{
                        'role': 'map_to_odom', 'node': 'amcl',
                        'gid': 'real-bag-amcl'}],
                    'resource_samples': [
                        {'steady_ns': 1, 'scan_count': 0,
                         'cpu_seconds': 0.0, 'rss_bytes': 512,
                         'callback_gap_s': 0.0, 'tf_gap_s': 0.0},
                        {'steady_ns': 2, 'scan_count': 1000,
                         'cpu_seconds': 1.0, 'rss_bytes': 1024,
                         'callback_gap_s': 0.01,
                         'tf_gap_s': 0.01}]}})}
    manifest['selection'].update({
        'frontier_reason': 'G005_GOLDEN_VECTORS_VERIFIED',
        'g003_g004_runtime_equivalence': 'PASS',
        'g005_golden_decision_vectors': 'PASS',
        'runtime_binary_identity': 'PASS'})
    manifest['decision'] = final_decision(
        manifest['gate_applicability'], manifest['external_hard_gates'],
        readiness)
    manifest['payload_tree'] = contract.tree_inventory(root / 'payload')
    manifest_path.write_bytes(contract.canonical_json_bytes(manifest))
    return manifest


@pytest.fixture(scope='module')
def bundle(tmp_path_factory):
    root = tmp_path_factory.mktemp('g006') / 'bundle'
    builder.build_bundle(root)
    return root


def test_gate_matrix_distinguishes_not_applicable_and_not_evaluated():
    matrix = gate_matrix()
    assert validate_gate_matrix(matrix)
    assert matrix['mapping_nominal_closed_loop']['mapping'] == 'NOT_EVALUATED'
    assert matrix['mapping_nominal_closed_loop']['saved_nav'] == 'NOT_APPLICABLE'
    attacked = deepcopy(matrix)
    attacked['mapping_nominal_closed_loop']['saved_nav'] = 'PASS'
    assert not validate_gate_matrix(attacked)


def test_final_decision_requires_every_applicable_and_external_gate():
    matrix = gate_matrix('PASS')
    strings = {'no_motion_exact_launch': 'PASS',
               'real_bag_isolated_replay': 'PASS'}
    external = {
        'no_motion_exact_launch': {
            'bundle_relative_path': 'evidence/no_motion.json',
            'size_bytes': 1, 'sha256': 'a' * 64},
        'real_bag_isolated_replay': {
            'bundle_relative_path': 'evidence/real_bag.json',
            'size_bytes': 1, 'sha256': 'b' * 64}}
    readiness = _complete_readiness()
    assert final_decision(matrix, strings, readiness) == FINAL_NOT_READY
    assert final_decision(matrix, external, readiness) == FINAL_NOT_READY
    matrix['saved_fixed_obstacle_detour']['motion'] = 'NOT_EVALUATED'
    assert final_decision(matrix, external, readiness) == FINAL_NOT_READY


def test_seed_lock_is_pending_until_all_hashes_and_then_is_pure():
    pending = held_out_seed_lock('salt', {'G002': 'NOT_EVALUATED'})
    assert pending == {'status': SEED_PENDING, 'protocol_salt': 'salt',
                       'scenario_seeds': {}}
    hashes = {name: character * 64 for name, character in zip(
        ('G002', 'G003', 'G004', 'G005', 'G008'), 'abcde')}
    first = held_out_seed_lock('salt', hashes)
    second = held_out_seed_lock('salt', hashes)
    assert first == second
    assert first['status'] == SEED_LOCKED
    assert tuple(first['scenario_seeds']) == SCENARIOS
    flat = [seed for seeds in first['scenario_seeds'].values() for seed in seeds]
    assert len(flat) == 36 == len(set(flat))
    assert not set(flat) & {11, 23, 42, 67, 89}


def test_ready_requires_all_six_semantic_seed_finals_without_resampling():
    matrix = gate_matrix('PASS')
    external = {
        'no_motion_exact_launch': {
            'bundle_relative_path': 'evidence/no_motion.json',
            'size_bytes': 1, 'sha256': 'a' * 64},
        'real_bag_isolated_replay': {
            'bundle_relative_path': 'evidence/real_bag.json',
            'size_bytes': 1, 'sha256': 'b' * 64}}
    valid = _complete_readiness()
    mutations = []
    missing_seed = deepcopy(valid)
    missing_seed['scenario_attempts'][SCENARIOS[0]].pop()
    mutations.append(missing_seed)
    retry_after_pass = deepcopy(valid)
    retry_after_pass['scenario_attempts'][SCENARIOS[0]].append({
        'attempt': 7,
        'seed': valid['seed_lock']['scenario_seeds'][SCENARIOS[0]][-1],
        'outcome': 'INFRA_RETRY', 'semantic_final': False,
        'evidence': {
            'bundle_relative_path': 'evidence/retry.json',
            'size_bytes': 1, 'sha256': 'c' * 64}})
    mutations.append(retry_after_pass)
    wrong_hash_set = deepcopy(valid)
    wrong_hash_set['predecessors'].pop('G008')
    mutations.append(wrong_hash_set)
    mutable_claim = deepcopy(valid)
    mutable_claim['claims']['real_motion_authorized'] = True
    mutations.append(mutable_claim)
    for attacked in mutations:
        assert final_decision(matrix, external, attacked) == FINAL_NOT_READY


def test_cartographer_metric_family_is_exact_typed_and_finite():
    values = _metric_values()
    assert validate_cartographer_metrics(values) == 'PASS'
    for mutation in (
            lambda row: row.pop('tf_gap_p95_s'),
            lambda row: row.__setitem__('extra', 1),
            lambda row: row.__setitem__('max_rss_bytes', True),
            lambda row: row.__setitem__('scan_matcher_score', float('nan')),
            lambda row: row.__setitem__('trajectory_start_pose_m_rad', [0, 0])):
        attacked = deepcopy(values)
        mutation(attacked)
        assert validate_cartographer_metrics(attacked) == 'INVALID'
    values['constraints_attempted'] = 1
    values['constraints_matched'] = 2
    assert validate_cartographer_metrics(values) == 'INVALID'


def test_cartographer_metrics_survive_canonical_json_key_sorting(tmp_path):
    path = tmp_path / 'metrics.json'
    path.write_bytes(__import__('g006_candidate_contract').canonical_json_bytes(
        _metric_values()))
    reloaded = strict_json_load(path)
    assert list(reloaded) == sorted(reloaded)
    assert validate_cartographer_metrics(reloaded) == 'PASS'


def test_offline_bundle_build_probe_and_fresh_validation(bundle):
    result = validate_bundle(bundle)
    assert result['status'] == 'PASS', result
    assert result['decision'] == FINAL_NOT_READY
    probe = probe_bundle(bundle)
    assert probe['status'] == 'PASS'
    assert probe['claim_scope'] == 'OFFLINE_STATIC_TOPOLOGY_PROBE_NO_ROS_NO_MOTION'


def test_synthetic_ready_claim_without_mcap_elf_and_official_predecessors_fails(
        bundle, tmp_path):
    copy = tmp_path / 'fake-ready'
    __import__('shutil').copytree(bundle, copy)
    manifest = _promote_bundle_to_complete(copy)
    manifest['decision'] = FINAL_READY
    (copy / 'bundle_manifest.json').write_bytes(
        __import__('g006_candidate_contract').canonical_json_bytes(manifest))
    result = validate_bundle(copy)
    assert result['status'] == 'FAIL'
    assert 'predecessors' in result['failures']
    assert 'runtime_binary_identity' in result['failures']
    assert 'external_gates' in result['failures']
    assert any(item.startswith('attempt_evidence:')
               for item in result['failures'])


def test_complete_evidence_field_mutations_fail_deep_validation(bundle, tmp_path):
    contract = __import__('g006_candidate_contract')

    def attack(name, select_reference, mutation):
        copy = tmp_path / name
        __import__('shutil').copytree(bundle, copy)
        manifest = _promote_bundle_to_complete(copy)
        reference = select_reference(manifest)
        evidence_path = copy / 'payload' / reference['bundle_relative_path']
        evidence = strict_json_load(evidence_path)
        mutation(evidence)
        evidence_path.write_bytes(contract.canonical_json_bytes(evidence))
        new_reference = {
            'bundle_relative_path': reference['bundle_relative_path'],
            'size_bytes': evidence_path.stat().st_size,
            'sha256': contract.sha256_file(evidence_path)}
        old_reference = dict(reference)

        def replace(value):
            if isinstance(value, dict):
                for key, item in value.items():
                    if item == old_reference:
                        value[key] = dict(new_reference)
                    else:
                        replace(item)
            elif isinstance(value, list):
                for index, item in enumerate(value):
                    if item == old_reference:
                        value[index] = dict(new_reference)
                    else:
                        replace(item)

        replace(manifest)
        manifest['payload_tree'] = contract.tree_inventory(copy / 'payload')
        (copy / 'bundle_manifest.json').write_bytes(
            contract.canonical_json_bytes(manifest))
        assert validate_bundle(copy)['status'] == 'FAIL'

    def first_attempt(row):
        return row['readiness_evidence']['scenario_attempts'][
            SCENARIOS[0]][0]['evidence']

    attack('attempt-parity', first_attempt,
           lambda row: row['common'].__setitem__('input_hash_parity', False))
    attack('attempt-raw-trace', first_attempt,
           lambda row: row.pop('raw_trace'))
    attack('attempt-input-identity', first_attempt,
           lambda row: row['raw_trace']['input_files'][0].__setitem__(
               'sha256', '0' * 64))
    attack('attempt-production-identity', first_attempt,
           lambda row: row['raw_trace']['production_files'][0].update({
               'before_sha256': '1' * 64,
               'after_sha256': '1' * 64}))
    attack('attempt-na', first_attempt,
           lambda row: row.__setitem__('saved_nav', {'status': 'PASS'}))
    attack('runtime-node', lambda row: row['readiness_evidence'][
        'runtime_binary_identity']['evidence'],
        lambda row: row['entities'][0].__setitem__('node_name', 'forged'))
    attack('runtime-executable', lambda row: row['readiness_evidence'][
        'runtime_binary_identity']['evidence'],
        lambda row: row['entities'][0].__setitem__(
            'executable_or_plugin', '/tmp/arbitrary'))
    attack('runtime-static-probe', lambda row: row['readiness_evidence'][
        'runtime_binary_identity']['evidence'],
        lambda row: row.__setitem__(
            'static_probe_used_as_runtime_equivalence', True))
    attack('runtime-proc-pid', lambda row: row['readiness_evidence'][
        'runtime_binary_identity']['evidence'],
        lambda row: row['entities'][0]['proc_maps'][0].__setitem__('pid', 999))
    attack('external-lifecycle', lambda row: row['external_hard_gates'][
        'no_motion_exact_launch'],
        lambda row: row.__setitem__('lifecycle_active', False))
    attack('external-raw', lambda row: row['external_hard_gates'][
        'real_bag_isolated_replay'],
        lambda row: row['raw_trace'].__setitem__(
            'recorded_amcl_pose_count', 1))


def test_runtime_binary_and_predecessor_bytes_are_not_self_asserted(
        bundle, tmp_path):
    contract = __import__('g006_candidate_contract')
    binary_attack = tmp_path / 'binary-attack'
    __import__('shutil').copytree(bundle, binary_attack)
    manifest = _promote_bundle_to_complete(binary_attack)
    runtime_ref = manifest['readiness_evidence'][
        'runtime_binary_identity']['evidence']
    runtime = strict_json_load(
        binary_attack / 'payload' / runtime_ref['bundle_relative_path'])
    binary_ref = runtime['entities'][0]['binary_snapshot']
    (binary_attack / 'payload' / binary_ref[
        'bundle_relative_path']).write_bytes(b'changed executable')
    assert validate_bundle(binary_attack)['status'] == 'FAIL'

    predecessor_attack = tmp_path / 'predecessor-attack'
    __import__('shutil').copytree(bundle, predecessor_attack)
    manifest = _promote_bundle_to_complete(predecessor_attack)
    record = manifest['predecessors']['G002']
    benchmark = predecessor_attack / 'payload/predecessors/G002/' \
        'benchmark_manifest.json'
    benchmark.write_bytes(contract.canonical_json_bytes({'status': 'PASS'}))
    digest = contract.sha256_file(benchmark)
    size = benchmark.stat().st_size
    record['identities'][0].update({'sha256': digest, 'size_bytes': size})
    record['snapshots'][0].update({'sha256': digest, 'size_bytes': size})
    record['verified_conclusion']['manifest_sha256'] = digest
    assert not validator._portable_predecessor_record_valid(
        predecessor_attack, 'G002', record)
    manifest['payload_tree'] = contract.tree_inventory(
        predecessor_attack / 'payload')
    (predecessor_attack / 'bundle_manifest.json').write_bytes(
        contract.canonical_json_bytes(manifest))
    assert validate_bundle(predecessor_attack)['status'] == 'FAIL'


def test_placeholder_mapping_outputs_cannot_support_ready(bundle, tmp_path):
    complete = tmp_path / 'mapping-placeholders'
    __import__('shutil').copytree(bundle, complete)
    manifest = _promote_bundle_to_complete(complete)
    attempt_ref = manifest['readiness_evidence']['scenario_attempts'][
        'mapping_nominal_closed_loop'][0]['evidence']
    attempt = strict_json_load(
        complete / 'payload' / attempt_ref['bundle_relative_path'])
    pbstream_ref = attempt['mapping']['pbstream']
    pbstream = complete / 'payload' / pbstream_ref['bundle_relative_path']
    pbstream.write_bytes(b'pbstream')
    assert not validator._pbstream_valid(complete, pbstream_ref)
    pgm_ref = attempt['mapping']['map_pgm']
    pgm = complete / 'payload' / pgm_ref['bundle_relative_path']
    pgm.write_bytes(b'P5\n1 1\n255\n\x00')
    assert not validator._map_files_valid(
        complete, attempt['mapping']['map_yaml'], pgm_ref)
    assert validate_bundle(complete)['status'] == 'FAIL'


def test_complete_seed_lock_is_recomputed_and_evidence_must_exist(
        bundle, tmp_path):
    contract = __import__('g006_candidate_contract')
    complete = tmp_path / 'seed-lock'
    __import__('shutil').copytree(bundle, complete)
    manifest = _promote_bundle_to_complete(complete)
    manifest['readiness_evidence']['seed_lock']['scenario_seeds'][
        SCENARIOS[0]][0] += 1
    manifest['seed_lock'] = manifest['readiness_evidence']['seed_lock']
    (complete / 'bundle_manifest.json').write_bytes(
        contract.canonical_json_bytes(manifest))
    assert validate_bundle(complete)['status'] == 'FAIL'

    missing = tmp_path / 'missing-evidence'
    __import__('shutil').copytree(bundle, missing)
    manifest = _promote_bundle_to_complete(missing)
    reference = manifest['readiness_evidence']['scenario_attempts'][
        SCENARIOS[0]][0]['evidence']
    (missing / 'payload' / reference['bundle_relative_path']).unlink()
    assert validate_bundle(missing)['status'] == 'FAIL'


def test_durable_validation_does_not_reopen_production_sources(
        bundle, monkeypatch):
    def forbidden(*_args, **_kwargs):
        raise AssertionError('portable validator reopened an external source')

    monkeypatch.setattr(builder, '_source_path', forbidden)
    monkeypatch.setattr(validator, '_package_versions', forbidden,
                        raising=False)
    assert validate_bundle(bundle)['status'] == 'PASS'


def test_bundle_records_exact_proxy_boundary_and_conservative_selection(bundle):
    manifest = strict_json_load(bundle / 'bundle_manifest.json')
    assert manifest['hardware_boundary']['mode'] == (
        'hardware_boundary_substituted_topology_proxy')
    assert manifest['hardware_boundary']['physical_execution_claim'] is False
    assert manifest['hardware_boundary']['pi_execution_claim'] is False
    assert manifest['selection']['patched_random_seed_binary_included'] is False
    assert manifest['selection']['amcl'] == 'stock_production_applicable_parameters'
    assert manifest['selection']['frontier_policy'] == 'current'
    assert manifest['selection']['g005_golden_decision_vectors'] == 'NOT_EVALUATED'
    assert manifest['pi_current_bundle'] == 'NOT_RUN'
    assert manifest['pi_resource_gate'] == 'NOT_EVALUATED_CURRENT_BUNDLE'
    assert manifest['scenario_contract']['attempts'] == {
        scenario: [] for scenario in SCENARIOS}


def test_topology_contracts_bind_profile_a_and_b(bundle):
    manifest = strict_json_load(bundle / 'bundle_manifest.json')
    profile_a = manifest['topologies']['A']
    assert profile_a['physical_launch_set'] == [
        'real_bringup.launch.py', 'cartographer_real.launch.py',
        'autonomous_mapping.launch.py']
    assert profile_a['expanded_nav2_launch'] == (
        'nav2_bringup/navigation_launch.py')
    assert profile_a['nav2_process_graph'] == 'use_composition=false'
    assert profile_a['cartographer_subdivision'] == 1
    assert profile_a['map_to_odom_sole_authority'] == 'cartographer_node'
    profile_b = manifest['topologies']['B']
    assert profile_b['nav2_process_graph'] == 'component_container_isolated_subset'
    assert profile_b['map_to_odom_sole_authority'] == 'amcl'
    assert profile_b['final_cmd_vel_sole_authority'] == 'collision_monitor'
    profile_sources = manifest['profiles']
    assert any(item['source_relative_path'].endswith('setup.py')
               for item in profile_sources['A'])
    assert any(item['source_relative_path'].endswith('frontier_explorer.py')
               for item in profile_sources['A'])
    assert any(item['source_relative_path'].endswith('nav2_liveness_guard.py')
               for item in profile_sources['B'])
    assert len(manifest['harness_sources']) == 4


def test_g008_uses_canonical_storage_conclusion_not_stale_summary(bundle):
    record = strict_json_load(bundle / 'bundle_manifest.json')['predecessors']['G008']
    assert record['artifact_tree']['manifest_sha256'] == (
        '5ab48856501098aec91829852f222aa0eede746b42bac1ad5306c1055e3b6917')
    assert record['artifact_tree']['tree_sha256'] == (
        '9360e5e5b3c357a3176a553bc588ed924b19c9007b1bc1f18f6e24e96424af08')
    assert record['artifact_tree']['final_status'] == 'RETAIN_PRODUCTION'
    assert record['artifact_tree']['selected_subdivision'] == 1


def test_payload_mutation_extra_file_and_symlink_fail_closed(bundle, tmp_path):
    for name, mutate in (
            ('mutation', lambda root: (root / 'payload/offline_probe.json').write_text(
                '{}\n', encoding='utf-8')),
            ('extra', lambda root: (root / 'payload/shadow').write_text(
                'x', encoding='utf-8')),
            ('symlink', lambda root: (root / 'payload/shadow').symlink_to(
                '/etc/hosts'))):
        copy = tmp_path / name
        __import__('shutil').copytree(bundle, copy)
        mutate(copy)
        assert validate_bundle(copy)['status'] == 'FAIL'


def test_manifest_unknown_duplicate_and_nonfinite_fail_closed(bundle, tmp_path):
    original = (bundle / 'bundle_manifest.json').read_text(encoding='utf-8')
    variants = {
        'unknown': json.dumps({**json.loads(original), 'extra': 1}),
        'duplicate': original.rstrip()[:-1] + ',"decision":"READY"}',
        'nonfinite': original.rstrip()[:-1] + ',"extra":NaN}',
    }
    for name, content in variants.items():
        copy = tmp_path / name
        __import__('shutil').copytree(bundle, copy)
        (copy / 'bundle_manifest.json').write_text(content, encoding='utf-8')
        assert validate_bundle(copy)['status'] == 'FAIL'


def test_manifest_claim_and_predecessor_tamper_fail_closed(bundle, tmp_path):
    for name, mutate in (
            ('claim', lambda row: row['claims'].__setitem__(
                'safety_certified', True)),
            ('ready', lambda row: row.__setitem__('decision', FINAL_READY)),
            ('predecessor', lambda row: row['predecessors']['G004'][
                'identities'][0].__setitem__('sha256', '0' * 64))):
        copy = tmp_path / name
        __import__('shutil').copytree(bundle, copy)
        path = copy / 'bundle_manifest.json'
        row = json.loads(path.read_text())
        mutate(row)
        path.write_text(json.dumps(row), encoding='utf-8')
        assert validate_bundle(copy)['status'] == 'FAIL'


def test_nested_malformed_values_return_fail_without_exception(bundle, tmp_path):
    def mutate_path(row):
        row['profiles']['A'][0]['relative_path'] = '../escape'

    def mutate_duplicate(row):
        row['profiles']['A'][1]['relative_path'] = (
            row['profiles']['A'][0]['relative_path'])

    mutations = (
        lambda row: row.__setitem__('profiles', None),
        mutate_path,
        lambda row: row['profiles']['A'][0].__setitem__('mode_octal', True),
        lambda row: row['profiles']['A'][0].__setitem__('size_bytes', -1),
        lambda row: row['profiles']['A'][0].__setitem__('sha256', None),
        mutate_duplicate,
        lambda row: row['predecessors']['G003']['artifact_tree'].__setitem__(
            'entries', None),
        lambda row: row['predecessors']['G003'].__setitem__(
            'canonical_root', 'relative/path'),
    )
    for index, mutation in enumerate(mutations):
        copy = tmp_path / f'nested-{index}'
        __import__('shutil').copytree(bundle, copy)
        path = copy / 'bundle_manifest.json'
        row = json.loads(path.read_text())
        mutation(row)
        path.write_text(json.dumps(row), encoding='utf-8')
        assert validate_bundle(copy)['status'] == 'FAIL'


def test_build_failure_is_atomic(monkeypatch, tmp_path):
    output = tmp_path / 'never-published'

    def fail(_payload):
        raise RuntimeError('injected copy failure')

    monkeypatch.setattr(builder, '_copy_profiles', fail)
    with pytest.raises(RuntimeError):
        builder.build_bundle(output)
    assert not output.exists()
    assert not list(tmp_path.glob('.never-published.stage-*'))
