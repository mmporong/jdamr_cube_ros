#!/usr/bin/env python3
"""Regression tests for the G002 AMCL determinism preflight."""

from pathlib import Path

from amcl_fault_contract import canonical_json_bytes, sha256_file
import amcl_particle_observer as observer
import pytest
import run_amcl_determinism_preflight as runner


def _cloud(stamp_ns=1, x=1.0):
    from nav2_msgs.msg import Particle, ParticleCloud
    result = ParticleCloud()
    result.header.frame_id = 'map'
    result.header.stamp.sec = stamp_ns // 1_000_000_000
    result.header.stamp.nanosec = stamp_ns % 1_000_000_000
    particle = Particle()
    particle.pose.position.x = x
    particle.pose.orientation.w = 1.0
    particle.weight = 0.25
    result.particles = [particle]
    return result


def _run(seed, digests, status='PASS'):
    return {
        'status': status, 'seed': seed,
        'observer': {'clouds': [
            {'payload_sha256': digest} for digest in digests]},
    }


def test_particle_digest_excludes_only_header_stamp_and_includes_weight():
    first = _cloud(stamp_ns=1)
    second = _cloud(stamp_ns=999)
    assert observer.particle_payload(first) == observer.particle_payload(second)
    second.particles[0].weight = 0.75
    assert observer.particle_payload(first) != observer.particle_payload(second)
    second.particles[0].weight = first.particles[0].weight
    second.particles[0].pose.position.x = 1.1
    assert observer.particle_payload(first) != observer.particle_payload(second)


def test_particle_digest_rejects_nonfinite_pose():
    cloud = _cloud()
    cloud.particles[0].pose.position.x = float('nan')
    with pytest.raises(ValueError, match='non-finite'):
        observer.particle_payload(cloud)


def test_particle_digest_rejects_nonfinite_weight():
    cloud = _cloud()
    cloud.particles[0].weight = float('inf')
    with pytest.raises(ValueError, match='non-finite'):
        observer.particle_payload(cloud)


def test_comparison_requires_exact_plan_and_seed_divergence():
    passing = runner._comparison([
        _run(11, ['a', 'b']), _run(11, ['a', 'b']),
        _run(23, ['a', 'c'])])
    assert passing == {
        'evaluated': True, 'same_seed_equal': True,
        'different_seed_differs': True}
    assert runner._comparison([
        _run(11, ['a']), _run(11, ['b']), _run(23, ['c'])
    ])['same_seed_equal'] is False
    assert runner._comparison([_run(11, ['a'])])['evaluated'] is False


def test_tf_bootstrap_selects_first_bracketed_scan_without_magic_offset():
    scans = [(0, -100), (100, 0), (200, 100)]
    transforms = [(70, 70), (130, 130)]
    plan = runner._select_tf_bootstrap_plan(scans, transforms, 0)
    assert plan['first_main_scan_header_ns'] == 100
    assert plan['lower_tf_header_ns'] == 70
    assert plan['upper_tf_header_ns'] == 130
    assert plan['start_offset_ns'] == 150


def test_tf_bootstrap_rejects_scan_without_transform_bracket():
    with pytest.raises(ValueError, match='no scan'):
        runner._select_tf_bootstrap_plan(
            [(0, -100), (100, 0)], [(70, 70)], 0)


def test_main_player_stays_paused_until_initialpose_after_tf_prelude():
    source = Path(runner.__file__).read_text(encoding='utf-8')
    run_one = source[source.index('def _run_one'):source.index(
        'def _comparison')]
    assert "'tf_prelude_player'" in run_one
    assert "'--topics', '/odom', '/tf', '/tf_static'" in run_one
    assert "'/rosbag2_player/burst'" not in run_one
    assert run_one.index("request.write_text('publish once") < run_one.index(
        "'/rosbag2_player/resume'")


def test_replay_bootstrap_validator_binds_commands_order_and_zero_tf_errors(
        monkeypatch, tmp_path):
    plan = {
        'source_start_storage_ns': 0,
        'previous_scan_storage_ns': 100,
        'first_main_scan_storage_ns': 200,
        'first_main_scan_header_ns': 100,
        'lower_tf_storage_ns': 70,
        'lower_tf_header_ns': 70,
        'upper_tf_storage_ns': 130,
        'upper_tf_header_ns': 130,
        'start_offset_ns': 150,
        'start_offset_s': 1.5e-7,
    }
    monkeypatch.setattr(runner, '_tf_bootstrap_plan', lambda _root: plan)
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    (run_dir / 'amcl.log').write_text('', encoding='utf-8')

    def started(value):
        return {'name': 'started', 'steady_ns': value, 'wall_ns': value}
    prelude_command = [
        str(runner.ROS2), 'bag', 'play', str(tmp_path), '--storage', 'mcap',
        '--topics', '/odom', '/tf', '/tf_static', '--clock-topics-all',
        '--disable-keyboard-controls', '--playback-duration', '1.5e-07',
        '--rate', '2.0']
    main_command = [
        str(runner.ROS2), 'bag', 'play', str(tmp_path), '--storage', 'mcap',
        '--topics', '/scan', '/odom', '/tf', '/tf_static',
        '--clock-topics-all', '--start-paused', '--disable-keyboard-controls',
        '--start-offset', '1.5e-07', '--playback-duration', '30.0',
        '--rate', '2.0']
    processes = [
        {'name': name, 'returncode': 0, 'survivors': [],
         'started': started(value), 'command': command}
        for name, value, command in (
            ('map_server', 1, []), ('amcl', 2, []), ('observer', 3, []),
            ('resource_sampler', 4, []),
            ('tf_prelude_player', 10, prelude_command),
            ('player', 30, main_command))]
    evidence = {
        'tf_bootstrap': plan,
        'amcl_tf_error_counts': {
            marker: 0 for marker in runner.AMCL_TF_ERROR_MARKERS},
        'playback_rate': 2.0, 'prefix_s': 30.0,
        'events': [{'name': 'tf_prelude_completed', 'steady_ns': 20}],
        'teardown': processes,
        'observer': {
            'scan_header_stamps_ns': [100],
            'events': [
                {'name': 'initialpose_published', 'steady_ns': 40,
                 'ros_ns': 40},
                {'name': 'first_scan_received', 'steady_ns': 50,
                 'ros_ns': 50},
                {'name': 'first_post_scan_cloud_received', 'steady_ns': 60,
                 'ros_ns': 60},
            ],
        },
    }
    runner._validate_replay_bootstrap(evidence, run_dir, tmp_path)
    (run_dir / 'amcl.log').write_text(
        'Message Filter dropping message', encoding='utf-8')
    with pytest.raises(ValueError, match='transform lookup'):
        runner._validate_replay_bootstrap(evidence, run_dir, tmp_path)


def _artifact(tmp_path: Path, seeds=(11,)) -> Path:
    sanitized = tmp_path / 'sanitized'
    sanitized.mkdir(parents=True)
    sanitized_manifest = sanitized / 'sanitizer_manifest.json'
    parity = {
        'message_count': 1, 'source_transform_count': 1,
        'output_transform_count': 1, 'source_ordered_digest': 'a' * 64,
        'output_ordered_digest': 'a' * 64}
    sanitized_value = {
        'schema_version': 1,
        'source': {'path': str(sanitized), 'mcap_size_bytes': 1,
                   'mcap_sha256': runner.REAL_BAG_SHA256,
                   'metadata_sha256': 'b' * 64},
        'output_topics': ['/scan', '/odom', '/tf', '/tf_static'],
        'input_topic_inventory': {'/scan': 1},
        'removed_map_to_odom_transforms': 7290,
        'tf_semantic_parity': {'dynamic_non_map': parity, 'static': parity},
        'topic_parity': {
            '/scan': {'message_count': 1, 'storage_stamp_sha256': 'c' * 64,
                      'header_stamp_sha256': 'd' * 64,
                      'payload_sha256': 'e' * 64},
            '/odom': {'message_count': 1, 'storage_stamp_sha256': 'f' * 64,
                      'header_stamp_sha256': '1' * 64,
                      'payload_sha256': '2' * 64},
            '/tf': {'message_count': 1, 'storage_stamp_sha256': '3' * 64},
            '/tf_static': {'message_count': 1,
                           'storage_stamp_sha256': '4' * 64}},
        'output_limit_bytes': 1024,
        'output': {'mcap_name': 'bag_0.mcap', 'mcap_size_bytes': 1,
                   'mcap_sha256': '5' * 64, 'metadata_sha256': '6' * 64},
    }
    sanitized_manifest.write_bytes(canonical_json_bytes(sanitized_value))
    root = tmp_path / 'artifact'
    root.mkdir(parents=True)
    sanitizer_snapshot = root / 'sanitizer_manifest_snapshot.json'
    sanitizer_snapshot.write_bytes(canonical_json_bytes(sanitized_value))
    upstream = root / 'upstream_lock_snapshot.json'
    upstream.write_bytes(canonical_json_bytes({'test': True}))
    contract_path = root / 'contract_snapshot.json'
    contract_path.write_bytes(canonical_json_bytes({
        'production_inputs': {}, 'harness_sources': {},
        'source_overlay': {
            'root': str(tmp_path / 'overlay'),
            'lock': {'sha256': sha256_file(upstream)}}}))
    install_lib = tmp_path / 'build/install/nav2_amcl/lib'
    executable = install_lib / 'nav2_amcl/amcl'
    executable.parent.mkdir(parents=True)
    executable.write_bytes(b'amcl')
    core = install_lib / 'libamcl_core.so'
    pf = install_lib / 'libpf_lib.so'
    core.write_bytes(b'core')
    pf.write_bytes(b'pf')
    loaded = {
        'amcl_executable': runner._identity(executable),
        'libamcl_core': runner._identity(core),
        'libpf_lib': runner._identity(pf),
        'rmw_library': runner._identity(Path('/usr/bin/true')),
    }
    build = root / 'build_attestation.json'
    build.write_bytes(canonical_json_bytes({
        'schema_version': 1, 'upstream_lock_sha256': sha256_file(upstream),
        'build_root': str(tmp_path / 'build'),
        'source_root': str(tmp_path / 'overlay/nav2_amcl'),
        'cmake_cache': runner._identity(Path('/usr/bin/true')),
        'build_returncode_file': runner._identity(Path('/usr/bin/true')),
        'install_files': {
            'amcl': loaded['amcl_executable'],
            'libamcl_core.so': loaded['libamcl_core'],
            'libpf_lib.so': loaded['libpf_lib']},
        'needed_sonames': {
            'amcl': ['libamcl_core.so'],
            'libamcl_core.so': ['libpf_lib.so']},
        'loaded_runtime': loaded,
    }))
    replay = root / 'replay_view_manifest.json'
    replay.write_bytes(canonical_json_bytes({
        'schema_version': 1,
        'source_sanitizer_manifest_sha256': sha256_file(sanitizer_snapshot),
        'bootstrap': {'test': True}, 'skipped_scan_count': 2,
        'prelude_topics': ['/odom', '/tf', '/tf_static'],
        'main_topics': ['/scan', '/odom', '/tf', '/tf_static'],
        'prelude': {}, 'main': {}, 'prelude_last_storage_ns': 10,
        'main_first_storage_ns': 20, 'main_last_scan_storage_ns': 30,
        'storage_gap_ns': 10, 'storage_overlap_count': 0,
    }))
    runs = []
    evidence_values = []
    for index, seed in enumerate(seeds, 1):
        run_dir = root / f'run_{index}'
        run_dir.mkdir()
        digest = 'same' if seed == 11 else 'different'
        cloud_count = 30 if len(seeds) == 3 else 1
        scan_count = 1937 if len(seeds) == 3 else 1
        stamps = list(range(1, scan_count + 1))
        clouds = [{
            'index': cloud_index, 'payload_sha256': digest,
            'triggering_scan_header_stamp_ns': cloud_index + 1,
            'pose_header_stamp_ns': cloud_index + 1,
            'pose_frame_id': 'map', 'frame_id': 'map',
            'particle_count': 1} for cloud_index in range(cloud_count)]
        process_names = (
            'player', 'tf_prelude_player', 'resource_sampler', 'observer',
            'amcl', 'map_server')
        evidence = {
            'schema_version': 1,
            'run_id': f'p0__seed_{seed}__attempt_{index}',
            'profile': 'P0', 'domain_id': 190 + index - 1,
            'status': 'PASS', 'failure': None, 'seed': seed,
            'events': [], 'operations': [{'returncode': 0}] * 6,
            'observer_source': runner._identity(Path('/usr/bin/true')),
            'observer': {
                'schema_version': 1,
                'run_id': f'p0__seed_{seed}__attempt_{index}', 'seed': seed,
                'max_clouds': cloud_count, 'publisher_matched_count': 1,
                'events': [], 'readiness': {}, 'initialpose_count': 1,
                'pre_initial_scan_count': 0, 'pre_initial_cloud_count': 0,
                'raw_cloud_received_count': cloud_count,
                'amcl_pose_received_count': cloud_count,
                'pending_cloud_count': 0, 'pending_pose_count': 0,
                'scan_count': scan_count, 'scan_header_stamps_ns': stamps,
                'scan_header_stamp_sha256': '7' * 64,
                'scan_payload_sha256': '8' * 64, 'clouds': clouds,
                'failure': None, 'done': True,
                'motion_command_applicability': 'NOT_APPLICABLE'},
            'scan_parity': {'message_count': scan_count},
            'sanitized_manifest': runner._identity(sanitized_manifest),
            'amcl_executable': runner._identity(executable),
            'map_yaml': runner._identity(Path('/usr/bin/true')),
            'params_file': runner._identity(Path('/usr/bin/true')),
            'output_mcap_count': 0, 'survivor_count': 0,
            'transform_lookup_drop_count': 0,
            'amcl_tf_error_counts': {
                marker: 0 for marker in runner.AMCL_TF_ERROR_MARKERS},
            'teardown': [], 'loaded_runtime': loaded,
            'prelude_observer': {}, 'tf_bootstrap': {'test': True},
            'prefix_s': 220.0, 'playback_rate': 2.0,
            'cmd_vel_publisher': 'NOT_APPLICABLE',
        }
        for name in (
                'amcl.log', 'initialpose.request', 'map_server.log',
                'observer.log', 'observer_state.json', 'player.log',
                'resource_sampler.log', 'tf_prelude_player.log'):
            (run_dir / name).write_text('', encoding='utf-8')
        resource = run_dir / 'amcl_resource.jsonl'
        resource.write_text('{}\n', encoding='utf-8')
        evidence['resource'] = {
            'file': runner._relative_identity(resource, run_dir)}
        for process_name in process_names:
            returncode = -2 if process_name == 'resource_sampler' else 0
            evidence['teardown'].append({
                'name': process_name, 'returncode': returncode,
                'survivors': [],
                'stop_stages': ([{'signal': 'SIGINT'}]
                                if process_name == 'resource_sampler' else []),
                'log': runner._relative_identity(
                    run_dir / (process_name + '.log')
                    if process_name != 'tf_prelude_player' else
                    run_dir / 'tf_prelude_player.log', run_dir)})
        path = run_dir / 'evidence.json'
        path.write_bytes(canonical_json_bytes(evidence))
        evidence_values.append(evidence)
        runs.append({'relative_path': str(path.relative_to(root)),
                     'size_bytes': path.stat().st_size,
                     'sha256': sha256_file(path)})
    mode = 'full' if len(seeds) == 3 else 'smoke'
    manifest = {
        'schema_version': 2, 'mode': mode,
        'claim_scope': runner.CLAIM_FULL if mode == 'full' else runner.CLAIM_SMOKE,
        'source_contract': runner._relative_identity(contract_path, root),
        'sanitizer_snapshot': runner._relative_identity(sanitizer_snapshot, root),
        'upstream_lock_snapshot': runner._relative_identity(upstream, root),
        'build_attestation': runner._relative_identity(build, root),
        'replay_view': runner._relative_identity(replay, root),
        'runtime': {
            'rmw_implementation': 'rmw_cyclonedds_cpp',
            'rmw_library': runner._identity(Path('/usr/bin/true')),
            'ros_localhost_only': '1',
            'domain_ids': [190 + index for index in range(len(seeds))],
            'amcl_executable': runner._identity(executable),
        },
        'runs': runs,
        'comparison': runner._comparison(evidence_values),
        'tree_bytes': runner._payload_tree_bytes(root),
        'tree_records': runner._tree_records(root),
        'tree_sha256': runner._tree_digest(runner._tree_records(root)),
    }
    (root / 'preflight_manifest.json').write_bytes(
        canonical_json_bytes(manifest))
    return root


def _refresh_manifest(root: Path) -> None:
    manifest_path = root / 'preflight_manifest.json'
    manifest = runner.strict_json_load(manifest_path)
    for key, filename in (
            ('sanitizer_snapshot', 'sanitizer_manifest_snapshot.json'),
            ('upstream_lock_snapshot', 'upstream_lock_snapshot.json'),
            ('build_attestation', 'build_attestation.json'),
            ('replay_view', 'replay_view_manifest.json')):
        manifest[key] = runner._relative_identity(root / filename, root)
    manifest['runs'] = [runner._relative_identity(
        root / f'run_{index}/evidence.json', root)
        for index in range(1, len(manifest['runs']) + 1)]
    manifest['tree_bytes'] = runner._payload_tree_bytes(root)
    manifest['tree_records'] = runner._tree_records(root)
    manifest['tree_sha256'] = runner._tree_digest(manifest['tree_records'])
    manifest_path.write_bytes(canonical_json_bytes(manifest))


def test_artifact_validator_accepts_smoke_and_exact_triple(
        monkeypatch, tmp_path):
    monkeypatch.setattr(
        runner, '_scan_parity',
        lambda value, _root: {'message_count': value['scan_count']})
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    assert runner.validate_smoke_artifact(_artifact(tmp_path / 'one'))
    assert runner.validate_full_artifact(_artifact(
        tmp_path / 'three', (11, 11, 23)))['comparison']['same_seed_equal']


def test_smoke_and_full_validators_are_not_interchangeable(
        monkeypatch, tmp_path):
    monkeypatch.setattr(runner, '_scan_parity',
                        lambda value, _root: {
                            'message_count': value['scan_count']})
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    smoke = _artifact(tmp_path / 'smoke')
    full = _artifact(tmp_path / 'full', (11, 11, 23))
    with pytest.raises(ValueError, match='version'):
        runner.validate_full_artifact(smoke)
    with pytest.raises(ValueError, match='version'):
        runner.validate_smoke_artifact(full)


def test_validator_is_self_contained_after_sanitized_scratch_is_removed(
        monkeypatch, tmp_path):
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    external = tmp_path / 'sanitized/sanitizer_manifest.json'
    external.unlink()
    external.parent.rmdir()
    assert runner.validate_smoke_artifact(root)


def test_validator_rejects_relinked_arbitrary_amcl_executable(
        monkeypatch, tmp_path):
    monkeypatch.setattr(runner, '_scan_parity',
                        lambda value, _root: {
                            'message_count': value['scan_count']})
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    manifest = runner.strict_json_load(root / 'preflight_manifest.json')
    build_path = root / 'build_attestation.json'
    build = runner.strict_json_load(build_path)
    arbitrary = runner._identity(Path('/usr/bin/true'))
    build['loaded_runtime']['amcl_executable'] = arbitrary
    build['install_files']['amcl'] = arbitrary
    build_path.write_bytes(canonical_json_bytes(build))
    manifest['runtime']['amcl_executable'] = arbitrary
    evidence_path = root / 'run_1/evidence.json'
    evidence = runner.strict_json_load(evidence_path)
    evidence['loaded_runtime']['amcl_executable'] = arbitrary
    evidence_path.write_bytes(canonical_json_bytes(evidence))
    (root / 'preflight_manifest.json').write_bytes(canonical_json_bytes(manifest))
    _refresh_manifest(root)
    with pytest.raises(ValueError, match='attested overlay build'):
        runner.validate_smoke_artifact(root)


@pytest.mark.parametrize('attack', ['unknown_key', 'missing_key', 'nan'])
def test_validator_rejects_noncanonical_json_schema(
        monkeypatch, tmp_path, attack):
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    evidence_path = root / 'run_1/evidence.json'
    evidence = runner.strict_json_load(evidence_path)
    if attack == 'unknown_key':
        evidence['shadow'] = 1
        evidence_path.write_bytes(canonical_json_bytes(evidence))
    elif attack == 'missing_key':
        del evidence['profile']
        evidence_path.write_bytes(canonical_json_bytes(evidence))
    else:
        evidence_path.write_text('{"value":NaN}', encoding='utf-8')
    _refresh_manifest(root)
    with pytest.raises((ValueError, KeyError), match='schema|non-finite'):
        runner.validate_smoke_artifact(root)


@pytest.mark.parametrize('attack', ['evidence', 'manifest', 'mcap', 'symlink'])
def test_artifact_validator_rejects_tamper(monkeypatch, tmp_path, attack):
    monkeypatch.setattr(
        runner, '_scan_parity',
        lambda value, _root: {'message_count': value['scan_count']})
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    if attack == 'evidence':
        (root / 'run_1/evidence.json').write_text('{}', encoding='utf-8')
    elif attack == 'manifest':
        manifest = runner.strict_json_load(root / 'preflight_manifest.json')
        manifest['tree_bytes'] += 1
        (root / 'preflight_manifest.json').write_bytes(
            canonical_json_bytes(manifest))
    elif attack == 'mcap':
        (root / 'run_1/shadow.mcap').write_bytes(b'not allowed')
    else:
        (root / 'link').symlink_to(root / 'run_1/evidence.json')
    with pytest.raises((ValueError, KeyError)):
        runner.validate_smoke_artifact(root)
