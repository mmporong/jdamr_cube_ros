#!/usr/bin/env python3
"""Regression tests for the G002 AMCL determinism preflight."""

from pathlib import Path
from types import SimpleNamespace

from amcl_fault_contract import canonical_json_bytes, sha256_file
from amcl_fault_contract import strict_json_loads
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


def _scan_parity(value, _root):
    stamps = value['scan_header_stamps_ns']
    return {
        'message_count': value['scan_count'],
        'header_stamp_sha256': value['scan_header_stamp_sha256'],
        'storage_stamp_sha256': '9' * 64,
        'payload_sha256': value['scan_payload_sha256'],
        'first_header_stamp_ns': stamps[0],
        'last_header_stamp_ns': stamps[-1],
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


@pytest.mark.parametrize('payload', [
    '{"value":NaN}', '{"value":Infinity}', '{"value":-Infinity}',
    '{"value":1,"value":2}',
])
def test_strict_json_rejects_nonfinite_and_duplicate_keys(payload):
    with pytest.raises(ValueError, match='non-finite|duplicate'):
        strict_json_loads(payload)


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


@pytest.mark.parametrize('mode,seeds,clouds,prefix_s', [
    ('smoke', [11], 1, 30.0),
    ('full', [11, 11, 23], 30, 220.0),
])
def test_mode_contract_accepts_only_exact_execution_plan(
        mode, seeds, clouds, prefix_s):
    args = SimpleNamespace(
        mode=mode, seeds=seeds, max_clouds=clouds, prefix_s=prefix_s,
        playback_rate=2.0)
    assert runner._mode_contract(args)[0] == mode
    args.max_clouds += 1
    with pytest.raises(ValueError, match='execution contract'):
        runner._mode_contract(args)


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
    dynamic_parity = {
        'message_count': runner.REAL_BAG_TOPIC_INVENTORY['/tf'],
        'source_transform_count': 1, 'output_transform_count': 1,
        'source_ordered_digest': 'a' * 64,
        'output_ordered_digest': 'a' * 64}
    static_parity = {
        'message_count': runner.REAL_BAG_TOPIC_INVENTORY['/tf_static'],
        'source_transform_count': 1, 'output_transform_count': 1,
        'source_ordered_digest': 'b' * 64,
        'output_ordered_digest': 'b' * 64}
    (sanitized / 'bag_0.mcap').write_bytes(b'mcap')
    (sanitized / 'metadata.yaml').write_bytes(b'metadata')
    sanitized_value = {
        'schema_version': 1,
        'source': {'path': str(sanitized), 'mcap_size_bytes': 1,
                   'mcap_sha256': runner.REAL_BAG_SHA256,
                   'metadata_sha256': 'b' * 64},
        'output_topics': ['/scan', '/odom', '/tf', '/tf_static'],
        'input_topic_inventory': dict(runner.REAL_BAG_TOPIC_INVENTORY),
        'removed_map_to_odom_transforms': 7290,
        'tf_semantic_parity': {
            'dynamic_non_map': dynamic_parity, 'static': static_parity},
        'topic_parity': {
            '/scan': {'message_count': runner.REAL_BAG_TOPIC_INVENTORY['/scan'],
                      'storage_stamp_sha256': 'c' * 64,
                      'header_stamp_sha256': 'd' * 64,
                      'payload_sha256': 'e' * 64},
            '/odom': {'message_count': runner.REAL_BAG_TOPIC_INVENTORY['/odom'],
                      'storage_stamp_sha256': 'f' * 64,
                      'header_stamp_sha256': '1' * 64,
                      'payload_sha256': '2' * 64},
            '/tf': {'message_count': runner.REAL_BAG_TOPIC_INVENTORY['/tf'],
                    'storage_stamp_sha256': '3' * 64},
            '/tf_static': {
                           'message_count': runner.REAL_BAG_TOPIC_INVENTORY[
                               '/tf_static'],
                           'storage_stamp_sha256': '4' * 64}},
        'output_limit_bytes': 1024,
        'output': {
            'mcap_name': 'bag_0.mcap', 'mcap_size_bytes': 4,
            'mcap_sha256': sha256_file(sanitized / 'bag_0.mcap'),
            'metadata_sha256': sha256_file(sanitized / 'metadata.yaml')},
    }
    sanitized_manifest.write_bytes(canonical_json_bytes(sanitized_value))
    root = tmp_path / 'artifact'
    root.mkdir(parents=True)
    sanitizer_snapshot = root / 'sanitizer_manifest_snapshot.json'
    sanitizer_snapshot.write_bytes(canonical_json_bytes(sanitized_value))
    overlay_source = tmp_path / 'overlay/nav2_amcl'
    overlay_source.mkdir(parents=True)
    (overlay_source / 'source.cpp').write_bytes(b'source')
    source_tree = runner._source_tree(overlay_source)
    upstream = root / 'upstream_lock_snapshot.json'
    upstream_value = {
        'schema_version': 1,
        'scope': 'evaluation-only; not a production install', 'upstream': {
            'commit': 'test', 'archive_url': runner.UPSTREAM_ARCHIVE_URL,
            'archive_size_bytes': runner.UPSTREAM_ARCHIVE_SIZE_BYTES,
            'archive_sha256': runner.UPSTREAM_ARCHIVE_SHA256,
            'license': {'relative_path': 'LICENSE', 'size_bytes': 1,
                        'sha256': runner.UPSTREAM_LICENSE_SHA256}},
        'baseline_tree': source_tree, 'overlay_tree': source_tree,
        'changed_files': [], 'unchanged_pf_c': True,
        'patch_classification': (
            'upstream random_seed surface plus local pf_pdf corrective')}
    upstream.write_bytes(canonical_json_bytes(upstream_value))
    params = tmp_path / 'nav2_params.yaml'
    params.write_bytes(canonical_json_bytes({
        'amcl': {'ros__parameters': dict(runner.P0_PARAMS)}}))
    contract_path = root / 'contract_snapshot.json'
    contract_path.write_bytes(canonical_json_bytes({
        'axis_a': {'map': {'yaml': runner._identity(Path('/usr/bin/true'))}},
        'profiles': {'P0': dict(runner.P0_PARAMS)},
        'production_inputs': {
            'production_params': runner._identity(params)},
        'harness_sources': {
            'amcl_particle_observer.py': runner._identity(Path('/usr/bin/true'))},
        'source_overlay': {
            'root': str(tmp_path / 'overlay'),
            'lock': runner._identity(upstream), 'changed_files': []}}))
    install_lib = tmp_path / 'build/install/nav2_amcl/lib'
    executable = install_lib / 'nav2_amcl/amcl'
    executable.parent.mkdir(parents=True)
    executable.write_bytes(b'amcl')
    core = install_lib / 'libamcl_core.so'
    pf = install_lib / 'libpf_lib.so'
    core.write_bytes(b'core')
    pf.write_bytes(b'pf')
    rmw = tmp_path / 'runtime/librmw_cyclonedds_cpp.so'
    rmw.parent.mkdir()
    rmw.write_bytes(b'rmw')
    loaded = {
        'amcl_executable': runner._identity(executable),
        'libamcl_core': runner._identity(core),
        'libpf_lib': runner._identity(pf),
        'rmw_library': runner._identity(rmw),
        'procfs_load_evidence': {
            'observation_method': 'proc_pid_exe_and_maps_before_replay',
            'executable_target': str(executable),
            'required_mapped_library_paths': {
                'libamcl_core': str(core), 'libpf_lib': str(pf),
                'rmw_library': str(rmw)}},
    }
    build = root / 'build_attestation.json'
    build.write_bytes(canonical_json_bytes({
        'schema_version': 1, 'upstream_lock_sha256': sha256_file(upstream),
        'build_root': str(tmp_path / 'build'),
        'source_root': str(tmp_path / 'overlay/nav2_amcl'),
        'overlay_tree_sha256': source_tree['tree_sha256'],
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
    smoke = len(seeds) == 1
    replay_scan_count = 1 if smoke else runner.FULL_SCAN_COUNT
    bootstrap = {
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
    replay = root / 'replay_view_manifest.json'
    replay.write_bytes(canonical_json_bytes({
        'schema_version': 1,
        'clock_contract': {
            'source': 'rosbag2_player_synthesized_clock',
            'handoff': 'SEQUENTIAL_DUAL_PLAYER_CLOCK_HANDOFF',
            'per_run_proof': 'run_evidence_clock_handoff'},
        'source_sanitizer_manifest_sha256': sha256_file(sanitizer_snapshot),
        'bootstrap': bootstrap, 'skipped_scan_count': 2,
        'prelude_topics': ['/odom', '/tf', '/tf_static'],
        'main_topics': ['/scan', '/odom', '/tf', '/tf_static'],
        'prelude': {'/odom': {
            'message_count': 1, 'storage_stamp_sha256': 'e' * 64,
            'raw_payload_sha256': 'f' * 64, 'tf_semantic_sha256': None}},
        'main': {'/scan': {
            'message_count': replay_scan_count,
            'storage_stamp_sha256': '9' * 64,
            'raw_payload_sha256': '8' * 64,
            'tf_semantic_sha256': None}},
        'prelude_last_storage_ns': 10,
        'main_first_storage_ns': 20, 'main_last_scan_storage_ns': 30,
        'storage_gap_ns': 10, 'storage_overlap_count': 0,
    }))
    runs = []
    evidence_values = []
    for index, seed in enumerate(seeds, 1):
        run_dir = root / f'run_{index}'
        run_dir.mkdir()
        digest = ('a' * 64) if seed == 11 else ('b' * 64)
        cloud_count = 30 if len(seeds) == 3 else 1
        scan_count = 1937 if len(seeds) == 3 else 1
        stamps = list(range(1, scan_count + 1))
        scan_header_sha = __import__('hashlib').sha256(''.join(
            f'{value}\n' for value in stamps).encode()).hexdigest()
        clouds = [{
            'header_stamp_ns': cloud_index + 1,
            'arrival_steady_ns': cloud_index + 10,
            'callback_ros_ns': cloud_index + 1,
            'index': cloud_index, 'payload_sha256': digest,
            'fifo_associated_pose_scan_header_stamp_ns': cloud_index + 1,
            'pose_header_stamp_ns': cloud_index + 1,
            'pose_frame_id': 'map', 'frame_id': 'map',
            'particle_count': 1,
            'pose_arrival_steady_ns': cloud_index + 10,
            'pose_callback_ros_ns': cloud_index + 1,
            'pose': [0.0] * 7, 'covariance': [0.0] * 36,
            'scan_arrival_steady_ns': cloud_index + 9,
            'scan_to_pose_steady_ns': 1,
            'cloud_stream_index': cloud_index,
            'pose_stream_index': cloud_index,
            'pair_arrival_delta_ns': 0,
        } for cloud_index in range(cloud_count)]
        callback_trace = [item for cloud_index in range(cloud_count) for item in (
            {'kind': 'particle_cloud', 'stream_index': cloud_index,
             'header_stamp_ns': cloud_index + 1,
             'arrival_steady_ns': cloud_index + 10},
            {'kind': 'amcl_pose', 'stream_index': cloud_index,
             'header_stamp_ns': cloud_index + 1,
             'arrival_steady_ns': cloud_index + 10})]
        prelude_clock_samples = [{'ros_ns': 100, 'arrival_steady_ns': 1}]
        final_clock_samples = [
            *prelude_clock_samples, {'ros_ns': 101, 'arrival_steady_ns': 2}]
        process_names = (
            'player', 'tf_prelude_player', 'resource_sampler', 'observer',
            'amcl', 'map_server')
        evidence = {
            'schema_version': 1,
            'run_id': f'p0__seed_{seed}__attempt_{index}',
            'profile': 'P0', 'domain_id': 190 + index - 1,
            'status': 'PASS', 'failure': None, 'seed': seed,
            'events': [], 'operations': [{
                'command': ['/opt/ros/jazzy/bin/ros2'],
                'started': {'name': 'request', 'steady_ns': 1, 'wall_ns': 1},
                'completed': {'name': 'response', 'steady_ns': 2, 'wall_ns': 2},
                'returncode': 0, 'output': '',
            } for _ in range(6)],
            'observer_source': runner._identity(Path('/usr/bin/true')),
            'observer': {
                'schema_version': 1,
                'run_id': f'p0__seed_{seed}__attempt_{index}', 'seed': seed,
                'max_clouds': cloud_count, 'publisher_matched_count': 1,
                'pose_publisher_matched_count': 1,
                'events': [], 'readiness': {
                    'map_count': 1, 'clock_count': 2, 'odom_count': 1,
                    'tf_count': 1, 'map_odom_tf_count': 1,
                    'tf_static_count': 1}, 'initialpose_count': 1,
                'pre_initial_scan_count': 0, 'pre_initial_cloud_count': 0,
                'raw_cloud_received_count': cloud_count,
                'amcl_pose_received_count': cloud_count,
                'pending_cloud_count': 0, 'pending_pose_count': 0,
                'scan_count': scan_count, 'scan_header_stamps_ns': stamps,
                'scan_header_stamp_sha256': scan_header_sha,
                'scan_payload_sha256': '8' * 64, 'clouds': clouds,
                'clock_samples': final_clock_samples,
                'callback_trace': callback_trace,
                'pairing_contract': runner.PAIRING_CONTRACT,
                'observation_qos_contract': runner.OBSERVATION_QOS_CONTRACT,
                'pose_causality': 'NOT_PROVEN',
                'failure': None, 'done': True,
                'motion_command_applicability': 'NOT_APPLICABLE'},
            'scan_parity': {
                'message_count': scan_count,
                'header_stamp_sha256': scan_header_sha,
                'storage_stamp_sha256': '9' * 64,
                'payload_sha256': '8' * 64,
                'first_header_stamp_ns': stamps[0],
                'last_header_stamp_ns': stamps[-1]},
            'sanitized_manifest': runner._identity(sanitized_manifest),
            'amcl_executable': runner._identity(executable),
            'map_yaml': runner._identity(Path('/usr/bin/true')),
            'params_file': runner._identity(params),
            'output_mcap_count': 0, 'survivor_count': 0,
            'transform_lookup_drop_count': 0,
            'amcl_tf_error_counts': {
                marker: 0 for marker in runner.AMCL_TF_ERROR_MARKERS},
            'teardown': [], 'loaded_runtime': loaded,
            'prelude_observer': {
                'schema_version': 1,
                'run_id': f'p0__seed_{seed}__attempt_{index}', 'seed': seed,
                'max_clouds': cloud_count, 'publisher_matched_count': 1,
                'pose_publisher_matched_count': 1,
                'events': [], 'readiness': {
                    'map_count': 1, 'clock_count': 1, 'odom_count': 1,
                    'tf_count': 1, 'map_odom_tf_count': 0,
                    'tf_static_count': 1},
                'initialpose_count': 0, 'pre_initial_scan_count': 0,
                'pre_initial_cloud_count': 0, 'raw_cloud_received_count': 0,
                'amcl_pose_received_count': 0, 'pending_cloud_count': 0,
                'pending_pose_count': 0, 'scan_count': 0,
                'scan_header_stamps_ns': [],
                'scan_header_stamp_sha256': __import__('hashlib').sha256().hexdigest(),
                'scan_payload_sha256': __import__('hashlib').sha256().hexdigest(),
                'clock_samples': prelude_clock_samples,
                'callback_trace': [],
                'pairing_contract': runner.PAIRING_CONTRACT,
                'observation_qos_contract': runner.OBSERVATION_QOS_CONTRACT,
                'pose_causality': 'NOT_PROVEN',
                'clouds': [], 'failure': None, 'done': False,
                'motion_command_applicability': 'NOT_APPLICABLE'},
            'tf_bootstrap': bootstrap,
            'prefix_s': (runner.FULL_PREFIX_S if len(seeds) == 3 else
                         runner.SMOKE_PREFIX_S),
            'playback_rate': runner.PLAYBACK_RATE,
            'cmd_vel_publisher': 'NOT_APPLICABLE',
        }
        observer_value = evidence.pop('observer')
        evidence['clock_handoff'] = runner._clock_handoff(
            evidence['prelude_observer'], observer_value)
        for name in (
                'amcl.log', 'initialpose.request', 'map_server.log',
                'observer.log', 'player.log',
                'resource_sampler.log', 'tf_prelude_player.log'):
            (run_dir / name).write_text('', encoding='utf-8')
        observer_state = run_dir / 'observer_state.json'
        observer_state.write_bytes(canonical_json_bytes(observer_value))
        evidence['observer_state'] = runner._relative_identity(
            observer_state, run_dir)
        resource = run_dir / 'amcl_resource.jsonl'
        resource.write_bytes(canonical_json_bytes({
            'monotonic_s': 1.0, 'cpu_total_s': 0.0,
            'cpu_pct_one_core': None, 'rss_mb': 0.0,
            'process_count': 1}))
        evidence['resource'] = {
            'sample_count': 1, 'cpu_total_start_s': 0.0,
            'cpu_total_end_s': 0.0, 'cpu_seconds': 0.0,
            'cpu_pct_median': 0.0, 'cpu_pct_p95': 0.0,
            'max_rss_mb': 0.0,
            'file': runner._relative_identity(resource, run_dir)}
        for process_name in process_names:
            returncode = -2 if process_name == 'resource_sampler' else 0
            evidence['teardown'].append({
                'name': process_name, 'returncode': returncode,
                'pid': index, 'pgid': index, 'command': ['/usr/bin/true'],
                'started': {'name': 'started', 'steady_ns': 1, 'wall_ns': 1},
                'survivors': [],
                'stop_stages': ([{'signal': 'SIGINT', 'steady_ns': 2}]
                                if process_name == 'resource_sampler' else []),
                'log': runner._relative_identity(
                    run_dir / (process_name + '.log')
                    if process_name != 'tf_prelude_player' else
                    run_dir / 'tf_prelude_player.log', run_dir)})
        path = run_dir / 'evidence.json'
        path.write_bytes(canonical_json_bytes(evidence))
        evidence_values.append({**evidence, 'observer': observer_value})
        runs.append({'relative_path': str(path.relative_to(root)),
                     'size_bytes': path.stat().st_size,
                     'sha256': sha256_file(path)})
    mode = 'full' if len(seeds) == 3 else 'smoke'
    manifest = {
        'schema_version': 3, 'mode': mode,
        'claim_scope': runner.CLAIM_FULL if mode == 'full' else runner.CLAIM_SMOKE,
        'claim_boundary': runner.CLAIM_BOUNDARY,
        'pairing_contract': runner.PAIRING_CONTRACT,
        'observation_qos_contract': runner.OBSERVATION_QOS_CONTRACT,
        'pose_causality': 'NOT_PROVEN',
        'source_contract': runner._relative_identity(contract_path, root),
        'sanitized_input': runner._tree_manifest(sanitized),
        'sanitizer_snapshot': runner._relative_identity(sanitizer_snapshot, root),
        'upstream_lock_snapshot': runner._relative_identity(upstream, root),
        'build_attestation': runner._relative_identity(build, root),
        'replay_view': runner._relative_identity(replay, root),
        'runtime': {
            'rmw_implementation': 'rmw_cyclonedds_cpp',
            'rmw_library': runner._identity(rmw),
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
    manifest['source_contract'] = runner._relative_identity(
        root / 'contract_snapshot.json', root)
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


def _rewrite_observer_state(root: Path, mutate, run_index: int = 1) -> None:
    state_path = root / f'run_{run_index}/observer_state.json'
    state = runner.strict_json_load(state_path)
    mutate(state)
    state_path.write_bytes(canonical_json_bytes(state))
    evidence_path = root / f'run_{run_index}/evidence.json'
    evidence = runner.strict_json_load(evidence_path)
    evidence['observer_state'] = runner._relative_identity(
        state_path, state_path.parent)
    evidence_path.write_bytes(canonical_json_bytes(evidence))
    _refresh_manifest(root)


def test_full_large_clock_state_is_referenced_once_under_run_cap(
        monkeypatch, tmp_path):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_scan_parity', _scan_parity)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path, (11, 11, 23))
    for run_index in range(1, 4):
        state_path = root / f'run_{run_index}/observer_state.json'
        state = runner.strict_json_load(state_path)
        prelude_prefix = state['clock_samples'][:1]
        state['clock_samples'] = [
            *prelude_prefix,
            *({'ros_ns': value + 99, 'arrival_steady_ns': value}
              for value in range(2, 25_001)),
        ]
        state['readiness']['clock_count'] = len(state['clock_samples'])
        state_path.write_bytes(canonical_json_bytes(state))
        assert (2 * state_path.stat().st_size >
                runner.STORAGE_LIMITS['run_output_limit_bytes'])
        evidence_path = root / f'run_{run_index}/evidence.json'
        evidence = runner.strict_json_load(evidence_path)
        evidence['observer_state'] = runner._relative_identity(
            state_path, state_path.parent)
        evidence['clock_handoff'] = runner._clock_handoff(
            evidence['prelude_observer'], state)
        evidence_path.write_bytes(canonical_json_bytes(evidence))
    _refresh_manifest(root)
    for run_index in range(1, 4):
        assert (runner._tree_bytes(root / f'run_{run_index}') <=
                runner.STORAGE_LIMITS['run_output_limit_bytes'])
    assert runner.validate_full_artifact(root)


def test_validator_rejects_observer_state_reference_identity_drift(
        monkeypatch, tmp_path):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_scan_parity', _scan_parity)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    state_path = root / 'run_1/observer_state.json'
    state_path.write_bytes(state_path.read_bytes() + b' ')
    _refresh_manifest(root)
    with pytest.raises(ValueError, match='reference identity drift'):
        runner.validate_smoke_artifact(root)


def _mock_replay_recompute(monkeypatch):
    monkeypatch.setattr(
        runner, '_validate_overlay_snapshot',
        lambda _contract, lock: lock['overlay_tree'])
    monkeypatch.setattr(
        runner, '_replay_view_manifest',
        lambda sanitized_root, _bootstrap, _last_scan: runner.strict_json_load(
            sanitized_root.parent / 'artifact/replay_view_manifest.json'))


def test_artifact_validator_accepts_smoke_and_exact_triple(
        monkeypatch, tmp_path):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(
        runner, '_scan_parity', _scan_parity)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    assert runner.validate_smoke_artifact(_artifact(tmp_path / 'one'))
    assert runner.validate_full_artifact(_artifact(
        tmp_path / 'three', (11, 11, 23)))['comparison']['same_seed_equal']


def test_smoke_and_full_validators_are_not_interchangeable(
        monkeypatch, tmp_path):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_scan_parity', _scan_parity)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    smoke = _artifact(tmp_path / 'smoke')
    full = _artifact(tmp_path / 'full', (11, 11, 23))
    with pytest.raises(ValueError, match='inventory|version'):
        runner.validate_full_artifact(smoke)
    with pytest.raises(ValueError, match='inventory|version'):
        runner.validate_smoke_artifact(full)


def test_validator_is_self_contained_after_sanitized_scratch_is_removed(
        monkeypatch, tmp_path):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    external = tmp_path / 'sanitized/sanitizer_manifest.json'
    for path in external.parent.iterdir():
        path.unlink()
    external.parent.rmdir()
    assert runner.validate_smoke_artifact(root)


def test_validator_rejects_relinked_arbitrary_amcl_executable(
        monkeypatch, tmp_path):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_scan_parity', _scan_parity)
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
    with pytest.raises(ValueError, match='attested overlay build|procfs'):
        runner.validate_smoke_artifact(root)


@pytest.mark.parametrize('name', ['libamcl_core', 'libpf_lib', 'rmw_library'])
def test_validator_rejects_relinked_runtime_library(
        monkeypatch, tmp_path, name):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_scan_parity', _scan_parity)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    build_path = root / 'build_attestation.json'
    build = runner.strict_json_load(build_path)
    replacement = Path('/usr/bin/false') if name == 'rmw_library' else Path('/usr/bin/true')
    build['loaded_runtime'][name] = runner._identity(replacement)
    if name == 'libamcl_core':
        build['install_files']['libamcl_core.so'] = build['loaded_runtime'][name]
    elif name == 'libpf_lib':
        build['install_files']['libpf_lib.so'] = build['loaded_runtime'][name]
    build_path.write_bytes(canonical_json_bytes(build))
    evidence_path = root / 'run_1/evidence.json'
    evidence = runner.strict_json_load(evidence_path)
    evidence['loaded_runtime'][name] = build['loaded_runtime'][name]
    evidence_path.write_bytes(canonical_json_bytes(evidence))
    manifest = runner.strict_json_load(root / 'preflight_manifest.json')
    if name == 'rmw_library':
        manifest['runtime']['rmw_library'] = build['loaded_runtime'][name]
    (root / 'preflight_manifest.json').write_bytes(canonical_json_bytes(manifest))
    _refresh_manifest(root)
    with pytest.raises(
            ValueError,
            match='attested overlay build|identity drift|RMW library|procfs'):
        runner.validate_smoke_artifact(root)


def test_validator_rejects_sanitized_leaf_and_tree_drift(
        monkeypatch, tmp_path):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_scan_parity', _scan_parity)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    sanitizer_path = root / 'sanitizer_manifest_snapshot.json'
    sanitizer = runner.strict_json_load(sanitizer_path)
    sanitizer['output']['mcap_size_bytes'] += 1
    sanitizer_path.write_bytes(canonical_json_bytes(sanitizer))
    _refresh_manifest(root)
    with pytest.raises(ValueError, match='leaf binding'):
        runner.validate_smoke_artifact(root)

    root = _artifact(tmp_path / 'tree')
    manifest_path = root / 'preflight_manifest.json'
    manifest = runner.strict_json_load(manifest_path)
    manifest['sanitized_input']['tree_sha256'] = '0' * 64
    manifest_path.write_bytes(canonical_json_bytes(manifest))
    with pytest.raises(ValueError, match='inventory or digest'):
        runner.validate_smoke_artifact(root)


def test_overlay_validator_rejects_fixture_commit_and_empty_changed_files(
        tmp_path):
    root = _artifact(tmp_path)
    contract_path = root / 'contract_snapshot.json'
    lock_path = root / 'upstream_lock_snapshot.json'
    contract = runner.strict_json_load(contract_path)
    lock = runner.strict_json_load(lock_path)
    changed = sorted(runner.OVERLAY_CHANGED_FILES)
    lock['changed_files'] = changed
    contract['source_overlay']['changed_files'] = changed
    lock_path.write_bytes(canonical_json_bytes(lock))
    contract['source_overlay']['lock'] = runner._identity(lock_path)
    with pytest.raises(ValueError, match='canonical identity'):
        runner._validate_overlay_snapshot(contract, lock)

    lock['upstream']['commit'] = runner.UPSTREAM_COMMIT
    lock['changed_files'] = []
    contract['source_overlay']['changed_files'] = []
    lock_path.write_bytes(canonical_json_bytes(lock))
    contract['source_overlay']['lock'] = runner._identity(lock_path)
    contract_path.write_bytes(canonical_json_bytes(contract))
    with pytest.raises(ValueError, match='snapshot binding'):
        runner._validate_overlay_snapshot(contract, lock)


def test_validator_rejects_replay_view_scan_prefix_drift(
        monkeypatch, tmp_path):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_scan_parity', _scan_parity)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    replay_path = root / 'replay_view_manifest.json'
    replay = runner.strict_json_load(replay_path)
    replay['main']['/scan']['raw_payload_sha256'] = '0' * 64
    replay_path.write_bytes(canonical_json_bytes(replay))
    _refresh_manifest(root)
    with pytest.raises(ValueError, match='accepted scan prefix'):
        runner.validate_smoke_artifact(root)


def test_validator_rejects_p0_params_contract_and_evidence_drift(
        monkeypatch, tmp_path):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_scan_parity', _scan_parity)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path / 'contract')
    contract_path = root / 'contract_snapshot.json'
    contract = runner.strict_json_load(contract_path)
    contract['profiles']['P0']['max_particles'] += 1
    contract_path.write_bytes(canonical_json_bytes(contract))
    _refresh_manifest(root)
    with pytest.raises(ValueError, match='P0 prepared contract'):
        runner.validate_smoke_artifact(root)

    root = _artifact(tmp_path / 'evidence')
    evidence_path = root / 'run_1/evidence.json'
    evidence = runner.strict_json_load(evidence_path)
    evidence['params_file'] = runner._identity(Path('/usr/bin/true'))
    evidence_path.write_bytes(canonical_json_bytes(evidence))
    _refresh_manifest(root)
    with pytest.raises(ValueError, match='source or executable identity'):
        runner.validate_smoke_artifact(root)


@pytest.mark.parametrize(
    'attack', ['drop', 'reorder', 'window', 'claim', 'boundary', 'qos'])
def test_validator_rejects_fifo_pairing_evidence_drift(
        monkeypatch, tmp_path, attack):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_scan_parity', _scan_parity)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    if attack in {'claim', 'boundary', 'qos'}:
        manifest_path = root / 'preflight_manifest.json'
        manifest = runner.strict_json_load(manifest_path)
        if attack == 'claim':
            manifest['pose_causality'] = 'PROVEN'
        elif attack == 'boundary':
            manifest['claim_boundary'] = 'DROP_FREE'
        else:
            manifest['observation_qos_contract']['particle_cloud'] = (
                'RELIABLE')
        manifest_path.write_bytes(canonical_json_bytes(manifest))
    else:
        if attack == 'drop':
            _rewrite_observer_state(
                root, lambda state: state['callback_trace'].pop())
        elif attack == 'reorder':
            _rewrite_observer_state(
                root, lambda state: state['callback_trace'][0].__setitem__(
                    'kind', 'amcl_pose'))
        else:
            _rewrite_observer_state(
                root, lambda state: state['clouds'][0].__setitem__(
                    'pair_arrival_delta_ns', runner.PAIRING_WINDOW_NS + 1))
    with pytest.raises(ValueError, match='claim boundary|callback|cloud scalar'):
        runner.validate_smoke_artifact(root)


def test_validator_rejects_dual_player_clock_handoff_drift(
        monkeypatch, tmp_path):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_scan_parity', _scan_parity)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    _rewrite_observer_state(
        root, lambda state: state['clock_samples'][1].__setitem__(
            'ros_ns', 99))
    with pytest.raises(ValueError, match='clock monotonicity'):
        runner.validate_smoke_artifact(root)


def test_validator_rejects_dual_player_clock_prelude_prefix_mismatch(
        monkeypatch, tmp_path):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(runner, '_scan_parity', _scan_parity)
    monkeypatch.setattr(runner, '_validate_replay_bootstrap',
                        lambda *_args: None)
    root = _artifact(tmp_path)
    _rewrite_observer_state(
        root, lambda state: state['clock_samples'][0].__setitem__(
            'ros_ns', 99))
    with pytest.raises(ValueError, match='clock prelude prefix mismatch'):
        runner.validate_smoke_artifact(root)


def test_observer_qos_matches_upstream_publishers():
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    assert (observer.CLOUD_OBSERVATION_QOS.reliability ==
            ReliabilityPolicy.BEST_EFFORT)
    assert (observer.CLOUD_OBSERVATION_QOS.durability ==
            DurabilityPolicy.VOLATILE)
    assert (observer.POSE_OBSERVATION_QOS.reliability ==
            ReliabilityPolicy.RELIABLE)
    assert (observer.POSE_OBSERVATION_QOS.durability ==
            DurabilityPolicy.TRANSIENT_LOCAL)
    assert observer.POSE_OBSERVATION_QOS.depth == 1
    assert (observer.OBSERVATION_QOS_CONTRACT ==
            runner.OBSERVATION_QOS_CONTRACT)


def test_publish_cleanup_is_atomic_after_post_rename_failure(
        monkeypatch, tmp_path):
    stage = tmp_path / 'stage'
    output = tmp_path / 'output'
    stage.mkdir()
    (stage / 'proof.txt').write_text('proof', encoding='utf-8')
    calls = {'count': 0}

    def validator(_root):
        calls['count'] += 1
        if calls['count'] == 2:
            raise ValueError('post-rename validation failed')
        return {}

    with pytest.raises(ValueError, match='post-rename'):
        runner._publish_validated_stage(stage, output, validator, ({},))
    assert not output.exists()


def test_runtime_guard_rejects_free_space_and_log_cap(monkeypatch, tmp_path):
    monkeypatch.setattr(
        runner, 'current_free_bytes',
        lambda _path: runner.STORAGE_LIMITS['abort_free_floor_bytes'] - 1)
    with pytest.raises(RuntimeError, match='free-space'):
        runner._runtime_guard(tmp_path)
    monkeypatch.setattr(
        runner, 'current_free_bytes',
        lambda _path: runner.STORAGE_LIMITS['abort_free_floor_bytes'])
    (tmp_path / 'oversize.log').write_bytes(b'x' * (runner.MAX_LOG_BYTES + 1))
    with pytest.raises(RuntimeError, match='log exceeds'):
        runner._runtime_guard(tmp_path)


@pytest.mark.parametrize(
    'attack', ['unknown_key', 'missing_key', 'nan', 'duplicate', 'whitespace'])
def test_validator_rejects_noncanonical_json_schema(
        monkeypatch, tmp_path, attack):
    _mock_replay_recompute(monkeypatch)
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
    elif attack == 'nan':
        evidence_path.write_text('{"value":NaN}', encoding='utf-8')
    elif attack == 'duplicate':
        evidence_path.write_text('{"value":1,"value":2}', encoding='utf-8')
    else:
        evidence_path.write_text(
            __import__('json').dumps(evidence, indent=2), encoding='utf-8')
    _refresh_manifest(root)
    with pytest.raises(
            (ValueError, KeyError),
            match='schema|non-finite|duplicate|canonical|Expecting'):
        runner.validate_smoke_artifact(root)


@pytest.mark.parametrize('attack', ['evidence', 'manifest', 'mcap', 'symlink'])
def test_artifact_validator_rejects_tamper(monkeypatch, tmp_path, attack):
    _mock_replay_recompute(monkeypatch)
    monkeypatch.setattr(
        runner, '_scan_parity', _scan_parity)
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
