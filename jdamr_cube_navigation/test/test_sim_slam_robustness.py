"""Contract tests for the multi-seed one-factor SLAM matrix."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest


EVALUATION = Path(__file__).resolve().parents[1] / 'evaluation'
PROFILE = (EVALUATION / 'media' / 'corridor_localdds_armed_20260904T152036'
           / 'sensor_profile.json')
sys.path.insert(0, str(EVALUATION))

from run_sim_slam_robustness import (  # noqa: E402,I100
    aggregate,
    default_conditions,
    realized_fault_passed,
    SEEDS,
    validate_contract,
)


def test_default_matrix_is_exactly_five_seeds_and_one_factor():
    """Every robustness condition differs from the baseline in one field."""
    conditions = default_conditions(PROFILE)

    validate_contract(conditions, SEEDS, PROFILE)

    assert SEEDS == (11, 23, 42, 67, 89)
    assert len(conditions) >= 8
    assert {condition['changed_factor'] for condition in conditions} == {
        None,
        'lidar_update_rate_hz',
        'scan_stamp_jitter_max_s',
        'scan_range_noise_stddev_m',
        'scan_message_dropout_probability',
        'wheel_translation_scale',
        'wheel_slip_stddev_fraction',
        'tf_transport_delay_s',
        'scan_transport_delay_s',
    }
    assert sum(condition['changed_factor'] is None
               for condition in conditions) == 1


def test_contract_rejects_multi_factor_and_false_measured_provenance():
    """Unknown real-world quantities cannot silently become measurements."""
    conditions = default_conditions(PROFILE)
    broken = copy.deepcopy(conditions)
    broken[1]['fault_values']['wheel_translation_scale'] = 0.9
    with pytest.raises(ValueError, match='exactly one'):
        validate_contract(broken, SEEDS, PROFILE)

    broken = copy.deepcopy(conditions)
    wheel = next(condition for condition in broken
                 if condition['changed_factor'] == 'wheel_translation_scale')
    wheel['provenance']['wheel_translation_scale']['kind'] = (
        'measured_observation')
    with pytest.raises(ValueError, match='cannot be claimed'):
        validate_contract(broken, SEEDS, PROFILE)

    broken = copy.deepcopy(conditions)
    broken[1]['provenance'].pop(broken[1]['changed_factor'])
    with pytest.raises(ValueError, match='lacks provenance'):
        validate_contract(broken, SEEDS, PROFILE)

    broken = copy.deepcopy(conditions)
    duplicate = copy.deepcopy(broken[0])
    duplicate['label'] = 'second_baseline'
    broken.append(duplicate)
    with pytest.raises(ValueError, match='exactly one baseline'):
        validate_contract(broken, SEEDS, PROFILE)


def test_realized_fault_gate_uses_observed_counts_and_magnitudes():
    """Configured dropout alone is insufficient without an observed drop."""
    condition = next(
        item for item in default_conditions(PROFILE)
        if item['changed_factor'] == 'scan_message_dropout_probability')

    assert not realized_fault_passed(condition, {'scan_dropped': 0})
    assert realized_fault_passed(
        condition, {'scan_received': 100, 'scan_dropped': 5})

    wheel = next(
        item for item in default_conditions(PROFILE)
        if item['changed_factor'] == 'wheel_translation_scale')
    assert not realized_fault_passed(wheel, {
        'wheel_scale_samples': 100,
        'wheel_scale_mean': 0.95,
        'wheel_tf_received': 100,
        'tf_wheel_corrections': 0,
    })
    assert realized_fault_passed(wheel, {
        'wheel_scale_samples': 100,
        'wheel_scale_mean': 0.95,
        'wheel_tf_received': 100,
        'tf_wheel_corrections': 100,
    })


def test_aggregate_separates_performance_fail_from_invalid_evidence(tmp_path):
    """A completed poor run is FAIL; corrupt evidence remains INVALID."""
    condition = default_conditions(PROFILE)[0]
    generated = tmp_path / 'generated'
    generated.mkdir()
    profile = generated / 'baseline.urdf'
    fault = generated / 'baseline.fault.json'
    bridge = generated / 'bridge.yaml'
    world = tmp_path / 'world.sdf'
    config = tmp_path / 'cartographer.lua'
    for path, content in (
            (profile, '<robot/>'), (bridge, '[]\n'),
            (world, '<sdf/>'), (config, 'return {}')):
        path.write_text(content)
    fault_document = {
        'schema_version': 1, 'label': condition['label'],
        'changed_factor': None, 'values': condition['fault_values'],
        'provenance': condition['provenance'],
    }
    fault.write_text(json.dumps(fault_document))

    def record(path):
        return {'path': str(path.resolve()),
                'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}

    matrix = {
        'backend': 'cartographer', 'conditions': [condition],
        'seeds': list(SEEDS),
        'fixed': {
            'world': record(world), 'route': 'corridor',
            'spawn_xy_m': [-8.0, 0.0], 'corridor_distance_m': 14.0,
            'cartographer_config': record(config),
            'bridge_config': record(bridge),
            'software': {
                'git_sha': 'a' * 40, 'workspace_dirty': True,
                'workspace_diff_sha256': 'b' * 64,
            },
        },
        'sensor_profile': record(PROFILE),
        'generated_profiles': {condition['label']: record(profile)},
        'generated_faults': {condition['label']: record(fault)},
        'commands': [],
    }
    for index, seed in enumerate(SEEDS):
        run_id = f'{condition["label"]}__cartographer__seed_{seed}'
        run_dir = tmp_path / run_id
        run_dir.mkdir()
        bag = run_dir / 'bag.mcap'
        map_yaml = run_dir / 'map.yaml'
        map_pgm = run_dir / 'map.pgm'
        map_state = run_dir / 'map_state.pbstream'
        metrics_path = run_dir / 'metrics.json'
        fault_stats = run_dir / 'fault_stats.json'
        resource_samples = run_dir / 'resource_samples.jsonl'
        bag.write_bytes(b'mcap')
        map_yaml.write_text(
            f'image: {map_pgm.resolve()}\nresolution: 0.05\n')
        map_pgm.write_bytes(b'P5\n3 1\n255\n\x00\x80\xff')
        map_state.write_bytes(b'pbstream')
        samples = [
            {'monotonic_s': 1.0, 'label': 'backend',
             'cpu_pct_one_core': None, 'rss_mb': 90.0,
             'process_count': 1},
            {'monotonic_s': 1.5, 'label': 'backend',
             'cpu_pct_one_core': 50.0, 'rss_mb': 100.0,
             'process_count': 1},
        ]
        resource_samples.write_text(
            ''.join(json.dumps(row) + '\n' for row in samples))
        stats = {'seed': seed, 'profile': condition['fault_values'],
                 'scan_received': 10}
        fault_stats.write_text(json.dumps(stats))
        metrics = {
            'validity': {'valid': True},
            'completion': {'completed': index != 4,
                           'ground_truth_path_ratio': 0.98},
            'rpe': {'translation_rms_m': 0.03, 'yaw_rms_rad': 0.01},
            'ate': {'translation_rms_m': 0.2, 'yaw_rms_rad': 0.02},
            'resources': {
                'sampling_interval_s': 0.5,
                'scope': ('launched process tree; CPU percent may exceed '
                          '100 on multicore'),
                'by_process_group': {'backend': {
                    'samples': 2, 'cpu_samples': 1,
                    'cpu_mean_pct_one_core': 50.0,
                    'cpu_p90_pct_one_core': 50.0,
                    'cpu_peak_pct_one_core': 50.0,
                    'rss_peak_mb': 100.0,
                }},
            },
        }
        metrics_path.write_text(json.dumps(metrics))
        map_images = [{**record(path), 'bytes': path.stat().st_size}
                      for path in (map_yaml, map_pgm)]
        execution = {
            'status': 'complete', 'failure': None, 'run_label': run_id,
            'backend': 'cartographer', 'seed': seed, 'route': 'corridor',
            'spawn_xy_m': [-8.0, 0.0], 'corridor_distance_m': 14.0,
            'profile': {
                'label': condition['label'], 'urdf': str(profile.resolve()),
                'sha256': record(profile)['sha256']},
            'fault_profile': {**record(fault), 'document': fault_document},
            'bridge_config': record(bridge), 'world': record(world),
            'cartographer_config': record(config),
            'exit_codes': {
                'route': 0, 'map_state': 0, 'map_render': 0,
                'recorder': 0, 'backend': 0, 'fault_injector': 0,
                'gazebo': 0},
            'teardown_remaining_processes': {
                'recorder': [], 'backend': [], 'fault_injector': [],
                'gazebo': []},
            'map_artifact': {**record(map_state),
                             'bytes': map_state.stat().st_size},
            'map_images': map_images,
            'resource_samples': {**record(resource_samples),
                                 'samples': len(samples)},
            'fault_realization': {**record(fault_stats), 'observed': stats},
        }
        (run_dir / 'execution_manifest.json').write_text(
            json.dumps(execution))
        outcome = 'PASS' if index != 4 else 'FAIL'
        artifacts = [
            {'kind': path.suffix.lstrip('.') or 'artifact',
             'uri': str(path.resolve()),
             'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
            for path in (bag, metrics_path, resource_samples, map_state,
                         map_yaml, map_pgm, fault_stats)]
        run_manifest = {
            'schema_version': 1, 'run_id': run_id, 'phase': 'phase4b',
            'purpose': 'Cartographer robustness evaluation',
            'environment': 'simulation',
            'captured_at': '2026-09-05T00:00:00+00:00',
            'source': {'dataset_id': run_id, 'bag_uri': str(bag.resolve()),
                       'bag_sha256': record(bag)['sha256']},
            'software': {
                'git_sha': 'a' * 40, 'workspace_dirty': True,
                'workspace_diff_sha256': 'b' * 64, 'ros_distro': 'jazzy',
                'packages': {'cartographer_ros': 'jazzy'}},
            'clock': {'mode': 'simulation', 'source': '/clock',
                      'verified': True, 'offset_uncertainty_ms': 0.0},
            'sensors': [{'name': 'lidar', 'topic': '/scan',
                         'frame_id': 'laser_link',
                         'timestamp_source': 'simulation_clock',
                         'calibration_id': 'test'}],
            'tf_authority': {
                'map_to_odom_publishers': ['cartographer_node'],
                'odom_to_base_publishers': ['sim_fault_injector']},
            'config': {'backend': 'cartographer',
                       'config_sha256': record(config)['sha256'],
                       'map_id': None},
            'motion': {
                'initial_pose': {'x': -8.0, 'y': 0.0, 'yaw': 0.0},
                'max_linear_mps': 0.35, 'max_angular_rps': 0.5,
                'schedule_id': 'corridor_14m'},
            'randomization': {'seed': seed,
                              'theta': condition['fault_values']},
            'outcome': {
                'status': outcome,
                'terminal_reason': (
                    'finite metrics and ground-truth completion gates passed'
                    if outcome == 'PASS'
                    else 'ground-truth completion gate failed'),
                'contact_count': None, 'invalid_goal_count': None},
            'artifacts': artifacts,
        }
        (run_dir / 'run_manifest.json').write_text(
            json.dumps(run_manifest))
        matrix['commands'].append([
            'python3', 'runner.py', '--run-id', run_id,
            '--profile-urdf', str(profile.resolve()),
            '--fault-profile', str(fault.resolve()),
            '--world', str(world.resolve()), '--seed', str(seed)])

    result = aggregate(matrix, tmp_path)

    summary = result['conditions'][condition['label']]
    assert result['overall'] == 'FAIL'
    assert summary['status'] == 'FAIL'
    assert summary['valid_seed_count'] == 5
    assert summary['pass_count'] == 4
    assert summary['fail_count'] == 1
    assert summary['invalid_count'] == 0
    assert summary['completion_rate'] == 0.8
    assert summary['ate_translation_rms_m']['count'] == 5
    assert summary['runs'][-1]['status'] == 'FAIL'

    first_run = tmp_path / (
        f'{condition["label"]}__cartographer__seed_{SEEDS[0]}')
    metrics = json.loads((first_run / 'metrics.json').read_text())
    metrics['resources']['by_process_group']['backend']['cpu_samples'] = 0
    (first_run / 'metrics.json').write_text(json.dumps(metrics))

    missing_resource = aggregate(matrix, tmp_path)

    assert missing_resource['overall'] == 'INCOMPLETE'
    assert missing_resource['conditions'][condition['label']][
        'runs'][0]['status'] == 'INVALID'

    execution_path = first_run / 'execution_manifest.json'
    execution = json.loads(execution_path.read_text())
    execution.pop('teardown_remaining_processes')
    execution_path.write_text(json.dumps(execution))

    missing_teardown = aggregate(matrix, tmp_path)

    assert missing_teardown['conditions'][condition['label']][
        'runs'][0]['execution_ok'] is False
