"""Test G004 Collision Monitor contracts before simulator execution."""

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

import yaml


ROOT = Path(__file__).resolve().parents[2]
EVALUATION = ROOT / 'jdamr_cube_navigation/evaluation'
sys.path.insert(0, str(EVALUATION))


def _load(name):
    path = EVALUATION / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_installed_collision_monitor_state_contract_is_locked():
    """The installed Jazzy message constants and fields stay explicit."""
    contract = _load('sim_collision_monitor_contract.py')
    derived = contract.derive_contract(20.0, 0.001)

    assert derived['collision_monitor_state_interface']['actions'] == {
        'DO_NOTHING': 0, 'STOP': 1, 'SLOWDOWN': 2,
        'APPROACH': 3, 'LIMIT': 4}
    assert derived['collision_monitor_state_interface']['fields'] == [
        'action_type', 'polygon_name']


def test_shortest_zero_false_stop_timeout_is_selected():
    """Synthetic traces reject 0.25 s and select 0.30 s before 0.40 s."""
    contract = _load('sim_collision_monitor_contract.py')

    assert contract.false_stop_count(
        0.25, contract.SYNTHETIC_VALID_SCAN_GAPS_S) > 0
    assert contract.false_stop_count(
        0.30, contract.SYNTHETIC_VALID_SCAN_GAPS_S) == 0
    assert contract.select_source_timeout_s() == 0.30


def test_timeout_clock_semantics_have_immutable_upstream_provenance():
    """The ROS-time source-age gate is bound to immutable Nav2 sources."""
    contract = _load('sim_collision_monitor_contract.py')
    provenance = contract.derive_contract(
        20.0, 0.001)['timeout_clock_provenance']

    assert len(provenance['commit']) == 40
    assert provenance['branch'] == 'jazzy'
    assert provenance['commit'] in provenance['scan_source']['url']
    assert provenance['commit'] in provenance['source_base']['url']
    assert len(provenance['scan_source']['sha256']) == 64
    assert len(provenance['source_base']['sha256']) == 64


def test_stop_deadline_is_derived_from_three_periods():
    """The deadline combines selected timeout, smoother period, and physics."""
    contract = _load('sim_collision_monitor_contract.py')

    assert contract.derive_stop_deadline_s(0.30, 20.0, 0.001) == pytest.approx(
        0.351)


def test_stop_zone_is_derived_from_motion_and_grid_contract():
    """The polygon includes grid, reaction, smoothing, and braking margins."""
    contract = _load('sim_collision_monitor_contract.py')
    zone = contract.derive_stop_zone(20.0)

    assert zone['front_m'] == pytest.approx(0.3261098)
    assert zone['rear_m'] == pytest.approx(-0.2653553)
    assert zone['half_width_m'] == pytest.approx(0.2353553)
    assert zone['derived']['reaction_time_s'] == pytest.approx(0.157525)
    derived = contract.derive_contract(20.0, 0.001)
    assert derived['sudden_obstacle'][
        'activation_surface_x_m'] == pytest.approx(0.3761098)
    assert derived['sudden_obstacle'][
        'activation_center_offset_x_m'] == pytest.approx(0.6261098)


def test_matrix_is_exact_three_by_five():
    """Every scenario has every fixed seed exactly once."""
    contract = _load('sim_collision_monitor_contract.py')
    matrix = contract.scenario_matrix()

    assert len(matrix) == 15
    assert len({item['run_id'] for item in matrix}) == 15
    assert {item['seed'] for item in matrix} == {11, 23, 42, 67, 89}


def test_scan_gate_contract_never_freezes_navigation_scan():
    """Only the Collision Monitor branch is freezeable."""
    contract = _load('sim_collision_monitor_contract.py')
    gate = contract.derive_contract(20.0, 0.001)['scan_gate']

    assert gate['input_topic'] == '/sim_raw/scan'
    assert gate['navigation_output_topic'] == '/scan'
    assert gate['collision_monitor_output_topic'] == (
        '/collision_monitor_scan')
    assert gate['navigation_output_freezable'] is False
    assert gate['collision_monitor_output_freezable'] is True


def test_prepare_writes_only_evaluation_overlay(tmp_path):
    """Preparation derives periods while leaving production bytes unchanged."""
    prepare = _load('prepare_sim_collision_monitor_run.py')
    production = tmp_path / 'nav2.yaml'
    world = tmp_path / 'world.sdf'
    production.write_bytes((
        ROOT / 'jdamr_cube_navigation/config/nav2_params.yaml').read_bytes())
    world.write_text(
        '<plugin filename="gz-sim-contact-system"/>'
        '<max_step_size>0.001</max_step_size>')
    before = production.read_bytes(), world.read_bytes()

    contract = prepare.prepare(production, world, tmp_path / 'output')
    overlay = yaml.safe_load(
        (tmp_path / 'output/collision_monitor_overlay.yaml').read_text())

    assert (production.read_bytes(), world.read_bytes()) == before
    assert contract['selected_source_timeout_s'] == 0.30
    assert contract['production_inputs']['production_params']['sha256']
    assert set(contract['production_inputs']) == {
        'production_params', 'navigation_launch',
        'onboard_nav2_core_launch', 'production_urdf'}
    assert contract['evaluation_world_source']['contact_system_present']
    world_asset = contract['evaluation_assets']['world']
    assert world_asset['source_path'] == str(world.resolve())
    assert world_asset['source_size_bytes'] == world.stat().st_size
    assert world_asset['size_bytes'] == world.stat().st_size
    assert world_asset['source_sha256'] == world_asset['sha256']
    assert world_asset['copy_byte_identical'] is True
    manifest = json.loads(
        (tmp_path / 'output/preparation_manifest.json').read_text())
    assert set(manifest['expected_generated_paths']) == {
        'contract.json', 'collision_monitor_overlay.yaml',
        'nav2_collision_monitor_eval.params.yaml',
        'jdamr_cube_collision_monitor_eval.urdf',
        'slam_corridor_eval.pgm', 'slam_corridor_eval.yaml',
        'slam_corridor_contact.world', 'collision_monitor_bridge.yaml'}
    assert {record['relative_path'] for record in manifest['records']} == set(
        manifest['expected_generated_paths'])
    assert contract['collision_monitor_package_version'].startswith('1.3.12')
    monitor = overlay['collision_monitor']['ros__parameters']
    assert monitor['scan']['topic'] == '/collision_monitor_scan'
    assert monitor['polygons'] == ['StopZone']


@pytest.mark.parametrize('field', [
    'speed', 'decel', 'footprint', 'resolution', 'goal_tolerance'])
def test_prepare_derives_motion_bounds_and_rejects_source_mismatch(
        tmp_path, field):
    """Production motion and geometry changes cannot retain stale bounds."""
    prepare = _load('prepare_sim_collision_monitor_run.py')
    params = yaml.safe_load((
        ROOT / 'jdamr_cube_navigation/config/nav2_params.yaml').read_text())
    if field == 'speed':
        params['controller_server']['ros__parameters']['FollowPath'][
            'desired_linear_vel'] = 0.42
        params['velocity_smoother']['ros__parameters'][
            'max_velocity'][0] = 0.42
    elif field == 'decel':
        params['velocity_smoother']['ros__parameters']['max_decel'][0] = -0.9
    elif field == 'footprint':
        params['local_costmap']['local_costmap']['ros__parameters'][
            'footprint'] = (
                '[[0.3, 0.2], [0.3, -0.2], '
                '[-0.3, -0.2], [-0.3, 0.2]]')
    else:
        if field == 'resolution':
            params['global_costmap']['global_costmap']['ros__parameters'][
                'resolution'] = 0.1
        else:
            params['controller_server']['ros__parameters'][
                'position_goal_checker']['xy_goal_tolerance'] = 0.2
    production = tmp_path / 'nav2.yaml'
    production.write_text(yaml.safe_dump(params, sort_keys=False))
    world = tmp_path / 'world.sdf'
    world.write_text(
        '<plugin filename="gz-sim-contact-system"/>'
        '<max_step_size>0.001</max_step_size>')
    if field in {'speed', 'decel'}:
        contract = prepare.prepare(production, world, tmp_path / 'output')
        if field == 'speed':
            assert contract['stop_zone']['inputs'][
                'max_forward_speed_mps'] == 0.42
            assert contract['maximum_stop_distance_m'] > 0.2
        else:
            assert contract['stop_zone']['inputs']['max_decel_mps2'] == 0.9
            assert contract['maximum_stop_distance_m'] < 0.09558
    else:
        with pytest.raises(ValueError):
            prepare.prepare(production, world, tmp_path / 'output')


def test_prepare_fails_before_writing_when_required_urdf_is_missing(
        tmp_path, monkeypatch):
    """A missing inherited simulation URDF cannot yield partial assets."""
    prepare = _load('prepare_sim_collision_monitor_run.py')
    missing_assets = tmp_path / 'missing_assets'
    monkeypatch.setattr(prepare, 'G003_ASSETS', missing_assets)
    output = tmp_path / 'output'

    with pytest.raises(FileNotFoundError):
        prepare.prepare(tmp_path / 'unused.yaml', tmp_path / 'unused.world',
                        output)

    assert not output.exists()


@pytest.mark.parametrize('field,value', [
    ('stateful', False), ('plugin', 'nav2_controller::PositionGoalChecker')])
def test_prepare_rejects_incompatible_selected_goal_checker(
        tmp_path, field, value):
    """The default selected checker must support the stateful-entry proof."""
    prepare = _load('prepare_sim_collision_monitor_run.py')
    params = yaml.safe_load((
        ROOT / 'jdamr_cube_navigation/config/nav2_params.yaml').read_text())
    params['controller_server']['ros__parameters'][
        'general_goal_checker'][field] = value
    production = tmp_path / 'nav2.yaml'
    production.write_text(yaml.safe_dump(params, sort_keys=False))
    world = tmp_path / 'world.sdf'
    world.write_text(
        '<plugin filename="gz-sim-contact-system"/>'
        '<max_step_size>0.001</max_step_size>')

    with pytest.raises(ValueError):
        prepare.prepare(production, world, tmp_path / 'output')


def _passing_evidence(scenario):
    contract_module = _load('sim_collision_monitor_contract.py')
    contract = contract_module.derive_contract(
        20.0, 0.001)
    scan_gate_source = (ROOT / 'jdamr_cube_navigation'
                        / 'jdamr_cube_navigation/sim_scan_gate.py').resolve()
    scenario_source = (ROOT / 'jdamr_cube_navigation'
                       / 'jdamr_cube_navigation'
                       / 'sim_collision_monitor_scenario.py').resolve()
    event_names = {
        'clear_baseline': [
            'goal_accepted', 'goal_position_tolerance_entered', 'succeeded'],
        'sudden_obstacle_stop_resume': [
            'goal_accepted', 'scan_gate_arm_requested',
            'scan_gate_arm_ack', 'obstacle_set_pose_requested',
            'stop_state', 'final_zero',
            'physical_stop', 'clear_set_pose_requested',
            'obstacle_deactivated', 'clear_set_pose_ack',
            'clear_pose_verified',
            'obstacle_set_pose_ack', 'obstacle_pose_verified',
            'do_nothing_after_stop', 'goal_position_tolerance_entered',
            'succeeded'],
        'scan_timeout_stop_resume': [
            'goal_accepted', 'monitor_scan_freeze_requested',
            'monitor_scan_frozen', 'stop_state',
            'final_zero', 'physical_stop',
            'monitor_scan_unfreeze_requested', 'monitor_scan_unfrozen',
            'do_nothing_after_stop', 'goal_position_tolerance_entered',
            'succeeded'],
    }[scenario]
    if scenario == 'sudden_obstacle_stop_resume':
        event_times = {
            'goal_accepted': (10, 10),
            'scan_gate_arm_requested': (50_000_000, 50_000_000),
            'scan_gate_arm_ack': (70_000_000, 70_000_000),
            'obstacle_set_pose_requested': (90_000_000, 90_000_000),
            'stop_state': (120_000_000, 120_000_000),
            'final_zero': (130_000_000, 130_000_000),
            'physical_stop': (140_000_000, 140_000_000),
            'clear_set_pose_requested': (145_000_000, 145_000_000),
            'obstacle_deactivated': (150_000_000, 150_000_000),
            'clear_set_pose_ack': (155_000_000, 155_000_000),
            'clear_pose_verified': (165_000_000, 165_000_000),
            'obstacle_set_pose_ack': (125_000_000, 125_000_000),
            'obstacle_pose_verified': (135_000_000, 135_000_000),
            'do_nothing_after_stop': (160_000_000, 160_000_000),
            'goal_position_tolerance_entered': (
                168_000_000, 168_000_000),
            'succeeded': (170_000_000, 170_000_000),
        }
    elif scenario == 'scan_timeout_stop_resume':
        event_times = {
            'goal_accepted': (10, 10),
            'monitor_scan_freeze_requested': (90_000_000, 90_000_000),
            'monitor_scan_frozen': (100_000_000, 100_000_000),
            'stop_state': (420_000_000, 420_000_000),
            'final_zero': (430_000_000, 430_000_000),
            'physical_stop': (440_000_000, 440_000_000),
            'monitor_scan_unfreeze_requested': (445_000_000, 445_000_000),
            'monitor_scan_unfrozen': (450_000_000, 450_000_000),
            'do_nothing_after_stop': (460_000_000, 460_000_000),
            'goal_position_tolerance_entered': (
                468_000_000, 468_000_000),
            'succeeded': (470_000_000, 470_000_000),
        }
    else:
        event_times = {
            'goal_accepted': (10, 10),
            'goal_position_tolerance_entered': (15, 15),
            'succeeded': (20, 20),
        }
    events = []
    for name in event_names:
        steady_ns, ros_ns = event_times[name]
        event = {'name': name, 'steady_ns': steady_ns, 'ros_ns': ros_ns}
        if name == 'goal_accepted':
            event['goal_uuid'] = 'a' * 32
        if name == 'goal_position_tolerance_entered':
            event.update({
                'goal_uuid': 'a' * 32, 'frame_id': 'map',
                'child_frame_id': 'base_link', 'pose_m': [6.0, 0.0],
                'goal_pose_m': [6.0, 0.0],
                'transform_stamp_ns': ros_ns,
                'transform_observer_ros_ns': ros_ns,
                'transform_stamp_offset_s': 0.0,
                'position_error_m': 0.0,
                'tf_inputs': [{
                    'parent': 'map', 'child': 'base_link',
                    'stamp_ns': ros_ns,
                    'translation': [6.0, 0.0, 0.0],
                    'rotation': [0.0, 0.0, 0.0, 1.0],
                    'is_static': False,
                }],
            })
        if name == 'stop_state':
            event['polygon'] = (
                'invalid source' if scenario == 'scan_timeout_stop_resume'
                else 'StopZone')
        events.append(event)
    timeout = scenario == 'scan_timeout_stop_resume'
    sudden = scenario == 'sudden_obstacle_stop_resume'
    terminal_ros_ns = event_times['succeeded'][1]
    goal_entry = next(
        event for event in events
        if event['name'] == 'goal_position_tolerance_entered')
    entity_states = []
    clearance_m = None
    clearance_samples = []
    clearance_witness = None
    if sudden:
        activation = [
            contract['sudden_obstacle']['activation_center_offset_x_m'],
            0.0, 0.5]
        entity_states = [
            {'entity_id': 60, 'pose_m': activation,
             'expected_pose_m': activation, 'position_error_m': 0.0,
             'defaulted_axes': [], 'orientation_yaw_rad': 0.0,
             'orientation_defaulted': True},
            {'entity_id': 60, 'pose_m': [0.0, 40.0, 0.5],
             'expected_pose_m': [0.0, 40.0, 0.5],
             'position_error_m': 0.0, 'defaulted_axes': ['x'],
             'orientation_yaw_rad': 0.0,
             'orientation_defaulted': True},
        ]
        clearance_m = (
            contract['sudden_obstacle']['activation_center_offset_x_m']
            - contract['sudden_obstacle']['length_m'] / 2.0
            - contract['stop_zone']['inputs']['footprint_front_m'])
        clearance_samples = [
            [1, 0.0, 0.0, 0.0], [2, 0.0, 0.0, 0.0],
            [3, 0.0, 0.0, 0.0]]
        clearance_witness = {
            'robot_pose_xyyaw': [0.0, 0.0, 0.0],
            'obstacle_center_xy': activation[:2],
            'obstacle_dimensions_m': [
                contract['sudden_obstacle']['length_m'],
                contract['sudden_obstacle']['width_m']],
            'clearance_m': clearance_m,
        }
    return {
        'run_id': f'{scenario}__seed_11',
        'scenario': scenario,
        'seed': 11,
        'contract': contract,
        'events': events,
        'entity_states': entity_states,
        'goal_uuid': 'a' * 32,
        'terminal_goal_uuid': 'a' * 32,
        'goal_send_count': 1,
        'goal_cancel_count': 0,
        'raw_scan_continued': True,
        'navigation_scan_continued': True,
        'collision_monitor_scan_frozen': scenario == (
            'scan_timeout_stop_resume'),
        'monitor_scan_count_at_freeze': 10,
        'monitor_scan_count_at_stop': 10,
        'monitor_scan_count_final': 11,
        'last_monitor_scan_steady_ns_at_freeze': 100_000_000,
        'last_monitor_scan_stamp_ns_at_freeze': 100_000_000,
        'freeze_ros_ns': 100_000_000 if timeout else None,
        'freeze_steady_ns': 100_000_000 if timeout else None,
        'freeze_request_ros_ns': 90_000_000 if timeout else None,
        'freeze_request_steady_ns': 90_000_000 if timeout else None,
        'raw_scan_count_at_trigger': 10,
        'raw_scan_count_at_stop': 14,
        'navigation_scan_count_at_trigger': 10,
        'navigation_scan_count_at_stop': 14,
        'contact_count': 0,
        'contact_matched_publisher_count_max': 1,
        'contact_subscription_created': True,
        'final_cmd_vel_zero': True,
        'final_zero_hold_s': 1.95,
        'identity_survivors': [],
        'identity_process_groups': [10_000 + index for index in range(5)],
        'process_identity': [
            {'name': name, 'pid': 10_000 + index}
            for index, name in enumerate((
                'gazebo', 'scan_gate', 'contact_bridge', 'navigation',
                'scenario', 'gazebo_resource_sampler',
                'nav2_resource_sampler'))
        ],
        'retained_run_bytes': 1024,
        'full_matrix_bag_recorded': False,
        'representative_bag_recorded': False,
        'production_hashes_unchanged': True,
        'harness_error': None,
        'teardown': {
            'measurement_finalized_steady_ns': 1_000_000_000,
            'processes': [
                {
                    'name': name,
                    'pid': 10_000 + index,
                    'returncode_at_measurement': (
                        0 if name == 'scenario' else None),
                    'returncode_before_stop': 0 if name == 'scenario' else None,
                    'runner_initiated': name != 'scenario',
                    'requested_signal': None if name == 'scenario' else 2,
                    'stop_requested_steady_ns': 2_000_000_000 + index,
                    'stop_completed_steady_ns': 3_000_000_000 + index,
                    'returncode_after_stop': 0 if name == 'scenario' else -2,
                    'logged_process_exits_before_stop': [],
                    'logged_process_exits_after_stop': [],
                    'log_path': '',
                    'log_size_bytes': 0,
                    'log_sha256': None,
                    'log_pre_stop_offset_bytes': 0,
                }
                for index, name in enumerate((
                    'gazebo', 'scan_gate', 'contact_bridge', 'navigation',
                    'scenario', 'gazebo_resource_sampler',
                    'nav2_resource_sampler'))
            ],
            'errors': [],
            'status': 'PASS',
        },
        'resource_evidence': {},
        'storage_preflight': {
            'passed': True,
            'home_free_bytes': 1024,
            'required_free_bytes': 512,
            'predicted_run_bytes': 256,
        },
        'activation_error': None,
        'canonical_contract_sha256': 'unit-contract-sha256',
        'scan_gate_source': {
            'path': str(scan_gate_source),
            'size_bytes': scan_gate_source.stat().st_size,
            'sha256': contract_module.sha256_file(scan_gate_source),
        },
        'scenario_source': {
            'path': str(scenario_source),
            'size_bytes': scenario_source.stat().st_size,
            'sha256': contract_module.sha256_file(scenario_source),
        },
        'initial_world_pose_m': [-8.0, 0.0],
        'final_world_pose_m': [6.0, 0.0],
        'final_estimated_pose_m': [6.0, 0.0],
        'final_estimated_pose_frame_id': 'map',
        'final_estimated_pose_stamp_ns': terminal_ros_ns,
        'final_estimated_pose_observed_ros_ns': terminal_ros_ns,
        'estimated_pose_stamp_offset_at_terminal_s': 0.0,
        'terminal_ground_truth_pose_m': [6.0, 0.0],
        'terminal_ground_truth_observed_ros_ns': terminal_ros_ns,
        'terminal_ground_truth_goal_error_m': 0.0,
        'terminal_estimated_to_gt_error_m': 0.0,
        'goal_position_tolerance_entry': goal_entry,
        'ros_start_ns': 10,
        'ros_end_ns': 20,
        'steady_start_ns': 30,
        'steady_end_ns': 40,
        'stop_state_count': 0 if scenario == 'clear_baseline' else 1,
        'final_action_type': 0,
        'reference_scan_stamp_ns': 90_000_000,
        'scan_gate_arm_request_steady_ns': 50_000_000 if sudden else None,
        'scan_gate_arm_request_ros_ns': 50_000_000 if sudden else None,
        'scan_gate_arm_ack_steady_ns': 70_000_000 if sudden else None,
        'scan_gate_arm_ack_ros_ns': 70_000_000 if sudden else None,
        'obstacle_activation_steady_ns': 125_000_000 if sudden else None,
        'obstacle_activation_ros_ns': 125_000_000 if sudden else None,
        'obstacle_request_steady_ns': 90_000_000 if sudden else None,
        'obstacle_request_ros_ns': 90_000_000 if sudden else None,
        'obstacle_verified_steady_ns': 135_000_000 if sudden else None,
        'obstacle_verified_ros_ns': 135_000_000 if sudden else None,
        'obstacle_request_to_zero_steady_s': 0.04 if sudden else None,
        'obstacle_ack_to_zero_steady_s': 0.005 if sudden else None,
        'clear_request_steady_ns': 145_000_000 if sudden else None,
        'clear_request_ros_ns': 145_000_000 if sudden else None,
        'clear_ack_steady_ns': 155_000_000 if sudden else None,
        'clear_ack_ros_ns': 155_000_000 if sudden else None,
        'clear_verified_steady_ns': 165_000_000 if sudden else None,
        'clear_verified_ros_ns': 165_000_000 if sudden else None,
        'unfreeze_request_steady_ns': 445_000_000 if timeout else None,
        'unfreeze_request_ros_ns': 445_000_000 if timeout else None,
        'unfreeze_ack_steady_ns': 450_000_000 if timeout else None,
        'unfreeze_ack_ros_ns': 450_000_000 if timeout else None,
        'trigger_steady_ns': 90_000_000,
        'trigger_world_pose_m': [0.0, 0.0],
        'clear_reference_scan_stamp_ns': 100_000_000,
        'clear_scan_stamp_ns': 110_000_000,
        'stop_action_type': 1,
        'stop_polygon_name': (
            'invalid source' if scenario == 'scan_timeout_stop_resume'
            else 'StopZone'),
        'stop_state_steady_ns': 420_000_000 if timeout else 120_000_000,
        'stop_state_ros_ns': 420_000_000 if timeout else 120_000_000,
        'zero_steady_ns': 430_000_000 if timeout else 130_000_000,
        'zero_ros_ns': 430_000_000 if timeout else 130_000_000,
        'physical_stop_steady_ns': (
            440_000_000 if timeout else 140_000_000),
        'freeze_ack_to_zero_ros_s': 0.33 if timeout else None,
        'freeze_ack_to_zero_steady_s': 0.33 if timeout else None,
        'physical_stop_observed': True,
        'stop_distance_m': 0.02,
        'stop_world_pose_m': [0.02, 0.0],
        'footprint_to_obstacle_clearance_m': clearance_m,
        'clearance_sample_count': 3 if sudden else 0,
        'clearance_samples': clearance_samples,
        'minimum_clearance_witness': clearance_witness,
        'minimum_observed_scan_range_m': 0.01,
        'resume_action_type': 0,
        'action_terminal': 'succeeded',
        'retention_excluded_paths': ['evidence.json', 'evaluation.json'],
    }


def _attach_scan_gate_evidence(evidence, tmp_path):
    """Attach one canonical same-process sudden-reaction sidecar."""
    if evidence['scenario'] != 'sudden_obstacle_stop_resume':
        return
    contract_module = _load('sim_collision_monitor_contract.py')
    source = (ROOT / 'jdamr_cube_navigation/jdamr_cube_navigation'
              / 'sim_scan_gate.py').resolve()
    sidecar = {
        'schema_version': 1,
        'run_id': evidence['run_id'],
        'scenario': evidence['scenario'],
        'seed': evidence['seed'],
        'contract_sha256': evidence['canonical_contract_sha256'],
        'source_path': str(source),
        'source_size_bytes': source.stat().st_size,
        'source_sha256': contract_module.sha256_file(source),
        'arm_receive_steady_ns': 60_000_000,
        'stamp_ns': 100_000_000,
        'publish_steady_ns': 100_000_000,
        'zero_receive_steady_ns': 130_000_000,
        'scan_publish_to_zero_receive_steady_s': 0.03,
        'frame_id': 'laser_link',
        'angle_min_rad': math.pi,
        'angle_increment_rad': 0.0,
        'ranges_m': [0.1, 0.1, 0.1],
        'stop_zone_points': [[0.1, 0.0], [0.1, 0.0], [0.1, 0.0]],
    }
    path = tmp_path / 'scan_gate_evidence.json'
    path.write_text(json.dumps(sidecar, sort_keys=True) + '\n')
    evidence['scan_gate_evidence'] = {
        'path': str(path.resolve()),
        'size_bytes': path.stat().st_size,
        'sha256': contract_module.sha256_file(path),
    }
    evidence['scan_header_to_stop_observer_ros_signed_s'] = (
        evidence['stop_state_ros_ns'] - sidecar['stamp_ns']) / 1e9


def _attach_resource_evidence(evidence, tmp_path):
    _attach_scan_gate_evidence(evidence, tmp_path)
    contract = _load('sim_collision_monitor_contract.py')
    for name in ('gazebo', 'nav2'):
        path = tmp_path / f'{name}_resources.jsonl'
        path.write_text(
            '{"monotonic_s":1.0,"cpu_total_s":1.0,'
            '"cpu_pct_one_core":null,"rss_mb":2.0,"process_count":1}\n'
            '{"monotonic_s":1.5,"cpu_total_s":1.1,'
            '"cpu_pct_one_core":20.0,"rss_mb":3.0,"process_count":0}\n')
        evidence['resource_evidence'][name] = {
            'path': str(path),
            'sha256': contract.sha256_file(path),
            'size_bytes': path.stat().st_size,
            'sample_count': 2,
            'cpu_sample_count': 1,
            'median_cpu_pct_one_core': 1.0,
            'p95_cpu_pct_one_core': 20.0,
            'max_rss_mb': 3.0,
            'max_process_count': 1,
        }
    evidence['resource_evidence']['gazebo'][
        'median_cpu_pct_one_core'] = 20.0
    evidence['resource_evidence']['nav2'][
        'median_cpu_pct_one_core'] = 20.0
    for record in evidence['teardown']['processes']:
        path = tmp_path / f'{record["name"]}.log'
        path.write_bytes(b'')
        record.update({
            'log_path': str(path.resolve()),
            'log_size_bytes': path.stat().st_size,
            'log_sha256': contract.sha256_file(path),
            'log_pre_stop_offset_bytes': 0,
        })
    evidence['retained_run_bytes'] = sum(
        path.stat().st_size for path in tmp_path.rglob('*') if path.is_file())


def _write_preparation_identity(
        root, canonical, contract_path, contract_module):
    expected = {
        'contract.json', 'collision_monitor_overlay.yaml',
        'nav2_collision_monitor_eval.params.yaml',
        'jdamr_cube_collision_monitor_eval.urdf',
        'slam_corridor_eval.pgm', 'slam_corridor_eval.yaml',
        'slam_corridor_contact.world', 'collision_monitor_bridge.yaml'}
    prepared = root / 'assets'
    prepared.mkdir()
    sources = root / 'sources'
    sources.mkdir()
    production = {}
    for key in (
            'production_params', 'navigation_launch',
            'onboard_nav2_core_launch', 'production_urdf'):
        source = sources / key
        if key == 'production_params':
            source.write_bytes((
                ROOT / 'jdamr_cube_navigation/config/nav2_params.yaml'
            ).read_bytes())
        elif key == 'production_urdf':
            source.write_bytes((
                ROOT / 'jdamr_cube_description/urdf/jdamr_cube.urdf'
            ).read_bytes())
        else:
            source.write_text(key)
        production[key] = {
            'path': str(source), 'size_bytes': source.stat().st_size,
            'sha256': contract_module.sha256_file(source)}
    canonical['production_inputs'] = production
    scan_profile = sources / 'sensor_profile.json'
    scan_profile.write_text(json.dumps({
        'timing': {'scan': {'header_interval_s': {'max': 0.107525}}}}))
    canonical['scan_gap_provenance'] = {
        'path': str(scan_profile),
        'size_bytes': scan_profile.stat().st_size,
        'sha256': contract_module.sha256_file(scan_profile),
        'json_pointer': '/timing/scan/header_interval_s/max',
        'value_s': 0.107525,
        'observed_on': '2026-09-04',
        'valid_for': 'G004 simulated StopZone reaction-margin derivation only',
    }
    asset_keys = {
        'jdamr_cube_collision_monitor_eval.urdf': 'urdf',
        'collision_monitor_overlay.yaml': 'collision_monitor_overlay',
        'nav2_collision_monitor_eval.params.yaml': 'nav2_evaluation_params',
        'slam_corridor_eval.pgm': 'slam_corridor_eval.pgm',
        'slam_corridor_eval.yaml': 'slam_corridor_eval.yaml',
        'slam_corridor_contact.world': 'world',
        'collision_monitor_bridge.yaml': 'bridge'}
    canonical['evaluation_assets'] = {}
    for name, key in asset_keys.items():
        path = prepared / name
        path.write_text(name)
        source = sources / f'asset_{name}'
        source.write_text(f'source_{name}')
        canonical['evaluation_assets'][key] = {
            'path': str(path), 'size_bytes': path.stat().st_size,
            'sha256': contract_module.sha256_file(path),
            'source_path': str(source),
            'source_size_bytes': source.stat().st_size,
            'source_sha256': contract_module.sha256_file(source)}
    contract_path.write_text(json.dumps(canonical))
    records = []
    for name in sorted(expected):
        path = contract_path if name == 'contract.json' else prepared / name
        records.append({
            'relative_path': name, 'size_bytes': path.stat().st_size,
            'sha256': contract_module.sha256_file(path)})
    manifest_path = root / 'preparation_manifest.json'
    manifest_path.write_text(json.dumps({
        'schema_version': 1, 'prepared_root': str(prepared),
        'expected_generated_paths': sorted(expected), 'records': records}))
    library = Path('/opt/ros/jazzy/lib/librmw_fastrtps_cpp.so')
    environment_fields = {
        'AMENT_PREFIX_PATH': '/opt/ros/jazzy',
        'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib',
        'PATH': '/opt/ros/jazzy/bin:/usr/bin',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    rmw = {
        'requested_identifier': 'rmw_fastrtps_cpp',
        'actual_identifier': 'rmw_fastrtps_cpp',
        'version': 'unit',
        'source': 'system_ros_installation',
        'prefix': '/opt/ros/jazzy',
        'library_path': str(library),
        'library_size_bytes': library.stat().st_size,
        'library_sha256': contract_module.sha256_file(library),
        'environment_fields': environment_fields,
        'environment_sha256': hashlib.sha256(json.dumps(
            environment_fields, sort_keys=True).encode()).hexdigest(),
        'domain_id': 160,
    }
    evaluation = ROOT / 'jdamr_cube_navigation/evaluation'
    package = ROOT / 'jdamr_cube_navigation/jdamr_cube_navigation'
    source_paths = {
        'contract': evaluation / 'sim_collision_monitor_contract.py',
        'prepare': evaluation / 'prepare_sim_collision_monitor_run.py',
        'runner': evaluation / 'run_sim_collision_monitor_eval.py',
        'evaluator': evaluation / 'evaluate_sim_collision_monitor.py',
        'finalizer': evaluation / 'finalize_sim_collision_monitor_eval.py',
        'resource_sampler': evaluation / 'sample_process_group_resources.py',
        'scenario': package / 'sim_collision_monitor_scenario.py',
        'scan_gate': package / 'sim_scan_gate.py',
    }
    harness_sources = {}
    for name, path in source_paths.items():
        retained = root / 'runtime_sources' / f'{name}.py'
        retained.parent.mkdir(parents=True, exist_ok=True)
        retained.write_bytes(path.read_bytes())
        harness_sources[name] = {
            'path': str(path.resolve()), 'size_bytes': path.stat().st_size,
            'sha256': contract_module.sha256_file(path),
            'retained_path': str(retained.resolve()),
            'retained_size_bytes': retained.stat().st_size,
            'retained_sha256': contract_module.sha256_file(retained),
        }
    retained_library = root / 'runtime_dependencies' / library.name
    retained_library.parent.mkdir(parents=True, exist_ok=True)
    retained_library.write_bytes(library.read_bytes())
    rmw.update({
        'retained_library_path': str(retained_library.resolve()),
        'retained_library_size_bytes': retained_library.stat().st_size,
        'retained_library_sha256': contract_module.sha256_file(
            retained_library),
    })
    (root / 'runtime_manifest.json').write_text(json.dumps({
        'evaluation_rmw': rmw,
        'canonical_contract_sha256': contract_module.sha256_file(
            contract_path),
        'preparation_manifest_sha256': contract_module.sha256_file(
            manifest_path),
        'domain_id_base': 160,
        'domain_ids': list(range(160, 175)),
        'execution_mode': 'matrix_evaluation',
        'planned_runs': [
            {'run_id': item['run_id'], 'domain_id': 160 + index}
            for index, item in enumerate(contract_module.scenario_matrix())],
        'harness_sources': harness_sources,
    }))


@pytest.mark.parametrize('scenario', [
    'clear_baseline', 'sudden_obstacle_stop_resume',
    'scan_timeout_stop_resume'])
def test_each_scenario_contract_passes_complete_evidence(
        scenario, tmp_path):
    """Each scenario exposes its intended state transition gates."""
    evaluator = _load('evaluate_sim_collision_monitor.py')

    evidence = _passing_evidence(scenario)
    _attach_resource_evidence(evidence, tmp_path)

    assert evaluator.evaluate_run(
        evidence, tmp_path / 'evidence.json')['status'] == 'PASS'


def test_timeout_freeze_fails_when_navigation_scan_stops():
    """Freezing /scan together with the monitor branch is invalid."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('scan_timeout_stop_resume')
    evidence['navigation_scan_continued'] = False

    result = evaluator.evaluate_run(evidence)

    assert result['status'] == 'FAIL'
    assert 'monitor_only_freeze' in result['failures']


def test_timeout_uses_scan_stamp_age_not_callback_receipt_age(tmp_path):
    """Collision Monitor timeout evidence follows the sensor stamp clock."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('scan_timeout_stop_resume')
    _attach_resource_evidence(evidence, tmp_path)
    evidence['stop_state_ros_ns'] = 300_000_100

    result = evaluator.evaluate_run(evidence)
    assert result['status'] == 'FAIL'
    assert 'timeout_elapsed_without_monitor_scan' in result['failures']


def test_retention_caps_are_512_and_128_mib():
    """Matrix and representative retention use the reduced disk caps."""
    contract = _load('sim_collision_monitor_contract.py')
    runner = _load('run_sim_collision_monitor_eval.py')

    assert contract.TOTAL_RETENTION_LIMIT_BYTES == 512 * 1024 * 1024
    assert contract.REPRESENTATIVE_BAG_LIMIT_BYTES == 128 * 1024 * 1024
    assert runner.retention_allowed(511 * 1024 * 1024, 1024 * 1024)
    assert not runner.retention_allowed(
        511 * 1024 * 1024, 2 * 1024 * 1024)
    assert runner.representative_bag_allowed(128 * 1024 * 1024)
    assert not runner.representative_bag_allowed(128 * 1024 * 1024 + 1)


def test_representative_recorder_includes_hidden_topics_exactly_once(
        monkeypatch, tmp_path):
    """Representative MCAP must retain the hidden Nav2 action status."""
    runner = _load('run_sim_collision_monitor_eval.py')
    starts = []
    help_calls = []
    monkeypatch.setattr(runner.subprocess, 'run', lambda command, **kwargs: (
        help_calls.append(command)
        or SimpleNamespace(returncode=0, stdout='--include-hidden-topics')))
    monkeypatch.setattr(runner, '_start', lambda command, log, env: (
        starts.append(command) or ('process', 'stream')))
    args = (
        tmp_path / 'bag', tmp_path / 'qos.yaml', ('/tf',),
        tmp_path / 'recorder.log', {'ROS_DOMAIN_ID': '180'})

    assert runner._start_representative_recorder(False, *args) == (None, None)
    assert starts == [] and help_calls == []

    assert runner._start_representative_recorder(True, *args) == (
        'process', 'stream')
    assert help_calls == [['ros2', 'bag', 'record', '--help']]
    assert starts[0].count('--include-hidden-topics') == 1
    assert starts[0][-2:] == ['--topics', '/tf']


def test_representative_recorder_fails_when_hidden_topics_unsupported(
        monkeypatch, tmp_path):
    """Promotion fails before launch when rosbag2 lacks hidden topics."""
    runner = _load('run_sim_collision_monitor_eval.py')
    monkeypatch.setattr(runner.subprocess, 'run', lambda *args, **kwargs: (
        SimpleNamespace(returncode=0, stdout='ros2 bag record')))
    monkeypatch.setattr(
        runner, '_start', lambda *args, **kwargs: pytest.fail(
            'recorder must not start without hidden-topic support'))

    with pytest.raises(RuntimeError, match='hidden topic'):
        runner._start_representative_recorder(
            True, tmp_path / 'bag', tmp_path / 'qos.yaml', ('/tf',),
            tmp_path / 'recorder.log', {})


def test_collision_media_uses_path_state_overlay_contract():
    """Portfolio media is a fixed path/state story, not a bar chart."""
    renderer = _load('render_sim_collision_monitor_media.py')

    assert (renderer.WIDTH, renderer.HEIGHT, renderer.FPS,
            renderer.FRAME_COUNT) == (1280, 720, 8, 96)
    assert renderer.MEDIA_LIMIT_BYTES == 128 * 1024 * 1024
    assert renderer.SCENARIOS == (
        'clear_baseline', 'sudden_obstacle_stop_resume',
        'scan_timeout_stop_resume')


def test_collision_media_state_timeline_is_fail_closed():
    """Dynamic media labels follow required event transitions."""
    renderer = _load('render_sim_collision_monitor_media.py')
    events = {
        'succeeded': {'ros_ns': 100},
        'obstacle_set_pose_requested': {'ros_ns': 20},
        'stop_state': {'ros_ns': 30},
        'do_nothing_after_stop': {'ros_ns': 50},
    }

    assert renderer._scenario_state(
        'sudden_obstacle_stop_resume', 25, events)[0] == 'OBSTACLE DETECTED'
    assert renderer._scenario_state(
        'sudden_obstacle_stop_resume', 35, events)[0] == 'STOP · StopZone'
    assert renderer._scenario_state(
        'sudden_obstacle_stop_resume', 60, events)[0] == 'RESUMED'
    with pytest.raises(ValueError, match='unknown scenario'):
        renderer._scenario_state('bar_chart', 0, events)


def test_collision_media_expands_short_stop_transition():
    """Short simulator stop intervals remain visible in the fixed video."""
    renderer = _load('render_sim_collision_monitor_media.py')
    events = {
        'goal_accepted': {'ros_ns': 0},
        'obstacle_set_pose_requested': {'ros_ns': 10},
        'stop_state': {'ros_ns': 110},
        'do_nothing_after_stop': {'ros_ns': 210},
        'succeeded': {'ros_ns': 1000},
    }
    anchors = renderer._timeline_anchors(
        'sudden_obstacle_stop_resume', events)

    assert renderer._target_time_ns(anchors, 0.20) < 110
    assert 110 < renderer._target_time_ns(anchors, 0.40) < 210
    assert renderer._target_time_ns(anchors, 0.50) < 210
    assert renderer._media_fraction(anchors, 110) == pytest.approx(0.32)


def test_collision_media_marker_uses_same_phase_as_state_card():
    """Equal ROS stamps cannot make a future STOP marker appear completed."""
    renderer = _load('render_sim_collision_monitor_media.py')
    progress = 20 / (renderer.FRAME_COUNT - 1)

    assert renderer._scenario_state_for_progress(
        'sudden_obstacle_stop_resume', progress)[0] == 'OBSTACLE DETECTED'
    assert renderer._marker_passed(progress, 0.18) is True
    assert renderer._marker_passed(progress, 0.32) is False


def test_collision_media_rejects_symlinked_output_root(tmp_path):
    """A copied valid media tree cannot be accepted through a root symlink."""
    renderer = _load('render_sim_collision_monitor_media.py')
    target = tmp_path / 'copied_media'
    target.mkdir()
    link = tmp_path / 'media_link'
    link.symlink_to(target, target_is_directory=True)

    with pytest.raises(ValueError, match='canonical directory'):
        renderer._validate_stored(tmp_path, tmp_path, link)


def test_domain_block_stays_bounded_and_avoids_physical_robot():
    """All 15 matrix domains stay valid and isolated from domain 12."""
    runner = _load('run_sim_collision_monitor_eval.py')

    runner.validate_domain_block(160)
    with pytest.raises(ValueError):
        runner.validate_domain_block(220)
    with pytest.raises(ValueError):
        runner.validate_domain_block(12)
    with pytest.raises(ValueError):
        runner.validate_domain_block(1)


def test_resource_summary_rejects_empty_and_reports_finite_values(tmp_path):
    """Compact sampler traces produce bounded finite summaries."""
    runner = _load('run_sim_collision_monitor_eval.py')
    trace = tmp_path / 'resources.jsonl'
    trace.write_text(
        '{"cpu_pct_one_core":null,"rss_mb":10,"process_count":2}\n'
        '{"cpu_pct_one_core":20,"rss_mb":12,"process_count":2}\n'
        '{"cpu_pct_one_core":10,"rss_mb":0,"process_count":0}\n')

    summary = runner.summarize_resource_samples(trace)

    assert summary['sample_count'] == 3
    assert summary['median_cpu_pct_one_core'] == 15
    assert summary['p95_cpu_pct_one_core'] == 20
    assert summary['max_rss_mb'] == 12


def test_evaluator_rejects_tampered_resource_trace(tmp_path):
    """A changed compact trace invalidates its recorded resource evidence."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    _attach_resource_evidence(evidence, tmp_path)

    Path(evidence['resource_evidence']['gazebo']['path']).write_text(
        '{"tampered":true}\n')

    result = evaluator.evaluate_run(evidence)
    assert result['status'] == 'FAIL'
    assert 'resource_files_match_evidence' in result['failures']


@pytest.mark.parametrize('field,value', [
    ('monotonic_s', True), ('cpu_total_s', False),
    ('cpu_pct_one_core', True), ('rss_mb', False),
    ('process_count', True)])
def test_evaluator_rejects_boolean_resource_values(tmp_path, field, value):
    """JSON booleans cannot masquerade as resource numbers or counts."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _load('sim_collision_monitor_contract.py')
    evidence = _passing_evidence('clear_baseline')
    _attach_resource_evidence(evidence, tmp_path)
    trace = Path(evidence['resource_evidence']['gazebo']['path'])
    rows = [json.loads(line) for line in trace.read_text().splitlines()]
    rows[1][field] = value
    trace.write_text('\n'.join(json.dumps(row) for row in rows) + '\n')
    evidence['resource_evidence']['gazebo'].update({
        'size_bytes': trace.stat().st_size,
        'sha256': contract.sha256_file(trace),
    })

    result = evaluator.evaluate_run(evidence, tmp_path / 'evidence.json')

    assert 'resource_files_match_evidence' in result['failures']


def test_evaluator_rejects_failed_home_storage_preflight(tmp_path):
    """Insufficient home-space reserve fails before evidence is promoted."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    _attach_resource_evidence(evidence, tmp_path)
    evidence['storage_preflight']['passed'] = False

    result = evaluator.evaluate_run(evidence)
    assert result['status'] == 'FAIL'
    assert 'home_storage_preflight' in result['failures']


@pytest.mark.parametrize('artifact', ['shadow.mcap', 'bag',
                                      'bag_quarantine.json'])
def test_bags_off_run_rejects_any_bag_artifact(tmp_path, artifact):
    """A bags-off run cannot hide MCAP, bag directories, or quarantine."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    _attach_resource_evidence(evidence, tmp_path)
    path = tmp_path / artifact
    if artifact == 'bag':
        path.mkdir()
    else:
        path.write_text('unexpected')

    result = evaluator.evaluate_run(evidence, tmp_path / 'evidence.json')

    assert result['status'] == 'FAIL'
    assert 'full_matrix_bag_artifacts_absent' in result['failures']


def test_evaluator_rejects_noncanonical_contract_and_resource_path(tmp_path):
    """Embedded contracts and resource paths cannot redirect validation."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    run_dir = tmp_path / evidence['run_id']
    run_dir.mkdir()
    _attach_resource_evidence(evidence, run_dir)
    evidence_path = run_dir / 'evidence.json'
    canonical = json.loads(json.dumps(evidence['contract']))
    evidence['contract']['stop_deadline_s'] += 1.0

    result = evaluator.evaluate_run(
        evidence, evidence_path, canonical, 'unit-contract-sha256')
    assert 'canonical_contract_match' in result['failures']

    evidence['contract'] = canonical
    outside = tmp_path / 'gazebo_resources.jsonl'
    outside.write_text('{}\n')
    evidence['resource_evidence']['gazebo'].update({
        'path': str(outside), 'size_bytes': outside.stat().st_size,
        'sha256': _load(
            'sim_collision_monitor_contract.py').sha256_file(outside),
    })
    result = evaluator.evaluate_run(
        evidence, evidence_path, canonical, 'unit-contract-sha256')
    assert 'resource_files_match_evidence' in result['failures']


def test_evaluator_rejects_missing_contact_subscription(tmp_path):
    """A connected topic is insufficient without a created subscriber."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    _attach_resource_evidence(evidence, tmp_path)
    evidence['contact_subscription_created'] = False

    result = evaluator.evaluate_run(evidence, tmp_path / 'evidence.json')
    assert 'contact_subscription_created' in result['failures']


def test_evaluator_recomputes_events_scalars_resources_and_retention(tmp_path):
    """Copied PASS scalars cannot hide damaged primary evidence."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('sudden_obstacle_stop_resume')
    _attach_resource_evidence(evidence, tmp_path)
    evidence_path = tmp_path / 'evidence.json'

    evidence['events'] = []
    assert 'required_events_ordered' in evaluator.evaluate_run(
        evidence, evidence_path)['failures']
    evidence = _passing_evidence('sudden_obstacle_stop_resume')
    _attach_resource_evidence(evidence, tmp_path)
    sidecar_path = Path(evidence['scan_gate_evidence']['path'])
    sidecar = json.loads(sidecar_path.read_text())
    sidecar['scan_publish_to_zero_receive_steady_s'] += 0.01
    sidecar_path.write_text(json.dumps(sidecar))
    evidence['scan_gate_evidence'].update({
        'size_bytes': sidecar_path.stat().st_size,
        'sha256': _load(
            'sim_collision_monitor_contract.py').sha256_file(sidecar_path),
    })
    assert 'scan_gate_evidence_scope' in evaluator.evaluate_run(
        evidence, evidence_path)['failures']
    evidence = _passing_evidence('sudden_obstacle_stop_resume')
    _attach_resource_evidence(evidence, tmp_path)
    evidence['resource_evidence']['gazebo'][
        'median_cpu_pct_one_core'] = 999.0
    assert 'resource_files_match_evidence' in evaluator.evaluate_run(
        evidence, evidence_path)['failures']
    evidence = _passing_evidence('sudden_obstacle_stop_resume')
    _attach_resource_evidence(evidence, tmp_path)
    trace = Path(evidence['resource_evidence']['gazebo']['path'])
    rows = [json.loads(line) for line in trace.read_text().splitlines()]
    rows[1]['cpu_pct_one_core'] = 0.0
    trace.write_text('\n'.join(json.dumps(row) for row in rows) + '\n')
    evidence['resource_evidence']['gazebo'].update({
        'sha256': _load(
            'sim_collision_monitor_contract.py').sha256_file(trace),
        'size_bytes': trace.stat().st_size,
        'median_cpu_pct_one_core': 0.0,
        'p95_cpu_pct_one_core': 0.0,
    })
    assert 'resource_files_match_evidence' in evaluator.evaluate_run(
        evidence, evidence_path)['failures']
    evidence = _passing_evidence('sudden_obstacle_stop_resume')
    _attach_resource_evidence(evidence, tmp_path)
    (tmp_path / 'unexpected.log').write_text('unexpected')
    assert 'retained_size_recomputed' in evaluator.evaluate_run(
        evidence, evidence_path)['failures']


@pytest.mark.parametrize('mutation', [
    'missing', 'wrong_path', 'stamp_bool', 'publish_float', 'zero_before',
    'range_bool', 'point_string', 'delta', 'raw_points', 'contract',
    'source_hash', 'arm_order', 'duplicate_key'])
def test_evaluator_rejects_scan_gate_sidecar_tampering(tmp_path, mutation):
    """The same-process reaction evidence is canonical and fail-closed."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract_module = _load('sim_collision_monitor_contract.py')
    evidence = _passing_evidence('sudden_obstacle_stop_resume')
    _attach_resource_evidence(evidence, tmp_path)
    evidence_path = tmp_path / 'evidence.json'
    sidecar_path = Path(evidence['scan_gate_evidence']['path'])
    sidecar = json.loads(sidecar_path.read_text())
    if mutation == 'missing':
        sidecar_path.unlink()
    elif mutation == 'wrong_path':
        outside = tmp_path.parent / f'{tmp_path.name}_gate.json'
        outside.write_text(sidecar_path.read_text())
        evidence['scan_gate_evidence']['path'] = str(outside)
    elif mutation == 'duplicate_key':
        sidecar_path.write_text('{"schema_version":1,"schema_version":1}\n')
    else:
        if mutation == 'stamp_bool':
            sidecar['stamp_ns'] = True
        elif mutation == 'publish_float':
            sidecar['publish_steady_ns'] = 100_000_000.0
        elif mutation == 'zero_before':
            sidecar['zero_receive_steady_ns'] = 99_000_000
        elif mutation == 'range_bool':
            sidecar['ranges_m'][0] = True
        elif mutation == 'point_string':
            sidecar['stop_zone_points'][0][0] = '0.1'
        elif mutation == 'delta':
            sidecar['scan_publish_to_zero_receive_steady_s'] = 0.04
        elif mutation == 'raw_points':
            sidecar['stop_zone_points'][0][0] += 0.01
        elif mutation == 'contract':
            sidecar['contract_sha256'] = '0' * 64
        elif mutation == 'source_hash':
            sidecar['source_sha256'] = '0' * 64
        else:
            sidecar['arm_receive_steady_ns'] = 80_000_000
        sidecar_path.write_text(json.dumps(sidecar, sort_keys=True) + '\n')
    if sidecar_path.exists():
        evidence['scan_gate_evidence'].update({
            'size_bytes': sidecar_path.stat().st_size,
            'sha256': contract_module.sha256_file(sidecar_path),
        })

    result = evaluator.evaluate_run(evidence, evidence_path)

    assert 'scan_gate_evidence_scope' in result['failures']


def test_non_sudden_run_rejects_scan_gate_sidecar_record(tmp_path):
    """Only sudden-obstacle runs may retain reaction sidecar evidence."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    _attach_resource_evidence(evidence, tmp_path)
    evidence['scan_gate_evidence'] = {
        'path': str(tmp_path / 'scan_gate_evidence.json'),
        'size_bytes': 0,
        'sha256': '0' * 64,
    }

    result = evaluator.evaluate_run(evidence, tmp_path / 'evidence.json')

    assert 'scan_gate_evidence_scope' in result['failures']


@pytest.mark.parametrize('mutation, expected_failure', [
    ('signed_ros_diagnostic', 'ros_clock_offset_diagnostic'),
    ('host_monotonic_order', 'same_host_monotonic_causal_order'),
])
def test_evaluator_rejects_scan_observer_clock_tampering(
        tmp_path, mutation, expected_failure):
    """Signed ROS diagnostics and same-host causal order are recomputed."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract_module = _load('sim_collision_monitor_contract.py')
    evidence = _passing_evidence('sudden_obstacle_stop_resume')
    _attach_resource_evidence(evidence, tmp_path)
    sidecar_path = Path(evidence['scan_gate_evidence']['path'])
    sidecar = json.loads(sidecar_path.read_text())
    if mutation == 'signed_ros_diagnostic':
        evidence['scan_header_to_stop_observer_ros_signed_s'] += 0.01
    else:
        sidecar['publish_steady_ns'] = evidence['stop_state_steady_ns']
        sidecar['scan_publish_to_zero_receive_steady_s'] = (
            sidecar['zero_receive_steady_ns']
            - sidecar['publish_steady_ns']) / 1e9
        sidecar_path.write_text(json.dumps(sidecar, sort_keys=True) + '\n')
        evidence['scan_gate_evidence'].update({
            'size_bytes': sidecar_path.stat().st_size,
            'sha256': contract_module.sha256_file(sidecar_path),
        })

    result = evaluator.evaluate_run(evidence, tmp_path / 'evidence.json')

    assert expected_failure in result['failures']


@pytest.mark.parametrize('payload', [
    '{"contact_count":0,"contact_count":1}',
    '{"contract":{"schema_version":1,"schema_version":2}}',
])
def test_strict_json_loader_rejects_duplicate_evidence_keys(payload):
    """Duplicate JSON keys cannot change evidence or contract meaning."""
    evaluator = _load('evaluate_sim_collision_monitor.py')

    with pytest.raises(ValueError, match='duplicate JSON key'):
        evaluator.strict_json_loads(payload)


@pytest.mark.parametrize('mutation', [
    'duplicate_steady', 'missing_ros', 'stop_scalar', 'zero_scalar',
    'physical_scalar', 'arm_scalar', 'freeze_scalar', 'duplicate_ack',
    'post_succeeded_ack', 'arm_after_request'])
def test_evaluator_rejects_event_timestamp_tampering(tmp_path, mutation):
    """Required event order and duplicated timestamp scalars are immutable."""
    scenario = ('scan_timeout_stop_resume'
                if mutation == 'freeze_scalar'
                else 'sudden_obstacle_stop_resume')
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence(scenario)
    _attach_resource_evidence(evidence, tmp_path)
    events = {event['name']: event for event in evidence['events']}
    if mutation == 'duplicate_steady':
        events['stop_state']['steady_ns'] = events['final_zero']['steady_ns']
    elif mutation == 'missing_ros':
        events['final_zero'].pop('ros_ns')
    elif mutation == 'stop_scalar':
        evidence['stop_state_steady_ns'] += 1
    elif mutation == 'zero_scalar':
        evidence['zero_ros_ns'] += 1
    elif mutation == 'physical_scalar':
        evidence['physical_stop_steady_ns'] += 1
    elif mutation == 'arm_scalar':
        evidence['scan_gate_arm_ack_steady_ns'] += 1
    elif mutation == 'duplicate_ack':
        evidence['events'].append(dict(events['obstacle_set_pose_ack']))
    elif mutation == 'post_succeeded_ack':
        events['obstacle_set_pose_ack']['steady_ns'] = 180_000_000
        evidence['obstacle_activation_steady_ns'] = 180_000_000
        evidence['obstacle_ack_to_zero_steady_s'] = -0.05
    elif mutation == 'arm_after_request':
        events['scan_gate_arm_ack']['steady_ns'] = 95_000_000
        evidence['scan_gate_arm_ack_steady_ns'] = 95_000_000
    else:
        evidence['freeze_ros_ns'] += 1

    result = evaluator.evaluate_run(evidence, tmp_path / 'evidence.json')

    assert result['status'] == 'FAIL'
    assert ({'required_events_ordered', 'event_scalar_bindings',
             'provenance_partial_order'}
            & set(result['failures']))


def test_aggregate_rejects_missing_or_tampered_scan_gap_profile(tmp_path):
    """The real scan-gap input is rehashed and independently re-read."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _load('sim_collision_monitor_contract.py')
    canonical = _passing_evidence('clear_baseline')['contract']
    contract_path = tmp_path / 'contract.json'
    _write_preparation_identity(
        tmp_path, canonical, contract_path, contract)
    profile = Path(canonical['scan_gap_provenance']['path'])
    assert evaluator._preparation_identity_valid(
        tmp_path, contract_path) is True
    shadow = tmp_path / 'assets/shadow.yaml'
    shadow.write_text('unexpected')
    assert evaluator._preparation_identity_valid(
        tmp_path, contract_path) is False
    shadow.unlink()
    profile.write_text('{}')
    assert evaluator._preparation_identity_valid(
        tmp_path, contract_path) is False


def test_preparation_identity_rejects_empty_or_tampered_rmw(tmp_path):
    """Runtime middleware identity cannot be omitted or self-asserted."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _load('sim_collision_monitor_contract.py')
    canonical = _passing_evidence('clear_baseline')['contract']
    contract_path = tmp_path / 'contract.json'
    _write_preparation_identity(
        tmp_path, canonical, contract_path, contract)
    runtime_path = tmp_path / 'runtime_manifest.json'
    runtime = json.loads(runtime_path.read_text())
    assert evaluator._preparation_identity_valid(
        tmp_path, contract_path) is True
    runtime['evaluation_rmw'] = {}
    runtime_path.write_text(json.dumps(runtime))
    assert evaluator._preparation_identity_valid(
        tmp_path, contract_path) is False


def test_runtime_manifest_binds_exact_matrix_domain_plan(tmp_path):
    """Unused or relabeled ROS domains invalidate the matrix manifest."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _load('sim_collision_monitor_contract.py')
    canonical = _passing_evidence('clear_baseline')['contract']
    contract_path = tmp_path / 'contract.json'
    _write_preparation_identity(
        tmp_path, canonical, contract_path, contract)
    runtime_path = tmp_path / 'runtime_manifest.json'
    runtime = json.loads(runtime_path.read_text())
    assert evaluator._preparation_identity_valid(
        tmp_path, contract_path) is True

    runtime['planned_runs'][0]['domain_id'] += 1
    runtime_path.write_text(json.dumps(runtime))

    assert evaluator._preparation_identity_valid(
        tmp_path, contract_path) is False


def test_promotion_rmw_comparison_rejects_identifier_or_hash_change():
    """Representative replay cannot switch middleware implementation."""
    runner = _load('run_sim_collision_monitor_eval.py')
    rmw = {
        'requested_identifier': 'rmw_cyclonedds_cpp',
        'actual_identifier': 'rmw_cyclonedds_cpp', 'version': 'unit',
        'source': 'temporary_deb_extract', 'library_size_bytes': 10,
        'library_sha256': 'a' * 64, 'environment_fields': {'RMW': 'cyclone'},
        'environment_sha256': 'b' * 64,
    }
    source = {'evaluation_rmw_identity': runner.rmw_comparison_identity(rmw)}
    assert runner.promotion_rmw_matches(source, rmw)
    changed = dict(rmw, actual_identifier='rmw_fastrtps_cpp')
    assert not runner.promotion_rmw_matches(source, changed)
    changed = dict(rmw, library_sha256='c' * 64)
    assert not runner.promotion_rmw_matches(source, changed)


@pytest.mark.parametrize('mode_args,message', [
    ([], '--prepared-root is required for matrix evaluation'),
    (['--retain-representatives', '--source-matrix-root', '/tmp/source',
      '--prepared-root', '/tmp/unused'],
     '--prepared-root is not accepted for promotion')])
def test_runner_cli_separates_matrix_and_promotion_inputs(
        monkeypatch, mode_args, message):
    """Promotion never depends on an unused temporary prepared root."""
    runner = _load('run_sim_collision_monitor_eval.py')
    monkeypatch.setattr(sys, 'argv', [
        'run_sim_collision_monitor_eval.py', '--output-root', '/tmp/output',
        '--domain-id', '160', *mode_args])

    with pytest.raises(SystemExit, match=message):
        runner.main()


def test_preparation_identity_rejects_tampered_retained_runtime_bytes(
        tmp_path):
    """Durable source and middleware copies must remain byte-identical."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _load('sim_collision_monitor_contract.py')
    canonical = _passing_evidence('clear_baseline')['contract']
    source_root = tmp_path / 'source_copy'
    source_root.mkdir()
    contract_path = source_root / 'contract.json'
    _write_preparation_identity(
        source_root, canonical, contract_path, contract)
    runtime = json.loads(
        (source_root / 'runtime_manifest.json').read_text())
    assert evaluator._preparation_identity_valid(
        source_root, contract_path) is True

    retained_source = Path(
        runtime['harness_sources']['scenario']['retained_path'])
    retained_source.write_bytes(retained_source.read_bytes() + b'\n')
    assert evaluator._preparation_identity_valid(
        source_root, contract_path) is False

    library_root = tmp_path / 'library_copy'
    library_root.mkdir()
    contract_path = library_root / 'contract.json'
    _write_preparation_identity(
        library_root, canonical, contract_path, contract)
    runtime = json.loads(
        (library_root / 'runtime_manifest.json').read_text())
    retained_library = Path(
        runtime['evaluation_rmw']['retained_library_path'])
    retained_library.write_bytes(retained_library.read_bytes() + b'x')
    assert evaluator._preparation_identity_valid(
        library_root, contract_path) is False


def test_runner_rejects_extra_prepared_asset(tmp_path):
    """A manifest cannot hide an untracked prepared asset."""
    runner = _load('run_sim_collision_monitor_eval.py')
    prepare = _load('prepare_sim_collision_monitor_run.py')
    production = tmp_path / 'nav2.yaml'
    production.write_bytes((
        ROOT / 'jdamr_cube_navigation/config/nav2_params.yaml').read_bytes())
    world = tmp_path / 'world.sdf'
    world.write_text(
        '<plugin filename="gz-sim-contact-system"/>'
        '<max_step_size>0.001</max_step_size>')
    prepared = tmp_path / 'prepared'
    prepare.prepare(production, world, prepared)
    (prepared / 'shadow.yaml').write_text('unexpected')

    with pytest.raises(ValueError, match='file set mismatch'):
        runner.validate_preparation_manifest(prepared)


def test_aggregate_sums_matrix_retention_and_validates_bag_hash(tmp_path):
    """Aggregate retention is global and representative records bind bytes."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _load('sim_collision_monitor_contract.py')
    paths = []
    canonical = _passing_evidence('clear_baseline')['contract']
    contract_path = tmp_path / 'contract.json'
    _write_preparation_identity(
        tmp_path, canonical, contract_path, contract)
    canonical_sha256 = contract.sha256_file(contract_path)
    for index, item in enumerate(contract.scenario_matrix()):
        document = _passing_evidence(item['scenario'])
        resource_root = tmp_path / item['run_id']
        resource_root.mkdir()
        _attach_resource_evidence(document, resource_root)
        document.update(item)
        document['domain_id'] = 160 + index
        document['contract'] = canonical
        document['canonical_contract_sha256'] = canonical_sha256
        if item['scenario'] == 'sudden_obstacle_stop_resume':
            Path(document['scan_gate_evidence']['path']).unlink()
            document.pop('scan_gate_evidence')
            _attach_scan_gate_evidence(document, resource_root)
            document['retained_run_bytes'] = sum(
                child.stat().st_size for child in resource_root.rglob('*')
                if child.is_file())
        path = resource_root / 'evidence.json'
        path.write_text(json.dumps(document))
        paths.append(path)
    result = evaluator.aggregate(paths)

    assert result['status'] == 'PASS'
    actual_bytes = evaluator._tree_size_excluding(
        tmp_path, {'aggregate.json', 'retention_manifest.json'})
    assert result['total_retained_bytes'] == actual_bytes
    bags = []
    for scenario in contract.SCENARIOS:
        path = tmp_path / f'{scenario}.mcap'
        path.write_bytes(scenario.encode())
        bags.append({
            'scenario': scenario,
            'path': str(path),
            'size_bytes': path.stat().st_size,
            'sha256': contract.sha256_file(path),
        })
    with_bags = evaluator.aggregate(
        paths, {'representative_bags': bags})
    assert with_bags['status'] == 'FAIL'
    assert with_bags['total_retained_bytes'] == evaluator._tree_size_excluding(
        tmp_path, {'aggregate.json', 'retention_manifest.json'})


def test_quarantine_manifest_never_deletes_incomplete_bags(tmp_path):
    """Incomplete bags are listed for cleanup without implicit deletion."""
    scenario = _load(
        '../jdamr_cube_navigation/sim_collision_monitor_scenario.py')
    bag = tmp_path / 'partial.mcap'
    bag.write_bytes(b'partial')
    manifest = tmp_path / 'quarantine.json'

    scenario.write_quarantine_manifest(manifest, [bag])
    document = json.loads(manifest.read_text())

    assert bag.exists()
    assert document['cleanup_performed'] is False
    assert document['incomplete_bags'] == [str(bag.resolve())]


@pytest.mark.parametrize('metadata_text', [None, 'broken: [', '{}'])
def test_representative_bag_inspection_fails_closed_without_raising(
        tmp_path, metadata_text):
    """Missing/corrupt metadata and short MCAP bytes remain quarantinable."""
    runner = _load('run_sim_collision_monitor_eval.py')
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    (bag_dir / 'clear_baseline__seed_11_0.mcap').write_bytes(b'short')
    if metadata_text is not None:
        (bag_dir / 'metadata.yaml').write_text(metadata_text)
    item = {
        'run_id': 'clear_baseline__seed_11',
        'scenario': 'clear_baseline', 'seed': 11}

    evidence, error, paths = runner.inspect_representative_bag(
        bag_dir, item, ('/tf',))

    assert evidence is None
    assert isinstance(error, str) and error
    assert len(paths) == 1


def test_concatenated_pose_info_requires_one_stable_entity_id():
    """Gazebo concatenated JSON retains the latest pose and stable ID."""
    scenario = _load(
        '../jdamr_cube_navigation/sim_collision_monitor_scenario.py')
    output = (
        '{"pose":[{"name":"probe","id":7,'
        '"position":{"x":1,"y":2,"z":3}}]}\n'
        '{"pose":[{"name":"probe","id":7,'
        '"position":{"x":4,"y":5,"z":6}}]}')

    assert scenario.parse_entity_pose_info(output, 'probe') == {
        'entity_id': 7, 'pose_m': [4.0, 5.0, 6.0],
        'defaulted_axes': [], 'orientation_yaw_rad': 0.0,
        'orientation_defaulted': True}


@pytest.mark.parametrize('pose', [
    {'id': True, 'position': {'x': 1, 'y': 2, 'z': 3}},
    {'id': 7.0, 'position': {'x': 1, 'y': 2, 'z': 3}},
    {'id': '7', 'position': {'x': 1, 'y': 2, 'z': 3}},
    {'id': 7, 'position': {'x': True, 'y': 2, 'z': 3}},
    {'id': 7, 'position': {'x': '1', 'y': 2, 'z': 3}},
])
def test_entity_pose_parser_rejects_coercible_or_incomplete_values(pose):
    """Gazebo entity identity and xyz require exact numeric source values."""
    scenario = _load(
        '../jdamr_cube_navigation/sim_collision_monitor_scenario.py')
    payload = json.dumps({'pose': [{'name': 'probe', **pose}]})

    with pytest.raises(ValueError):
        scenario.parse_entity_pose_info(payload, 'probe')


def test_entity_pose_parser_records_protojson_defaulted_axes():
    """Omitted ProtoJSON numeric defaults remain explicit provenance."""
    scenario = _load(
        '../jdamr_cube_navigation/sim_collision_monitor_scenario.py')
    payload = json.dumps({
        'pose': [{'name': 'probe', 'id': 7,
                  'position': {'y': 40.0, 'z': 0.5}}]})

    assert scenario.parse_entity_pose_info(payload, 'probe') == {
        'entity_id': 7, 'pose_m': [0.0, 40.0, 0.5],
        'defaulted_axes': ['x'], 'orientation_yaw_rad': 0.0,
        'orientation_defaulted': True}


def test_scan_gate_latches_first_publish_and_first_post_trigger_zero(
        tmp_path, monkeypatch):
    """The same-process gate cannot move either reaction endpoint."""
    gate = _load('../jdamr_cube_navigation/sim_scan_gate.py')
    node = gate.SimScanGate.__new__(gate.SimScanGate)
    evidence_path = tmp_path / 'scan_gate_evidence.json'
    node.args = SimpleNamespace(
        evidence=evidence_path, run_id='sudden_obstacle_stop_resume__seed_11',
        scenario='sudden_obstacle_stop_resume', seed=11)
    node.contract = _load(
        'sim_collision_monitor_contract.py').derive_contract(20.0, 0.001)
    node.contract_sha256 = 'unit-contract-sha256'
    node.moving_observed = True
    node.pending_trigger = None
    node.reaction_armed = True
    node.arm_receive_steady_ns = 60
    node.monitor_frozen = False
    node.navigation_publisher = SimpleNamespace(publish=lambda message: None)
    node.monitor_publisher = SimpleNamespace(publish=lambda message: None)
    timestamps = iter([100, 130])
    monkeypatch.setattr(gate.time, 'monotonic_ns', lambda: next(timestamps))
    scan = lambda stamp: SimpleNamespace(  # noqa: E731
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=stamp),
            frame_id='laser_link'),
        ranges=[0.1] * 3, angle_min=math.pi, angle_increment=0.0)

    node.scan_callback(scan(11))
    node.scan_callback(scan(12))
    node.cmd_callback(SimpleNamespace(
        linear=SimpleNamespace(x=0.0, y=0.0),
        angular=SimpleNamespace(z=0.0)))
    node.cmd_callback(SimpleNamespace(
        linear=SimpleNamespace(x=0.0, y=0.0),
        angular=SimpleNamespace(z=0.0)))

    document = json.loads(evidence_path.read_text())
    assert document['stamp_ns'] == 11
    assert document['publish_steady_ns'] == 100
    assert document['zero_receive_steady_ns'] == 130


def test_sudden_trigger_uses_one_post_arm_pose_snapshot():
    """Concurrent ground-truth updates cannot split trigger and entity pose."""
    module = _load(
        '../jdamr_cube_navigation/sim_collision_monitor_scenario.py')
    node = module.CollisionMonitorScenario.__new__(
        module.CollisionMonitorScenario)
    node.args = SimpleNamespace(scenario='sudden_obstacle_stop_resume')
    node.contract = _load(
        'sim_collision_monitor_contract.py').derive_contract(20.0, 0.001)
    node.world_pose = (1.0, 2.0)
    node.raw_scan_count = 3
    node.navigation_scan_count = 4
    node.last_monitor_scan_stamp_ns = 5
    node.events = []
    node._event = lambda name: {'steady_ns': 10, 'ros_ns': 20}

    def arm():
        node.world_pose = (1.1, 2.1)
        return True

    captured = []
    node._arm_reaction_capture = arm
    node._set_entity_pose = lambda pose, transition: captured.append(pose) or True
    node.obstacle_center_m = None
    node.obstacle_active = False
    node.obstacle_request_steady_ns = None
    node.obstacle_request_ros_ns = None

    assert node._trigger() is True
    offset = node.contract['sudden_obstacle']['activation_center_offset_x_m']
    assert node.trigger_pose == (1.1, 2.1)
    assert captured == [(1.1 + offset, 2.1, 0.5)]


def test_crossing_obstacle_uses_fast_unverified_steps(monkeypatch):
    """Only the final pedestrian pose needs the expensive truth readback."""
    module = _load(
        '../jdamr_cube_navigation/sim_collision_monitor_scenario.py')
    node = module.CollisionMonitorScenario.__new__(
        module.CollisionMonitorScenario)
    node.args = SimpleNamespace(
        obstacle_crossing_s=0.3, obstacle_entry_side='right')
    node.events = []
    node.obstacle_active = False
    calls = []
    node._event = lambda name, **details: {
        'name': name, **details}
    node._set_entity_pose = lambda pose, transition=None, verify=True: (
        calls.append((pose, transition, verify)) or True)
    monkeypatch.setattr(module.time, 'sleep', lambda duration: None)

    assert node._activate_crossing_obstacle(2.0, 0.0) is True
    assert len(calls) == 4
    assert calls[0] == ((2.0, -0.85, 0.5), None, False)
    assert calls[-1] == ((2.0, 0.0, 0.5), 'obstacle', True)


@pytest.mark.parametrize('scenario,request_name', [
    ('sudden_obstacle_stop_resume', 'clear_set_pose_requested'),
    ('scan_timeout_stop_resume', 'monitor_scan_unfreeze_requested'),
])
def test_clear_request_anchor_precedes_concurrent_resume(
        scenario, request_name):
    """Resume observation cannot overtake the causal clear request event."""
    module = _load(
        '../jdamr_cube_navigation/sim_collision_monitor_scenario.py')
    node = module.CollisionMonitorScenario.__new__(
        module.CollisionMonitorScenario)
    node.args = SimpleNamespace(scenario=scenario)
    node.contract = _load(
        'sim_collision_monitor_contract.py').derive_contract(20.0, 0.001)
    node.events = []
    node.last_monitor_scan_stamp_ns = 10
    node.clear_request_ros_ns = None
    node.clear_request_steady_ns = None
    node.unfreeze_request_ros_ns = None
    node.unfreeze_request_steady_ns = None
    node.obstacle_active = True

    def event(name, **details):
        record = {
            'name': name, 'steady_ns': len(node.events) + 1,
            'ros_ns': len(node.events) + 1, **details}
        node.events.append(record)
        return record

    def concurrent_resume(*args, **kwargs):
        event('do_nothing_after_stop')
        return True

    node._event = event
    node._freeze = concurrent_resume
    node._set_entity_pose = concurrent_resume

    assert node._clear_trigger() is True
    names = [item['name'] for item in node.events]
    assert names.index(request_name) < names.index('do_nothing_after_stop')


def test_evaluator_rejects_unbounded_stop_distance():
    """Observed stopping distance must not exceed the derived bound."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('sudden_obstacle_stop_resume')
    evidence['stop_distance_m'] = (
        evidence['contract']['maximum_stop_distance_m'] + 0.001)

    assert 'bounded_stop_distance' in evaluator.evaluate_run(
        evidence)['failures']


def test_evaluator_rejects_contact_even_when_stop_resumes():
    """A successful action cannot launder scenario-obstacle contact."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('sudden_obstacle_stop_resume')
    evidence['contact_count'] = 1

    assert 'contact_zero' in evaluator.evaluate_run(evidence)['failures']


@pytest.mark.parametrize('mutation', [
    'empty', 'id_mismatch', 'id_bool', 'id_float', 'id_string',
    'pose_tamper', 'error_tamper'])
def test_evaluator_rejects_tampered_entity_state_truth(mutation):
    """Obstacle activation and removal poses are independently recomputed."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('sudden_obstacle_stop_resume')
    if mutation == 'empty':
        evidence['entity_states'] = []
    elif mutation == 'id_mismatch':
        evidence['entity_states'][1]['entity_id'] = 61
    elif mutation == 'id_bool':
        evidence['entity_states'][0]['entity_id'] = True
        evidence['entity_states'][1]['entity_id'] = True
    elif mutation == 'id_float':
        evidence['entity_states'][0]['entity_id'] = 60.0
        evidence['entity_states'][1]['entity_id'] = 60.0
    elif mutation == 'id_string':
        evidence['entity_states'][0]['entity_id'] = '60'
        evidence['entity_states'][1]['entity_id'] = '60'
    elif mutation == 'pose_tamper':
        evidence['entity_states'][0]['pose_m'][0] += 1.0
    else:
        evidence['entity_states'][0]['position_error_m'] = 0.01

    assert 'entity_state_truth' in evaluator.evaluate_run(
        evidence)['failures']


@pytest.mark.parametrize('mutation,expected_failure', [
    ('missing_samples', 'positive_clearance'),
    ('scan_fallback', 'positive_clearance'),
    ('timeout_claim', 'clearance_evidence'),
])
def test_evaluator_separates_gt_clearance_from_scan_range(
        mutation, expected_failure):
    """Keep LiDAR minimum range separate from obstacle GT clearance."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    scenario = ('scan_timeout_stop_resume'
                if mutation == 'timeout_claim'
                else 'sudden_obstacle_stop_resume')
    evidence = _passing_evidence(scenario)
    if mutation == 'missing_samples':
        evidence['clearance_sample_count'] = 0
    elif mutation == 'scan_fallback':
        evidence['footprint_to_obstacle_clearance_m'] = None
    else:
        evidence['footprint_to_obstacle_clearance_m'] = 0.5

    assert expected_failure in evaluator.evaluate_run(evidence)['failures']


@pytest.mark.parametrize('field,value,expected_failure', [
    ('goal_send_count', True, 'single_goal_uuid'),
    ('goal_send_count', 1.0, 'single_goal_uuid'),
    ('goal_send_count', '1', 'single_goal_uuid'),
    ('goal_cancel_count', False, 'goal_not_cancelled'),
    ('contact_count', False, 'contact_zero'),
    ('contact_matched_publisher_count_max', True,
     'contact_source_connected'),
    ('stop_state_count', True, 'stop_observed'),
    ('stop_action_type', True, 'stop_observed'),
    ('stop_action_type', 1.0, 'stop_observed'),
    ('stop_action_type', '1', 'stop_observed'),
    ('resume_action_type', False, 'resume_clear'),
    ('final_action_type', False, 'resume_clear'),
])
def test_evaluator_rejects_boolean_count_and_enum_fields(
        field, value, expected_failure):
    """JSON booleans cannot masquerade as integer counts or action enums."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('scan_timeout_stop_resume')
    evidence[field] = value

    assert expected_failure in evaluator.evaluate_run(evidence)['failures']


@pytest.mark.parametrize('mutation', [
    'status', 'errors', 'pre_measurement', 'unexpected_code',
    'logged_pre_crash', 'survivor_signal_bool', 'finalized_before_terminal',
    'pre_crash_log_laundered', 'line_crossing_offset', 'wrong_log',
    'log_hash', 'log_size', 'arbitrary_pid', 'missing_identity',
    'duplicate_pid', 'sampler_early_exit'])
def test_evaluator_rejects_teardown_evidence_tampering(
        tmp_path, mutation):
    """Teardown success is recomputed from typed per-process outcomes."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    _attach_resource_evidence(evidence, tmp_path)
    teardown = evidence['teardown']
    navigation = next(
        record for record in teardown['processes']
        if record['name'] == 'navigation')
    if mutation == 'status':
        teardown['status'] = 'FAIL'
    elif mutation == 'errors':
        teardown['errors'] = ['hidden failure']
    elif mutation == 'pre_measurement':
        navigation['stop_requested_steady_ns'] = 999
    elif mutation == 'unexpected_code':
        navigation['returncode_after_stop'] = -9
    elif mutation == 'logged_pre_crash':
        navigation['logged_process_exits_before_stop'] = [{}]
    elif mutation == 'survivor_signal_bool':
        navigation['requested_signal'] = True
    elif mutation == 'finalized_before_terminal':
        teardown['measurement_finalized_steady_ns'] = 19
    elif mutation in {'pre_crash_log_laundered', 'line_crossing_offset'}:
        log_path = Path(navigation['log_path'])
        log_path.write_text(_launch_death_line(
            'component_container_isolated-1', 2, -6,
            '/opt/ros/jazzy/lib/rclcpp_components/'
            'component_container_isolated --ros-args'))
        navigation['log_size_bytes'] = log_path.stat().st_size
        navigation['log_sha256'] = _load(
            'sim_collision_monitor_contract.py').sha256_file(log_path)
        navigation['log_pre_stop_offset_bytes'] = (
            log_path.stat().st_size if mutation == 'pre_crash_log_laundered'
            else 10)
        navigation['logged_process_exits_before_stop'] = []
        navigation['logged_process_exits_after_stop'] = []
    elif mutation == 'wrong_log':
        navigation['log_path'] = str(tmp_path / 'gazebo.log')
    elif mutation == 'log_hash':
        navigation['log_sha256'] = '0' * 64
    elif mutation == 'log_size':
        navigation['log_size_bytes'] = 1
    elif mutation == 'arbitrary_pid':
        navigation['pid'] = 99_999
    elif mutation == 'missing_identity':
        evidence['process_identity'].pop()
    elif mutation == 'duplicate_pid':
        evidence['process_identity'][-1]['pid'] = (
            evidence['process_identity'][0]['pid'])
    else:
        sampler = next(
            record for record in teardown['processes']
            if record['name'] == 'nav2_resource_sampler')
        sampler['returncode_at_measurement'] = 0

    assert 'teardown_evidence' in evaluator.evaluate_run(
        evidence, tmp_path / 'evidence.json')['failures']


def _launch_death_line(label, pid, exit_code, command):
    """Build one ros2 launch child-death line without hand quoting."""
    return (f'[ERROR] [{label}]: process has died [pid {pid}, '
            f'exit code {exit_code}, cmd {command!r}].\n')


def test_runner_classifies_expected_post_measurement_sigabrt(
        tmp_path, monkeypatch):
    """A Nav2 abort caused inside the runner cleanup window is explicit."""
    runner = _load('run_sim_collision_monitor_eval.py')
    log_path = tmp_path / 'navigation.log'
    log_path.write_text('ready\n')

    class Process:
        pid = 12345
        returncode = None

        def poll(self):
            return self.returncode

    process = Process()

    def stop(target):
        log_path.write_text(
            log_path.read_text()
            + _launch_death_line(
                'component_container_isolated-1', 2, -6,
                '/opt/ros/jazzy/lib/rclcpp_components/'
                'component_container_isolated --ros-args'))
        target.returncode = -6

    monkeypatch.setattr(runner, '_stop', stop)
    record, error = runner._stop_with_evidence(
        'navigation', process, log_path, 1, None)

    assert error is None
    assert record['runner_initiated'] is True
    assert record['requested_signal'] == 2
    assert record['returncode_after_stop'] == -6
    assert record['logged_process_exits_after_stop'][0]['exit_code'] == -6


def test_runner_accepts_exact_gazebo_parameter_bridge_cleanup_abort(
        tmp_path, monkeypatch):
    """The observed v48 bridge-only post-stop abort has a narrow allowlist."""
    runner = _load('run_sim_collision_monitor_eval.py')
    log_path = tmp_path / 'gazebo.log'
    log_path.write_bytes(b'ready\n')

    class Process:
        pid = 12345
        returncode = None

        def poll(self):
            return self.returncode

    process = Process()

    def stop(target):
        with log_path.open('ab') as stream:
            stream.write((_launch_death_line(
                'gazebo-1', 10, -2, 'ruby /opt/ros/jazzy/bin/gz sim')
                + _launch_death_line(
                    'parameter_bridge-4', 11, -6,
                    '/opt/ros/jazzy/lib/ros_gz_bridge/'
                    'parameter_bridge --ros-args')).encode())
        target.returncode = -2

    monkeypatch.setattr(runner, '_stop', stop)
    record, error = runner._stop_with_evidence(
        'gazebo', process, log_path, 1, None)

    assert error is None
    assert [item['label'] for item in
            record['logged_process_exits_after_stop']] == [
                'gazebo-1', 'parameter_bridge-4']


@pytest.mark.parametrize('mutation', [
    'wrong_child', 'wrong_command', 'other_code', 'duplicate', 'field'])
def test_gazebo_bridge_abort_allowlist_rejects_hostile_records(mutation):
    """No other Gazebo child death can borrow the bridge cleanup exception."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    record = {
        'label': 'parameter_bridge-4', 'pid': 11, 'exit_code': -6,
        'command': '/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge '
                   '--ros-args', 'byte_offset': 100}
    records = [record]
    if mutation == 'wrong_child':
        record['label'] = 'gazebo-1'
    elif mutation == 'wrong_command':
        record['command'] = '/tmp/parameter_bridge --ros-args'
    elif mutation == 'other_code':
        record['exit_code'] = -9
    elif mutation == 'duplicate':
        records.append(dict(record, pid=12, byte_offset=200))
    else:
        record['label'] = True

    assert evaluator._logged_process_exits_allowed('gazebo', records) is False


@pytest.mark.parametrize('mutation', [
    'malformed', 'invalid_utf8', 'duplicate', 'cross_label_same_pid'])
def test_launch_death_parser_fails_closed(mutation):
    """Every death marker is strict UTF-8 and identifies one unique child."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    normal = _launch_death_line(
        'gazebo-1', 10, -2, 'ruby /opt/ros/jazzy/bin/gz sim').encode()
    if mutation == 'malformed':
        content = b'process has died [pid 10, exit code -9]\n'
    elif mutation == 'invalid_utf8':
        content = (b'[ERROR] [bad-\xff]: process has died [pid 10, '
                   b"exit code -2, cmd 'ruby gz sim'].\n")
    elif mutation == 'duplicate':
        content = normal + normal
    else:
        content = normal + _launch_death_line(
            'parameter_bridge-4', 10, -6,
            '/opt/ros/jazzy/lib/ros_gz_bridge/'
            'parameter_bridge --ros-args').encode()

    valid, _ = evaluator._logged_process_exits(content)

    assert valid is False


@pytest.mark.parametrize('case', [
    'normal_scenario', 'pre_measurement_crash', 'unexpected_teardown'])
def test_runner_teardown_classification_is_fail_closed(
        tmp_path, monkeypatch, case):
    """Normal completion passes while early and unexpected exits fail."""
    runner = _load('run_sim_collision_monitor_eval.py')
    log_path = tmp_path / f'{case}.log'
    log_path.write_text('')

    class Process:
        pid = 12345

        def __init__(self, returncode):
            self.returncode = returncode

        def poll(self):
            return self.returncode

    if case == 'normal_scenario':
        process = Process(0)
        monkeypatch.setattr(runner, '_stop', lambda target: None)
        name = 'scenario'
        finalized = 1
    elif case == 'pre_measurement_crash':
        process = Process(-9)
        monkeypatch.setattr(runner, '_stop', lambda target: None)
        name = 'navigation'
        finalized = 1
    else:
        process = Process(None)

        def stop(target):
            target.returncode = -9

        monkeypatch.setattr(runner, '_stop', stop)
        name = 'gazebo'
        finalized = 1
    record, error = runner._stop_with_evidence(
        name, process, log_path, finalized,
        0 if case == 'normal_scenario' else (
            -9 if case == 'pre_measurement_crash' else None))

    if case == 'normal_scenario':
        assert error is None
        assert record['runner_initiated'] is False
    else:
        assert error is not None


def test_scan_gate_ignores_only_already_shutdown_rcl_error(monkeypatch):
    """Shutdown races stay quiet, while live-context RCLErrors propagate."""
    gate = _load('../jdamr_cube_navigation/sim_scan_gate.py')

    def fail(_node):
        raise gate.RCLError('context already shutdown')

    monkeypatch.setattr(gate.rclpy, 'spin', fail)
    monkeypatch.setattr(gate.rclpy, 'ok', lambda: False)
    gate.spin_until_shutdown(object())
    monkeypatch.setattr(gate.rclpy, 'ok', lambda: True)
    with pytest.raises(gate.RCLError):
        gate.spin_until_shutdown(object())


def test_evaluator_treats_terminal_pose_as_diagnostic_after_stateful_entry():
    """A stateful checker may drift after its exact tolerance entry."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    evidence['final_estimated_pose_m'] = [6.151, 0.0]
    evidence['terminal_estimated_to_gt_error_m'] = 0.151

    failures = evaluator.evaluate_run(evidence)['failures']
    assert 'goal_position_tolerance_entry' not in failures
    assert 'terminal_estimated_pose_diagnostic' not in failures


def test_evaluator_accepts_same_goal_success_without_optional_tf_witness():
    """A callback-order miss cannot erase an otherwise proven goal result."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    evidence['goal_position_tolerance_entry'] = None
    evidence['events'] = [
        event for event in evidence['events']
        if event['name'] != 'goal_position_tolerance_entered']

    failures = evaluator.evaluate_run(evidence)['failures']

    assert 'goal_position_tolerance_entry' not in failures
    assert 'same_goal_succeeded' not in failures


def test_evaluator_rejects_optional_tf_witness_field_event_mismatch():
    """An omitted witness is valid only when its duplicate event is absent."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    evidence['goal_position_tolerance_entry'] = None

    assert 'goal_position_tolerance_entry' in evaluator.evaluate_run(
        evidence)['failures']


@pytest.mark.parametrize('mutation,expected_failure', [
    ('frame', 'terminal_estimated_pose_diagnostic'),
    ('stale', 'estimated_pose_fresh_at_terminal'),
    ('terminal_time', 'estimated_pose_fresh_at_terminal'),
    ('gt_error', 'ground_truth_position_diagnostic'),
    ('localization_error', 'ground_truth_position_diagnostic'),
])
def test_evaluator_rejects_estimated_pose_provenance_tampering(
        mutation, expected_failure):
    """Goal and localization diagnostics remain bound to terminal evidence."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    if mutation == 'frame':
        evidence['final_estimated_pose_frame_id'] = 'odom'
    elif mutation == 'stale':
        evidence['final_estimated_pose_stamp_ns'] += 2_000_000_000
        evidence['estimated_pose_stamp_offset_at_terminal_s'] = 2.0
    elif mutation == 'terminal_time':
        evidence['final_estimated_pose_observed_ros_ns'] += 1
        evidence['estimated_pose_stamp_offset_at_terminal_s'] = -1e-9
    elif mutation == 'gt_error':
        evidence['terminal_ground_truth_goal_error_m'] = 1.0
    else:
        evidence['terminal_estimated_to_gt_error_m'] = 1.0

    assert expected_failure in evaluator.evaluate_run(evidence)['failures']


@pytest.mark.parametrize('mutation', [
    'duplicate', 'uuid', 'frame', 'child', 'pose', 'error', 'future', 'stale',
    'after_terminal', 'missing_tf_input', 'tf_input_tamper',
    'disconnected_tf', 'connected_unused_tf', 'duplicate_tf',
    'scaled_quaternion', 'missing_bracket', 'reversed_edges',
    'reversed_bracket'])
def test_evaluator_rejects_goal_tolerance_entry_tampering(mutation):
    """The first source-tolerance entry is exact, fresh, and goal-bound."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    entry = evidence['goal_position_tolerance_entry']
    if mutation == 'duplicate':
        evidence['events'].append(dict(entry))
    elif mutation == 'uuid':
        entry['goal_uuid'] = 'b' * 32
    elif mutation == 'frame':
        entry['frame_id'] = 'odom'
    elif mutation == 'child':
        entry['child_frame_id'] = 'base_footprint'
    elif mutation == 'pose':
        entry['pose_m'] = [999.0, 999.0]
    elif mutation == 'error':
        entry['position_error_m'] = 0.1
    elif mutation == 'future':
        entry['transform_stamp_ns'] += 1
        entry['transform_stamp_offset_s'] = 1e-9
    elif mutation == 'stale':
        entry['transform_stamp_ns'] -= 2_000_000_000
        entry['transform_stamp_offset_s'] = -2.0
    elif mutation == 'after_terminal':
        entry['steady_ns'] = 21
    elif mutation == 'missing_tf_input':
        entry['tf_inputs'] = []
    elif mutation == 'tf_input_tamper':
        entry['tf_inputs'][0]['translation'][0] = 5.0
    elif mutation == 'disconnected_tf':
        extra = dict(entry['tf_inputs'][0])
        extra.update(parent='foo', child='bar')
        entry['tf_inputs'].append(extra)
    elif mutation == 'connected_unused_tf':
        extra = dict(entry['tf_inputs'][0])
        extra.update(parent='map', child='unused')
        entry['tf_inputs'].append(extra)
    elif mutation == 'duplicate_tf':
        entry['tf_inputs'].append(dict(entry['tf_inputs'][0]))
    elif mutation == 'scaled_quaternion':
        entry['tf_inputs'][0]['rotation'][3] = 2.0
    elif mutation == 'missing_bracket':
        entry['tf_inputs'][0]['stamp_ns'] -= 1
    elif mutation == 'reversed_edges':
        entry['tf_inputs'] = [
            {'parent': 'mid', 'child': 'base_link',
             'stamp_ns': entry['transform_stamp_ns'],
             'translation': [6.0, 0.0, 0.0],
             'rotation': [0.0, 0.0, 0.0, 1.0], 'is_static': False},
            {'parent': 'map', 'child': 'mid', 'stamp_ns': 0,
             'translation': [0.0, 0.0, 0.0],
             'rotation': [0.0, 0.0, 0.0, 1.0], 'is_static': True},
        ]
    else:
        stamp = entry['transform_stamp_ns']
        entry['tf_inputs'] = [
            {'parent': 'map', 'child': 'base_link', 'stamp_ns': stamp + 1,
             'translation': [6.0, 0.0, 0.0],
             'rotation': [0.0, 0.0, 0.0, 1.0], 'is_static': False},
            {'parent': 'map', 'child': 'base_link', 'stamp_ns': stamp - 1,
             'translation': [6.0, 0.0, 0.0],
             'rotation': [0.0, 0.0, 0.0, 1.0], 'is_static': False},
        ]

    assert 'goal_position_tolerance_entry' in evaluator.evaluate_run(
        evidence)['failures']


@pytest.mark.parametrize('error_m, expected_pass', [
    (0.15, True), (0.150000001, False)])
def test_goal_tf_witness_uses_exact_tolerance_boundary(error_m, expected_pass):
    """Floating representation accepts .15 but does not widen the contract."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    evidence = _passing_evidence('clear_baseline')
    entry = evidence['goal_position_tolerance_entry']
    entry['pose_m'] = [6.0 - error_m, 0.0]
    entry['position_error_m'] = math.dist(entry['pose_m'], [6.0, 0.0])
    entry['tf_inputs'][0]['translation'][0] = entry['pose_m'][0]

    passed = 'goal_position_tolerance_entry' not in (
        evaluator.evaluate_run(evidence)['failures'])

    assert passed is expected_pass


def test_goal_tf_witness_latches_update_missed_by_timer_and_never_overwrites():
    """A short .1499 TF update is retained before later terminal drift."""
    import threading

    import tf2_py
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    scenario = _load('../jdamr_cube_navigation/'
                     'sim_collision_monitor_scenario.py')
    contract_module = _load('sim_collision_monitor_contract.py')

    class Clock:
        def now(self):
            return type('Now', (), {'nanoseconds': 500_000_000})()

    class Witness:
        _tf_record = staticmethod(
            scenario.CollisionMonitorScenario._tf_record)
        _observe_goal_tolerance_entry = (
            scenario.CollisionMonitorScenario._observe_goal_tolerance_entry)
        _canonical_goal_tf_inputs = (
            scenario.CollisionMonitorScenario._canonical_goal_tf_inputs)
        _goal_tf_update = scenario.CollisionMonitorScenario._goal_tf_update

        def __init__(self):
            self.goal_tf_lock = threading.Lock()
            self.goal_tf_buffer = tf2_py.BufferCore()
            self.goal_tf_records = {}
            self.goal_uuid = 'a' * 32
            self.goal_tolerance_entry = None
            self.action_terminal = 'unknown'
            self.contract = contract_module.derive_contract(20.0, 0.001)

        def get_clock(self):
            return Clock()

        def _event(self, name, **details):
            return {'name': name, 'steady_ns': 250_000_000,
                    'ros_ns': 500_000_000, **details}

    witness = Witness()
    for index, error_m in enumerate((0.151, 0.1499, 0.15052), start=1):
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.header.stamp.sec = 0
        transform.header.stamp.nanosec = index * 100_000_000
        transform.transform.translation.x = 6.0 - error_m
        transform.transform.rotation.w = 1.0
        witness._goal_tf_update(TFMessage(transforms=[transform]), False)

    assert witness.goal_tolerance_entry['position_error_m'] == pytest.approx(
        0.1499)
    assert witness.goal_tolerance_entry['transform_stamp_ns'] == 200_000_000
    witness.goal_tolerance_entry = None
    witness.action_terminal = 'succeeded'
    witness._goal_tf_update(TFMessage(transforms=[transform]), False)
    assert witness.goal_tolerance_entry is None


def test_goal_tf_witness_and_terminal_diagnostic_share_one_buffer():
    """The result snapshot uses exactly the BufferCore fed by raw TF updates."""
    import threading

    import tf2_py
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    scenario = _load('../jdamr_cube_navigation/'
                     'sim_collision_monitor_scenario.py')
    contract_module = _load('sim_collision_monitor_contract.py')

    class Clock:
        def now(self):
            return type('Now', (), {'nanoseconds': 500_000_000})()

    class ResultFuture:
        @staticmethod
        def result():
            return type('WrappedResult', (), {
                'status': GoalStatus.STATUS_SUCCEEDED})()

    class Witness:
        _tf_record = staticmethod(
            scenario.CollisionMonitorScenario._tf_record)
        _observe_goal_tolerance_entry = (
            scenario.CollisionMonitorScenario._observe_goal_tolerance_entry)
        _canonical_goal_tf_inputs = (
            scenario.CollisionMonitorScenario._canonical_goal_tf_inputs)
        _goal_tf_update = scenario.CollisionMonitorScenario._goal_tf_update
        _action_result = scenario.CollisionMonitorScenario._action_result

        def __init__(self):
            self.goal_tf_lock = threading.Lock()
            self.goal_tf_buffer = tf2_py.BufferCore()
            self.goal_tf_records = {}
            self.goal_uuid = 'a' * 32
            self.terminal_goal_uuid = None
            self.goal_tolerance_entry = None
            self.action_terminal = 'unknown'
            self.contract = contract_module.derive_contract(20.0, 0.001)
            self.events = []
            self.world_pose = (6.0, 0.0)
            self.terminal_ground_truth_pose = None
            self.final_estimated_pose = None
            self.final_estimated_pose_stamp_ns = None
            self.final_estimated_pose_observed_ros_ns = None
            self.harness_error = None
            self.phase = 'RUNNING'

        def get_clock(self):
            return Clock()

        def _event(self, name, **details):
            event = {'name': name, 'steady_ns': 250_000_000,
                     'ros_ns': 500_000_000, **details}
            self.events.append(event)
            return event

    witness = Witness()
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.header.stamp.nanosec = 400_000_000
    transform.transform.translation.x = 5.9
    transform.transform.rotation.w = 1.0
    witness._goal_tf_update(TFMessage(transforms=[transform]), False)
    witness._action_result(ResultFuture())

    assert witness.goal_tolerance_entry['pose_m'] == [5.9, 0.0]
    assert witness.final_estimated_pose == (5.9, 0.0)
    assert witness.final_estimated_pose_stamp_ns == 400_000_000
    assert witness.action_terminal == 'succeeded'


def test_result_first_keeps_optional_witness_absent_after_delayed_tf():
    """A TF callback processed after the result cannot invent a witness."""
    import threading

    import tf2_py
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    scenario = _load('../jdamr_cube_navigation/'
                     'sim_collision_monitor_scenario.py')
    contract_module = _load('sim_collision_monitor_contract.py')

    class Clock:
        def now(self):
            return type('Now', (), {'nanoseconds': 500_000_000})()

    class ResultFuture:
        @staticmethod
        def result():
            return type('WrappedResult', (), {
                'status': GoalStatus.STATUS_SUCCEEDED})()

    class Witness:
        _tf_record = staticmethod(
            scenario.CollisionMonitorScenario._tf_record)
        _observe_goal_tolerance_entry = (
            scenario.CollisionMonitorScenario._observe_goal_tolerance_entry)
        _canonical_goal_tf_inputs = (
            scenario.CollisionMonitorScenario._canonical_goal_tf_inputs)
        _goal_tf_update = scenario.CollisionMonitorScenario._goal_tf_update
        _action_result = scenario.CollisionMonitorScenario._action_result

        def __init__(self):
            self.goal_tf_lock = threading.Lock()
            self.goal_tf_buffer = tf2_py.BufferCore()
            self.goal_tf_records = {}
            self.goal_uuid = 'a' * 32
            self.terminal_goal_uuid = None
            self.goal_tolerance_entry = None
            self.action_terminal = 'unknown'
            self.contract = contract_module.derive_contract(20.0, 0.001)
            self.events = []
            self.world_pose = (6.0, 0.0)
            self.terminal_ground_truth_pose = None
            self.final_estimated_pose = None
            self.final_estimated_pose_stamp_ns = None
            self.final_estimated_pose_observed_ros_ns = None
            self.harness_error = None
            self.phase = 'RUNNING'

        def get_clock(self):
            return Clock()

        def _event(self, name, **details):
            event = {'name': name, 'steady_ns': 250_000_000,
                     'ros_ns': 500_000_000, **details}
            self.events.append(event)
            return event

    witness = Witness()
    outside = TransformStamped()
    outside.header.frame_id = 'map'
    outside.child_frame_id = 'base_link'
    outside.header.stamp.nanosec = 300_000_000
    outside.transform.translation.x = 5.8
    outside.transform.rotation.w = 1.0
    witness._goal_tf_update(TFMessage(transforms=[outside]), False)
    witness._action_result(ResultFuture())
    inside = TransformStamped()
    inside.header.frame_id = 'map'
    inside.child_frame_id = 'base_link'
    inside.header.stamp.nanosec = 400_000_000
    inside.transform.translation.x = 5.9
    inside.transform.rotation.w = 1.0
    witness._goal_tf_update(TFMessage(transforms=[inside]), False)

    assert witness.goal_tolerance_entry is None
    assert witness.final_estimated_pose == (5.8, 0.0)
    assert witness.action_terminal == 'succeeded'


def _write_representative_fixture(
        root, scenario, *, extra_uuid=False, wrong_polygon=False,
        post_clear_stop=False, canceling=False,
        unexpected_monitor_action=False):
    """Write one tiny real MCAP satisfying the representative schema."""
    import gc

    import rosbag2_py
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from geometry_msgs.msg import TransformStamped
    from nav2_msgs.msg import CollisionMonitorState
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    contract_module = _load('sim_collision_monitor_contract.py')
    evidence = _passing_evidence(scenario)
    entry_stamp_ns = evidence[
        'goal_position_tolerance_entry']['transform_stamp_ns']
    run_dir = root / evidence['run_id']
    bag_dir = run_dir / 'bag'
    run_dir.mkdir(parents=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''))
    topics = evidence['contract']['retention']['representative_topics']
    typed = {
        '/tf': 'tf2_msgs/msg/TFMessage',
        '/tf_static': 'tf2_msgs/msg/TFMessage',
        '/collision_monitor_state': 'nav2_msgs/msg/CollisionMonitorState',
        '/navigate_to_pose/_action/status': 'action_msgs/msg/GoalStatusArray',
    }
    for index, topic in enumerate(topics):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=index, name=topic, type=typed.get(topic, 'std_msgs/msg/String'),
            serialization_format='cdr'))
    static = TransformStamped()
    static.header.frame_id = 'map'
    static.child_frame_id = 'odom'
    static.transform.rotation.w = 1.0
    dynamic = TransformStamped()
    dynamic.header.frame_id = 'odom'
    dynamic.child_frame_id = 'base_link'
    dynamic.header.stamp.sec = entry_stamp_ns // 1_000_000_000
    dynamic.header.stamp.nanosec = entry_stamp_ns % 1_000_000_000
    dynamic.transform.translation.x = 6.0
    dynamic.transform.rotation.w = 1.0
    evidence['goal_position_tolerance_entry']['tf_inputs'] = [
        {'parent': 'map', 'child': 'odom', 'stamp_ns': 0,
         'translation': [0.0, 0.0, 0.0],
         'rotation': [0.0, 0.0, 0.0, 1.0], 'is_static': True},
        {'parent': 'odom', 'child': 'base_link',
         'stamp_ns': entry_stamp_ns,
         'translation': [6.0, 0.0, 0.0],
         'rotation': [0.0, 0.0, 0.0, 1.0], 'is_static': False},
    ]
    writer.write('/tf_static', serialize_message(
        TFMessage(transforms=[static])), 100)
    writer.write('/tf', serialize_message(
        TFMessage(transforms=[dynamic])), 100)
    goal_uuid = bytes.fromhex(evidence['goal_uuid'])

    def status_message(status, uuid=goal_uuid):
        item = GoalStatus()
        item.goal_info.goal_id.uuid = list(uuid)
        item.status = status
        return GoalStatusArray(status_list=[item])

    writer.write('/navigate_to_pose/_action/status', serialize_message(
        status_message(GoalStatus.STATUS_EXECUTING)), 1000)
    if extra_uuid:
        writer.write('/navigate_to_pose/_action/status', serialize_message(
            status_message(GoalStatus.STATUS_EXECUTING, b'b' * 16)), 1500)
    if canceling:
        writer.write('/navigate_to_pose/_action/status', serialize_message(
            status_message(GoalStatus.STATUS_CANCELING)), 1500)
    writer.write('/navigate_to_pose/_action/status', serialize_message(
        status_message(GoalStatus.STATUS_SUCCEEDED)), 2000)
    states = []
    if unexpected_monitor_action:
        states.append((CollisionMonitorState.SLOWDOWN, 'unexpected', 1000))
    if scenario != 'clear_baseline':
        polygon = ('invalid source'
                   if scenario == 'scan_timeout_stop_resume'
                   else 'StopZone')
        states.extend([(1, 'wrong' if wrong_polygon else polygon, 1100),
                       (0, '', 1500)])
        if post_clear_stop:
            states.append((1, polygon, 1600))
    for action, polygon, timestamp in states:
        writer.write('/collision_monitor_state', serialize_message(
            CollisionMonitorState(
                action_type=action, polygon_name=polygon)), timestamp)
    for index, topic in enumerate(topics):
        if topic not in typed and not topic.endswith('/contact'):
            writer.write(topic, serialize_message(String(data='g004')),
                         3000 + index)
    del writer
    gc.collect()
    metadata_path = bag_dir / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text())[
        'rosbag2_bagfile_information']
    bag_path = bag_dir / 'bag_0.mcap'
    record = {
        'run_id': evidence['run_id'], 'scenario': scenario, 'seed': 11,
        'path': str(bag_path.resolve()),
        'size_bytes': bag_path.stat().st_size,
        'sha256': contract_module.sha256_file(bag_path),
        'metadata_size_bytes': metadata_path.stat().st_size,
        'metadata_sha256': contract_module.sha256_file(metadata_path),
        'topic_inventory': metadata['topics_with_message_count'],
        'requested_topics': topics,
    }
    evidence['representative_bag'] = record
    evidence['representative_bag_recorded'] = True
    evidence['teardown']['processes'].append({
        'name': 'recorder', 'pid': 20_000,
        'returncode_at_measurement': None,
        'returncode_before_stop': None, 'runner_initiated': True,
        'requested_signal': 2,
        'stop_requested_steady_ns': 2_100_000_000,
        'stop_completed_steady_ns': 3_100_000_000,
        'returncode_after_stop': -2,
        'logged_process_exits_before_stop': [],
        'logged_process_exits_after_stop': [],
        'log_path': '', 'log_size_bytes': 0, 'log_sha256': None,
        'log_pre_stop_offset_bytes': 0,
    })
    evidence['process_identity'].append({'name': 'recorder', 'pid': 20_000})
    (run_dir / 'evidence.json').write_text(json.dumps(evidence))
    return record


@pytest.mark.parametrize('mutation', [
    'extra_uuid', 'wrong_polygon', 'post_clear_stop', 'canceling',
    'unexpected_monitor_action'])
def test_representative_bag_rejects_hostile_action_and_state_sequences(
        tmp_path, mutation):
    """Raw MCAP UUID and monitor sequences fail closed under relabeling."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _passing_evidence('clear_baseline')['contract']
    records = []
    for scenario in contract['retention']['representative_scenarios']:
        records.append(_write_representative_fixture(
            tmp_path, scenario,
            extra_uuid=(mutation == 'extra_uuid'
                        and scenario == 'clear_baseline'),
            wrong_polygon=(mutation == 'wrong_polygon'
                           and scenario == 'sudden_obstacle_stop_resume'),
            post_clear_stop=(mutation == 'post_clear_stop'
                             and scenario == 'scan_timeout_stop_resume'),
            canceling=(mutation == 'canceling'
                       and scenario == 'clear_baseline'),
            unexpected_monitor_action=(
                mutation == 'unexpected_monitor_action'
                and scenario == 'sudden_obstacle_stop_resume')))

    checks = evaluator._representative_retention_checks(
        {'representative_bags': records}, True, contract, tmp_path)

    assert not all(checks.values())


def test_representative_bag_accepts_exact_raw_sequences(tmp_path):
    """Three predeclared seed-11 bags pass the independent raw reader."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _passing_evidence('clear_baseline')['contract']
    records = [
        _write_representative_fixture(tmp_path, scenario)
        for scenario in contract['retention']['representative_scenarios']]

    checks = evaluator._representative_retention_checks(
        {'representative_bags': records}, True, contract, tmp_path)

    assert all(checks.values()), checks


@pytest.mark.parametrize('scenario', [
    'clear_baseline', 'sudden_obstacle_stop_resume',
    'scan_timeout_stop_resume'])
def test_representative_bag_accepts_absent_optional_tf_witness(
        tmp_path, scenario):
    """Representative raw action proof remains valid without a TF witness."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _passing_evidence('clear_baseline')['contract']
    records = [
        _write_representative_fixture(tmp_path, item)
        for item in contract['retention']['representative_scenarios']]
    run_id = f'{scenario}__seed_11'
    evidence_path = tmp_path / run_id / 'evidence.json'
    evidence = json.loads(evidence_path.read_text())
    evidence['goal_position_tolerance_entry'] = None
    evidence['events'] = [
        event for event in evidence['events']
        if event['name'] != 'goal_position_tolerance_entered']
    evidence_path.write_text(json.dumps(evidence))

    checks = evaluator._representative_retention_checks(
        {'representative_bags': records}, True, contract, tmp_path)

    assert all(checks.values()), checks


@pytest.mark.parametrize('missing_side', ['field', 'event'])
def test_representative_bag_rejects_one_sided_optional_tf_witness(
        tmp_path, missing_side):
    """Optional TF evidence must be wholly present or wholly absent."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _passing_evidence('clear_baseline')['contract']
    records = [
        _write_representative_fixture(tmp_path, scenario)
        for scenario in contract['retention']['representative_scenarios']]
    evidence_path = tmp_path / records[0]['run_id'] / 'evidence.json'
    evidence = json.loads(evidence_path.read_text())
    if missing_side == 'field':
        evidence['goal_position_tolerance_entry'] = None
    else:
        evidence['events'] = [
            event for event in evidence['events']
            if event['name'] != 'goal_position_tolerance_entered']
    evidence_path.write_text(json.dumps(evidence))

    checks = evaluator._representative_retention_checks(
        {'representative_bags': records}, True, contract, tmp_path)

    assert checks['representative_0_identity'] is False


def test_representative_bag_rejects_fabricated_equivalent_tf_path(tmp_path):
    """Equivalent self-supplied intermediate frames cannot relabel raw TF."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _passing_evidence('clear_baseline')['contract']
    records = [
        _write_representative_fixture(tmp_path, scenario)
        for scenario in contract['retention']['representative_scenarios']]
    evidence_path = tmp_path / records[0]['run_id'] / 'evidence.json'
    evidence = json.loads(evidence_path.read_text())
    entry = evidence['goal_position_tolerance_entry']
    entry['tf_inputs'] = [
        {'parent': 'map', 'child': 'mid', 'stamp_ns': 0,
         'translation': [0.0, 0.0, 0.0],
         'rotation': [0.0, 0.0, 0.0, 1.0], 'is_static': True},
        {'parent': 'mid', 'child': 'base_link',
         'stamp_ns': entry['transform_stamp_ns'],
         'translation': [6.0, 0.0, 0.0],
         'rotation': [0.0, 0.0, 0.0, 1.0], 'is_static': False},
    ]
    evidence_path.write_text(json.dumps(evidence))

    checks = evaluator._representative_retention_checks(
        {'representative_bags': records}, True, contract, tmp_path)

    assert checks['representative_0_identity'] is False


@pytest.mark.parametrize('mutation', [
    'wrong_filename', 'duplicate_mcap', 'nested_mcap', 'metadata_mismatch',
    'symlink_escape', 'directory_symlink_escape'])
def test_representative_bag_rejects_noncanonical_tree(tmp_path, mutation):
    """Representative bags use one lexical, regular, root-contained tree."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract = _passing_evidence('clear_baseline')['contract']
    records = [
        _write_representative_fixture(tmp_path, scenario)
        for scenario in contract['retention']['representative_scenarios']]
    path = Path(records[0]['path'])
    if mutation == 'wrong_filename':
        path.rename(path.parent / 'renamed_0.mcap')
    elif mutation == 'duplicate_mcap':
        (path.parent / 'duplicate.mcap').write_bytes(path.read_bytes())
    elif mutation == 'nested_mcap':
        nested = path.parent / 'nested'
        nested.mkdir()
        (nested / 'duplicate.mcap').write_bytes(path.read_bytes())
    elif mutation == 'metadata_mismatch':
        metadata_path = path.parent / 'metadata.yaml'
        metadata = yaml.safe_load(metadata_path.read_text())
        metadata['rosbag2_bagfile_information']['relative_file_paths'] = [
            'renamed_0.mcap']
        metadata_path.write_text(yaml.safe_dump(metadata))
    elif mutation == 'symlink_escape':
        outside = tmp_path / 'outside.mcap'
        path.rename(outside)
        path.symlink_to(outside)
    else:
        bag_dir = path.parent
        outside = tmp_path / 'outside_bag'
        bag_dir.rename(outside)
        bag_dir.symlink_to(outside, target_is_directory=True)

    checks = evaluator._representative_retention_checks(
        {'representative_bags': records}, True, contract, tmp_path)

    assert checks['representative_0_identity'] is False


def test_promotion_runtime_rehashes_retained_sources_and_exact_domains(
        tmp_path):
    """Promotion mode validates assets, source copies, RMW bytes, and domains."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract_module = _load('sim_collision_monitor_contract.py')
    canonical = _passing_evidence('clear_baseline')['contract']
    contract_path = tmp_path / 'contract.json'
    _write_preparation_identity(
        tmp_path, canonical, contract_path, contract_module)
    runtime_path = tmp_path / 'runtime_manifest.json'
    runtime = json.loads(runtime_path.read_text())
    planned = runtime['planned_runs'][::5]
    for index, record in enumerate(planned):
        record['domain_id'] = 180 + index
        run_dir = tmp_path / record['run_id']
        run_dir.mkdir()
        (run_dir / 'evidence.json').write_text(json.dumps({
            'domain_id': 180 + index}))
    runtime.update({
        'execution_mode': 'representative_promotion',
        'domain_id_base': 180, 'domain_ids': [180, 181, 182],
        'planned_runs': planned})
    runtime['evaluation_rmw']['domain_id'] = 180
    runtime_path.write_text(json.dumps(runtime))
    records = [{'run_id': record['run_id']} for record in planned]
    source = {
        'preparation_manifest_sha256': contract_module.sha256_file(
            tmp_path / 'preparation_manifest.json'),
        'evaluation_rmw_identity': {
            key: runtime['evaluation_rmw'][key] for key in (
                'requested_identifier', 'actual_identifier', 'version', 'source',
                'library_size_bytes', 'library_sha256', 'environment_fields',
                'environment_sha256')}}

    assert evaluator.promotion_runtime_valid(
        tmp_path, records, source) is True

    retained = Path(runtime['harness_sources']['runner']['retained_path'])
    retained.write_bytes(retained.read_bytes() + b'\n')
    assert evaluator.promotion_runtime_valid(
        tmp_path, records, source) is False


def _write_source_matrix_fixture(root):
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract_module = _load('sim_collision_monitor_contract.py')
    runner = _load('run_sim_collision_monitor_eval.py')
    root.mkdir()
    canonical = _passing_evidence('clear_baseline')['contract']
    contract_path = root / 'contract.json'
    _write_preparation_identity(root, canonical, contract_path, contract_module)
    canonical_sha = contract_module.sha256_file(contract_path)
    paths = []
    for index, item in enumerate(contract_module.scenario_matrix()):
        run_dir = root / item['run_id']
        run_dir.mkdir()
        evidence = _passing_evidence(item['scenario'])
        evidence.update(item)
        evidence['contract'] = canonical
        evidence['domain_id'] = 160 + index
        evidence['canonical_contract_sha256'] = canonical_sha
        _attach_resource_evidence(evidence, run_dir)
        if item['scenario'] == 'sudden_obstacle_stop_resume':
            _attach_scan_gate_evidence(evidence, run_dir)
        evidence['retained_run_bytes'] = sum(
            path.stat().st_size for path in run_dir.rglob('*')
            if path.is_file())
        evidence_path = run_dir / 'evidence.json'
        evidence_path.write_text(json.dumps(evidence))
        evaluation = evaluator.evaluate_run(
            evidence, evidence_path, canonical, canonical_sha)
        assert evaluation['status'] == 'PASS', evaluation['failures']
        (run_dir / 'evaluation.json').write_text(json.dumps(evaluation))
        paths.append(evidence_path)
    result = evaluator.aggregate(paths)
    assert result['status'] == 'PASS'
    aggregate_path = root / 'aggregate.json'
    aggregate_path.write_text(json.dumps(result))
    finalizer = _load('finalize_sim_collision_monitor_eval.py')
    (root / 'final_summary.json').write_text(json.dumps(
        finalizer.finalize(root)))
    return runner.validate_source_matrix(root)


def _write_promotion_fixture(root, source):
    evaluator = _load('evaluate_sim_collision_monitor.py')
    contract_module = _load('sim_collision_monitor_contract.py')
    canonical = _passing_evidence('clear_baseline')['contract']
    contract_path = root / 'contract.json'
    _write_preparation_identity(root, canonical, contract_path, contract_module)
    source_root = Path(source['path'])
    contract_path.write_bytes((source_root / 'contract.json').read_bytes())
    (root / 'preparation_manifest.json').write_bytes(
        (source_root / 'preparation_manifest.json').read_bytes())
    canonical = json.loads(contract_path.read_text())
    runtime_path = root / 'runtime_manifest.json'
    runtime = json.loads(runtime_path.read_text())
    planned = runtime['planned_runs'][::5]
    for index, record in enumerate(planned):
        record['domain_id'] = 180 + index
    runtime.update({
        'execution_mode': 'representative_promotion',
        'domain_id_base': 180, 'domain_ids': [180, 181, 182],
        'planned_runs': planned})
    runtime['evaluation_rmw']['domain_id'] = 180
    runtime['canonical_contract_sha256'] = contract_module.sha256_file(
        contract_path)
    runtime['preparation_manifest_sha256'] = contract_module.sha256_file(
        root / 'preparation_manifest.json')
    runtime_path.write_text(json.dumps(runtime))
    records = []
    canonical_sha = contract_module.sha256_file(contract_path)
    for index, scenario in enumerate(
            canonical['retention']['representative_scenarios']):
        record = _write_representative_fixture(root, scenario)
        evidence_path = root / record['run_id'] / 'evidence.json'
        evidence = json.loads(evidence_path.read_text())
        evidence['contract'] = canonical
        evidence['domain_id'] = 180 + index
        evidence['canonical_contract_sha256'] = canonical_sha
        _attach_resource_evidence(evidence, evidence_path.parent)
        if scenario == 'sudden_obstacle_stop_resume':
            _attach_scan_gate_evidence(evidence, evidence_path.parent)
        evidence['retained_run_bytes'] = sum(
            path.stat().st_size for path in evidence_path.parent.rglob('*')
            if path.is_file() and path.name not in {
                'evidence.json', 'evaluation.json'})
        evidence_path.write_text(json.dumps(evidence))
        result = evaluator.evaluate_run(
            evidence, evidence_path, canonical, canonical_sha,
            representative_run=True)
        assert result['status'] == 'PASS', result['failures']
        (evidence_path.parent / 'evaluation.json').write_text(
            json.dumps(result))
        records.append(record)
    checks = evaluator._representative_retention_checks(
        {'representative_bags': records}, True, canonical, root)
    checks['promotion_runtime_identity'] = evaluator.promotion_runtime_valid(
        root, records, source)
    retained_files = [{
        'relative_path': str(path.relative_to(root)),
        'size_bytes': path.stat().st_size,
        'sha256': contract_module.sha256_file(path),
    } for path in sorted(root.rglob('*')) if path.is_file()
        and path.name != 'promotion_manifest.json']
    checks.update({
        'promotion_total_retention': True,
        'promotion_file_exact_set': True,
        'promotion_directory_exact_set': True})
    manifest = {
        'schema_version': 1, 'status': 'PASS', 'source_matrix': source,
        'representative_bags': records, 'checks': checks,
        'retained_bytes_excluding_manifest': sum(
            item['size_bytes'] for item in retained_files),
        'retained_file_manifest': retained_files,
        'retained_directories': sorted(
            str(path.relative_to(root))
            for path in root.rglob('*') if path.is_dir()),
    }
    (root / 'promotion_manifest.json').write_text(json.dumps(manifest))


@pytest.mark.parametrize('mutation', [
    'delete_bag', 'rmw', 'domain', 'domain_bool', 'extra_run', 'status',
    'source_final_metric_relinked', 'source_aggregate_relinked'])
def test_verify_promotion_fails_closed_on_retained_tampering(
        tmp_path, mutation):
    """Durable promotion verification rejects missing or relabeled bytes."""
    evaluator = _load('evaluate_sim_collision_monitor.py')
    source = _write_source_matrix_fixture(tmp_path / 'source')
    promotion = tmp_path / 'promotion'
    promotion.mkdir()
    _write_promotion_fixture(promotion, source)
    assert evaluator.verify_promotion(promotion)['status'] == 'PASS'
    if mutation.startswith('source_'):
        source_root = Path(source['path'])
        aggregate_path = source_root / 'aggregate.json'
        final_path = source_root / 'final_summary.json'
        if mutation == 'source_final_metric_relinked':
            final = json.loads(final_path.read_text())
            final['scenarios']['clear_baseline'][
                'minimum_observed_scan_range_m']['max'] = 999999.0
            final_path.write_text(json.dumps(final))
        else:
            aggregate = json.loads(aggregate_path.read_text())
            aggregate['fabricated_claim'] = 999999.0
            aggregate_path.write_text(json.dumps(aggregate))
            final = json.loads(final_path.read_text())
            final['aggregate_input_sha256'] = _load(
                'sim_collision_monitor_contract.py').sha256_file(
                    aggregate_path)
            final_path.write_text(json.dumps(final))
        manifest_path = promotion / 'promotion_manifest.json'
        manifest = json.loads(manifest_path.read_text())
        for name, path in (
                ('aggregate', aggregate_path), ('final_summary', final_path)):
            manifest['source_matrix'][f'{name}_size_bytes'] = (
                path.stat().st_size)
            manifest['source_matrix'][f'{name}_sha256'] = _load(
                'sim_collision_monitor_contract.py').sha256_file(path)
        manifest_path.write_text(json.dumps(manifest))
    elif mutation == 'delete_bag':
        next(promotion.glob('*/bag/*.mcap')).unlink()
    elif mutation == 'extra_run':
        extra = promotion / 'unexpected_run'
        extra.mkdir()
        (extra / 'evidence.json').write_text('{}')
    elif mutation == 'status':
        manifest_path = promotion / 'promotion_manifest.json'
        manifest = json.loads(manifest_path.read_text())
        manifest['status'] = 'FAIL'
        manifest_path.write_text(json.dumps(manifest))
    else:
        runtime_path = promotion / 'runtime_manifest.json'
        runtime = json.loads(runtime_path.read_text())
        if mutation == 'rmw':
            runtime['evaluation_rmw']['library_sha256'] = '0' * 64
        elif mutation == 'domain':
            runtime['planned_runs'][0]['domain_id'] += 1
        else:
            runtime['domain_id_base'] = True
        runtime_path.write_text(json.dumps(runtime))

    assert evaluator.verify_promotion(promotion)['status'] == 'FAIL'


@pytest.mark.parametrize('mutation', ['final_metric', 'aggregate_claim'])
def test_source_matrix_validation_recomputes_all_summaries(tmp_path, mutation):
    """Relinked summary hashes cannot launder fabricated derived claims."""
    runner = _load('run_sim_collision_monitor_eval.py')
    contract_module = _load('sim_collision_monitor_contract.py')
    root = tmp_path / 'source'
    _write_source_matrix_fixture(root)
    aggregate_path = root / 'aggregate.json'
    final_path = root / 'final_summary.json'
    if mutation == 'final_metric':
        final = json.loads(final_path.read_text())
        final['scenarios']['clear_baseline'][
            'minimum_observed_scan_range_m']['max'] = 999999.0
        final_path.write_text(json.dumps(final))
    else:
        aggregate = json.loads(aggregate_path.read_text())
        aggregate['fabricated_claim'] = 999999.0
        aggregate_path.write_text(json.dumps(aggregate))
        final = json.loads(final_path.read_text())
        final['aggregate_input_sha256'] = contract_module.sha256_file(
            aggregate_path)
        final_path.write_text(json.dumps(final))

    with pytest.raises(ValueError, match='summary binding mismatch'):
        runner.validate_source_matrix(root)
