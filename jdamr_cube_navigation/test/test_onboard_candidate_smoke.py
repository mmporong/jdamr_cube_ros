"""Tests for the bounded real-onboard candidate simulation smoke."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys  # noqa: I100
from types import SimpleNamespace

from jdamr_cube_navigation.sim_collision_monitor_scenario import (
    robot_entity_contact_pair,
)

import pytest

import yaml


EVALUATION = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(EVALUATION))
SPEC = importlib.util.spec_from_file_location(
    'run_onboard_candidate_smoke',
    EVALUATION / 'run_onboard_candidate_smoke.py')
SMOKE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SMOKE)
REEVALUATE_SPEC = importlib.util.spec_from_file_location(
    'reevaluate_keepout_evidence',
    EVALUATION / 'reevaluate_keepout_evidence.py')
REEVALUATE = importlib.util.module_from_spec(REEVALUATE_SPEC)
REEVALUATE_SPEC.loader.exec_module(REEVALUATE)


def test_candidate_params_have_only_allowed_measurement_deltas(tmp_path):
    """Keep the candidate copy equal to production outside measurement."""
    prepared = SMOKE.prepare_candidate_assets(tmp_path)
    production = yaml.safe_load(
        SMOKE.PRODUCTION_PARAMS.read_text(encoding='utf-8'))
    candidate = yaml.safe_load(prepared['params'].read_text(encoding='utf-8'))

    assert SMOKE._leaf_differences(production, candidate) == (
        SMOKE.ALLOWED_PARAM_DELTAS)
    assert 'stop_params' not in prepared
    assert prepared['mask_report']['keepout_cells'] > 0
    assert prepared['mask_report']['zones'] == [
        'sim_route_away_northeast_corner']
    direct = json.loads(
        prepared['stop_contract'].read_text(encoding='utf-8'))
    assert direct['integration_kind'] == 'onboard_candidate_direct_scan'
    assert direct['preloaded_obstacle']['role'] == 'front_observation_probe'
    assert direct['scan_gate'] == {
        'used': False, 'monitor_input_topic': '/scan'}
    assert direct['sudden_obstacle']['activation_surface_x_m'] == (
        pytest.approx(0.45))
    assert direct['stop_zone']['front_m'] == pytest.approx(0.40)
    assert direct['slowdown_zone']['front_m'] == pytest.approx(0.50)
    assert direct['slowdown_zone']['slowdown_ratio'] == pytest.approx(0.60)
    assert direct['non_contact_margin']['remaining_margin_m'] > 0.0
    envelope = direct['travel_pose_envelope']
    assert envelope['bounds_m']['front_m'] == pytest.approx(
        0.30255615917893763)
    assert envelope['witnesses']['front_m']['link'] == 'arm_moving_jaw_link'
    assert envelope['production_stop_zone_deficit_m'] > 0.0
    assert direct['protected_envelope'] == {
        'front_m': pytest.approx(0.30255615917893763),
        'rear_m': -0.23,
        'half_width_m': 0.2,
        'composition': 'union_AABB_of_base_footprint_and_stowed_arm',
    }
    protection = SMOKE.load_mobile_manipulator_protection(
        SMOKE.MOBILE_MANIPULATOR_PROTECTION)
    assert protection['collision_monitor_overrides'] == {
        'StopZone.points': direct['stop_zone']['points'],
        'SlowdownZone.points': direct['slowdown_zone']['points'],
    }
    assert direct['candidate_params']['path'] == str(
        prepared['params'].resolve())
    assert direct['runtime_protection_config']['sha256'] == SMOKE._sha256(
        SMOKE.MOBILE_MANIPULATOR_PROTECTION)


def test_keepout_demo_builds_connected_corridor_mask(tmp_path):
    prepared = SMOKE.prepare_candidate_assets(tmp_path, keepout_demo=True)

    assert prepared['keepout_demo'] == {
        'zone_id': 'sim_portfolio_corridor_keepout',
        'polygon_m': [
            [-4.5, -1.1], [-3.5, -1.1],
            [-3.5, 0.1], [-4.5, 0.1]],
        'safety_margin_m': 0.35,
        'expanded_bounds_m': {
            'min_x_m': pytest.approx(-4.85),
            'max_x_m': pytest.approx(-3.15),
            'min_y_m': pytest.approx(-1.45),
            'max_y_m': pytest.approx(0.45),
        },
    }
    assert prepared['mask_report']['zones'] == [
        'sim_portfolio_corridor_keepout']
    assert prepared['mask_report']['connectivity_checks'][0][
        'connected'] is True
    candidate = yaml.safe_load(
        prepared['params'].read_text(encoding='utf-8'))
    for name in ('local_costmap', 'global_costmap'):
        costmap = candidate[name][name]['ros__parameters']
        assert costmap['filters'] == [
            'keepout_filter', 'keepout_inflation']
        assert costmap['keepout_inflation'] == {
            'plugin': 'nav2_costmap_2d::InflationLayer',
            'inflation_radius': 0.45,
            'cost_scaling_factor': 3.0,
        }
    assert SMOKE._leaf_differences(
        yaml.safe_load(SMOKE.PRODUCTION_PARAMS.read_text(encoding='utf-8')),
        candidate) == (
            SMOKE.ALLOWED_PARAM_DELTAS | SMOKE.KEEPOUT_DEMO_PARAM_DELTAS)


def test_main_forwards_keepout_demo_to_asset_preparation(
        monkeypatch, tmp_path):
    captured = {}

    def fake_prepare(output_root, keepout_demo=False):
        captured['output_root'] = output_root
        captured['keepout_demo'] = keepout_demo
        return {}

    monkeypatch.setattr(SMOKE, 'prepare_candidate_assets', fake_prepare)
    monkeypatch.setattr(
        SMOKE, 'run_case', lambda *_args, **_kwargs: {'status': 'PASS'})
    monkeypatch.setattr(SMOKE, '_cap_output', lambda _root: 1)
    monkeypatch.setattr(sys, 'argv', [
        'run_onboard_candidate_smoke.py', '--output-root', str(tmp_path),
        '--domain-id', '186', '--case', 'detour', '--keepout-demo'])

    assert SMOKE.main() == 0
    assert captured == {'output_root': tmp_path, 'keepout_demo': True}


def test_json_string_parameter_compares_the_runtime_geometry_semantically():
    """Ignore harmless whitespace while retaining exact numeric geometry."""
    output = 'String value is: [[0.4, 0.25], [0.4, -0.25]]'

    assert SMOKE._json_string_parameter(output) == [
        [0.4, 0.25], [0.4, -0.25]]

    with pytest.raises(ValueError, match='expected ROS string parameter'):
        SMOKE._json_string_parameter('Double value is: 0.4')


def test_parameter_retries_only_transient_discovery(monkeypatch):
    """An active node's first discovery timeout must not abort the capture."""
    calls = []
    responses = iter([
        subprocess.CompletedProcess([], 1, '', 'timed out waiting for parameter services'),
        subprocess.CompletedProcess([], 0, 'Boolean value is: True', ''),
    ])

    def run(command, environment, **kwargs):
        calls.append(command)
        return next(responses)

    monkeypatch.setattr(SMOKE, '_run', run)
    assert SMOKE._parameter('/collision_monitor', 'use_sim_time', {}) == (
        'Boolean value is: True')
    assert len(calls) == 2
    assert all('--no-daemon' in command for command in calls)


@pytest.mark.parametrize('errors,expected_calls', [
    ([], 1), (['Parameter not set'], 1),
    (['Node not found', 'Node not found'], 2),
    (['timed out waiting for parameter services', 'second failure'], 2),
])
def test_parameter_retry_boundaries(monkeypatch, errors, expected_calls):
    """Retries neither hide permanent failures nor repeat indefinitely."""
    calls = []

    def run(command, environment, **kwargs):
        calls.append(command)
        index = len(calls) - 1
        error = errors[index] if index < len(errors) else ''
        return subprocess.CompletedProcess([], int(bool(error)), 'True', error)

    monkeypatch.setattr(SMOKE, '_run', run)
    if errors:
        with pytest.raises(RuntimeError, match=errors[-1]):
            SMOKE._parameter('/collision_monitor', 'use_sim_time', {})
    else:
        assert SMOKE._parameter('/collision_monitor', 'use_sim_time', {}) == 'True'
    assert len(calls) == expected_calls


@pytest.mark.parametrize('domain_id', [12, 185, 188])
def test_main_rejects_physical_or_unassigned_domains(
        monkeypatch, tmp_path, domain_id):
    """Never allow the physical domain or domains outside the allocation."""
    monkeypatch.setattr(sys, 'argv', [
        'run_onboard_candidate_smoke.py', '--output-root', str(tmp_path),
        '--domain-id', str(domain_id), '--startup-only'])
    with pytest.raises(SystemExit) as error:
        SMOKE.main()
    assert error.value.code == 2


def test_main_rejects_two_cases_that_would_reach_domain_188(
        monkeypatch, tmp_path):
    """Validate every allocated case domain before preparing any output."""
    monkeypatch.setattr(sys, 'argv', [
        'run_onboard_candidate_smoke.py', '--output-root', str(tmp_path),
        '--domain-id', '187', '--startup-only'])
    with pytest.raises(SystemExit) as error:
        SMOKE.main()
    assert error.value.code == 2


def test_main_rejects_nonempty_output_root(monkeypatch, tmp_path):
    """Preserve all evidence owned by a previous smoke attempt."""
    (tmp_path / 'owned-by-earlier-run').write_text('preserve')
    monkeypatch.setattr(sys, 'argv', [
        'run_onboard_candidate_smoke.py', '--output-root', str(tmp_path),
        '--domain-id', '186', '--case', 'detour', '--startup-only'])
    with pytest.raises(SystemExit) as error:
        SMOKE.main()
    assert error.value.code == 2


def test_main_rejects_gui_without_a_display(monkeypatch, tmp_path):
    """Do not label a headless run as a simulator GUI recording."""
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.setattr(sys, 'argv', [
        'run_onboard_candidate_smoke.py', '--output-root', str(tmp_path),
        '--domain-id', '186', '--case', 'sudden_stop_resume', '--gui'])

    with pytest.raises(SystemExit) as error:
        SMOKE.main()

    assert error.value.code == 2


def test_status_summary_extracts_uuid_and_terminal(monkeypatch, tmp_path):
    """Retain the raw action goal identity and terminal status."""
    path = tmp_path / 'bag.mcap'
    path.write_bytes(b'evidence identity')

    class Value:
        pass

    def status(code):
        value = Value()
        value.status = code
        value.goal_info = Value()
        value.goal_info.goal_id = Value()
        value.goal_info.goal_id.uuid = list(range(16))
        return value

    message = Value()
    message.ros_msg = Value()
    message.ros_msg.status_list = [status(2), status(4)]
    monkeypatch.setattr(
        SMOKE, 'read_navigation_messages', lambda *_args, **_kwargs: [message])

    summary = SMOKE._status_summary(path)

    assert summary['goal_uuids'] == [
        '000102030405060708090a0b0c0d0e0f']
    assert summary['final_status'] == 4


def test_action_success_without_ground_truth_motion_is_not_a_detour():
    """Reject Nav2 success caused by a wrong localization transform."""
    document = {
        'action_terminal': 'succeeded', 'contact_count': 0,
        'final_cmd_vel_zero': True,
        'final_robot_pose_xy_yaw': [-8.0, 0.0, 0.0],
        'mark_eligible_scan': None, 'plans_after_mark': [],
    }

    assert not SMOKE.scenario_passed(document, 'detour', 0)


def _valid_detour():
    return {
        'action_terminal': 'succeeded', 'contact_count': 0,
        'contact_matched_publisher_count_max': 1,
        'global_blocking': True, 'local_blocking': True,
        'final_cmd_vel_zero': True,
        'final_robot_pose_xy_yaw': [6.0, 0.0, 0.0],
        'mark_eligible_scan': {'stamp_ns': 10},
        'plans_after_mark': [{'stamp_ns': 20}],
    }


def test_detour_requires_observed_costmap_and_contact_evidence():
    """Accept detour only with all independent observation paths."""
    assert SMOKE.scenario_passed(_valid_detour(), 'detour', 0)


@pytest.mark.parametrize('field,missing', [
    ('global_blocking', False), ('local_blocking', False),
    ('contact_matched_publisher_count_max', 0),
])
def test_missing_obstacle_or_contact_observation_cannot_pass(field, missing):
    """Require the contact publisher before accepting zero contacts."""
    document = _valid_detour()
    document[field] = missing
    assert not SMOKE.scenario_passed(document, 'detour', 0)
    del document[field]
    assert not SMOKE.scenario_passed(document, 'detour', 0)


def test_compact_recorder_uses_wall_log_time_and_hidden_status_qos(
        monkeypatch, tmp_path):
    """Keep sensor and same-goal evidence on one recorder wall clock."""
    captured = {}

    def fake_start(command, log_path, environment):
        captured.update(command=command, log_path=log_path,
                        environment=environment)
        return object(), object()

    monkeypatch.setattr(SMOKE, '_start', fake_start)
    SMOKE._start_compact_recorder(tmp_path, {'ROS_DOMAIN_ID': '186'})

    command = captured['command']
    assert command.count('ros2') == 1
    assert '--include-hidden-topics' in command
    assert '--use-sim-time' not in command
    assert '/scan' in command
    assert '/joint_states' not in command
    assert set(SMOKE.RECORDED_TOPICS) <= set(command)
    qos = yaml.safe_load(
        (tmp_path / 'recording_qos.yaml').read_text(encoding='utf-8'))
    status = qos['/navigate_to_pose/_action/status']
    assert status == {
        'depth': 1, 'durability': 'transient_local',
        'history': 'keep_last', 'reliability': 'reliable'}
    assert all(
        profile['reliability'] == 'best_effort'
        for topic, profile in qos.items()
        if topic != '/navigate_to_pose/_action/status')


def test_travel_pose_sample_rejects_a_joint_outside_tolerance():
    """Bind the expanded field to the exact arm pose used to derive it."""
    output = """header: {}
name: [arm_shoulder_pan, arm_shoulder_lift]
position: [0.0, -0.95]
velocity: []
effort: []
"""
    sample = SMOKE._travel_pose_sample(
        output, {'arm_shoulder_pan': 0.0, 'arm_shoulder_lift': -1.0}, 0.03)

    assert sample['status'] == 'FAIL'
    assert sample['maximum_error_rad'] == pytest.approx(0.05)
    assert sample['missing_joints'] == []


def test_travel_pose_sample_ignores_ros_cli_loss_report():
    """Ignore ros2 topic echo diagnostics surrounding the YAML document."""
    output = """A message was lost!!!
\ttotal count change:1
header: {}
name: [arm_shoulder_pan]
position: [0.0]
velocity: []
effort: []
---
\ttotal count change:2
"""

    sample = SMOKE._travel_pose_sample(
        output, {'arm_shoulder_pan': 0.0}, 0.03)

    assert sample['status'] == 'PASS'


def test_read_travel_pose_retries_until_all_arm_joints_arrive(monkeypatch):
    """Do not reject startup while the broadcaster is still adding joints."""
    partial = subprocess.CompletedProcess(
        args=[], returncode=0,
        stdout='name: [left_wheel_joint]\nposition: [0.0]\n', stderr='')
    complete = subprocess.CompletedProcess(
        args=[], returncode=0,
        stdout=('name: [arm_shoulder_pan, arm_shoulder_lift]\n'
                'position: [0.0, -1.0]\n'), stderr='')
    responses = iter((partial, complete))
    monkeypatch.setattr(SMOKE, '_run', lambda *args, **kwargs: next(responses))
    contract = {
        'travel_pose_envelope': {
            'joint_positions_rad': {
                'arm_shoulder_pan': 0.0,
                'arm_shoulder_lift': -1.0}},
        'travel_pose_gate': {'position_tolerance_rad': 0.03},
    }

    sample = SMOKE._read_travel_pose({}, contract, timeout_s=1.0)

    assert sample['status'] == 'PASS'
    assert sample['missing_joints'] == []


def test_output_cap_fails_without_rewriting_raw_logs(monkeypatch, tmp_path):
    """Preserve original evidence when the hard size cap is exceeded."""
    raw = b'original recorder evidence'
    log = tmp_path / 'recorder.log'
    log.write_bytes(raw)
    monkeypatch.setattr(SMOKE, 'CAP_BYTES', len(raw) - 1)

    with pytest.raises(RuntimeError, match='exceeds 64 MiB cap'):
        SMOKE._cap_output(tmp_path)

    assert log.read_bytes() == raw


def test_sudden_case_uses_direct_scan_on_the_operational_monitor_input(
        tmp_path):
    """Route the sudden case to the Collision Monitor scenario only."""
    prepared = {
        'contract': tmp_path / 'navigation.json',
        'stop_contract': tmp_path / 'stop.json',
    }
    command = SMOKE._scenario_command(
        'sudden_stop_resume', prepared, tmp_path / 'evidence.json',
        'probe', '/contact')

    assert 'sim_collision_monitor_scenario' in command
    assert 'sudden_obstacle_stop_resume' in command
    assert '--direct-scan' in command
    assert command[command.index('--obstacle-hold-s') + 1] == '0.0'
    assert command[command.index('--obstacle-crossing-s') + 1] == '0.0'
    assert command[
        command.index('--obstacle-crossing-edge-y-m') + 1] == '1.0'
    assert command[command.index('--obstacle-entry-side') + 1] == 'left'
    assert str(prepared['stop_contract']) in command
    assert '--behavior-tree' not in command


def test_combined_case_delays_pedestrian_until_after_static_detour(tmp_path):
    prepared = {
        'contract': tmp_path / 'navigation.json',
        'stop_contract': tmp_path / 'stop.json',
    }

    command = SMOKE._scenario_command(
        'detour_sudden_stop_resume', prepared,
        tmp_path / 'evidence.json', 'probe', '/contact')

    assert command[command.index('--trigger-after-x-m') + 1] == '1.0'
    assert '--direct-scan' in command


def test_capture_crossing_can_start_outside_the_doorway(tmp_path):
    """The capture-only start does not change the canonical corridor run."""
    prepared = {'stop_contract': tmp_path / 'stop.json'}
    command = SMOKE._scenario_command(
        'detour_sudden_stop_resume', prepared,
        tmp_path / 'evidence.json', 'probe', '/contact',
        obstacle_crossing_edge_y_m=1.5)
    assert command[command.index('--obstacle-crossing-edge-y-m') + 1] == '1.5'


def test_detour_evidence_requires_real_lateral_motion_and_clearance(
        monkeypatch, tmp_path):
    def sample(x_m, y_m, stamp_ns):
        stamp = SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000)
        pose = SimpleNamespace(
            position=SimpleNamespace(x=x_m, y=y_m),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
        return SimpleNamespace(
            ros_msg=SimpleNamespace(
                header=SimpleNamespace(stamp=stamp), pose=pose))

    samples = [
        sample(-2.0, 0.0, 1), sample(-1.4, 0.48, 2),
        sample(-1.0, 0.55, 3), sample(-0.6, 0.48, 4),
        sample(0.0, 0.0, 5),
    ]
    monkeypatch.setattr(
        SMOKE, 'read_navigation_messages',
        lambda *_args, **_kwargs: samples)
    route = {
        'name': 'route', 'active_pose_m': [-1.0, 0.0, 0.5],
        'length_m': 0.5, 'width_m': 0.4,
    }
    contract = {'stop_zone': {'inputs': {
        'footprint_front_m': 0.23, 'footprint_rear_m': -0.23,
        'footprint_half_width_m': 0.2,
    }}}

    evidence = SMOKE._detour_evidence(
        tmp_path / 'run.mcap', route, contract)

    assert evidence['status'] == 'PASS'
    assert evidence['straight_centerline_blocked'] is True
    assert evidence['maximum_abs_lateral_offset_m'] == pytest.approx(0.55)
    assert evidence['minimum_clearance_m'] > 0.0


def test_keepout_evidence_requires_north_passage_and_footprint_clearance(
        monkeypatch, tmp_path):
    def sample(x_m, y_m, stamp_ns):
        stamp = SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000)
        pose = SimpleNamespace(
            position=SimpleNamespace(x=x_m, y=y_m),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
        return SimpleNamespace(
            ros_msg=SimpleNamespace(
                header=SimpleNamespace(stamp=stamp), pose=pose))

    pixels = bytearray([254]) * 25
    pixels[3 * 5 + 2] = 0
    mask_image = tmp_path / 'mask.pgm'
    mask_image.write_bytes(b'P5\n5 5\n255\n' + bytes(pixels))
    mask_yaml = tmp_path / 'mask.yaml'
    mask_yaml.write_text(yaml.safe_dump({
        'image': mask_image.name, 'resolution': 1.0,
        'origin': [0.0, 0.0, 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }), encoding='utf-8')
    samples = [
        sample(1.0, 2.5, 1), sample(2.5, 2.5, 2),
        sample(4.0, 2.5, 3),
    ]
    monkeypatch.setattr(
        SMOKE, 'read_navigation_messages',
        lambda *_args, **_kwargs: samples)
    keepout = {
        'zone_id': 'demo', 'polygon_m': [], 'safety_margin_m': 0.35,
    }
    contract = {'stop_zone': {'inputs': {
        'footprint_front_m': 0.2, 'footprint_rear_m': -0.2,
        'footprint_half_width_m': 0.2,
    }}}

    evidence = SMOKE._keepout_route_evidence(
        tmp_path / 'run.mcap', mask_yaml, keepout, contract)

    assert evidence['status'] == 'PASS'
    assert evidence['route_side'] == 'north'
    assert evidence['minimum_center_y_m'] == pytest.approx(2.5)
    assert evidence['minimum_footprint_clearance_m'] > 0.0
    assert evidence['mask']['occupied_cell_count'] == 1
    assert evidence['sample_count'] == 3

    samples[1] = sample(2.5, 1.5, 2)
    evidence = SMOKE._keepout_route_evidence(
        tmp_path / 'run.mcap', mask_yaml, keepout, contract)
    assert evidence['status'] == 'FAIL'


def test_video_route_activation_records_empty_scene_interval(monkeypatch):
    sleeps = []
    wall_ns = iter((100, 200))
    steady_ns = iter((300, 400))
    route = {
        'name': 'route', 'active_pose_m': [-1.0, 0.0, 0.5],
        'length_m': 0.5, 'width_m': 0.4, 'height_m': 1.0,
    }
    monkeypatch.setattr(SMOKE.time, 'sleep', sleeps.append)
    monkeypatch.setattr(SMOKE.time, 'time_ns', lambda: next(wall_ns))
    monkeypatch.setattr(
        SMOKE.time, 'monotonic_ns', lambda: next(steady_ns))
    monkeypatch.setattr(SMOKE, '_set_pose', lambda *_args: True)
    monkeypatch.setattr(
        SMOKE, '_wait_entity_pose',
        lambda *_args, **_kwargs: {
            'entity_id': 7, 'pose_m': [-1.0, 0.0, 0.5]})

    evidence = SMOKE._activate_route_obstacle(
        route, {}, 'verified_after_recorders_ready_before_goal', 2.0)

    assert sleeps == [2.0]
    assert evidence['activation_requested_wall_ns'] == 100
    assert evidence['activation_verified_wall_ns'] == 200
    assert evidence['activation_requested_steady_ns'] == 300
    assert evidence['activation_verified_steady_ns'] == 400
    assert evidence['pre_activation_observation_s'] == 2.0


def test_reevaluation_preserves_source_and_gates_revised_status(
        monkeypatch, tmp_path):
    source = tmp_path / 'source'
    with pytest.raises(ValueError, match='outside'):
        REEVALUATE.reevaluate(source, source / 'assets' / 'nested')
    case = source / 'detour_sudden_stop_resume'
    assets = source / 'assets'
    bag = case / 'bag'
    bag.mkdir(parents=True)
    assets.mkdir()
    source_scenario = {
        'action_terminal': 'tampered', 'contact_count': 9,
        'clear_scan_stamp_ns': None,
    }
    (case / 'scenario.json').write_text(
        json.dumps(source_scenario), encoding='utf-8')
    mcap = bag / 'bag_0.mcap'
    mcap.write_bytes(b'mcap')
    stop_contract = assets / 'onboard_stop_contract.json'
    stop_contract.write_text(json.dumps({
        'goal_pose': {'x_m': 6.0, 'y_m': 0.0},
        'final_zero_hold_s': 1.95,
    }), encoding='utf-8')
    mask_yaml = assets / 'sim_keepout_mask.yaml'
    mask_yaml.write_text(
        'image: mask.pgm\n', encoding='utf-8')
    mask_image = assets / 'mask.pgm'
    mask_image.write_bytes(b'mask')
    zones = assets / 'sim_keepout_zones.yaml'
    zones.write_text(yaml.safe_dump({
        'safety_margin_m': 0.35,
        'zones': [{'id': 'demo', 'polygon': [[0, 0], [1, 0], [0, 1]]}],
    }), encoding='utf-8')
    (assets / 'sim_keepout_mask.json').write_text(json.dumps({
        'mask_sha256': SMOKE._sha256(mask_image),
        'mask_yaml': str(mask_yaml.resolve()),
        'zones_yaml': str(zones.resolve()),
        'zones': ['demo'], 'keepout_cells': 1, 'safety_margin_m': 0.35,
    }), encoding='utf-8')
    summary_scenario = {
        'returncode': 0, 'action_terminal': 'succeeded',
        'goal_uuid': '01' * 16, 'terminal_goal_uuid': '01' * 16,
        'goal_send_count': 1, 'goal_cancel_count': 0,
        'stop_action_type': 1, 'stop_polygon_name': 'StopZone',
        'resume_action_type': 0, 'physical_stop_observed': True,
        'contact_matched_publisher_count_max': 1, 'contact_count': 0,
        'minimum_clearance_m': 0.05,
        'protected_envelope_minimum_clearance_m': 0.02,
        'final_cmd_vel_zero': True, 'final_zero_hold_s': 2.0,
        'final_world_pose_m': [6.0, 0.0], 'activation_error': None,
        'harness_error': None, 'same_goal_command_verdict': 'CONFIRMED',
        'direct_scan_capture': {
            'scan_receive_steady_ns': 1, 'zero_receive_steady_ns': 2},
        'events': ['physical_stop', 'obstacle_deactivated', 'succeeded'],
    }
    original_result = {
        'case': case.name, 'status': 'FAIL', 'harness_error': None,
        'scenario': summary_scenario,
        'detour_evidence': {'status': 'PASS'},
        'same_goal_command_evidence': {'evidence': {
            'verdict': 'CONFIRMED', 'terminal_succeeded': True,
            'episodes': [{
                'same_goal_resumed': True, 'terminal_succeeded': True}]}},
        'raw_action_status': {'final_status': 4},
        'recording': {'mcap': {'sha256': SMOKE._sha256(mcap)}},
        'source_identity': {
            'keepout_mask': REEVALUATE._source_identity(mask_yaml),
            'keepout_zones': REEVALUATE._source_identity(zones),
            'scenario_contract': REEVALUATE._source_identity(stop_contract),
        },
        'teardown': {
            'remaining_process_groups': [], 'identity_survivors': []},
        'keepout_route_evidence': {'status': 'FAIL'},
    }
    root_summary = {'status': 'FAIL', 'results': [original_result]}
    (source / 'summary.json').write_text(
        json.dumps(root_summary), encoding='utf-8')
    (case / 'summary.json').write_text(
        json.dumps(original_result), encoding='utf-8')
    monkeypatch.setattr(
        REEVALUATE, '_keepout_route_evidence',
        lambda *_args: {
            'status': 'PASS', 'minimum_footprint_clearance_m': 0.04,
            'mask': {'occupied_cell_count': 1}})
    original_sha = REEVALUATE._sha256(source / 'summary.json')

    output = tmp_path / 'revised'
    result = REEVALUATE.reevaluate(source, output)

    assert result['status'] == 'PASS'
    assert result['original_status'] == 'FAIL'
    revised = json.loads((output / case.name / 'summary.json').read_text())
    assert revised['status'] == 'PASS'
    assert revised['original_status'] == 'FAIL'
    assert revised['reevaluation']['other_original_checks_passed'] is True
    assert revised['reevaluation']['scenario_usage'] == (
        'retained_copy_only_not_status_input')
    assert revised['reevaluation'][
        'historical_scenario_integrity_verified'] is False
    assert revised['reevaluation']['source_files_modified'] is False
    assert (output / 'source_summary.json').is_file()
    assert REEVALUATE._sha256(source / 'summary.json') == original_sha

    original_result['recording_error'] = 'incomplete recording'
    original_result['teardown_error'] = 'owned process remains'
    root_summary['results'] = [original_result]
    root_summary['output_error'] = 'evidence cap exceeded'
    (source / 'summary.json').write_text(
        json.dumps(root_summary), encoding='utf-8')
    (case / 'summary.json').write_text(
        json.dumps(original_result), encoding='utf-8')

    blocked = REEVALUATE.reevaluate(source, tmp_path / 'blocked')

    assert blocked['status'] == 'FAIL'
    blocked_checks = blocked['reevaluation']['other_original_checks']
    assert blocked_checks['original_recording_error_absent'] is False
    assert blocked_checks['original_teardown_error_absent'] is False
    assert blocked_checks['root_output_error_absent'] is False


def _valid_sudden_stop_resume():
    return {
        'goal_uuid': '01' * 16,
        'terminal_goal_uuid': '01' * 16,
        'goal_send_count': 1,
        'goal_cancel_count': 0,
        'action_terminal': 'succeeded',
        'pre_stop_action_types': [2],
        'stop_action_type': 1,
        'stop_polygon_name': 'StopZone',
        'resume_action_type': 0,
        'physical_stop_observed': True,
        'clear_scan_stamp_ns': 3,
        'contact_matched_publisher_count_max': 1,
        'contact_count': 0,
        'footprint_to_obstacle_clearance_m': 0.05,
        'protected_envelope_to_obstacle_clearance_m': 0.02,
        'final_cmd_vel_zero': True,
        'final_zero_hold_s': 2.0,
        'final_world_pose_m': [6.0, 0.0],
        'direct_scan_capture': {
            'scan_receive_steady_ns': 1,
            'zero_receive_steady_ns': 2,
        },
        'activation_error': None,
        'harness_error': None,
        'contract': {
            'goal_pose': {'x_m': 6.0, 'y_m': 0.0},
            'final_zero_hold_s': 1.95,
        },
    }


def test_sudden_case_requires_physical_and_recorded_same_goal_evidence():
    """Do not pass a command-only or physical-only stop-resume result."""
    same_goal = {
        'evidence': {'verdict': 'CONFIRMED', 'terminal_succeeded': True}}
    document = _valid_sudden_stop_resume()

    assert SMOKE._case_passed(document, 'sudden_stop_resume', 0, same_goal)

    document['physical_stop_observed'] = False
    assert not SMOKE._case_passed(
        document, 'sudden_stop_resume', 0, same_goal)
    document['physical_stop_observed'] = True
    same_goal['evidence']['verdict'] = 'NOT_CONFIRMED'
    assert not SMOKE._case_passed(
        document, 'sudden_stop_resume', 0, same_goal)
    same_goal['evidence']['verdict'] = 'CONFIRMED'
    document['footprint_to_obstacle_clearance_m'] = 0.0
    assert not SMOKE._case_passed(
        document, 'sudden_stop_resume', 0, same_goal)
    document['footprint_to_obstacle_clearance_m'] = 0.05
    document['protected_envelope_to_obstacle_clearance_m'] = 0.0
    assert not SMOKE._case_passed(
        document, 'sudden_stop_resume', 0, same_goal)


def test_combined_case_requires_static_detour_evidence():
    same_goal = {
        'evidence': {'verdict': 'CONFIRMED', 'terminal_succeeded': True}}
    document = _valid_sudden_stop_resume()

    assert SMOKE._case_passed(
        document, 'detour_sudden_stop_resume', 0, same_goal,
        {'status': 'PASS'})
    assert not SMOKE._case_passed(
        document, 'detour_sudden_stop_resume', 0, same_goal,
        {'status': 'FAIL'})


def test_contact_filter_excludes_ground_and_keeps_robot_pair():
    """Exclude ground support while retaining probe-to-robot contacts."""
    entity = 'g003_preloaded_front_observation_probe'

    def contact(other):
        return SimpleNamespace(
            collision1=SimpleNamespace(name=f'{entity}::body::collision'),
            collision2=SimpleNamespace(name=other))

    assert robot_entity_contact_pair(
        contact('default::ground_plane::link::collision'), entity) is None
    assert robot_entity_contact_pair(
        contact('jdamr_cube::base_link::collision'), entity) is not None
