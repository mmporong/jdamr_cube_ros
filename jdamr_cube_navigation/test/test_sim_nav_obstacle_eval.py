"""Lock the deterministic fixed-obstacle Nav2 evaluation contract."""

import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path  # noqa: I100

import pytest

import yaml


ROOT = Path(__file__).resolve().parents[2]
EVALUATION = ROOT / 'jdamr_cube_navigation' / 'evaluation'
ASSETS = EVALUATION / 'assets' / 'nav_obstacle'
BT = (ROOT / 'jdamr_cube_navigation' / 'behavior_trees'
      / 'navigate_to_pose_dynamic_obstacle_eval.xml')
PRODUCTION_WORLD = (ROOT / 'jdamr_cube_gazebo' / 'worlds'
                    / 'slam_corridor.world')
PRODUCTION_URDF = (ROOT / 'jdamr_cube_description' / 'urdf'
                   / 'jdamr_cube.urdf')
sys.path.insert(0, str(EVALUATION))
sys.path.insert(0, str(ROOT / 'jdamr_cube_navigation'))


def _load_module(filename):
    spec = importlib.util.spec_from_file_location(
        filename.removesuffix('.py'), EVALUATION / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_scenario_module():
    path = (ROOT / 'jdamr_cube_navigation' / 'jdamr_cube_navigation'
            / 'sim_nav_obstacle_scenario.py')
    spec = importlib.util.spec_from_file_location(
        'sim_nav_obstacle_scenario_source', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _ClearFuture:
    def __init__(self, *, done=True, result=object()):
        self._done = done
        self._result = result

    def done(self):
        return self._done

    def result(self):
        return self._result


class _ClearClient:
    def __init__(self, *, available=True, future=None):
        self.available = available
        self.future = future or _ClearFuture()
        self.calls = 0

    def wait_for_service(self, timeout_sec):
        assert timeout_sec > 0.0
        return self.available

    def call_async(self, request):
        self.calls += 1
        return self.future


def _clear_observer(module):
    observer = module.ScenarioObserver.__new__(module.ScenarioObserver)
    observer.args = type('Args', (), {'scenario': 'event_driven_removal'})()
    observer.contract = {
        'clear_observation_count': 2,
        'passive_clear_window_s': 6.0,
        'clear_service_timeout_s': 5.0,
        'post_recovery_clear_window_s': 6.0,
    }
    observer.events = []
    observer.start_s = module.time.monotonic()
    observer.clear_mode = None
    observer.clear_observation_counts = {'global': 0, 'local': 0}
    observer.clear_service_request_counts = {'global': 0, 'local': 0}
    observer.clear_service_response_counts = {'global': 0, 'local': 0}
    observer.clear_service_futures = {}
    observer.clear_observation_trace = []
    observer.clear_observation_sequence_counts = {'global': 0, 'local': 0}
    observer.clear_roi_was_observable = {'global': False, 'local': False}
    observer.clear_clients = {
        'global': _ClearClient(), 'local': _ClearClient()}
    observer.new_scan_after_deactivate_s = None
    observer.recovery_started_s = None
    observer.recovery_responses_complete_s = None
    observer.clear_epoch = 0
    observer.final_clear_completion_s = None
    observer.final_post_clear_plan_s = None
    observer.clear_completion_history = []
    observer.post_clear_plan_history = []
    observer.final_clear_samples = {}
    observer.removal_passage = None
    observer.global_cleared = False
    observer.local_cleared = False
    observer.action_terminal_s = None
    observer.activation_error = None
    observer.robot_pose_xy_yaw = None
    observer.odom_pose_xy_yaw = None
    return observer


def _clear_statistics(*, blocking=False, sampled_cell_count=1):
    cost = 253 if blocking else 0
    return {
        'blocking': blocking,
        'blocking_count': int(blocking),
        'lethal': False,
        'lethal_count': 0,
        'inscribed_count': int(blocking),
        'unknown_count': 0,
        'blocking_cells': ([{'index_x': 1, 'index_y': 2,
                             'center_x_m': 0.1, 'center_y_m': 0.2,
                             'cost': cost}] if blocking else []),
        'sampled_cell_count': sampled_cell_count,
        'baseline_excess_count': int(blocking),
        'influence_excess_count': int(blocking),
        'influence_probe': {
            'sampled_cell_count': sampled_cell_count,
            'unknown_count': 0,
        },
        'max_cost': cost if sampled_cell_count else -1,
        'index_bounds': [0, sampled_cell_count, 0, 1],
        'metadata': {'origin_x_m': 0.0, 'origin_y_m': 0.0,
                     'resolution_m': 0.05, 'size_x': 10, 'size_y': 10},
    }


def _costmap_message():
    stamp = type('Stamp', (), {'sec': 1, 'nanosec': 2})()
    header = type('Header', (), {'stamp': stamp})()
    return type('Costmap', (), {'header': header})()


def _grid_costmap(costs, stamp_ns=1, frame_id='map'):
    stamp = type('Stamp', (), {
        'sec': stamp_ns // 1_000_000_000,
        'nanosec': stamp_ns % 1_000_000_000})()
    header = type('Header', (), {'stamp': stamp, 'frame_id': frame_id})()
    point = type('Point', (), {'x': 0.0, 'y': 0.0})()
    origin = type('Origin', (), {'position': point})()
    metadata = type('Metadata', (), {
        'resolution': 0.05, 'size_x': 2, 'size_y': 2,
        'origin': origin})()
    return type('Costmap', (), {
        'header': header, 'metadata': metadata, 'data': costs})()


def _clear_trace_fixture():
    return [
        {'source': source, 'stamp_ns': sequence,
         'surface_probe': {'blocking': False},
         'padded_planning_probe': {
             'blocking': True, 'baseline_excess_count': 0,
             'influence_excess_count': 0,
             'influence_probe': {
                 'sampled_cell_count': 1, 'unknown_count': 0},
             'tf_stamp_ns': sequence},
         'observable': True, 'counter_before': sequence - 1,
         'counter_after': sequence, 'decision': 'increment'}
        for source in ('global', 'local') for sequence in (1, 2)]


def _removal_epoch_fixture():
    geometry = {'resolution_m': 0.05}
    baseline = [
        {'stamp_ns': stamp_ns, 'geometry': geometry,
         'canonical_class_sha256': 'fixed',
         'sampled_cell_count': 1, 'unknown_count': 0}
        for stamp_ns in (1, 2)]
    mark = {
        source: {'baseline_excess_count': 1,
                 'influence_roi_intersects': True}
        for source in ('global', 'local')}
    return {
        'baseline_ready': True,
        'global_baseline_snapshots': baseline,
        'mark_excess_snapshots': mark,
        'pending_local_costmap_failures': [],
        'final_clear_completion_elapsed_s': 10.0,
        'final_post_clear_plan_elapsed_s': 11.0,
        'action_terminal_elapsed_s': 12.0,
        'clear_completion_history': [{
            'epoch': 1,
            'source_samples': {
                source: {'stamp_ns': 100 + index,
                         'receive_elapsed_s': 10.0 + index * 0.1}
                for index, source in enumerate(('global', 'local'))},
        }],
        'post_clear_plan_history': [{
            'epoch': 1, 'stamp_ns': 110, 'receive_elapsed_s': 11.0}],
        'removal_passage': {
            'epoch': 1, 'stamp_ns': 120, 'receive_elapsed_s': 11.5,
            'robot_x_m': 0.0},
    }


def test_contact_parser_accepts_unquoted_ros_yaml_and_excludes_base():
    """ROS YAML collision names need no quotes and arm_base is not base."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    probe = 'g003_contact_probe_sensor::body::collision'
    arm_base = ('jdamr_cube::base_footprint::base_footprint_fixed_joint_lump'
                '__arm_base_link_collision_4')
    arm_camera = ('jdamr_cube::arm_wrist_link::arm_wrist_link_fixed_joint_lump'
                  '__arm_wrist_camera_link_collision_2')
    base = ('jdamr_cube::base_footprint::fixed_joint_lump'
            '__base_link_collision')
    output = '\n'.join([
        'contacts:', '- collision1:', f'    name: {probe}',
        '  collision2:', f'    name: {arm_base}', '- collision1:',
        f'    name: {probe}', '  collision2:', f'    name: {arm_camera}',
        '- collision1:', f'    name: {probe}', '  collision2:',
        f'    name: {base}'])

    names = module._collision_names(output)
    arm_tokens = module._arm_link_tokens(ASSETS / 'jdamr_cube_nav_eval.urdf')
    arm_names, unexpected_names = module._classify_robot_collisions(
        names, arm_tokens)

    assert len(names) == 4
    assert len(arm_names) == 2
    assert len(unexpected_names) == 1
    assert unexpected_names[0].endswith('__base_link_collision')


def test_scan_stream_requires_complete_strictly_newer_document():
    """A persistent scan stream rejects stale and incomplete messages."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    stream = (
        'header:\n  stamp:\n    sec: 5\n    nanosec: 500000000\n'
        'ranges: [1.0]\n---\n'
        'header:\n  stamp:\n    sec: 5\n    nanosec: 600000000\n'
        'ranges: [2.0]\n')

    assert module._newer_scan_document(
        stream, newer_than_ns=5_500_000_000) is None
    document = module._newer_scan_document(
        stream + '---\n', newer_than_ns=5_500_000_000)

    assert module._scan_stamp_ns(document) == 5_600_000_000
    assert 'ranges: [2.0]' in document


def test_ros_echo_yaml_parser_ignores_transport_preamble():
    """Transport diagnostics before a ROS YAML message are not payload."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    parsed = module._yaml_message(
        'total count change:1\n\theader diagnostic\n'
        'header:\n  frame_id: laser_link\nranges: [1.0]\n---\n')

    assert parsed == {'header': {'frame_id': 'laser_link'}, 'ranges': [1.0]}


def test_preflight_cache_rejects_stale_hash_and_seed():
    """Only PASS evidence with exact provenance may skip a preflight."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    hashes = {'contract': 'abc', 'asset_manifest': 'def'}
    cached = {'status': 'PASS', 'input_hashes': hashes, 'seed': 23}

    assert module._preflight_cache_matches(cached, hashes, 23)
    assert not module._preflight_cache_matches(cached, hashes, 42)
    assert not module._preflight_cache_matches(
        cached, {'contract': 'changed', 'asset_manifest': 'def'}, 23)
    assert not module._preflight_cache_matches(
        {**cached, 'status': 'FAIL'}, hashes, 23)
    assert not module._preflight_cache_matches(
        {'status': 'PASS', 'seed': 23}, hashes, 23)


def test_contact_smoke_cache_requires_pass_and_exact_hashes(tmp_path):
    """A failed or stale contact preflight must not start the matrix."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    path = tmp_path / 'contact_smoke.json'
    hashes = {'evaluation_runner': 'expected'}

    assert not module._contact_smoke_cache_matches(path, hashes)
    path.write_text(json.dumps({
        'status': 'FAIL', 'input_hashes': hashes}), encoding='utf-8')
    assert not module._contact_smoke_cache_matches(path, hashes)
    path.write_text(json.dumps({
        'status': 'PASS',
        'input_hashes': {'evaluation_runner': 'stale'},
    }), encoding='utf-8')
    assert not module._contact_smoke_cache_matches(path, hashes)
    path.write_text(json.dumps({
        'status': 'PASS', 'input_hashes': hashes}), encoding='utf-8')
    assert module._contact_smoke_cache_matches(path, hashes)
    assert module.CONTACT_SMOKE_MAX_ATTEMPTS == 2


def test_failed_contact_smoke_archive_has_content_hash(tmp_path):
    """Retry audit must retain the failed preflight bytes and digest."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    source = tmp_path / 'contact_smoke.json'
    source.write_text('{"status":"FAIL"}\n', encoding='utf-8')

    archived = module._archive_contact_smoke(tmp_path)
    manifest = json.loads(
        (archived / 'attempt_manifest.json').read_text(encoding='utf-8'))

    assert ((archived / 'contact_smoke.json').read_bytes()
            == source.read_bytes())
    assert manifest['status'] == 'FAIL'
    assert manifest['file']['sha256'] == module._sha256(source)


def test_missing_contact_smoke_archive_is_audited_without_crashing(tmp_path):
    """A preflight exception before artifact creation remains auditable."""
    module = _load_module('run_sim_nav_obstacle_eval.py')

    archived = module._archive_contact_smoke(tmp_path)
    manifest = json.loads(
        (archived / 'attempt_manifest.json').read_text(encoding='utf-8'))

    assert manifest['status'] == 'FAIL'
    assert manifest['failure_reason'] == 'contact_smoke_artifact_missing'
    assert manifest['file'] is None


def test_process_group_wait_rechecks_until_members_are_reaped(monkeypatch):
    """A transient post-kill member must be rechecked before evidence."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    observations = iter(([101], [101], []))
    sleeps = []
    monkeypatch.setattr(
        module, '_group_members', lambda process_group: next(observations))
    monkeypatch.setattr(module.time, 'sleep', sleeps.append)

    assert module._wait_process_group_empty(99, 2.0) is True
    assert sleeps == [0.05, 0.05]


def test_process_group_wait_reports_bounded_timeout(monkeypatch):
    """A persistent process-group member remains a fail-closed result."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    monotonic_values = iter((10.0, 12.0))
    monkeypatch.setattr(module, '_group_members', lambda process_group: [101])
    monkeypatch.setattr(
        module.time, 'monotonic', lambda: next(monotonic_values))

    assert module._wait_process_group_empty(99, 2.0) is False


@pytest.mark.parametrize('output', [
    'inactive [2]\n',
    'unconfigured [1]\n',
    'finalized [4]\n',
    '',
])
def test_lifecycle_parser_rejects_non_active_states(output):
    """Substring matches must not make an inactive Nav2 node ready."""
    module = _load_module('run_sim_nav_obstacle_eval.py')

    assert not module._lifecycle_state_is_active(output)


def test_lifecycle_parser_accepts_only_active_state_three():
    """The Jazzy CLI active state includes numeric lifecycle ID three."""
    module = _load_module('run_sim_nav_obstacle_eval.py')

    assert module._lifecycle_state_is_active('active [3]\n')
    assert not module._lifecycle_state_is_active('active [2]\n')


def test_tf_echo_parser_requires_complete_transform_sample():
    """A waiting diagnostic is not map-to-base TF readiness evidence."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    complete = (
        'At time 10.0\n'
        '- Translation: [1.0, 2.0, 0.0]\n'
        '- Rotation: in Quaternion (xyzw) [0.0, 0.0, 0.0, 1.0]\n')

    assert module._tf_echo_has_transform(complete)
    assert module._tf_echo_has_transform(complete.encode())
    assert not module._tf_echo_has_transform(
        '[INFO] Waiting for transform map -> base_footprint\n')


def test_contract_derives_clearance_and_time_limits_from_production_config():
    """Timing and clearance limits must come from production values."""
    module = _load_module('sim_nav_obstacle_contract.py')
    params = yaml.safe_load((ROOT / 'jdamr_cube_navigation' / 'config'
                            / 'nav2_params.yaml').read_text())

    contract = module.derive_contract(params)

    assert contract['nav2_process_topology'] == 'component_container_isolated'
    expected_radius_m = math.hypot(0.23, 0.20)
    expected_clearance_m = max(expected_radius_m, 0.30) + 0.05 / math.sqrt(2)
    assert contract['robot_circumscribed_radius_m'] == expected_radius_m
    assert contract['path_clearance_m'] == expected_clearance_m
    assert contract['global_update_period_s'] == 1.0
    assert contract['global_publish_period_s'] == 2.0
    assert contract['costmap_resolution_m'] == 0.05
    assert contract['final_zero_hold_s'] == 2.0
    assert contract['blocked_terminal_limit_s'] == 5.02
    assert contract['clear_observation_count'] == 2
    assert contract['local_publish_period_s'] == 1.0
    assert contract['passive_clear_window_s'] == 6.0
    assert contract['clear_service_timeout_s'] == 5.0
    assert contract['post_recovery_clear_window_s'] == 6.0


def test_scenario_matrix_is_exact_and_uses_five_fixed_seeds():
    """The durable matrix must contain every scenario and seed once."""
    module = _load_module('sim_nav_obstacle_contract.py')

    matrix = module.scenario_matrix()

    assert len(matrix) == 25
    assert {item['scenario'] for item in matrix} == {
        'baseline', 'detour', 'event_driven_removal', 'full_block',
        'goal_occupied'}
    assert {item['seed'] for item in matrix} == {11, 23, 42, 67, 89}
    assert len({item['run_id'] for item in matrix}) == 25


def test_evaluation_bt_replans_and_has_one_bounded_clear_wait_recovery():
    """The evaluation tree must replan without motion recovery behaviors."""
    root = ET.parse(BT).getroot()
    text = BT.read_text(encoding='utf-8')

    assert '<Spin' not in text
    assert '<BackUp' not in text
    assert root.find('.//PipelineSequence') is not None
    rate = root.find('.//RateController')
    assert rate is not None and float(rate.attrib['hz']) == 1.0
    recovery = root.find('.//RecoveryNode')
    assert recovery is not None
    assert recovery.attrib['number_of_retries'] == '1'
    global_clear = root.find(
        './/ClearEntireCostmap[@service_name='
        "'global_costmap/clear_entirely_global_costmap']")
    local_clear = root.find(
        './/ClearEntireCostmap[@service_name='
        "'local_costmap/clear_entirely_local_costmap']")
    assert global_clear is not None
    assert local_clear is not None
    wait = root.find('.//Wait')
    assert wait is not None and float(wait.attrib['wait_duration']) == 1.0


def test_generated_assets_are_deterministic_and_preserve_source_world(
        tmp_path):
    """Fresh generation must be byte-stable and leave source untouched."""
    module = _load_module('generate_sim_nav_obstacle_assets.py')
    before = module.sha256_file(PRODUCTION_WORLD)

    first = module.generate_assets(tmp_path / 'first')
    second = module.generate_assets(tmp_path / 'second')

    assert module.sha256_file(PRODUCTION_WORLD) == before
    assert first['source_world']['sha256'] == before
    assert first['content_hashes'] == second['content_hashes']
    world = ET.parse(tmp_path / 'first' / 'slam_corridor_contact.world')
    plugins = {plugin.attrib.get('filename')
               for plugin in world.getroot().findall('./world/plugin')}
    assert 'gz-sim-contact-system' in plugins
    preloaded = first['preloaded_obstacles']
    assert preloaded['count'] == 6
    assert preloaded['runtime_create_delete_forbidden'] is True
    assert preloaded['minimum_pairwise_aabb_clearance_m'] > 0.0
    assert set(preloaded['models']) == {
        'route', 'full_block', 'goal_occupied',
        'sub_minimum_probe', 'front_observation_probe', 'contact_control'}
    for spec in preloaded['models'].values():
        assert spec['static'] is True
        assert spec['visual_visibility_flags'] == 4
        assert spec['contact_collision_ref'] == 'collision'
    map_yaml = yaml.safe_load(
        (tmp_path / 'first' / 'slam_corridor_eval.yaml').read_text())
    assert map_yaml['resolution'] == 0.05
    assert map_yaml['image'] == 'slam_corridor_eval.pgm'
    robot = ET.parse(tmp_path / 'first' / 'jdamr_cube_nav_eval.urdf')
    source_robot = ET.parse(PRODUCTION_URDF)
    source_collisions = module._collision_records(source_robot.getroot())
    generated_collisions = module._collision_records(robot.getroot())
    assert generated_collisions == source_collisions
    assert first['source_robot']['removed_collision_elements'] == []
    assert first['source_robot']['collision_elements'] == source_collisions
    wheel_slip = robot.getroot().find(
        ".//plugin[@filename='gz-sim-wheel-slip-system']")
    assert wheel_slip is not None
    wheels = wheel_slip.findall('wheel')
    assert [wheel.get('link_name') for wheel in wheels] == [
        'left_wheel_link', 'right_wheel_link']
    assert [wheel.findtext('slip_compliance_longitudinal')
            for wheel in wheels] == ['0.0', '0.0']
    assert first['simulation_support_plane_correction'][
        'support_plane_correction'][
            'wheel_slip_runtime_fault']['wheel_normal_force_n'] == 70.0
    assert module._visual_records(robot.getroot()) == module._visual_records(
        source_robot.getroot())
    changed = first['source_robot']['allowed_joint_deltas']
    assert [item['name'] for item in changed] == [
        'caster_front_joint', 'caster_rear_joint']
    for item in changed:
        assert item['before_z_m'] == -0.005
        assert item['after_z_m'] == pytest.approx(-0.053)
        origin = robot.getroot().find(
            f"./joint[@name='{item['name']}']/origin")
        assert float(origin.attrib['xyz'].split()[2]) == pytest.approx(-0.053)
    sensor_path = (
        "./gazebo[@reference='laser_link']/sensor[@name='laser_sensor']")
    source_sensor = source_robot.getroot().find(sensor_path)
    generated_sensor = robot.getroot().find(sensor_path)
    for child_path in (
            'pose', 'update_rate', 'always_on', 'topic', 'gz_frame_id',
            'lidar/scan/horizontal/samples',
            'lidar/scan/horizontal/resolution', 'lidar/range/max',
            'lidar/range/resolution', 'lidar/noise/type',
            'lidar/noise/mean', 'lidar/noise/stddev'):
        assert generated_sensor.findtext(child_path) == source_sensor.findtext(
            child_path)
    horizontal = generated_sensor.find('lidar/scan/horizontal')
    sample_count = int(horizontal.findtext('samples'))
    angle_increment_rad = 2.0 * math.pi / sample_count
    assert float(horizontal.findtext('min_angle')) == -math.pi
    assert float(horizontal.findtext('max_angle')) == pytest.approx(
        -math.pi + (sample_count - 1) * angle_increment_rad)
    generated_range_min_m = float(generated_sensor.findtext(
        'lidar/range/min'))
    assert generated_range_min_m == pytest.approx(0.28)
    assert generated_sensor.findtext('lidar/visibility_mask') == '4'
    visual_links = {
        link.attrib['name'] for link in source_robot.getroot().findall('link')
        if link.findall('visual')}
    flagged_links = {
        gazebo.attrib['reference']
        for gazebo in robot.getroot().findall('gazebo')
        if gazebo.findtext('visual/visibility_flags') == '11'}
    assert len(visual_links) == 17
    assert flagged_links == visual_links
    assert first['source_robot']['allowed_sensor_deltas'] == {
        'laser_horizontal_min_angle_rad': -math.pi,
        'laser_horizontal_max_angle_rad': pytest.approx(
            -math.pi + (sample_count - 1) * angle_increment_rad),
        'laser_range_min_m': pytest.approx(0.28),
        'laser_visibility_mask': 4,
    }
    converted = first['simulation_support_plane_correction'][
        'converted_sdf_validation']
    assert converted['visual_count'] == 27
    assert converted['visual_visibility_flags'] == 11
    assert converted['lidar_visibility_mask'] == 4
    assert converted['visual_semantics_equal_except_visibility_flags'] is True


def test_level_caster_height_is_derived_from_support_geometry():
    """Caster correction must follow wheel and sphere geometry."""
    module = _load_module('generate_sim_nav_obstacle_assets.py')
    root = ET.parse(PRODUCTION_URDF).getroot()

    geometry = module.derive_level_caster_geometry(root)

    assert geometry['wheel_bottom_from_base_m'] == pytest.approx(-0.083)
    assert geometry['caster_bottom_from_base_m'] == pytest.approx(-0.035)
    assert geometry['current_support_height_delta_m'] == pytest.approx(0.048)
    assert geometry['level_caster_joint_z_m'] == pytest.approx(-0.053)


def test_front_observation_probe_is_derived_and_noncontacting():
    """The sensor probe must start in range without touching footprint."""
    module = _load_module('generate_sim_nav_obstacle_assets.py')

    probe = module.derive_front_observation_probe()

    assert probe['start_to_center_m'] == 1.5
    assert probe['start_to_near_surface_m'] == 1.25
    assert probe['footprint_to_near_surface_m'] == 1.02
    assert probe['contact_expected'] is False


def test_fixed_assets_match_fresh_generation(tmp_path):
    """Checked-in evaluation assets must match the generator exactly."""
    module = _load_module('generate_sim_nav_obstacle_assets.py')
    manifest = module.generate_assets(tmp_path)
    fixed = json.loads((ASSETS / 'asset_manifest.json').read_text())

    assert manifest['content_hashes'] == fixed['content_hashes']
    for filename in manifest['content_hashes']:
        assert ((tmp_path / filename).read_bytes()
                == (ASSETS / filename).read_bytes())


def test_path_clearance_rejects_segments_crossing_inflated_obstacle():
    """Finite path segments must stay outside the expanded obstacle AABB."""
    module = _load_module('sim_nav_obstacle_contract.py')
    obstacle = module.SCENARIOS['detour']['obstacle']
    safe = [(-2.0, 0.80), (-1.0, 0.80), (0.0, 0.80)]
    unsafe = [(-2.0, 0.0), (0.0, 0.0)]

    assert module.path_clears_obstacle(safe, obstacle, 0.340)
    assert not module.path_clears_obstacle(unsafe, obstacle, 0.340)
    assert not module.path_clears_obstacle(
        [(-2.0, 0.0), (float('nan'), 0.8)], obstacle, 0.340)


def test_event_sequence_and_blocked_outcomes_are_fail_closed():
    """Missing evidence and runner cancellation must never pass."""
    module = _load_module('sim_nav_obstacle_contract.py')
    valid_removal = [
        'global_blocking', 'local_blocking', 'post_mark_plan',
        'deactivate_ack',
        'new_scan', 'global_cleared', 'local_cleared',
        'post_clear_new_plan', 'succeeded']
    invalid_removal = [
        'global_blocking', 'deactivate_ack', 'new_scan', 'local_cleared',
        'succeeded']

    assert module.validate_removal_sequence(valid_removal) == []
    assert module.validate_removal_sequence(invalid_removal)
    assert module.classify_terminal(
        'full_block', 'aborted', 4.0, False) == 'BLOCKED'
    assert module.classify_terminal(
        'full_block', 'aborted', -0.1, False) == 'FAIL'
    assert module.classify_terminal(
        'full_block', 'cancelled', 4.0, True) == 'INVALID'
    assert module.classify_terminal(
        'detour', 'succeeded', 30.0, False) == 'PASS'


def test_removal_recovery_sequence_allows_independent_costmap_cadence():
    """Each costmap may publish both clear samples before the other map."""
    module = _load_module('sim_nav_obstacle_contract.py')
    events = [
        'global_blocking', 'local_blocking', 'post_mark_plan',
        'deactivate_ack', 'new_scan', 'passive_clear_window_expired',
        'global_clear_requested', 'local_clear_requested',
        'global_clear_responded', 'local_clear_responded',
        'post_recovery_global_clear_1',
        'post_recovery_global_clear_2',
        'global_cleared',
        'post_recovery_local_clear_1',
        'post_recovery_local_clear_2',
        'local_cleared', 'post_clear_new_plan',
        'succeeded']

    assert module.validate_removal_sequence(
        events, 'bounded_recovery') == []
    assert module.validate_removal_sequence(
        [event for event in events if event != 'local_clear_responded'],
        'bounded_recovery')

    invalid = list(events)
    invalid.remove('global_cleared')
    invalid.insert(invalid.index('post_recovery_global_clear_2'),
                   'global_cleared')
    assert module.validate_removal_sequence(
        invalid, 'bounded_recovery')


def test_passive_clear_requires_two_consecutive_updates_and_no_service():
    """One update is insufficient; two clear updates per map pass passively."""
    module = _load_scenario_module()
    observer = _clear_observer(module)

    message = _costmap_message()
    clear = _clear_statistics()
    observer._record_clear_observation('global', message, clear, clear)
    observer._record_clear_observation('local', message, clear, clear)
    assert not observer.global_cleared
    assert not observer.local_cleared
    observer._record_clear_observation('global', message, clear, clear)
    observer._record_clear_observation('local', message, clear, clear)

    assert observer.clear_mode == 'passive'
    assert observer.global_cleared and observer.local_cleared
    assert observer.clear_service_request_counts == {'global': 0, 'local': 0}
    assert len(observer.clear_observation_trace) == 4


def test_bounded_clear_requests_each_service_exactly_once():
    """A second recovery poll cannot issue duplicate clear requests."""
    module = _load_scenario_module()
    observer = _clear_observer(module)

    observer._request_bounded_clear_recovery()
    observer._request_bounded_clear_recovery()

    assert observer.clear_mode == 'bounded_recovery'
    assert observer.clear_service_request_counts == {'global': 1, 'local': 1}
    assert {name: client.calls for name, client in
            observer.clear_clients.items()} == {'global': 1, 'local': 1}


def test_recovery_epoch_revokes_partial_passive_clear_and_reemits_events():
    """Recovery cannot reuse a source-cleared event from the passive epoch."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    message = _costmap_message()
    clear = _clear_statistics()
    observer._record_clear_observation('local', message, clear, clear)
    observer._record_clear_observation('local', message, clear, clear)
    assert 'local_cleared' in [item['name'] for item in observer.events]
    observer.clear_epoch = 1
    observer.clear_completion_history = [{'epoch': 1, 'stale': True}]
    observer.post_clear_plan_history = [{'epoch': 1, 'stale': True}]
    observer.final_clear_completion_s = module.time.monotonic()
    observer.final_post_clear_plan_s = module.time.monotonic()

    observer._request_bounded_clear_recovery()

    assert observer.clear_observation_counts == {'global': 0, 'local': 0}
    assert observer.global_cleared is False
    assert observer.local_cleared is False
    assert observer.final_clear_completion_s is None
    assert observer.final_post_clear_plan_s is None
    assert observer.final_clear_samples == {}
    assert 'local_cleared' not in [item['name'] for item in observer.events]
    observer.recovery_responses_complete_s = module.time.monotonic()
    for source in ('global', 'global', 'local', 'local'):
        observer._record_clear_observation(source, message, clear, clear)

    names = [item['name'] for item in observer.events]
    assert names.index('local_cleared') > names.index('global_cleared')
    assert observer.clear_epoch == 2
    assert observer.clear_completion_history[0] == {
        'epoch': 1, 'stale': True}
    assert observer.clear_completion_history[-1]['epoch'] == 2


def test_bounded_clear_unavailable_service_fails_without_retry():
    """An unavailable standard service fails immediately and is not retried."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    observer.clear_clients['global'] = _ClearClient(available=False)

    observer._request_bounded_clear_recovery()
    observer._request_bounded_clear_recovery()

    assert observer.activation_error == 'global_clear_service_unavailable'
    assert observer.clear_service_request_counts == {'global': 0, 'local': 0}


def test_bounded_clear_false_response_fails_fast():
    """A completed service future without a response is an invalid recovery."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    observer.clear_clients['global'] = _ClearClient(
        future=_ClearFuture(result=None))
    observer._request_bounded_clear_recovery()

    observer.poll_clear_recovery()

    assert observer.activation_error == 'global_clear_service_false'


def test_bounded_clear_response_timeout_fails_without_retry():
    """Outstanding futures have one contract-derived response deadline."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    pending = _ClearFuture(done=False)
    observer.clear_clients = {
        'global': _ClearClient(future=pending),
        'local': _ClearClient(future=pending),
    }
    observer._request_bounded_clear_recovery()
    observer.recovery_started_s -= observer.contract[
        'clear_service_timeout_s'] + 0.01

    observer.poll_clear_recovery()

    assert observer.activation_error == 'clear_service_response_timeout'
    assert observer.clear_service_request_counts == {'global': 1, 'local': 1}


def test_recovery_clear_requires_two_consecutive_post_response_updates():
    """Blocking resets the consecutive counter after service responses."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    observer.clear_mode = 'bounded_recovery'
    observer.recovery_responses_complete_s = module.time.monotonic()

    message = _costmap_message()
    clear = _clear_statistics()
    blocking = _clear_statistics(blocking=True)
    observer._record_clear_observation('global', message, clear, clear)
    observer._record_clear_observation('local', message, clear, clear)
    observer._record_clear_observation(
        'global', message, clear, blocking)
    observer._record_clear_observation('local', message, clear, clear)

    assert observer.clear_observation_counts == {'global': 0, 'local': 2}
    assert observer.activation_error is None
    assert not observer.global_cleared
    assert observer.local_cleared
    assert observer.clear_observation_trace[2]['decision'] == 'reset'


def test_clear_observation_ignores_empty_roi_and_requires_padded_clear():
    """Empty or padded-blocking observations cannot advance clear evidence."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    message = _costmap_message()
    empty = _clear_statistics(sampled_cell_count=0)
    clear = _clear_statistics()
    blocking = _clear_statistics(blocking=True)

    observer._record_clear_observation('global', message, empty, empty)
    observer._record_clear_observation(
        'global', message, clear, blocking)

    assert observer.clear_observation_counts['global'] == 0
    assert [item['decision'] for item in observer.clear_observation_trace] == [
        'ignored', 'reset']


def test_local_roi_eviction_before_two_clear_updates_fails():
    """A rolling local window cannot turn disappearance into clear evidence."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    message = _costmap_message()
    clear = _clear_statistics()
    empty = _clear_statistics(sampled_cell_count=0)

    observer._record_clear_observation('local', message, clear, clear)
    observer._record_clear_observation('local', message, empty, empty)

    assert observer.activation_error == 'local_roi_evicted_before_clear'
    assert observer.clear_observation_counts['local'] == 0
    assert observer.clear_observation_trace[-1]['decision'] == 'evicted'


def test_nonobservable_global_sample_breaks_clear_consecutiveness():
    """An empty global sample after clear1 must reset its counter."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    message = _costmap_message()
    clear = _clear_statistics()
    empty = _clear_statistics(sampled_cell_count=0)

    observer._record_clear_observation('global', message, clear, clear)
    observer._record_clear_observation('global', message, empty, empty)

    assert observer.clear_observation_counts['global'] == 0
    assert observer.clear_observation_trace[-1]['decision'] == 'ignored'


def test_local_eviction_after_clear_and_plan_revokes_all_causal_evidence():
    """Passage-before local eviction remains invalid even after clear2."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    message = _costmap_message()
    clear = _clear_statistics()
    empty = _clear_statistics(sampled_cell_count=0)
    for source in ('global', 'local'):
        observer._record_clear_observation(source, message, clear, clear)
        observer._record_clear_observation(source, message, clear, clear)
    observer.final_post_clear_plan_s = module.time.monotonic()

    observer._record_clear_observation('local', message, empty, empty)

    assert observer.activation_error == 'local_roi_evicted_before_clear'
    assert observer.clear_observation_counts['local'] == 0
    assert observer.local_cleared is False
    assert observer.final_clear_completion_s is None
    assert observer.final_post_clear_plan_s is None


def test_clear_reset_then_two_clear_updates_passes():
    """A transient 253 resets without ending the bounded window."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    message = _costmap_message()
    clear = _clear_statistics()
    blocking = _clear_statistics(blocking=True)

    for source in ('global', 'local'):
        observer._record_clear_observation(source, message, clear, clear)
    observer._record_clear_observation(
        'global', message, clear, blocking)
    for source in ('global', 'global', 'local'):
        observer._record_clear_observation(source, message, clear, clear)

    assert observer.clear_mode == 'passive'
    assert observer.global_cleared and observer.local_cleared


def test_persistent_blocking_reaches_bounded_recovery_timeout():
    """Persistent padded blocking never becomes clear and times out."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    observer.clear_mode = 'bounded_recovery'
    observer.recovery_responses_complete_s = (
        module.time.monotonic()
        - observer.contract['post_recovery_clear_window_s'] - 0.01)
    observer._record_clear_observation(
        'global', _costmap_message(), _clear_statistics(),
        _clear_statistics(blocking=True))

    observer.poll_clear_recovery()

    assert observer.activation_error == 'post_recovery_clear_timeout'


def test_clear_reset_revokes_completion_and_post_clear_plan():
    """Residual excess invalidates an earlier clear epoch and its plan."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    message = _costmap_message()
    clear = _clear_statistics()
    blocking = _clear_statistics(blocking=True)
    for source in ('global', 'global', 'local', 'local'):
        observer._record_clear_observation(source, message, clear, clear)
    observer.final_post_clear_plan_s = module.time.monotonic()

    observer._record_clear_observation(
        'global', message, clear, blocking)

    assert not observer.global_cleared
    assert observer.local_cleared
    assert observer.final_clear_completion_s is None
    assert observer.final_post_clear_plan_s is None


def test_global_baseline_requires_two_stable_nonunknown_snapshots():
    """A fixed reference grid becomes valid after two matching stamps."""
    module = _load_scenario_module()
    observer = module.ScenarioObserver.__new__(module.ScenarioObserver)
    observer.reference_costmap_probe = {
        'x_m': 0.05, 'y_m': 0.05, 'length_m': 0.1, 'width_m': 0.1}
    observer.global_baseline_snapshots = []
    observer.global_baseline_ranks = {}
    observer.global_baseline_geometry = None
    observer.baseline_ready = False
    observer.activation_error = None

    observer._capture_global_baseline(_grid_costmap([0, 253, 0, 0], 1))
    observer._capture_global_baseline(_grid_costmap([0, 253, 0, 0], 2))

    assert observer.baseline_ready
    assert len(observer.global_baseline_snapshots) == 2
    assert set(observer.global_baseline_ranks.values()) == {0, 1}


def test_global_baseline_mismatch_is_invalid():
    """Changing canonical classes cannot be accepted as a fixed baseline."""
    module = _load_scenario_module()
    observer = module.ScenarioObserver.__new__(module.ScenarioObserver)
    observer.reference_costmap_probe = {
        'x_m': 0.05, 'y_m': 0.05, 'length_m': 0.1, 'width_m': 0.1}
    observer.global_baseline_snapshots = []
    observer.global_baseline_ranks = {}
    observer.global_baseline_geometry = None
    observer.baseline_ready = False
    observer.activation_error = None

    observer._capture_global_baseline(_grid_costmap([0, 253, 0, 0], 1))
    observer._capture_global_baseline(_grid_costmap([0, 254, 0, 0], 2))

    assert observer.activation_error == 'baseline_unstable'
    assert not observer.baseline_ready


def test_evaluation_params_are_generated_without_editing_production(tmp_path):
    """Preparation must only create evaluation-local rewritten parameters."""
    module = _load_module('prepare_sim_nav_obstacle_run.py')
    production = (ROOT / 'jdamr_cube_navigation' / 'config'
                  / 'nav2_params.yaml')
    before = production.read_bytes()

    outputs = module.prepare(tmp_path)

    assert production.read_bytes() == before
    params = yaml.safe_load(outputs['params'].read_text())
    amcl = params['amcl']['ros__parameters']['initial_pose']
    assert (amcl['x'], amcl['y'], amcl['yaw']) == (-8.0, 0.0, 0.0)
    assert params['bt_navigator']['ros__parameters'][
        'default_nav_to_pose_bt_xml'].endswith(
            'navigate_to_pose_dynamic_obstacle_eval.xml')
    for name in ('local_costmap', 'global_costmap'):
        obstacle = params[name][name]['ros__parameters']['obstacle_layer']
        assert obstacle['scan']['observation_persistence'] == 0.0
        assert obstacle['scan']['inf_is_valid'] is True
        assert obstacle['scan']['obstacle_min_range'] == 0.0
        assert obstacle['scan']['raytrace_min_range'] == 0.0
        assert params[name][name]['ros__parameters'][
            'always_send_full_costmap'] is True
    contract = json.loads(outputs['contract'].read_text())
    assert contract['scenario_count'] == 5
    assert contract['run_count'] == 25
    contact_scope = contract['contact_scope']
    assert contact_scope['measured_geometry'] == (
        'full_evaluation_robot_collision_set_against_preloaded_obstacle')
    assert contact_scope['collision_geometry_preserved'] is True
    assert contact_scope['excluded_collision_elements'] == []
    assert contact_scope['arm_probe']['overlap_pose_world_m'] == [
        -7.80, 0.0, 0.30]
    contact_control = contract['preloaded_obstacles']['models'][
        'contact_control']
    assert [contact_control[key] for key in (
        'length_m', 'width_m', 'height_m')] == [0.14, 0.12, 0.40]
    assert contact_scope['arm_probe']['robot_relative_x_extent_m'] == [
        0.13, 0.27]
    no_return = contract['simulation_support_plane_correction'][
        'scan_preflight_contract']['real_scan_reference'][
            'no_return_observations']
    assert no_return['message_count'] == 7304
    assert no_return['sample_count'] == (
        no_return['finite_count'] + no_return['positive_infinity_count'])
    assert no_return['positive_infinity_fraction'] == pytest.approx(
        no_return['positive_infinity_count'] / no_return['sample_count'])
    scan_contract = contract['simulation_support_plane_correction'][
        'scan_preflight_contract']
    assert scan_contract['expected_wall_beams_m'] == {
        'south': pytest.approx(1.15),
        'west': pytest.approx(1.95),
        'north': pytest.approx(1.15),
    }
    assert scan_contract['lidar_noise_stddev_m'] == 0.01
    multiple = scan_contract['wall_beam_multiple_comparison']
    assert multiple['protocol_version'] == 2
    assert multiple['unit_of_analysis'] == 'direction_mean_bias'
    assert multiple['familywise_alpha'] == 0.01
    assert multiple['comparison_count'] == 3
    assert multiple['direction_count'] == 3
    assert multiple['replicate_count'] == 5
    assert multiple['replicate_seeds'] == [11, 23, 42, 67, 89]
    assert multiple['two_sided'] is True
    assert multiple['correction'] == 'bonferroni'
    assert multiple['noise_sigma_m'] == 0.01
    assert multiple['critical_z'] == pytest.approx(2.9351994688666982)
    assert multiple['standard_error_m'] == pytest.approx(
        0.01 / math.sqrt(5))
    assert multiple['wall_beam_mean_tolerance_m'] == pytest.approx(
        0.01312661107981443)


def test_rerun_archives_previous_attempt_with_hash_manifest(tmp_path):
    """A rerun must preserve an earlier failure before opening new logs."""
    module = _load_module('prepare_sim_nav_obstacle_run.py')
    run_dir = tmp_path / 'detour__seed_11'
    run_dir.mkdir()
    evidence = run_dir / 'evidence.json'
    evidence.write_text('{"status":"FAIL"}\n', encoding='utf-8')

    archived = module.archive_existing_attempt(run_dir)

    assert archived == (tmp_path / 'development_audit'
                        / 'detour__seed_11_attempt_1')
    assert not run_dir.exists()
    manifest = json.loads(
        (archived / 'attempt_manifest.json').read_text(encoding='utf-8'))
    assert manifest['run_id'] == 'detour__seed_11'
    assert manifest['files'] == [{
        'path': 'evidence.json',
        'size_bytes': (archived / 'evidence.json').stat().st_size,
        'sha256': module._sha256(archived / 'evidence.json'),
    }]


def test_scenario_controller_does_not_publish_velocity_commands():
    """The scenario process may observe but must never command velocity."""
    source = (ROOT / 'jdamr_cube_navigation' / 'jdamr_cube_navigation'
              / 'sim_nav_obstacle_scenario.py').read_text(encoding='utf-8')

    assert 'create_publisher(Twist' not in source
    assert 'create_publisher(TwistStamped' not in source
    assert "create_subscription(Twist, '/cmd_vel'" in source
    assert 'ros_gz_sim create' not in source
    assert '/world/slam_corridor/remove' not in source
    assert '/world/slam_corridor/set_pose' in source
    quote = chr(39)
    expected_padding_source = (
        'self.obstacle,\n                self.contract[' + quote
        + 'costmap_resolution_m' + quote + '])')
    assert expected_padding_source in source


def test_costmap_roi_bounds_stay_inside_a_rolling_window():
    """An obstacle outside a rolling costmap must produce an empty ROI."""
    module = _load_module('../jdamr_cube_navigation/costmap_evidence.py')
    point = type('Point', (), {'x': 5.85, 'y': -1.45})()
    origin = type('Origin', (), {'position': point})()
    metadata = type('Metadata', (), {
        'resolution': 0.05, 'size_x': 60, 'size_y': 60,
        'origin': origin})()
    costmap = type('Costmap', (), {
        'metadata': metadata, 'data': [0] * (60 * 60)})()

    statistics = module.roi_statistics(
        costmap, {'x_m': -1.0, 'y_m': 0.0,
                  'length_m': 0.5, 'width_m': 0.4})

    assert statistics['index_bounds'] == [0, 0, 25, 33]
    assert statistics['max_cost'] == -1
    assert statistics['lethal'] is False


@pytest.mark.parametrize(('cost', 'expected'), (
    (0, {
        'blocking': False, 'blocking_count': 0, 'lethal_count': 0,
        'inscribed_count': 0, 'unknown_count': 0}),
    (253, {
        'blocking': True, 'blocking_count': 1, 'lethal_count': 0,
        'inscribed_count': 1, 'unknown_count': 0}),
    (254, {
        'blocking': True, 'blocking_count': 1, 'lethal_count': 1,
        'inscribed_count': 0, 'unknown_count': 0}),
    (255, {
        'blocking': False, 'blocking_count': 0, 'lethal_count': 0,
        'inscribed_count': 0, 'unknown_count': 1}),
))
def test_costmap_roi_uses_nav2_blocking_constants(cost, expected):
    """Only INSCRIBED=253 and LETHAL=254 are blocking; UNKNOWN=255 is not."""
    module = _load_module('../jdamr_cube_navigation/costmap_evidence.py')
    point = type('Point', (), {'x': 0.0, 'y': 0.0})()
    origin = type('Origin', (), {'position': point})()
    metadata = type('Metadata', (), {
        'resolution': 0.05, 'size_x': 1, 'size_y': 1,
        'origin': origin})()
    costmap = type('Costmap', (), {
        'metadata': metadata, 'data': [cost]})()

    statistics = module.roi_statistics(
        costmap, {'x_m': 0.025, 'y_m': 0.025,
                  'length_m': 0.05, 'width_m': 0.05})

    for key, value in expected.items():
        assert statistics[key] == value
    if cost in {253, 254}:
        assert statistics['blocking_cells'] == [{
            'index_x': 0, 'index_y': 0,
            'center_x_m': 0.025, 'center_y_m': 0.025,
            'cost': cost}]
    else:
        assert statistics['blocking_cells'] == []


@pytest.mark.parametrize(('baseline_rank', 'observed_cost', 'expected'), (
    (1, 253, False),
    (1, 254, True),
    (0, 253, True),
    (None, 253, True),
))
def test_baseline_delta_uses_canonical_blocking_rank(
        baseline_rank, observed_cost, expected):
    """Only stronger or unmatched blocking cells remain after subtraction."""
    module = _load_module('../jdamr_cube_navigation/costmap_evidence.py')
    observed = [{
        'index_x': 0, 'index_y': 0,
        'center_x_m': 0.025, 'center_y_m': 0.025,
        'cost': observed_cost}]
    baseline = ({} if baseline_rank is None else {(0, 0): baseline_rank})

    excess = module.excess_blocking_cells(
        observed, baseline, 0.0, 0.0, 0.05)

    assert bool(excess) is expected


def test_transformed_local_cells_are_selected_in_fixed_map_roi():
    """Local cell selection applies map-from-odom before ROI filtering."""
    module = _load_module('../jdamr_cube_navigation/costmap_evidence.py')
    point = type('Point', (), {'x': 6.0, 'y': -0.05})()
    origin = type('Origin', (), {'position': point})()
    metadata = type('Metadata', (), {
        'resolution': 0.05, 'size_x': 2, 'size_y': 2,
        'origin': origin})()
    costmap = type('Costmap', (), {
        'metadata': metadata, 'data': [253, 0, 0, 0]})()
    obstacle = {'x_m': -1.975, 'y_m': -0.025,
                'length_m': 0.05, 'width_m': 0.05}

    samples = module.transformed_roi_cell_samples(
        costmap, obstacle, lambda x_m, y_m: (x_m - 8.0, y_m))

    assert len(samples['cells']) == 1
    assert samples['cells'][0]['map_center_x_m'] == pytest.approx(-1.975)
    assert samples['cells'][0]['cost'] == 253


def test_world_obstacle_is_transformed_into_odom_frame():
    """Local-costmap evidence must use odom rather than world coordinates."""
    module = _load_module('../jdamr_cube_navigation/costmap_evidence.py')
    obstacle = {
        'x_m': -6.5, 'y_m': 0.0, 'length_m': 0.5, 'width_m': 0.4}

    transformed = module.transform_world_obstacle_to_odom(
        obstacle, (-8.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    assert transformed == {
        'x_m': pytest.approx(1.5), 'y_m': pytest.approx(0.0),
        'length_m': 0.5, 'width_m': 0.4}


@pytest.mark.parametrize(('beam_index', 'center_x_m', 'center_y_m'), (
    (0, -6.5, 0.0),
    (180, -9.5, 0.0),
    (90, -8.0, 1.5),
))
def test_surface_beam_projection_handles_laser_yaw_and_seam(
        beam_index, center_x_m, center_y_m):
    """Projection maps the scan seam and cardinal beams into base axes."""
    module = _load_module('../jdamr_cube_navigation/costmap_evidence.py')
    ranges = [math.inf] * 360
    ranges[beam_index] = 1.25
    obstacle = {
        'x_m': center_x_m, 'y_m': center_y_m,
        'length_m': 0.5, 'width_m': 0.4}

    count = module.count_obstacle_surface_beams(
        ranges, -math.pi, 2.0 * math.pi / 360,
        (-8.0, 0.0, 0.0), math.pi, obstacle, 0.05)
    endpoints = module.obstacle_surface_endpoints(
        ranges, -math.pi, 2.0 * math.pi / 360,
        (-8.0, 0.0, 0.0), math.pi, obstacle, 0.05)

    assert count == 1
    assert len(endpoints) == 1
    assert endpoints[0][0] == 1.25
    assert abs(endpoints[0][1] - center_x_m) <= 0.25
    assert abs(endpoints[0][2] - center_y_m) <= 0.25


def test_blocking_probe_covers_robot_radius_without_reaching_walls():
    """The probe covers inflation drift while remaining in corridor space."""
    module = _load_module('../jdamr_cube_navigation/costmap_evidence.py')
    endpoints = [
        (2.4, 5.14, -0.18), (2.4, 5.16, 0.18),
        (2.4, 5.15, 0.40)]

    probe = module.blocking_probe_from_endpoints(
        endpoints, obstacle_center_y_m=0.0, core_half_width_m=0.2,
        robot_circumscribed_radius_m=math.hypot(0.23, 0.20))

    assert probe['x_m'] == pytest.approx(5.15)
    assert probe['length_m'] >= 2.0 * math.hypot(0.23, 0.20)
    assert probe['width_m'] == pytest.approx(
        0.36 + 2.0 * math.hypot(0.23, 0.20))
    assert probe['y_m'] + probe['width_m'] / 2.0 < 1.15


def test_clear_probe_uses_resolution_padding_not_robot_inflation_band():
    """Clear evidence samples the former surface, not the moving robot band."""
    module = _load_module('../jdamr_cube_navigation/costmap_evidence.py')
    endpoints = [(2.4, -1.26, -0.18), (2.4, -1.24, 0.18)]

    mark_probe = module.blocking_probe_from_endpoints(
        endpoints, 0.0, 0.2, math.hypot(0.23, 0.20))
    clear_probe = module.blocking_probe_from_endpoints(
        endpoints, 0.0, 0.2, 0.05)

    assert clear_probe['x_m'] == mark_probe['x_m']
    assert clear_probe['y_m'] == mark_probe['y_m']
    assert clear_probe['length_m'] < mark_probe['length_m']
    assert clear_probe['width_m'] < mark_probe['width_m']
    for axis, extent in (('x_m', 'length_m'), ('y_m', 'width_m')):
        assert (mark_probe[axis] - mark_probe[extent] / 2.0
                <= clear_probe[axis] - clear_probe[extent] / 2.0)
        assert (clear_probe[axis] + clear_probe[extent] / 2.0
                <= mark_probe[axis] + mark_probe[extent] / 2.0)


def test_obstacle_window_preserves_positive_infinity_evidence():
    """A parked front target leaves positive-infinity beams in its window."""
    module = _load_module('../jdamr_cube_navigation/costmap_evidence.py')
    ranges = [1.0] * 360
    for index in list(range(350, 360)) + list(range(0, 11)):
        ranges[index] = math.inf
    obstacle = {
        'x_m': -6.5, 'y_m': 0.0,
        'length_m': 0.5, 'width_m': 0.4}

    evidence = module.obstacle_angular_window_samples(
        ranges, -math.pi, 2.0 * math.pi / 360,
        (-8.0, 0.0, 0.0), math.pi, obstacle)

    assert evidence['beam_count'] > 0
    assert evidence['positive_infinity_count'] == evidence['beam_count']
    assert evidence['finite_count'] == 0


def test_pose_info_parser_preserves_preloaded_entity_identity():
    """Pose_V parsing must retain the model ID and explicit zero axes."""
    module = _load_module(
        '../jdamr_cube_navigation/sim_nav_obstacle_scenario.py')
    output = json.dumps({'pose': [{
        'name': 'g003_preloaded_contact_control',
        'id': 65,
        'position': {'y': 45.0, 'z': 0.2},
    }]})

    state = module.parse_entity_pose_info(
        output, 'g003_preloaded_contact_control')

    assert state == {'entity_id': 65, 'pose_m': [0.0, 45.0, 0.2]}


def test_pose_info_parser_accepts_concatenated_samples_and_uses_latest():
    """Gazebo may emit adjacent Pose_V documents despite ``-n 1``."""
    module = _load_module(
        '../jdamr_cube_navigation/sim_nav_obstacle_scenario.py')
    output = '\n'.join(json.dumps({'pose': [{
        'name': 'g003_preloaded_route',
        'id': 61,
        'position': {'x': x_m, 'y': 0.0, 'z': 0.2},
    }]}) for x_m in (40.0, -1.0))

    state = module.parse_entity_pose_info(
        output, 'g003_preloaded_route')

    assert state == {'entity_id': 61, 'pose_m': [-1.0, 0.0, 0.2]}


def test_pose_info_parser_rejects_identity_change_between_samples():
    """Adjacent samples cannot silently switch the tracked entity ID."""
    module = _load_module(
        '../jdamr_cube_navigation/sim_nav_obstacle_scenario.py')
    output = ''.join(json.dumps({'pose': [{
        'name': 'g003_preloaded_route',
        'id': entity_id,
        'position': {'x': 40.0},
    }]}) for entity_id in (61, 62))

    with pytest.raises(ValueError, match='entity ID changed'):
        module.parse_entity_pose_info(output, 'g003_preloaded_route')


def test_preflight_local_costmap_uses_exact_frame_transform():
    """Preflight local ROI cannot be sampled as though odom were map."""
    module = _load_scenario_module()
    observer = module.ScenarioObserver.__new__(module.ScenarioObserver)
    observer.obstacle = {'x_m': -6.5}
    observer.args = type(
        'Args', (), {'scenario': 'front_observation_probe'})()
    observer.activated = True
    observer.costmap_message_counts = {'global': 0, 'local': 0}
    observer.pending_local_costmaps = module.deque()
    observer.contract = {'pending_local_costmap_limit': 3}

    def transform(x_m, y_m):
        return x_m - 8.0, y_m
    observer._map_from_costmap_transform = lambda message: (transform, 17)
    received = []
    observer._process_costmap = lambda *args: received.append(args)
    message = object()

    observer._costmap('local', message)

    assert received == [('local', message, transform, 17)]


def _pending_costmap_observer(module, scenario):
    observer = module.ScenarioObserver.__new__(module.ScenarioObserver)
    observer.obstacle = {'x_m': -1.0}
    observer.args = type('Args', (), {'scenario': scenario})()
    observer.activated = True
    observer.start_s = 0.0
    observer.costmap_message_counts = {'global': 0, 'local': 0}
    observer.pending_local_costmaps = module.deque()
    observer.pending_local_costmap_failures = []
    observer.activation_error = None
    observer.contract = {
        'pending_local_costmap_limit': 2,
        'costmap_tf_timeout_s': 3.0,
    }
    observer._process_costmap = lambda *args: None
    return observer


def test_detour_drops_stale_snapshot_then_processes_valid_exact_tf(
        monkeypatch):
    """One stale detour snapshot cannot discard a later exact transform."""
    module = _load_scenario_module()
    observer = _pending_costmap_observer(module, 'detour')
    stale = _costmap_message()
    valid = _costmap_message()
    observer.pending_local_costmaps.extend(((0.0, stale), (9.0, valid)))

    def transform(x_m, y_m):
        return x_m, y_m
    observer._map_from_costmap_transform = lambda message: (transform, 17)
    processed = []
    observer._process_costmap = lambda *args: processed.append(args)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.0)

    observer._poll_pending_local_costmaps()

    assert observer.activation_error is None
    assert processed == [('local', valid, transform, 17)]
    assert observer.pending_local_costmap_failures[0]['reason'] == (
        'exact_stamp_tf_snapshot_expired_drop')


@pytest.mark.parametrize('scenario', [
    'detour', 'full_block', 'goal_occupied', 'front_observation_probe'])
def test_nonremoval_exact_tf_expiry_is_diagnostic(scenario, monkeypatch):
    """Non-removal snapshots expire individually without ending the run."""
    module = _load_scenario_module()
    observer = _pending_costmap_observer(module, scenario)
    observer.pending_local_costmaps.append((0.0, _costmap_message()))
    observer._map_from_costmap_transform = lambda message: (False, None)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.0)

    observer._poll_pending_local_costmaps()

    assert observer.activation_error is None
    assert not observer.pending_local_costmaps
    assert len(observer.pending_local_costmap_failures) == 1


def test_removal_exact_tf_expiry_and_overflow_remain_fatal(monkeypatch):
    """Removal canonical delta evidence rejects timeout and overflow."""
    module = _load_scenario_module()
    timeout_observer = _pending_costmap_observer(
        module, 'event_driven_removal')
    timeout_observer.pending_local_costmaps.append(
        (0.0, _costmap_message()))
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.0)
    timeout_observer._poll_pending_local_costmaps()
    assert timeout_observer.activation_error == 'costmap_exact_tf_timeout'

    overflow_observer = _pending_costmap_observer(
        module, 'event_driven_removal')
    overflow_observer.contract['pending_local_costmap_limit'] = 1
    overflow_observer.pending_local_costmaps.append(
        (9.0, _costmap_message()))
    overflow_observer._map_from_costmap_transform = (
        lambda message: (False, None))
    overflow_observer._costmap('local', _costmap_message())
    assert overflow_observer.activation_error == (
        'pending_local_costmap_overflow')


def test_baseline_inactive_local_costmap_never_enters_tf_queue():
    """Baseline with no obstacle returns before any local TF lookup."""
    module = _load_scenario_module()
    observer = _pending_costmap_observer(module, 'baseline')
    observer.obstacle = None
    observer.activated = True
    observer._map_from_costmap_transform = (
        lambda message: pytest.fail('baseline must not request local TF'))

    observer._costmap('local', _costmap_message())

    assert not observer.pending_local_costmaps


def _stamp_mismatch_transform(module):
    stamp = type('Stamp', (), {'sec': 1, 'nanosec': 3})()
    header = type('Header', (), {'stamp': stamp})()
    return type('Transform', (), {'header': header})()


@pytest.mark.parametrize('scenario', [
    'detour', 'front_observation_probe', 'full_block', 'goal_occupied'])
def test_nonremoval_exact_tf_stamp_mismatch_drops_snapshot(
        scenario, monkeypatch):
    """A non-removal mismatch is diagnostic and waits for later exact TF."""
    module = _load_scenario_module()
    observer = _pending_costmap_observer(module, scenario)
    message = _costmap_message()
    message.header.frame_id = 'odom'
    transform = _stamp_mismatch_transform(module)
    monkeypatch.setattr(module, 'Time', type('Time', (), {
        'from_msg': staticmethod(lambda message: object())}))
    observer.tf_buffer = type('Buffer', (), {
        'lookup_transform': lambda *args, **kwargs: transform})()
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.0)

    observer._costmap('local', message)
    assert observer.activation_error is None
    assert len(observer.pending_local_costmaps) == 1
    observer.pending_local_costmaps[0] = (0.0, message)
    observer._poll_pending_local_costmaps()

    assert observer.activation_error is None
    assert not observer.pending_local_costmaps
    assert observer.pending_local_costmap_failures[-1]['reason'] == (
        'exact_stamp_tf_snapshot_expired_drop')


def test_removal_exact_tf_stamp_mismatch_is_fatal(monkeypatch):
    """Removal canonical comparison rejects a mismatched TF stamp."""
    module = _load_scenario_module()
    observer = _pending_costmap_observer(module, 'event_driven_removal')
    message = _costmap_message()
    message.header.frame_id = 'odom'
    transform = _stamp_mismatch_transform(module)
    monkeypatch.setattr(module, 'Time', type('Time', (), {
        'from_msg': staticmethod(lambda message: object())}))
    observer.tf_buffer = type('Buffer', (), {
        'lookup_transform': lambda *args, **kwargs: transform})()

    assert observer._map_from_costmap_transform(message)[0] is False
    assert observer.activation_error == 'costmap_tf_stamp_mismatch'


def _wall_beam_gate_fixture(module, direction_count=3, replicate_count=5):
    generator = _load_module('generate_sim_nav_obstacle_assets.py')
    return {
        'expected_wall_beams_m': {
            'south': 1.15, 'west': 1.95, 'north': 1.15},
        'wall_beam_multiple_comparison':
            generator.derive_wall_beam_multiple_comparison_contract(
                0.01, direction_count, (11, 23, 42, 67, 89)[
                    :replicate_count]),
    }


def _wall_beam_evaluations(error_m=0.0):
    return [{'profile': {'wall_beams_m': {
        'south': 1.15 + error_m,
        'west': 1.95 + error_m,
        'north': 1.15 + error_m,
    }}} for _ in range(5)]


def test_wall_beam_mean_gate_derives_fwer_and_standard_error():
    """The direction-mean threshold is derived from m and replicate n."""
    module = _load_module('generate_sim_nav_obstacle_assets.py')
    gate_3x5 = module.derive_wall_beam_multiple_comparison_contract(
        0.01, 3, (11, 23, 42, 67, 89))
    gate_6x5 = module.derive_wall_beam_multiple_comparison_contract(
        0.01, 6, (11, 23, 42, 67, 89))
    gate_3x10 = module.derive_wall_beam_multiple_comparison_contract(
        0.01, 3, tuple(range(10)))

    assert gate_3x5['critical_z'] == pytest.approx(2.9351994688666982)
    assert gate_3x5['standard_error_m'] == pytest.approx(0.01 / math.sqrt(5))
    assert gate_3x5['wall_beam_mean_tolerance_m'] == pytest.approx(
        gate_3x5['critical_z'] * gate_3x5['standard_error_m'])
    assert gate_6x5['critical_z'] > gate_3x5['critical_z']
    assert gate_3x10['wall_beam_mean_tolerance_m'] < (
        gate_3x5['wall_beam_mean_tolerance_m'])


def test_wall_beam_gate_uses_closed_derived_boundary():
    """The exact Bonferroni boundary passes and the next float fails."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    gate = _wall_beam_gate_fixture(module)
    tolerance_m = gate['wall_beam_multiple_comparison'][
        'wall_beam_mean_tolerance_m']
    boundary = _wall_beam_evaluations()
    outside = _wall_beam_evaluations()
    for evaluation in boundary:
        for direction, expected_m in gate['expected_wall_beams_m'].items():
            evaluation['profile']['wall_beams_m'][direction] = math.nextafter(
                expected_m + tolerance_m, -math.inf)
    for evaluation in outside:
        for direction, expected_m in gate['expected_wall_beams_m'].items():
            evaluation['profile']['wall_beams_m'][direction] = math.nextafter(
                expected_m + tolerance_m, math.inf)

    assert module._wall_beam_comparison_evidence(
        boundary, (11, 23, 42, 67, 89), gate)['status'] == 'PASS'
    assert module._wall_beam_comparison_evidence(
        outside, (11, 23, 42, 67, 89), gate)['status'] == 'FAIL'


@pytest.mark.parametrize('bad_value', [None, math.nan, math.inf])
def test_wall_beam_gate_fails_closed_for_missing_or_nonfinite(bad_value):
    """Missing and non-finite wall observations never pass the gate."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    gate = _wall_beam_gate_fixture(module)
    evaluations = _wall_beam_evaluations()
    evaluations[2]['profile']['wall_beams_m']['north'] = bad_value

    evidence = module._wall_beam_comparison_evidence(
        evaluations, (11, 23, 42, 67, 89), gate)

    assert evidence['status'] == 'FAIL'
    assert evidence['complete'] is False


def test_wall_beam_gate_rejects_persistent_fourteen_mm_bias():
    """A persistent 14 mm direction bias exceeds the mean-bias limit."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    gate = _wall_beam_gate_fixture(module)

    rejected = module._wall_beam_comparison_evidence(
        _wall_beam_evaluations(0.014),
        (11, 23, 42, 67, 89), gate)

    assert rejected['status'] == 'FAIL'


def test_wall_beam_evidence_has_15_unique_records_and_maximum():
    """Evidence enumerates every seed-direction comparison exactly once."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    gate = _wall_beam_gate_fixture(module)
    evidence = module._wall_beam_comparison_evidence(
        _wall_beam_evaluations(0.02),
        (11, 23, 42, 67, 89), gate)

    identities = {(item['seed'], item['direction'])
                  for item in evidence['records']}
    assert len(evidence['records']) == len(identities) == 15
    assert evidence['max_absolute_error_m'] == pytest.approx(0.02)
    assert evidence['raw_comparison_count'] == 15
    assert len(evidence['direction_records']) == 3
    assert evidence['per_comparison_alpha'] == pytest.approx(0.01 / 3)


@pytest.mark.parametrize('seeds', [
    (11, 23, 42, 67),
    (11, 23, 42, 67, 67),
    (11, 23, 42, 67, 90),
])
def test_wall_beam_gate_rejects_missing_or_duplicate_seed(seeds):
    """The five fixed, unique replicate seeds are mandatory."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    gate = _wall_beam_gate_fixture(module)
    evaluations = _wall_beam_evaluations()[:len(seeds)]

    evidence = module._wall_beam_comparison_evidence(
        evaluations, seeds, gate)

    assert evidence['status'] == 'FAIL'
    assert not evidence['contract_valid'] or not evidence['complete']


def test_wall_beam_gate_rejects_missing_direction():
    """Every seed must include all three preregistered directions."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    gate = _wall_beam_gate_fixture(module)
    evaluations = _wall_beam_evaluations()
    del evaluations[3]['profile']['wall_beams_m']['west']

    evidence = module._wall_beam_comparison_evidence(
        evaluations, (11, 23, 42, 67, 89), gate)

    assert evidence['status'] == 'FAIL'
    assert evidence['complete'] is False


def test_wall_beam_gate_rejects_tampered_derived_contract():
    """Stored critical values must equal a fresh contract recalculation."""
    module = _load_module('run_sim_nav_obstacle_eval.py')
    gate = _wall_beam_gate_fixture(module)
    gate['wall_beam_multiple_comparison']['critical_z'] = 3.5

    evidence = module._wall_beam_comparison_evidence(
        _wall_beam_evaluations(), (11, 23, 42, 67, 89), gate)

    assert evidence['contract_valid'] is False
    assert evidence['status'] == 'FAIL'


def test_removal_passage_is_strict_and_symmetric():
    """Passage excludes equality and mirrors for a negative-x route."""
    module = _load_module('sim_nav_obstacle_contract.py')
    positive = module.derive_removal_passage_contract(
        {'x_m': -8.0, 'y_m': 0.0}, {'x_m': 6.0, 'y_m': 0.0},
        {'x_m': -1.0, 'y_m': 0.0, 'length_m': 0.5}, 0.05, 0.3048)
    threshold = positive['strict_robot_center_threshold_x_m']
    assert not module.removal_passage_crossed(threshold, positive)
    assert module.removal_passage_crossed(
        math.nextafter(threshold, math.inf), positive)

    negative = module.derive_removal_passage_contract(
        {'x_m': 8.0, 'y_m': 0.0}, {'x_m': -6.0, 'y_m': 0.0},
        {'x_m': 1.0, 'y_m': 0.0, 'length_m': 0.5}, 0.05, 0.3048)
    negative_threshold = negative['strict_robot_center_threshold_x_m']
    assert not module.removal_passage_crossed(
        negative_threshold, negative)
    assert module.removal_passage_crossed(
        math.nextafter(negative_threshold, -math.inf), negative)


def test_removal_passage_rejects_misaligned_route():
    """The generated far-side proof requires one aligned route axis."""
    module = _load_module('sim_nav_obstacle_contract.py')

    with pytest.raises(ValueError, match='y-aligned'):
        module.derive_removal_passage_contract(
            {'x_m': -8.0, 'y_m': 0.0}, {'x_m': 6.0, 'y_m': 0.1},
            {'x_m': -1.0, 'y_m': 0.0, 'length_m': 0.5}, 0.05, 0.3048)


def test_background_excess_is_diagnostic_and_does_not_revoke_clear():
    """Static fringe outside the influence ROI cannot reset a clear pair."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    message = _costmap_message()
    clear = _clear_statistics()
    background = _clear_statistics()
    background['baseline_excess_count'] = 1

    observer._record_clear_observation('global', message, clear, clear)
    observer._record_clear_observation('global', message, clear, clear)
    observer._record_clear_observation(
        'global', message, clear, background)

    assert observer.global_cleared is True
    assert observer.clear_observation_counts['global'] == 2
    assert observer.clear_observation_trace[-1]['decision'] == (
        'diagnostic_background_excess')


def test_background_excess_does_not_block_authoritative_clear_increment():
    """Static fringe cannot consume the bounded influence-clear window."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    message = _costmap_message()
    clear = _clear_statistics()
    background = _clear_statistics()
    background['baseline_excess_count'] = 1

    observer._record_clear_observation(
        'global', message, clear, background)
    observer._record_clear_observation(
        'global', message, clear, background)

    assert observer.clear_observation_counts['global'] == 2
    assert observer.global_cleared is True
    assert observer.clear_observation_trace[-1]['decision'] == (
        'diagnostic_background_excess')


def test_influence_excess_revokes_clear_before_passage():
    """Residual blocking inside the physical influence ROI resets evidence."""
    module = _load_scenario_module()
    observer = _clear_observer(module)
    message = _costmap_message()
    clear = _clear_statistics()
    residual = _clear_statistics(blocking=True)

    observer._record_clear_observation('global', message, clear, clear)
    observer._record_clear_observation('global', message, clear, clear)
    observer._record_clear_observation(
        'global', message, clear, residual)

    assert observer.global_cleared is False
    assert observer.clear_observation_counts['global'] == 0
    assert observer.clear_observation_trace[-1]['decision'] == 'reset'


def test_evaluator_rejects_stale_removal_evidence(tmp_path):
    """Removal cannot pass without a post-deactivate scan and clear events."""
    module = _load_module('evaluate_sim_nav_obstacle.py')
    contract_module = _load_module('sim_nav_obstacle_contract.py')
    params = yaml.safe_load((ROOT / 'jdamr_cube_navigation' / 'config'
                            / 'nav2_params.yaml').read_text())
    contract = {
        **contract_module.derive_contract(params),
        'observation_persistence_s': 0.0,
    }
    document = {
        'run_id': 'event_driven_removal__seed_11',
        'scenario': 'event_driven_removal',
        'seed': 11,
        'contract': contract,
        'events': [
            {'name': name} for name in (
                'global_blocking', 'local_blocking', 'post_mark_plan',
                'deactivate_ack',
                'global_cleared', 'local_cleared',
                'post_clear_new_plan', 'succeeded')],
        'global_blocking': True,
        'local_blocking': True,
        'global_cleared': True,
        'local_cleared': True,
        'clear_mode': 'passive',
        'clear_service_request_counts': {'global': 0, 'local': 0},
        'clear_service_response_counts': {'global': 0, 'local': 0},
        'clear_observation_trace': _clear_trace_fixture(),
        'new_scan_after_deactivate': False,
        'plans_after_mark': [[[-2.0, 0.8], [-1.0, 0.8], [0.0, 0.8]]],
        'action_terminal': 'succeeded',
        'runner_cancelled': False,
        'runner_timed_out': False,
        'terminal_elapsed_from_mark_s': 4.0,
        'first_zero_from_mark_s': 0.1,
        'contact_count': 0,
        'final_cmd_vel_zero': True,
        'final_zero_hold_s': 2.0,
        'contact_smoke_status': 'PASS',
        'observation_persistence_s': 0.0,
        'teardown': {
            'identity_survivors': [], 'remaining_process_groups': []},
    }

    result = module.evaluate_run(document)

    assert result['status'] == 'FAIL'
    assert 'global_and_local_cleared' in result['failures']


def test_detour_does_not_require_a_stop_after_obstacle_marking():
    """A successful detour must keep moving rather than satisfy stop timing."""
    module = _load_module('evaluate_sim_nav_obstacle.py')
    contract_module = _load_module('sim_nav_obstacle_contract.py')
    params = yaml.safe_load((ROOT / 'jdamr_cube_navigation' / 'config'
                            / 'nav2_params.yaml').read_text())
    contact_scope = {
        'collision_geometry_preserved': True,
        'excluded_collision_elements': [],
    }
    contract = {
        **contract_module.derive_contract(params),
        'observation_persistence_s': 0.0,
        'contact_scope': contact_scope,
    }
    document = {
        'run_id': 'detour__seed_11', 'scenario': 'detour', 'seed': 11,
        'contract': contract, 'events': [
            {'name': name} for name in (
                'activate_ack', 'activation_surface_scan', 'goal_accepted',
                'pre_mark_plan', 'mark_eligible_scan', 'global_blocking',
                'local_blocking', 'new_plan', 'succeeded')],
        'global_blocking': True,
        'local_blocking': True,
        'entity_identity_stable': True,
        'activation_reference_scan_stamp_ns': 100,
        'activation_surface_scan': {
            'stamp_ns': 200, 'surface_beam_count': 3,
            'minimum_surface_range_m': 6.75},
        'mark_eligible_scan': {
            'stamp_ns': 300, 'surface_beam_count': 8,
            'minimum_surface_range_m': 2.4},
        'plans_before_mark': [[[-2.0, 0.0], [0.0, 0.0]]],
        'plans_after_mark': [[[-2.0, 0.8], [-1.0, 0.8], [0.0, 0.8]]],
        'action_terminal': 'succeeded', 'runner_cancelled': False,
        'runner_timed_out': False, 'terminal_elapsed_from_mark_s': 4.0,
        'first_zero_from_mark_s': None, 'contact_count': 0,
        'contact_matched_publisher_count_max': 1,
        'minimum_robot_obstacle_aabb_distance_m': 0.5,
        'contact_scope': contact_scope, 'final_cmd_vel_zero': True,
        'final_zero_hold_s': 2.0, 'contact_smoke_status': 'PASS',
        'observation_persistence_s': 0.0,
        'teardown': {
            'identity_survivors': [], 'remaining_process_groups': []},
    }

    result = module.evaluate_run(document)

    assert result['status'] == 'PASS'
    assert result['terminal_classification'] == 'PASS'


def _terminal_evidence(scenario, events):
    module = _load_module('sim_nav_obstacle_contract.py')
    params = yaml.safe_load((ROOT / 'jdamr_cube_navigation' / 'config'
                            / 'nav2_params.yaml').read_text())
    contact_scope = {
        'collision_geometry_preserved': True,
        'excluded_collision_elements': [],
    }
    return {
        'run_id': f'{scenario}__seed_11', 'scenario': scenario, 'seed': 11,
        'contract': {
            **module.derive_contract(params),
            'observation_persistence_s': 0.0,
            'contact_scope': contact_scope,
        },
        'events': [{'name': name} for name in events],
        'global_blocking': True, 'local_blocking': False,
        'entity_identity_stable': True,
        'activation_reference_scan_stamp_ns': 100,
        'activation_surface_scan': {
            'stamp_ns': 200, 'surface_beam_count': 3,
            'minimum_surface_range_m': 6.75},
        'mark_eligible_scan': {
            'stamp_ns': 300, 'surface_beam_count': 8,
            'minimum_surface_range_m': 2.4},
        'plans_before_mark': [[[-2.0, 0.0], [0.0, 0.0]]],
        'plans_after_mark': [],
        'action_terminal': 'aborted', 'runner_cancelled': False,
        'runner_timed_out': False, 'terminal_elapsed_from_mark_s': 3.5,
        'first_zero_from_mark_s': 0.1, 'contact_count': 0,
        'contact_matched_publisher_count_max': 1,
        'minimum_robot_obstacle_aabb_distance_m': 0.5,
        'contact_scope': contact_scope, 'final_cmd_vel_zero': True,
        'final_zero_hold_s': 2.0, 'contact_smoke_status': 'PASS',
        'observation_persistence_s': 0.0,
        'teardown': {
            'identity_survivors': [], 'remaining_process_groups': []},
    }


@pytest.mark.parametrize('scenario', ['full_block', 'goal_occupied'])
def test_blocked_scenario_requires_global_not_out_of_window_local_mark(
        scenario):
    """A global block must stop before the obstacle enters the local window."""
    evaluator = _load_module('evaluate_sim_nav_obstacle.py')
    document = _terminal_evidence(scenario, (
        'activate_ack', 'activation_surface_scan', 'goal_accepted',
        'pre_mark_plan', 'mark_eligible_scan', 'global_blocking',
        'aborted', 'final_zero'))
    if scenario == 'goal_occupied':
        document['plans_before_mark'] = [[[5.0, 0.0], [6.0, 0.0]]]

    result = evaluator.evaluate_run(document)

    assert result['status'] == 'PASS'
    assert result['terminal_classification'] == 'BLOCKED'
    assert result['checks']['global_blocking_after_eligible_scan'] is True


def test_detour_without_exact_local_blocking_fails():
    """Dropping stale local snapshots cannot turn global-only detour green."""
    evaluator = _load_module('evaluate_sim_nav_obstacle.py')
    document = _terminal_evidence('detour', (
        'activate_ack', 'activation_surface_scan', 'goal_accepted',
        'pre_mark_plan', 'mark_eligible_scan', 'global_blocking',
        'post_mark_plan', 'succeeded', 'final_zero'))
    document.update({
        'action_terminal': 'succeeded',
        'plans_after_mark': [[[-2.0, 0.8], [0.0, 0.8]]],
    })

    result = evaluator.evaluate_run(document)

    assert result['status'] == 'FAIL'
    assert 'global_and_local_blocking' in result['failures']


def test_removal_clears_without_requiring_a_sudden_stop():
    """G003 removal is a clearing test; sudden-stop timing belongs to G004."""
    evaluator = _load_module('evaluate_sim_nav_obstacle.py')
    document = _terminal_evidence('event_driven_removal', (
        'activate_ack', 'activation_surface_scan', 'goal_accepted',
        'pre_mark_plan', 'mark_eligible_scan', 'global_blocking',
        'local_blocking', 'post_mark_plan', 'deactivate_ack', 'new_scan',
        'global_cleared', 'local_cleared', 'post_clear_new_plan',
        'far_side_passage', 'succeeded', 'final_zero'))
    document.update({
        **_removal_epoch_fixture(),
        'local_blocking': True, 'global_cleared': True,
        'local_cleared': True, 'new_scan_after_deactivate': True,
        'clear_mode': 'passive',
        'clear_service_request_counts': {'global': 0, 'local': 0},
        'clear_service_response_counts': {'global': 0, 'local': 0},
        'clear_observation_trace': _clear_trace_fixture(),
        'plans_after_mark': [
            [[-2.0, 0.8], [-1.0, 0.8], [0.0, 0.8]],
            [[-2.0, 0.0], [0.0, 0.0]]],
        'action_terminal': 'succeeded',
        'terminal_elapsed_from_mark_s': 30.0,
        'first_zero_from_mark_s': 30.0,
        'minimum_robot_obstacle_aabb_distance_before_deactivate_m': 0.5,
    })

    result = evaluator.evaluate_run(document)

    assert result['status'] == 'PASS'
    document['clear_observation_trace'] = []
    result = evaluator.evaluate_run(document)
    assert result['status'] == 'FAIL'
    assert 'clear_observation_trace' in result['failures']


def test_removal_bounded_recovery_requires_exact_service_evidence():
    """Bounded recovery passes only with one response from both services."""
    evaluator = _load_module('evaluate_sim_nav_obstacle.py')
    document = _terminal_evidence('event_driven_removal', (
        'activate_ack', 'activation_surface_scan', 'goal_accepted',
        'pre_mark_plan', 'mark_eligible_scan', 'global_blocking',
        'local_blocking', 'post_mark_plan', 'deactivate_ack', 'new_scan',
        'passive_clear_window_expired', 'global_clear_requested',
        'local_clear_requested', 'global_clear_responded',
        'local_clear_responded', 'post_recovery_global_clear_1',
        'post_recovery_global_clear_2', 'post_recovery_local_clear_1',
        'post_recovery_local_clear_2', 'global_cleared', 'local_cleared',
        'post_clear_new_plan', 'far_side_passage', 'succeeded', 'final_zero'))
    document.update({
        **_removal_epoch_fixture(),
        'local_blocking': True, 'global_cleared': True,
        'local_cleared': True, 'new_scan_after_deactivate': True,
        'clear_mode': 'bounded_recovery',
        'clear_service_request_counts': {'global': 1, 'local': 1},
        'clear_service_response_counts': {'global': 1, 'local': 1},
        'clear_observation_trace': _clear_trace_fixture(),
        'plans_after_mark': [
            [[-2.0, 0.8], [-1.0, 0.8], [0.0, 0.8]],
            [[-2.0, 0.0], [0.0, 0.0]]],
        'action_terminal': 'succeeded',
        'terminal_elapsed_from_mark_s': 30.0,
        'first_zero_from_mark_s': 30.0,
        'minimum_robot_obstacle_aabb_distance_before_deactivate_m': 0.5,
    })

    assert evaluator.evaluate_run(document)['status'] == 'PASS'
    document['clear_service_response_counts']['local'] = 0
    result = evaluator.evaluate_run(document)
    assert result['status'] == 'FAIL'
    assert 'bounded_clear_policy' in result['failures']


def _write_aggregate_matrix(tmp_path, matrix):
    paths = []
    for item in matrix:
        path = tmp_path / f"{item['run_id']}.json"
        path.write_text(json.dumps(item))
        paths.append(path)
    return paths


def test_aggregate_rejects_all_baseline_identity_laundering(
        tmp_path, monkeypatch):
    """Run IDs cannot disguise 25 baseline documents as the matrix."""
    evaluator = _load_module('evaluate_sim_nav_obstacle.py')
    contract = _load_module('sim_nav_obstacle_contract.py')
    matrix = contract.scenario_matrix()
    documents = [
        {**item, 'scenario': 'baseline'} for item in matrix]
    monkeypatch.setattr(evaluator, 'evaluate_run', lambda document: {
        **document, 'status': 'PASS'})

    result = evaluator.aggregate(_write_aggregate_matrix(tmp_path, documents))

    assert result['status'] == 'FAIL'
    assert result['matrix_complete'] is False
    assert result['missing_identities']
    assert result['unexpected_identities']


def test_aggregate_rejects_duplicate_extra_and_filename_mismatch(
        tmp_path, monkeypatch):
    """Exact identity multiplicity and file naming are fail-closed."""
    evaluator = _load_module('evaluate_sim_nav_obstacle.py')
    contract = _load_module('sim_nav_obstacle_contract.py')
    matrix = contract.scenario_matrix()
    paths = _write_aggregate_matrix(tmp_path, matrix)
    duplicate = tmp_path / 'extra.json'
    duplicate.write_text(json.dumps(matrix[0]))
    paths.append(duplicate)
    monkeypatch.setattr(evaluator, 'evaluate_run', lambda document: {
        **document, 'status': 'PASS'})

    result = evaluator.aggregate(paths)

    assert result['status'] == 'FAIL'
    assert result['duplicate_identities'] == [(
        matrix[0]['run_id'], matrix[0]['scenario'], matrix[0]['seed'])]
    assert result['identity_errors'][0]['reason'] == (
        'filename_run_id_mismatch')


def test_aggregate_allows_named_preflight_but_rejects_unknown_extra(
        tmp_path, monkeypatch):
    """Only the contract's non-run preflight artifact is excluded."""
    evaluator = _load_module('evaluate_sim_nav_obstacle.py')
    contract = _load_module('sim_nav_obstacle_contract.py')
    paths = _write_aggregate_matrix(tmp_path, contract.scenario_matrix())
    preflight = tmp_path / 'scan_ab_preflight' / 'evidence.json'
    preflight.parent.mkdir()
    preflight.write_text(json.dumps({'status': 'PASS'}))
    monkeypatch.setattr(evaluator, 'evaluate_run', lambda document: {
        **document, 'status': 'PASS'})

    assert evaluator.aggregate([*paths, preflight])['status'] == 'PASS'

    unknown = tmp_path / 'unknown.json'
    unknown.write_text(json.dumps({'status': 'PASS'}))
    result = evaluator.aggregate([*paths, preflight, unknown])

    assert result['status'] == 'FAIL'
    assert result['identity_errors'][0]['reason'] == 'missing_identity'


def test_activation_error_is_a_hard_per_run_failure():
    """A semantic-looking terminal cannot hide an activation failure."""
    evaluator = _load_module('evaluate_sim_nav_obstacle.py')
    document = _terminal_evidence('detour', (
        'activate_ack', 'activation_surface_scan', 'goal_accepted',
        'pre_mark_plan', 'mark_eligible_scan', 'global_blocking',
        'local_blocking', 'succeeded', 'final_zero'))
    document.update({
        'local_blocking': True,
        'plans_after_mark': [[[-2.0, 0.8], [0.0, 0.8]]],
        'activation_error': 'costmap_exact_tf_timeout',
    })

    result = evaluator.evaluate_run(document)

    assert result['status'] == 'FAIL'
    assert 'activation_error_absent' in result['failures']


def _fake_cyclone_prefix(tmp_path):
    prefix = tmp_path / 'opt' / 'ros' / 'jazzy'
    package = prefix / 'share' / 'rmw_cyclonedds_cpp'
    package.mkdir(parents=True)
    (prefix / 'lib').mkdir()
    (prefix / 'lib' / 'librmw_cyclonedds_cpp.so').write_bytes(b'library')
    (package / 'package.xml').write_text(
        '<package><version>2.2.3</version></package>')
    return prefix


def test_evaluation_rmw_environment_is_scoped_to_children(tmp_path):
    """The selected RMW environment must affect evaluation children only."""
    runner = _load_module('run_sim_nav_obstacle_eval.py')
    prefix = _fake_cyclone_prefix(tmp_path)
    runner.configure_evaluation_rmw('rmw_cyclonedds_cpp', prefix)

    environment = runner._environment('rmw-test', 181)

    assert environment['RMW_IMPLEMENTATION'] == 'rmw_cyclonedds_cpp'
    assert environment['AMENT_PREFIX_PATH'].split(':')[0] == str(prefix)
    assert environment['LD_LIBRARY_PATH'].split(':')[0] == str(prefix / 'lib')
    assert 'FASTDDS_BUILTIN_TRANSPORTS' not in environment
    runner.configure_evaluation_rmw('rmw_fastrtps_cpp', None)
    fast_environment = runner._environment('rmw-test-fast', 182)
    assert fast_environment['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    assert fast_environment['FASTDDS_BUILTIN_TRANSPORTS'] == 'UDPv4'


def test_evaluation_rmw_configuration_fails_closed(tmp_path):
    """Incomplete or incompatible RMW configuration must be rejected."""
    runner = _load_module('run_sim_nav_obstacle_eval.py')

    with pytest.raises(ValueError, match='overlay prefix is incomplete'):
        runner.configure_evaluation_rmw('rmw_cyclonedds_cpp', tmp_path)
    with pytest.raises(ValueError, match='does not accept an overlay'):
        runner.configure_evaluation_rmw('rmw_fastrtps_cpp', tmp_path)
    with pytest.raises(ValueError, match='unsupported evaluation RMW'):
        runner.configure_evaluation_rmw('rmw_unknown', None)


def test_fastdds_load_node_timeout_is_immediate_infrastructure_invalid(
        tmp_path, monkeypatch):
    """The known FastDDS service race must stop readiness polling."""
    runner = _load_module('run_sim_nav_obstacle_eval.py')
    startup_log = tmp_path / 'navigation.log'
    startup_log.write_text(
        'component_container: '
        + runner.FASTDDS_LOAD_NODE_TIMEOUT_SIGNATURE)
    monkeypatch.setattr(
        runner.subprocess, 'run',
        lambda *args, **kwargs: pytest.fail('topic polling must not continue'))

    with pytest.raises(
            runner.InfrastructureInvalid,
            match='INFRA_INVALID_FASTDDS_SERVICE_DISCOVERY'):
        runner._wait_topics({'/plan'}, {}, 1.0, startup_log)


def test_actual_rmw_mismatch_fails_closed(tmp_path, monkeypatch):
    """Runtime RMW identity must match the requested implementation."""
    runner = _load_module('run_sim_nav_obstacle_eval.py')
    prefix = _fake_cyclone_prefix(tmp_path)
    runner.configure_evaluation_rmw('rmw_cyclonedds_cpp', prefix)

    result = type('Result', (), {
        'returncode': 0, 'stdout': 'rmw_fastrtps_cpp\n', 'stderr': ''})()
    monkeypatch.setattr(
        runner.subprocess, 'run', lambda *args, **kwargs: result)

    with pytest.raises(RuntimeError, match='evaluation RMW mismatch'):
        runner.evaluation_rmw_evidence(runner._environment('mismatch', 183))


def _trace_clear_sample(decision, before, after, *, blocking=False):
    return {
        'source': 'global', 'decision': decision,
        'counter_before': before, 'counter_after': after,
        'observable': True,
        'surface_probe': {'blocking': blocking},
        'padded_planning_probe': {
            'baseline_excess_count': int(
                decision == 'diagnostic_background_excess'),
            'influence_excess_count': int(blocking),
            'influence_probe': {
                'sampled_cell_count': 1, 'unknown_count': 0}},
    }


@pytest.mark.parametrize('decisions', [
    ('increment', 'diagnostic_background_excess'),
    ('diagnostic_background_excess', 'increment'),
    ('diagnostic_background_excess', 'diagnostic_background_excess'),
])
def test_evaluator_accepts_adjacent_authoritative_clear_pairs(decisions):
    """Mixed and diagnostic-only authoritative pairs are valid."""
    evaluator = _load_module('evaluate_sim_nav_obstacle.py')
    trace = [
        _trace_clear_sample(decisions[0], 0, 1),
        _trace_clear_sample(decisions[1], 1, 2),
    ]

    assert evaluator._clear_trace_has_final_pair(trace, 'global')


@pytest.mark.parametrize('intervening', [
    _trace_clear_sample('reset', 1, 0, blocking=True),
    _trace_clear_sample('ignored', 1, 0),
    _trace_clear_sample('evicted', 1, 0),
])
def test_evaluator_does_not_pair_across_intervening_samples(intervening):
    """A pair must be adjacent in the original per-source trace."""
    evaluator = _load_module('evaluate_sim_nav_obstacle.py')
    trace = [
        _trace_clear_sample('increment', 0, 1), intervening,
        _trace_clear_sample('increment', 1, 2),
    ]

    assert not evaluator._clear_trace_has_final_pair(trace, 'global')


def test_startup_rmw_matrix_is_alternating_and_domain_isolated(tmp_path):
    """Every A/B attempt must use its own valid ROS domain."""
    probe = _load_module('probe_nav2_startup_topology.py')

    matrix = probe.startup_matrix(181, 20, tmp_path)

    assert len(matrix) == 40
    assert len({item['domain_id'] for item in matrix}) == 40
    assert [item['implementation'] for item in matrix[:4]] == [
        'rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp',
        'rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp']
    assert sum(item['overlay_prefix'] is not None for item in matrix) == 20
    with pytest.raises(ValueError, match='outside 0..232'):
        probe.startup_matrix(220, 20, tmp_path)


def _startup_result(implementation, status='PASS', timeout_count=0):
    return {
        'evaluation_rmw': {'requested_identifier': implementation},
        'status': status,
        'timeout_count': timeout_count,
        'identity_survivors': [],
        'infrastructure_classification': None,
    }


def test_startup_rmw_summary_requires_complete_readiness_contract():
    """A/B passes only with active/topics, no timeout, and survivor zero."""
    probe = _load_module('probe_nav2_startup_topology.py')
    results = [
        _startup_result(implementation)
        for _ in range(2) for implementation in probe.RMW_IMPLEMENTATIONS]

    assert probe.summarize(results, 2)['status'] == 'PASS'
    results[-1]['timeout_count'] = 1
    summary = probe.summarize(results, 2)
    assert summary['status'] == 'FAIL'
    assert summary['implementations']['rmw_cyclonedds_cpp'][
        'timeout_count'] == 1


def test_startup_summary_preserves_fastdds_infrastructure_classification():
    """Known FastDDS discovery failures remain explicit in A/B evidence."""
    probe = _load_module('probe_nav2_startup_topology.py')
    result = _startup_result('rmw_fastrtps_cpp', 'FAIL', 1)
    result['infrastructure_classification'] = (
        'INFRA_INVALID_FASTDDS_SERVICE_DISCOVERY')
    results = [result, _startup_result('rmw_cyclonedds_cpp')]

    summary = probe.summarize(results, 1)

    assert summary['status'] == 'FAIL'
    assert summary['implementations']['rmw_fastrtps_cpp'][
        'infrastructure_invalid_count'] == 1
