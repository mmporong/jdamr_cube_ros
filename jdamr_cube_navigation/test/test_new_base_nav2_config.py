"""The physical candidate rejects a stale JD-AMR footprint before launch."""

import hashlib
import importlib.util
import math
from pathlib import Path
import time
from types import SimpleNamespace
from xml.etree import ElementTree as ET

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from jdamr_cube_navigation.corridor_route import (
    _positive_finite_config, CorridorRoute, current_map_xy,
    finite_localization_xy, NAVIGATION_BEHAVIOR_TREES,
    odom_distance_since_stamp, revisit_goal_witness,
    revisit_map_correction_ok, revisit_plan_length_ok,
)
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.utilities import evaluate_parameters
from nav2_common.launch import RewrittenYaml
from nav_msgs.msg import Odometry
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'jdamr_cube_navigation/launch/onboard_nav2_core.launch.py'
PARAMS = ROOT / 'jdamr_cube_navigation/config/new_base_nav2_params.yaml'
OLD_PARAMS = ROOT / 'jdamr_cube_navigation/config/nav2_params.yaml'
WRAPPER = ROOT / 'jdamr_cube_navigation/launch/onboard_keepout_navigation.launch.py'
LEGACY_AUTORUN = ROOT / 'jdamr_cube_navigation/scripts/corridor_autorun.sh'
REVISIT_BT = (ROOT / 'jdamr_cube_navigation/behavior_trees/'
              'navigate_to_pose_dynamic_obstacle_eval.xml')


def _load_validator(path, map_name='new_base_live_20260915T1407.yaml',
                    mask_name='new_base_live_20260915T1407_keepout.yaml'):
    spec = importlib.util.spec_from_file_location('new_base_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class FixedConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, _context):
            return {
                'params_file': str(path),
                'map': map_name,
                'keepout_mask': mask_name,
            }[self.name]

    module.LaunchConfiguration = FixedConfiguration
    module.get_package_share_directory = lambda _name: str(
        ROOT / 'jdamr_cube_description')
    return module._validate_new_base_params


def test_physical_candidate_is_accepted():
    assert _load_validator(PARAMS)(None) == []
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    assert document['amcl']['ros__parameters']['set_initial_pose'] is False
    assert document['amcl']['ros__parameters']['transform_tolerance'] == 1.0
    assert document['velocity_smoother']['ros__parameters']['max_velocity'][0] == 0.08
    controller = document['controller_server']['ros__parameters']
    assert controller['progress_checker']['plugin'] == (
        'nav2_controller::PoseProgressChecker')
    assert controller['progress_checker']['required_movement_radius'] == 0.05
    assert controller['progress_checker']['required_movement_angle'] == 0.10
    assert controller['progress_checker']['movement_time_allowance'] == 10.0
    assert controller['FollowPath']['rotate_to_heading_min_angle'] >= 1.57


def test_stop_zone_has_requested_geometric_margin():
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    local = document['local_costmap']['local_costmap']['ros__parameters']
    monitor = document['collision_monitor']['ros__parameters']
    footprint = yaml.safe_load(local['footprint'])
    stop_zone = monitor['StopZone']
    stop = yaml.safe_load(stop_zone['stopped']['points'])
    margins = (
        max(point[0] for point in stop) - max(point[0] for point in footprint),
        min(point[0] for point in footprint) - min(point[0] for point in stop),
        max(abs(point[1]) for point in stop)
        - max(abs(point[1]) for point in footprint),
    )
    assert margins == pytest.approx((0.05, 0.05, 0.05))
    assert stop_zone['type'] == 'velocity_polygon'
    assert stop_zone['velocity_polygons'] == [
        'rotation', 'rotation_clockwise', 'translation_forward',
        'translation_backward', 'stopped']
    rotation = yaml.safe_load(stop_zone['rotation']['points'])
    clockwise = yaml.safe_load(stop_zone['rotation_clockwise']['points'])
    assert clockwise == rotation
    assert stop_zone['rotation']['theta_min'] > 0.0
    assert stop_zone['rotation_clockwise']['theta_max'] < 0.0
    assert stop_zone['stopped']['theta_min'] <= 0.0
    assert stop_zone['stopped']['theta_max'] >= 0.0
    footprint_radius = max(math.hypot(*point) for point in footprint)
    edge_distances = []
    for start, end in zip(rotation, rotation[1:] + rotation[:1]):
        edge_distances.append(abs(
            start[0] * end[1] - start[1] * end[0]) / math.hypot(
                end[0] - start[0], end[1] - start[1]))
    assert len(rotation) >= 12
    assert min(edge_distances) >= footprint_radius
    forward = yaml.safe_load(stop_zone['translation_forward']['points'])
    assert max(point[0] for point in forward) == pytest.approx(
        max(point[0] for point in footprint) + 0.05)
    assert max(abs(point[1]) for point in forward) == pytest.approx(
        max(abs(point[1]) for point in footprint))
    backward = yaml.safe_load(stop_zone['translation_backward']['points'])
    assert min(point[0] for point in backward) == pytest.approx(
        min(point[0] for point in footprint) - 0.05)
    assert max(abs(point[1]) for point in backward) == pytest.approx(
        max(abs(point[1]) for point in footprint))
    assert monitor['source_timeout'] == 1.0
    assert monitor['FootprintApproach']['enabled'] is True
    assert document['controller_server']['ros__parameters']['FollowPath'][
        'use_collision_detection'] is True
    planner = document['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_navfn_planner::NavfnPlanner'
    assert planner['use_astar'] is False


def test_revisit_rejects_long_detour_outside_recorded_corridor():
    assert revisit_plan_length_ok(78.0, 76.42)
    assert not revisit_plan_length_ok(120.0, 76.42)


@pytest.mark.parametrize('invalid', [1.5, '1.0', False])
def test_revisit_rejects_amcl_tf_tolerance_contract_drift(
        tmp_path, invalid):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    document['amcl']['ros__parameters']['transform_tolerance'] = invalid
    candidate = tmp_path / 'candidate.yaml'
    candidate.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(RuntimeError, match='transform tolerance'):
        _load_validator(candidate)(None)


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), 0.0, -1.0])
@pytest.mark.parametrize('name', [
    'revisit_motion_amcl_freshness_s',
    'revisit_motion_min_distance_m',
    'revisit_motion_min_rotation_rad',
])
def test_revisit_motion_guard_rejects_invalid_thresholds(name, invalid):
    with pytest.raises(ValueError, match=name):
        _positive_finite_config({name: invalid}, name, 1.0)


def test_revisit_uses_previous_bounded_dynamic_replan_tree():
    root = ET.parse(REVISIT_BT).getroot()
    recoveries = list(root.iter('RecoveryNode'))
    assert len(recoveries) == 1
    assert recoveries[0].attrib['number_of_retries'] == '1'
    assert len(list(root.iter('ComputePathToPose'))) == 1
    assert len(list(root.iter('FollowPath'))) == 1
    assert root.find('.//PipelineSequence[@name="ReplanWhileDriving"]') is not None
    assert root.find('.//RateController[@hz="1.0"]') is not None
    assert root.find('.//Sequence[@name="OneBoundedCostmapRecovery"]') is not None
    assert len(list(root.iter('Wait'))) == 1
    assert not any(node.tag in {'Spin', 'BackUp'}
                   for node in root.iter())
    assert NAVIGATION_BEHAVIOR_TREES['new_base_revisit_candidate'] == (
        REVISIT_BT.name)
    assert REVISIT_BT.name in LAUNCH.read_text(
        encoding='utf-8')


def test_autonomous_mapping_recovery_never_commands_blind_motion():
    """Mapping recovery may clear and wait but never spin or reverse."""
    mapping_bt = (ROOT / 'jdamr_cube_navigation/behavior_trees/'
                  'navigate_to_pose_safe_mapping.xml')
    root = ET.parse(mapping_bt).getroot()
    recoveries = list(root.iter('RecoveryNode'))

    assert recoveries[0].attrib['number_of_retries'] == '1'
    assert not any(node.tag in {'Spin', 'BackUp'} for node in root.iter())
    assert len(list(root.iter('Wait'))) == 1


def test_revisit_rejects_the_recorded_instant_false_success():
    start = (0.0, -0.1)
    goal = (2.0, -0.42)
    assert not revisit_goal_witness(start, goal, start, 0.0, 0.35, 0.25)
    assert not revisit_goal_witness(start, goal, goal, 0.0, 0.35, 0.25)
    assert not revisit_goal_witness(start, goal, start, 2.0, 0.35, 0.25)
    assert not revisit_goal_witness(start, goal, start, 0.0, 2.1, 0.25)
    assert revisit_goal_witness(start, goal, (1.92, -0.44),
                                1.8, 0.35, 0.25)
    assert revisit_goal_witness(start, goal, goal, 1.8, 0.35, 0.25)


def test_revisit_uses_live_odom_pose_with_last_map_correction():
    map_to_odom = TransformStamped()
    map_to_odom.transform.translation.x = 1.0
    map_to_odom.transform.translation.y = -0.1
    yaw = math.pi / 2.0
    map_to_odom.transform.rotation.z = math.sin(yaw / 2.0)
    map_to_odom.transform.rotation.w = math.cos(yaw / 2.0)
    odom_to_base = TransformStamped()
    odom_to_base.transform.translation.x = 2.0
    odom_to_base.transform.rotation.w = 1.0
    assert current_map_xy(map_to_odom, odom_to_base) == pytest.approx(
        (1.0, 1.9))


def test_revisit_localization_correction_must_be_recent_or_stationary():
    assert revisit_map_correction_ok(-0.83, 0.141)
    assert revisit_map_correction_ok(2.0, 0.4)
    assert revisit_map_correction_ok(7.0, 0.05)
    assert not revisit_map_correction_ok(7.0, 0.4)
    assert not revisit_map_correction_ok(26.0, 0.0)


def test_revisit_quiet_fallback_uses_correction_stamp_not_amcl_receipt():
    history = [(1_000_000_000, 0.0),
               (10_000_000_000, 0.5),
               (20_000_000_000, 2.0)]
    since = odom_distance_since_stamp(history, 2.0, 10_000_000_000)
    assert since == 1.5
    assert not revisit_map_correction_ok(7.0, since)
    assert odom_distance_since_stamp(history, 2.0, 0) == math.inf


def test_revisit_amcl_sanity_does_not_reject_delayed_valid_pose():
    assert finite_localization_xy((1.746, -0.279))
    assert not finite_localization_xy((math.nan, -0.279))


def test_revisit_cancels_stale_amcl_only_after_accumulated_odometry_motion():
    route = object.__new__(CorridorRoute)
    now = time.monotonic()
    route.samples = {name: now for name in ('battery', 'odom', 'scan')}
    route.freshness_s = 10.0
    route.battery_freshness_s = 30.0
    route.battery_voltage = 12.0
    route.minimum_battery_v = 10.5
    route.amcl_seen = now - 26.0
    route.amcl_covariance = (0.25, 0.25)
    route.amcl_position = (0.0, 0.0)
    route.max_amcl_covariance = (50.0, 50.0)
    route.amcl_freshness_s = 60.0
    route.revisit_motion_amcl_freshness_s = 25.0
    route.revisit_motion_min_distance_m = 0.25
    route.revisit_motion_min_rotation_rad = 0.5
    route.navigation_profile = 'new_base_revisit_candidate'
    route.start_check_pending = False
    route.amcl_motion_distance_m = 0.0
    route.amcl_motion_rotation_rad = 0.0
    assert route._guard_failure(require_fresh_amcl=False) is None
    route.amcl_motion_rotation_rad = 0.49
    assert route._guard_failure(require_fresh_amcl=False) is None
    route.amcl_motion_rotation_rad = 0.5
    assert route._guard_failure(require_fresh_amcl=False).startswith(
        'AMCL pose stale:')
    route.amcl_seen = now - 17.38
    assert route._guard_failure(require_fresh_amcl=False) is None
    route.amcl_seen = now - 26.0
    route.amcl_motion_rotation_rad = 0.0
    route.amcl_motion_distance_m = 0.25
    assert route._guard_failure(require_fresh_amcl=False).startswith(
        'AMCL pose stale:')


def test_revisit_odom_counts_alternating_yaw_across_wrap_and_amcl_resets():
    route = object.__new__(CorridorRoute)
    route.samples = {'odom': None}
    route.odom_last_pose = None
    route.odom_total_distance_m = 0.0
    route.amcl_motion_distance_m = 0.0
    route.amcl_motion_rotation_rad = 0.0
    route.parking_contract = None
    for yaw in (3.0, -3.0, 3.0):
        odom = Odometry()
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        route._odom_callback(odom)
    assert 0.55 < route.amcl_motion_rotation_rad < 0.58
    assert route.amcl_motion_distance_m == 0.0
    route._amcl_callback(PoseWithCovarianceStamped())
    assert route.amcl_motion_rotation_rad == 0.0
    assert route.amcl_motion_distance_m == 0.0


def test_revisit_requires_collision_monitor_and_both_keepout_publishers():
    expected = {
        '/cmd_vel': 'collision_monitor',
        '/keepout_filter_mask': 'keepout_filter_mask_server',
        '/keepout_costmap_filter_info':
            'keepout_costmap_filter_info_server',
    }

    class Graph:
        def __init__(self, publishers):
            self.publishers = publishers

        def get_publishers_info_by_topic(self, topic):
            return [SimpleNamespace(node_name=name)
                    for name in self.publishers.get(topic, [])]

    graph = Graph({topic: [node] for topic, node in expected.items()})
    assert CorridorRoute._revisit_protection_ready(graph) is None
    graph.publishers['/cmd_vel'] = ['collision_monitor', 'web_teleop']
    assert '/cmd_vel' in CorridorRoute._revisit_protection_ready(graph)
    graph.publishers['/cmd_vel'] = ['collision_monitor']
    graph.publishers['/keepout_filter_mask'] = []
    assert '/keepout_filter_mask' in CorridorRoute._revisit_protection_ready(
        graph)


def test_jazzy_rewrite_seeds_only_the_revisit_home_pose():
    rewritten = RewrittenYaml(
        source_file=str(PARAMS), root_key='', convert_types=True,
        param_rewrites={
            'amcl.ros__parameters.set_initial_pose': 'true',
            'amcl.ros__parameters.initial_pose.x': '0.0',
            'amcl.ros__parameters.initial_pose.y': '-0.1',
            'amcl.ros__parameters.initial_pose.yaw': '0.0',
        })
    effective = yaml.safe_load(Path(rewritten.perform(LaunchContext()))
                               .read_text(encoding='utf-8'))
    amcl = effective['amcl']['ros__parameters']
    assert amcl['set_initial_pose'] is True
    assert amcl['initial_pose']['y'] == -0.1
    assert yaml.safe_load(PARAMS.read_text(encoding='utf-8'))[
        'amcl']['ros__parameters']['set_initial_pose'] is False


def test_revisit_loads_previous_wait_only_recovery_server(
        monkeypatch):
    spec = importlib.util.spec_from_file_location('new_base_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, '_validate_new_base_params',
                        lambda _context, revisit=False: [])
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda _name: str(ROOT / 'jdamr_cube_navigation'))
    context = LaunchContext()
    context.launch_configurations.update({
        'navigation_profile': 'new_base_revisit_candidate',
        'map': '/tmp/reference-map.yaml',
        'keepout_mask': '/tmp/reference-mask.yaml',
        'params_file': str(PARAMS),
        'use_sim_time': 'false',
        'autostart': 'false',
    })
    actions = module._launch_navigation(context)
    container = next(action for action in actions if
                     isinstance(action, ComposableNodeContainer))
    descriptions = container._ComposableNodeContainer__composable_node_descriptions
    names = [''.join(part.perform(context) for part in node.node_name)
             for node in descriptions]
    assert 'behavior_server' in names
    lifecycle_names = [
        evaluate_parameters(context, action._Node__parameters)[0]['node_names']
        for action in actions
        if getattr(action, '_Node__node_name', None) ==
        'lifecycle_manager_navigation'
    ]
    assert 'behavior_server' in lifecycle_names[0]


def test_onboard_collision_monitor_runs_as_required_isolated_process(
        monkeypatch):
    spec = importlib.util.spec_from_file_location('new_base_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, '_validate_new_base_params',
                        lambda _context, revisit=False: [])
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda _name: str(ROOT / 'jdamr_cube_navigation'))
    context = LaunchContext()
    context.launch_configurations.update({
        'navigation_profile': 'obstacle_candidate',
        'map': '/tmp/new-base-map.yaml',
        'keepout_mask': '/tmp/new-base-mask.yaml',
        'params_file': str(OLD_PARAMS),
        'use_sim_time': 'true',
        'autostart': 'false',
    })

    actions = module._launch_navigation(context)
    container = next(action for action in actions if
                     isinstance(action, ComposableNodeContainer))
    descriptions = container._ComposableNodeContainer__composable_node_descriptions
    component_names = {
        ''.join(part.perform(context) for part in description.node_name)
        for description in descriptions
    }
    assert 'collision_monitor' not in component_names

    monitor = next(
        action for action in actions
        if isinstance(action, Node)
        and getattr(action, '_Node__node_name', None) == 'collision_monitor')
    assert monitor._Node__package == 'nav2_collision_monitor'
    assert monitor._Node__node_executable == 'collision_monitor'
    monitor_params = evaluate_parameters(context, monitor._Node__parameters)
    assert monitor_params[-1]['use_sim_time'] is True
    protection = module.load_mobile_manipulator_protection(
        ROOT / 'jdamr_cube_navigation/config/mobile_manipulator_protection.yaml')
    assert monitor_params[-2] == protection['collision_monitor_overrides']
    assert [
        (perform_substitutions(context, source),
         perform_substitutions(context, target))
        for source, target in monitor._Node__remappings
    ] == [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle = next(
        action for action in actions
        if getattr(action, '_Node__node_name', None) ==
        'lifecycle_manager_navigation')
    lifecycle_params = evaluate_parameters(
        context, lifecycle._Node__parameters)[0]
    assert 'collision_monitor' in lifecycle_params['node_names']

    exit_handlers = [
        action for action in actions
        if action.__class__.__name__ == 'RegisterEventHandler']
    required_targets = {
        handler.event_handler._OnActionEventBase__action_matcher
        for handler in exit_handlers
    }
    assert monitor in required_targets


def test_revisit_accepts_only_verified_legacy_map_with_new_base_geometry():
    old_map = Path.home() / 'maps/autonomous_20260826T161908.yaml'
    old_mask = Path.home() / 'maps/autonomous_20260826T161908_keepout_multi.yaml'
    if not old_map.is_file() or not old_mask.is_file():
        pytest.skip('verified old corridor artifacts are not on this host')
    validator = _load_validator(PARAMS, str(old_map), str(old_mask))
    assert validator(None, revisit=True) == []


def test_physical_wrapper_fails_closed_without_new_map_and_mask():
    source = WRAPPER.read_text(encoding='utf-8')
    assert "'navigation_profile', default_value='new_base_candidate'" in source
    assert "choices=['new_base_candidate', 'new_base_revisit_candidate']" in source
    assert "'autostart', default_value='false'" in source
    assert "package_share, 'config', 'new_base_nav2_params.yaml'" in source
    assert "'map',\n            default_value=''" in source
    assert "'keepout_mask',\n            default_value=''" in source


def test_legacy_corridor_runner_rejects_old_profile_on_new_base_before_logging():
    source = LEGACY_AUTORUN.read_text(encoding='utf-8')
    marker = 'install/jdamr_cube_description/share/jdamr_cube_description/urdf/new_base_real.urdf'
    assert marker in source
    assert source.index(marker) < source.index('A="$HOME/jdamr_artifacts"')
    assert 'exit 2' in source[source.index(marker):source.index('A="$HOME/jdamr_artifacts"')]
    assert '"$NAVIGATION_PROFILE" != new_base_revisit_candidate' in source
    assert '오래된 출발 신호가 남아' in source


def test_revisit_runner_records_inputs_and_checks_graph_before_nav2():
    source = LEGACY_AUTORUN.read_text(encoding='utf-8')
    assert 'check_revisit_isolation || exit 4' in source
    assert 'jdamr-cartographer-session.service jdamr-webteleop.service' in source
    assert 'Publisher count' in source
    assert 'autostart:=true bag_output:=' in source
    assert '.inputs.sha256' in source
    assert 'motion_start_utc=' in source
    assert 'motion_end_utc=' in source
    assert source.count('check_revisit_isolation || exit 4') == 1
    assert source.index('check_revisit_isolation || exit 4') < source.index(
        'setsid nohup ros2 launch')
    assert 'REVISIT_INITIAL_X_SET=1' in source
    assert 'REVISIT_INITIAL_Y_SET=1' in source
    assert 'REVISIT_INITIAL_YAW_SET=1' in source


def test_preflight_requires_collision_monitor_not_only_one_publisher():
    source = (ROOT / 'jdamr_cube_navigation/scripts/corridor_preflight.sh'
              ).read_text(encoding='utf-8')
    assert 'ros2 topic info /cmd_vel --verbose' in source
    assert '[ "$publisher" = collision_monitor ]' in source
    assert 'Cartographer·웹 조종기 미실행' in source
    assert '로봇 정지 확인' in source


def test_revisit_reference_pins_both_yaml_and_image_hashes(tmp_path):
    spec = importlib.util.spec_from_file_location('new_base_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    artifacts = {}
    for label, yaml_name, image_name in (
            ('map', 'autonomous_20260826T161908.yaml',
             'autonomous_20260826T161908.pgm'),
            ('mask', 'autonomous_20260826T161908_keepout_multi.yaml',
             'autonomous_20260826T161908_keepout_multi.pgm')):
        image = tmp_path / image_name
        image.write_bytes(b'P5\n1 1\n255\n\xff')
        metadata = tmp_path / yaml_name
        metadata.write_text(yaml.safe_dump({'image': image_name}),
                            encoding='utf-8')
        module.REVISIT_REFERENCE[label] = (
            yaml_name, hashlib.sha256(metadata.read_bytes()).hexdigest(),
            image_name, hashlib.sha256(image.read_bytes()).hexdigest())
        artifacts[label] = (metadata, image)

    class FixedConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, _context):
            return str(artifacts[{'map': 'map',
                                  'keepout_mask': 'mask'}[self.name]][0])

    module.LaunchConfiguration = FixedConfiguration
    assert module._validate_revisit_reference(None) == []
    artifacts['map'][0].write_text('image: wrong.pgm\n', encoding='utf-8')
    with pytest.raises(RuntimeError, match='map YAML hash mismatch'):
        module._validate_revisit_reference(None)


def test_direct_core_rejects_legacy_profile_on_installed_new_base():
    spec = importlib.util.spec_from_file_location('new_base_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    selected = {'profile': 'corridor'}

    class FixedConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, _context):
            return {'use_sim_time': 'false',
                    'navigation_profile': selected['profile']}[self.name]

    module.LaunchConfiguration = FixedConfiguration
    module.get_package_share_directory = lambda _name: str(
        ROOT / 'jdamr_cube_description')
    with pytest.raises(RuntimeError, match='rejects legacy navigation profiles'):
        module._reject_legacy_profile_on_new_base(None)
    selected['profile'] = 'new_base_revisit_candidate'
    assert module._reject_legacy_profile_on_new_base(None) == []


def test_direct_revisit_launch_rejects_active_teleop_and_unknown_graph(
        monkeypatch):
    spec = importlib.util.spec_from_file_location('new_base_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _name: str(
        ROOT / 'jdamr_cube_description')

    class FixedConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, _context):
            return {'use_sim_time': 'false',
                    'navigation_profile': 'new_base_revisit_candidate'}[
                        self.name]

    module.LaunchConfiguration = FixedConfiguration
    monkeypatch.setenv('ROS_DOMAIN_ID', '12')

    def active_teleop(args, **_kwargs):
        active = args[-1] == 'jdamr-webteleop.service'
        return SimpleNamespace(returncode=(0 if active else 3),
                               stdout='')

    monkeypatch.setattr(module.subprocess, 'run', active_teleop)
    with pytest.raises(RuntimeError, match='active jdamr-webteleop.service'):
        module._validate_revisit_isolation(None)

    def unknown_graph(args, **_kwargs):
        if args[:3] == ['ros2', 'node', 'list']:
            return SimpleNamespace(returncode=1, stdout='')
        return SimpleNamespace(returncode=3, stdout='')

    monkeypatch.setattr(module.subprocess, 'run', unknown_graph)
    with pytest.raises(RuntimeError, match='cannot read ROS nodes'):
        module._validate_revisit_isolation(None)


def test_legacy_params_are_rejected():
    with pytest.raises(RuntimeError, match='footprint is smaller'):
        _load_validator(OLD_PARAMS)(None)


def test_old_map_is_rejected():
    with pytest.raises(RuntimeError, match='requires a new-base map and mask'):
        _load_validator(PARAMS, map_name='autonomous_20260826T161908.yaml')(None)


def test_narrowed_stop_zone_is_rejected(tmp_path):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    points = \
        '[[0.28, 0.25], [0.28, -0.25], [-0.28, -0.25], [-0.28, 0.25]]'
    stop_zone = document['collision_monitor']['ros__parameters']['StopZone']
    stop_zone['stopped']['points'] = points
    narrowed = tmp_path / 'narrowed.yaml'
    narrowed.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(RuntimeError, match='StopZone does not contain'):
        _load_validator(narrowed)(None)


def test_stop_zone_below_requested_margin_is_rejected(tmp_path):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    points = \
        '[[0.125, 0.33], [0.125, -0.33], [-0.335, -0.33], [-0.335, 0.33]]'
    stop_zone = document['collision_monitor']['ros__parameters']['StopZone']
    stop_zone['stopped']['points'] = points
    narrowed = tmp_path / 'below_margin.yaml'
    narrowed.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(RuntimeError, match='margin is below 0.05m'):
        _load_validator(narrowed)(None)


def test_rotation_stop_zone_below_swept_radius_is_rejected(tmp_path):
    """The launch gate rejects a polygon that misses rotating corners."""
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    points = [
        [round(0.40 * math.cos(index * math.pi / 6.0), 6),
         round(0.40 * math.sin(index * math.pi / 6.0), 6)]
        for index in range(12)
    ]
    stop_zone = document['collision_monitor']['ros__parameters']['StopZone']
    stop_zone['rotation']['points'] = points
    stop_zone['rotation_clockwise']['points'] = points
    narrowed = tmp_path / 'narrowed_rotation.yaml'
    narrowed.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match='swept corner radius'):
        _load_validator(narrowed)(None)


def _write_rotation_polygon(tmp_path, points, name):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    stop_zone = document['collision_monitor']['ros__parameters']['StopZone']
    stop_zone['rotation']['points'] = points
    stop_zone['rotation_clockwise']['points'] = points
    path = tmp_path / name
    path.write_text(yaml.safe_dump(document), encoding='utf-8')
    return path


def test_collinear_rotation_polygon_is_rejected(tmp_path):
    points = [[0.45, -0.4 + index * 0.8 / 11.0]
              for index in range(12)]
    invalid = _write_rotation_polygon(
        tmp_path, points, 'collinear_rotation.yaml')

    with pytest.raises(RuntimeError, match='zero area'):
        _load_validator(invalid)(None)


def test_rotation_polygon_shifted_away_from_origin_is_rejected(tmp_path):
    source = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    points = yaml.safe_load(source['collision_monitor']['ros__parameters'][
        'StopZone']['rotation']['points'])
    shifted = [[x + 1.0, y] for x, y in points]
    invalid = _write_rotation_polygon(
        tmp_path, shifted, 'shifted_rotation.yaml')

    with pytest.raises(RuntimeError, match='swept corner radius'):
        _load_validator(invalid)(None)


def test_self_intersecting_rotation_polygon_is_rejected(tmp_path):
    points = [
        [round(0.55 * math.cos(index * 5 * math.pi / 6.0), 6),
         round(0.55 * math.sin(index * 5 * math.pi / 6.0), 6)]
        for index in range(12)
    ]
    invalid = _write_rotation_polygon(
        tmp_path, points, 'star_rotation.yaml')

    with pytest.raises(RuntimeError, match='must be convex'):
        _load_validator(invalid)(None)


@pytest.mark.parametrize('reverse', [False, True])
def test_rotation_polygon_accepts_both_winding_directions(tmp_path, reverse):
    source = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    points = yaml.safe_load(source['collision_monitor']['ros__parameters'][
        'StopZone']['rotation']['points'])
    if reverse:
        points.reverse()
    candidate = _write_rotation_polygon(
        tmp_path, points, f'rotation_winding_{reverse}.yaml')

    assert _load_validator(candidate)(None) == []


def test_rotation_stop_zone_matching_zero_velocity_is_rejected(tmp_path):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    stop_zone = document['collision_monitor']['ros__parameters']['StopZone']
    stop_zone['rotation']['theta_min'] = 0.0
    invalid = tmp_path / 'rotation_matches_zero.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match='exclude zero velocity'):
        _load_validator(invalid)(None)


def test_velocity_polygon_order_is_fail_closed(tmp_path):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    stop_zone = document['collision_monitor']['ros__parameters']['StopZone']
    stop_zone['velocity_polygons'] = [
        'stopped', 'rotation', 'rotation_clockwise',
        'translation_forward', 'translation_backward']
    invalid = tmp_path / 'stopped_first.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match='order is incomplete or unsafe'):
        _load_validator(invalid)(None)


@pytest.mark.parametrize('value,expected', [
    (float('nan'), 'range is invalid'),
    (2.0, 'range is invalid'),
])
def test_velocity_polygon_ranges_must_be_finite_and_ordered(
        tmp_path, value, expected):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    rotation = document['collision_monitor']['ros__parameters'][
        'StopZone']['rotation']
    if math.isnan(value):
        rotation['linear_min'] = value
    else:
        rotation['linear_min'] = value
        rotation['linear_max'] = 1.0
    invalid = tmp_path / 'invalid_velocity_range.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match=expected):
        _load_validator(invalid)(None)


def test_velocity_polygon_ranges_are_pinned_to_approved_policy(tmp_path):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    stop_zone = document['collision_monitor']['ros__parameters']['StopZone']
    stop_zone['rotation']['theta_min'] = 0.9
    stop_zone['rotation_clockwise']['theta_max'] = -0.9
    invalid = tmp_path / 'rotation_policy_gap.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match=(
            'rotation velocity range violates approved policy')):
        _load_validator(invalid)(None)


def test_polygon_coordinates_must_be_finite_numeric_pairs(tmp_path):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    stop_zone = document['collision_monitor']['ros__parameters']['StopZone']
    points = yaml.safe_load(stop_zone['rotation']['points'])
    points[0][0] = float('nan')
    stop_zone['rotation']['points'] = points
    stop_zone['rotation_clockwise']['points'] = points
    invalid = tmp_path / 'nan_polygon.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match='finite numeric pairs'):
        _load_validator(invalid)(None)


@pytest.mark.parametrize('plugin_name', ['FollowPath', 'Parking'])
def test_registered_rpp_controller_requires_collision_detection(
        tmp_path, plugin_name):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    controller = document['controller_server']['ros__parameters']
    if plugin_name == 'Parking':
        controller['controller_plugins'].append(plugin_name)
        controller[plugin_name] = dict(controller['FollowPath'])
    controller[plugin_name]['use_collision_detection'] = False
    invalid = tmp_path / f'{plugin_name}_collision_disabled.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match=(
            f'{plugin_name} collision detection must be active')):
        _load_validator(invalid)(None)


@pytest.mark.parametrize('plugins', [[], ['Parking']])
def test_registered_controllers_require_follow_path(tmp_path, plugins):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    controller = document['controller_server']['ros__parameters']
    controller['controller_plugins'] = plugins
    invalid = tmp_path / 'missing_follow_path.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match='must include FollowPath'):
        _load_validator(invalid)(None)


def test_missing_registered_controller_is_rejected(tmp_path):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    controller = document['controller_server']['ros__parameters']
    controller['controller_plugins'].append('Parking')
    invalid = tmp_path / 'missing_registered_controller.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match=(
            'registered controller Parking is missing')):
        _load_validator(invalid)(None)


def test_unknown_registered_controller_plugin_is_rejected(tmp_path):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    controller = document['controller_server']['ros__parameters']
    controller['controller_plugins'].append('Unsafe')
    controller['Unsafe'] = {
        'plugin': 'dwb_core::DWBLocalPlanner',
        'desired_linear_vel': 0.08,
        'use_collision_detection': True,
    }
    invalid = tmp_path / 'unsupported_controller.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match=(
            'controller Unsafe plugin is unsupported')):
        _load_validator(invalid)(None)


@pytest.mark.parametrize('velocity', [float('nan'), 0.0, -0.01, 0.081])
def test_registered_controller_velocity_is_finite_positive_and_bounded(
        tmp_path, velocity):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    controller = document['controller_server']['ros__parameters']
    controller['FollowPath']['desired_linear_vel'] = velocity
    invalid = tmp_path / 'unsafe_controller_velocity.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match=(
            'controller speed exceeds uncalibrated limit')):
        _load_validator(invalid)(None)


@pytest.mark.parametrize('mutate,expected', [
    (lambda monitor: monitor['StopZone'].update(enabled=False),
     'StopZone is not active'),
    (lambda monitor: monitor['StopZone'].update(action_type='slowdown'),
     'StopZone is not active'),
    (lambda monitor: monitor['StopZone'].update(min_points=100000),
     'StopZone is not active'),
    (lambda monitor: monitor['SlowdownZone'].update(min_points=True),
     'SlowdownZone is not active'),
    (lambda monitor: monitor['SlowdownZone'].update(slowdown_ratio=1.0),
     'slowdown ratio violates approved policy'),
    (lambda monitor: monitor['SlowdownZone'].update(slowdown_ratio=float('nan')),
     'slowdown ratio violates approved policy'),
    (lambda monitor: monitor['FootprintApproach'].update(enabled=False),
     'approach monitor is not active'),
    (lambda monitor: monitor['FootprintApproach'].update(type='circle'),
     'approach monitor is not active'),
    (lambda monitor: monitor['FootprintApproach'].update(min_points=4),
     'approach monitor is not active'),
    (lambda monitor: monitor['FootprintApproach'].update(
        time_before_collision=0.0),
     'approach time_before_collision violates approved policy'),
    (lambda monitor: monitor['FootprintApproach'].update(
        simulation_time_step=100.0),
     'approach simulation_time_step violates approved policy'),
    (lambda monitor: monitor.update(base_frame_id='base_link'),
     'collision monitor needs base_footprint'),
    (lambda monitor: monitor['FootprintApproach'].update(
        footprint_topic='/wrong_footprint'),
     'approach monitor is not active'),
    (lambda monitor: monitor['scan'].update(enabled=False),
     'scan collision source is not active'),
    (lambda monitor: monitor.update(observation_sources=[]),
     'scan collision source is not active'),
    (lambda monitor: monitor.update(source_timeout=3.0),
     'scan source timeout must be 1.0s'),
    (lambda monitor: monitor.update(source_timeout=True),
     'scan source timeout must be 1.0s'),
    (lambda monitor: monitor['scan'].update(source_timeout=1.1),
     'scan timeout override must be exactly 1.0s'),
    (lambda monitor: monitor['scan'].update(source_timeout=0.0),
     'scan timeout override must be exactly 1.0s'),
    (lambda monitor: monitor['scan'].update(source_timeout=0.5),
     'scan timeout override must be exactly 1.0s'),
    (lambda monitor: monitor['StopZone']['stopped'].update(points=(
        '[[0.35, 0.35], [0.35, 0.30], [-0.38, 0.30], [-0.38, 0.35]]')),
     'polygon must cover both sides'),
    (lambda monitor: monitor['StopZone']['stopped'].update(points=(
        '[[0.35, 0.35], [-0.38, -0.35], [0.35, -0.35], [-0.38, 0.35]]')),
     'corners must follow the perimeter'),
])
def test_disabled_or_one_sided_monitor_is_rejected(tmp_path, mutate, expected):
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    mutate(document['collision_monitor']['ros__parameters'])
    invalid = tmp_path / 'invalid.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(RuntimeError, match=expected):
        _load_validator(invalid)(None)


@pytest.mark.parametrize('name', ['StopZone', 'SlowdownZone', 'FootprintApproach'])
@pytest.mark.parametrize('sources', [[], ['missing'], ['scan', 'missing']])
def test_polygon_cannot_drop_scan_observations(tmp_path, name, sources):
    """An active scan source must still participate in every safety polygon."""
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    document['collision_monitor']['ros__parameters'][name]['sources_names'] = sources
    invalid = tmp_path / 'invalid_polygon_sources.yaml'
    invalid.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(RuntimeError, match='must observe the scan source'):
        _load_validator(invalid)(None)


def test_explicit_scan_source_is_allowed_for_safety_polygons(tmp_path):
    """Keep explicit and upstream-default scan selection equivalent."""
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    for name in ('StopZone', 'SlowdownZone', 'FootprintApproach'):
        document['collision_monitor']['ros__parameters'][name]['sources_names'] = ['scan']
    valid = tmp_path / 'explicit_polygon_sources.yaml'
    valid.write_text(yaml.safe_dump(document), encoding='utf-8')
    assert _load_validator(valid)(None) == []
