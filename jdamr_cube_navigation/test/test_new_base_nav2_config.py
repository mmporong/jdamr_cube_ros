"""The physical candidate rejects a stale JD-AMR footprint before launch."""

import hashlib
import importlib.util
import math
from pathlib import Path
import time
from types import SimpleNamespace
from xml.etree import ElementTree as ET

from jdamr_cube_navigation.corridor_route import (
    CorridorRoute, NAVIGATION_BEHAVIOR_TREES, _positive_finite_config,
    revisit_plan_length_ok,
    revisit_goal_witness,
    current_map_xy,
    revisit_map_correction_ok,
    odom_distance_since_stamp,
    finite_localization_xy,
)
from launch import LaunchContext
from launch_ros.actions import ComposableNodeContainer
from launch_ros.utilities import evaluate_parameters
from nav2_common.launch import RewrittenYaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
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
        'nav2_controller::SimpleProgressChecker')
    assert controller['progress_checker']['required_movement_radius'] == 0.05
    assert controller['progress_checker']['movement_time_allowance'] == 15.0
    assert controller['FollowPath']['rotate_to_heading_min_angle'] >= 1.57


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
    document['collision_monitor']['ros__parameters']['StopZone']['points'] = \
        '[[0.28, 0.25], [0.28, -0.25], [-0.28, -0.25], [-0.28, 0.25]]'
    narrowed = tmp_path / 'narrowed.yaml'
    narrowed.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(RuntimeError, match='StopZone does not contain'):
        _load_validator(narrowed)(None)


@pytest.mark.parametrize('mutate,expected', [
    (lambda monitor: monitor['StopZone'].update(enabled=False),
     'StopZone is not active'),
    (lambda monitor: monitor['StopZone'].update(action_type='slowdown'),
     'StopZone is not active'),
    (lambda monitor: monitor['scan'].update(enabled=False),
     'scan collision source is not active'),
    (lambda monitor: monitor.update(observation_sources=[]),
     'scan collision source is not active'),
    (lambda monitor: monitor['StopZone'].update(points=(
        '[[0.35, 0.35], [0.35, 0.30], [-0.38, 0.30], [-0.38, 0.35]]')),
     'polygon must cover both sides'),
    (lambda monitor: monitor['StopZone'].update(points=(
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
