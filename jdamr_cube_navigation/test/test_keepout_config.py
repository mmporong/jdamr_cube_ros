"""Regression tests for saved-map keepout navigation."""

import ast
import hashlib
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from geometry_msgs.msg import TransformStamped
from jdamr_cube_navigation.corridor_route import (
    AMCL_QOS,
    CorridorRoute,
    load_route,
)
from jdamr_cube_navigation.keepout_mask import build_mask, validate_mask
from jdamr_cube_navigation.keepout_zone_capture import (
    order_polygon_points,
    zone_document,
    zones_document,
)
from jdamr_cube_navigation.onboard_recording import RECORDED_TOPICS
from jdamr_cube_navigation.tf_replay_filter import filter_tf_message
import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMS_PATH = PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
ZONES_EXAMPLE = PACKAGE_ROOT / 'config' / 'keepout_zones.example.yaml'
ZONES_AUTONOMOUS = (
    PACKAGE_ROOT / 'config' / 'keepout_zones.autonomous_20260826.yaml')
ROUNDTRIP_ROUTE = (
    PACKAGE_ROOT / 'config' / 'corridor_roundtrip.autonomous_20260826.yaml')
NAVIGATION_LAUNCH = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
KEEPOUT_LAUNCH = PACKAGE_ROOT / 'launch' / 'keepout_navigation.launch.py'
CAPTURE_LAUNCH = PACKAGE_ROOT / 'launch' / 'keepout_capture.launch.py'
REPLAY_LAUNCH = PACKAGE_ROOT / 'launch' / 'offline_replay_guard.launch.py'
ONBOARD_LAUNCH = (
    PACKAGE_ROOT / 'launch' / 'onboard_keepout_navigation.launch.py')
ONBOARD_CORE_LAUNCH = (
    PACKAGE_ROOT / 'launch' / 'onboard_nav2_core.launch.py')
OPERATOR_VIEW_LAUNCH = (
    PACKAGE_ROOT / 'launch' / 'keepout_operator_view.launch.py')
KEEPOUT_RVIZ = PACKAGE_ROOT / 'rviz' / 'keepout_navigation.rviz'
SETUP_PATH = PACKAGE_ROOT / 'setup.py'
ROUTE_SOURCE = (
    PACKAGE_ROOT / 'jdamr_cube_navigation' / 'corridor_route.py')
CORRIDOR_BT = (
    PACKAGE_ROOT / 'behavior_trees' /
    'navigate_to_pose_corridor_fail_fast.xml')


def _params():
    with PARAMS_PATH.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _costmap(config, name):
    return config[name][name]['ros__parameters']


def _write_source_map(root):
    image = root / 'source.pgm'
    image.write_bytes(b'P5\n30 30\n255\n' + bytes([254]) * 900)
    map_yaml = root / 'source.yaml'
    map_yaml.write_text(
        yaml.safe_dump({
            'image': image.name,
            'mode': 'trinary',
            'resolution': 0.1,
            'origin': [0.0, 0.0, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
        }, sort_keys=False),
        encoding='utf-8',
    )
    return map_yaml


def _write_zones(
        root, map_yaml, enabled=True, margin=0.4,
        polygon=None, connectivity_checks=None):
    if polygon is None:
        polygon = [
            [1.4, 1.4], [1.6, 1.4],
            [1.6, 1.6], [1.4, 1.6],
        ]
    zones_yaml = root / 'zones.yaml'
    zones_yaml.write_text(
        yaml.safe_dump({
            'schema_version': 1,
            'map_yaml': str(map_yaml),
            'safety_margin_m': margin,
            'zones': [{
                'id': 'stairs',
                'enabled': enabled,
                'polygon': polygon,
            }],
            'connectivity_checks': connectivity_checks or [],
        }, sort_keys=False),
        encoding='utf-8',
    )
    return zones_yaml


def test_global_and_local_costmaps_have_disabled_keepout_filters():
    """Keep ordinary launches compatible while the safe wrapper enables both."""
    config = _params()
    for name in ('global_costmap', 'local_costmap'):
        costmap = _costmap(config, name)
        keepout = costmap['keepout_filter']

        assert costmap['filters'] == ['keepout_filter'], name
        assert keepout['plugin'] == 'nav2_costmap_2d::KeepoutFilter', name
        assert keepout['enabled'] is False, name
        assert keepout['filter_info_topic'] == \
            '/keepout_costmap_filter_info', name
        assert keepout['override_lethal_cost'] is False, name


def test_waypoint_follower_stops_when_keepout_makes_goal_unreachable():
    """Do not skip a blocked waypoint and continue along an unexpected route."""
    waypoint = _params()['waypoint_follower']['ros__parameters']

    assert waypoint['stop_on_failure'] is True


def test_keepout_launch_is_required_and_fail_closed():
    """Require aligned mask servers and shut down if either server exits."""
    navigation_source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')
    keepout_source = KEEPOUT_LAUNCH.read_text(encoding='utf-8')

    ast.parse(navigation_source)
    ast.parse(keepout_source)
    assert "'use_keepout': 'true'" in keepout_source
    assert 'OpaqueFunction(function=_validate_keepout)' in navigation_source
    assert "name='keepout_filter_mask_server'" in navigation_source
    assert "name='keepout_costmap_filter_info_server'" in navigation_source
    assert "name='lifecycle_manager_keepout'" in navigation_source
    assert navigation_source.count('OnProcessExit(') == 2
    assert "'use_composition': use_composition" in navigation_source
    assert "'use_composition', default_value='false'" in navigation_source
    assert "'slam': 'False'" in navigation_source
    assert "'use_localization': 'True'" in navigation_source


def test_saved_map_is_injected_into_nav2_parameters():
    """Keep the main map path even when nested launch argument scope is lost."""
    source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')

    assert "'yaml_filename': map_yaml" in source


def test_saved_map_navigation_injects_the_safe_behavior_tree():
    """Keep NavigateToPose and waypoint_follower from loading an empty tree."""
    source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')

    assert "'default_nav_to_pose_bt_xml': safe_bt" in source
    assert "'behavior_trees', 'navigate_to_pose_safe_mapping.xml'" in source


def test_keepout_launch_starts_dedicated_rviz_by_default():
    """Show the saved map, mask, localization, and Nav2 goal tools together."""
    source = KEEPOUT_LAUNCH.read_text(encoding='utf-8')

    ast.parse(source)
    assert "name='keepout_navigation_rviz'" in source
    assert "executable='rviz2'" in source
    assert 'condition=IfCondition(use_rviz)' in source
    assert "DeclareLaunchArgument(\n            'use_rviz', default_value='true'" in source
    assert "'rviz', 'keepout_navigation.rviz'" in source

    config = yaml.safe_load(KEEPOUT_RVIZ.read_text(encoding='utf-8'))
    manager = config['Visualization Manager']
    displays = {display['Name']: display for display in manager['Displays']}

    assert manager['Global Options']['Fixed Frame'] == 'map'
    assert displays['Map']['Topic']['Value'] == '/map'
    assert displays['Keepout Zones']['Topic']['Value'] == \
        '/keepout_filter_mask'
    assert displays['Keepout Zones']['Enabled'] is True
    assert 0.0 < displays['Keepout Zones']['Alpha'] < 1.0
    assert displays['Global Costmap']['Enabled'] is False
    assert displays['Localization']['Enabled'] is True
    assert displays['LaserScan']['Enabled'] is False
    assert manager['Global Options']['Frame Rate'] == 10
    tool_classes = {tool['Class'] for tool in manager['Tools']}
    assert 'rviz_default_plugins/SetInitialPose' in tool_classes
    assert 'nav2_rviz_plugins/GoalTool' in tool_classes


def test_capture_launch_has_map_and_rviz_but_no_navigation_servers():
    """Capture coordinates without starting any node that can drive the base."""
    source = CAPTURE_LAUNCH.read_text(encoding='utf-8')

    ast.parse(source)
    assert "executable='map_server'" in source
    assert "executable='keepout_zone_capture'" in source
    assert "executable='rviz2'" in source
    assert source.count('OnProcessExit(') == 2
    for forbidden in ('controller_server', 'planner_server', 'bt_navigator'):
        assert forbidden not in source


def test_offline_replay_launch_whitelists_inputs_and_requires_isolation():
    """Never replay saved maps or motion commands into the physical domain."""
    source = REPLAY_LAUNCH.read_text(encoding='utf-8')

    ast.parse(source)
    assert "default_value='199'" in source
    assert "actual_domain == '12' or expected_domain == '12'" in source
    assert 'actual_domain != expected_domain' in source
    assert 'not 0.0 < rate <= 1.0' in source
    assert "'/scan', '/odom', '/tf', '/tf_static'" in source
    assert "'/tf:=/tf_recorded'" in source
    assert "'/tf_static:=/tf_static_recorded'" in source
    for excluded in ("'/map'", "'/cmd_vel'"):
        assert excluded not in source


def test_onboard_navigation_keeps_control_and_recording_off_wifi():
    """Run Nav2 and evidence capture together on the robot computer."""
    source = ONBOARD_LAUNCH.read_text(encoding='utf-8')

    ast.parse(source)
    assert "'onboard_nav2_core.launch.py'" in source
    assert 'rviz2' not in source
    assert "'record_bag', default_value='true'" in source
    assert "'--storage', 'mcap'" in source
    assert "'--topics', *RECORDED_TOPICS" in source
    assert "'ionice', '--class', 'best-effort', '--classdata', '7'" in source
    assert "'nice', '--adjustment', '10'" in source
    # The topic list itself moved into the shared recording contract so the
    # launch file and the QoS overrides cannot drift apart.
    assert '/scan' in RECORDED_TOPICS
    assert '/odom' in RECORDED_TOPICS
    assert '/tf' in RECORDED_TOPICS
    assert '/joint_states' not in RECORDED_TOPICS
    assert 'onboard recorder exited; stopping navigation' in source
    assert 'if context.is_shutdown' in source
    assert 'bag_output already exists' in source
    assert source.rfind('navigation,') < source.rfind('recorder,')


def test_onboard_core_loads_only_corridor_required_nav2_processes():
    """Keep unused servers off Pi and isolate each required Nav2 process."""
    source = ONBOARD_CORE_LAUNCH.read_text(encoding='utf-8')

    ast.parse(source)
    for required in (
            "executable='map_server'", "executable='amcl'",
            "executable='controller_server'",
            "executable='planner_server'",
            "executable='bt_navigator'",
            "executable='velocity_smoother'",
            "executable='collision_monitor'"):
        assert required in source
    for omitted in (
            'nav2_route::RouteServer', 'opennav_docking::DockingServer',
            'nav2_smoother::SmootherServer',
            'nav2_waypoint_follower::WaypointFollower',
            'behavior_server::BehaviorServer'):
        assert omitted not in source
    assert "'navigate_to_pose_corridor_fail_fast.xml'" in source
    assert "'yaml_filename': map_yaml" in source
    assert source.count("'keepout_filter.enabled'): 'true'") == 2
    assert 'ComposableNodeContainer' not in source
    assert 'required_exit_handlers' in source
    assert 'OpaqueFunction(function=_validate_keepout)' in source
    assert "name='keepout_filter_mask_server'" in source
    assert "name='keepout_costmap_filter_info_server'" in source


def test_operator_view_never_starts_navigation_or_recording():
    """Let Wi-Fi clients inspect state without owning the motion loop."""
    source = OPERATOR_VIEW_LAUNCH.read_text(encoding='utf-8')

    ast.parse(source)
    assert "name='keepout_operator_view'" in source
    assert "executable='rviz2'" in source
    for forbidden in (
            'controller_server', 'bt_navigator', 'ros2 bag', 'map_server'):
        assert forbidden not in source


def test_corridor_route_uses_fail_fast_tree_and_continuous_guards():
    """Never hide a corridor fault behind spin, wait, or repeated goals."""
    source = ROUTE_SOURCE.read_text(encoding='utf-8')
    root = ET.parse(CORRIDOR_BT).getroot()
    tags = {element.tag for element in root.iter()}

    assert 'navigate_to_pose_corridor_fail_fast.xml' in source
    assert 'RecoveryNode' not in tags
    assert 'Spin' not in tags
    assert 'BackUp' not in tags
    assert 'Wait' not in tags
    assert 'ComputePathToPose' in tags
    assert 'FollowPath' in tags
    assert 'PipelineSequence' not in tags
    assert 'RateController' not in tags
    # The BT attribute overrides bt_navigator's default_server_timeout, so
    # both must move together.  1000ms aborted every first goal on 2026-09-03
    # because it landed while planner_server was still busy with the preflight.
    assert root.findall(
        './/ComputePathToPose')[0].attrib['server_timeout'] == '3000'
    assert root.findall('.//FollowPath')[0].attrib['server_timeout'] == '3000'
    assert 'if not self._navigation_ready()' in source
    assert 'self._guard_failure()' in source


def test_corridor_bt_matches_separate_process_response_budget():
    """Avoid treating normal Pi DDS latency as a planner failure."""
    config = _params()
    navigator = config['bt_navigator']['ros__parameters']
    planner = config['planner_server']['ros__parameters']

    assert navigator['default_server_timeout'] == 3000
    assert navigator['wait_for_service_timeout'] == 5000
    assert planner['expected_planner_frequency'] == 1.0


def test_tf_replay_filter_removes_only_recorded_map_to_odom():
    """Preserve odometry and sensor TF while dropping localization authority."""
    map_to_odom = TransformStamped()
    map_to_odom.header.frame_id = '/map'
    map_to_odom.child_frame_id = '/odom'
    odom_to_base = TransformStamped()
    odom_to_base.header.frame_id = 'odom'
    odom_to_base.child_frame_id = 'base_footprint'
    base_to_laser = TransformStamped()
    base_to_laser.header.frame_id = 'base_link'
    base_to_laser.child_frame_id = 'laser_link'

    filtered, dropped = filter_tf_message(TFMessage(
        transforms=[map_to_odom, odom_to_base, base_to_laser]))

    assert dropped == 1
    assert filtered.transforms == [odom_to_base, base_to_laser]


def test_mask_builder_preserves_map_geometry_and_adds_margin(tmp_path):
    """Generate a non-empty mask aligned exactly with its source map."""
    map_yaml = _write_source_map(tmp_path)
    zones_yaml = _write_zones(tmp_path, map_yaml)
    prefix = tmp_path / 'keepout'

    report = build_mask(zones_yaml, prefix)
    validated = validate_mask(prefix.with_suffix('.yaml'), map_yaml)

    assert report['keepout_cells'] > 16
    assert report['safety_margin_m'] == 0.4
    assert report['zones'] == ['stairs']
    assert validated['width'] == 30
    assert validated['height'] == 30
    assert validated['resolution'] == 0.1
    assert validated['origin'] == [0.0, 0.0, 0.0]


def test_mask_builder_rejects_empty_or_under_margin_zones(tmp_path):
    """Refuse masks that would falsely claim keepout protection."""
    map_yaml = _write_source_map(tmp_path)
    disabled = _write_zones(tmp_path, map_yaml, enabled=False)

    with pytest.raises(ValueError, match='enabled keepout zone'):
        build_mask(disabled, tmp_path / 'disabled')

    narrow = _write_zones(tmp_path, map_yaml, margin=0.1)
    with pytest.raises(ValueError, match='safety_margin_m'):
        build_mask(narrow, tmp_path / 'narrow')


def test_mask_builder_preserves_required_route_or_fails_closed(tmp_path):
    """Reject a keepout polygon that disconnects a declared clear route."""
    map_yaml = _write_source_map(tmp_path)
    check = [{
        'id': 'main_corridor',
        'start': [0.5, 1.5],
        'goal': [2.5, 1.5],
        'clearance_m': 0.1,
    }]
    passable = _write_zones(
        tmp_path, map_yaml, connectivity_checks=check)
    report = build_mask(passable, tmp_path / 'passable')

    assert report['connectivity_checks'][0]['connected'] is True
    assert report['connectivity_checks'][0]['reachable_cells'] > 0

    blocking = _write_zones(
        tmp_path, map_yaml,
        polygon=[[1.4, 0.0], [1.6, 0.0], [1.6, 3.0], [1.4, 3.0]],
        connectivity_checks=check,
    )
    with pytest.raises(ValueError, match='disconnects main_corridor'):
        build_mask(blocking, tmp_path / 'blocking')


def test_example_requires_replacing_placeholder_before_build():
    """Keep the repository example non-operational until coordinates are real."""
    example = yaml.safe_load(ZONES_EXAMPLE.read_text(encoding='utf-8'))

    assert example['safety_margin_m'] >= 0.55
    assert not any(zone['enabled'] for zone in example['zones'])


def test_annotated_corridor_keepout_matches_the_confirmed_rviz_polygons():
    """Keep the two user-confirmed stair branches reproducible in map frame."""
    config = yaml.safe_load(ZONES_AUTONOMOUS.read_text(encoding='utf-8'))
    zones = {zone['id']: zone for zone in config['zones']}

    assert config['map_yaml'].endswith(
        'maps/autonomous_20260826T161908.yaml')
    assert config['safety_margin_m'] == 0.55
    assert config['annotation_alignment']['matched_features'] == 8
    assert config['verification']['expected_keepout_cells'] == 9406
    assert config['connectivity_checks'] == [{
        'id': 'main_corridor_left_to_right',
        'start': [-0.002, 0.0],
        'goal': [37.498, -4.3],
        'clearance_m': 0.25,
    }]
    assert set(zones) == {'keepout_1', 'keepout_2'}
    assert all(zone['enabled'] for zone in zones.values())
    assert max(point[1] for point in zones['keepout_1']['polygon']) <= -2.22
    assert max(point[1] for point in zones['keepout_2']['polygon']) <= -3.74


def test_confirmed_roundtrip_route_keeps_outbound_turnaround_and_return():
    """Preserve the 80 m Nav2-preflighted corridor route for the next run."""
    config = yaml.safe_load(ROUNDTRIP_ROUTE.read_text(encoding='utf-8'))
    waypoints = config['waypoints']

    assert config['planned_length_m'] == 76.42
    assert config['sensor_freshness_s'] == 2.5
    assert config['minimum_battery_v'] == 10.5
    assert config['max_amcl_x_covariance'] == 2.0
    assert config['max_amcl_y_covariance'] == 1.0
    assert config['amcl_freshness_s'] == 60.0
    assert config['max_resume_start_distance_m'] == 6.0
    assert len(waypoints) == 20
    assert waypoints[0]['id'] == 'outbound_02m'
    assert waypoints[9]['id'] == 'turnaround'
    assert waypoints[-1] == {
        'id': 'home', 'x': 0.0, 'y': -0.1, 'yaw': 0.0}


def test_corridor_route_is_planning_first_and_signal_safe():
    """Do not move by default or invalidate ROS before goal cancellation."""
    source = ROUTE_SOURCE.read_text(encoding='utf-8')

    ast.parse(source)
    assert "'--execute', action='store_true'" in source
    assert 'ComputePathThroughPoses' in source
    assert 'goal.behavior_tree = self.behavior_tree' in source
    assert 'SignalHandlerOptions.NO' in source
    assert 'self._guard_failure()' in source
    assert 'Publisher(' not in source
    assert source.count('previous_sigint = signal.signal') == 1


def test_corridor_route_receives_amcl_pose_when_started_after_localization():
    """Read AMCL's latched pose instead of waiting for robot movement."""
    assert AMCL_QOS.depth == 1
    assert AMCL_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert AMCL_QOS.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_corridor_route_uses_axis_specific_amcl_covariance_limits():
    """Allow corridor-axis ambiguity while keeping lateral drift strict."""
    route = object.__new__(CorridorRoute)
    now = time.monotonic()
    route.samples = {name: now for name in ('battery', 'odom', 'scan')}
    route.freshness_s = 2.5
    route.battery_voltage = 12.0
    route.minimum_battery_v = 10.5
    route.amcl_seen = now
    route.amcl_covariance = (0.519, 0.151)
    route.amcl_position = (0.0, 0.0)
    route.max_amcl_covariance = (2.0, 1.0)
    route.amcl_freshness_s = 60.0
    route.max_resume_start_distance_m = 6.0
    route.resume_start_check_pending = False

    assert route._guard_failure() is None
    route.amcl_covariance = (0.519, 1.001)
    assert route._guard_failure() == (
        'AMCL y covariance high: value=1.001 limit=1.000')


def test_corridor_route_rejects_resume_after_amcl_resets_to_origin():
    """Do not drive a remaining route from a silently reset map pose."""
    route = object.__new__(CorridorRoute)
    now = time.monotonic()
    route.samples = {name: now for name in ('battery', 'odom', 'scan')}
    route.freshness_s = 2.5
    route.battery_voltage = 12.0
    route.minimum_battery_v = 10.5
    route.amcl_seen = now
    route.amcl_covariance = (0.25, 0.25)
    route.max_amcl_covariance = (2.0, 1.0)
    route.amcl_freshness_s = 60.0
    route.waypoints = [{'x': 30.0, 'y': -2.37}]
    route.amcl_position = (0.0, 0.0)
    route.max_resume_start_distance_m = 6.0
    route.resume_start_check_pending = True

    assert route._guard_failure() == (
        'AMCL resume start too far: distance=30.093m limit=6.000m')

    route.resume_start_check_pending = False
    assert route._guard_failure() is None


def test_route_loader_rejects_a_changed_keepout_mask(tmp_path):
    """Bind a route to the exact reviewed mask instead of only its filename."""
    source_map = tmp_path / 'map.yaml'
    source_map.write_text('image: map.pgm\n', encoding='utf-8')
    mask_image = tmp_path / 'mask.pgm'
    mask_image.write_bytes(b'P5\n1 1\n255\n\x00')
    mask_yaml = tmp_path / 'mask.yaml'
    mask_yaml.write_text('image: mask.pgm\n', encoding='utf-8')
    expected_hash = hashlib.sha256(mask_image.read_bytes()).hexdigest()
    route_yaml = tmp_path / 'route.yaml'
    route_yaml.write_text(yaml.safe_dump({
        'schema_version': 1,
        'map_yaml': str(source_map),
        'keepout_mask_yaml': str(mask_yaml),
        'expected_mask_sha256': expected_hash,
        'waypoints': [
            {'id': 'start', 'x': 0.0, 'y': 0.0},
            {'id': 'goal', 'x': 1.0, 'y': 0.0},
        ],
    }), encoding='utf-8')

    loaded = load_route(route_yaml)
    assert loaded['keepout_mask_image'] == mask_image

    mask_image.write_bytes(b'changed')
    with pytest.raises(ValueError, match='hash mismatch'):
        load_route(route_yaml)


def test_clicked_points_become_an_enabled_map_frame_zone():
    """Keep the RViz capture output compatible with the mask builder."""
    document = zone_document(
        Path('/tmp/map.yaml'), 'stairs', 0.55,
        [[1.0, 2.0], [2.0, 2.0], [2.0, 3.0]],
    )

    assert document['schema_version'] == 1
    assert document['safety_margin_m'] == 0.55
    assert document['zones'][0]['enabled'] is True
    assert len(document['zones'][0]['polygon']) == 3


def test_multiple_clicked_rectangles_share_one_zone_document():
    """Keep every completed rectangle in one mask-builder input file."""
    zones = [
        {
            'id': 'keepout_1',
            'enabled': True,
            'polygon': [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]],
        },
        {
            'id': 'keepout_2',
            'enabled': True,
            'polygon': [[4.0, 4.0], [5.0, 4.0], [5.0, 5.0], [4.0, 5.0]],
        },
    ]

    document = zones_document(Path('/tmp/map.yaml'), 0.55, zones)

    assert document['zones'] == zones
    assert [zone['id'] for zone in document['zones']] == [
        'keepout_1', 'keepout_2']


def test_rectangle_points_are_ordered_even_when_clicked_across_diagonal():
    """Avoid a bow-tie mask when opposite corners are clicked in sequence."""
    clicked = [[1.0, 2.0], [3.0, 2.0], [1.0, 0.0], [3.0, 0.0]]

    ordered = order_polygon_points(clicked)

    assert ordered == [
        [1.0, 0.0], [3.0, 0.0], [3.0, 2.0], [1.0, 2.0]]


def test_package_installs_keepout_command_and_assets():
    """Install the mask tool, launch files, and zone configuration."""
    setup_source = SETUP_PATH.read_text(encoding='utf-8')

    assert 'keepout_mask = jdamr_cube_navigation.keepout_mask:main' in \
        setup_source
    assert 'keepout_zone_capture = ' in setup_source
    assert 'tf_replay_filter = ' in setup_source
    assert "glob('launch/*.launch.py')" in setup_source
    assert "glob('config/*.yaml')" in setup_source
    assert "glob('rviz/*.rviz')" in setup_source


def test_lifecycle_managers_tolerate_pi_service_latency():
    """A slow bond reply must not read as a dead node."""
    # 2026-09-03: the default 4 s bond timeout reported a healthy keepout
    # server as failed and aborted its bringup, leaving the costmap filter
    # info server inactive so keepout zones were not applied at all.
    source = ONBOARD_CORE_LAUNCH.read_text(encoding='utf-8')

    assert source.count("'bond_timeout': 10.0") == 3
    assert source.count("'bond_respawn_max_duration': 20.0") == 3


def test_replay_guard_orders_odometry_against_scans():
    """Late odometry kills Cartographer mid-replay, so the guard drops it."""
    # corridor_keepout_roundtrip_20260901T150446 carries 63 odometry messages
    # stamped before the newest scan, up to 1.340 s inverted, because Wi-Fi
    # delayed them during recording.  Cartographer aborts with
    # "Check failed: odometry_data.time >= timed_pose_queue_.back().time".
    from jdamr_cube_navigation.tf_replay_filter import stamp_seconds

    guard = (PACKAGE_ROOT / 'jdamr_cube_navigation'
             / 'tf_replay_filter.py').read_text(encoding='utf-8')
    replay = (PACKAGE_ROOT / 'launch'
              / 'offline_replay_guard.launch.py').read_text(encoding='utf-8')

    assert "'/odom_recorded'" in guard
    assert "'/odom:=/odom_recorded'" in replay
    assert 'self.late_odometry' in guard

    class _Stamp:
        sec = 5
        nanosec = 500_000_000

    assert stamp_seconds(_Stamp()) == 5.5


def test_autorun_never_drives_without_passing_every_gate():
    """A run the operator cannot watch must refuse itself on any doubt."""
    # 2026-09-03: Wi-Fi dropped in the corridor and the start command never
    # reached the robot.  Recording always lived on the Pi's SD card, so the
    # link is only needed to start, stop and collect.  Moving the start into
    # the robot removes the dependency, which means nobody is watching while
    # it decides to move.
    source = (PACKAGE_ROOT / 'scripts'
              / 'corridor_autorun.sh').read_text(encoding='utf-8')

    for gate in ('lifecycle 매니저', '사전점검 FAIL', '전체 경로 계획 실패'):
        assert gate in source
    assert source.count('주행하지 않는다') >= 4
    assert 'jdamr_abort' in source
    # The stack must come down even when a gate aborts the script.
    assert 'trap stop_stack EXIT' in source
    # Signal strength is logged to the robot so a dropped link still leaves
    # evidence of where the corridor coverage failed.
    assert '/proc/net/wireless' in source
