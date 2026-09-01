"""Regression tests for saved-map keepout navigation."""

import ast
from pathlib import Path

from geometry_msgs.msg import TransformStamped
from jdamr_cube_navigation.keepout_mask import build_mask, validate_mask
from jdamr_cube_navigation.keepout_zone_capture import zone_document
from jdamr_cube_navigation.tf_replay_filter import filter_tf_message
import pytest
from tf2_msgs.msg import TFMessage
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMS_PATH = PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
ZONES_EXAMPLE = PACKAGE_ROOT / 'config' / 'keepout_zones.example.yaml'
ZONES_AUTONOMOUS = (
    PACKAGE_ROOT / 'config' / 'keepout_zones.autonomous_20260826.yaml')
NAVIGATION_LAUNCH = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
KEEPOUT_LAUNCH = PACKAGE_ROOT / 'launch' / 'keepout_navigation.launch.py'
CAPTURE_LAUNCH = PACKAGE_ROOT / 'launch' / 'keepout_capture.launch.py'
REPLAY_LAUNCH = PACKAGE_ROOT / 'launch' / 'offline_replay_guard.launch.py'
KEEPOUT_RVIZ = PACKAGE_ROOT / 'rviz' / 'keepout_navigation.rviz'
SETUP_PATH = PACKAGE_ROOT / 'setup.py'


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
    assert "'use_composition': 'False'" in navigation_source
    assert "'slam': 'False'" in navigation_source
    assert "'use_localization': 'True'" in navigation_source


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


def test_annotated_corridor_keepout_has_two_enabled_lower_branches():
    """Keep the two user-marked yellow branches reproducible in map frame."""
    config = yaml.safe_load(ZONES_AUTONOMOUS.read_text(encoding='utf-8'))
    zones = {zone['id']: zone for zone in config['zones']}

    assert config['map_yaml'].endswith(
        'maps/autonomous_20260826T161908.yaml')
    assert config['safety_margin_m'] == 0.55
    assert config['annotation_alignment']['matched_features'] >= 55
    assert config['verification']['expected_keepout_cells'] == 9984
    assert config['connectivity_checks'] == [{
        'id': 'main_corridor_left_to_right',
        'start': [-0.002, 0.0],
        'goal': [37.498, -4.3],
        'clearance_m': 0.25,
    }]
    assert set(zones) == {
        'yellow_left_lower_branch',
        'yellow_right_lower_branch',
    }
    assert all(zone['enabled'] for zone in zones.values())
    assert max(point[1] for point in zones[
        'yellow_left_lower_branch']['polygon']) <= -2.55
    assert max(point[1] for point in zones[
        'yellow_right_lower_branch']['polygon']) <= -6.4


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
