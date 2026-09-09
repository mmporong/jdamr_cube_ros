"""Tests for visual-only JDAMR portfolio capture assets."""

import hashlib
import importlib.util
import json
import sys
import xml.etree.ElementTree as ET

import pytest
from pathlib import Path


EVALUATION = Path(__file__).resolve().parents[1] / 'evaluation'
ASSETS = EVALUATION / 'assets/nav_obstacle'
sys.path.insert(0, str(EVALUATION))
SPEC = importlib.util.spec_from_file_location(
    'portfolio_capture_world', EVALUATION / 'portfolio_capture_world.py')
CAPTURE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CAPTURE)
RENDER_SPEC = importlib.util.spec_from_file_location(
    'render_simulator_portfolio_media',
    EVALUATION / 'render_simulator_portfolio_media.py')
RENDER = importlib.util.module_from_spec(RENDER_SPEC)
RENDER_SPEC.loader.exec_module(RENDER)
REEL_SPEC = importlib.util.spec_from_file_location(
    'render_bidirectional_reel',
    EVALUATION / 'render_bidirectional_reel.py')
REEL = importlib.util.module_from_spec(REEL_SPEC)
REEL_SPEC.loader.exec_module(REEL)


def test_historical_chase_camera_cannot_pass_fixed_view_contract():
    """New doorway video cannot be claimed from the old chase-camera source."""
    with pytest.raises(RuntimeError, match='fixed-camera doorway capture required'):
        RENDER._validate_capture_geometry({'simulator_video': {'capture_world': {
            'collision_xml_unchanged': True}}})


def test_rotation_is_not_reported_as_a_zero_velocity_command():
    """A turning robot must not acquire a commanded-stop badge."""
    frames = {'stop_state': 10, 'clear_set_pose_requested': 30}
    text, _, reason = RENDER._observation(20, frames, True, 0.0, 0.3)
    assert '/cmd_vel 0' not in reason
    assert '정지 명령' not in text


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_capture_world_adds_fixed_camera_and_physical_doorway(tmp_path):
    """Align the pedestrian doorway across collision and camera geometry."""
    source = ASSETS / 'slam_corridor_contact.world'
    source_hash = _sha256(source)
    output = tmp_path / 'capture.world'

    report = CAPTURE.build_capture_world(source, output)
    world = ET.parse(output).getroot().find('world')
    person = world.find(
        "./model[@name='g003_preloaded_front_observation_probe']/link")

    assert _sha256(source) == source_hash
    assert report['collision_xml_unchanged'] is False
    assert report['collision_change_scope'] == (
        'corridor_edge_pedestrian_doorways')
    doorway = report['pedestrian_doorway']
    assert doorway['walls'] == ['wall_north', 'wall_south']
    assert doorway['supported_entry_sides'] == ['left', 'right']
    assert doorway['crossing_edge_y_m'] == 1.5
    for wall_name in doorway['walls']:
        wall = world.find(f"./model[@name='{wall_name}']")
        collisions = wall.findall('.//collision')
        visuals = wall.findall('.//visual')
        assert [item.attrib['name'] for item in collisions] == [
            'door_west_collision', 'door_east_collision']
        assert [item.attrib['name'] for item in visuals] == [
            'door_west_visual', 'door_east_visual']
        for collision, visual in zip(collisions, visuals):
            assert collision.findtext('pose') == visual.findtext('pose')
            assert collision.findtext('geometry/box/size') == (
                visual.findtext('geometry/box/size'))
            assert visual.findtext('visibility_flags') == '4'
        west_center_x_m = float(collisions[0].findtext('pose').split()[0])
        west_length_m = float(
            collisions[0].findtext('geometry/box/size').split()[0])
        east_center_x_m = float(collisions[1].findtext('pose').split()[0])
        east_length_m = float(
            collisions[1].findtext('geometry/box/size').split()[0])
        west_edge_x_m = west_center_x_m + west_length_m / 2
        east_edge_x_m = east_center_x_m - east_length_m / 2
        assert abs(west_edge_x_m - (
            doorway['center_x_m'] - doorway['width_m'] / 2)) < 1e-9
        assert abs(east_edge_x_m - (
            doorway['center_x_m'] + doorway['width_m'] / 2)) < 1e-9

    north_wall = world.find("./model[@name='wall_north']")
    assert north_wall.findtext('.//visual/material/diffuse') == (
        '0.31 0.38 0.46 1')
    proxy_flags = person.find(
        "visual[@name='lidar_proxy']/visibility_flags")
    assert proxy_flags.text == '4'
    assert len([visual for visual in person.findall('visual')
                if visual.attrib['name'].startswith('person_')]) >= 10
    shell = world.find("./model[@name='portfolio_architectural_shell']")
    assert shell is not None
    assert shell.findall('.//collision') == []
    assert shell.find(".//visual[@name='north_wall_west']") is not None
    assert shell.find(".//visual[@name='north_wall_east']") is not None
    assert shell.find(".//visual[@name='north_wall']") is None
    assert shell.find(".//visual[@name='south_wall']") is None
    assert all(light.findtext('cast_shadows') == 'false'
               for light in world.findall('light')
               if light.attrib['name'].startswith('portfolio_'))
    route = world.find(
        "./model[@name='g003_preloaded_route_obstacle']/link")
    assert route.find("visual[@name='crate_body']") is not None
    route_flags = route.find(
        "visual[@name='lidar_proxy']/visibility_flags")
    assert route_flags.text == '4'
    assert world.findtext('scene/shadows') == 'true'
    camera_model = world.find("./model[@name='portfolio_scene_camera']")
    camera = camera_model.find(
        "./link/sensor[@name='portfolio_camera']")
    assert report['camera']['view'] == 'fixed_world_oblique_full_route'
    assert camera.findtext('topic') == '/portfolio_scene/image_raw'
    assert camera.findtext('camera/horizontal_fov') == '1.18'
    model_pose = [float(value) for value in
                  camera_model.findtext('pose').split()]
    sensor_pose = [float(value) for value in camera.findtext('pose').split()]
    assert model_pose[:3] == [-0.5, -11.0, 5.6]
    assert sensor_pose[4:] == [0.43, 1.5708]


def test_capture_urdf_hides_arm_visuals_without_follow_camera(tmp_path):
    """Remove upper visuals and leave camera ownership to the fixed world."""
    source = ASSETS / 'jdamr_cube_nav_eval.urdf'
    source_hash = _sha256(source)
    output = tmp_path / 'capture.urdf'

    report = CAPTURE.build_capture_urdf(source, output)
    robot = ET.parse(output).getroot()

    assert _sha256(source) == source_hash
    assert report['hidden_visual_count'] > 0
    assert report['styled_navigation_link_count'] == 6
    assert report['collision_xml_unchanged'] is True
    assert report['inertial_xml_unchanged'] is True
    assert report['joint_xml_unchanged'] is True
    assert robot.find(".//sensor[@name='portfolio_camera']") is None
    assert robot.find("./link[@name='base_link']/visual") is not None
    assert all(
        not link.findall('visual')
        for link in robot.findall('link')
        if link.attrib.get('name', '').startswith(('arm_', 'rgbd_')))


def test_verified_event_clock_maps_to_camera_frames():
    """Media timing follows evidence events instead of color heuristics."""
    evidence = {'events': [
        {'name': 'obstacle_crossing_started', 'steady_ns': 2_000_000_000},
        {'name': 'clear_set_pose_requested', 'steady_ns': 4_000_000_000},
        {'name': 'obstacle_crossing_exit_completed',
         'steady_ns': 5_000_000_000},
    ]}
    metadata = {
        'first_frame_steady_ns': 1_000_000_000,
        'last_frame_steady_ns': 6_000_000_000,
    }

    assert RENDER._event_frame_interval(evidence, metadata, 101) == (20, 80)


def test_irregular_capture_uses_recorded_frame_times():
    """Use actual frame timestamps when camera delivery is irregular."""
    metadata = {'frame_timestamps': [
        {'steady_ns': value, 'wall_ns': value + 1000}
        for value in (100, 110, 120, 900)]}
    evidence = {'events': [{'name': 'stop_state', 'steady_ns': 115}]}
    assert RENDER._event_frames(evidence, metadata, 4)['stop_state'] == 2
    assert RENDER._frame_wall_ns(metadata, 2, 4) == 1120


def test_crossing_does_not_claim_detection_or_resume_before_motion():
    """Keep overlay labels behind recorded crossing and command state."""
    frames = {'obstacle_crossing_started': 10, 'stop_state': 20,
              'clear_set_pose_requested': 30,
              'obstacle_crossing_exit_completed': 40}
    assert RENDER._state(15, frames, False, 0.18)[0] == '보행자 횡단 중'
    assert RENDER._state(35, frames, False, 0.0)[0] == '정지 유지'


def test_real_scan_transform_rejects_invalid_returns():
    """Exclude non-finite and out-of-range LiDAR returns."""
    import numpy as np
    scan = (0, 0.0, 0.1, 0.1, 5.0, np.array([1.0, np.inf, 0.0]))
    points = RENDER._scan_world_points(scan, (0, 2.0, 1.0, 0.0))
    np.testing.assert_allclose(points, [[1.0, 1.0]], atol=1e-6)


def test_localization_overlay_never_uses_future_pose():
    """Use only localization samples available at the rendered time."""
    poses = [(100, 0.0, 0.0, 0.0), (200, 10.0, 0.0, 0.0)]
    assert RENDER._sample_pose(poses, 99) is None
    assert RENDER._sample_pose(poses, 150) == poses[0]


def test_map_overlay_preserves_metric_aspect_ratio():
    """Render equal map distances with equal pixel lengths."""
    project, _ = RENDER._map_transform(RENDER.MAP_RECT, 0.0)
    origin = project(0.0, 0.0)
    x_axis, y_axis = project(1.0, 0.0), project(0.0, 1.0)
    assert abs((x_axis[0] - origin[0]) - (origin[1] - y_axis[1])) <= 1


def test_bidirectional_source_requires_passing_hashed_highlight(tmp_path):
    """Only concatenate media tied to a passing directional manifest."""
    video = tmp_path / REEL.HIGHLIGHT_NAME
    video.write_bytes(b'verified-video')
    manifest_path = tmp_path / 'portfolio_media_manifest.json'
    manifest_path.write_text(json.dumps({
        'status': 'PASS',
        'verified_metrics': {'entry_side': 'left', 'contact_count': 0},
        'outputs': [{
            'path': str(video),
            'sha256': REEL._sha256(video),
        }],
    }), encoding='utf-8')

    source = REEL._load_directional_source(manifest_path)

    assert source['direction'] == 'left'
    assert source['video_sha256'] == REEL._sha256(video)
