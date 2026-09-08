"""Tests for visual-only JDAMR portfolio capture assets."""

import hashlib
import importlib.util
import json
import sys
import xml.etree.ElementTree as ET
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


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_capture_world_adds_people_without_changing_collisions(tmp_path):
    """Keep the canonical world exact while adding visual-only people."""
    source = ASSETS / 'slam_corridor_contact.world'
    source_hash = _sha256(source)
    output = tmp_path / 'capture.world'

    report = CAPTURE.build_capture_world(source, output)
    world = ET.parse(output).getroot().find('world')
    person = world.find(
        "./model[@name='g003_preloaded_front_observation_probe']/link")

    assert _sha256(source) == source_hash
    assert report['collision_xml_unchanged'] is True
    north_wall = world.find("./model[@name='wall_north']")
    assert north_wall.findtext('.//visual/material/diffuse') == (
        '0.31 0.38 0.46 1')
    proxy_flags = person.find(
        "visual[@name='lidar_proxy']/visibility_flags")
    assert proxy_flags.text == '4'
    assert len([visual for visual in person.findall('visual')
                if visual.attrib['name'].startswith('person_')]) == 4
    for name in ('portfolio_pedestrian_left',
                 'portfolio_pedestrian_right'):
        actor = world.find(f"./model[@name='{name}']")
        assert actor is not None
        assert actor.findall('.//collision') == []


def test_capture_urdf_hides_arm_visuals_without_changing_physics(tmp_path):
    """Remove SO-101 from camera view, not from evaluation dynamics."""
    source = ASSETS / 'jdamr_cube_nav_eval.urdf'
    source_hash = _sha256(source)
    output = tmp_path / 'capture.urdf'

    report = CAPTURE.build_capture_urdf(source, output)
    robot = ET.parse(output).getroot()

    assert _sha256(source) == source_hash
    assert report['hidden_visual_count'] > 0
    assert report['collision_xml_unchanged'] is True
    assert report['inertial_xml_unchanged'] is True
    assert report['joint_xml_unchanged'] is True
    camera = robot.find(
        "./gazebo[@reference='base_footprint']/sensor"
        "[@name='portfolio_camera']")
    pose = [float(value) for value in camera.findtext('pose').split()]
    assert report['camera']['view'] == 'robot_follow_oblique_3d_corridor'
    assert pose[0] < -0.5
    assert 0.4 < pose[2] < 1.2
    assert 0.2 < pose[4] < 1.0
    assert camera.findtext('update_rate') == '30.0'
    assert camera.findtext('camera/visibility_mask') == '1'
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
    ]}
    metadata = {
        'first_frame_steady_ns': 1_000_000_000,
        'last_frame_steady_ns': 6_000_000_000,
    }

    assert RENDER._event_frame_interval(evidence, metadata, 101) == (20, 60)


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
