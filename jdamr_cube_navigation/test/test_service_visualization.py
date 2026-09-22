"""Ensure display-only assets cannot masquerade as taught table poses."""

from copy import deepcopy

from jdamr_cube_navigation.service_visualization import build_markers
import pytest
from visualization_msgs.msg import Marker


@pytest.fixture
def assets():
    """Provide synthetic provenance and one measured home pose."""
    return ({
        'frame_id': 'map', 'map': {'image_sha256': 'abc'}, 'tables': [],
        'home': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0,
                 'parking_direction': 'reverse'},
    }, {'source_map_sha256': 'abc', 'markers': {
        'table_1': {'approximate_map_xy_m': [1.0, 2.0]},
        'table_2': {'approximate_map_xy_m': [3.0, 4.0]},
    }})


def test_approximate_regions_have_no_docking_arrows(assets):
    registry, annotation = assets
    original = deepcopy(registry)
    markers = build_markers(registry, annotation).markers
    assert markers[0].action == Marker.DELETEALL
    assert registry == original
    assert [m.id for m in markers if m.type == Marker.ARROW] == [100]
    areas = [m for m in markers if m.type == Marker.CYLINDER]
    assert [(m.pose.position.x, m.pose.position.y) for m in areas] == [(1, 2), (3, 4)]
    assert any('TABLE_1_SELECTED' in m.text and 'AREA_ONLY' in m.text for m in markers)
    assert all(m.header.frame_id == 'map' for m in markers)


def test_wrong_map_annotation_rejected(assets):
    registry, annotation = assets
    annotation['source_map_sha256'] = 'other'
    with pytest.raises(ValueError, match='different map'):
        build_markers(registry, annotation)


def test_taught_pose_gains_heading_arrow(assets):
    registry, annotation = assets
    registry['tables'] = [{'table_id': 'table_01', 'service_poses': [
        {'x_m': 1.2, 'y_m': 2.2, 'yaw_rad': 1.5707963267948966}]}]
    markers = build_markers(registry, annotation).markers
    arrow = next(m for m in markers if m.id == 12)
    assert arrow.pose.orientation.z == pytest.approx(2**-0.5)
    assert arrow.pose.position.x == 1.2
    assert any('DOCK_POSE_TAUGHT' in m.text for m in markers)


def test_republish_removes_old_taught_pose(assets):
    registry, annotation = assets
    registry['tables'] = [{'table_id': 'table_01', 'service_poses': [
        {'x_m': 1.2, 'y_m': 2.2, 'yaw_rad': 0.0}]}]
    cache = {}
    for message in (build_markers(registry, annotation), build_markers(
            {**registry, 'tables': [], 'home': None}, annotation)):
        for item in message.markers:
            if item.action == Marker.DELETEALL:
                cache.clear()
            else:
                cache[(item.ns, item.id)] = item
    assert not any(m.type == Marker.ARROW for m in cache.values())
