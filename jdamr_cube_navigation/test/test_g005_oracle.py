"""Hostile tests for the pure G005 oracle map state."""

import math

from frontier_policy_contract import build_layout, LAYOUT_SEEDS, reveal_scan

from g005_oracle import (
    clearance_m,
    occupancy_grid_metadata,
    OracleMapState,
    ROBOT_CIRCUMSCRIBED_RADIUS_M,
    world_pose_to_cell,
)

import pytest


def _cell_center(layout, index):
    cell_x = index % layout['width']
    cell_y = index // layout['width']
    resolution = layout['resolution_m_per_cell']
    origin_x, origin_y, yaw = layout['origin_m_rad']
    local_x = (cell_x + 0.5) * resolution
    local_y = (cell_y + 0.5) * resolution
    return (origin_x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
            origin_y + math.sin(yaw) * local_x + math.cos(yaw) * local_y)


@pytest.mark.parametrize('seed', LAYOUT_SEEDS)
def test_all_seed_oracles_are_deterministic_monotonic_and_reach_coverage(seed):
    layout = build_layout(seed)
    left = OracleMapState(layout)
    right = OracleMapState(build_layout(seed))
    start = layout['start_cell'][1] * layout['width'] + layout['start_cell'][0]
    path = [start, *left.reachable_cell_indices]
    previous_sequence = 0
    previous_revealed = 0
    for scan_index, index in enumerate(path):
        x_m, y_m = _cell_center(layout, index)
        left_update = left.update_scan(x_m, y_m, 0.0, scan_index)
        right_update = right.update_scan(x_m, y_m, 0.0, scan_index)
        assert left_update == right_update
        sample = left.coverage_sample(float(scan_index))
        assert left_update['map_sequence'] >= previous_sequence
        assert sample['revealed_reachable_cells'] >= previous_revealed
        previous_sequence = left_update['map_sequence']
        previous_revealed = sample['revealed_reachable_cells']
    assert left.observed == right.observed
    assert previous_revealed / left.reachable_denominator_cells >= 0.85


@pytest.mark.parametrize('seed', LAYOUT_SEEDS)
def test_unchanged_payload_does_not_increment_map_sequence(seed):
    layout = build_layout(seed)
    oracle = OracleMapState(layout)
    start = layout['start_cell'][1] * layout['width'] + layout['start_cell'][0]
    x_m, y_m = _cell_center(layout, start)
    first = oracle.update_scan(x_m, y_m, 0.0, 2)
    repeated = oracle.update_scan(x_m, y_m, 0.0, 2)
    assert first['map_changed'] is True
    assert repeated['map_changed'] is False
    assert repeated['map_sequence'] == first['map_sequence']
    assert repeated['map_payload_sha256'] == first['map_payload_sha256']


@pytest.mark.parametrize('seed', LAYOUT_SEEDS)
def test_metadata_and_start_clearance_match_gt_raster(seed):
    layout = build_layout(seed)
    metadata = occupancy_grid_metadata(layout)
    assert metadata == occupancy_grid_metadata(build_layout(seed))
    assert metadata['frame_id'] == 'map'
    assert metadata['width'] == layout['width']
    assert metadata['height'] == layout['height']
    assert metadata['resolution'] == layout['resolution_m_per_cell']
    start = layout['start_cell'][1] * layout['width'] + layout['start_cell'][0]
    x_m, y_m = _cell_center(layout, start)
    measured = clearance_m(layout, x_m, y_m)
    assert measured >= 0.0
    resolution = layout['resolution_m_per_cell']
    local_x = x_m - layout['origin_m_rad'][0]
    local_y = y_m - layout['origin_m_rad'][1]
    occupied_distances = []
    for index, value in enumerate(layout['data']):
        if value < 65:
            continue
        cell_x = index % layout['width']
        cell_y = index // layout['width']
        left = cell_x * resolution
        right = left + resolution
        bottom = cell_y * resolution
        top = bottom + resolution
        dx = max(left - local_x, 0.0, local_x - right)
        dy = max(bottom - local_y, 0.0, local_y - top)
        occupied_distances.append(math.hypot(dx, dy))
    expected = max(
        0.0, min(occupied_distances) - ROBOT_CIRCUMSCRIBED_RADIUS_M)
    assert measured == pytest.approx(expected)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf, True])
def test_pose_validation_rejects_nonfinite_and_boolean_values(bad):
    layout = build_layout(11)
    with pytest.raises(ValueError, match='finite'):
        world_pose_to_cell(layout, bad, 0.0)


def test_pose_and_state_fail_closed_for_outside_occupied_or_regression():
    layout = build_layout(11)
    with pytest.raises(ValueError, match='outside'):
        world_pose_to_cell(layout, -100.0, -100.0)
    occupied_x_m, occupied_y_m = _cell_center(layout, 0)
    with pytest.raises(ValueError, match='free space'):
        world_pose_to_cell(layout, occupied_x_m, occupied_y_m)
    oracle = OracleMapState(layout)
    start = layout['start_cell'][1] * layout['width'] + layout['start_cell'][0]
    x_m, y_m = _cell_center(layout, start)
    oracle.update_scan(x_m, y_m, 0.0, 3)
    with pytest.raises(ValueError, match='scan index regressed'):
        oracle.update_scan(x_m, y_m, 0.0, 2)
    next_cell = oracle.reachable_cell_indices[1]
    with pytest.raises(ValueError, match='changed pose'):
        oracle.update_scan(*_cell_center(layout, next_cell), 0.0, 3)
    with pytest.raises(ValueError, match='changed pose'):
        oracle.update_scan(x_m, y_m, 0.1, 3)
    oracle.coverage_sample(2.0)
    with pytest.raises(ValueError, match='elapsed time regressed'):
        oracle.coverage_sample(1.0)


def test_delay_clock_restarts_for_each_sensor_origin_cell():
    layout = build_layout(11)
    oracle = OracleMapState(layout)
    first_cell, second_cell = oracle.reachable_cell_indices[:2]
    first = oracle.update_scan(*_cell_center(layout, first_cell), 0.0, 0)
    second = oracle.update_scan(*_cell_center(layout, second_cell), 0.0, 1)
    assert first['pose_cell'] == first_cell
    assert second['pose_cell'] == second_cell
    direct_first = reveal_from_fresh_origin(layout, second_cell)
    assert oracle.observed == direct_first


def reveal_from_fresh_origin(layout, pose_cell):
    observed = [-1] * len(layout['data'])
    first_cell = OracleMapState(layout).reachable_cell_indices[0]
    observed = reveal_scan(layout, observed, first_cell, 0.0, 0)
    return reveal_scan(layout, observed, pose_cell, 0.0, 0)


def test_oracle_state_does_not_expose_or_retain_mutable_map_payloads():
    layout = build_layout(11)
    oracle = OracleMapState(layout)
    layout['data'][0] = 0
    first_copy = oracle.observed
    first_copy[0] = 100
    assert oracle.observed[0] == -1
    assert world_pose_to_cell(layout, *_cell_center(layout, 0)) == 0
    with pytest.raises(ValueError, match='free space'):
        oracle.update_scan(*_cell_center(layout, 0), 0.0, 0)


def test_layout_and_frame_metadata_reject_schema_drift():
    layout = build_layout(11)
    layout['data'] = layout['data'][:-1]
    with pytest.raises(ValueError, match='fields are invalid'):
        OracleMapState(layout)
    with pytest.raises(ValueError, match='frame ID'):
        occupancy_grid_metadata(build_layout(11), ' map ')
