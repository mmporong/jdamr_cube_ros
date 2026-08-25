"""Tests for the ROS-independent frontier extraction core."""

import math
import time

from jdamr_cube_navigation.frontier_core import (
    BlacklistEntry,
    FrontierConfig,
    FrontierCore,
    GridMap,
)

import pytest


def make_grid(rows, resolution=1.0, origin=(0.0, 0.0, 0.0)):
    """Build a GridMap from top-to-bottom-looking test rows."""
    return GridMap(
        width=len(rows[0]),
        height=len(rows),
        resolution=resolution,
        origin_x=origin[0],
        origin_y=origin[1],
        origin_yaw=origin[2],
        data=tuple(value for row in rows for value in row),
    )


def test_all_known_free_has_no_frontier():
    """Known free space without unknown neighbors has no frontier."""
    grid = make_grid([[0] * 5 for _ in range(5)])

    assert FrontierCore(FrontierConfig(clearance_m=0.0)).extract(
        grid, 2.5, 2.5) == []


def test_single_frontier_cluster_and_yaw_toward_unknown():
    """A free island produces one frontier facing surrounding unknowns."""
    grid = make_grid([
        [-1, -1, -1, -1, -1],
        [-1, 0, 0, 0, -1],
        [-1, 0, 0, 0, -1],
        [-1, 0, 0, 0, -1],
        [-1, -1, -1, -1, -1],
    ])
    candidates = FrontierCore(FrontierConfig(
        clearance_m=0.0,
        min_cluster_size=3,
    )).extract(grid, 2.5, 2.5)

    assert len(candidates) == 1
    assert grid.value(candidates[0].cell) == 0
    assert candidates[0].cluster_size == 8
    assert math.isfinite(candidates[0].yaw)


def test_single_right_unknown_counts_once():
    """One unknown directly right of one frontier contributes one gain."""
    grid = make_grid([[0, -1]])

    candidate = FrontierCore(FrontierConfig(
        clearance_m=0.0,
        min_cluster_size=1,
    )).extract(grid, 0.5, 0.5)[0]

    assert candidate.cell == (0, 0)
    assert candidate.information_gain == 1
    assert candidate.yaw == pytest.approx(0.0)


def test_occupied_clearance_rejects_frontier_region():
    """Frontiers inside configured obstacle clearance are rejected."""
    grid = make_grid([
        [100, 100, 100, 100, 100, 100, 100],
        [100, 0, 0, 0, -1, -1, 100],
        [100, 0, 0, 0, -1, -1, 100],
        [100, 0, 0, 0, -1, -1, 100],
        [100, 100, 100, 100, 100, 100, 100],
    ], resolution=0.2)

    candidates = FrontierCore(FrontierConfig(
        clearance_m=0.41,
        min_cluster_size=1,
    )).extract(grid, 0.5, 0.5)

    assert candidates == []


def test_unreachable_free_island_does_not_create_candidate():
    """Disconnected free cells never become navigation candidates."""
    grid = make_grid([
        [0, 0, 0, 100, -1, -1, -1],
        [0, 0, 0, 100, -1, 0, -1],
        [0, 0, 0, 100, -1, 0, -1],
        [0, 0, 0, 100, -1, -1, -1],
    ])

    candidates = FrontierCore(FrontierConfig(
        clearance_m=0.0,
        min_cluster_size=1,
    )).extract(grid, 1.5, 1.5)

    assert candidates == []


def test_rotated_origin_world_cell_round_trip():
    """Cell transforms round-trip with a translated, rotated origin."""
    grid = make_grid(
        [[0] * 8 for _ in range(6)],
        resolution=0.25,
        origin=(3.0, -2.0, math.pi / 3.0),
    )

    for cell in ((0, 0), (3, 2), (7, 5)):
        world = grid.cell_to_world(cell)
        assert grid.world_to_cell(*world) == cell


def test_minimum_cluster_size_rejects_n_minus_one_and_accepts_n():
    """The cluster-size threshold is inclusive at exactly N cells."""
    grid = make_grid([
        [-1, -1, -1],
        [0, 0, 0],
        [0, 0, 0],
    ])

    rejected = FrontierCore(FrontierConfig(
        clearance_m=0.0,
        min_cluster_size=4,
    )).extract(grid, 1.5, 2.5)
    accepted = FrontierCore(FrontierConfig(
        clearance_m=0.0,
        min_cluster_size=3,
    )).extract(grid, 1.5, 2.5)

    assert rejected == []
    assert len(accepted) == 1
    assert accepted[0].cluster_size == 3


def test_blacklist_expires_and_is_invalidated_by_new_map_generation():
    """Blacklist entries apply only before expiry in their map epoch."""
    grid = make_grid([
        [-1, -1, -1, -1, -1],
        [-1, 0, 0, 0, -1],
        [-1, 0, 0, 0, -1],
        [-1, 0, 0, 0, -1],
        [-1, -1, -1, -1, -1],
    ])
    core = FrontierCore(FrontierConfig(clearance_m=0.0))
    candidate = core.extract(grid, 2.5, 2.5)[0]
    entry = BlacklistEntry(
        x=candidate.x,
        y=candidate.y,
        radius_m=0.1,
        expires_at=10.0,
        generation=4,
    )

    assert core.extract(
        grid, 2.5, 2.5, blacklist=[entry], now=5.0,
        generation=4) == []
    assert core.extract(
        grid, 2.5, 2.5, blacklist=[entry], now=10.0,
        generation=4)
    assert core.extract(
        grid, 2.5, 2.5, blacklist=[entry], now=5.0,
        generation=5)


def test_ranking_is_deterministic_and_rewards_information_gain():
    """Repeated ranking is stable and rewards larger unknown boundaries."""
    grid = make_grid([
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, -1, 0, 0, 0, -1, -1, -1, 0],
        [0, 100, 100, 0, 0, -1, -1, -1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
    ])
    core = FrontierCore(FrontierConfig(
        clearance_m=0.0,
        min_cluster_size=1,
        information_gain_weight=10.0,
        distance_weight=0.01,
        heading_weight=0.0,
    ))

    first = core.extract(grid, 0.5, 3.5)
    second = core.extract(grid, 0.5, 3.5)

    assert first == second
    assert len(first) >= 2
    assert first[0].information_gain == max(
        item.information_gain for item in first)
    assert first[0].score > first[-1].score


@pytest.mark.parametrize('barrier', [50, 100])
def test_diagonal_corner_squeeze_is_not_reachable(barrier):
    """Neither ambiguous nor occupied diagonal gaps permit corner cuts."""
    grid = make_grid([
        [0, barrier, 0],
        [barrier, 0, -1],
        [0, 0, -1],
    ])

    candidates = FrontierCore(FrontierConfig(
        clearance_m=0.0,
        min_cluster_size=1,
    )).extract(grid, 0.5, 0.5)

    assert candidates == []


def test_ambiguous_value_is_clearance_source_and_rejects_close_frontier():
    """Known ambiguous occupancy contributes finite unsafe clearance."""
    grid = make_grid([
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 50, -1],
    ])
    core = FrontierCore(FrontierConfig(
        clearance_m=0.6,
        min_cluster_size=1,
    ))

    distances = core._occupied_distances(grid, 0.6)

    assert math.isfinite(core._clearance_cells(distances[grid.index((2, 1))]))
    assert core._clearance_cells(distances[grid.index((2, 1))]) == 0.5
    assert core.extract(grid, 0.5, 0.5) == []


def test_candidate_representative_satisfies_all_grid_postconditions():
    """Every representative remains safe, reachable, free, and reversible."""
    grid = make_grid([
        [-1, -1, -1, -1, -1],
        [-1, 0, 0, 0, -1],
        [-1, 0, 0, 0, -1],
        [-1, 0, 0, 0, -1],
        [-1, -1, -1, -1, -1],
    ], resolution=0.2, origin=(1.0, -3.0, 0.4))
    config = FrontierConfig(clearance_m=0.2, min_cluster_size=1)
    core = FrontierCore(config)
    robot_world = grid.cell_to_world((2, 2))

    candidates = core.extract(grid, *robot_world)
    distances = core._occupied_distances(
        grid, config.clearance_m / grid.resolution)
    reached, _ = core._reachable(
        grid, (2, 2), distances, config.clearance_m / grid.resolution)

    assert candidates
    for candidate in candidates:
        index = grid.index(candidate.cell)
        assert core.is_free(grid.data[index])
        assert reached[index]
        assert candidate.clearance_m >= config.clearance_m
        assert grid.world_to_cell(candidate.x, candidate.y) == candidate.cell


def test_rotated_origin_candidate_faces_unknown_centroid():
    """Candidate yaw includes the map origin rotation."""
    grid = make_grid([
        [0, 0, -1],
        [0, 0, -1],
        [0, 0, -1],
    ], origin=(2.0, -1.0, math.pi / 2.0))
    robot_world = grid.cell_to_world((0, 1))

    candidate = FrontierCore(FrontierConfig(
        clearance_m=0.0,
        min_cluster_size=3,
    )).extract(grid, *robot_world)[0]

    assert candidate.yaw == pytest.approx(math.pi / 2.0)


def test_exact_score_ties_use_row_major_cell_order():
    """Exact utility ties resolve by stable row-major representative index."""
    grid = make_grid([
        [0, 0, 0, 0, 0],
        [0, -1, 100, -1, 0],
        [0, 100, 100, 100, 0],
        [0, 0, 0, 0, 0],
    ])
    core = FrontierCore(FrontierConfig(
        clearance_m=0.0,
        min_cluster_size=1,
        information_gain_weight=0.0,
        distance_weight=0.0,
        heading_weight=0.0,
    ))

    candidates = core.extract(grid, 0.5, 3.5)
    indices = [grid.index(candidate.cell) for candidate in candidates]

    assert len(indices) >= 2
    assert indices == sorted(indices)


def test_clusters_stream_flat_indices_without_consuming_later_clusters():
    """Cluster extraction yields one flat-index cluster before scanning on."""
    grid = make_grid([[0, 0, 0, 0, 0]])
    remaining = bytearray((1, 1, 0, 1, 0))
    clusters = FrontierCore()._clusters(grid, remaining)

    first = next(clusters)

    assert first == [0, 1]
    assert all(isinstance(index, int) for index in first)
    assert remaining[3] == 1
    assert next(clusters) == [3]
    with pytest.raises(StopIteration):
        next(clusters)


def test_candidate_reuses_and_clears_unknown_marker_storage():
    """Many clusters share one marker/touched pair without leaking marks."""
    class RecordingCore(FrontierCore):
        def __init__(self, config):
            super().__init__(config)
            self.marker_ids = []
            self.touched_ids = []
            self.marker = None
            self.touched = None

        def _candidate(self, *args):
            self.marker = args[5]
            self.touched = args[6]
            self.marker_ids.append(id(self.marker))
            self.touched_ids.append(id(self.touched))
            return super()._candidate(*args)

    grid = make_grid([
        [0, 0, 0, 0, 0],
        [0, -1, 0, -1, 0],
        [0, 0, 0, 0, 0],
    ])
    core = RecordingCore(FrontierConfig(
        clearance_m=0.0,
        min_cluster_size=1,
    ))

    candidates = core.extract(grid, 0.5, 0.5)

    assert len(candidates) > 1
    assert len(set(core.marker_ids)) == 1
    assert len(set(core.touched_ids)) == 1
    assert not any(core.marker)
    assert core.touched == []


def test_dense_isolated_unknown_extraction_has_bounded_runtime():
    """Thousands of singleton frontiers avoid per-cluster allocation blowup."""
    width = height = 180
    data = [0] * (width * height)
    for y in range(1, height, 2):
        for x in range(1, width, 2):
            data[y * width + x] = -1
    grid = GridMap(width, height, 0.05, 0.0, 0.0, 0.0, data)
    core = FrontierCore(FrontierConfig(
        clearance_m=0.0,
        min_cluster_size=1,
    ))

    started = time.perf_counter()
    candidates = core.extract(grid, 0.025, 0.025)
    elapsed = time.perf_counter() - started

    assert len(candidates) == 16200
    # Measured near 0.10 s locally; generous for contended shared CI while
    # still catching the former all-clusters/Cell-tuples/per-cluster-set path.
    assert elapsed < 1.5


def test_250_square_extraction_has_practical_linear_time_ceiling():
    """A representative grid stays well below the former heap runtime."""
    width = height = 250
    data = [0] * (width * height)
    data[:width] = [-1] * width
    for y in range(20, height, 53):
        for x in range(20, width, 47):
            data[y * width + x] = 100
    grid = GridMap(width, height, 0.05, 0.0, 0.0, 0.0, data)
    core = FrontierCore(FrontierConfig(
        clearance_m=0.25,
        min_cluster_size=3,
    ))

    started = time.perf_counter()
    core.extract(grid, 6.275, 6.275)
    elapsed = time.perf_counter() - started

    # Generous enough for shared CI; catches the former O(N log N) tuple/heap
    # implementation, which took more than four seconds on this machine.
    assert elapsed < 2.5


def test_grid_rejects_malformed_data():
    """Grid construction rejects storage with the wrong cell count."""
    with pytest.raises(ValueError):
        GridMap(2, 2, 1.0, 0.0, 0.0, 0.0, [0])


def test_start_safety_is_known_free_inside_and_matches_clearance_math():
    """The local start gate matches the extraction clearance convention."""
    grid = make_grid([
        [0, 0, 0, 0, 0],
        [0, 0, 50, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ])
    core = FrontierCore(FrontierConfig(clearance_m=1.0))
    clearance = core._occupied_distances(grid, 1.0)

    assert not core.start_is_safe(grid, -0.5, 0.5)
    assert not core.start_is_safe(grid, 2.5, 1.5)
    assert not core.start_is_safe(grid, 2.5, 2.5)
    assert core.start_is_safe(grid, 2.5, 3.5)
    for y in range(grid.height):
        for x in range(grid.width):
            expected = (
                core.is_free(grid.value((x, y))) and
                core._clearance_cells(clearance[grid.index((x, y))]) >= 1.0)
            assert core.start_is_safe(grid, x + 0.5, y + 0.5) == expected


def test_safe_start_continues_normal_frontier_extraction():
    """The preflight gate preserves extraction from a valid free start."""
    grid = make_grid([
        [-1, -1, -1, -1, -1],
        [-1, 0, 0, 0, -1],
        [-1, 0, 0, 0, -1],
        [-1, 0, 0, 0, -1],
        [-1, -1, -1, -1, -1],
    ])
    core = FrontierCore(FrontierConfig(
        clearance_m=0.0, min_cluster_size=1))
    assert core.start_is_safe(grid, 2.5, 2.5)
    assert core.extract(grid, 2.5, 2.5)
