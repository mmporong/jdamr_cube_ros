"""
ROS-independent frontier extraction for autonomous map exploration.

The implementation deliberately uses only the Python standard library so it
can run and be tested on the robot without NumPy or SciPy.  All full-grid
operations are O(N), use flat row-major storage, and avoid per-cell objects.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, Iterator, Optional, Sequence, Tuple


Cell = Tuple[int, int]


@dataclass(frozen=True)
class GridMap:
    """Minimal, immutable equivalent of the useful OccupancyGrid fields."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Sequence[int]

    def __post_init__(self) -> None:
        """Validate dimensions, scale, and backing storage."""
        if self.width <= 0 or self.height <= 0:
            raise ValueError('grid dimensions must be positive')
        if self.resolution <= 0.0:
            raise ValueError('grid resolution must be positive')
        if len(self.data) != self.width * self.height:
            raise ValueError('grid data length does not match dimensions')

    def contains(self, cell: Cell) -> bool:
        """Return whether ``cell`` lies inside the grid."""
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def index(self, cell: Cell) -> int:
        """Return the row-major index for an in-bounds cell."""
        if not self.contains(cell):
            raise IndexError(f'cell outside grid: {cell}')
        return cell[1] * self.width + cell[0]

    def value(self, cell: Cell) -> int:
        """Return the occupancy value for ``cell``."""
        return self.data[self.index(cell)]

    def cell_to_world(self, cell: Cell) -> Tuple[float, float]:
        """Convert a cell center to world coordinates, including origin yaw."""
        if not self.contains(cell):
            raise IndexError(f'cell outside grid: {cell}')
        local_x = (cell[0] + 0.5) * self.resolution
        local_y = (cell[1] + 0.5) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )

    def world_to_cell(self, x: float, y: float) -> Optional[Cell]:
        """Convert a world point to its cell, or return ``None`` if outside."""
        delta_x = x - self.origin_x
        delta_y = y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        cell = (
            math.floor(local_x / self.resolution),
            math.floor(local_y / self.resolution),
        )
        return cell if self.contains(cell) else None


@dataclass(frozen=True)
class FrontierConfig:
    """Thresholds and deterministic utility weights for extraction."""

    # Cartographer의 관측 경계는 미관측(-1) 바로 안쪽에 49~50을 만든다.
    free_threshold: int = 50
    occupied_threshold: int = 65
    # 후보 전처리 여유만 둔다. 차체 외곽과 최종 여유는 Nav2 costmap이
    # URDF 기반 footprint로 검증하므로 여기서 차체 반경을 다시 더하지 않는다.
    clearance_m: float = 0.10
    min_cluster_size: int = 3
    information_gain_weight: float = 1.0
    distance_weight: float = 0.20
    heading_weight: float = 0.10

    def __post_init__(self) -> None:
        """Validate extraction thresholds and weights."""
        if self.free_threshold < 0:
            raise ValueError('free_threshold must be non-negative')
        if self.occupied_threshold <= self.free_threshold:
            raise ValueError('occupied_threshold must exceed free_threshold')
        if self.clearance_m < 0.0:
            raise ValueError('clearance_m must be non-negative')
        if self.min_cluster_size <= 0:
            raise ValueError('min_cluster_size must be positive')


@dataclass(frozen=True)
class BlacklistEntry:
    """A failed world-space goal region with time and map-generation expiry."""

    x: float
    y: float
    radius_m: float
    expires_at: float
    generation: int

    def active(self, now: float, generation: int) -> bool:
        """Return whether the entry applies to this map at ``now``."""
        return (
            self.radius_m >= 0.0
            and now < self.expires_at
            and generation == self.generation
        )

    def contains(self, x: float, y: float, now: float,
                 generation: int) -> bool:
        """Return whether an active entry covers a world-space point."""
        return self.active(now, generation) and math.hypot(
            x - self.x, y - self.y) <= self.radius_m


@dataclass(frozen=True)
class FrontierCandidate:
    """A safe known-free navigation goal derived from one frontier cluster."""

    cell: Cell
    x: float
    y: float
    yaw: float
    cluster_size: int
    information_gain: int
    path_distance_m: float
    clearance_m: float
    heading_change: float
    score: float


class FrontierCore:
    """Extract and deterministically rank safe reachable frontier goals."""

    def __init__(self, config: Optional[FrontierConfig] = None) -> None:
        """Create a core using ``config`` or conservative defaults."""
        self.config = config or FrontierConfig()

    def is_free(self, value: int) -> bool:
        """Return whether an occupancy value is known free."""
        return 0 <= value <= self.config.free_threshold

    def is_occupied(self, value: int) -> bool:
        """Return whether an occupancy value is occupied."""
        return value >= self.config.occupied_threshold

    @staticmethod
    def is_unknown(value: int) -> bool:
        """Return whether an occupancy value is unknown."""
        return value < 0

    def extract(
        self,
        grid: GridMap,
        robot_x: float,
        robot_y: float,
        robot_yaw: float = 0.0,
        blacklist: Iterable[BlacklistEntry] = (),
        now: float = 0.0,
        generation: int = 0,
    ) -> list[FrontierCandidate]:
        """
        Return safe candidates sorted by descending deterministic utility.

        The robot start must lie in known-free space.  Reachability, unknown
        adjacency, and frontier clustering use conservative 4-connectivity.
        Clearance is a lower bound to the nearest occupied-cell boundary.
        Ambiguous cells remain non-traversable but do not grow a false
        obstacle halo around otherwise free robot poses.
        """
        if not self.start_is_safe(grid, robot_x, robot_y):
            return []
        start = grid.world_to_cell(robot_x, robot_y)
        assert start is not None

        min_clearance_cells = self.config.clearance_m / grid.resolution
        clearance = self._occupied_distances(grid, min_clearance_cells)
        reachable, steps = self._reachable(
            grid, start, clearance, min_clearance_cells)
        frontier = self._frontier_flags(grid, reachable)
        entries = tuple(blacklist)
        candidates = []
        unknown_seen = bytearray(len(grid.data))
        unknown_touched: list[int] = []
        origin_cosine = math.cos(grid.origin_yaw)
        origin_sine = math.sin(grid.origin_yaw)
        for cluster in self._clusters(grid, frontier):
            if len(cluster) < self.config.min_cluster_size:
                continue
            candidate = self._candidate(
                grid, cluster, steps, clearance, robot_yaw,
                unknown_seen, unknown_touched, origin_cosine, origin_sine)
            if any(entry.contains(candidate.x, candidate.y, now, generation)
                   for entry in entries):
                continue
            candidates.append(candidate)

        candidates.sort(key=lambda item: (
            -item.score, grid.index(item.cell)))
        return candidates

    def start_is_safe(self, grid: GridMap, robot_x: float,
                      robot_y: float) -> bool:
        """Check known-free start clearance using a bounded local window."""
        start = grid.world_to_cell(robot_x, robot_y)
        if start is None or not self.is_free(grid.value(start)):
            return False
        minimum = self.config.clearance_m / grid.resolution
        radius = math.ceil(minimum + 0.5)
        start_x, start_y = start
        min_x = max(0, start_x - radius)
        max_x = min(grid.width - 1, start_x + radius)
        min_y = max(0, start_y - radius)
        max_y = min(grid.height - 1, start_y + radius)
        for y in range(min_y, max_y + 1):
            row = y * grid.width
            for x in range(min_x, max_x + 1):
                value = grid.data[row + x]
                if value < self.config.occupied_threshold:
                    continue
                center_distance = max(abs(x - start_x), abs(y - start_y))
                if self._clearance_cells(center_distance) < minimum:
                    return False
        return True

    def _occupied_distances(
        self, grid: GridMap, minimum_clearance: float = 0.0,
    ) -> list[int]:
        """
        Return capped Chebyshev center distances to occupied cells.

        Propagation stops after the distance needed for the configured safety
        decision.  The sentinel therefore means "at least this far", not
        infinity, and remains a conservative lower bound.  Eight-way spreading
        computes Chebyshev distance in O(N); reachability itself stays 4-way.
        """
        propagation_limit = max(1, math.ceil(minimum_clearance + 0.5))
        sentinel = propagation_limit + 1
        size = len(grid.data)
        distances = [sentinel] * size
        queue = deque()
        occupied_threshold = self.config.occupied_threshold
        for index, value in enumerate(grid.data):
            if value >= occupied_threshold:
                distances[index] = 0
                queue.append(index)

        width = grid.width
        last_row = size - width
        while queue:
            index = queue.popleft()
            distance = distances[index]
            if distance >= propagation_limit:
                continue
            next_distance = distance + 1
            x = index % width
            if index >= width:
                upper = index - width
                if distances[upper] > next_distance:
                    distances[upper] = next_distance
                    queue.append(upper)
                if x and distances[upper - 1] > next_distance:
                    distances[upper - 1] = next_distance
                    queue.append(upper - 1)
                if x + 1 < width and distances[upper + 1] > next_distance:
                    distances[upper + 1] = next_distance
                    queue.append(upper + 1)
            if index < last_row:
                lower = index + width
                if distances[lower] > next_distance:
                    distances[lower] = next_distance
                    queue.append(lower)
                if x and distances[lower - 1] > next_distance:
                    distances[lower - 1] = next_distance
                    queue.append(lower - 1)
                if x + 1 < width and distances[lower + 1] > next_distance:
                    distances[lower + 1] = next_distance
                    queue.append(lower + 1)
            if x and distances[index - 1] > next_distance:
                distances[index - 1] = next_distance
                queue.append(index - 1)
            if x + 1 < width and distances[index + 1] > next_distance:
                distances[index + 1] = next_distance
                queue.append(index + 1)
        return distances

    def _reachable(
        self,
        grid: GridMap,
        start: Cell,
        clearance: Sequence[float],
        minimum_clearance: float,
    ) -> Tuple[bytearray, list[int]]:
        size = len(grid.data)
        reached = bytearray(size)
        steps = [-1] * size
        start_index = start[1] * grid.width + start[0]
        if self._clearance_cells(clearance[start_index]) < minimum_clearance:
            return reached, steps
        reached[start_index] = 1
        steps[start_index] = 0
        queue = deque([start_index])
        width = grid.width
        last_row = size - width
        data = grid.data
        free_threshold = self.config.free_threshold
        while queue:
            index = queue.popleft()
            next_step = steps[index] + 1
            x = index % width
            if index >= width:
                self._enqueue_free(
                    index - width, next_step, data, clearance,
                    minimum_clearance, free_threshold, reached, steps, queue)
            if x:
                self._enqueue_free(
                    index - 1, next_step, data, clearance,
                    minimum_clearance, free_threshold, reached, steps, queue)
            if x + 1 < width:
                self._enqueue_free(
                    index + 1, next_step, data, clearance,
                    minimum_clearance, free_threshold, reached, steps, queue)
            if index < last_row:
                self._enqueue_free(
                    index + width, next_step, data, clearance,
                    minimum_clearance, free_threshold, reached, steps, queue)
        return reached, steps

    def _frontier_flags(
        self, grid: GridMap, reachable: bytearray,
    ) -> bytearray:
        frontier = bytearray(len(grid.data))
        width = grid.width
        size = len(grid.data)
        last_row = size - width
        data = grid.data
        for index, is_reachable in enumerate(reachable):
            if not is_reachable:
                continue
            x = index % width
            if ((index >= width and data[index - width] < 0)
                    or (x and data[index - 1] < 0)
                    or (x + 1 < width and data[index + 1] < 0)
                    or (index < last_row and data[index + width] < 0)):
                frontier[index] = 1
        return frontier

    def _clusters(
        self, grid: GridMap, remaining: bytearray,
    ) -> Iterator[list[int]]:
        """
        Yield one 4-connected cluster as flat indices at a time.

        Streaming avoids retaining every cluster and avoids constructing a
        ``Cell`` tuple for every frontier cell.  The seed scan remains
        row-major, preserving deterministic behavior.
        """
        width = grid.width
        size = len(remaining)
        last_row = size - width
        for seed in range(size):
            if not remaining[seed]:
                continue
            remaining[seed] = 0
            cluster_indices = [seed]
            cursor = 0
            while cursor < len(cluster_indices):
                index = cluster_indices[cursor]
                cursor += 1
                x = index % width
                if index >= width:
                    self._enqueue_frontier(
                        index - width, remaining, cluster_indices)
                if x:
                    self._enqueue_frontier(
                        index - 1, remaining, cluster_indices)
                if x + 1 < width:
                    self._enqueue_frontier(
                        index + 1, remaining, cluster_indices)
                if index < last_row:
                    self._enqueue_frontier(
                        index + width, remaining, cluster_indices)
            yield cluster_indices

    def _candidate(
        self,
        grid: GridMap,
        cluster: Sequence[int],
        steps: Sequence[int],
        clearance: Sequence[int],
        robot_yaw: float,
        unknown_seen: bytearray,
        unknown_touched: list[int],
        origin_cosine: float,
        origin_sine: float,
    ) -> FrontierCandidate:
        width = grid.width
        size = len(grid.data)
        last_row = size - width
        cluster_size = len(cluster)
        if cluster_size == 1:
            representative_index = cluster[0]
        else:
            x_sum = 0
            y_sum = 0
            for index in cluster:
                x_sum += index % width
                y_sum += index // width
            centroid_x = x_sum / cluster_size
            centroid_y = y_sum / cluster_size
            representative_index = min(
                cluster,
                key=lambda index: (
                    -clearance[index],
                    (index % width - centroid_x) ** 2
                    + (index // width - centroid_y) ** 2,
                    index,
                ),
            )
        data = grid.data
        unknown_x_sum = 0.0
        unknown_y_sum = 0.0
        for index in cluster:
            x = index % width
            if index >= width:
                unknown_index = index - width
                if data[unknown_index] < 0 and not unknown_seen[unknown_index]:
                    unknown_seen[unknown_index] = 1
                    unknown_touched.append(unknown_index)
                    unknown_x_sum += unknown_index % width + 0.5
                    unknown_y_sum += unknown_index // width + 0.5
            if x:
                unknown_index = index - 1
                if data[unknown_index] < 0 and not unknown_seen[unknown_index]:
                    unknown_seen[unknown_index] = 1
                    unknown_touched.append(unknown_index)
                    unknown_x_sum += unknown_index % width + 0.5
                    unknown_y_sum += unknown_index // width + 0.5
            if x + 1 < width:
                unknown_index = index + 1
                if data[unknown_index] < 0 and not unknown_seen[unknown_index]:
                    unknown_seen[unknown_index] = 1
                    unknown_touched.append(unknown_index)
                    unknown_x_sum += unknown_index % width + 0.5
                    unknown_y_sum += unknown_index // width + 0.5
            if index < last_row:
                unknown_index = index + width
                if data[unknown_index] < 0 and not unknown_seen[unknown_index]:
                    unknown_seen[unknown_index] = 1
                    unknown_touched.append(unknown_index)
                    unknown_x_sum += unknown_index % width + 0.5
                    unknown_y_sum += unknown_index // width + 0.5

        information_gain = len(unknown_touched)
        unknown_x = unknown_x_sum / information_gain
        unknown_y = unknown_y_sum / information_gain
        for index in unknown_touched:
            unknown_seen[index] = 0
        unknown_touched.clear()

        representative = (
            representative_index % width,
            representative_index // width,
        )
        local_goal_x = (representative[0] + 0.5) * grid.resolution
        local_goal_y = (representative[1] + 0.5) * grid.resolution
        goal_x = (
            grid.origin_x + origin_cosine * local_goal_x
            - origin_sine * local_goal_y)
        goal_y = (
            grid.origin_y + origin_sine * local_goal_x
            + origin_cosine * local_goal_y)
        target_local_x = unknown_x * grid.resolution
        target_local_y = unknown_y * grid.resolution
        target_yaw = grid.origin_yaw + math.atan2(
            target_local_y - (representative[1] + 0.5) * grid.resolution,
            target_local_x - (representative[0] + 0.5) * grid.resolution,
        )
        target_yaw = self._wrap_angle(target_yaw)
        heading_change = abs(self._wrap_angle(target_yaw - robot_yaw))
        path_distance = steps[representative_index] * grid.resolution
        score = (
            self.config.information_gain_weight * information_gain
            - self.config.distance_weight * path_distance
            - self.config.heading_weight * heading_change
        )
        return FrontierCandidate(
            cell=representative,
            x=goal_x,
            y=goal_y,
            yaw=target_yaw,
            cluster_size=cluster_size,
            information_gain=information_gain,
            path_distance_m=path_distance,
            clearance_m=(
                self._clearance_cells(clearance[representative_index])
                * grid.resolution),
            heading_change=heading_change,
            score=score,
        )

    @staticmethod
    def _clearance_cells(center_distance: int) -> float:
        """Conservative distance to the nearest obstacle cell boundary."""
        return max(0.0, center_distance - 0.5)

    @staticmethod
    def _enqueue_free(
        index: int,
        step: int,
        data: Sequence[int],
        clearance: Sequence[int],
        minimum_clearance: float,
        free_threshold: int,
        reached: bytearray,
        steps: list[int],
        queue: deque,
    ) -> None:
        if reached[index]:
            return
        value = data[index]
        if value < 0 or value > free_threshold:
            return
        if max(0.0, clearance[index] - 0.5) < minimum_clearance:
            return
        reached[index] = 1
        steps[index] = step
        queue.append(index)

    @staticmethod
    def _enqueue_frontier(
        index: int, remaining: bytearray, cluster: list[int],
    ) -> None:
        if remaining[index]:
            remaining[index] = 0
            cluster.append(index)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))
