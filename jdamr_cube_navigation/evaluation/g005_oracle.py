#!/usr/bin/env python3
"""Pure, evaluation-only oracle map state for the G005 runtime study."""

from __future__ import annotations

import hashlib
import math
from typing import Sequence

from frontier_policy_contract import (
    canonical_json_bytes,
    connected_reachable_cells,
    LAYOUT_SEEDS,
    reveal_scan,
)


ROBOT_CIRCUMSCRIBED_RADIUS_M = math.hypot(0.23, 0.20)


def _finite_number(value: object) -> bool:
    return (type(value) in (int, float) and
            math.isfinite(float(value)))


def _layout_fields(layout: object) -> tuple[
        int, int, float, list[float], Sequence[int]]:
    """Validate the GT raster fields consumed by the oracle."""
    if type(layout) is not dict:
        raise ValueError('G005 oracle layout must be an object')
    required = {'width', 'height', 'resolution_m_per_cell',
                'origin_m_rad', 'data', 'start_cell', 'layout_seed'}
    if not required.issubset(layout):
        raise ValueError('G005 oracle layout schema drift')
    width = layout['width']
    height = layout['height']
    resolution = layout['resolution_m_per_cell']
    origin = layout['origin_m_rad']
    data = layout['data']
    if (type(width) is not int or type(height) is not int or
            width <= 0 or height <= 0 or
            not _finite_number(resolution) or resolution <= 0.0 or
            type(origin) is not list or len(origin) != 3 or
            any(not _finite_number(value) for value in origin) or
            type(data) is not list or len(data) != width * height or
            any(type(value) is not int or value not in (0, 100)
                for value in data) or
            layout['layout_seed'] not in LAYOUT_SEEDS):
        raise ValueError('G005 oracle layout fields are invalid')
    start_cell = layout['start_cell']
    if (type(start_cell) is not list or len(start_cell) != 2 or
            any(type(value) is not int for value in start_cell) or
            not 0 <= start_cell[0] < width or
            not 0 <= start_cell[1] < height or
            data[start_cell[1] * width + start_cell[0]] != 0):
        raise ValueError('G005 oracle start cell is invalid')
    return width, height, float(resolution), origin, data


def world_pose_to_cell(layout: dict, x_m: float, y_m: float) -> int:
    """Return a validated free-cell index for a finite GT world position."""
    width, height, resolution, origin, data = _layout_fields(layout)
    if not _finite_number(x_m) or not _finite_number(y_m):
        raise ValueError('G005 oracle GT pose must be finite')
    delta_x = float(x_m) - origin[0]
    delta_y = float(y_m) - origin[1]
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    cell_x = math.floor(local_x / resolution)
    cell_y = math.floor(local_y / resolution)
    if not 0 <= cell_x < width or not 0 <= cell_y < height:
        raise ValueError('G005 oracle GT pose is outside the map')
    index = cell_y * width + cell_x
    if data[index] != 0:
        raise ValueError('G005 oracle GT pose is not in free space')
    return index


def occupancy_grid_metadata(layout: dict, frame_id: str = 'map') -> dict:
    """Return canonical ROS OccupancyGrid metadata without timestamps."""
    width, height, resolution, origin, _ = _layout_fields(layout)
    if (type(frame_id) is not str or not frame_id or
            frame_id.strip() != frame_id):
        raise ValueError('G005 oracle frame ID is invalid')
    half_yaw = origin[2] / 2.0
    return {
        'frame_id': frame_id,
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin': {
            'position': {'x': float(origin[0]), 'y': float(origin[1]),
                         'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0,
                            'z': math.sin(half_yaw),
                            'w': math.cos(half_yaw)},
        },
    }


def clearance_m(layout: dict, x_m: float, y_m: float) -> float:
    """Measure footprint clearance from the robot center to occupied AABBs."""
    width, _, resolution, origin, data = _layout_fields(layout)
    world_pose_to_cell(layout, x_m, y_m)
    delta_x = float(x_m) - origin[0]
    delta_y = float(y_m) - origin[1]
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    minimum = math.inf
    for index, value in enumerate(data):
        if value < 65:
            continue
        cell_x = index % width
        cell_y = index // width
        left = cell_x * resolution
        right = left + resolution
        bottom = cell_y * resolution
        top = bottom + resolution
        distance_x = max(left - local_x, 0.0, local_x - right)
        distance_y = max(bottom - local_y, 0.0, local_y - top)
        minimum = min(minimum, math.hypot(distance_x, distance_y))
    if not math.isfinite(minimum):
        raise ValueError('G005 oracle layout has no occupied cells')
    return max(0.0, minimum - ROBOT_CIRCUMSCRIBED_RADIUS_M)


class OracleMapState:
    """Maintain one deterministic, monotonic oracle-reveal occupancy map."""

    def __init__(self, layout: dict):
        _layout_fields(layout)
        self._layout = dict(layout)
        self._layout['origin_m_rad'] = list(layout['origin_m_rad'])
        self._layout['start_cell'] = list(layout['start_cell'])
        self._layout['data'] = list(layout['data'])
        self._observed = [-1] * len(layout['data'])
        self._reachable = tuple(connected_reachable_cells(self._layout))
        if not self._reachable:
            raise ValueError('G005 oracle reachable denominator is empty')
        self._map_sequence = 0
        self._last_scan_index: int | None = None
        self._last_pose_cell: int | None = None
        self._last_pose_yaw_rad: float | None = None
        self._last_elapsed_s: float | None = None
        self._origin_scan_counts: dict[int, int] = {}

    @property
    def map_sequence(self) -> int:
        return self._map_sequence

    @property
    def observed(self) -> list[int]:
        return list(self._observed)

    @property
    def reachable_denominator_cells(self) -> int:
        return len(self._reachable)

    @property
    def reachable_cell_indices(self) -> tuple[int, ...]:
        return self._reachable

    @property
    def map_payload_sha256(self) -> str:
        return hashlib.sha256(
            canonical_json_bytes(self._observed)).hexdigest()

    def update_scan(
            self, x_m: float, y_m: float, yaw_rad: float,
            scan_index: int) -> dict:
        """Apply one indexed scan and increment sequence only on map change."""
        if type(scan_index) is not int or scan_index < 0:
            raise ValueError('G005 oracle scan index is invalid')
        if (self._last_scan_index is not None and
                scan_index < self._last_scan_index):
            raise ValueError('G005 oracle scan index regressed')
        pose_cell = world_pose_to_cell(self._layout, x_m, y_m)
        if not _finite_number(yaw_rad):
            raise ValueError('G005 oracle GT yaw must be finite')
        normalized_yaw = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))
        if scan_index == self._last_scan_index:
            if (pose_cell != self._last_pose_cell or
                    not math.isclose(
                        normalized_yaw, self._last_pose_yaw_rad,
                        rel_tol=0.0, abs_tol=1e-12)):
                raise ValueError('duplicate G005 scan index changed pose')
            origin_scan_index = self._origin_scan_counts[pose_cell] - 1
        else:
            origin_scan_index = self._origin_scan_counts.get(pose_cell, 0)
            self._origin_scan_counts[pose_cell] = origin_scan_index + 1
        newer = reveal_scan(
            self._layout, self._observed, pose_cell, normalized_yaw,
            origin_scan_index)
        if any(old != -1 and old != new
               for old, new in zip(self._observed, newer)):
            raise ValueError('G005 oracle reveal is not monotonic')
        changed = newer != self._observed
        if changed:
            self._observed = newer
            self._map_sequence += 1
        self._last_scan_index = scan_index
        self._last_pose_cell = pose_cell
        self._last_pose_yaw_rad = normalized_yaw
        return {
            'map_sequence': self._map_sequence,
            'map_changed': changed,
            'pose_cell': pose_cell,
            'map_payload_sha256': self.map_payload_sha256,
        }

    def coverage_sample(self, elapsed_s: float) -> dict:
        """Sample revealed start-connected free cells at monotonic sim time."""
        if not _finite_number(elapsed_s) or elapsed_s < 0.0:
            raise ValueError('G005 oracle elapsed time is invalid')
        elapsed = float(elapsed_s)
        if self._last_elapsed_s is not None and elapsed < self._last_elapsed_s:
            raise ValueError('G005 oracle elapsed time regressed')
        self._last_elapsed_s = elapsed
        revealed = sum(
            self._observed[index] == self._layout['data'][index]
            for index in self._reachable)
        return {'elapsed_s': elapsed,
                'revealed_reachable_cells': revealed}
