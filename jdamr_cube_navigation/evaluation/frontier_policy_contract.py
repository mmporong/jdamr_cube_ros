#!/usr/bin/env python3
"""Pure, evaluation-only contracts for the G005 frontier policy study."""

from __future__ import annotations

from collections import deque
import hashlib
import json
import math
from pathlib import Path
import struct


LAYOUT_SEEDS = (11, 23, 42, 67, 89)
POLICIES = ('current', 'nearest', 'gain_nav')
FULL_PLAN = tuple((policy, seed) for policy in POLICIES
                  for seed in LAYOUT_SEEDS)
PROTOCOL_SALT = b'G005-frontier-neutral-sampler-v1\0'
GRID_WIDTH = 48
GRID_HEIGHT = 32
RESOLUTION_M_PER_CELL = 0.25
FOOTPRINT_CLEARANCE_M = 0.20
LIDAR_RATE_HZ = 10.0
LIDAR_BEAMS = 360
LIDAR_MIN_RANGE_M = 0.05
LIDAR_RANGE_M = 8.0
LIDAR_MIN_ANGLE_RAD = -2.862
LIDAR_MAX_ANGLE_RAD = 2.862
LIDAR_BASE_YAW_RAD = math.pi
SAMPLE_LIMIT = 5
RUN_OUTPUT_LIMIT_BYTES = 2 * 1024 * 1024
ARTIFACT_LIMIT_BYTES = 32 * 1024 * 1024


def canonical_json_bytes(value) -> bytes:
    """Serialize strict JSON deterministically and reject non-finite values."""
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=False, allow_nan=False) + '\n').encode()


def strict_json_load(path: Path):
    """Read JSON while rejecting duplicate keys and non-standard numbers."""
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f'duplicate JSON key: {key}')
            result[key] = value
        return result

    def reject(value):
        raise ValueError(f'non-finite JSON number: {value}')

    return json.loads(path.read_text(encoding='utf-8'),
                      object_pairs_hook=pairs, parse_constant=reject)


def sha256_file(path: Path) -> str:
    """Return the SHA-256 digest of one regular file."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def file_identity(path: Path, *, relative_to: Path | None = None) -> dict:
    """Create a stable regular-file identity."""
    resolved = path.resolve(strict=True)
    if path.is_symlink() or not resolved.is_file():
        raise ValueError('identity target must be a regular non-symlink file')
    name = (str(resolved.relative_to(relative_to.resolve()))
            if relative_to is not None else str(resolved))
    return {'path': name, 'size_bytes': resolved.stat().st_size,
            'sha256': sha256_file(resolved)}


def build_layout(layout_seed: int) -> dict:
    """Build a deterministic branched corridor and room occupancy raster."""
    if layout_seed not in LAYOUT_SEEDS:
        raise ValueError('layout seed is not preregistered')
    width = GRID_WIDTH
    height = GRID_HEIGHT
    data = [100] * (width * height)

    def carve(x0, x1, y0, y1):
        for y in range(y0, y1):
            for x in range(x0, x1):
                data[y * width + x] = 0

    center_y = 16
    branch_x = 18 + layout_seed % 5
    room_shift = (layout_seed // 10) % 3 - 1
    carve(2, 45, center_y - 2, center_y + 3)
    carve(branch_x - 2, branch_x + 3, 4, center_y + 3)
    carve(branch_x - 6, branch_x + 8, 3, 10)
    carve(32, 45, center_y - 8 + room_shift, center_y + 10 + room_shift)
    carve(5, 13, center_y - 10, center_y - 2)
    start_cell = [5, center_y]
    return {'width': width, 'height': height,
            'resolution_m_per_cell': RESOLUTION_M_PER_CELL,
            'origin_m_rad': [-6.0, -4.0, 0.0], 'data': data,
            'start_cell': start_cell, 'layout_seed': layout_seed}


def connected_reachable_cells(layout: dict) -> list[int]:
    """Return start-connected free cells satisfying footprint clearance."""
    width = layout['width']
    height = layout['height']
    data = layout['data']
    clearance_cells = math.ceil(
        FOOTPRINT_CLEARANCE_M / layout['resolution_m_per_cell'])

    def safe(index):
        x = index % width
        y = index // width
        if data[index] != 0:
            return False
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                nx = x + dx
                ny = y + dy
                if (0 <= nx < width and 0 <= ny < height and
                        data[ny * width + nx] >= 65 and
                        math.hypot(dx, dy) * layout[
                            'resolution_m_per_cell'] <=
                        FOOTPRINT_CLEARANCE_M):
                    return False
        return True

    start = layout['start_cell'][1] * width + layout['start_cell'][0]
    if not safe(start):
        raise ValueError('layout start does not satisfy footprint clearance')
    reached = {start}
    queue = deque([start])
    while queue:
        index = queue.popleft()
        x = index % width
        for other in (index - width, index - 1, index + 1, index + width):
            if (other < 0 or other >= len(data) or
                    other in reached or not safe(other)):
                continue
            cell_delta = (abs(other % width - x) +
                          abs(other // width - index // width))
            if cell_delta != 1:
                continue
            reached.add(other)
            queue.append(other)
    return sorted(reached)


def _noise_value(layout_seed: int, sensor_origin_cell: int,
                 beam_bin: int, target_cell: int) -> int:
    payload = struct.pack(
        '<4q', layout_seed, sensor_origin_cell, beam_bin, target_cell)
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], 'little')


def lidar_beam_angle_rad(beam_bin: int) -> float:
    """Return the exact endpoint-inclusive Gazebo horizontal beam angle."""
    if type(beam_bin) is not int or not 0 <= beam_bin < LIDAR_BEAMS:
        raise ValueError('LiDAR beam index is outside the profile')
    return (LIDAR_MIN_ANGLE_RAD +
            (LIDAR_MAX_ANGLE_RAD - LIDAR_MIN_ANGLE_RAD) *
            beam_bin / (LIDAR_BEAMS - 1))


def lidar_world_beam_angle_rad(beam_bin: int, base_yaw_rad: float) -> float:
    """Transform one laser-frame beam through the fixed base extrinsic."""
    if (type(base_yaw_rad) not in (int, float) or
            not math.isfinite(float(base_yaw_rad))):
        raise ValueError('LiDAR base yaw is not finite')
    return (float(base_yaw_rad) + LIDAR_BASE_YAW_RAD +
            lidar_beam_angle_rad(beam_bin))


def reveal_scan(layout: dict, observed: list[int], pose_cell: int,
                pose_yaw_rad: float, origin_scan_index: int) -> list[int]:
    """Reveal GT classes monotonically using public delay/dropout lookup."""
    if (len(observed) != len(layout['data']) or
            type(origin_scan_index) is not int or origin_scan_index < 0):
        raise ValueError('reveal input shape drift')
    result = list(observed)
    width = layout['width']
    origin_x = pose_cell % width
    origin_y = pose_cell // width
    max_cells = math.floor(LIDAR_RANGE_M / layout['resolution_m_per_cell'])
    for beam_bin in range(LIDAR_BEAMS):
        angle_rad = (lidar_world_beam_angle_rad(beam_bin, pose_yaw_rad) -
                     layout['origin_m_rad'][2])
        for step in range(1, max_cells + 1):
            x = origin_x + round(math.cos(angle_rad) * step)
            y = origin_y + round(math.sin(angle_rad) * step)
            if not (0 <= x < width and 0 <= y < layout['height']):
                break
            target = y * width + x
            noise = _noise_value(
                layout['layout_seed'], pose_cell, beam_bin, target)
            dropout = noise % 1000 < 20
            delay_scans = (noise // 1000) % 3
            if not dropout and origin_scan_index >= delay_scans:
                current = result[target]
                truth = layout['data'][target]
                if current not in (-1, truth):
                    raise ValueError('monotonic reveal attempted a class flip')
                result[target] = truth
            if layout['data'][target] >= 65:
                break
    if result[pose_cell] not in (-1, layout['data'][pose_cell]):
        raise ValueError('monotonic reveal attempted a class flip')
    result[pose_cell] = layout['data'][pose_cell]
    return result


def neutral_sample(
        layout_seed: int, candidate_cell_indices: list[int]) -> dict:
    """Select at most five candidates by policy-neutral stable hashing."""
    if len(candidate_cell_indices) != len(set(candidate_cell_indices)):
        raise ValueError('candidate IDs must be unique')
    ranked = sorted(candidate_cell_indices, key=lambda index: (
        hashlib.sha256(PROTOCOL_SALT + struct.pack(
            '<qq', layout_seed, index)).digest(), index))
    selected = ranked[:SAMPLE_LIMIT]
    return {'raw_candidate_ids': sorted(candidate_cell_indices),
            'selected_candidate_ids': selected,
            'omitted_candidate_ids': ranked[SAMPLE_LIMIT:]}


def policy_rank(policy: str, candidates: list[dict]) -> list[dict]:
    """Rank eligible candidates using preregistered formulas and ties."""
    if policy not in POLICIES or not candidates:
        raise ValueError('policy ranking requires candidates')
    for item in candidates:
        required = {'cell_index', 'gain_cells', 'bfs_distance_m',
                    'heading_rad', 'nav_length_m'}
        if set(item) != required or any(
                type(item[key]) not in (int, float) or
                not math.isfinite(item[key]) for key in required):
            raise ValueError('candidate metric schema drift')
    if policy == 'current':
        for item in candidates:
            item['utility'] = (item['gain_cells'] -
                               0.20 * item['bfs_distance_m'] -
                               0.10 * item['heading_rad'])
        return sorted(candidates, key=lambda item: (
            -item['utility'], item['cell_index']))
    elif policy == 'nearest':
        for item in candidates:
            item['utility'] = -item['nav_length_m']
        return sorted(candidates, key=lambda item: (
            item['nav_length_m'], item['cell_index']))
    else:
        gains = [item['gain_cells'] for item in candidates]
        lengths = [item['nav_length_m'] for item in candidates]
        gain_range = max(gains) - min(gains)
        length_range = max(lengths) - min(lengths)
        for item in candidates:
            gain_norm = ((item['gain_cells'] - min(gains)) / gain_range
                         if gain_range else 0.0)
            length_norm = ((item['nav_length_m'] - min(lengths)) /
                           length_range if length_range else 0.0)
            item['utility'] = 0.5 * gain_norm - 0.5 * length_norm
        return sorted(candidates, key=lambda item: (
            -item['utility'], item['nav_length_m'],
            -item['gain_cells'], item['cell_index']))


def decision_token(run_id: str, policy: str, map_sequence: int,
                   map_payload_sha256: str, start_record: dict,
                   blacklist_ids: list[int], candidate_ids: list[int]) -> str:
    """Freeze every input that can affect one planner validation batch."""
    value = {'run_id': run_id, 'policy': policy,
             'map_sequence': map_sequence,
             'map_payload_sha256': map_payload_sha256,
             'start_record_sha256': hashlib.sha256(
                 canonical_json_bytes(start_record)).hexdigest(),
             'blacklist_sha256': hashlib.sha256(
                 canonical_json_bytes(blacklist_ids)).hexdigest(),
             'candidate_sha256': hashlib.sha256(
                 canonical_json_bytes(candidate_ids)).hexdigest()}
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def validate_planner_batch(records: list[dict], token: str,
                           before_costmap_sha256: str,
                           after_costmap_sha256: str) -> list[dict]:
    """Reject stale, changing, malformed, or unreachable path results."""
    if before_costmap_sha256 != after_costmap_sha256:
        raise ValueError('costmap changed during frozen planner batch')
    eligible = []
    for record in records:
        expected = {'candidate', 'token', 'use_start', 'planner_id',
                    'timeout_s', 'error_code', 'frame_id', 'poses'}
        if (type(record) is not dict or set(record) != expected or
                record['token'] != token or record['use_start'] is not True or
                record['planner_id'] != 'GridBased' or
                record['timeout_s'] != 2.0 or
                record['frame_id'] != 'map' or
                type(record['error_code']) is not int or
                record['error_code'] != 0 or
                not record['poses']):
            continue
        poses = record['poses']
        if any(type(pose) is not list or len(pose) != 2 or any(
                type(value) not in (int, float) or not math.isfinite(value)
                for value in pose) for pose in poses):
            continue
        length_m = sum(math.hypot(right[0] - left[0],
                                  right[1] - left[1])
                       for left, right in zip(poses, poses[1:]))
        item = dict(record['candidate'])
        item['nav_length_m'] = length_m
        eligible.append(item)
    return eligible


def unresolved_after_three(fresh_batches: list[dict]) -> bool:
    """Require three identical fresh all-unreachable observations."""
    if len(fresh_batches) != 3:
        return False
    first = fresh_batches[0]
    required = {'map_sha256', 'start_sha256', 'candidate_sha256',
                'fresh', 'reachable_count'}
    return (all(type(item) is dict and set(item) == required and
                item['fresh'] is True and item['reachable_count'] == 0
                for item in fresh_batches) and
            all(item['map_sha256'] == first['map_sha256'] and
                item['start_sha256'] == first['start_sha256'] and
                item['candidate_sha256'] == first['candidate_sha256']
                for item in fresh_batches[1:]))


def unreachable_outcome(fresh_batches: list[dict]) -> dict:
    """Keep an all-unreachable frontier set distinct from completion."""
    unresolved = unresolved_after_three(fresh_batches)
    return {'status': ('UNRESOLVED_FRONTIERS' if unresolved else 'PENDING'),
            'result': ('FAIL' if unresolved else 'NOT_EVALUATED'),
            'exploration_complete': False,
            'blacklist_exhausted': False}
