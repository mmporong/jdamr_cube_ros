#!/usr/bin/env python3
"""Run one bounded offline G005 frontier decision smoke."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shutil
import tempfile

from evaluate_frontier_policy import (
    FULL_METRIC_FIELDS,
    promotion_decision,
    validate_artifact,
)
from frontier_policy_contract import (
    ARTIFACT_LIMIT_BYTES,
    canonical_json_bytes,
    decision_token,
    file_identity,
    FULL_PLAN,
    neutral_sample,
    policy_rank,
    reveal_scan,
    strict_json_load,
    validate_planner_batch,
)
from generate_frontier_policy_assets import validate_assets
from jdamr_cube_navigation.frontier_core import (
    BlacklistEntry,
    FrontierCore,
    GridMap,
)


def _candidate(candidate: object, grid: GridMap) -> dict:
    return {'cell_index': grid.index(candidate.cell),
            'gain_cells': candidate.information_gain,
            'bfs_distance_m': candidate.path_distance_m,
            'heading_rad': candidate.heading_change,
            'nav_length_m': 0.0}


def run_smoke(asset_root: Path, output_root: Path) -> dict:
    """Atomically evaluate current policy on seed 11's first decision."""
    validate_assets(asset_root, 'smoke')
    if output_root.exists() or output_root.is_symlink() or not output_root.is_absolute():
        raise ValueError('G005 smoke output must be absent and absolute')
    layout = strict_json_load(asset_root / 'layout_11_gt.json')
    width = layout['width']
    start_cell = tuple(layout['start_cell'])
    start_index = start_cell[1] * width + start_cell[0]
    observed = [-1] * len(layout['data'])
    for scan_index in range(3):
        newer = reveal_scan(layout, observed, start_index, scan_index)
        if any(old != -1 and old != new
               for old, new in zip(observed, newer)):
            raise ValueError('G005 reveal was not monotonic')
        observed = newer
    grid = GridMap(width, layout['height'], layout['resolution_m_per_cell'],
                   *layout['origin_m_rad'], observed)
    start_x_m, start_y_m = grid.cell_to_world(start_cell)
    core = FrontierCore()
    raw = core.extract(grid, start_x_m, start_y_m, robot_yaw=0.0)
    if len(raw) < 3:
        raise ValueError('G005 smoke did not expose enough frontiers')
    excluded = raw[-1]
    blacklist = [BlacklistEntry(
        excluded.x, excluded.y, 0.01, expires_at=10.0, generation=1)]
    filtered = core.extract(
        grid, start_x_m, start_y_m, robot_yaw=0.0,
        blacklist=blacklist, now=1.0, generation=1)
    raw_ids = [grid.index(item.cell) for item in raw]
    filtered_ids = [grid.index(item.cell) for item in filtered]
    excluded_ids = sorted(set(raw_ids) - set(filtered_ids))
    sample = neutral_sample(11, filtered_ids)
    sample['raw_candidate_ids'] = sorted(raw_ids)
    sample['excluded_candidate_ids'] = excluded_ids
    selected_ids = sample['selected_candidate_ids']
    by_id = {grid.index(item.cell): item for item in filtered}
    selected_candidates = [by_id[index] for index in selected_ids]
    map_sha = hashlib.sha256(canonical_json_bytes(observed)).hexdigest()
    start_record = {'frame_id': 'map', 'stamp_ns': 300000000,
                    'pose_xy_yaw': [start_x_m, start_y_m, 0.0]}
    run_id = 'current__seed_11'
    token = decision_token(
        run_id, 'current', 1, map_sha, start_record,
        excluded_ids, selected_ids)
    planner_results = []
    raw_candidate_records = []
    for candidate in selected_candidates:
        record = _candidate(candidate, grid)
        raw_candidate_records.append(record)
        middle = [(start_x_m + candidate.x) / 2.0,
                  (start_y_m + candidate.y) / 2.0]
        planner_results.append({
            'candidate': record, 'token': token, 'use_start': True,
            'planner_id': 'GridBased', 'timeout_s': 2.0,
            'error_code': 0, 'frame_id': 'map',
            'poses': [[start_x_m, start_y_m], middle,
                      [candidate.x, candidate.y]]})
    costmap_sha = hashlib.sha256(
        canonical_json_bytes(observed)).hexdigest()
    eligible = validate_planner_batch(
        planner_results, token, costmap_sha, costmap_sha)
    if len(eligible) < 2:
        raise ValueError('G005 smoke requires two reachable candidates')
    ranked = policy_rank('current', [dict(item) for item in eligible])
    evidence = {
        'schema_version': 1, 'run_id': run_id, 'mode': 'smoke',
        'policy': 'current', 'layout_seed': 11, 'map_sequence': 1,
        'map_payload_sha256': map_sha, 'start_record': start_record,
        'blacklist_ids': excluded_ids, 'candidate_sampling': sample,
        'raw_candidates': raw_candidate_records,
        'planner_contract': {
            'action': 'ComputePathToPose', 'use_start': True,
            'planner_id': 'GridBased', 'timeout_s': 2.0,
            'same_frozen_start_for_all': True,
            'execution_status':
            'SYNTHETIC_FIXTURE_NOT_RUNTIME_COMPUTE_PATH'},
        'planner_results': planner_results,
        'costmap_sha256_before': costmap_sha,
        'costmap_sha256_after': costmap_sha,
        'decision_token': token, 'eligible_candidates': eligible,
        'selected_goal': ranked[0], 'unreachable_observations': [],
        'status': 'PASS', 'full_metric_schema': list(FULL_METRIC_FIELDS),
        'runtime_metrics': {field: 'NOT_MEASURED_SMOKE'
                            for field in FULL_METRIC_FIELDS},
        'promotion': promotion_decision([])}
    with tempfile.TemporaryDirectory(
            prefix='.g005-frontier-smoke-', dir=output_root.parent) as temp:
        stage = Path(temp) / 'result'
        stage.mkdir()
        evidence_name = 'current__seed_11.json'
        (stage / evidence_name).write_bytes(canonical_json_bytes(evidence))
        records = [file_identity(stage / evidence_name, relative_to=stage)]
        tree_sha = hashlib.sha256(canonical_json_bytes(records)).hexdigest()
        manifest = {
            'schema_version': 1, 'mode': 'smoke',
            'claim_scope':
            'OFFLINE_FRONTIER_POLICY_CONTRACT_SMOKE_NO_NAV2_NO_MOTION',
            'asset_root': str(asset_root.resolve()),
            'asset_manifest': file_identity(
                asset_root / 'asset_manifest.json'),
            'full_plan': [{'policy': policy, 'layout_seed': seed}
                          for policy, seed in FULL_PLAN],
            'executed_plan': [{'policy': 'current', 'layout_seed': 11}],
            'runs': [{'path': evidence_name}],
            'promotion': promotion_decision([]),
            'evaluator_source': file_identity(Path(
                __import__('evaluate_frontier_policy').__file__).resolve()),
            'runner_source': file_identity(Path(__file__).resolve()),
            'tree_files': records, 'tree_sha256': tree_sha,
            'storage_limit_bytes': ARTIFACT_LIMIT_BYTES}
        (stage / 'manifest.json').write_bytes(canonical_json_bytes(manifest))
        validate_artifact(stage, 'smoke')
        stage.rename(output_root)
    try:
        return validate_artifact(output_root, 'smoke')
    except Exception:
        if output_root.is_dir() and not output_root.is_symlink():
            shutil.rmtree(output_root)
        raise


def main() -> int:
    """Run exactly one preregistered G005 smoke decision."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--asset-root', required=True, type=Path)
    parser.add_argument('--output-root', required=True, type=Path)
    args = parser.parse_args()
    run_smoke(args.asset_root, args.output_root)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
