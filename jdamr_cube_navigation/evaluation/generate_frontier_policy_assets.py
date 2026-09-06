#!/usr/bin/env python3
"""Generate deterministic simulation assets for G005 frontier evaluation."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shutil
import tempfile

from frontier_policy_contract import (
    ARTIFACT_LIMIT_BYTES,
    build_layout,
    canonical_json_bytes,
    connected_reachable_cells,
    file_identity,
    FOOTPRINT_CLEARANCE_M,
    FULL_PLAN,
    LAYOUT_SEEDS,
    LIDAR_BEAMS,
    LIDAR_RANGE_M,
    LIDAR_RATE_HZ,
    POLICIES,
    sha256_file,
    strict_json_load,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
PRODUCTION_INPUTS = {
    'frontier_core': REPO_ROOT / 'jdamr_cube_navigation' /
    'jdamr_cube_navigation' / 'frontier_core.py',
    'frontier_explorer': REPO_ROOT / 'jdamr_cube_navigation' /
    'jdamr_cube_navigation' / 'frontier_explorer.py',
    'nav2_params': REPO_ROOT / 'jdamr_cube_navigation' / 'config' /
    'nav2_params.yaml',
    'cartographer_config': REPO_ROOT / 'jdamr_cube_cartographer' / 'config' /
    'jdamr_cube_2d.lua',
}


def _tree_records(root: Path, names: list[str]) -> tuple[list[dict], str]:
    records = [file_identity(root / name, relative_to=root)
               for name in sorted(names)]
    return records, hashlib.sha256(canonical_json_bytes(records)).hexdigest()


def _sdf(layout: dict) -> bytes:
    resolution_m = layout['resolution_m_per_cell']
    origin_x_m, origin_y_m, _ = layout['origin_m_rad']
    models = []
    for index, value in enumerate(layout['data']):
        if value < 65:
            continue
        x_m = origin_x_m + (index % layout['width'] + 0.5) * resolution_m
        y_m = origin_y_m + (index // layout['width'] + 0.5) * resolution_m
        models.append(
            f'<model name="wall_{index}"><static>true</static>'
            f'<pose>{x_m:.6f} {y_m:.6f} 0.5 0 0 0</pose><link name="body">'
            f'<collision name="collision"><geometry><box><size>'
            f'{resolution_m:.6f} {resolution_m:.6f} 1.0</size></box>'
            f'</geometry></collision><visual name="visual"><geometry><box><size>'
            f'{resolution_m:.6f} {resolution_m:.6f} 1.0</size></box>'
            f'</geometry></visual></link></model>')
    content = ('<?xml version="1.0"?><sdf version="1.9">'
               '<world name="g005_frontier"><physics name="fixed" '
               'type="ignored"><max_step_size>0.001</max_step_size>'
               '<real_time_factor>1.0</real_time_factor></physics>'
               + ''.join(models) + '</world></sdf>\n')
    return content.encode()


def validate_assets(root: Path, expected_mode: str) -> dict:
    """Rehash the exact generated tree and validate every asset contract."""
    if (not root.is_absolute() or not root.is_dir() or root.is_symlink() or
            expected_mode not in ('smoke', 'full')):
        raise ValueError('non-canonical G005 asset root')
    actual = sorted(path.name for path in root.iterdir())
    seeds = (11,) if expected_mode == 'smoke' else LAYOUT_SEEDS
    expected_assets = [item for seed in seeds for item in (
        f'layout_{seed}.world', f'layout_{seed}_gt.json')]
    expected = sorted(expected_assets + ['asset_manifest.json'])
    if actual != expected or any(path.is_symlink() or not path.is_file()
                                 for path in root.iterdir()):
        raise ValueError('G005 asset tree inventory drift')
    manifest = strict_json_load(root / 'asset_manifest.json')
    keys = {'schema_version', 'mode', 'layout_seeds', 'full_plan',
            'policies', 'layout_contract', 'layouts', 'production_inputs',
            'asset_files', 'asset_tree_sha256', 'generator_source',
            'storage_limit_bytes'}
    if (type(manifest) is not dict or set(manifest) != keys or
            manifest['schema_version'] != 1 or
            manifest['mode'] != expected_mode or
            manifest['layout_seeds'] != list(seeds) or
            manifest['policies'] != list(POLICIES) or
            manifest['full_plan'] != [
                {'policy': policy, 'layout_seed': seed}
                for policy, seed in FULL_PLAN] or
            manifest['storage_limit_bytes'] != ARTIFACT_LIMIT_BYTES):
        raise ValueError('G005 asset manifest schema drift')
    expected_layout_contract = {
        'resolution_m_per_cell': 0.25,
        'footprint_clearance_m': FOOTPRINT_CLEARANCE_M,
        'lidar': {'rate_hz': LIDAR_RATE_HZ, 'beam_count': LIDAR_BEAMS,
                  'range_m': LIDAR_RANGE_M},
        'reveal': {'monotonic': True, 'dropout_per_mille': 20,
                   'delay_scan_values': [0, 1, 2],
                   'noise_key': ['layout_seed', 'sensor_origin_cell',
                                 'beam_bin', 'target_cell']}}
    if (manifest['layout_contract'] != expected_layout_contract or
            type(manifest['layouts']) is not dict or
            set(manifest['layouts']) != {str(seed) for seed in seeds}):
        raise ValueError('G005 layout contract drift')
    records, tree_sha = _tree_records(root, expected_assets)
    if (manifest['asset_files'] != records or
            manifest['asset_tree_sha256'] != tree_sha):
        raise ValueError('G005 asset tree hash drift')
    if manifest['generator_source'] != file_identity(Path(__file__).resolve()):
        raise ValueError('G005 generator source drift')
    expected_production = {name: file_identity(path)
                           for name, path in PRODUCTION_INPUTS.items()}
    if manifest['production_inputs'] != expected_production:
        raise ValueError('G005 production input drift')
    for seed in seeds:
        layout = strict_json_load(root / f'layout_{seed}_gt.json')
        expected_layout = build_layout(seed)
        if layout != expected_layout:
            raise ValueError('G005 GT occupancy drift')
        record = manifest['layouts'][str(seed)]
        if (type(record) is not dict or set(record) != {
                'gazebo_seed', 'start_cell_index',
                'reachable_denominator_cells', 'reachable_cell_indices',
                'gt_occupancy_sha256', 'sdf_sha256'} or
                record['gazebo_seed'] != seed or
                record['start_cell_index'] != (
                    layout['start_cell'][1] * layout['width'] +
                    layout['start_cell'][0]) or
                record['gt_occupancy_sha256'] != sha256_file(
                    root / f'layout_{seed}_gt.json') or
                record['sdf_sha256'] != sha256_file(
                    root / f'layout_{seed}.world') or
                record['reachable_cell_indices'] !=
                connected_reachable_cells(layout) or
                record['reachable_denominator_cells'] !=
                len(record['reachable_cell_indices'])):
            raise ValueError('G005 reachable denominator drift')
    if sum(path.stat().st_size for path in root.iterdir()) > ARTIFACT_LIMIT_BYTES:
        raise ValueError('G005 asset storage cap exceeded')
    return manifest


def generate(output_root: Path, mode: str) -> dict:
    """Atomically create smoke or full preregistered layout assets."""
    if output_root.exists() or output_root.is_symlink() or not output_root.is_absolute():
        raise ValueError('G005 output root must be absent and absolute')
    if mode not in ('smoke', 'full'):
        raise ValueError('unknown G005 generation mode')
    seeds = (11,) if mode == 'smoke' else LAYOUT_SEEDS
    with tempfile.TemporaryDirectory(
            prefix='.g005-frontier-assets-', dir=output_root.parent) as temp:
        stage = Path(temp) / 'assets'
        stage.mkdir()
        layouts = {}
        names = []
        for seed in seeds:
            layout = build_layout(seed)
            gt_name = f'layout_{seed}_gt.json'
            world_name = f'layout_{seed}.world'
            (stage / gt_name).write_bytes(canonical_json_bytes(layout))
            (stage / world_name).write_bytes(_sdf(layout))
            names.extend((gt_name, world_name))
            reachable = connected_reachable_cells(layout)
            start_index = (layout['start_cell'][1] * layout['width'] +
                           layout['start_cell'][0])
            layouts[str(seed)] = {
                'gazebo_seed': seed, 'start_cell_index': start_index,
                'reachable_denominator_cells': len(reachable),
                'reachable_cell_indices': reachable,
                'gt_occupancy_sha256': sha256_file(stage / gt_name),
                'sdf_sha256': sha256_file(stage / world_name)}
        records, tree_sha = _tree_records(stage, names)
        manifest = {
            'schema_version': 1, 'mode': mode,
            'layout_seeds': list(seeds),
            'full_plan': [{'policy': policy, 'layout_seed': seed}
                          for policy, seed in FULL_PLAN],
            'policies': list(POLICIES),
            'layout_contract': {
                'resolution_m_per_cell': 0.25,
                'footprint_clearance_m': FOOTPRINT_CLEARANCE_M,
                'lidar': {'rate_hz': LIDAR_RATE_HZ, 'beam_count': LIDAR_BEAMS,
                          'range_m': LIDAR_RANGE_M},
                'reveal': {'monotonic': True, 'dropout_per_mille': 20,
                           'delay_scan_values': [0, 1, 2],
                           'noise_key': ['layout_seed', 'sensor_origin_cell',
                                         'beam_bin', 'target_cell']}},
            'layouts': layouts,
            'production_inputs': {name: file_identity(path)
                                  for name, path in PRODUCTION_INPUTS.items()},
            'asset_files': records, 'asset_tree_sha256': tree_sha,
            'generator_source': file_identity(Path(__file__).resolve()),
            'storage_limit_bytes': ARTIFACT_LIMIT_BYTES}
        (stage / 'asset_manifest.json').write_bytes(
            canonical_json_bytes(manifest))
        validate_assets(stage, mode)
        stage.rename(output_root)
    try:
        return validate_assets(output_root, mode)
    except Exception:
        if output_root.is_dir() and not output_root.is_symlink():
            shutil.rmtree(output_root)
        raise


def main() -> int:
    """Generate one exact G005 asset root."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--mode', choices=('smoke', 'full'), required=True)
    args = parser.parse_args()
    generate(args.output_root, args.mode)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
