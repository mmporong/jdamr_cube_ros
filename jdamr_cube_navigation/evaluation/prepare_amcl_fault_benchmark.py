#!/usr/bin/env python3
"""Prepare the immutable, metrics-only G002 benchmark contract."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from amcl_fault_contract import AXIS_B_SCENARIOS
from amcl_fault_contract import canonical_json_bytes
from amcl_fault_contract import current_free_bytes
from amcl_fault_contract import exact_regular_file
from amcl_fault_contract import G002_ARTIFACT_LIMIT_BYTES
from amcl_fault_contract import G002_RUN_OUTPUT_LIMIT_BYTES
from amcl_fault_contract import map_identity
from amcl_fault_contract import OVERLAY_CHANGED_FILES
from amcl_fault_contract import PROFILES
from amcl_fault_contract import REAL_BAG_SHA256
from amcl_fault_contract import REAL_BAG_TOPIC_INVENTORY
from amcl_fault_contract import REAL_MAP_PGM_SHA256
from amcl_fault_contract import REAL_MAP_YAML_SHA256
from amcl_fault_contract import SEEDS
from amcl_fault_contract import sha256_file
from amcl_fault_contract import STORAGE_LIMITS
from amcl_fault_contract import strict_json_load
from amcl_fault_contract import UPSTREAM_AMCL_TREE_FILE_COUNT
from amcl_fault_contract import UPSTREAM_AMCL_TREE_SHA256
from amcl_fault_contract import UPSTREAM_ARCHIVE_SHA256
from amcl_fault_contract import UPSTREAM_ARCHIVE_SIZE_BYTES
from amcl_fault_contract import UPSTREAM_ARCHIVE_URL
from amcl_fault_contract import UPSTREAM_COMMIT
from amcl_fault_contract import UPSTREAM_LICENSE_SHA256
from amcl_fault_contract import validate_storage_budget
from prepare_amcl_source_overlay import canonical_source_tree
from prepare_amcl_source_overlay import validate_source_tree_delta


ROOT = Path(__file__).resolve().parents[2]
NAV = ROOT / 'jdamr_cube_navigation'
G004_ROOT = Path.home() / 'jdamr_artifacts' / (
    'sim_collision_monitor_eval_20260906_v08_full15_cyclone')
G004_CONTRACT_SHA256 = (
    '446c57e10c9142e95782ee8396b5e0f49b055dd33740575ac5e64c2964ed4a91')
REAL_BAG_ROOT = Path.home() / 'jdamr_artifacts' / (
    'corridor_localdds_armed_20260904T152036')
REAL_BAG_NAME = 'corridor_localdds_armed_20260904T152036_0.mcap'
REAL_MAP = Path.home() / 'maps' / 'autonomous_20260826T161908.yaml'
PRODUCTION_FILES = {
    'navigation_launch': NAV / 'launch' / 'navigation.launch.py',
    'onboard_nav2_core_launch': NAV / 'launch' / 'onboard_nav2_core.launch.py',
    'production_params': NAV / 'config' / 'nav2_params.yaml',
    'production_urdf': ROOT / 'jdamr_cube_description' / 'urdf' / 'jdamr_cube.urdf',
}
HARNESS_FILES = {
    name: Path(__file__).resolve().parent / name for name in (
        'amcl_fault_contract.py', 'prepare_amcl_source_overlay.py',
        'prepare_amcl_fault_benchmark.py', 'g002_tf_sanitizer.py',
        'amcl_particle_observer.py', 'run_amcl_determinism_preflight.py')}


def _record(path: Path) -> dict:
    return exact_regular_file(path)


def _validate_overlay(overlay_root: Path) -> dict:
    overlay_root = overlay_root.expanduser()
    if (not overlay_root.is_absolute() or overlay_root.is_symlink() or
            not overlay_root.is_dir() or overlay_root != overlay_root.resolve()):
        raise ValueError('overlay root must be canonical')
    lock_path = overlay_root / 'UPSTREAM_LOCK.json'
    lock = strict_json_load(lock_path)
    if set(lock) != {
            'schema_version', 'scope', 'upstream', 'baseline_tree',
            'overlay_tree', 'changed_files', 'unchanged_pf_c',
            'patch_classification'}:
        raise ValueError('upstream lock schema drift')
    if lock.get('schema_version') != 1:
        raise ValueError('unsupported upstream lock schema')
    if lock['scope'] != 'evaluation-only; not a production install':
        raise ValueError('upstream lock scope drift')
    if lock['patch_classification'] != (
            'upstream random_seed surface plus local pf_pdf corrective'):
        raise ValueError('upstream patch classification drift')
    upstream = lock.get('upstream', {})
    if set(upstream) != {
            'commit', 'archive_url', 'archive_size_bytes',
            'archive_sha256', 'license'}:
        raise ValueError('upstream identity schema drift')
    if upstream['commit'] != UPSTREAM_COMMIT:
        raise ValueError('upstream commit drift')
    if (upstream['archive_url'] != UPSTREAM_ARCHIVE_URL or
            upstream['archive_size_bytes'] != UPSTREAM_ARCHIVE_SIZE_BYTES or
            upstream['archive_sha256'] != UPSTREAM_ARCHIVE_SHA256):
        raise ValueError('upstream archive hash drift')
    license_record = upstream['license']
    if (set(license_record) != {'relative_path', 'size_bytes', 'sha256'} or
            license_record['relative_path'] != 'LICENSE' or
            license_record['sha256'] != UPSTREAM_LICENSE_SHA256):
        raise ValueError('upstream license schema drift')
    actual_license = exact_regular_file(
        overlay_root / 'LICENSE', license_record['sha256'])
    if actual_license['size_bytes'] != license_record['size_bytes']:
        raise ValueError('upstream license size drift')
    if lock.get('changed_files') != sorted(OVERLAY_CHANGED_FILES):
        raise ValueError('overlay allowlist drift')
    if lock.get('unchanged_pf_c') is not True:
        raise ValueError('pf.c invariant missing')
    baseline_tree = lock['baseline_tree']
    if (set(baseline_tree) != {'file_count', 'files', 'tree_sha256'} or
            baseline_tree['file_count'] != UPSTREAM_AMCL_TREE_FILE_COUNT or
            baseline_tree['tree_sha256'] != UPSTREAM_AMCL_TREE_SHA256):
        raise ValueError('baseline full-tree identity drift')
    actual_tree = canonical_source_tree(overlay_root / 'nav2_amcl')
    if actual_tree != lock['overlay_tree']:
        raise ValueError('overlay full-tree identity drift')
    if validate_source_tree_delta(baseline_tree, actual_tree) != sorted(
            OVERLAY_CHANGED_FILES):
        raise ValueError('overlay changed-file allowlist drift')
    if {child.name for child in overlay_root.iterdir()} != {
            'nav2_amcl', 'LICENSE', 'UPSTREAM_LOCK.json'}:
        raise ValueError('overlay root inventory drift')
    return {'root': str(overlay_root), 'lock': _record(lock_path),
            'changed_files': sorted(OVERLAY_CHANGED_FILES)}


def _axis_b_identity(g004_root: Path) -> dict:
    if (not g004_root.is_absolute() or g004_root.is_symlink() or
            g004_root != g004_root.resolve() or not g004_root.is_dir()):
        raise ValueError('G004 root must be canonical')
    if g004_root != G004_ROOT:
        raise ValueError('G004 root is not the frozen final root')
    contract_path = g004_root / 'contract.json'
    exact_regular_file(contract_path, G004_CONTRACT_SHA256)
    contract = strict_json_load(contract_path)
    assets = contract['evaluation_assets']
    if float(contract['lidar_rate_hz']) != 10.0:
        raise ValueError('Axis B requires the frozen 10 Hz G004 LiDAR')
    selected = {}
    for key in ('slam_corridor_eval.yaml', 'slam_corridor_eval.pgm',
                'world', 'urdf'):
        record = assets[key]
        path = Path(record['path'])
        expected_asset_root = g004_root / 'assets'
        if (path.parent != expected_asset_root or path.is_symlink() or
                path != path.resolve()):
            raise ValueError(f'Axis B asset escaped final root: {key}')
        actual = exact_regular_file(path, record['sha256'])
        if actual['size_bytes'] != record['size_bytes']:
            raise ValueError(f'Axis B asset size drift: {key}')
        source_path = Path(record['source_path'])
        expected_source_root = NAV / 'evaluation' / 'assets' / 'nav_obstacle'
        if (source_path.parent != expected_source_root or
                source_path.is_symlink() or source_path != source_path.resolve()):
            raise ValueError(f'Axis B source escaped repository assets: {key}')
        source_actual = exact_regular_file(
            source_path, record['source_sha256'])
        if source_actual['size_bytes'] != record['source_size_bytes']:
            raise ValueError(f'Axis B source size drift: {key}')
        selected[key] = {
            'path': str(path), 'size_bytes': record['size_bytes'],
            'sha256': record['sha256'],
            'source_path': record['source_path'],
            'source_size_bytes': record['source_size_bytes'],
            'source_sha256': record['source_sha256'],
        }
    urdf_tree = ET.parse(Path(selected['urdf']['path']))
    lidar = urdf_tree.find('.//sensor[@type="gpu_lidar"]/lidar/scan/horizontal')
    update_rate = urdf_tree.find('.//sensor[@type="gpu_lidar"]/update_rate')
    if lidar is None or update_rate is None:
        raise ValueError('Axis B LiDAR geometry missing')
    samples = int(lidar.findtext('samples', '0'))
    angle_min = float(lidar.findtext('min_angle', 'nan'))
    angle_max = float(lidar.findtext('max_angle', 'nan'))
    rate_hz = float(update_rate.text)
    if samples < 2 or not all(map(math.isfinite, (angle_min, angle_max))):
        raise ValueError('Axis B LiDAR geometry invalid')
    if rate_hz != 10.0:
        raise ValueError('Axis B URDF LiDAR rate is not 10 Hz')
    map_profile = map_identity(
        Path(selected['slam_corridor_eval.yaml']['path']),
        selected['slam_corridor_eval.yaml']['sha256'],
        selected['slam_corridor_eval.pgm']['sha256'])
    return {'g004_contract': _record(contract_path),
            'lidar_profile': {
                'rate_hz': rate_hz, 'samples': samples,
                'angle_min_rad': angle_min, 'angle_max_rad': angle_max,
                'angle_increment_rad': (angle_max - angle_min) / (samples - 1),
            }, 'map_profile': map_profile, 'assets': selected}


def prepare_benchmark(output_root: Path, overlay_root: Path,
                      g004_root: Path = G004_ROOT,
                      projected_durable_bytes: int = 0,
                      projected_scratch_bytes: int = 0) -> dict:
    """Create a small immutable contract; no AMCL build or replay occurs."""
    output_root = output_root.expanduser()
    if (not output_root.is_absolute() or output_root.exists() or
            output_root.is_symlink() or output_root.parent != output_root.parent.resolve()):
        raise ValueError('output root must be absent and canonical')
    output_root.parent.mkdir(parents=True, exist_ok=True)
    budget = validate_storage_budget(
        current_free_bytes(output_root.parent), projected_durable_bytes,
        projected_scratch_bytes)
    if not budget['pass']:
        raise RuntimeError(f'storage preflight failed: {budget["failures"]}')
    axis_a_mcap = REAL_BAG_ROOT / REAL_BAG_NAME
    axis_a = {
        'map': map_identity(
            REAL_MAP, REAL_MAP_YAML_SHA256, REAL_MAP_PGM_SHA256),
        'source_bag': _record(axis_a_mcap),
        'source_metadata': _record(REAL_BAG_ROOT / 'metadata.yaml'),
        'source_mcap_expected_sha256': REAL_BAG_SHA256,
        'source_topic_inventory': dict(REAL_BAG_TOPIC_INVENTORY),
        'sanitizer': {
            'removed_map_to_odom_transforms': 7290,
            'publish_topics': ['/scan', '/odom', '/tf', '/tf_static'],
            'recorded_amcl_pose_use': 'offline-reference-only',
            'single_scratch_output': True,
        },
        'matrix': [
            {'profile': profile, 'seed': seed,
             'run_id': f'axis_a__{profile}__seed_{seed}'}
            for profile in PROFILES for seed in SEEDS],
    }
    if axis_a['source_bag']['sha256'] != REAL_BAG_SHA256:
        raise ValueError('Axis A canonical input SHA drift')
    axis_b = _axis_b_identity(g004_root)
    contract = {
        'schema_version': 1,
        'claim_scope': (
            'evaluation-only AMCL replay; no GT/ATE/RPE, physical safety, '
            'production qualification, or statistical-generalization claim'),
        'source_overlay': _validate_overlay(overlay_root),
        'axis_a': axis_a,
        'axis_b': {
            **axis_b,
            'scenarios': list(AXIS_B_SCENARIOS),
            'scenario_contracts': {
                'correct_init': {
                    'true_pose': [-8.0, 0.0, 0.0],
                    'initial_estimate_offset': [0.0, 0.0, 0.0]},
                'initial_offset': {
                    'true_pose': [-8.0, 0.0, 0.0],
                    'translation_offset_m': 0.5,
                    'yaw_offset_rad': math.pi / 12.0},
                'kidnapped': {
                    'start_pose': [-8.0, 0.0, 0.0],
                    'teleport_pose': [0.0, 0.0, math.pi],
                    'observation_rotation_rad': 2.0 * math.pi,
                    'translation_jump_tolerance_m': (
                        2.0 * axis_b['map_profile'][
                            'resolution_m_per_cell']),
                    'yaw_jump_tolerance_rad': axis_b['lidar_profile'][
                        'angle_increment_rad'],
                    'recovery_translation_m': 0.15,
                    'recovery_yaw_rad': 0.25,
                    'consecutive_pose_count': 3,
                },
            },
            'input_bag_policy': {
                'canonical_inputs': 3,
                'each_limit_bytes': STORAGE_LIMITS['axis_b_bag_limit_bytes'],
                'total_limit_bytes': STORAGE_LIMITS[
                    'axis_b_total_bag_limit_bytes'],
            },
        },
        'profiles': {
            'P0': {'min_particles': 500, 'max_particles': 2000,
                   'pf_err': 0.05, 'pf_z': 0.99,
                   'recovery_alpha_fast': 0.0,
                   'recovery_alpha_slow': 0.0},
            'P1': {'min_particles': 2000, 'max_particles': 2000,
                   'pf_err': 0.05, 'pf_z': 0.99,
                   'recovery_alpha_fast': 0.0,
                   'recovery_alpha_slow': 0.0},
            'P2': {'min_particles': 500, 'max_particles': 2000,
                   'pf_err': 0.05, 'pf_z': 0.99,
                   'recovery_alpha_fast': 0.1,
                   'recovery_alpha_slow': 0.001},
        },
        'storage_contract': budget,
        'run_artifact_policy': {
            'output_mcap_count': 0,
            'per_run_limit_bytes': G002_RUN_OUTPUT_LIMIT_BYTES,
            'total_limit_bytes': G002_ARTIFACT_LIMIT_BYTES,
            'repeated_runs': 'metrics-only',
        },
        'production_inputs': {
            name: _record(path) for name, path in PRODUCTION_FILES.items()},
        'harness_sources': {
            name: _record(path) for name, path in HARNESS_FILES.items()},
    }
    with tempfile.TemporaryDirectory(
            prefix='.g002-prepare-', dir=output_root.parent) as temp_name:
        stage = Path(temp_name) / 'prepared'
        stage.mkdir()
        (stage / 'contract.json').write_bytes(canonical_json_bytes(contract))
        contract_path = stage / 'contract.json'
        files = {'contract.json': {
            'relative_path': 'contract.json',
            'size_bytes': contract_path.stat().st_size,
            'sha256': sha256_file(contract_path)}}
        manifest = {'schema_version': 1, 'files': files,
                    'tree_payload_sha256': sha256_file(stage / 'contract.json')}
        (stage / 'preparation_manifest.json').write_bytes(
            canonical_json_bytes(manifest))
        stage.rename(output_root)
    return contract


def main() -> int:
    """Prepare and verify the G002 benchmark contract assets."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--overlay-root', type=Path, required=True)
    parser.add_argument('--g004-root', type=Path, default=G004_ROOT)
    parser.add_argument('--projected-durable-bytes', type=int, default=0)
    parser.add_argument('--projected-scratch-bytes', type=int, default=0)
    args = parser.parse_args()
    prepare_benchmark(
        args.output_root, args.overlay_root, args.g004_root,
        args.projected_durable_bytes, args.projected_scratch_bytes)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
