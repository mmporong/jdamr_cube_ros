#!/usr/bin/env python3
"""Prepare generated G003 Nav2 parameters and a machine-readable contract."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from sim_nav_obstacle_contract import (
    derive_contract, GOAL_POSE, scenario_matrix, SCENARIOS, START_POSE)

import yaml


ROOT = Path(__file__).resolve().parents[2]
PRODUCTION_PARAMS = (ROOT / 'jdamr_cube_navigation' / 'config'
                     / 'nav2_params.yaml')
EVALUATION_BT = (ROOT / 'jdamr_cube_navigation' / 'behavior_trees'
                 / 'navigate_to_pose_dynamic_obstacle_eval.xml')
ASSET_MANIFEST = (ROOT / 'jdamr_cube_navigation' / 'evaluation' / 'assets'
                  / 'nav_obstacle' / 'asset_manifest.json')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def archive_existing_attempt(run_dir: Path) -> Path | None:
    """Move an existing runner-owned attempt into a hashed audit directory."""
    if not run_dir.exists() or not any(run_dir.iterdir()):
        return None
    audit_root = run_dir.parent / 'development_audit'
    audit_root.mkdir(exist_ok=True)
    attempt = 1
    while (audit_root / f'{run_dir.name}_attempt_{attempt}').exists():
        attempt += 1
    archived = audit_root / f'{run_dir.name}_attempt_{attempt}'
    run_dir.rename(archived)
    files = []
    for path in sorted(item for item in archived.rglob('*') if item.is_file()):
        files.append({
            'path': str(path.relative_to(archived)),
            'size_bytes': path.stat().st_size,
            'sha256': _sha256(path),
        })
    (archived / 'attempt_manifest.json').write_text(json.dumps({
        'schema_version': 1,
        'run_id': run_dir.name,
        'attempt': attempt,
        'files': files,
    }, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return archived


def prepare(output_dir: Path) -> dict[str, Path]:
    """Write evaluation-only config derived from production config."""
    output_dir.mkdir(parents=True, exist_ok=True)
    params = yaml.safe_load(PRODUCTION_PARAMS.read_text(encoding='utf-8'))
    initial = params['amcl']['ros__parameters']['initial_pose']
    initial.update({
        'x': START_POSE['x_m'],
        'y': START_POSE['y_m'],
        'z': 0.0,
        'yaw': START_POSE['yaw_rad'],
    })
    params['bt_navigator']['ros__parameters'][
        'default_nav_to_pose_bt_xml'] = str(EVALUATION_BT.resolve())
    for name in ('local_costmap', 'global_costmap'):
        scan = params[name][name]['ros__parameters'][
            'obstacle_layer']['scan']
        scan['observation_persistence'] = 0.0
        scan['inf_is_valid'] = True
        params[name][name]['ros__parameters'][
            'always_send_full_costmap'] = True

    params_path = output_dir / 'nav2_obstacle_eval.params.yaml'
    params_path.write_text(
        yaml.safe_dump(params, sort_keys=False), encoding='utf-8')
    derived = derive_contract(params)
    asset_manifest = json.loads(ASSET_MANIFEST.read_text(encoding='utf-8'))
    contract = {
        **derived,
        'schema_version': 1,
        'start_pose': START_POSE,
        'goal_pose': GOAL_POSE,
        'scenarios': SCENARIOS,
        'scenario_count': len(SCENARIOS),
        'run_count': len(scenario_matrix()),
        'seeds': sorted({item['seed'] for item in scenario_matrix()}),
        'observation_persistence_s': 0.0,
        'simulation_support_plane_correction': asset_manifest[
            'simulation_support_plane_correction'],
        'preloaded_obstacles': asset_manifest['preloaded_obstacles'],
        'contact_scope': asset_manifest[
            'simulation_support_plane_correction']['contact_scope'],
        'production_params': {
            'path': str(PRODUCTION_PARAMS.resolve()),
            'sha256': _sha256(PRODUCTION_PARAMS),
        },
        'evaluation_bt': {
            'path': str(EVALUATION_BT.resolve()),
            'sha256': _sha256(EVALUATION_BT),
        },
        'claim_scope': (
            'Gazebo의 고정 장애물 재계획 평가이며 이동 물체·사람 안전·'
            '실차 충돌 안전을 입증하지 않는다.'),
    }
    contract_path = output_dir / 'contract.json'
    contract_path.write_text(
        json.dumps(contract, ensure_ascii=False, indent=2,
                   sort_keys=True) + '\n', encoding='utf-8')
    return {'params': params_path, 'contract': contract_path}


def main() -> int:
    """Generate the run contract and evaluation-only parameter file."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    outputs = prepare(args.output)
    print(json.dumps({key: str(value) for key, value in outputs.items()},
                     sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
