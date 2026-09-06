#!/usr/bin/env python3
"""Run metrics-only G002 Axis A AMCL evaluation."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import tempfile

from amcl_fault_contract import canonical_json_bytes, current_free_bytes
from amcl_fault_contract import STORAGE_LIMITS, strict_json_load
from evaluate_amcl_axis_a import _tree_bytes, _tree_digest, _tree_records
from evaluate_amcl_axis_a import CLAIM_FULL, CLAIM_SMOKE, FULL_PLAN, PROFILES
from evaluate_amcl_axis_a import derive_metrics, pair_recorded, profile_overrides
from evaluate_amcl_axis_a import EVALUATOR, OBSERVER, PREFLIGHT, RUNNER
from evaluate_amcl_axis_a import recorded_reference
from evaluate_amcl_axis_a import validate_axis_a_full, validate_axis_a_smoke
import run_amcl_determinism_preflight as preflight


def _canonical_root(path: Path) -> Path:
    path = path.expanduser()
    if not path.is_absolute() or path.exists() or path.parent.is_symlink():
        raise ValueError('output root must be absent and absolute')
    if path.parent.resolve() != path.parent:
        raise ValueError('output parent must be canonical')
    return path


def run(args) -> dict:
    """Execute smoke or the exact preregistered fifteen-run matrix."""
    output_root = _canonical_root(args.output_root)
    plan = [('P0', 11)] if args.smoke else list(FULL_PLAN)
    if current_free_bytes(output_root.parent) < STORAGE_LIMITS['minimum_start_free_bytes']:
        raise RuntimeError('less than 6 GiB free before Axis A evaluation')
    contract = strict_json_load(args.prepared_root / 'contract.json')
    preflight._validate_source_records(contract['production_inputs'])
    sanitized = strict_json_load(args.sanitized_root / 'sanitizer_manifest.json')
    attestation = strict_json_load(
        args.attestation_root / 'build_attestation.json')
    if (contract['axis_a']['source_bag']['sha256'] !=
            sanitized['source']['mcap_sha256'] or
            attestation['loaded_runtime']['amcl_executable'] !=
            preflight._identity(args.amcl_executable) or
            attestation['loaded_runtime']['rmw_library'] !=
            preflight._identity(args.rmw_library)):
        raise ValueError('Axis A source or runtime attestation drift')
    source_root = Path(sanitized['source']['path'])
    recorded, recorded_identity = recorded_reference(source_root)
    with tempfile.TemporaryDirectory(
            prefix='.g002-axis-a-', dir=output_root.parent) as temp_name:
        stage = Path(temp_name) / 'artifact'
        stage.mkdir()
        for source, target in (
                (args.prepared_root / 'contract.json', 'contract_snapshot.json'),
                (args.sanitized_root / 'sanitizer_manifest.json',
                 'sanitizer_manifest_snapshot.json'),
                (args.attestation_root / 'build_attestation.json',
                 'runtime_attestation_snapshot.json')):
            shutil.copyfile(source, stage / target)
        runs = []
        plan_records = []
        bootstrap = preflight._tf_bootstrap_plan(args.sanitized_root)
        for index, (profile, seed) in enumerate(plan):
            domain_id = args.domain_base + index
            run_id = f'axis_a__{profile}__seed_{seed}'
            plan_records.append({'run_id': run_id, 'profile': profile,
                                 'seed': seed, 'domain_id': domain_id})
            args.attempt_index = index + 1
            args.profile = profile
            args.run_id = run_id
            args.amcl_overrides = profile_overrides(profile)
            base = preflight._run_one(
                stage / f'run_{index + 1}', seed, domain_id,
                args, dict(os.environ), bootstrap)
            if base['status'] != 'PASS':
                raise RuntimeError(f'Axis A run failed: {base["failure"]}')
            pairs = pair_recorded(base['observer']['clouds'], recorded)
            base_path = stage / f'run_{index + 1}/evidence.json'
            axis = {
                'schema_version': 1, 'run_id': run_id, 'profile': profile,
                'seed': seed, 'domain_id': domain_id, 'status': 'PASS',
                'failure': None,
                'base_evidence': preflight._relative_identity(base_path, stage),
                'profile_parameters': PROFILES[profile],
                'recorded_pose_pairs': pairs,
                'metrics': derive_metrics(base, pairs),
                'map_odom_authority': {
                    'sanitized_input_count': 0,
                    'generated_observed_count':
                        base['observer']['readiness']['map_odom_tf_count'],
                    'sole_runtime_authority': True,
                },
            }
            axis_path = stage / f'run_{index + 1}/axis_a_evidence.json'
            axis_path.write_bytes(canonical_json_bytes(axis))
            runs.append(preflight._relative_identity(axis_path, stage))
        mode = 'smoke' if args.smoke else 'full'
        records = _tree_records(stage)
        manifest = {
            'schema_version': 1, 'mode': mode,
            'claim_scope': CLAIM_SMOKE if args.smoke else CLAIM_FULL,
            'plan': plan_records,
            'source_contract': preflight._relative_identity(
                stage / 'contract_snapshot.json', stage),
            'sanitizer_snapshot': preflight._relative_identity(
                stage / 'sanitizer_manifest_snapshot.json', stage),
            'runtime_attestation': preflight._relative_identity(
                stage / 'runtime_attestation_snapshot.json', stage),
            'harness_sources': {
                path.name: preflight._identity(path)
                for path in (EVALUATOR, RUNNER, OBSERVER, PREFLIGHT)},
            'runs': runs, 'tree_records': records,
            'run_contract': {
                'max_clouds': args.max_clouds, 'prefix_s': args.prefix_s,
                'playback_rate': args.playback_rate, 'output_mcap_count': 0,
                'per_run_limit_bytes':
                    STORAGE_LIMITS['run_output_limit_bytes']},
            'recorded_reference': recorded_identity,
            'tree_sha256': _tree_digest(records),
            'tree_bytes': _tree_bytes(stage),
            'production_unchanged': True,
        }
        (stage / 'axis_a_manifest.json').write_bytes(
            canonical_json_bytes(manifest))
        validator = validate_axis_a_smoke if args.smoke else validate_axis_a_full
        validator(stage)
        preflight._validate_source_records(contract['production_inputs'])
        stage.rename(output_root)
    preflight._validate_source_records(contract['production_inputs'])
    try:
        return validator(output_root)
    except Exception:
        preflight._safe_remove_created_root(output_root)
        raise


def main() -> int:
    """Parse Axis A evaluation arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--prepared-root', required=True, type=Path)
    parser.add_argument('--sanitized-root', required=True, type=Path)
    parser.add_argument('--attestation-root', required=True, type=Path)
    parser.add_argument('--amcl-executable', required=True, type=Path)
    parser.add_argument('--params-file', required=True, type=Path)
    parser.add_argument('--map-yaml', required=True, type=Path)
    parser.add_argument('--rmw-library', required=True, type=Path)
    parser.add_argument('--domain-base', type=int, default=180)
    parser.add_argument('--max-clouds', type=int, default=30)
    parser.add_argument('--prefix-s', type=float, default=220.0)
    parser.add_argument('--playback-rate', type=float, default=2.0)
    parser.add_argument('--smoke', action='store_true')
    args = parser.parse_args()
    if args.smoke:
        args.max_clouds = 1
        args.prefix_s = 30.0
    run(args)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
