#!/usr/bin/env python3
"""Freeze and validate the G009 failure taxonomy for Axis B full45."""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import shutil
import tempfile

import amcl_fault_contract as fault_contract

import evaluate_amcl_axis_b as axis_b


SEEDS = fault_contract.SEEDS
canonical_json_bytes = fault_contract.canonical_json_bytes
sha256_file = fault_contract.sha256_file
strict_json_load = fault_contract.strict_json_load
SCHEMA_VERSION = 1
DECISION_NO_CHANGE = 'NO_FALSE_CONFIDENCE_OBSERVED_NO_CHANGE_JUSTIFIED'
DECISION_SELECTION_REQUIRED = 'FALSE_CONFIDENCE_OBSERVED_SELECTION_REQUIRED'
TAXONOMY_ORDER = (
    'GROUND_TRUTH_NEAR_PARTICLE_ABSENCE',
    'CORRECT_CANDIDATE_DISAPPEARANCE',
    'WRONG_POSE_CONFIDENT_CONVERGENCE',
    'OVERLY_CONSERVATIVE_DECISION_THRESHOLD',
    'PROCESSING_DELAY',
)
# G002 fixed the promotion ceiling at 1.10 relative to P0. G009 reuses it
# instead of defining a post-hoc outlier threshold after reading the results.
RESOURCE_RATIO_LIMIT = 1.10
RESOURCE_METRICS = (
    ('CPU_RATIO_P2_OVER_P0', 'cpu_ratios_by_seed'),
    ('LATENCY_P95_RATIO_P2_OVER_P0', 'latency_p95_ratios_by_seed'),
)


def _exact_keys(value: object, expected: set[str], label: str) -> None:
    if type(value) is not dict or set(value) != expected:
        raise ValueError(f'{label} schema drift')


def _identity(path: pathlib.Path) -> dict:
    if not path.is_file() or path.is_symlink():
        raise ValueError(f'non-regular evidence file: {path}')
    return {
        'path': str(path.resolve()),
        'size_bytes': path.stat().st_size,
        'sha256': sha256_file(path),
    }


def _relative_identity(path: pathlib.Path, root: pathlib.Path) -> dict:
    identity = _identity(path)
    return {
        'relative_path': path.resolve().relative_to(root.resolve()).as_posix(),
        'size_bytes': identity['size_bytes'],
        'sha256': identity['sha256'],
    }


def _tree_records(root: pathlib.Path) -> list[dict]:
    records = []
    for path in root.rglob('*'):
        if path.is_symlink():
            raise ValueError('G009 artifacts cannot contain symlinks')
        relative = path.relative_to(root).as_posix()
        if path.is_file() and relative != 'g009_manifest.json':
            records.append(_relative_identity(path, root))
    return sorted(records, key=lambda row: row['relative_path'].encode())


def _tree_digest(records: list[dict]) -> str:
    return hashlib.sha256(canonical_json_bytes(records)).hexdigest()


def metric_contract() -> dict:
    """Return the already-frozen G002 thresholds consumed by G009."""
    return {
        'translation_error_strictly_greater_than_m':
            axis_b.TRANSLATION_THRESHOLD_M,
        'yaw_error_strictly_greater_than_rad': axis_b.YAW_THRESHOLD_RAD,
        'three_sigma_translation_at_most_m':
            axis_b.TRANSLATION_THRESHOLD_M,
        'three_sigma_yaw_at_most_rad': axis_b.YAW_THRESHOLD_RAD,
        'consecutive_samples_required': axis_b.CONSECUTIVE_REQUIRED,
        'resource_ratio_strictly_greater_than': RESOURCE_RATIO_LIMIT,
        'resource_reference_profile': 'P0',
        'resource_candidate_profile': 'P2',
    }


def _longest_streak(samples: list[dict], predicate) -> int:
    longest = 0
    current = 0
    for sample in samples:
        current = current + 1 if predicate(sample) else 0
        longest = max(longest, current)
    return longest


def _sample_summary(metrics: dict, t0_scan_index: int) -> dict:
    samples = metrics['samples']
    if type(samples) is not list or not samples:
        raise ValueError('G009 requires non-empty Axis B metric samples')
    required = {
        'scan_index', 'translation_error_m', 'yaw_error_rad',
        'three_sigma_translation_m', 'three_sigma_yaw_rad',
    }
    for sample in samples:
        _exact_keys(sample, required, 'Axis B metric sample')

    def false_confident(row):
        return (
            (row['translation_error_m'] > axis_b.TRANSLATION_THRESHOLD_M or
             row['yaw_error_rad'] > axis_b.YAW_THRESHOLD_RAD) and
            row['three_sigma_translation_m'] <=
            axis_b.TRANSLATION_THRESHOLD_M and
            row['three_sigma_yaw_rad'] <= axis_b.YAW_THRESHOLD_RAD)

    after_t0 = [row for row in samples if row['scan_index'] >= t0_scan_index]
    if not after_t0:
        raise ValueError('G009 requires post-t0 metric samples')

    def inside(row):
        return (
            row['translation_error_m'] <= axis_b.TRANSLATION_THRESHOLD_M and
            row['yaw_error_rad'] <= axis_b.YAW_THRESHOLD_RAD)

    normalized_errors = [max(
        row['translation_error_m'] / axis_b.TRANSLATION_THRESHOLD_M,
        row['yaw_error_rad'] / axis_b.YAW_THRESHOLD_RAD,
    ) for row in after_t0]
    return {
        'sample_count': len(samples),
        'post_t0_sample_count': len(after_t0),
        'longest_false_confidence_streak': _longest_streak(
            samples, false_confident),
        'longest_post_t0_in_threshold_streak': _longest_streak(
            after_t0, inside),
        'minimum_post_t0_normalized_error': min(normalized_errors),
    }


def _non_recovery_taxonomy(run: dict, summary: dict) -> dict:
    false_status = (
        'OBSERVED' if summary['longest_false_confidence_streak'] >=
        axis_b.CONSECUTIVE_REQUIRED else 'RULED_OUT')
    threshold_status = (
        'SUPPORTED' if summary['longest_post_t0_in_threshold_streak'] >=
        axis_b.CONSECUTIVE_REQUIRED else 'NOT_SUPPORTED')
    ordered_findings = [
        {
            'category': TAXONOMY_ORDER[0],
            'status': 'NOT_OBSERVABLE',
            'evidence': (
                'frozen clouds retain particle count, aggregate pose, '
                'covariance, and payload digest but not particle coordinates '
                'or weights'),
        },
        {
            'category': TAXONOMY_ORDER[1],
            'status': 'NOT_OBSERVABLE',
            'evidence': (
                'the frozen observer has no before/after resampling candidate '
                'mass or resampling event trace'),
        },
        {
            'category': TAXONOMY_ORDER[2],
            'status': false_status,
            'evidence': {
                'longest_consecutive_samples':
                    summary['longest_false_confidence_streak'],
                'required_consecutive_samples': axis_b.CONSECUTIVE_REQUIRED,
                'recorded_false_convergence':
                    run['metrics']['false_convergence'],
            },
        },
        {
            'category': TAXONOMY_ORDER[3],
            'status': threshold_status,
            'evidence': {
                'longest_post_t0_in_threshold_streak':
                    summary['longest_post_t0_in_threshold_streak'],
                'required_consecutive_samples': axis_b.CONSECUTIVE_REQUIRED,
                'minimum_post_t0_normalized_error':
                    summary['minimum_post_t0_normalized_error'],
                'interpretation': (
                    'the frozen criterion did not hide a three-sample '
                    'in-threshold recovery' if threshold_status ==
                    'NOT_SUPPORTED' else
                    'the frozen evidence contradicts the recorded recovery'),
            },
        },
        {
            'category': TAXONOMY_ORDER[4],
            'status': 'NOT_ESTABLISHED',
            'evidence': (
                'resource samples contain CPU and RSS observations but no '
                'pre-registered callback deadline or causal intervention'),
        },
    ]
    return {
        'run_id': run['run_id'],
        'scenario': run['scenario'],
        'profile': run['profile'],
        'seed': run['seed'],
        'evidence': run['_evidence_identity'],
        'sample_summary': summary,
        'ordered_findings': ordered_findings,
        'classification': 'UNRESOLVED_WITH_FROZEN_G002_TELEMETRY',
        'missing_evidence': [
            'per-particle coordinates and weights',
            'before/after resampling candidate mass and event trace',
            'pre-registered callback deadline with causal timing evidence',
        ],
    }


def _resource_outliers(promotion: dict) -> list[dict]:
    ratios = promotion['axis_a_ratios']
    outliers = []
    for metric, key in RESOURCE_METRICS:
        values = ratios[key]
        if type(values) is not list or len(values) != len(SEEDS):
            raise ValueError('G009 resource ratio cardinality drift')
        for seed, value in zip(SEEDS, values):
            if value > RESOURCE_RATIO_LIMIT:
                outliers.append({
                    'category': TAXONOMY_ORDER[4],
                    'metric': metric,
                    'profile': 'P2',
                    'reference_profile': 'P0',
                    'seed': seed,
                    'ratio': value,
                    'limit': RESOURCE_RATIO_LIMIT,
                    'status': 'RESOURCE_COST_OUTLIER',
                    'causal_interpretation': 'PROCESSING_DELAY_NOT_PROVEN',
                })
    return outliers


def build_taxonomy(axis_b_manifest_path: pathlib.Path) -> dict:
    """Revalidate full45 and classify every required G009 observation."""
    manifest_path = axis_b_manifest_path
    if (not manifest_path.is_absolute() or
            manifest_path != manifest_path.resolve() or
            manifest_path.is_symlink()):
        raise ValueError('Axis B manifest path must be canonical')
    manifest = axis_b.validate_manifest(manifest_path, 'full')
    root = manifest_path.parent
    run_rows = []
    non_recoveries = []
    false_convergences = []
    for record in manifest['runs']:
        evidence_path = root / record['relative_path']
        run = strict_json_load(evidence_path)
        run['_evidence_identity'] = record
        summary = _sample_summary(run['metrics'], run['t0_scan_index'])
        row = {
            'run_id': run['run_id'],
            'scenario': run['scenario'],
            'profile': run['profile'],
            'seed': run['seed'],
            'recovered': run['metrics']['recovered'],
            'false_convergence': run['metrics']['false_convergence'],
            'evidence': record,
            'sample_summary': summary,
        }
        run_rows.append(row)
        if not row['recovered']:
            non_recoveries.append(_non_recovery_taxonomy(run, summary))
        if row['false_convergence']:
            false_convergences.append(row)
    false_observed = bool(false_convergences)
    contract = metric_contract()
    resource_outliers = _resource_outliers(manifest['promotion'])
    return {
        'schema_version': SCHEMA_VERSION,
        'source_axis_b': {
            'root': str(root),
            'manifest': _identity(manifest_path),
            'tree_sha256': manifest['tree_sha256'],
            'tree_bytes': manifest['tree_bytes'],
            'claim_scope': manifest['claim_scope'],
        },
        'taxonomy_order': list(TAXONOMY_ORDER),
        'metric_contract': contract,
        'metric_contract_sha256': hashlib.sha256(
            canonical_json_bytes(contract)).hexdigest(),
        'run_inventory': run_rows,
        'non_recoveries': non_recoveries,
        'false_convergences': false_convergences,
        'resource_outliers': resource_outliers,
        'counts': {
            'runs': len(run_rows),
            'non_recoveries': len(non_recoveries),
            'false_convergences': len(false_convergences),
            'resource_outliers': len(resource_outliers),
        },
        'algorithm_change_gate': {
            'enabled': false_observed,
            'reason': (
                'AT_LEAST_ONE_FALSE_CONFIDENCE_RUN_WITH_THREE_SAMPLES'
                if false_observed else
                'NO_FALSE_CONFIDENCE_RUN_WITH_THREE_SAMPLES'),
        },
        'decision': (DECISION_SELECTION_REQUIRED if false_observed else
                     DECISION_NO_CHANGE),
        'production_nav2_amcl_changed': False,
    }


def write_artifact(
        axis_b_manifest_path: pathlib.Path,
        output_root: pathlib.Path) -> pathlib.Path:
    """Write an immutable two-file G009 artifact through an atomic rename."""
    output = output_root
    if (not output.is_absolute() or output != output.resolve() or
            output.exists() or output.is_symlink() or
            not output.parent.is_dir()):
        raise ValueError('G009 output root must be absent and canonical')
    stage = pathlib.Path(tempfile.mkdtemp(
        prefix=f'{output.name}.tmp-', dir=output.parent))
    try:
        taxonomy = build_taxonomy(axis_b_manifest_path)
        taxonomy_path = stage / 'failure_taxonomy.json'
        taxonomy_path.write_bytes(canonical_json_bytes(taxonomy))
        source_dir = stage / 'harness_sources'
        source_dir.mkdir()
        source_snapshot = source_dir / pathlib.Path(__file__).name
        shutil.copyfile(pathlib.Path(__file__).resolve(), source_snapshot)
        records = _tree_records(stage)
        manifest = {
            'schema_version': SCHEMA_VERSION,
            'source_axis_b': taxonomy['source_axis_b'],
            'taxonomy': _relative_identity(taxonomy_path, stage),
            'harness_source': _relative_identity(source_snapshot, stage),
            'tree_records': records,
            'tree_sha256': _tree_digest(records),
            'tree_bytes': sum(row['size_bytes'] for row in records),
            'decision': taxonomy['decision'],
            'production_nav2_amcl_changed': False,
        }
        manifest_path = stage / 'g009_manifest.json'
        manifest_path.write_bytes(canonical_json_bytes(manifest))
        stage.rename(output)
        return output / 'g009_manifest.json'
    except BaseException:
        shutil.rmtree(stage, ignore_errors=True)
        raise


def validate_artifact(manifest_path: pathlib.Path) -> dict:
    """Recompute source evidence, taxonomy, and every artifact identity."""
    path = manifest_path
    if (not path.is_absolute() or path != path.resolve() or path.is_symlink()):
        raise ValueError('G009 manifest path must be canonical')
    root = path.parent
    contains_symlink = any(item.is_symlink() for item in root.rglob('*'))
    if (not path.is_file() or path.is_symlink() or not root.is_dir() or
            root.is_symlink() or contains_symlink):
        raise ValueError('G009 artifact root drift')
    manifest = strict_json_load(path)
    _exact_keys(manifest, {
        'schema_version', 'source_axis_b', 'taxonomy', 'harness_source',
        'tree_records', 'tree_sha256', 'tree_bytes', 'decision',
        'production_nav2_amcl_changed'}, 'G009 manifest')
    if (manifest['schema_version'] != SCHEMA_VERSION or
            manifest['production_nav2_amcl_changed'] is not False):
        raise ValueError('G009 manifest scalar drift')
    taxonomy_path = root / manifest['taxonomy']['relative_path']
    source_path = root / manifest['harness_source']['relative_path']
    if (manifest['taxonomy'] != _relative_identity(taxonomy_path, root) or
            manifest['harness_source'] !=
            _relative_identity(source_path, root) or
            source_path.read_bytes() !=
            pathlib.Path(__file__).resolve().read_bytes()):
        raise ValueError('G009 file or source snapshot drift')
    taxonomy = strict_json_load(taxonomy_path)
    source_manifest = pathlib.Path(
        taxonomy['source_axis_b']['manifest']['path'])
    expected = build_taxonomy(source_manifest)
    if taxonomy != expected or manifest['source_axis_b'] != \
            taxonomy['source_axis_b'] or manifest['decision'] != \
            taxonomy['decision']:
        raise ValueError('G009 taxonomy recomputation drift')
    records = _tree_records(root)
    if (manifest['tree_records'] != records or
            manifest['tree_sha256'] != _tree_digest(records) or
            manifest['tree_bytes'] !=
            sum(row['size_bytes'] for row in records)):
        raise ValueError('G009 artifact tree drift')
    return manifest


def main() -> int:
    """Create or independently validate one G009 taxonomy artifact."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis-b-manifest', type=pathlib.Path)
    parser.add_argument('--output-root', type=pathlib.Path)
    parser.add_argument('--validate-manifest', type=pathlib.Path)
    args = parser.parse_args()
    if args.validate_manifest is not None:
        result = validate_artifact(args.validate_manifest)
        print(f"PASS {result['decision']}")
        return 0
    if args.axis_b_manifest is None or args.output_root is None:
        parser.error('--axis-b-manifest and --output-root are required')
    result_path = write_artifact(args.axis_b_manifest, args.output_root)
    result = validate_artifact(result_path)
    print(f"PASS {result['decision']} {result_path}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
