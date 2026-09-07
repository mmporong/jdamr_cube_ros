#!/usr/bin/env python3
"""Boundary tests for the G009 Axis B failure taxonomy."""

from pathlib import Path

import classify_amcl_axis_b_failures as classifier

import pytest


def _sample(index, translation, yaw, sigma_translation=1.0, sigma_yaw=1.0):
    return {
        'scan_index': index,
        'translation_error_m': translation,
        'yaw_error_rad': yaw,
        'three_sigma_translation_m': sigma_translation,
        'three_sigma_yaw_rad': sigma_yaw,
    }


def test_false_confidence_uses_strict_error_and_three_sample_gate():
    """Require strict error excess and exactly three consecutive samples."""
    boundary = _sample(1, 0.15, 0.25, 0.15, 0.25)
    wrong = _sample(2, 0.150000001, 0.25, 0.15, 0.25)
    metrics = {'samples': [boundary, wrong, wrong]}
    summary = classifier._sample_summary(metrics, 0)
    assert summary['longest_false_confidence_streak'] == 2
    metrics['samples'].append(wrong)
    summary = classifier._sample_summary(metrics, 0)
    assert summary['longest_false_confidence_streak'] == 3


def test_recovery_summary_resets_streak_before_t0():
    """Count recovery evidence only from the scenario t0 boundary."""
    metrics = {'samples': [
        _sample(0, 0.15, 0.25),
        _sample(1, 0.15, 0.25),
        _sample(2, 0.16, 0.25),
        _sample(3, 0.15, 0.25),
        _sample(4, 0.15, 0.25),
        _sample(5, 0.15, 0.25),
    ]}
    summary = classifier._sample_summary(metrics, 2)
    assert summary['longest_post_t0_in_threshold_streak'] == 3
    assert summary['post_t0_sample_count'] == 4


def test_non_recovery_taxonomy_preserves_observability_limits():
    """Keep unobserved particle causes explicit instead of inventing one."""
    run = {
        'run_id': 'axis_b__kidnapped__P0__seed_11',
        'scenario': 'kidnapped', 'profile': 'P0', 'seed': 11,
        '_evidence_identity': {
            'relative_path': 'run_1/axis_b_evidence.json',
            'size_bytes': 1, 'sha256': '0' * 64},
        'metrics': {'false_convergence': False},
    }
    summary = {
        'sample_count': 3, 'post_t0_sample_count': 3,
        'longest_false_confidence_streak': 0,
        'longest_post_t0_in_threshold_streak': 0,
        'minimum_post_t0_normalized_error': 40.0,
    }
    result = classifier._non_recovery_taxonomy(run, summary)
    assert [row['category'] for row in result['ordered_findings']] == \
        list(classifier.TAXONOMY_ORDER)
    assert result['ordered_findings'][0]['status'] == 'NOT_OBSERVABLE'
    assert result['ordered_findings'][1]['status'] == 'NOT_OBSERVABLE'
    assert result['ordered_findings'][2]['status'] == 'RULED_OUT'
    assert result['ordered_findings'][3]['status'] == 'NOT_SUPPORTED'
    assert result['ordered_findings'][4]['status'] == 'NOT_ESTABLISHED'
    assert result['classification'] == \
        'UNRESOLVED_WITH_FROZEN_G002_TELEMETRY'


def test_resource_outliers_reuse_frozen_g002_limit():
    """Reuse the preregistered strict 1.10 resource ceiling."""
    promotion = {'axis_a_ratios': {
        'cpu_ratios_by_seed': [1.0, 1.100000001, 1.10, 1.2, 0.9],
        'latency_p95_ratios_by_seed': [1.0, 1.0, 1.0, 1.0, 1.0],
    }}
    result = classifier._resource_outliers(promotion)
    assert [(row['seed'], row['metric']) for row in result] == [
        (23, 'CPU_RATIO_P2_OVER_P0'),
        (67, 'CPU_RATIO_P2_OVER_P0'),
    ]
    assert all(row['causal_interpretation'] ==
               'PROCESSING_DELAY_NOT_PROVEN' for row in result)


def test_resource_outliers_reject_ratio_cardinality_drift():
    """Reject incomplete seed coverage before selecting outliers."""
    promotion = {'axis_a_ratios': {
        'cpu_ratios_by_seed': [1.0],
        'latency_p95_ratios_by_seed': [1.0] * 5,
    }}
    try:
        classifier._resource_outliers(promotion)
    except ValueError as exc:
        assert 'cardinality' in str(exc)
    else:
        raise AssertionError('ratio cardinality drift was accepted')


def test_artifact_validator_rejects_taxonomy_byte_tampering(
        monkeypatch, tmp_path):
    """Bind the decision to canonical taxonomy bytes and tree identity."""
    fake_taxonomy = {
        'source_axis_b': {
            'manifest': {'path': str(tmp_path / 'axis_b_manifest.json')},
        },
        'decision': classifier.DECISION_NO_CHANGE,
    }
    monkeypatch.setattr(
        classifier, 'build_taxonomy', lambda _path: fake_taxonomy)
    output = tmp_path / 'artifact'
    manifest_path = classifier.write_artifact(
        Path('/unused/axis_b_manifest.json'), output)
    assert classifier.validate_artifact(manifest_path)['decision'] == \
        classifier.DECISION_NO_CHANGE
    taxonomy_path = output / 'failure_taxonomy.json'
    taxonomy_path.write_bytes(taxonomy_path.read_bytes() + b' ')
    with pytest.raises(ValueError, match='file or source snapshot drift'):
        classifier.validate_artifact(manifest_path)


def test_tree_records_exclude_only_root_manifest(tmp_path):
    """Include a nested same-name payload in the canonical tree."""
    (tmp_path / 'g009_manifest.json').write_text('{}', encoding='utf-8')
    nested = tmp_path / 'nested'
    nested.mkdir()
    (nested / 'g009_manifest.json').write_text('{}', encoding='utf-8')
    paths = [row['relative_path'] for row in
             classifier._tree_records(tmp_path)]
    assert 'g009_manifest.json' not in paths
    assert 'nested/g009_manifest.json' in paths


def test_noncanonical_paths_are_rejected_before_validation(
        monkeypatch, tmp_path):
    """Reject relative and symlink-parent aliases before canonicalization."""
    called = False

    def unexpected_validator(*_args):
        nonlocal called
        called = True

    monkeypatch.setattr(classifier.axis_b, 'validate_manifest',
                        unexpected_validator)
    with pytest.raises(ValueError, match='must be canonical'):
        classifier.build_taxonomy(
            Path('relative/axis_b_manifest.json'))
    real_parent = tmp_path / 'real'
    real_parent.mkdir()
    alias_parent = tmp_path / 'alias'
    alias_parent.symlink_to(real_parent, target_is_directory=True)
    with pytest.raises(ValueError, match='must be canonical'):
        classifier.build_taxonomy(alias_parent / 'axis_b_manifest.json')
    with pytest.raises(ValueError, match='must be absent and canonical'):
        classifier.write_artifact(
            tmp_path / 'axis_b_manifest.json', alias_parent / 'output')
    assert called is False
