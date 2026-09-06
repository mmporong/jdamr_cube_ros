#!/usr/bin/env python3
"""Pure metric and promotion contracts for G002 Axis B."""

from __future__ import annotations

import math
import statistics

from amcl_fault_contract import PROFILES, SEEDS, strict_json_load
from generate_amcl_axis_b_input import SCENARIOS


TRANSLATION_THRESHOLD_M = 0.15
YAW_THRESHOLD_RAD = 0.25
CONSECUTIVE_REQUIRED = 3
FULL_PLAN = tuple((scenario, profile, seed)
                  for scenario in SCENARIOS
                  for profile in PROFILES for seed in SEEDS)


def _yaw(pose: list[float]) -> float:
    x, y, z, w = pose[3:7]
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def recovery_metrics(clouds: list[dict], gt_pairs: list[dict],
                     t0_scan_index: int) -> dict:
    """Calculate first three-consecutive recovery and false convergence."""
    if len(clouds) != len(gt_pairs) or type(t0_scan_index) is not int:
        raise ValueError('Axis B causal cardinality drift')
    consecutive = 0
    false_consecutive = 0
    recovery = None
    false_convergence = False
    samples = []
    for cloud, pair in zip(clouds, gt_pairs):
        if pair['scan_index'] < t0_scan_index:
            continue
        translation_m = math.dist(cloud['pose'][:2], pair['gt_pose'][:2])
        yaw_rad = abs(math.remainder(
            _yaw(cloud['pose']) - _yaw(pair['gt_pose']), 2.0 * math.pi))
        covariance = cloud['covariance']
        sigma_translation_m = 3.0 * math.sqrt(max(covariance[0], covariance[7]))
        sigma_yaw_rad = 3.0 * math.sqrt(covariance[35])
        inside = (translation_m <= TRANSLATION_THRESHOLD_M and
                  yaw_rad <= YAW_THRESHOLD_RAD)
        confident = (sigma_translation_m <= TRANSLATION_THRESHOLD_M and
                     sigma_yaw_rad <= YAW_THRESHOLD_RAD)
        consecutive = consecutive + 1 if inside else 0
        false_consecutive = false_consecutive + 1 if confident and not inside else 0
        samples.append({
            'scan_index': pair['scan_index'],
            'translation_error_m': translation_m, 'yaw_error_rad': yaw_rad,
            'three_sigma_translation_m': sigma_translation_m,
            'three_sigma_yaw_rad': sigma_yaw_rad,
        })
        if consecutive >= CONSECUTIVE_REQUIRED and recovery is None:
            recovery = {
                'scan_index': pair['scan_index'],
                'scans_after_t0': pair['scan_index'] - t0_scan_index,
                'triggering_scan_header_stamp_ns':
                    cloud['triggering_scan_header_stamp_ns'],
            }
        if false_consecutive >= CONSECUTIVE_REQUIRED:
            false_convergence = True
    return {'recovered': recovery is not None, 'first_recovery': recovery,
            'false_convergence': false_convergence, 'samples': samples}


def promotion_decision(full_results: list[dict], axis_a_ratios: dict | None) -> dict:
    """Apply the preregistered P2 promotion gate without fallback."""
    if len(full_results) != 45 or axis_a_ratios is None:
        return {'status': 'NOT_EVALUATED', 'promote_p2': False}
    by_key = {(row['scenario'], row['profile'], row['seed']): row
              for row in full_results}
    if set(by_key) != set(FULL_PLAN):
        raise ValueError('Axis B full result set drift')
    kidnapped = [
        (by_key[('kidnapped', 'P0', seed)],
         by_key[('kidnapped', 'P2', seed)]) for seed in SEEDS]
    p2_recovers = all(right['recovered'] for _, right in kidnapped)
    false_count = sum(right['false_convergence'] for _, right in kidnapped)
    non_worse = [right['recovery_scan'] <= left['recovery_scan']
                 for left, right in kidnapped]
    improvements = [left['recovery_scan'] - right['recovery_scan']
                    for left, right in kidnapped]
    correct_offset = [by_key[(scenario, 'P2', seed)]['recovered']
                      for scenario in ('correct_init', 'initial_offset')
                      for seed in SEEDS]
    regressions = []
    for scenario in ('correct_init', 'initial_offset'):
        for seed in SEEDS:
            regressions.append(
                by_key[(scenario, 'P2', seed)]['recovery_scan'] -
                by_key[(scenario, 'P0', seed)]['recovery_scan'])
    cpu_median = axis_a_ratios['cpu_median']
    latency_median = axis_a_ratios['latency_p95_median']
    individual = axis_a_ratios['individual_max']
    passed = (p2_recovers and false_count == 0 and all(non_worse) and
              sum(value > 0 for value in improvements) >= 3 and
              statistics.median(improvements) >= 1 and all(correct_offset) and
              statistics.median(regressions) <= 1 and
              cpu_median <= 1.05 and latency_median <= 1.05 and
              individual <= 1.10)
    return {'status': 'EVALUATED', 'promote_p2': passed,
            'kidnapped_recovered_count': sum(
                right['recovered'] for _, right in kidnapped),
            'kidnapped_false_convergence_count': false_count,
            'kidnapped_non_worse_count': sum(non_worse),
            'kidnapped_strict_improvement_count': sum(
                value > 0 for value in improvements),
            'kidnapped_paired_median_improvement_scans':
                statistics.median(improvements),
            'correct_offset_recovered_count': sum(correct_offset),
            'correct_offset_median_regression_scans':
                statistics.median(regressions),
            'axis_a_ratios': axis_a_ratios}


def validate_manifest(path, expected_mode: str) -> dict:
    """Validate smoke cardinality or the exact 45-run plan."""
    value = strict_json_load(path)
    if set(value) != {'schema_version', 'mode', 'claim_scope', 'full_plan',
                      'executed_plan', 'runs', 'promotion'}:
        raise ValueError('Axis B manifest schema drift')
    if value['schema_version'] != 1 or value['mode'] != expected_mode:
        raise ValueError('Axis B manifest scalar drift')
    expected_full = [{'scenario': scenario, 'profile': profile, 'seed': seed}
                     for scenario, profile, seed in FULL_PLAN]
    if value['full_plan'] != expected_full:
        raise ValueError('Axis B full plan drift')
    if expected_mode == 'smoke':
        expected = [{'scenario': 'correct_init', 'profile': 'P0', 'seed': 11}]
        if value['executed_plan'] != expected or len(value['runs']) != 1 or \
                value['promotion'] != {
                    'status': 'NOT_EVALUATED', 'promote_p2': False}:
            raise ValueError('Axis B smoke contract drift')
    elif value['executed_plan'] != expected_full or len(value['runs']) != 45:
        raise ValueError('Axis B full cardinality drift')
    return value
