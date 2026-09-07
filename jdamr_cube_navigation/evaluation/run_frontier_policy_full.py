#!/usr/bin/env python3
"""Run and seal the G005 15-case frontier-policy runtime matrix."""

from __future__ import annotations

import argparse
import hashlib
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from evaluate_frontier_policy import (
    coverage_metrics,
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
    RUN_OUTPUT_LIMIT_BYTES,
    strict_json_load,
    unreachable_outcome as classify_unreachable_outcome,
    validate_planner_batch,
)

from generate_frontier_policy_assets import validate_assets
from sample_process_group_resources import process_group_totals


CLAIM_SCOPE = 'ACTUAL_GAZEBO_NAV2_FROZEN_PLANNER_FRONTIER_EVALUATION'
FULL_RUNTIME_PATH_ENABLED = True
FULL_RUNTIME_BLOCKER = None
SIMULATION_HORIZON_S = 900.0
FINAL_ZERO_HOLD_S = 1.0
RUNTIME_LOG_TAIL_BYTES = 128 * 1024
LAUNCH_ALIVE_PROBE_DELAY_S = 1.0
PROCESS_GROUP_PROBE_DELAY_S = 0.1
RUNTIME_PROOF_KEYS = {
    'schema_version', 'claim_scope', 'run_id', 'policy', 'layout_seed',
    'validity', 'outcome', 'invalid_reasons', 'request_sha256',
    'asset_identity',
    'asset_manifest_identity', 'production_inputs_sha256',
    'shared_initial_sha256',
    'shared_reveal_sha256', 'shared_runtime_sha256', 'simulation_horizon_s',
    'runtime_identity', 'decisions', 'coverage_samples', 'gt_pose_samples',
    'planner_batch_timeline',
    'observer_health',
    'contact_count', 'command_authority', 'tf_authority', 'lifecycle',
    'navigation_outcomes', 'resources', 'runner_cancellation_count',
    'production_inputs_before', 'production_inputs_after',
    'observer_measurement', 'teardown',
}

RUNTIME_COMPONENT_KEYS = {
    'behavior_tree', 'bridge_config', 'evaluation_params',
    'evaluation_robot', 'ground_truth_localization', 'nav2_launch',
    'observer', 'coordinator', 'resource_sampler', 'runtime_launch',
}
NAV2_LIFECYCLE_NODES = {
    'behavior_server', 'bt_navigator', 'collision_monitor',
    'controller_server', 'planner_server', 'velocity_smoother',
}
MEASUREMENT_FIELDS = (
    'decisions', 'coverage_samples', 'gt_pose_samples',
    'planner_batch_timeline', 'observer_health', 'contact_count',
    'command_authority',
    'tf_authority', 'lifecycle', 'navigation_outcomes', 'resources',
    'runner_cancellation_count', 'production_inputs_before',
    'production_inputs_after',
)
OBSERVER_HEALTH_KEYS = {
    'accepted_scan_count', 'actual_mean_scan_rate_hz',
    'allowed_jitter_ns', 'expected_scan_period_ns', 'health',
    'last_accepted_stamp_ns', 'map_payload_sha256', 'map_sequence',
    'observed_max_gap_ns',
    'observed_scan_count', 'rejected_scan_count', 'rejection_counts',
    'schema_version',
}


def _finite(value: object) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _lower_sha256(value: object) -> bool:
    return (type(value) is str and
            re.fullmatch(r'[0-9a-f]{64}', value) is not None)


def _sha256_json(value: object) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def _package_version(package_name: str) -> str:
    package_xml = Path(
        get_package_share_directory(package_name)) / 'package.xml'
    version = ET.parse(package_xml).getroot().findtext('version')
    if not version:
        raise ValueError(f'G005 {package_name} version is unavailable')
    return f'{package_name}={version}'


def _runtime_versions() -> dict:
    ros_distro = os.environ.get('ROS_DISTRO', '')
    if not ros_distro:
        raise ValueError('G005 ROS distribution identity is unavailable')
    return {
        'gazebo': _package_version('ros_gz_sim'),
        'nav2': _package_version('nav2_bringup'),
        'ros': ros_distro,
    }


def _regular_identity_valid(value: object) -> bool:
    return (type(value) is dict and set(value) == {
        'path', 'size_bytes', 'sha256'} and
        type(value['path']) is str and bool(value['path']) and
        type(value['size_bytes']) is int and value['size_bytes'] >= 0 and
        _lower_sha256(value['sha256']))


def _validate_decision(value: object, run_id: str, policy: str,
                       layout_seed: int) -> list[dict]:
    keys = {
        'planner_batch_attempt_index', 'map_sequence',
        'map_payload_sha256', 'start_record',
        'blacklist_ids', 'candidate_sampling', 'decision_token',
        'costmap_sha256_before', 'costmap_sha256_after', 'planner_action',
        'planner_results', 'eligible_candidates', 'selected_goal',
        'stale',
    }
    if (type(value) is not dict or set(value) != keys or
            type(value['planner_batch_attempt_index']) is not int or
            value['planner_batch_attempt_index'] <= 0 or
            type(value['map_sequence']) is not int or
            value['map_sequence'] <= 0 or value['stale'] is not False or
            value['planner_action'] != {
                'type': 'nav2_msgs/action/ComputePathToPose',
                'planner_id': 'GridBased', 'use_start': True,
                'timeout_s': 2.0, 'execution': 'ACTUAL_NAV2_ACTION'}):
        raise ValueError('G005 runtime decision schema drift')
    start = value['start_record']
    if (type(start) is not dict or set(start) != {
            'frame_id', 'stamp_ns', 'pose_xy_yaw'} or
            start['frame_id'] != 'map' or type(start['stamp_ns']) is not int or
            start['stamp_ns'] <= 0 or type(start['pose_xy_yaw']) is not list or
            len(start['pose_xy_yaw']) != 3 or
            any(not _finite(item) for item in start['pose_xy_yaw']) or
            not _lower_sha256(value['map_payload_sha256']) or
            not _lower_sha256(value['decision_token']) or
            not _lower_sha256(value['costmap_sha256_before']) or
            not _lower_sha256(value['costmap_sha256_after']) or
            value['costmap_sha256_before'] !=
            value['costmap_sha256_after']):
        raise ValueError('G005 runtime decision identity schema drift')
    sampling = value['candidate_sampling']
    if (type(sampling) is not dict or set(sampling) != {
            'raw_candidate_ids', 'excluded_candidate_ids',
            'selected_candidate_ids', 'omitted_candidate_ids'}):
        raise ValueError('G005 runtime candidate sampling drift')
    if any(type(items) is not list or any(
            type(item) is not int for item in items)
           for items in sampling.values()):
        raise ValueError('G005 runtime candidate identifiers drift')
    raw = sampling['raw_candidate_ids']
    excluded = sampling['excluded_candidate_ids']
    selected = sampling['selected_candidate_ids']
    if (raw != sorted(set(raw)) or excluded != sorted(set(excluded)) or
            not set(excluded).issubset(raw) or set(excluded) & set(selected)):
        raise ValueError('G005 runtime blacklist evidence drift')
    if value['blacklist_ids'] != excluded:
        raise ValueError('G005 runtime blacklist exclusion is not bound')
    expected_sample = neutral_sample(
        layout_seed, sorted(set(raw) - set(excluded)))
    if (selected != expected_sample['selected_candidate_ids'] or
            sampling['omitted_candidate_ids'] !=
            expected_sample['omitted_candidate_ids']):
        raise ValueError('G005 runtime neutral sample drift')
    token = decision_token(
        run_id, policy, value['map_sequence'], value['map_payload_sha256'],
        value['start_record'], excluded, selected)
    if token != value['decision_token']:
        raise ValueError('G005 runtime decision token drift')
    eligible = validate_planner_batch(
        value['planner_results'], token, value['costmap_sha256_before'],
        value['costmap_sha256_after'])
    if eligible != value['eligible_candidates'] or not eligible:
        raise ValueError('G005 runtime planner batch drift')
    start_xy = value['start_record'].get('pose_xy_yaw', [])[:2]
    if (len(start_xy) != 2 or
            [item['candidate']['cell_index']
             for item in value['planner_results']] != selected or
            any(item['poses'][0] != start_xy
                for item in value['planner_results'])):
        raise ValueError('G005 runtime frozen planner start drift')
    ranked = policy_rank(policy, [dict(item) for item in eligible])
    if value['selected_goal'] != ranked[0]:
        raise ValueError('G005 runtime selected frontier drift')
    return ranked


def _validate_coverage(value: object) -> dict:
    if type(value) is not dict or set(value) != {
            'reachable_denominator_cells', 'samples'}:
        raise ValueError('G005 runtime coverage schema drift')
    return coverage_metrics(
        value['samples'], value['reachable_denominator_cells'],
        SIMULATION_HORIZON_S)


def _validate_observer_health(value: object) -> None:
    if (type(value) is not dict or set(value) != OBSERVER_HEALTH_KEYS or
            value['schema_version'] != 1 or
            value['health'] not in ('VALID', 'INVALID') or
            type(value['accepted_scan_count']) is not int or
            value['accepted_scan_count'] < 1 or
            type(value['observed_scan_count']) is not int or
            value['observed_scan_count'] < value['accepted_scan_count'] or
            type(value['rejected_scan_count']) is not int or
            value['rejected_scan_count'] < 0 or
            type(value['rejection_counts']) is not dict or
            any(type(name) is not str or not name or
                type(count) is not int or count < 0
                for name, count in value['rejection_counts'].items()) or
            sum(value['rejection_counts'].values()) !=
            value['rejected_scan_count'] or
            (value['health'] == 'VALID') !=
            (value['rejected_scan_count'] == 0) or
            type(value['last_accepted_stamp_ns']) is not int or
            value['last_accepted_stamp_ns'] <= 0 or
            type(value['map_sequence']) is not int or
            value['map_sequence'] < 1 or
            not _lower_sha256(value['map_payload_sha256']) or
            type(value['expected_scan_period_ns']) is not int or
            value['expected_scan_period_ns'] != 100_000_000 or
            type(value['allowed_jitter_ns']) is not int or
            not 0 <= value['allowed_jitter_ns'] <
            value['expected_scan_period_ns'] or
            type(value['observed_max_gap_ns']) is not int or
            value['observed_max_gap_ns'] <= 0 or
            not _finite(value['actual_mean_scan_rate_hz']) or
            value['actual_mean_scan_rate_hz'] <= 0.0 or
            (value['health'] == 'VALID' and (
                value['observed_max_gap_ns'] >
                value['expected_scan_period_ns'] +
                value['allowed_jitter_ns'] or
                not (1_000_000_000 / (
                    value['expected_scan_period_ns'] +
                    value['allowed_jitter_ns']) <=
                    value['actual_mean_scan_rate_hz'] <=
                    1_000_000_000 / (
                        value['expected_scan_period_ns'] -
                        value['allowed_jitter_ns']))))):
        raise ValueError('G005 observer health evidence drift')


def _gt_metrics(value: object, coverage_t85_s: float | None) -> dict:
    keys = {'steady_elapsed_s', 'sim_elapsed_s', 'x_m', 'y_m',
            'clearance_m'}
    if (type(value) is not list or len(value) < 2 or
            any(type(item) is not dict or set(item) != keys or
                any(not _finite(item[key]) for key in keys) or
                item['steady_elapsed_s'] < 0.0 or
                not 0.0 <= item['sim_elapsed_s'] <= SIMULATION_HORIZON_S or
                item['clearance_m'] < 0.0
                for item in value)):
        raise ValueError('G005 runtime GT pose trace drift')
    if any(right['steady_elapsed_s'] <= left['steady_elapsed_s'] or
           right['sim_elapsed_s'] <= left['sim_elapsed_s']
           for left, right in zip(value, value[1:])):
        raise ValueError('G005 runtime GT pose time is not monotonic')
    if (value[0]['steady_elapsed_s'] != 0.0 or
            value[0]['sim_elapsed_s'] != 0.0 or
            value[-1]['sim_elapsed_s'] != SIMULATION_HORIZON_S):
        raise ValueError('G005 runtime GT trace does not span exact horizon')
    if coverage_t85_s is None:
        return {'gt_path_length_to_85_m': None,
                'minimum_clearance_m': min(
                    item['clearance_m'] for item in value)}
    if not value[0]['sim_elapsed_s'] <= coverage_t85_s <= value[-1][
            'sim_elapsed_s']:
        raise ValueError('G005 GT trace does not bracket 85 percent coverage')
    distance = 0.0
    if coverage_t85_s > value[0]['sim_elapsed_s']:
        for left, right in zip(value, value[1:]):
            if coverage_t85_s >= right['sim_elapsed_s']:
                distance += math.hypot(right['x_m'] - left['x_m'],
                                       right['y_m'] - left['y_m'])
                if coverage_t85_s == right['sim_elapsed_s']:
                    break
                continue
            ratio = ((coverage_t85_s - left['sim_elapsed_s']) /
                     (right['sim_elapsed_s'] - left['sim_elapsed_s']))
            interpolated_x = left['x_m'] + ratio * (
                right['x_m'] - left['x_m'])
            interpolated_y = left['y_m'] + ratio * (
                right['y_m'] - left['y_m'])
            distance += math.hypot(interpolated_x - left['x_m'],
                                   interpolated_y - left['y_m'])
            break
    return {'gt_path_length_to_85_m': distance,
            'minimum_clearance_m': min(item['clearance_m'] for item in value)}


def _validate_planner_batch_timeline(value: object,
                                     decisions: list[dict]) -> dict:
    keys = {'attempt_index', 'map_sequence', 'map_payload_sha256',
            'start_record_sha256', 'candidate_sha256', 'fresh',
            'reachable_count'}
    if (type(value) is not list or not value or
            any(type(item) is not dict or set(item) != keys
                for item in value)):
        raise ValueError('G005 planner batch timeline schema drift')
    if [item['attempt_index'] for item in value] != list(
            range(1, len(value) + 1)):
        raise ValueError('G005 planner batch timeline index drift')
    if any(type(item['map_sequence']) is not int or
           item['map_sequence'] <= 0 or item['fresh'] is not True or
           type(item['reachable_count']) is not int or
           item['reachable_count'] < 0 or
           any(not _lower_sha256(item[key]) for key in (
               'map_payload_sha256', 'start_record_sha256',
               'candidate_sha256')) for item in value):
        raise ValueError('G005 planner batch timeline identity drift')
    if any(right['map_sequence'] < left['map_sequence']
           for left, right in zip(value, value[1:])):
        raise ValueError('G005 planner batch map sequence regressed')
    if any((right['map_sequence'] == left['map_sequence']) !=
           (right['map_payload_sha256'] == left['map_payload_sha256'])
           for left, right in zip(value, value[1:])):
        raise ValueError('G005 planner batch map identity drift')
    decision_indices = [item['planner_batch_attempt_index']
                        for item in decisions]
    reachable_indices = [item['attempt_index'] for item in value
                         if item['reachable_count'] > 0]
    if decision_indices != reachable_indices:
        raise ValueError('G005 planner batch decision ordered binding drift')
    by_attempt = {item['planner_batch_attempt_index']: item
                  for item in decisions}
    for item in value:
        decision = by_attempt.get(item['attempt_index'])
        if item['reachable_count'] == 0:
            if decision is not None:
                raise ValueError(
                    'G005 unreachable planner batch selected a goal')
            continue
        if decision is None:
            raise ValueError('G005 reachable planner batch was omitted')
        sampling = decision['candidate_sampling']['selected_candidate_ids']
        if (item['reachable_count'] != len(decision['eligible_candidates']) or
                item['map_sequence'] != decision['map_sequence'] or
                item['map_payload_sha256'] !=
                decision['map_payload_sha256'] or
                item['start_record_sha256'] !=
                _sha256_json(decision['start_record']) or
                item['candidate_sha256'] != _sha256_json(sampling)):
            raise ValueError('G005 planner batch timeline binding drift')
    unresolved = False
    for offset in range(len(value) - 2):
        window = value[offset:offset + 3]
        if not all(item['reachable_count'] == 0 for item in window):
            continue
        converted = [{
            'map_sha256': item['map_payload_sha256'],
            'start_sha256': item['start_record_sha256'],
            'candidate_sha256': item['candidate_sha256'],
            'fresh': item['fresh'],
            'reachable_count': item['reachable_count']} for item in window]
        if classify_unreachable_outcome(converted)['status'] != (
                'UNRESOLVED_FRONTIERS'):
            continue
        if offset + 3 != len(value):
            raise ValueError('G005 planner timeline continued after terminal')
        unresolved = True
    trailing_unreachable = 0
    for item in reversed(value):
        if item['reachable_count'] != 0:
            break
        trailing_unreachable += 1
    status = ('UNRESOLVED_FRONTIERS' if unresolved else
              ('PENDING' if trailing_unreachable else 'NOT_OBSERVED'))
    return {'status': status,
            'result': ('FAIL' if unresolved else 'NOT_EVALUATED'),
            'exploration_complete': False, 'blacklist_exhausted': False}


def _validate_runtime_proof_shape(value: object, policy: str,
                                  layout_seed: int,
                                  asset_manifest: dict | None = None) -> dict:
    """Validate the dormant runtime proof schema without enabling it."""
    run_id = f'{policy}__seed_{layout_seed}'
    if (type(value) is not dict or set(value) != RUNTIME_PROOF_KEYS or
            value['schema_version'] != 1 or value['claim_scope'] !=
            CLAIM_SCOPE or value['run_id'] != run_id or
            value['policy'] != policy or value['layout_seed'] != layout_seed or
            value['validity'] not in ('VALID', 'INVALID') or
            value['outcome'] not in ('PASS', 'FAIL') or
            type(value['invalid_reasons']) is not list or
            any(type(item) is not str or not item
                for item in value['invalid_reasons']) or
            ((value['validity'] == 'VALID') !=
             (value['invalid_reasons'] == [])) or
            (value['validity'] == 'INVALID' and
             value['outcome'] != 'FAIL') or
            value['simulation_horizon_s'] != SIMULATION_HORIZON_S or
            not _regular_identity_valid(value['asset_identity']) or
            not _regular_identity_valid(value['asset_manifest_identity'])):
        raise ValueError('G005 runtime proof schema drift')
    for key in ('request_sha256', 'shared_initial_sha256',
                'shared_reveal_sha256', 'shared_runtime_sha256'):
        if not _lower_sha256(value[key]):
            raise ValueError('G005 runtime proof hash drift')
    runtime = value['runtime_identity']
    if (type(runtime) is not dict or set(runtime) != {
            'gazebo_actual', 'nav2_actual', 'compute_path_action_actual',
            'oracle_reveal_map', 'ground_truth_localization',
            'not_slam_evaluation', 'synthetic_fixture', 'components',
            'upstream_versions'} or
            {key: runtime[key] for key in (
                'gazebo_actual', 'nav2_actual',
                'compute_path_action_actual', 'oracle_reveal_map',
                'ground_truth_localization', 'not_slam_evaluation',
                'synthetic_fixture')} != {
                'gazebo_actual': True, 'nav2_actual': True,
                'compute_path_action_actual': True,
                'oracle_reveal_map': True,
                'ground_truth_localization': True,
                'not_slam_evaluation': True,
                'synthetic_fixture': False} or
            type(runtime['components']) is not dict or
            set(runtime['components']) != RUNTIME_COMPONENT_KEYS or
            any(not _regular_identity_valid(item)
                for item in runtime['components'].values()) or
            type(runtime['upstream_versions']) is not dict or
            set(runtime['upstream_versions']) != {'gazebo', 'nav2', 'ros'} or
            any(type(item) is not str or not item
                for item in runtime['upstream_versions'].values())):
        raise ValueError('G005 runtime identity is not actual Gazebo/Nav2')
    if type(value['decisions']) is not list:
        raise ValueError('G005 runtime decision list schema drift')
    ranked = [_validate_decision(item, run_id, policy, layout_seed)
              for item in value['decisions']]
    unresolved = _validate_planner_batch_timeline(
        value['planner_batch_timeline'], value['decisions'])
    if unresolved['status'] == 'UNRESOLVED_FRONTIERS' and (
            value['outcome'] != 'FAIL'):
        raise ValueError('G005 all-unreachable history was laundered')
    if value['outcome'] == 'PASS' and unresolved['status'] != 'NOT_OBSERVED':
        raise ValueError('G005 PASS contains unresolved frontiers')
    if value['outcome'] == 'PASS' and not ranked:
        raise ValueError('G005 PASS has no reachable planner decision')
    coverage = _validate_coverage(value['coverage_samples'])
    _validate_observer_health(value['observer_health'])
    gt = _gt_metrics(value['gt_pose_samples'], coverage['coverage_t85_s'])
    command = value['command_authority']
    if (type(command) is not dict or set(command) != {
            'topic', 'publisher_gids', 'publisher_nodes',
            'final_zero_hold_s', 'final_linear_x', 'final_angular_z'} or
            command['topic'] != '/cmd_vel' or
            len(command['publisher_gids']) != 1 or
            any(type(gid) is not str or not gid
                for gid in command['publisher_gids']) or
            command['publisher_nodes'] != ['/collision_monitor'] or
            not _finite(command['final_zero_hold_s']) or
            command['final_zero_hold_s'] < FINAL_ZERO_HOLD_S or
            command['final_linear_x'] != 0.0 or
            command['final_angular_z'] != 0.0):
        raise ValueError('G005 runtime final command authority drift')
    authority = value['tf_authority']
    if (type(authority) is not dict or set(authority) != {
            'map_to_odom_publisher_gids', 'map_to_odom_publisher_nodes'} or
            len(authority['map_to_odom_publisher_gids']) != 1 or
            any(type(gid) is not str or not gid
                for gid in authority['map_to_odom_publisher_gids']) or
            authority['map_to_odom_publisher_nodes'] !=
            ['/g005_frontier_coordinator']):
        raise ValueError('G005 runtime TF authority drift')
    lifecycle = value['lifecycle']
    if (type(lifecycle) is not dict or
            set(lifecycle) != NAV2_LIFECYCLE_NODES or
            any(type(name) is not str or state not in {
                'active', 'finalized', 'inactive', 'unconfigured', 'unknown'}
                for name, state in lifecycle.items())):
        raise ValueError('G005 runtime lifecycle drift')
    navigation = value['navigation_outcomes']
    if (type(navigation) is not dict or set(navigation) != {
            'recovery_count', 'failure_count'} or
            any(type(navigation[key]) is not int or navigation[key] < 0
                for key in navigation)):
        raise ValueError('G005 runtime navigation outcome drift')
    resources = value['resources']
    if (type(resources) is not dict or set(resources) != {
            'cpu_seconds', 'peak_rss_bytes', 'samples'} or
            not _finite(resources['cpu_seconds']) or
            resources['cpu_seconds'] <= 0.0 or
            type(resources['peak_rss_bytes']) is not int or
            resources['peak_rss_bytes'] <= 0 or
            type(resources['samples']) is not int or
            resources['samples'] < 2):
        raise ValueError('G005 runtime resource evidence drift')
    if (type(asset_manifest) is not dict or
            type(asset_manifest.get('production_inputs')) is not dict or
            not asset_manifest['production_inputs']):
        raise ValueError('G005 asset manifest production inputs missing')
    expected_production = asset_manifest['production_inputs']
    if value['production_inputs_sha256'] != _sha256_json(expected_production):
        raise ValueError('G005 production input manifest binding drift')
    before = value['production_inputs_before']
    after = value['production_inputs_after']
    if (type(before) is not dict or before != expected_production or
            before != after or
            any(not _regular_identity_valid(item)
                for item in before.values())):
        raise ValueError('G005 runtime production hash parity drift')
    measurement = value['observer_measurement']
    expected_payload = {field: _sha256_json(value[field])
                        for field in MEASUREMENT_FIELDS}
    if (type(measurement) is not dict or set(measurement) != {
            'finalized_before_teardown', 'payload', 'payload_sha256'} or
            measurement['finalized_before_teardown'] is not True or
            measurement['payload'] != expected_payload or
            measurement['payload_sha256'] != _sha256_json(expected_payload)):
        raise ValueError('G005 observer measurement seal drift')
    teardown = value['teardown']
    teardown_payload = {
        'survivor_count': 0, 'finalized_after_teardown': True,
        'observer_payload_sha256': measurement['payload_sha256']}
    if (type(teardown) is not dict or set(teardown) != {
            *teardown_payload, 'final_sha256'} or
            {key: teardown[key] for key in teardown_payload} !=
            teardown_payload or
            teardown['final_sha256'] != _sha256_json(teardown_payload) or
            type(value['contact_count']) is not int or
            value['contact_count'] < 0 or
            type(value['runner_cancellation_count']) is not int or
            value['runner_cancellation_count'] != 0):
        raise ValueError('G005 runtime safety or teardown gate failed')
    if (value['validity'] == 'VALID' and
            value['observer_health']['health'] != 'VALID'):
        raise ValueError('G005 observer INVALID was laundered')
    success = (value['validity'] == 'VALID' and
               value['observer_health']['health'] == 'VALID' and
               coverage['final_coverage_ratio'] >= 0.85 and
               coverage['coverage_t85_s'] is not None and
               unresolved['status'] == 'NOT_OBSERVED' and bool(ranked) and
               gt['minimum_clearance_m'] >= 0.05 and
               value['contact_count'] == 0 and
               all(state == 'active' for state in lifecycle.values()))
    if value['outcome'] != ('PASS' if success else 'FAIL'):
        raise ValueError('G005 runtime outcome does not match measurements')
    return {'ranked': ranked, 'coverage': coverage, 'gt': gt,
            'unreachable_outcome': unresolved}


def validate_runtime_proof(value: object, policy: str, layout_seed: int,
                           asset_manifest: dict | None = None) -> dict:
    """Validate one proof produced by the repository-owned ROS adapter."""
    return _validate_runtime_proof_shape(
        value, policy, layout_seed, asset_manifest)


def validate_paired_first_decisions(proofs: list[dict]) -> None:
    """Require identical first-decision inputs for three policies per seed."""
    if type(proofs) is not list or len(proofs) != 15:
        raise ValueError('G005 paired proof matrix is incomplete')
    for seed in sorted({seed for _, seed in FULL_PLAN}):
        if any(type(item) is not dict for item in proofs):
            raise ValueError('G005 paired proof schema drift')
        paired = [item for item in proofs if item.get('layout_seed') == seed]
        if {item.get('policy') for item in paired} != {
                'current', 'nearest', 'gain_nav'} or len(paired) != 3:
            raise ValueError('G005 paired policy set drift')
        if any(type(item.get('decisions')) is not list or
               not item['decisions'] for item in paired):
            raise ValueError('G005 paired first decision missing')
        decisions = [item['decisions'][0] for item in paired]
        fields = ('map_sequence', 'map_payload_sha256',
                  'blacklist_ids', 'candidate_sampling',
                  'costmap_sha256_before', 'costmap_sha256_after',
                  )
        if any(type(decision) is not dict or
               not set(fields).issubset(decision) for decision in decisions):
            raise ValueError('G005 paired first decision schema drift')
        if any(
                len({_sha256_json(decision[field]) for decision in decisions})
                != 1 for field in fields):
            raise ValueError('G005 paired first decision parity drift')
        poses = [{
            'frame_id': decision['start_record']['frame_id'],
            'pose_xy_yaw': decision['start_record']['pose_xy_yaw'],
        } for decision in decisions]
        if len({_sha256_json(item) for item in poses}) != 1:
            raise ValueError('G005 paired first decision parity drift')
    runtime_identities = [item.get('runtime_identity') for item in proofs]
    if (any(type(item) is not dict for item in runtime_identities) or
            len({_sha256_json(item) for item in runtime_identities}) != 1):
        raise ValueError('G005 shared runtime identity parity drift')


def compact_runtime_proof(value: dict, asset_manifest: dict) -> dict:
    """Derive the exact compact full-run record from sealed runtime proof."""
    derived = _validate_runtime_proof_shape(
        value, value['policy'], value['layout_seed'], asset_manifest)
    coverage = derived['coverage']
    gt = derived['gt']
    decisions = value['decisions']
    planning_rejects = sum(
        len(item['candidate_sampling']['selected_candidate_ids']) -
        len(item['eligible_candidates']) for item in decisions)
    score_batches = []
    for decision_index, (decision, ranked) in enumerate(zip(
            decisions, derived['ranked']), start=1):
        gains = [row['gain_cells'] for row in ranked]
        lengths = [row['nav_length_m'] for row in ranked]
        gain_span = max(gains) - min(gains)
        length_span = max(lengths) - min(lengths)
        records = [{
            **item,
            'gain_norm': ((item['gain_cells'] - min(gains)) / gain_span
                          if gain_span else 0.0),
            'length_norm': ((item['nav_length_m'] - min(lengths)) /
                            length_span if length_span else 0.0),
        } for item in ranked]
        score_batches.append({
            'decision_index': decision_index,
            'decision_token': decision['decision_token'],
            'records': records,
        })
    return {
        'schema_version': 1, 'run_id': value['run_id'], 'mode': 'full',
        'policy': value['policy'], 'layout_seed': value['layout_seed'],
        'validity': value['validity'], 'outcome': value['outcome'],
        'invalid_reasons': value['invalid_reasons'],
        'first_goal_cell_index': (decisions[0]['selected_goal']['cell_index']
                                  if decisions else -1),
        **coverage, **gt, 'planning_reject_count': planning_rejects,
        'recovery_count': value['navigation_outcomes']['recovery_count'],
        'failure_count': value['navigation_outcomes']['failure_count'],
        'score_decompositions': score_batches,
        'contact_count': value['contact_count'],
        'cpu_seconds': value['resources']['cpu_seconds'],
        'peak_rss_bytes': value['resources']['peak_rss_bytes'],
        'final_zero': True, 'map_to_odom_authority_count': 1,
        'cmd_vel_publisher_count': 1,
        'runner_cancellation_count': value['runner_cancellation_count'],
        'lifecycle_active': all(
            state == 'active' for state in value['lifecycle'].values()),
        'initial_parity': True,
        'input_parity': True, 'hash_parity': True,
        'production_hashes_unchanged': True,
        'first_decision_reachable_count':
        (len(decisions[0]['eligible_candidates']) if decisions else 0),
        'survivor_count': value['teardown']['survivor_count'],
    }


def build_execution_plan(asset_root: Path, base_domain_id: int) -> dict:
    """Create the immutable 15-run request plan without starting ROS."""
    assets = validate_assets(asset_root, 'full')
    if (type(base_domain_id) is not int or base_domain_id < 20 or
            base_domain_id + len(FULL_PLAN) - 1 > 232):
        raise ValueError('G005 domain block must fit 20..232')
    package_root = Path(__file__).resolve().parents[1]
    source_identities = {
        name: file_identity(Path(path).resolve()) for name, path in {
            'contract': __import__('frontier_policy_contract').__file__,
            'evaluator': __import__('evaluate_frontier_policy').__file__,
            'runner': __file__,
        }.items()}
    runtime_components = {
        name: file_identity(path) for name, path in {
            'runtime_launch': package_root / 'launch' /
            'g005_frontier_runtime.launch.py',
            'nav2_launch': package_root / 'launch' /
            'g005_frontier_eval.launch.py',
            'bridge_config': package_root.parent / 'jdamr_cube_gazebo' /
            'params' / 'bridge.yaml',
            'ground_truth_localization': package_root /
            'jdamr_cube_navigation' / 'g005_ground_truth_localization.py',
            'observer': package_root / 'jdamr_cube_navigation' /
            'g005_frontier_observer.py',
            'coordinator': package_root / 'jdamr_cube_navigation' /
            'g005_frontier_coordinator.py',
            'resource_sampler': package_root / 'evaluation' /
            'sample_process_group_resources.py',
            'evaluation_params': asset_root / 'g005_nav2_params.yaml',
            'behavior_tree': package_root / 'behavior_trees' /
            'navigate_to_pose_safe_mapping.xml',
            'evaluation_robot': asset_root / 'g005_robot.urdf',
        }.items()}
    upstream_versions = _runtime_versions()
    shared_runtime_sha = _sha256_json({
        'harness': source_identities, 'components': runtime_components,
        'upstream_versions': upstream_versions,
    })
    asset_manifest_identity = file_identity(
        asset_root / 'asset_manifest.json')
    production_inputs_sha = _sha256_json(assets['production_inputs'])
    requests = []
    for offset, (policy, seed) in enumerate(FULL_PLAN):
        layout_record = assets['layouts'][str(seed)]
        shared_initial = _sha256_json({
            'layout_seed': seed,
            'start_cell_index': layout_record['start_cell_index'],
            'gt_occupancy_sha256': layout_record['gt_occupancy_sha256'],
        })
        shared_reveal = _sha256_json({
            'layout_seed': seed,
            'reveal': assets['layout_contract']['reveal'],
        })
        request = {
            'schema_version': 1, 'run_id': f'{policy}__seed_{seed}',
            'policy': policy, 'layout_seed': seed,
            'ros_domain_id': base_domain_id + offset,
            'simulation_horizon_s': SIMULATION_HORIZON_S,
            'asset_root': str(asset_root.resolve()),
            'asset_identity': file_identity(
                asset_root / f'layout_{seed}_gt.json'),
            'asset_manifest_identity': asset_manifest_identity,
            'production_inputs_sha256': production_inputs_sha,
            'runtime_components': runtime_components,
            'upstream_versions_required': upstream_versions,
            'shared_initial_sha256': shared_initial,
            'shared_reveal_sha256': shared_reveal,
            'shared_runtime_sha256': shared_runtime_sha,
            'required_runtime_claim': CLAIM_SCOPE,
        }
        request['request_sha256'] = _sha256_json(request)
        requests.append(request)
    return {'schema_version': 1, 'mode': 'full',
            'claim_scope': 'G005_EXECUTION_PLAN_NO_RUNTIME_CLAIM',
            'runtime_path': {
                'enabled': FULL_RUNTIME_PATH_ENABLED,
                'status': 'READY', 'blocker': FULL_RUNTIME_BLOCKER,
                'external_adapter_results_accepted': False},
            'asset_manifest': asset_manifest_identity,
            'requests': requests}


def _write_atomic_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
            prefix=f'.{path.name}.', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(canonical_json_bytes(value))
        stream.flush()
        os.fsync(stream.fileno())
    try:
        temporary.replace(path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def _keep_log_tail(path: Path) -> None:
    if path.stat().st_size <= RUNTIME_LOG_TAIL_BYTES:
        return
    with path.open('rb') as stream:
        stream.seek(-RUNTIME_LOG_TAIL_BYTES, os.SEEK_END)
        tail = stream.read()
    marker = b'G005_RUNTIME_LOG_TRUNCATED_TO_TAIL\n'
    with tempfile.NamedTemporaryFile(
            prefix=f'.{path.name}.', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(marker)
        stream.write(tail)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def _rehash_production_inputs(records: dict) -> dict:
    if type(records) is not dict or not records:
        raise ValueError('G005 production input inventory is unavailable')
    result = {}
    for name, record in records.items():
        if type(record) is not dict or type(record.get('path')) is not str:
            raise ValueError('G005 production input identity drift')
        result[name] = file_identity(Path(record['path']))
    return result


def _runtime_commands(
        request: dict, measurement_path: Path) -> tuple[list, list]:
    ros2 = shutil.which('ros2')
    if ros2 is None:
        raise ValueError('G005 ros2 executable is unavailable')
    launch = [
        ros2, 'launch', 'jdamr_cube_navigation',
        'g005_frontier_runtime.launch.py',
        f"asset_root:={request['asset_root']}", 'asset_mode:=full',
        f"layout_seed:={request['layout_seed']}",
    ]
    coordinator = [
        ros2, 'run', 'jdamr_cube_navigation',
        'g005_frontier_coordinator', '--ros-args',
        '-p', 'use_sim_time:=true',
        '-p', f"request_path:={request['_request_path']}",
        '-p', f'measurement_path:={measurement_path}',
    ]
    return launch, coordinator


def _stop_process_group(process: subprocess.Popen,
                        grace_s: float = 10.0) -> int:
    process_group = process.pid
    for signal_number in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process_group, 0)
            os.killpg(process_group, signal_number)
        except ProcessLookupError:
            break
        if process.poll() is None:
            try:
                process.wait(timeout=grace_s)
            except subprocess.TimeoutExpired:
                continue
        time.sleep(PROCESS_GROUP_PROBE_DELAY_S)
    try:
        os.killpg(process_group, 0)
    except ProcessLookupError:
        return 0
    return 1


def _sample_process_groups(process_groups: tuple[int, ...], state: dict) -> dict:
    """Accumulate CPU deltas and peak RSS across owned runtime groups."""
    state['samples'] += 1
    rss_bytes = 0
    for process_group in process_groups:
        cpu_seconds, group_rss_bytes, _ = process_group_totals(process_group)
        previous = state['last_cpu_seconds'].get(process_group)
        state['cpu_seconds'] += (
            cpu_seconds if previous is None
            else max(0.0, cpu_seconds - previous))
        state['last_cpu_seconds'][process_group] = cpu_seconds
        rss_bytes += group_rss_bytes
    state['peak_rss_bytes'] = max(state['peak_rss_bytes'], rss_bytes)
    return {
        'cpu_seconds': state['cpu_seconds'],
        'peak_rss_bytes': state['peak_rss_bytes'],
        'samples': state['samples'],
    }


def _assemble_runtime_proof(request: dict, measurement: dict,
                            production_before: dict,
                            production_after: dict,
                            survivor_count: int) -> dict:
    expected_measurement = {
        'validity', 'outcome', 'invalid_reasons', *MEASUREMENT_FIELDS}
    if (type(measurement) is not dict or
            set(measurement) != expected_measurement or
            measurement['production_inputs_before'] != production_before or
            measurement['production_inputs_after'] != production_after):
        raise ValueError('G005 coordinator measurement schema drift')
    proof = {
        'schema_version': 1, 'claim_scope': CLAIM_SCOPE,
        'run_id': request['run_id'], 'policy': request['policy'],
        'layout_seed': request['layout_seed'],
        'validity': measurement['validity'],
        'outcome': measurement['outcome'],
        'invalid_reasons': measurement['invalid_reasons'],
        'request_sha256': request['request_sha256'],
        'asset_identity': request['asset_identity'],
        'asset_manifest_identity': request['asset_manifest_identity'],
        'production_inputs_sha256': request['production_inputs_sha256'],
        'shared_initial_sha256': request['shared_initial_sha256'],
        'shared_reveal_sha256': request['shared_reveal_sha256'],
        'shared_runtime_sha256': request['shared_runtime_sha256'],
        'simulation_horizon_s': request['simulation_horizon_s'],
        'runtime_identity': {
            'gazebo_actual': True, 'nav2_actual': True,
            'compute_path_action_actual': True,
            'oracle_reveal_map': True,
            'ground_truth_localization': True,
            'not_slam_evaluation': True, 'synthetic_fixture': False,
            'components': request['runtime_components'],
            'upstream_versions': request['upstream_versions_required'],
        },
        **{field: measurement[field] for field in MEASUREMENT_FIELDS
           if field not in {
               'production_inputs_before', 'production_inputs_after'}},
        'production_inputs_before': production_before,
        'production_inputs_after': production_after,
    }
    payload = {field: _sha256_json(proof[field])
               for field in MEASUREMENT_FIELDS}
    proof['observer_measurement'] = {
        'finalized_before_teardown': True,
        'payload': payload,
        'payload_sha256': _sha256_json(payload),
    }
    teardown_payload = {
        'survivor_count': survivor_count,
        'finalized_after_teardown': True,
        'observer_payload_sha256': proof[
            'observer_measurement']['payload_sha256'],
    }
    proof['teardown'] = {
        **teardown_payload, 'final_sha256': _sha256_json(teardown_payload)}
    return proof


def _execute_one_request(request: dict, stage: Path, timeout_s: float,
                         production_inputs: dict) -> dict:
    request_path = stage / f"{request['run_id']}.request.json"
    measurement_path = stage / f"{request['run_id']}.measurement.json"
    runtime_log_path = stage / f"{request['run_id']}.runtime.log"
    request_for_disk = dict(request)
    _write_atomic_json(request_path, request_for_disk)
    command_request = dict(request)
    command_request['_request_path'] = str(request_path)
    launch_command, coordinator_command = _runtime_commands(
        command_request, measurement_path)
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(request['ros_domain_id'])
    environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    production_before = _rehash_production_inputs(production_inputs)
    launch_process = None
    coordinator_process = None
    survivor_count = 0
    resources = {
        'cpu_seconds': 0.0, 'peak_rss_bytes': 0, 'samples': 0,
        'last_cpu_seconds': {},
    }
    started = time.monotonic()
    with runtime_log_path.open('wb') as runtime_log:
        try:
            launch_process = subprocess.Popen(
                launch_command, env=environment, stdout=runtime_log,
                stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(LAUNCH_ALIVE_PROBE_DELAY_S)
            if launch_process.poll() is not None:
                raise RuntimeError('G005 runtime launch exited before readiness')
            coordinator_process = subprocess.Popen(
                coordinator_command, env=environment, stdout=runtime_log,
                stderr=subprocess.STDOUT, start_new_session=True)
            process_groups = (launch_process.pid, coordinator_process.pid)
            while coordinator_process.poll() is None:
                if launch_process.poll() is not None:
                    raise RuntimeError(
                        'G005 runtime launch exited during evaluation')
                if time.monotonic() - started >= timeout_s:
                    raise subprocess.TimeoutExpired(
                        coordinator_command, timeout_s)
                _sample_process_groups(process_groups, resources)
                time.sleep(PROCESS_GROUP_PROBE_DELAY_S)
            _sample_process_groups(process_groups, resources)
            return_code = coordinator_process.returncode
            if return_code != 0:
                raise RuntimeError(
                    f'G005 coordinator exited with code {return_code}')
            if (not measurement_path.is_file() or
                    measurement_path.is_symlink() or
                    measurement_path.stat().st_size >
                    RUN_OUTPUT_LIMIT_BYTES):
                raise RuntimeError('G005 coordinator measurement is missing')
            measurement = strict_json_load(measurement_path)
            measurement['resources'] = {
                key: resources[key] for key in (
                    'cpu_seconds', 'peak_rss_bytes', 'samples')}
        except subprocess.TimeoutExpired as error:
            raise RuntimeError('G005 runtime wall timeout') from error
        finally:
            if coordinator_process is not None:
                survivor_count += _stop_process_group(coordinator_process)
            if launch_process is not None:
                survivor_count += _stop_process_group(launch_process)
    _keep_log_tail(runtime_log_path)
    production_after = _rehash_production_inputs(production_inputs)
    proof = _assemble_runtime_proof(
        request, measurement, production_before, production_after,
        survivor_count)
    validate_runtime_proof(
        proof, request['policy'], request['layout_seed'],
        {'production_inputs': production_inputs})
    return proof


def run_full(asset_root: Path, output_root: Path, command: list[str],
             base_domain_id: int, timeout_s: float) -> dict:
    """Run and atomically seal the repository-owned 15-case ROS matrix."""
    assets = validate_assets(asset_root, 'full')
    if command:
        raise ValueError('G005 external runtime adapters are not accepted')
    if (not output_root.is_absolute() or output_root.exists() or
            output_root.is_symlink()):
        raise ValueError('G005 full output must be absent and absolute')
    if not math.isfinite(timeout_s) or timeout_s <= SIMULATION_HORIZON_S:
        raise ValueError('G005 wall timeout must exceed the simulation horizon')
    plan = build_execution_plan(asset_root, base_domain_id)
    output_root.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
            prefix='.g005-frontier-full-', dir=output_root.parent) as temp:
        stage = Path(temp) / 'result'
        stage.mkdir()
        execution_plan_path = stage / 'execution_plan.json'
        _write_atomic_json(execution_plan_path, plan)
        proofs = []
        compact_runs = []
        for request in plan['requests']:
            proof = _execute_one_request(
                request, stage, timeout_s, assets['production_inputs'])
            proof_name = f"{request['run_id']}.runtime.json"
            _write_atomic_json(stage / proof_name, proof)
            compact = compact_runtime_proof(proof, assets)
            _write_atomic_json(stage / f"{request['run_id']}.json", compact)
            proofs.append(proof)
            compact_runs.append(compact)
        validate_paired_first_decisions(proofs)
        promotion = promotion_decision(compact_runs)
        handoff = {
            'decision': ('GAIN_NAV_EVAL_CANDIDATE'
                         if promotion['promote_gain_nav'] else
                         'RETAIN_CURRENT'),
            'selected_policy': ('gain_nav'
                                if promotion['promote_gain_nav'] else
                                'current'),
            'production_change_authorized': False,
        }
        names = sorted(path.name for path in stage.iterdir())
        records = [file_identity(stage / name, relative_to=stage)
                   for name in names]
        manifest = {
            'schema_version': 1, 'mode': 'full',
            'claim_scope':
            'SIMULATION_ONLY_FRONTIER_POLICY_PAIRED_EVALUATION',
            'asset_root': str(asset_root.resolve()),
            'asset_manifest': file_identity(
                asset_root / 'asset_manifest.json'),
            'full_plan': [{'policy': policy, 'layout_seed': seed}
                          for policy, seed in FULL_PLAN],
            'executed_plan': [{'policy': policy, 'layout_seed': seed}
                              for policy, seed in FULL_PLAN],
            'runs': [{'path': f'{policy}__seed_{seed}.json'}
                     for policy, seed in FULL_PLAN],
            'promotion': promotion,
            'evaluator_source': file_identity(Path(
                __import__('evaluate_frontier_policy').__file__).resolve()),
            'runner_source': file_identity(Path(__file__).resolve()),
            'runtime_executor': {
                'argv': ['builtin:g005_frontier_runtime.launch.py',
                         'builtin:g005_frontier_coordinator'],
                'executable': file_identity(Path(__file__).resolve()),
            },
            'runtime_proofs': [
                {'path': f'{policy}__seed_{seed}.runtime.json'}
                for policy, seed in FULL_PLAN],
            'contract_identity': file_identity(Path(
                __import__('frontier_policy_contract').__file__).resolve()),
            'execution_plan_sha256': file_identity(
                execution_plan_path)['sha256'],
            'frontier_policy_handoff': handoff,
            'tree_files': records,
            'tree_sha256': _sha256_json(records),
            'storage_limit_bytes': ARTIFACT_LIMIT_BYTES,
        }
        _write_atomic_json(stage / 'manifest.json', manifest)
        validate_artifact(stage, 'full')
        stage.rename(output_root)
    try:
        return validate_artifact(output_root, 'full')
    except Exception:
        if output_root.is_dir() and not output_root.is_symlink():
            shutil.rmtree(output_root)
        raise


def main() -> int:
    """Plan or execute the exact G005 full matrix."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--asset-root', required=True, type=Path)
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--base-domain-id', type=int, default=100)
    parser.add_argument('--timeout-s', type=float, default=1200.0)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('runtime_command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.timeout_s <= SIMULATION_HORIZON_S:
        raise ValueError(
            'G005 wall timeout must exceed the simulation horizon')
    command = args.runtime_command
    if command[:1] == ['--']:
        command = command[1:]
    if args.dry_run:
        if command:
            raise ValueError('G005 dry-run does not accept a runtime command')
        if args.output_root.exists() or args.output_root.is_symlink():
            raise ValueError('G005 dry-run output must be absent')
        _write_atomic_json(
            args.output_root,
            build_execution_plan(args.asset_root, args.base_domain_id))
        return 0
    if command:
        parser.error('G005 external runtime adapters are not accepted')
    run_full(args.asset_root, args.output_root, [], args.base_domain_id,
             args.timeout_s)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
