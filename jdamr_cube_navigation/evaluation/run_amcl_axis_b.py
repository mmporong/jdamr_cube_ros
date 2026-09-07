#!/usr/bin/env python3
"""Run smoke or the exact 45-run G002 Axis B matrix."""

from __future__ import annotations

import argparse
import bisect
import math
import os
from pathlib import Path
import shutil
import tempfile

from amcl_fault_contract import canonical_json_bytes, STORAGE_LIMITS
from amcl_fault_contract import strict_json_load
from evaluate_amcl_axis_a import profile_overrides
from evaluate_amcl_axis_b import (
    _expected_cloud_count,
    _t0_scan_index,
    _tree_digest,
    _tree_records,
    authority_limited_promotion,
    axis_a_ratios,
    CLAIM_FULL,
    CLAIM_SMOKE,
    FULL_PLAN,
    FULL_PREFIX_S,
    kidnapped_metrics,
    PLAYBACK_RATE,
    promotion_decision,
    recovery_metrics,
    SCHEMA_VERSION,
    SMOKE_PREFIX_S,
    validate_manifest,
)
from generate_amcl_axis_b_input import SCENARIOS, validate_input
from geometry_msgs.msg import PoseStamped, Twist
from prepare_amcl_fault_benchmark import current_free_bytes
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import run_amcl_determinism_preflight as preflight
from sensor_msgs.msg import LaserScan


GT_PAIR_MAX_DELTA_NS = 100_000_000


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _input_truth(root: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topic_types = {item.name: get_message(item.type)
                   for item in reader.get_all_topics_and_types()}
    scans = []
    poses = []
    contact_count = 0
    final_twist = None
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic == '/scan':
            scans.append(_stamp_ns(
                deserialize_message(serialized, LaserScan).header.stamp))
        elif topic == '/ground_truth_pose':
            message = deserialize_message(serialized, PoseStamped)
            pose = message.pose
            poses.append({'stamp_ns': _stamp_ns(message.header.stamp),
                          'frame_id': message.header.frame_id,
                          'pose': [
                              pose.position.x, pose.position.y, pose.position.z,
                              pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w]})
        elif topic == '/contact':
            message = deserialize_message(serialized, topic_types[topic])
            contact_count += len(message.contacts)
        elif topic == '/cmd_vel':
            message = deserialize_message(serialized, Twist)
            final_twist = [message.linear.x, message.linear.y,
                           message.angular.z]
    reader.close()
    if not scans or not poses or final_twist is None:
        raise ValueError('Axis B input truth topics are incomplete')
    if (any(not item['frame_id'] for item in poses) or
            len({item['frame_id'] for item in poses}) != 1 or
            any(not math.isfinite(float(value))
                for item in poses for value in item['pose']) or
            any(left['stamp_ns'] >= right['stamp_ns']
                for left, right in zip(poses, poses[1:]))):
        raise ValueError('Axis B GT is non-finite')
    return {'scan_stamps_ns': scans, 'gt_poses': poses,
            'contact_count': contact_count, 'final_twist': final_twist}


def _pair_gt(clouds: list[dict], truth: dict) -> list[dict]:
    result = []
    scans = truth['scan_stamps_ns']
    scan_indices = {stamp: index for index, stamp in enumerate(scans)}
    if len(scan_indices) != len(scans):
        raise ValueError('Axis B input scan stamps are not unique')
    gt_stamps = [item['stamp_ns'] for item in truth['gt_poses']]
    for cloud in clouds:
        stamp_ns = cloud['fifo_associated_pose_scan_header_stamp_ns']
        if stamp_ns not in scan_indices:
            raise ValueError('AMCL FIFO association is absent from input scans')
        upper_index = bisect.bisect_left(gt_stamps, stamp_ns)
        if upper_index == len(gt_stamps):
            raise ValueError('AMCL scan is newer than bracketed GT')
        if gt_stamps[upper_index] == stamp_ns:
            lower_index = upper_index
        elif upper_index == 0:
            raise ValueError('AMCL scan is older than bracketed GT')
        else:
            lower_index = upper_index - 1
        lower = truth['gt_poses'][lower_index]
        upper = truth['gt_poses'][upper_index]
        before_delta = stamp_ns - lower['stamp_ns']
        after_delta = upper['stamp_ns'] - stamp_ns
        if (before_delta < 0 or after_delta < 0 or
                before_delta > GT_PAIR_MAX_DELTA_NS or
                after_delta > GT_PAIR_MAX_DELTA_NS):
            raise ValueError('AMCL scan has stale GT bracket')
        span = upper['stamp_ns'] - lower['stamp_ns']
        fraction = 0.0 if span == 0 else before_delta / span
        lower_pose = lower['pose']
        upper_pose = upper['pose']
        lower_yaw = 2.0 * math.atan2(lower_pose[5], lower_pose[6])
        upper_yaw = 2.0 * math.atan2(upper_pose[5], upper_pose[6])
        yaw = lower_yaw + fraction * math.remainder(
            upper_yaw - lower_yaw, 2.0 * math.pi)
        pose = [lower_pose[index] + fraction *
                (upper_pose[index] - lower_pose[index]) for index in range(3)]
        pose.extend([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])
        result.append({'scan_index': scan_indices[stamp_ns],
                       'scan_header_stamp_ns': stamp_ns,
                       'gt_before_stamp_ns': lower['stamp_ns'],
                       'gt_after_stamp_ns': upper['stamp_ns'],
                       'gt_max_delta_ns': max(before_delta, after_delta),
                       'gt_interpolation_fraction': fraction,
                       'gt_frame_id': lower['frame_id'],
                       'gt_pose': pose})
    return result


def _axis_b_bootstrap(root: Path) -> dict:
    """Choose an offset after the matching TF and before the next scan."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    from tf2_msgs.msg import TFMessage
    scans = []
    transforms = []
    source_start_ns = None
    while reader.has_next():
        topic, serialized, storage_ns = reader.read_next()
        if source_start_ns is None:
            source_start_ns = storage_ns
        if topic == '/scan':
            scans.append((storage_ns, _stamp_ns(
                deserialize_message(serialized, LaserScan).header.stamp)))
        elif topic == '/tf':
            message = deserialize_message(serialized, TFMessage)
            transforms.extend(
                (storage_ns, _stamp_ns(value.header.stamp))
                for value in message.transforms
                if value.header.frame_id == 'odom' and
                value.child_frame_id == 'base_footprint')
    reader.close()
    for index, (scan_storage_ns, scan_header_ns) in enumerate(scans[1:], 1):
        before = [item for item in transforms if item[1] <= scan_header_ns]
        after = [item for item in transforms if item[1] >= scan_header_ns]
        if not before or not after:
            continue
        lower = max(before, key=lambda item: item[1])
        upper = min(after, key=lambda item: item[1])
        if upper[0] >= scan_storage_ns:
            continue
        start_storage_ns = (upper[0] + scan_storage_ns) // 2
        return {
            'source_start_storage_ns': source_start_ns,
            'previous_scan_storage_ns': scans[index - 1][0],
            'first_main_scan_storage_ns': scan_storage_ns,
            'first_main_scan_header_ns': scan_header_ns,
            'lower_tf_storage_ns': lower[0], 'lower_tf_header_ns': lower[1],
            'upper_tf_storage_ns': upper[0], 'upper_tf_header_ns': upper[1],
            'start_offset_ns': start_storage_ns - source_start_ns,
            'start_offset_s': (start_storage_ns - source_start_ns) / 1e9,
        }
    raise ValueError('Axis B input has no scan with prior TF bracket')


def initial_estimate(scenario: str) -> list[float]:
    """Bind AMCL initialization to true pose plus scenario offset."""
    contract = SCENARIOS[scenario]
    return [contract['true_start_pose'][index] +
            contract['initial_estimate_offset'][index] for index in range(3)]


def parse_input_roots(values: list[str]) -> dict[str, Path]:
    """Parse scenario=/absolute/root bindings without implicit fallback."""
    result = {}
    for value in values:
        if '=' not in value:
            raise ValueError('input binding must be scenario=/absolute/root')
        scenario, raw_path = value.split('=', 1)
        path = Path(raw_path)
        if scenario in result or scenario not in SCENARIOS or not path.is_absolute():
            raise ValueError('Axis B input binding drift')
        result[scenario] = path
    return result


def readiness(mode: str, input_roots: dict[str, Path],
              axis_a_root: Path | None) -> dict:
    """Return a deterministic PENDING/PASS plan without executing ROS."""
    required = {'correct_init'} if mode == 'smoke' else set(SCENARIOS)
    blockers = []
    records = []
    for scenario in SCENARIOS:
        if scenario not in required:
            continue
        root = input_roots.get(scenario)
        if root is None:
            blockers.append(f'missing_canonical_input:{scenario}')
            continue
        try:
            manifest = validate_input(root)
            if manifest['scenario'] != scenario:
                raise ValueError('scenario mismatch')
            records.append({'scenario': scenario, 'root': str(root),
                            'manifest': preflight._identity(
                                root / 'axis_b_input_manifest.json')})
        except Exception as exc:
            blockers.append(f'invalid_canonical_input:{scenario}:{exc}')
    if mode == 'full':
        if axis_a_root is None:
            blockers.append('missing_axis_a_full_artifact')
        else:
            try:
                axis_a_ratios(axis_a_root)
            except Exception as exc:
                blockers.append(f'invalid_axis_a_full_artifact:{exc}')
    plan = [('correct_init', 'P0', 11)] if mode == 'smoke' else list(FULL_PLAN)
    return {
        'schema_version': 1, 'mode': mode,
        'status': 'PENDING' if blockers else 'PASS',
        'blockers': blockers,
        'plan': [{'scenario': scenario, 'profile': profile, 'seed': seed}
                 for scenario, profile, seed in plan],
        'canonical_inputs': records,
        'synthetic_inputs_canonical_eligible': False,
        'executes_ros': False,
    }


def _canonical_output(path: Path) -> Path:
    path = path.expanduser()
    if (not path.is_absolute() or path.exists() or path.is_symlink() or
            path.parent.resolve() != path.parent):
        raise ValueError('output root must be absent and canonical')
    return path


def run(args) -> dict:
    """Execute the selected exact matrix and atomically publish evidence."""
    output_root = _canonical_output(args.output_root)
    input_roots = parse_input_roots(args.input)
    state = readiness(args.mode, input_roots, args.axis_a_root)
    if state['status'] != 'PASS':
        raise RuntimeError('PENDING: ' + '; '.join(state['blockers']))
    if args.domain_base != 180:
        raise ValueError('Axis B canonical domain base is 180')
    if current_free_bytes(output_root.parent) < \
            STORAGE_LIMITS['minimum_start_free_bytes']:
        raise RuntimeError('less than 6 GiB free before Axis B evaluation')
    plan = [('correct_init', 'P0', 11)] if args.mode == 'smoke' else list(FULL_PLAN)
    contract = strict_json_load(args.prepared_root / 'contract.json')
    preflight._validate_source_records(contract['production_inputs'])
    attestation = strict_json_load(
        args.attestation_root / 'build_attestation.json')
    if (attestation['loaded_runtime']['amcl_executable'] !=
            preflight._identity(args.amcl_executable) or
            attestation['loaded_runtime']['rmw_library'] !=
            preflight._identity(args.rmw_library)):
        raise ValueError('Axis B runtime attestation drift')
    prefix_s = SMOKE_PREFIX_S if args.mode == 'smoke' else FULL_PREFIX_S
    with tempfile.TemporaryDirectory(
            prefix='.g002-axis-b-', dir=output_root.parent) as temp_name:
        stage = Path(temp_name) / 'artifact'
        stage.mkdir()
        shutil.copyfile(args.prepared_root / 'contract.json',
                        stage / 'contract_snapshot.json')
        shutil.copyfile(args.attestation_root / 'build_attestation.json',
                        stage / 'runtime_attestation_snapshot.json')
        runs = []
        promotion_rows = []
        truths = {scenario: _input_truth(root)
                  for scenario, root in input_roots.items()}
        bootstraps = {scenario: _axis_b_bootstrap(root)
                      for scenario, root in input_roots.items()}
        cloud_counts = {
            scenario: _expected_cloud_count(scenario, root, args.mode)
            for scenario, root in input_roots.items()}
        for index, (scenario, profile, seed) in enumerate(plan):
            domain_id = args.domain_base + index
            if not 0 <= domain_id <= 232:
                raise ValueError('Axis B domain plan exceeds ROS range')
            run_id = f'axis_b__{scenario}__{profile}__seed_{seed}'
            run_dir = stage / f'run_{index + 1}'
            estimate = initial_estimate(scenario)
            args.profile = profile
            args.run_id = run_id
            args.attempt_index = index + 1
            args.amcl_overrides = profile_overrides(profile)
            args.initial_x_m, args.initial_y_m, args.initial_yaw_rad = estimate
            args.sanitized_root = input_roots[scenario]
            args.max_clouds = cloud_counts[scenario]
            args.prefix_s = prefix_s
            args.playback_rate = PLAYBACK_RATE
            base = preflight._run_one(
                run_dir, seed, domain_id, args, dict(os.environ),
                bootstraps[scenario])
            if base['status'] != 'PASS':
                raise RuntimeError(f'Axis B run failed: {base["failure"]}')
            base_path = run_dir / 'evidence.json'
            current = preflight._load_observer_state(
                base['observer_state'], run_dir)
            pairs = _pair_gt(current['clouds'], truths[scenario])
            t0 = _t0_scan_index(
                scenario, truths[scenario], input_roots[scenario])
            if scenario == 'kidnapped':
                input_manifest = strict_json_load(
                    input_roots[scenario] / 'axis_b_input_manifest.json')
                trace = strict_json_load(Path(
                    input_manifest['trace_evidence']['path']))
                metrics = kidnapped_metrics(current['clouds'], pairs, t0, trace)
            else:
                metrics = recovery_metrics(current['clouds'], pairs, t0)
            gates = {
                'input_parity': True, 'profile_parity': True,
                'map_odom_authority': {
                    'input_transform_count': 0,
                    'isolated_expected_runtime_sources': ['amcl'],
                    'proof_status':
                        'ISOLATED_EXPECTED_RUNTIME_SOURCE_NO_GID_PROOF',
                    'observed_transform_count':
                        current['readiness']['map_odom_tf_count']},
                'lifecycle_active': all(
                    operation['returncode'] == 0
                    for operation in base['operations']),
                'finite_metrics': True,
                'stationary_window': ('PASS' if scenario == 'kidnapped'
                                      else 'NOT_APPLICABLE'),
                'kidnapped_pre_t0_initial_convergence': (
                    metrics['pre_t0_initial_converged']
                    if scenario == 'kidnapped' else 'NOT_APPLICABLE'),
                'kidnapped_post_zero_observation': (
                    metrics['post_zero_hold_observed']
                    if scenario == 'kidnapped' else 'NOT_APPLICABLE'),
                'contact_zero': truths[scenario]['contact_count'] == 0,
                'final_zero': all(abs(value) <= 1e-9
                                  for value in truths[scenario]['final_twist']),
                'survivor_zero': base['survivor_count'] == 0,
                'production_hashes_unchanged': True,
            }
            scalar_gates = [value for value in gates.values()
                            if type(value) is not dict]
            if (gates['map_odom_authority']['observed_transform_count'] <= 0 or
                    not all(value is True or
                            value in ('PASS', 'NOT_APPLICABLE')
                            for value in scalar_gates)):
                raise RuntimeError(f'Axis B run gate failed: {gates}')
            evidence = {
                'schema_version': SCHEMA_VERSION, 'run_id': run_id,
                'scenario': scenario, 'profile': profile, 'seed': seed,
                'domain_id': domain_id, 'status': 'PASS', 'failure': None,
                'base_evidence': preflight._relative_identity(base_path, stage),
                'input_manifest': preflight._identity(
                    input_roots[scenario] / 'axis_b_input_manifest.json'),
                'profile_parameters': dict(profile_overrides(profile)),
                't0_scan_index': t0, 'metrics': metrics, 'gates': gates,
            }
            path = run_dir / 'axis_b_evidence.json'
            path.write_bytes(canonical_json_bytes(evidence))
            runs.append(preflight._relative_identity(path, stage))
            promotion_rows.append({
                'scenario': scenario, 'profile': profile, 'seed': seed,
                'recovered': metrics['recovered'],
                'false_convergence': metrics['false_convergence'],
                'recovery_scan': (metrics['first_recovery']['scans_after_t0']
                                  if metrics['first_recovery'] else None),
            })
        input_records = list(state['canonical_inputs'])
        axis_a = (None if args.mode == 'smoke' else {
            'root': str(args.axis_a_root),
            'manifest': preflight._identity(
                args.axis_a_root / 'axis_a_manifest.json')})
        tree_records = _tree_records(stage)
        manifest = {
            'schema_version': SCHEMA_VERSION, 'mode': args.mode,
            'claim_scope': CLAIM_SMOKE if args.mode == 'smoke' else CLAIM_FULL,
            'full_plan': [{'scenario': scenario, 'profile': profile,
                           'seed': seed} for scenario, profile, seed in FULL_PLAN],
            'executed_plan': [{'scenario': scenario, 'profile': profile,
                               'seed': seed} for scenario, profile, seed in plan],
            'input_roots': input_records, 'axis_a_artifact': axis_a,
            'prepared_contract': preflight._relative_identity(
                stage / 'contract_snapshot.json', stage),
            'runtime_attestation': preflight._relative_identity(
                stage / 'runtime_attestation_snapshot.json', stage),
            'harness_sources': {source.name: preflight._identity(source)
                                for source in (
                Path(__file__).resolve(),
                Path(__file__).with_name('evaluate_amcl_axis_b.py').resolve(),
                Path(__file__).with_name(
                    'generate_amcl_axis_b_input.py').resolve(),
                Path(__file__).with_name('axis_b_kidnapped_driver.py').resolve(),
                Path(__file__).with_name('amcl_particle_observer.py').resolve(),
                Path(__file__).with_name(
                    'run_amcl_determinism_preflight.py').resolve())},
            'run_contract': {'cloud_counts': cloud_counts, 'prefix_s': prefix_s,
                             'playback_rate': PLAYBACK_RATE,
                             'output_mcap_count': 0},
            'runs': runs,
            'promotion': authority_limited_promotion(promotion_decision(
                promotion_rows if args.mode == 'full' else [],
                axis_a_ratios(args.axis_a_root)
                if args.mode == 'full' else None)),
            'tree_records': tree_records,
            'tree_sha256': _tree_digest(tree_records),
            'tree_bytes': sum(row['size_bytes'] for row in tree_records),
            'production_unchanged': True,
        }
        (stage / 'axis_b_manifest.json').write_bytes(
            canonical_json_bytes(manifest))
        validate_manifest(stage / 'axis_b_manifest.json', args.mode)
        preflight._validate_source_records(contract['production_inputs'])
        stage.rename(output_root)
    try:
        return validate_manifest(
            output_root / 'axis_b_manifest.json', args.mode)
    except Exception:
        preflight._safe_remove_created_root(output_root)
        raise


def main() -> int:
    """Parse dry-run, smoke, or full Axis B execution arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('smoke', 'full'), required=True)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--output-root', type=Path)
    parser.add_argument('--input', action='append', default=[],
                        metavar='SCENARIO=/ABSOLUTE/ROOT')
    parser.add_argument('--axis-a-root', type=Path)
    parser.add_argument('--prepared-root', type=Path)
    parser.add_argument('--attestation-root', type=Path)
    parser.add_argument('--amcl-executable', type=Path)
    parser.add_argument('--params-file', type=Path)
    parser.add_argument('--map-yaml', type=Path)
    parser.add_argument('--rmw-library', type=Path)
    parser.add_argument('--domain-base', type=int, default=180)
    args = parser.parse_args()
    inputs = parse_input_roots(args.input)
    if args.dry_run:
        print(canonical_json_bytes(
            readiness(args.mode, inputs, args.axis_a_root)).decode(), end='')
        return 0
    required = ('output_root', 'prepared_root', 'attestation_root',
                'amcl_executable', 'params_file', 'map_yaml', 'rmw_library')
    missing = [name for name in required if getattr(args, name) is None]
    if missing:
        parser.error('execution requires: ' + ', '.join(missing))
    run(args)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
