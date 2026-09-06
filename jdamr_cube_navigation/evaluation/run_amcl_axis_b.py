#!/usr/bin/env python3
"""Run the G002 Axis B AMCL relocalization scenarios."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import tempfile

from amcl_fault_contract import canonical_json_bytes
from evaluate_amcl_axis_a import profile_overrides
from evaluate_amcl_axis_b import FULL_PLAN, promotion_decision, recovery_metrics
from generate_amcl_axis_b_input import SCENARIOS
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.serialization import deserialize_message
import rosbag2_py
import run_amcl_determinism_preflight as preflight
from sensor_msgs.msg import LaserScan


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _input_truth(root: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
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
            poses.append((_stamp_ns(message.header.stamp), [
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w]))
        elif topic == '/contact':
            contact_count += 1
        elif topic == '/cmd_vel':
            message = deserialize_message(serialized, Twist)
            final_twist = [message.linear.x, message.linear.y,
                           message.angular.z]
    if not scans or not poses or final_twist is None:
        raise ValueError('Axis B input truth topics are incomplete')
    if any(not math.isfinite(float(value)) for _, pose in poses for value in pose):
        raise ValueError('Axis B GT is non-finite')
    return {'scan_stamps_ns': scans, 'gt_poses': poses,
            'contact_count': contact_count, 'final_twist': final_twist}


def _pair_gt(clouds: list[dict], truth: dict) -> list[dict]:
    result = []
    scans = truth['scan_stamps_ns']
    for cloud in clouds:
        stamp_ns = cloud['triggering_scan_header_stamp_ns']
        if stamp_ns not in scans:
            raise ValueError('AMCL update is not tied to an input scan')
        gt_stamp_ns, pose = min(
            truth['gt_poses'], key=lambda item: abs(item[0] - stamp_ns))
        result.append({'scan_index': scans.index(stamp_ns),
                       'scan_header_stamp_ns': stamp_ns,
                       'gt_header_stamp_ns': gt_stamp_ns,
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
            'lower_tf_storage_ns': lower[0],
            'lower_tf_header_ns': lower[1],
            'upper_tf_storage_ns': upper[0],
            'upper_tf_header_ns': upper[1],
            'start_offset_ns': start_storage_ns - source_start_ns,
            'start_offset_s': (start_storage_ns - source_start_ns) / 1e9,
        }
    raise ValueError('Axis B input has no scan with prior TF bracket')


def run_smoke(args) -> dict:
    """Execute only the preregistered correct-init/P0/seed-11 smoke."""
    if args.output_root.exists() or not args.output_root.is_absolute():
        raise ValueError('output root must be absent and absolute')
    input_manifest = preflight.strict_json_load(
        args.input_root / 'axis_b_input_manifest.json')
    if input_manifest['scenario'] != 'correct_init':
        raise ValueError('smoke input must be correct_init')
    truth = _input_truth(args.input_root)
    expected_start = SCENARIOS['correct_init']['true_start_pose']
    if math.dist(truth['gt_poses'][0][1][:2], expected_start[:2]) > 0.02:
        raise ValueError('correct_init true start pose drift')
    with tempfile.TemporaryDirectory(
            prefix='.g002-axis-b-smoke-', dir=args.output_root.parent) as name:
        stage = Path(name) / 'artifact'
        stage.mkdir()
        args.profile = 'P0'
        args.run_id = 'axis_b__correct_init__P0__seed_11'
        args.attempt_index = 1
        args.amcl_overrides = profile_overrides('P0')
        args.initial_x_m = -8.0
        args.initial_y_m = 0.0
        args.initial_yaw_rad = 0.0
        args.sanitized_root = args.input_root
        args.max_clouds = 3
        args.prefix_s = 30.0
        base = preflight._run_one(
            stage / 'run_1', 11, args.domain_id, args, dict(os.environ),
            _axis_b_bootstrap(args.input_root))
        if base['status'] != 'PASS':
            raise RuntimeError(base['failure'])
        pairs = _pair_gt(base['observer']['clouds'], truth)
        metrics = recovery_metrics(base['observer']['clouds'], pairs, 0)
        if not metrics['recovered']:
            raise RuntimeError('correct_init did not meet three-pose recovery gate')
        if truth['contact_count'] != 0 or any(
                abs(value) > 1e-9 for value in truth['final_twist']):
            raise RuntimeError('contact or final zero input gate failed')
        run_record = {
            'run_id': args.run_id, 'scenario': 'correct_init', 'profile': 'P0',
            'seed': 11, 'domain_id': args.domain_id,
            'status': 'PASS', 'metrics': metrics, 'gt_pairs': pairs,
            'contact_count': 0, 'final_zero': True,
            'stationary_window': 'NOT_APPLICABLE',
            'map_odom_sole_authority':
                base['observer']['readiness']['map_odom_tf_count'] > 0,
            'survivor_count': base['survivor_count'],
            'base_evidence': preflight._relative_identity(
                stage / 'run_1/evidence.json', stage),
        }
        run_path = stage / 'run_1/axis_b_evidence.json'
        run_path.write_bytes(canonical_json_bytes(run_record))
        full_plan = [{'scenario': scenario, 'profile': profile, 'seed': seed}
                     for scenario, profile, seed in FULL_PLAN]
        manifest = {
            'schema_version': 1, 'mode': 'smoke',
            'claim_scope': 'AXIS_B_RELOCALIZATION_SMOKE_SIMULATION_ONLY',
            'full_plan': full_plan,
            'executed_plan': [
                {'scenario': 'correct_init', 'profile': 'P0', 'seed': 11}],
            'runs': [preflight._relative_identity(run_path, stage)],
            'promotion': promotion_decision([], None),
        }
        (stage / 'axis_b_manifest.json').write_bytes(canonical_json_bytes(manifest))
        stage.rename(args.output_root)
    return manifest


def initial_estimate(scenario: str) -> list[float]:
    """Bind AMCL initialization to true pose plus scenario offset."""
    contract = SCENARIOS[scenario]
    true_pose = contract['true_start_pose']
    offset = contract['initial_estimate_offset']
    return [true_pose[index] + offset[index] for index in range(3)]


def main() -> int:
    """Parse the correct-init smoke inputs; full execution stays preregistered."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--smoke', action='store_true', required=True)
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--input-root', required=True, type=Path)
    parser.add_argument('--amcl-executable', required=True, type=Path)
    parser.add_argument('--params-file', required=True, type=Path)
    parser.add_argument('--map-yaml', required=True, type=Path)
    parser.add_argument('--rmw-library', required=True, type=Path)
    parser.add_argument('--domain-id', type=int, default=220)
    parser.add_argument('--playback-rate', type=float, default=2.0)
    args = parser.parse_args()
    run_smoke(args)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
