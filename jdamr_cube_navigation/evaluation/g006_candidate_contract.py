#!/usr/bin/env python3
"""Strict, evaluation-only contract for the G006 candidate bundle."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path


SCHEMA_VERSION = 1
DEV_SEEDS = frozenset({11, 23, 42, 67, 89})
SCENARIOS = (
    'mapping_nominal_closed_loop',
    'mapping_bounded_frontier',
    'saved_fixed_obstacle_detour',
    'saved_sudden_obstacle_stop_resume',
    'saved_scan_timeout_stop_resume',
    'saved_amcl_offset_recovery_then_nav',
)
GATE_FAMILIES = (
    'common', 'motion', 'mapping', 'saved_nav', 'real_bag', 'no_motion')
GATE_VALUES = ('PASS', 'FAIL', 'INVALID', 'NOT_EVALUATED', 'NOT_APPLICABLE')
FINAL_READY = 'READY_FOR_SUPERVISED_PHYSICAL_VALIDATION'
FINAL_NOT_READY = 'NOT_READY_RETAIN_PRODUCTION_BASELINE'
SEED_PENDING = 'NOT_GENERATED_DEPENDENCIES_PENDING'
SEED_LOCKED = 'GENERATED_DEPENDENCIES_LOCKED'
READY_PATH_ENABLED = False
PREDECESSOR_KEYS = ('G002', 'G003', 'G004', 'G005', 'G008')
CLAIMS_LOCK = {
    'real_motion_authorized': False,
    'safety_certified': False,
    'production_qualified': False,
    'map_provenance': 'INCOMPLETE',
    'lidar_extrinsic': 'UNVERIFIED',
}
RUNTIME_ENTITY_FIELDS = (
    'profile', 'package', 'executable_or_plugin', 'node_name',
    'process_class', 'authority_role')
RUNTIME_ENTITIES = (
    ('A', 'robot_state_publisher', 'robot_state_publisher',
     'robot_state_publisher', 'bringup_source', 'tf_static'),
    ('A', 'joint_state_publisher', 'joint_state_publisher',
     'joint_state_publisher', 'bringup_source', 'none'),
    ('A', 'jdamr_base_driver', 'base_driver_node', 'jdamr_base_driver',
     'bringup_source', 'odom_to_base_tf'),
    ('A', 'ydlidar_g4_ros2', 'ydlidar_g4_node', 'ydlidar_g4_node',
     'bringup_source', 'scan'),
    ('A', 'cartographer_ros', 'cartographer_node', 'cartographer_node',
     'mapping', 'map_to_odom'),
    ('A', 'cartographer_ros', 'cartographer_occupancy_grid_node',
     'occupancy_grid_node', 'mapping', 'map'),
    ('A', 'nav2_controller', 'controller_server', 'controller_server',
     'nav2_noncomposed', 'cmd_vel_nav'),
    ('A', 'nav2_planner', 'planner_server', 'planner_server',
     'nav2_noncomposed', 'none'),
    ('A', 'nav2_behaviors', 'behavior_server', 'behavior_server',
     'nav2_noncomposed', 'cmd_vel_nav'),
    ('A', 'nav2_bt_navigator', 'bt_navigator', 'bt_navigator',
     'nav2_noncomposed', 'navigate_to_pose'),
    ('A', 'nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower',
     'nav2_noncomposed', 'none'),
    ('A', 'nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
     'nav2_noncomposed', 'cmd_vel_nav'),
    ('A', 'nav2_collision_monitor', 'collision_monitor', 'collision_monitor',
     'nav2_noncomposed', 'final_cmd_vel'),
    ('A', 'opennav_docking', 'docking_server', 'docking_server',
     'nav2_noncomposed', 'cmd_vel_nav'),
    ('A', 'nav2_map_server', 'map_saver_server', 'map_saver',
     'mapping_support', 'map_save'),
    ('A', 'nav2_lifecycle_manager', 'lifecycle_manager',
     'lifecycle_manager_map_saver', 'lifecycle', 'none'),
    ('A', 'jdamr_cube_navigation', 'frontier_explorer', 'frontier_explorer',
     'frontier', 'navigate_to_pose_client'),
    ('B', 'rclcpp_components', 'component_container_isolated',
     'nav2_container', 'component_container', 'none'),
    ('B', 'nav2_map_server', 'nav2_map_server::MapServer', 'map_server',
     'composed_component', 'map'),
    ('B', 'nav2_amcl', 'nav2_amcl::AmclNode', 'amcl',
     'composed_component', 'map_to_odom'),
    ('B', 'nav2_controller', 'nav2_controller::ControllerServer',
     'controller_server', 'composed_component', 'cmd_vel_nav'),
    ('B', 'nav2_planner', 'nav2_planner::PlannerServer', 'planner_server',
     'composed_component', 'none'),
    ('B', 'nav2_velocity_smoother',
     'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother',
     'composed_component', 'cmd_vel_nav'),
    ('B', 'nav2_collision_monitor',
     'nav2_collision_monitor::CollisionMonitor', 'collision_monitor',
     'composed_component', 'final_cmd_vel'),
    ('B', 'nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator',
     'bt_navigator', 'composed_component', 'navigate_to_pose'),
    ('B', 'nav2_map_server', 'nav2_map_server::MapServer',
     'keepout_filter_mask_server', 'composed_component', 'keepout_mask'),
    ('B', 'nav2_map_server', 'nav2_map_server::CostmapFilterInfoServer',
     'keepout_costmap_filter_info_server', 'composed_component',
     'keepout_filter_info'),
    ('B', 'nav2_lifecycle_manager', 'lifecycle_manager',
     'lifecycle_manager_keepout', 'lifecycle', 'none'),
    ('B', 'nav2_lifecycle_manager', 'lifecycle_manager',
     'lifecycle_manager_localization', 'lifecycle', 'none'),
    ('B', 'nav2_lifecycle_manager', 'lifecycle_manager',
     'lifecycle_manager_navigation', 'lifecycle', 'none'),
    ('B', 'jdamr_cube_navigation', 'nav2_liveness_guard',
     'nav2_liveness_guard', 'liveness_guard', 'graph_liveness'),
    ('B', 'rosbag2_transport', 'ros2 bag record', 'onboard_recorder',
     'low_priority_recorder', 'evidence_recording'),
)
# The copied launch/config/map payload is below 1 MiB on this host.  A 64 MiB
# ceiling leaves over 60 MiB for exact manifests while preventing bag/source
# archives from entering this metrics-and-configuration-only bundle.
BUNDLE_LIMIT_BYTES = 64 * 1024 * 1024

APPLICABILITY = {
    'mapping_nominal_closed_loop': {
        'common', 'motion', 'mapping'},
    'mapping_bounded_frontier': {
        'common', 'motion', 'mapping'},
    'saved_fixed_obstacle_detour': {
        'common', 'motion', 'saved_nav'},
    'saved_sudden_obstacle_stop_resume': {
        'common', 'motion', 'saved_nav'},
    'saved_scan_timeout_stop_resume': {
        'common', 'motion', 'saved_nav'},
    'saved_amcl_offset_recovery_then_nav': {
        'common', 'motion', 'saved_nav'},
}

CARTOGRAPHER_METRICS = {
    'local_trajectory_latency_s': {'unit': 's', 'type': 'finite_float',
                                   'minimum': 0.0},
    'local_trajectory_realtime_ratio': {
        'unit': 'ratio', 'type': 'finite_float', 'minimum': 0.0},
    'scan_matcher_score': {'unit': 'ratio', 'type': 'finite_float',
                           'minimum': 0.0, 'maximum': 1.0},
    'scan_matcher_cost': {'unit': 'cost', 'type': 'finite_float',
                          'minimum': 0.0},
    'scan_matcher_residual': {'unit': 'residual', 'type': 'finite_float',
                              'minimum': 0.0},
    'pose_graph_work_queue_size': {
        'unit': 'count', 'type': 'nonnegative_int'},
    'constraints_attempted': {'unit': 'count', 'type': 'nonnegative_int'},
    'constraints_matched': {'unit': 'count', 'type': 'nonnegative_int'},
    'constraint_match_score': {'unit': 'ratio', 'type': 'finite_float',
                               'minimum': 0.0, 'maximum': 1.0},
    'trajectory_start_pose_m_rad': {
        'unit': '[m,m,rad]', 'type': 'finite_vector3'},
    'trajectory_end_pose_m_rad': {
        'unit': '[m,m,rad]', 'type': 'finite_vector3'},
    'trajectory_constraint_count': {
        'unit': 'count', 'type': 'nonnegative_int'},
    'cpu_seconds_per_1000_scans': {
        'unit': 'cpu_s/1000_scan', 'type': 'finite_float',
        'minimum': 0.0},
    'max_rss_bytes': {'unit': 'byte', 'type': 'nonnegative_int'},
    'callback_gap_p95_s': {'unit': 's', 'type': 'finite_float',
                           'minimum': 0.0},
    'tf_gap_p95_s': {'unit': 's', 'type': 'finite_float', 'minimum': 0.0},
}


def canonical_json_bytes(value) -> bytes:
    """Serialize JSON deterministically while rejecting non-finite numbers."""
    return (json.dumps(value, ensure_ascii=False, sort_keys=True,
                       separators=(',', ':'), allow_nan=False) + '\n').encode()


def strict_json_load(path: Path):
    """Load strict JSON with duplicate-key and non-finite rejection."""
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f'duplicate JSON key: {key}')
            result[key] = value
        return result

    def reject(value):
        raise ValueError(f'non-finite JSON number: {value}')

    return json.loads(path.read_text(encoding='utf-8'),
                      object_pairs_hook=pairs, parse_constant=reject)


def sha256_file(path: Path) -> str:
    """Hash one regular file."""
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'not a regular non-symlink file: {path}')
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def file_identity(path: Path, relative_path: str) -> dict:
    """Return the exact identity of one source or copied file."""
    if not relative_path or Path(relative_path).is_absolute() or '..' in Path(
            relative_path).parts:
        raise ValueError('identity relative path must be canonical')
    mode = path.stat().st_mode & 0o777
    return {'relative_path': relative_path, 'mode_octal': f'{mode:04o}',
            'size_bytes': path.stat().st_size, 'sha256': sha256_file(path)}


def tree_inventory(root: Path) -> dict:
    """Enumerate a canonical regular-file-only tree."""
    if root.is_symlink() or not root.is_dir():
        raise ValueError('tree root must be a non-symlink directory')
    entries = []
    for path in sorted(root.rglob('*'), key=lambda item: item.as_posix()):
        if path.is_symlink() or not (path.is_file() or path.is_dir()):
            raise ValueError(f'unsupported tree entry: {path}')
        if path.is_file():
            relative = path.relative_to(root).as_posix()
            entries.append(file_identity(path, relative))
    payload = b''.join(
        (f"{item['sha256']} {item['size_bytes']} "
         f"{item['mode_octal']} {item['relative_path']}\n").encode()
        for item in entries)
    return {'algorithm': 'sha256_path_size_mode_file_sha256_v1',
            'file_count': len(entries),
            'total_bytes': sum(item['size_bytes'] for item in entries),
            'tree_sha256': hashlib.sha256(payload).hexdigest(),
            'entries': entries}


def held_out_seed_lock(protocol_salt: str, predecessor_hashes: dict) -> dict:
    """Generate six unique held-out seeds per scenario only when fixed."""
    valid_hashes = (
        isinstance(predecessor_hashes, dict) and
        set(predecessor_hashes) == set(PREDECESSOR_KEYS) and
        all(isinstance(value, str) and len(value) == 64 and
            all(character in '0123456789abcdef' for character in value)
            for value in predecessor_hashes.values()))
    if not protocol_salt or not valid_hashes:
        return {'status': SEED_PENDING, 'protocol_salt': protocol_salt,
                'scenario_seeds': {}}
    used = set(DEV_SEEDS)
    scenario_seeds = {}
    for scenario in SCENARIOS:
        seeds = []
        counter = 0
        while len(seeds) < 6:
            digest = hashlib.sha256(
                f'{protocol_salt}\0{scenario}\0{counter}'.encode()).digest()
            candidate = int.from_bytes(digest[:4], 'little') & 0x7fffffff
            counter += 1
            if candidate in used:
                continue
            used.add(candidate)
            seeds.append(candidate)
        scenario_seeds[scenario] = seeds
    return {'status': SEED_LOCKED, 'protocol_salt': protocol_salt,
            'scenario_seeds': scenario_seeds}


def gate_matrix(default: str = 'NOT_EVALUATED') -> dict:
    """Build the exact scenario gate applicability matrix."""
    if default not in GATE_VALUES or default == 'NOT_APPLICABLE':
        raise ValueError('invalid applicable-gate default')
    result = {}
    for scenario in SCENARIOS:
        result[scenario] = {
            family: (default if family in APPLICABILITY[scenario]
                     else 'NOT_APPLICABLE')
            for family in GATE_FAMILIES}
    return result


def validate_gate_matrix(matrix: dict) -> bool:
    """Validate exact keys and distinguish N/A from pass/fail states."""
    if not isinstance(matrix, dict) or set(matrix) != set(SCENARIOS):
        return False
    for scenario, values in matrix.items():
        if not isinstance(values, dict) or set(values) != set(GATE_FAMILIES):
            return False
        for family, value in values.items():
            if value not in GATE_VALUES:
                return False
            applicable = family in APPLICABILITY[scenario]
            if applicable == (value == 'NOT_APPLICABLE'):
                return False
    return True


def _complete_readiness_valid(readiness: dict) -> bool:
    """Validate the full evidence surface required for a READY decision."""
    keys = {
        'status', 'predecessors', 'seed_lock', 'scenario_attempts',
        'runtime_equivalence', 'golden_vectors', 'runtime_binary_identity',
        'claims'}
    if not isinstance(readiness, dict) or set(readiness) != keys:
        return False
    if readiness['status'] != 'COMPLETE' or readiness['claims'] != CLAIMS_LOCK:
        return False
    predecessors = readiness['predecessors']
    if (not isinstance(predecessors, dict) or
            set(predecessors) != set(PREDECESSOR_KEYS)):
        return False
    for record in predecessors.values():
        if (record.keys() != {'status', 'verified',
                              'canonical_tree_sha256'} or
                record['status'] != 'COMPLETE' or
                record['verified'] is not True):
            return False
        digest = record['canonical_tree_sha256']
        if (not isinstance(digest, str) or len(digest) != 64 or
                any(character not in '0123456789abcdef' for character in digest)):
            return False
    seed_lock = readiness['seed_lock']
    if (not isinstance(seed_lock, dict) or set(seed_lock) != {
            'status', 'protocol_salt', 'scenario_seeds'} or
            seed_lock['status'] != SEED_LOCKED or
            not isinstance(seed_lock['protocol_salt'], str) or
            not seed_lock['protocol_salt']):
        return False
    scenario_seeds = seed_lock['scenario_seeds']
    if (not isinstance(scenario_seeds, dict) or
            set(scenario_seeds) != set(SCENARIOS)):
        return False
    flat_seeds = []
    for seeds in scenario_seeds.values():
        if (not isinstance(seeds, list) or len(seeds) != 6 or
                any(isinstance(seed, bool) or not isinstance(seed, int) or seed < 0
                    for seed in seeds)):
            return False
        flat_seeds.extend(seeds)
    if len(set(flat_seeds)) != 36 or set(flat_seeds) & DEV_SEEDS:
        return False
    attempts = readiness['scenario_attempts']
    if not isinstance(attempts, dict) or set(attempts) != set(SCENARIOS):
        return False
    for scenario, records in attempts.items():
        if not isinstance(records, list) or not records:
            return False
        seed_index = 0
        for index, record in enumerate(records, 1):
            if (not isinstance(record, dict) or set(record) != {
                    'attempt', 'seed', 'outcome', 'semantic_final',
                    'evidence'} or
                    isinstance(record['attempt'], bool) or
                    record['attempt'] != index or
                    isinstance(record['seed'], bool) or
                    seed_index >= 6 or
                    record['seed'] != scenario_seeds[scenario][seed_index] or
                    record['outcome'] not in ('PASS', 'INFRA_RETRY') or
                    not isinstance(record['semantic_final'], bool) or
                    not evidence_identity_shape_valid(record['evidence'])):
                return False
            if record['outcome'] == 'INFRA_RETRY':
                if record['semantic_final'] is not False:
                    return False
            elif record['semantic_final'] is True:
                seed_index += 1
            else:
                return False
        if seed_index != 6:
            return False
    runtime_identity = readiness['runtime_binary_identity']
    if (not isinstance(runtime_identity, dict) or set(runtime_identity) != {
            'status', 'evidence'} or runtime_identity['status'] != 'PASS' or
            not evidence_identity_shape_valid(runtime_identity['evidence'])):
        return False
    runtime_equivalence = readiness['runtime_equivalence']
    if (not isinstance(runtime_equivalence, dict) or
            set(runtime_equivalence) != {'status', 'evidence'} or
            runtime_equivalence['status'] != 'PASS' or
            runtime_equivalence['evidence'] != runtime_identity['evidence']):
        return False
    return (
            readiness['golden_vectors'] == 'PASS')


def evidence_identity_shape_valid(identity: dict) -> bool:
    """Validate a bundle-relative sealed JSON evidence identity."""
    if not isinstance(identity, dict) or set(identity) != {
            'bundle_relative_path', 'size_bytes', 'sha256'}:
        return False
    relative = identity['bundle_relative_path']
    digest = identity['sha256']
    return (isinstance(relative, str) and not Path(relative).is_absolute() and
            '..' not in Path(relative).parts and
            isinstance(identity['size_bytes'], int) and
            not isinstance(identity['size_bytes'], bool) and
            identity['size_bytes'] > 0 and isinstance(digest, str) and
            len(digest) == 64 and all(
                character in '0123456789abcdef' for character in digest))


def final_decision(matrix: dict, external_gates: dict,
                   readiness: dict | None = None) -> str:
    """Allow readiness only when all runtime and predecessor evidence passes."""
    # READY remains disabled until the portable MCAP/runtime-graph validator
    # can validate real G002/G005 full artifacts. Synthetic summaries cannot
    # cross this boundary.
    if not READY_PATH_ENABLED:
        return FINAL_NOT_READY
    try:
        matrix_valid = validate_gate_matrix(matrix)
        readiness_valid = _complete_readiness_valid(readiness)
    except (AttributeError, KeyError, TypeError, ValueError):
        return FINAL_NOT_READY
    if not matrix_valid:
        return FINAL_NOT_READY
    applicable = [value for row in matrix.values() for value in row.values()
                  if value != 'NOT_APPLICABLE']
    external_ready = (
        isinstance(external_gates, dict) and set(external_gates) == {
            'no_motion_exact_launch', 'real_bag_isolated_replay'} and
        all(evidence_identity_shape_valid(value)
            for value in external_gates.values()))
    if (applicable and all(value == 'PASS' for value in applicable) and
            external_ready and
            readiness_valid):
        return FINAL_READY
    return FINAL_NOT_READY


def validate_cartographer_metrics(metrics: dict) -> str:
    """Return PASS or INVALID for the exact metric family contract."""
    if not isinstance(metrics, dict) or set(metrics) != set(
            CARTOGRAPHER_METRICS):
        return 'INVALID'
    for name, contract in CARTOGRAPHER_METRICS.items():
        value = metrics[name]
        if contract['type'] == 'nonnegative_int':
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                return 'INVALID'
        elif contract['type'] == 'finite_vector3':
            if (not isinstance(value, list) or len(value) != 3 or
                    any(isinstance(item, bool) or
                        not isinstance(item, (int, float)) or
                        not math.isfinite(item) for item in value)):
                return 'INVALID'
        else:
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value)):
                return 'INVALID'
            if value < contract.get('minimum', -math.inf):
                return 'INVALID'
            if value > contract.get('maximum', math.inf):
                return 'INVALID'
    if metrics['constraints_matched'] > metrics['constraints_attempted']:
        return 'INVALID'
    return 'PASS'
