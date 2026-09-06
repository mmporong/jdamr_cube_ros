#!/usr/bin/env python3
"""Build an immutable, hardware-substituted G006 candidate bundle."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import platform
import shutil
import tempfile
import xml.etree.ElementTree as ET

from g006_candidate_contract import BUNDLE_LIMIT_BYTES
from g006_candidate_contract import canonical_json_bytes, CLAIMS_LOCK
from g006_candidate_contract import file_identity
from g006_candidate_contract import final_decision, gate_matrix
from g006_candidate_contract import held_out_seed_lock, sha256_file
from g006_candidate_contract import tree_inventory
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
HOME_MAPS = Path.home() / 'maps'
ARTIFACT_ROOT = Path.home() / 'jdamr_artifacts'

PROFILE_SOURCES = {
    'A': (
        ('repository', 'jdamr_cube_bringup/launch/real_bringup.launch.py'),
        ('repository', 'jdamr_cube_cartographer/launch/cartographer_real.launch.py'),
        ('repository', 'jdamr_cube_navigation/launch/autonomous_mapping.launch.py'),
        ('repository', 'jdamr_cube_navigation/behavior_trees/navigate_to_pose_safe_mapping.xml'),
        ('repository', 'jdamr_cube_navigation/config/nav2_params.yaml'),
        ('repository', 'jdamr_cube_cartographer/config/jdamr_cube_2d_real.lua'),
        ('repository', 'jdamr_cube_description/urdf/jdamr_cube.urdf'),
        ('repository', 'jdamr_cube_bringup/package.xml'),
        ('repository', 'jdamr_cube_cartographer/package.xml'),
        ('repository', 'jdamr_cube_navigation/package.xml'),
        ('repository', 'jdamr_cube_description/package.xml'),
        ('repository', 'jdamr_cube_navigation/setup.py'),
        ('repository',
         'jdamr_cube_navigation/jdamr_cube_navigation/frontier_explorer.py'),
        ('ros_share', 'nav2_bringup/launch/navigation_launch.py'),
        ('ros_share', 'nav2_bringup/package.xml'),
    ),
    'B': (
        ('repository', 'jdamr_cube_navigation/launch/onboard_keepout_navigation.launch.py'),
        ('repository', 'jdamr_cube_navigation/launch/onboard_nav2_core.launch.py'),
        ('repository',
         'jdamr_cube_navigation/behavior_trees/'
         'navigate_to_pose_corridor_fail_fast.xml'),
        ('repository', 'jdamr_cube_navigation/config/nav2_params.yaml'),
        ('repository', 'jdamr_cube_navigation/jdamr_cube_navigation/keepout_mask.py'),
        ('repository', 'jdamr_cube_navigation/jdamr_cube_navigation/onboard_recording.py'),
        ('repository', 'jdamr_cube_navigation/evaluation/mcap_writer_options.yaml'),
        ('repository', 'jdamr_cube_navigation/evaluation/qos_overrides.yaml'),
        ('repository', 'jdamr_cube_navigation/package.xml'),
        ('repository', 'jdamr_cube_navigation/setup.py'),
        ('repository',
         'jdamr_cube_navigation/jdamr_cube_navigation/nav2_liveness_guard.py'),
        ('home_maps', 'autonomous_20260826T161908.yaml'),
        ('home_maps', 'autonomous_20260826T161908.pgm'),
        ('home_maps', 'autonomous_20260826T161908_keepout_multi.yaml'),
        ('home_maps', 'autonomous_20260826T161908_keepout_multi.pgm'),
    ),
}

HARNESS_SOURCES = (
    'jdamr_cube_navigation/evaluation/g006_candidate_contract.py',
    'jdamr_cube_navigation/evaluation/build_g006_candidate_bundle.py',
    'jdamr_cube_navigation/evaluation/probe_g006_candidate_bundle.py',
    'jdamr_cube_navigation/evaluation/validate_g006_candidate_bundle.py',
)

PREDECESSORS = {
    'G003': (ARTIFACT_ROOT / 'sim_nav_obstacle_eval_20260906_v64_full25_cyclone',
             ('contract.json', 'aggregate.json'), 'PASS'),
    'G004': (ARTIFACT_ROOT / 'sim_collision_monitor_eval_20260906_v08_full15_cyclone',
             ('contract.json', 'aggregate.json', 'final_summary.json'), 'PASS'),
    'G008': (ARTIFACT_ROOT / 'cartographer_subdivision_ab_20260905',
             ('evidence_tree_manifest.json', 'storage_manifest.json'),
             'RETAIN_PRODUCTION_SUBDIVISION_1'),
}

HARDWARE_STUBS = (
    {'endpoint': '/scan', 'ros_type': 'sensor_msgs/msg/LaserScan',
     'direction': 'typed_stub_publisher', 'replaces': 'YDLIDAR_G4'},
    {'endpoint': '/odom', 'ros_type': 'nav_msgs/msg/Odometry',
     'direction': 'typed_stub_publisher', 'replaces': 'ESP32_BASE_DRIVER'},
    {'endpoint': '/tf', 'ros_type': 'tf2_msgs/msg/TFMessage',
     'direction': 'typed_stub_publisher', 'replaces': 'DYNAMIC_HARDWARE_TF'},
    {'endpoint': '/tf_static', 'ros_type': 'tf2_msgs/msg/TFMessage',
     'direction': 'typed_stub_publisher', 'replaces': 'ROBOT_STATE_PUBLISHER'},
    {'endpoint': '/cmd_vel', 'ros_type': 'geometry_msgs/msg/Twist',
     'direction': 'typed_stub_subscriber', 'replaces': 'MOTOR_COMMAND_SINK'},
)


def _source_path(root_name: str, relative: str) -> Path:
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    root = {'repository': REPO_ROOT, 'home_maps': HOME_MAPS,
            'ros_share': Path('/opt/ros') / ros_distro / 'share'}[root_name]
    path = root / relative
    resolved = path.resolve(strict=True)
    if path.is_symlink() or not resolved.is_file() or root.resolve() not in (
            resolved, *resolved.parents):
        raise ValueError(f'invalid production source: {path}')
    return resolved


def _package_versions() -> list[dict]:
    result = []
    for package in ('jdamr_cube_bringup', 'jdamr_cube_cartographer',
                    'jdamr_cube_navigation', 'jdamr_cube_description'):
        root = ET.parse(REPO_ROOT / package / 'package.xml').getroot()
        result.append({'name': root.findtext('name'),
                       'version': root.findtext('version')})
    root = ET.parse(_source_path('ros_share', 'nav2_bringup/package.xml')).getroot()
    result.append({'name': root.findtext('name'),
                   'version': root.findtext('version')})
    return result


def _predecessor_records(payload: Path) -> dict:
    records = {}
    for name, (root, files, conclusion) in PREDECESSORS.items():
        identities = []
        snapshots = []
        for relative in files:
            path = root / relative
            identities.append({'relative_path': relative,
                               'size_bytes': path.stat().st_size,
                               'sha256': sha256_file(path)})
            bundle_relative = f'predecessors/{name}/{relative}'
            destination = payload / bundle_relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, destination, follow_symlinks=False)
            snapshots.append({
                'source_relative_path': relative,
                'bundle_relative_path': bundle_relative,
                'size_bytes': destination.stat().st_size,
                'sha256': sha256_file(destination)})
        records[name] = {
            'canonical_root': str(root.resolve()), 'status': conclusion,
            'identities': identities, 'snapshots': snapshots}
        if name in ('G003', 'G004'):
            records[name]['artifact_tree'] = tree_inventory(root)
            aggregate = __import__('g006_candidate_contract').strict_json_load(
                root / 'aggregate.json')
            expected_count = 25 if name == 'G003' else 15
            if (aggregate.get('status') != 'PASS' or
                    aggregate.get('run_count') != expected_count):
                raise ValueError(f'{name} canonical result is not PASS')
            records[name]['verified_conclusion'] = {
                'status': aggregate['status'],
                'run_count': aggregate['run_count']}
        else:
            storage = __import__('g006_candidate_contract').strict_json_load(
                root / 'storage_manifest.json')
            tree = __import__('g006_candidate_contract').strict_json_load(
                root / 'evidence_tree_manifest.json')
            final = storage['final_summary']
            canonical = storage['canonical_tree']
            if (final['final_status'] != 'RETAIN_PRODUCTION' or
                    final['selected_subdivision'] != 1 or
                    canonical['manifest_sha256'] != identities[0]['sha256'] or
                    canonical['tree_sha256'] != tree['tree_sha256']):
                raise ValueError('G008 canonical conclusion drift')
            records[name]['artifact_tree'] = {
                'algorithm': tree['algorithm']['name'],
                'file_count': canonical['file_count'],
                'total_bytes': canonical['total_bytes'],
                'tree_sha256': canonical['tree_sha256'],
                'entries': [],
                'manifest_sha256': canonical['manifest_sha256'],
                'final_status': final['final_status'],
                'selected_subdivision': final['selected_subdivision'],
            }
            records[name]['verified_conclusion'] = {
                'final_status': final['final_status'],
                'selected_subdivision': final['selected_subdivision']}
    records['G002'] = {'status': 'NOT_EVALUATED', 'reason': 'FULL_RESULT_ABSENT'}
    records['G005'] = {'status': 'NOT_EVALUATED', 'reason': 'FULL_RESULT_ABSENT'}
    return records


def _topologies() -> dict:
    return {
        'A': {
            'mode': 'hardware_boundary_substituted_topology_proxy',
            'physical_launch_set': ['real_bringup.launch.py',
                                    'cartographer_real.launch.py',
                                    'autonomous_mapping.launch.py'],
            'expanded_nav2_launch': 'nav2_bringup/navigation_launch.py',
            'nav2_process_graph': 'use_composition=false',
            'behavior_tree': 'navigate_to_pose_safe_mapping.xml',
            'cartographer_subdivision': 1,
            'map_to_odom_sole_authority': 'cartographer_node',
            'frontier_policy': 'current',
            'process_classes': [
                'hardware_stub', 'robot_state', 'cartographer',
                'nav2_non_composed', 'frontier', 'resource_observer'],
        },
        'B': {
            'mode': 'hardware_boundary_substituted_topology_proxy',
            'launch_chain': ['onboard_keepout_navigation.launch.py(record_bag=true)',
                             'onboard_nav2_core.launch.py'],
            'nav2_process_graph': 'component_container_isolated_subset',
            'behavior_tree': 'navigate_to_pose_corridor_fail_fast.xml',
            'localization': ['map_server', 'amcl'],
            'map_to_odom_sole_authority': 'amcl',
            'final_cmd_vel_sole_authority': 'collision_monitor',
            'liveness_guard': 'nav2_liveness_guard',
            'recorder': 'ionice_best_effort_7_nice_10_shutdown_on_exit',
            'process_classes': [
                'hardware_stub', 'robot_state', 'nav2_composed_isolated',
                'lifecycle', 'liveness_guard', 'low_priority_recorder',
                'resource_observer'],
        },
    }


def _hardware_boundary() -> dict:
    return {
        'mode': 'hardware_boundary_substituted_topology_proxy',
        'substitute_endpoints': list(HARDWARE_STUBS),
        'physical_execution_claim': False,
        'pi_execution_claim': False,
    }


def _scenario_contract() -> dict:
    scenarios = __import__('g006_candidate_contract').SCENARIOS
    return {
        'scenarios': list(scenarios),
        'attempts': {scenario: [] for scenario in scenarios},
        'mapping_bounded_frontier_terminal': {
            'goal_count': 3, 'sim_time_limit_s': 900.0,
            'termination': 'FIRST_OF_GOAL_COUNT_OR_SIM_TIME'},
        'semantic_failure_resample': False,
        'infra_retry': 'SAME_SEED_WITH_ATTEMPT_HISTORY',
        'code_or_config_change': 'NEW_BUNDLE_AND_PROTOCOL_ID',
    }


def _measurement_contract() -> dict:
    return {
        'authority_identity': 'publisher_gid',
        'authority_expectations': {
            'A': {'map_to_odom': 'cartographer_node',
                  'final_cmd_vel': 'collision_monitor'},
            'B': {'map_to_odom': 'amcl',
                  'final_cmd_vel': 'collision_monitor'}},
        'required_resources': [
            'cpu_seconds_per_1000_scans', 'max_rss_bytes',
            'callback_gap_p95_s', 'tf_gap_p95_s'],
        'resource_provenance': {
            'process_identity': 'executable_path_size_sha256_pid_pgid',
            'sample_clock': 'host_monotonic',
            'cpu_unit': 'cpu_seconds', 'rss_unit': 'bytes'},
        'resource_applicability': {
            'no_motion_exact_launch': 'REQUIRED_WHEN_EXECUTED',
            'real_bag_isolated_replay': 'REQUIRED_WHEN_EXECUTED',
            'six_motion_scenarios': 'REQUIRED_WHEN_EXECUTED'},
        'cartographer_metrics': __import__(
            'g006_candidate_contract').CARTOGRAPHER_METRICS,
        'mapping_claim': 'SIMULATION_MAPPING_ONLY',
        'real_bag_claim': 'ISOLATED_REPLAY_DIAGNOSTIC_NO_GT',
    }


def _resolved_profile(profile: str) -> dict:
    common = {'use_sim_time': False, 'autostart': True,
              'hardware_substitution': True}
    if profile == 'A':
        return {**common, 'use_composition': False,
                'num_subdivisions_per_laser_scan': 1,
                'default_nav_to_pose_bt_xml':
                    'navigate_to_pose_safe_mapping.xml',
                'nav2_parameter_source_sha256': sha256_file(
                    REPO_ROOT / 'jdamr_cube_navigation/config/nav2_params.yaml')}
    parameters = yaml.safe_load((
        REPO_ROOT / 'jdamr_cube_navigation/config/nav2_params.yaml').read_text(
            encoding='utf-8'))
    return {**common, 'record_bag': True,
            'container_executable': 'component_container_isolated',
            'default_nav_to_pose_bt_xml':
                'navigate_to_pose_corridor_fail_fast.xml',
            'amcl_runtime': 'stock_production_binary_only',
            'stock_amcl_applicable_parameters': parameters['amcl'][
                'ros__parameters'],
            'keepout_enabled': True}


def _copy_profiles(payload: Path) -> tuple[dict, list[dict]]:
    profile_records = {}
    all_sources = []
    for profile, specifications in PROFILE_SOURCES.items():
        records = []
        for root_name, relative in specifications:
            source = _source_path(root_name, relative)
            output_relative = f'profiles/{profile}/sources/{root_name}/{relative}'
            destination = payload / output_relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination, follow_symlinks=False)
            source_record = {
                'source_root': root_name, 'source_relative_path': relative,
                **file_identity(source, output_relative),
            }
            source_record['bundle_sha256'] = sha256_file(destination)
            source_record['bundle_size_bytes'] = destination.stat().st_size
            records.append(source_record)
            all_sources.append(source_record)
        resolved_path = payload / f'profiles/{profile}/resolved_params.json'
        resolved_path.parent.mkdir(parents=True, exist_ok=True)
        resolved_path.write_bytes(canonical_json_bytes(_resolved_profile(profile)))
        records.append({'source_root': 'generated',
                        'source_relative_path': 'resolved_params.json',
                        **file_identity(
                            resolved_path,
                            f'profiles/{profile}/resolved_params.json'),
                        'bundle_sha256': sha256_file(resolved_path),
                        'bundle_size_bytes': resolved_path.stat().st_size})
        all_sources.append(records[-1])
        profile_records[profile] = records
    return profile_records, all_sources


def _copy_harness_sources(payload: Path) -> list[dict]:
    records = []
    for relative in HARNESS_SOURCES:
        source = _source_path('repository', relative)
        output_relative = f'harness/{Path(relative).name}'
        destination = payload / output_relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination, follow_symlinks=False)
        record = {'source_root': 'repository',
                  'source_relative_path': relative,
                  **file_identity(source, output_relative),
                  'bundle_sha256': sha256_file(destination),
                  'bundle_size_bytes': destination.stat().st_size}
        records.append(record)
    return records


def _protocol_id(profiles: dict, harness_sources: list,
                 predecessors: dict, topologies: dict,
                 hardware_boundary: dict, scenario_contract: dict,
                 measurement_contract: dict) -> str:
    """Bind immutable inputs and contracts without circular run attempts."""
    sources = [item for profile in ('A', 'B') for item in profiles[profile]]
    scenario_definition = {
        key: value for key, value in scenario_contract.items()
        if key != 'attempts'}
    material = canonical_json_bytes({
        'sources': sources, 'harness_sources': harness_sources,
        'predecessor_tree_hashes': {
            name: (record.get('artifact_tree', {}).get('tree_sha256') or
                   record['status'])
            for name, record in predecessors.items()},
        'topologies': topologies, 'hardware_boundary': hardware_boundary,
        'scenario_contract': scenario_definition,
        'measurement_contract': measurement_contract})
    return hashlib.sha256(material).hexdigest()


def build_bundle(output_root: Path) -> dict:
    """Build and atomically publish one immutable offline bundle."""
    output_root = output_root.absolute()
    if output_root.exists() or output_root.is_symlink():
        raise FileExistsError(output_root)
    output_root.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=f'.{output_root.name}.stage-',
                                  dir=output_root.parent))
    try:
        payload = stage / 'payload'
        payload.mkdir()
        profiles, all_sources = _copy_profiles(payload)
        harness_sources = _copy_harness_sources(payload)
        predecessors = _predecessor_records(payload)
        dependency_hashes = {
            name: (record['identities'][0]['sha256']
                   if 'identities' in record else record['status'])
            for name, record in predecessors.items()}
        dependency_hashes['G002'] = 'NOT_EVALUATED'
        dependency_hashes['G005'] = 'NOT_EVALUATED'
        topologies = _topologies()
        hardware_boundary = _hardware_boundary()
        scenario_contract = _scenario_contract()
        measurement_contract = _measurement_contract()
        protocol_id = _protocol_id(
            profiles, harness_sources, predecessors, topologies,
            hardware_boundary, scenario_contract, measurement_contract)
        seed_lock = held_out_seed_lock(
            f'G006-{protocol_id}', dependency_hashes)
        gates = gate_matrix()
        external = {'no_motion_exact_launch': 'NOT_EVALUATED',
                    'real_bag_isolated_replay': 'NOT_EVALUATED'}
        readiness = {
            'status': 'PENDING',
            'blockers': [
                'G002_FULL_NOT_EVALUATED', 'G005_FULL_NOT_EVALUATED',
                'SCENARIO_ATTEMPTS_NOT_EXECUTED',
                'RUNTIME_EQUIVALENCE_NOT_EVALUATED',
                'GOLDEN_VECTORS_NOT_EVALUATED',
                'RUNTIME_BINARY_IDENTITY_NOT_EVALUATED',
                'SEALED_ROS2_MCAP_RUNTIME_GRAPH_VALIDATION_PENDING'],
        }
        manifest = {
            'schema_version': 1,
            'claim_scope': 'OFFLINE_IMMUTABLE_TOPOLOGY_PROXY_NO_MOTION',
            'bundle_id': protocol_id,
            'profiles': profiles,
            'harness_sources': harness_sources,
            'topologies': topologies,
            'hardware_boundary': hardware_boundary,
            'runtime': {
                'ros_distro': os.environ.get('ROS_DISTRO', 'jazzy'),
                'python_version': platform.python_version(),
                'os_system': platform.system(),
                'os_release': platform.release(),
                'machine': platform.machine(),
                'packages': _package_versions(),
            },
            'predecessors': predecessors,
            'selection': {
                'amcl': 'stock_production_applicable_parameters',
                'patched_random_seed_binary_included': False,
                'frontier_policy': 'current',
                'frontier_reason': 'G005_FULL_NOT_EVALUATED',
                'g003_g004_runtime_equivalence': 'NOT_EVALUATED',
                'g005_golden_decision_vectors': 'NOT_EVALUATED',
                'runtime_binary_identity': 'NOT_EVALUATED',
                'cartographer_subdivision': 1,
                'cartographer_reason': 'G008_RETAIN_PRODUCTION_SUBDIVISION_1',
            },
            'seed_lock': seed_lock,
            'scenario_contract': scenario_contract,
            'measurement_contract': measurement_contract,
            'gate_applicability': gates,
            'external_hard_gates': external,
            'pi_historical_saved_nav': 'NOT_EVALUATED',
            'pi_current_bundle': 'NOT_RUN',
            'pi_resource_gate': 'NOT_EVALUATED_CURRENT_BUNDLE',
            'claims': CLAIMS_LOCK,
            'readiness_evidence': readiness,
            'decision': final_decision(gates, external, readiness),
        }
        probe_path = payload / 'offline_probe.json'
        probe_path.write_bytes(canonical_json_bytes({
            'status': 'NOT_EVALUATED',
            'reason': 'BUILDER_DOES_NOT_LAUNCH_ROS_OR_HARDWARE'}))
        stub_path = payload / 'hardware_stub_topology.json'
        stub_path.write_bytes(canonical_json_bytes({
            'mode': 'typed_endpoint_contract_only_no_publishers_started',
            'endpoints': list(HARDWARE_STUBS)}))
        manifest['payload_tree'] = tree_inventory(payload)
        manifest_path = stage / 'bundle_manifest.json'
        manifest_path.write_bytes(canonical_json_bytes(manifest))
        total = manifest_path.stat().st_size + manifest['payload_tree']['total_bytes']
        if total > BUNDLE_LIMIT_BYTES:
            raise ValueError('bundle exceeds 64 MiB cap')
        # Validate sources immediately before atomic publication.
        for record in all_sources + harness_sources:
            if record['source_root'] == 'generated':
                continue
            source = _source_path(record['source_root'],
                                  record['source_relative_path'])
            if (sha256_file(source) != record['sha256'] or
                    source.stat().st_size != record['size_bytes']):
                raise ValueError('production source changed during build')
        stage.rename(output_root)
        return manifest
    except BaseException:
        if stage.exists():
            shutil.rmtree(stage)
        raise


def main() -> int:
    """Build a single offline candidate bundle."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args()
    manifest = build_bundle(args.output_root)
    print(manifest['decision'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
