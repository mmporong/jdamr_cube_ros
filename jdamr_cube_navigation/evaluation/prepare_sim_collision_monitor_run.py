#!/usr/bin/env python3
"""Generate deterministic G004 evaluation contract and parameter overlay."""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from sim_collision_monitor_contract import (
    derive_contract, sha256_file, write_contract)

import yaml


ROOT = Path(__file__).resolve().parents[2]
G003_ASSETS = (ROOT / 'jdamr_cube_navigation' / 'evaluation' / 'assets'
               / 'nav_obstacle')
PRODUCTION_BRIDGE = ROOT / 'jdamr_cube_gazebo' / 'params' / 'bridge.yaml'
SCAN_PROFILE = (
    ROOT / 'jdamr_cube_navigation' / 'evaluation' / 'media'
    / 'corridor_localdds_armed_20260904T152036' / 'sensor_profile.json')
PRODUCTION_FILES = {
    'navigation_launch': (
        ROOT / 'jdamr_cube_navigation' / 'launch' / 'navigation.launch.py'),
    'onboard_nav2_core_launch': (
        ROOT / 'jdamr_cube_navigation' / 'launch'
        / 'onboard_nav2_core.launch.py'),
    'production_urdf': (
        ROOT / 'jdamr_cube_description' / 'urdf' / 'jdamr_cube.urdf'),
}


def installed_nav2_versions() -> dict[str, str]:
    """Capture the installed Debian versions used by this evaluation."""
    packages = (
        'ros-jazzy-nav2-collision-monitor', 'ros-jazzy-nav2-msgs')
    result = subprocess.run(
        ['dpkg-query', '-W', '-f=${Package}=${Version}\n', *packages],
        check=True, capture_output=True, text=True)
    return dict(line.split('=', 1) for line in result.stdout.splitlines())


def _production_motion_inputs(params: dict, urdf_path: Path) -> dict:
    """Read motion bounds from the exact production inputs."""
    controller = params['controller_server']['ros__parameters'][
        'FollowPath']
    goal_checkers = params['controller_server']['ros__parameters']
    smoother = params['velocity_smoother']['ros__parameters']
    controller_speed = float(controller['desired_linear_vel'])
    smoother_speed = float(smoother['max_velocity'][0])
    if not math.isclose(controller_speed, smoother_speed, abs_tol=1e-12):
        raise ValueError('controller and smoother forward speeds differ')
    max_decel_mps2 = abs(float(smoother['max_decel'][0]))
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = params['global_costmap']['global_costmap'][
        'ros__parameters']
    if local['footprint'] != global_costmap['footprint']:
        raise ValueError('local and global costmap footprints differ')
    if not math.isclose(
            float(local['resolution']), float(global_costmap['resolution']),
            abs_tol=1e-12):
        raise ValueError('local and global costmap resolutions differ')
    points = json.loads(local['footprint'])
    footprint = {
        'front_m': max(float(point[0]) for point in points),
        'rear_m': min(float(point[0]) for point in points),
        'half_width_m': max(abs(float(point[1])) for point in points),
    }
    tree = ET.parse(urdf_path)
    joints = {joint.get('name'): joint for joint in tree.findall('.//joint')}
    links = {link.get('name'): link for link in tree.findall('.//link')}
    front_joint = joints['caster_front_joint'].find('origin')
    rear_joint = joints['caster_rear_joint'].find('origin')
    front_radius = float(links['caster_link_front'].find(
        'collision/geometry/sphere').get('radius'))
    rear_radius = float(links['caster_link_rear'].find(
        'collision/geometry/sphere').get('radius'))
    left_joint = joints['left_wheel_joint'].find('origin')
    left_width = float(links['left_wheel_link'].find(
        'collision/geometry/cylinder').get('length'))
    urdf_footprint = {
        'front_m': float(front_joint.get('xyz').split()[0]) + front_radius,
        'rear_m': float(rear_joint.get('xyz').split()[0]) - rear_radius,
        'half_width_m': (
            abs(float(left_joint.get('xyz').split()[1])) + left_width / 2.0),
    }
    if any(not math.isclose(footprint[key], urdf_footprint[key], abs_tol=1e-12)
           for key in footprint):
        raise ValueError('costmap footprint differs from production URDF')
    if min(controller_speed, max_decel_mps2, local['resolution']) <= 0.0:
        raise ValueError('motion bounds must be positive')
    general_tolerance = float(
        goal_checkers['general_goal_checker']['xy_goal_tolerance'])
    position_tolerance = float(
        goal_checkers['position_goal_checker']['xy_goal_tolerance'])
    if not math.isclose(
            general_tolerance, position_tolerance, abs_tol=1e-12):
        raise ValueError('configured goal position tolerances differ')
    configured_goal_checkers = goal_checkers['goal_checker_plugins']
    if (not isinstance(configured_goal_checkers, list)
            or not configured_goal_checkers):
        raise ValueError('at least one goal checker must be configured')
    selected_goal_checker = configured_goal_checkers[0]
    selected_checker = goal_checkers[selected_goal_checker]
    if selected_checker.get('plugin') != 'nav2_controller::SimpleGoalChecker':
        raise ValueError('selected goal checker plugin is not SimpleGoalChecker')
    selected_checker_stateful = selected_checker.get('stateful')
    if selected_checker_stateful is not True:
        raise ValueError('selected goal checker must be stateful')
    estimate_stamp_tolerance_s = float(
        params['amcl']['ros__parameters']['transform_tolerance'])
    if estimate_stamp_tolerance_s <= 0.0:
        raise ValueError('AMCL transform tolerance must be positive')
    return {
        'max_forward_speed_mps': controller_speed,
        'max_decel_mps2': max_decel_mps2,
        'costmap_resolution_m': float(local['resolution']),
        'footprint_front_m': footprint['front_m'],
        'footprint_rear_m': footprint['rear_m'],
        'footprint_half_width_m': footprint['half_width_m'],
        'goal_position_tolerance_m': general_tolerance,
        'estimate_stamp_tolerance_s': estimate_stamp_tolerance_s,
        'goal_checker_name': selected_goal_checker,
        'goal_checker_plugin': selected_checker['plugin'],
        'goal_checker_stateful': selected_checker_stateful,
    }


def prepare(production_params: Path, world: Path, output: Path) -> dict:
    """Prepare evaluation-only parameter deltas without editing production."""
    source_urdf = G003_ASSETS / 'jdamr_cube_nav_eval.urdf'
    if not source_urdf.is_file():
        raise FileNotFoundError(
            f'required G003 evaluation URDF is missing: {source_urdf}')
    if not SCAN_PROFILE.is_file():
        raise FileNotFoundError(
            f'required scan timing profile is missing: {SCAN_PROFILE}')
    params = yaml.safe_load(production_params.read_text())
    smoother = params['velocity_smoother']['ros__parameters']
    production_urdf = PRODUCTION_FILES['production_urdf']
    motion = _production_motion_inputs(params, production_urdf)
    world_text = world.read_text()
    if 'gz-sim-contact-system' not in world_text:
        raise ValueError('evaluation world must include the Contact system')
    marker = '<max_step_size>'
    physics_step_s = float(
        world_text.split(marker, 1)[1].split('</max_step_size>', 1)[0])
    scan_profile = json.loads(SCAN_PROFILE.read_text())
    max_scan_gap_s = scan_profile['timing']['scan'][
        'header_interval_s']['max']
    contract = derive_contract(
        float(smoother['smoothing_frequency']), physics_step_s,
        installed_nav2_versions(), max_scan_gap_s, **motion)
    contract['scan_gap_provenance'] = {
        'path': str(SCAN_PROFILE.resolve()),
        'size_bytes': SCAN_PROFILE.stat().st_size,
        'sha256': sha256_file(SCAN_PROFILE),
        'json_pointer': '/timing/scan/header_interval_s/max',
        'value_s': max_scan_gap_s,
        'observed_on': '2026-09-04',
        'valid_for': 'G004 simulated StopZone reaction-margin derivation only',
    }
    production_files = {
        'production_params': production_params,
        **PRODUCTION_FILES,
    }
    contract['production_inputs'] = {
        name: {
            'path': str(path.resolve()),
            'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path),
        }
        for name, path in production_files.items()
    }
    contract['evaluation_world_source'] = {
        'path': str(world.resolve()),
        'size_bytes': world.stat().st_size,
        'sha256': sha256_file(world),
        'contact_system_present': True,
    }
    monitor_parameters = {
            'source_timeout': contract['selected_source_timeout_s'],
            'use_sim_time': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'cmd_vel_in_topic': 'cmd_vel_smoothed',
            'cmd_vel_out_topic': 'cmd_vel',
            'state_topic': 'collision_monitor_state',
            'observation_sources': ['scan'],
            'scan': {
                'type': 'scan',
                'topic': '/collision_monitor_scan',
                'enabled': True,
            },
            'polygons': ['StopZone'],
            'StopZone': {
                'type': 'polygon',
                'points': json.dumps([
                    [contract['stop_zone']['front_m'],
                     contract['stop_zone']['half_width_m']],
                    [contract['stop_zone']['front_m'],
                     -contract['stop_zone']['half_width_m']],
                    [contract['stop_zone']['rear_m'],
                     -contract['stop_zone']['half_width_m']],
                    [contract['stop_zone']['rear_m'],
                     contract['stop_zone']['half_width_m']],
                ]),
                'action_type': 'stop',
                'min_points': 3,
                'enabled': True,
            },
    }
    overlay = {'collision_monitor': {'ros__parameters': monitor_parameters}}
    evaluation_params = copy.deepcopy(params)
    evaluation_params.setdefault('amcl', {}).setdefault(
        'ros__parameters', {}).setdefault('initial_pose', {}).update({
            'x': -8.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0})
    evaluation_params.setdefault('collision_monitor', {}).setdefault(
        'ros__parameters', {}).update(monitor_parameters)
    contract['start_pose'] = {'x_m': -8.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    contract['goal_pose'] = {'x_m': 6.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    output.mkdir(parents=True, exist_ok=False)
    write_contract(output / 'contract.json', contract)
    (output / 'collision_monitor_overlay.yaml').write_text(
        yaml.safe_dump(overlay, sort_keys=True))
    (output / 'nav2_collision_monitor_eval.params.yaml').write_text(
        yaml.safe_dump(evaluation_params, sort_keys=False))
    if source_urdf.is_file():
        tree = ET.parse(source_urdf)
        lidar = next(sensor for sensor in tree.findall('.//sensor')
                     if sensor.get('type') == 'gpu_lidar')
        laser_joint = next(
            joint for joint in tree.findall('.//joint')
            if joint.get('name') == 'laser_joint')
        origin = laser_joint.find('origin')
        xyz = [float(value) for value in origin.get('xyz').split()]
        rpy = [float(value) for value in origin.get('rpy').split()]
        contract['scan_to_base_transform'] = {
            'x_m': xyz[0], 'y_m': xyz[1], 'yaw_rad': rpy[2],
            'source_joint': 'laser_joint',
            'source_frame': laser_joint.find('child').get('link')}
        lidar.find('update_rate').text = '10'
        lidar.find('topic').text = 'sim_raw/scan'
        eval_urdf = output / 'jdamr_cube_collision_monitor_eval.urdf'
        tree.write(eval_urdf, encoding='unicode', xml_declaration=True)
        contract['evaluation_assets'] = {
            'urdf': {
                'path': str(eval_urdf.resolve()),
                'size_bytes': eval_urdf.stat().st_size,
                'sha256': sha256_file(eval_urdf),
                'source_path': str(source_urdf.resolve()),
                'source_size_bytes': source_urdf.stat().st_size,
                'source_sha256': sha256_file(source_urdf),
                'allowed_deltas': {
                    'gpu_lidar.update_rate_hz': [2.0, 10.0],
                    'gpu_lidar.topic': ['scan', 'sim_raw/scan'],
                },
            },
            'collision_monitor_overlay': {
                'path': str((
                    output / 'collision_monitor_overlay.yaml').resolve()),
                'size_bytes': (
                    output / 'collision_monitor_overlay.yaml').stat().st_size,
                'sha256': sha256_file(
                    output / 'collision_monitor_overlay.yaml'),
                'source_path': str(production_params.resolve()),
                'source_size_bytes': production_params.stat().st_size,
                'source_sha256': sha256_file(production_params),
            },
            'nav2_evaluation_params': {
                'path': str((
                    output
                    / 'nav2_collision_monitor_eval.params.yaml').resolve()),
                'size_bytes': (
                    output / 'nav2_collision_monitor_eval.params.yaml'
                ).stat().st_size,
                'sha256': sha256_file(
                    output / 'nav2_collision_monitor_eval.params.yaml'),
                'source_path': str(production_params.resolve()),
                'source_size_bytes': production_params.stat().st_size,
                'source_sha256': sha256_file(production_params),
            },
        }
        for name in ('slam_corridor_eval.pgm', 'slam_corridor_eval.yaml'):
            source = G003_ASSETS / name
            target = output / name
            target.write_bytes(source.read_bytes())
            contract['evaluation_assets'][name] = {
                'path': str(target.resolve()),
                'size_bytes': target.stat().st_size,
                'sha256': sha256_file(target),
                'source_path': str(source.resolve()),
                'source_size_bytes': source.stat().st_size,
                'source_sha256': sha256_file(source),
                'copy_byte_identical': (
                    sha256_file(target) == sha256_file(source)),
            }
        eval_world = output / 'slam_corridor_contact.world'
        eval_world.write_bytes(world.read_bytes())
        bridge = yaml.safe_load(PRODUCTION_BRIDGE.read_text())
        scan_bridge = next(
            item for item in bridge if item['ros_type_name']
            == 'sensor_msgs/msg/LaserScan')
        scan_bridge['ros_topic_name'] = 'sim_raw/scan'
        scan_bridge['gz_topic_name'] = 'sim_raw/scan'
        bridge_path = output / 'collision_monitor_bridge.yaml'
        bridge_path.write_text(yaml.safe_dump(bridge, sort_keys=False))
        contract['evaluation_assets']['world'] = {
            'path': str(eval_world.resolve()),
            'size_bytes': eval_world.stat().st_size,
            'sha256': sha256_file(eval_world),
            'contact_system_present': True,
            'source_path': str(world.resolve()),
            'source_size_bytes': world.stat().st_size,
            'source_sha256': sha256_file(world),
            'copy_byte_identical': (
                sha256_file(eval_world) == sha256_file(world)),
        }
        contract['evaluation_assets']['bridge'] = {
            'path': str(bridge_path.resolve()),
            'size_bytes': bridge_path.stat().st_size,
            'sha256': sha256_file(bridge_path),
            'source_path': str(PRODUCTION_BRIDGE.resolve()),
            'source_size_bytes': PRODUCTION_BRIDGE.stat().st_size,
            'source_sha256': sha256_file(PRODUCTION_BRIDGE),
        }
        write_contract(output / 'contract.json', contract)
        generated_names = {
            'contract.json', 'collision_monitor_overlay.yaml',
            'nav2_collision_monitor_eval.params.yaml',
            'jdamr_cube_collision_monitor_eval.urdf',
            'slam_corridor_eval.pgm', 'slam_corridor_eval.yaml',
            'slam_corridor_contact.world', 'collision_monitor_bridge.yaml',
        }
        manifest_records = []
        for name in sorted(generated_names):
            path = output / name
            manifest_records.append({
                'relative_path': name,
                'size_bytes': path.stat().st_size,
                'sha256': sha256_file(path),
            })
        (output / 'preparation_manifest.json').write_text(json.dumps({
            'schema_version': 1,
            'prepared_root': str(output.resolve()),
            'expected_generated_paths': sorted(generated_names),
            'records': manifest_records,
        }, indent=2, sort_keys=True) + '\n')
    return contract


def main() -> int:
    """Prepare one versioned G004 evaluation directory."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--production-params', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.production_params, args.world, args.output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
