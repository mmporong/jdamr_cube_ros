#!/usr/bin/env python3
"""Run one bounded Nav2 precision-parking simulation and retain evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Any

from navigation_mcap_reader import read_navigation_messages
from prepare_parking_params import prepare as prepare_parking_params
from run_onboard_candidate_smoke import (
    _cap_output, _cleanup_identity, _environment, _group_members,
    _one_finalized_mcap, _parameter, _source_identity, _start,
    _start_compact_recorder, _stop, _wait_bag_ready,
    _wait_lifecycle_active, _wait_log_markers, _wait_tf_available,
    _wait_topics, ASSETS, prepare_candidate_assets,
)
import yaml


DOMAIN_IDS = {186, 187}
SEED = 11
GOAL_X_M = -5.5
GOAL_Y_M = 0.0
GOAL_YAW_RAD = math.pi / 2.0


def _yaw(rotation: Any) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))


def _angle_error_rad(actual_rad: float, expected_rad: float) -> float:
    difference_rad = actual_rad - expected_rad
    return abs(math.atan2(math.sin(difference_rad),
                          math.cos(difference_rad)))


def _mask_image_and_hash(mask_yaml: Path) -> tuple[Path, str]:
    metadata = yaml.safe_load(mask_yaml.read_text(encoding='utf-8'))
    image = (mask_yaml.parent / metadata['image']).resolve()
    return image, hashlib.sha256(image.read_bytes()).hexdigest()


def _write_route(path: Path, prepared: dict[str, Any]) -> None:
    _image, mask_hash = _mask_image_and_hash(prepared['mask'])
    path.write_text(yaml.safe_dump({
        'schema_version': 1,
        'frame_id': 'map',
        'map_yaml': str((ASSETS / 'slam_corridor_eval.yaml').resolve()),
        'keepout_mask_yaml': str(prepared['mask'].resolve()),
        'expected_mask_sha256': mask_hash,
        'start_pose': {'x': -8.0, 'y': 0.0},
        'minimum_battery_v': 10.5,
        'waypoints': [
            {'id': 'parking_approach', 'x': -7.0, 'y': 0.0, 'yaw': 0.0},
            {'id': 'parking_target', 'x': GOAL_X_M, 'y': GOAL_Y_M,
             'yaw': GOAL_YAW_RAD},
        ],
    }, sort_keys=False), encoding='utf-8')


def _evaluate(mcap: Path, route_log: Path, contract: dict[str, Any]) -> dict:
    poses = []
    commands = []
    statuses = []
    for message in read_navigation_messages(
            mcap, topics=['/ground_truth_pose', '/cmd_vel',
                          '/navigate_to_pose/_action/status']):
        if message.channel.topic == '/ground_truth_pose':
            pose = message.ros_msg.pose
            poses.append((message.log_time_ns, float(pose.position.x),
                          float(pose.position.y), _yaw(pose.orientation)))
        elif message.channel.topic == '/cmd_vel':
            velocity = message.ros_msg
            commands.append((message.log_time_ns, float(velocity.linear.x),
                             float(velocity.angular.z)))
        else:
            statuses.extend(
                (message.log_time_ns,
                 bytes(item.goal_info.goal_id.uuid).hex(), int(item.status))
                for item in message.ros_msg.status_list
                if any(item.goal_info.goal_id.uuid))
    if not poses or not commands or not statuses:
        raise RuntimeError('parking MCAP is missing required evidence')
    final = poses[-1]
    position_error_m = math.hypot(final[1] - GOAL_X_M,
                                  final[2] - GOAL_Y_M)
    yaw_error_rad = _angle_error_rad(final[3], GOAL_YAW_RAD)
    text = route_log.read_text(errors='replace')
    confirmed = '"event":"parking_estimate_confirmed"' in text
    goal_uuids = sorted({item[1] for item in statuses})
    terminal_by_uuid = {
        uuid: [status for _stamp, candidate, status in statuses
               if candidate == uuid][-1]
        for uuid in goal_uuids
    }
    return {
        'status': 'PASS' if (
            confirmed and position_error_m <= contract['xy_tolerance_m']
            and yaw_error_rad <= contract['yaw_tolerance_rad']
            and list(terminal_by_uuid.values()).count(4) == 2) else 'FAIL',
        'claim_scope': 'GAZEBO_SIMULATION_ONLY',
        'parking_estimate_confirmed': confirmed,
        'physical_accuracy': 'NOT_MEASURED',
        'ground_truth': {
            'final_pose_xy_yaw': list(final[1:]),
            'position_error_m': position_error_m,
            'yaw_error_rad': yaw_error_rad,
            'yaw_error_deg': math.degrees(yaw_error_rad),
            'sample_count': len(poses),
        },
        'commands': {'sample_count': len(commands)},
        'action_status': {
            'goal_uuids': goal_uuids,
            'terminal_status_by_uuid': terminal_by_uuid,
        },
    }


def run(output_root: Path, domain_id: int) -> dict[str, Any]:
    """Start isolated simulation, execute the parking route, and tear down."""
    output_root.mkdir(parents=True, exist_ok=False)
    prepared = prepare_candidate_assets(output_root)
    parking_params = output_root / 'assets' / 'parking_nav2_params.yaml'
    prepare_parking_params(
        prepared['params'],
        Path(__file__).resolve().parents[1] / 'config/parking_contract.yaml',
        parking_params)
    route = output_root / 'assets' / 'parking_route.yaml'
    _write_route(route, prepared)
    contract_path = (Path(__file__).resolve().parents[1] /
                     'config/parking_contract.yaml')
    contract = yaml.safe_load(contract_path.read_text(encoding='utf-8'))
    contract['yaw_tolerance_rad'] = math.radians(contract['yaw_tolerance_deg'])
    run_id = f'parking_smoke_seed_{SEED}'
    environment = _environment(run_id, domain_id)
    environment.update({
        'ROS_LOCALHOST_ONLY': '1',
        'GZ_PARTITION': f'jdamr_parking_{os.getpid()}',
    })
    launched = []
    recorder = None
    bag_dir = None
    result = {
        'schema_version': 1, 'status': 'FAIL', 'seed': SEED,
        'domain_id': domain_id, 'run_id': run_id,
        'claim_scope': 'GAZEBO_SIMULATION_ONLY',
        'physical_accuracy': 'NOT_MEASURED',
        'source_identity': {
            'runner': _source_identity(Path(__file__)),
            'parking_params': _source_identity(parking_params),
            'parking_route': _source_identity(route),
            'parking_contract': _source_identity(contract_path),
        },
    }
    try:
        launched.append(_start([
            'ros2', 'launch', 'jdamr_cube_gazebo', 'gazebo.launch.py',
            f'world:={ASSETS / "slam_corridor_contact.world"}',
            f'urdf_file:={ASSETS / "jdamr_cube_nav_eval.urdf"}',
            'gui:=false', 'enable_image_bridges:=false', f'seed:={SEED}',
            'x_pose:=-8.0', 'y_pose:=0.0', 'z_pose:=0.01',
        ], output_root / 'gazebo.log', environment))
        _wait_topics({'/scan', '/odom', '/ground_truth_pose'},
                     environment, 60.0)
        launched.append(_start([
            'ros2', 'launch', 'jdamr_cube_navigation',
            'onboard_nav2_core.launch.py',
            f'map:={ASSETS / "slam_corridor_eval.yaml"}',
            f'keepout_mask:={prepared["mask"]}',
            f'params_file:={parking_params}', 'use_sim_time:=true',
            'autostart:=true', 'navigation_profile:=corridor',
        ], output_root / 'onboard_nav2_core.log', environment))
        _wait_topics({'/clock', '/cmd_vel', '/plan',
                      '/keepout_filter_mask'}, environment, 75.0,
                     output_root / 'onboard_nav2_core.log')
        _wait_lifecycle_active((
            '/keepout_filter_mask_server',
            '/keepout_costmap_filter_info_server', '/bt_navigator'),
            environment, 75.0)
        _wait_tf_available('map', 'base_link', environment, 60.0,
                           output_root / 'tf_readiness.log')
        _wait_log_markers(output_root / 'onboard_nav2_core.log', (
            ('[local_costmap.local_costmap]: KeepoutFilter: '
             'Received filter mask'),
            ('[global_costmap.global_costmap]: KeepoutFilter: '
             'Received filter mask'),
        ), 15.0)
        launched.append(_start([
            'ros2', 'topic', 'pub', '--rate', '2', '/battery_state',
            'sensor_msgs/msg/BatteryState', '{voltage: 12.0}',
        ], output_root / 'battery.log', environment))
        recorder, bag_dir = _start_compact_recorder(output_root, environment)
        launched.append(recorder)
        _wait_bag_ready(recorder[0], bag_dir)
        route_process = _start([
            'ros2', 'run', 'jdamr_cube_navigation', 'corridor_route',
            '--route', str(route), '--park-final',
            '--parking-contract', str(contract_path), '--execute',
        ], output_root / 'route.log', environment)
        launched.append(route_process)
        returncode = route_process[0].wait(timeout=180.0)
        if recorder[0].poll() is not None:
            raise RuntimeError('compact recorder exited during parking')
        _stop(recorder[0])
        mcap = _one_finalized_mcap(bag_dir)
        result.update(_evaluate(mcap, output_root / 'route.log', contract))
        result['route_returncode'] = returncode
        result['recording'] = {
            'mcap': _source_identity(mcap), 'size_bytes': mcap.stat().st_size,
            'log_time_basis': 'wall_time',
            'message_header_time_basis': 'publisher_defined_sim_time',
        }
        result['runtime_parameters'] = {
            name: _parameter('/controller_server', name, environment)
            for name in (
                'Parking.desired_linear_vel',
                'Parking.min_approach_linear_velocity',
                'Parking.rotate_to_heading_angular_vel',
                'parking_goal_checker.xy_goal_tolerance',
                'parking_goal_checker.yaw_goal_tolerance')
        }
        if returncode != 0:
            result['status'] = 'FAIL'
    except Exception as error:
        result['harness_error'] = f'{type(error).__name__}: {error}'
    finally:
        groups = [process.pid for process, _stream in launched]
        for process, _stream in reversed(launched):
            _stop(process)
        for _process, stream in launched:
            stream.close()
        result['teardown'] = {
            'remaining_process_groups': [group for group in groups
                                         if _group_members(group)],
            'identity_survivors': _cleanup_identity(run_id, domain_id),
        }
        if any(result['teardown'].values()):
            result['status'] = 'FAIL'
        result['output_bytes'] = _cap_output(output_root)
        (output_root / 'summary.json').write_text(json.dumps(
            result, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--domain-id', type=int, default=186)
    args = parser.parse_args()
    if args.domain_id not in DOMAIN_IDS:
        parser.error('only isolated ROS domain 186 or 187 is allowed')
    if args.output_root.exists():
        parser.error('output root must not exist')
    result = run(args.output_root, args.domain_id)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if result['status'] == 'PASS' else 2


if __name__ == '__main__':
    sys.exit(main())
