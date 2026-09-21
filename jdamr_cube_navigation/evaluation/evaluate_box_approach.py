"""Evaluate ideal planar feedback without ROS, drivers, or motion topics."""

import argparse
import csv
import json
import math
from pathlib import Path

from jdamr_cube_navigation.box_approach import BoxApproach, relative_target, wrap
from jdamr_cube_navigation.parking import load_parking_contract


def evaluate(output):
    """Save every proposal and terminal error for fifteen synthetic scenarios."""
    output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    contract = load_parking_contract(root / 'config/parking_contract.yaml')
    # Synthetic fixture geometry, not a new physical measurement.
    camera, body = (.065, 0., 0.), (-.275, .065, -.27, .27)
    initial_poses = [(0., 0., 0.), (0., .15, 0.), (0., -.15, 0.),
                     (0., .10, .25), (0., -.10, -.25)]
    results = []
    for index, (initial, face_yaw) in enumerate(
            (p, a) for p in initial_poses for a in (-.15, 0., .15)):
        policy = BoxApproach(contract, camera, body)
        pose, velocity, rows = initial, (0., 0.), []
        for step in range(1800):
            now_s = 1. + .05 * step
            x, y, yaw = pose
            dx, dy = 1.2 - x, -y
            c, s = math.cos(yaw), math.sin(yaw)
            sample = {
                'stamp_s': now_s, 'detected': True, 'stable': True,
                'surface_kind': 'front', 'confidence': .95,
                'front_distance_m': c*dx + s*dy - camera[0],
                'lateral_error_m': s*dx - c*dy,
                'edge_angle_deg': math.degrees(wrap(face_yaw - yaw)),
            }
            result = policy.step(
                sample, pose, now_s=now_s, odom_stamp_s=now_s,
                linear_mps=velocity[0], angular_radps=velocity[1],
                cmd_linear_mps=velocity[0], cmd_angular_radps=velocity[1])
            v, w = result['linear_mps'], result['angular_radps']
            rows.append([now_s, x, y, yaw, v, w, result['state']])
            if result['state'] in ('SUCCEEDED', 'ABORTED'):
                break
            pose = (x + v*c*.05, y + v*s*.05, wrap(yaw + w*.05))
            velocity = v, w
        goal, clearance = relative_target(sample, camera, body, policy.config)
        with (output / f'case_{index:02d}.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s', 'x_m', 'y_m', 'yaw_rad',
                             'linear_mps', 'angular_radps', 'state'])
            writer.writerows(rows)
        results.append({
            'case': index, 'initial_pose': initial, 'face_yaw_rad': face_yaw,
            'state': result['state'], 'duration_s': now_s - 1.,
            'position_error_m': math.hypot(*goal[:2]),
            'yaw_error_deg': math.degrees(abs(goal[2])),
            'lower_base_clearance_m': clearance,
        })
    report = {
        'scope': 'ideal planar kinematics; not physics, sensor noise or physical validation',
        'geometry_scope': 'synthetic lower base without arms or payload',
        'cases': results, 'passed': sum(r['state'] == 'SUCCEEDED' for r in results),
        'max_position_error_m': max(r['position_error_m'] for r in results),
        'max_yaw_error_deg': max(r['yaw_error_deg'] for r in results),
    }
    (output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if report['passed'] != len(results):
        raise RuntimeError('synthetic approach did not converge')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    evaluate(parser.parse_args().output)
