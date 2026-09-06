#!/usr/bin/env python3
"""Read-only topology probe for a hardware-substituted G006 bundle."""

from __future__ import annotations

import argparse
from pathlib import Path

from g006_candidate_contract import canonical_json_bytes, strict_json_load


REQUIRED_TOKENS = {
    'A': {
        'jdamr_cube_bringup/launch/real_bringup.launch.py': (
            "executable='base_driver_node'", "executable='ydlidar_g4_node'"),
        'jdamr_cube_cartographer/launch/cartographer_real.launch.py': (
            "executable='cartographer_node'", 'jdamr_cube_2d_real.lua'),
        'jdamr_cube_navigation/launch/autonomous_mapping.launch.py': (
            "'use_composition': 'False'", 'navigate_to_pose_safe_mapping.xml'),
        'nav2_bringup/launch/navigation_launch.py': (
            "use_composition = LaunchConfiguration('use_composition')",
            "condition=IfCondition(PythonExpression(['not ', use_composition]))",
            'condition=IfCondition(use_composition)',
            "executable='collision_monitor'",
            "plugin='nav2_collision_monitor::CollisionMonitor'"),
        'jdamr_cube_cartographer/config/jdamr_cube_2d_real.lua': (
            'num_subdivisions_per_laser_scan = 1',
            'provide_odom_frame = false'),
    },
    'B': {
        'jdamr_cube_navigation/launch/onboard_keepout_navigation.launch.py': (
            "'record_bag', default_value='true'", "'ionice'",
            '_shutdown_on_recorder_exit'),
        'jdamr_cube_navigation/launch/onboard_nav2_core.launch.py': (
            "executable='component_container_isolated'", "'amcl'",
            "'collision_monitor'", "executable='nav2_liveness_guard'"),
        'jdamr_cube_navigation/behavior_trees/navigate_to_pose_corridor_fail_fast.xml': (
            'NavigateCorridorForwardOnly', 'position_goal_checker'),
    },
}


def probe_bundle(root: Path) -> dict:
    """Verify copied topology tokens without starting ROS processes."""
    root = root.absolute()
    manifest = strict_json_load(root / 'bundle_manifest.json')
    checks = []
    for profile, files in REQUIRED_TOKENS.items():
        records = manifest['profiles'][profile]
        by_source = {item['source_relative_path']: item for item in records}
        for source_relative, tokens in files.items():
            record = by_source.get(source_relative)
            if record is None:
                checks.append({'profile': profile, 'source': source_relative,
                               'status': 'FAIL', 'missing_tokens': list(tokens)})
                continue
            path = root / 'payload' / record['relative_path']
            content = path.read_text(encoding='utf-8')
            missing = [token for token in tokens if token not in content]
            checks.append({'profile': profile, 'source': source_relative,
                           'status': 'PASS' if not missing else 'FAIL',
                           'missing_tokens': missing})
    return {
        'schema_version': 1,
        'claim_scope': 'OFFLINE_STATIC_TOPOLOGY_PROBE_NO_ROS_NO_MOTION',
        'bundle_id': manifest['bundle_id'],
        'checks': checks,
        'status': ('PASS' if checks and all(
            item['status'] == 'PASS' for item in checks) else 'FAIL'),
        'physical_execution': 'NOT_RUN',
        'pi_execution': 'NOT_RUN',
    }


def main() -> int:
    """Print a deterministic read-only probe document."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle-root', type=Path, required=True)
    args = parser.parse_args()
    result = probe_bundle(args.bundle_root)
    print(canonical_json_bytes(result).decode(), end='')
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
