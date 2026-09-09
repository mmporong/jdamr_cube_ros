#!/usr/bin/env python3
"""Re-evaluate one preserved keepout run against its generated mask raster."""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import shutil
from typing import Any

from run_onboard_candidate_smoke import (
    _keepout_route_evidence, _sha256, _source_identity)

import yaml


def _load_json(path: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError(f'expected JSON object: {path}')
    return document


def _single_mcap(case_dir: Path) -> Path:
    matches = sorted((case_dir / 'bag').glob('*.mcap'))
    if len(matches) != 1:
        raise ValueError(f'expected exactly one source MCAP: {case_dir}')
    return matches[0]


def _identity_matches(identity: Any, path: Path) -> bool:
    if not isinstance(identity, dict):
        return False
    try:
        recorded_path = Path(str(identity['path'])).resolve()
    except (KeyError, OSError, TypeError, ValueError):
        return False
    return recorded_path == path.resolve() and identity.get('sha256') == _sha256(
        path)


def _summary_behavior_passed(
        original: dict[str, Any], stop_contract: dict[str, Any]) -> bool:
    """Accept only behavior evidence already preserved in the case summary."""
    scenario = original.get('scenario', {})
    same_goal = original.get('same_goal_command_evidence', {}).get(
        'evidence', {})
    episodes = same_goal.get('episodes', [])
    goal_uuid = scenario.get('goal_uuid')
    pose = scenario.get('final_world_pose_m')
    goal = stop_contract.get('goal_pose', {})
    arrived = (
        isinstance(pose, list) and len(pose) == 2
        and all(math.isfinite(value) for value in pose)
        and 'x_m' in goal and 'y_m' in goal
        and math.dist(pose, (goal['x_m'], goal['y_m'])) <= 0.5)
    capture = scenario.get('direct_scan_capture') or {}
    events = set(scenario.get('events', []))
    return all((
        original.get('detour_evidence', {}).get('status') == 'PASS',
        original.get('raw_action_status', {}).get('final_status') == 4,
        scenario.get('returncode') == 0,
        scenario.get('action_terminal') == 'succeeded',
        bool(goal_uuid),
        scenario.get('terminal_goal_uuid') == goal_uuid,
        scenario.get('goal_send_count') == 1,
        scenario.get('goal_cancel_count') == 0,
        scenario.get('stop_action_type') == 1,
        scenario.get('stop_polygon_name') == 'StopZone',
        scenario.get('resume_action_type') == 0,
        scenario.get('physical_stop_observed') is True,
        scenario.get('contact_matched_publisher_count_max', 0) > 0,
        scenario.get('contact_count') == 0,
        scenario.get('minimum_clearance_m', 0) > 0,
        scenario.get('protected_envelope_minimum_clearance_m', 0) > 0,
        scenario.get('final_cmd_vel_zero') is True,
        scenario.get('final_zero_hold_s', 0)
        >= stop_contract.get('final_zero_hold_s', math.inf),
        capture.get('zero_receive_steady_ns', 0)
        > capture.get('scan_receive_steady_ns', math.inf),
        scenario.get('activation_error') is None,
        scenario.get('harness_error') is None,
        scenario.get('same_goal_command_verdict') == 'CONFIRMED',
        same_goal.get('verdict') == 'CONFIRMED',
        same_goal.get('terminal_succeeded') is True,
        len(episodes) == 1,
        episodes[0].get('same_goal_resumed') is True,
        episodes[0].get('terminal_succeeded') is True,
        {'physical_stop', 'obstacle_deactivated', 'succeeded'} <= events,
        arrived,
    ))


def reevaluate(input_root: Path, output_root: Path) -> dict[str, Any]:
    """Write a provenance-linked result without modifying source evidence."""
    input_root = input_root.resolve()
    output_root = output_root.resolve()
    if input_root == output_root or output_root.is_relative_to(input_root):
        raise ValueError('output must be outside the input evidence tree')
    source_summary_path = input_root / 'summary.json'
    source_summary = _load_json(source_summary_path)
    results = source_summary.get('results')
    if not isinstance(results, list) or len(results) != 1:
        raise ValueError('source summary must contain exactly one result')
    case = str(results[0].get('case', ''))
    if case != 'detour_sudden_stop_resume':
        raise ValueError('only the combined keepout case can be re-evaluated')
    source_case_dir = input_root / case
    source_case_summary_path = source_case_dir / 'summary.json'
    original = _load_json(source_case_summary_path)
    if original.get('case') != case:
        raise ValueError('case summary identity does not match root summary')
    if output_root.exists() and any(output_root.iterdir()):
        raise ValueError('output root must be new or empty')

    zones_path = input_root / 'assets' / 'sim_keepout_zones.yaml'
    zones = yaml.safe_load(zones_path.read_text(encoding='utf-8'))
    enabled = [zone for zone in zones.get('zones', [])
               if zone.get('enabled', True)]
    if len(enabled) != 1:
        raise ValueError('expected exactly one enabled keepout zone')
    keepout_spec = {
        'zone_id': str(enabled[0]['id']),
        'polygon_m': enabled[0]['polygon'],
        'safety_margin_m': float(zones['safety_margin_m']),
    }
    stop_contract_path = input_root / 'assets' / 'onboard_stop_contract.json'
    stop_contract = _load_json(stop_contract_path)
    mask_yaml = input_root / 'assets' / 'sim_keepout_mask.yaml'
    mask_definition = yaml.safe_load(mask_yaml.read_text(encoding='utf-8'))
    mask_image = (mask_yaml.parent / mask_definition['image']).resolve()
    mask_report_path = input_root / 'assets' / 'sim_keepout_mask.json'
    mask_report = _load_json(mask_report_path)
    mcap = _single_mcap(source_case_dir)
    evidence = _keepout_route_evidence(
        mcap, mask_yaml, keepout_spec, stop_contract)

    scenario_path = source_case_dir / 'scenario.json'
    _load_json(scenario_path)
    behavioral_gate = _summary_behavior_passed(original, stop_contract)
    raw_action_succeeded = original.get('raw_action_status', {}).get(
        'final_status') == 4
    teardown = original.get('teardown', {})
    teardown_clean = not (
        teardown.get('remaining_process_groups')
        or teardown.get('identity_survivors'))
    recorded_hash = original.get('recording', {}).get('mcap', {}).get('sha256')
    mcap_hash_matches = recorded_hash == _sha256(mcap)
    identities = original.get('source_identity', {})
    asset_identities_match = all((
        _identity_matches(identities.get('keepout_mask'), mask_yaml),
        _identity_matches(identities.get('keepout_zones'), zones_path),
        _identity_matches(
            identities.get('scenario_contract'), stop_contract_path),
    ))
    mask_report_matches = all((
        mask_report.get('mask_sha256') == _sha256(mask_image),
        Path(str(mask_report.get('mask_yaml', ''))).resolve()
        == mask_yaml.resolve(),
        Path(str(mask_report.get('zones_yaml', ''))).resolve()
        == zones_path.resolve(),
        mask_report.get('zones') == [keepout_spec['zone_id']],
        mask_report.get('keepout_cells')
        == evidence.get('mask', {}).get('occupied_cell_count'),
        mask_report.get('safety_margin_m') == keepout_spec['safety_margin_m'],
    ))
    keepout_only_failure = all((
        source_summary.get('status') == 'FAIL',
        original.get('status') == 'FAIL',
        original.get('keepout_route_evidence', {}).get('status') == 'FAIL',
        results[0] == original,
        behavioral_gate,
    ))
    checks = {
        'asset_source_identities_match': asset_identities_match,
        'keepout_was_only_failed_gate': keepout_only_failure,
        'mask_report_matches_generated_assets': mask_report_matches,
        'original_behavior_gate': behavioral_gate,
        'original_harness_error_absent': not original.get('harness_error'),
        'original_recording_error_absent': not original.get('recording_error'),
        'original_teardown_error_absent': not original.get('teardown_error'),
        'raw_action_terminal_succeeded': raw_action_succeeded,
        'root_output_error_absent': not source_summary.get('output_error'),
        'root_result_matches_case_summary': results[0] == original,
        'source_mcap_hash_matches': mcap_hash_matches,
        'teardown_clean': teardown_clean,
    }
    other_checks_passed = all(checks.values())
    status = (
        'PASS' if other_checks_passed and evidence['status'] == 'PASS'
        else 'FAIL')

    output_root.mkdir(parents=True, exist_ok=True)
    output_assets = output_root / 'assets'
    shutil.copytree(input_root / 'assets', output_assets,
                    dirs_exist_ok=True)
    output_case_dir = output_root / case
    output_case_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_summary_path, output_root / 'source_summary.json')
    shutil.copy2(
        source_case_summary_path, output_case_dir / 'source_summary.json')
    shutil.copy2(scenario_path, output_case_dir / 'scenario.json')

    revised = copy.deepcopy(original)
    revised['original_status'] = original.get('status')
    revised['keepout_route_evidence'] = evidence
    revised['status'] = status
    revised['reevaluation'] = {
        'status': status,
        'method': 'generated_mask_raster_cells_vs_rotated_robot_footprint',
        'other_original_checks': checks,
        'other_original_checks_passed': other_checks_passed,
        'source': {
            'root_summary': _source_identity(source_summary_path),
            'case_summary': _source_identity(source_case_summary_path),
            'scenario': _source_identity(scenario_path),
            'mcap': _source_identity(mcap),
            'mask_image': _source_identity(mask_image),
            'mask_report': _source_identity(mask_report_path),
            'mask_yaml': _source_identity(mask_yaml),
            'stop_contract': _source_identity(stop_contract_path),
            'zones': _source_identity(zones_path),
        },
        'evaluator': _source_identity(Path(__file__)),
        'source_root': str(input_root),
        'output_root': str(output_root),
        'source_files_modified': False,
        'scenario_usage': 'retained_copy_only_not_status_input',
        'historical_scenario_integrity_verified': False,
        'limitations': [
            ('the source run did not preserve a pre-existing scenario hash; '
             'this re-evaluation does not assert historical scenario integrity'),
            ('behavior gating uses only fields preserved in the original case '
             'summary and does not re-run the scenario verdict'),
        ],
    }
    revised.pop('output_bytes', None)
    (output_case_dir / 'summary.json').write_text(json.dumps(
        revised, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')

    combined = copy.deepcopy(source_summary)
    combined['original_status'] = source_summary.get('status')
    combined['status'] = status
    combined['results'] = [revised]
    combined['reevaluation'] = revised['reevaluation']
    combined.pop('output_bytes', None)
    (output_root / 'summary.json').write_text(json.dumps(
        combined, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    return combined


def main() -> int:
    """Run the fail-closed offline keepout re-evaluation CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-root', required=True, type=Path)
    parser.add_argument('--output-root', required=True, type=Path)
    args = parser.parse_args()
    try:
        result = reevaluate(args.input_root, args.output_root)
    except (FileNotFoundError, OSError, TypeError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if result['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
