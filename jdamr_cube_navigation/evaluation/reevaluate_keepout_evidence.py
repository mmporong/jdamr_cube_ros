#!/usr/bin/env python3
"""Re-evaluate one preserved keepout run against its generated mask raster."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import shutil
from typing import Any  # noqa: I100

from run_onboard_candidate_smoke import (
    _case_passed, _keepout_route_evidence, _sha256, _source_identity)

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


def reevaluate(input_root: Path, output_root: Path) -> dict[str, Any]:
    """Write a provenance-linked result without modifying source evidence."""
    input_root = input_root.resolve()
    output_root = output_root.resolve()
    if input_root == output_root:
        raise ValueError('input and output roots must differ')
    source_summary_path = input_root / 'summary.json'
    source_summary = _load_json(source_summary_path)
    results = source_summary.get('results')
    if not isinstance(results, list) or len(results) != 1:
        raise ValueError('source summary must contain exactly one result')
    case = str(results[0].get('case', ''))
    if not case:
        raise ValueError('source result has no case name')
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
    mcap = _single_mcap(source_case_dir)
    evidence = _keepout_route_evidence(
        mcap, mask_yaml, keepout_spec, stop_contract)

    scenario_path = source_case_dir / 'scenario.json'
    scenario = _load_json(scenario_path)
    same_goal = original.get('same_goal_command_evidence', {})
    returncode = int(original.get('scenario', {}).get('returncode', -1))
    behavioral_gate = _case_passed(
        scenario, case, returncode, same_goal,
        original.get('detour_evidence'))
    raw_action_succeeded = original.get('raw_action_status', {}).get(
        'final_status') == 4
    teardown = original.get('teardown', {})
    teardown_clean = not (
        teardown.get('remaining_process_groups')
        or teardown.get('identity_survivors'))
    recorded_hash = original.get('recording', {}).get('mcap', {}).get('sha256')
    mcap_hash_matches = recorded_hash == _sha256(mcap)
    checks = {
        'original_behavior_gate': behavioral_gate,
        'original_harness_error_absent': not original.get('harness_error'),
        'raw_action_terminal_succeeded': raw_action_succeeded,
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
            'mask_yaml': _source_identity(mask_yaml),
        },
        'evaluator': _source_identity(Path(__file__)),
        'source_root': str(input_root),
        'output_root': str(output_root),
        'source_files_modified': False,
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
