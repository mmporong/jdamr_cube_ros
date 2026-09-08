#!/usr/bin/env python3
"""Create opt-in parking parameters without launching ROS or changing defaults."""

import argparse
import copy
import hashlib
import json
from pathlib import Path

from jdamr_cube_navigation.parking import load_parking_contract
from jdamr_cube_navigation.parking import parking_controller_overrides
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def prepare(source, contract_path, output):
    """Preserve the source and exclusively create a complete Nav2 params file."""
    source = Path(source)
    output = Path(output)
    raw = source.read_bytes()
    document = yaml.safe_load(raw)
    contract = load_parking_contract(contract_path)
    overrides = parking_controller_overrides(document, contract)
    candidate = copy.deepcopy(document)
    candidate['controller_server']['ros__parameters'].update(overrides)
    payload = yaml.safe_dump(candidate, sort_keys=False)
    with output.open('x', encoding='utf-8') as stream:
        stream.write(payload)
    return {
        'params_file': str(output.resolve()),
        'source_sha256': hashlib.sha256(raw).hexdigest(),
        'contract_sha256': hashlib.sha256(
            Path(contract_path).read_bytes()).hexdigest(),
        'output_sha256': hashlib.sha256(payload.encode()).hexdigest(),
        'physical_accuracy': 'NOT_MEASURED',
    }


def main():
    """Write an explicit candidate, refusing to overwrite any existing file."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=(
        PACKAGE_ROOT / 'config' / 'nav2_params.yaml'))
    parser.add_argument('--contract', type=Path, default=(
        PACKAGE_ROOT / 'config' / 'parking_contract.yaml'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.contract, args.output),
                     ensure_ascii=False, sort_keys=True))


if __name__ == '__main__':
    main()
