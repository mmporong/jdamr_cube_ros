"""Register runs and maps in the portfolio evidence ledger."""

# Every corridor run produced evidence that stayed only in a terminal:
# hashes were computed by hand, verdicts lived in prose, and the
# registries kept one dataset while five real runs went unrecorded.
# This tool turns a finished bag or a saved map into a registry record
# with the same gates every time, so a portfolio claim traces back to a
# file on disk instead of a memory of what happened.
#
# Nothing here promotes a run on its own.  A bag becomes
# PROTOCOL_QUALIFIED only when every automatic gate passes and the
# operator asserts the drive completed, because no file can prove the
# robot returned to its start.

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from inspect_mcap import inspect, sha256_file  # noqa: E402
import yaml  # noqa: E402


EVALUATION_ROOT = Path(__file__).resolve().parent
DATASETS_PATH = EVALUATION_ROOT / 'datasets.yaml'
MAP_REGISTRY_PATH = EVALUATION_ROOT / 'map_registry.yaml'

# Sensing and transform evidence must actually contain messages: an empty
# /scan means the offline SLAM comparison cannot be re-derived at all.
REQUIRED_SENSING_TOPICS = (
    '/scan',
    '/odom',
    '/tf',
    '/tf_static',
    '/imu/data_raw',
)

# Command and safety evidence only has to be subscribed.  A static soak whose
# whole point is that the robot never moved legitimately records zero
# /cmd_vel messages, and that must not read as a missing topic.
REQUIRED_DECLARED_TOPICS = (
    '/cmd_vel',
)

QUALIFICATIONS = ('DIAGNOSTIC_ONLY', 'PARTIAL_SUCCESS', 'PROTOCOL_QUALIFIED')


def home_relative(path: Path) -> str:
    """Render an absolute path against $HOME so records stay portable."""
    home = Path.home()
    try:
        return f'$HOME/{path.resolve().relative_to(home)}'
    except ValueError:
        return str(path.resolve())


def find_bag_file(run_dir: Path) -> Path:
    """Return the single MCAP file inside a rosbag2 output directory."""
    candidates = sorted(run_dir.glob('*.mcap'))
    if not candidates:
        raise FileNotFoundError(f'no .mcap file under {run_dir}')
    if len(candidates) > 1:
        raise ValueError(
            f'{run_dir} holds {len(candidates)} MCAP files; split bags are not '
            'registered as one sample')
    return candidates[0]


def read_bag_metadata(run_dir: Path):
    """Return the rosbag2 metadata body, or None if never written."""
    # A recorder killed with SIGKILL leaves a readable MCAP behind an
    # empty metadata.yaml.  That is evidence about the shutdown path,
    # not a reason to drop the run from the ledger.
    path = run_dir / 'metadata.yaml'
    if not path.is_file() or path.stat().st_size == 0:
        return None
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not document:
        return None
    return document['rosbag2_bagfile_information']


def topic_records(metadata, counts) -> list[dict[str, Any]]:
    """Merge declared topic metadata with counts read from the file."""
    if metadata is None:
        return [
            {'name': name, 'type': 'UNKNOWN', 'messages': count,
             'reliability': 'UNKNOWN', 'durability': 'UNKNOWN'}
            for name, count in sorted(counts.items())
        ]
    records = []
    for entry in metadata['topics_with_message_count']:
        topic = entry['topic_metadata']
        profiles = topic.get('offered_qos_profiles') or [{}]
        records.append({
            'name': topic['name'],
            'type': topic['type'],
            'messages': counts.get(topic['name'], entry['message_count']),
            'reliability': profiles[0].get('reliability', 'UNKNOWN'),
            'durability': profiles[0].get('durability', 'UNKNOWN'),
        })
    records.sort(key=lambda record: record['name'])
    return records


def inspect_or_report_damage(bag: Path) -> tuple[dict[str, Any], str]:
    """Inspect a bag, or describe why the file cannot be read at all."""
    # A truncated MCAP still belongs in the ledger.  The 2026-09-01 split
    # container trial needed SIGKILL and left a file with no footer, which is
    # itself the evidence that shutdown path was rejected.
    try:
        return inspect(bag), ''
    except Exception as error:  # noqa: BLE001 - any read failure is evidence
        report = {
            'path': str(bag.resolve()),
            'size_bytes': bag.stat().st_size,
            'sha256': sha256_file(bag),
            'message_count': 'UNKNOWN',
            'duration_ns': 'UNKNOWN',
            'topic_counts': {},
            'integrity': {
                'validate_crcs': 'UNREADABLE',
                'chunk_count': 0,
                'chunks_with_crc': 0,
                'data_section_crc': 0,
                'summary_crc': 0,
                'chunk_index_count': 0,
                'message_index_count': 0,
            },
        }
        return report, f'{type(error).__name__}: {error}'


def evaluate_gates(report: dict[str, Any],
                   topics: list[dict[str, Any]],
                   transport_loss: int | None) -> dict[str, Any]:
    """Return the automatic pass/fail gates for a candidate sample."""
    integrity = report['integrity']
    declared = {record['name'] for record in topics}
    populated = {record['name'] for record in topics
                 if record['messages'] > 0}
    missing = [topic for topic in REQUIRED_SENSING_TOPICS
               if topic not in populated]
    missing += [topic for topic in REQUIRED_DECLARED_TOPICS
                if topic not in declared]
    gates = {
        'full_message_iteration': 'PASS',
        'recorder_finalized': 'PASS',
        'chunk_crc': (
            'PASS'
            if integrity['chunk_count'] > 0
            and integrity['chunks_with_crc'] == integrity['chunk_count']
            else 'FAIL'),
        'data_section_crc': (
            'PASS' if integrity['data_section_crc'] else 'FAIL'),
        'summary_crc': 'PASS' if integrity['summary_crc'] else 'FAIL',
        'message_index': (
            'PASS' if integrity['message_index_count'] else 'FAIL'),
        'required_evidence_present': 'PASS' if not missing else 'FAIL',
    }
    if transport_loss is None:
        gates['transport_loss_zero'] = 'UNKNOWN'
    else:
        gates['transport_loss_zero'] = (
            'PASS' if transport_loss == 0 else 'FAIL')
    if missing:
        gates['missing_topics'] = missing
    return gates


def build_dataset_record(run_dir: Path, *, role: str, qualification: str,
                         transport_loss: int | None,
                         outcome: str,
                         exclusions: list[str]) -> dict[str, Any]:
    """Return one datasets.yaml record for a finished run directory."""
    bag = find_bag_file(run_dir)
    metadata = read_bag_metadata(run_dir)
    report, damage = inspect_or_report_damage(bag)
    topics = topic_records(metadata, report['topic_counts'])
    gates = evaluate_gates(report, topics, transport_loss)
    if metadata is None:
        gates['recorder_finalized'] = 'FAIL'
        gates['required_evidence_present'] = 'UNKNOWN'
    if damage:
        gates['full_message_iteration'] = 'FAIL'
        gates['read_error'] = damage
    failing = [name for name, value in gates.items()
               if value == 'FAIL']

    if qualification == 'PROTOCOL_QUALIFIED' and failing:
        raise ValueError(
            'refusing to qualify a run with failing gates: '
            + ', '.join(sorted(failing)))

    metadata_path = run_dir / 'metadata.yaml'
    record: dict[str, Any] = {
        'dataset_id': run_dir.name,
        'role': role,
        'qualification': qualification,
        'protocol_qualified': qualification == 'PROTOCOL_QUALIFIED',
        'uri': home_relative(run_dir),
        'storage': {
            'identifier': (
                'mcap' if metadata is None
                else metadata['storage_identifier']),
            'relative_file': bag.name,
            'size_bytes': report['size_bytes'],
            'sha256': report['sha256'],
            'metadata_sha256': (
                'ABSENT_RECORDER_NOT_FINALIZED'
                if metadata is None else sha256_file(metadata_path)),
        },
        'capture': {
            'started_at_unix_ns': (
                'UNKNOWN' if metadata is None
                else metadata['starting_time']['nanoseconds_since_epoch']),
            'duration_ns': (
                report['duration_ns'] if metadata is None
                else metadata['duration']['nanoseconds']),
            'message_count': (
                report['message_count'] if metadata is None
                else metadata['message_count']),
            'ros_distro': 'jazzy',
            'clock_mode': 'wall',
        },
        'outcome': outcome,
        'topics': topics,
        'integrity': {
            'inspector': 'python-mcap-1.4.0',
            'full_message_iteration': 'FAIL' if damage else 'PASS',
            'iterated_message_count': report['message_count'],
            'chunk_crc': {
                'chunks_with_crc': report['integrity']['chunks_with_crc'],
                'total_chunks': report['integrity']['chunk_count'],
            },
            'data_section_crc': report['integrity']['data_section_crc'],
            'summary_crc': report['integrity']['summary_crc'],
            'message_index': {
                'records': report['integrity']['message_index_count']},
            'chunk_index': {
                'records': report['integrity']['chunk_index_count']},
        },
        'gates': gates,
        'recorder': {
            'transport_loss':
                'UNKNOWN' if transport_loss is None else transport_loss,
        },
    }
    if exclusions:
        record['exclusion_reasons'] = exclusions
    return record


def build_map_record(map_yaml: Path, *, state: str, source_run: str,
                     note: str) -> dict[str, Any]:
    """Return one map_registry.yaml record for a saved map."""
    document = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    image = (map_yaml.parent / document['image']).resolve()
    if not image.is_file():
        raise FileNotFoundError(f'map image not found: {image}')
    origin = document['origin']
    return {
        'map_id': map_yaml.stem,
        'state': state,
        'uri': home_relative(map_yaml),
        'source_run': source_run,
        'resolution_m': float(document['resolution']),
        'origin': [float(value) for value in origin],
        'checksums': {
            'yaml_sha256': sha256_file(map_yaml),
            'image_sha256': sha256_file(image),
        },
        'image_bytes': image.stat().st_size,
        'note': note,
    }


def _load(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _write(path: Path, document: dict[str, Any]) -> None:
    path.write_text(
        yaml.safe_dump(document, allow_unicode=True, sort_keys=False,
                       default_flow_style=False, width=100),
        encoding='utf-8')


def _upsert(entries: list[dict[str, Any]], record: dict[str, Any],
            key: str) -> str:
    for index, existing in enumerate(entries):
        if existing.get(key) == record[key]:
            entries[index] = record
            return 'updated'
    entries.append(record)
    return 'added'


def command_bag(args: argparse.Namespace) -> int:
    """Register a finished run directory in datasets.yaml."""
    record = build_dataset_record(
        Path(args.run_dir).expanduser().resolve(),
        role=args.role,
        qualification=args.qualification,
        transport_loss=args.transport_loss,
        outcome=args.outcome,
        exclusions=args.exclude or [])
    if not args.apply:
        print(yaml.safe_dump(record, allow_unicode=True, sort_keys=False))
        return 0
    document = _load(DATASETS_PATH)
    action = _upsert(document['datasets'], record, 'dataset_id')
    _write(DATASETS_PATH, document)
    print(f"{action} {record['dataset_id']} ({record['qualification']})")
    return 0


def command_map(args: argparse.Namespace) -> int:
    """Register a saved map in map_registry.yaml."""
    record = build_map_record(
        Path(args.map_yaml).expanduser().resolve(),
        state=args.state,
        source_run=args.source_run,
        note=args.note)
    if not args.apply:
        print(yaml.safe_dump(record, allow_unicode=True, sort_keys=False))
        return 0
    document = _load(MAP_REGISTRY_PATH)
    action = _upsert(document['maps'], record, 'map_id')
    _write(MAP_REGISTRY_PATH, document)
    print(f"{action} {record['map_id']} ({record['state']})")
    return 0


def main(argv: list[str] | None = None) -> int:
    """Dispatch the ledger subcommands."""
    parser = argparse.ArgumentParser(
        description='Register runs and maps in the evidence ledger')
    subparsers = parser.add_subparsers(dest='command', required=True)

    bag = subparsers.add_parser('bag', help='register a run directory')
    bag.add_argument('run_dir')
    bag.add_argument('--role', required=True)
    bag.add_argument('--qualification', required=True, choices=QUALIFICATIONS)
    bag.add_argument('--outcome', required=True,
                     help='one sentence on what the robot actually did')
    bag.add_argument('--transport-loss', type=int, default=None,
                     help='messages lost by the recorder, if measured')
    bag.add_argument('--exclude', action='append',
                     help='reason this sample cannot be a final comparison')
    bag.add_argument('--apply', action='store_true')
    bag.set_defaults(func=command_bag)

    map_parser = subparsers.add_parser('map', help='register a saved map')
    map_parser.add_argument('map_yaml')
    map_parser.add_argument('--state', required=True,
                            choices=('candidate', 'published', 'deprecated'))
    map_parser.add_argument('--source-run', required=True)
    map_parser.add_argument('--note', default='')
    map_parser.add_argument('--apply', action='store_true')
    map_parser.set_defaults(func=command_map)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
