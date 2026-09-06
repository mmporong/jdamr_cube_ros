#!/usr/bin/env python3
"""Run one deterministic Gazebo SLAM experiment and compute ATE/RPE."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import threading
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory

from evaluate_sim_slam import analyse


SCHEMA_METADATA_KEYWORDS = {
    '$schema', '$id', '$defs', 'title', 'description'}
SCHEMA_ASSERTION_KEYWORDS = {
    '$ref', 'type', 'required', 'additionalProperties', 'properties',
    'enum', 'const', 'pattern', 'minLength', 'minimum', 'minItems',
    'maxItems', 'items', 'minProperties', 'format'}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def valid_run_label(label: str) -> bool:
    """Return whether a run label is safe as one output directory name."""
    return re.fullmatch(r'[a-z0-9][a-z0-9_-]{5,79}', label) is not None


def backend_command(
        backend: str,
        slam_params: Path | None,
        cartographer_config_dir: Path | None = None,
        cartographer_config: str = 'jdamr_cube_2d_real.lua') -> list[str]:
    """Return the selected mapping backend command without RViz."""
    if backend == 'cartographer':
        config_dir = (
            cartographer_config_dir
            if cartographer_config_dir is not None
            else Path(get_package_share_directory(
                'jdamr_cube_cartographer')) / 'config')
        return [
            'ros2', 'run', 'cartographer_ros', 'cartographer_node',
            '-configuration_directory', str(config_dir),
            '-configuration_basename', cartographer_config,
            '--ros-args', '-p', 'use_sim_time:=true',
        ]
    if backend == 'slam_toolbox':
        if slam_params is None:
            raise ValueError('slam_toolbox requires --slam-params')
        return [
            'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            'use_sim_time:=true', f'slam_params_file:={slam_params}',
        ]
    raise ValueError(f'unsupported backend: {backend}')


def resolve_cartographer_config(
        requested: str | Path, installed_config_dir: Path
        ) -> tuple[Path, str]:
    """Resolve an installed basename or an absolute evaluation config."""
    requested_path = Path(requested)
    if requested_path.is_absolute():
        config_path = requested_path.resolve()
    else:
        if requested_path.name != str(requested_path):
            raise ValueError(
                'relative Cartographer config must be a basename')
        config_path = (installed_config_dir / requested_path).resolve()
    if not config_path.is_file():
        raise ValueError('Cartographer config does not exist')
    return config_path.parent, config_path.name


def _start(command: list[str], log_path: Path,
           environment: dict[str, str]):
    log_stream = log_path.open('wb')
    process = subprocess.Popen(
        command,
        env=environment,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        start_new_session=True)
    return process, log_stream


def process_group_members(process_group: int) -> list[int]:
    """Return all current /proc members of one launched process group."""
    members = []
    for stat_path in Path('/proc').glob('[0-9]*/stat'):
        try:
            raw = stat_path.read_text(encoding='utf-8')
            fields = raw[raw.rfind(')') + 2:].split()
            if int(fields[2]) == process_group:
                members.append(int(stat_path.parent.name))
        except (OSError, ValueError, IndexError):
            continue
    return sorted(members)


def _stop(process: subprocess.Popen | None, timeout_s: float = 12.0):
    if process is None:
        return None
    returncode = process.poll()
    if returncode is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            returncode = process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                returncode = process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                returncode = process.wait(timeout=5.0)
    if not process_group_members(process.pid):
        return returncode
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return returncode
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return returncode
        time.sleep(0.1)
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return returncode
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if not process_group_members(process.pid):
            return returncode
        time.sleep(0.1)
    return 125


def _wait_for_topics(
        required: set[str],
        environment: dict[str, str],
        *,
        timeout_s: float = 40.0) -> None:
    deadline = time.monotonic() + timeout_s
    last_topics: set[str] = set()
    while time.monotonic() < deadline:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            env=environment,
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False)
        if result.returncode == 0:
            last_topics = set(result.stdout.splitlines())
            if required <= last_topics:
                return
        time.sleep(0.5)
    missing = sorted(required - last_topics)
    raise RuntimeError(f'simulation topics did not appear: {missing}')


def _one_mcap(bag_dir: Path) -> Path:
    mcaps = sorted(bag_dir.glob('*.mcap'))
    if len(mcaps) != 1:
        raise RuntimeError(
            f'expected one finalized MCAP, found {len(mcaps)}')
    metadata = bag_dir / 'metadata.yaml'
    if not metadata.is_file() or metadata.stat().st_size == 0:
        raise RuntimeError('rosbag metadata is missing or empty')
    return mcaps[0]


class ResourceSampler:
    """Sample CPU time and resident memory for launched process trees."""

    def __init__(self, processes: dict[str, subprocess.Popen | None],
                 interval_s: float = 0.5):
        """Bind the sampler to mutable launched-process slots."""
        self._processes = processes
        self._interval_s = interval_s
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.samples: list[dict[str, Any]] = []
        self._previous: dict[str, tuple[float, float]] = {}

    def start(self) -> None:
        """Start background sampling."""
        self._thread.start()

    def stop(self) -> None:
        """Stop sampling and wait for the background thread to finish."""
        self._stop_event.set()
        self._thread.join(timeout=max(2.0, self._interval_s * 3.0))

    @staticmethod
    def _tree_totals(root_pid: int) -> tuple[float, int, int]:
        ticks_per_s = os.sysconf('SC_CLK_TCK')
        page_size = os.sysconf('SC_PAGE_SIZE')
        cpu_s = 0.0
        rss_bytes = 0
        count = 0
        for stat_path in Path('/proc').glob('[0-9]*/stat'):
            try:
                raw = stat_path.read_text(encoding='utf-8')
                fields = raw[raw.rfind(')') + 2:].split()
                process_group = int(fields[2])
                if process_group != root_pid:
                    continue
                cpu_s += (int(fields[11]) + int(fields[12])) / ticks_per_s
                rss_bytes += int(fields[21]) * page_size
                count += 1
            except (OSError, ValueError, IndexError):
                continue
        return cpu_s, rss_bytes, count

    def _sample(self) -> None:
        now = time.monotonic()
        for label, launched in list(self._processes.items()):
            if launched is None or launched.poll() is not None:
                continue
            cpu_s, rss_bytes, process_count = self._tree_totals(launched.pid)
            previous = self._previous.get(label)
            cpu_pct = None
            if previous is not None and now > previous[0]:
                cpu_pct = max(
                    0.0, (cpu_s - previous[1]) / (now - previous[0]) * 100.0)
            self._previous[label] = (now, cpu_s)
            self.samples.append({
                'monotonic_s': now,
                'label': label,
                'cpu_pct_one_core': cpu_pct,
                'rss_mb': rss_bytes / (1024.0 * 1024.0),
                'process_count': process_count,
            })

    def _run(self) -> None:
        while not self._stop_event.wait(self._interval_s):
            self._sample()


def _percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = round((len(ordered) - 1) * quantile)
    return ordered[index]


def summarize_resources(samples: list[dict[str, Any]]) -> dict[str, Any]:
    """Summarize measured per-process-tree CPU and RSS samples."""
    labels = sorted({sample['label'] for sample in samples})
    by_label = {}
    for label in labels:
        rows = [sample for sample in samples if sample['label'] == label]
        cpu = [row['cpu_pct_one_core'] for row in rows
               if row['cpu_pct_one_core'] is not None]
        rss = [row['rss_mb'] for row in rows]
        by_label[label] = {
            'samples': len(rows),
            'cpu_samples': len(cpu),
            'cpu_mean_pct_one_core': statistics.fmean(cpu) if cpu else None,
            'cpu_p90_pct_one_core': _percentile(cpu, 0.9),
            'cpu_peak_pct_one_core': max(cpu) if cpu else None,
            'rss_peak_mb': max(rss) if rss else None,
        }
    return {
        'sampling_interval_s': 0.5,
        'scope': (
            'launched process tree; CPU percent may exceed 100 on multicore'),
        'by_process_group': by_label,
    }


def _save_cartographer_state(
        output: Path, environment: dict[str, str],
        log_path: Path) -> int:
    request = (
        "{filename: '" + str(output.resolve())
        + "', include_unfinished_submaps: true}")
    with log_path.open('wb') as stream:
        result = subprocess.run(
            ['ros2', 'service', 'call', '/write_state',
             'cartographer_ros_msgs/srv/WriteState', request],
            env=environment, stdout=stream, stderr=subprocess.STDOUT,
            timeout=30.0, check=False)
    if (result.returncode == 0
            and (not output.is_file() or output.stat().st_size == 0)):
        return 2
    return result.returncode


def _render_cartographer_map(
        state_path: Path, run_dir: Path,
        environment: dict[str, str]) -> tuple[int, list[dict[str, Any]]]:
    map_stem = run_dir / 'map'
    with (run_dir / 'map_render.log').open('wb') as stream:
        result = subprocess.run([
            'ros2', 'run', 'cartographer_ros',
            'cartographer_pbstream_to_ros_map',
            f'-pbstream_filename={state_path.resolve()}',
            f'-map_filestem={map_stem.resolve()}',
            '-resolution=0.05',
        ], env=environment, stdout=stream, stderr=subprocess.STDOUT,
            timeout=60.0, check=False)
    artifacts = []
    for path in (map_stem.with_suffix('.yaml'), map_stem.with_suffix('.pgm')):
        if path.is_file() and path.stat().st_size > 0:
            artifacts.append({
                'path': str(path.resolve()),
                'sha256': _sha256(path),
                'bytes': path.stat().st_size,
            })
    if result.returncode == 0 and len(artifacts) != 2:
        return 2, artifacts
    return result.returncode, artifacts


def _git_commit(repository: Path) -> str:
    result = subprocess.run(
        ['git', 'rev-parse', 'HEAD'], cwd=repository,
        capture_output=True, text=True, check=True)
    return result.stdout.strip()


def _workspace_fingerprint(repository: Path) -> tuple[bool, str]:
    scopes = (
        'jdamr_cube_navigation', 'jdamr_cube_gazebo',
        'jdamr_cube_description', 'jdamr_cube_cartographer')
    diff = subprocess.run(
        ['git', 'diff', '--binary', 'HEAD', '--', *scopes],
        cwd=repository, capture_output=True, check=True).stdout
    untracked_output = subprocess.run(
        ['git', 'ls-files', '--others', '--exclude-standard', '--', *scopes],
        cwd=repository, capture_output=True, text=True, check=True).stdout
    digest = hashlib.sha256(diff)
    for relative in sorted(untracked_output.splitlines()):
        path = repository / relative
        digest.update(relative.encode('utf-8') + b'\0')
        if path.is_file():
            digest.update(path.read_bytes())
    return bool(diff or untracked_output), digest.hexdigest()


def _schema_type_matches(value: Any, expected: str) -> bool:
    checks = {
        'object': lambda item: isinstance(item, dict),
        'array': lambda item: isinstance(item, list),
        'string': lambda item: isinstance(item, str),
        'integer': lambda item: (
            isinstance(item, int) and not isinstance(item, bool)),
        'number': lambda item: (
            isinstance(item, (int, float)) and not isinstance(item, bool)),
        'boolean': lambda item: isinstance(item, bool),
        'null': lambda item: item is None,
    }
    return checks[expected](value)


def lint_json_schema(schema: dict[str, Any], path: str = '$schema') -> None:
    """Fail when any reachable schema node uses an unsupported keyword."""
    unknown = (
        set(schema) - SCHEMA_METADATA_KEYWORDS - SCHEMA_ASSERTION_KEYWORDS)
    if unknown:
        raise ValueError(
            f'{path} schema uses unsupported keywords: {sorted(unknown)}')
    for container in ('$defs', 'properties'):
        for name, child in schema.get(container, {}).items():
            lint_json_schema(child, f'{path}.{container}.{name}')
    items = schema.get('items')
    if isinstance(items, dict):
        lint_json_schema(items, f'{path}.items')
    additional = schema.get('additionalProperties')
    if isinstance(additional, dict):
        lint_json_schema(additional, f'{path}.additionalProperties')


def validate_json_schema(
        value: Any, schema: dict[str, Any], root: dict[str, Any],
        path: str = '$', *, _schema_linted: bool = False) -> None:
    """Validate the JSON-Schema subset used by the experiment contract."""
    import re

    if not _schema_linted:
        lint_json_schema(root)
        if schema is not root:
            lint_json_schema(schema)

    if '$ref' in schema:
        target: Any = root
        for token in schema['$ref'].removeprefix('#/').split('/'):
            target = target[token]
        validate_json_schema(
            value, target, root, path, _schema_linted=True)
        return
    expected = schema.get('type')
    if expected is not None:
        alternatives = expected if isinstance(expected, list) else [expected]
        if not any(_schema_type_matches(value, item) for item in alternatives):
            raise ValueError(f'{path} has invalid type')
    if 'const' in schema and value != schema['const']:
        raise ValueError(f'{path} does not match const')
    if 'enum' in schema and value not in schema['enum']:
        raise ValueError(f'{path} is outside enum')
    if isinstance(value, dict):
        missing = set(schema.get('required', ())) - set(value)
        if missing:
            raise ValueError(
                f'{path} missing required keys: {sorted(missing)}')
        properties = schema.get('properties', {})
        if schema.get('additionalProperties') is False:
            extra = set(value) - set(properties)
            if extra:
                raise ValueError(f'{path} has extra keys: {sorted(extra)}')
        if len(value) < schema.get('minProperties', 0):
            raise ValueError(f'{path} has too few properties')
        additional = schema.get('additionalProperties')
        for key, item in value.items():
            child = properties.get(key)
            if child is None and isinstance(additional, dict):
                child = additional
            if child is not None:
                validate_json_schema(
                    item, child, root, f'{path}.{key}', _schema_linted=True)
    elif isinstance(value, list):
        if len(value) < schema.get('minItems', 0):
            raise ValueError(f'{path} has too few items')
        if 'maxItems' in schema and len(value) > schema['maxItems']:
            raise ValueError(f'{path} has too many items')
        if 'items' in schema:
            for index, item in enumerate(value):
                validate_json_schema(
                    item, schema['items'], root, f'{path}[{index}]',
                    _schema_linted=True)
    elif isinstance(value, str):
        if len(value) < schema.get('minLength', 0):
            raise ValueError(f'{path} is too short')
        if 'pattern' in schema and re.search(schema['pattern'], value) is None:
            raise ValueError(f'{path} does not match pattern')
        if 'format' in schema:
            if schema['format'] != 'date-time':
                raise ValueError(f'{path} uses unsupported format')
            try:
                parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
            except ValueError as error:
                raise ValueError(f'{path} is not ISO8601 date-time') from error
            if parsed.tzinfo is None:
                raise ValueError(f'{path} date-time lacks timezone')
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        if 'minimum' in schema and value < schema['minimum']:
            raise ValueError(f'{path} is below minimum')


def classify_experiment_outcome(
        *, valid: bool, completed: bool,
        execution_ok: bool) -> tuple[str, str]:
    """Separate performance failure from invalid or missing evidence."""
    if not valid or not execution_ok:
        return 'INVALID', 'process exit or metric evidence validity failed'
    if not completed:
        return 'FAIL', 'ground-truth completion gate failed'
    return 'PASS', 'finite metrics and ground-truth completion gates passed'


def build_experiment_manifest(
        *, run_id: str, captured_at: str, repository: Path,
        bag: Path, metrics_path: Path, metrics: dict[str, Any],
        args: argparse.Namespace, backend_config_path: Path,
        fault_document: dict[str, Any] | None,
        artifact_paths: list[Path], execution_ok: bool) -> dict[str, Any]:
    """Build the repository experiment schema from measured run artifacts."""
    completion = metrics.get('completion') or {}
    valid = bool(metrics.get('validity', {}).get('valid'))
    completed = bool(completion.get('completed'))
    status, terminal_reason = classify_experiment_outcome(
        valid=valid, completed=completed, execution_ok=execution_ok)
    workspace_dirty, workspace_diff_sha256 = _workspace_fingerprint(repository)
    manifest = {
        'schema_version': 1,
        'run_id': run_id,
        'phase': 'phase4b',
        'purpose': 'Cartographer one-factor simulation robustness evaluation',
        'environment': 'simulation',
        'captured_at': captured_at,
        'source': {
            'dataset_id': run_id,
            'bag_uri': str(bag.resolve()),
            'bag_sha256': _sha256(bag),
        },
        'software': {
            'git_sha': _git_commit(repository),
            'workspace_dirty': workspace_dirty,
            'workspace_diff_sha256': workspace_diff_sha256,
            'ros_distro': 'jazzy',
            'packages': {
                'jdamr_cube_navigation': 'workspace',
                'jdamr_cube_gazebo': 'workspace',
                ('cartographer_ros' if args.backend == 'cartographer'
                 else 'slam_toolbox'): 'jazzy',
            },
        },
        'clock': {
            'mode': 'simulation',
            'source': '/clock',
            'verified': True,
            'offset_uncertainty_ms': 0.0,
        },
        'sensors': [{
            'name': 'gpu_lidar',
            'topic': '/scan',
            'frame_id': 'laser_link',
            'timestamp_source': 'simulation_clock',
            'calibration_id': 'sim_urdf_sha256:' + _sha256(args.profile_urdf),
        }],
        'tf_authority': {
            'map_to_odom_publishers': [
                'cartographer_node' if args.backend == 'cartographer'
                else 'slam_toolbox'],
            'odom_to_base_publishers': [
                'sim_fault_injector' if fault_document else 'ros_gz_bridge'],
        },
        'config': {
            'backend': args.backend,
            'config_sha256': _sha256(backend_config_path),
            'map_id': None,
        },
        'motion': {
            'initial_pose': {
                'x': args.spawn_x_m, 'y': args.spawn_y_m, 'yaw': 0.0},
            'max_linear_mps': 0.35 if args.route == 'corridor' else 0.2,
            'max_angular_rps': 0.5 if args.route == 'corridor' else 0.65,
            'schedule_id': (
                f'{args.route}_{args.corridor_distance_m:g}m'),
        },
        'randomization': {
            'seed': args.seed,
            'theta': (fault_document or {}).get('values', {}),
        },
        'outcome': {
            'status': status,
            'terminal_reason': terminal_reason,
            'contact_count': None,
            'invalid_goal_count': None,
        },
        'artifacts': [
            {'kind': path.suffix.lstrip('.') or 'artifact',
             'uri': str(path.resolve()), 'sha256': _sha256(path)}
            for path in artifact_paths if path.is_file()
        ],
    }
    schema_path = Path(__file__).with_name('experiment_manifest.schema.json')
    schema = json.loads(schema_path.read_text(encoding='utf-8'))
    validate_json_schema(manifest, schema, schema)
    return manifest


def main(argv: list[str] | None = None) -> int:
    """Run one isolated SLAM experiment and persist its evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('cartographer', 'slam_toolbox'),
                        required=True)
    parser.add_argument('--profile-urdf', type=Path, required=True)
    parser.add_argument('--profile-label', required=True)
    parser.add_argument('--run-id')
    parser.add_argument('--bridge-config', type=Path)
    parser.add_argument('--fault-profile', type=Path)
    parser.add_argument('--slam-params', type=Path)
    parser.add_argument('--cartographer-config',
                        default='jdamr_cube_2d_real.lua')
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--out-root', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--route', choices=('square', 'corridor'),
                        default='square')
    parser.add_argument('--spawn-x', dest='spawn_x_m', type=float,
                        default=0.0)
    parser.add_argument('--spawn-y', dest='spawn_y_m', type=float,
                        default=0.0)
    parser.add_argument('--corridor-distance-m', type=float, default=14.0)
    args = parser.parse_args(argv)
    for name, path in (
            ('--profile-urdf', args.profile_urdf),
            ('--world', args.world)):
        if not path.is_file():
            parser.error(f'{name} must be an existing file')
    if ((args.bridge_config is None) != (args.fault_profile is None)):
        parser.error(
            '--bridge-config and --fault-profile must be used together')
    if args.bridge_config is not None and not args.bridge_config.is_file():
        parser.error('--bridge-config must be an existing file')
    if args.fault_profile is not None and not args.fault_profile.is_file():
        parser.error('--fault-profile must be an existing file')
    if args.slam_params is not None and not args.slam_params.is_file():
        parser.error('--slam-params must be an existing file')
    if args.backend == 'slam_toolbox' and args.slam_params is None:
        parser.error('slam_toolbox requires --slam-params')

    run_label = (
        args.run_id
        or f'{args.profile_label}__{args.backend}__seed_{args.seed}')
    if not valid_run_label(run_label):
        parser.error(
            '--run-id/profile-derived label must match '
            '[a-z0-9][a-z0-9_-]{5,79}')
    run_dir = args.out_root / run_label
    if run_dir.exists():
        parser.error(f'run output already exists: {run_dir}')
    run_dir.mkdir(parents=True)
    captured_at = datetime.now(timezone.utc).isoformat()
    bag_dir = run_dir / 'bag'
    environment = os.environ.copy()
    environment.update({
        'ROS_DOMAIN_ID': '199',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    })
    gazebo_share = Path(get_package_share_directory('jdamr_cube_gazebo'))
    gazebo_command = [
        'ros2', 'launch', 'jdamr_cube_gazebo', 'gazebo.launch.py',
        'gui:=false', f'seed:={args.seed}',
        'enable_image_bridges:=false',
        f'world:={args.world.resolve()}',
        f'urdf_file:={args.profile_urdf.resolve()}',
        f'x_pose:={args.spawn_x_m}', f'y_pose:={args.spawn_y_m}',
    ]
    if args.bridge_config is not None:
        gazebo_command.append(
            f'bridge_config:={args.bridge_config.resolve()}')
    installed_cartographer_config_dir = Path(get_package_share_directory(
        'jdamr_cube_cartographer')) / 'config'
    cartographer_config_path = None
    cartographer_config_dir = installed_cartographer_config_dir
    cartographer_config_name = args.cartographer_config
    if args.backend == 'cartographer':
        try:
            cartographer_config_dir, cartographer_config_name = (
                resolve_cartographer_config(
                    args.cartographer_config,
                    installed_cartographer_config_dir))
        except ValueError as error:
            parser.error(str(error))
        cartographer_config_path = (
            cartographer_config_dir / cartographer_config_name)
    selected_backend_command = backend_command(
        args.backend,
        args.slam_params.resolve() if args.slam_params else None,
        cartographer_config_dir,
        cartographer_config_name)
    recorder_command = [
        'ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_dir),
        '--topics', '/tf', '/odom', '/ground_truth_pose', '/scan', '/clock',
    ]
    fault_document = None
    fault_command = None
    if args.fault_profile is not None:
        fault_document = json.loads(
            args.fault_profile.read_text(encoding='utf-8'))
        fault_values = fault_document.get('values', fault_document)
        fault_command = [
            'ros2', 'run', 'jdamr_cube_navigation', 'sim_fault_injector',
            '--profile', json.dumps(fault_values, separators=(',', ':')),
            '--seed', str(args.seed),
            '--stats', str((run_dir / 'fault_stats.json').resolve()),
            '--ros-args', '-p', 'use_sim_time:=true',
        ]
    route_command = [
        'ros2', 'run', 'jdamr_cube_navigation', 'sim_slam_route',
        '--route', args.route,
        '--distance-m', str(args.corridor_distance_m),
        '--ros-args', '-p', 'use_sim_time:=true',
    ]
    if args.route == 'corridor':
        ros_index = route_command.index('--ros-args')
        route_command[ros_index:ros_index] = [
            '--linear-mps', '0.35', '--angular-radps', '0.5']
    commands = {
        'gazebo': gazebo_command,
        'fault_injector': fault_command,
        'backend': selected_backend_command,
        'recorder': recorder_command,
        'route': route_command,
    }
    processes: dict[str, subprocess.Popen | None] = {
        'gazebo': None,
        'fault_injector': None,
        'backend': None,
        'recorder': None,
    }
    streams = []
    exit_codes: dict[str, int | None] = {}
    teardown_remaining: dict[str, list[int]] = {}
    status = 'failed'
    failure = None
    map_artifact = None
    map_images: list[dict[str, Any]] = []
    mcap = None
    metrics = None
    sampler = ResourceSampler(processes)
    try:
        sampler.start()
        processes['gazebo'], stream = _start(
            gazebo_command, run_dir / 'gazebo.log', environment)
        streams.append(stream)
        required_topics = {'/ground_truth_pose', '/odom', '/scan'}
        if fault_command is not None:
            required_topics = {
                '/ground_truth_pose', '/sim_raw/odom', '/sim_raw/scan',
                '/sim_raw/tf',
            }
        _wait_for_topics(required_topics, environment)
        if fault_command is not None:
            processes['fault_injector'], stream = _start(
                fault_command, run_dir / 'fault_injector.log', environment)
            streams.append(stream)
            _wait_for_topics({'/odom', '/scan'}, environment)
        processes['backend'], stream = _start(
            selected_backend_command, run_dir / 'backend.log', environment)
        streams.append(stream)
        time.sleep(3.0)
        processes['recorder'], stream = _start(
            recorder_command, run_dir / 'recorder.log', environment)
        streams.append(stream)
        time.sleep(1.5)
        with (run_dir / 'route.log').open('wb') as route_log:
            route_result = subprocess.run(
                route_command,
                env=environment,
                stdout=route_log,
                stderr=subprocess.STDOUT,
                timeout=150.0,
                check=False)
        exit_codes['route'] = route_result.returncode
        if route_result.returncode != 0:
            raise RuntimeError(
                f'route exited with {route_result.returncode}')
        time.sleep(3.0)
        if args.backend == 'cartographer':
            state_path = run_dir / 'map_state.pbstream'
            exit_codes['map_state'] = _save_cartographer_state(
                state_path, environment, run_dir / 'map_state.log')
            if exit_codes['map_state'] != 0:
                raise RuntimeError(
                    'Cartographer map state could not be saved')
            map_artifact = {
                'path': str(state_path.resolve()),
                'sha256': _sha256(state_path),
                'bytes': state_path.stat().st_size,
            }
            exit_codes['map_render'], map_images = _render_cartographer_map(
                state_path, run_dir, environment)
            if exit_codes['map_render'] != 0:
                raise RuntimeError(
                    'Cartographer YAML/PGM map could not be rendered')
        exit_codes['recorder'] = _stop(processes['recorder'])
        teardown_remaining['recorder'] = process_group_members(
            processes['recorder'].pid)
        processes['recorder'] = None
        if exit_codes['recorder'] != 0:
            raise RuntimeError(
                f'recorder exited with {exit_codes["recorder"]}')
        mcap = _one_mcap(bag_dir)
        sampler.stop()
        commanded_path_m = (
            2.0 * args.corridor_distance_m
            if args.route == 'corridor' else 3.2)
        metrics = analyse(
            mcap, args.backend, commanded_path_m,
            args.corridor_distance_m if args.route == 'corridor' else None)
        metrics['resources'] = summarize_resources(sampler.samples)
        (run_dir / 'metrics.json').write_text(
            json.dumps(metrics, ensure_ascii=False, indent=2),
            encoding='utf-8')
        status = (
            'complete'
            if metrics['validity']['valid']
            else 'invalid')
    except Exception as error:
        failure = f'{type(error).__name__}: {error}'
        raise
    finally:
        for name in ('recorder', 'backend', 'fault_injector', 'gazebo'):
            if processes[name] is not None:
                exit_codes[name] = _stop(processes[name])
                teardown_remaining[name] = process_group_members(
                    processes[name].pid)
                processes[name] = None
        for stream in streams:
            stream.close()
        sampler.stop()
        samples_path = run_dir / 'resource_samples.jsonl'
        samples_path.write_text(
            ''.join(json.dumps(sample, ensure_ascii=False) + '\n'
                    for sample in sampler.samples),
            encoding='utf-8')
        fault_stats_path = run_dir / 'fault_stats.json'
        process_exit_ok = all(code == 0 for code in exit_codes.values())
        if status == 'complete' and not process_exit_ok:
            status = 'invalid'
            failure = 'one or more launched processes exited non-zero'
        manifest: dict[str, Any] = {
            'schema_version': 1,
            'status': status,
            'failure': failure,
            'run_label': run_label,
            'backend': args.backend,
            'seed': args.seed,
            'route': args.route,
            'spawn_xy_m': [args.spawn_x_m, args.spawn_y_m],
            'corridor_distance_m': args.corridor_distance_m,
            'ros_domain_id': 199,
            'discovery_range': 'LOCALHOST',
            'profile': {
                'label': args.profile_label,
                'urdf': str(args.profile_urdf.resolve()),
                'sha256': _sha256(args.profile_urdf),
            },
            'fault_profile': (
                {
                    'path': str(args.fault_profile.resolve()),
                    'sha256': _sha256(args.fault_profile),
                    'document': fault_document,
                }
                if args.fault_profile is not None else None),
            'bridge_config': (
                {
                    'path': str(args.bridge_config.resolve()),
                    'sha256': _sha256(args.bridge_config),
                }
                if args.bridge_config is not None else None),
            'world': {
                'path': str(args.world.resolve()),
                'sha256': _sha256(args.world),
            },
            'slam_params': (
                {
                    'path': str(args.slam_params.resolve()),
                    'sha256': _sha256(args.slam_params),
                }
                if args.slam_params is not None else None),
            'cartographer_config': (
                {
                    'path': str(cartographer_config_path.resolve()),
                    'sha256': _sha256(cartographer_config_path),
                }
                if args.backend == 'cartographer' else None),
            'commands': commands,
            'exit_codes': exit_codes,
            'teardown_remaining_processes': teardown_remaining,
            'outcome_status': (
                metrics.get('completion', {}).get('completed')
                and metrics.get('validity', {}).get('valid')
                if metrics is not None else False),
            'map_artifact': map_artifact,
            'map_images': map_images,
            'fault_realization': (
                {
                    'path': str(fault_stats_path.resolve()),
                    'sha256': _sha256(fault_stats_path),
                    'observed': json.loads(
                        fault_stats_path.read_text(encoding='utf-8')),
                }
                if fault_stats_path.is_file() else None),
            'resource_samples': {
                'path': str(samples_path.resolve()),
                'sha256': _sha256(samples_path),
                'samples': len(sampler.samples),
            },
            'gazebo_share': str(gazebo_share),
        }
        execution_manifest_path = run_dir / 'execution_manifest.json'
        execution_manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding='utf-8')
        if (status in ('complete', 'invalid')
                and mcap is not None and metrics is not None):
            metrics_path = run_dir / 'metrics.json'
            artifact_paths = [
                mcap, bag_dir / 'metadata.yaml', metrics_path, samples_path,
                run_dir / 'map_state.pbstream', run_dir / 'map.yaml',
                run_dir / 'map.pgm',
            ]
            if fault_stats_path.is_file():
                artifact_paths.append(fault_stats_path)
            experiment_manifest = build_experiment_manifest(
                run_id=run_label, captured_at=captured_at,
                repository=Path(__file__).resolve().parents[2], bag=mcap,
                metrics_path=metrics_path, metrics=metrics, args=args,
                backend_config_path=(
                    cartographer_config_path
                    if args.backend == 'cartographer'
                    else args.slam_params),
                fault_document=fault_document,
                artifact_paths=artifact_paths,
                execution_ok=process_exit_ok)
            (run_dir / 'run_manifest.json').write_text(
                json.dumps(experiment_manifest, ensure_ascii=False, indent=2),
                encoding='utf-8')
    print(json.dumps({
        'status': status,
        'run_dir': str(run_dir),
    }, ensure_ascii=False))
    return 0 if status == 'complete' else 2


if __name__ == '__main__':
    raise SystemExit(main())
