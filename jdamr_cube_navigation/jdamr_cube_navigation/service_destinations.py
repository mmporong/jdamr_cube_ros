"""Map-bound, taught service destinations; no ROS or motion side effects."""

from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import math
import os
from pathlib import Path
import re
import struct
import tempfile

from jdamr_cube_navigation.keepout_mask import _read_pgm
import numpy as np
import yaml


def expanded_path(value, parent=None):
    """Resolve an operator path, allowing portable HOME-based registries."""
    path = Path(os.path.expandvars(os.path.expanduser(str(value))))
    return (path if path.is_absolute() else (parent or Path.cwd()) / path).resolve()


def map_identity(path):
    """Bind metadata (origin/resolution included) and image bytes separately."""
    path = expanded_path(path)
    raw = path.read_bytes()
    metadata = yaml.safe_load(raw)
    if not isinstance(metadata, dict) or not isinstance(metadata.get('image'), str):
        raise ValueError('map YAML requires an image path')
    image_path = expanded_path(metadata['image'], path.parent)
    return {
        'yaml_path': str(path),
        'yaml_sha256': hashlib.sha256(raw).hexdigest(),
        'image_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
    }


def verify_identity(expected, path=None):
    """Check content rather than hostname-specific paths."""
    actual = map_identity(path or expected['yaml_path'])
    for key in ('yaml_sha256', 'image_sha256'):
        if actual[key] != expected[key]:
            raise ValueError(f'map/keepout {key} mismatch: {actual["yaml_path"]}')
    return actual


def grid_signature(width, height, resolution_m, origin, data, frame_id='map'):
    """Hash metric geometry and signed occupancy cells, excluding timestamps."""
    if (width <= 0 or height <= 0 or resolution_m <= 0.0 or frame_id != 'map'
            or not all(math.isfinite(value) for value in (resolution_m, *origin))):
        raise ValueError('invalid live map geometry or frame')
    cells = np.asarray(data, dtype=np.int16).reshape(-1)
    if cells.size != width * height or np.any(cells < -1) or np.any(cells > 100):
        raise ValueError('invalid live occupancy cells')
    x_m, y_m, yaw_rad = origin
    yaw_rad = (yaw_rad + math.pi) % (2 * math.pi) - math.pi
    metadata = struct.pack('<IIfddd', width, height, resolution_m,
                           round(x_m, 9), round(y_m, 9), round(yaw_rad, 9))
    return hashlib.sha256(metadata + cells.astype(np.int8).tobytes()).hexdigest()


def map_grid_signature(path):
    """Decode the project's trinary P5 maps using Nav2 occupancy semantics."""
    path = expanded_path(path)
    metadata = yaml.safe_load(path.read_text(encoding='utf-8'))
    if metadata.get('mode', 'trinary') != 'trinary':
        raise ValueError('service destinations require a trinary PGM map')
    width, height, pixels = _read_pgm(expanded_path(metadata['image'], path.parent))
    normalized = np.frombuffer(pixels, dtype=np.uint8).astype(np.float32) / 255.0
    if not metadata['negate']:
        normalized = 1.0 - normalized
    free, occupied = float(metadata['free_thresh']), float(metadata['occupied_thresh'])
    if not 0.0 <= free < occupied <= 1.0:
        raise ValueError('invalid map occupancy thresholds')
    cells = np.full(normalized.shape, -1, dtype=np.int8)
    cells[normalized >= occupied] = 100
    cells[normalized <= free] = 0
    cells = np.flipud(cells.reshape(height, width)).reshape(-1)
    return grid_signature(width, height, float(metadata['resolution']), metadata['origin'], cells)


def new_registry(map_yaml, keepout_yaml):
    """Create an empty usable registry without invented table coordinates."""
    return {
        'schema_version': 1, 'frame_id': 'map', 'robot_base_frame': 'base_link',
        'map': map_identity(map_yaml), 'keepout': map_identity(keepout_yaml),
        'home': None, 'tables': [],
    }


def _number(value, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise ValueError(f'{label} must be finite')
    return float(value)


def _identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[a-zA-Z0-9_-]+', value):
        raise ValueError('IDs must contain letters, numbers, underscores or hyphens')
    return value


def validate_registry(document, verify_files=True):
    """Reject ambiguous destinations and stale map bindings before ROS startup."""
    if (not isinstance(document, dict)
            or type(document.get('schema_version')) is not int
            or document['schema_version'] != 1
            or document.get('frame_id') != 'map'
            or document.get('robot_base_frame') != 'base_link'):
        raise ValueError('service registry requires schema 1, map and base_link')
    for name in ('map', 'keepout'):
        identity = document.get(name)
        if not isinstance(identity, dict) or not isinstance(identity.get('yaml_path'), str):
            raise ValueError(f'{name} identity missing')
        for key in ('yaml_sha256', 'image_sha256'):
            if not re.fullmatch(r'[0-9a-f]{64}', str(identity.get(key, ''))):
                raise ValueError(f'{name} {key} invalid')
        if verify_files:
            verify_identity(identity)
    tables = document.get('tables')
    if not isinstance(tables, list):
        raise ValueError('tables must be a list')
    table_ids, pose_ids = set(), set()
    for table in tables:
        if not isinstance(table, dict):
            raise ValueError('table must be a mapping')
        table_id = _identifier(table.get('table_id'))
        if table_id in table_ids:
            raise ValueError('duplicate table ID')
        table_ids.add(table_id)
        if type(table.get('enabled')) is not bool:
            raise ValueError('table enabled must be boolean')
        poses = table.get('service_poses')
        if not isinstance(poses, list) or not 1 <= len(poses) <= 2:
            raise ValueError('each table needs one primary and at most one alternate pose')
        priorities = set()
        for pose in poses:
            if not isinstance(pose, dict):
                raise ValueError('service pose must be a mapping')
            pose_id = _identifier(pose.get('id'))
            if pose_id in pose_ids:
                raise ValueError('duplicate service pose ID')
            pose_ids.add(pose_id)
            priority = pose.get('priority')
            if type(priority) is not int or priority not in (1, 2) or priority in priorities:
                raise ValueError('pose priority must be unique: 1 or 2')
            priorities.add(priority)
            for key in ('x_m', 'y_m', 'yaw_rad'):
                _number(pose.get(key), key)
            offset_m = _number(pose.get('approach_offset_m'), 'approach_offset_m')
            if not 0.2 <= offset_m <= 2.0:
                raise ValueError('approach_offset_m must be between 0.2 and 2.0')
            teaching = pose.get('teaching')
            if not isinstance(teaching, dict) or teaching.get('source') != 'map_to_base_link_tf':
                raise ValueError('pose must retain teaching provenance')
            validate_gap_measurement(
                teaching.get('table_gap_m'), teaching.get('gap_measurement_note'))
            target_gap_m = pose.get('target_front_gap_m')
            if target_gap_m is not None and _number(target_gap_m, 'target_front_gap_m') <= 0:
                raise ValueError('target_front_gap_m must be positive')
        if 1 not in priorities:
            raise ValueError('primary pose (priority 1) required')
    home = document.get('home')
    if home is not None:
        if not isinstance(home, dict):
            raise ValueError('home must be a taught pose or null')
        home_id = _identifier(home.get('id'))
        if home_id in pose_ids:
            raise ValueError('duplicate service pose ID')
        for key in ('x_m', 'y_m', 'yaw_rad'):
            _number(home.get(key), key)
        offset_m = _number(home.get('approach_offset_m'), 'approach_offset_m')
        if not 0.2 <= offset_m <= 2.0:
            raise ValueError('approach_offset_m must be between 0.2 and 2.0')
        teaching = home.get('teaching')
        if not isinstance(teaching, dict) or teaching.get('source') != 'map_to_base_link_tf':
            raise ValueError('home must retain teaching provenance')
    return document


def load_registry(path):
    """Read a registry and resolve asset paths relative to its own location."""
    path = expanded_path(path)
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if isinstance(document, dict):
        for key in ('map', 'keepout'):
            identity = document.get(key)
            if isinstance(identity, dict) and 'yaml_path' in identity:
                identity['yaml_path'] = str(expanded_path(identity['yaml_path'], path.parent))
    return validate_registry(document)


def save_registry(path, document, replace=False):
    """Publish complete YAML atomically; retain existing data on failure."""
    validate_registry(document)
    path = expanded_path(path)
    payload = yaml.safe_dump(document, sort_keys=False, allow_unicode=True)
    descriptor, temporary = tempfile.mkstemp(prefix='.service-', suffix='.yaml', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        if replace:
            os.replace(temporary, path)
        else:
            os.link(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def validate_gap_measurement(measured_front_gap_m, measurement_note):
    """Keep a manual teaching measurement separate from runtime sensing."""
    if measured_front_gap_m is None:
        if measurement_note is not None:
            raise ValueError('gap measurement note requires a measured front gap')
        return
    if _number(measured_front_gap_m, 'measured_front_gap_m') <= 0:
        raise ValueError('measured_front_gap_m must be positive')
    if not isinstance(measurement_note, str) or not measurement_note.strip():
        raise ValueError('measured front gap requires a measurement note')


def front_gap_evidence(pose):
    """Report intent and teaching evidence without claiming a new arrival measurement."""
    teaching = pose.get('teaching', {})
    return {
        'reference': 'chassis_front_to_table_nearest_surface',
        'target_m': pose.get('target_front_gap_m'),
        'teaching_measured_m': teaching.get('table_gap_m'),
        'teaching_measurement_note': teaching.get('gap_measurement_note'),
        'arrival_measured_m': None,
        'arrival_verification': 'NOT_MEASURED',
    }


def taught_pose(pose_id, actual_pose, observation, priority=1, approach_offset_m=0.5,
                measured_front_gap_m=None, gap_measurement_note=None,
                target_front_gap_m=None):
    """Preserve the stationary capture and derived approach's provenance."""
    validate_gap_measurement(measured_front_gap_m, gap_measurement_note)
    if target_front_gap_m is not None and _number(target_front_gap_m, 'target_front_gap_m') <= 0:
        raise ValueError('target_front_gap_m must be positive')
    x_m, y_m, yaw_rad = actual_pose
    return {
        'id': _identifier(pose_id), 'priority': priority,
        'x_m': float(x_m), 'y_m': float(y_m), 'yaw_rad': float(yaw_rad),
        'approach_offset_m': approach_offset_m,
        'target_front_gap_m': target_front_gap_m,
        'teaching': {
            'source': 'map_to_base_link_tf',
            'captured_at': datetime.now(timezone.utc).isoformat(),
            'observation': deepcopy(observation),
            'physical_accuracy': 'NOT_MEASURED',
            'table_gap_m': measured_front_gap_m,
            'gap_measurement_note': gap_measurement_note,
        },
    }


def add_pose(document, table_id, pose, replace=False):
    """Upsert one taught pose without disturbing other table registrations."""
    result = deepcopy(document)
    _identifier(table_id)
    table = next((item for item in result['tables'] if item['table_id'] == table_id), None)
    if table is None:
        table = {'table_id': table_id, 'enabled': True, 'service_poses': []}
        result['tables'].append(table)
    poses = table['service_poses']
    existing = next((item for item in poses if item['id'] == pose['id']), None)
    if existing is not None:
        if not replace:
            raise ValueError('pose already taught; use --replace to re-teach it')
        poses.remove(existing)
    poses.append(deepcopy(pose))
    poses.sort(key=lambda item: item['priority'])
    return validate_registry(result)


def set_home_pose(document, pose, replace=False):
    """Register one map-bound charging home without changing table poses."""
    result = deepcopy(document)
    if result.get('home') is not None and not replace:
        raise ValueError('home already taught; use --replace to re-teach it')
    result['home'] = deepcopy(pose)
    return validate_registry(result)


def home_pose(document):
    """Return the taught charging home or fail before planning any motion."""
    pose = document.get('home')
    if pose is None:
        raise ValueError('home is not taught')
    return deepcopy(pose)


def candidates(document, table_id):
    """Select the named enabled table, never a different table on failure."""
    for table in document['tables']:
        if table['table_id'] == table_id:
            if not table['enabled']:
                raise ValueError(f'table disabled: {table_id}')
            return sorted(table['service_poses'], key=lambda pose: pose['priority'])
    raise ValueError(f'unknown table: {table_id}')


def route_config(document, pose):
    """Build transit approach and final service pose using the taught yaw."""
    x_m, y_m, yaw_rad = (pose[key] for key in ('x_m', 'y_m', 'yaw_rad'))
    offset_m = pose['approach_offset_m']
    return {
        'frame_id': document['frame_id'],
        'waypoints': [
            {'id': pose['id'] + '_approach',
             'x': x_m - offset_m * math.cos(yaw_rad),
             'y': y_m - offset_m * math.sin(yaw_rad), 'yaw': yaw_rad},
            {'id': pose['id'], 'x': x_m, 'y': y_m, 'yaw': yaw_rad},
        ],
    }
