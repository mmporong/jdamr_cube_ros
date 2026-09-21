"""Behavior tests for map-bound restaurant service destinations."""

from copy import deepcopy
import math
from pathlib import Path

from jdamr_cube_navigation import service_destinations as destinations
import numpy as np
import pytest
import yaml


def test_trinary_pgm_matches_ros_grid_row_order_and_float_resolution(tmp_path):
    """Compare file pixels to a bottom-up OccupancyGrid with float32 resolution."""
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n2 2\n255\n' + bytes([0, 254, 205, 0]))
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text(yaml.safe_dump({
        'image': 'map.pgm', 'mode': 'trinary', 'resolution': 0.05,
        'origin': [1.0, -2.0, 0.3], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196}))
    actual = destinations.map_grid_signature(map_yaml)
    expected = destinations.grid_signature(
        2, 2, float(np.float32(0.05)), (1.0, -2.0, 0.3), [-1, 100, 100, 0])
    assert actual == expected
    assert actual != destinations.grid_signature(
        2, 2, 0.05, (1.1, -2.0, 0.3), [-1, 100, 100, 0])


@pytest.fixture
def map_files(tmp_path):
    """Create real map and keepout YAML/image assets."""
    paths = {}
    for name, pixels in (('map', b'0 1 2 3'), ('keepout', b'3 2 1 0')):
        image = tmp_path / f'{name}.pgm'
        image.write_bytes(b'P2\n2 2\n255\n' + pixels + b'\n')
        metadata = tmp_path / f'{name}.yaml'
        metadata.write_text(
            yaml.safe_dump({
                'image': image.name,
                'resolution': 0.05,
                'origin': [0.0, 0.0, 0.0],
            }),
            encoding='utf-8',
        )
        paths[f'{name}_yaml'] = metadata
        paths[f'{name}_image'] = image
    return paths


@pytest.fixture
def registry(map_files):
    """Return a valid registry with two independently taught tables."""
    document = destinations.new_registry(
        map_files['map_yaml'], map_files['keepout_yaml'])
    table_1_primary = destinations.taught_pose(
        'table_1_primary', (1.0, 2.0, 0.0), {'sample': 'first'})
    table_1_alternate = destinations.taught_pose(
        'table_1_alternate', (1.5, 2.5, 0.3), {'sample': 'alternate'},
        priority=2,
    )
    table_2_primary = destinations.taught_pose(
        'table_2_primary', (4.0, 5.0, 1.0), {'sample': 'second'})
    document = destinations.add_pose(document, 'table_1', table_1_primary)
    document = destinations.add_pose(document, 'table_1', table_1_alternate)
    return destinations.add_pose(document, 'table_2', table_2_primary)


def test_new_registry_binds_real_map_and_keepout_files(map_files):
    """A new registry records verifiable identities for both asset pairs."""
    document = destinations.new_registry(
        map_files['map_yaml'], map_files['keepout_yaml'])

    assert destinations.validate_registry(document) is document
    assert document['tables'] == []
    assert document['map'] == destinations.map_identity(map_files['map_yaml'])
    assert document['keepout'] == destinations.map_identity(
        map_files['keepout_yaml'])


@pytest.mark.parametrize(
    ('asset', 'part'),
    [('map', 'yaml'), ('map', 'image'),
     ('keepout', 'yaml'), ('keepout', 'image')],
)
def test_validate_registry_rejects_changed_bound_asset(
        map_files, asset, part):
    """Any map or keepout YAML/image byte change invalidates the registry."""
    document = destinations.new_registry(
        map_files['map_yaml'], map_files['keepout_yaml'])
    changed = map_files[f'{asset}_{part}']

    if part == 'yaml':
        changed.write_text(
            changed.read_text(encoding='utf-8') + 'negate: 1\n',
            encoding='utf-8',
        )
    else:
        changed.write_bytes(changed.read_bytes() + b'0\n')

    with pytest.raises(ValueError, match=f'{part}_sha256 mismatch'):
        destinations.validate_registry(document)


@pytest.mark.parametrize(
    ('field', 'value'),
    [('schema_version', True), ('schema_version', 2),
     ('frame_id', 'odom'), ('robot_base_frame', 'base_footprint')],
)
def test_validate_registry_rejects_incompatible_schema(
        registry, field, value):
    """Schema and frame contract deviations are rejected."""
    registry[field] = value

    with pytest.raises(ValueError, match='requires schema 1'):
        destinations.validate_registry(registry)


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        (lambda document: document.update(map={}), 'map identity missing'),
        (lambda document: document.update(tables={}), 'tables must be a list'),
        (lambda document: document['tables'][0].update(service_poses=[]),
         'each table needs one primary'),
        (lambda document: document['tables'][0]['service_poses'][0].update(
            teaching={}), 'pose must retain teaching provenance'),
    ],
)
def test_validate_registry_rejects_incomplete_document_structure(
        registry, mutation, message):
    """Missing identity, table, pose, or provenance structure is rejected."""
    mutation(registry)

    with pytest.raises(ValueError, match=message):
        destinations.validate_registry(registry)


def test_validate_registry_rejects_duplicate_table_id(registry):
    """A table ID may identify only one destination."""
    registry['tables'].append(deepcopy(registry['tables'][0]))

    with pytest.raises(ValueError, match='duplicate table ID'):
        destinations.validate_registry(registry)


def test_validate_registry_rejects_duplicate_pose_id(registry):
    """A service pose ID is unique across all tables."""
    registry['tables'][1]['service_poses'][0]['id'] = 'table_1_primary'

    with pytest.raises(ValueError, match='duplicate service pose ID'):
        destinations.validate_registry(registry)


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf, True, '1.0'])
def test_validate_registry_rejects_non_finite_or_non_numeric_pose_value(
        registry, value):
    """Pose coordinates accept finite real numbers but not booleans."""
    registry['tables'][0]['service_poses'][0]['x_m'] = value

    with pytest.raises(ValueError, match='x_m must be finite'):
        destinations.validate_registry(registry)


@pytest.mark.parametrize('value', [0, 3, True, 1.0])
def test_validate_registry_rejects_invalid_pose_priority(registry, value):
    """Priorities are integer values one or two only."""
    registry['tables'][0]['service_poses'][0]['priority'] = value

    with pytest.raises(ValueError, match='pose priority must be unique'):
        destinations.validate_registry(registry)


def test_validate_registry_rejects_duplicate_pose_priority(registry):
    """Primary and alternate poses cannot share a priority."""
    registry['tables'][0]['service_poses'][1]['priority'] = 1

    with pytest.raises(ValueError, match='pose priority must be unique'):
        destinations.validate_registry(registry)


@pytest.mark.parametrize('value', [0, 1, None, 'true'])
def test_validate_registry_rejects_non_boolean_enabled(registry, value):
    """A table enabled flag must be a YAML boolean."""
    registry['tables'][0]['enabled'] = value

    with pytest.raises(ValueError, match='table enabled must be boolean'):
        destinations.validate_registry(registry)


def test_registry_init_save_load_round_trip_uses_real_files(
        tmp_path, registry):
    """Initialized and taught data survives a file-backed YAML round trip."""
    output = tmp_path / 'service_destinations.yaml'

    destinations.save_registry(output, registry)
    loaded = destinations.load_registry(output)

    assert loaded == registry
    assert yaml.safe_load(output.read_text(encoding='utf-8')) == registry


def test_save_registry_does_not_overwrite_existing_file(tmp_path, registry):
    """The default save mode preserves an existing registry byte-for-byte."""
    output = tmp_path / 'service_destinations.yaml'
    output.write_text('operator-owned\n', encoding='utf-8')

    with pytest.raises(FileExistsError):
        destinations.save_registry(output, registry)

    assert output.read_text(encoding='utf-8') == 'operator-owned\n'


def test_save_registry_replace_publishes_with_atomic_rename(
        tmp_path, registry, monkeypatch):
    """Replacement publishes a complete temporary file through os.replace."""
    output = tmp_path / 'service_destinations.yaml'
    output.write_text('old\n', encoding='utf-8')
    observed = {}
    real_replace = destinations.os.replace

    def record_replace(source, target):
        observed['source'] = Path(source)
        observed['target'] = Path(target)
        observed['payload'] = Path(source).read_text(encoding='utf-8')
        real_replace(source, target)

    monkeypatch.setattr(destinations.os, 'replace', record_replace)

    destinations.save_registry(output, registry, replace=True)

    assert observed['target'] == output
    assert observed['source'].parent == output.parent
    assert yaml.safe_load(observed['payload']) == registry
    assert destinations.load_registry(output) == registry


def test_candidates_never_fall_back_to_another_table(registry):
    """Candidate selection returns poses for the requested table only."""
    selected = destinations.candidates(registry, 'table_2')

    assert [pose['id'] for pose in selected] == ['table_2_primary']


def test_candidates_returns_alternate_after_primary(registry):
    """The alternate candidate follows the primary according to priority."""
    registry['tables'][0]['service_poses'].reverse()

    selected = destinations.candidates(registry, 'table_1')

    assert [pose['priority'] for pose in selected] == [1, 2]
    assert [pose['id'] for pose in selected] == [
        'table_1_primary', 'table_1_alternate']


def test_taught_pose_preserves_capture_values_and_observation():
    """A taught pose retains coordinates, yaw, offset and capture evidence."""
    observation = {'operator': 'lim', 'samples': [1, 2]}

    pose = destinations.taught_pose(
        'table_3_primary', (3.2, -1.4, 1.2), observation,
        approach_offset_m=0.7,
    )
    observation['samples'].append(3)

    assert (pose['x_m'], pose['y_m'], pose['yaw_rad']) == (3.2, -1.4, 1.2)
    assert pose['approach_offset_m'] == 0.7
    assert pose['teaching']['observation'] == {
        'operator': 'lim', 'samples': [1, 2]}
    assert pose['teaching']['source'] == 'map_to_base_link_tf'


def test_add_pose_preserves_other_table(registry):
    """Teaching one table leaves every other table unchanged."""
    untouched = deepcopy(registry['tables'][1])
    replacement = destinations.taught_pose(
        'table_1_primary', (9.0, 8.0, 0.5), {'sample': 're-taught'})

    updated = destinations.add_pose(
        registry, 'table_1', replacement, replace=True)

    assert updated['tables'][1] == untouched
    assert registry['tables'][1] == untouched


def test_route_config_applies_offset_along_taught_yaw(registry):
    """The approach point is offset behind the service pose's taught yaw."""
    pose = destinations.taught_pose(
        'angled', (4.0, -2.0, math.pi / 2), {'sample': 'angled'},
        approach_offset_m=0.75,
    )

    route = destinations.route_config(registry, pose)

    approach, service = route['waypoints']
    assert approach['x'] == pytest.approx(4.0)
    assert approach['y'] == pytest.approx(-2.75)
    assert approach['yaw'] == pytest.approx(math.pi / 2)
    assert service == {
        'id': 'angled', 'x': 4.0, 'y': -2.0, 'yaw': math.pi / 2}


def test_teaching_gap_is_not_a_live_arrival_measurement(tmp_path, registry):
    """Persist a real teaching measurement without shifting or certifying arrival."""
    pose = destinations.taught_pose(
        'table_1_primary', (1.0, 2.0, 0.3), {},
        measured_front_gap_m=0.07, gap_measurement_note='ruler, chassis front to box face',
        target_front_gap_m=0.05)
    updated = destinations.add_pose(registry, 'table_1', pose, replace=True)
    path = tmp_path / 'destinations.yaml'
    destinations.save_registry(path, updated)
    saved = destinations.candidates(destinations.load_registry(path), 'table_1')[0]
    gap = destinations.front_gap_evidence(saved)
    assert gap['target_m'] == 0.05
    assert gap['teaching_measured_m'] == 0.07
    assert gap['arrival_measured_m'] is None
    assert gap['arrival_verification'] == 'NOT_MEASURED'
    final = destinations.route_config(updated, saved)['waypoints'][-1]
    assert (final['x'], final['y'], final['yaw']) == (1.0, 2.0, 0.3)


@pytest.mark.parametrize('value', [0, -0.1, True, math.nan, math.inf, '0.05'])
def test_invalid_teaching_gap_is_rejected(value, registry):
    """Reject invalid manual lengths both at capture and on YAML load."""
    with pytest.raises(ValueError):
        destinations.taught_pose('p', (0, 0, 0), {}, measured_front_gap_m=value,
                                 gap_measurement_note='ruler')
    registry['tables'][0]['service_poses'][0]['teaching'].update(
        table_gap_m=value, gap_measurement_note='ruler')
    with pytest.raises(ValueError):
        destinations.validate_registry(registry)


@pytest.mark.parametrize('gap,note', [(0.05, None), (0.05, ''), (0.05, '  '),
                                      (None, 'ruler')])
def test_gap_and_measurement_note_must_be_paired(gap, note):
    """A value alone or a note alone must not become measurement evidence."""
    with pytest.raises(ValueError):
        destinations.taught_pose('p', (0, 0, 0), {}, measured_front_gap_m=gap,
                                 gap_measurement_note=note)


def test_legacy_registry_has_no_assumed_gap(registry):
    """Existing taught poses remain usable without an invented clearance."""
    pose = registry['tables'][0]['service_poses'][0]
    pose.pop('target_front_gap_m')
    pose['teaching'].pop('gap_measurement_note')
    destinations.validate_registry(registry)
    gap = destinations.front_gap_evidence(pose)
    assert gap['target_m'] is None and gap['teaching_measured_m'] is None
