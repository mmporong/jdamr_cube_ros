"""Test immutable G003 portfolio media generation helpers."""

import importlib.util
import json
from pathlib import Path

import numpy as np

import pytest


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = (ROOT / 'jdamr_cube_navigation' / 'evaluation'
               / 'render_sim_nav_obstacle_portfolio.py')


def _module():
    spec = importlib.util.spec_from_file_location(
        'portfolio_media', MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _valid_manifest_tree(module, tmp_path):
    input_root = tmp_path / 'input'
    output = tmp_path / 'output'
    output.mkdir()
    source_records = []
    for relative_path in sorted(module.expected_source_paths()):
        path = input_root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative_path.encode())
        source_records.append({
            'relative_path': relative_path,
            'size_bytes': path.stat().st_size,
            'sha256': module.sha256_file(path),
        })
    generated_records = []
    for relative_path in sorted(module.expected_generated_paths()):
        path = output / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative_path.encode())
        generated_records.append({
            'relative_path': relative_path,
            'size_bytes': path.stat().st_size,
            'sha256': module.sha256_file(path),
        })
    inventories = []
    for scenario in module.SCENARIOS:
        run_id = f'{scenario}__seed_11'
        relative_path = (
            f'portfolio_media/bags/{scenario}_seed11/data.mcap')
        bag = input_root / relative_path
        bag.parent.mkdir(parents=True, exist_ok=True)
        bag.write_bytes(scenario.encode())
        inventories.append({
            'scenario': scenario,
            'seed': 11,
            'run_id': run_id,
            'relative_path': relative_path,
            'evidence_relative_path': (
                'portfolio_media/representative_evidence/'
                f'{run_id}.json'),
            'size_bytes': bag.stat().st_size,
            'sha256': module.sha256_file(bag),
        })
    manifest = {
        'generator': {'sha256': module.sha256_file(MODULE_PATH)},
        'bag_inventories': inventories,
        'source_evidence': source_records,
        'generated_files': generated_records,
    }
    (output / 'manifest.json').write_text(json.dumps(manifest))
    return input_root, output, manifest


def test_nonempty_output_is_never_overwritten(tmp_path):
    """Existing artifacts require a new versioned output directory."""
    module = _module()
    (tmp_path / 'keep.txt').write_text('keep')

    with pytest.raises(FileExistsError):
        module.require_empty_output(tmp_path)

    assert (tmp_path / 'keep.txt').read_text() == 'keep'


def test_blocking_points_excludes_unknown_and_free_cells():
    """Only Nav2 costs 253 and 254 become overlay points."""
    module = _module()
    position = type('Position', (), {'x': -1.0, 'y': -2.0})()
    origin = type('Origin', (), {'position': position})()
    metadata = type('Metadata', (), {
        'size_x': 3, 'size_y': 2, 'resolution': 0.5,
        'origin': origin})()
    costmap = type('Costmap', (), {
        'metadata': metadata,
        'data': bytes([0, 253, 255, 254, 1, 0])})()

    x_values, y_values = module.blocking_points(costmap)

    assert np.allclose(x_values, [-0.25, -0.75])
    assert np.allclose(y_values, [-1.75, -1.25])


def test_local_costmap_points_apply_eight_meter_map_transform():
    """An odom-frame costmap is transformed instead of drawn as map."""
    module = _module()
    position = type('Position', (), {'x': 0.0, 'y': 0.0})()
    origin = type('Origin', (), {'position': position})()
    metadata = type('Metadata', (), {
        'size_x': 1, 'size_y': 1, 'resolution': 1.0,
        'origin': origin})()
    costmap = type('Costmap', (), {
        'metadata': metadata, 'data': bytes([254])})()

    x_values, y_values = module.blocking_points(
        costmap, (-8.0, 0.0, 0.0))

    assert np.allclose(x_values, [-7.5])
    assert np.allclose(y_values, [0.5])


def test_exact_stamp_transform_interpolates_and_missing_tf_fails():
    """TF interpolation uses both stamp brackets and never latest."""
    module = _module()

    assert module.interpolate_map_transform([
        (1_000, -8.0, 0.0, 0.0),
        (3_000, -6.0, 0.0, 0.0),
    ], 2_000) == (-7.0, 0.0, 0.0)
    with pytest.raises(ValueError, match='missing exact-stamp TF'):
        module.interpolate_map_transform([
            (3_000, -6.0, 0.0, 0.0),
        ], 2_000)


def test_message_log_time_uses_integer_nanoseconds():
    """The installed MCAP ROS 2 reader exposes log_time_ns directly."""
    module = _module()
    message = type('Message', (), {'log_time_ns': 2_500_000_000})()

    assert module.message_log_time_s(message) == 2.5


def test_removal_obstacle_disappears_only_after_deactivate_ack():
    """Removal media reflects the recorded causal deactivation event."""
    module = _module()
    events = [{'name': 'deactivate_ack', 'elapsed_s': 4.0}]

    assert module.obstacle_is_active('detour', events, 99.0)
    assert module.obstacle_is_active('event_driven_removal', events, 3.99)
    assert not module.obstacle_is_active(
        'event_driven_removal', events, 4.0)


def test_bag_time_uses_goal_anchor_without_normalization():
    """A 3.271-second event offset remains 3.271 seconds in bag time."""
    module = _module()
    binding = {
        'executing_log_time_ns': 100_000_000_000,
        'evidence_goal_accepted_elapsed_s': 5.412,
    }

    elapsed_s = module.evidence_elapsed_at_bag_time(binding, 103.271)

    assert elapsed_s == pytest.approx(8.683)


def test_action_binding_rejects_wrong_evidence_goal_uuid(
        tmp_path, monkeypatch):
    """A bag goal UUID cannot be attached to different evidence."""
    module = _module()
    goal_uuid = bytes(range(16))

    def status(code):
        goal_id = type('GoalId', (), {'uuid': goal_uuid})()
        goal_info = type('GoalInfo', (), {'goal_id': goal_id})()
        return type('Status', (), {
            'goal_info': goal_info, 'status': code})()

    raw_start = type('Raw', (), {'log_time': 10_000_000_000})()
    raw_end = type('Raw', (), {'log_time': 12_000_000_000})()
    messages = [
        ('status', raw_start, type('Array', (), {
            'status_list': [status(2)]})()),
        ('status', raw_end, type('Array', (), {
            'status_list': [status(4)]})()),
    ]
    monkeypatch.setattr(module, '_raw_messages', lambda *args: messages)
    evidence = {
        'goal_uuid': 'f' * 32,
        'action_terminal': 'succeeded',
        'events': [
            {'name': 'goal_accepted', 'elapsed_s': 3.0},
            {'name': 'succeeded', 'elapsed_s': 5.0},
        ],
        'contract': {'controller_period_s': 0.1},
    }

    with pytest.raises(ValueError, match='goal UUID mismatch'):
        module.bind_action_status(tmp_path / 'unused.mcap', evidence)


def test_representative_identity_rejects_seed_mismatch(tmp_path):
    """Evidence and bag filenames cannot be rebound to another run."""
    module = _module()
    evidence_path = tmp_path / 'detour__seed_11.json'
    bag_path = tmp_path / 'detour_seed11' / 'data.mcap'
    evidence = {
        'run_id': 'detour__seed_11', 'scenario': 'detour', 'seed': 23}

    with pytest.raises(ValueError, match='identity mismatch'):
        module.validate_representative_identity(
            'detour', evidence_path, bag_path, evidence)


def test_manifest_verifier_detects_generated_tamper(tmp_path):
    """Generated media cannot change after its hash manifest is written."""
    module = _module()
    input_root, output, _ = _valid_manifest_tree(module, tmp_path)
    module.verify_manifest(input_root, output)

    artifact = output / 'data/scenario_seed_metrics.json'
    artifact.write_text('tampered')
    with pytest.raises(ValueError, match='manifest mismatch'):
        module.verify_manifest(input_root, output)


def test_manifest_verifier_detects_representative_bag_tamper(tmp_path):
    """Representative MCAP inputs are immutable manifest dependencies."""
    module = _module()
    input_root, output, manifest = _valid_manifest_tree(module, tmp_path)
    module.verify_manifest(input_root, output)

    bag = input_root / manifest['bag_inventories'][0]['relative_path']
    bag.write_bytes(b'tampered-bag')
    with pytest.raises(ValueError, match='manifest mismatch'):
        module.verify_manifest(input_root, output)


def test_action_result_absence_is_explicit_contract():
    """The manifest contract never invents an action-result bag message."""
    module = _module()

    assert '/navigate_to_pose/_action/status' in module.REQUIRED_TOPICS
    assert '/navigate_to_pose/_action/feedback' in module.REQUIRED_TOPICS
    assert '/navigate_to_pose/_action/result' not in module.REQUIRED_TOPICS


@pytest.mark.parametrize('relative_path', [
    'contract.json', 'assets/slam_corridor_eval.pgm'])
@pytest.mark.parametrize('mutation', ['tamper', 'delete'])
def test_manifest_verifier_rejects_direct_source_mutation(
        tmp_path, relative_path, mutation):
    """Every directly rendered contract/map input remains immutable."""
    module = _module()
    input_root, output, _ = _valid_manifest_tree(module, tmp_path)
    target = input_root / relative_path
    if mutation == 'delete':
        target.unlink()
    else:
        target.write_bytes(b'tampered')

    with pytest.raises(ValueError, match='manifest mismatch'):
        module.verify_manifest(input_root, output)


def test_manifest_verifier_rejects_missing_direct_source_records(tmp_path):
    """A shadow input cannot omit contract and map records from a manifest."""
    module = _module()
    input_root, output, manifest = _valid_manifest_tree(module, tmp_path)
    manifest['source_evidence'] = [
        item for item in manifest['source_evidence']
        if item['relative_path'] not in module.REQUIRED_DIRECT_SOURCE_PATHS]
    (output / 'manifest.json').write_text(json.dumps(manifest))

    with pytest.raises(ValueError, match='source_evidence record set mismatch'):
        module.verify_manifest(input_root, output)


@pytest.mark.parametrize('source_kind', [
    'aggregate', 'run_evidence', 'representative_evidence'])
@pytest.mark.parametrize('mutation', ['remove', 'duplicate', 'extra'])
def test_source_record_multiplicity_is_exact(
        tmp_path, source_kind, mutation):
    """Every source class rejects removed, duplicate, and extra records."""
    module = _module()
    input_root, output, manifest = _valid_manifest_tree(module, tmp_path)
    selected = {
        'aggregate': 'aggregate.json',
        'run_evidence': 'evidence/baseline__seed_11.json',
        'representative_evidence': (
            'portfolio_media/representative_evidence/'
            'baseline__seed_11.json'),
    }[source_kind]
    record = next(item for item in manifest['source_evidence']
                  if item['relative_path'] == selected)
    if mutation == 'remove':
        manifest['source_evidence'].remove(record)
    elif mutation == 'duplicate':
        manifest['source_evidence'].append(dict(record))
    else:
        manifest['source_evidence'].append({
            **record, 'relative_path': f'extra/{source_kind}.json'})
    (output / 'manifest.json').write_text(json.dumps(manifest))

    with pytest.raises(ValueError, match='source_evidence record set mismatch'):
        module.verify_manifest(input_root, output)


@pytest.mark.parametrize('mutation', ['remove', 'duplicate', 'extra'])
def test_bag_inventory_multiplicity_is_exact(tmp_path, mutation):
    """The representative bag inventory is exactly one per scenario."""
    module = _module()
    input_root, output, manifest = _valid_manifest_tree(module, tmp_path)
    if mutation == 'remove':
        manifest['bag_inventories'].pop()
    elif mutation == 'duplicate':
        manifest['bag_inventories'].append(
            dict(manifest['bag_inventories'][0]))
    else:
        manifest['bag_inventories'].append({
            **manifest['bag_inventories'][0],
            'scenario': 'extra', 'run_id': 'extra__seed_11'})
    (output / 'manifest.json').write_text(json.dumps(manifest))

    with pytest.raises(ValueError, match='bag inventory'):
        module.verify_manifest(input_root, output)


@pytest.mark.parametrize('mutation', ['remove', 'duplicate', 'extra'])
def test_generated_record_multiplicity_is_exact(tmp_path, mutation):
    """Generated-file records reject removed, duplicate, and extra paths."""
    module = _module()
    input_root, output, manifest = _valid_manifest_tree(module, tmp_path)
    record = manifest['generated_files'][0]
    if mutation == 'remove':
        manifest['generated_files'].pop(0)
    elif mutation == 'duplicate':
        manifest['generated_files'].append(dict(record))
    else:
        manifest['generated_files'].append({
            **record, 'relative_path': 'visuals/extra.png'})
    (output / 'manifest.json').write_text(json.dumps(manifest))

    with pytest.raises(ValueError, match='generated_files record set mismatch'):
        module.verify_manifest(input_root, output)


@pytest.mark.parametrize('mutation', ['missing', 'extra'])
def test_actual_generated_file_set_is_exact(tmp_path, mutation):
    """Unrecorded output additions and missing outputs both fail closed."""
    module = _module()
    input_root, output, _ = _valid_manifest_tree(module, tmp_path)
    if mutation == 'missing':
        (output / 'visuals/baseline_seed11_overlay.png').unlink()
    else:
        (output / 'visuals/extra.png').write_bytes(b'extra')

    with pytest.raises(ValueError, match='generated output file set mismatch'):
        module.verify_manifest(input_root, output)
