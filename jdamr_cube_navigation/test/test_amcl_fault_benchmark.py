"""Regression tests for the G002 Stage A-C contracts."""

from __future__ import annotations

import hashlib
from pathlib import Path
import shutil
import sys

EVALUATION = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(EVALUATION))

import amcl_fault_contract as contract  # noqa: E402
import g002_tf_sanitizer as sanitizer  # noqa: E402
import prepare_amcl_fault_benchmark as benchmark  # noqa: E402
import prepare_amcl_source_overlay as overlay  # noqa: E402
import pytest  # noqa: E402


def test_strict_json_rejects_nonfinite_numbers(tmp_path):
    path = tmp_path / 'nonfinite.json'
    for token in ('NaN', 'Infinity', '-Infinity'):
        path.write_text('{"value":' + token + '}', encoding='utf-8')
        with pytest.raises(ValueError, match='non-finite'):
            contract.strict_json_load(path)


def _transform(parent, child, x, stamp_ns):
    from geometry_msgs.msg import TransformStamped
    value = TransformStamped()
    value.header.frame_id = parent
    value.child_frame_id = child
    value.header.stamp.sec = stamp_ns // 1_000_000_000
    value.header.stamp.nanosec = stamp_ns % 1_000_000_000
    value.transform.translation.x = x
    value.transform.rotation.w = 1.0
    return value


def _make_bag(root: Path, include_forbidden=False) -> tuple[Path, dict]:
    import rosbag2_py
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(
        uri=str(root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    definitions = [
        ('/scan', 'sensor_msgs/msg/LaserScan'),
        ('/odom', 'nav_msgs/msg/Odometry'),
        ('/tf', 'tf2_msgs/msg/TFMessage'),
        ('/tf_static', 'tf2_msgs/msg/TFMessage'),
    ]
    if include_forbidden:
        definitions.append(
            ('/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped'))
    for name, type_name in definitions:
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=name, type=type_name, serialization_format='cdr'))
    scan = LaserScan()
    scan.header.stamp.nanosec = 10
    scan.ranges = [1.0, 2.0]
    odom_early = Odometry()
    odom_early.header.stamp.nanosec = 20
    odom_late = Odometry()
    odom_late.header.stamp.nanosec = 50
    odom_late.pose.pose.position.x = 9.0
    static = TFMessage(transforms=[_transform('base_link', 'laser', 0.1, 1)])
    mixed = TFMessage(transforms=[
        _transform('map', 'odom', 1.0, 2),
        _transform('odom', 'base_link', 2.0, 2),
    ])
    writer.write('/scan', serialize_message(scan), 10)
    writer.write('/odom', serialize_message(odom_early), 20)
    writer.write('/tf_static', serialize_message(static), 30)
    writer.write('/tf', serialize_message(mixed), 40)
    writer.write('/odom', serialize_message(odom_late), 50)
    if include_forbidden:
        writer.write('/amcl_pose', serialize_message(
            PoseWithCovarianceStamped()), 60)
    writer.close()
    mcap = next(root.glob('*.mcap'))
    inventory = {name: 0 for name, _ in definitions}
    inventory.update({'/scan': 1, '/odom': 2, '/tf': 1, '/tf_static': 1})
    if include_forbidden:
        inventory['/amcl_pose'] = 1
    return mcap, inventory


def _read_output(root: Path):
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(root), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    records = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/tf':
            value = deserialize_message(data, TFMessage)
            records.append((topic, stamp, [
                (item.header.frame_id, item.child_frame_id)
                for item in value.transforms]))
        elif topic == '/odom':
            value = deserialize_message(data, Odometry)
            records.append((topic, stamp, value.pose.pose.position.x))
        else:
            records.append((topic, stamp, None))
    reader.close()
    return records


def test_sanitizer_filters_only_map_odom_and_keeps_late_odom(tmp_path):
    source = tmp_path / 'source'
    mcap, inventory = _make_bag(source, include_forbidden=True)
    output = tmp_path / 'output'
    result = sanitizer.sanitize_bag(
        source, output, contract.sha256_file(mcap), 1,
        expected_input_inventory=inventory)
    records = _read_output(output)
    assert ('/tf', 40, [('odom', 'base_link')]) in records
    assert ('/odom', 50, 9.0) in records
    assert all(record[0] in contract.PUBLISH_TOPICS for record in records)
    assert result['removed_map_to_odom_transforms'] == 1
    assert result['input_topic_inventory']['/amcl_pose'] == 1


def test_sanitizer_is_deterministic_and_rejects_hash_drift(tmp_path):
    source = tmp_path / 'source'
    mcap, _ = _make_bag(source)
    expected = contract.sha256_file(mcap)
    first = tmp_path / 'first'
    second = tmp_path / 'second'
    one = sanitizer.sanitize_bag(source, first, expected, 1)
    two = sanitizer.sanitize_bag(source, second, expected, 1)
    assert one['output'] == two['output']
    for name in ('bag_0.mcap', 'metadata.yaml', 'sanitizer_manifest.json'):
        assert (first / name).read_bytes() == (second / name).read_bytes()
    with pytest.raises(ValueError, match='hash mismatch'):
        sanitizer.sanitize_bag(
            source, tmp_path / 'bad', hashlib.sha256(b'bad').hexdigest(), 1)


def test_sanitizer_rejects_count_inventory_and_symlink(tmp_path):
    source = tmp_path / 'source'
    mcap, inventory = _make_bag(source)
    expected = contract.sha256_file(mcap)
    with pytest.raises(ValueError, match='removal count'):
        sanitizer.sanitize_bag(source, tmp_path / 'bad-count', expected, 2)
    with pytest.raises(ValueError, match='inventory drift'):
        sanitizer.sanitize_bag(
            source, tmp_path / 'bad-inventory', expected, 1,
            expected_input_inventory={**inventory, '/scan': 2})
    link = tmp_path / 'source-link'
    link.symlink_to(source, target_is_directory=True)
    with pytest.raises(ValueError, match='canonical'):
        sanitizer.sanitize_bag(link, tmp_path / 'bad-link', expected, 1)


def test_sanitized_output_rejects_corruption_and_nested_file(tmp_path):
    source = tmp_path / 'source'
    mcap, _ = _make_bag(source)
    output = tmp_path / 'output'
    sanitizer.sanitize_bag(source, output, contract.sha256_file(mcap), 1)
    mcap_output = output / 'bag_0.mcap'
    original = mcap_output.read_bytes()
    mcap_output.write_bytes(original[:-1] + bytes([original[-1] ^ 1]))
    with pytest.raises(ValueError, match='hash or size mismatch'):
        sanitizer.validate_sanitized_bag(output)

    second = tmp_path / 'second'
    sanitizer.sanitize_bag(source, second, contract.sha256_file(mcap), 1)
    (second / 'nested').mkdir()
    with pytest.raises(ValueError, match='regular files only'):
        sanitizer.validate_sanitized_bag(second)


def test_input_rejects_duplicate_or_nested_mcap(tmp_path):
    source = tmp_path / 'source'
    mcap, _ = _make_bag(source)
    duplicate = source / 'duplicate.mcap'
    shutil.copyfile(mcap, duplicate)
    with pytest.raises(ValueError, match='exactly one MCAP'):
        sanitizer.sanitize_bag(
            source, tmp_path / 'duplicate-output',
            contract.sha256_file(mcap), 1)
    duplicate.unlink()
    nested = source / 'nested'
    nested.mkdir()
    with pytest.raises(ValueError, match='regular files only'):
        sanitizer.sanitize_bag(
            source, tmp_path / 'nested-output',
            contract.sha256_file(mcap), 1)


def test_sanitizer_enforces_output_size_cap(tmp_path):
    source = tmp_path / 'source'
    mcap, _ = _make_bag(source)
    with pytest.raises(RuntimeError, match='scratch cap'):
        sanitizer.sanitize_bag(
            source, tmp_path / 'small-cap', contract.sha256_file(mcap),
            1, output_limit_bytes=1)


def test_overlay_changed_file_allowlist_and_pf_c_invariant(monkeypatch):
    baseline = dict(contract.AMCL_BASELINE_FILES)
    after = dict(baseline)
    for name in contract.OVERLAY_CHANGED_FILES:
        after[name] = 'f' * 64
    assert set(overlay.validate_overlay_hashes(baseline, after)) == (
        contract.OVERLAY_CHANGED_FILES)
    bad = dict(after)
    bad['nav2_amcl/include/nav2_amcl/pf/pf.hpp'] = 'e' * 64
    with pytest.raises(ValueError, match='changed-file drift'):
        overlay.validate_overlay_hashes(baseline, bad)
    monkeypatch.setitem(after, contract.PF_C_PATH, '0' * 64)
    with pytest.raises(ValueError, match='pf.c'):
        overlay.validate_overlay_hashes(baseline, after)


def _minimal_patch_tree(root: Path) -> None:
    files = {
        'nav2_amcl/src/amcl_node.cpp': (
            '#include <algorithm>\n#include <memory>\n'
            '  add_parameter(\n    "alpha1",\n'
            '  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];\n\n'
            '  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);\n'
            '\nvoid\nAmclNode::initParameters()\n'
            '  get_parameter("alpha1", alpha1_);\n'
            '  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);\n\n'
            '  pf_init_ = false;\n'),
        'nav2_amcl/include/nav2_amcl/amcl_node.hpp': (
            '  void initParameters();\n  double alpha1_;\n'),
        'nav2_amcl/src/pf/pf_pdf.c': (
            'static unsigned int pf_pdf_seed;\n'
            '  srand48(++pf_pdf_seed);\n'),
        'nav2_amcl/src/pf/pf.c': 'legacy particle filter\n',
        'nav2_amcl/include/nav2_amcl/pf/pf_pdf.hpp': (
            '// Create a gaussian pdf\n'),
    }
    for relative, text in files.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding='utf-8')


def test_overlay_patch_generation_is_deterministic(tmp_path):
    first = tmp_path / 'first'
    second = tmp_path / 'second'
    _minimal_patch_tree(first)
    _minimal_patch_tree(second)
    overlay.apply_deterministic_patch(first)
    overlay.apply_deterministic_patch(second)
    for relative in contract.OVERLAY_CHANGED_FILES:
        assert (first / relative).read_bytes() == (second / relative).read_bytes()
    cpp = (first / 'nav2_amcl/src/amcl_node.cpp').read_text()
    assert cpp.count('seedParticleFilter();') == 2
    assert 'add_parameter("random_seed", rclcpp::ParameterValue(-1));' in cpp
    assert 'if (random_seed_ == -1)' in cpp
    assert 'random_seed must be -1 or uint32' in cpp
    assert 'pf_pdf_set_seed(seed);' in cpp


@pytest.mark.parametrize('attack', [
    'extra', 'mutation', 'deletion', 'symlink'])
def test_full_overlay_tree_rejects_inventory_attacks(tmp_path, attack):
    baseline = tmp_path / 'baseline'
    candidate = tmp_path / 'candidate'
    _minimal_patch_tree(baseline)
    readme = baseline / 'nav2_amcl/README.md'
    readme.write_text('locked\n', encoding='utf-8')
    shutil.copytree(baseline, candidate)
    overlay.apply_deterministic_patch(candidate)
    before = overlay.canonical_source_tree(baseline / 'nav2_amcl')
    after_root = candidate / 'nav2_amcl'
    if attack == 'extra':
        (after_root / 'extra.txt').write_text('extra', encoding='utf-8')
    elif attack == 'mutation':
        (after_root / 'README.md').write_text('changed\n', encoding='utf-8')
    elif attack == 'deletion':
        (after_root / 'README.md').unlink()
    else:
        (after_root / 'link').symlink_to(after_root / 'README.md')
    with pytest.raises(ValueError):
        after = overlay.canonical_source_tree(after_root)
        overlay.validate_source_tree_delta(before, after)


def _integrated_overlay_fixture(monkeypatch, tmp_path):
    baseline_root = tmp_path / 'baseline'
    result_root = tmp_path / 'overlay'
    _minimal_patch_tree(baseline_root)
    (baseline_root / 'nav2_amcl/README.md').write_text(
        'locked\n', encoding='utf-8')
    shutil.copytree(baseline_root / 'nav2_amcl', result_root / 'nav2_amcl')
    overlay.apply_deterministic_patch(result_root)
    license_path = result_root / 'LICENSE'
    license_path.write_text('test license\n', encoding='utf-8')
    baseline_tree = overlay.canonical_source_tree(
        baseline_root / 'nav2_amcl')
    overlay_tree = overlay.canonical_source_tree(result_root / 'nav2_amcl')
    pf_sha = contract.sha256_file(
        baseline_root / 'nav2_amcl/src/pf/pf.c')
    monkeypatch.setattr(
        benchmark, 'UPSTREAM_AMCL_TREE_FILE_COUNT',
        baseline_tree['file_count'])
    monkeypatch.setattr(
        benchmark, 'UPSTREAM_AMCL_TREE_SHA256',
        baseline_tree['tree_sha256'])
    monkeypatch.setattr(
        benchmark, 'UPSTREAM_LICENSE_SHA256',
        contract.sha256_file(license_path))
    monkeypatch.setitem(contract.AMCL_BASELINE_FILES,
                        contract.PF_C_PATH, pf_sha)
    lock = {
        'schema_version': 1,
        'scope': 'evaluation-only; not a production install',
        'upstream': {
            'commit': contract.UPSTREAM_COMMIT,
            'archive_url': contract.UPSTREAM_ARCHIVE_URL,
            'archive_size_bytes': contract.UPSTREAM_ARCHIVE_SIZE_BYTES,
            'archive_sha256': contract.UPSTREAM_ARCHIVE_SHA256,
            'license': {
                'relative_path': 'LICENSE',
                'size_bytes': license_path.stat().st_size,
                'sha256': contract.sha256_file(license_path)}},
        'baseline_tree': baseline_tree,
        'overlay_tree': overlay_tree,
        'changed_files': sorted(contract.OVERLAY_CHANGED_FILES),
        'unchanged_pf_c': True,
        'patch_classification': (
            'upstream random_seed surface plus local pf_pdf corrective'),
    }
    (result_root / 'UPSTREAM_LOCK.json').write_bytes(
        contract.canonical_json_bytes(lock))
    assert benchmark._validate_overlay(result_root)['changed_files'] == sorted(
        contract.OVERLAY_CHANGED_FILES)
    return result_root


@pytest.mark.parametrize('attack', [
    'extra', 'mutation', 'deletion', 'symlink'])
def test_integrated_overlay_validation_rejects_full_tree_attacks(
        monkeypatch, tmp_path, attack):
    root = _integrated_overlay_fixture(monkeypatch, tmp_path)
    tree = root / 'nav2_amcl'
    if attack == 'extra':
        (tree / 'extra.txt').write_text('extra', encoding='utf-8')
    elif attack == 'mutation':
        (tree / 'README.md').write_text('changed', encoding='utf-8')
    elif attack == 'deletion':
        (tree / 'README.md').unlink()
    else:
        (tree / 'link').symlink_to(tree / 'README.md')
    with pytest.raises(ValueError):
        benchmark._validate_overlay(root)


@pytest.mark.parametrize(('field', 'value'), [
    ('scope', 'production install allowed'),
    ('patch_classification', 'upstream-only backport')])
def test_overlay_rejects_claim_string_tamper(
        monkeypatch, tmp_path, field, value):
    root = _integrated_overlay_fixture(monkeypatch, tmp_path)
    lock_path = root / 'UPSTREAM_LOCK.json'
    lock = contract.strict_json_load(lock_path)
    lock[field] = value
    lock_path.write_bytes(contract.canonical_json_bytes(lock))
    with pytest.raises(ValueError, match='scope|classification'):
        benchmark._validate_overlay(root)


def test_tf_parity_ignores_serialized_padding_but_rejects_pose_mutation():
    from rclpy.serialization import deserialize_message, serialize_message
    from tf2_msgs.msg import TFMessage
    message = TFMessage(transforms=[
        _transform('odom', 'base_link', 2.0, 100),
        _transform('frame_length_17', 'x', 3.0, 101)])
    semantic = [{
        'storage_ns': 200,
        'transforms': [sanitizer._tf_record(value)
                       for value in message.transforms]}]
    first = {}
    second = {}
    sanitizer._raw_stats_update(
        first, '/tf', 200, b'cdr-padding-a', None,
        include_payload=False)
    sanitizer._raw_stats_update(
        second, '/tf', 200, b'cdr-padding-b', None,
        include_payload=False)
    assert sanitizer._finish_stats(first) == sanitizer._finish_stats(second)
    serialized = bytes(serialize_message(message))
    varied = bytearray(serialized)
    varied[25:28] = b'\xaa\xbb\xcc'
    canonical = sanitizer._canonical_tf_serialization(serialized)
    assert canonical == sanitizer._canonical_tf_serialization(bytes(varied))
    round_trip = deserialize_message(canonical, TFMessage)
    assert [sanitizer._tf_record(value) for value in round_trip.transforms] == (
        semantic[0]['transforms'])
    parity = sanitizer._tf_parity_record(semantic, semantic)
    sanitizer._validate_tf_parity(parity, semantic, semantic)
    mutated = json_clone(semantic)
    mutated[0]['transforms'][0]['translation'][0] = 2.1
    with pytest.raises(ValueError, match='semantic parity'):
        sanitizer._validate_tf_parity(parity, semantic, mutated)


@pytest.mark.parametrize('attack', ['encapsulation', 'length', 'nul', 'trailing'])
def test_canonical_tf_serialization_rejects_malformed_cdr(attack):
    from rclpy.serialization import serialize_message
    from tf2_msgs.msg import TFMessage
    serialized = bytearray(serialize_message(TFMessage(transforms=[
        _transform('odom', 'base_link', 2.0, 100)])))
    if attack == 'encapsulation':
        serialized[:4] = b'\x00\x00\x00\x00'
    elif attack == 'length':
        serialized[16:20] = (2 ** 32 - 1).to_bytes(4, 'little')
    elif attack == 'nul':
        serialized[24] = 1
    else:
        serialized.extend(b'junk')
    with pytest.raises(ValueError):
        sanitizer._canonical_tf_serialization(bytes(serialized))


def json_clone(value):
    """Clone a JSON-compatible fixture without sharing nested lists."""
    import json
    return json.loads(json.dumps(value))


def test_axis_b_final_identity_and_root_escape(monkeypatch, tmp_path):
    actual = benchmark._axis_b_identity(benchmark.G004_ROOT)
    assert actual['g004_contract']['sha256'] == benchmark.G004_CONTRACT_SHA256
    swapped = tmp_path / 'swapped-g004'
    swapped.mkdir()
    shutil.copyfile(benchmark.G004_ROOT / 'contract.json',
                    swapped / 'contract.json')
    monkeypatch.setattr(benchmark, 'G004_ROOT', swapped)
    with pytest.raises(ValueError, match='escaped final root'):
        benchmark._axis_b_identity(swapped)
    (swapped / 'contract.json').write_text('{}', encoding='utf-8')
    with pytest.raises(ValueError, match='SHA-256 mismatch'):
        benchmark._axis_b_identity(swapped)


def test_axis_b_rejects_symlink_root(tmp_path):
    link = tmp_path / 'g004-link'
    link.symlink_to(benchmark.G004_ROOT, target_is_directory=True)
    with pytest.raises(ValueError, match='canonical'):
        benchmark._axis_b_identity(link)


def test_axis_b_rejects_coherent_swapped_contract(monkeypatch, tmp_path):
    original_contract = contract.strict_json_load(
        benchmark.G004_ROOT / 'contract.json')
    swapped = tmp_path / 'coherent-swap'
    assets_root = swapped / 'assets'
    assets_root.mkdir(parents=True)
    for key in ('slam_corridor_eval.yaml', 'slam_corridor_eval.pgm',
                'world', 'urdf'):
        record = original_contract['evaluation_assets'][key]
        source = Path(record['path'])
        destination = assets_root / source.name
        shutil.copyfile(source, destination)
        record['path'] = str(destination)
        record['size_bytes'] = destination.stat().st_size
        record['sha256'] = contract.sha256_file(destination)
    (swapped / 'contract.json').write_bytes(
        contract.canonical_json_bytes(original_contract))
    monkeypatch.setattr(benchmark, 'G004_ROOT', swapped)
    with pytest.raises(ValueError, match='SHA-256 mismatch'):
        benchmark._axis_b_identity(swapped)


def test_sanitizer_validates_stage_before_publish(monkeypatch, tmp_path):
    source = tmp_path / 'source'
    mcap, _ = _make_bag(source)
    output = tmp_path / 'output'
    original = sanitizer.validate_sanitized_bag

    def corrupt_stage(root):
        bag = root / 'bag_0.mcap'
        payload = bag.read_bytes()
        bag.write_bytes(payload[:-1] + bytes([payload[-1] ^ 1]))
        return original(root)

    monkeypatch.setattr(sanitizer, 'validate_sanitized_bag', corrupt_stage)
    with pytest.raises(ValueError, match='hash or size mismatch'):
        sanitizer.sanitize_bag(
            source, output, contract.sha256_file(mcap), 1)
    assert not output.exists()


def test_sanitizer_source_change_before_publish_leaves_no_output(
        monkeypatch, tmp_path):
    source = tmp_path / 'source'
    mcap, _ = _make_bag(source)
    output = tmp_path / 'output'
    expected = contract.sha256_file(mcap)
    original = sanitizer.validate_sanitized_bag

    def mutate_source(root):
        payload = mcap.read_bytes()
        mcap.write_bytes(payload[:-1] + bytes([payload[-1] ^ 1]))
        return original(root)

    monkeypatch.setattr(sanitizer, 'validate_sanitized_bag', mutate_source)
    with pytest.raises(ValueError, match='source hash or size mismatch'):
        sanitizer.sanitize_bag(source, output, expected, 1)
    assert not output.exists()


def test_sanitizer_final_validation_failure_removes_published_root(
        monkeypatch, tmp_path):
    source = tmp_path / 'source'
    mcap, _ = _make_bag(source)
    output = tmp_path / 'output'
    original = sanitizer.validate_sanitized_bag
    calls = 0

    def corrupt_after_publish(root):
        nonlocal calls
        calls += 1
        if calls == 2:
            bag = root / 'bag_0.mcap'
            payload = bag.read_bytes()
            bag.write_bytes(payload[:-1] + bytes([payload[-1] ^ 1]))
        return original(root)

    monkeypatch.setattr(
        sanitizer, 'validate_sanitized_bag', corrupt_after_publish)
    with pytest.raises(ValueError, match='hash or size mismatch'):
        sanitizer.sanitize_bag(
            source, output, contract.sha256_file(mcap), 1)
    assert calls == 2
    assert not output.exists()


@pytest.mark.parametrize('field', [
    'minimum_start_free_bytes', 'durable_limit_bytes',
    'scratch_and_source_build_limit_bytes', 'abort_free_floor_bytes'])
def test_storage_contract_is_fail_closed(field):
    limits = contract.STORAGE_LIMITS
    good = contract.validate_storage_budget(7 * contract.GIB, 1, 1)
    assert good['pass'] is True
    if field == 'minimum_start_free_bytes':
        bad = contract.validate_storage_budget(limits[field] - 1, 0, 0)
    elif field == 'durable_limit_bytes':
        bad = contract.validate_storage_budget(
            7 * contract.GIB, limits[field] + 1, 0)
    elif field == 'scratch_and_source_build_limit_bytes':
        bad = contract.validate_storage_budget(
            7 * contract.GIB, 0, limits[field] + 1)
    else:
        bad = contract.validate_storage_budget(6 * contract.GIB, 0,
                                               3 * contract.GIB)
    assert bad['pass'] is False


def test_strict_json_and_map_profile_swap_fail(tmp_path):
    duplicate = tmp_path / 'duplicate.json'
    duplicate.write_text('{"x":1,"x":2}', encoding='utf-8')
    with pytest.raises(ValueError, match='duplicate'):
        contract.strict_json_load(duplicate)
    pgm = tmp_path / 'map.pgm'
    pgm.write_bytes(b'P5\n1 1\n255\n\x00')
    profile = {
        'image': 'map.pgm', 'mode': 'trinary', 'resolution': 0.05,
        'origin': [0.0, 0.0, 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196}
    yaml_path = tmp_path / 'map.yaml'
    import yaml
    yaml_path.write_text(yaml.safe_dump(profile), encoding='utf-8')
    yaml_sha = contract.sha256_file(yaml_path)
    pgm_sha = contract.sha256_file(pgm)
    assert contract.map_identity(yaml_path, yaml_sha, pgm_sha)[
        'resolution_m_per_cell'] == 0.05
    pgm.write_bytes(b'changed')
    with pytest.raises(ValueError, match='SHA-256 mismatch'):
        contract.map_identity(yaml_path, yaml_sha, pgm_sha)
