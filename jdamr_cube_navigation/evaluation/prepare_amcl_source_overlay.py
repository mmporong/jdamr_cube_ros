#!/usr/bin/env python3
"""Create a hash-locked, evaluation-only nav2_amcl source overlay."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shutil
import tarfile
import tempfile
import urllib.request

from amcl_fault_contract import AMCL_BASELINE_FILES
from amcl_fault_contract import canonical_json_bytes
from amcl_fault_contract import exact_regular_file
from amcl_fault_contract import OVERLAY_CHANGED_FILES
from amcl_fault_contract import PF_C_PATH
from amcl_fault_contract import sha256_file
from amcl_fault_contract import UPSTREAM_AMCL_TREE_FILE_COUNT
from amcl_fault_contract import UPSTREAM_AMCL_TREE_SHA256
from amcl_fault_contract import UPSTREAM_ARCHIVE_SHA256
from amcl_fault_contract import UPSTREAM_ARCHIVE_URL
from amcl_fault_contract import UPSTREAM_COMMIT
from amcl_fault_contract import UPSTREAM_LICENSE_SHA256


def _replace_once(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding='utf-8')
    if text.count(old) != 1:
        raise ValueError(f'patch anchor count is not one: {path}: {old[:40]}')
    path.write_text(text.replace(old, new), encoding='utf-8')


def verify_baseline(source_root: Path) -> dict:
    """Verify the exact upstream files used by the deterministic patch."""
    records = {}
    for relative, expected_sha in AMCL_BASELINE_FILES.items():
        records[relative] = exact_regular_file(
            (source_root / relative).resolve(), expected_sha)
    license_record = exact_regular_file(
        (source_root / 'LICENSE').resolve(), UPSTREAM_LICENSE_SHA256)
    return {'files': records, 'license': license_record}


def canonical_source_tree(root: Path) -> dict:
    """Hash the exact regular-file inventory below one source subtree."""
    if root.is_symlink() or not root.is_dir() or root != root.resolve():
        raise ValueError('source tree root must be a canonical directory')
    records = []
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise ValueError(f'source tree contains a symlink: {path}')
        if path.is_dir():
            continue
        if not path.is_file():
            raise ValueError(f'source tree contains a special file: {path}')
        records.append({
            'relative_path': path.relative_to(root).as_posix(),
            'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path),
        })
    if not records:
        raise ValueError('source tree is empty')
    return {
        'file_count': len(records),
        'files': records,
        'tree_sha256': hashlib.sha256(
            canonical_json_bytes(records)).hexdigest(),
    }


def validate_source_tree_delta(baseline_tree: dict,
                               overlay_tree: dict) -> list[str]:
    """Require equal inventories and exactly four modified files."""
    validate_tree_manifest(baseline_tree)
    validate_tree_manifest(overlay_tree)
    before = {item['relative_path']: item for item in baseline_tree['files']}
    after = {item['relative_path']: item for item in overlay_tree['files']}
    if set(before) != set(after):
        raise ValueError('overlay full-tree inventory drift')
    changed = {
        f'nav2_amcl/{path}' for path in before
        if before[path] != after[path]}
    if changed != OVERLAY_CHANGED_FILES:
        raise ValueError(f'overlay changed-file drift: {sorted(changed)}')
    pf_relative = PF_C_PATH.removeprefix('nav2_amcl/')
    if after[pf_relative]['sha256'] != AMCL_BASELINE_FILES[PF_C_PATH]:
        raise ValueError('pf.c must remain byte-identical to upstream')
    return sorted(changed)


def validate_tree_manifest(tree: dict) -> None:
    """Validate the canonical tree schema, ordering and payload digest."""
    if set(tree) != {'file_count', 'files', 'tree_sha256'}:
        raise ValueError('source tree manifest schema drift')
    records = tree['files']
    if type(tree['file_count']) is not int or tree['file_count'] != len(records):
        raise ValueError('source tree file count drift')
    paths = []
    for record in records:
        if set(record) != {'relative_path', 'size_bytes', 'sha256'}:
            raise ValueError('source tree file record schema drift')
        relative = record['relative_path']
        if (not isinstance(relative, str) or not relative or
                Path(relative).is_absolute() or '..' in Path(relative).parts):
            raise ValueError('source tree relative path invalid')
        if type(record['size_bytes']) is not int or record['size_bytes'] < 0:
            raise ValueError('source tree file size invalid')
        digest = record['sha256']
        if (not isinstance(digest, str) or len(digest) != 64 or
                any(char not in '0123456789abcdef' for char in digest)):
            raise ValueError('source tree file digest invalid')
        paths.append(relative)
    if paths != sorted(set(paths)):
        raise ValueError('source tree paths are not unique and canonical')
    expected = hashlib.sha256(canonical_json_bytes(records)).hexdigest()
    if tree['tree_sha256'] != expected:
        raise ValueError('source tree payload digest mismatch')


def apply_deterministic_patch(source_root: Path) -> None:
    """Apply the startup-only random seed surface and PDF corrective."""
    cpp = source_root / 'nav2_amcl/src/amcl_node.cpp'
    hpp = source_root / 'nav2_amcl/include/nav2_amcl/amcl_node.hpp'
    pdf_c = source_root / 'nav2_amcl/src/pf/pf_pdf.c'
    pdf_hpp = source_root / 'nav2_amcl/include/nav2_amcl/pf/pf_pdf.hpp'
    _replace_once(
        cpp, '#include <algorithm>\n#include <memory>',
        '#include <algorithm>\n#include <cstdlib>\n#include <limits>\n'
        '#include <memory>\n#include <stdexcept>')
    _replace_once(
        cpp, '  add_parameter(\n    "alpha1",',
        '  add_parameter("random_seed", rclcpp::ParameterValue(-1));\n\n'
        '  add_parameter(\n    "alpha1",')
    _replace_once(
        cpp, '  get_parameter("alpha1", alpha1_);',
        '  get_parameter("random_seed", random_seed_);\n'
        '  if (random_seed_ < -1 ||\n'
        '    random_seed_ > static_cast<int64_t>(\n'
        '      std::numeric_limits<unsigned int>::max()))\n'
        '  {\n'
        '    throw std::invalid_argument("random_seed must be -1 or uint32");\n'
        '  }\n\n'
        '  get_parameter("alpha1", alpha1_);')
    first_anchor = (
        '  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];\n\n'
        '  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);')
    _replace_once(
        cpp, first_anchor,
        '  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];\n\n'
        '  seedParticleFilter();\n'
        '  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);')
    second_anchor = (
        '  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);\n\n'
        '  pf_init_ = false;')
    _replace_once(
        cpp, second_anchor,
        '  seedParticleFilter();\n'
        '  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);\n\n'
        '  pf_init_ = false;')
    insert_anchor = '\nvoid\nAmclNode::initParameters()\n'
    helper = (
        '\nvoid\nAmclNode::seedParticleFilter()\n{\n'
        '  if (random_seed_ == -1) {\n    return;\n  }\n'
        '  const auto seed = static_cast<unsigned int>(random_seed_);\n'
        '  pf_pdf_set_seed(seed);\n  srand48(seed);\n}\n\n'
        'void\nAmclNode::initParameters()\n')
    _replace_once(cpp, insert_anchor, helper)
    _replace_once(
        hpp, '  void initParameters();\n  double alpha1_;',
        '  void initParameters();\n  void seedParticleFilter();\n'
        '  int64_t random_seed_{-1};\n  double alpha1_;')
    _replace_once(
        pdf_hpp, '// Create a gaussian pdf\n',
        '// Set the next Gaussian allocation seed for evaluation replay.\n'
        'void pf_pdf_set_seed(unsigned int seed);\n\n'
        '// Create a gaussian pdf\n')
    _replace_once(
        pdf_c, 'static unsigned int pf_pdf_seed;\n',
        'static unsigned int pf_pdf_seed;\n'
        'static int pf_pdf_seed_override_pending;\n\n'
        'void pf_pdf_set_seed(unsigned int seed)\n{\n'
        '  pf_pdf_seed = seed;\n'
        '  pf_pdf_seed_override_pending = 1;\n'
        '  srand48(seed);\n}\n')
    _replace_once(
        pdf_c, '  srand48(++pf_pdf_seed);',
        '  if (pf_pdf_seed_override_pending) {\n'
        '    srand48(pf_pdf_seed);\n'
        '    pf_pdf_seed_override_pending = 0;\n'
        '  } else {\n    srand48(++pf_pdf_seed);\n  }')


def validate_overlay_hashes(baseline_hashes: dict,
                            after_hashes: dict) -> list[str]:
    """Reject any source delta outside the four-file evaluation overlay."""
    if set(baseline_hashes) != set(after_hashes):
        raise ValueError('overlay file inventory drift')
    changed = {
        path for path in baseline_hashes
        if baseline_hashes[path] != after_hashes[path]}
    if changed != OVERLAY_CHANGED_FILES:
        raise ValueError(f'overlay changed-file drift: {sorted(changed)}')
    if after_hashes[PF_C_PATH] != AMCL_BASELINE_FILES[PF_C_PATH]:
        raise ValueError('pf.c must remain byte-identical to upstream')
    return sorted(changed)


def _download_archive(destination: Path) -> None:
    if not UPSTREAM_ARCHIVE_URL.startswith('https://'):
        raise ValueError('upstream archive must use HTTPS')
    with urllib.request.urlopen(UPSTREAM_ARCHIVE_URL) as response:
        destination.write_bytes(response.read())


def prepare_overlay(output_root: Path, archive_path: Path | None = None) -> dict:
    """Verify, patch and atomically publish one source overlay."""
    output_root = output_root.expanduser()
    if not output_root.is_absolute():
        raise ValueError('output root must be absolute')
    if output_root.exists() or output_root.is_symlink():
        raise FileExistsError(output_root)
    output_root.parent.mkdir(parents=True, exist_ok=True)
    if output_root.parent != output_root.parent.resolve():
        raise ValueError('output parent must be canonical')
    with tempfile.TemporaryDirectory(
            prefix='.g002-overlay-', dir=output_root.parent) as temp_name:
        temp_root = Path(temp_name)
        archive = temp_root / 'navigation2.tar.gz'
        if archive_path is None:
            _download_archive(archive)
        else:
            archive_path = archive_path.expanduser()
            if (not archive_path.is_absolute() or archive_path.is_symlink() or
                    archive_path != archive_path.resolve()):
                raise ValueError('archive path must be canonical')
            shutil.copyfile(archive_path, archive)
        archive_record = exact_regular_file(
            archive.resolve(), UPSTREAM_ARCHIVE_SHA256)
        with tarfile.open(archive, 'r:gz') as bundle:
            members = bundle.getmembers()
            prefix = f'navigation2-{UPSTREAM_COMMIT}/'
            root_name = prefix.rstrip('/')
            if any(member.name != root_name and
                   not member.name.startswith(prefix) for member in members):
                raise ValueError('archive contains an unexpected root')
            bundle.extractall(temp_root, filter='data')
        source = temp_root / f'navigation2-{UPSTREAM_COMMIT}'
        before = verify_baseline(source)
        baseline_tree = canonical_source_tree(source / 'nav2_amcl')
        if (baseline_tree['file_count'] != UPSTREAM_AMCL_TREE_FILE_COUNT or
                baseline_tree['tree_sha256'] != UPSTREAM_AMCL_TREE_SHA256):
            raise ValueError('upstream nav2_amcl full-tree identity drift')
        baseline_hashes = {
            relative: sha256_file(source / relative)
            for relative in AMCL_BASELINE_FILES}
        apply_deterministic_patch(source)
        overlay_tree = canonical_source_tree(source / 'nav2_amcl')
        after_hashes = {
            relative: sha256_file(source / relative)
            for relative in AMCL_BASELINE_FILES}
        validate_overlay_hashes(baseline_hashes, after_hashes)
        changed = validate_source_tree_delta(baseline_tree, overlay_tree)
        overlay = temp_root / 'overlay'
        shutil.copytree(source / 'nav2_amcl', overlay / 'nav2_amcl')
        shutil.copy2(source / 'LICENSE', overlay / 'LICENSE')
        if canonical_source_tree(overlay / 'nav2_amcl') != overlay_tree:
            raise RuntimeError('copied nav2_amcl tree identity mismatch')
        lock = {
            'schema_version': 1,
            'scope': 'evaluation-only; not a production install',
            'upstream': {'commit': UPSTREAM_COMMIT,
                         'archive_url': UPSTREAM_ARCHIVE_URL,
                         'archive_size_bytes': archive_record['size_bytes'],
                         'archive_sha256': archive_record['sha256'],
                         'license': {
                             'relative_path': 'LICENSE',
                             'size_bytes': before['license']['size_bytes'],
                             'sha256': before['license']['sha256']}},
            'baseline_tree': baseline_tree,
            'overlay_tree': overlay_tree,
            'changed_files': changed,
            'unchanged_pf_c': True,
            'patch_classification': (
                'upstream random_seed surface plus local pf_pdf corrective'),
        }
        (overlay / 'UPSTREAM_LOCK.json').write_bytes(canonical_json_bytes(lock))
        overlay.rename(output_root)
    return lock


def main() -> int:
    """Create and verify the evaluation-only AMCL source overlay."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--archive', type=Path)
    args = parser.parse_args()
    prepare_overlay(args.output_root, args.archive)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
