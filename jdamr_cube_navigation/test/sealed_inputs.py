"""
Byte-exact repository inputs sealed with past evaluation studies.

Production files keep changing after a study is sealed (the 2026-09-15 base
moved the LiDAR mount and the 2026-09-17 mapping launch dropped
nav2_bringup). Tests of a sealed study read these snapshots instead, so they
still prove that the study's harness accepts its sealing-time repository
inputs. Each file equals ``git show <commit>:<relative path>`` and its
SHA-256 is pinned here; a changed snapshot fails before any harness code
runs. Inputs outside this repository (the Nav2 install under /opt/ros, maps
under ~/maps, sealed artifacts under ~/jdamr_artifacts) are still read live.
"""

import hashlib
from pathlib import Path
import shutil


FIXTURES = Path(__file__).resolve().parent / 'fixtures'
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]

# G005 frontier study: generator sealed in 5c84289 (2026-09-07). The hashes
# equal every production_inputs record of the sealed G005 asset manifest
# written on 2026-09-08 (g005_audit_20260908).
G005_ROOT = FIXTURES / 'sealed_g005_5c84289'
G005_INPUTS = {
    'frontier_core': (
        'jdamr_cube_navigation/jdamr_cube_navigation/frontier_core.py',
        'd988e0edc685312421d3dc204b53649d835464f0298c32756bf8fbbfe3210f26'),
    'frontier_explorer': (
        'jdamr_cube_navigation/jdamr_cube_navigation/frontier_explorer.py',
        'f0dee2f7c7f790742b381a2a22c10099b8dc2aa9c9f1d58a3816241d847120f4'),
    'cartographer_config': (
        'jdamr_cube_cartographer/config/jdamr_cube_2d.lua',
        '6c0e833c8672ac48b56390b38cfeea88acf782add6c192d1b567435a5242148e'),
    'robot_urdf': (
        'jdamr_cube_description/urdf/jdamr_cube.urdf',
        '5a19b69adda290467f8ca38e5b6f1bfac0420e310408241623f891b251084444'),
    'nav2_params': (
        'jdamr_cube_navigation/config/nav2_params.yaml',
        '99db2ca2ca6790c6f27ba2a56a55c12ac1146dec29a4913a4d73d9948a7da50c'),
}

# G006 candidate bundle: harness sealed in 84b9128 (2026-09-07). The hashes
# equal every repository profile source recorded by the sealed G006 bundle
# g006_candidate_bundle_20260907_v05_smoke.
G006_ROOT = FIXTURES / 'sealed_g006_84b9128'
G006_REPOSITORY_SOURCES = {
    'jdamr_cube_bringup/launch/real_bringup.launch.py':
        '7c639957bdfecb818c3685c369f362e1a7bbad0d73c5c8406b2476ee33814683',
    'jdamr_cube_bringup/package.xml':
        '6d4cf43551b11339e48332753840cbe2772f17425acc44f10ca36d4e89bb9579',
    'jdamr_cube_cartographer/config/jdamr_cube_2d_real.lua':
        '51d9ec30095bc32c42f0831d73719164f84579a24c105f446196e01c0a2e0b45',
    'jdamr_cube_cartographer/launch/cartographer_real.launch.py':
        '713f29c33d1ef590039332c42fa97285bc0f7a6c95f6ad9557c4e2416fec8d70',
    'jdamr_cube_cartographer/package.xml':
        '46affa470a6fbee1dddde72fae369760c9f847c5105c6d1067547870be69e681',
    'jdamr_cube_description/package.xml':
        '6ed93081a2c061ce8ec0c2985f9e9791d45b524a127f7cd1d1f3e318c932291e',
    'jdamr_cube_description/urdf/jdamr_cube.urdf':
        '5a19b69adda290467f8ca38e5b6f1bfac0420e310408241623f891b251084444',
    'jdamr_cube_navigation/behavior_trees/'
    'navigate_to_pose_corridor_fail_fast.xml':
        'cc3ea5fbb6b40350d602be8f84bcea11c68e7dcd6daefa5edaa2008fbcd64324',
    'jdamr_cube_navigation/behavior_trees/navigate_to_pose_safe_mapping.xml':
        'a828a440e36e42b033f139dc189314a7087e74cc6e07a9d9c28e7b5b5d8933d6',
    'jdamr_cube_navigation/config/nav2_params.yaml':
        '99db2ca2ca6790c6f27ba2a56a55c12ac1146dec29a4913a4d73d9948a7da50c',
    'jdamr_cube_navigation/evaluation/mcap_writer_options.yaml':
        '527cdca86c115386cc65a6b1dcdf8ae13a3d92ed739aba8f1bf0368aa19fc6c9',
    'jdamr_cube_navigation/evaluation/qos_overrides.yaml':
        'e7c940fb59baa6351bc5856bb0c203f56af182beba07305d886423b095e38890',
    'jdamr_cube_navigation/jdamr_cube_navigation/frontier_explorer.py':
        'f0dee2f7c7f790742b381a2a22c10099b8dc2aa9c9f1d58a3816241d847120f4',
    'jdamr_cube_navigation/jdamr_cube_navigation/keepout_mask.py':
        '06589fcd084d30b88ee4fe2c84de7e5c5166d33064b1690f792fadfe0505d660',
    'jdamr_cube_navigation/jdamr_cube_navigation/nav2_liveness_guard.py':
        '7d3520a9de743f1e3a5af678b58a55fada6751ce05cf36c2297a0642db7b0303',
    'jdamr_cube_navigation/jdamr_cube_navigation/onboard_recording.py':
        '1ea2f42d513709ed5c9cb1dbf990090dc4183d7d4966c6666321c02ff8c2b2be',
    'jdamr_cube_navigation/launch/autonomous_mapping.launch.py':
        'ceab0c6b275d706a91716ab94e0b13d95709a553fee834f252d26990526a5b7f',
    'jdamr_cube_navigation/launch/onboard_keepout_navigation.launch.py':
        'ccafcbac729c0b73b30fb9a25cd70af166c893495cd4c3eeaae0e7e4a2da7983',
    'jdamr_cube_navigation/launch/onboard_nav2_core.launch.py':
        '14946e92c4e5c8cc22eeaf4dc067ecb9f8325cc301f64a3643969dc70d1e409f',
    'jdamr_cube_navigation/package.xml':
        '5c74a43bc158a075960314b59d3acbbc21b18cf4ffdc0ca70179e72e4d788848',
    'jdamr_cube_navigation/setup.py':
        'c4a00d61fbd2f4a570355423be735ccaac544cae7cd52b037c50b4c64ad64079',
}


def sealed_file(root: Path, relative: str, sha256: str) -> Path:
    """Return one snapshot after proving its bytes match the pinned hash."""
    path = root / relative
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'sealed snapshot is missing: {path}')
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != sha256:
        raise ValueError(f'sealed snapshot drift: {path}')
    return path


def g005_sealed_inputs() -> dict:
    """Map G005 production-input names to their sealed snapshots."""
    return {name: sealed_file(G005_ROOT, relative, sha256)
            for name, (relative, sha256) in G005_INPUTS.items()}


def bind_g005_sealed_inputs(production_inputs: dict, monkeypatch=None) -> None:
    """Point every G005 generator production input at its sealed snapshot."""
    if set(production_inputs) != set(G005_INPUTS):
        raise ValueError('G005 production input set differs from snapshot')
    for name, path in g005_sealed_inputs().items():
        if monkeypatch is None:
            production_inputs[name] = path
        else:
            monkeypatch.setitem(production_inputs, name, path)


def g006_sealed_repository(target: Path, profile_sources: dict,
                           harness_sources: tuple) -> Path:
    """Lay out sealed G006 profile sources beside the live G006 harness."""
    expected = {relative for specifications in profile_sources.values()
                for root_name, relative in specifications
                if root_name == 'repository'}
    if expected != set(G006_REPOSITORY_SOURCES):
        raise ValueError('G006 repository source set differs from snapshot')
    if set(harness_sources) & expected:
        raise ValueError('G006 harness would overwrite a sealed source')
    for relative, sha256 in sorted(G006_REPOSITORY_SOURCES.items()):
        destination = target / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(sealed_file(G006_ROOT, relative, sha256), destination)
    for relative in harness_sources:
        destination = target / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPOSITORY_ROOT / relative, destination)
    return target
