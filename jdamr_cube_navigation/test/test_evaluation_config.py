"""Validate the offline SLAM evaluation contracts."""

import hashlib
from importlib.metadata import version
import json
from pathlib import Path

from jdamr_cube_navigation.onboard_recording import RECORDED_TOPICS
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EVALUATION_ROOT = PACKAGE_ROOT / 'evaluation'


def load_yaml(name):
    """Load an evaluation YAML document."""
    with (EVALUATION_ROOT / name).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_experiment_manifest_schema_is_strict():
    """Keep the manifest contract versioned and closed to unknown fields."""
    with (EVALUATION_ROOT / 'experiment_manifest.schema.json').open(
        encoding='utf-8'
    ) as stream:
        schema = json.load(stream)

    assert schema['$schema'] == 'https://json-schema.org/draft/2020-12/schema'
    assert schema['additionalProperties'] is False
    assert 'run_id' in schema['required']
    map_to_odom = schema['properties']['tf_authority']['properties'][
        'map_to_odom_publishers'
    ]
    assert map_to_odom['maxItems'] == 1


def test_diagnostic_dataset_matches_inspected_bag():
    """Freeze the known diagnostic bag identity and message counts."""
    dataset = load_yaml('datasets.yaml')['datasets'][0]

    assert dataset['dataset_id'] == 'g4_userloop_reset_20260824T175151'
    assert dataset['qualification'] == 'DIAGNOSTIC_ONLY'
    assert dataset['protocol_qualified'] is False
    assert dataset['storage']['sha256'] == (
        '9525afb5d693e63c9ff07541e761aca6f196b69374634d714d49142028cea6d6'
    )
    assert dataset['capture']['message_count'] == 63955
    topics = {topic['name']: topic for topic in dataset['topics']}
    assert topics['/scan']['messages'] == 1883
    assert dataset['integrity']['chunk_crc']['total_chunks'] == 39
    assert dataset['integrity']['chunk_crc']['chunks_with_crc'] == 0
    assert dataset['integrity']['chunk_index']['records'] == 39
    assert dataset['integrity']['message_index']['records'] == 182


def test_writer_options_preserve_integrity_records():
    """Require CRC and indexes for future protocol-qualified captures."""
    options = load_yaml('mcap_writer_options.yaml')

    assert options['noChunkCRC'] is False
    assert options['enableDataCRC'] is True
    assert options['noSummaryCRC'] is False
    assert options['noChunking'] is False
    assert options['noMessageIndex'] is False
    assert options['noSummary'] is False


def test_mcap_inspector_dependency_is_pinned_and_available():
    """Keep the offline inspector independent of an accidental user install."""
    requirements = (EVALUATION_ROOT / 'requirements.txt').read_text(
        encoding='utf-8'
    ).splitlines()

    assert requirements == [
        'mcap==1.4.0',
        'lz4==4.4.5',
        'zstandard==0.25.0',
    ]
    assert version('mcap') == '1.4.0'


def test_qos_overrides_cover_all_recorded_topics():
    """Make recording and playback QoS provenance explicit."""
    # The onboard recorder subscribes to eleven topics.  Six had no override
    # until 2026-09-03, so they fell back to depth 10 and dropped messages
    # whenever the Pi stalled.
    qos = load_yaml('qos_overrides.yaml')

    assert set(qos) >= set(RECORDED_TOPICS)
    assert qos['/tf_static']['durability'] == 'transient_local'
    assert qos['/tf_static']['depth'] == 1
    for topic, profile in qos.items():
        # /imu/data_raw is offered best-effort; an override may not upgrade it.
        assert profile['reliability'] in {'reliable', 'best_effort'}, topic
        assert profile['history'] == 'keep_last', topic


def test_map_registry_does_not_revive_deprecated_maps():
    """Keep deprecated maps terminal and require evidence for publication."""
    registry = load_yaml('map_registry.yaml')

    assert registry['state_transitions']['deprecated'] == []
    assert 'manual_review_passed' in registry['publication_gates']


def test_unverified_calibrations_cannot_enable_fusion():
    """Block fusion claims until spatial and temporal calibration is verified."""
    registry = load_yaml('calibration_registry.yaml')

    for calibration in registry['calibrations']:
        canonical = calibration['canonical_transform'].encode('utf-8')
        assert hashlib.sha256(canonical).hexdigest() == calibration[
            'canonical_sha256'
        ]
        if calibration['status'] != 'VERIFIED':
            assert calibration['fusion_allowed'] is False


def test_phase0_status_blocks_real_motion_and_tracks_so101_drift():
    """Keep static readiness distinct from hardware authorization."""
    status = load_yaml('phase0_status.yaml')

    assert status['state'] == 'ONBOARD_STATIC_READY'
    assert status['real_motion_authorized'] is False
    assert set(status['gates'].values()) == {'PASS'}
    onboard = status['verification']['onboard_static']
    assert onboard['autorun_exit_code'] == 0
    assert onboard['resource_gate'] == 'PASS'
    assert onboard['cmd_vel_messages'] == 0
    assert onboard['cmd_vel_nav_messages'] == 0
    assert status['so101_audit']['plan_review_state'] == 'ARCHITECT_APPROVE'
    assert status['so101_audit']['plan_declared_implementation'] == 'NOT_STARTED'
    assert status['so101_audit']['untracked_candidate'] == 'mobile_mission.py'


def test_setup_installs_evaluation_assets():
    """Ensure installed packages retain the offline evaluation contract."""
    setup_text = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    evaluation_glob = 'glob(' + repr('evaluation/*.*') + ')'

    assert evaluation_glob in setup_text
