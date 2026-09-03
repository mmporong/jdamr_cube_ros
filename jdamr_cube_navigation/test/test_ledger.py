"""Regression tests for the portfolio evidence ledger."""

# The ledger is the only thing standing between a portfolio claim and a
# memory of what happened, so its gates must not pass a run the evidence
# does not support, and must not fail a run for behaving correctly.

from pathlib import Path
import sys

import pytest


sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

pytest.importorskip('mcap', reason='evaluation deps are installed separately')

from ledger import (  # noqa: E402,I100
    build_map_record,
    evaluate_gates,
    home_relative,
    REQUIRED_DECLARED_TOPICS,
    REQUIRED_SENSING_TOPICS,
)


def _report(**overrides):
    integrity = {
        'chunk_count': 45,
        'chunks_with_crc': 45,
        'data_section_crc': 12345,
        'summary_crc': 6789,
        'message_index_count': 45,
        'chunk_index_count': 45,
    }
    integrity.update(overrides)
    return {'integrity': integrity}


def _topics(**counts):
    return [{'name': name, 'messages': count}
            for name, count in counts.items()]


def _full_topics(**overrides):
    counts = {topic: 100 for topic in REQUIRED_SENSING_TOPICS}
    counts.update({topic: 10 for topic in REQUIRED_DECLARED_TOPICS})
    counts.update(overrides)
    return _topics(**counts)


def test_a_clean_sample_passes_every_automatic_gate():
    """Keep a good run from being blocked by the ledger itself."""
    gates = evaluate_gates(_report(), _full_topics(), 0)

    assert set(gates.values()) == {'PASS'}


def test_a_static_soak_with_no_motion_still_passes_evidence():
    """Zero /cmd_vel is the expected result of a non-driving soak."""
    gates = evaluate_gates(_report(), _full_topics(**{'/cmd_vel': 0}), 0)

    assert gates['required_evidence_present'] == 'PASS'
    assert 'missing_topics' not in gates


def test_an_undeclared_command_topic_is_a_real_gap():
    """A recorder that never subscribed leaves no safety evidence at all."""
    topics = [record for record in _full_topics()
              if record['name'] != '/cmd_vel']
    gates = evaluate_gates(_report(), topics, 0)

    assert gates['required_evidence_present'] == 'FAIL'
    assert gates['missing_topics'] == ['/cmd_vel']


def test_an_empty_sensing_topic_fails():
    """Offline SLAM cannot be re-derived from a bag with no scans."""
    gates = evaluate_gates(_report(), _full_topics(**{'/scan': 0}), 0)

    assert gates['required_evidence_present'] == 'FAIL'
    assert gates['missing_topics'] == ['/scan']


def test_unmeasured_recorder_loss_is_unknown_not_pass():
    """Never let an unmeasured gate read as a passing one."""
    assert evaluate_gates(
        _report(), _full_topics(), None)['transport_loss_zero'] == 'UNKNOWN'
    assert evaluate_gates(
        _report(), _full_topics(), 3)['transport_loss_zero'] == 'FAIL'


def test_missing_chunk_crc_fails_the_integrity_gate():
    """A bag without chunk CRCs cannot be promoted to a final sample."""
    gates = evaluate_gates(
        _report(chunks_with_crc=0), _full_topics(), 0)

    assert gates['chunk_crc'] == 'FAIL'


def test_paths_are_recorded_against_home():
    """Keep records portable between the laptop and the robot."""
    assert home_relative(Path.home() / 'maps' / 'a.yaml') == '$HOME/maps/a.yaml'


def test_map_record_requires_the_image_it_names(tmp_path):
    """Refuse to register a map whose image is not on disk."""
    map_yaml = tmp_path / 'broken.yaml'
    map_yaml.write_text(
        'image: missing.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n',
        encoding='utf-8')

    with pytest.raises(FileNotFoundError):
        build_map_record(map_yaml, state='candidate', source_run='x', note='')
