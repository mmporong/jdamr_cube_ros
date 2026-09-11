"""Timeline tests for the integrated restaurant replay renderer."""

import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

from render_restaurant_replay_media import (  # noqa: E402,I100,I201
    _guard_segments,
    _latest,
    _scenario_offset,
    _state_at,
)


def _summary():
    return {
        'scenario': {
            'interventions': [{'trigger_elapsed_s': 10.0}],
            'events': [{
                'name': 'traction_fault_cleared_on_protective_stop',
                'elapsed_s': 20.0,
            }],
        },
        'traction_guard': {'events': [
            {'from': 'NAVIGATING', 'to': 'PROTECTIVE_STOP', 'at_s': 50.0},
            {'from': 'PROTECTIVE_STOP', 'to': 'RELOCALIZE', 'at_s': 51.0},
        ]},
    }


def test_latest_never_uses_a_future_monitor_sample():
    """A future obstacle STOP cannot relabel an earlier traction event."""
    samples = [(5.0, 'STOP'), (6.0, 'CLEAR')]

    assert _latest(samples, 4.99) is None
    assert _latest(samples, 5.0) == samples[0]
    assert _latest(samples, 6.1) == samples[1]


def test_scenario_and_guard_clocks_align_to_mcap_time():
    """Both producer clocks map onto the recorder clock deterministically."""
    summary = _summary()
    offset_s = _scenario_offset(summary, [(12.0, 1, 'StopZone')])
    segments = _guard_segments(summary, offset_s)

    assert offset_s == 2.0
    assert segments[0]['media_s'] == 22.0
    assert segments[1]['media_s'] == 23.0
    assert _state_at(segments, 21.99) == 'NAVIGATING'
    assert _state_at(segments, 22.0) == 'PROTECTIVE_STOP'
    assert _state_at(segments, 23.0) == 'RELOCALIZE'
