"""Tests for conservative same-goal command-resume evidence."""

from pathlib import Path
import sys


EVALUATION_ROOT = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(EVALUATION_ROOT))

from same_goal_resume_evidence import summarize_same_goal_resume  # noqa: E402


GOAL_A = '01' * 16
GOAL_B = '02' * 16


def _snapshot(stamp_ns, *statuses):
    return {
        'stamp_ns': stamp_ns,
        'statuses': [
            {'goal_uuid': goal_uuid, 'status': status}
            for goal_uuid, status in statuses
        ],
        'malformed': False,
    }


def _evaluate(snapshots, commands=None, collisions=None):
    return summarize_same_goal_resume(
        ([(100, 1, 'StopZone'), (103, 0, '')]
         if collisions is None else collisions),
        ([(99, 'NONZERO'), (101, 'ZERO'), (105, 'NONZERO')]
         if commands is None else commands),
        snapshots, 90, 108, 113)


def test_confirms_same_goal_resume_and_separate_terminal_success():
    result = _evaluate([
        _snapshot(95, (GOAL_A, 2)),
        _snapshot(102, (GOAL_A, 2)),
        _snapshot(104, (GOAL_A, 2)),
        _snapshot(110, (GOAL_A, 4)),
    ])

    assert result['verdict'] == 'CONFIRMED'
    assert result['scope'] == 'COMMAND_SPACE_ONLY'
    assert result['episodes'][0] == {
        'goal_uuid': GOAL_A,
        'stop_stamp_ns': 100,
        'zero_command_stamp_ns': 101,
        'clear_stamp_ns': 103,
        'resume_command_stamp_ns': 105,
        'same_goal_resumed': True,
        'terminal_succeeded': True,
        'exclusion_reasons': [],
        'terminal_exclusion_reasons': [],
    }


def test_rejects_new_active_goal_during_episode():
    result = _evaluate([
        _snapshot(95, (GOAL_A, 2)),
        _snapshot(102, (GOAL_A, 2)),
        _snapshot(104, (GOAL_B, 2)),
    ])

    assert result['verdict'] == 'NOT_CONFIRMED'
    assert result['episodes'][0]['same_goal_resumed'] is False
    assert 'ACTIVE_GOAL_CHANGED' in result['episodes'][0][
        'exclusion_reasons']


def test_rejects_canceling_goal_before_command_resume():
    result = _evaluate([
        _snapshot(95, (GOAL_A, 2)),
        _snapshot(102, (GOAL_A, 2)),
        _snapshot(104, (GOAL_A, 3)),
    ])

    assert result['verdict'] == 'NOT_CONFIRMED'
    assert 'GOAL_CANCELING_BEFORE_RESUME' in result['episodes'][0][
        'exclusion_reasons']


def test_missing_zero_command_does_not_confirm_resume():
    result = _evaluate(
        [_snapshot(95, (GOAL_A, 2)), _snapshot(104, (GOAL_A, 2))],
        commands=[(99, 'NONZERO'), (105, 'NONZERO')])

    assert result['verdict'] == 'NOT_CONFIRMED'
    assert result['episodes'][0]['exclusion_reasons'] == [
        'NO_ZERO_COMMAND_EVIDENCE']


def test_ongoing_goal_can_resume_without_terminal_success():
    result = _evaluate([
        _snapshot(95, (GOAL_A, 2)),
        _snapshot(102, (GOAL_A, 2)),
        _snapshot(104, (GOAL_A, 2)),
        _snapshot(110, (GOAL_A, 2)),
    ])

    assert result['verdict'] == 'CONFIRMED'
    assert result['episodes'][0]['same_goal_resumed'] is True
    assert result['episodes'][0]['terminal_succeeded'] is False


def test_old_bag_without_action_status_remains_not_measured():
    result = _evaluate([])

    assert result['verdict'] == 'NOT_MEASURED'
    assert result['episodes'] == []
    assert result['data_gap_reasons'] == ['NO_ACTION_STATUS_OBSERVATIONS']


def test_status_data_without_stop_event_is_not_observed_not_missing():
    result = _evaluate(
        [_snapshot(95, (GOAL_A, 2))], collisions=[(100, 0, '')])

    assert result['verdict'] == 'NOT_OBSERVED'
    assert result['action_status_messages'] == 1


def test_resume_for_one_stop_cannot_cross_the_next_stop_boundary():
    result = _evaluate(
        [_snapshot(95, (GOAL_A, 2)),
         _snapshot(102, (GOAL_A, 2)),
         _snapshot(106, (GOAL_A, 2))],
        commands=[(99, 'NONZERO'), (101, 'ZERO'), (106, 'NONZERO')],
        collisions=[(100, 1, 'StopZone'), (103, 0, ''),
                    (104, 1, 'StopZone')])

    assert result['verdict'] == 'NOT_CONFIRMED'
    assert result['episodes'][0]['exclusion_reasons'] == [
        'NO_NONZERO_COMMAND_AFTER_CLEAR']


def test_old_zero_is_invalidated_by_nonzero_before_monitor_clear():
    result = _evaluate(
        [_snapshot(95, (GOAL_A, 2)), _snapshot(104, (GOAL_A, 2))],
        commands=[(98, 'ZERO'), (101, 'NONZERO'), (105, 'NONZERO')])

    assert result['verdict'] == 'NOT_CONFIRMED'
    assert result['episodes'][0]['exclusion_reasons'] == [
        'NO_ZERO_COMMAND_EVIDENCE']


def test_pre_stop_zero_alone_cannot_prove_a_stop_interval():
    result = _evaluate(
        [_snapshot(95, (GOAL_A, 2)), _snapshot(104, (GOAL_A, 2))],
        commands=[(90, 'ZERO'), (105, 'NONZERO')])

    assert result['verdict'] == 'NOT_CONFIRMED'
    assert result['episodes'][0]['exclusion_reasons'] == [
        'NO_ZERO_COMMAND_EVIDENCE']


def test_unknown_after_clear_blocks_resume_confirmation():
    result = _evaluate(
        [_snapshot(95, (GOAL_A, 2)), _snapshot(102, (GOAL_A, 2))],
        collisions=[(100, 1, 'StopZone'), (103, 0, ''),
                    (104, 99, 'unknown')])

    assert result['verdict'] == 'NOT_CONFIRMED'
    assert result['episodes'][0]['exclusion_reasons'] == [
        'UNKNOWN_COLLISION_ACTION_BEFORE_RESUME']


def test_unknown_collision_action_is_not_treated_as_clear():
    result = _evaluate(
        [_snapshot(95, (GOAL_A, 2)), _snapshot(104, (GOAL_A, 2))],
        collisions=[(100, 1, 'StopZone'), (102, 99, 'unknown'),
                    (103, 0, '')])

    assert result['verdict'] == 'NOT_CONFIRMED'
    assert result['episodes'][0]['exclusion_reasons'] == [
        'UNKNOWN_COLLISION_ACTION_BEFORE_CLEAR']


def test_missing_event_stream_is_insufficient_not_not_observed():
    result = _evaluate([_snapshot(95, (GOAL_A, 2))], collisions=[])

    assert result['verdict'] == 'INSUFFICIENT_DATA'
    assert result['data_gap_reasons'] == [
        'NO_COLLISION_STATE_OBSERVATIONS']


def test_malformed_terminal_snapshot_cannot_prove_success():
    malformed_terminal = _snapshot(110, (GOAL_A, 4))
    malformed_terminal['malformed'] = True
    result = _evaluate([
        _snapshot(95, (GOAL_A, 2)),
        _snapshot(102, (GOAL_A, 2)),
        _snapshot(104, (GOAL_A, 2)),
        malformed_terminal,
    ])

    episode = result['episodes'][0]
    assert result['verdict'] == 'CONFIRMED'
    assert episode['terminal_succeeded'] is False
    assert episode['terminal_exclusion_reasons'] == [
        'MALFORMED_TERMINAL_STATUS']
