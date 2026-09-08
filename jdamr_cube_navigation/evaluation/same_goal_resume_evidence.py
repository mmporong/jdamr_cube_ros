#!/usr/bin/env python3
"""Conservatively bind collision-stop command resumption to one Nav2 goal."""

from __future__ import annotations

from typing import Any


ACTIVE_STATUSES = {1, 2}
TERMINAL_STATUSES = {4, 5, 6}


def goal_status_snapshot(stamp_ns: int, message: Any) -> dict[str, Any]:
    """Convert one GoalStatusArray to JSON-safe recorder-time evidence."""
    statuses = []
    malformed = False
    try:
        items = list(message.status_list)
    except (AttributeError, TypeError):
        items = []
        malformed = True
    for item in items:
        try:
            uuid_bytes = bytes(item.goal_info.goal_id.uuid)
            status = int(item.status)
        except (AttributeError, TypeError, ValueError):
            malformed = True
            continue
        if len(uuid_bytes) != 16 or not any(uuid_bytes) or status not in range(7):
            malformed = True
            continue
        statuses.append({'goal_uuid': uuid_bytes.hex(), 'status': status})
    return {
        'stamp_ns': int(stamp_ns),
        'statuses': statuses,
        'malformed': malformed,
    }


def _active_goals(snapshot: dict[str, Any]) -> list[str]:
    return sorted({entry['goal_uuid'] for entry in snapshot['statuses']
                   if entry['status'] in ACTIVE_STATUSES})


def _latest_before(snapshots: list[dict[str, Any]], stamp_ns: int):
    candidates = [item for item in snapshots if item['stamp_ns'] < stamp_ns]
    return candidates[-1] if candidates else None


def _episode(stop_ns: int, clear_ns: int | None, next_stop_ns: int | None,
             command_states: list[tuple[int, str]],
             snapshots: list[dict[str, Any]], drive_end_ns: int,
             terminal_end_ns: int, ordering_ambiguous: bool,
             collision_segment: list[tuple[int, int, str]],
             collision_reason: str | None = None) -> dict[str, Any]:
    result = {
        'goal_uuid': None,
        'stop_stamp_ns': stop_ns,
        'zero_command_stamp_ns': None,
        'clear_stamp_ns': clear_ns,
        'resume_command_stamp_ns': None,
        'same_goal_resumed': False,
        'terminal_succeeded': False,
        'exclusion_reasons': [],
        'terminal_exclusion_reasons': [],
    }
    reasons = result['exclusion_reasons']
    if ordering_ambiguous:
        reasons.append('RECORDER_EVENT_ORDER_AMBIGUOUS')
        return result
    if collision_reason is not None:
        reasons.append(collision_reason)
        return result
    if clear_ns is None:
        reasons.append('NO_COLLISION_MONITOR_CLEAR')
        return result
    if clear_ns == stop_ns:
        reasons.append('SAME_TIMESTAMP_ORDER_AMBIGUOUS')
        return result

    commands = sorted(command_states)
    after_stop = [item for item in commands
                  if stop_ns < item[0] < clear_ns and item[1] == 'ZERO']
    if after_stop:
        zero = after_stop[0]
    else:
        reasons.append('NO_ZERO_COMMAND_EVIDENCE')
        return result
    result['zero_command_stamp_ns'] = zero[0]
    if any(state != 'ZERO' for stamp_ns, state in commands
           if zero[0] < stamp_ns < clear_ns):
        reasons.append('COMMAND_NOT_ZERO_THROUGH_CLEAR')
        return result

    resumes = [item for item in commands
               if (clear_ns < item[0] <= drive_end_ns
                   and (next_stop_ns is None or item[0] < next_stop_ns)
                   and item[1] == 'NONZERO')]
    if not resumes:
        reasons.append('NO_NONZERO_COMMAND_AFTER_CLEAR')
        return result
    resume_ns = resumes[0][0]
    result['resume_command_stamp_ns'] = resume_ns
    if any(action_type not in range(5)
           for stamp_ns, action_type, _name in collision_segment
           if stop_ns < stamp_ns < resume_ns):
        reasons.append('UNKNOWN_COLLISION_ACTION_BEFORE_RESUME')
        return result
    if any(stamp_ns in {stop_ns, clear_ns}
           for stamp_ns, _state in commands):
        reasons.append('SAME_TIMESTAMP_ORDER_AMBIGUOUS')
        return result
    if any(item['stamp_ns'] in {stop_ns, clear_ns, resume_ns}
           for item in snapshots):
        reasons.append('SAME_TIMESTAMP_ORDER_AMBIGUOUS')
        return result
    if any(state == 'UNKNOWN_NONFINITE'
           for stamp_ns, state in commands
           if zero[0] <= stamp_ns <= resume_ns):
        reasons.append('NONFINITE_COMMAND')
        return result

    phase_snapshots = [item for item in snapshots
                       if stop_ns <= item['stamp_ns'] < resume_ns]
    milestone_snapshots = [
        _latest_before(snapshots, stop_ns),
        _latest_before(snapshots, clear_ns),
        _latest_before(snapshots, resume_ns),
    ]
    if any(item is None for item in milestone_snapshots):
        reasons.append('NO_ACTIVE_GOAL_EVIDENCE_AT_MILESTONE')
        return result
    relevant_ids = {
        id(item) for item in [*milestone_snapshots, *phase_snapshots]}
    relevant_snapshots = [item for item in snapshots
                          if id(item) in relevant_ids]
    if any(item['malformed'] for item in relevant_snapshots):
        reasons.append('MALFORMED_ACTION_STATUS')
        return result
    active_sets = [_active_goals(item) for item in relevant_snapshots]
    if any(len(active) > 1 for active in active_sets):
        reasons.append('MULTIPLE_ACTIVE_GOALS')
        return result
    first_active = _active_goals(milestone_snapshots[0])
    if len(first_active) != 1:
        reasons.append('NO_ACTIVE_GOAL_EVIDENCE_AT_MILESTONE')
        return result
    goal_uuid = first_active[0]
    result['goal_uuid'] = goal_uuid
    target_before_resume = [
        entry['status']
        for item in relevant_snapshots
        for entry in item['statuses']
        if entry['goal_uuid'] == goal_uuid
    ]
    if 3 in target_before_resume:
        reasons.append('GOAL_CANCELING_BEFORE_RESUME')
        return result
    if any(status in TERMINAL_STATUSES for status in target_before_resume):
        reasons.append('GOAL_TERMINAL_BEFORE_RESUME')
        return result
    if any(not active for active in active_sets):
        reasons.append('GOAL_NOT_ACTIVE_DURING_EPISODE')
        return result
    if any(active != [goal_uuid] for active in active_sets):
        reasons.append('ACTIVE_GOAL_CHANGED')
        return result

    result['same_goal_resumed'] = True
    terminal_snapshots = [item for item in snapshots
                          if resume_ns < item['stamp_ns'] <= terminal_end_ns]
    terminal_reasons = result['terminal_exclusion_reasons']
    if any(item['malformed'] for item in terminal_snapshots):
        terminal_reasons.append('MALFORMED_TERMINAL_STATUS')
    target_sequences = []
    for item in terminal_snapshots:
        target_statuses = [entry['status'] for entry in item['statuses']
                           if entry['goal_uuid'] == goal_uuid]
        if len(set(target_statuses)) > 1:
            terminal_reasons.append('CONTRADICTORY_TERMINAL_STATUS')
        target_sequences.extend(target_statuses)
    terminal_positions = [index for index, status in enumerate(target_sequences)
                          if status in TERMINAL_STATUSES]
    if terminal_positions:
        first_terminal = terminal_positions[0]
        if target_sequences[first_terminal] != 4:
            terminal_reasons.append('TERMINAL_STATUS_NOT_SUCCEEDED')
        if any(status in ACTIVE_STATUSES
               for status in target_sequences[first_terminal + 1:]):
            terminal_reasons.append('ACTIVE_STATUS_AFTER_TERMINAL')
        if any(status in {5, 6} for status in target_sequences):
            terminal_reasons.append('CONTRADICTORY_TERMINAL_STATUS')
        result['terminal_succeeded'] = not terminal_reasons
    return result


def summarize_same_goal_resume(
        collision_states: list[tuple[int, int, str]],
        command_states: list[tuple[int, str]],
        status_snapshots: list[dict[str, Any]], drive_start_ns: int,
        drive_end_ns: int, terminal_end_ns: int) -> dict[str, Any]:
    """Evaluate STOP-to-resume episodes without inferring physical motion."""
    snapshot_stamps = [item['stamp_ns'] for item in status_snapshots]
    collision_stamps = [item[0] for item in collision_states]
    command_stamps = [item[0] for item in command_states]
    ordering_ambiguous = any(
        stamps != sorted(stamps) or len(stamps) != len(set(stamps))
        for stamps in (snapshot_stamps, collision_stamps, command_stamps)
    )
    snapshots = sorted(status_snapshots, key=lambda item: item['stamp_ns'])
    if not snapshots:
        return {
            'verdict': 'NOT_MEASURED',
            'scope': 'COMMAND_SPACE_ONLY',
            'action_status_messages': 0,
            'action_status_observations': [],
            'terminal_succeeded': None,
            'episodes': [],
            'data_gap_reasons': ['NO_ACTION_STATUS_OBSERVATIONS'],
        }

    collisions = sorted(
        item for item in collision_states
        if drive_start_ns <= item[0] <= drive_end_ns)
    commands = sorted(
        item for item in command_states
        if drive_start_ns <= item[0] <= drive_end_ns)
    episodes = []
    stop_indices = [
        index for index, item in enumerate(collisions)
        if item[1] == 1 and (index == 0 or collisions[index - 1][1] != 1)
    ]
    for stop_number, index in enumerate(stop_indices):
        stamp_ns = collisions[index][0]
        next_stop_ns = (
            collisions[stop_indices[stop_number + 1]][0]
            if stop_number + 1 < len(stop_indices) else None)
        segment = collisions[index + 1:]
        if next_stop_ns is not None:
            segment = [item for item in segment if item[0] < next_stop_ns]
        unknown_before_clear = next(
            (item for item in segment if item[1] not in range(5)), None)
        clear_ns = next((later_stamp for later_stamp, later_action, _name
                         in segment if later_action in {0, 2, 3, 4}), None)
        if (unknown_before_clear is not None
                and (clear_ns is None or unknown_before_clear[0] < clear_ns)):
            collision_reason = 'UNKNOWN_COLLISION_ACTION_BEFORE_CLEAR'
        else:
            collision_reason = None
        if index > 0 and collisions[index - 1][0] == stamp_ns:
            collision_reason = 'SAME_TIMESTAMP_ORDER_AMBIGUOUS'
        if clear_ns is not None and clear_ns == stamp_ns:
            collision_reason = 'SAME_TIMESTAMP_ORDER_AMBIGUOUS'
        if collisions[index][1] != 1:
            continue
        episodes.append(_episode(
            stamp_ns, clear_ns, next_stop_ns, commands, snapshots,
            drive_end_ns, terminal_end_ns, ordering_ambiguous,
            segment, collision_reason))

    if not collisions or not commands:
        verdict = 'INSUFFICIENT_DATA'
    elif any(item['same_goal_resumed'] for item in episodes):
        verdict = 'CONFIRMED'
    elif episodes:
        verdict = 'NOT_CONFIRMED'
    else:
        verdict = 'NOT_OBSERVED'
    return {
        'verdict': verdict,
        'scope': 'COMMAND_SPACE_ONLY',
        'action_status_messages': len(snapshots),
        'action_status_observations': snapshots,
        'terminal_succeeded': any(
            item['terminal_succeeded'] for item in episodes),
        'episodes': episodes,
        'data_gap_reasons': [
            reason for missing, reason in (
                (not collisions, 'NO_COLLISION_STATE_OBSERVATIONS'),
                (not commands, 'NO_COMMAND_OBSERVATIONS'),
            ) if missing
        ],
    }
