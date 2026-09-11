"""Contract tests for the restaurant replay scenario node."""

import math

from jdamr_cube_navigation.sim_restaurant_replay_scenario import (
    scene_schedule,
    waypoint_yaw,
    wheel_slip_commands,
)

import pytest


def _contract():
    return {
        'route': {'simulation_waypoint_count': 20},
        'obstacle_interventions': {
            'events': [
                {'event': index, 'trigger': {
                    'waypoint_index': waypoint,
                    'waypoint_id': f'wp_{waypoint}',
                }}
                for index, waypoint in enumerate(
                    (6, 6, 9, 9, 18, 18), start=1)
            ],
            'scenes': [
                {'scene_id': 'box_1', 'trigger_waypoint_index': 6,
                 'source_event_ids': [1, 2]},
                {'scene_id': 'box_2', 'trigger_waypoint_index': 9,
                 'source_event_ids': [3, 4]},
                {'scene_id': 'person', 'trigger_waypoint_index': 18,
                 'source_event_ids': [5, 6]},
            ],
        },
    }


def test_real_event_pairs_are_grouped_without_losing_source_order():
    """Two interventions remain attached to each measured route phase."""
    schedule = scene_schedule(_contract())

    assert list(schedule) == [6, 9, 18]
    assert [[item['scene_id'] for item in schedule[index]]
            for index in schedule] == [['box_1'], ['box_2'], ['person']]


def test_invalid_event_count_is_rejected():
    """A partial replay cannot masquerade as the six-event source run."""
    contract = _contract()
    contract['obstacle_interventions']['events'].pop()

    with pytest.raises(ValueError, match='six records, three scenes'):
        scene_schedule(contract)


def test_waypoint_heading_matches_outbound_and_return_direction():
    """The transformed two-lane route faces along its travel direction."""
    assert waypoint_yaw({'id': 'outbound_22m'}) == 0.0
    assert waypoint_yaw({'id': 'turnaround'}) == math.pi
    assert waypoint_yaw({'id': 'return_06m'}) == math.pi
    assert waypoint_yaw({'id': 'home'}) == math.pi


def test_wheel_slip_commands_target_scoped_drive_links():
    """Fault injection updates both wheel links through the same world."""
    commands = wheel_slip_commands(
        ['left_wheel_link', 'right_wheel_link'], 0.0, 10.0)

    assert len(commands) == 2
    assert all('/world/slam_corridor/wheel_slip' in item for item in commands)
    assert 'jdamr_cube::left_wheel_link' in commands[0][-1]
    assert 'slip_compliance_longitudinal: 10.0' in commands[1][-1]
