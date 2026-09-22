"""Pure readiness checks for operator-driven mapping."""

from types import SimpleNamespace

from jdamr_cube_navigation.operator_mapping_preflight import (
    directional_counts,
    endpoint_names,
    point_in_polygon,
    readiness_blockers,
    sample_metrics,
)

import pytest


def _ready_checks():
    return {
        'nodes': {'missing': [], 'duplicates': [], 'forbidden': []},
        'command_chain': {
            topic: {'valid': True}
            for topic in ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel')
        },
        'lifecycle': {
            'velocity_smoother': 'active',
            'collision_monitor': 'active',
            'map_saver': 'active',
        },
        'scan': {'age_s': 0.05, 'rate_hz': 9.7, 'max_gap_s': 0.12},
        'odom': {'age_s': 0.01, 'rate_hz': 50.0, 'max_gap_s': 0.03},
        'stationary': True,
        'map': {'age_s': 0.5, 'resolution_m': 0.05, 'valid_size': True},
        'battery': {'age_s': 0.5, 'voltage_v': 12.3},
        'tf': {'map_to_base': True, 'base_to_laser': True},
        'system': {
            'temperature_c': 74.0,
            'current_throttle_bits': 0,
            'free_bytes': 1024 * 1024 * 1024,
        },
    }


def test_ready_snapshot_has_no_blockers():
    """All measured prerequisites produce one unambiguous PASS."""
    assert readiness_blockers(_ready_checks()) == []


def test_known_repeat_failures_block_motion_with_specific_reasons():
    """Past stale scan, duplicate command and motion faults fail closed."""
    checks = _ready_checks()
    checks['command_chain']['/cmd_vel']['valid'] = False
    checks['scan']['max_gap_s'] = 1.2
    checks['stationary'] = False
    checks['tf']['map_to_base'] = False
    blockers = readiness_blockers(checks)
    assert '명령 경로 불일치: /cmd_vel' in blockers
    assert '라이다 입력 공백' in blockers
    assert '차체가 정지 상태가 아님' in blockers
    assert 'map→base_footprint TF 없음' in blockers


def test_sample_metrics_reports_rate_gap_and_age():
    """Receive timing makes intermittent scan stalls visible."""
    metrics = sample_metrics([1.0, 1.1, 1.4], 1.45)
    assert metrics['age_s'] == pytest.approx(0.05)
    assert metrics['rate_hz'] == pytest.approx(5.0)
    assert metrics['max_gap_s'] == pytest.approx(0.3)


def test_endpoint_names_preserves_namespaces():
    """Graph ownership comparisons use fully qualified node names."""
    endpoints = [
        SimpleNamespace(node_name='collision_monitor', node_namespace='/'),
        SimpleNamespace(node_name='driver', node_namespace='/base'),
    ]
    assert endpoint_names(endpoints) == {
        '/collision_monitor', '/base/driver'}


def test_directional_counts_match_collision_polygon_membership():
    """Direction buttons use the same minimum-point geometry as monitoring."""
    square = [[1.0, 1.0], [1.0, -1.0], [-1.0, -1.0], [-1.0, 1.0]]
    points = [(0.0, 0.0), (0.5, 0.5), (2.0, 0.0)]
    assert point_in_polygon(0.0, 0.0, square)
    assert not point_in_polygon(2.0, 0.0, square)
    assert directional_counts(points, {'forward': square}) == {'forward': 2}
