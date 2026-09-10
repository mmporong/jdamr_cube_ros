"""Regression tests for corridor evidence extraction and media helpers."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

from PIL import Image, ImageSequence

import numpy as np
import pytest  # noqa: I201
import yaml  # noqa: I201


EVALUATION_ROOT = Path(__file__).resolve().parents[1] / 'evaluation'
sys.path.insert(0, str(EVALUATION_ROOT))

from corridor_run_media import (  # noqa: E402,I100,I201
    ANIMATION_BOTTOM_PX,
    ANIMATION_TOP_PX,
    _cluster_points,
    _collision_badge,
    _compose_pose2d,
    _plan_observation,
    _point_to_polyline_distance,
    _project_scan_points,
    _route_deviation_episodes,
    _select_frame_collision_event,
    _select_path_obstacle_cluster,
    analyse_run,
    gap_statistics,
    parse_route_log,
    render_animation,
    summarize_navigation_events,
    write_media_manifest,
)


def _vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def _path(stamp_ns, frame_id, points):
    poses = [SimpleNamespace(pose=SimpleNamespace(
        position=_vector(*point),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )) for point in points]
    return _plan_observation(
        stamp_ns, SimpleNamespace(
            header=SimpleNamespace(frame_id=frame_id), poses=poses))


def _message(topic, stamp_ns, ros_msg):
    return SimpleNamespace(
        channel=SimpleNamespace(topic=topic),
        log_time_ns=stamp_ns,
        ros_msg=ros_msg,
    )


def test_parse_route_log_preserves_success_evidence():
    """Derive the route verdict only from timestamped node output."""
    text = '\n'.join((
        '[INFO] [100.000000000] [jdamr_corridor_route]: route preflight '
        'passed: poses=50 length=8.250m',
        '[INFO] [101.000000000] [jdamr_corridor_route]: send 1/2 '
        'outbound=(4.00,-0.50)',
        '[INFO] [103.000000000] [jdamr_corridor_route]: waypoint=1/2 '
        'remaining=0.20m recoveries=0 battery=12.10V',
        '[INFO] [104.000000000] [jdamr_corridor_route]: send 2/2 '
        'home=(0.00,-0.10)',
        '[INFO] [108.000000000] [jdamr_corridor_route]: waypoint=2/2 '
        'remaining=0.10m recoveries=0 battery=12.00V',
        '[INFO] [109.000000000] [jdamr_corridor_route]: corridor '
        'roundtrip succeeded',
    ))
    result = parse_route_log(text)

    assert result['success'] is True
    assert result['sent_waypoints'] == 2
    assert result['declared_waypoints'] == 2
    assert result['duration_s'] == pytest.approx(8.0)
    assert result['max_recoveries'] == 0
    assert result['battery_min_v'] == pytest.approx(12.0)
    assert result['preflight']['planned_path_length_m'] == pytest.approx(8.25)


def test_parse_route_log_does_not_promote_an_incomplete_run():
    """Keep a cancelled run incomplete when no success line exists."""
    text = '\n'.join((
        '[INFO] [200.000000000] [jdamr_corridor_route]: send 1/2 '
        'outbound=(4.00,-0.50)',
        '[INFO] [203.000000000] [jdamr_corridor_route]: waypoint=1/2 '
        'remaining=1.20m recoveries=1 battery=11.90V',
        '[WARN] [205.000000000] [jdamr_corridor_route]: cancel requested: '
        'scan stale',
    ))
    result = parse_route_log(text)

    assert result['success'] is False
    assert result['sent_waypoints'] == 1
    assert result['max_recoveries'] == 1
    assert result['end_stamp_ns'] == 205_000_000_000


def test_gap_statistics_reports_rate_and_tail_gap():
    """Calculate message continuity from recorder time in SI units."""
    result = gap_statistics([
        1_000_000_000,
        1_100_000_000,
        1_200_000_000,
        1_500_000_000,
    ])

    assert result['messages'] == 4
    assert result['rate_hz'] == pytest.approx(8.0)
    assert result['max_gap_s'] == pytest.approx(0.3)
    assert result['p99_gap_s'] == pytest.approx(0.296)


def test_gap_statistics_includes_silent_window_edges():
    """Count silence after the last message as a control-path gap."""
    result = gap_statistics(
        [1_100_000_000, 1_200_000_000],
        window_start_ns=1_000_000_000,
        window_end_ns=2_000_000_000,
    )

    assert result['messages'] == 2
    assert result['rate_hz'] == pytest.approx(2.0)
    assert result['max_gap_s'] == pytest.approx(0.8)


def test_gap_statistics_handles_empty_and_singleton_streams():
    """Represent missing continuity evidence without inventing a rate."""
    assert gap_statistics([]) == {
        'messages': 0,
        'rate_hz': None,
        'max_gap_s': None,
        'p99_gap_s': None,
    }
    assert gap_statistics([1]) == {
        'messages': 1,
        'rate_hz': None,
        'max_gap_s': None,
        'p99_gap_s': None,
    }


def test_navigation_events_summarize_recorded_transitions_and_plans():
    """Keep transitions tied to recorder time and geometry evidence."""
    first_plan = _path(30, 'map', [(0.0, 0.0, 0.0), (3.0, 4.0, 0.0)])
    repeated_plan = _path(
        31, 'map', [(0.0, 0.0, 0.0), (3.0, 4.0, 0.0)])
    changed_plan = _path(
        32, 'map', [(0.0, 0.0, 0.0), (0.0, 2.0, 0.0)])

    result = summarize_navigation_events(
        [(10, 0, ''), (11, 1, 'StopZone'), (12, 0, '')],
        [(20, 'ZERO'), (21, 'NONZERO'), (22, 'NONZERO'),
         (23, 'ZERO'), (24, 'NONZERO')],
        [first_plan, repeated_plan, changed_plan],
    )

    assert [event['stamp_ns'] for event in
            result['collision_monitor_state']['transitions']] == [11, 12]
    assert result['collision_monitor_state']['transitions'][0][
        'to']['action_name'] == 'STOP'
    assert result['cmd_vel_zero_to_nonzero'][
        'transition_stamps_ns'] == [21, 24]
    assert result['plan_geometry']['geometry_change_count'] == 1
    assert result['plan_geometry']['path_length_m'] == {
        'samples': 3, 'min': 2.0, 'max': 5.0, 'latest': 2.0}
    assert result['same_goal_resumed'] == 'NOT_MEASURED'
    assert 'do not by themselves prove obstacle avoidance' in (
        result['plan_geometry']['interpretation'])


@pytest.mark.parametrize(
    ('plans', 'reason'),
    [
        ([], 'NO_OBSERVATIONS_IN_DRIVE_WINDOW'),
        ([_path(1, 'map', [])], 'EMPTY_PATH'),
        ([_path(1, 'map', [(0.0, 0.0, 0.0)]),
          _path(2, 'odom', [(0.0, 0.0, 0.0)])],
         'FRAME_ID_MISMATCH'),
    ],
)
def test_navigation_events_marks_incomparable_plan_data_insufficient(
        plans, reason):
    """Do not calculate a change count from missing or incomparable paths."""
    result = summarize_navigation_events([], [], plans)

    plan = result['plan_geometry']
    assert plan['evidence_status'] == 'INSUFFICIENT_DATA'
    assert plan['geometry_change_count'] is None
    assert reason in plan['data_gap_reasons']


def test_navigation_events_rejects_pose_frame_mismatch_and_nonfinite_cmd():
    """Do not infer comparable paths or resumed motion from invalid values."""
    path_message = SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        poses=[SimpleNamespace(
            header=SimpleNamespace(frame_id='odom'),
            pose=SimpleNamespace(
                position=_vector(),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0)))])
    plan = _plan_observation(30, path_message)
    result = summarize_navigation_events(
        [], [(20, 'ZERO'), (21, 'UNKNOWN_NONFINITE'),
             (22, 'NONZERO')], [plan])

    assert result['cmd_vel_zero_to_nonzero'][
        'transition_stamps_ns'] == []
    assert result['cmd_vel_zero_to_nonzero'][
        'evidence_status'] == 'INSUFFICIENT_DATA'
    assert 'NONFINITE_COMMAND' in result['cmd_vel_zero_to_nonzero'][
        'data_gap_reasons']
    assert 'POSE_FRAME_ID_MISMATCH' in result['plan_geometry'][
        'data_gap_reasons']


def test_scan_projection_distinguishes_map_wall_and_free_space_returns():
    """Label only free-space endpoints as obstacle candidates."""
    occupancy = np.full((20, 20), 254, dtype='uint8')
    occupancy[:, 15] = 0
    scan = {
        'angle_min_rad': 0.0,
        'angle_increment_rad': math.pi / 2,
        'range_min_m': 0.1,
        'range_max_m': 20.0,
        'ranges_m': (5.0, 3.0),
    }
    result = _project_scan_points(
        scan, (10.0, 10.0, 0.0), (0.0, 0.0, 0.0), occupancy,
        1.0, [0.0, 0.0, 0.0], wall_margin_cells=0)

    assert result['static'] == [(15.0, 10.0)]
    assert result['obstacle_candidates'][0] == pytest.approx((10.0, 13.0))


def test_frame_collision_event_prefers_stop_and_ignores_startup_fault():
    """Make brief safety events visible without presenting startup faults."""
    events = [
        {'stamp_ns': 100, 'action_name': 'STOP',
         'polygon_name': 'invalid source'},
        {'stamp_ns': 102, 'action_name': 'SLOWDOWN',
         'polygon_name': 'SlowdownZone'},
        {'stamp_ns': 104, 'action_name': 'STOP',
         'polygon_name': 'StopZone'},
    ]

    selected = _select_frame_collision_event(events, 103, 4)

    assert selected['stamp_ns'] == 104


def test_collision_badge_hides_internal_stop_zone_names():
    """Present safety meaning instead of internal monitor identifiers."""
    badge = _collision_badge('STOP', 'StopZone', 0.14)

    assert badge == '자동 안전 정지 0.14초 · 충돌 방지 개입'
    assert 'STOP' not in badge
    assert 'StopZone' not in badge


@pytest.mark.parametrize(
    ('action_name', 'expected'),
    [
        ('SLOWDOWN', '안전 감속 제어'),
        ('APPROACH', '접근 속도 제어'),
        ('LIMIT', '속도 제한'),
    ],
)
def test_collision_badge_localizes_non_stop_actions(action_name, expected):
    """Keep internal collision-monitor action names out of operator media."""
    assert _collision_badge(action_name, 'InternalZone') == expected


def test_compose_pose2d_rotates_child_translation():
    """Compose child translation in the parent's rotated frame."""
    result = _compose_pose2d(
        (1.0, 2.0, math.pi / 2), (3.0, 0.0, math.pi / 2))

    assert result == pytest.approx((1.0, 5.0, math.pi))


def test_point_to_polyline_distance_measures_lateral_route_departure():
    """Measure cross-track distance instead of distance to one waypoint."""
    distance = _point_to_polyline_distance(
        (2.0, 1.5), [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0)])

    assert distance == pytest.approx(1.5)


def test_lidar_clustering_boxes_nearby_returns_without_object_label():
    """Group measured endpoints while leaving object identity unknown."""
    clusters = _cluster_points([
        (0.0, 0.0), (0.1, 0.0), (0.05, 0.08),
        (3.0, 3.0), (3.1, 3.0),
    ], radius_m=0.2, min_points=3)

    assert len(clusters) == 1
    assert len(clusters[0]) == 3


def test_path_obstacle_selection_ignores_unrelated_lidar_clusters():
    """Keep only a cluster on the baseline and within local sensor range."""
    blocking = [(1.0, 0.1), (1.1, 0.1), (1.05, 0.2)]
    unrelated = [(3.0, 1.0), (3.1, 1.0), (3.05, 1.1)]
    too_far_from_robot = [(6.0, 0.1), (6.1, 0.1), (6.05, 0.2)]

    selected = _select_path_obstacle_cluster(
        [unrelated, too_far_from_robot, blocking],
        [(0.0, 0.0), (8.0, 0.0)], (0.0, 0.0),
        route_distance_m=0.5, robot_distance_m=2.5)

    assert selected == blocking


def test_route_deviation_episodes_group_only_contiguous_threshold_crossings():
    """Keep obstacle-linked detours separate from ordinary tracked travel."""
    samples = [
        {'stamp_ns': 100}, {'stamp_ns': 200}, {'stamp_ns': 300},
        {'stamp_ns': 400}, {'stamp_ns': 500},
    ]
    deviations = [0.05, 0.13, 0.30, 0.08, 0.14]

    episodes = _route_deviation_episodes(samples, deviations)

    assert [(item['start_index'], item['end_index'])
            for item in episodes] == [(1, 2), (4, 4)]
    assert episodes[0]['peak_deviation_m'] == pytest.approx(0.30)


def test_analyse_run_collects_navigation_events_in_existing_reader_loop(
        tmp_path, monkeypatch):
    """Integrate event extraction without a second bag read or ROS runtime."""
    run_dir = tmp_path / 'run_01'
    run_dir.mkdir()
    (run_dir / 'run.mcap').write_bytes(b'fake-mcap')
    (tmp_path / 'run_01.route.log').write_text('\n'.join((
        '[INFO] [101.000000000] [route]: send 1/1 home=(1.00,0.00)',
        '[INFO] [109.000000000] [route]: corridor roundtrip succeeded',
    )), encoding='utf-8')
    covariance = [0.0] * 36

    def amcl_pose(x):
        return SimpleNamespace(pose=SimpleNamespace(
            pose=SimpleNamespace(position=_vector(x=x)),
            covariance=covariance))

    def twist(x):
        return SimpleNamespace(linear=_vector(x=x), angular=_vector())

    def path_message(x):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id='map'),
            poses=[SimpleNamespace(pose=SimpleNamespace(
                position=_vector(),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0))),
                   SimpleNamespace(pose=SimpleNamespace(
                       position=_vector(x=x),
                       orientation=SimpleNamespace(
                           x=0.0, y=0.0, z=0.0, w=1.0)))])

    def action_status(status):
        return SimpleNamespace(status_list=[SimpleNamespace(
            goal_info=SimpleNamespace(goal_id=SimpleNamespace(
                uuid=bytes.fromhex('01' * 16))),
            status=status)])
    messages = [
        _message('/navigate_to_pose/_action/status', 100_000_000_000,
                 action_status(2)),
        _message('/amcl_pose', 101_000_000_000, amcl_pose(0.0)),
        _message('/battery_state', 102_000_000_000,
                 SimpleNamespace(voltage=12.0)),
        _message('/cmd_vel', 103_000_000_000, twist(0.0)),
        _message('/cmd_vel', 104_000_000_000, twist(0.2)),
        _message('/collision_monitor_state', 105_000_000_000,
                 SimpleNamespace(action_type=0, polygon_name='')),
        _message('/collision_monitor_state', 106_000_000_000,
                 SimpleNamespace(action_type=1, polygon_name='StopZone')),
        _message('/plan', 107_000_000_000, path_message(1.0)),
        _message('/plan', 108_000_000_000, path_message(2.0)),
        _message('/navigate_to_pose/_action/status', 108_500_000_000,
                 action_status(2)),
        _message('/amcl_pose', 109_000_000_000, amcl_pose(1.0)),
        _message('/cmd_vel', 110_000_000_000, twist(0.0)),
    ]
    reads = []

    def fake_read(_bag, topics):
        reads.append(topics)
        return iter(messages)

    monkeypatch.setattr(
        'corridor_run_media.read_navigation_messages', fake_read)
    monkeypatch.setattr(
        'corridor_run_media.inspect',
        lambda _bag: {
            'sha256': 'fake',
            'integrity': 'PASS',
            'message_count': len(messages),
            'duration_ns': 9_000_000_000,
        })

    metrics, _series = analyse_run(run_dir)

    events = metrics['navigation_events']
    assert len(reads) == 1
    assert '/collision_monitor_state' in reads[0]
    assert '/navigate_to_pose/_action/status' in reads[0]
    assert events['same_goal_resume_evidence'][
        'action_status_messages'] == 2
    assert events['cmd_vel_zero_to_nonzero'][
        'transition_stamps_ns'] == [104_000_000_000]
    assert events['collision_monitor_state']['transitions'][0][
        'stamp_ns'] == 106_000_000_000
    assert events['plan_geometry']['geometry_change_count'] == 1


def test_media_manifest_hashes_generated_artifacts(tmp_path):
    """Make the exported portfolio bundle independently verifiable."""
    artifact = tmp_path / 'sample.csv'
    artifact.write_text('value\n1\n', encoding='utf-8')
    metrics = {
        'run_id': 'sample_run',
        'source': {
            'mcap_sha256': 'mcap-hash',
            'route_log_sha256': 'route-hash',
        },
    }

    manifest_path = write_media_manifest(tmp_path, metrics)
    manifest = yaml.safe_load(
        manifest_path.read_text(encoding='utf-8'))

    assert manifest['run_id'] == 'sample_run'
    assert manifest['files'][0]['file'] == 'sample.csv'
    assert manifest['files'][0]['size_bytes'] == artifact.stat().st_size
    assert len(manifest['files'][0]['sha256']) == 64


def test_animation_preserves_map_outline_across_gif_frames(tmp_path):
    """Static map walls must not vanish between optimized GIF frames."""
    map_image = Image.new('L', (80, 50), 205)
    for x in range(80):
        map_image.putpixel((x, 10), 0)
        map_image.putpixel((x, 40), 0)
    map_image.save(tmp_path / 'map.pgm')
    Image.new('L', (80, 50), 255).save(tmp_path / 'mask.pgm')
    for name in ('map', 'mask'):
        (tmp_path / f'{name}.yaml').write_text(
            f'image: {name}.pgm\nresolution: 0.1\n'
            'origin: [0.0, 0.0, 0.0]\nnegate: 0\n'
            'occupied_thresh: 0.65\nfree_thresh: 0.196\n',
            encoding='utf-8')
    route = tmp_path / 'route.yaml'
    route.write_text(
        'map_yaml: ' + str(tmp_path / 'map.yaml') + '\n'
        'keepout_mask_yaml: ' + str(tmp_path / 'mask.yaml') + '\n'
        'start_pose: {x: 1.0, y: 2.0}\n'
        'waypoints:\n  - {id: end, x: 6.0, y: 2.0}\n',
        encoding='utf-8')
    output = tmp_path / 'route.gif'
    metrics = {'capture': {
        'drive_start_unix_ns': 1_000_000_000,
        'drive_end_unix_ns': 2_000_000_000,
        'drive_duration_s': 1.0,
    }}
    series = {'amcl': [
        {'stamp_ns': 1_000_000_000, 'x_m': 1.0, 'y_m': 2.0},
        {'stamp_ns': 2_000_000_000, 'x_m': 6.0, 'y_m': 2.0},
    ]}

    render_animation(route, metrics, series, output, frames=4, fps=2)

    gif = Image.open(output)
    wall_counts = []
    for frame in ImageSequence.Iterator(gif):
        crop = frame.convert('RGB').crop((
            0, ANIMATION_TOP_PX, gif.width,
            gif.height - ANIMATION_BOTTOM_PX))
        wall_counts.append(sum(
            1 for red, green, blue in crop.getdata()
            if red < 40 and green < 40 and blue < 40))
    assert min(wall_counts) > 0
    assert len(set(wall_counts)) == 1
