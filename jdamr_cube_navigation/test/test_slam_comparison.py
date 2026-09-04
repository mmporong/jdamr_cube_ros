"""Regression tests for the offline SLAM comparison maths."""

# These tests pin two output-integrity defects: high-rate odometry noise must
# not inflate path length, and frame offsets must not be reported as backend
# deviation from the saved-map localization reference.

import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))

pytest.importorskip('mcap_ros2',
                    reason='evaluation deps are installed separately')

from compare_slam_runs import (  # noqa: E402,I100
    apply_rigid,
    comparison_precondition_errors,
    comparison_summary,
    compose,
    decimate,
    main,
    map_statistics,
    nearest,
    path_length,
    render_report,
    rigid_align,
    yaw_of,
)


class _Quaternion:
    def __init__(self, z, w):
        self.x = 0.0
        self.y = 0.0
        self.z = z
        self.w = w


def _straight_line(metres, step, noise=0.0):
    points = []
    distance = 0.0
    index = 0
    while distance <= metres:
        offset = noise if index % 2 else -noise
        points.append((float(index), distance, offset, 0.0))
        distance += step
        index += 1
    return points


def test_path_length_bounds_high_rate_sensor_noise():
    """Bound the noise inflation and state the bound explicitly."""
    # Summing every consecutive pair of the noisy stream gives over 40 m for a
    # 10 m line.  Decimation does not remove the inflation, it bounds it to
    # the lateral noise carried across each retained step, so the contract is
    # "within about ten percent", not "exact".
    clean = _straight_line(10.0, 0.004)
    noisy = _straight_line(10.0, 0.004, noise=0.01)
    naive = sum(math.dist(a[1:3], b[1:3])
                for a, b in zip(noisy, noisy[1:]))

    assert path_length(clean) == pytest.approx(10.0, abs=0.1)
    assert naive > 40.0
    assert path_length(noisy) < 0.25 * naive
    assert path_length(noisy) == pytest.approx(10.0, rel=0.10)


def test_decimate_keeps_the_endpoints():
    """Dropping the last sample would shorten every measured trajectory."""
    points = _straight_line(1.0, 0.004)
    kept = decimate(points, 0.05)

    assert kept[0] == points[0]
    assert kept[-1] == points[-1]
    assert len(kept) < len(points)


def test_rigid_align_recovers_a_known_frame_offset():
    """The alignment must remove the frame offset, not the estimate error."""
    theta = math.radians(30.0)
    source = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0)]
    target = [(math.cos(theta) * x - math.sin(theta) * y + 5.0,
               math.sin(theta) * x + math.cos(theta) * y - 3.0)
              for x, y in source]

    transform = rigid_align(source, target)

    assert transform is not None
    assert math.degrees(transform[0]) == pytest.approx(30.0, abs=0.01)
    for point, expected in zip(source, target):
        moved = apply_rigid(transform, *point)
        assert math.dist(moved, expected) < 1e-6


def test_rigid_align_never_rescales_the_trajectory():
    """Scaling would hide a backend that under- or over-estimates distance."""
    source = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    target = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]

    transform = rigid_align(source, target)
    moved = [apply_rigid(transform, *point) for point in source]

    assert math.dist(moved[0], moved[-1]) == pytest.approx(2.0, abs=1e-6)


def test_compose_matches_a_hand_computed_transform():
    """map->odom composed with odom->base must rotate the child offset."""
    composed = compose((1.0, 2.0, math.radians(90.0)), (3.0, 0.0, 0.0))

    assert composed[0] == pytest.approx(1.0, abs=1e-9)
    assert composed[1] == pytest.approx(5.0, abs=1e-9)


def test_yaw_of_reads_a_quarter_turn():
    """A quaternion at 90 degrees must not read as zero."""
    assert math.degrees(
        yaw_of(_Quaternion(math.sin(math.pi / 4), math.cos(math.pi / 4)))
    ) == pytest.approx(90.0, abs=1e-6)


def test_nearest_uses_sorted_timestamp_neighbours():
    """Timestamp matching must stay correct after logarithmic lookup."""
    samples = [(1.0, 'first'), (2.0, 'second'), (3.0, 'third')]

    assert nearest(samples, 1.6) == samples[1]
    assert nearest(samples, 0.9) == samples[0]
    assert nearest(samples, 4.0) is None


def test_map_statistics_preserves_unknown_cells_and_square_units(tmp_path):
    """Trinary unknown pixels are neither free nor a one-dimensional length."""
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n2 2\n255\n' + bytes((0, 254, 205, 128)))
    metadata = tmp_path / 'map.yaml'
    metadata.write_text(
        'image: map.pgm\n'
        'mode: trinary\n'
        'resolution: 0.05\n'
        'origin: [0, 0, 0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n',
        encoding='utf-8')

    result = map_statistics(metadata)

    assert result['occupied_cells'] == 1
    assert result['free_cells'] == 1
    assert result['unknown_cells'] == 2
    assert result['occupied_area_m2'] == pytest.approx(0.0025)
    assert 'occupied_length_m' not in result


def test_backend_selection_requires_both_consistency_criteria_to_agree():
    """Do not select a backend when observable criteria disagree."""
    records = [
        {'backend': 'a', 'start_to_end_m': 1.0,
         'deviation_from_amcl': {'rms_m': 2.0}},
        {'backend': 'b', 'start_to_end_m': 2.0,
         'deviation_from_amcl': {'rms_m': 1.0}},
    ]

    assert comparison_summary(records)['selected_backend'] is None


def test_backend_selection_requires_two_distinct_completed_backends():
    """One replay result is evidence, but it is not a comparison."""
    record = {'backend': 'a', 'start_to_end_m': 1.0,
              'deviation_from_amcl': {'rms_m': 0.5}}

    assert comparison_summary([record]) == {
        'selected_backend': None,
        'reason': 'insufficient_data',
    }


def test_comparison_rejects_mixed_source_bags():
    """Backends are comparable only when their source bag is identical."""
    records = [
        {'backend': 'a', 'source_bag': 'one.mcap',
         'source_bag_sha256': 'aaa', 'start_to_end_m': 1.0,
         'deviation_from_amcl': {'rms_m': 0.5}, 'map': {},
         'map_yaml': 'a.yaml'},
        {'backend': 'b', 'source_bag': 'two.mcap',
         'source_bag_sha256': 'bbb', 'start_to_end_m': 2.0,
         'deviation_from_amcl': {'rms_m': 1.0}, 'map': {},
         'map_yaml': 'b.yaml'},
    ]

    errors = comparison_precondition_errors(records)

    assert 'all results must use one identical source bag hash' in errors
    assert 'all results must use one identical source bag name' in errors


def test_comparison_main_refuses_missing_map_without_writing_outputs(tmp_path):
    """An incomplete replay must not produce portfolio comparison artifacts."""
    results = tmp_path / 'results'
    result_dir = results / 'run__cartographer_result'
    result_dir.mkdir(parents=True)
    (result_dir / 'result.mcap').touch()
    (result_dir / 'metadata.yaml').write_text('complete: true\n')
    source_root = tmp_path / 'source'
    source_dir = source_root / 'run'
    source_dir.mkdir(parents=True)
    (source_dir / 'source.mcap').touch()
    output = tmp_path / 'comparison.json'

    with pytest.raises(SystemExit) as error:
        main(['--results', str(results), '--source-root', str(source_root),
              '--output', str(output)])

    assert error.value.code == 2
    assert not output.exists()


def test_report_labels_amcl_as_reference_instead_of_ground_truth():
    """Portfolio evidence must not turn AMCL consistency into accuracy."""
    records = [
        {'backend': 'a', 'estimated_length_m': 10.0,
         'start_to_end_m': 1.0,
         'deviation_from_amcl': {'rms_m': 0.5, 'max_m': 1.0},
         'map': {'extent_m': [5.0, 2.0]}},
        {'backend': 'b', 'estimated_length_m': 11.0,
         'start_to_end_m': 2.0,
         'deviation_from_amcl': {'rms_m': 1.0, 'max_m': 2.0},
         'map': {'extent_m': [6.0, 3.0]}},
    ]

    report = render_report(records)

    assert '`a`를 현재 복도 데이터의 기본 백엔드로 선택' in report
    assert '외부 ground truth가 없으므로' in report
    assert 'ATE/RPE가 아니다' in report


def test_report_defers_selection_when_consistency_criteria_disagree():
    """A split decision must be described as deferred, never as None."""
    records = [
        {'backend': 'a', 'estimated_length_m': 10.0,
         'start_to_end_m': 1.0,
         'deviation_from_amcl': {'rms_m': 2.0, 'max_m': 3.0},
         'map': {'extent_m': [5.0, 2.0]}},
        {'backend': 'b', 'estimated_length_m': 11.0,
         'start_to_end_m': 2.0,
         'deviation_from_amcl': {'rms_m': 1.0, 'max_m': 2.0},
         'map': {'extent_m': [6.0, 3.0]}},
    ]

    report = render_report(records)

    assert '선택을 보류한다' in report
    assert '`None`' not in report
    assert 'None배' not in report


def _harness_source():
    return (Path(__file__).resolve().parents[1]
            / 'scripts' / 'offline_slam_replay.sh').read_text(encoding='utf-8')


def test_slam_toolbox_is_started_through_its_lifecycle_launch():
    """ros2 run leaves the Jazzy node unconfigured and silently mapless."""
    source = _harness_source()

    assert 'ros2 launch slam_toolbox online_async_launch.py' in source
    assert 'ros2 run slam_toolbox' not in source
    assert 'slam_params_file' in source


def test_cartographer_gflags_precede_ros_args():
    """Put gflags before --ros-args or cartographer_node exits at once."""
    # The failure is silent in the launch log except for one glog line:
    # "Check failed: !FLAGS_configuration_directory.empty()".
    source = _harness_source()
    invocation = source.split(
        'ros2 run cartographer_ros cartographer_node', 1)[1]
    invocation = invocation.split('&', 1)[0]

    assert invocation.index('-configuration_directory') < invocation.index(
        '--ros-args')


def test_replay_refuses_the_physical_domain():
    """Replaying on domain 12 would inject a map frame at the real robot."""
    source = _harness_source()

    assert 'ROS_DOMAIN_ID" = "12"' in source
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' in source


def test_replay_aborts_without_progress_evidence():
    """No long job may keep running once it has stopped producing."""
    source = _harness_source()

    assert 'STALL_LIMIT' in source
    assert 'STALL_LIMIT="${STALL_LIMIT:-240}"' in source
    assert 'kill -0 "$REPLAY_PID"' in source
    for reason in ('backend($probe) 가 죽었다',
                   '결과 기록이 ${STALL_LIMIT}초 동안 늘지 않았다'):
        assert reason in source


def test_cleanup_only_stops_process_groups_started_by_the_harness():
    """An offline experiment must not kill unrelated ROS processes."""
    source = _harness_source()

    assert 'pkill -f' not in source
    assert 'PROCESS_GROUPS=()' in source
    assert 'kill -TERM -- "-$process_group"' in source


def test_replay_requires_finalized_result_and_saved_map():
    """A backend comparison needs both a closed MCAP and final map files."""
    source = _harness_source()

    assert '최종 지도 저장 실패' in source
    assert '결과 MCAP metadata 마감 실패' in source
    assert '[ ! -s "$RESULT_DIR/metadata.yaml" ]' in source
    assert '--storage-config-file "$WRITER_CONFIG"' in source


def test_replay_detects_a_dead_result_recorder_immediately():
    """A dead recorder must stop replay without waiting for the stall timer."""
    source = _harness_source()

    assert 'kill -0 "$RECORDER_PID"' in source
    assert '결과 recorder 기동 실패' in source
    assert '감시: 결과 recorder가 죽었다' in source


def test_watchdog_uses_only_signals_that_cannot_time_out():
    """A watchdog that kills healthy runs is worse than none at all."""
    source = _harness_source()

    assert 'topic info /map' not in source
    assert 'FIRST_MAP_DEADLINE' not in source
