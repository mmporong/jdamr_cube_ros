"""읽기 전용 센서 지연 진단의 순수 통계를 검증한다."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location(
    'live_latency_probe', Path(__file__).parents[1]
    / 'evaluation/probe_live_sensor_latency.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def sample(received, age, stamp=100.0):
    """Build one raw timing sample."""
    return {
        'recv_offset_s': received,
        'header_stamp_s': stamp,
        'age_s': age,
    }


def test_missing_stream_has_no_invented_frequency():
    """An absent stream cannot claim a rate."""
    assert module.summarize([]) == {'count': 0}


def test_arrival_frequency_age_and_gap_thresholds_are_independent():
    """Arrival gaps and header age remain separate measurements."""
    report = module.summarize([
        sample(0, 1.5), sample(.1, 1.4), sample(.35, 1.6),
        sample(.95, 1.7), sample(2.05, 1.8),
    ])
    assert report['hz'] == pytest.approx(4 / 2.05)
    assert report['age_p50_s'] == 1.6
    assert report['age_p95_s'] == 1.8
    assert report['gap_p50_s'] == pytest.approx(.25)
    assert report['gap_p95_s'] == pytest.approx(1.1)
    assert report['gap_over_0_2_s'] == 3
    assert report['gap_over_0_5_s'] == 2
    assert report['gap_over_1_0_s'] == 1


def test_single_sample_does_not_claim_continuity():
    """One sample cannot establish continuity."""
    report = module.summarize([sample(1, .02)])
    assert report['hz'] is None
    assert report['gap_p50_s'] is None
    assert report['gap_p95_s'] is None
    assert report['gap_max_s'] is None


def test_stream_report_exposes_bounded_buffer_truncation():
    """Retained window and dropped count disclose buffer truncation."""
    report = module.stream_report([sample(2, .1), sample(3, .1)], 5)
    assert report['received_count'] == 5
    assert report['retained_count'] == 2
    assert report['dropped_count'] == 3
    assert report['retained_window_s'] == 1


def test_future_dated_tf_is_reported_without_clamping():
    """Future-dated TF remains visible in the report."""
    assert module.summarize([
        sample(0, -1), sample(.1, -.9)])['age_min_s'] == -1


def test_map_odom_alignment_subtracts_declared_transform_tolerance():
    """The declared AMCL future tolerance is removed before matching."""
    transforms = [sample(0, -.9, 101.00), sample(.1, -.9, 102.03)]
    report = module.alignment_report(
        [100.0, 101.0], transforms, tolerance_s=1.0,
        epsilon_s=.05, raw_limit=10)
    assert report['transform_tolerance_s'] == 1.0
    assert report['matched_count'] == 2
    assert report['abs_delta_max_s'] == pytest.approx(.03)
    assert report['raw_samples'][1]['adjusted_stamp_s'] == pytest.approx(
        101.03)


def test_alignment_raw_samples_are_bounded():
    """Alignment evidence respects its raw-sample bound."""
    transforms = [sample(index, 0, 101.0 + index) for index in range(5)]
    report = module.alignment_report(
        [100.0 + index for index in range(5)], transforms,
        tolerance_s=1.0, epsilon_s=.01, raw_limit=2)
    assert report['count'] == 2
    assert len(report['raw_samples']) == 2


def test_stationary_report_allows_small_odom_noise():
    """Small odometry noise does not invalidate a stationary run."""
    odom = [
        {'x_m': 0.0, 'y_m': 0.0, 'speed_mps': .004,
         'yaw_rate_rps': .008},
        {'x_m': .009, 'y_m': -.004, 'speed_mps': -.006,
         'yaw_rate_rps': -.01},
    ]
    report = module.stationary_report(odom, .02, .05, .03)
    assert report['valid_stationary'] is True
    assert '측정 노이즈로 허용' in report['noise_tolerance_note']


@pytest.mark.parametrize(
    'field,value', [('speed_mps', .021), ('yaw_rate_rps', .051)])
def test_stationary_report_rejects_velocity_above_threshold(field, value):
    """Measured base velocity above either limit invalidates the run."""
    moving = {'x_m': 0.0, 'y_m': 0.0, 'speed_mps': 0.0,
              'yaw_rate_rps': 0.0}
    moving[field] = value
    report = module.stationary_report([moving], .02, .05, .03)
    assert report['valid_stationary'] is False


def test_stationary_report_rejects_displacement_above_threshold():
    """Net displacement above the noise bound invalidates the run."""
    odom = [
        {'x_m': 0.0, 'y_m': 0.0, 'speed_mps': 0.0,
         'yaw_rate_rps': 0.0},
        {'x_m': .031, 'y_m': 0.0, 'speed_mps': 0.0,
         'yaw_rate_rps': 0.0},
    ]
    assert module.stationary_report(odom, .02, .05, .03)[
        'valid_stationary'] is False


def test_missing_cpu_frequency_is_safe(tmp_path):
    """Missing cpufreq sysfs entries are represented as null."""
    report = module.system_sample(.5, sysfs_root=tmp_path)
    assert report['recv_offset_s'] == .5
    assert report['cpu_frequency_khz'] is None


def test_cpu_usage_uses_proc_stat_delta(tmp_path):
    """Short A/B samples use /proc/stat deltas instead of load alone."""
    proc_stat = tmp_path / 'stat'
    proc_stat.write_text('cpu  10 0 10 80 0 0 0 0 0 0\n', encoding='utf-8')
    first = module.system_sample(
        0, sysfs_root=tmp_path, proc_stat_path=proc_stat)
    proc_stat.write_text('cpu  20 0 20 100 0 0 0 0 0 0\n', encoding='utf-8')
    second = module.system_sample(
        1, sysfs_root=tmp_path,
        previous_cpu_ticks=(first['cpu_total_ticks'], first['cpu_idle_ticks']),
        proc_stat_path=proc_stat)
    assert second['cpu_used_percent'] == pytest.approx(50)


def test_amcl_prior_serialization_keeps_pose_covariance_and_stamp():
    """The optional restart prior retains all needed AMCL fields."""
    vector = SimpleNamespace(x=1.2, y=-.3, z=0.0)
    orientation = SimpleNamespace(x=0.0, y=0.0, z=.2, w=.98)
    pose = SimpleNamespace(position=vector, orientation=orientation)
    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=500_000_000),
            frame_id='map'),
        pose=SimpleNamespace(pose=pose, covariance=list(range(36))),
    )
    record = module.amcl_pose_record(message)
    assert record['header_stamp_s'] == 12.5
    assert record['position']['x'] == 1.2
    assert record['covariance'] == list(range(36))
    assert 'not localization proof' in record['purpose']
