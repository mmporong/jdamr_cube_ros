"""Regression tests for the onboard load and recording-loss gates."""

# The 2026-09-01 independent-process soak passed node survival but failed two
# gates: load1 stayed near 9.85 against a limit of 4.0, and the recorder lost
# three messages.  These tests pin the contract changes made in response and
# keep the soak sampler able to attribute load to a named process.

from pathlib import Path

from jdamr_cube_navigation.onboard_recording import (
    durability,
    NOMINAL_RATES_HZ,
    OFFERED_PROFILES,
    RECORDED_TOPICS,
    RECORDER_BEST_EFFORT,
    recorder_reliability,
    reliability,
    required_depth,
)
from jdamr_cube_navigation.soak_metrics import (
    classify,
    cpu_percent,
    evaluate_resource_gate,
    parse_proc_stat,
    parse_proc_status_rss_kb,
    percentile,
    read_loadavg,
    summarize,
    TSV_HEADER,
)

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMS_PATH = PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
QOS_PATH = PACKAGE_ROOT / 'evaluation' / 'qos_overrides.yaml'
LAUNCH_PATH = (
    PACKAGE_ROOT / 'launch' / 'onboard_keepout_navigation.launch.py')


def _costmap(name):
    config = yaml.safe_load(PARAMS_PATH.read_text(encoding='utf-8'))
    return config[name][name]['ros__parameters']


def test_costmaps_send_updates_instead_of_the_whole_grid():
    """Stop paying full-grid serialization for a view-only laptop."""
    for name in ('global_costmap', 'local_costmap'):
        assert _costmap(name)['always_send_full_costmap'] is False, name


def test_global_costmap_publishes_at_most_twice_per_second():
    """Keep the 894x212 saved-map grid off the per-second publish path."""
    global_costmap = _costmap('global_costmap')

    assert global_costmap['publish_frequency'] <= 0.5
    # The planning loop still needs a once-per-second obstacle refresh.
    assert global_costmap['update_frequency'] == 1.0


def test_every_recorded_topic_has_a_qos_override():
    """Close the default depth-10 gap that caused recorder transport loss."""
    qos = yaml.safe_load(QOS_PATH.read_text(encoding='utf-8'))

    assert set(qos) == set(NOMINAL_RATES_HZ)
    for topic in RECORDED_TOPICS:
        assert topic in qos, topic
        assert qos[topic]['history'] == 'keep_last', topic
        assert qos[topic]['depth'] == required_depth(topic), topic


def test_overrides_match_the_qos_each_publisher_offers():
    """Reject an override the publisher's offer would never match."""
    # /imu/data_raw is published best-effort on this robot.  A reliable
    # subscription would never match it and the IMU would silently vanish
    # from every future bag.
    qos = yaml.safe_load(QOS_PATH.read_text(encoding='utf-8'))

    assert OFFERED_PROFILES['/imu/data_raw'][0] == 'best_effort'
    assert OFFERED_PROFILES['/amcl_pose'][1] == 'transient_local'
    for topic in RECORDED_TOPICS:
        assert qos[topic]['reliability'] == recorder_reliability(topic), topic
        assert qos[topic]['durability'] == durability(topic), topic
        # A best-effort reader is always compatible with a reliable writer,
        # never the other way round.
        if recorder_reliability(topic) == 'reliable':
            assert reliability(topic) == 'reliable', topic


def test_reliable_recorder_queues_absorb_a_two_second_stall():
    """A reliable reader must buffer, since dropping is not an option there."""
    # Best-effort topics deliberately keep a shallow queue: they drop under
    # load rather than make a publisher wait, so buffering only ages the data.
    for topic in RECORDED_TOPICS:
        if topic == '/tf_static' or topic in RECORDER_BEST_EFFORT:
            continue
        assert required_depth(topic) >= NOMINAL_RATES_HZ[topic] * 2.0, topic


def test_latched_transforms_keep_the_transient_local_contract():
    """Do not inflate a latched topic that is replayed exactly once."""
    qos = yaml.safe_load(QOS_PATH.read_text(encoding='utf-8'))

    assert qos['/tf_static']['durability'] == 'transient_local'
    assert qos['/tf_static']['depth'] == 1
    assert required_depth('/tf_static') == 1


def test_launch_shares_the_recording_contract():
    """Keep the recorded topic list from drifting away from the QoS file."""
    source = LAUNCH_PATH.read_text(encoding='utf-8')

    assert 'from jdamr_cube_navigation.onboard_recording import' in source
    assert 'RECORDED_TOPICS = [' not in source
    assert '--disable-keyboard-controls' in source


def test_soak_sampler_labels_the_onboard_process_set():
    """Attribute load to a named Nav2 process instead of a system total."""
    assert classify('/opt/ros/jazzy/lib/nav2_amcl/amcl --ros-args') == 'amcl'
    assert classify(
        '/opt/ros/jazzy/lib/nav2_controller/controller_server'
    ) == 'controller_server'
    assert classify(
        '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 bag record --storage mcap'
    ) == 'recorder'
    assert classify('/usr/lib/systemd/systemd --user') == 'other'


def test_soak_sampler_reads_process_accounting():
    """Parse jiffies and thread counts past an executable name with spaces."""
    stat = (
        '4242 (ros2 bag record) S 1 4242 4242 0 -1 4194304 900 0 0 0 '
        '150 60 0 0 20 0 12 0 99 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0'
    )
    jiffies, threads = parse_proc_stat(stat)

    assert jiffies == 210
    assert threads == 12
    assert parse_proc_status_rss_kb('Name:\tamcl\nVmRSS:\t 51200 kB\n') == 51200
    assert read_loadavg('9.85 7.56 4.13 3/390 6127') == ('9.85', '7.56', '4.13')


def test_soak_sampler_converts_jiffies_to_core_percent():
    """One fully busy core over the sample window must read as 100 percent."""
    assert cpu_percent(500, 5.0, 100) == 100.0
    assert cpu_percent(250, 5.0, 100) == 50.0
    assert cpu_percent(500, 0.0, 100) == 0.0


def test_soak_summary_ranks_the_hottest_process_first():
    """Make the load owner the first line an operator reads."""
    summary = summarize([
        {'label': 'amcl', 'cpu_pct': 10.0, 'threads': 4},
        {'label': 'amcl', 'cpu_pct': 20.0, 'threads': 6},
        {'label': 'controller_server', 'cpu_pct': 80.0, 'threads': 8},
    ])

    assert summary[0]['label'] == 'controller_server'
    assert summary[0]['mean_cpu_pct'] == 80.0
    assert summary[1]['label'] == 'amcl'
    assert summary[1]['mean_cpu_pct'] == 15.0
    assert summary[1]['max_threads'] == 6


def test_the_gate_scores_sustained_load_not_the_startup_spike():
    """Bringing twelve processes up at once must not block the drive."""
    # 2026-09-03 shape: one 296% configure spike over a flat 203% plateau.
    samples = [{'cpu_pct': 296.0, 'temp_c': 66.0, 'throttled': '0x0'}]
    samples += [{'cpu_pct': 203.0, 'temp_c': 69.0, 'throttled': '0x0'}
                for _ in range(66)]
    verdict = evaluate_resource_gate([], samples, core_count=4)

    assert verdict['sustained_cpu_pct'] == 203.0
    assert verdict['peak_cpu_pct'] == 296.0
    assert verdict['gates']['cpu_headroom'] == 'PASS'
    assert verdict['verdict'] == 'PASS'


def test_a_genuinely_saturated_pi_still_fails():
    """Relaxing the gate must not make it unable to fail."""
    samples = [{'cpu_pct': 380.0, 'temp_c': 70.0, 'throttled': '0x0'}
               for _ in range(20)]
    verdict = evaluate_resource_gate([], samples, core_count=4)

    assert verdict['gates']['cpu_headroom'] == 'FAIL'
    assert verdict['verdict'] == 'FAIL'


def test_thermal_throttling_fails_regardless_of_cpu():
    """A throttled Pi is slow no matter what the CPU percentage says."""
    samples = [{'cpu_pct': 100.0, 'temp_c': 70.0, 'throttled': '0x50005'}
               for _ in range(10)]
    verdict = evaluate_resource_gate([], samples, core_count=4)

    assert verdict['gates']['thermal_throttle'] == 'FAIL'
    assert verdict['verdict'] == 'FAIL'


def test_hot_but_unthrottled_still_fails_on_margin():
    """Stop before the 80C soft-throttle rather than after it."""
    samples = [{'cpu_pct': 100.0, 'temp_c': 78.0, 'throttled': '0x0'}
               for _ in range(10)]

    assert evaluate_resource_gate(
        [], samples, core_count=4)['gates']['temperature'] == 'FAIL'


def test_missing_temperature_is_unknown_not_pass():
    """Never let an unmeasured thermal state read as a passing one."""
    samples = [{'cpu_pct': 100.0, 'temp_c': None, 'throttled': '0x0'}
               for _ in range(10)]
    verdict = evaluate_resource_gate([], samples, core_count=4)

    assert verdict['gates']['temperature'] == 'UNKNOWN'
    assert verdict['verdict'] == 'UNKNOWN'


def test_load_average_is_recorded_but_does_not_decide():
    """load1 counts threads waiting on I/O, so it cannot gate the drive."""
    assert 'load1' in TSV_HEADER
    samples = [{'cpu_pct': 203.0, 'temp_c': 69.0, 'throttled': '0x0'}
               for _ in range(20)]

    assert 'load' not in ' '.join(
        evaluate_resource_gate([], samples, core_count=4)['gates'])


def test_percentile_picks_the_sustained_value():
    """The gate metric must ignore a single outlier sample."""
    assert percentile([203.0] * 9 + [296.0], 0.90) == 296.0
    assert percentile([203.0] * 19 + [296.0], 0.90) == 203.0
    assert percentile([], 0.90) == 0.0


def test_recording_cannot_back_pressure_the_control_loop():
    """Logging must never block a publisher the robot depends on."""
    # 2026-09-04: with the recorder subscribing reliably to /tf, /odom and
    # /scan, a real drive stalled with CPU at 61-146% of 400% and load 2.8-6.1
    # -- resources to spare.  The control loop fell from 10 Hz to 1.7 Hz, the
    # lifecycle heartbeats stopped, and nine Nav2 nodes left the graph at once.
    # The process was blocked, not busy.
    for topic in ('/scan', '/odom', '/tf', '/imu/data_raw'):
        assert topic in RECORDER_BEST_EFFORT, topic
        assert recorder_reliability(topic) == 'best_effort', topic
        # Dropping is the point, so a deep queue would only hold stale data.
        assert required_depth(topic) <= NOMINAL_RATES_HZ[topic], topic

    # Latched transforms arrive once and must not be dropped.
    assert '/tf_static' not in RECORDER_BEST_EFFORT
    assert recorder_reliability('/tf_static') == 'reliable'
    # Low-rate evidence carries no back-pressure risk and stays reliable.
    for topic in ('/cmd_vel', '/amcl_pose', '/battery_state', '/plan'):
        assert recorder_reliability(topic) == 'reliable', topic
