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
    reliability,
    required_depth,
)
from jdamr_cube_navigation.soak_metrics import (
    classify,
    cpu_percent,
    parse_proc_stat,
    parse_proc_status_rss_kb,
    read_loadavg,
    summarize,
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
        assert qos[topic]['reliability'] == reliability(topic), topic
        assert qos[topic]['durability'] == durability(topic), topic


def test_recorder_queue_absorbs_a_two_second_stall():
    """Require the queue to outlast the scheduling stalls measured on the Pi."""
    for topic in RECORDED_TOPICS:
        if topic == '/tf_static':
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
