"""Regression tests for the onboard load and recording-loss gates."""

# These tests keep the recording contract and resource gate executable.  Run
# measurements stay in evaluation reports so comments cannot become a second,
# stale source of truth.

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
import jdamr_cube_navigation.soak_metrics as soak_metrics
from jdamr_cube_navigation.soak_metrics import (
    classify,
    cpu_percent,
    evaluate_resource_gate,
    format_row,
    parse_proc_stat,
    parse_proc_status_rss_kb,
    parse_system_cpu_stat,
    percentile,
    process_coverage,
    process_mode,
    read_loadavg,
    select_recent_window,
    summarize,
    system_cpu_percent,
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


def _composed_process_rows(sample_count=13):
    rows = []
    for index in range(sample_count):
        sample = str(index + 1)
        elapsed_s = 60.0 * index / max(1, sample_count - 1)
        rows.extend((
            {
                'sample': sample,
                'elapsed_s': elapsed_s,
                'label': 'nav2_container',
                'cpu_pct': 0.0,
                'threads': 1,
                'command': (
                    'component_container_isolated '
                    '__node:=nav2_container'),
            },
            {
                'sample': sample,
                'elapsed_s': elapsed_s,
                'label': 'recorder',
                'cpu_pct': 0.0,
                'threads': 1,
                'command': 'ros2 bag record --topics /scan',
            },
        ))
    return rows


def _resource_samples(cpu_values, temperature_c=69.0, throttled='0x0'):
    values = list(cpu_values)
    return [
        {
            'sample': str(index + 1),
            'elapsed_s': 60.0 * index / max(1, len(values) - 1),
            'system_cpu_pct': value,
            'cpu_pct': 0.0,
            'temp_c': temperature_c,
            'throttled': throttled,
        }
        for index, value in enumerate(values)
    ]


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


def test_hidden_goal_status_is_recorded_with_latched_action_qos():
    """Preserve goal identity without imposing reliable QoS on sensor streams."""
    topic = '/navigate_to_pose/_action/status'
    qos = yaml.safe_load(QOS_PATH.read_text(encoding='utf-8'))
    assert topic in RECORDED_TOPICS
    assert qos[topic] == {
        'reliability': 'reliable', 'durability': 'transient_local',
        'history': 'keep_last', 'depth': 1,
    }
    launch = (PACKAGE_ROOT / 'launch' / 'onboard_keepout_navigation.launch.py')
    assert '--include-hidden-topics' in launch.read_text(encoding='utf-8')


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
    assert classify(
        '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 bag record '
        '--topics /scan /collision_monitor_state'
    ) == 'recorder'
    assert classify(
        '/opt/ros/jazzy/lib/rclcpp_components/component_container_isolated '
        '--ros-args -r __node:=nav2_container'
    ) == 'nav2_container'
    assert classify(
        '/opt/ros/jazzy/lib/rclcpp_components/component_container_isolated '
        '--ros-args -r __node:=camera_container'
    ) == 'other'
    assert classify(
        '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 run '
        'jdamr_cube_navigation corridor_route --route route.yaml'
    ) == 'corridor_route'
    assert classify(
        '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch '
        'jdamr_cube_navigation onboard_keepout_navigation.launch.py'
    ) == 'nav2_launch'
    assert classify('/usr/bin/other --topic /amcl') == 'other'
    assert classify('/usr/lib/systemd/systemd --user') == 'other'


def test_resource_gate_requires_the_nav2_process_set_to_be_measured():
    """Never report spare CPU after omitting the composed Nav2 container."""
    split = []
    for index in range(13):
        sample = str(index + 1)
        elapsed_s = float(index * 5)
        split.extend({
            'sample': sample,
            'elapsed_s': elapsed_s,
            'label': label,
            'cpu_pct': 0.0,
            'threads': 1,
            'command': label,
        } for label in (
            'map_server', 'amcl', 'controller_server', 'planner_server',
            'collision_monitor', 'bt_navigator', 'recorder'))

    assert process_mode(_composed_process_rows()) == 'composed'
    assert process_mode(split) == 'split'
    assert process_mode([{'label': 'recorder'}]) is None
    assert process_mode([
        {key: value for key, value in row.items() if key != 'command'}
        for row in split
    ]) is None
    verdict = evaluate_resource_gate(
        [{'label': 'recorder', 'cpu_pct': 20.0, 'threads': 10}],
        [{'system_cpu_pct': 20.0, 'temp_c': 60.0,
          'throttled': '0x0'}],
        core_count=4,
    )
    assert verdict['gates']['process_coverage'] == 'FAIL'
    assert verdict['verdict'] == 'FAIL'


def test_resource_coverage_is_simultaneous_and_continuous_after_warmup():
    """Never combine labels from different times into a false PASS."""
    warmup_then_complete = [
        {
            'sample': '0', 'elapsed_s': -5.0,
            'label': 'recorder', 'command': 'ros2 bag record',
        },
        *_composed_process_rows(),
    ]
    disappears = [
        *_composed_process_rows(),
        {
            'sample': '14', 'elapsed_s': 65.0,
            'label': 'recorder', 'command': 'ros2 bag record',
        },
    ]
    one_sample = [
        row for row in _composed_process_rows() if row['sample'] == '1'
    ]
    distributed_split = [
        {
            'sample': str(index), 'elapsed_s': float(index * 10),
            'label': label, 'command': label,
        }
        for index, label in enumerate((
            'map_server', 'amcl', 'controller_server', 'planner_server',
            'collision_monitor', 'bt_navigator', 'recorder'))
    ]

    assert process_mode(warmup_then_complete) == 'composed'
    assert process_mode(disappears) is None
    assert process_mode(one_sample) is None
    assert process_mode(distributed_split) is None
    coverage = process_coverage(disappears)
    assert coverage['mode'] == 'incomplete'
    assert coverage['complete'] == 13
    assert coverage['checked'] == 14
    assert coverage['duration_s'] == 65.0
    assert coverage['max_gap_s'] == 5.0
    assert coverage['sample_ids'] == tuple(str(index) for index in range(1, 15))


def test_resource_coverage_rejects_a_sixty_second_observation_gap():
    """Two distant samples are not evidence of continuous process health."""
    sparse = _composed_process_rows(sample_count=2)

    coverage = process_coverage(sparse)

    assert coverage['mode'] == 'incomplete'
    assert coverage['duration_s'] == 60.0
    assert coverage['max_gap_s'] == 60.0
    assert coverage['cadence_valid'] is False


def test_resource_coverage_records_a_tick_when_every_process_disappears():
    """A missing interval must be evidence of loss, not a missing TSV row."""
    rows = [
        *_composed_process_rows(),
        {
            'sample': '14',
            'elapsed_s': 65.0,
            'label': 'sample_sentinel',
            'command': 'soak_metrics internal sample sentinel',
        },
    ]

    coverage = process_coverage(rows)

    assert coverage['mode'] == 'incomplete'
    assert coverage['complete'] == 13
    assert coverage['checked'] == 14


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


def test_soak_rows_keep_the_command_that_explains_the_label():
    """Make every future process classification auditable from the TSV."""
    row = {
        'label': 'recorder',
        'pid': 42,
        'cpu_pct': 12.5,
        'threads': 3,
        'rss_mb': 10.0,
        'command': 'ros2 bag record --topics /collision_monitor_state',
    }
    rendered = format_row(
        1, 'now', row, ('1.0', '1.0', '1.0'), '60.0', '0x0')

    assert TSV_HEADER.endswith('\tcommand')
    assert len(rendered.split('\t')) == len(TSV_HEADER.split('\t'))
    assert rendered.split('\t')[2:4] == ['', '']
    assert rendered.split('\t')[-1] == row['command']


def test_soak_sampler_writes_an_empty_process_tick(tmp_path, monkeypatch):
    """Keep total process loss visible in the persisted evidence."""
    output = tmp_path / 'empty_tick.tsv'
    monkeypatch.setattr(
        soak_metrics, 'sample_processes', lambda *_args: ([], {}))
    monkeypatch.setattr(soak_metrics.time, 'monotonic', lambda: 0.0)

    result = soak_metrics.main([
        '--output', str(output), '--duration', '0', '--cores', '4',
    ])

    lines = output.read_text(encoding='utf-8').splitlines()
    assert result == 1
    assert len(lines) == 2
    fields = lines[1].split('\t')
    assert len(fields) == len(TSV_HEADER.split('\t'))
    assert fields[4] == 'sample_sentinel'
    assert fields[-1] == 'soak_metrics internal sample sentinel'


def test_soak_sampler_converts_jiffies_to_core_percent():
    """One fully busy core over the sample window must read as 100 percent."""
    assert cpu_percent(500, 5.0, 100) == 100.0
    assert cpu_percent(250, 5.0, 100) == 50.0
    assert cpu_percent(500, 0.0, 100) == 0.0


def test_soak_sampler_converts_system_jiffies_to_all_core_percent():
    """Whole-system usage must use the same scale as the multicore budget."""
    stat = 'cpu  100 10 40 800 50 0 0 0 0 0\ncpu0 1 1 1 1 1 1 1 1'

    assert parse_system_cpu_stat(stat) == (150, 1000)
    assert system_cpu_percent((100, 1000), (300, 1400), 4) == 200.0
    assert system_cpu_percent((300, 1400), (200, 1500), 4) is None
    assert system_cpu_percent((100, 1400), (300, 1500), 4) is None


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
    samples = _resource_samples([296.0] + [203.0] * 66)
    verdict = evaluate_resource_gate(
        _composed_process_rows(len(samples)), samples, core_count=4)

    assert verdict['sustained_cpu_pct'] == 203.0
    assert verdict['peak_cpu_pct'] == 296.0
    assert verdict['gates']['cpu_headroom'] == 'PASS'
    assert verdict['verdict'] == 'PASS'


def test_recent_window_excludes_startup_samples_and_keeps_full_duration():
    """Use the settled suffix while retaining at least sixty seconds."""
    samples = _resource_samples([380.0, 370.0] + [203.0] * 13)
    rows = _composed_process_rows(len(samples))
    for index, sample in enumerate(samples):
        sample['elapsed_s'] = 5.0 * index
    for row in rows:
        row['elapsed_s'] = 5.0 * (int(row['sample']) - 1)

    recent_rows, recent_samples = select_recent_window(
        rows, samples, window_seconds=60.0)
    verdict = evaluate_resource_gate(
        recent_rows, recent_samples, core_count=4)

    assert len(recent_samples) == 13
    assert verdict['coverage_duration_s'] == 60.0
    assert verdict['sustained_cpu_pct'] == 203.0
    assert verdict['verdict'] == 'PASS'


def test_recent_window_remains_incomplete_when_less_than_sixty_seconds_exist():
    """A short startup trace must not become valid by window selection."""
    samples = _resource_samples([203.0] * 10)
    rows = _composed_process_rows(len(samples))
    for index, sample in enumerate(samples):
        sample['elapsed_s'] = 5.0 * index
    for row in rows:
        row['elapsed_s'] = 5.0 * (int(row['sample']) - 1)

    recent_rows, recent_samples = select_recent_window(
        rows, samples, window_seconds=60.0)
    verdict = evaluate_resource_gate(
        recent_rows, recent_samples, core_count=4)

    assert verdict['coverage_duration_s'] == 45.0
    assert verdict['gates']['process_coverage'] == 'FAIL'
    assert verdict['verdict'] == 'FAIL'


def test_window_seconds_requires_evaluation_mode(tmp_path):
    """Reject a window option that cannot affect sampling mode."""
    try:
        soak_metrics.main([
            '--output', str(tmp_path / 'samples.tsv'),
            '--window-seconds', '60',
        ])
    except SystemExit:
        pass
    else:
        raise AssertionError('window option was accepted without --evaluate')


def test_a_genuinely_saturated_pi_still_fails():
    """Relaxing the gate must not make it unable to fail."""
    samples = _resource_samples([380.0] * 20, temperature_c=70.0)
    verdict = evaluate_resource_gate(
        _composed_process_rows(len(samples)), samples, core_count=4)

    assert verdict['gates']['cpu_headroom'] == 'FAIL'
    assert verdict['verdict'] == 'FAIL'


def test_thermal_throttling_fails_regardless_of_cpu():
    """A throttled Pi is slow no matter what the CPU percentage says."""
    samples = _resource_samples(
        [100.0] * 10, temperature_c=70.0, throttled='0x50005')
    verdict = evaluate_resource_gate(
        _composed_process_rows(len(samples)), samples, core_count=4)

    assert verdict['gates']['thermal_throttle'] == 'FAIL'
    assert verdict['verdict'] == 'FAIL'


def test_hot_but_unthrottled_still_fails_on_margin():
    """Stop before the 80C soft-throttle rather than after it."""
    samples = _resource_samples([100.0] * 10, temperature_c=78.0)

    assert evaluate_resource_gate(
        _composed_process_rows(len(samples)), samples,
        core_count=4)['gates']['temperature'] == 'FAIL'


def test_missing_temperature_is_unknown_not_pass():
    """Never let an unmeasured thermal state read as a passing one."""
    samples = _resource_samples([100.0] * 10, temperature_c=None)
    verdict = evaluate_resource_gate(
        _composed_process_rows(len(samples)), samples, core_count=4)

    assert verdict['gates']['temperature'] == 'UNKNOWN'
    assert verdict['verdict'] == 'UNKNOWN'


def test_missing_cpu_or_throttle_samples_are_unknown_not_pass():
    """An absent resource channel cannot prove safe operating headroom."""
    empty = evaluate_resource_gate(
        _composed_process_rows(), [], core_count=4)
    no_throttle = evaluate_resource_gate(
        _composed_process_rows(),
        _resource_samples([100.0] * 13, temperature_c=60.0, throttled=''),
        core_count=4,
    )

    assert empty['gates']['cpu_headroom'] == 'UNKNOWN'
    assert empty['gates']['thermal_throttle'] == 'UNKNOWN'
    assert empty['verdict'] == 'UNKNOWN'
    assert no_throttle['gates']['cpu_headroom'] == 'PASS'
    assert no_throttle['gates']['thermal_throttle'] == 'UNKNOWN'
    assert no_throttle['verdict'] == 'UNKNOWN'


def test_load_average_is_recorded_but_does_not_decide():
    """load1 counts threads waiting on I/O, so it cannot gate the drive."""
    assert 'load1' in TSV_HEADER
    samples = _resource_samples([203.0] * 20)

    assert 'load' not in ' '.join(
        evaluate_resource_gate(
            _composed_process_rows(len(samples)), samples,
            core_count=4)['gates'])


def test_resource_gate_excludes_warmup_and_uses_whole_system_cpu():
    """Low startup load and partial process sums cannot hide saturation."""
    rows = [
        {
            'sample': '0', 'elapsed_s': -5.0,
            'label': 'recorder', 'cpu_pct': 1.0, 'threads': 1,
            'command': 'ros2 bag record',
        },
        *_composed_process_rows(),
    ]
    samples = [
        {
            'sample': '0', 'system_cpu_pct': 1.0, 'cpu_pct': 1.0,
            'temp_c': 60.0, 'throttled': '0x0',
        },
        {
            'sample': '1', 'system_cpu_pct': 380.0, 'cpu_pct': 5.0,
            'temp_c': 60.0, 'throttled': '0x0',
        },
        {
            'sample': '2', 'system_cpu_pct': 380.0, 'cpu_pct': 5.0,
            'temp_c': 60.0, 'throttled': '0x0',
        },
    ]

    verdict = evaluate_resource_gate(rows, samples, core_count=4)

    assert verdict['mean_cpu_pct'] == 380.0
    assert verdict['gates']['cpu_headroom'] == 'FAIL'


def test_percentile_picks_the_sustained_value():
    """The gate metric must ignore a single outlier sample."""
    assert percentile([203.0] * 9 + [296.0], 0.90) == 296.0
    assert percentile([203.0] * 19 + [296.0], 0.90) == 203.0
    assert percentile([], 0.90) == 0.0


def test_high_rate_recording_uses_loss_tolerant_subscriptions():
    """Keep diagnostic recording from demanding reliable sensor delivery."""
    # Recorder back-pressure is still a hypothesis.  Best-effort remains the
    # less intrusive diagnostic policy and needs a controlled run for proof.
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
