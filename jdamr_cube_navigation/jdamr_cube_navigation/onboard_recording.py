"""Single source of truth for the onboard protocol recording contract."""

import math


# Keep only mapping, localization, command, and safety evidence.  RViz-only
# state such as joint_states stays off the Pi recorder by default.
RECORDED_TOPICS = [
    '/scan',
    '/odom',
    '/tf',
    '/tf_static',
    '/imu/data_raw',
    '/cmd_vel',
    '/cmd_vel_nav',
    '/amcl_pose',
    '/battery_state',
    '/plan',
    '/collision_monitor_state',
]

# Publication rates and the QoS each publisher actually offers, read from
# corridor_keepout_roundtrip_20260901T150446, onboard_minimal_soak_20260901T161338
# and static_repeat_stop_fix_20260901T174200.  A recorder override is not a
# request: a reliable subscription never matches a best-effort publisher, so an
# override that disagrees with the offer records nothing at all.
OFFERED_PROFILES = {
    '/scan': ('reliable', 'volatile', 10.0),
    '/odom': ('reliable', 'volatile', 50.0),
    '/tf': ('reliable', 'volatile', 100.0),
    '/tf_static': ('reliable', 'transient_local', 0.0),
    '/imu/data_raw': ('best_effort', 'volatile', 50.0),
    '/cmd_vel': ('reliable', 'volatile', 20.0),
    '/cmd_vel_nav': ('reliable', 'volatile', 20.0),
    '/amcl_pose': ('reliable', 'transient_local', 2.0),
    '/battery_state': ('reliable', 'volatile', 2.0),
    '/plan': ('reliable', 'volatile', 2.0),
    '/collision_monitor_state': ('reliable', 'volatile', 10.0),
    '/joint_states': ('reliable', 'volatile', 20.0),
}

# 2026-09-04: 기록은 제어 경로에 역압을 주면 안 된다.
#
# 기록기가 고주기 토픽을 reliable 로 구독하면, 기록기가 밀릴 때 발행자의
# write 가 막힌다. 실주행에서 CPU 61~146%(400% 중), load 2.8~6.1 로 자원이
# 남는데도 컨트롤 루프가 10Hz -> 1.7Hz 로 떨어지고 lifecycle heartbeat 가
# 끊겨 노드 9개가 한꺼번에 사라졌다. 바빠서가 아니라 막혀서다.
#
# 고주기 센서·TF 는 best_effort 로 받는다. 부하가 걸리면 기록기가 몇 건을
# 잃되 로봇은 계속 달린다. 저주기 토픽은 역압 위험이 없어 reliable 을
# 유지하고, /tf_static 은 한 번만 오는 latched 라 반드시 reliable 이어야 한다.
RECORDER_BEST_EFFORT = {
    '/scan',
    '/odom',
    '/tf',
    '/imu/data_raw',
    '/joint_states',
}


def recorder_reliability(topic):
    """Return the reliability the recorder subscribes with."""
    if topic in RECORDER_BEST_EFFORT:
        return 'best_effort'
    return OFFERED_PROFILES[topic][0]


NOMINAL_RATES_HZ = {
    topic: profile[2] for topic, profile in OFFERED_PROFILES.items()
}

# The 2026-09-01 independent-process soak lost 3 messages while load1 reached
# 9.85.  Two seconds of queue covers the scheduling stalls measured there
# without letting the recorder hold stale data across a run.
MIN_BUFFER_SECONDS = 2.0

# Latched transforms are replayed once, so depth stays at the transient-local
# contract rather than the rate-derived buffer.
LATCHED_TOPICS = {'/tf_static'}


def reliability(topic):
    """Return the reliability the publisher offers for *topic*."""
    return OFFERED_PROFILES[topic][0]


def durability(topic):
    """Return the durability the publisher offers for *topic*."""
    return OFFERED_PROFILES[topic][1]


def required_depth(topic):
    """Return the recorder queue depth for *topic*."""
    if topic in LATCHED_TOPICS:
        return 1
    if topic in RECORDER_BEST_EFFORT:
        # A best-effort reader drops instead of blocking, so a deep queue only
        # holds stale data.  Half a second of buffer is enough.
        return max(10, int(math.ceil(NOMINAL_RATES_HZ[topic] * 0.5)))
    rate = NOMINAL_RATES_HZ[topic]
    return max(10, int(math.ceil(rate * MIN_BUFFER_SECONDS)))
