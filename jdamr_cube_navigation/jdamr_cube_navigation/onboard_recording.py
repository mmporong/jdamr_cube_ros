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
    """Return the smallest recorder queue depth that survives a 2 s stall."""
    if topic in LATCHED_TOPICS:
        return 1
    rate = NOMINAL_RATES_HZ[topic]
    return max(10, int(math.ceil(rate * MIN_BUFFER_SECONDS)))
