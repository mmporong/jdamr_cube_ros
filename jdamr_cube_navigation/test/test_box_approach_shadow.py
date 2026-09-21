"""Verify the non-actuating adapter and explicit interlock failures."""

import json
from pathlib import Path
from types import SimpleNamespace

import jdamr_cube_navigation.box_approach_shadow as shadow_module
from jdamr_cube_navigation.box_approach_shadow import (
    BoxApproachShadow, guard_reasons, lower_base_geometry, pose_at,
)
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.context import Context
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


def safe_inputs():
    """Return synthetic independent guard evidence, not hardware approval."""
    channels = ('perception', 'odom', 'scan', 'battery', 'collision', 'command')
    return {'observation': {'control_ready': True, 'frame_id': 'camera_color_optical_frame'},
            'received': dict.fromkeys(channels, 1.),
            'now_s': 1.1, 'battery_status': BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
            'collision_action': 0, 'final_owners': [('collision_monitor', '/')],
            'input_owners': []}


@pytest.mark.parametrize('key,value,reason', [
    ('battery_status', BatteryState.POWER_SUPPLY_STATUS_CHARGING, 'charging_or_unknown'),
    ('battery_status', None, 'charging_or_unknown'),
    ('collision_action', 1, 'collision_not_clear'),
    ('final_owners', [('other', '/')], 'command_owner_conflict'),
    ('input_owners', [('velocity_smoother', '/')], 'command_owner_conflict'),
    ('observation', {'control_ready': False}, 'perception_not_approved'),
])
def test_guard_rejects_unsafe_or_unapproved_inputs(key, value, reason):
    """Existing controllers must relinquish command ownership before any handoff."""
    args = safe_inputs()
    args[key] = value
    assert reason in guard_reasons(**args)


def test_guard_requires_fresh_inputs():
    """Callbacks must be recent for every safety channel."""
    args = safe_inputs()
    assert guard_reasons(**args) == []
    args['received']['scan'] = 0.
    assert 'scan_stale' in guard_reasons(**args)


def test_image_pose_is_interpolated_and_never_extrapolated():
    """Delayed image geometry uses capture-time odometry, not current pose."""
    history = [(1., (0., 0., 0.)), (1.2, (.02, -.01, .1))]
    assert pose_at(history, 1.1) == pytest.approx((.01, -.005, .05))
    with pytest.raises(ValueError):
        pose_at(history, 1.3)


def test_ros_shadow_has_no_actuator_publishers():
    """Construct the real ROS node and run a missing-input timer safely."""
    context = Context()
    rclpy.init(context=context)
    node = BoxApproachShadow(context=context)
    try:
        node.tick()
        topics = [name for name, _ in node.get_publisher_names_and_types_by_node(
            node.get_name(), node.get_namespace())]
        assert '/box_parking/approach_status' in topics
        assert not any(name.startswith('/cmd_vel') for name in topics)
        assert node.policy.state == 'WAITING'
        malformed = String(data='[]')
        node.on_observation(malformed)
        assert node.observation == {}
        node.tick()
        assert node.policy.goal is None
    finally:
        node.destroy_node()
        context.shutdown()


def test_launch_is_observation_only():
    """The installed launch cannot start Nav2 or a base driver."""
    source = (Path(__file__).resolve().parents[1]
              / 'launch/box_approach_shadow.launch.py').read_text()
    assert 'cmd_vel' not in source
    assert 'base_driver' not in source
    assert 'navigate_to_pose' not in source


def test_geometry_scope_cannot_silently_change(tmp_path):
    """A whole-robot interpretation cannot be inferred from chassis measurements."""
    path = tmp_path / 'geometry.yaml'
    path.write_text('claim_scope: unknown\n')
    with pytest.raises(ValueError, match='geometry_scope'):
        lower_base_geometry(path)


def test_shadow_acquires_real_perception_contract_without_authorizing_motion(monkeypatch):
    """An unapproved observer can exercise the proposal path, never actuation."""
    context = Context()
    rclpy.init(context=context)
    node = BoxApproachShadow(context=context)
    captured = []
    node.status = SimpleNamespace(
        publish=lambda message: captured.append(json.loads(message.data)))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_100_000_000))
    try:
        for stamp_ns in (10_000_000_000, 10_100_000_000):
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            odom.pose.pose.orientation.w = 1.
            node.on_odom(odom)
        node.on_observation(String(data=json.dumps({
            'stamp_s': 10.05, 'frame_id': 'camera_color_optical_frame',
            'detected': True, 'stable': True, 'control_ready': False,
            'surface_kind': 'front', 'confidence': .95,
            'front_distance_m': 1.1, 'lateral_error_m': 0., 'edge_angle_deg': 0.,
        })))
        node.tick()
        status = captured[-1]
        assert status['state'] == 'APPROACH'
        assert status['proposal_only']['linear_mps'] > 0
        assert status['guarded_proposal']['linear_mps'] == 0
        assert status['motion_output_enabled'] is False
        assert 'perception_not_approved' in status['blockers']
        assert status['clearance_reference'] == 'lower_base_front_not_arm_or_payload'
        # Synthetic guard verdicts exercise routing and the independent latch.
        faults = []
        monkeypatch.setattr(shadow_module, 'guard_reasons', lambda *a, **k: list(faults))
        node.tick()
        assert captured[-1]['guarded_state'] == 'APPROACH'
        faults.append('charging_or_unknown')
        node.tick()
        assert captured[-1]['guarded_state'] == 'ABORTED'
        faults.clear()
        node.tick()
        assert captured[-1]['guarded_state'] == 'ABORTED'
        assert captured[-1]['guarded_proposal']['linear_mps'] == 0
    finally:
        node.destroy_node()
        context.shutdown()
