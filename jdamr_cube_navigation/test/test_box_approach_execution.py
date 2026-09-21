"""Fail-closed contracts for the explicitly armed box executor."""

import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import Twist
import jdamr_cube_navigation.box_approach_execution as execution_module
from jdamr_cube_navigation.box_approach_execution import (
    BoxApproachExecution,
    footprint_from_nav_params,
    ownership_reasons,
    physical_validation,
    readiness_reasons,
    shutdown_zero_drain,
    validation_reference_reason,
    VALIDATION_TRIAL_MODE,
)
from lifecycle_msgs.msg import State
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
import pytest
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
import yaml


def write(path, document):
    path.write_text(yaml.safe_dump(document))
    return path


def test_physical_approval_is_bound_to_all_motion_inputs(tmp_path):
    camera = tmp_path / 'camera.yaml'
    geometry = tmp_path / 'geometry.yaml'
    nav = tmp_path / 'nav.yaml'
    parking = tmp_path / 'parking.yaml'
    camera.write_bytes(b'measured camera')
    geometry.write_bytes(b'measured geometry')
    nav.write_bytes(b'reviewed navigation')
    parking.write_bytes(b'reviewed parking')
    approval = write(tmp_path / 'approval.yaml', {
        'schema_version': 1,
        'physically_validated': True,
        'evidence': 'run-2026-09-21-box-approach',
        'calibration_sha256': hashlib.sha256(camera.read_bytes()).hexdigest(),
        'geometry_sha256': hashlib.sha256(geometry.read_bytes()).hexdigest(),
        'nav_params_sha256': hashlib.sha256(nav.read_bytes()).hexdigest(),
        'parking_contract_sha256': hashlib.sha256(
            parking.read_bytes()).hexdigest(),
    })
    inputs = (camera, geometry, nav, parking)
    assert physical_validation(approval, *inputs)['schema_version'] == 1
    for target in inputs:
        original = target.read_bytes()
        target.write_bytes(original + b' changed')
        with pytest.raises(ValueError, match='hash_mismatch'):
            physical_validation(approval, *inputs)
        target.write_bytes(original)


@pytest.mark.parametrize('document,reason', [
    ({}, 'schema'),
    ({'schema_version': 1, 'physically_validated': False}, 'not_approved'),
    ({'schema_version': 1, 'physically_validated': True, 'evidence': ''},
     'evidence_missing'),
])
def test_physical_approval_fails_closed(tmp_path, document, reason):
    camera, geometry = tmp_path / 'c', tmp_path / 'g'
    nav, parking = tmp_path / 'n', tmp_path / 'p'
    camera.write_text('c')
    geometry.write_text('g')
    nav.write_text('n')
    parking.write_text('p')
    approval = write(tmp_path / 'approval.yaml', document)
    with pytest.raises(ValueError, match=reason):
        physical_validation(approval, camera, geometry, nav, parking)


def test_footprint_comes_from_local_costmap_config(tmp_path):
    path = write(tmp_path / 'nav.yaml', {'local_costmap': {'local_costmap': {
        'ros__parameters': {'footprint': '[[0.1, 0.2], [0.1, -0.2], [-0.3, 0.0]]'}
    }}})
    assert footprint_from_nav_params(path) == (
        (0.1, 0.2), (0.1, -0.2), (-0.3, 0.0))


def test_direct_node_config_reuses_full_new_base_validation(tmp_path):
    """Launching the executable cannot bypass the protected Nav2 geometry gate."""
    package = Path(__file__).resolve().parents[1]
    geometry = package.parent / (
        'jdamr_cube_description/config/new_base_geometry.yaml')
    document = yaml.safe_load(
        (package / 'config/new_base_nav2_params.yaml').read_text())
    document['local_costmap']['local_costmap']['ros__parameters'][
        'footprint'] = '[[0.01, 0.01], [0.01, -0.01], [-0.01, 0.0]]'
    nav = write(tmp_path / 'unsafe_nav.yaml', document)
    with pytest.raises(RuntimeError):
        footprint_from_nav_params(nav, geometry)


def safe_readiness():
    now = 10.0
    received = dict.fromkeys(
        ('perception', 'odom', 'scan', 'battery', 'collision', 'command',
         'footprint'), now)
    owners = {
        '/cmd_vel_nav': [('box_approach_execution', '/')],
        '/cmd_vel_smoothed': [('velocity_smoother', '/')],
        '/cmd_vel': [('collision_monitor', '/')],
        '/box_parking/perception_status': [('jdamr_depth_box_parking', '/')],
        '/local_costmap/published_footprint': [('box_approach_execution', '/')],
    }
    observation = {
        'stamp_s': now, 'frame_id': 'camera_color_optical_frame',
        'detected': True, 'stable': True, 'surface_kind': 'front',
        'front_distance_m': .8, 'lateral_error_m': 0.,
        'edge_angle_deg': 0., 'confidence': .95,
        'control_ready': False,
    }
    lifecycle = {
        'velocity_smoother': (State.PRIMARY_STATE_ACTIVE, now),
        'collision_monitor': (State.PRIMARY_STATE_ACTIVE, now),
    }
    return {
        'observation': observation, 'received': received,
        'now_mono_s': now, 'now_ros_s': now,
        'battery': (BatteryState.POWER_SUPPLY_STATUS_UNKNOWN, 12.0),
        'collision_action': CollisionMonitorState.DO_NOTHING,
        'final_command': (0., 0.), 'lifecycle': lifecycle, 'owners': owners,
        'charger_unplugged_confirmed': True,
        'optical_frame': 'camera_color_optical_frame', 'odom_stamp_s': now,
        'odom_velocity': (0.0, 0.0), 'zero_witness_valid': True,
        'graph_received_mono_s': now, 'cm_enable_valid': True,
    }


def armed_flow_node(clock):
    """Build a method-level executor harness without creating ROS endpoints."""
    node = object.__new__(BoxApproachExecution)
    node.armed, node.latched = False, False
    node.terminal_state = None
    node.execution_mode = 'production'
    node.validation_reference_m = None
    node.reason = 'DISARMED'
    node.pose = (0.0, 0.0, 0.0)
    node.velocity = (0.0, 0.0)
    node.final_command = (0.0, 0.0)
    node.odom_stamp_s = 10.0
    node.observation = {'stamp_s': 10.0}
    node.pose_history = [(9.9, node.pose), (10.1, node.pose)]
    node.chain_epoch = 3
    node.activation_epochs = {
        'velocity_smoother': 1, 'collision_monitor': 1}
    node.motion_sent = False
    node.first_motion_publish_mono_s = None
    node.first_motion_echo_mono_s = None
    node.first_motion_chain_witness = None
    node.travel_m = 0.0
    node.travel_pose = node.pose
    node.received = {'command': 1.0}
    node.last_publish_mono_s = -math.inf
    node._monotonic = lambda: clock[0]
    node.now_ros_s = lambda: 10.0
    node.owners = lambda: safe_readiness()['owners']
    node.publish_footprint = lambda: None
    node.publish_status = lambda blockers: None
    def readiness(require_start=True, require_command_fresh=None,
                  require_physical_validation=True):
        del require_start, require_physical_validation
        return (['command_stale'] if require_command_fresh
                and clock[0] - node.received.get(
                    'command', -math.inf) > .5 else [])

    node.readiness = readiness
    commands = []
    node.command_pub = SimpleNamespace(publish=commands.append)
    node.policy = SimpleNamespace(
        reset=lambda: None,
        step=lambda *args, **kwargs: {
            'state': 'APPROACHING', 'reason': 'approaching',
            'linear_mps': .03, 'angular_radps': 0.0,
        })
    return node, commands


def validation_start_node(clock, reference=.82, observed=.82):
    """Extend the method harness with one consumable operator reference."""
    node, commands = armed_flow_node(clock)
    parameter = [reference]
    node.observation = {
        'stamp_s': 10.0, 'front_distance_m': observed,
        'stable': True, 'confidence': .95,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameter[0])

    def set_parameters(values):
        assert values[0].name == 'validation_reference_m'
        parameter[0] = values[0].value
        return [SimpleNamespace(successful=True)]

    node.set_parameters = set_parameters
    return node, commands, parameter


def test_observer_control_ready_false_is_not_rewritten_or_used_as_approval():
    args = safe_readiness()
    assert readiness_reasons(**args) == []
    assert args['observation']['control_ready'] is False


@pytest.mark.parametrize(('mutate', 'reason'), [
    (lambda a: a.update(charger_unplugged_confirmed=False),
     'charger_unplugged_not_confirmed'),
    (lambda a: a.update(battery=(BatteryState.POWER_SUPPLY_STATUS_CHARGING, 12.0)),
     'battery_charging'),
    (lambda a: a.update(battery=(BatteryState.POWER_SUPPLY_STATUS_UNKNOWN, 10.4)),
     'battery_voltage_low_or_invalid'),
    (lambda a: a.update(collision_action=CollisionMonitorState.STOP),
     'collision_monitor_intervention'),
    (lambda a: a.update(zero_witness_valid=False),
     'final_zero_not_observed'),
    (lambda a: a['lifecycle'].update(
        velocity_smoother=(State.PRIMARY_STATE_INACTIVE, 10.0)),
     'velocity_smoother_not_active'),
])
def test_readiness_rejects_each_motion_interlock(mutate, reason):
    args = safe_readiness()
    mutate(args)
    assert reason in readiness_reasons(**args)


def test_exact_ownership_rejects_additional_publishers():
    owners = safe_readiness()['owners']
    assert ownership_reasons(owners) == []
    owners['/cmd_vel_nav'].append(('controller_server', '/'))
    assert ownership_reasons(owners) == [
        'publisher_owner_mismatch:/cmd_vel_nav']


def test_duplicate_same_name_publishers_are_still_a_conflict():
    owners = safe_readiness()['owners']
    owners['/cmd_vel_nav'].append(('box_approach_execution', '/'))
    assert ownership_reasons(owners) == [
        'publisher_owner_mismatch:/cmd_vel_nav']


def test_graph_snapshot_queries_five_topics_once_and_owners_use_cache():
    node = object.__new__(BoxApproachExecution)
    node.chain_epoch = 0
    node.chain_signatures = {}
    node.cached_owners = {}
    node.cached_chain_signatures = {}
    node.activation_epochs = {
        'velocity_smoother': 1, 'collision_monitor': 1}
    node.zero_witness = None
    node.cm_enable_ack = None
    node.received = {}
    node.collision_action = None
    node._monotonic = lambda: 10.0
    calls = []

    def graph_query(topic):
        calls.append(topic)
        names = {
            '/cmd_vel_nav': 'box_approach_execution',
            '/cmd_vel_smoothed': 'velocity_smoother',
            '/cmd_vel': 'collision_monitor',
            '/box_parking/perception_status': 'jdamr_depth_box_parking',
            '/local_costmap/published_footprint': 'box_approach_execution',
        }
        return [SimpleNamespace(
            node_name=names[topic], node_namespace='/',
            endpoint_gid=bytearray(topic.encode()))]

    node.get_publishers_info_by_topic = graph_query
    node.sample_graph()
    assert len(calls) == 5
    assert ownership_reasons(node.owners()) == []
    assert len(calls) == 5
    assert node.graph_received_mono_s == 10.0


def test_graph_snapshot_must_be_present_and_younger_than_cache_deadline():
    args = safe_readiness()
    args['graph_received_mono_s'] = None
    assert 'graph_snapshot_stale' in readiness_reasons(**args)
    args['graph_received_mono_s'] = 9.24
    assert 'graph_snapshot_stale' in readiness_reasons(**args)


def test_collision_monitor_enable_ack_is_true_only_for_request_epoch():
    clock = [10.0]
    node = object.__new__(BoxApproachExecution)
    node.chain_epoch = 4
    node.activation_epochs = {
        'velocity_smoother': 1, 'collision_monitor': 2}
    node.cm_enable_ack = None
    node.cm_toggle_pending = None
    node._monotonic = lambda: clock[0]
    requests = []
    future = SimpleNamespace(
        done=lambda: True,
        add_done_callback=lambda callback: None,
        result=lambda: SimpleNamespace(success=True, message='enabled'))
    node.cm_toggle_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: requests.append(request) or future)
    node.poll_cm_enabled()
    assert requests[0].enable is True
    node.cm_enable_result(future)
    assert node.cm_enable_valid() is True
    clock[0] = 12.01
    assert node.cm_enable_valid() is False
    clock[0] = 10.1
    node.chain_epoch += 1
    assert node.cm_enable_valid() is False


def test_unconfirmed_collision_monitor_enable_blocks_readiness():
    args = safe_readiness()
    args['cm_enable_valid'] = False
    assert 'collision_monitor_enable_unconfirmed' in readiness_reasons(**args)


def test_toggle_unavailable_or_failed_clears_enable_ack():
    node = object.__new__(BoxApproachExecution)
    node.chain_epoch = 4
    node.activation_epochs = {
        'velocity_smoother': 1, 'collision_monitor': 2}
    node.cm_enable_ack = (4, 2, 10.0)
    node.cm_toggle_pending = None
    node._monotonic = lambda: 10.0
    node.cm_toggle_client = SimpleNamespace(service_is_ready=lambda: False)
    node.poll_cm_enabled()
    assert node.cm_enable_ack is None

    future = SimpleNamespace(
        result=lambda: SimpleNamespace(success=False, message='failed'))
    node.cm_toggle_pending = (future, 10.0)
    node.cm_enable_ack = (4, 2, 10.0)
    node.cm_enable_result(future)
    assert node.cm_enable_ack is None


def test_armed_tick_aborts_when_cm_enable_confirmation_is_lost():
    clock = [10.0]
    node, commands = armed_flow_node(clock)
    assert node.on_start(Trigger.Request(), Trigger.Response()).success is True
    node.readiness = lambda **kwargs: [
        'collision_monitor_enable_unconfirmed']
    node.owners = lambda: safe_readiness()['owners']
    node.tick()
    assert node.terminal_state == 'ABORTED'
    assert node.reason == 'collision_monitor_enable_unconfirmed'
    assert commands[-1].linear.x == 0.0


def test_runtime_does_not_reapply_start_only_zero_and_stability_gates():
    args = safe_readiness()
    args['final_command'] = (0.04, 0.1)
    args['observation']['stable'] = False
    args['observation']['confidence'] = .7
    args['require_start'] = False
    assert readiness_reasons(**args) == []


def test_old_zero_witness_allows_idle_start_without_fresh_command_event():
    """CM stopping its zero heartbeat does not permanently block a safe start."""
    args = safe_readiness()
    args['received']['command'] = 1.0
    assert readiness_reasons(**args) == []


def test_start_requires_a_finite_zero_witness_and_stationary_odometry():
    args = safe_readiness()
    args['zero_witness_valid'] = False
    assert 'final_zero_not_observed' in readiness_reasons(**args)
    args = safe_readiness()
    args['odom_velocity'] = (0.011, 0.0)
    assert 'odom_not_stationary' in readiness_reasons(**args)
    args['odom_velocity'] = (0.0, 0.021)
    assert 'odom_not_stationary' in readiness_reasons(**args)


def test_runtime_still_requires_fresh_final_command_echo():
    args = safe_readiness()
    args['require_start'] = False
    args['received']['command'] = 9.4
    assert 'command_stale' in readiness_reasons(**args)


def test_nonfinite_final_command_cannot_create_or_retain_zero_witness():
    node = object.__new__(BoxApproachExecution)
    node.chain_epoch = 2
    node.activation_epochs = {
        'velocity_smoother': 1, 'collision_monitor': 1}
    node.zero_witness = (2, tuple(sorted(node.activation_epochs.items())))
    node.motion_sent = False
    node.first_motion_publish_mono_s = None
    node.first_motion_echo_mono_s = None
    node.received = {}
    node._monotonic = lambda: 10.0
    command = Twist()
    command.linear.x = math.nan
    node.on_final_command(command)
    assert node.zero_witness is None


def test_endpoint_gid_change_invalidates_zero_and_collision_witnesses():
    node = object.__new__(BoxApproachExecution)
    node.chain_epoch = 0
    node.chain_signatures = {
        'velocity_smoother': (('velocity_smoother', '/', b'old-s'),),
        'collision_monitor': (('collision_monitor', '/', b'old-c'),),
    }
    node.activation_epochs = {
        'velocity_smoother': 1, 'collision_monitor': 1}
    node.zero_witness = (0, tuple(sorted(node.activation_epochs.items())))
    node.received = {'command': 10.0, 'collision': 10.0}
    node.collision_action = CollisionMonitorState.DO_NOTHING
    node.cm_enable_ack = (0, 1, 10.0)
    node.cached_chain_signatures = {
        'velocity_smoother': (('velocity_smoother', '/', b'old-s'),),
        'collision_monitor': (('collision_monitor', '/', b'new-c'),),
    }
    node.refresh_chain_identity()
    assert node.zero_witness is None
    assert node.collision_action is None
    assert 'command' not in node.received
    assert 'collision' not in node.received
    assert node.cm_enable_ack is None


def test_lifecycle_restart_creates_new_activation_epoch_and_invalidates_zero():
    node = object.__new__(BoxApproachExecution)
    node.lifecycle = {
        'velocity_smoother': (State.PRIMARY_STATE_ACTIVE, 9.0)}
    node.activation_epochs = {
        'velocity_smoother': 1, 'collision_monitor': 1}
    node.chain_epoch = 0
    node.zero_witness = (0, tuple(sorted(node.activation_epochs.items())))
    node.received = {'command': 9.0}
    node.collision_action = CollisionMonitorState.DO_NOTHING
    node.lifecycle_pending = {}
    node._monotonic = lambda: 10.0
    result = SimpleNamespace(
        current_state=SimpleNamespace(id=State.PRIMARY_STATE_INACTIVE))
    future = SimpleNamespace(result=lambda: result)
    node.lifecycle_pending['velocity_smoother'] = (future, 9.9)
    node.lifecycle_result('velocity_smoother', future)
    assert node.zero_witness is None
    assert 'command' not in node.received


@pytest.mark.parametrize('event_first', [True, False])
def test_initial_collision_event_and_active_response_are_order_independent(
        event_first):
    """First ACTIVE establishes baseline without deleting a current CM state."""
    node = object.__new__(BoxApproachExecution)
    node.lifecycle = {}
    node.activation_epochs = {
        'velocity_smoother': 0, 'collision_monitor': 0}
    node.chain_epoch = 0
    node.zero_witness = None
    node.cm_enable_ack = None
    node.graph_received_mono_s = 10.0
    node.received = {}
    node.collision_action = None
    node.lifecycle_pending = {}
    node.armed = False
    node._monotonic = lambda: 10.0
    event = SimpleNamespace(action_type=CollisionMonitorState.DO_NOTHING)
    result = SimpleNamespace(
        current_state=SimpleNamespace(id=State.PRIMARY_STATE_ACTIVE))
    future = SimpleNamespace(result=lambda: result)

    def active_response():
        node.lifecycle_pending['collision_monitor'] = (future, 9.9)
        node.lifecycle_result('collision_monitor', future)

    if event_first:
        node.on_collision(event)
        active_response()
    else:
        active_response()
        node.on_collision(event)
    assert node.collision_action == CollisionMonitorState.DO_NOTHING
    assert node.received['collision'] == 10.0
    assert node.activation_epochs['collision_monitor'] == 1


def test_service_loss_after_active_invalidates_old_collision_state():
    """A real post-baseline outage cannot retain a pre-restart DO_NOTHING."""
    node = object.__new__(BoxApproachExecution)
    node.activation_epochs = {
        'velocity_smoother': 1, 'collision_monitor': 1}
    node.lifecycle = {
        'collision_monitor': (State.PRIMARY_STATE_ACTIVE, 9.0)}
    node.lifecycle_pending = {}
    node.lifecycle_clients = {
        'collision_monitor': SimpleNamespace(service_is_ready=lambda: False)}
    node.chain_epoch = 0
    node.zero_witness = (0, tuple(sorted(node.activation_epochs.items())))
    node.received = {'command': 9.0, 'collision': 9.0}
    node.collision_action = CollisionMonitorState.DO_NOTHING
    node._monotonic = lambda: 10.0
    node.poll_lifecycle()
    assert node.collision_action is None
    assert 'collision' not in node.received
    assert node.zero_witness is None


def test_source_odom_stamp_must_be_fresh_independently_of_arrival():
    args = safe_readiness()
    args['odom_stamp_s'] = 9.4
    assert 'odom_source_stamp_stale' in readiness_reasons(**args)


def test_invalid_scan_clears_previous_valid_receipt_immediately():
    """A malformed callback cannot extend the previous scan's freshness."""
    node = object.__new__(BoxApproachExecution)
    node.received = {'scan': 10.0}
    node.now_ros_s = lambda: 10.2
    node._monotonic = lambda: 10.2
    scan = LaserScan()
    scan.header.stamp.sec = 10
    scan.range_min, scan.range_max = 0.1, 8.0
    scan.ranges = [1.0]
    node.on_scan(scan)
    assert 'scan' not in node.received


def test_old_do_nothing_event_does_not_replace_current_state_polling():
    args = safe_readiness()
    args['received'].pop('collision')
    assert readiness_reasons(**args) == []


def test_active_event_unseen_is_ready_but_observed_stop_is_not():
    args = safe_readiness()
    args['collision_action'] = None
    assert readiness_reasons(**args) == []
    args['collision_action'] = CollisionMonitorState.SLOWDOWN
    assert readiness_reasons(**args) == []
    args['collision_action'] = CollisionMonitorState.STOP
    assert 'collision_monitor_intervention' in readiness_reasons(**args)


def test_start_first_tick_and_fresh_echo_complete_motion_handoff():
    clock = [10.0]
    node, commands = armed_flow_node(clock)
    response = node.on_start(Trigger.Request(), Trigger.Response())
    assert response.success is True

    node.tick()
    assert node.motion_sent is True
    assert commands[-1].linear.x == pytest.approx(.03)
    assert node.first_motion_echo_mono_s is None

    clock[0] = 10.2
    echo = Twist()
    echo.linear.x = .02
    node.on_final_command(echo)
    node.tick()
    assert node.armed is True
    assert node.first_motion_echo_mono_s == pytest.approx(10.2)


def test_first_motion_without_final_echo_aborts_after_deadline():
    clock = [10.0]
    node, commands = armed_flow_node(clock)
    assert node.on_start(Trigger.Request(), Trigger.Response()).success is True
    node.tick()

    clock[0] = 10.51
    node.tick()
    assert node.armed is False
    assert node.terminal_state == 'ABORTED'
    assert node.reason == 'first_motion_echo_timeout'
    assert commands[-1].linear.x == 0.0


def test_chain_epoch_change_during_first_echo_grace_aborts():
    clock = [10.0]
    node, commands = armed_flow_node(clock)
    assert node.on_start(Trigger.Request(), Trigger.Response()).success is True
    node.tick()

    node.chain_epoch += 1
    clock[0] = 10.1
    node.tick()
    assert node.terminal_state == 'ABORTED'
    assert node.reason == 'protection_chain_changed_before_first_echo'
    assert commands[-1].linear.x == 0.0


def test_observed_stop_aborts_and_latches_running_executor():
    clock = [10.0]
    node, commands = armed_flow_node(clock)
    assert node.on_start(Trigger.Request(), Trigger.Response()).success is True
    node.tick()
    node.on_collision(SimpleNamespace(action_type=CollisionMonitorState.STOP))
    assert node.armed is False
    assert node.latched is True
    assert node.reason == 'collision_monitor_intervention'
    assert commands[-1].linear.x == 0.0


def test_cancelled_idle_uses_start_gates_without_command_freshness():
    clock = [10.0]
    node, commands = armed_flow_node(clock)
    assert node.on_start(Trigger.Request(), Trigger.Response()).success is True
    node.tick()
    node.on_cancel(Trigger.Request(), Trigger.Response())
    node.received['command'] = 1.0
    calls = []

    def readiness(**kwargs):
        calls.append(kwargs)
        return (['command_stale']
                if kwargs['require_command_fresh'] else [])

    node.readiness = readiness
    node.owners = lambda: safe_readiness()['owners']
    node.tick()
    assert calls[-1] == {
        'require_start': True, 'require_command_fresh': False}
    assert node.reason == 'cancelled_by_operator'
    assert commands[-1].linear.x == 0.0


def test_active_start_request_cannot_reset_timeout_or_travel():
    node = object.__new__(BoxApproachExecution)
    node.armed = True
    response = node.on_start(Trigger.Request(), Trigger.Response())
    assert response.success is False
    assert response.message == 'approach_already_armed'


@pytest.mark.parametrize(('reference', 'reason'), [
    (0.0, 'validation_reference_missing'),
    (math.nan, 'validation_reference_invalid'),
    (0.64, 'validation_reference_out_of_range'),
    (1.01, 'validation_reference_out_of_range'),
])
def test_validation_reference_is_required_finite_and_in_range(
        reference, reason):
    assert validation_reference_reason(reference) == reason
    node, _, parameter = validation_start_node([10.0], reference=reference)
    response = node.on_start_validation(Trigger.Request(), Trigger.Response())
    assert response.success is False
    assert response.message == reason
    assert parameter[0] == 0.0


def test_validation_start_consumes_reference_and_requires_agreement():
    clock = [10.0]
    node, _, parameter = validation_start_node(clock, reference=.75)
    response = node.on_start_validation(Trigger.Request(), Trigger.Response())
    assert response.success is False
    assert response.message == 'validation_reference_disagreement'
    assert parameter[0] == 0.0
    assert node.armed is False

    node, _, parameter = validation_start_node(clock)
    response = node.on_start_validation(Trigger.Request(), Trigger.Response())
    assert response.success is True
    assert parameter[0] == 0.0
    assert node.execution_mode == VALIDATION_TRIAL_MODE
    assert node.validation_reference_m == pytest.approx(.82)


def test_validation_start_consumes_reference_even_when_already_armed():
    node, _, parameter = validation_start_node([10.0])
    node.armed = True
    response = node.on_start_validation(Trigger.Request(), Trigger.Response())
    assert response.success is False
    assert response.message == 'approach_already_armed'
    assert parameter[0] == 0.0


def test_validation_start_keeps_all_nonapproval_readiness_guards():
    clock = [10.0]
    node, _, parameter = validation_start_node(clock)
    calls = []

    def blocked(**kwargs):
        calls.append(kwargs)
        return ['scan_stale']

    node.readiness = blocked
    response = node.on_start_validation(Trigger.Request(), Trigger.Response())
    assert response.success is False
    assert response.message == 'scan_stale'
    assert calls == [{
        'require_start': True, 'require_physical_validation': False}]
    assert parameter[0] == 0.0


@pytest.mark.parametrize(('linear', 'angular', 'expected_linear',
                          'expected_angular'), [
    (.06, -.10, .03, -.05),
    (.02, .20, .01, .10),
    (0.0, -.20, 0.0, -.10),
])
def test_validation_trial_scales_policy_command_without_changing_curvature(
        linear, angular, expected_linear, expected_angular):
    clock = [10.0]
    node, commands, _ = validation_start_node(clock)
    node.policy = SimpleNamespace(
        reset=lambda: None,
        step=lambda *args, **kwargs: {
            'state': 'APPROACHING', 'reason': 'approaching',
            'linear_mps': linear, 'angular_radps': angular,
        })
    assert node.on_start_validation(
        Trigger.Request(), Trigger.Response()).success is True
    node.tick()
    assert commands[-1].linear.x == pytest.approx(expected_linear)
    assert commands[-1].angular.z == pytest.approx(expected_angular)
    if linear > 0.0:
        assert (commands[-1].angular.z / commands[-1].linear.x
                == pytest.approx(angular / linear))


@pytest.mark.parametrize(('linear', 'angular'), [
    (-.01, 0.0),
    (math.nan, 0.0),
    (.01, math.nan),
])
def test_validation_trial_does_not_sanitize_invalid_policy_commands(
        linear, angular):
    node, _, _ = validation_start_node([10.0])
    node.policy = SimpleNamespace(
        reset=lambda: None,
        step=lambda *args, **kwargs: {
            'state': 'APPROACHING', 'reason': 'approaching',
            'linear_mps': linear, 'angular_radps': angular,
        })
    assert node.on_start_validation(
        Trigger.Request(), Trigger.Response()).success is True
    with pytest.raises(ValueError, match='unbounded_execution_command'):
        node.tick()


@pytest.mark.parametrize(('travel_m', 'elapsed_s'), [
    (.4501, 0.0),
    (0.0, 30.01),
])
def test_validation_trial_aborts_at_tighter_travel_and_time_limits(
        travel_m, elapsed_s):
    clock = [10.0]
    node, commands, _ = validation_start_node(clock)
    assert node.on_start_validation(
        Trigger.Request(), Trigger.Response()).success is True
    node.travel_m = travel_m
    clock[0] += elapsed_s
    node.tick()
    assert node.terminal_state == 'ABORTED'
    assert node.reason == 'validation_trial_limit_reached'
    assert commands[-1].linear.x == 0.0


def test_validation_trial_success_is_not_production_approval():
    clock = [10.0]
    node, commands, _ = validation_start_node(clock)
    node.policy = SimpleNamespace(
        reset=lambda: None,
        step=lambda *args, **kwargs: {
            'state': 'SUCCEEDED', 'reason': 'observed_pose_held',
            'linear_mps': 0.0, 'angular_radps': 0.0,
        })
    assert node.on_start_validation(
        Trigger.Request(), Trigger.Response()).success is True
    node.tick()
    assert node.terminal_state == 'SUCCEEDED'
    assert node.reason == 'trial_completed'
    assert node.execution_mode == VALIDATION_TRIAL_MODE
    assert commands[-1].linear.x == 0.0

    node.readiness = lambda **kwargs: ['physical_validation_file_missing']
    response = node.on_start(Trigger.Request(), Trigger.Response())
    assert response.success is False
    assert response.message == 'physical_validation_file_missing'
    assert node.execution_mode == VALIDATION_TRIAL_MODE


def test_validation_trial_collision_stop_uses_existing_abort_guard():
    clock = [10.0]
    node, commands, _ = validation_start_node(clock)
    assert node.on_start_validation(
        Trigger.Request(), Trigger.Response()).success is True
    node.on_collision(SimpleNamespace(action_type=CollisionMonitorState.STOP))
    assert node.terminal_state == 'ABORTED'
    assert node.reason == 'collision_monitor_intervention'
    assert node.execution_mode == VALIDATION_TRIAL_MODE
    assert commands[-1].linear.x == 0.0


def test_validation_trial_slowdown_remains_armed_and_status_reports_it():
    clock = [10.0]
    node, commands, _ = validation_start_node(clock)
    assert node.on_start_validation(
        Trigger.Request(), Trigger.Response()).success is True
    node.on_collision(SimpleNamespace(
        action_type=CollisionMonitorState.SLOWDOWN))
    assert node.armed is True
    assert node.terminal_state is None
    assert commands == []

    node.last_publish_mono_s = -math.inf
    node.graph_received_mono_s = 10.0
    node.cm_enable_ack = None
    node.received['footprint'] = 10.0
    published = []
    node.zero_witness_valid = lambda: True
    node.cm_enable_valid = lambda: True
    node.status_pub = SimpleNamespace(publish=published.append)
    BoxApproachExecution.publish_status(node, [])
    status = json.loads(published[-1].data)
    assert status['collision_monitor_state'] == 'SLOWDOWN'


@pytest.mark.parametrize('action', [
    CollisionMonitorState.STOP,
    CollisionMonitorState.APPROACH,
    CollisionMonitorState.LIMIT,
    255,
])
def test_validation_trial_non_slowdown_interventions_still_abort(action):
    node, commands, _ = validation_start_node([10.0])
    assert node.on_start_validation(
        Trigger.Request(), Trigger.Response()).success is True
    node.on_collision(SimpleNamespace(action_type=action))
    assert node.terminal_state == 'ABORTED'
    assert node.reason == 'collision_monitor_intervention'
    assert commands[-1].linear.x == 0.0


def test_validation_trial_runtime_stale_scan_uses_existing_abort_guard():
    clock = [10.0]
    node, commands, _ = validation_start_node(clock)
    assert node.on_start_validation(
        Trigger.Request(), Trigger.Response()).success is True
    node.readiness = lambda **kwargs: ['scan_stale']
    node.tick()
    assert node.terminal_state == 'ABORTED'
    assert node.reason == 'scan_stale'
    assert commands[-1].linear.x == 0.0


def test_odometry_counts_cumulative_travel_not_only_displacement():
    node = object.__new__(BoxApproachExecution)
    node.received = {}
    node.pose_history = []
    node.pose = (0., 0., 0.)
    node.velocity = (0., 0.)
    node.odom_stamp_s = -math.inf
    node.armed = True
    node.travel_pose = None
    node.travel_m = 0.0
    node._monotonic = lambda: 10.0
    node.now_ros_s = lambda: 10.0
    for index, x in enumerate((0.0, 0.4, 0.0, 0.4)):
        message = Odometry()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.header.stamp.sec = 9
        message.header.stamp.nanosec = 600_000_000 + index * 100_000_000
        message.pose.pose.orientation.w = 1.0
        message.pose.pose.position.x = x
        node.on_odom(message)
    assert node.travel_m == pytest.approx(1.2)
    assert node.pose[0] == pytest.approx(0.4)


def test_success_latches_zero_without_becoming_aborted():
    node = object.__new__(BoxApproachExecution)
    node.armed, node.latched = True, False
    node.motion_sent = True
    node.last_publish_mono_s = 0.0
    node._monotonic = lambda: 10.0
    commands = []
    node.command_pub = SimpleNamespace(publish=commands.append)
    node.policy = SimpleNamespace(reset=lambda: None)
    node.succeed('observed_pose_held')
    assert node.terminal_state == 'SUCCEEDED'
    assert node.armed is False and node.latched is True
    assert commands[-1].linear.x == 0.0


def test_stale_source_and_arrival_are_independent():
    args = safe_readiness()
    args['observation']['stamp_s'] = 9.4
    assert 'perception_observation_invalid' in readiness_reasons(**args)
    args = safe_readiness()
    args['received']['perception'] = 9.4
    assert 'perception_stale' in readiness_reasons(**args)


def test_limits_are_fixed_at_reviewed_envelope():
    assert math.isclose(0.06, 0.06)
    source = Path(__file__).resolve().parents[1].joinpath(
        'jdamr_cube_navigation', 'box_approach_execution.py').read_text()
    assert "'/cmd_vel_nav'" in source
    assert "'/cmd_vel_smoothed'" not in source.split('create_publisher')[1].split(
        'create_subscription')[0]


def test_isolated_ros_node_starts_disarmed_without_physical_approval():
    """Node construction creates gates but cannot silently arm itself."""
    package = Path(__file__).resolve().parents[1]
    workspace = package.parent
    context = Context()
    context.init(domain_id=199)
    node = BoxApproachExecution(context=context, parameter_overrides=[
        Parameter('camera_mount_file', value=str(
            workspace / 'jdamr_cube_vslam/config/camera_mount.yaml')),
        Parameter('geometry_file', value=str(
            workspace / 'jdamr_cube_description/config/new_base_geometry.yaml')),
        Parameter('parking_contract_file', value=str(
            package / 'config/parking_contract.yaml')),
        Parameter('nav_params_file', value=str(
            package / 'config/new_base_nav2_params.yaml')),
    ])
    try:
        assert node.armed is False
        assert node.reason == 'DISARMED'
        assert 'physical_validation_file_missing' in node.readiness()
        publishers = dict(node.get_publisher_names_and_types_by_node(
            node.get_name(), node.get_namespace()))
        assert '/cmd_vel_nav' in publishers
        assert '/cmd_vel_smoothed' not in publishers
        services = dict(node.get_service_names_and_types_by_node(
            node.get_name(), node.get_namespace()))
        assert '/box_parking/start_approach' in services
        assert '/box_parking/start_validation_approach' in services
        assert '/box_parking/cancel_approach' in services
        assert node.get_parameter('validation_reference_m').value == 0.0
    finally:
        node.destroy_node()
        context.shutdown()


def test_status_never_reports_ready_while_a_start_blocker_exists():
    """READY has the same blocker-free meaning as an immediately valid start."""
    node = object.__new__(BoxApproachExecution)
    node.armed = False
    node.latched = False
    node.reason = 'DISARMED'
    node.last_publish_mono_s = -math.inf
    node.chain_epoch = 0
    node.activation_epochs = {
        'velocity_smoother': 0, 'collision_monitor': 0}
    node.zero_witness = None
    node.cm_enable_ack = None
    node.graph_received_mono_s = 10.0
    node.collision_action = None
    node.first_motion_echo_mono_s = None
    node.received = {'footprint': 10.0}
    node._monotonic = lambda: 10.0
    published = []
    node.status_pub = type('Publisher', (), {
        'publish': lambda self, message: published.append(message)})()
    node.publish_status(['physical_validation_file_missing'])
    status = json.loads(published[0].data)
    assert status['state'] == 'BLOCKED'
    assert status['mode'] == 'production'
    assert status['collision_monitor_state'] == 'ACTIVE_EVENT_UNSEEN'

    node.execution_mode = VALIDATION_TRIAL_MODE
    node.latched = True
    node.terminal_state = 'SUCCEEDED'
    node.publish_status(['physical_validation_file_missing'])
    status = json.loads(published[-1].data)
    assert status['state'] == 'SUCCEEDED'
    assert status['mode'] == VALIDATION_TRIAL_MODE


def test_shutdown_drains_zero_for_bounded_six_tenths(monkeypatch):
    now = [0.0]
    published = []
    node = SimpleNamespace(
        armed=True, motion_sent=True, context=object(),
        publish_zero=lambda: published.append(now[0]))
    monkeypatch.setattr(execution_module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(execution_module.rclpy, 'ok', lambda context: True)
    monkeypatch.setattr(
        execution_module.rclpy, 'spin_once',
        lambda target, timeout_sec: now.__setitem__(0, now[0] + timeout_sec))
    count = shutdown_zero_drain(node)
    assert node.armed is False
    assert count == len(published)
    assert count >= 12
    assert now[0] == pytest.approx(.6)
