"""Lifecycle-only startup gates tested without a robot or ROS processes."""

import json
from types import SimpleNamespace
from unittest.mock import Mock

from jdamr_cube_navigation import activate_navigation as module
from lifecycle_msgs.msg import State
from nav2_msgs.srv import ManageLifecycleNodes, SetInitialPose
import pytest
from rclpy.task import Future


def prepared_node():
    client = Mock()
    client.wait_for_service.return_value = True
    node = Mock()
    node.stop_requested = False
    node.create_client.return_value = client
    node._wait.return_value = SimpleNamespace(success=True)
    node.wait_until_ready.return_value = True
    node.capture_stationary_pose.return_value = ((1., 2., 0.), {'confirmed': True})
    node._guard_failure.return_value = None
    return node, client


def initial_pose():
    """Return one explicit pose with no generated defaults."""
    return {
        'frame_id': 'map', 'x_m': 1.0, 'y_m': -2.0, 'yaw_rad': .4,
        'covariance_x_m2': .1, 'covariance_y_m2': .2,
        'covariance_yaw_rad2': .03,
    }


def converging_initialization_node():
    """Return a node whose AMCL service calls complete immediately."""
    initial_client = Mock()
    update_client = Mock()
    initial_client.wait_for_service.return_value = True
    update_client.wait_for_service.return_value = True
    update_future = Future()
    update_future.set_result(SimpleNamespace())
    update_client.call_async.return_value = update_future
    node = Mock()
    node.stop_requested = False
    node.create_client.side_effect = [initial_client, update_client]
    node._wait.return_value = SimpleNamespace()
    request_time = SimpleNamespace(
        to_msg=lambda: SimpleNamespace(sec=99, nanosec=0))
    ack_time = SimpleNamespace(nanoseconds=100_000_000_000)
    node.get_clock.return_value.now.side_effect = [request_time, ack_time]
    return node, initial_client, update_client


def amcl_pose(stamp_s, covariance):
    values = [0.0] * 36
    values[0], values[7], values[35] = covariance
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp_s, nanosec=0)),
        pose=SimpleNamespace(covariance=values),
    )


def test_initial_pose_loader_requires_every_explicit_field(tmp_path):
    path = tmp_path / 'pose.yaml'
    pose = initial_pose()
    for missing in module.INITIAL_POSE_FIELDS:
        path.write_text('\n'.join(
            f'{key}: {value}' for key, value in pose.items()
            if key != missing), encoding='utf-8')
        with pytest.raises(ValueError, match='fields missing'):
            module.load_initial_pose(path)


@pytest.mark.parametrize(
    ('field', 'value', 'message'), [
        ('frame_id', 'odom', 'frame_id must be map'),
        ('x_m', float('nan'), 'must be finite'),
        ('y_m', True, 'must be finite'),
        ('yaw_rad', float('inf'), 'must be finite'),
        ('covariance_x_m2', 0.0, 'must be positive'),
        ('covariance_y_m2', -1.0, 'must be positive'),
        ('covariance_yaw_rad2', float('nan'), 'must be finite'),
    ])
def test_initial_pose_loader_rejects_invalid_values(
        tmp_path, field, value, message):
    path = tmp_path / 'pose.yaml'
    pose = initial_pose()
    pose[field] = value
    path.write_text('\n'.join(
        f'{key}: {candidate}' for key, candidate in pose.items()),
        encoding='utf-8')
    with pytest.raises(ValueError, match=message):
        module.load_initial_pose(path)


def test_initial_pose_loader_accepts_yaml_or_json_mapping(tmp_path):
    yaml_path = tmp_path / 'pose.yaml'
    json_path = tmp_path / 'pose.json'
    pose = initial_pose()
    yaml_path.write_text('\n'.join(
        f'{key}: {value}' for key, value in pose.items()), encoding='utf-8')
    json_path.write_text(json.dumps(pose), encoding='utf-8')
    assert module.load_initial_pose(yaml_path) == pose
    assert module.load_initial_pose(json_path) == pose


def test_initialization_uses_service_ack_and_post_ack_amcl_stamp(monkeypatch):
    check = Mock()
    monkeypatch.setattr(module, 'require_active', check)
    initial_client = Mock()
    update_client = Mock()
    initial_client.wait_for_service.return_value = True
    update_client.wait_for_service.return_value = True
    update_future = Mock()
    update_future.done.return_value = True
    update_future.exception.return_value = None
    update_future.result.return_value = SimpleNamespace()
    update_client.call_async.return_value = update_future
    node = Mock()
    node.stop_requested = False
    node.create_client.side_effect = [initial_client, update_client]
    node._wait.return_value = SimpleNamespace()
    request_time = SimpleNamespace(
        to_msg=lambda: SimpleNamespace(sec=99, nanosec=0))
    ack_time = SimpleNamespace(nanoseconds=100_000_000_000)
    node.get_clock.return_value.now.side_effect = [request_time, ack_time]

    def spin_once(_node, timeout_sec):
        assert timeout_sec == .05
        callback = node.create_subscription.call_args.args[2]
        callback(SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=101, nanosec=0))))

    monkeypatch.setattr(module.rclpy, 'spin_once', spin_once)
    module.initialize_localization(node, initial_pose())

    check.assert_called_once_with(node, module.LOCALIZATION_NODES)
    node.verify_live_maps.assert_called_once_with(require_command_path=False)
    request = initial_client.call_async.call_args.args[0]
    assert isinstance(request, SetInitialPose.Request)
    assert request.pose.header.frame_id == 'map'
    assert request.pose.pose.pose.position.x == 1.0
    assert request.pose.pose.covariance[0] == .1
    assert request.pose.pose.covariance[7] == .2
    assert request.pose.pose.covariance[35] == .03
    assert initial_client.call_async.call_count == 1
    update_client.call_async.assert_called()
    node.destroy_subscription.assert_called_once()
    assert [item.args[0] for item in node.destroy_client.call_args_list] == [
        initial_client, update_client]


def test_initialization_rejects_cached_amcl_pose_after_timeout(monkeypatch):
    monkeypatch.setattr(module, 'require_active', Mock())
    initial_client = Mock()
    update_client = Mock()
    initial_client.wait_for_service.return_value = True
    update_client.wait_for_service.return_value = True
    node = Mock()
    node.stop_requested = False
    node.create_client.side_effect = [initial_client, update_client]
    node._wait.return_value = SimpleNamespace()
    request_time = SimpleNamespace(
        to_msg=lambda: SimpleNamespace(sec=99, nanosec=0))
    ack_time = SimpleNamespace(nanoseconds=100_000_000_000)
    node.get_clock.return_value.now.side_effect = [request_time, ack_time]
    with pytest.raises(RuntimeError, match='fresh AMCL pose missing'):
        module.initialize_localization(node, initial_pose(), timeout_s=0.0)
    update_client.call_async.assert_not_called()


def test_initialization_requires_service_response(monkeypatch):
    monkeypatch.setattr(module, 'require_active', Mock())
    initial_client = Mock()
    update_client = Mock()
    initial_client.wait_for_service.return_value = True
    update_client.wait_for_service.return_value = True
    node = Mock()
    node.stop_requested = False
    node.create_client.side_effect = [initial_client, update_client]
    node._wait.return_value = None
    node.get_clock.return_value.now.return_value.to_msg.return_value = (
        SimpleNamespace(sec=99, nanosec=0))
    with pytest.raises(RuntimeError, match='response unconfirmed'):
        module.initialize_localization(node, initial_pose())
    update_client.call_async.assert_not_called()


def test_initialization_rejects_amcl_stamp_equal_to_ack(monkeypatch):
    monkeypatch.setattr(module, 'require_active', Mock())
    initial_client = Mock()
    update_client = Mock()
    initial_client.wait_for_service.return_value = True
    update_client.wait_for_service.return_value = True
    update_future = Mock()
    update_future.done.return_value = False
    update_client.call_async.return_value = update_future
    node = Mock()
    node.stop_requested = False
    node.create_client.side_effect = [initial_client, update_client]
    node._wait.return_value = SimpleNamespace()
    request_time = SimpleNamespace(
        to_msg=lambda: SimpleNamespace(sec=99, nanosec=0))
    ack_time = SimpleNamespace(nanoseconds=100_000_000_000)
    node.get_clock.return_value.now.side_effect = [request_time, ack_time]
    clock = iter([0.0, .1, .1, 2.0])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(clock))

    def spin_once(_node, timeout_sec):
        callback = node.create_subscription.call_args.args[2]
        callback(SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=100, nanosec=0))))

    monkeypatch.setattr(module.rclpy, 'spin_once', spin_once)
    with pytest.raises(RuntimeError, match='fresh AMCL pose missing'):
        module.initialize_localization(node, initial_pose(), timeout_s=1.0)


def test_initialization_keeps_nomotion_updates_until_covariance_converges(
        monkeypatch):
    monkeypatch.setattr(module, 'require_active', Mock())
    node, initial_client, update_client = converging_initialization_node()
    now_s = [0.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now_s[0])
    covariance_samples = iter([
        (0.25, 0.25, 0.0685),
        (0.04, 0.03, 0.04),
        (0.0065, 0.0046, 0.0034),
    ])

    def spin_once(_node, timeout_sec):
        assert timeout_sec == .05
        callback = node.create_subscription.call_args.args[2]
        callback(amcl_pose(101, next(covariance_samples)))
        now_s[0] += .1

    monkeypatch.setattr(module.rclpy, 'spin_once', spin_once)

    module.initialize_localization(
        node, initial_pose(), timeout_s=1.0,
        covariance_limits=(.01, .01, .03))

    assert initial_client.call_async.call_count == 1
    assert update_client.call_async.call_count == 2


def test_initialization_fails_before_startup_when_covariance_never_converges(
        monkeypatch):
    monkeypatch.setattr(module, 'require_active', Mock())
    node, _initial_client, update_client = converging_initialization_node()
    node.max_amcl_covariance = (.01, .01)
    node.service_contract = {'max_yaw_covariance_rad2': .03}
    now_s = [0.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now_s[0])

    def spin_once(_node, timeout_sec):
        callback = node.create_subscription.call_args.args[2]
        callback(amcl_pose(101, (0.25, 0.25, 0.0685)))
        now_s[0] += .1

    monkeypatch.setattr(module.rclpy, 'spin_once', spin_once)
    activate = Mock()
    monkeypatch.setattr(module, 'activate_prepared', activate)

    with pytest.raises(RuntimeError, match='covariance did not converge'):
        module.initialize_and_activate(node, initial_pose())

    activate.assert_not_called()
    assert 1 < update_client.call_async.call_count <= 40


def test_initialization_stop_during_covariance_convergence(monkeypatch):
    monkeypatch.setattr(module, 'require_active', Mock())
    node, _initial_client, _update_client = converging_initialization_node()
    now_s = [0.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now_s[0])

    def spin_once(_node, timeout_sec):
        callback = node.create_subscription.call_args.args[2]
        callback(amcl_pose(101, (0.25, 0.25, 0.0685)))
        node.stop_requested = True
        now_s[0] += .1

    monkeypatch.setattr(module.rclpy, 'spin_once', spin_once)

    with pytest.raises(RuntimeError, match='initialization interrupted'):
        module.initialize_localization(
            node, initial_pose(), timeout_s=1.0,
            covariance_limits=(.01, .01, .03))


@pytest.mark.parametrize('bad_value', [float('nan'), float('inf'), -0.01])
def test_initialization_rejects_nonfinite_or_negative_covariance(
        monkeypatch, bad_value):
    monkeypatch.setattr(module, 'require_active', Mock())
    node, _initial_client, _update_client = converging_initialization_node()
    now_s = [0.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now_s[0])

    def spin_once(_node, timeout_sec):
        callback = node.create_subscription.call_args.args[2]
        callback(amcl_pose(101, (bad_value, .001, .001)))
        now_s[0] += .1

    monkeypatch.setattr(module.rclpy, 'spin_once', spin_once)

    with pytest.raises(RuntimeError, match='covariance did not converge'):
        module.initialize_localization(
            node, initial_pose(), timeout_s=.3,
            covariance_limits=(.01, .01, .03))


@pytest.mark.parametrize('limits', [
    (), (.01,), (.01, .01), (.01, .01, .03, .04),
    (.01, .01, float('nan')), (.01, float('inf'), .03),
    (.01, 0.0, .03), (.01, -0.01, .03), (.01, True, .03),
])
def test_initialization_rejects_invalid_covariance_limits_before_pose_request(
        monkeypatch, limits):
    active = Mock()
    monkeypatch.setattr(module, 'require_active', active)
    node = Mock()

    with pytest.raises(ValueError, match='three finite positive values'):
        module.initialize_localization(
            node, initial_pose(), covariance_limits=limits)

    active.assert_not_called()
    node.create_client.assert_not_called()


def test_initialization_failure_never_reaches_navigation_startup(monkeypatch):
    initialize = Mock(side_effect=RuntimeError('fresh AMCL pose missing'))
    activate = Mock()
    monkeypatch.setattr(module, 'initialize_localization', initialize)
    monkeypatch.setattr(module, 'activate_prepared', activate)
    node = Mock()
    node.max_amcl_covariance = (.01, .01)
    node.service_contract = {'max_yaw_covariance_rad2': .03}
    with pytest.raises(RuntimeError, match='fresh AMCL pose missing'):
        module.initialize_and_activate(node, initial_pose())
    activate.assert_not_called()


def test_default_activation_path_does_not_initialize_or_send_motion(monkeypatch):
    initialize = Mock()
    activate = Mock()
    monkeypatch.setattr(module, 'initialize_localization', initialize)
    monkeypatch.setattr(module, 'activate_prepared', activate)
    node = Mock()
    module.initialize_and_activate(node)
    initialize.assert_not_called()
    activate.assert_called_once_with(node)
    node.create_publisher.assert_not_called()


@pytest.mark.parametrize('failure', ['maps', 'sensors', 'stationary', 'freshness'])
def test_unconfirmed_localization_never_requests_startup(monkeypatch, failure):
    monkeypatch.setattr(module, 'require_active', Mock())
    node, client = prepared_node()
    if failure == 'maps':
        node.verify_live_maps.side_effect = RuntimeError('map mismatch')
    elif failure == 'sensors':
        node.wait_until_ready.return_value = False
    elif failure == 'stationary':
        node.capture_stationary_pose.side_effect = RuntimeError('no current TF')
    else:
        node._guard_failure.return_value = 'AMCL pose stale'
    with pytest.raises(RuntimeError):
        module.activate_prepared(node)
    client.call_async.assert_not_called()


@pytest.mark.parametrize('success', [False, True])
def test_startup_ack_and_active_nodes_are_required(monkeypatch, success):
    check = Mock()
    monkeypatch.setattr(module, 'require_active', check)
    rollback = Mock()
    monkeypatch.setattr(module, 'rollback_navigation', rollback)
    node, client = prepared_node()
    node._wait.return_value = SimpleNamespace(success=success)
    if success:
        module.activate_prepared(node)
        assert check.call_args_list[1].args[1] == module.NAVIGATION_NODES
        node.emit.assert_called_once()
        assert node.verify_live_maps.call_count == 2
        assert node.verify_live_maps.call_args_list[0].kwargs == {
            'require_command_path': False}
        assert node.verify_live_maps.call_args_list[1].kwargs == {}
        rollback.assert_not_called()
    else:
        with pytest.raises(RuntimeError, match='startup failed'):
            module.activate_prepared(node)
        node.emit.assert_not_called()
        rollback.assert_called_once_with(node, client)
    request = client.call_async.call_args.args[0]
    assert request.command == ManageLifecycleNodes.Request.STARTUP
    node.destroy_client.assert_called_once_with(client)


@pytest.mark.parametrize('state', [State.PRIMARY_STATE_ACTIVE,
                                   State.PRIMARY_STATE_UNCONFIGURED])
def test_lifecycle_presence_is_not_active_evidence(state):
    node, client = prepared_node()
    node._wait.return_value = SimpleNamespace(current_state=SimpleNamespace(id=state))
    if state == State.PRIMARY_STATE_ACTIVE:
        module.require_active(node, ['amcl'])
    else:
        with pytest.raises(RuntimeError, match='state unconfirmed'):
            module.require_active(node, ['amcl'])
    node.destroy_client.assert_called_once_with(client)


def test_lifecycle_read_timeout_retries_once_with_same_client(monkeypatch):
    node, client = prepared_node()
    first = Future()
    second = Future()
    second.set_result(SimpleNamespace(
        current_state=SimpleNamespace(id=State.PRIMARY_STATE_ACTIVE)))
    client.call_async.side_effect = [first, second]
    node._wait.side_effect = [RuntimeError('timeout'), second.result()]
    clock = iter([10.0, 10.0, 12.0, 12.0])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(clock))

    module.require_active(node, ['amcl'])

    assert node.create_client.call_count == 1
    assert client.call_async.call_count == 2
    assert [item.args[1] for item in node._wait.call_args_list] == [2.0, 3.0]
    client.remove_pending_request.assert_called_once_with(first)
    warning = node.get_logger.return_value.warning
    warning.assert_called_once()
    evidence = warning.call_args.args[0]
    assert 'node=amcl attempt=1/2' in evidence
    assert 'original 5s response budget' in evidence
    assert 'error=timeout' in evidence


def test_lifecycle_read_retry_stays_inside_response_deadline(monkeypatch):
    node, client = prepared_node()
    first = Future()
    second = Future()
    second.set_result(SimpleNamespace(
        current_state=SimpleNamespace(id=State.PRIMARY_STATE_ACTIVE)))
    client.call_async.side_effect = [first, second]
    node._wait.side_effect = [RuntimeError('timeout'), second.result()]
    clock = iter([20.0, 20.0, 21.9, 21.9])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(clock))

    module.require_active(node, ['amcl'])

    timeouts = [call.args[1] for call in node._wait.call_args_list]
    assert timeouts[0] == 2.0
    assert timeouts[1] == pytest.approx(5.0 - 1.9)


def test_lifecycle_read_fails_after_two_bounded_timeouts(monkeypatch):
    node, client = prepared_node()
    first = Future()
    second = Future()
    client.call_async.side_effect = [first, second]
    node._wait.side_effect = [RuntimeError('first timeout'),
                              RuntimeError('second timeout')]
    clock = iter([30.0, 30.0, 32.0, 32.0])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(clock))

    with pytest.raises(RuntimeError, match='second timeout'):
        module.require_active(node, ['amcl'])

    assert client.call_async.call_count == 2
    assert client.remove_pending_request.call_args_list == [
        ((first,),), ((second,),)]


def test_lifecycle_read_does_not_retry_after_stop(monkeypatch):
    node, client = prepared_node()
    future = Future()
    client.call_async.return_value = future

    def interrupt(_future, _timeout):
        node.stop_requested = True
        raise RuntimeError('interrupted')

    node._wait.side_effect = interrupt
    clock = Mock(side_effect=[0.0, 0.0])
    monkeypatch.setattr(module.time, 'monotonic', clock)

    with pytest.raises(RuntimeError, match='interrupted'):
        module.require_active(node, ['amcl'])

    client.call_async.assert_called_once()
    client.remove_pending_request.assert_called_once_with(future)


def test_lifecycle_read_does_not_retry_completed_exception(monkeypatch):
    node, client = prepared_node()
    future = Future()
    future.set_exception(RuntimeError('rmw failure'))
    client.call_async.return_value = future
    node._wait.side_effect = RuntimeError('request failed')
    clock = Mock(side_effect=[0.0, 0.0])
    monkeypatch.setattr(module.time, 'monotonic', clock)

    with pytest.raises(RuntimeError, match='request failed'):
        module.require_active(node, ['amcl'])

    client.call_async.assert_called_once()
    client.remove_pending_request.assert_not_called()


def test_lifecycle_read_does_not_retry_wrong_state(monkeypatch):
    node, client = prepared_node()
    response = SimpleNamespace(
        current_state=SimpleNamespace(id=State.PRIMARY_STATE_UNCONFIGURED))
    node._wait.return_value = response
    clock = Mock(side_effect=[0.0, 0.0])
    monkeypatch.setattr(module.time, 'monotonic', clock)

    with pytest.raises(RuntimeError, match='state unconfirmed'):
        module.require_active(node, ['amcl'])

    client.call_async.assert_called_once()


def test_startup_timeout_does_not_retry_mutating_request(monkeypatch):
    node, client = prepared_node()
    monkeypatch.setattr(module, 'require_active', Mock())
    monkeypatch.setattr(module, 'rollback_navigation', Mock())
    node._wait.side_effect = RuntimeError('startup timeout')

    with pytest.raises(RuntimeError, match='startup timeout'):
        module.activate_prepared(node)

    client.call_async.assert_called_once()
    request = client.call_async.call_args.args[0]
    assert request.command == ManageLifecycleNodes.Request.STARTUP


@pytest.mark.parametrize('failure', ['timeout', 'post_identity', 'post_lifecycle'])
def test_failure_after_startup_requires_rollback(monkeypatch, failure):
    node, client = prepared_node()
    check = Mock()
    monkeypatch.setattr(module, 'require_active', check)
    rollback = Mock()
    monkeypatch.setattr(module, 'rollback_navigation', rollback)
    if failure == 'timeout':
        node._wait.side_effect = RuntimeError('timeout')
    elif failure == 'post_identity':
        node.verify_live_maps.side_effect = [None, RuntimeError('map changed')]
    else:
        check.side_effect = [None, RuntimeError('partial active')]
    with pytest.raises(RuntimeError):
        module.activate_prepared(node)
    if failure == 'post_identity':
        rollback.assert_not_called()
        assert node.emit.call_args.kwargs['servers_kept_active'] is True
        assert node.emit.call_args.kwargs['navigation_ready'] is False
    else:
        rollback.assert_called_once_with(node, client)


def test_uncertain_rollback_is_not_activation_success(monkeypatch):
    node, client = prepared_node()
    monkeypatch.setattr(module, 'require_active', Mock())
    node._wait.side_effect = RuntimeError('startup timeout')
    monkeypatch.setattr(module, 'rollback_navigation', Mock(side_effect=RuntimeError('no ACK')))
    with pytest.raises(RuntimeError, match='do not depart'):
        module.activate_prepared(node)
    assert node.emit.call_args.args[0] == 'navigation_activation_rollback_unconfirmed'


def test_rollback_waits_despite_stop_requested(monkeypatch):
    from rclpy.task import Future
    node, client = prepared_node()
    node.stop_requested = True
    future = Future()
    future.set_result(SimpleNamespace(success=True))
    client.call_async.return_value = future
    check = Mock()
    monkeypatch.setattr(module, 'require_active', check)
    module.rollback_navigation(node, client)
    assert client.call_async.call_args.args[0].command == ManageLifecycleNodes.Request.RESET
    assert check.call_args.kwargs['state_id'] == State.PRIMARY_STATE_UNCONFIGURED
    node._wait.assert_not_called()
