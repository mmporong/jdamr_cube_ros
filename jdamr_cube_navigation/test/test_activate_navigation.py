"""Lifecycle-only startup gates tested without a robot or ROS processes."""

from types import SimpleNamespace
from unittest.mock import Mock

from lifecycle_msgs.msg import State
from nav2_msgs.srv import ManageLifecycleNodes
import pytest

from jdamr_cube_navigation import activate_navigation as module


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
