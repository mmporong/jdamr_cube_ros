"""Exercise single-request parking startup without a ROS graph or robot."""

import argparse
from types import SimpleNamespace
from unittest.mock import Mock

from jdamr_cube_navigation import box_parking_start as start
import pytest


@pytest.mark.parametrize('value', ['0', '0.64', '1.01', 'nan', 'inf'])
def test_reference_rejects_invalid(value):
    """Do not send out-of-envelope measurements to the robot."""
    with pytest.raises(argparse.ArgumentTypeError):
        start.reference_distance(value)


def fixture_clients(monkeypatch):
    """Provide fake service clients; no middleware is initialized."""
    params, begin, cancel, node = Mock(), Mock(), Mock(), Mock()
    node.create_client.side_effect = [begin, cancel]
    monkeypatch.setattr(start, 'AsyncParameterClient', lambda *args: params)
    results = [SimpleNamespace(results=[SimpleNamespace(successful=True)] * 2),
               SimpleNamespace(success=True, message='validation trial armed')]
    wait = Mock(side_effect=results)
    monkeypatch.setattr(start, 'await_result', wait)
    return node, params, begin, cancel, wait


def test_one_call_sets_evidence_and_starts(monkeypatch):
    """One invocation performs configuration and exactly one start."""
    node, params, begin, cancel, _ = fixture_clients(monkeypatch)
    assert start.start_once(node, .82) == 'validation trial armed'
    assert params.set_parameters.call_count == begin.call_async.call_count == 1
    assert [p.value for p in params.set_parameters.call_args.args[0]] == [True, .82]
    cancel.call_async.assert_not_called()


def test_config_rejection_never_starts(monkeypatch):
    """A rejected parameter update cannot trigger movement."""
    node, _, begin, cancel, wait = fixture_clients(monkeypatch)
    wait.side_effect = [SimpleNamespace(results=[SimpleNamespace(successful=False)])]
    with pytest.raises(RuntimeError, match='configuration rejected'):
        start.start_once(node, .82)
    begin.call_async.assert_not_called()
    cancel.call_async.assert_not_called()


def test_start_timeout_cancels_without_retry(monkeypatch):
    """An ambiguous start response requests cancel rather than another start."""
    node, params, begin, cancel, wait = fixture_clients(monkeypatch)
    ordering = Mock()
    ordering.attach_mock(params.set_parameters, 'set_parameters')
    ordering.attach_mock(begin.call_async, 'start')
    ordering.attach_mock(cancel.call_async, 'cancel')
    wait.side_effect = [SimpleNamespace(results=[SimpleNamespace(successful=True)] * 2),
                        TimeoutError('lost response'),
                        SimpleNamespace(results=[SimpleNamespace(successful=True)]),
                        SimpleNamespace(success=True)]
    with pytest.raises(TimeoutError):
        start.start_once(node, .82)
    assert begin.call_async.call_count == cancel.call_async.call_count == 1
    assert [call[0] for call in ordering.mock_calls] == [
        'set_parameters', 'start', 'set_parameters', 'cancel']
    assert params.set_parameters.call_args.args[0][0].value == 0.0


def test_failed_revocation_reports_unknown_and_still_cancels(monkeypatch):
    """Never claim stopped when the single-use reference was not revoked."""
    node, _, begin, cancel, wait = fixture_clients(monkeypatch)
    wait.side_effect = [SimpleNamespace(results=[SimpleNamespace(successful=True)] * 2),
                        TimeoutError('lost response'),
                        SimpleNamespace(results=[SimpleNamespace(successful=False)]),
                        SimpleNamespace(success=True)]
    with pytest.raises(RuntimeError, match='state unknown'):
        start.start_once(node, .82)
    assert begin.call_async.call_count == cancel.call_async.call_count == 1


def test_rejected_start_is_not_retried(monkeypatch):
    """The operator sees the real blocker without automatic retry."""
    node, _, begin, cancel, wait = fixture_clients(monkeypatch)
    wait.side_effect = [SimpleNamespace(results=[SimpleNamespace(successful=True)] * 2),
                        SimpleNamespace(success=False, message='scan_stale')]
    with pytest.raises(RuntimeError, match='scan_stale'):
        start.start_once(node, .82)
    assert begin.call_async.call_count == 1
    cancel.call_async.assert_not_called()
