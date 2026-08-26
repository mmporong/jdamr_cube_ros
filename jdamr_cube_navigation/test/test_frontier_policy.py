"""Unit tests for the fail-closed frontier explorer policy."""

import ast
from dataclasses import replace
import inspect
import textwrap
import threading
import time

from jdamr_cube_navigation.frontier_core import (
    FrontierCandidate, FrontierConfig, GridMap)
import jdamr_cube_navigation.frontier_explorer as frontier_explorer_module
from jdamr_cube_navigation.frontier_explorer import (
    _cleanup_after_spin,
    battery_voltage_ready,
    collision_state_ready, command_owner_is_collision_monitor,
    exploration_transition_allowed,
    ExplorerPolicy, ExplorerState, frontier_result_is_current, FrontierExplorer,
    FrontierExtractionRequest, FrontierExtractionResult,
    FrontierExtractionWorker, manual_save_allowed, nav_callback_is_current,
    odom_fresh_ready, odom_stationary_ready, prepare_map_url_prefix, Readiness)
from jdamr_cube_navigation.frontier_explorer import ros_stamp_fresh_ready


HEALTHY = Readiness(
    map_fresh=True, scan_fresh=True, tf_fresh=True, odom_fresh=True,
    collision_monitor_active=True, planner_ready=True,
    navigator_ready=True, command_owner_ok=True, battery_ok=True,
    stationary=True)


def candidate(x=1.0, y=2.0):
    """Return a minimal deterministic candidate."""
    return FrontierCandidate(
        cell=(1, 2), x=x, y=y, yaw=0.0, cluster_size=3,
        information_gain=4, path_distance_m=1.0, clearance_m=0.7,
        heading_change=0.0, score=3.8)


def extraction_request(sequence, token=1, epoch=1):
    """Return a tiny immutable worker request."""
    return FrontierExtractionRequest(
        token=token, map_sequence=sequence, map_epoch=epoch,
        grid=GridMap(1, 1, 0.05, 0.0, 0.0, 0.0, (0,)),
        robot_pose=(0.025, 0.025, 0.0), blacklist=(), now=1.0)


def wait_for_result(worker, timeout=1.0):
    """Poll a worker result with a bounded test-only deadline."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = worker.poll()
        if result is not None:
            return result
        threading.Event().wait(0.001)
    raise AssertionError('frontier worker did not complete')


class ControlledFuture:
    """Small manually completed future for action transition tests."""

    def __init__(self, result=None, error=None):
        self._result = result
        self._error = error
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result

    def complete(self):
        assert self.callback is not None
        self.callback(self)


class ActionHandle:
    """Controlled accepted action handle with separate cancel and result."""

    accepted = True

    def __init__(self):
        self.cancel_future = ControlledFuture()
        self.result_future = ControlledFuture()

    def cancel_goal_async(self):
        return self.cancel_future

    def get_result_async(self):
        return self.result_future


def cancel_test_explorer(now=10.0):
    """Build the node-light fields used by Nav2 transition tests."""
    explorer = object.__new__(FrontierExplorer)
    explorer.policy = ExplorerPolicy()
    explorer.policy.state = ExplorerState.PAUSED
    explorer._path_token = 0
    explorer._nav_token = 1
    explorer._nav_goal_handle = None
    explorer._nav_send_future = None
    explorer._nav_send_started = float('-inf')
    explorer._nav_cancel_future = None
    explorer._nav_cancel_handle = None
    explorer._nav_cancel_started = float('-inf')
    explorer._nav_cancel_token = 0
    explorer._shutdown_requested = False
    explorer._stop_save_pending = False
    explorer._save_in_flight = False
    explorer._last_save_status = 'none'
    explorer._monotonic = lambda: now
    explorer.get_parameter = (
        lambda _name: type('Parameter', (), {'value': 1.0})())
    failures = []
    explorer._request_process_shutdown = failures.append
    return explorer, failures


def navigating(policy, now=0.0):
    """Move a policy through start, validation, and goal acceptance."""
    assert policy.start(HEALTHY)[0]
    assert policy.begin_validation(candidate())
    assert policy.navigation_started(now)


def test_startup_is_idle_and_missing_predicate_is_named():
    """Startup must not dispatch and failed readiness must be explicit."""
    policy = ExplorerPolicy()
    assert policy.state == ExplorerState.IDLE
    ok, reason = policy.start(replace(HEALTHY, scan_fresh=False))
    assert not ok
    assert 'scan_fresh' in reason
    assert policy.active_goal is None


def test_healthy_start_and_single_goal_invariant():
    """Healthy start selects while a second concurrent goal is rejected."""
    policy = ExplorerPolicy()
    assert policy.start(HEALTHY) == (True, 'ready')
    assert policy.state == ExplorerState.SELECT
    assert policy.begin_validation(candidate())
    assert not policy.begin_validation(candidate(3.0, 4.0))
    assert policy.navigation_started(1.0)
    assert policy.state == ExplorerState.NAVIGATE


def test_every_health_fault_cancels_and_latches_paused():
    """Every hard motion safety predicate fails closed in one tick."""
    names = (
        'collision_monitor_active', 'planner_ready', 'navigator_ready',
        'command_owner_ok', 'battery_ok')
    for name in names:
        policy = ExplorerPolicy()
        navigating(policy)
        decision = policy.tick(0.1, replace(HEALTHY, **{name: False}))
        assert decision.cancel_goal
        assert name in decision.fault
        assert policy.state == ExplorerState.PAUSED
        assert not policy.resume(replace(HEALTHY, **{name: False}))[0]


def test_stationary_is_start_gate_but_not_required_during_navigation():
    """Expected odometry motion must not cancel an active Nav2 goal."""
    policy = ExplorerPolicy()
    assert not policy.start(replace(HEALTHY, stationary=False))[0]
    navigating(policy)
    decision = policy.tick(0.1, replace(HEALTHY, stationary=False))
    assert not decision.cancel_goal
    assert policy.state == ExplorerState.NAVIGATE


def test_select_waits_for_transient_health_without_latching_pause():
    """An idle selection cycle recovers automatically before goal creation."""
    policy = ExplorerPolicy()
    assert policy.start(HEALTHY)[0]

    degraded = replace(HEALTHY, tf_fresh=False)
    decision = policy.tick(0.1, degraded)

    assert degraded.motion_missing() == ()
    assert degraded.selection_missing() == ('tf_fresh',)
    assert not decision.cancel_goal
    assert not decision.fault
    assert policy.state == ExplorerState.SELECT


def test_transient_data_gaps_block_start_but_do_not_latch_navigation():
    """Nav2 and Collision Monitor own transient motion-data recovery."""
    for name in ('map_fresh', 'scan_fresh', 'tf_fresh', 'odom_fresh'):
        degraded = replace(HEALTHY, **{name: False})
        assert not ExplorerPolicy().start(degraded)[0]

        policy = ExplorerPolicy()
        navigating(policy)
        decision = policy.tick(0.2, degraded)
        assert not decision.cancel_goal
        assert not decision.fault
        assert policy.state == ExplorerState.NAVIGATE


def test_probe_abort_cancels_and_never_auto_resumes():
    """Manual abort latches until a separate healthy resume request."""
    policy = ExplorerPolicy()
    navigating(policy)
    decision = policy.tick(0.1, HEALTHY, probe_abort=True)
    assert decision.cancel_goal
    assert policy.state == ExplorerState.PAUSED
    assert policy.tick(1.0, HEALTHY).cancel_goal is False
    assert policy.state == ExplorerState.PAUSED
    assert policy.resume(HEALTHY)[0]


def test_stall_and_recovery_failure_blacklist_before_reselection():
    """Navigation failures blacklist their region before reselection."""
    policy = ExplorerPolicy(stall_timeout=5.0, max_recoveries=1)
    navigating(policy, now=10.0)
    decision = policy.tick(15.0, HEALTHY, generation=7)
    assert decision.cancel_goal
    assert policy.state == ExplorerState.SELECT
    assert len(policy.blacklist) == 1
    assert policy.blacklist[0].x == 1.0
    assert policy.blacklist[0].generation == 7

    assert policy.begin_validation(candidate(3.0, 4.0))
    assert policy.navigation_started(20.0)
    decision = policy.feedback(
        20.1, 2.0, recoveries=2, generation=7)
    assert decision.cancel_goal
    assert len(policy.blacklist) == 2
    assert policy.blacklist[1].generation == 7
    assert policy.state == ExplorerState.SELECT


def test_progress_resets_stall_clock():
    """Meaningful distance progress extends the stall deadline."""
    policy = ExplorerPolicy(stall_timeout=5.0)
    navigating(policy, now=0.0)
    policy.feedback(4.0, 3.0, 0)
    assert not policy.tick(8.0, HEALTHY).cancel_goal
    assert policy.tick(9.0, HEALTHY).cancel_goal


def test_completion_needs_three_distinct_maps_and_settle_then_saves_once():
    """Completion requires independent observations and stationary settle."""
    policy = ExplorerPolicy(settle_seconds=10.0, empty_cycles=3)
    assert policy.start(HEALTHY)[0]
    policy.observe_frontiers(0, generation=1, now=0.0)
    policy.observe_frontiers(0, generation=1, now=1.0)
    policy.observe_frontiers(0, generation=2, now=2.0)
    assert policy.state == ExplorerState.SELECT
    policy.observe_frontiers(0, generation=3, now=3.0)
    assert policy.state == ExplorerState.SETTLING
    assert not policy.tick(12.9, HEALTHY).request_save
    decision = policy.tick(13.0, HEALTHY)
    assert decision.request_save
    assert policy.state == ExplorerState.SAVING
    assert not policy.tick(14.0, HEALTHY).request_save


def test_frontier_reappearance_resets_completion_and_save_result_is_safe():
    """A frontier resets completion and a saver fault latches PAUSED."""
    policy = ExplorerPolicy(settle_seconds=0.0, empty_cycles=3)
    assert policy.start(HEALTHY)[0]
    policy.observe_frontiers(0, 1, 0.0)
    policy.observe_frontiers(0, 2, 0.1)
    policy.observe_frontiers(1, 3, 0.2)
    policy.observe_frontiers(0, 4, 0.3)
    policy.observe_frontiers(0, 5, 0.4)
    assert policy.state == ExplorerState.SELECT
    policy.observe_frontiers(0, 6, 0.5)
    assert policy.tick(0.5, HEALTHY).request_save
    policy.save_result(False)
    assert policy.state == ExplorerState.PAUSED
    assert policy.fault == 'map save failed'


def test_stop_cancels_active_goal_and_returns_idle():
    """Operator stop cancels the active goal and returns to safe IDLE."""
    policy = ExplorerPolicy()
    navigating(policy)
    decision = policy.stop()
    assert decision.cancel_goal
    assert policy.state == ExplorerState.IDLE
    assert policy.active_goal is None


def test_late_path_rejection_cannot_unpause_policy():
    """A stale planner callback must not undo a safety pause."""
    policy = ExplorerPolicy()
    assert policy.start(HEALTHY)[0]
    assert policy.begin_validation(candidate())
    policy.pause('scan stale')
    policy.path_rejected()
    assert policy.state == ExplorerState.PAUSED


def test_battery_voltage_readiness_is_finite_fresh_and_above_floor():
    """Voltage alone is accepted only when finite, fresh, and safe."""
    assert battery_voltage_ready(12.1, 9.0, 10.0, 2.5, 10.5)
    assert not battery_voltage_ready(float('nan'), 9.0, 10.0, 2.5, 10.5)
    assert not battery_voltage_ready(float('inf'), 9.0, 10.0, 2.5, 10.5)
    assert not battery_voltage_ready(12.1, float('-inf'), 10.0, 2.5, 10.5)
    assert not battery_voltage_ready(12.1, 7.0, 10.0, 2.5, 10.5)
    assert not battery_voltage_ready(10.49, 9.0, 10.0, 2.5, 10.5)


def test_battery_fault_rejects_start_and_cancels_navigation():
    """Missing or unsafe battery telemetry always fails closed."""
    policy = ExplorerPolicy()
    ok, reason = policy.start(replace(HEALTHY, battery_ok=False))
    assert not ok
    assert 'battery_ok' in reason
    navigating(policy)
    decision = policy.tick(0.1, replace(HEALTHY, battery_ok=False))
    assert decision.cancel_goal
    assert policy.state == ExplorerState.PAUSED
    assert 'battery_ok' in decision.fault


def test_missing_or_stale_odom_blocks_start_resume_and_manual_save():
    """Stationary is unknown unless a fresh odometry sample proves it."""
    for received_at in (float('-inf'), 8.0):
        assert not odom_fresh_ready(received_at, 10.0, 0.5)
        stationary = odom_stationary_ready(
            0.0, 0.0, received_at, now=10.0, stale_seconds=0.5)
        assert not stationary
        unsafe = replace(HEALTHY, stationary=stationary)

        policy = ExplorerPolicy()
        assert not policy.start(unsafe)[0]
        policy.state = ExplorerState.PAUSED
        assert not policy.resume(unsafe)[0]
        assert not manual_save_allowed(
            ExplorerState.IDLE, unsafe, True, False)[0]

    assert odom_stationary_ready(0.0, 0.0, 9.75, 10.0, 0.5)
    assert odom_fresh_ready(9.75, 10.0, 0.5)
    assert not odom_stationary_ready(0.011, 0.0, 9.75, 10.0, 0.5)
    # The physical base is still at the measured two-encoder-tick yaw noise.
    assert odom_stationary_ready(0.0, 0.027488, 9.75, 10.0, 0.5)
    assert not odom_stationary_ready(0.0, 0.031, 9.75, 10.0, 0.5)
    stale_but_zero = replace(
        HEALTHY, odom_fresh=False, stationary=True)
    assert not ExplorerPolicy().start(stale_but_zero)[0]
    paused = ExplorerPolicy()
    paused.state = ExplorerState.PAUSED
    assert not paused.resume(stale_but_zero)[0]
    assert manual_save_allowed(
        ExplorerState.IDLE, stale_but_zero, True, False) == (
            False, 'odom not fresh')


def test_collision_active_response_expires_after_ttl():
    """A previously ACTIVE collision monitor must never latch forever."""
    assert collision_state_ready(True, 10.0, 10.9, 1.0)
    assert not collision_state_ready(True, 10.0, 11.01, 1.0)
    assert not collision_state_ready(False, 10.0, 10.1, 1.0)
    assert not collision_state_ready(True, float('-inf'), 10.1, 1.0)


def test_collision_lifecycle_query_is_single_flight_and_stale_safe():
    """Pending or invalidated lifecycle responses cannot overwrite state."""
    class Response:
        def __init__(self, state_id):
            self.current_state = type(
                'LifecycleState', (), {'id': state_id})()

    class Future:
        def __init__(self, response=None, error=None):
            self.response = response
            self.error = error
            self.callback = None
            self.cancelled = False

        def add_done_callback(self, callback):
            self.callback = callback

        def result(self):
            if self.error is not None:
                raise self.error
            return self.response

        def cancel(self):
            self.cancelled = True

    class Lifecycle:
        def __init__(self):
            self.ready = True
            self.calls = []

        def service_is_ready(self):
            return self.ready

        def call_async(self, _request):
            future = Future(Response(
                frontier_explorer_module.State.PRIMARY_STATE_ACTIVE))
            self.calls.append(future)
            return future

    explorer = object.__new__(FrontierExplorer)
    explorer._lifecycle = Lifecycle()
    explorer._collision_active = False
    explorer._collision_state_received = float('-inf')
    explorer._collision_query_future = None
    explorer._collision_query_started = float('-inf')
    explorer._collision_query_token = 0
    explorer._collision_query_error = 'old error'
    explorer._monotonic = lambda: 10.0
    explorer.get_parameter = (
        lambda _name: type('Parameter', (), {'value': 1.0})())

    explorer._poll_collision_lifecycle()
    first = explorer._lifecycle.calls[0]
    explorer._poll_collision_lifecycle()
    assert len(explorer._lifecycle.calls) == 1

    explorer._lifecycle.ready = False
    explorer._poll_collision_lifecycle()
    assert first.cancelled
    first.callback(first)
    assert not explorer._collision_active
    assert explorer._collision_state_received == float('-inf')

    explorer._lifecycle.ready = True
    explorer._poll_collision_lifecycle()
    active = explorer._lifecycle.calls[1]
    active.callback(active)
    assert explorer._collision_active
    assert explorer._collision_state_received == 10.0
    assert explorer._collision_query_error == ''

    explorer._poll_collision_lifecycle()
    inactive = explorer._lifecycle.calls[2]
    inactive.response = Response(0)
    inactive.callback(inactive)
    assert not explorer._collision_active

    explorer._poll_collision_lifecycle()
    failed = explorer._lifecycle.calls[3]
    failed.error = RuntimeError('service failed')
    failed.callback(failed)
    assert not explorer._collision_active
    assert explorer._collision_state_received == float('-inf')
    assert explorer._collision_query_error == (
        'RuntimeError: service failed')

    explorer._lifecycle.call_async = (
        lambda _request: (_ for _ in ()).throw(RuntimeError('race')))
    explorer._poll_collision_lifecycle()
    assert explorer._collision_query_future is None
    assert not explorer._collision_active
    assert explorer._collision_query_error == 'RuntimeError: race'


def test_delayed_collision_active_response_cannot_become_fresh():
    """A lifecycle reply older than its TTL is cancelled and ignored."""
    class Future:
        def __init__(self):
            self.callback = None
            self.cancelled = False

        def add_done_callback(self, callback):
            self.callback = callback

        def cancel(self):
            self.cancelled = True

        @staticmethod
        def result():
            state = type('LifecycleState', (), {
                'id': frontier_explorer_module.State.PRIMARY_STATE_ACTIVE})()
            return type('Response', (), {'current_state': state})()

    class Lifecycle:
        def __init__(self):
            self.calls = []

        @staticmethod
        def service_is_ready():
            return True

        def call_async(self, _request):
            future = Future()
            self.calls.append(future)
            return future

    now = [10.0]
    explorer = object.__new__(FrontierExplorer)
    explorer._lifecycle = Lifecycle()
    explorer._collision_active = False
    explorer._collision_state_received = float('-inf')
    explorer._collision_query_future = None
    explorer._collision_query_started = float('-inf')
    explorer._collision_query_token = 0
    explorer._monotonic = lambda: now[0]
    explorer.get_parameter = (
        lambda _name: type('Parameter', (), {'value': 1.0})())

    explorer._poll_collision_lifecycle()
    delayed = explorer._lifecycle.calls[0]
    now[0] = 11.1
    explorer._poll_collision_lifecycle()
    assert delayed.cancelled
    assert len(explorer._lifecycle.calls) == 2
    delayed.callback(delayed)
    assert not explorer._collision_active
    assert explorer._collision_state_received == float('-inf')


def test_manual_save_gate_requires_safe_idle_stationary_fresh_state():
    """Manual saves are accepted, not reported complete, after all gates."""
    assert manual_save_allowed(
        ExplorerState.IDLE, HEALTHY, True, False) == (
            True, 'map save request accepted')
    assert not manual_save_allowed(
        ExplorerState.NAVIGATE, HEALTHY, True, False)[0]
    assert not manual_save_allowed(
        ExplorerState.IDLE, replace(HEALTHY, map_fresh=False),
        True, False)[0]
    assert not manual_save_allowed(
        ExplorerState.IDLE, replace(HEALTHY, stationary=False),
        True, False)[0]
    assert not manual_save_allowed(
        ExplorerState.IDLE, HEALTHY, False, False)[0]
    assert not manual_save_allowed(
        ExplorerState.IDLE, HEALTHY, True, True)[0]


def test_save_in_progress_blocks_start_and_resume_transition():
    """Exploration cannot overlap a pending asynchronous map write."""
    assert exploration_transition_allowed(False) == (True, '')
    assert exploration_transition_allowed(True) == (
        False, 'map save in progress')


def test_old_navigation_callbacks_cannot_affect_new_goal_generation():
    """Only feedback/results tagged with the current goal may be consumed."""
    old_goal = 4
    new_goal = 5
    assert not nav_callback_is_current(old_goal, new_goal)
    assert nav_callback_is_current(new_goal, new_goal)


def test_stale_navigation_feedback_and_result_leave_goal_untouched():
    """Late feedback and terminal results cannot mutate the current goal."""
    class Handle:
        accepted = True

        def __init__(self):
            self.cancelled = False

    class Future:
        def __init__(self, result):
            self._result = result

        def result(self):
            return self._result

    class Policy:
        state = ExplorerState.NAVIGATE

        def feedback(self, *_args):
            raise AssertionError('stale feedback reached policy')

    explorer = object.__new__(FrontierExplorer)
    explorer._nav_token = 2
    current_handle = Handle()
    explorer._nav_goal_handle = current_handle
    explorer._nav_cancel_handle = None
    explorer.policy = Policy()

    explorer._nav_feedback(object(), token=1)
    explorer._nav_result(Future(object()), token=1, handle=Handle())
    assert explorer._nav_goal_handle is current_handle


def test_navigation_cancel_pending_blocks_transitions_until_terminal():
    """Cancel acceptance alone is insufficient to permit another goal."""
    explorer, failures = cancel_test_explorer()
    handle = ActionHandle()
    explorer._nav_goal_handle = handle
    explorer._begin_nav_cancel(handle)

    assert explorer._navigation_transition_pending()
    assert not exploration_transition_allowed(False, True)[0]
    handle.cancel_future._result = type(
        'CancelResponse', (), {
            'return_code':
                frontier_explorer_module.CancelGoal.Response.ERROR_NONE})()
    handle.cancel_future.complete()
    assert explorer._navigation_transition_pending()

    handle.result_future._result = type(
        'Result', (), {
            'status': frontier_explorer_module.GoalStatus.STATUS_CANCELED})()
    explorer._nav_cancel_terminal(handle.result_future, handle)
    assert not explorer._navigation_transition_pending()
    assert explorer._nav_goal_handle is None
    assert failures == []


def test_pause_and_stop_responses_expose_pending_cancellation():
    """Operator responses never imply stop completion before Nav2 terminal."""
    explorer, _failures = cancel_test_explorer()
    explorer.policy = ExplorerPolicy()
    navigating(explorer.policy)
    handle = ActionHandle()
    explorer._nav_goal_handle = handle
    explorer._selection_token = 0
    explorer._path_goal_handle = None
    explorer.get_logger = lambda: type('Logger', (), {
        'error': lambda _self, _message: None})()
    response = type('Response', (), {})()

    explorer._pause(None, response)
    assert response.success
    assert response.message == (
        'pause requested; navigation cancellation pending')
    explorer._stop(None, response)
    assert response.message == (
        'stop requested; navigation cancellation pending; map save pending')


def test_stop_waits_for_terminal_and_safe_stationary_then_saves_once(tmp_path):
    """Stop cancellation, standstill, and automatic save form one sequence."""
    class MapSaver:
        def __init__(self):
            self.calls = []
            self.future = ControlledFuture()

        @staticmethod
        def service_is_ready():
            return True

        def call_async(self, request):
            self.calls.append(request)
            return self.future

    explorer, failures = cancel_test_explorer()
    explorer.policy = ExplorerPolicy()
    navigating(explorer.policy)
    handle = ActionHandle()
    explorer._nav_goal_handle = handle
    explorer._selection_token = 0
    explorer._path_goal_handle = None
    explorer._save_automatic = False
    explorer._last_save_error = ''
    explorer._map_saver = MapSaver()
    explorer.get_parameter = (
        lambda name: type('Parameter', (), {
            'value': (str(tmp_path / 'maps' / 'autonomous')
                      if name == 'map_url_prefix' else 1.0)})())
    explorer.get_logger = lambda: type('Logger', (), {
        'error': lambda _self, _message: None})()
    response = type('Response', (), {})()

    explorer._stop(None, response)
    assert explorer._stop_save_pending
    assert 'map save pending' in response.message
    explorer._tick_stop_save(HEALTHY)
    assert len(explorer._map_saver.calls) == 0

    handle.cancel_future._result = type(
        'CancelResponse', (), {
            'return_code':
                frontier_explorer_module.CancelGoal.Response.ERROR_NONE})()
    handle.cancel_future.complete()
    handle.result_future._result = type(
        'Result', (), {
            'status': frontier_explorer_module.GoalStatus.STATUS_CANCELED})()
    explorer._nav_cancel_terminal(handle.result_future, handle)

    explorer._tick_stop_save(replace(HEALTHY, stationary=False))
    explorer._tick_stop_save(replace(
        HEALTHY, odom_fresh=False, stationary=True))
    assert len(explorer._map_saver.calls) == 0
    assert exploration_transition_allowed(False, False, True) == (
        False, 'stop map save pending')
    start_response = type('Response', (), {})()
    explorer._start(None, start_response)
    assert start_response.success is False
    assert start_response.message == 'stop map save pending'
    explorer._resume(None, start_response)
    assert start_response.success is False
    manual_response = type('Response', (), {})()
    explorer._manual_save_map(None, manual_response)
    assert manual_response.success is False
    assert manual_response.message == 'stop map save pending'

    explorer._tick_stop_save(HEALTHY)
    assert explorer.policy.state == ExplorerState.SAVING
    assert explorer._save_in_flight
    assert len(explorer._map_saver.calls) == 1
    explorer._tick_stop_save(HEALTHY)
    explorer._stop(None, response)
    assert len(explorer._map_saver.calls) == 1

    explorer._map_saver.future._result = type(
        'SaveResponse', (), {'result': True})()
    explorer._map_saver.future.complete()
    assert explorer.policy.state == ExplorerState.FINISHED
    assert explorer._last_save_status == 'succeeded'
    assert failures == []


def test_navigation_cancel_rejection_exception_and_timeout_shutdown():
    """Every unproven cancellation path requests launch-level shutdown."""
    explorer, failures = cancel_test_explorer()
    handle = ActionHandle()
    explorer._nav_goal_handle = handle
    explorer._begin_nav_cancel(handle)
    handle.cancel_future._result = type(
        'CancelResponse', (), {'return_code': 1})()
    handle.cancel_future.complete()
    assert 'rejected' in failures[0]

    explorer, failures = cancel_test_explorer()
    handle = ActionHandle()
    handle.cancel_future._error = RuntimeError('transport')
    explorer._nav_goal_handle = handle
    explorer._begin_nav_cancel(handle)
    handle.cancel_future.complete()
    assert 'response failed' in failures[0]

    explorer, failures = cancel_test_explorer(now=10.0)
    handle = ActionHandle()
    explorer._nav_goal_handle = handle
    explorer._begin_nav_cancel(handle)
    explorer._check_navigation_transition_timeout(11.0)
    assert failures == ['navigation cancel timed out']


def test_cancel_result_must_be_successful_and_terminal():
    """Callback arrival without a proven terminal status is fail-closed."""
    explorer, failures = cancel_test_explorer()
    handle = ActionHandle()
    explorer._nav_goal_handle = handle
    explorer._begin_nav_cancel(handle)
    handle.result_future._error = RuntimeError('result transport')
    explorer._nav_cancel_terminal(handle.result_future, handle)
    assert 'cancel result failed' in failures[0]

    explorer, failures = cancel_test_explorer()
    handle = ActionHandle()
    explorer._nav_goal_handle = handle
    explorer._begin_nav_cancel(handle)
    handle.result_future._result = type(
        'Result', (), {
            'status': frontier_explorer_module.GoalStatus.STATUS_EXECUTING})()
    explorer._nav_cancel_terminal(handle.result_future, handle)
    assert 'not terminal' in failures[0]


def test_stale_goal_acceptance_enters_tracked_cancel_and_send_times_out():
    """Late acceptance is cancelled and a missing send response shuts down."""
    explorer, failures = cancel_test_explorer()
    send = ControlledFuture(ActionHandle())
    explorer._nav_send_future = send
    explorer._nav_send_started = 10.0
    explorer._nav_token = 2
    explorer._nav_goal_response(send, token=1)
    assert explorer._nav_send_future is None
    assert explorer._nav_cancel_handle is send._result
    assert failures == []

    explorer, failures = cancel_test_explorer(now=10.0)
    explorer._nav_send_future = ControlledFuture()
    explorer._nav_send_started = 10.0
    explorer._check_navigation_transition_timeout(11.0)
    assert failures == ['navigation goal send timed out']


def test_navigation_result_exception_requests_fail_safe_shutdown():
    """A transport exception cannot be assumed to mean an aborted goal."""
    explorer, failures = cancel_test_explorer()
    handle = ActionHandle()
    explorer._nav_goal_handle = handle
    result = ControlledFuture(error=RuntimeError('lost result'))
    explorer._nav_result(result, token=1, handle=handle)
    assert failures == ['navigation result failed: lost result']


def test_manual_save_is_rejected_while_motion_transition_is_pending():
    """Pause does not permit saving before Nav2 cancellation is terminal."""
    explorer, _failures = cancel_test_explorer()
    explorer._nav_send_future = ControlledFuture()
    explorer._refresh_tf = lambda: (_ for _ in ()).throw(
        AssertionError('readiness must not run while navigation is pending'))
    response = type('Response', (), {})()
    assert explorer._manual_save_map(None, response) is response
    assert response.success is False
    assert response.message == 'navigation send/cancel still pending'


def test_cancel_pending_prevents_any_frontier_or_goal_work():
    """No selection work starts before cancellation becomes terminal."""
    class Worker:
        polls = 0
        offers = 0

        def poll(self):
            self.polls += 1
            return None

        def offer(self, _request):
            self.offers += 1

    explorer, _failures = cancel_test_explorer()
    explorer.policy.state = ExplorerState.SELECT
    explorer._nav_cancel_handle = object()
    explorer._frontier_worker = Worker()
    explorer._select(10.0)
    assert explorer._frontier_worker.polls == 0
    assert explorer._frontier_worker.offers == 0


def test_unsafe_raw_map_start_waits_without_latching_or_counting_empty():
    """A transient raw-map start waits for the next map without a latch."""
    class Worker:
        offers = 0

        @staticmethod
        def poll():
            return None

        def offer(self, _request):
            self.offers += 1

    cases = (
        (GridMap(1, 1, 1.0, 0.0, 0.0, 0.0, (0,)), (-0.5, 0.5, 0.0)),
        (GridMap(1, 1, 1.0, 0.0, 0.0, 0.0, (100,)), (0.5, 0.5, 0.0)),
        (GridMap(2, 1, 1.0, 0.0, 0.0, 0.0, (0, 100)), (0.5, 0.5, 0.0)),
    )
    for grid, pose in cases:
        explorer = object.__new__(FrontierExplorer)
        explorer.policy = ExplorerPolicy()
        assert explorer.policy.start(HEALTHY)[0]
        explorer.core = frontier_explorer_module.FrontierCore(
            FrontierConfig(clearance_m=1.0))
        explorer._nav_send_future = None
        explorer._nav_cancel_handle = None
        explorer._frontier_worker = Worker()
        explorer._map = grid
        explorer._robot_pose = pose
        explorer._map_sequence = 1
        explorer._map_epoch = 1
        explorer._last_selected_sequence = -1
        explorer._selection_token = 1
        explorer._apply = lambda _decision: None

        explorer._select(10.0)
        assert explorer.policy.state == ExplorerState.SELECT
        assert explorer.policy.fault == ''
        assert explorer.policy._empty_cycles == 0
        assert explorer._last_selected_sequence == explorer._map_sequence
        assert explorer._frontier_worker.offers == 0


def test_wrapper_does_not_shadow_rclpy_node_clock():
    """Monotonic policy time must not replace Node's internal ROS clock."""
    source = textwrap.dedent(inspect.getsource(FrontierExplorer.__init__))
    tree = ast.parse(source)
    assigned = {
        target.attr
        for node in ast.walk(tree)
        if isinstance(node, (ast.Assign, ast.AnnAssign))
        for target in (
            node.targets if isinstance(node, ast.Assign) else [node.target])
        if isinstance(target, ast.Attribute) and
        isinstance(target.value, ast.Name) and target.value.id == 'self'
    }
    assert '_clock' not in assigned
    assert '_monotonic' in assigned
    assert 'self.get_clock()' in inspect.getsource(
        FrontierExplorer._refresh_tf)


def test_transient_tf_lookup_failure_uses_last_verified_sample_until_ttl():
    """One lookup miss must not invalidate a transform still inside its TTL."""
    class MissingBuffer:
        @staticmethod
        def lookup_transform(*_args):
            raise RuntimeError('temporary lookup miss')

    explorer = object.__new__(FrontierExplorer)
    explorer._tf_buffer = MissingBuffer()
    explorer._monotonic = lambda: 10.0
    explorer._tf_received = 9.7
    explorer._tf_fresh = True
    explorer.get_parameter = lambda _name: type(
        'Parameter', (), {'value': 0.5})()

    explorer._refresh_tf()
    assert explorer._tf_fresh

    explorer._tf_received = 9.4
    explorer._refresh_tf()
    assert not explorer._tf_fresh


def test_transient_invalid_tf_stamp_uses_last_verified_sample_until_ttl():
    """One bad TF timestamp must not discard a still-fresh verified sample."""
    transform = type('TransformStamped', (), {
        'header': type('Header', (), {
            'stamp': type('Stamp', (), {'sec': 9, 'nanosec': 0})()})(),
        'transform': type('Transform', (), {
            'translation': type(
                'Translation', (), {'x': 1.0, 'y': 2.0})(),
            'rotation': type('Rotation', (), {
                'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0})()})()})()

    class StaleBuffer:
        @staticmethod
        def lookup_transform(*_args):
            return transform

    explorer = object.__new__(FrontierExplorer)
    explorer._tf_buffer = StaleBuffer()
    explorer._monotonic = lambda: 10.0
    explorer._tf_received = 9.7
    explorer._tf_fresh = True
    explorer._robot_pose = (0.0, 0.0, 0.0)
    explorer.get_parameter = lambda _name: type(
        'Parameter', (), {'value': 0.5})()
    explorer.get_clock = lambda: type('Clock', (), {
        'now': lambda _self: type(
            'Time', (), {'nanoseconds': 10_000_000_000})()})()

    explorer._refresh_tf()
    assert explorer._tf_fresh
    assert explorer._robot_pose == (0.0, 0.0, 0.0)

    explorer._tf_received = 9.4
    explorer._refresh_tf()
    assert not explorer._tf_fresh


def test_ros_stamp_freshness_rejects_future_and_stale_samples():
    """TF header age validation is inclusive and fails closed."""
    now_ns = 10_000_000_000
    assert ros_stamp_fresh_ready(now_ns, now_ns, 0.5)
    assert ros_stamp_fresh_ready(now_ns - 500_000_000, now_ns, 0.5)
    assert not ros_stamp_fresh_ready(now_ns + 1, now_ns, 0.5)
    assert not ros_stamp_fresh_ready(now_ns - 500_000_001, now_ns, 0.5)


def test_main_preserves_ros_import_error_as_runtime_cause():
    """Missing ROS modules remain diagnosable when the executable is run."""
    source = inspect.getsource(frontier_explorer_module.main)
    assert 'from ROS_IMPORT_ERROR' in source


def test_status_separates_expected_motion_from_blocking_readiness():
    """Moving navigation reports stationary separately without false fault."""
    published = []
    explorer = object.__new__(FrontierExplorer)
    explorer.policy = ExplorerPolicy()
    explorer.policy.state = ExplorerState.NAVIGATE
    explorer._readiness = lambda: replace(HEALTHY, stationary=False)
    explorer._battery_voltage = 12.0
    explorer._last_save_status = 'none'
    explorer._frontier_count = 3
    explorer._map_sequence = 7
    explorer._nav_send_future = None
    explorer._nav_cancel_handle = None
    explorer._status_pub = type('Publisher', (), {
        'publish': lambda _self, msg: published.append(msg.data)})()

    explorer._publish_status()
    assert 'stationary=False' in published[-1]
    assert 'readiness_missing=none' in published[-1]
    assert 'motion_transition=clear' in published[-1]
    assert 'frontiers=3;map_sequence=7' in published[-1]

    explorer.policy.state = ExplorerState.SETTLING
    explorer._publish_status()
    assert 'readiness_missing=stationary' in published[-1]


def test_cmd_vel_owner_requires_root_collision_monitor_exclusively():
    """A same-named node in another namespace cannot spoof ownership."""
    class Publisher:
        def __init__(self, name, namespace):
            self.node_name = name
            self.node_namespace = namespace

    root_monitor = Publisher('collision_monitor', '/')
    spoofed_monitor = Publisher('collision_monitor', '/other')
    root_teleop = Publisher('web_teleop', '/')
    assert command_owner_is_collision_monitor([root_monitor])
    assert not command_owner_is_collision_monitor([])
    assert not command_owner_is_collision_monitor(
        [root_monitor, root_monitor])
    assert not command_owner_is_collision_monitor([spoofed_monitor])
    assert not command_owner_is_collision_monitor(
        [root_monitor, root_teleop])


def test_status_can_reuse_tick_readiness_without_a_second_graph_pass(
        monkeypatch):
    """Status publication reuses the exact safety snapshot from the tick."""
    monkeypatch.setattr(
        frontier_explorer_module, 'String',
        type('String', (), {'data': ''}), raising=False)
    published = []
    explorer = object.__new__(FrontierExplorer)
    explorer.policy = ExplorerPolicy()
    explorer._readiness = lambda: (_ for _ in ()).throw(
        AssertionError('tick readiness must be reused'))
    explorer._battery_voltage = 12.0
    explorer._last_save_status = 'none'
    explorer._frontier_count = None
    explorer._map_sequence = 0
    explorer._nav_send_future = None
    explorer._nav_cancel_handle = None
    explorer._status_pub = type('Publisher', (), {
        'publish': lambda _self, msg: published.append(msg.data)})()

    explorer._publish_status(HEALTHY)
    assert 'readiness_missing=none' in published[-1]


def test_blocked_extraction_does_not_block_pause_or_stop_policy():
    """Operator state transitions remain synchronous during heavy compute."""
    started = threading.Event()
    release = threading.Event()

    def blocked(_request):
        started.set()
        assert release.wait(1.0)
        return ()

    worker = FrontierExtractionWorker(blocked)
    policy = ExplorerPolicy()
    assert policy.start(HEALTHY)[0]
    worker.offer(extraction_request(1))
    assert started.wait(1.0)
    assert policy.pause().fault == 'operator pause'
    assert policy.state == ExplorerState.PAUSED
    assert policy.stop().cancel_goal is False
    assert policy.state == ExplorerState.IDLE
    release.set()
    worker.shutdown()


def test_extraction_worker_coalesces_pending_maps_to_latest_only():
    """A busy worker never queues every intermediate occupancy grid."""
    calls = []
    first_started = threading.Event()
    release_first = threading.Event()

    def extract(request):
        calls.append(request.map_sequence)
        if request.map_sequence == 1:
            first_started.set()
            assert release_first.wait(1.0)
        return ()

    worker = FrontierExtractionWorker(extract)
    worker.offer(extraction_request(1))
    assert first_started.wait(1.0)
    worker.offer(extraction_request(2))
    worker.offer(extraction_request(3))
    release_first.set()
    assert wait_for_result(worker).request.map_sequence == 1
    assert wait_for_result(worker).request.map_sequence == 3
    assert calls == [1, 3]
    worker.shutdown()


def test_extraction_freshness_rejects_pause_resume_and_new_map_results():
    """State, generation, sequence, and epoch jointly gate worker output."""
    result = FrontierExtractionResult(extraction_request(4, token=7), ())
    assert frontier_result_is_current(
        result, token=7, map_sequence=4, map_epoch=1,
        state=ExplorerState.SELECT)
    assert not frontier_result_is_current(
        result, token=8, map_sequence=4, map_epoch=1,
        state=ExplorerState.SELECT)
    assert not frontier_result_is_current(
        result, token=7, map_sequence=5, map_epoch=1,
        state=ExplorerState.SELECT)
    assert not frontier_result_is_current(
        result, token=7, map_sequence=4, map_epoch=2,
        state=ExplorerState.SELECT)
    assert not frontier_result_is_current(
        result, token=7, map_sequence=4, map_epoch=1,
        state=ExplorerState.PAUSED)


def test_wrapper_consumes_only_current_extraction_result():
    """Only a current result may publish and dispatch path validation."""
    explorer = object.__new__(FrontierExplorer)
    explorer._selection_token = 5
    explorer._map_sequence = 9
    explorer._map_epoch = 2
    explorer._last_selected_sequence = -1
    explorer.policy = ExplorerPolicy()
    assert explorer.policy.start(HEALTHY)[0]
    published = []
    validations = []
    explorer._publish_markers = (
        lambda candidates: published.append(candidates))
    explorer.get_parameter = (
        lambda _name: type('Parameter', (), {'value': 5})())
    explorer._validate_next = lambda: validations.append(
        explorer._path_candidates[0])

    stale = FrontierExtractionResult(
        extraction_request(9, token=4, epoch=2), (candidate(),))
    explorer._consume_frontier_result(stale)
    assert published == []
    assert validations == []
    assert explorer._last_selected_sequence == -1

    current = FrontierExtractionResult(
        extraction_request(9, token=5, epoch=2), (candidate(),))
    explorer._consume_frontier_result(current)
    assert published == [(candidate(),)]
    assert validations == [candidate()]
    assert explorer._last_selected_sequence == 9
    assert explorer._frontier_count == 1
    explorer._new_selection_generation()
    assert explorer._frontier_count is None


def test_worker_done_callback_only_stores_data_without_ros_side_effects():
    """Worker completion must leave policy and ROS APIs to the timer thread."""
    source = inspect.getsource(FrontierExtractionWorker._done)
    for forbidden in (
            'policy.', '_publish', '_validate_next', 'send_goal_async',
            'get_clock', 'get_logger'):
        assert forbidden not in source


def test_current_extraction_error_pauses_fail_closed_on_timer_thread():
    """A current compute failure must latch PAUSED instead of retry-spinning."""
    explorer = object.__new__(FrontierExplorer)
    explorer._selection_token = 3
    explorer._map_sequence = 6
    explorer._map_epoch = 2
    explorer.policy = ExplorerPolicy()
    assert explorer.policy.start(HEALTHY)[0]
    decisions = []
    explorer._apply = decisions.append

    failed = FrontierExtractionResult(
        extraction_request(6, token=3, epoch=2), (), RuntimeError('boom'))
    explorer._consume_frontier_result(failed)

    assert explorer.policy.state == ExplorerState.PAUSED
    assert explorer.policy.fault == (
        'frontier extraction failed: RuntimeError: boom')
    assert explorer._selection_token == 4
    assert decisions[0].fault == (
        'frontier extraction failed: RuntimeError: boom')


def test_cleanup_swallows_shutdown_interrupts_and_keeps_cleaning(monkeypatch):
    """A second Ctrl-C during entity cleanup must not escape as traceback."""
    calls = []

    class Node:
        def destroy_node(self):
            calls.append('destroy_node')
            raise KeyboardInterrupt

    class Rclpy:
        @staticmethod
        def ok():
            calls.append('ok')
            return True

        @staticmethod
        def shutdown():
            calls.append('shutdown')
            raise KeyboardInterrupt

    monkeypatch.setattr(
        frontier_explorer_module, 'ExternalShutdownException',
        type('ExternalShutdownException', (Exception,), {}), raising=False)
    monkeypatch.setattr(
        frontier_explorer_module, 'rclpy', Rclpy, raising=False)
    _cleanup_after_spin(Node())
    assert calls == ['destroy_node', 'ok', 'shutdown']


def test_prepare_map_url_prefix_creates_parent_directory(tmp_path):
    """A missing map directory is created before SaveMap is requested."""
    prefix = tmp_path / 'nested' / 'maps' / 'autonomous'
    prepared = prepare_map_url_prefix(prefix)
    assert prepared == str(prefix)
    assert prefix.parent.is_dir()


def test_prepare_map_url_prefix_propagates_directory_failure(monkeypatch):
    """Directory creation errors remain explicit for fail-closed handling."""
    error = OSError('read-only filesystem')

    def fail_mkdir(*_args, **_kwargs):
        raise error

    monkeypatch.setattr(frontier_explorer_module.Path, 'mkdir', fail_mkdir)
    try:
        prepare_map_url_prefix('/blocked/maps/autonomous')
    except OSError as caught:
        assert caught is error
    else:
        raise AssertionError('directory failure was hidden')


def test_map_save_directory_failure_never_calls_service_and_fails_closed(
        monkeypatch):
    """Manual and automatic saves fail explicitly before the ROS service."""
    class MapSaver:
        def __init__(self):
            self.calls = 0

        @staticmethod
        def service_is_ready():
            return True

        def call_async(self, _request):
            self.calls += 1
            raise AssertionError('SaveMap service must not be called')

    def fail_prepare(_prefix):
        raise OSError('permission denied')

    monkeypatch.setattr(
        frontier_explorer_module, 'prepare_map_url_prefix', fail_prepare)
    explorer = object.__new__(FrontierExplorer)
    explorer._save_in_flight = False
    explorer._last_save_status = 'none'
    explorer._last_save_error = ''
    explorer._map_saver = MapSaver()
    explorer.get_parameter = (
        lambda _name: type('Parameter', (), {'value': '/blocked/map'})())
    explorer.policy = ExplorerPolicy()
    explorer._refresh_tf = lambda: None
    explorer._readiness = lambda: HEALTHY
    response = type('Response', (), {})()

    assert explorer._manual_save_map(None, response) is response
    assert response.success is False
    assert 'permission denied' in response.message
    assert explorer._last_save_status == 'failed'
    assert 'permission denied' in explorer._last_save_error
    assert explorer.policy.state == ExplorerState.IDLE
    assert explorer._map_saver.calls == 0

    explorer.policy.state = ExplorerState.SAVING
    assert not explorer._request_map_save(automatic=True)
    assert explorer.policy.state == ExplorerState.PAUSED
    assert explorer.policy.fault == explorer._last_save_error
    assert explorer._map_saver.calls == 0


def test_map_saver_call_race_and_response_exception_are_explicit(tmp_path):
    """Save transport failures clear in-flight and preserve their causes."""
    class MapSaver:
        @staticmethod
        def service_is_ready():
            return True

        @staticmethod
        def call_async(_request):
            raise RuntimeError('service disappeared')

    explorer = object.__new__(FrontierExplorer)
    explorer._save_in_flight = False
    explorer._save_automatic = False
    explorer._last_save_status = 'none'
    explorer._last_save_error = ''
    explorer._map_saver = MapSaver()
    explorer.get_parameter = (
        lambda _name: type('Parameter', (), {
            'value': str(tmp_path / 'maps' / 'autonomous')})())
    explorer.policy = ExplorerPolicy()
    explorer._nav_send_future = None
    explorer._nav_cancel_handle = None
    explorer._refresh_tf = lambda: None
    explorer._readiness = lambda: HEALTHY
    response = type('Response', (), {})()

    explorer._manual_save_map(None, response)
    assert response.success is False
    assert 'service disappeared' in response.message
    assert explorer._last_save_status == 'failed'
    assert not explorer._save_in_flight

    explorer.policy.state = ExplorerState.SAVING
    assert not explorer._request_map_save(automatic=True)
    assert explorer.policy.state == ExplorerState.PAUSED
    assert 'service disappeared' in explorer.policy.fault
    assert not explorer._save_in_flight

    explorer._save_in_flight = True
    explorer._save_automatic = True
    explorer.policy.state = ExplorerState.SAVING
    failed_result = ControlledFuture(error=RuntimeError('reply lost'))
    explorer._save_result(failed_result)
    assert explorer._last_save_status == 'failed'
    assert explorer._last_save_error == 'map save response failed: reply lost'
    assert explorer.policy.state == ExplorerState.PAUSED
    assert explorer.policy.fault == explorer._last_save_error


def test_current_path_transport_errors_pause_but_stale_error_is_ignored():
    """Planner transport errors remain visible and never become rejection."""
    explorer = object.__new__(FrontierExplorer)
    explorer.policy = ExplorerPolicy()
    assert explorer.policy.start(HEALTHY)[0]
    assert explorer.policy.begin_validation(candidate())
    explorer._path_token = 3
    explorer._path_goal_handle = None
    explorer._nav_token = 0
    explorer._nav_goal_handle = None
    explorer._nav_send_future = None
    explorer._nav_cancel_handle = None
    explorer.get_logger = lambda: type('Logger', (), {
        'error': lambda _self, _message: None,
        'warning': lambda _self, _message: None})()

    explorer._path_goal_response(
        ControlledFuture(error=RuntimeError('goal rpc')), token=3)
    assert explorer.policy.state == ExplorerState.PAUSED
    assert explorer.policy.fault == 'path goal response failed: goal rpc'

    explorer._path_token = 3
    explorer.policy.state = ExplorerState.VALIDATE_PATH
    explorer.policy.active_goal = candidate()
    explorer._path_goal_response(
        ControlledFuture(error=RuntimeError('stale rpc')), token=2)
    assert explorer.policy.state == ExplorerState.VALIDATE_PATH

    explorer._path_result(
        ControlledFuture(error=RuntimeError('result rpc')), token=3)
    assert explorer.policy.state == ExplorerState.PAUSED
    assert explorer.policy.fault == 'path result failed: result rpc'


def test_nonempty_nav2_path_dispatches_without_racing_raw_map_snapshot():
    """Nav2's synchronized costmap result is authoritative for dispatch."""
    class Navigator:
        def __init__(self):
            self.goals = []

        def send_goal_async(self, goal, feedback_callback):
            self.goals.append((goal, feedback_callback))
            return ControlledFuture()

    explorer = object.__new__(FrontierExplorer)
    explorer.policy = ExplorerPolicy()
    assert explorer.policy.start(HEALTHY)[0]
    assert explorer.policy.begin_validation(candidate())
    explorer._path_token = 1
    explorer._nav_token = 0
    explorer._navigator = Navigator()
    explorer._navigation_transition_pending = lambda: False
    explorer.get_parameter = lambda _name: type(
        'Parameter', (), {'value': False})()
    explorer.get_clock = lambda: type('Clock', (), {
        'now': lambda _self: frontier_explorer_module.rclpy.time.Time()})()
    explorer._monotonic = lambda: 1.0
    # A concurrently refreshed raw map may mark the same cells unknown even
    # though Nav2 planned them with allow_unknown=false on its costmap snapshot.
    explorer._map = GridMap(1, 1, 0.05, 0.0, 0.0, 0.0, (-1,))
    path = type('Path', (), {'poses': [object()]})()
    result = type('Result', (), {'path': path})()
    wrapped = type('Wrapped', (), {'result': result})()

    explorer._path_result(ControlledFuture(wrapped), token=1)

    assert len(explorer._navigator.goals) == 1
    assert explorer._nav_send_future is not None


def test_path_send_and_result_request_sync_exceptions_pause():
    """Synchronous planner client races are explicit non-motion faults."""
    class Planner:
        @staticmethod
        def send_goal_async(_goal):
            raise RuntimeError('send race')

    explorer = object.__new__(FrontierExplorer)
    explorer.policy = ExplorerPolicy()
    assert explorer.policy.start(HEALTHY)[0]
    explorer._path_candidates = [candidate()]
    explorer._path_index = 0
    explorer._path_token = 0
    explorer._path_goal_handle = None
    explorer._nav_token = 0
    explorer._nav_goal_handle = None
    explorer._planner = Planner()
    explorer.get_clock = lambda: type('Clock', (), {
        'now': lambda _self: frontier_explorer_module.rclpy.time.Time()})()
    explorer.get_logger = lambda: type('Logger', (), {
        'error': lambda _self, _message: None,
        'warning': lambda _self, _message: None})()
    explorer._validate_next()
    assert explorer.policy.state == ExplorerState.PAUSED
    assert explorer.policy.fault == 'path goal send failed: send race'

    class Handle:
        accepted = True

        @staticmethod
        def get_result_async():
            raise RuntimeError('result request race')

        @staticmethod
        def cancel_goal_async():
            return ControlledFuture()

    explorer.policy.state = ExplorerState.VALIDATE_PATH
    explorer.policy.active_goal = candidate()
    explorer._path_token = 5
    explorer._path_goal_response(ControlledFuture(Handle()), token=5)
    assert explorer.policy.state == ExplorerState.PAUSED
    assert explorer.policy.fault == (
        'path result request failed: result request race')
