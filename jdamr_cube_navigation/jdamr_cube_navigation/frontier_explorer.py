"""
Fail-closed frontier exploration policy and ROS 2 orchestration node.

The policy is deliberately ROS-independent.  It owns only exploration state;
Nav2 owns motion and the collision monitor remains the sole final ``/cmd_vel``
publisher.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
import threading
import time
from typing import Callable, Optional, Sequence

from .frontier_core import (
    BlacklistEntry, FrontierCandidate, FrontierCore, GridMap)


class ExplorerState(str, Enum):
    """Externally visible explorer states."""

    IDLE = 'IDLE'
    SELECT = 'SELECT'
    VALIDATE_PATH = 'VALIDATE_PATH'
    NAVIGATE = 'NAVIGATE'
    PAUSED = 'PAUSED'
    SETTLING = 'SETTLING'
    SAVING = 'SAVING'
    FINISHED = 'FINISHED'


@dataclass(frozen=True)
class Readiness:
    """Safety predicates which must all be true before goal dispatch."""

    map_fresh: bool = False
    scan_fresh: bool = False
    tf_fresh: bool = False
    odom_fresh: bool = False
    collision_monitor_active: bool = False
    planner_ready: bool = False
    navigator_ready: bool = False
    command_owner_ok: bool = False
    battery_ok: bool = False
    stationary: bool = False

    def missing(self) -> tuple[str, ...]:
        """Return failed predicates in stable diagnostic order."""
        return tuple(name for name in (
            'map_fresh', 'scan_fresh', 'tf_fresh', 'odom_fresh',
            'collision_monitor_active', 'planner_ready', 'navigator_ready',
            'command_owner_ok', 'battery_ok',
            'stationary') if not getattr(self, name))


@dataclass(frozen=True)
class PolicyDecision:
    """Side effects requested from the ROS wrapper during one policy tick."""

    cancel_goal: bool = False
    request_save: bool = False
    fault: str = ''


def battery_voltage_ready(voltage: float, received_at: float, now: float,
                          stale_seconds: float,
                          minimum_voltage: float) -> bool:
    """Return true only for a fresh, finite voltage above the safe floor."""
    return (
        math.isfinite(voltage) and
        math.isfinite(received_at) and
        0.0 <= now - received_at <= stale_seconds and
        voltage >= minimum_voltage)


def odom_stationary_ready(linear: float, angular: float,
                          received_at: float, now: float,
                          stale_seconds: float) -> bool:
    """Require fresh finite odometry and conservative stationary velocity."""
    return (
        math.isfinite(linear) and
        math.isfinite(angular) and
        math.isfinite(received_at) and
        0.0 <= now - received_at <= stale_seconds and
        linear <= 0.01 and angular <= 0.02)


def odom_fresh_ready(received_at: float, now: float,
                     stale_seconds: float) -> bool:
    """Return whether an odometry sample timestamp is finite and fresh."""
    return (
        math.isfinite(received_at) and
        0.0 <= now - received_at <= stale_seconds)


def collision_state_ready(active: bool, received_at: float, now: float,
                          stale_seconds: float) -> bool:
    """Accept ACTIVE only while its lifecycle response remains fresh."""
    return (
        active and
        math.isfinite(received_at) and
        0.0 <= now - received_at <= stale_seconds)


def manual_save_allowed(state: ExplorerState, readiness: Readiness,
                        saver_ready: bool, save_in_flight: bool
                        ) -> tuple[bool, str]:
    """Fail closed before accepting an asynchronous manual map save."""
    if state not in (
            ExplorerState.IDLE, ExplorerState.PAUSED,
            ExplorerState.FINISHED):
        return False, f'exploration active ({state.value})'
    if not readiness.map_fresh:
        return False, 'map not fresh'
    if not readiness.odom_fresh:
        return False, 'odom not fresh'
    if not readiness.stationary:
        return False, 'robot not stationary'
    if not saver_ready:
        return False, 'map saver not ready'
    if save_in_flight:
        return False, 'map save already in progress'
    return True, 'map save request accepted'


def exploration_transition_allowed(
        save_in_flight: bool,
        navigation_pending: bool = False,
        stop_save_pending: bool = False) -> tuple[bool, str]:
    """Do not start or resume exploration while a map write is pending."""
    if save_in_flight:
        return False, 'map save in progress'
    if navigation_pending:
        return False, 'navigation send/cancel still pending'
    if stop_save_pending:
        return False, 'stop map save pending'
    return True, ''


def prepare_map_url_prefix(prefix: object) -> str:
    """Expand a map prefix and create its parent directory if necessary."""
    expanded = Path(str(prefix)).expanduser()
    expanded.parent.mkdir(parents=True, exist_ok=True)
    return str(expanded)


def command_owner_is_collision_monitor(publishers: Sequence[object]) -> bool:
    """Accept only the root-namespace collision monitor on final cmd_vel."""
    return len(publishers) == 1 and all(
        getattr(info, 'node_name', None) == 'collision_monitor' and
        getattr(info, 'node_namespace', None) == '/'
        for info in publishers)


def nav_callback_is_current(callback_token: int, current_token: int) -> bool:
    """Reject feedback and results belonging to an invalidated Nav2 goal."""
    return callback_token == current_token


@dataclass(frozen=True)
class FrontierExtractionRequest:
    """Immutable inputs captured on the ROS callback thread."""

    token: int
    map_sequence: int
    map_epoch: int
    grid: GridMap
    robot_pose: tuple[float, float, float]
    blacklist: tuple[BlacklistEntry, ...]
    now: float

    @property
    def key(self) -> tuple[int, int, int]:
        """Identify the policy generation and exact map snapshot."""
        return self.token, self.map_sequence, self.map_epoch


@dataclass(frozen=True)
class FrontierExtractionResult:
    """Worker output consumed only by the ROS timer callback."""

    request: FrontierExtractionRequest
    candidates: tuple[FrontierCandidate, ...]
    error: Optional[Exception] = None


def frontier_result_is_current(result: FrontierExtractionResult, *,
                               token: int, map_sequence: int,
                               map_epoch: int,
                               state: ExplorerState) -> bool:
    """Reject results made obsolete by state transitions or newer maps."""
    request = result.request
    return (
        result.error is None and
        state == ExplorerState.SELECT and
        request.token == token and
        request.map_sequence == map_sequence and
        request.map_epoch == map_epoch)


class FrontierExtractionWorker:
    """Run one extraction at a time with a single latest-only pending slot."""

    def __init__(self, extractor: Callable[[FrontierExtractionRequest],
                                           Sequence[FrontierCandidate]],
                 executor=None) -> None:
        self._extractor = extractor
        self._executor = executor or ThreadPoolExecutor(
            max_workers=1, thread_name_prefix='frontier-extraction')
        self._owns_executor = executor is None
        self._lock = threading.RLock()
        self._future: Optional[Future] = None
        self._active_key: Optional[tuple[int, int, int]] = None
        self._pending: Optional[FrontierExtractionRequest] = None
        self._completed: Optional[FrontierExtractionResult] = None
        self._closed = False

    def offer(self, request: FrontierExtractionRequest) -> None:
        """Submit immediately or replace the sole pending request."""
        with self._lock:
            if self._closed:
                return
            completed_key = (
                self._completed.request.key
                if self._completed is not None else None)
            pending_key = self._pending.key if self._pending else None
            if request.key in (self._active_key, pending_key, completed_key):
                return
            self._pending = request
            self._start_pending_locked()

    def poll(self) -> Optional[FrontierExtractionResult]:
        """Take one completed result and start the newest pending request."""
        with self._lock:
            result = self._completed
            self._completed = None
            self._start_pending_locked()
            return result

    def shutdown(self) -> None:
        """Reject queued work and join the owned worker thread cleanly."""
        with self._lock:
            self._closed = True
            self._pending = None
            future = self._future
        if future is not None:
            future.cancel()
        if self._owns_executor:
            self._executor.shutdown(wait=True, cancel_futures=True)

    def _start_pending_locked(self) -> None:
        if (self._closed or self._future is not None or
                self._completed is not None or self._pending is None):
            return
        request = self._pending
        self._pending = None
        self._active_key = request.key
        future = self._executor.submit(self._run, request)
        self._future = future
        future.add_done_callback(self._done)

    def _run(self, request: FrontierExtractionRequest
             ) -> FrontierExtractionResult:
        try:
            candidates = tuple(self._extractor(request))
            return FrontierExtractionResult(request, candidates)
        except Exception as error:
            return FrontierExtractionResult(request, (), error)

    def _done(self, future: Future) -> None:
        # This callback runs on the worker. It stores data only: no ROS API,
        # policy transition, marker publication, or Nav2 dispatch is allowed.
        with self._lock:
            if future is not self._future:
                return
            self._future = None
            self._active_key = None
            if not self._closed:
                self._completed = future.result()


class ExplorerPolicy:
    """Fail-closed state machine for one-goal-at-a-time mapping."""

    def __init__(self, *, stall_timeout: float = 20.0,
                 settle_seconds: float = 10.0, empty_cycles: int = 3,
                 max_recoveries: int = 2, blacklist_seconds: float = 90.0,
                 blacklist_radius: float = 0.75) -> None:
        """Initialize policy thresholds without enabling exploration."""
        self.state = ExplorerState.IDLE
        self.fault = ''
        self.active_goal: Optional[FrontierCandidate] = None
        self.blacklist: list[BlacklistEntry] = []
        self.stall_timeout = stall_timeout
        self.settle_seconds = settle_seconds
        self.empty_cycles_required = empty_cycles
        self.max_recoveries = max_recoveries
        self.blacklist_seconds = blacklist_seconds
        self.blacklist_radius = blacklist_radius
        self._best_distance = math.inf
        self._last_progress_at = 0.0
        self._last_empty_generation: Optional[int] = None
        self._empty_cycles = 0
        self._settling_since: Optional[float] = None
        self._save_requested = False

    def start(self, readiness: Readiness) -> tuple[bool, str]:
        """Start only when every readiness predicate is proven healthy."""
        if self.state not in (ExplorerState.IDLE, ExplorerState.FINISHED):
            return False, f'already {self.state.value}'
        return self._enter_select(readiness)

    def resume(self, readiness: Readiness) -> tuple[bool, str]:
        """Resume a latched pause; faults never auto-resume."""
        if self.state != ExplorerState.PAUSED:
            return False, f'not paused ({self.state.value})'
        return self._enter_select(readiness)

    def pause(self, reason: str = 'operator pause') -> PolicyDecision:
        """Latch PAUSED and cancel an active goal."""
        cancel = self.active_goal is not None
        self.active_goal = None
        self.state = ExplorerState.PAUSED
        self.fault = reason
        return PolicyDecision(cancel_goal=cancel, fault=reason)

    def stop(self) -> PolicyDecision:
        """Stop exploration without automatic motion or restart."""
        cancel = self.active_goal is not None
        self.active_goal = None
        self.state = ExplorerState.IDLE
        self.fault = ''
        self._reset_completion()
        return PolicyDecision(cancel_goal=cancel)

    def tick(self, now: float, readiness: Readiness,
             probe_abort: bool = False, generation: int = 0) -> PolicyDecision:
        """Apply health, abort, stall, and completion gates once."""
        if probe_abort and self.state not in (
                ExplorerState.IDLE, ExplorerState.FINISHED):
            return self.pause('probe_abort')
        if self.state in (
                ExplorerState.SELECT, ExplorerState.VALIDATE_PATH,
                ExplorerState.NAVIGATE, ExplorerState.SETTLING):
            # Motion is expected during navigation; stationary gates only
            # start, resume, and final map saving.
            missing = tuple(item for item in readiness.missing()
                            if item != 'stationary')
            if missing:
                return self.pause('readiness: ' + ','.join(missing))
        if (self.state == ExplorerState.NAVIGATE and
                now - self._last_progress_at >= self.stall_timeout):
            return self.fail_goal(now, generation, 'navigation stalled')
        if (self.state == ExplorerState.SETTLING and
                self._settling_since is not None and
                now - self._settling_since >= self.settle_seconds and
                readiness.stationary and not self._save_requested):
            self._save_requested = True
            self.state = ExplorerState.SAVING
            return PolicyDecision(request_save=True)
        return PolicyDecision()

    def begin_validation(self, goal: FrontierCandidate) -> bool:
        """Reserve one candidate for path validation."""
        if self.state != ExplorerState.SELECT or self.active_goal is not None:
            return False
        self.active_goal = goal
        self.state = ExplorerState.VALIDATE_PATH
        return True

    def path_rejected(self) -> None:
        """Release a candidate whose computed path was unsafe or empty."""
        if self.state == ExplorerState.VALIDATE_PATH:
            self.active_goal = None
            self.state = ExplorerState.SELECT

    def navigation_started(self, now: float) -> bool:
        """Mark the validated candidate as the sole active Nav2 goal."""
        if (self.state != ExplorerState.VALIDATE_PATH or
                self.active_goal is None):
            return False
        self.state = ExplorerState.NAVIGATE
        self._best_distance = math.inf
        self._last_progress_at = now
        return True

    def feedback(self, now: float, distance_remaining: float,
                 recoveries: int, generation: int = 0) -> PolicyDecision:
        """Track meaningful progress and fail on excessive recovery."""
        if self.state != ExplorerState.NAVIGATE:
            return PolicyDecision()
        if recoveries > self.max_recoveries:
            return self.fail_goal(
                now, generation, 'recovery limit exceeded')
        if distance_remaining + 0.05 < self._best_distance:
            self._best_distance = distance_remaining
            self._last_progress_at = now
        return PolicyDecision()

    def navigation_succeeded(self) -> None:
        """Release the completed goal and request fresh frontier selection."""
        self.active_goal = None
        self.state = ExplorerState.SELECT
        self._reset_completion()

    def fail_goal(self, now: float, generation: int,
                  reason: str) -> PolicyDecision:
        """Blacklist the failed region before returning to selection."""
        if self.active_goal is not None:
            self.blacklist.append(BlacklistEntry(
                self.active_goal.x, self.active_goal.y,
                self.blacklist_radius, now + self.blacklist_seconds,
                generation))
        cancel = self.state == ExplorerState.NAVIGATE
        self.active_goal = None
        self.state = ExplorerState.SELECT
        self.fault = reason
        return PolicyDecision(cancel_goal=cancel, fault=reason)

    def observe_frontiers(self, count: int, generation: int,
                          now: float) -> None:
        """Count distinct fresh-map cycles before declaring completion."""
        if self.state != ExplorerState.SELECT:
            return
        if count:
            self._reset_completion()
            return
        if generation == self._last_empty_generation:
            return
        self._last_empty_generation = generation
        self._empty_cycles += 1
        if self._empty_cycles >= self.empty_cycles_required:
            self._settling_since = now
            self.state = ExplorerState.SETTLING

    def save_result(self, success: bool) -> None:
        """Finish only after map saver confirms success."""
        if self.state != ExplorerState.SAVING:
            return
        if success:
            self.state = ExplorerState.FINISHED
            self.fault = ''
        else:
            self.state = ExplorerState.PAUSED
            self.fault = 'map save failed'

    def _enter_select(self, readiness: Readiness) -> tuple[bool, str]:
        missing = readiness.missing()
        if missing:
            reason = 'not ready: ' + ','.join(missing)
            self.fault = reason
            return False, reason
        self.state = ExplorerState.SELECT
        self.fault = ''
        self.active_goal = None
        self._reset_completion()
        return True, 'ready'

    def _reset_completion(self) -> None:
        self._empty_cycles = 0
        self._last_empty_generation = None
        self._settling_since = None
        self._save_requested = False


# ROS imports stay below the pure policy so unit tests can exercise it without
# constructing a ROS context.
try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import Point, PoseStamped
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav2_msgs.msg import CollisionMonitorState
    from nav2_msgs.srv import SaveMap
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState, LaserScan
    from std_msgs.msg import Empty, String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray
    ROS_AVAILABLE = True
    ROS_IMPORT_ERROR = None
except ModuleNotFoundError as error:
    # Pure policy tests intentionally need no ROS setup.
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = error
    Node = object


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pose(candidate: FrontierCandidate, stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = stamp
    pose.pose.position.x = candidate.x
    pose.pose.position.y = candidate.y
    pose.pose.orientation.z = math.sin(candidate.yaw / 2.0)
    pose.pose.orientation.w = math.cos(candidate.yaw / 2.0)
    return pose


class FrontierExplorer(Node):
    """Thin ROS wrapper around :class:`ExplorerPolicy` and Nav2 actions."""

    def __init__(self) -> None:
        """Create fail-closed subscriptions, services, and action clients."""
        super().__init__('frontier_explorer')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('top_k', 5)
        self.declare_parameter('map_stale_seconds', 3.0)
        self.declare_parameter('scan_stale_seconds', 0.5)
        self.declare_parameter('tf_stale_seconds', 0.5)
        self.declare_parameter('odom_stale_seconds', 0.5)
        self.declare_parameter('collision_state_stale_seconds', 1.0)
        self.declare_parameter('cancel_timeout_seconds', 1.0)
        self.declare_parameter('battery_stale_seconds', 2.5)
        self.declare_parameter('min_battery_voltage', 10.5)
        self.declare_parameter('map_url_prefix', '/home/lim/maps/autonomous')
        self.policy = ExplorerPolicy()
        self.core = FrontierCore()
        self._map: Optional[GridMap] = None
        self._map_received = -math.inf
        self._scan_received = -math.inf
        self._battery_received = -math.inf
        self._battery_voltage = math.nan
        self._odom_received = -math.inf
        self._tf_fresh = False
        self._map_epoch = 0
        self._map_sequence = 0
        self._map_geometry = None
        self._last_selected_sequence = -1
        self._frontier_count: Optional[int] = None
        self._selection_token = 0
        self._robot_pose: Optional[tuple[float, float, float]] = None
        self._linear = math.inf
        self._angular = math.inf
        self._collision_active = False
        self._collision_state_received = -math.inf
        self._collision_query_future = None
        self._collision_query_started = -math.inf
        self._collision_query_token = 0
        self._collision_query_error = ''
        self._probe_abort = False
        self._path_candidates: list[FrontierCandidate] = []
        self._path_index = 0
        self._path_token = 0
        self._nav_token = 0
        self._path_goal_handle = None
        self._nav_goal_handle = None
        self._nav_send_future = None
        self._nav_send_started = -math.inf
        self._nav_cancel_future = None
        self._nav_cancel_handle = None
        self._nav_cancel_started = -math.inf
        self._nav_cancel_token = 0
        self._shutdown_requested = False
        self._save_in_flight = False
        self._save_automatic = False
        self._stop_save_pending = False
        self._last_save_status = 'none'
        self._last_save_error = ''
        # Do not shadow rclpy.node.Node._clock; timers and get_clock() own it.
        self._monotonic = time.monotonic
        self._frontier_worker = FrontierExtractionWorker(
            self._extract_frontiers)

        self._planner = ActionClient(self, ComputePathToPose,
                                     'compute_path_to_pose')
        self._navigator = ActionClient(self, NavigateToPose,
                                       'navigate_to_pose')
        self._lifecycle = self.create_client(
            GetState, '/collision_monitor/get_state')
        self._map_saver = self.create_client(SaveMap, '/map_saver/save_map')
        self._status_pub = self.create_publisher(
            String, '/frontier_explorer/status', 10)
        self._markers_pub = self.create_publisher(
            MarkerArray, '/frontier_explorer/markers', 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 1)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(
            BatteryState, '/battery_state', self._on_battery, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Empty, '/probe_abort', self._on_abort, 10)
        self.create_subscription(CollisionMonitorState,
                                 '/collision_monitor_state',
                                 self._on_collision_state, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        for name, callback in (
                ('start', self._start), ('pause', self._pause),
                ('resume', self._resume), ('stop', self._stop)):
            self.create_service(
                Trigger, f'/frontier_explorer/{name}', callback)
            self.create_service(Trigger, f'/autonomy/{name}', callback)
        self.create_service(
            Trigger, '/autonomy/save_map', self._manual_save_map)
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._poll_collision_lifecycle)

    def _on_map(self, msg: OccupancyGrid) -> None:
        q = msg.info.origin.orientation
        geometry = (
            msg.header.frame_id, msg.info.width, msg.info.height,
            msg.info.resolution, msg.info.origin.position.x,
            msg.info.origin.position.y, _yaw_from_quaternion(q))
        if geometry != self._map_geometry:
            self._map_epoch += 1
            self._map_geometry = geometry
        self._map = GridMap(
            msg.info.width, msg.info.height, msg.info.resolution,
            msg.info.origin.position.x, msg.info.origin.position.y,
            _yaw_from_quaternion(q), tuple(msg.data))
        self._map_received = self._monotonic()
        self._map_sequence += 1
        self._frontier_count = None

    def _on_scan(self, _msg: LaserScan) -> None:
        self._scan_received = self._monotonic()

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_voltage = float(msg.voltage)
        self._battery_received = self._monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self._linear = math.hypot(msg.twist.twist.linear.x,
                                  msg.twist.twist.linear.y)
        self._angular = abs(msg.twist.twist.angular.z)
        self._odom_received = self._monotonic()

    def _on_abort(self, _msg: Empty) -> None:
        self._probe_abort = True

    def _on_collision_state(self, _msg: CollisionMonitorState) -> None:
        # Observability only. Lifecycle state is the readiness authority.
        pass

    def _poll_collision_lifecycle(self) -> None:
        if not self._lifecycle.service_is_ready():
            self._invalidate_collision_query()
            return
        if self._collision_query_future is not None:
            age = self._monotonic() - self._collision_query_started
            timeout = float(self.get_parameter(
                'collision_state_stale_seconds').value)
            if age <= timeout:
                return
            self._invalidate_collision_query()
        self._collision_query_token += 1
        token = self._collision_query_token
        try:
            future = self._lifecycle.call_async(GetState.Request())
        except Exception as error:
            self._invalidate_collision_query()
            self._collision_query_error = (
                f'{type(error).__name__}: {error}')
            return
        self._collision_query_future = future
        self._collision_query_started = self._monotonic()
        future.add_done_callback(
            lambda result: self._collision_state_result(result, token))

    def _collision_state_result(self, future, token: int) -> None:
        if (token != self._collision_query_token or
                future is not self._collision_query_future):
            return
        self._collision_query_future = None
        age = self._monotonic() - self._collision_query_started
        timeout = float(self.get_parameter(
            'collision_state_stale_seconds').value)
        self._collision_query_started = -math.inf
        if age > timeout:
            self._collision_active = False
            self._collision_state_received = -math.inf
            self._collision_query_token += 1
            return
        try:
            self._collision_active = (
                future.result().current_state.id == State.PRIMARY_STATE_ACTIVE)
            self._collision_state_received = self._monotonic()
            self._collision_query_error = ''
        except Exception as error:
            self._collision_active = False
            self._collision_state_received = -math.inf
            self._collision_query_error = (
                f'{type(error).__name__}: {error}')

    def _invalidate_collision_query(self) -> None:
        """Fail closed and reject any delayed lifecycle service response."""
        self._collision_query_token += 1
        self._collision_active = False
        self._collision_state_received = -math.inf
        future = self._collision_query_future
        self._collision_query_future = None
        self._collision_query_started = -math.inf
        if future is not None:
            future.cancel()

    def _refresh_tf(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception:
            self._tf_fresh = False
            return
        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1.0e9
        maximum_age = float(self.get_parameter('tf_stale_seconds').value)
        self._tf_fresh = 0.0 <= age <= maximum_age
        if not self._tf_fresh:
            return
        translation = transform.transform.translation
        self._robot_pose = (
            translation.x, translation.y,
            _yaw_from_quaternion(transform.transform.rotation))

    def _command_owner_ok(self) -> bool:
        return command_owner_is_collision_monitor(
            self.get_publishers_info_by_topic('/cmd_vel'))

    def _readiness(self, now: Optional[float] = None) -> Readiness:
        current = self._monotonic() if now is None else now
        return Readiness(
            map_fresh=current - self._map_received <= float(
                self.get_parameter('map_stale_seconds').value),
            scan_fresh=current - self._scan_received <= float(
                self.get_parameter('scan_stale_seconds').value),
            tf_fresh=self._tf_fresh,
            odom_fresh=odom_fresh_ready(
                self._odom_received, current, float(
                    self.get_parameter('odom_stale_seconds').value)),
            collision_monitor_active=collision_state_ready(
                self._collision_active, self._collision_state_received,
                current, float(self.get_parameter(
                    'collision_state_stale_seconds').value)),
            planner_ready=self._planner.server_is_ready(),
            navigator_ready=self._navigator.server_is_ready(),
            command_owner_ok=self._command_owner_ok(),
            battery_ok=battery_voltage_ready(
                self._battery_voltage, self._battery_received, current,
                float(self.get_parameter('battery_stale_seconds').value),
                float(self.get_parameter('min_battery_voltage').value)),
            stationary=(
                math.isfinite(self._linear) and
                math.isfinite(self._angular) and
                self._linear <= 0.01 and self._angular <= 0.02))

    def _start(self, _request, response):
        allowed, message = exploration_transition_allowed(
            self._save_in_flight, self._navigation_transition_pending(),
            getattr(self, '_stop_save_pending', False))
        if not allowed:
            response.success, response.message = allowed, message
            return response
        self._refresh_tf()
        response.success, response.message = self.policy.start(
            self._readiness())
        if response.success:
            self._new_selection_generation()
        return response

    def _pause(self, _request, response):
        self._invalidate_selection()
        self._apply(self.policy.pause())
        response.success = True
        response.message = (
            'pause requested; navigation cancellation pending'
            if self._navigation_transition_pending() else 'paused')
        return response

    def _resume(self, _request, response):
        allowed, message = exploration_transition_allowed(
            self._save_in_flight, self._navigation_transition_pending(),
            getattr(self, '_stop_save_pending', False))
        if not allowed:
            response.success, response.message = allowed, message
            return response
        self._refresh_tf()
        response.success, response.message = self.policy.resume(
            self._readiness())
        if response.success:
            self._new_selection_generation()
        return response

    def _stop(self, _request, response):
        if (getattr(self, '_stop_save_pending', False) or
                self._save_in_flight or
                self.policy.state == ExplorerState.SAVING):
            response.success = True
            response.message = (
                'stop requested; navigation cancellation pending; '
                'map save pending'
                if self._navigation_transition_pending()
                else 'stop requested; map save pending')
            return response
        self._invalidate_selection()
        self._apply(self.policy.stop())
        self._stop_save_pending = True
        self._last_save_status = 'waiting_for_stop'
        response.success = True
        response.message = (
            'stop requested; navigation cancellation pending; map save pending'
            if self._navigation_transition_pending()
            else 'stop requested; map save pending')
        return response

    def _manual_save_map(self, _request, response):
        if getattr(self, '_stop_save_pending', False):
            response.success = False
            response.message = 'stop map save pending'
            return response
        if self._navigation_transition_pending():
            response.success = False
            response.message = 'navigation send/cancel still pending'
            return response
        self._refresh_tf()
        allowed, message = manual_save_allowed(
            self.policy.state, self._readiness(),
            self._map_saver.service_is_ready(), self._save_in_flight)
        response.success = allowed
        response.message = message
        if allowed and not self._request_map_save(automatic=False):
            response.success = False
            response.message = (
                self._last_save_error or 'map save request rejected')
        return response

    def _tick(self) -> None:
        now = self._monotonic()
        self._check_navigation_transition_timeout(now)
        if self._shutdown_requested:
            return
        self._refresh_tf()
        readiness = self._readiness(now)
        if self._stop_save_pending:
            self._tick_stop_save(readiness)
            self._probe_abort = False
            self._publish_status()
            return
        previous_state = self.policy.state
        decision = self.policy.tick(
            now, readiness, self._probe_abort, self._map_epoch)
        self._probe_abort = False
        if (previous_state == ExplorerState.SELECT and
                self.policy.state != ExplorerState.SELECT):
            self._invalidate_selection()
        self._apply(decision)
        if self.policy.state == ExplorerState.SELECT:
            self._select(now)
        self._publish_status()

    def _tick_stop_save(self, readiness: Readiness) -> None:
        """Save exactly once after stop cancellation and safe standstill."""
        if not self._stop_save_pending:
            return
        if self._navigation_transition_pending():
            return
        if not (
                readiness.map_fresh and readiness.odom_fresh and
                readiness.stationary and
                self._map_saver.service_is_ready()):
            return
        self._stop_save_pending = False
        self.policy.state = ExplorerState.SAVING
        if not self._request_map_save(automatic=True):
            self.policy.fault = (
                self._last_save_error or 'automatic stop map save failed')

    def _select(self, now: float) -> None:
        if self._navigation_transition_pending():
            return
        result = self._frontier_worker.poll()
        if result is not None:
            self._consume_frontier_result(result)
        if self.policy.state != ExplorerState.SELECT:
            return
        if self._map is None or self._robot_pose is None:
            return
        if self._last_selected_sequence == self._map_sequence:
            return
        if not self.core.start_is_safe(
                self._map, self._robot_pose[0], self._robot_pose[1]):
            self._invalidate_selection()
            self._apply(self.policy.pause(
                'robot pose is not a safe known-free start'))
            return
        self._frontier_worker.offer(FrontierExtractionRequest(
            token=self._selection_token,
            map_sequence=self._map_sequence,
            map_epoch=self._map_epoch,
            grid=self._map,
            robot_pose=self._robot_pose,
            blacklist=tuple(self.policy.blacklist),
            now=now))

    def _extract_frontiers(
            self, request: FrontierExtractionRequest
            ) -> Sequence[FrontierCandidate]:
        """Perform CPU-only frontier extraction on the worker thread."""
        return self.core.extract(
            request.grid, *request.robot_pose,
            blacklist=request.blacklist, now=request.now,
            generation=request.map_epoch)

    def _consume_frontier_result(
            self, result: FrontierExtractionResult) -> None:
        """Apply a fresh worker result from the ROS timer callback only."""
        request = result.request
        if result.error is not None:
            if (
                    self.policy.state == ExplorerState.SELECT and
                    request.token == self._selection_token and
                    request.map_sequence == self._map_sequence and
                    request.map_epoch == self._map_epoch):
                self._invalidate_selection()
                reason = (
                    'frontier extraction failed: '
                    f'{type(result.error).__name__}: {result.error}')
                self._apply(self.policy.pause(reason))
            return
        if not frontier_result_is_current(
                result, token=self._selection_token,
                map_sequence=self._map_sequence,
                map_epoch=self._map_epoch, state=self.policy.state):
            return
        self._last_selected_sequence = result.request.map_sequence
        candidates = result.candidates
        self._frontier_count = len(candidates)
        self.policy.observe_frontiers(
            len(candidates), result.request.map_sequence, result.request.now)
        self._publish_markers(candidates)
        if not candidates or self.policy.state != ExplorerState.SELECT:
            return
        self._path_candidates = candidates[:max(
            1, min(5, int(self.get_parameter('top_k').value)))]
        self._path_index = 0
        self._validate_next()

    def _new_selection_generation(self) -> None:
        """Begin selection after a successful explicit start or resume."""
        self._selection_token += 1
        self._last_selected_sequence = -1
        self._frontier_count = None

    def _invalidate_selection(self) -> None:
        """Make every in-flight extraction result stale immediately."""
        self._selection_token += 1

    def _validate_next(self) -> None:
        if self._path_index >= len(self._path_candidates):
            return
        candidate = self._path_candidates[self._path_index]
        if not self.policy.begin_validation(candidate):
            return
        goal = ComputePathToPose.Goal()
        goal.goal = _pose(candidate, self.get_clock().now().to_msg())
        goal.use_start = False
        self._path_token += 1
        token = self._path_token
        try:
            future = self._planner.send_goal_async(goal)
        except Exception as error:
            self._apply(self.policy.pause(
                f'path goal send failed: {error}'))
            return
        future.add_done_callback(
            lambda result: self._path_goal_response(result, token))

    def _path_goal_response(self, future, token: int) -> None:
        try:
            handle = future.result()
        except Exception as error:
            if (token == self._path_token and
                    self.policy.state == ExplorerState.VALIDATE_PATH):
                self._apply(self.policy.pause(
                    f'path goal response failed: {error}'))
            return
        if handle is None or not handle.accepted:
            self._try_next_path(token)
            return
        if (token != self._path_token or
                self.policy.state != ExplorerState.VALIDATE_PATH):
            self._best_effort_path_cancel(handle)
            return
        self._path_goal_handle = handle
        try:
            result_future = handle.get_result_async()
        except Exception as error:
            self._apply(self.policy.pause(
                f'path result request failed: {error}'))
            return
        result_future.add_done_callback(
            lambda result: self._path_result(result, token))

    def _path_result(self, future, token: int) -> None:
        if (token != self._path_token or
                self.policy.state != ExplorerState.VALIDATE_PATH):
            return
        try:
            wrapped = future.result()
            path = wrapped.result.path
            valid = bool(path.poses) and self._path_known_free(path.poses)
        except Exception as error:
            self._apply(self.policy.pause(
                f'path result failed: {error}'))
            return
        if not valid:
            self._try_next_path(token)
            return
        candidate = self.policy.active_goal
        if candidate is None:
            return
        if bool(self.get_parameter('dry_run').value):
            self.policy.path_rejected()
            return
        if self._navigation_transition_pending():
            self._apply(self.policy.pause('navigation transition pending'))
            return
        goal = NavigateToPose.Goal()
        goal.pose = _pose(candidate, self.get_clock().now().to_msg())
        self._nav_token += 1
        token = self._nav_token
        try:
            future = self._navigator.send_goal_async(
                goal,
                feedback_callback=lambda msg: self._nav_feedback(msg, token))
        except Exception as error:
            self._trigger_fail_safe(
                f'navigation goal send failed: {error}')
            return
        self._nav_send_future = future
        self._nav_send_started = self._monotonic()
        future.add_done_callback(
            lambda result: self._nav_goal_response(result, token))

    def _try_next_path(self, token: int) -> None:
        if token != self._path_token:
            return
        self.policy.path_rejected()
        self._path_index += 1
        self._validate_next()

    def _path_known_free(self, poses: Sequence[PoseStamped]) -> bool:
        if self._map is None:
            return False
        for stamped in poses:
            cell = self._map.world_to_cell(
                stamped.pose.position.x, stamped.pose.position.y)
            if cell is None or not self.core.is_free(self._map.value(cell)):
                return False
        return True

    def _nav_goal_response(self, future, token: int) -> None:
        if future is not self._nav_send_future:
            return
        self._nav_send_future = None
        self._nav_send_started = -math.inf
        try:
            handle = future.result()
        except Exception as error:
            self._trigger_fail_safe(
                f'navigation goal response failed: {error}')
            return
        if not nav_callback_is_current(token, self._nav_token):
            if handle is not None and handle.accepted:
                self._track_stale_accepted_goal(handle)
            return
        if handle is None or not handle.accepted:
            self.policy.fail_goal(
                self._monotonic(), self._map_epoch,
                'navigation goal rejected')
            return
        self._nav_goal_handle = handle
        try:
            result_future = handle.get_result_async()
        except Exception as error:
            self._trigger_fail_safe(
                f'navigation result request failed: {error}')
            return
        result_future.add_done_callback(
            lambda result: self._nav_result(result, token, handle))
        if not self.policy.navigation_started(self._monotonic()):
            # A pause/stop may have arrived while goal acceptance was pending.
            self._begin_nav_cancel(handle)
            return

    def _nav_feedback(self, msg, token: int) -> None:
        if not nav_callback_is_current(token, self._nav_token):
            return
        feedback = msg.feedback
        self._apply(self.policy.feedback(
            self._monotonic(), feedback.distance_remaining,
            feedback.number_of_recoveries, self._map_epoch))

    def _nav_result(self, future, token: int, handle) -> None:
        if handle is self._nav_cancel_handle:
            self._nav_cancel_terminal(future, handle)
            return
        if (not nav_callback_is_current(token, self._nav_token) or
                handle is not self._nav_goal_handle):
            return
        try:
            status = future.result().status
        except Exception as error:
            self._trigger_fail_safe(
                f'navigation result failed: {error}')
            return
        if not self._goal_status_is_terminal(status):
            self._trigger_fail_safe(
                f'navigation result was not terminal ({status})')
            return
        self._nav_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.policy.navigation_succeeded()
        elif self.policy.state == ExplorerState.NAVIGATE:
            self.policy.fail_goal(
                self._monotonic(), self._map_epoch,
                f'navigation failed ({status})')

    def _navigation_transition_pending(self) -> bool:
        """Return whether goal send or cancellation is not yet terminal."""
        return (
            getattr(self, '_nav_send_future', None) is not None or
            getattr(self, '_nav_cancel_handle', None) is not None)

    def _track_stale_accepted_goal(self, handle) -> None:
        """Attach terminal tracking before cancelling a late acceptance."""
        if (self._nav_goal_handle is not None and
                self._nav_goal_handle is not handle):
            self._trigger_fail_safe('multiple navigation goals detected')
            return
        self._nav_goal_handle = handle
        self._begin_nav_cancel(handle)
        if handle is not self._nav_cancel_handle:
            return
        try:
            result_future = handle.get_result_async()
        except Exception as error:
            self._trigger_fail_safe(
                f'navigation result request failed: {error}')
            return
        result_future.add_done_callback(
            lambda result: self._nav_cancel_terminal(result, handle))

    def _begin_nav_cancel(self, handle) -> None:
        """Track cancellation until Nav2 confirms a terminal goal result."""
        if self._nav_cancel_handle is handle:
            return
        if self._nav_cancel_handle is not None:
            self._trigger_fail_safe('multiple navigation cancels detected')
            return
        self._nav_cancel_handle = handle
        self._nav_cancel_started = self._monotonic()
        self._nav_cancel_token += 1
        token = self._nav_cancel_token
        try:
            future = handle.cancel_goal_async()
        except Exception as error:
            self._trigger_fail_safe(f'navigation cancel failed: {error}')
            return
        self._nav_cancel_future = future
        future.add_done_callback(
            lambda result: self._nav_cancel_response(result, token, handle))

    def _nav_cancel_response(self, future, token: int, handle) -> None:
        if (token != self._nav_cancel_token or
                handle is not self._nav_cancel_handle or
                future is not self._nav_cancel_future):
            return
        self._nav_cancel_future = None
        try:
            return_code = future.result().return_code
        except Exception as error:
            self._trigger_fail_safe(
                f'navigation cancel response failed: {error}')
            return
        if return_code == CancelGoal.Response.ERROR_NONE:
            return
        if return_code == CancelGoal.Response.ERROR_GOAL_TERMINATED:
            self._finish_nav_cancel()
            return
        self._trigger_fail_safe(
            f'navigation cancel rejected ({return_code})')

    def _nav_cancel_terminal(self, future, handle) -> None:
        if handle is not self._nav_cancel_handle:
            return
        try:
            status = future.result().status
        except Exception as error:
            self._trigger_fail_safe(
                f'navigation cancel result failed: {error}')
            return
        if not self._goal_status_is_terminal(status):
            self._trigger_fail_safe(
                f'navigation cancel result was not terminal ({status})')
            return
        self._finish_nav_cancel()

    @staticmethod
    def _goal_status_is_terminal(status: int) -> bool:
        return status in (
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED)

    def _finish_nav_cancel(self) -> None:
        """Clear the motion transition only after terminal confirmation."""
        handle = self._nav_cancel_handle
        self._nav_cancel_token += 1
        self._nav_cancel_future = None
        self._nav_cancel_handle = None
        self._nav_cancel_started = -math.inf
        if self._nav_goal_handle is handle:
            self._nav_goal_handle = None

    def _check_navigation_transition_timeout(self, now: float) -> None:
        timeout = float(self.get_parameter('cancel_timeout_seconds').value)
        if (self._nav_send_future is not None and
                now - self._nav_send_started >= timeout):
            self._trigger_fail_safe('navigation goal send timed out')
        elif (self._nav_cancel_handle is not None and
                now - self._nav_cancel_started >= timeout):
            self._trigger_fail_safe('navigation cancel timed out')

    def _trigger_fail_safe(self, reason: str) -> None:
        """Latch a fault and exit so the launch-level shutdown boundary runs."""
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self._path_token += 1
        self._nav_token += 1
        self.policy.pause(reason)
        self.policy.fault = reason
        self._request_process_shutdown(reason)

    def _request_process_shutdown(self, reason: str) -> None:
        self.get_logger().fatal(reason)
        if rclpy.ok():
            rclpy.shutdown()

    def _best_effort_path_cancel(self, handle) -> None:
        try:
            future = handle.cancel_goal_async()
        except Exception as error:
            self.get_logger().warning(
                f'path validation cancel failed: {error}')
            return
        future.add_done_callback(self._path_cancel_result)

    def _path_cancel_result(self, future) -> None:
        try:
            future.result()
        except Exception as error:
            self.get_logger().warning(
                f'path validation cancel response failed: {error}')

    def _apply(self, decision: PolicyDecision) -> None:
        if decision.cancel_goal:
            # Invalidates even a send_goal future that has no handle yet.
            self._path_token += 1
            self._nav_token += 1
        if (decision.cancel_goal and self._path_goal_handle is not None):
            self._best_effort_path_cancel(self._path_goal_handle)
            self._path_goal_handle = None
        if (decision.cancel_goal and self._nav_goal_handle is not None):
            self._begin_nav_cancel(self._nav_goal_handle)
        if decision.request_save:
            self._request_map_save(automatic=True)
        if decision.fault:
            self.get_logger().error(decision.fault)

    def _request_map_save(self, *, automatic: bool) -> bool:
        if self._save_in_flight or not self._map_saver.service_is_ready():
            if automatic:
                self.policy.save_result(False)
            self._last_save_status = 'rejected'
            self._last_save_error = 'map saver unavailable or save in progress'
            return False
        try:
            prefix = prepare_map_url_prefix(
                self.get_parameter('map_url_prefix').value)
        except OSError as error:
            self._last_save_error = (
                f'map save directory unavailable: {error}')
            self._last_save_status = 'failed'
            if automatic:
                self.policy.save_result(False)
            self.policy.fault = self._last_save_error
            return False
        request = SaveMap.Request()
        request.map_topic = '/map'
        request.map_url = (
            f'{prefix}_'
            f"{time.strftime('%Y%m%dT%H%M%S')}")
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        self._save_in_flight = True
        self._save_automatic = automatic
        self._last_save_status = 'in_progress'
        self._last_save_error = ''
        try:
            future = self._map_saver.call_async(request)
            future.add_done_callback(self._save_result)
        except Exception as error:
            self._save_in_flight = False
            self._last_save_status = 'failed'
            self._last_save_error = f'map save request failed: {error}'
            if automatic:
                self.policy.save_result(False)
            self.policy.fault = self._last_save_error
            self._save_automatic = False
            return False
        return True

    def _save_result(self, future) -> None:
        self._save_in_flight = False
        try:
            success = bool(future.result().result)
        except Exception as error:
            self._last_save_status = 'failed'
            self._last_save_error = f'map save response failed: {error}'
            if self._save_automatic:
                self.policy.save_result(False)
            self.policy.fault = self._last_save_error
            self._save_automatic = False
            return
        self._last_save_status = 'succeeded' if success else 'failed'
        if self._save_automatic:
            self.policy.save_result(success)
        elif not success:
            self.policy.fault = 'manual map save failed'
        self._save_automatic = False

    def _publish_status(self) -> None:
        msg = String()
        goal = self.policy.active_goal
        readiness = self._readiness()
        missing_items = readiness.missing()
        if self.policy.state in (
                ExplorerState.SELECT, ExplorerState.VALIDATE_PATH,
                ExplorerState.NAVIGATE):
            missing_items = tuple(
                item for item in missing_items if item != 'stationary')
        missing = ','.join(missing_items) or 'none'
        voltage = (f'{self._battery_voltage:.2f}'
                   if math.isfinite(self._battery_voltage) else 'unknown')
        frontiers = (
            str(self._frontier_count)
            if self._frontier_count is not None else 'unknown')
        motion_transition = (
            'pending' if self._navigation_transition_pending() else 'clear')
        collision_error = (
            getattr(self, '_collision_query_error', '') or 'none')
        msg.data = (
            f'state={self.policy.state.value};fault={self.policy.fault};'
            f'goal={"none" if goal is None else f"{goal.x:.2f},{goal.y:.2f}"};'
            f'battery_voltage={voltage};battery_ok={readiness.battery_ok};'
            f'stationary={readiness.stationary};'
            f'readiness_missing={missing};save={self._last_save_status};'
            f'motion_transition={motion_transition};frontiers={frontiers};'
            f'map_sequence={self._map_sequence};'
            f'collision_lifecycle_error={collision_error}')
        self._status_pub.publish(msg)

    def _publish_markers(
            self, candidates: Sequence[FrontierCandidate]) -> None:
        array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontiers'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = 0.08
        marker.color.g = marker.color.a = 1.0
        marker.points = [Point(x=item.x, y=item.y, z=0.05)
                         for item in candidates]
        array.markers.append(marker)
        if self.policy.active_goal is not None:
            goal = Marker()
            goal.header = marker.header
            goal.ns = 'goal'
            goal.id = 0
            goal.type = Marker.SPHERE
            goal.action = Marker.ADD
            goal.pose.position.x = self.policy.active_goal.x
            goal.pose.position.y = self.policy.active_goal.y
            goal.pose.orientation.w = 1.0
            goal.scale.x = goal.scale.y = goal.scale.z = 0.18
            goal.color.r = goal.color.a = 1.0
            array.markers.append(goal)
        self._markers_pub.publish(array)

    def destroy_node(self):
        """Join the extraction worker before destroying ROS entities."""
        if self._path_goal_handle is not None:
            try:
                self._path_goal_handle.cancel_goal_async()
            except Exception:
                pass
        if (self._nav_goal_handle is not None and
                self._nav_cancel_handle is None):
            try:
                self._nav_goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._frontier_worker.shutdown()
        return super().destroy_node()


def _cleanup_after_spin(node) -> None:
    """Finish best-effort ROS cleanup without leaking a shutdown SIGINT."""
    try:
        node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def main(args=None) -> None:
    """Run the frontier explorer node; it always starts in IDLE."""
    if not ROS_AVAILABLE:
        raise RuntimeError(
            'ROS 2 Python modules are unavailable') from ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        _cleanup_after_spin(node)


if __name__ == '__main__':
    main()
