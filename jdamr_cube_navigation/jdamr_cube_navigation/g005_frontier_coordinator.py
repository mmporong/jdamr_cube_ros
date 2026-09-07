#!/usr/bin/env python3
"""Coordinate one actual Nav2 run for the G005 frontier evaluation."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import resource
import tempfile
import time

from action_msgs.msg import GoalStatus

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
from geometry_msgs.msg import PoseStamped, Twist
from jdamr_cube_navigation.frontier_core import FrontierCore, GridMap
from jdamr_cube_navigation.g005_ground_truth_localization import (
    quaternion_yaw,
    stamp_ns,
)
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


SIMULATION_HORIZON_S = 900.0
PLANNER_TIMEOUT_S = 2.0
FINAL_ZERO_HOLD_S = 1.0
READY_TIMEOUT_S = 45.0
TIMER_PERIOD_S = 0.05
RESOURCE_SAMPLE_PERIOD_S = 0.5
MAP_STABILITY_S = 0.5
GT_SAMPLE_PERIOD_S = 0.5
ACTION_CANCELLATION_TIMEOUT_S = 2.0
LIFECYCLE_SERVICE_MISS_LIMIT_COUNT = 3
LIFECYCLE_QUERY_TIMEOUT_S = 2.0
AUTHORIZED_TF_DIGEST_HISTORY_COUNT = 200
FOOTPRINT_POLYGON_M = (
    (0.23, 0.20),
    (0.23, -0.20),
    (-0.23, -0.20),
    (-0.23, 0.20),
)
EXPECTED_TF_AUTHORITY_NODE = '/g005_frontier_coordinator'
EXPECTED_TF_PUBLISHER_NODES = {
    '/g005_frontier_coordinator',
    '/g005_runtime_bridge',
    '/robot_state_publisher',
}
EXPECTED_TF_SOURCE_NODE = '/ground_truth_localization'
NAV2_LIFECYCLE_NODES = (
    'behavior_server',
    'bt_navigator',
    'collision_monitor',
    'controller_server',
    'planner_server',
    'velocity_smoother',
)
MEASUREMENT_FIELDS = (
    'observer_health',
    'decisions',
    'coverage_samples',
    'gt_pose_samples',
    'planner_batch_timeline',
    'contact_count',
    'command_authority',
    'tf_authority',
    'lifecycle',
    'navigation_outcomes',
    'resources',
    'runner_cancellation_count',
    'production_inputs_before',
    'production_inputs_after',
)


def _strict_json_load(path: Path):
    """Read strict JSON and reject duplicate keys and non-finite values."""
    def pairs(items):
        value = {}
        for key, item in items:
            if key in value:
                raise ValueError(f'duplicate JSON key: {key}')
            value[key] = item
        return value

    def reject(value):
        raise ValueError(f'non-finite JSON number: {value}')

    return json.loads(
        path.read_text(encoding='utf-8'), object_pairs_hook=pairs,
        parse_constant=reject)


def _canonical_json_bytes(value) -> bytes:
    return (json.dumps(
        value, sort_keys=True, separators=(',', ':'), ensure_ascii=False,
        allow_nan=False) + '\n').encode()


def _sha256_json(value) -> str:
    return hashlib.sha256(_canonical_json_bytes(value)).hexdigest()


def _atomic_json_write(path: Path, value) -> None:
    """Commit one canonical JSON document without exposing a partial file."""
    if not path.is_absolute() or path.exists() or path.is_symlink():
        raise ValueError('measurement_path must be an absent absolute path')
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
            prefix=f'.{path.name}.', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(_canonical_json_bytes(value))
        stream.flush()
        os.fsync(stream.fileno())
    try:
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def _evaluation_directory() -> Path:
    source = Path(__file__).resolve().parents[1] / 'evaluation'
    try:
        installed = (Path(get_package_share_directory(
            'jdamr_cube_navigation')) / 'evaluation')
        if (installed / 'frontier_policy_contract.py').is_file():
            return installed
    except PackageNotFoundError:
        pass
    if (source / 'frontier_policy_contract.py').is_file():
        return source
    raise ValueError('G005 evaluation contract is unavailable')


def load_policy_contract():
    """Load the installed pure policy contract without ambient path drift."""
    directory = _evaluation_directory()
    path = (directory / 'frontier_policy_contract.py').resolve(strict=True)
    spec = importlib.util.spec_from_file_location(
        '_g005_coordinator_frontier_policy_contract', path)
    if spec is None or spec.loader is None:
        raise ValueError('G005 policy contract loader drift')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if Path(module.__file__).resolve(strict=True) != path:
        raise ValueError('G005 policy contract import identity drift')
    return module


def validate_request(request: object, request_path: Path,
                     measurement_path: Path, contract) -> dict:
    """Validate the run inputs consumed by this process."""
    required = {
        'schema_version', 'run_id', 'policy', 'layout_seed', 'ros_domain_id',
        'simulation_horizon_s', 'asset_root', 'asset_identity',
        'asset_manifest_identity', 'production_inputs_sha256',
        'runtime_components', 'upstream_versions_required',
        'shared_initial_sha256', 'shared_reveal_sha256',
        'shared_runtime_sha256', 'required_runtime_claim', 'request_sha256',
    }
    if (not request_path.is_absolute() or not request_path.is_file()
            or request_path.is_symlink() or not measurement_path.is_absolute()
            or measurement_path.exists() or measurement_path.is_symlink()
            or type(request) is not dict or set(request) != required):
        raise ValueError('G005 coordinator request envelope drift')
    expected_hash = _sha256_json({
        key: value for key, value in request.items()
        if key != 'request_sha256'})
    if (request['schema_version'] != 1
            or request['request_sha256'] != expected_hash
            or request['policy'] not in contract.POLICIES
            or request['layout_seed'] not in contract.LAYOUT_SEEDS
            or request['run_id'] != (
                f"{request['policy']}__seed_{request['layout_seed']}")
            or request['simulation_horizon_s'] != SIMULATION_HORIZON_S
            or not Path(request['asset_root']).is_absolute()):
        raise ValueError('G005 coordinator request contract drift')
    return dict(request)


def occupancy_payload(message: OccupancyGrid) -> dict:
    """Return the immutable fields that define a planner map snapshot."""
    orientation = message.info.origin.orientation
    yaw = quaternion_yaw(
        orientation.x, orientation.y, orientation.z, orientation.w)
    payload = {
        'frame_id': message.header.frame_id,
        'height': int(message.info.height),
        'origin_xy_yaw': [
            float(message.info.origin.position.x),
            float(message.info.origin.position.y),
            yaw,
        ],
        'resolution_m_per_cell': float(message.info.resolution),
        'width': int(message.info.width),
        'data': [int(value) for value in message.data],
    }
    GridMap(
        width=payload['width'], height=payload['height'],
        resolution=payload['resolution_m_per_cell'],
        origin_x=payload['origin_xy_yaw'][0],
        origin_y=payload['origin_xy_yaw'][1],
        origin_yaw=payload['origin_xy_yaw'][2], data=payload['data'])
    if payload['frame_id'] != 'map':
        raise ValueError('G005 map frame drift')
    return payload


def grid_from_payload(payload: dict) -> GridMap:
    """Convert a validated snapshot payload to the shared frontier grid."""
    return GridMap(
        width=payload['width'], height=payload['height'],
        resolution=payload['resolution_m_per_cell'],
        origin_x=payload['origin_xy_yaw'][0],
        origin_y=payload['origin_xy_yaw'][1],
        origin_yaw=payload['origin_xy_yaw'][2], data=payload['data'])


def extract_candidate_batch(contract, layout_seed: int, map_payload: dict,
                            start_xy_yaw: list[float],
                            blacklist_ids: set[int]) -> tuple[dict, list[dict]]:
    """Extract all frontiers, hard-exclude blacklist IDs, then sample."""
    grid = grid_from_payload(map_payload)
    candidates = FrontierCore().extract(
        grid, start_xy_yaw[0], start_xy_yaw[1], start_xy_yaw[2])
    by_id = {grid.index(item.cell): item for item in candidates}
    raw_ids = sorted(by_id)
    excluded = sorted(set(raw_ids) & blacklist_ids)
    allowed = sorted(set(raw_ids) - set(excluded))
    sampled = contract.neutral_sample(layout_seed, allowed)
    sampling = {
        'raw_candidate_ids': raw_ids,
        'excluded_candidate_ids': excluded,
        'selected_candidate_ids': sampled['selected_candidate_ids'],
        'omitted_candidate_ids': sampled['omitted_candidate_ids'],
    }
    records = []
    for cell_index in sampling['selected_candidate_ids']:
        item = by_id[cell_index]
        records.append({
            'cell_index': cell_index,
            'gain_cells': float(item.information_gain),
            'bfs_distance_m': float(item.path_distance_m),
            'heading_rad': float(item.heading_change),
            'nav_length_m': 0.0,
            'goal_xy_yaw': [float(item.x), float(item.y), float(item.yaw)],
        })
    return sampling, records


def _pose(x: float, y: float, yaw: float, stamp) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.header.stamp = stamp
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def _stamp_from_ns(value: int):
    """Construct the exact ROS stamp sealed in a decision start record."""
    if type(value) is not int or value <= 0:
        raise ValueError('G005 start stamp must be a positive integer')
    message = PoseStamped().header.stamp
    message.sec = value // 1_000_000_000
    message.nanosec = value % 1_000_000_000
    return message


def build_planner_goal(batch: dict, candidate: dict, goal_stamp):
    """Build the actual planner request from the sealed batch record."""
    goal = ComputePathToPose.Goal()
    start_stamp = _stamp_from_ns(batch['start_record']['stamp_ns'])
    goal.start = _pose(*batch['start_record']['pose_xy_yaw'], start_stamp)
    goal.goal = _pose(*candidate['goal_xy_yaw'], goal_stamp)
    goal.planner_id = 'GridBased'
    goal.use_start = True
    return goal


def map_status_matches(map_payload_sha256: str | None,
                       observer_status: dict | None) -> bool:
    """Require the independently delivered map and status identities to bind."""
    return (
        type(map_payload_sha256) is str
        and type(observer_status) is dict
        and type(observer_status.get('map_sequence')) is int
        and observer_status['map_sequence'] > 0
        and observer_status.get('map_payload_sha256') == map_payload_sha256
    )


def _point_segment_distance(point, start, end) -> float:
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    squared = delta_x * delta_x + delta_y * delta_y
    if squared == 0.0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = max(0.0, min(1.0, (
        (point[0] - start[0]) * delta_x
        + (point[1] - start[1]) * delta_y) / squared))
    nearest = (
        start[0] + fraction * delta_x,
        start[1] + fraction * delta_y,
    )
    return math.hypot(point[0] - nearest[0], point[1] - nearest[1])


def _cross(start, end, point) -> float:
    return ((end[0] - start[0]) * (point[1] - start[1])
            - (end[1] - start[1]) * (point[0] - start[0]))


def _segments_intersect(first_start, first_end,
                        second_start, second_end) -> bool:
    first_a = _cross(first_start, first_end, second_start)
    first_b = _cross(first_start, first_end, second_end)
    second_a = _cross(second_start, second_end, first_start)
    second_b = _cross(second_start, second_end, first_end)
    epsilon = 1e-12

    def on_segment(start, end, point):
        return (min(start[0], end[0]) - epsilon <= point[0] <=
                max(start[0], end[0]) + epsilon
                and min(start[1], end[1]) - epsilon <= point[1] <=
                max(start[1], end[1]) + epsilon)

    strictly_crosses = (
        ((first_a > epsilon and first_b < -epsilon)
         or (first_a < -epsilon and first_b > epsilon))
        and ((second_a > epsilon and second_b < -epsilon)
             or (second_a < -epsilon and second_b > epsilon)))
    return strictly_crosses or (
        abs(first_a) <= epsilon
        and on_segment(first_start, first_end, second_start)
        or abs(first_b) <= epsilon
        and on_segment(first_start, first_end, second_end)
        or abs(second_a) <= epsilon
        and on_segment(second_start, second_end, first_start)
        or abs(second_b) <= epsilon
        and on_segment(second_start, second_end, first_end)
    )


def _polygon_distance(first, second) -> float:
    first_edges = list(zip(first, first[1:] + first[:1]))
    second_edges = list(zip(second, second[1:] + second[:1]))

    def inside(point, edges):
        crosses = [_cross(start, end, point) for start, end in edges]
        return (all(value >= -1e-12 for value in crosses)
                or all(value <= 1e-12 for value in crosses))

    if (any(_segments_intersect(*left, *right)
            for left in first_edges for right in second_edges)
            or inside(first[0], second_edges)
            or inside(second[0], first_edges)):
        return 0.0
    return min(
        *(_point_segment_distance(point, *edge)
          for point in first for edge in second_edges),
        *(_point_segment_distance(point, *edge)
          for point in second for edge in first_edges),
    )


def footprint_clearance_m(layout: dict, x_m: float, y_m: float,
                          yaw_rad: float) -> float:
    """Measure body-polygon clearance to occupied grid-cell squares."""
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    footprint = [
        (x_m + cosine * x - sine * y,
         y_m + sine * x + cosine * y)
        for x, y in FOOTPRINT_POLYGON_M
    ]
    resolution = float(layout['resolution_m_per_cell'])
    origin_x, origin_y, origin_yaw = [
        float(value) for value in layout['origin_m_rad']]
    origin_cosine = math.cos(origin_yaw)
    origin_sine = math.sin(origin_yaw)

    def world(local_x, local_y):
        return (
            origin_x + origin_cosine * local_x - origin_sine * local_y,
            origin_y + origin_sine * local_x + origin_cosine * local_y,
        )

    best = math.inf
    for index, value in enumerate(layout['data']):
        if value < 65:
            continue
        cell_x = index % layout['width'] * resolution
        cell_y = index // layout['width'] * resolution
        occupied = [
            world(cell_x, cell_y),
            world(cell_x + resolution, cell_y),
            world(cell_x + resolution, cell_y + resolution),
            world(cell_x, cell_y + resolution),
        ]
        best = min(best, _polygon_distance(footprint, occupied))
    if not math.isfinite(best):
        raise ValueError('G005 layout has no occupied cell')
    return best


def _endpoint_identity(info) -> tuple[str, str]:
    gid = bytes(info.endpoint_gid).hex()
    namespace = info.node_namespace.rstrip('/')
    node = f'{namespace}/{info.node_name}' if namespace else f'/{info.node_name}'
    return gid, node


def _map_to_odom_payload(message: TFMessage) -> dict:
    matches = [item for item in message.transforms
               if item.header.frame_id == 'map'
               and item.child_frame_id == 'odom']
    if len(matches) != 1 or len(message.transforms) != 1:
        raise ValueError('expected one map to odom transform')
    item = matches[0]
    return {
        'parent_frame': item.header.frame_id,
        'child_frame': item.child_frame_id,
        'stamp_ns': (int(item.header.stamp.sec) * 1_000_000_000 +
                     int(item.header.stamp.nanosec)),
        'translation_xyz': [
            float(item.transform.translation.x),
            float(item.transform.translation.y),
            float(item.transform.translation.z),
        ],
        'rotation_xyzw': [
            float(item.transform.rotation.x),
            float(item.transform.rotation.y),
            float(item.transform.rotation.z),
            float(item.transform.rotation.w),
        ],
    }


class MeasurementState:
    """Accumulate sticky validity and raw evidence for exactly one run."""

    def __init__(self, request: dict, reachable_denominator: int,
                 production_inputs: dict) -> None:
        self.request = request
        self.validity = 'VALID'
        self.outcome = 'FAIL'
        self.invalid_reasons: list[str] = []
        self.observer_health = {}
        self.decisions: list[dict] = []
        self.coverage_samples = {
            'reachable_denominator_cells': reachable_denominator,
            'samples': [],
        }
        self.gt_pose_samples: list[dict] = []
        self.planner_batch_timeline: list[dict] = []
        self.contact_count = 0
        self.command_authority = {
            'topic': '/cmd_vel', 'publisher_gids': [],
            'publisher_nodes': [], 'final_zero_hold_s': 0.0,
            'final_linear_x': 0.0, 'final_angular_z': 0.0,
        }
        self.tf_authority = {
            'map_to_odom_publisher_gids': [],
            'map_to_odom_publisher_nodes': [],
        }
        self.lifecycle = {name: 'unknown' for name in NAV2_LIFECYCLE_NODES}
        self.navigation_outcomes = {'recovery_count': 0, 'failure_count': 0}
        self.resources = {'cpu_seconds': 0.0, 'peak_rss_bytes': 0,
                          'samples': 0}
        self.runner_cancellation_count = 0
        self.production_inputs_before = deepcopy(production_inputs)
        self.production_inputs_after = deepcopy(production_inputs)

    def invalidate(self, reason: str) -> None:
        self.validity = 'INVALID'
        if reason not in self.invalid_reasons:
            self.invalid_reasons.append(reason)

    def payload(self) -> dict:
        value = {
            'validity': self.validity,
            'outcome': self.outcome,
            'invalid_reasons': sorted(self.invalid_reasons),
        }
        for field in MEASUREMENT_FIELDS:
            value[field] = deepcopy(getattr(self, field))
        return value


class FrontierCoordinator(Node):
    """Bind frontier decisions to actual map, pose, planner, and Nav2 data."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            'g005_frontier_coordinator',
            parameter_overrides=parameter_overrides)
        self.declare_parameter('request_path', '')
        self.declare_parameter('measurement_path', '')
        self._request_path = Path(str(
            self.get_parameter('request_path').value))
        self._measurement_path = Path(str(
            self.get_parameter('measurement_path').value))
        self._contract = load_policy_contract()
        raw_request = _strict_json_load(self._request_path)
        self._request = validate_request(
            raw_request, self._request_path, self._measurement_path,
            self._contract)
        asset_root = Path(self._request['asset_root'])
        layout_path = asset_root / (
            f"layout_{self._request['layout_seed']}_gt.json")
        manifest_path = asset_root / 'asset_manifest.json'
        layout = _strict_json_load(layout_path)
        manifest = _strict_json_load(manifest_path)
        if layout != self._contract.build_layout(self._request['layout_seed']):
            raise ValueError('G005 coordinator layout drift')
        production = manifest.get('production_inputs')
        if (type(production) is not dict or not production
                or _sha256_json(production) !=
                self._request['production_inputs_sha256']):
            raise ValueError('G005 coordinator production input drift')
        reachable = set(self._contract.connected_reachable_cells(layout))
        self._reachable_cells = reachable
        self._state = MeasurementState(
            self._request, len(reachable), production)
        self._map_payload = None
        self._map_payload_sha256 = None
        self._map_status_bound = False
        self._last_map_change = None
        self._map_sequence = 0
        self._observer_status = None
        self._gt_pose = None
        self._last_gt_sample_sim_s = None
        self._first_gt_steady = None
        self._first_sim_s = None
        self._latest_sim_s = None
        self._steady_start = time.monotonic()
        self._last_resource_sample = 0.0
        self._last_cmd = (0.0, 0.0)
        self._zero_since = None
        self._terminal_reason = None
        self._terminal_since = None
        self._written = False
        self._busy = False
        self._planner_index = 0
        self._planner_records = []
        self._candidate_records = []
        self._batch = None
        self._blacklist_ids: set[int] = set()
        self._unreachable_batches: list[dict] = []
        self._retry_snapshot = None
        self._retry_after_observer_stamp_ns = None
        self._planner = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose')
        self._navigator = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self._lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in NAV2_LIFECYCLE_NODES
        }
        self._lifecycle_pending: dict[str, float] = {}
        self._lifecycle_service_misses = {
            name: 0 for name in NAV2_LIFECYCLE_NODES}
        self._last_lifecycle_query = 0.0
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self._on_costmap, latched)
        self.create_subscription(
            PoseStamped, '/ground_truth_pose', self._on_ground_truth,
            qos_profile_sensor_data)
        self.create_subscription(
            String, '/g005/observer_status', self._on_observer, latched)
        self.create_subscription(
            Contacts, '/g005_contacts', self._on_contacts,
            qos_profile_sensor_data)
        self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, qos_profile_sensor_data)
        self._costmap_payload = None
        self._cmd_received = False
        self._planner_timeout_timer = None
        self._planner_goal_handle = None
        self._planner_serial = 0
        self._navigation_goal_handle = None
        self._planner_send_pending = False
        self._navigation_send_pending = False
        self._cancel_futures_pending = 0
        self._current_goal_recoveries = 0
        self._lifecycle_seen_active: set[str] = set()
        self._cmd_authorities: dict[str, str] = {}
        self._tf_authorities: dict[str, str] = {}
        self._authorized_tf_payload_sha256 = deque(
            maxlen=AUTHORIZED_TF_DIGEST_HISTORY_COUNT)
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.RELIABLE)
        self._tf_publisher = self.create_publisher(
            TFMessage, '/tf', tf_qos)
        self.create_subscription(
            TFMessage, '/g005/map_to_odom_tf',
            self._on_authoritative_tf, tf_qos)
        self.create_subscription(
            TFMessage, '/tf', self._on_tf, qos_profile_sensor_data)
        self.create_timer(TIMER_PERIOD_S, self._tick)

    def _sim_s(self, message) -> float:
        stamp = stamp_ns(message)
        value = stamp / 1_000_000_000
        self._latest_sim_s = value
        if self._first_sim_s is None:
            self._first_sim_s = value
        return value - self._first_sim_s

    def _on_map(self, message: OccupancyGrid) -> None:
        try:
            payload = occupancy_payload(message)
            if payload != self._map_payload:
                self._last_map_change = time.monotonic()
            self._map_payload = payload
            self._map_payload_sha256 = _sha256_json(payload)
            self._bind_map_status()
            elapsed = self._sim_s(message)
            revealed = sum(
                1 for index in self._reachable_cells
                if self._map_payload['data'][index] >= 0)
            samples = self._state.coverage_samples['samples']
            if not samples:
                samples.append({
                    'elapsed_s': 0.0,
                    'revealed_reachable_cells': revealed,
                })
            elif revealed != samples[-1]['revealed_reachable_cells']:
                samples.append({
                    'elapsed_s': min(elapsed, SIMULATION_HORIZON_S),
                    'revealed_reachable_cells': revealed,
                })
        except (TypeError, ValueError) as error:
            self._invalidate(f'map_contract:{error}')

    def _on_costmap(self, message: OccupancyGrid) -> None:
        try:
            self._costmap_payload = occupancy_payload(message)
        except ValueError as error:
            self._invalidate(f'costmap_contract:{error}')

    def _on_observer(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if type(value) is not dict or value.get('health') not in {
                    'VALID', 'INVALID'}:
                raise ValueError('status schema drift')
            sequence = value.get('map_sequence')
            payload_sha256 = value.get('map_payload_sha256')
            if (type(sequence) is not int or sequence < 0
                    or (sequence == 0 and payload_sha256 is not None)
                    or (sequence > 0 and (
                        type(payload_sha256) is not str
                        or len(payload_sha256) != 64
                        or any(character not in '0123456789abcdef'
                               for character in payload_sha256)))):
                raise ValueError('status map identity drift')
            self._observer_status = value
            self._state.observer_health = deepcopy(value)
            self._bind_map_status()
            if value['health'] == 'INVALID':
                self._invalidate('observer_status_invalid')
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self._invalidate(f'observer_status_contract:{error}')

    def _bind_map_status(self) -> None:
        self._map_status_bound = False
        if map_status_matches(
                self._map_payload_sha256, self._observer_status):
            self._map_sequence = self._observer_status['map_sequence']
            self._map_status_bound = True

    def _on_ground_truth(self, message: PoseStamped) -> None:
        try:
            if message.header.frame_id != 'g005_frontier':
                raise ValueError('ground-truth frame drift')
            pose = message.pose
            yaw = quaternion_yaw(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
            elapsed = self._sim_s(message)
            steady_now = time.monotonic()
            if self._first_gt_steady is None:
                self._first_gt_steady = steady_now
            sample = {
                'steady_elapsed_s': steady_now - self._first_gt_steady,
                'sim_elapsed_s': min(elapsed, SIMULATION_HORIZON_S),
                'x_m': float(pose.position.x),
                'y_m': float(pose.position.y),
                'clearance_m': footprint_clearance_m(
                    self._contract.build_layout(
                        self._request['layout_seed']),
                    float(pose.position.x), float(pose.position.y), yaw),
            }
            horizon_sample_due = (
                sample['sim_elapsed_s'] == SIMULATION_HORIZON_S and
                self._last_gt_sample_sim_s != SIMULATION_HORIZON_S)
            if (self._last_gt_sample_sim_s is None or horizon_sample_due
                    or sample['sim_elapsed_s'] - self._last_gt_sample_sim_s >=
                    GT_SAMPLE_PERIOD_S):
                self._state.gt_pose_samples.append(sample)
                self._last_gt_sample_sim_s = sample['sim_elapsed_s']
            self._gt_pose = [sample['x_m'], sample['y_m'], yaw]
        except ValueError as error:
            self._invalidate(f'ground_truth_contract:{error}')

    def _on_contacts(self, message: Contacts) -> None:
        if message.contacts:
            self._state.contact_count += len(message.contacts)
            self._invalidate('nonempty_contact')

    def _on_cmd_vel(self, message: Twist) -> None:
        publishers = sorted({_endpoint_identity(item) for item in
                             self.get_publishers_info_by_topic('/cmd_vel')})
        nodes = {node for _, node in publishers}
        unexpected = {node for node in nodes
                      if '_UNKNOWN_' not in node and
                      node != '/collision_monitor'}
        if unexpected:
            self._invalidate(
                f'cmd_vel_authority_drift:{sorted(unexpected)[0]}')
        expected = [item for item in publishers
                    if item[1] == '/collision_monitor']
        if len(expected) > 1:
            self._invalidate(
                f'cmd_vel_authority_count:{len(expected)}')
        elif expected:
            gid, node = expected[0]
            self._cmd_authorities[gid] = node
        self._cmd_received = True
        self._last_cmd = (float(message.linear.x), float(message.angular.z))
        if self._last_cmd == (0.0, 0.0):
            if self._zero_since is None:
                self._zero_since = time.monotonic()
        else:
            self._zero_since = None

    def _on_authoritative_tf(self, message: TFMessage) -> None:
        publishers = sorted({_endpoint_identity(item) for item in
                             self.get_publishers_info_by_topic(
                                 '/g005/map_to_odom_tf')})
        if not publishers or any(
                '_UNKNOWN_' in node for _, node in publishers):
            return
        sources = [item for item in publishers
                   if item[1] == EXPECTED_TF_SOURCE_NODE]
        if len(publishers) != 1 or len(sources) != 1:
            self._invalidate('map_to_odom_source_authority_drift')
            return
        try:
            payload = _map_to_odom_payload(message)
        except ValueError as error:
            self._invalidate(f'map_to_odom_source_contract:{error}')
            return
        self._authorized_tf_payload_sha256.append(_sha256_json(payload))
        self._tf_publisher.publish(message)

    def _on_tf(self, message: TFMessage, message_info) -> None:
        del message_info
        contains_map_to_odom = any(
            item.header.frame_id == 'map' and item.child_frame_id == 'odom'
            for item in message.transforms)
        if not contains_map_to_odom:
            return
        try:
            payload = _map_to_odom_payload(message)
        except ValueError as error:
            self._invalidate(f'map_to_odom_message_contract:{error}')
            return
        digest = _sha256_json(payload)
        if digest not in self._authorized_tf_payload_sha256:
            self._invalidate('map_to_odom_payload_drift')
            return
        self._authorized_tf_payload_sha256.remove(digest)
        publishers = sorted({_endpoint_identity(item) for item in
                             self.get_publishers_info_by_topic('/tf')})
        if not publishers:
            return
        nodes = {node for _, node in publishers}
        if any('_UNKNOWN_' in node for node in nodes):
            return
        unexpected = nodes - EXPECTED_TF_PUBLISHER_NODES
        if unexpected:
            self._invalidate(
                f'tf_publisher_set_drift:{sorted(unexpected)[0]}')
            return
        if nodes != EXPECTED_TF_PUBLISHER_NODES:
            return
        authorities = [item for item in publishers
                       if item[1] == EXPECTED_TF_AUTHORITY_NODE]
        if len(authorities) != 1:
            self._invalidate(
                f'map_to_odom_authority_count:{len(authorities)}')
            return
        gid, node = authorities[0]
        self._tf_authorities[gid] = node

    def _invalidate(self, reason: str) -> None:
        if reason not in self._state.invalid_reasons:
            self.get_logger().error(f'G005 runtime invalid: {reason}')
        self._state.invalidate(reason)
        self._enter_terminal('INVALID')

    def _enter_terminal(self, reason: str) -> None:
        if self._terminal_reason is None:
            self._terminal_reason = reason
            self._terminal_since = time.monotonic()
            self._cancel_active_actions()

    def _sample_resources(self, now: float) -> None:
        if now - self._last_resource_sample < RESOURCE_SAMPLE_PERIOD_S:
            return
        usage = resource.getrusage(resource.RUSAGE_SELF)
        self._state.resources = {
            'cpu_seconds': float(usage.ru_utime + usage.ru_stime),
            'peak_rss_bytes': int(usage.ru_maxrss * 1024),
            'samples': self._state.resources['samples'] + 1,
        }
        self._last_resource_sample = now

    def _ready(self) -> bool:
        return (self._map_payload is not None
                and self._costmap_payload is not None
                and self._gt_pose is not None
                and self._observer_status is not None
                and self._observer_status['health'] == 'VALID'
                and self._map_status_bound
                and (self._retry_after_observer_stamp_ns is None or
                     self._observer_status['last_accepted_stamp_ns'] >
                     self._retry_after_observer_stamp_ns)
                and self._map_sequence > 0
                and self._last_map_change is not None
                and time.monotonic() - self._last_map_change >=
                MAP_STABILITY_S
                and self._planner.server_is_ready()
                and self._navigator.server_is_ready()
                and all(state == 'active'
                        for state in self._state.lifecycle.values()))

    def _tick(self) -> None:
        now = time.monotonic()
        self._sample_resources(now)
        self._query_lifecycle(now)
        if self._written:
            return
        horizon_reached = (
            self._latest_sim_s is not None and self._first_sim_s is not None
            and self._latest_sim_s - self._first_sim_s >=
            SIMULATION_HORIZON_S)
        if horizon_reached:
            self._enter_terminal('HORIZON')
        if (not self._ready() and self._terminal_reason is None
                and now - self._steady_start >= READY_TIMEOUT_S):
            self._invalidate('runtime_readiness_timeout')
        if self._terminal_reason is not None:
            if horizon_reached:
                self._finish_when_stopped(now)
            return
        if not self._busy and self._ready():
            self._begin_decision()

    def _begin_decision(self) -> None:
        try:
            map_payload = deepcopy(self._map_payload)
            costmap_sha = _sha256_json(self._costmap_payload)
            map_sha = _sha256_json(map_payload)
            if (self._retry_snapshot is not None and
                    self._retry_snapshot['map_payload_sha256'] == map_sha):
                start = deepcopy(self._retry_snapshot['start_record'])
                sampling = deepcopy(self._retry_snapshot['sampling'])
                candidates = deepcopy(self._retry_snapshot['candidates'])
            else:
                self._retry_snapshot = None
                start = {
                    'frame_id': 'map',
                    'stamp_ns': int(self.get_clock().now().nanoseconds),
                    'pose_xy_yaw': list(self._gt_pose),
                }
                sampling, candidates = extract_candidate_batch(
                    self._contract, self._request['layout_seed'], map_payload,
                    start['pose_xy_yaw'], self._blacklist_ids)
            if not sampling['selected_candidate_ids']:
                self._enter_terminal('NO_FRONTIERS')
                return
            token = self._contract.decision_token(
                self._request['run_id'], self._request['policy'],
                self._map_sequence, map_sha, start,
                sampling['excluded_candidate_ids'],
                sampling['selected_candidate_ids'])
            self._batch = {
                'planner_batch_attempt_index': (
                    len(self._state.planner_batch_timeline) + 1),
                'map_sequence': self._map_sequence,
                'map_payload_sha256': map_sha,
                'start_record': start,
                'blacklist_ids': sampling['excluded_candidate_ids'],
                'candidate_sampling': sampling,
                'decision_token': token,
                'costmap_sha256_before': costmap_sha,
                'planner_action': {
                    'type': 'nav2_msgs/action/ComputePathToPose',
                    'planner_id': 'GridBased', 'use_start': True,
                    'timeout_s': PLANNER_TIMEOUT_S,
                    'execution': 'ACTUAL_NAV2_ACTION',
                },
            }
            self._candidate_records = candidates
            self._planner_records = []
            self._planner_index = 0
            self._busy = True
            self._plan_next()
        except (TypeError, ValueError) as error:
            self._invalidate(f'decision_contract:{error}')

    def _plan_next(self) -> None:
        if self._terminal_reason is not None:
            return
        if self._planner_index >= len(self._candidate_records):
            self._complete_planner_batch()
            return
        candidate = self._candidate_records[self._planner_index]
        goal = build_planner_goal(
            self._batch, candidate, self.get_clock().now().to_msg())
        self._planner_serial += 1
        serial = self._planner_serial
        self._planner_send_pending = True
        future = self._planner.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self._planner_goal_response(result, serial))
        self._planner_timeout_timer = self.create_timer(
            PLANNER_TIMEOUT_S,
            lambda: self._planner_timeout(serial))

    def _planner_goal_response(self, future, serial: int) -> None:
        self._planner_send_pending = False
        try:
            handle = future.result()
            if serial != self._planner_serial:
                if handle is not None and handle.accepted:
                    self._cancel_goal_handle(handle)
                return
            if self._terminal_reason is not None:
                if handle is not None and handle.accepted:
                    self._cancel_goal_handle(handle)
                return
            if handle is None or not handle.accepted:
                self._record_planner_result(None, 1)
                return
            self._planner_goal_handle = handle
            result = handle.get_result_async()
            result.add_done_callback(
                lambda completed: self._planner_result(completed, serial))
        except Exception as error:
            self._invalidate(f'planner_action:{type(error).__name__}')

    def _planner_result(self, future, serial: int) -> None:
        if serial != self._planner_serial or self._terminal_reason is not None:
            return
        try:
            wrapped = future.result()
            path = wrapped.result.path
            poses = [[float(item.pose.position.x),
                      float(item.pose.position.y)] for item in path.poses]
            frame_id = str(path.header.frame_id)
            error_code = int(getattr(wrapped.result, 'error_code', 0))
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                error_code = error_code or int(wrapped.status)
            self._record_planner_result(poses, error_code, frame_id)
        except Exception as error:
            self._invalidate(f'planner_result:{type(error).__name__}')

    def _record_planner_result(self, poses, error_code: int,
                               frame_id: str = '') -> None:
        self._planner_serial += 1
        self._planner_goal_handle = None
        if self._planner_timeout_timer is not None:
            self._planner_timeout_timer.cancel()
            self.destroy_timer(self._planner_timeout_timer)
            self._planner_timeout_timer = None
        if self._terminal_reason is not None:
            return
        candidate = dict(self._candidate_records[self._planner_index])
        candidate.pop('goal_xy_yaw')
        self._planner_records.append({
            'candidate': candidate,
            'token': self._batch['decision_token'],
            'use_start': True, 'planner_id': 'GridBased',
            'timeout_s': PLANNER_TIMEOUT_S, 'error_code': error_code,
            'frame_id': frame_id, 'poses': poses or [],
        })
        self._planner_index += 1
        self._plan_next()

    def _planner_timeout(self, serial: int) -> None:
        if serial != self._planner_serial:
            return
        if self._planner_goal_handle is not None:
            self._cancel_goal_handle(self._planner_goal_handle)
        self._record_planner_result(None, 255)

    def _complete_planner_batch(self) -> None:
        after = _sha256_json(self._costmap_payload)
        self._batch['costmap_sha256_after'] = after
        if _sha256_json(self._map_payload) != self._batch[
                'map_payload_sha256']:
            self._retry_snapshot = None
            self._busy = False
            return
        if after != self._batch['costmap_sha256_before']:
            self._busy = False
            return
        try:
            eligible = self._contract.validate_planner_batch(
                self._planner_records, self._batch['decision_token'],
                self._batch['costmap_sha256_before'], after)
        except ValueError as error:
            self._invalidate(f'frozen_costmap:{error}')
            return
        timeline = {
            'attempt_index': self._batch['planner_batch_attempt_index'],
            'map_sequence': self._batch['map_sequence'],
            'map_payload_sha256': self._batch['map_payload_sha256'],
            'start_record_sha256': _sha256_json(
                self._batch['start_record']),
            'candidate_sha256': _sha256_json(
                self._batch['candidate_sampling']['selected_candidate_ids']),
            'fresh': True, 'reachable_count': len(eligible),
        }
        self._state.planner_batch_timeline.append(timeline)
        if not eligible:
            if self._retry_snapshot is None:
                self._retry_snapshot = {
                    'map_payload_sha256': timeline['map_payload_sha256'],
                    'start_record': deepcopy(self._batch['start_record']),
                    'sampling': deepcopy(self._batch['candidate_sampling']),
                    'candidates': deepcopy(self._candidate_records),
                }
            self._unreachable_batches.append({
                'map_sha256': timeline['map_payload_sha256'],
                'start_sha256': timeline['start_record_sha256'],
                'candidate_sha256': timeline['candidate_sha256'],
                'fresh': True, 'reachable_count': 0,
            })
            if self._contract.unresolved_after_three(
                    self._unreachable_batches[-3:]):
                self._enter_terminal('UNRESOLVED_FRONTIERS')
            self._retry_after_observer_stamp_ns = int(
                self._observer_status['last_accepted_stamp_ns'])
            self._busy = False
            return
        self._unreachable_batches.clear()
        self._retry_snapshot = None
        self._retry_after_observer_stamp_ns = None
        ranked = self._contract.policy_rank(
            self._request['policy'], deepcopy(eligible))
        selected = ranked[0]
        self._batch.update({
            'planner_results': self._planner_records,
            'eligible_candidates': eligible,
            'selected_goal': selected,
            'stale': False,
        })
        self._state.decisions.append(deepcopy(self._batch))
        original = next(item for item in self._candidate_records
                        if item['cell_index'] == selected['cell_index'])
        self._navigate(original)

    def _navigate(self, candidate: dict) -> None:
        if self._terminal_reason is not None:
            return
        self._current_goal_recoveries = 0
        goal = NavigateToPose.Goal()
        goal.pose = _pose(
            *candidate['goal_xy_yaw'], self.get_clock().now().to_msg())
        self._navigation_send_pending = True
        future = self._navigator.send_goal_async(
            goal, feedback_callback=self._navigation_feedback)
        future.add_done_callback(self._navigation_goal_response)

    def _navigation_goal_response(self, future) -> None:
        self._navigation_send_pending = False
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._navigation_failed()
                return
            if self._terminal_reason is not None:
                self._cancel_goal_handle(handle)
                return
            self._navigation_goal_handle = handle
            result = handle.get_result_async()
            result.add_done_callback(self._navigation_result)
        except Exception as error:
            self._invalidate(f'navigation_action:{type(error).__name__}')

    def _navigation_result(self, future) -> None:
        if self._terminal_reason is not None:
            return
        try:
            wrapped = future.result()
            self._navigation_goal_handle = None
            self._finish_navigation_recoveries()
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                self._navigation_failed()
                return
            self._busy = False
        except Exception as error:
            self._invalidate(f'navigation_result:{type(error).__name__}')

    def _navigation_failed(self) -> None:
        if self._terminal_reason is not None:
            return
        self._finish_navigation_recoveries()
        self._state.navigation_outcomes['failure_count'] += 1
        if self._state.decisions:
            self._blacklist_ids.add(
                self._state.decisions[-1]['selected_goal']['cell_index'])
        self._busy = False

    def _navigation_feedback(self, message) -> None:
        if self._terminal_reason is not None:
            return
        recoveries = int(message.feedback.number_of_recoveries)
        self._current_goal_recoveries = max(
            self._current_goal_recoveries, recoveries)

    def _finish_navigation_recoveries(self) -> None:
        self._state.navigation_outcomes['recovery_count'] += (
            self._current_goal_recoveries)
        self._current_goal_recoveries = 0

    def _cancel_goal_handle(self, handle) -> None:
        self._cancel_futures_pending += 1
        future = handle.cancel_goal_async()
        future.add_done_callback(self._cancel_complete)

    def _cancel_complete(self, future) -> None:
        try:
            future.result()
        except Exception:
            self._state.invalidate('action_cancel_failed')
        self._cancel_futures_pending -= 1

    def _cancel_active_actions(self) -> None:
        self._planner_serial += 1
        if self._planner_timeout_timer is not None:
            self._planner_timeout_timer.cancel()
            self.destroy_timer(self._planner_timeout_timer)
            self._planner_timeout_timer = None
        if self._planner_goal_handle is not None:
            self._cancel_goal_handle(self._planner_goal_handle)
            self._planner_goal_handle = None
        if self._navigation_goal_handle is not None:
            self._cancel_goal_handle(self._navigation_goal_handle)
            self._navigation_goal_handle = None

    def _finish_when_stopped(self, now: float) -> None:
        if (self._planner_send_pending or self._navigation_send_pending
                or self._cancel_futures_pending):
            return
        if (not self._cmd_received and self._terminal_since is not None
                and now - self._terminal_since >= FINAL_ZERO_HOLD_S):
            self._state.invalidate('cmd_vel_evidence_unavailable')
            self._capture_graph_identity()
            self._append_terminal_samples()
            self._refresh_production_inputs()
            _atomic_json_write(self._measurement_path, self._state.payload())
            self._written = True
            rclpy.shutdown()
            return
        if self._last_cmd != (0.0, 0.0) or self._zero_since is None:
            return
        hold = now - self._zero_since
        if hold < FINAL_ZERO_HOLD_S:
            return
        self._state.command_authority.update({
            'final_zero_hold_s': hold,
            'final_linear_x': self._last_cmd[0],
            'final_angular_z': self._last_cmd[1],
        })
        self._capture_graph_identity()
        self._append_terminal_samples()
        self._refresh_production_inputs()
        _atomic_json_write(self._measurement_path, self._state.payload())
        self._written = True
        rclpy.shutdown()

    def _capture_graph_identity(self) -> None:
        command = sorted(self._cmd_authorities.items())
        self._state.command_authority['publisher_gids'] = [
            item[0] for item in command]
        self._state.command_authority['publisher_nodes'] = [
            item[1] for item in command]
        tf = sorted(self._tf_authorities.items())
        self._state.tf_authority = {
            'map_to_odom_publisher_gids': [item[0] for item in tf],
            'map_to_odom_publisher_nodes': [item[1] for item in tf],
        }
        if self._state.command_authority['publisher_nodes'] != [
                '/collision_monitor']:
            self._state.invalidate('cmd_vel_authority_drift')
        if self._state.tf_authority['map_to_odom_publisher_nodes'] != [
                EXPECTED_TF_AUTHORITY_NODE]:
            self._state.invalidate('map_to_odom_authority_drift')

    def _append_terminal_samples(self) -> None:
        coverage = self._state.coverage_samples['samples']
        revealed = coverage[-1]['revealed_reachable_cells'] if coverage else 0
        if not coverage:
            coverage.append({
                'elapsed_s': 0.0, 'revealed_reachable_cells': revealed})
        if coverage[-1]['elapsed_s'] != SIMULATION_HORIZON_S:
            coverage.append({
                'elapsed_s': SIMULATION_HORIZON_S,
                'revealed_reachable_cells': revealed,
            })
        if len(self._state.gt_pose_samples) < 2:
            self._state.invalidate('insufficient_ground_truth_trace')
        if self._state.resources['samples'] < 2:
            self._state.invalidate('insufficient_resource_samples')
        if not all(state == 'active'
                   for state in self._state.lifecycle.values()):
            self._state.invalidate('lifecycle_evidence_unavailable')
        final_ratio = (
            revealed /
            self._state.coverage_samples['reachable_denominator_cells'])
        minimum_clearance = min(
            (sample['clearance_m'] for sample in self._state.gt_pose_samples),
            default=-math.inf)
        success = (
            self._state.validity == 'VALID'
            and self._state.observer_health.get('health') == 'VALID'
            and final_ratio >= 0.85
            and bool(self._state.decisions)
            and self._terminal_reason != 'UNRESOLVED_FRONTIERS'
            and minimum_clearance >= 0.05
            and self._state.contact_count == 0
            and all(state == 'active'
                    for state in self._state.lifecycle.values()))
        self._state.outcome = 'PASS' if success else 'FAIL'

    def _refresh_production_inputs(self) -> None:
        manifest = _strict_json_load(
            Path(self._request['asset_root']) / 'asset_manifest.json')
        production = manifest.get('production_inputs')
        if type(production) is not dict:
            self._state.invalidate('production_inputs_after_unavailable')
            production = {}
        self._state.production_inputs_after = production
        if production != self._state.production_inputs_before:
            self._state.invalidate('production_inputs_changed_during_run')

    def _query_lifecycle(self, now: float) -> None:
        if now - self._last_lifecycle_query < 1.0:
            return
        self._last_lifecycle_query = now
        for name, client in self._lifecycle_clients.items():
            if name in self._lifecycle_pending:
                if (now - self._lifecycle_pending[name] >=
                        LIFECYCLE_QUERY_TIMEOUT_S):
                    self._lifecycle_pending.pop(name, None)
                    if name in self._lifecycle_seen_active:
                        self._invalidate(
                            f'lifecycle_query_timeout:{name}')
                continue
            if not client.service_is_ready():
                if name in self._lifecycle_seen_active:
                    self._lifecycle_service_misses[name] += 1
                    if self._lifecycle_service_misses[name] >= \
                            LIFECYCLE_SERVICE_MISS_LIMIT_COUNT:
                        self._invalidate(
                            f'lifecycle_service_departed:{name}')
                continue
            self._lifecycle_service_misses[name] = 0
            self._lifecycle_pending[name] = now
            future = client.call_async(GetState.Request())
            future.add_done_callback(
                lambda result, node_name=name:
                self._lifecycle_response(node_name, result))

    def _lifecycle_response(self, name: str, future) -> None:
        self._lifecycle_pending.pop(name, None)
        try:
            label = str(future.result().current_state.label).lower()
            normalized = label if label in {
                'active', 'finalized', 'inactive', 'unconfigured'} else 'unknown'
            if normalized == 'active':
                self._lifecycle_seen_active.add(name)
                self._lifecycle_service_misses[name] = 0
            elif name in self._lifecycle_seen_active:
                self._invalidate(
                    f'lifecycle_departed_active:{name}:{normalized}')
            self._state.lifecycle[name] = normalized
        except Exception:
            if name in self._lifecycle_seen_active:
                self._invalidate(f'lifecycle_query_failed:{name}')
            else:
                self._state.lifecycle[name] = 'unknown'


def main(args=None) -> None:
    """Run one G005 coordinator and always preserve an honest measurement."""
    rclpy.init(args=args)
    node = None
    try:
        node = FrontierCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None and not node._written:
            node._state.invalidate('coordinator_interrupted')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
