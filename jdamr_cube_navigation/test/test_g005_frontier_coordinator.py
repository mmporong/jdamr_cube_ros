"""Contract tests for the G005 actual-runtime frontier coordinator."""

from collections import deque
from copy import deepcopy
import json
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from jdamr_cube_navigation.g005_frontier_coordinator import (
    _atomic_json_write,
    _sha256_json,
    build_planner_goal,
    extract_candidate_batch,
    footprint_clearance_m,
    FrontierCoordinator,
    grid_from_payload,
    load_policy_contract,
    map_status_matches,
    MeasurementState,
    occupancy_payload,
    validate_request,
)
from nav_msgs.msg import OccupancyGrid

import pytest


def _request(contract, asset_root):
    value = {
        'schema_version': 1,
        'run_id': 'current__seed_11',
        'policy': 'current',
        'layout_seed': 11,
        'ros_domain_id': 90,
        'simulation_horizon_s': 900.0,
        'asset_root': str(asset_root),
        'asset_identity': {},
        'asset_manifest_identity': {},
        'production_inputs_sha256': 'a' * 64,
        'runtime_components': {},
        'upstream_versions_required': {},
        'shared_initial_sha256': 'b' * 64,
        'shared_reveal_sha256': 'c' * 64,
        'shared_runtime_sha256': 'd' * 64,
        'required_runtime_claim': 'claim',
    }
    value['request_sha256'] = _sha256_json(value)
    return value


def _map(layout):
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.width = layout['width']
    message.info.height = layout['height']
    message.info.resolution = layout['resolution_m_per_cell']
    message.info.origin.position.x = layout['origin_m_rad'][0]
    message.info.origin.position.y = layout['origin_m_rad'][1]
    message.info.origin.orientation.w = 1.0
    message.data = layout['data']
    return message


def test_request_hash_paths_and_policy_are_fail_closed(tmp_path):
    contract = load_policy_contract()
    request_path = tmp_path / 'request.json'
    request_path.write_text('{}', encoding='utf-8')
    measurement_path = tmp_path / 'measurement.json'
    request = _request(contract, tmp_path)
    assert validate_request(
        request, request_path, measurement_path, contract) == request
    changed = deepcopy(request)
    changed['policy'] = 'nearest'
    with pytest.raises(ValueError, match='contract drift'):
        validate_request(
            changed, request_path, measurement_path, contract)
    measurement_path.write_text('{}', encoding='utf-8')
    with pytest.raises(ValueError, match='envelope drift'):
        validate_request(
            request, request_path, measurement_path, contract)


def test_occupancy_snapshot_round_trip_and_hash_are_deterministic():
    contract = load_policy_contract()
    layout = contract.build_layout(11)
    payload = occupancy_payload(_map(layout))
    assert payload == occupancy_payload(_map(layout))
    assert _sha256_json(payload) == _sha256_json(deepcopy(payload))
    grid = grid_from_payload(payload)
    assert (grid.width, grid.height) == (
        layout['width'], layout['height'])
    assert list(grid.data) == layout['data']
    bad = _map(layout)
    bad.header.frame_id = 'odom'
    with pytest.raises(ValueError, match='frame drift'):
        occupancy_payload(bad)


def test_blacklist_is_hard_excluded_before_neutral_sampling():
    contract = load_policy_contract()
    layout = contract.build_layout(11)
    observed = deepcopy(layout)
    observed['data'] = [value if value >= 65 else -1
                        for value in layout['data']]
    start = layout['start_cell'][1] * layout['width'] + layout['start_cell'][0]
    observed['data'][start] = 0
    for x in range(layout['start_cell'][0], 16):
        y = layout['start_cell'][1]
        observed['data'][y * layout['width'] + x] = 0
        observed['data'][(y - 1) * layout['width'] + x] = 0
        observed['data'][(y + 1) * layout['width'] + x] = 0
    payload = occupancy_payload(_map(observed))
    grid = grid_from_payload(payload)
    x, y = grid.cell_to_world(tuple(layout['start_cell']))
    sampling, candidates = extract_candidate_batch(
        contract, 11, payload, [x, y, 0.0], set())
    assert sampling['raw_candidate_ids']
    blocked = sampling['selected_candidate_ids'][0]
    changed, remaining = extract_candidate_batch(
        contract, 11, payload, [x, y, 0.0], {blocked})
    assert blocked in changed['excluded_candidate_ids']
    assert blocked not in changed['selected_candidate_ids']
    assert all(item['cell_index'] != blocked for item in remaining)
    assert len(changed['selected_candidate_ids']) <= contract.SAMPLE_LIMIT
    assert {item['cell_index'] for item in candidates} == set(
        sampling['selected_candidate_ids'])


def test_measurement_validity_is_sticky_and_schema_is_complete(tmp_path):
    contract = load_policy_contract()
    request = _request(contract, tmp_path)
    state = MeasurementState(request, 123, {'frontier_core': {'sha256': 'a'}})
    state.invalidate('contact')
    state.invalidate('contact')
    state.invalidate('observer')
    payload = state.payload()
    assert payload['validity'] == 'INVALID'
    assert payload['invalid_reasons'] == ['contact', 'observer']
    expected = {
        'validity', 'outcome', 'invalid_reasons', 'observer_health', 'decisions',
        'coverage_samples', 'gt_pose_samples', 'planner_batch_timeline',
        'contact_count',
        'command_authority', 'tf_authority', 'lifecycle',
        'navigation_outcomes', 'resources', 'runner_cancellation_count',
        'production_inputs_before', 'production_inputs_after',
    }
    assert set(payload) == expected
    state.validity = 'VALID'
    state.invalidate('late')
    assert state.validity == 'INVALID'


def test_atomic_measurement_write_refuses_overwrite(tmp_path):
    path = tmp_path / 'measurement.json'
    payload = {'validity': 'INVALID', 'invalid_reasons': ['test']}
    _atomic_json_write(path, payload)
    assert json.loads(path.read_text(encoding='utf-8')) == payload
    assert path.read_bytes().endswith(b'\n')
    assert not list(tmp_path.glob('.measurement.json.*'))
    with pytest.raises(ValueError, match='absent absolute path'):
        _atomic_json_write(path, payload)


def test_planner_goal_uses_exact_sealed_start_stamp():
    batch = {'start_record': {
        'stamp_ns': 12_345_678_901,
        'pose_xy_yaw': [1.0, 2.0, 0.3],
    }}
    candidate = {'goal_xy_yaw': [3.0, 4.0, -0.2]}
    goal = build_planner_goal(batch, candidate, Time(sec=99, nanosec=7))
    assert goal.start.header.stamp.sec == 12
    assert goal.start.header.stamp.nanosec == 345_678_901
    assert goal.goal.header.stamp == Time(sec=99, nanosec=7)
    assert goal.use_start is True
    assert goal.planner_id == 'GridBased'


def test_map_and_observer_status_must_bind_sequence_and_payload_hash():
    digest = 'a' * 64
    assert map_status_matches(digest, {
        'map_sequence': 3, 'map_payload_sha256': digest})
    assert not map_status_matches(digest, {
        'map_sequence': 3, 'map_payload_sha256': 'b' * 64})
    assert not map_status_matches(digest, {
        'map_sequence': 0, 'map_payload_sha256': digest})


def test_footprint_clearance_measures_body_polygon_to_cell_square():
    layout = {
        'width': 1, 'height': 1, 'resolution_m_per_cell': 0.25,
        'origin_m_rad': [1.0, 0.0, 0.0], 'data': [100],
    }
    assert footprint_clearance_m(layout, 0.0, 0.125, 0.0) == pytest.approx(
        0.77)
    assert footprint_clearance_m(layout, 0.8, 0.125, 0.0) == 0.0


def test_recovery_counts_accumulate_per_navigation_goal():
    node = object.__new__(FrontierCoordinator)
    node._terminal_reason = None
    node._current_goal_recoveries = 0
    node._state = SimpleNamespace(
        navigation_outcomes={'recovery_count': 0, 'failure_count': 0})
    for recoveries in (1, 2, 2):
        node._navigation_feedback(SimpleNamespace(
            feedback=SimpleNamespace(number_of_recoveries=recoveries)))
    node._finish_navigation_recoveries()
    node._current_goal_recoveries = 0
    node._navigation_feedback(SimpleNamespace(
        feedback=SimpleNamespace(number_of_recoveries=3)))
    node._finish_navigation_recoveries()
    assert node._state.navigation_outcomes['recovery_count'] == 5


def test_terminal_callbacks_cannot_add_planner_or_navigation_results():
    node = object.__new__(FrontierCoordinator)
    node._terminal_reason = 'HORIZON'
    node._planner_serial = 8
    node._planner_goal_handle = None
    node._planner_timeout_timer = None
    node._planner_records = []
    node._state = SimpleNamespace(
        decisions=[], navigation_outcomes={
            'recovery_count': 4, 'failure_count': 1})
    node._current_goal_recoveries = 3
    node._blacklist_ids = set()
    node._busy = True
    exploding = SimpleNamespace(result=lambda: pytest.fail(
        'terminal callback consumed a late result'))
    node._planner_result(exploding, 8)
    node._navigation_result(exploding)
    node._record_planner_result([[0.0, 0.0]], 0, 'map')
    node._navigation_failed()
    assert node._planner_records == []
    assert node._state.decisions == []
    assert node._state.navigation_outcomes == {
        'recovery_count': 4, 'failure_count': 1}


def test_actual_planner_path_frame_is_preserved_and_fail_closed():
    node = object.__new__(FrontierCoordinator)
    node._terminal_reason = None
    node._planner_serial = 1
    node._planner_goal_handle = None
    node._planner_timeout_timer = None
    node._planner_index = 0
    node._candidate_records = [{
        'cell_index': 7, 'gain_cells': 3.0, 'bfs_distance_m': 1.0,
        'heading_rad': 0.2, 'nav_length_m': 0.0,
        'goal_xy_yaw': [1.0, 2.0, 0.0],
    }]
    node._planner_records = []
    node._batch = {'decision_token': 'a' * 64}
    node._plan_next = lambda: None
    node._record_planner_result([[0.0, 0.0], [1.0, 2.0]], 0, 'odom')
    record = node._planner_records[0]
    assert record['frame_id'] == 'odom'
    contract = load_policy_contract()
    assert contract.validate_planner_batch(
        [record], 'a' * 64, 'b' * 64, 'b' * 64) == []


def test_terminal_cancels_both_active_actions_once():
    class CompletedFuture:
        def result(self):
            return object()

        def add_done_callback(self, callback):
            callback(self)

    class GoalHandle:
        def __init__(self):
            self.cancel_count = 0

        def cancel_goal_async(self):
            self.cancel_count += 1
            return CompletedFuture()

    node = object.__new__(FrontierCoordinator)
    planner = GoalHandle()
    navigator = GoalHandle()
    node._terminal_reason = None
    node._terminal_since = None
    node._planner_serial = 2
    node._planner_timeout_timer = None
    node._planner_goal_handle = planner
    node._navigation_goal_handle = navigator
    node._cancel_futures_pending = 0
    node._state = SimpleNamespace(invalidate=lambda reason: None)
    node._enter_terminal('HORIZON')
    node._enter_terminal('HORIZON')
    assert node._terminal_reason == 'HORIZON'
    assert planner.cancel_count == 1
    assert navigator.cancel_count == 1
    assert node._cancel_futures_pending == 0
    assert node._planner_goal_handle is None
    assert node._navigation_goal_handle is None


def test_tf_authority_and_lifecycle_departure_are_sticky_invalid():
    reasons = []
    expected_gid = bytes(range(16))
    endpoint = SimpleNamespace(
        endpoint_gid=expected_gid,
        node_namespace='/',
        node_name='g005_frontier_coordinator',
    )
    source_endpoint = SimpleNamespace(
        endpoint_gid=b'g' * 16,
        node_namespace='/',
        node_name='ground_truth_localization',
    )
    robot_state_endpoint = SimpleNamespace(
        endpoint_gid=b'r' * 16,
        node_namespace='/',
        node_name='robot_state_publisher',
    )
    bridge_endpoint = SimpleNamespace(
        endpoint_gid=b'b' * 16,
        node_namespace='/',
        node_name='g005_runtime_bridge',
    )
    node = object.__new__(FrontierCoordinator)
    node._tf_authorities = {}
    node._authorized_tf_payload_sha256 = deque(maxlen=200)
    node._terminal_reason = None
    node._invalidate = reasons.append
    expected_endpoints = [endpoint, robot_state_endpoint, bridge_endpoint]
    node.get_publishers_info_by_topic = lambda topic: (
        [source_endpoint] if topic == '/g005/map_to_odom_tf'
        else expected_endpoints)
    published = []
    node._tf_publisher = SimpleNamespace(publish=published.append)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'
    message = SimpleNamespace(transforms=[transform])
    FrontierCoordinator._on_authoritative_tf(node, message)
    assert published == [message]
    info = SimpleNamespace(publisher_gid=expected_gid)
    FrontierCoordinator._on_tf(node, message, info)
    assert node._tf_authorities == {
        expected_gid.hex(): '/g005_frontier_coordinator'}
    assert reasons == []

    FrontierCoordinator._on_authoritative_tf(node, message)
    unknown_endpoint = SimpleNamespace(
        endpoint_gid=expected_gid,
        node_namespace='_NODE_NAMESPACE_UNKNOWN_',
        node_name='_NODE_NAME_UNKNOWN_',
    )
    node.get_publishers_info_by_topic = lambda topic: [unknown_endpoint]
    FrontierCoordinator._on_tf(node, message, {})
    assert node._tf_authorities == {
        expected_gid.hex(): '/g005_frontier_coordinator'}

    competitor = SimpleNamespace(
        endpoint_gid=bytes(reversed(range(16))),
        node_namespace='/',
        node_name='competing_localizer',
    )
    node.get_publishers_info_by_topic = lambda topic: (
        [source_endpoint] if topic == '/g005/map_to_odom_tf'
        else expected_endpoints)
    FrontierCoordinator._on_authoritative_tf(node, message)
    node.get_publishers_info_by_topic = lambda topic: [
        *expected_endpoints, competitor]
    FrontierCoordinator._on_tf(node, message, {})
    assert reasons == ['tf_publisher_set_drift:/competing_localizer']

    node._lifecycle_pending = {'planner_server': 1.0}
    node._lifecycle_seen_active = set()
    node._lifecycle_service_misses = {'planner_server': 0}
    node._state = SimpleNamespace(lifecycle={'planner_server': 'unknown'})

    def lifecycle(label):
        return SimpleNamespace(result=lambda: SimpleNamespace(
            current_state=SimpleNamespace(label=label)))

    FrontierCoordinator._lifecycle_response(
        node, 'planner_server', lifecycle('active'))
    FrontierCoordinator._lifecycle_response(
        node, 'planner_server', lifecycle('inactive'))
    assert reasons == [
        'tf_publisher_set_drift:/competing_localizer',
        'lifecycle_departed_active:planner_server:inactive',
    ]


def test_non_mux_map_to_odom_payload_is_sticky_invalid():
    reasons = []
    node = object.__new__(FrontierCoordinator)
    node._authorized_tf_payload_sha256 = deque(['a' * 64], maxlen=200)
    node._invalidate = reasons.append
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'
    message = SimpleNamespace(transforms=[transform])
    FrontierCoordinator._on_tf(node, message, {})
    assert reasons == ['map_to_odom_payload_drift']


def test_tf_mux_accepts_source_echo_reordering_and_rejects_mixed_message():
    reasons = []
    source = SimpleNamespace(
        endpoint_gid=b'g' * 16,
        node_namespace='/', node_name='ground_truth_localization')
    mux = SimpleNamespace(
        endpoint_gid=b'm' * 16,
        node_namespace='/', node_name='g005_frontier_coordinator')
    bridge = SimpleNamespace(
        endpoint_gid=b'b' * 16,
        node_namespace='/', node_name='g005_runtime_bridge')
    robot_state = SimpleNamespace(
        endpoint_gid=b'r' * 16,
        node_namespace='/', node_name='robot_state_publisher')
    node = object.__new__(FrontierCoordinator)
    node._authorized_tf_payload_sha256 = deque(maxlen=200)
    node._tf_authorities = {}
    node._invalidate = reasons.append
    node._tf_publisher = SimpleNamespace(publish=lambda message: None)
    node.get_publishers_info_by_topic = lambda topic: (
        [source] if topic == '/g005/map_to_odom_tf'
        else [mux, bridge, robot_state])

    first = TransformStamped()
    first.header.frame_id = 'map'
    first.child_frame_id = 'odom'
    second = deepcopy(first)
    second.header.stamp.nanosec = 1
    FrontierCoordinator._on_authoritative_tf(
        node, SimpleNamespace(transforms=[first]))
    FrontierCoordinator._on_authoritative_tf(
        node, SimpleNamespace(transforms=[second]))
    FrontierCoordinator._on_tf(
        node, SimpleNamespace(transforms=[first]), {})
    assert reasons == []

    extra = TransformStamped()
    extra.header.frame_id = 'odom'
    extra.child_frame_id = 'base_link'
    FrontierCoordinator._on_tf(
        node, SimpleNamespace(transforms=[second, extra]), {})
    assert reasons == [
        'map_to_odom_message_contract:'
        'expected one map to odom transform']


def test_cmd_vel_authority_is_run_wide_and_sticky():
    reasons = []
    expected_gid = b'c' * 16
    expected = SimpleNamespace(
        endpoint_gid=expected_gid,
        node_namespace='/',
        node_name='collision_monitor',
    )
    node = object.__new__(FrontierCoordinator)
    node._cmd_authorities = {}
    node._cmd_received = False
    node._last_cmd = (0.0, 0.0)
    node._zero_since = None
    node._invalidate = reasons.append
    node.get_publishers_info_by_topic = lambda topic: [expected]
    FrontierCoordinator._on_cmd_vel(node, Twist())
    assert node._cmd_authorities == {
        expected_gid.hex(): '/collision_monitor'}
    assert reasons == []

    competitor = SimpleNamespace(
        endpoint_gid=b'x' * 16,
        node_namespace='/',
        node_name='unsafe_commander',
    )
    node.get_publishers_info_by_topic = lambda topic: [expected, competitor]
    FrontierCoordinator._on_cmd_vel(node, Twist())
    assert reasons == ['cmd_vel_authority_drift:/unsafe_commander']


def test_active_lifecycle_service_departure_is_sticky_invalid():
    reasons = []
    unavailable = SimpleNamespace(service_is_ready=lambda: False)
    node = object.__new__(FrontierCoordinator)
    node._last_lifecycle_query = 0.0
    node._lifecycle_clients = {'planner_server': unavailable}
    node._lifecycle_pending = {}
    node._lifecycle_seen_active = {'planner_server'}
    node._lifecycle_service_misses = {'planner_server': 0}
    node._invalidate = reasons.append
    for now in (2.0, 4.0, 6.0):
        FrontierCoordinator._query_lifecycle(node, now)
    assert reasons == ['lifecycle_service_departed:planner_server']

    reasons.clear()
    node._last_lifecycle_query = 0.0
    node._lifecycle_pending = {'planner_server': 1.0}
    FrontierCoordinator._query_lifecycle(node, 4.0)
    assert reasons == ['lifecycle_query_timeout:planner_server']
