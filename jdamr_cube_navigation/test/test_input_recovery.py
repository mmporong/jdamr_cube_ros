"""Bounded input recovery without sending commands to robot hardware."""

from collections import deque
import inspect
import io
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import call, Mock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from jdamr_cube_navigation import box_service, corridor_route, restaurant_service
from jdamr_cube_navigation.box_service import (
    BoxObservationUnavailable, BoxSearchBudget, BoxServiceRoute,
)
from jdamr_cube_navigation.corridor_route import _quaternion_yaw, CorridorRoute
from jdamr_cube_navigation.parking import load_parking_contract, parking_controller_overrides
from jdamr_cube_navigation.restaurant_service import load_service_contract, ServiceRoute
from jdamr_cube_navigation.reverse_parking import reverse_controller_overrides
from jdamr_cube_navigation.service_destinations import route_config, taught_pose
from nav2_msgs.action import ComputePathThroughPoses, FollowPath, NavigateToPose, Spin
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import ReliabilityPolicy
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, CameraInfo, LaserScan
from std_msgs.msg import String
from tf2_ros import LookupException
import yaml


@pytest.mark.parametrize('reason,expected', [
    ('scan stale: age=0.6s limit=0.5s', True),
    ('odom stale: age=0.6s limit=0.5s', True),
    ('AMCL stale after motion', True),
    ('battery low: voltage=10.4V', False),
    ('AMCL x covariance high: value=1.0', False),
    ('live map changed', False),
    ('operator interrupt', False),
])
def test_only_temporary_input_gaps_are_recoverable(reason, expected):
    assert CorridorRoute._input_gap_recoverable(reason) is expected


def test_recovery_requires_new_stopped_odometry_and_never_dispatches(monkeypatch):
    node = object.__new__(CorridorRoute)
    node.stop_requested = False
    node._guard_failure = Mock(return_value=None)
    node.get_logger = lambda: Mock()
    clock = [0.0]
    node.latest_motion = (-1.0, 0.0, 0.0)
    monkeypatch.setattr(corridor_route.time, 'monotonic', lambda: clock[0])

    def spin(_node, timeout_sec):
        clock[0] += 0.1
        node.latest_motion = (clock[0], 0.02 if clock[0] < 0.3 else 0.0, 0.0)

    monkeypatch.setattr(corridor_route.rclpy, 'spin_once', spin)
    assert node._wait_for_input_recovery('scan stale: age=1s')
    assert clock[0] >= 0.3


def test_operator_stop_never_resumes(monkeypatch):
    node = object.__new__(CorridorRoute)
    node.stop_requested = True
    node.get_logger = lambda: Mock()
    node._guard_failure = Mock(return_value=None)
    spin = Mock()
    monkeypatch.setattr(corridor_route.rclpy, 'spin_once', spin)
    assert not node._wait_for_input_recovery('scan stale: age=1s')
    spin.assert_not_called()


def test_recovery_retries_only_remaining_waypoint_once():
    node = object.__new__(CorridorRoute)
    node.stop_requested = False
    node._wait_for_input_recovery = Mock(return_value=True)
    calls = []

    def attempt():
        calls.append(node._resume_waypoint_index)
        node._resume_waypoint_index = 1
        node._retry_guard_reason = 'scan stale: age=1s'
        return False

    assert not node._run_with_input_recovery(attempt)
    assert calls == [0, 1]
    node._wait_for_input_recovery.assert_called_once()


# ---------------------------------------------------------------------------
# Live-node harness.  One fake clock drives real subscription callbacks, the
# real guard and real recovery code.  New production symbols are read lazily
# (getattr or local import) so that collecting this module never depends on
# the fix being present.  HEAD regressions fail with 'HEADFAIL[Tn]' messages.
# ---------------------------------------------------------------------------

PACKAGE = Path(__file__).resolve().parents[1]
CONTRACT = PACKAGE / 'config/parking_contract.yaml'
BOX_CONTRACT = PACKAGE / 'config/box_parking_contract.yaml'
# The gating logic is exercised with the gated fixture; the deployed contract disables it.
SERVICE_CONTRACT = Path(__file__).parent / 'fixtures/restaurant_service_contract_gated.yaml'
PARAMS = PACKAGE / 'config/new_base_nav2_params.yaml'
OBSERVER_CONFIG = PACKAGE / 'config/depth_box_parking.yaml'
T0_S = 1000.0
DT_S = 0.05
LETHAL = 254
FRONT_EXTENT_M = 0.065
# The face center shares the 0.05 m costmap column [0.95, 1.00) with the
# 5 cm-gap front edge (0.955 m), as in the recorded grid quantization.
FACE = (0.985, 0.0)
OUTWARD = (-1.0, 0.0)
PARKED_X_M = FACE[0] - (FRONT_EXTENT_M + 0.05)
REGION_XY = (1.0, 0.0)
MOUNT = {'camera_mount': {
    'status': 'measured', 'parent_frame': 'base_link', 'child_frame': 'camera_link',
    'transform': {'x_m': 0.065, 'y_m': 0.0, 'z_m': 0.215,
                  'roll_rad': 0.0, 'pitch_rad': 0.0, 'yaw_rad': 0.0}}}
GEOMETRY = {'front_to_wheel_axis': {'value': FRONT_EXTENT_M},
            'wheel_outer_width': {'value': 0.540}}
HOME = {**taught_pose('home_dock', (-1.5, 0.0, 0.0), {}, approach_offset_m=0.7),
        'parking_direction': 'reverse'}


class _StopScenario(Exception):
    """End a scenario at a recorded action boundary."""


def _headfail(tag, error):
    """Report the exception HEAD raises as the documented regression reason."""
    raise AssertionError(f'HEADFAIL[{tag}]: {type(error).__name__}: {error}') from error


def _new_symbol(tag, module, name):
    """Read one symbol that only the planned fix provides."""
    value = getattr(module, name, None)
    assert value is not None, f'NEW[{tag}]: {module.__name__}.{name} is not implemented'
    return value


def _require_parameter(tag, function, name):
    assert name in inspect.signature(function).parameters, (
        f'NEW[{tag}]: {function.__qualname__} has no {name} parameter')


def _done(value):
    future = Future()
    future.set_result(value)
    return future


def _stamp(seconds):
    return Time(nanoseconds=round(seconds * 1e9)).to_msg()


def _handle(wrapped):
    return SimpleNamespace(
        accepted=True, goal_id=SimpleNamespace(uuid=bytes(range(16))),
        get_result_async=lambda: _done(wrapped),
        cancel_goal_async=lambda: _done(SimpleNamespace(goals_canceling=[])))


def _pose_xyyaw(pose):
    return (pose.position.x, pose.position.y, _quaternion_yaw(pose.orientation))


def _events(node, name=None):
    records = [json.loads(line) for line in node.result_stream.getvalue().splitlines()]
    return [record for record in records if name is None or record['event'] == name]


def _live_parameters(contract):
    """Generate the runtime parameters that the session launch would load."""
    document = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    controller = reverse_controller_overrides(
        parking_controller_overrides(document, contract))
    flat = {}

    def flatten(prefix, value):
        if isinstance(value, dict):
            for key, item in value.items():
                flatten(f'{prefix}{key}.', item)
        else:
            flat[prefix[:-1]] = value

    flatten('', controller)
    costmap = document['local_costmap']['local_costmap']['ros__parameters']
    footprint = {'footprint': costmap['footprint'],
                 'footprint_padding': float(costmap['footprint_padding']),
                 'robot_base_frame': 'base_footprint'}
    speed = float(contract['desired_linear_mps'])
    return {
        'controller_server': flat,
        'velocity_smoother': {'min_velocity': [-speed, 0.0, -0.2],
                              'max_velocity': [speed, 0.0, 0.2]},
        'global_costmap/global_costmap': dict(footprint),
        'local_costmap/local_costmap': dict(footprint),
    }


def _live_node(cls, monkeypatch, contract_path, home_contract_path=None):
    """Create a route with constructor defaults, without DDS or hardware."""
    monkeypatch.setattr(
        restaurant_service, 'get_package_share_directory', lambda _name: str(PACKAGE))
    monkeypatch.setattr(
        box_service, 'get_package_share_directory', lambda _name: str(PACKAGE))
    node = object.__new__(cls)
    contract = load_parking_contract(contract_path)
    service = load_service_contract(SERVICE_CONTRACT)
    registry = {'frame_id': 'map', 'tables': []}
    config = route_config(registry, {'id': 'capture', 'priority': 1, 'x_m': 0.0,
                                     'y_m': 0.0, 'yaw_rad': 0.0,
                                     'approach_offset_m': 0.5})
    defaults = {
        # CorridorRoute.__init__
        'config': config, 'start_index': 0, 'waypoints': config['waypoints'],
        'freshness_s': 2.5, 'battery_freshness_s': 30.0, 'amcl_freshness_s': 15.0,
        'revisit_motion_amcl_freshness_s': 25.0, 'revisit_motion_min_distance_m': 0.25,
        'revisit_motion_min_rotation_rad': 0.5, 'revisit_goal_amcl_tolerance_m': 0.35,
        'max_resume_start_distance_m': 6.0, 'start_reference': config['waypoints'][0],
        'start_distance_limit_m': 6.0, 'start_check_kind': 'route',
        'stop_requested': False,
        'samples': {'battery': None, 'odom': None, 'scan': None},
        'battery_voltage': None, 'amcl_seen': None, 'amcl_covariance': None,
        'amcl_position': None, 'odom_last_pose': None, 'odom_total_distance_m': 0.0,
        'odom_history': deque(maxlen=3000), 'amcl_motion_distance_m': 0.0,
        'amcl_motion_rotation_rad': 0.0, 'mobile_manipulator_protection': None,
        'travel_pose_status': None,
        'behavior_tree': str(
            PACKAGE / 'behavior_trees/navigate_to_pose_dynamic_obstacle_eval.xml'),
        'navigation_profile': 'obstacle_base_candidate', 'parking_contract': contract,
        'parking_behavior_tree': str(PACKAGE / 'behavior_trees/navigate_to_pose_parking.xml'),
        'parking_odom': None, 'parking_command': None, 'parking_motion_revision': 0,
        'parking_observation_diagnostics': None, '_last_feedback': 0.0,
        # ServiceRoute.__init__
        'start_check_pending': False, 'registry': registry,
        'result_stream': io.StringIO(), 'table_id': None, 'pose_id': None,
        'confirmation': None, 'selected_pose': None, 'active_handle': None,
        'pending_goal': None, 'navigation_result': None, 'navigation_uuid': None,
        'active_action_type': NavigateToPose, 'amcl_yaw_covariance_rad2': None,
        'alignment_behavior_tree': str(
            PACKAGE / 'behavior_trees/navigate_to_pose_alignment.xml'),
        'service_contract': service,
        'minimum_battery_v': service['minimum_running_battery_v'],
        'max_amcl_covariance': (service['max_x_covariance_m2'],
                                service['max_y_covariance_m2']),
        'live_grids': {}, 'expected_grids': {}, 'map_mismatch': None,
        'box_status': None, 'box_status_received_s': None, 'run_deadline_s': None,
        '_parameter_readers': {},
        # Attributes the planned constructor adds; the fix reads them with getattr.
        'home_contract': (load_parking_contract(home_contract_path)
                          if home_contract_path is not None else contract),
        'amcl_odom_reference': None,
    }
    if issubclass(cls, BoxServiceRoute):
        observer = yaml.safe_load(OBSERVER_CONFIG.read_text(encoding='utf-8'))
        defaults.update({
            'last_scan': None, 'candidate_trial': True, 'box_geometry': {},
            'last_box_face': None, 'depth_camera_info': None,
            'depth_info_subscription': None,
            'observer_stable_frames': observer['jdamr_depth_box_parking'][
                'ros__parameters']['stable_frames'],
        })
    for name, value in defaults.items():
        setattr(node, name, value)
    for name in ('cancel_navigation', 'query_navigation', 'cancel_reverse',
                 'query_reverse', 'cancel_spin', 'query_spin'):
        setattr(node, name, Mock())
    node.get_logger = lambda: Mock()
    return node


class _Clock:
    """Expose world time as the node's ROS clock."""

    def __init__(self, world):
        self.world = world

    def now(self):
        return Time(nanoseconds=round(self.world.now * 1e9))


class _ActionPeer:
    """Record a Nav2 motion goal, then complete it at the requested pose."""

    ACTIONS = {'NavigateToPose': NavigateToPose, 'FollowPath': FollowPath, 'Spin': Spin}

    def __init__(self, world, kind):
        self.world = world
        self.kind = kind

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, goal_uuid=None, feedback_callback=None):
        self.world.motions.append((self.kind, goal))
        if self.world.stop_on_motion:
            raise _StopScenario(self.kind)
        self.world.complete_motion(self.kind, goal)
        return _done(_handle(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED, result=self.ACTIONS[self.kind].Result())))


class _Planner:
    """Return a successful two-pose plan that ends at the requested goal."""

    def __init__(self, world):
        self.world = world

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, *args, **kwargs):
        self.world.plans.append(goal)
        target = goal.goals[-1].pose
        end = (target.position.x, target.position.y)
        result = ComputePathThroughPoses.Result()
        start = PoseStamped()
        start.pose.position.x, start.pose.position.y = self.world.pose[:2]
        start.pose.orientation.w = 1.0
        finish = PoseStamped()
        finish.pose.position.x, finish.pose.position.y = end
        finish.pose.orientation = target.orientation
        result.path.poses = [start, finish]
        return _done(_handle(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED, result=result)))


class _PathValidator:
    """Model Nav2 1.3.12 IsPathValid footprint-outline semantics."""

    def __init__(self, world):
        self.world = world

    def wait_for_service(self, timeout_sec=None):
        return True

    def call_async(self, request):
        self.world.validations.append(request.path)
        return _done(self.world.is_path_valid(request.path))


class _ParameterClient:
    """Answer read-only parameter queries from the generated launch values."""

    def __init__(self, world, remote):
        self.world = world
        self.remote = remote

    def wait_for_services(self, timeout_sec=None):
        return True

    def wait_for_service(self, timeout_sec=None):
        return True

    def get_parameters(self, names):
        return _done(self._response(names))

    def call_async(self, request):
        return _done(self._response(request.names))

    def remove_pending_request(self, _future):
        return None

    def _response(self, names):
        self.world.parameter_reads.append((self.remote, list(names)))
        values = self.world.parameters.get(self.remote, {})
        return SimpleNamespace(values=[
            Parameter('value', value=values.get(name)).get_parameter_value()
            for name in names])


class _World:
    """Drive the node's real callbacks from one deterministic 0.05 s clock."""

    def __init__(self, monkeypatch, tmp_path, node, *, pose=(0.0, 0.0, 0.0),
                 amcl_tick=1, amcl_before_odom=True,
                 covariance=(0.001, 0.001, 0.001), home=None, lethal=(),
                 camera_info=None, retain_scan=True):
        self.node = node
        self.tick = 0
        self.pose = [float(value) for value in pose]
        self.twist = (0.0, 0.0)
        self.scan_gaps = []
        self.scan_lags = []
        self.amcl_alive = True
        self.amcl_publishes = True
        self.amcl_tick = amcl_tick
        self.amcl_before_odom = amcl_before_odom
        self.amcl_reference = None
        self.amcl_scan_stamp = None
        self.covariance = tuple(covariance)
        self.tf_offset_s = 0.0
        self.last_odom_stamp = None
        self.battery_v = 12.0
        self.battery_due = True
        self.command_due = True
        self.statuses = []
        self.events = []
        self.motions = []
        self.plans = []
        self.validations = []
        self.parameter_reads = []
        self.tf_lookups = 0
        self.stop_on_motion = False
        self.lethal = set(lethal)
        self.subscriptions = []
        self.destroyed = []
        self.camera_info = camera_info
        self.retain_scan = retain_scan
        params = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
        costmap = params['local_costmap']['local_costmap']['ros__parameters']
        self.footprint = yaml.safe_load(costmap['footprint'])
        self.resolution = float(costmap['resolution'])
        self.origin = (-3.0, -3.0)
        self.size = 120
        self.parameters = _live_parameters(node.parking_contract)
        registry = {'frame_id': 'map', 'tables': [], **self._write_maps(tmp_path)}
        if home is not None:
            registry['home'] = home
        node.registry = registry
        node.get_clock = lambda: _Clock(self)
        node.parking_tf = SimpleNamespace(lookup_transform=self.lookup_transform)
        node.navigate = _ActionPeer(self, 'NavigateToPose')
        node.follow_reverse = _ActionPeer(self, 'FollowPath')
        node.spin_search = _ActionPeer(self, 'Spin')
        node.compute = _Planner(self)
        node.validate_reverse_path = _PathValidator(self)
        node.parking_parameters = _ParameterClient(self, 'controller_server')
        node.create_client = self.create_client
        node.create_subscription = self.create_subscription
        node.destroy_subscription = self.destroy_subscription
        monkeypatch.setattr(time, 'monotonic', lambda: self.now)
        monkeypatch.setattr(rclpy, 'spin_once', self.spin_once)

    @property
    def now(self):
        return T0_S + self.tick * DT_S

    def _write_maps(self, tmp_path):
        pixels = bytes([254]) * (self.size * self.size)
        paths = {}
        for name in ('map', 'keepout'):
            image = tmp_path / f'{name}.pgm'
            image.write_bytes(b'P5\n%d %d\n255\n' % (self.size, self.size) + pixels)
            metadata = tmp_path / f'{name}.yaml'
            metadata.write_text(yaml.safe_dump({
                'image': image.name, 'resolution': self.resolution,
                'origin': [*self.origin, 0.0], 'negate': 0,
                'occupied_thresh': 0.65, 'free_thresh': 0.196}))
            paths[name] = {'yaml_path': str(metadata)}
        return paths

    # ROS facades -----------------------------------------------------------
    def spin_once(self, _node=None, *, executor=None, timeout_sec=None):
        if timeout_sec is not None and timeout_sec <= 0.0:
            return
        self.step()

    def create_client(self, _service_type, name, *args, **kwargs):
        suffix = '/get_parameters'
        if name.endswith(suffix):
            return _ParameterClient(self, name[:-len(suffix)])
        return Mock()

    def create_subscription(self, message_type, topic, callback, qos, *args, **kwargs):
        subscription = SimpleNamespace(
            message_type=message_type, topic=topic, callback=callback, qos=qos, active=True)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        subscription.active = False
        self.destroyed.append(subscription)
        return True

    def lookup_transform(self, target_frame, source_frame, _time, *args, **kwargs):
        """Latest common time: min(AMCL scan + 1.0 s, odom), frozen if AMCL stops."""
        self.tf_lookups += 1
        if self.amcl_scan_stamp is None or self.last_odom_stamp is None:
            raise LookupException(f'"{target_frame}" frame does not exist')
        stamp = min(self.amcl_scan_stamp + 1.0, self.last_odom_stamp) + self.tf_offset_s
        transform = TransformStamped()
        transform.header.frame_id = target_frame
        transform.child_frame_id = source_frame
        transform.header.stamp = _stamp(stamp)
        transform.transform.translation.x = self.pose[0]
        transform.transform.translation.y = self.pose[1]
        transform.transform.rotation.z = math.sin(self.pose[2] / 2.0)
        transform.transform.rotation.w = math.cos(self.pose[2] / 2.0)
        return transform

    # Scenario controls -----------------------------------------------------
    def run(self, seconds):
        for _ in range(round(seconds / DT_S)):
            self.step()

    def at(self, when_s, action):
        self.events.append((when_s, action))

    def scan_gap(self, start_s, end_s=math.inf):
        self.scan_gaps.append((start_s, end_s))

    def scan_lag(self, start_s, end_s, lag_s):
        self.scan_lags.append((start_s, end_s, lag_s))

    def drive(self, dx, dy, dyaw, duration_s=1.0):
        steps = round(duration_s / DT_S)
        self.twist = (math.hypot(dx, dy) / duration_s, dyaw / duration_s)
        for _ in range(steps):
            self.pose[0] += dx / steps
            self.pose[1] += dy / steps
            self.pose[2] += dyaw / steps
            self.step()
        self.twist = (0.0, 0.0)
        self.run(0.5)

    def set_battery(self, voltage_v):
        self.battery_v = voltage_v
        self.battery_due = True

    def status(self, pub_s, document, receive_age_s=0.12):
        self.statuses.append({'pub': pub_s, 'age': receive_age_s,
                              'document': document, 'sent': False})

    def status_train(self, start_s, end_s, interval_s, document, receive_age_s=0.12):
        count = 0
        while start_s + count * interval_s < end_s:
            self.status(start_s + count * interval_s, document(), receive_age_s)
            count += 1

    def status_train_back(self, last_s, after_s, interval_s, document,
                          receive_age_s=0.12):
        count = 0
        while last_s - count * interval_s > after_s:
            self.status(last_s - count * interval_s, document(), receive_age_s)
            count += 1

    def clear_statuses(self, after_s, before_s=math.inf):
        self.statuses = [item for item in self.statuses
                         if item['sent'] or not after_s < item['pub'] < before_s]

    # Simulation ------------------------------------------------------------
    def step(self):
        self.tick += 1
        now = self.now
        due = [event for event in self.events if event[0] <= now + 1e-9]
        self.events = [event for event in self.events if event[0] > now + 1e-9]
        for _when, action in due:
            action()
        if self.amcl_tick == self.tick and self.amcl_before_odom:
            self.publish_amcl()
        self._publish_odom(now)
        if self.amcl_tick == self.tick and not self.amcl_before_odom:
            self.publish_amcl()
        if self.tick % 2 == 0 and not any(start <= now < end for start, end in self.scan_gaps):
            self._publish_scan(now)
        if self.battery_due or self.tick % 20 == 0:
            self.battery_due = False
            message = BatteryState()
            message.voltage = float(self.battery_v)
            self.node._battery_callback(message)
        if self.command_due:
            self.command_due = False
            self.node._parking_command_callback(Twist())
        for item in sorted(self.statuses, key=lambda value: value['pub']):
            if item['sent'] or item['pub'] > now + 1e-9:
                continue
            item['sent'] = True
            document = dict(item['document'])
            if 'frame_id' in document and 'stamp_s' not in document:
                document['stamp_s'] = item['pub'] - item['age']
            self.node._box_callback(String(data=json.dumps(document)))
        for subscription in self.subscriptions:
            if (subscription.active and self.camera_info is not None
                    and subscription.topic == '/camera/depth/camera_info'):
                subscription.callback(self.camera_info)
        deadline_tick = getattr(self.node, '_deadline_tick', None)
        if deadline_tick is not None:
            deadline_tick()

    def publish_amcl(self, covariance=None):
        if covariance is not None:
            self.covariance = tuple(covariance)
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = _stamp(self.now)
        message.pose.pose.position.x = self.pose[0]
        message.pose.pose.position.y = self.pose[1]
        message.pose.pose.orientation.z = math.sin(self.pose[2] / 2.0)
        message.pose.pose.orientation.w = math.cos(self.pose[2] / 2.0)
        values = [0.0] * 36
        values[0], values[7], values[35] = self.covariance
        message.pose.covariance = values
        self.node._amcl_callback(message)
        self.amcl_reference = tuple(self.pose)

    def _publish_odom(self, now):
        message = Odometry()
        message.header.stamp = _stamp(now)
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_footprint'
        message.pose.pose.position.x = self.pose[0]
        message.pose.pose.position.y = self.pose[1]
        message.pose.pose.orientation.z = math.sin(self.pose[2] / 2.0)
        message.pose.pose.orientation.w = math.cos(self.pose[2] / 2.0)
        message.twist.twist.linear.x = self.twist[0]
        message.twist.twist.angular.z = self.twist[1]
        self.node._odom_callback(message)
        self.last_odom_stamp = now

    def _publish_scan(self, now):
        lag = next((value for start, end, value in self.scan_lags if start <= now < end), 0.0)
        message = LaserScan()
        message.header.stamp = _stamp(now - lag)
        message.header.frame_id = 'laser_link'
        message.angle_min = -math.pi
        message.angle_increment = math.tau / 16
        message.range_min = 0.28
        message.range_max = 12.0
        message.ranges = [1.0] * 16
        if self.retain_scan:
            self.node._scan_callback(message)
        else:
            CorridorRoute._scan_callback(self.node, message)
        # AMCL republishes map->odom for every scan (stamp + transform_tolerance)
        # and publishes a pose only after an update_min_d/update_min_a motion.
        if self.amcl_alive and self.amcl_reference is not None:
            self.amcl_scan_stamp = now - lag
            dx, dy, dyaw = (abs(self.pose[index] - self.amcl_reference[index])
                            for index in range(3))
            if self.amcl_publishes and (dx > 0.05 or dy > 0.05 or dyaw > 0.05):
                self.publish_amcl()

    def complete_motion(self, kind, goal):
        if kind == 'NavigateToPose':
            self.pose = list(_pose_xyyaw(goal.pose.pose))
        elif kind == 'FollowPath':
            self.pose = list(_pose_xyyaw(goal.path.poses[-1].pose))
        else:
            self.pose[2] += goal.target_yaw
        self.command_due = True

    # Costmap model ---------------------------------------------------------
    def _cell(self, x_m, y_m):
        if x_m < self.origin[0] or y_m < self.origin[1]:
            return None
        cell = (int((x_m - self.origin[0]) / self.resolution),
                int((y_m - self.origin[1]) / self.resolution))
        return cell if max(cell) < self.size else None

    def _line_cost(self, start, end):
        (x0, y0), (x1, y1) = start, end
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        sx, sy = (1 if x1 >= x0 else -1), (1 if y1 >= y0 else -1)
        error = dx + dy
        cost = 0
        while True:
            cost = max(cost, LETHAL if (x0, y0) in self.lethal else 0)
            if (x0, y0) == (x1, y1):
                return cost
            doubled = 2 * error
            if doubled >= dy:
                error += dy
                x0 += sx
            if doubled <= dx:
                error += dx
                y0 += sy

    def footprint_cost(self, x_m, y_m, yaw):
        corners = [self._cell(x_m + px * math.cos(yaw) - py * math.sin(yaw),
                              y_m + px * math.sin(yaw) + py * math.cos(yaw))
                   for px, py in self.footprint]
        if any(corner is None for corner in corners):
            return LETHAL
        return max(self._line_cost(corners[index], corners[(index + 1) % len(corners)])
                   for index in range(len(corners)))

    def is_path_valid(self, path):
        response = IsPathValid.Response()
        response.is_valid = bool(path.poses)
        if not path.poses:
            return response
        closest = min(range(len(path.poses)), key=lambda index: math.hypot(
            path.poses[index].pose.position.x - self.pose[0],
            path.poses[index].pose.position.y - self.pose[1]))
        for item in path.poses[closest:]:
            if self.footprint_cost(*_pose_xyyaw(item.pose)) >= LETHAL:
                response.is_valid = False
                break
        return response


def _no_box():
    return {'frame_id': 'camera_color_optical_frame', 'detected': False, 'stable': False,
            'surface_kind': None, 'stable_frame_count': 0,
            'reason': 'no_box_surface_candidate', 'control_ready': False}


def _front_box():
    return {'frame_id': 'camera_color_optical_frame', 'detected': True, 'stable': True,
            'surface_kind': 'front', 'front_distance_m': 0.5, 'lateral_error_m': 0.0,
            'plane_normal': [0.0, 0.0, -1.0], 'confidence': 0.9,
            'stable_frame_count': 8, 'control_ready': False}


def _nan_box():
    return {**_no_box(), 'front_distance_m': math.nan, 'lateral_error_m': math.nan}


def _rejection(reason='stale_or_future_depth'):
    return {'detected': False, 'stable': False, 'stable_frame_count': 0,
            'reason': reason, 'control_ready': False}


def _box_target(face=FACE, outward=OUTWARD, gap_m=0.05):
    offset = FRONT_EXTENT_M + gap_m
    return {'x_m': face[0] + outward[0] * offset, 'y_m': face[1] + outward[1] * offset,
            'yaw_rad': math.atan2(-outward[1], -outward[0]),
            'face_center_map_xy_m': list(face), 'outward_normal_map_xy': list(outward)}


def _perception(monkeypatch):
    """Replace depth geometry and LiDAR fusion; keep their failure contract."""

    def compute(observation, _camera_mount, _robot_pose, *, requested_gap_m,
                front_extent_m, now_s):
        if (observation.get('detected') is not True or observation.get('stable') is not True
                or observation.get('surface_kind') != 'front'):
            raise ValueError('stable detected front surface is required')
        return {**_box_target(gap_m=requested_gap_m), 'estimate_gap_m': requested_gap_m,
                'provenance': 'NOMINAL_CAMERA_MOUNT_ESTIMATE'}

    def witness(*_args, **_kwargs):
        return {'fused_face_center_map_xy_m': list(FACE),
                'outward_normal_map_xy': list(OUTWARD), 'support_count': 8,
                'provenance': 'LIDAR_DEPTH_FACE_AGREEMENT_NOT_EXTERNAL_ACCURACY'}

    monkeypatch.setattr(box_service, 'compute_box_docking_target', compute)
    monkeypatch.setattr(box_service, 'witness_box_face_with_lidar', witness)


def _spy_recovery(node):
    """Record each real input-recovery wait and the world time it consumed."""
    calls = []
    real = node._wait_for_input_recovery

    def wait(reason, *args, **kwargs):
        started_s = time.monotonic()
        result = real(reason, *args, **kwargs)
        calls.append({'reason': reason, 'result': result,
                      'elapsed_s': time.monotonic() - started_s})
        return result

    node._wait_for_input_recovery = wait
    return calls


def _waits(calls):
    return [item for item in calls if CorridorRoute._input_gap_recoverable(item['reason'])]


def _spy_capture(node, hooks=()):
    """Run the real stationary capture, then an optional scenario hook."""
    captured = []
    real = node.capture_stationary_pose

    def capture(*args, **kwargs):
        result = real(*args, **kwargs)
        captured.append(time.monotonic())
        if len(captured) <= len(hooks) and hooks[len(captured) - 1] is not None:
            hooks[len(captured) - 1]()
        return result

    node.capture_stationary_pose = capture
    return captured


def _observer_world(monkeypatch, tmp_path, **options):
    node = _live_node(BoxServiceRoute, monkeypatch, BOX_CONTRACT, CONTRACT)
    world = _World(monkeypatch, tmp_path, node, pose=options.pop('pose', (0.4, 0.0, 0.0)),
                   **options)
    _perception(monkeypatch)
    world.run(1.0)
    world.status_train(world.now, world.now + 60.0, 0.52, _no_box)
    return node, world


def _observe(node, timeout_s=12.0, gap_m=0.45):
    return node.observe_target(MOUNT, FRONT_EXTENT_M, gap_m, REGION_XY, 0.6,
                               timeout_s=timeout_s)


def _service_world(monkeypatch, tmp_path, contract=CONTRACT, home_contract=None, **options):
    node = _live_node(ServiceRoute, monkeypatch, contract, home_contract)
    world = _World(monkeypatch, tmp_path, node, **options)
    world.run(1.0)
    return node, world


# T1/T2: AMCL is quiet by design while the robot is stationary --------------

@pytest.mark.parametrize('amcl_tick,before_odom', [(3, False), (1, True)],
                         ids=['quiet_fresh_tf', 'latched_before_odom'])
def test_t1a_quiet_capture_accepts_fresh_map_tf(monkeypatch, tmp_path, amcl_tick,
                                                before_odom):
    """Accept a 20 s quiet AMCL pose while map->base_link TF stays current."""
    node, world = _service_world(monkeypatch, tmp_path, amcl_tick=amcl_tick,
                                 amcl_before_odom=before_odom)
    world.run(20.0)
    lookups = world.tf_lookups
    try:
        pose, _evidence = node.capture_stationary_pose(timeout_s=3.0)
    except RuntimeError as error:
        _headfail('T1a', error)
    assert pose == pytest.approx(tuple(world.pose))
    assert world.tf_lookups > lookups


def test_t1b_quiet_capture_uses_amcl_axis_rule(monkeypatch, tmp_path):
    """Accept a diagonal drift that AMCL itself treats as no update."""
    node, world = _service_world(monkeypatch, tmp_path)
    world.drive(0.045, 0.040, 0.0)
    world.run(20.0)
    try:
        pose, _evidence = node.capture_stationary_pose(timeout_s=3.0)
    except RuntimeError as error:
        _headfail('T1b', error)
    assert pose == pytest.approx((0.045, 0.040, 0.0))


@pytest.mark.parametrize('motion', [(0.06, 0.0, 0.0), (0.0, 0.0, 0.06)],
                         ids=['axis_0p06', 'yaw_0p06'])
def test_t1c_quiet_capture_rejects_axis_motion(monkeypatch, tmp_path, motion):
    """Reject one axis beyond AMCL's update threshold before any TF lookup."""
    node, world = _service_world(monkeypatch, tmp_path)
    world.amcl_publishes = False
    world.drive(*motion)
    world.run(20.0)
    lookups = world.tf_lookups
    with pytest.raises(RuntimeError, match='stationary teaching unavailable'):
        node.capture_stationary_pose(timeout_s=2.0)
    assert world.tf_lookups == lookups
    reason = node._guard_failure(True)
    assert 'moved since AMCL pose' in str(reason), f'HEADFAIL[T1c]: {reason}'


def test_t1d_uninitialized_localization_stays_missing(monkeypatch, tmp_path):
    """Keep an uninitialized AMCL as missing, never as quiet."""
    node, _world = _service_world(monkeypatch, tmp_path, amcl_tick=None)
    assert node._guard_failure(True) == 'AMCL pose missing'
    with pytest.raises(RuntimeError, match='stationary teaching unavailable'):
        node.capture_stationary_pose(timeout_s=2.0)


@pytest.mark.parametrize('fault,expected', [
    ('amcl_tf_stopped', 'TF stale'), ('tf_ahead', 'ahead of ROS time')],
    ids=['amcl_tf_stopped', 'tf_ahead'])
def test_t1e_quiet_capture_requires_live_map_tf(monkeypatch, tmp_path, fault, expected):
    """Deny the quiet exemption when map->base_link TF is stale or future-dated."""
    node, world = _service_world(monkeypatch, tmp_path)
    if fault == 'amcl_tf_stopped':
        world.amcl_alive = False
    else:
        world.tf_offset_s = 0.3
    world.run(20.0)
    with pytest.raises(RuntimeError, match='stationary teaching unavailable'):
        node.capture_stationary_pose(timeout_s=2.0)
    reason = node._guard_failure(True)
    assert expected in str(reason), f'HEADFAIL[T1e]: {reason}'


def test_t1f_quiet_exemption_keeps_covariance_gate(monkeypatch, tmp_path):
    """Keep every non-age gate, including covariance, after the exemption."""
    node, world = _service_world(monkeypatch, tmp_path, covariance=(0.02, 0.001, 0.001))
    world.run(20.0)
    with pytest.raises(RuntimeError, match='stationary teaching unavailable'):
        node.capture_stationary_pose(timeout_s=2.0)
    reason = node._guard_failure(True)
    assert 'AMCL x covariance high' in str(reason), f'HEADFAIL[T1f]: {reason}'


@pytest.mark.parametrize('case', ['ok', 'tf_stopped'])
def test_t2_quiet_dwell_readiness(monkeypatch, tmp_path, case):
    """Readiness after a same-process 20 s dwell follows the live TF."""
    node, world = _service_world(monkeypatch, tmp_path)
    if case == 'tf_stopped':
        world.amcl_alive = False
    world.run(20.0)
    ready = node.wait_until_ready(timeout=1.0)
    if case == 'ok':
        assert ready is True, f'HEADFAIL[T2]: quiet dwell blocked: {node._guard_failure(True)}'
    else:
        assert ready is False
        reason = node._guard_failure(True)
        assert 'TF stale' in str(reason), f'HEADFAIL[T2]: {reason}'


# T3-T6, T19, T21, T22: one stationary observation window --------------------

def _gap_hook(world, duration_s=2.7):
    return lambda: world.scan_gap(world.now, world.now + duration_s)


def test_t3_scan_gap_recovers_once_with_new_pose_and_face(monkeypatch, tmp_path):
    """Recover one scan gap, recapture, and accept only a post-recovery face."""
    node, world = _observer_world(monkeypatch, tmp_path)
    waits = _spy_recovery(node)

    def gap():
        world.clear_statuses(world.now)
        world.scan_gap(world.now, world.now + 2.7)
        world.status_train(world.now + 0.1, world.now + 2.7, 0.52, _no_box)
        world.status_train(world.now + 2.7, world.now + 30.0, 0.52, _front_box)

    captures = _spy_capture(node, [gap])
    try:
        target = _observe(node)
    except RuntimeError as error:
        _headfail('T3', error)
    assert len(_waits(waits)) == 1 and _waits(waits)[0]['result'] is True
    assert len(captures) == 2
    observed = _events(node, 'box_target_observed')[-1]
    assert observed['observation']['stamp_s'] > captures[1]
    assert target['face_center_map_xy_m'] == pytest.approx(list(FACE))


@pytest.mark.parametrize('case', ['unrecovered', 'second_gap'])
def test_t4_unrecovered_observation_gap_never_searches(monkeypatch, tmp_path, case):
    """End an unrecovered or second input gap without any search rotation."""
    node, world = _observer_world(monkeypatch, tmp_path)
    node.search_rotation = Mock(return_value=True)
    waits = _spy_recovery(node)
    hooks = ([_gap_hook(world, math.inf)] if case == 'unrecovered'
             else [_gap_hook(world), _gap_hook(world, math.inf)])
    _spy_capture(node, hooks)
    with pytest.raises(RuntimeError) as caught:
        node._observe_with_search(
            MOUNT, FRONT_EXTENT_M, 0.45, REGION_XY, 0.6, phase='face_alignment',
            search_enabled=True, search_budget=BoxSearchBudget())
    assert type(caught.value) is RuntimeError
    assert len(_waits(waits)) == 1, (
        f'HEADFAIL[T4]: recovery waits={len(_waits(waits))}: {caught.value}')
    node.search_rotation.assert_not_called()


def _recovered_short_window(world, first):
    def gap():
        world.clear_statuses(world.now)
        gap_end = world.now + 2.7
        world.scan_gap(world.now, gap_end)
        world.status_train(world.now + 0.1, world.now + 0.6, 0.52, _no_box)
        world.status_train(world.now + 0.6, gap_end, 0.52, first)
        world.status_train(gap_end, world.now + 30.0, 0.52, _no_box)
    return gap


def test_t5_pre_gap_face_is_not_reused(monkeypatch, tmp_path):
    """Never finish from a face observed before a recovered input gap."""
    node, world = _observer_world(monkeypatch, tmp_path)
    _spy_capture(node, [_recovered_short_window(world, _front_box)])
    try:
        with pytest.raises(BoxObservationUnavailable) as caught:
            _observe(node, timeout_s=3.0)
    except RuntimeError as error:
        _headfail('T5', error)
    assert caught.value.retryable is False
    assert getattr(caught.value, 'latency_limited', None) is True
    assert not _events(node, 'box_target_observed')


@pytest.mark.parametrize('fault', ['battery', 'cov', 'map', 'stop'])
def test_t6_non_input_observation_failure_never_waits(monkeypatch, tmp_path, fault):
    """Fail at once for low battery, covariance, map change or operator stop."""
    node, world = _observer_world(monkeypatch, tmp_path)
    waits = _spy_recovery(node)
    node.expected_grids = {'map': 'registered-map-signature'}
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 2
    grid.info.resolution = 0.05
    grid.info.origin.orientation.w = 1.0
    grid.data = [0, 100, -1, 0]
    trigger = {}
    actions = {
        'battery': lambda: world.set_battery(10.4),
        'cov': lambda: world.publish_amcl(covariance=(0.02, 0.001, 0.001)),
        'map': lambda: node._map_callback('map', grid),
        'stop': node.request_stop,
    }

    def schedule():
        trigger['s'] = world.now + 0.5
        world.at(trigger['s'], actions[fault])

    _spy_capture(node, [schedule])
    with pytest.raises(RuntimeError) as caught:
        _observe(node)
    assert world.now - trigger['s'] <= 0.1 + 1e-9
    assert _waits(waits) == []
    assert sum(item['elapsed_s'] for item in waits) == 0.0
    expected = {'battery': 'battery low', 'cov': 'AMCL x covariance high'}.get(fault)
    if expected is not None:
        assert str(caught.value).startswith(expected)


def _anchored(world, timeout_s, scenario):
    """Re-anchor the status train to the observation deadline after capture."""
    def hook():
        deadline = world.now + timeout_s
        world.clear_statuses(world.now)
        scenario(world, deadline)
    return hook


def _latency_scenario(case):
    def scenario(world, deadline):
        world.status_train_back(deadline - 0.2, world.now, 0.52, _no_box)
        if case == 'observer_stale_short':
            world.status(deadline - 2.54, _rejection())
        elif case == 'observer_silent':
            world.clear_statuses(deadline - 4.4, deadline - 2.3)
        elif case == 'stale_first_eval':
            world.status(deadline - 2.54, _no_box(), receive_age_s=0.6)
        else:
            world.status(deadline - 2.54, _front_box())
            world.scan_lag(deadline - 3.2, deadline - 2.0, 0.8)
    return scenario


@pytest.mark.parametrize('case', ['observer_stale_short', 'observer_silent',
                                  'stale_first_eval', 'stale_scan_first_eval'])
def test_t19_latency_limited_window_is_not_retryable(monkeypatch, tmp_path, case):
    """Do not rotate on a window whose verdict had fewer than 8 fresh frames."""
    node, world = _observer_world(monkeypatch, tmp_path)
    _spy_capture(node, [_anchored(world, 6.0, _latency_scenario(case))])
    with pytest.raises(BoxObservationUnavailable) as caught:
        _observe(node, timeout_s=6.0)
    assert caught.value.retryable is False, (
        f'HEADFAIL[T19]: latency-limited {case} stayed retryable: {caught.value.reason}')
    assert getattr(caught.value, 'latency_limited', None) is True


def test_t19_recovered_gap_short_window_is_not_retryable(monkeypatch, tmp_path):
    """Treat a recovered input gap followed by a short window as latency-limited."""
    node, world = _observer_world(monkeypatch, tmp_path)
    _spy_capture(node, [_recovered_short_window(world, _no_box)])
    try:
        with pytest.raises(BoxObservationUnavailable) as caught:
            _observe(node, timeout_s=3.0)
    except RuntimeError as error:
        _headfail('T19', error)
    assert caught.value.retryable is False
    assert getattr(caught.value, 'latency_limited', None) is True


def test_t19c_enough_fresh_frames_keep_search_retryable(monkeypatch, tmp_path):
    """Keep search available after 8+ fresh frames follow one latency event."""
    node, world = _observer_world(monkeypatch, tmp_path)

    def scenario(world, deadline):
        world.status_train_back(deadline - 0.2, world.now, 0.52, _no_box)
        world.status(world.now + 0.3, _rejection())

    _spy_capture(node, [_anchored(world, 6.0, scenario)])
    with pytest.raises(BoxObservationUnavailable) as caught:
        _observe(node, timeout_s=6.0)
    assert caught.value.retryable is True
    assert caught.value.reason == 'stable detected front surface is required'


def test_t19d_reevaluated_status_is_not_latency(monkeypatch, tmp_path):
    """Evaluate each status stamp once; waiting for the next is not latency."""
    node, world = _observer_world(monkeypatch, tmp_path)

    def scenario(world, deadline):
        world.status_train_back(deadline - 0.45, world.now, 0.52, _no_box)

    _spy_capture(node, [_anchored(world, 4.0, scenario)])
    with pytest.raises(BoxObservationUnavailable) as caught:
        _observe(node, timeout_s=4.0)
    assert caught.value.retryable is True, (
        f'HEADFAIL[T19d]: re-evaluated status blocked search: {caught.value.reason}')
    evidence = _events(node, 'box_observation_unavailable_evidence')
    assert len(evidence) == 1 and evidence[0]['latency_reasons'] == []


def _contains(value, predicate):
    if predicate(value):
        return True
    if isinstance(value, dict):
        return any(_contains(item, predicate) for item in value.values())
    if isinstance(value, list):
        return any(_contains(item, predicate) for item in value)
    return False


@pytest.mark.parametrize('case', ['window_end', 'input_unavailable', 'last_scan_none',
                                  'nan_status'])
def test_t21_unavailable_evidence_is_strict_json(monkeypatch, tmp_path, case):
    """Emit one strict-JSON evidence record before either failure path."""
    node, world = _observer_world(monkeypatch, tmp_path,
                                  retain_scan=case != 'last_scan_none')
    documents = {'window_end': _no_box, 'input_unavailable': _no_box,
                 'last_scan_none': _front_box, 'nan_status': _nan_box}

    def hook():
        world.clear_statuses(world.now)
        world.status_train(world.now + 0.1, world.now + 30.0, 0.52, documents[case])
        if case == 'input_unavailable':
            world.scan_gap(world.now, math.inf)

    _spy_capture(node, [hook])
    with pytest.raises(RuntimeError) as caught:
        _observe(node, timeout_s=4.0 if case == 'input_unavailable' else 2.0)
    expected_type = RuntimeError if case == 'input_unavailable' else BoxObservationUnavailable
    assert type(caught.value) is expected_type
    evidence = _events(node, 'box_observation_unavailable_evidence')
    assert len(evidence) == 1, f'HEADFAIL[T21]: evidence events={len(evidence)} ({case})'
    record = evidence[0]
    for key in ('retryable', 'latency_limited', 'latency_reasons', 'fresh_since_latency'):
        assert key in record
    if case == 'last_scan_none':
        assert 'scan' in record and record['scan'] is None
    if case == 'nan_status':
        assert _contains(record, lambda value: isinstance(value, dict)
                         and 'front_distance_m' in value
                         and value['front_distance_m'] is None)


def _camera_info(fx):
    message = CameraInfo()
    message.width = 320
    message.height = 240
    message.k = [math.nan] * 9
    message.p = [fx, 0.0, 160.0, 0.0, 0.0, fx, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    return message


@pytest.mark.parametrize('case', ['e8_geometry', 'fx_zero', 'info_missing'])
def test_t22_pose_diagnostic_is_motionless(monkeypatch, tmp_path, case):
    """Report camera bearing and field of view without moving or aiming."""
    info = {'e8_geometry': _camera_info(285.171), 'fx_zero': _camera_info(0.0),
            'info_missing': None}[case]
    node, world = _observer_world(
        monkeypatch, tmp_path, pose=(1.446, 0.243, math.radians(61.7)), camera_info=info)
    for _ in range(2):
        with pytest.raises(BoxObservationUnavailable):
            node.observe_target(MOUNT, FRONT_EXTENT_M, 0.45, (1.896, 0.303), 0.6,
                                timeout_s=1.0)
    diagnostics = _events(node, 'box_observation_pose_diagnostic')
    assert diagnostics, f'HEADFAIL[T22]: no box_observation_pose_diagnostic ({case})'
    first = diagnostics[0]
    assert first['region_xy'] == pytest.approx([1.896, 0.303])
    if case == 'e8_geometry':
        assert first['in_fov'] is False
        assert first['half_fov_rad'] == pytest.approx(math.atan2(160.0, 285.171))
    else:
        assert first['half_fov_rad'] is None
    subscriptions = [item for item in world.subscriptions
                     if item.topic == '/camera/depth/camera_info']
    assert len(subscriptions) == 1
    assert subscriptions[0].message_type is CameraInfo
    assert subscriptions[0].qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert world.destroyed == ([] if info is None else subscriptions)
    assert not world.motions


# T7/T8: every _verify_parking_stop caller rechecks one input gap -------------

TABLE = taught_pose('table_01_main', (1.0, 0.5, 0.3), {}, approach_offset_m=0.5)


def _verify_scenario(monkeypatch, tmp_path, caller):
    registry_home = HOME if caller in ('home_shortcut', 'reverse') else None
    node, world = _service_world(monkeypatch, tmp_path, home=registry_home)
    if caller in ('post_goal', 'corridor', 'dwell_5s'):
        node.config = route_config(node.registry, TABLE)
        node.waypoints = node.config['waypoints']
        node.selected_pose = TABLE
        first = node.waypoints[0] if caller != 'dwell_5s' else node.waypoints[-1]
        world.pose = [first['x'], first['y'], first['yaw']]
    else:
        node.config = route_config(node.registry, HOME)
        node.waypoints = node.config['waypoints']
        start = node.waypoints[0] if caller == 'reverse' else node.waypoints[-1]
        world.pose = [start['x'], start['y'], start['yaw']]
    world.run(1.0)
    world.scan_gap(world.now, world.now + 2.7)
    world.run(0.9)
    actions = {
        'post_goal': lambda: node.execute(),
        'corridor': lambda: CorridorRoute.execute(node),
        'dwell_5s': lambda: node.wait_parked(5.0),
        'home_shortcut': lambda: node._go_home_reverse(HOME, True),
        'reverse': lambda: node._execute_reverse_path(node._make_reverse_path(
            tuple(world.pose), (HOME['x_m'], HOME['y_m'], HOME['yaw_rad']))),
    }
    return node, world, actions[caller]


@pytest.mark.parametrize('caller', ['post_goal', 'dwell_5s', 'home_shortcut', 'reverse',
                                    'corridor'])
def test_t7_verify_stop_rechecks_one_input_gap(monkeypatch, tmp_path, caller):
    """Recheck a stationary pose once after a scan gap, without a new goal."""
    node, world, action = _verify_scenario(monkeypatch, tmp_path, caller)
    waits = _spy_recovery(node)
    goals = {'post_goal': 2, 'corridor': 2, 'reverse': 1}.get(caller, 0)
    ok = action()
    assert ok is True, f'HEADFAIL[T7]: {caller} stop verification ended on a scan gap'
    assert len(_waits(waits)) == 1
    assert len(world.motions) == goals
    assert str(node.confirmation.get('input_recovery_reason')).startswith('scan stale:')


def _verify_node(monkeypatch, tmp_path):
    node, world = _service_world(monkeypatch, tmp_path)
    node.config = route_config(node.registry, TABLE)
    node.waypoints = node.config['waypoints']
    final = node.waypoints[-1]
    world.pose = [final['x'], final['y'], final['yaw']]
    world.run(1.0)
    return node, world, final


@pytest.mark.parametrize('fault', ['battery', 'stop'])
def test_t8_verify_stop_non_input_failure_never_waits(monkeypatch, tmp_path, fault):
    """End stop verification on low battery or operator stop without waiting."""
    node, world, final = _verify_node(monkeypatch, tmp_path)
    waits = _spy_recovery(node)
    world.at(world.now + 0.3, (lambda: world.set_battery(10.4)) if fault == 'battery'
             else node.request_stop)
    assert node._verify_parking_stop(1, final, None, hold_s=5.0) is False
    assert _waits(waits) == []
    assert sum(item['elapsed_s'] for item in waits) == 0.0
    assert not world.motions


def test_t8_verify_stop_second_gap_fails_after_one_recheck(monkeypatch, tmp_path):
    """Allow one recheck per verification; a second gap fails the stop."""
    node, world, final = _verify_node(monkeypatch, tmp_path)
    waits = _spy_recovery(node)
    start = world.now + 0.9
    world.scan_gap(world.now, start + 1.8)
    world.scan_gap(start + 2.6)
    world.run(0.9)
    assert node._verify_parking_stop(1, final, None, hold_s=5.0) is False
    assert len(_waits(waits)) == 1, f'HEADFAIL[T8]: recovery waits={len(_waits(waits))}'
    assert not world.motions


def test_t9_plan_pose_reports_guard_reason(monkeypatch, tmp_path):
    """Keep the guard's own reason in a navigation_not_ready plan result."""
    node, world = _service_world(monkeypatch, tmp_path)
    world.scan_gap(world.now)
    world.run(2.6)
    planned = node.plan_pose(TABLE)
    assert planned['ok'] is False and planned['reason'] == 'navigation_not_ready'
    assert str(planned.get('guard_failure')).startswith('scan stale:'), (
        f'HEADFAIL[T9]: plan result lacks guard reason: {planned}')
    json.dumps(planned, allow_nan=False)
    assert not world.plans


# T10/T11: planning after a box observation ---------------------------------

def _box_visit(monkeypatch, tmp_path, stubs):
    node = _live_node(BoxServiceRoute, monkeypatch, BOX_CONTRACT, CONTRACT)
    world = _World(monkeypatch, tmp_path, node, pose=(0.4, 0.0, 0.0))
    route = {'frame_id': 'map', 'map_yaml': 'map', 'keepout_mask_yaml': 'mask',
             'start_pose': {'x': -1.0, 'y': 0.0}, 'max_route_start_distance_m': 0.3,
             'waypoints': [{'id': 'home_exit', 'x': -0.5, 'y': 0.0},
                           {'id': 'table_01_observation', 'x': 0.4, 'y': 0.0,
                            'yaw': 0.0}]}
    monkeypatch.setattr(box_service, 'load_route', lambda _path: route)
    monkeypatch.setattr(box_service, 'verify_identity', Mock())
    node.verify_live_maps = Mock()
    node._precision_collision_ready = Mock(return_value=True)
    observations = []

    def observe(*_args, phase, **_kwargs):
        observations.append(phase)
        if len(observations) > len(stubs):
            raise _StopScenario(phase)
        return stubs[len(observations) - 1](world)

    node._observe_with_search = observe
    planned = []
    real_plan = node.plan_pose

    def plan(pose, *args, **kwargs):
        planned.append(pose['x_m'])
        return real_plan(pose, *args, **kwargs)

    node.plan_pose = plan
    world.run(1.0)
    return node, world, observations, planned


def _visit(node):
    try:
        return node.visit_observed_box(
            'route', MOUNT, GEOMETRY, 'table_01', REGION_XY, 0.6, execute=True,
            candidate_trial=True, resume_at_observation=True)
    except _StopScenario:
        return 'next_phase'


def _observed_after_gap(x_m, gap=True):
    def observe(world):
        if gap:
            world.scan_gap(world.now, world.now + 2.7)
            world.run(2.6)
        return {**_box_target(gap_m=0.45), 'x_m': x_m}
    return observe


def test_t10_box_plan_gap_reobserves_before_replanning(monkeypatch, tmp_path):
    """Discard the pre-gap face, recover once, observe again and plan the new face."""
    node, world, observations, planned = _box_visit(
        monkeypatch, tmp_path, [_observed_after_gap(0.50), _observed_after_gap(0.52, False)])
    result = _visit(node)
    assert len(planned) >= 2 and planned[1] == pytest.approx(0.52), (
        f'HEADFAIL[T10]: planned={planned} result={result}')
    assert observations[:2] == ['face_alignment', 'face_alignment']
    assert world.plans[0].goals[-1].pose.position.x == pytest.approx(0.52)


def test_t11_box_plan_non_input_failure_never_reobserves(monkeypatch, tmp_path):
    """Stop planning on low battery without a second observation."""
    def low_battery(world):
        world.set_battery(10.4)
        world.run(0.1)
        return _box_target(gap_m=0.45)

    node, world, observations, _planned = _box_visit(monkeypatch, tmp_path, [low_battery])
    assert _visit(node) is False
    assert observations == ['face_alignment']
    assert not world.motions


def test_t11_box_plan_second_gap_fails_after_one_reobservation(monkeypatch, tmp_path):
    """Recover one planning gap; a second gap ends the visit."""
    node, world, observations, _planned = _box_visit(
        monkeypatch, tmp_path, [_observed_after_gap(0.50), _observed_after_gap(0.52)])
    result = _visit(node)
    assert result is False
    assert observations == ['face_alignment', 'face_alignment'], (
        f'HEADFAIL[T11]: observations={observations}')
    assert not world.motions


# T23-T27: leave the box straight back, then return to the dock --------------

def _parked_world(monkeypatch, tmp_path, distance_m=0.115, lethal=()):
    node = _live_node(BoxServiceRoute, monkeypatch, BOX_CONTRACT, CONTRACT)
    x_m = FACE[0] - distance_m
    world = _World(monkeypatch, tmp_path, node, pose=(x_m, 0.0, 0.0), home=HOME,
                   lethal=lethal)
    node.verify_live_maps = Mock()
    node.last_box_face = {'face_center_map_xy_m': list(FACE),
                          'outward_normal_map_xy': list(OUTWARD)}
    world.stop_on_motion = True
    world.run(1.0)
    return node, world


def _go_home(node):
    try:
        return node.go_home(execute=True, timeout_s=500.0)
    except _StopScenario as stopped:
        return f'stopped at {stopped}'


def _path_length(path):
    return sum(math.dist((a.pose.position.x, a.pose.position.y),
                         (b.pose.position.x, b.pose.position.y))
               for a, b in zip(path.poses, path.poses[1:]))


def _escape_path(node, contract):
    target = (PARKED_X_M - 0.45, 0.0, 0.0)
    node.config = {'frame_id': 'map', 'waypoints': [
        {'id': 'box_escape', 'x': target[0], 'y': target[1], 'yaw': target[2]}]}
    node.waypoints = node.config['waypoints']
    return node._make_reverse_path((PARKED_X_M, 0.0, 0.0), target, path_contract=contract)


def test_t23_escape_is_first_motion_before_box(monkeypatch, tmp_path):
    """Back straight away from the box before any rotation or NavigateToPose."""
    node, world = _parked_world(monkeypatch, tmp_path)
    result = _go_home(node)
    assert world.motions, f'HEADFAIL[T23]: no motion was dispatched ({result})'
    kind, goal = world.motions[0]
    assert kind == 'FollowPath', f'HEADFAIL[T23]: first motion is {kind}'
    assert goal.controller_id == 'ParkingReverse'
    assert _path_length(goal.path) == pytest.approx(0.45, abs=0.01)


FRONT_FACE_CELLS = [(79, row) for row in range(57, 63)]
REAR_TAIL_CELL = [(62, 60)]


@pytest.mark.parametrize('case', ['front_lethal_proceeds', 'rear_lethal_blocked'])
def test_t24_escape_tail_validation(monkeypatch, tmp_path, case):
    """Exclude only the band inside the current chassis from escape validation."""
    lethal = FRONT_FACE_CELLS if case == 'front_lethal_proceeds' else REAR_TAIL_CELL
    node, world = _parked_world(monkeypatch, tmp_path, lethal=lethal)
    result = _go_home(node)
    kinds = [kind for kind, _goal in world.motions]
    if case == 'rear_lethal_blocked':
        assert kinds == [], f'HEADFAIL[T24]: blocked escape dispatched {kinds}'
        assert result is False
        assert any('box_escape_blocked' in (record['event'], record.get('reason'))
                   for record in _events(node))
        return
    assert kinds[:1] == ['FollowPath'], f'HEADFAIL[T24]: first motion is {kinds[:1]}'
    poses = world.motions[0][1].path.poses
    cumulative = [0.0]
    for a, b in zip(poses, poses[1:]):
        cumulative.append(cumulative[-1] + math.dist(
            (a.pose.position.x, a.pose.position.y), (b.pose.position.x, b.pose.position.y)))
    exclude_m = _new_symbol('T24', box_service, 'ESCAPE_VALIDATION_EXCLUDE_M')
    k = next((i for i, value in enumerate(cumulative) if value >= exclude_m - 1e-9),
             len(poses))
    k = min(k, len(poses) - 1)
    checked = _events(node, 'reverse_path_checked')[0]
    assert checked['excluded_indices'] == [0, k - 1]
    assert checked['excluded_front_band_m'] == pytest.approx(cumulative[k])
    assert world.validations[0].poses[0].pose.position.x == pytest.approx(
        poses[k].pose.position.x)


def test_t24b_short_escape_is_skipped(monkeypatch, tmp_path):
    """Skip an escape of 0.05 m or less and continue to the normal staging."""
    node, world = _parked_world(monkeypatch, tmp_path, distance_m=0.53)
    _go_home(node)
    assert [kind for kind, _goal in world.motions][:1] == ['NavigateToPose']


def test_t24c_escape_tail_keeps_one_pose(monkeypatch, tmp_path):
    """Validate at least the final pose of an escape shorter than the band."""
    _new_symbol('T24c', box_service, 'ESCAPE_VALIDATION_EXCLUDE_M')
    node, world = _parked_world(monkeypatch, tmp_path, distance_m=0.485)
    checked = []
    real = restaurant_service.static_corridor_clear

    def static(map_yaml, keepout_yaml, poses, footprint):
        checked.append(len(list(poses)))
        return real(map_yaml, keepout_yaml, poses, footprint)

    monkeypatch.setattr(restaurant_service, 'static_corridor_clear', static)
    _go_home(node)
    assert [kind for kind, _goal in world.motions][:1] == ['FollowPath']
    assert checked and min(checked) >= 1
    assert len(world.validations[0].poses) == 1


def _home_stub_node(monkeypatch, tmp_path, contract, home_contract):
    node, world = _service_world(monkeypatch, tmp_path, contract=contract,
                                 home_contract=home_contract, home=HOME,
                                 pose=(0.5, 0.0, 0.0))
    node.parking_command = None
    node.verify_live_maps = Mock()
    node.plan_pose = Mock(return_value={'ok': True, 'error_code': 0, 'reason': 'planned'})
    node.execute = Mock(return_value=True)
    node._reverse_path_valid = Mock(return_value=True)
    node._verify_parking_stop = Mock(return_value=True)
    stage = route_config(node.registry, HOME)['waypoints'][0]
    node.capture_stationary_pose = Mock(return_value=((stage['x'], stage['y'], stage['yaw']),
                                                      {}))
    return node, world, stage


def test_t25a_dock_retry_rebuilds_with_home_contract(monkeypatch, tmp_path):
    """Rebuild a resumed dock path with the 5 cm home contract, not 1 cm."""
    node, _world, stage = _home_stub_node(monkeypatch, tmp_path, BOX_CONTRACT, CONTRACT)
    # Staging heading check, reverse start, then the rebuilt retry start.
    node.capture_stationary_pose = Mock(side_effect=[
        ((stage['x'], stage['y'], stage['yaw']), {}),
        ((stage['x'], stage['y'], stage['yaw']), {}), ((-1.2, 0.02, 0.0), {})])
    node._wait_for_input_recovery = Mock(return_value=True)
    attempts = []
    options = []

    def once(path, **kwargs):
        attempts.append(path)
        options.append(kwargs)
        if len(attempts) == 1:
            node._retry_guard_reason = 'scan stale: age=2.600s limit=2.500s'
            return False
        return True

    node._execute_reverse_once = once
    try:
        ok = node._go_home_reverse(HOME, True)
    except ValueError as error:
        _headfail('T25a', error)
    assert ok is True
    assert len(attempts) == 2
    start = attempts[1].poses[0].pose.position
    assert (start.x, start.y) == pytest.approx((-1.2, 0.02))
    # The resumed dock keeps the dock's goal checker and acceptance contract.
    home = load_parking_contract(CONTRACT)
    assert options[1].get('goal_checker_id') == 'alignment_goal_checker'
    assert options[1].get('verify_contract') == {**home, 'reference_frame': 'odom'}
    assert options[1] == options[0]


def test_t25b_escape_retry_already_at_goal_is_reported(monkeypatch, tmp_path):
    """Report an escape remainder of 0.03 m as reached and let recapture judge."""
    _require_parameter('T25b', ServiceRoute._execute_reverse_path, 'final')
    contract = _new_symbol('T25b', box_service, 'ESCAPE_PATH_CONTRACT')
    node, _world = _parked_world(monkeypatch, tmp_path)
    path = _escape_path(node, contract)
    escape_x = node.waypoints[-1]['x']
    node.capture_stationary_pose = Mock(return_value=((escape_x + 0.03, 0.0, 0.0), {}))
    node._wait_for_input_recovery = Mock(return_value=True)
    attempts = []

    def once(path, **_kwargs):
        attempts.append(path)
        node._retry_guard_reason = 'scan stale: age=2.600s limit=2.500s'
        return False

    node._execute_reverse_once = once
    assert node._execute_reverse_path(
        path, path_contract=contract, validate_from_m=0.10,
        goal_checker_id='alignment_goal_checker', final=False) is True
    assert len(attempts) == 1


def test_t26_precision_staging_and_dock_use_home_contract(monkeypatch, tmp_path):
    """Stage with alignment and home verification; dock with the live checker."""
    node, world, stage = _home_stub_node(monkeypatch, tmp_path, BOX_CONTRACT, CONTRACT)
    home = load_parking_contract(CONTRACT)
    assert node._go_home_reverse(HOME, True) is True
    # Staging position leg, then the alignment leg (one odom turn only when needed).
    assert node.execute.call_args_list == [
        call(final_parking=False, staging=True),
        call(final_parking=False, alignment=True)], (
        f'HEADFAIL[T26]: precision staging used {node.execute.call_args_list}')
    # The staging plan is judged with the alignment checker's 0.05 m tolerance.
    assert len(node.plan_pose.call_args_list) == 1
    staged = node.plan_pose.call_args
    assert staged.kwargs.get('single') is True
    assert staged.kwargs.get('end_tolerance_m') == 0.05
    assert staged.args[0]['x_m'] == pytest.approx(stage['x'])
    verified = node._verify_parking_stop.call_args_list
    assert verified[0].args[0] == 0 and verified[0].args[2] is None
    assert verified[0].args[1]['x'] == pytest.approx(stage['x'])
    # Staging is confirmed on the map; the dock leg in odom (AMCL jumps, 2026-09-30).
    assert verified[0].kwargs.get('contract') == home
    assert all(item.kwargs.get('contract') == {**home, 'reference_frame': 'odom'}
               for item in verified[1:])
    kinds = [kind for kind, _goal in world.motions]
    assert kinds == ['FollowPath']
    assert world.motions[0][1].goal_checker_id == 'alignment_goal_checker'
    assert any('alignment_goal_checker.xy_goal_tolerance' in names
               for _remote, names in world.parameter_reads)


def test_t26_standard_home_keeps_head_call_arguments(monkeypatch, tmp_path):
    """Keep the standard session's staging and docking calls unchanged."""
    node, _world, stage = _home_stub_node(monkeypatch, tmp_path, CONTRACT, None)
    real_make = node._make_reverse_path
    node._make_reverse_path = Mock(side_effect=real_make)
    node._execute_reverse_path = Mock(return_value=True)
    assert node._go_home_reverse(HOME, True) is True
    assert len(node.plan_pose.call_args_list) == 1
    planned = node.plan_pose.call_args
    assert planned.kwargs == {'single': True} and len(planned.args) == 1
    assert planned.args[0]['x_m'] == pytest.approx(stage['x'])
    assert node.execute.call_args_list == [call()]
    assert all(len(item.args) == 2 and not item.kwargs
               for item in node._make_reverse_path.call_args_list)
    assert all(len(item.args) == 1 and not item.kwargs
               for item in node._reverse_path_valid.call_args_list)
    assert len(node._execute_reverse_path.call_args_list) == 1
    docked = node._execute_reverse_path.call_args
    assert len(docked.args) == 1 and not docked.kwargs
    node._verify_parking_stop.assert_not_called()


def test_t27_staging_plan_recovers_one_input_gap(monkeypatch, tmp_path):
    """Replan the same registry staging pose once after a recovered scan gap."""
    node, world = _service_world(monkeypatch, tmp_path, home=HOME, pose=(0.0, 0.0, 0.0))
    node.verify_live_maps = Mock()
    world.stop_on_motion = True
    waits = _spy_recovery(node)
    world.scan_gap(world.now, world.now + 2.7)
    world.run(2.6)
    try:
        result = node._go_home_reverse(HOME, True)
    except _StopScenario as stopped:
        result = f'stopped at {stopped}'
    kinds = [kind for kind, _goal in world.motions]
    assert kinds[:1] == ['NavigateToPose'], (
        f'HEADFAIL[T27]: staging never dispatched after a scan gap ({result})')
    assert len(_waits(waits)) == 1
    stage = route_config(node.registry, HOME)['waypoints'][0]
    assert world.plans[-1].goals[-1].pose.position.x == pytest.approx(stage['x'])


def test_t28_final_box_pose_real_dwell(monkeypatch, tmp_path):
    """Hold five verified seconds at the final box target with the box contract."""
    node = _live_node(BoxServiceRoute, monkeypatch, BOX_CONTRACT, CONTRACT)
    target = _box_target()
    world = _World(monkeypatch, tmp_path, node,
                   pose=(target['x_m'], target['y_m'], target['yaw_rad']))
    world.run(1.0)
    node.selected_pose = {'id': 'final_approach', 'x_m': target['x_m'],
                          'y_m': target['y_m'], 'yaw_rad': target['yaw_rad']}
    node.config = {'frame_id': 'map', 'waypoints': [
        {'id': 'final_approach', 'x': target['x_m'], 'y': target['y_m'],
         'yaw': target['yaw_rad']}]}
    node.waypoints = node.config['waypoints']
    started_s = world.now
    assert node.wait_parked(5.0) is True
    assert world.now - started_s >= 5.0
    assert [record['event'] for record in _events(node)] == [
        'parked_dwell_started', 'parked_dwell_observation', 'parked_dwell_complete']
    assert node.confirmation['confirmed'] is True
    assert not world.motions


# Review fixes: escape retry band, escape outcomes and pre-motion guards -------

def _box_escape_failures(node):
    return [record for record in _events(node, 'failed')
            if record.get('phase') == 'box_escape']


@pytest.mark.parametrize('case', ['rear_lethal_blocked', 'clear_proceeds'])
def test_m1_escape_retry_validates_band_already_reversed(monkeypatch, tmp_path, case):
    """Shrink a retry's excluded front band by the distance already reversed."""
    contract = _new_symbol('M1', box_service, 'ESCAPE_PATH_CONTRACT')
    node, world = _parked_world(monkeypatch, tmp_path)
    world.stop_on_motion = False
    path = _escape_path(node, contract)
    # The first reverse stops 0.295 m along the 0.45 m path; 0.155 m remain.
    stopped_x = PARKED_X_M - 0.295
    rear_x = stopped_x + min(x for x, _y in world.footprint)
    cell = world._cell(rear_x - 0.055, 0.0)
    low = world.origin[0] + cell[0] * world.resolution
    # The cell spans 0.03-0.08 m behind the current rear edge.
    assert (rear_x - low - world.resolution, rear_x - low) == pytest.approx((0.03, 0.08))
    real_once = node._execute_reverse_once
    attempts = []

    def once(sent, **kwargs):
        attempts.append(sent)
        if len(attempts) > 1:
            return real_once(sent, **kwargs)
        world.pose = [stopped_x, 0.0, 0.0]
        world.run(1.0)
        if case == 'rear_lethal_blocked':
            world.lethal = {cell}
        node._retry_guard_reason = 'scan stale: age=2.600s limit=2.500s'
        return False

    node._execute_reverse_once = once
    node._wait_for_input_recovery = Mock(return_value=True)
    ok = node._execute_reverse_path(
        path, path_contract=contract, validate_from_m=0.10,
        goal_checker_id='alignment_goal_checker', final=False)
    kinds = [kind for kind, _goal in world.motions]
    if case == 'rear_lethal_blocked':
        assert kinds == [], f'REVIEW[M1]: retry dispatched {kinds} past a live lethal cell'
        assert ok is False
        assert _events(node, 'reverse_path_checked')[-1]['valid'] is False
        return
    assert ok is True and kinds == ['FollowPath']
    sent = world.motions[0][1].path
    assert sent.poses[0].pose.position.x == pytest.approx(stopped_x)
    assert _path_length(sent) == pytest.approx(0.155, abs=0.005)


def test_m3_escape_success_clears_face_then_stages(monkeypatch, tmp_path):
    """Finish the escape and forget the face before the staging NavigateToPose."""
    node, world = _parked_world(monkeypatch, tmp_path)
    world.stop_on_motion = False
    faces_at_staging = []
    real_complete = world.complete_motion

    def complete(kind, goal):
        if kind == 'NavigateToPose':
            faces_at_staging.append(node.last_box_face)
        real_complete(kind, goal)

    world.complete_motion = complete
    assert node.go_home(execute=True, timeout_s=500.0) is True
    assert [kind for kind, _goal in world.motions] == [
        'FollowPath', 'NavigateToPose', 'NavigateToPose', 'FollowPath']
    events = [record['event'] for record in _events(node)]
    assert events.index('box_escape_finished') < events.index('reverse_staging_planned')
    # Staging position leg and alignment leg both run after the face is gone.
    assert node.last_box_face is None and faces_at_staging == [None, None]
    finished = _events(node, 'box_escape_finished')[0]
    assert finished['face_distance_m'] == pytest.approx(
        _new_symbol('M3', box_service, 'ESCAPE_CLEARANCE_M'))
    stage = route_config(node.registry, HOME)['waypoints'][0]
    staging = world.motions[1][1].pose.pose.position
    assert (staging.x, staging.y) == pytest.approx((stage['x'], stage['y']))


def test_m3_escape_not_confirmed_never_stages(monkeypatch, tmp_path):
    """Stop before staging when the recaptured face distance falls short."""
    node, world = _parked_world(monkeypatch, tmp_path)
    world.stop_on_motion = False
    real_complete = world.complete_motion

    def short(kind, goal):
        real_complete(kind, goal)
        if kind == 'FollowPath':
            # The reverse reports success 0.065 m short of its end: d = 0.50 m.
            world.pose[0] += 0.065

    world.complete_motion = short
    assert node.go_home(execute=True, timeout_s=500.0) is False
    (failed,) = _box_escape_failures(node)
    assert failed['reason'] == 'box_escape_not_confirmed'
    assert failed['face_distance_m'] == pytest.approx(0.50, abs=0.005)
    assert [kind for kind, _goal in world.motions] == ['FollowPath']
    assert _events(node, 'failed')[-1]['reason'] == 'parked_pose_exit_failed'
    assert node.last_box_face is not None


def test_t44_escape_heading_drift_still_clears_face(monkeypatch, tmp_path):
    """Confirm the escape by face clearance; heading and lateral drift only log.

    2026-09-30 water_station escapes reversed to 0.526 m from the face but ended
    6.0 deg off heading, and the service stopped before the table.
    """
    node, world = _parked_world(monkeypatch, tmp_path)
    world.stop_on_motion = False
    real_complete = world.complete_motion
    drifted = []

    def drift(kind, goal):
        real_complete(kind, goal)
        if kind == 'FollowPath' and not drifted:
            drifted.append(True)
            world.pose[1] += 0.04
            world.pose[2] += math.radians(6.0)

    world.complete_motion = drift
    assert node.go_home(execute=True, timeout_s=500.0) is True
    assert _box_escape_failures(node) == []
    finished = _events(node, 'box_escape_finished')[0]
    assert finished['yaw_error_rad'] == pytest.approx(math.radians(6.0), abs=0.01)
    assert finished['position_error_m'] == pytest.approx(0.04, abs=0.005)
    assert [kind for kind, _goal in world.motions] == [
        'FollowPath', 'NavigateToPose', 'NavigateToPose', 'FollowPath']


@pytest.mark.parametrize('case,pose', [
    ('heading_off', (PARKED_X_M, 0.0, math.radians(20.0))),
    ('not_in_front', (FACE[0] + 0.01, 0.0, 0.0)),
])
def test_l3_escape_refuses_unaligned_or_out_of_range_pose(
        monkeypatch, tmp_path, case, pose):
    """Refuse the escape before any motion unless the robot faces the recorded face."""
    node, world = _parked_world(monkeypatch, tmp_path)
    world.pose = list(pose)
    world.run(1.0)
    result = _go_home(node)
    kinds = [kind for kind, _goal in world.motions]
    assert kinds == [], f'REVIEW[L3]: {case} dispatched {kinds} ({result})'
    assert result is False
    failures = _box_escape_failures(node)
    assert [record['reason'] for record in failures] == ['box_escape_unavailable']
    assert failures[0]['detail']
    assert node.last_box_face is not None


@pytest.mark.parametrize('distance_m,yaw_deg', [(0.554, 33.0), (0.60, 0.0)])
def test_escape_is_skipped_once_rotation_clearance_exists(
        monkeypatch, tmp_path, distance_m, yaw_deg):
    """2026-09-30 table_02: a reverse ended 33 deg off at 0.554 m; go home from there."""
    node, world = _parked_world(monkeypatch, tmp_path)
    world.pose = [FACE[0] - distance_m, 0.0, math.radians(yaw_deg)]
    world.run(1.0)
    result = _go_home(node)
    assert _box_escape_failures(node) == []
    assert _events(node, 'box_escape_skipped')
    assert [kind for kind, _goal in world.motions] == ['NavigateToPose'], result


class _ShortReverse(_ActionPeer):
    """End the first reverse early with 105, as the Pi did twice on 2026-09-30."""

    def __init__(self, world, end_distance_m):
        super().__init__(world, 'FollowPath')
        self.end_distance_m = end_distance_m
        self.ended_short = False

    def send_goal_async(self, goal, goal_uuid=None, feedback_callback=None):
        if self.ended_short:
            return super().send_goal_async(goal, goal_uuid, feedback_callback)
        self.ended_short = True
        self.world.motions.append((self.kind, goal))
        self.world.pose = [FACE[0] - self.end_distance_m, 0.0, self.world.pose[2]]
        self.world.command_due = True
        result = FollowPath.Result()
        result.error_code = 105
        return _done(_handle(SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=result)))


@pytest.mark.parametrize('end_distance_m,home', [(0.526, True), (0.45, False)])
def test_short_reverse_counts_only_with_rotation_clearance(
        monkeypatch, tmp_path, end_distance_m, home):
    node, world = _parked_world(monkeypatch, tmp_path)
    world.stop_on_motion = False
    node.follow_reverse = _ShortReverse(world, end_distance_m)
    assert node.go_home(execute=True, timeout_s=500.0) is home
    kinds = [kind for kind, _goal in world.motions]
    if home:
        assert _box_escape_failures(node) == []
        assert _events(node, 'box_escape_finished')[0]['nav2_goal_reached'] is False
        assert kinds == ['FollowPath', 'NavigateToPose', 'NavigateToPose', 'FollowPath']
    else:
        assert [r['reason'] for r in _box_escape_failures(node)] == ['box_escape_failed']
        assert kinds == ['FollowPath']


def test_l4_escape_error_is_recorded_and_reraised(monkeypatch, tmp_path):
    """Record an escape validation error, restore the route and re-raise it."""
    node, world = _parked_world(monkeypatch, tmp_path)
    saved = (node.config, node.waypoints)
    node.validate_reverse_path = SimpleNamespace(
        wait_for_service=lambda timeout_sec=None: False)
    with pytest.raises(RuntimeError) as caught:
        node.go_home(execute=True, timeout_s=500.0)
    assert type(caught.value) is RuntimeError
    assert str(caught.value) == 'reverse path collision validation unavailable'
    failures = _box_escape_failures(node)
    assert failures, 'REVIEW[L4]: no box_escape failure event before the re-raise'
    assert failures[0]['reason'] == 'box_escape_unavailable'
    assert failures[0]['error_type'] == 'RuntimeError'
    assert node.config is saved[0] and node.waypoints is saved[1]
    assert not world.motions


def test_l2_dock_checker_mismatch_fails_before_escape(monkeypatch, tmp_path):
    """Reject a live alignment checker mismatch before the box escape moves."""
    node, world = _parked_world(monkeypatch, tmp_path)
    world.parameters['controller_server']['alignment_goal_checker.xy_goal_tolerance'] = 0.06
    try:
        result = node.go_home(execute=True, timeout_s=500.0)
    except _StopScenario as stopped:
        result = f'stopped at {stopped}'
    except RuntimeError as error:
        result = error
    kinds = [kind for kind, _goal in world.motions]
    assert kinds == [], f'REVIEW[L2]: {kinds} dispatched before the dock checker check'
    assert isinstance(result, RuntimeError)
    assert str(result) == 'live alignment goal checker does not match the home contract'


@pytest.mark.parametrize('case', ['window_end', 'input_unavailable'])
def test_l12_evidence_emit_failure_keeps_exception_type(monkeypatch, tmp_path, case):
    """Keep the observation exception type when its evidence record fails."""
    node, world = _observer_world(monkeypatch, tmp_path)
    real_emit = node.emit

    def emit(event, **fields):
        if event == 'box_observation_unavailable_evidence':
            raise ValueError('Out of range float values are not JSON compliant')
        return real_emit(event, **fields)

    node.emit = emit

    def hook():
        world.clear_statuses(world.now)
        world.status_train(world.now + 0.1, world.now + 30.0, 0.52, _no_box)
        if case == 'input_unavailable':
            world.scan_gap(world.now, math.inf)

    _spy_capture(node, [hook])
    with pytest.raises(RuntimeError) as caught:
        _observe(node, timeout_s=4.0 if case == 'input_unavailable' else 2.0)
    expected_type = RuntimeError if case == 'input_unavailable' else BoxObservationUnavailable
    assert type(caught.value) is expected_type
    failed = _events(node, 'evidence_emit_failed')
    assert [record['source'] for record in failed] == [
        'box_observation_unavailable_evidence']
    assert failed[0]['error_type'] == 'ValueError'
    assert not _events(node, 'box_observation_unavailable_evidence')


def test_l12_failed_dwell_never_returns_home(monkeypatch):
    """Skip the home return entirely when the parked dwell fails."""
    node = _live_node(BoxServiceRoute, monkeypatch, BOX_CONTRACT, CONTRACT)
    node.wait_parked = Mock(return_value=False)
    node.go_home = Mock(return_value=True)
    assert node.dwell_and_return_home(5.0, 500.0) is False
    node.wait_parked.assert_called_once_with(5.0)
    assert node.go_home.call_count == 0
    assert _events(node, 'failed')[-1]['phase'] == 'table_dwell'


# T31-T35: intermediate legs keep only the lost-localization bound ----------

# 9/29 transit stop (table_recovered_start_20260929_OCoNlZ): x = 0.010213 m2.
E1_X_COVARIANCE_M2 = 0.010213
E1_COVARIANCE = (E1_X_COVARIANCE_M2, 0.001, 0.001)
# Recorded unconverged states: early activation stop (xy 0.24/0.17), seed spread 0.21.
UNCONVERGED = {'e2': (0.24, 0.17, 0.001), 'seeded': (0.21, 0.21, 0.001)}
TRANSIT_WAYPOINTS = [{'id': 'home_exit', 'x': 0.5, 'y': 0.0},
                     {'id': 'table_observation', 'x': 1.0, 'y': 0.0, 'yaw': 0.0}]


def _stage_bound(node, tag):
    """Read the new stage-bound context lazily so HEAD reports HEADFAIL."""
    bound = getattr(node, '_localization_bound', None)
    if bound is None:
        raise AssertionError(f'HEADFAIL[{tag}]: stage localization bound is missing')
    return bound


def _intermediate(node):
    return getattr(node, '_intermediate_localization', None)


class _HeldNavigate(_ActionPeer):
    """Keep each NavigateToPose goal running so the in-flight guard is exercised."""

    def __init__(self, world, hold_s):
        super().__init__(world, 'NavigateToPose')
        self.hold_s = hold_s

    def send_goal_async(self, goal, goal_uuid=None, feedback_callback=None):
        world = self.world
        world.motions.append((self.kind, goal))
        done_s = world.now + self.hold_s
        wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                                  result=NavigateToPose.Result())

        class _Running:
            completed = False

            def done(self):
                if not self.completed and world.now >= done_s - 1e-9:
                    self.completed = True
                    world.complete_motion('NavigateToPose', goal)
                return self.completed

            def result(self):
                return wrapped

            def exception(self):
                return None

        running = _Running()
        return _done(SimpleNamespace(
            accepted=True, goal_id=SimpleNamespace(uuid=bytes(range(16))),
            get_result_async=lambda: running,
            cancel_goal_async=lambda: _done(SimpleNamespace(goals_canceling=[]))))


def _transit_world(monkeypatch, tmp_path, covariance, hold_s=None):
    node, world = _service_world(monkeypatch, tmp_path, covariance=covariance)
    node.waypoints = [dict(waypoint) for waypoint in TRANSIT_WAYPOINTS]
    node.config = {'frame_id': 'map', 'waypoints': node.waypoints}
    if hold_s is not None:
        node.navigate = _HeldNavigate(world, hold_s)
    return node, world


@pytest.mark.parametrize('covariance,axis', [
    (E1_COVARIANCE, 'x'), ((0.001, 0.02, 0.001), 'y'), ((0.001, 0.001, 0.04), 'yaw')],
    ids=['e1_x', 'y', 'yaw'])
def test_t31a_transit_departs_above_confidence_limit(monkeypatch, tmp_path, covariance, axis):
    """Dispatch the observation route when AMCL is only above the 0.01 m2 confidence limit."""
    node, world = _transit_world(monkeypatch, tmp_path, covariance)
    assert node._guard_failure(False) is not None, f'{axis} setup must break the strict bound'
    succeeded = node.execute(final_parking=False)
    assert succeeded and len(world.motions) == len(TRANSIT_WAYPOINTS), (
        f'HEADFAIL[T31a-{axis}]: transit blocked by {node._guard_failure(False)}')
    assert _intermediate(node) is False


def test_t31b_transit_rise_during_goal_keeps_driving(monkeypatch, tmp_path):
    """Keep the running transit goal when x covariance rises to the recorded E1 value."""
    node, world = _transit_world(monkeypatch, tmp_path, (0.001, 0.001, 0.001), hold_s=2.0)
    world.at(world.now + 0.5, lambda: world.publish_amcl(E1_COVARIANCE))
    succeeded = node.execute(final_parking=False)
    interrupted = _events(node, 'interrupted')
    assert succeeded and not interrupted, f'HEADFAIL[T31b]: {interrupted}'
    assert len(world.motions) == len(TRANSIT_WAYPOINTS)


@pytest.mark.parametrize('mode', [
    {'final_parking': False, 'alignment': True}, {'final_parking': True}],
    ids=['alignment_outside_stage', 'final'])
def test_t31c_strict_legs_keep_confidence_limit(monkeypatch, tmp_path, mode):
    """Refuse a standalone alignment leg and any final leg above 0.01 m2 before motion."""
    node, world = _transit_world(monkeypatch, tmp_path, E1_COVARIANCE)
    assert node.execute(**mode) is False
    assert not world.motions
    reason = str(node._guard_failure(False))
    assert 'AMCL x covariance high' in reason


def test_t31d_stationary_checks_keep_limit_after_transit(monkeypatch, tmp_path):
    """Restore the strict bound for stationary checks that follow a successful transit."""
    node, world = _transit_world(monkeypatch, tmp_path, E1_COVARIANCE)
    assert node.execute(final_parking=False) is True, 'HEADFAIL[T31d]: transit blocked'
    assert len(world.motions) == len(TRANSIT_WAYPOINTS)
    world.run(1.0)
    assert 'AMCL x covariance high' in str(node._guard_failure(True))
    with pytest.raises(RuntimeError, match='stationary teaching unavailable'):
        node.capture_stationary_pose(timeout_s=2.0)


@pytest.mark.parametrize('covariance,expected', [
    ((0.3, 0.001, 0.001), 'AMCL x covariance high'),
    ((0.001, 0.001, 0.07), 'AMCL yaw covariance high'),
    ((math.nan, 0.001, 0.001), 'AMCL x covariance non-finite'),
    ((0.001, -0.001, 0.001), 'AMCL position covariance invalid')],
    ids=['x_lost', 'yaw_lost', 'nan', 'negative'])
def test_t31e_transit_stops_on_lost_localization(monkeypatch, tmp_path, covariance, expected):
    """Keep the lost-localization bound and validity checks before a transit goal."""
    node, world = _transit_world(monkeypatch, tmp_path, covariance)
    assert node.execute(final_parking=False) is False
    assert not world.motions
    with _stage_bound(node, 'T31e')(True):
        assert expected in str(node._guard_failure(False))


def test_t31f_transit_input_recovery_uses_intermediate_bound(monkeypatch, tmp_path):
    """Resume the current transit waypoint after a scan gap while x covariance is E1."""
    node, world = _transit_world(monkeypatch, tmp_path, E1_COVARIANCE, hold_s=4.0)
    world.scan_gap(world.now + 0.5, world.now + 3.6)
    assert node.execute(final_parking=False) is True, 'HEADFAIL[T31f]: no resume at E1'
    reasons = [record['reason'] for record in _events(node, 'interrupted')]
    assert len(reasons) == 1 and reasons[0].startswith('scan stale:')
    assert len(world.motions) == len(TRANSIT_WAYPOINTS) + 1


def test_t31g_transit_stops_when_localization_is_lost_in_flight(monkeypatch, tmp_path):
    """Cancel a running transit goal at the lost-localization bound without a retry."""
    node, world = _transit_world(monkeypatch, tmp_path, (0.001, 0.001, 0.001), hold_s=2.0)
    world.at(world.now + 0.5, lambda: world.publish_amcl((0.3, 0.001, 0.001)))
    assert node.execute(final_parking=False) is False
    reasons = [record['reason'] for record in _events(node, 'interrupted')]
    assert reasons and 'AMCL x covariance high' in reasons[0]
    assert 'limit=0.040' in reasons[0] and 'bound=intermediate' in reasons[0], (
        f'HEADFAIL[T31g]: {reasons}')
    assert len(world.motions) == 1


def _latched_box_visit(monkeypatch, tmp_path, after_alignment=None, latched=E1_COVARIANCE):
    """Resume at the observation waypoint with AMCL latched at a transit covariance."""
    stubs = [lambda world: _box_target(gap_m=0.45)]

    def final_observation(world):
        if after_alignment is not None:
            # AMCL publishes again only after the alignment leg moved the base.
            world.publish_amcl(after_alignment)
        return _box_target(gap_m=0.05)

    stubs.append(final_observation)
    node, world, observations, _planned = _box_visit(monkeypatch, tmp_path, stubs)
    world.publish_amcl(latched)
    world.run(1.0)
    return node, world, observations


def test_t32a_latched_covariance_allows_alignment_but_not_final(monkeypatch, tmp_path):
    """Observe and align at E1, then refuse the final approach on the strict bound."""
    node, world, observations = _latched_box_visit(monkeypatch, tmp_path)
    try:
        result = _visit(node)
    except RuntimeError as error:
        _headfail('T32a', error)
    assert result is False
    assert observations == ['face_alignment', 'final_approach']
    assert [kind for kind, _goal in world.motions] == ['NavigateToPose']
    failed = _events(node, 'failed')[-1]
    assert failed['phase'] == 'final_approach'
    assert 'AMCL x covariance high' in failed['planning']['guard_failure']
    assert 'bound=strict' in failed['planning']['guard_failure']
    assert _intermediate(node) is False


def test_t32b_refreshed_covariance_allows_final_approach(monkeypatch, tmp_path):
    """Send the final approach once AMCL republishes a covariance inside 0.01 m2."""
    node, world, observations = _latched_box_visit(
        monkeypatch, tmp_path, after_alignment=(0.004, 0.001, 0.001))
    try:
        _visit(node)
    except RuntimeError as error:
        _headfail('T32b', error)
    assert observations[:2] == ['face_alignment', 'final_approach']
    # The final approach is a straight Parking FollowPath held in odom.
    assert [kind for kind, _goal in world.motions] == ['NavigateToPose', 'FollowPath']
    final = world.motions[1][1]
    assert final.controller_id == 'Parking' and final.path.header.frame_id == 'odom'


def test_t32c_observation_capture_follows_stage_bound(monkeypatch, tmp_path):
    """Capture for observation at E1 only inside the intermediate stage."""
    node, world = _observer_world(monkeypatch, tmp_path, covariance=E1_COVARIANCE)
    world.status_train(world.now, world.now + 12.0, 0.1, _front_box)
    with _stage_bound(node, 'T32c')(True):
        target = _observe(node)
    assert target['face_center_map_xy_m'] == pytest.approx(list(FACE))
    with pytest.raises(RuntimeError, match='stationary teaching unavailable.*covariance high'):
        _observe(node, timeout_s=3.0)


@pytest.mark.parametrize('precision', [True, False], ids=['precision', 'standard'])
def test_t33_home_staging_bound_follows_session(monkeypatch, tmp_path, precision):
    """Dispatch precision staging at E1; keep the standard session strict."""
    contracts = ((BOX_CONTRACT, CONTRACT) if precision else (CONTRACT, None))
    node, world = _service_world(monkeypatch, tmp_path, *contracts, home=HOME,
                                 pose=(0.0, 0.0, 0.0), covariance=E1_COVARIANCE)
    node.verify_live_maps = Mock()
    world.stop_on_motion = True
    try:
        result = node._go_home_reverse(HOME, True)
    except _StopScenario as stopped:
        result = f'stopped at {stopped}'
    kinds = [kind for kind, _goal in world.motions]
    if precision:
        assert kinds == ['NavigateToPose'], f'HEADFAIL[T33]: staging blocked ({result})'
    else:
        assert result is False and not kinds
    assert _intermediate(node) is False, 'HEADFAIL[T33]: stage flag missing or leaked'


def test_t34a_fresh_start_stays_strict(monkeypatch, tmp_path):
    """Refuse a fresh observation route start at E1 before any motion."""
    node, world, _observations = _latched_box_visit(monkeypatch, tmp_path)
    with pytest.raises(RuntimeError, match='localization or sensor data unavailable'):
        node.visit_observed_box(
            'route', MOUNT, GEOMETRY, 'table_01', REGION_XY, 0.6, execute=True,
            candidate_trial=True, resume_at_observation=False)
    assert not world.motions


def test_staging_turns_once_in_odom_when_the_heading_is_far_off(monkeypatch, tmp_path):
    """2026-09-30: the staging turn swung the long way; turn once, direction fixed at rest."""
    node, world, stage = _home_stub_node(monkeypatch, tmp_path, BOX_CONTRACT, CONTRACT)
    spins = []
    node.capture_stationary_pose = Mock(side_effect=[
        ((stage['x'], stage['y'], stage['yaw'] + math.radians(150.0)), {}),
        ((stage['x'], stage['y'], stage['yaw']), {})])
    node._search_rotation_once = Mock(side_effect=lambda delta, **kw: spins.append(
        (delta, kw)) or True)
    node._verify_parking_stop = Mock(return_value=True)
    node._execute_reverse_path = Mock(return_value=True)
    assert node._go_home_reverse(HOME, True) is True
    assert len(spins) == 1
    delta, options = spins[0]
    assert delta == pytest.approx(math.radians(-150.0))
    assert options == {'limit_rad': math.pi, 'event': 'staging_turn', 'measure_in_odom': True}


def test_final_approach_defaults_to_rpp_and_graceful_is_opt_in(monkeypatch, tmp_path):
    node, world, observations = _latched_box_visit(
        monkeypatch, tmp_path, after_alignment=(0.004, 0.001, 0.001))
    _visit(node)
    assert world.motions[1][1].controller_id == 'Parking'


def test_t34b_precision_dock_stays_strict_after_staging(monkeypatch, tmp_path):
    """Stage at E1, then refuse the staging confirmation and the dock strictly."""
    node, world = _service_world(monkeypatch, tmp_path, BOX_CONTRACT, CONTRACT, home=HOME,
                                 pose=(0.0, 0.0, 0.0), covariance=E1_COVARIANCE)
    node.verify_live_maps = Mock()
    result = node._go_home_reverse(HOME, True)
    kinds = [kind for kind, _goal in world.motions]
    # Staging position leg and alignment leg; the strict confirmation refuses.
    assert kinds == ['NavigateToPose', 'NavigateToPose'], f'HEADFAIL[T34b]: {kinds} ({result})'
    assert result is False
    confirmation = node.confirmation or {}
    assert 'covariance high' in json.dumps(confirmation), confirmation


def test_t34c_final_leg_never_inherits_intermediate(monkeypatch, tmp_path):
    """Keep a final leg strict even inside an intermediate stage."""
    node, world = _transit_world(monkeypatch, tmp_path, E1_COVARIANCE)
    with _stage_bound(node, 'T34c')(True):
        assert node.execute(final_parking=True) is False
    assert not world.motions


def test_t34d_box_escape_capture_stays_strict(monkeypatch, tmp_path):
    """Refuse the straight escape at E1 before any reverse motion."""
    node, world = _parked_world(monkeypatch, tmp_path)
    world.publish_amcl(E1_COVARIANCE)
    world.run(1.0)
    with pytest.raises(RuntimeError, match='stationary teaching unavailable.*covariance high'):
        node._leave_parked_pose()
    assert not world.motions


def test_t34e_stage_bound_restores_after_exception(monkeypatch, tmp_path):
    """Restore the strict bound when an intermediate stage raises."""
    node, _world = _transit_world(monkeypatch, tmp_path, E1_COVARIANCE)
    with pytest.raises(RuntimeError, match='stage failure'):
        with _stage_bound(node, 'T34e')(True):
            assert node._guard_failure(False) is None
            raise RuntimeError('stage failure')
    assert _intermediate(node) is False
    assert 'bound=strict' in str(node._guard_failure(False))


@pytest.mark.parametrize('state', sorted(UNCONVERGED))
def test_t35_unconverged_localization_never_moves(monkeypatch, tmp_path, state):
    """Refuse recorded unconverged filters on the transit and at an observation resume."""
    node, world = _transit_world(monkeypatch, tmp_path, UNCONVERGED[state])
    assert node.execute(final_parking=False) is False
    assert not world.motions
    resume_path = tmp_path / 'resume'
    resume_path.mkdir()
    visit_node, visit_world, _observations = _latched_box_visit(
        monkeypatch, resume_path, latched=UNCONVERGED[state])
    with pytest.raises(RuntimeError, match='localization or sensor data unavailable'):
        _visit(visit_node)
    assert not visit_world.motions


def test_final_approach_follows_the_reobserved_face_when_squared_off():
    """A 3.5 deg residual runs the path along the face normal instead of failing."""
    face, outward = (0.0, -1.0), (0.0, 1.0)
    yaw = -math.pi / 2 - math.radians(3.5)
    sent = {}
    fake = SimpleNamespace(
        capture_stationary_pose=lambda: ((0.0, -0.47, yaw), None),
        parking_contract={'yaw_tolerance_rad': math.radians(3.0)},
        emit=lambda name, **fields: sent.setdefault(name, fields),
        _json_scalar=float, config=None, waypoints=None,
        _pose=lambda _i, wp: SimpleNamespace(header=None, yaw=wp['yaw'], x=wp['x']),
        _frozen_in_odom=lambda path: (path, lambda x, y, yaw: (x, y, yaw)))

    def execute(path, **kwargs):
        sent['path'] = path
        sent['end'] = kwargs['verify_waypoint']
        return True
    fake._execute_reverse_path = execute
    reached, _ = box_service.BoxServiceRoute._straight_final_approach(
        fake, {'face_center_map_xy_m': face, 'outward_normal_map_xy': outward}, 0.085)
    assert reached
    assert sent['final_approach_straight']['heading_basis'] == 'face'
    assert sent['end']['yaw'] == pytest.approx(-math.pi / 2)
    assert sent['final_approach_straight']['travel_m'] == pytest.approx(0.53 - 0.135)
    assert all(pose.yaw == pytest.approx(-math.pi / 2) for pose in sent['path'].poses)
