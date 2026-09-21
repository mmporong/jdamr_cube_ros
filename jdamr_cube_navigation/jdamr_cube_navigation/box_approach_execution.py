"""Explicitly armed, fail-closed executor for metric box approach."""

from collections import deque
import hashlib
import json
import math
from pathlib import Path
import signal
import time

from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped, Twist
from jdamr_cube_navigation.box_approach import ApproachConfig, BoxApproach
from jdamr_cube_navigation.box_approach_shadow import (
    camera_geometry, lower_base_geometry, pose_at, stamp_seconds,
)
from jdamr_cube_navigation.new_base_contract import validate_new_base_params
from jdamr_cube_navigation.parking import load_parking_contract
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import Toggle
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


MAX_TRAVEL_M = 1.0
MAX_RUN_S = 90.0
MAX_LINEAR_MPS = 0.06
MAX_ANGULAR_RADPS = 0.20
VALIDATION_REFERENCE_MIN_M = 0.65
VALIDATION_REFERENCE_MAX_M = 1.0
VALIDATION_REFERENCE_TOLERANCE_M = 0.04
VALIDATION_TRIAL_MAX_TRAVEL_M = 0.45
VALIDATION_TRIAL_MAX_RUN_S = 30.0
VALIDATION_TRIAL_MAX_LINEAR_MPS = 0.03
VALIDATION_TRIAL_MAX_ANGULAR_RADPS = 0.10
VALIDATION_TRIAL_MODE = 'physical_validation_trial'
GRAPH_SAMPLE_S = 0.5
GRAPH_FRESHNESS_S = 0.75
CM_ENABLE_POLL_S = 1.0
CM_ENABLE_FRESHNESS_S = 2.0
# Longer than the 0.4 s executor watchdog and 0.5 s smoother timeout.
SHUTDOWN_ZERO_DRAIN_S = 0.6
SHUTDOWN_ZERO_PERIOD_S = 0.05


def file_sha256(path):
    """Hash the exact deployed bytes used by the executor."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def physical_validation(
        path, camera_mount_file, geometry_file, nav_params_file,
        parking_contract_file):
    """Require an external approval bound to every motion-defining input."""
    if not path:
        raise ValueError('physical_validation_file_missing')
    document = yaml.safe_load(Path(path).read_text())
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('physical_validation_schema_invalid')
    if document.get('physically_validated') is not True:
        raise ValueError('physical_validation_not_approved')
    evidence = document.get('evidence')
    if not isinstance(evidence, str) or not evidence.strip():
        raise ValueError('physical_validation_evidence_missing')
    expected = {
        'calibration_sha256': file_sha256(camera_mount_file),
        'geometry_sha256': file_sha256(geometry_file),
        'nav_params_sha256': file_sha256(nav_params_file),
        'parking_contract_sha256': file_sha256(parking_contract_file),
    }
    if any(document.get(key) != value for key, value in expected.items()):
        raise ValueError('physical_validation_hash_mismatch')
    return document


def footprint_from_nav_params(path, geometry_file=None):
    """Load the exact padded local-costmap footprint, without synthesizing it."""
    document = yaml.safe_load(Path(path).read_text())
    if geometry_file is not None:
        geometry = yaml.safe_load(Path(geometry_file).read_text())
        validate_new_base_params(document, geometry)
    value = document['local_costmap']['local_costmap']['ros__parameters']['footprint']
    points = yaml.safe_load(value) if isinstance(value, str) else value
    if (not isinstance(points, list) or len(points) < 3
            or any(not isinstance(point, list) or len(point) != 2
                   or any(isinstance(v, bool) or not isinstance(v, (int, float))
                          or not math.isfinite(v) for v in point)
                   for point in points)):
        raise ValueError('invalid_local_costmap_footprint')
    return tuple((float(x), float(y)) for x, y in points)


def ownership_reasons(owners):
    """Require the unshared smoother-to-collision-monitor velocity chain."""
    expected = {
        '/cmd_vel_nav': {('box_approach_execution', '/')},
        '/cmd_vel_smoothed': {('velocity_smoother', '/')},
        '/cmd_vel': {('collision_monitor', '/')},
        '/box_parking/perception_status': {('jdamr_depth_box_parking', '/')},
        '/local_costmap/published_footprint': {('box_approach_execution', '/')},
    }
    return [f'publisher_owner_mismatch:{topic}'
            for topic, required in expected.items()
            if (len(owners.get(topic, ())) != len(required)
                or set(owners.get(topic, ())) != required)]


def validation_reference_reason(reference_m):
    """Validate the one-shot measured starting distance for a trial."""
    if (isinstance(reference_m, bool)
            or not isinstance(reference_m, (int, float))
            or not math.isfinite(reference_m)):
        return 'validation_reference_invalid'
    if reference_m == 0.0:
        return 'validation_reference_missing'
    if not VALIDATION_REFERENCE_MIN_M <= reference_m <= VALIDATION_REFERENCE_MAX_M:
        return 'validation_reference_out_of_range'
    return None


def readiness_reasons(
        observation, received, *, now_mono_s, now_ros_s, battery,
        collision_action, final_command, lifecycle, owners,
        charger_unplugged_confirmed, optical_frame, odom_stamp_s,
        odom_velocity, zero_witness_valid, require_start=True,
        require_command_fresh=None, graph_received_mono_s=None,
        cm_enable_valid=True):
    """Evaluate independent execution evidence without changing observer data."""
    reasons = ownership_reasons(owners)
    if (graph_received_mono_s is None
            or not 0.0 <= now_mono_s - graph_received_mono_s
            <= GRAPH_FRESHNESS_S):
        reasons.append('graph_snapshot_stale')
    if not cm_enable_valid:
        reasons.append('collision_monitor_enable_unconfirmed')
    if not charger_unplugged_confirmed:
        reasons.append('charger_unplugged_not_confirmed')
    status, voltage = battery
    if status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
        reasons.append('battery_charging')
    if not math.isfinite(voltage) or voltage < 10.5:
        reasons.append('battery_voltage_low_or_invalid')
    limits = {'perception': .5, 'odom': .5, 'scan': .5,
              'battery': 3., 'footprint': .5}
    if require_command_fresh is None:
        require_command_fresh = not require_start
    if require_command_fresh:
        limits['command'] = .5
    for name, limit in limits.items():
        age = now_mono_s - received.get(name, -math.inf)
        if not 0.0 <= age <= limit:
            reasons.append(f'{name}_stale')
    for name in ('velocity_smoother', 'collision_monitor'):
        state, observed = lifecycle.get(name, (None, -math.inf))
        if state != State.PRIMARY_STATE_ACTIVE:
            reasons.append(f'{name}_not_active')
        if not 0.0 <= now_mono_s - observed <= 3.0:
            reasons.append(f'{name}_lifecycle_stale')
    if (collision_action is not None
            and collision_action not in (
                CollisionMonitorState.DO_NOTHING,
                CollisionMonitorState.SLOWDOWN)):
        reasons.append('collision_monitor_intervention')
    if require_start and not zero_witness_valid:
        reasons.append('final_zero_not_observed')
    if (require_start
            and (not all(math.isfinite(v) for v in odom_velocity)
                 or abs(odom_velocity[0]) > .01
                 or abs(odom_velocity[1]) > .02)):
        reasons.append('odom_not_stationary')
    if (not math.isfinite(odom_stamp_s)
            or not 0.0 <= now_ros_s - odom_stamp_s <= .5):
        reasons.append('odom_source_stamp_stale')
    try:
        stamp_s = float(observation['stamp_s'])
        values = (observation['front_distance_m'],
                  observation['lateral_error_m'],
                  observation['edge_angle_deg'], observation['confidence'])
        if (observation.get('detected') is not True
                or observation.get('surface_kind') != 'front'
                or observation.get('frame_id') != optical_frame
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in values)
                or not 0.0 < observation['front_distance_m'] <= 2.0
                or (require_start and observation.get('stable') is not True)
                or (require_start and observation['confidence'] < .8)
                or not 0.0 <= now_ros_s - stamp_s <= .5):
            raise ValueError
    except (KeyError, TypeError, ValueError, OverflowError):
        reasons.append('perception_observation_invalid')
    return reasons


class BoxApproachExecution(Node):
    """Own calculation, authorization, and bounded Nav2 input publication."""

    def __init__(self, **kwargs):
        """Start disarmed and publish only zero while ownership is exclusive."""
        super().__init__('box_approach_execution', **kwargs)
        defaults = {
            'camera_mount_file': '', 'geometry_file': '',
            'parking_contract_file': '', 'nav_params_file': '',
            'physical_validation_file': '',
            # One service call consumes this measured starting distance.
            'validation_reference_m': 0.0,
            # Ephemeral operator confirmation: every process restart resets false.
            'charger_unplugged_confirmed': False,
            'optical_frame': 'camera_color_optical_frame',
        }
        defaults.update(vars(ApproachConfig()))
        self.declare_parameters('', list(defaults.items()))
        values = {name: self.get_parameter(name).value for name in defaults}
        camera = camera_geometry(values['camera_mount_file'])
        body, self.envelope_scope = lower_base_geometry(values['geometry_file'])
        contract = load_parking_contract(Path(values['parking_contract_file']))
        config = ApproachConfig(**{name: values[name]
                                   for name in vars(ApproachConfig())})
        self.policy = BoxApproach(contract, camera, body, config)
        self.footprint = footprint_from_nav_params(
            values['nav_params_file'], values['geometry_file'])
        self.values = values
        self.observation, self.received = {}, {}
        self.pose_history = deque(maxlen=256)
        self.pose, self.velocity, self.final_command = ((0., 0., 0.),
                                                        (0., 0.), (0., 0.))
        self.odom_stamp_s = -math.inf
        self.battery = (BatteryState.POWER_SUPPLY_STATUS_UNKNOWN, math.nan)
        self.collision_action = None
        self.lifecycle = {}
        self.lifecycle_pending = {}
        self.activation_epochs = {
            'velocity_smoother': 0, 'collision_monitor': 0}
        self.chain_epoch = 0
        self.chain_signatures = {}
        self.cached_owners = {}
        self.cached_chain_signatures = {}
        self.graph_received_mono_s = -math.inf
        self.zero_witness = None
        self.cm_enable_ack = None
        self.cm_toggle_pending = None
        self.armed = False
        self.latched = False
        self.terminal_state = None
        self.execution_mode = 'production'
        self.validation_reference_m = None
        self.reason = 'DISARMED'
        self.start_pose = None
        self.started_mono_s = None
        self.motion_sent = False
        self.first_motion_publish_mono_s = None
        self.first_motion_echo_mono_s = None
        self.first_motion_chain_witness = None
        self.travel_m = 0.0
        self.travel_pose = None
        self.last_publish_mono_s = -math.inf
        self._monotonic = time.monotonic
        self.command_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.status_pub = self.create_publisher(
            String, '/box_parking/execution_status', 10)
        self.footprint_pub = self.create_publisher(
            PolygonStamped, '/local_costmap/published_footprint', 10)
        self.create_subscription(String, '/box_parking/perception_status',
                                 self.on_observation, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/battery_state', self.on_battery,
                                 qos_profile_sensor_data)
        self.create_subscription(CollisionMonitorState,
                                 '/collision_monitor_state', self.on_collision, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_final_command, 10)
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in ('velocity_smoother', 'collision_monitor')}
        self.cm_toggle_client = self.create_client(
            Toggle, '/collision_monitor/toggle')
        self.create_service(Trigger, '/box_parking/start_approach', self.on_start)
        self.create_service(
            Trigger, '/box_parking/start_validation_approach',
            self.on_start_validation)
        self.create_service(Trigger, '/box_parking/cancel_approach', self.on_cancel)
        self.create_timer(.1, self.tick)
        self.create_timer(.5, self.poll_lifecycle)
        self.create_timer(GRAPH_SAMPLE_S, self.sample_graph)
        self.create_timer(CM_ENABLE_POLL_S, self.poll_cm_enabled)

    def now_ros_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def owners(self):
        """Return the latest bounded-rate graph snapshot without querying ROS."""
        return {topic: list(items) for topic, items in self.cached_owners.items()}

    def invalidate_chain_witness(self, component):
        """Forget zero evidence after a concrete protection-chain epoch change."""
        self.chain_epoch += 1
        self.zero_witness = None
        self.received.pop('command', None)
        if component == 'collision_monitor':
            self.collision_action = None
            self.received.pop('collision', None)
            self.cm_enable_ack = None

    def refresh_chain_identity(self):
        """Invalidate witnesses from the latest cached publisher identities."""
        current = self.cached_chain_signatures
        for component, signature in current.items():
            previous = self.chain_signatures.get(component)
            if previous is not None and signature != previous:
                self.invalidate_chain_witness(component)
            self.chain_signatures[component] = signature

    def sample_graph(self):
        """Query five topics once per cache interval and derive one snapshot."""
        topics = ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel',
                  '/box_parking/perception_status',
                  '/local_costmap/published_footprint')
        infos = {topic: tuple(self.get_publishers_info_by_topic(topic))
                 for topic in topics}
        self.cached_owners = {
            topic: [(item.node_name, item.node_namespace) for item in items]
            for topic, items in infos.items()}
        self.cached_chain_signatures = {
            name: tuple((item.node_name, item.node_namespace,
                         bytes(item.endpoint_gid)) for item in infos[topic])
            for name, topic in {
                'velocity_smoother': '/cmd_vel_smoothed',
                'collision_monitor': '/cmd_vel',
            }.items()
        }
        self.refresh_chain_identity()
        self.graph_received_mono_s = self._monotonic()

    def zero_witness_valid(self):
        """Check that zero belongs to the current endpoint and activation epoch."""
        if self.zero_witness is None:
            return False
        return self.zero_witness == (
            self.chain_epoch, tuple(sorted(self.activation_epochs.items())))

    def cm_enable_valid(self):
        """Bind a recent successful enable acknowledgement to the chain epoch."""
        if self.cm_enable_ack is None:
            return False
        epoch, activation_epoch, observed = self.cm_enable_ack
        return (epoch == self.chain_epoch
                and activation_epoch == self.activation_epochs[
                    'collision_monitor']
                and 0.0 <= self._monotonic() - observed
                <= CM_ENABLE_FRESHNESS_S)

    def on_observation(self, message):
        """Retain valid JSON without rewriting perception approval fields."""
        def reject_constant(value):
            raise ValueError(f'non_finite_json:{value}')

        try:
            document = json.loads(message.data, parse_constant=reject_constant)
            if not isinstance(document, dict):
                raise ValueError
            self.observation = document
            self.received['perception'] = self._monotonic()
        except (json.JSONDecodeError, TypeError, ValueError):
            self.observation = {}
            if self.armed:
                self.abort('invalid_perception_json')

    def on_odom(self, message):
        """Accept finite, increasing base odometry for capture-time interpolation."""
        self.received.pop('odom', None)
        if (message.header.frame_id != 'odom'
                or message.child_frame_id not in ('base_link', 'base_footprint')):
            return
        q = message.pose.pose.orientation
        if not math.isclose(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1., abs_tol=.01):
            return
        p, velocity = message.pose.pose.position, message.twist.twist
        pose = (p.x, p.y, math.atan2(
            2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z)))
        if not all(math.isfinite(v) for v in (*pose, velocity.linear.x,
                                              velocity.angular.z)):
            return
        stamp = stamp_seconds(message.header)
        if not 0.0 <= self.now_ros_s() - stamp <= .5:
            return
        if self.pose_history and stamp <= self.pose_history[-1][0]:
            if self.armed:
                self.abort('odometry_time_regressed')
            self.pose_history.clear()
        self.pose, self.velocity, self.odom_stamp_s = (
            pose, (velocity.linear.x, velocity.angular.z), stamp)
        if self.armed and self.travel_pose is not None:
            self.travel_m += math.hypot(
                pose[0] - self.travel_pose[0], pose[1] - self.travel_pose[1])
        self.travel_pose = pose
        self.pose_history.append((stamp, pose))
        self.received['odom'] = self._monotonic()

    def on_scan(self, message):
        """Record only fresh scans containing at least one usable return."""
        self.received.pop('scan', None)
        age = self.now_ros_s() - stamp_seconds(message.header)
        bounds_valid = (bool(message.header.frame_id)
                        and math.isfinite(message.range_min)
                        and math.isfinite(message.range_max)
                        and 0.0 <= message.range_min < message.range_max)
        usable = bounds_valid and any(math.isfinite(value)
                                      and message.range_min <= value
                                      <= message.range_max
                                      for value in message.ranges)
        if 0.0 <= age <= .5 and usable:
            self.received['scan'] = self._monotonic()

    def on_battery(self, message):
        """Keep the actual status and voltage without inferring DISCHARGING."""
        self.battery = message.power_supply_status, float(message.voltage)
        self.received['battery'] = self._monotonic()

    def on_collision(self, message):
        """Latch any collision-monitor intervention during execution."""
        self.collision_action = message.action_type
        self.received['collision'] = self._monotonic()
        if (self.armed and message.action_type not in (
                CollisionMonitorState.DO_NOTHING,
                CollisionMonitorState.SLOWDOWN)):
            self.abort('collision_monitor_intervention')

    def on_final_command(self, message):
        """Observe the final protected command for startup zero echo."""
        self.final_command = message.linear.x, message.angular.z
        now = self._monotonic()
        finite = all(math.isfinite(value) for value in self.final_command)
        self.received['command'] = now
        if (self.motion_sent and finite
                and self.first_motion_publish_mono_s is not None
                and now >= self.first_motion_publish_mono_s):
            self.first_motion_echo_mono_s = now
        if (finite
                and abs(self.final_command[0]) <= 1e-6
                and abs(self.final_command[1]) <= 1e-6):
            self.zero_witness = (
                self.chain_epoch,
                tuple(sorted(self.activation_epochs.items())))
        else:
            self.zero_witness = None

    def publish_zero(self):
        self.publish_command(0.0, 0.0)

    def publish_command(self, linear, angular):
        """Publish only bounded forward commands to the smoother input."""
        if (not 0.0 <= linear <= MAX_LINEAR_MPS
                or abs(angular) > MAX_ANGULAR_RADPS
                or not all(math.isfinite(v) for v in (linear, angular))):
            raise ValueError('unbounded_execution_command')
        message = Twist()
        message.linear.x, message.angular.z = float(linear), float(angular)
        now = self._monotonic()
        is_motion = abs(linear) > 0.0 or abs(angular) > 0.0
        if is_motion and not self.motion_sent:
            self.first_motion_publish_mono_s = now
            self.first_motion_echo_mono_s = None
            self.first_motion_chain_witness = (
                self.chain_epoch,
                tuple(sorted(self.activation_epochs.items())))
        self.command_pub.publish(message)
        self.last_publish_mono_s = now
        self.motion_sent |= is_motion

    def abort(self, reason):
        """Stop an owned run and require a new explicit start request."""
        if self.motion_sent or not ownership_reasons(self.owners()):
            self.publish_zero()
        self.armed, self.latched, self.reason = False, True, str(reason)
        self.terminal_state = 'ABORTED'
        self.policy.reset()

    def succeed(self, reason):
        """Publish terminal zero while preserving the successful outcome."""
        self.publish_zero()
        self.armed, self.latched, self.reason = False, True, str(reason)
        self.terminal_state = 'SUCCEEDED'
        self.policy.reset()

    def readiness(
            self, require_start=True, require_command_fresh=None,
            require_physical_validation=True):
        """Return every current start/runtime blocker."""
        self.refresh_chain_identity()
        reasons = readiness_reasons(
            self.observation, self.received,
            now_mono_s=self._monotonic(), now_ros_s=self.now_ros_s(),
            battery=self.battery, collision_action=self.collision_action,
            final_command=self.final_command, lifecycle=self.lifecycle,
            owners=self.owners(),
            charger_unplugged_confirmed=bool(
                self.get_parameter('charger_unplugged_confirmed').value),
            optical_frame=self.values['optical_frame'],
            odom_stamp_s=self.odom_stamp_s, odom_velocity=self.velocity,
            zero_witness_valid=self.zero_witness_valid(),
            require_start=require_start,
            require_command_fresh=require_command_fresh,
            graph_received_mono_s=self.graph_received_mono_s,
            cm_enable_valid=self.cm_enable_valid())
        if require_physical_validation:
            try:
                physical_validation(
                    self.values['physical_validation_file'],
                    self.values['camera_mount_file'], self.values['geometry_file'],
                    self.values['nav_params_file'],
                    self.values['parking_contract_file'])
            except (OSError, ValueError, yaml.YAMLError) as error:
                reasons.append(str(error))
        return reasons

    def arm(self, mode, validation_reference_m=None):
        """Initialize one run without weakening any readiness decision."""
        self.policy.reset()
        self.armed, self.latched = True, False
        self.terminal_state = None
        self.reason = 'ARMED'
        self.execution_mode = mode
        self.validation_reference_m = validation_reference_m
        self.start_pose = self.pose
        self.started_mono_s = self._monotonic()
        self.motion_sent = False
        self.first_motion_publish_mono_s = None
        self.first_motion_echo_mono_s = None
        self.first_motion_chain_witness = None
        self.travel_m = 0.0
        self.travel_pose = self.pose

    def on_start(self, request, response):
        """Start only after all independent evidence is simultaneously valid."""
        del request
        if self.armed:
            response.success, response.message = False, 'approach_already_armed'
            return response
        reasons = self.readiness(require_start=True)
        if reasons:
            response.success, response.message = False, reasons[0]
            return response
        self.arm('production')
        response.success, response.message = True, 'approach armed'
        return response

    def on_start_validation(self, request, response):
        """Consume one measured reference and arm a tightly bounded trial."""
        del request
        reference_m = self.get_parameter('validation_reference_m').value
        result = self.set_parameters([
            Parameter('validation_reference_m', value=0.0)])[0]
        if not result.successful:
            response.success, response.message = (
                False, 'validation_reference_consumption_failed')
            return response
        if self.armed:
            response.success, response.message = False, 'approach_already_armed'
            return response
        reason = validation_reference_reason(reference_m)
        if reason is not None:
            response.success, response.message = False, reason
            return response
        reasons = self.readiness(
            require_start=True, require_physical_validation=False)
        if reasons:
            response.success, response.message = False, reasons[0]
            return response
        if abs(self.observation['front_distance_m'] - reference_m) > (
                VALIDATION_REFERENCE_TOLERANCE_M):
            response.success, response.message = (
                False, 'validation_reference_disagreement')
            return response
        self.arm(VALIDATION_TRIAL_MODE, float(reference_m))
        response.success, response.message = True, 'validation trial armed'
        return response

    def on_cancel(self, request, response):
        """Latch a zero command; another start call is required."""
        del request
        self.abort('cancelled_by_operator')
        response.success, response.message = True, self.reason
        return response

    def poll_lifecycle(self):
        """Poll current lifecycle states; callback arrival time defines freshness."""
        for name, client in self.lifecycle_clients.items():
            pending = self.lifecycle_pending.get(name)
            if pending is not None and not pending[0].done():
                if self._monotonic() - pending[1] <= 3.0:
                    continue
                self.lifecycle_pending.pop(name, None)
            if not client.service_is_ready():
                if self.activation_epochs[name] > 0:
                    previous = self.lifecycle.get(name, (None, None))[0]
                    if previous is not None:
                        self.invalidate_chain_witness(name)
                self.lifecycle[name] = None, self._monotonic()
                continue
            future = client.call_async(GetState.Request())
            self.lifecycle_pending[name] = future, self._monotonic()
            future.add_done_callback(
                lambda result, component=name: self.lifecycle_result(
                    component, result))

    def poll_cm_enabled(self):
        """Request only enable=true and retain a short epoch-bound ack."""
        pending = self.cm_toggle_pending
        if pending is not None and not pending[0].done():
            if self._monotonic() - pending[1] <= CM_ENABLE_FRESHNESS_S:
                return
            self.cm_toggle_pending = None
            self.cm_enable_ack = None
        if not self.cm_toggle_client.service_is_ready():
            self.cm_enable_ack = None
            return
        request = Toggle.Request()
        request.enable = True
        future = self.cm_toggle_client.call_async(request)
        self.cm_toggle_pending = future, self._monotonic()
        future.add_done_callback(self.cm_enable_result)

    def cm_enable_result(self, future):
        """Accept only a successful reply to this node's enable=true request."""
        pending = self.cm_toggle_pending
        if pending is None or pending[0] is not future:
            return
        self.cm_toggle_pending = None
        try:
            response = future.result()
        except Exception:
            self.cm_enable_ack = None
            return
        if response is None or response.success is not True:
            self.cm_enable_ack = None
            return
        self.cm_enable_ack = (
            self.chain_epoch,
            self.activation_epochs['collision_monitor'],
            self._monotonic())

    def lifecycle_result(self, name, future):
        pending = self.lifecycle_pending.get(name)
        if pending is None or pending[0] is not future:
            return
        self.lifecycle_pending.pop(name, None)
        try:
            state = future.result().current_state.id
        except Exception:
            self.lifecycle.pop(name, None)
            return
        previous = self.lifecycle.get(name, (None, None))[0]
        if state != State.PRIMARY_STATE_ACTIVE:
            self.invalidate_chain_witness(name)
        elif self.activation_epochs[name] == 0:
            self.activation_epochs[name] += 1
            if self.zero_witness is not None:
                self.zero_witness = (
                    self.chain_epoch,
                    tuple(sorted(self.activation_epochs.items())))
        elif previous != State.PRIMARY_STATE_ACTIVE:
            self.activation_epochs[name] += 1
            self.invalidate_chain_witness(name)
        self.lifecycle[name] = state, self._monotonic()

    def publish_footprint(self):
        """Republish the configured padded footprint with a fresh self stamp."""
        message = PolygonStamped()
        message.header.frame_id = 'base_footprint'
        message.header.stamp = self.get_clock().now().to_msg()
        message.polygon.points = [Point32(x=x, y=y) for x, y in self.footprint]
        self.footprint_pub.publish(message)
        self.received['footprint'] = self._monotonic()

    def publish_status(self, blockers):
        state = ('ARMED' if self.armed else self.terminal_state if self.latched
                 else 'READY' if not blockers else 'BLOCKED')
        document = {
            'state': state,
            'mode': getattr(self, 'execution_mode', 'production'),
            'reason': self.reason, 'blockers': blockers,
            'chain_epoch': self.chain_epoch,
            'activation_epochs': dict(self.activation_epochs),
            'zero_witness_valid': self.zero_witness_valid(),
            'graph_age_s': (
                self._monotonic() - self.graph_received_mono_s
                if math.isfinite(self.graph_received_mono_s) else None),
            'collision_monitor_enable_confirmed': self.cm_enable_valid(),
            'collision_monitor_state': (
                'ACTIVE_EVENT_UNSEEN' if self.collision_action is None else {
                    CollisionMonitorState.DO_NOTHING: 'DO_NOTHING',
                    CollisionMonitorState.STOP: 'STOP',
                    CollisionMonitorState.SLOWDOWN: 'SLOWDOWN',
                    CollisionMonitorState.APPROACH: 'APPROACH',
                    CollisionMonitorState.LIMIT: 'LIMIT',
                }.get(self.collision_action, 'UNKNOWN')),
            'first_motion_echo_observed': (
                self.first_motion_echo_mono_s is not None),
            'last_publish_monotonic_s': (
                self.last_publish_mono_s
                if math.isfinite(self.last_publish_mono_s) else None),
            'footprint_age_s': self._monotonic() - self.received.get(
                'footprint', -math.inf),
        }
        self.status_pub.publish(String(data=json.dumps(document, allow_nan=False)))

    def tick(self):
        """Publish footprint/zero heartbeat or one fully guarded policy command."""
        self.publish_footprint()
        waiting_for_first_echo = (
            self.armed and self.motion_sent
            and self.first_motion_echo_mono_s is None)
        if waiting_for_first_echo:
            current_witness = (
                self.chain_epoch,
                tuple(sorted(self.activation_epochs.items())))
            if current_witness != self.first_motion_chain_witness:
                blockers = ['protection_chain_changed_before_first_echo']
                self.abort(blockers[0])
                self.publish_status(blockers)
                return
            if (self._monotonic() - self.first_motion_publish_mono_s > .5):
                blockers = ['first_motion_echo_timeout']
                self.abort(blockers[0])
                self.publish_status(blockers)
                return
        readiness_arguments = {
            'require_start': (not self.armed or not self.motion_sent),
            'require_command_fresh': (
                self.armed and self.motion_sent and not waiting_for_first_echo),
        }
        trial = (self.armed and getattr(
            self, 'execution_mode', 'production') == VALIDATION_TRIAL_MODE)
        if trial:
            readiness_arguments['require_physical_validation'] = False
        blockers = self.readiness(**readiness_arguments)
        if not self.armed:
            if not ownership_reasons(self.owners()):
                self.publish_zero()
            self.publish_status(blockers)
            return
        if blockers:
            self.abort(blockers[0])
            self.publish_status(blockers)
            return
        max_run_s = VALIDATION_TRIAL_MAX_RUN_S if trial else MAX_RUN_S
        max_travel_m = (
            VALIDATION_TRIAL_MAX_TRAVEL_M if trial else MAX_TRAVEL_M)
        if (self._monotonic() - self.started_mono_s > max_run_s
                or self.travel_m > max_travel_m):
            reason = ('validation_trial_limit_reached' if trial
                      else 'execution_limit_reached')
            self.abort(reason)
            self.publish_status([reason])
            return
        try:
            source_pose = pose_at(
                self.pose_history, float(self.observation['stamp_s']))
        except (KeyError, TypeError, ValueError) as error:
            self.abort(str(error))
            self.publish_status([str(error)])
            return
        result = self.policy.step(
            self.observation, self.pose, now_s=self.now_ros_s(),
            odom_stamp_s=self.odom_stamp_s,
            linear_mps=self.velocity[0], angular_radps=self.velocity[1],
            cmd_linear_mps=self.final_command[0],
            cmd_angular_radps=self.final_command[1],
            observation_pose=source_pose)
        if result['state'] == 'SUCCEEDED':
            self.succeed('trial_completed' if trial else result['reason'])
        elif result['state'] == 'ABORTED':
            self.abort(result['reason'])
        else:
            linear = result['linear_mps']
            angular = result['angular_radps']
            if (trial and linear >= 0.0
                    and all(math.isfinite(value) for value in (linear, angular))):
                scale = min(
                    1.0,
                    (VALIDATION_TRIAL_MAX_LINEAR_MPS / linear
                     if linear > 0.0 else 1.0),
                    (VALIDATION_TRIAL_MAX_ANGULAR_RADPS / abs(angular)
                     if angular != 0.0 else 1.0),
                )
                linear *= scale
                angular *= scale
            self.publish_command(linear, angular)
            self.reason = result['reason']
        self.publish_status([])


def shutdown_zero_drain(node):
    """Keep the context alive while a bounded 20 Hz terminal zero drains."""
    node.armed = False
    if not node.motion_sent:
        return 0
    deadline = time.monotonic() + SHUTDOWN_ZERO_DRAIN_S
    count = 0
    while rclpy.ok(context=node.context) and time.monotonic() < deadline:
        node.publish_zero()
        count += 1
        remaining = deadline - time.monotonic()
        if remaining > 0.0:
            rclpy.spin_once(
                node, timeout_sec=min(SHUTDOWN_ZERO_PERIOD_S, remaining))
    return count


def _interrupt_main(signum, frame):
    del signum, frame
    raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_handlers = {
        signum: signal.signal(signum, _interrupt_main)
        for signum in (signal.SIGINT, signal.SIGTERM)}
    node = BoxApproachExecution()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        try:
            shutdown_zero_drain(node)
        except Exception:
            pass
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
