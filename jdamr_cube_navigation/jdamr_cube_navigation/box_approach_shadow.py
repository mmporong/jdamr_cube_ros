"""Evaluate box-approach proposals without any velocity publisher."""

from collections import deque
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from jdamr_cube_navigation.box_approach import ApproachConfig, BoxApproach, relative_target, wrap
from jdamr_cube_navigation.depth_obstacle_filter import body_bounds_from_geometry
from jdamr_cube_navigation.parking import load_parking_contract
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


def stamp_seconds(header):
    """Read ROS time at the input boundary."""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def guard_reasons(observation, received, *, now_s, battery_status, collision_action,
                  final_owners, input_owners, optical_frame='camera_color_optical_frame'):
    """Fail closed on unavailable interlocks; this does not certify free space."""
    reasons = []
    if observation.get('control_ready') is not True:
        reasons.append('perception_not_approved')
    if observation.get('frame_id') != optical_frame:
        reasons.append('perception_frame_mismatch')
    for name in ('perception', 'odom', 'scan', 'battery', 'collision', 'command'):
        limit_s = 3. if name in ('battery', 'collision') else .5
        age_s = now_s - received.get(name, -math.inf)
        if not 0 <= age_s <= limit_s:
            reasons.append(name + '_stale')
    if battery_status != BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
        reasons.append('charging_or_unknown')
    if collision_action != CollisionMonitorState.DO_NOTHING:
        reasons.append('collision_not_clear')
    if final_owners != [('collision_monitor', '/')] or input_owners:
        reasons.append('command_owner_conflict')
    return reasons


def pose_at(history, stamp_s, max_gap_s=.5):
    """Interpolate a bracketed pose at image time without extrapolation."""
    for (ta, a), (tb, b) in zip(history, list(history)[1:]):
        if ta <= stamp_s <= tb and 0 < tb - ta <= max_gap_s:
            ratio = (stamp_s - ta) / (tb - ta)
            return (a[0] + ratio * (b[0] - a[0]),
                    a[1] + ratio * (b[1] - a[1]),
                    wrap(a[2] + ratio * wrap(b[2] - a[2])))
    raise ValueError('image_time_odometry_unavailable')


def lower_base_geometry(path):
    """Keep lower-base provenance explicit; do not claim arm or payload clearance."""
    scope = yaml.safe_load(Path(path).read_text()).get('claim_scope')
    if scope != 'installed_lower_base_without_arms_or_payload':
        raise ValueError('unsupported_or_missing_geometry_scope')
    return body_bounds_from_geometry(path), scope


class BoxApproachShadow(Node):
    """Publish JSON only, even when a hypothetical approach would be allowed."""

    def __init__(self, **kwargs):
        """Load geometry from existing provenance and subscribe without actuation."""
        super().__init__('box_approach_shadow', **kwargs)
        nav = Path(get_package_share_directory('jdamr_cube_navigation'))
        vslam = Path(get_package_share_directory('jdamr_cube_vslam'))
        description = Path(get_package_share_directory('jdamr_cube_description'))
        defaults = {
            'camera_mount_file': str(vslam / 'config/camera_mount.yaml'),
            'geometry_file': str(description / 'config/new_base_geometry.yaml'),
            'parking_contract_file': str(nav / 'config/parking_contract.yaml'),
            'optical_frame': 'camera_color_optical_frame',
        }
        defaults.update(vars(ApproachConfig()))
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        mount = yaml.safe_load(Path(self.get_parameter(
            'camera_mount_file').value).read_text())['camera_mount']
        transform = mount['transform']
        if (mount['parent_frame'] != 'base_link'
                or mount['child_frame'] != 'camera_link'
                or any(abs(transform[k]) > 1e-9 for k in ('roll_rad', 'pitch_rad'))):
            raise ValueError('shadow requires nominal level base_link camera geometry')
        camera = tuple(transform[k] for k in ('x_m', 'y_m', 'yaw_rad'))
        body, self.envelope_scope = lower_base_geometry(
            self.get_parameter('geometry_file').value)
        contract = load_parking_contract(Path(self.get_parameter('parking_contract_file').value))
        config = ApproachConfig(**{k: self.get_parameter(k).value
                                   for k in vars(ApproachConfig())})
        self.policy = BoxApproach(contract, camera, body, config)
        self.guarded_policy = BoxApproach(contract, camera, body, config)
        self.observation, self.received = {}, {}
        self.pose_history = deque(maxlen=256)
        self.pose, self.velocity, self.command = (0., 0., 0.), (0., 0.), (0., 0.)
        self.odom_stamp_s = -math.inf
        self.battery_status, self.collision_action = None, None
        self.status = self.create_publisher(String, '/box_parking/approach_status', 10)
        self.create_subscription(String, '/box_parking/perception_status', self.on_observation, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/battery_state', self.on_battery,
                                 qos_profile_sensor_data)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                 self.on_collision, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_service(Trigger, '/box_parking/reset_shadow', self.on_reset)
        self.create_timer(.1, self.tick)

    def on_observation(self, message):
        """Invalidate malformed input instead of retaining a last good target."""
        try:
            document = json.loads(message.data)
            if not isinstance(document, dict):
                raise ValueError('not an object')
            self.observation = document
        except (ValueError, TypeError):
            self.observation = {}
        self.received['perception'] = time.monotonic()

    def on_odom(self, message):
        """Require the expected wheel reference frame and a valid quaternion."""
        self.received.pop('odom', None)
        if (message.header.frame_id != 'odom'
                or message.child_frame_id not in ('base_link', 'base_footprint')):
            return
        q = message.pose.pose.orientation
        if not math.isclose(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1., abs_tol=.01):
            return
        p, velocity = message.pose.pose.position, message.twist.twist
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        self.pose = (p.x, p.y, yaw)
        self.velocity = velocity.linear.x, velocity.angular.z
        self.odom_stamp_s = stamp_seconds(message.header)
        if self.pose_history and self.odom_stamp_s <= self.pose_history[-1][0]:
            self.pose_history.clear()
            self.policy.reject('odometry_time_regressed')
            self.guarded_policy.reject('odometry_time_regressed')
        self.pose_history.append((self.odom_stamp_s, self.pose))
        self.received['odom'] = time.monotonic()

    def on_scan(self, message):
        """Require a fresh source stamp and usable returns, not just callbacks."""
        self.received.pop('scan', None)
        age_s = self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(message.header)
        usable = any(math.isfinite(r) and message.range_min <= r <= message.range_max
                     for r in message.ranges)
        if 0 <= age_s <= .5 and usable:
            self.received['scan'] = time.monotonic()

    def on_battery(self, message):
        """Treat unknown charging state as a blocker."""
        self.battery_status = message.power_supply_status
        self.received['battery'] = time.monotonic()

    def on_collision(self, message):
        """Observe existing collision supervision without reconfiguring it."""
        self.collision_action = message.action_type
        self.received['collision'] = time.monotonic()

    def on_command(self, message):
        """Observe final commands for the stationary hold check."""
        self.command = message.linear.x, message.angular.z
        self.received['command'] = time.monotonic()

    def on_reset(self, request, response):
        """Forget a target after relocation; this never starts hardware."""
        del request
        self.policy.reset()
        self.guarded_policy.reset()
        self.pose_history.clear()
        self.observation = {}
        self.received.pop('perception', None)
        response.success, response.message = True, 'shadow target cleared; no motion output'
        return response

    def tick(self):
        """Emit geometric proposals, explicit blockers, and zero actuator authority."""
        def owners(topic):
            return [(i.node_name, i.node_namespace)
                    for i in self.get_publishers_info_by_topic(topic)]
        blockers = guard_reasons(
            self.observation, self.received, now_s=time.monotonic(),
            battery_status=self.battery_status, collision_action=self.collision_action,
            final_owners=owners('/cmd_vel'), input_owners=owners('/cmd_vel_smoothed'),
            optical_frame=self.get_parameter('optical_frame').value)
        source_pose = None
        try:
            source_pose = pose_at(self.pose_history, float(self.observation['stamp_s']))
        except (KeyError, TypeError, ValueError) as error:
            blockers.append(str(error))
        now = self.get_clock().now().nanoseconds * 1e-9
        preview_blockers = [reason for reason in blockers
                            if reason in ('perception_stale', 'odom_stale',
                                          'perception_frame_mismatch')]
        if source_pose is None:
            preview_blockers.append('image_time_odometry_unavailable')
        result = self.policy.step(
            self.observation, self.pose, now_s=now, odom_stamp_s=self.odom_stamp_s,
            linear_mps=self.velocity[0], angular_radps=self.velocity[1],
            cmd_linear_mps=self.command[0], cmd_angular_radps=self.command[1],
            blockers=preview_blockers, observation_pose=source_pose)
        # Preview readiness is not physical approval. Only JSON is published;
        # preserve all physical blockers and never advertise executable motion.
        proposal = {key: result.pop(key) for key in ('linear_mps', 'angular_radps')}
        result['proposal_only'] = proposal
        guarded = self.guarded_policy.step(
            self.observation, self.pose, now_s=now, odom_stamp_s=self.odom_stamp_s,
            linear_mps=self.velocity[0], angular_radps=self.velocity[1],
            cmd_linear_mps=self.command[0], cmd_angular_radps=self.command[1],
            blockers=blockers, observation_pose=source_pose)
        result['guarded_proposal'] = {
            key: guarded[key] for key in ('linear_mps', 'angular_radps')}
        result['guarded_state'] = guarded['state']
        result['guarded_reason'] = guarded['reason']
        result.update(mode='SHADOW_ONLY', motion_output_enabled=False, blockers=blockers,
                      envelope_scope=self.envelope_scope,
                      clearance_reference='lower_base_front_not_arm_or_payload')
        try:
            goal, clearance = relative_target(self.observation, self.policy.camera_pose,
                                              self.policy.body_bounds, self.policy.config)
            age = now - float(self.observation['stamp_s'])
            if not 0 <= age <= self.policy.config.max_age_s:
                raise ValueError('stale geometry')
            result.update(candidate_goal_base=goal, observed_clearance_m=clearance,
                          geometry_status='NOMINAL_EXTRINSICS_NOT_APPROVED')
        except (KeyError, TypeError, ValueError):
            result.update(candidate_goal_base=None, observed_clearance_m=None)
        message = String()
        message.data = json.dumps(result, allow_nan=False)
        self.status.publish(message)


def main(args=None):
    """Run only the observation adapter, without a driver or navigator."""
    rclpy.init(args=args)
    node = BoxApproachShadow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
