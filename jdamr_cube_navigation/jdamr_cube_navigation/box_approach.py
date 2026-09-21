"""Bounded planar box approach policy; no ROS or actuator side effects."""

from dataclasses import dataclass
import math

from jdamr_cube_navigation.parking import ParkingHold


def wrap(angle_rad):
    """Return a signed shortest angle."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


@dataclass(frozen=True)
class ApproachConfig:
    """Candidate control bounds, not measured hardware performance."""

    standoff_m: float = 0.45
    minimum_clearance_m: float = 0.38
    position_tolerance_m: float = 0.02
    max_linear_mps: float = 0.06
    max_angular_radps: float = 0.20
    max_age_s: float = 0.5
    timeout_s: float = 90.0
    target_jump_m: float = 0.10
    target_jump_rad: float = 0.17
    min_confidence: float = 0.8

    def __post_init__(self):
        """Reject invalid gains and targets inside the unobserved near range."""
        if not all(type(v) in (int, float) and math.isfinite(v) and v > 0
                   for v in vars(self).values()):
            raise ValueError('approach bounds must be finite and positive')
        if (self.standoff_m - self.position_tolerance_m
                <= self.minimum_clearance_m or self.min_confidence > 1):
            raise ValueError('invalid standoff or confidence')
        ceilings = {'position_tolerance_m': .02, 'max_linear_mps': .06,
                    'max_angular_radps': .20, 'max_age_s': .5, 'timeout_s': 90.,
                    'target_jump_m': .10, 'target_jump_rad': .17}
        if (any(getattr(self, key) > limit for key, limit in ceilings.items())
                or self.standoff_m < .45 or self.minimum_clearance_m < .38
                or self.min_confidence < .8):
            raise ValueError('outside the tested approach envelope')


def relative_target(observation, camera_pose, body_bounds, config):
    """Convert an optical front-face observation to a base-frame goal.

    Camera pose is planar (x, y, yaw) in base coordinates. The face normal
    points toward the camera. Body bounds are rear/front/right/left in metres.
    This is a nominal planar model, not proof of camera calibration.
    """
    values = [observation[k] for k in (
        'front_distance_m', 'lateral_error_m', 'edge_angle_deg', 'confidence')]
    values += list(camera_pose) + list(body_bounds)
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
               and math.isfinite(v) for v in values):
        raise ValueError('non_finite_observation_or_geometry')
    if (observation.get('detected') is not True
            or observation.get('surface_kind') != 'front'
            or observation['confidence'] < config.min_confidence
            or not 0 < observation['front_distance_m'] <= 2.0):
        raise ValueError('front_face_not_usable')
    rear, front, right, left = body_bounds
    if not rear < front or not right < left:
        raise ValueError('invalid_body_bounds')
    cx, cy, yaw = camera_pose
    depth, optical_x = values[:2]
    c, s = math.cos(yaw), math.sin(yaw)
    center_x = cx + c * depth + s * optical_x
    center_y = cy + s * depth - c * optical_x
    heading = wrap(yaw + math.radians(observation['edge_angle_deg']))
    if abs(wrap(heading - yaw)) > math.pi / 4:
        raise ValueError('face_angle_outside_capture_envelope')
    nx, ny = math.cos(heading), math.sin(heading)
    goal_x = center_x - (config.standoff_m + front) * nx
    goal_y = center_y - (config.standoff_m + front) * ny
    support = max(x * nx + y * ny
                  for x in (rear, front) for y in (right, left))
    clearance_m = center_x * nx + center_y * ny - support
    return (goal_x, goal_y, heading), clearance_m


def to_world(relative, pose):
    """Compose two planar poses."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * relative[0] - s * relative[1],
            y + s * relative[0] + c * relative[1],
            wrap(yaw + relative[2]))


def to_relative(goal, pose):
    """Express a fixed goal in the current base frame."""
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap(goal[2] - pose[2])


class BoxApproach:
    """Track one acquired face and latch faults until explicit reset."""

    def __init__(self, contract, camera_pose, body_bounds, config=None):
        """Reuse the existing stopped-pose hold contract."""
        self.config = config or ApproachConfig()
        self.camera_pose, self.body_bounds = camera_pose, body_bounds
        self.contract = contract
        self.reset()

    def reset(self):
        """Discard the old target after relocation or an aborted attempt."""
        self.goal = None
        self.started_s = None
        self.last_stamp_s = None
        self.hold = ParkingHold(self.contract)
        self.state = 'WAITING'
        self.reason = 'acquiring_stable_face'

    def result(self, state, reason, linear_mps=0.0, angular_radps=0.0):
        """Return a bounded proposal; the caller owns motion authorization."""
        self.state, self.reason = state, reason
        return {'state': state, 'reason': reason, 'linear_mps': linear_mps,
                'angular_radps': angular_radps, 'goal_odom': self.goal,
                'physical_accuracy': 'NOT_MEASURED'}

    def reject(self, reason):
        """Stop immediately and latch faults once an approach has started."""
        return self.result('ABORTED' if self.goal else 'WAITING', reason)

    def step(self, observation, pose, *, now_s, odom_stamp_s,
             linear_mps=0.0, angular_radps=0.0,
             cmd_linear_mps=0.0, cmd_angular_radps=0.0, blockers=(),
             observation_pose=None):
        """Calculate one forward-only step from fresh sensors and interlocks."""
        if self.state in {'ABORTED', 'SUCCEEDED'}:
            return self.result(self.state, self.reason)
        if blockers:
            return self.reject(str(blockers[0]))
        try:
            stamp_s = float(observation['stamp_s'])
            source_pose = pose if observation_pose is None else observation_pose
            numbers = (now_s, odom_stamp_s, stamp_s, *pose, *source_pose,
                       linear_mps, angular_radps, cmd_linear_mps,
                       cmd_angular_radps)
            if (not all(math.isfinite(v) for v in numbers)
                    or len(pose) != 3 or len(source_pose) != 3):
                raise ValueError('non_finite_state')
            if any(not 0 <= now_s - t <= self.config.max_age_s
                   for t in (stamp_s, odom_stamp_s)):
                raise ValueError('stale_or_future_observation')
            if self.last_stamp_s is not None and stamp_s < self.last_stamp_s:
                raise ValueError('observation_time_regressed')
            relative, clearance = relative_target(
                observation, self.camera_pose, self.body_bounds, self.config)
        except (KeyError, TypeError, ValueError) as error:
            return self.reject(str(error))
        if clearance < self.config.minimum_clearance_m:
            return self.reject('near_range_or_body_clearance_limit')
        candidate = to_world(relative, source_pose)
        if self.goal is None:
            if observation.get('stable') is not True:
                return self.result('WAITING', 'acquiring_stable_face')
            self.goal, self.started_s = candidate, now_s
        delta = to_relative(candidate, self.goal)
        if (math.hypot(*delta[:2]) > self.config.target_jump_m
                or abs(delta[2]) > self.config.target_jump_rad):
            return self.reject('target_changed_or_odometry_discontinuity')
        if now_s - self.started_s > self.config.timeout_s:
            return self.reject('approach_timeout')
        x_m, y_m, yaw_rad = to_relative(candidate, pose)
        nx, ny = math.cos(yaw_rad), math.sin(yaw_rad)
        rear, front, right, left = self.body_bounds
        support = max(x * nx + y * ny
                      for x in (rear, front) for y in (right, left))
        current_clearance = (x_m * nx + y_m * ny
                             + self.config.standoff_m + front - support)
        if current_clearance < self.config.minimum_clearance_m:
            return self.reject('current_body_clearance_limit')
        distance_m = math.hypot(x_m, y_m)
        fresh = self.last_stamp_s != stamp_s
        self.last_stamp_s = stamp_s
        if (distance_m > self.config.position_tolerance_m
                or abs(yaw_rad) > self.contract['yaw_tolerance_rad']):
            self.hold = ParkingHold(self.contract)
        if distance_m <= self.config.position_tolerance_m:
            if abs(yaw_rad) > self.contract['yaw_tolerance_rad']:
                return self.result('ALIGN_FINAL', 'align_face_normal',
                                   angular_radps=self.clip_yaw(yaw_rad))
            if fresh:
                hold = self.hold.observe(
                    stamp_s, candidate, pose, linear_mps, angular_radps,
                    cmd_linear_mps, cmd_angular_radps, now_s - stamp_s)
                if hold['confirmed']:
                    return self.result('SUCCEEDED', 'observed_pose_held')
            return self.result('HOLD', 'waiting_for_zero_velocity_hold')
        # A failed arrival cannot be repaired by reversing into an unseen area.
        alpha = math.atan2(y_m, x_m)
        if abs(alpha) >= math.pi / 2:
            return self.reject('goal_behind_reposition_required')
        if abs(alpha) > math.pi / 4:
            return self.result('ALIGN', 'align_approach_bearing',
                               angular_radps=self.clip_yaw(alpha))
        beta = wrap(yaw_rad - alpha)
        # Gains map distance [m] to speed [m/s] and angles [rad] to [rad/s].
        speed = min(self.config.max_linear_mps, 0.4 * distance_m)
        return self.result('APPROACH', 'curve_toward_face',
                           speed * math.cos(alpha), self.clip_yaw(alpha - 0.4 * beta))

    def clip_yaw(self, value):
        """Bound angular proposals in radians per second."""
        limit = self.config.max_angular_radps
        return max(-limit, min(limit, value))
