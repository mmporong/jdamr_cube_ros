"""Approach a mapped table region, then park using a fresh observed box face."""

import argparse
from collections import deque
from itertools import islice
import json
import math
from pathlib import Path
import signal
import time

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.box_docking_target import (
    _validated_mount, compute_box_docking_target, MINIMUM_DISTANCE_M,
)
from jdamr_cube_navigation.box_lidar_witness import witness_box_face_with_lidar
from jdamr_cube_navigation.corridor_route import load_route, TRANSIT_PLAN_END_TOLERANCE_M
from jdamr_cube_navigation.docking_stop_profile import apply_docking_stop_profile
from jdamr_cube_navigation.parking import (
    load_parking_contract, MAXIMUM_CONTRACT_VALUES, pose_errors,
)
from jdamr_cube_navigation.restaurant_service import ServiceRoute
from jdamr_cube_navigation.reverse_parking import ALREADY_AT_GOAL_DISTANCE_M
from jdamr_cube_navigation.service_destinations import load_registry, verify_identity
from nav2_msgs.action import Spin
import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo
import yaml


SEARCH_STEP_RAD = math.radians(30.0)
SEARCH_MAX_STEPS = 12
SEARCH_MAX_CUMULATIVE_RAD = math.tau
SEARCH_RECOVERABLE_REASONS = frozenset({
    'stable detected front surface is required',
    'LiDAR face support is below five points',
    'LiDAR face tangent spread is too narrow',
    'LiDAR face line residual is too large',
    'LiDAR and depth face normals disagree',
    'LiDAR and depth face distances disagree',
    'observed_box_outside_selected_table_region',
})
# Observer statuses without a frame: the depth input itself was late.
OBSERVER_LATENCY_REASONS = frozenset({'stale_or_future_depth', 'camera_info_missing'})
# Derived from recorded status timing; see evaluation/
# depth_box_parking_provenance.yaml observer_status_timing_20260929 (test_t30).
OBSERVER_SILENT_S = 1.5
# Box-side rejections that describe input age, not the observed scene.
BOX_INPUT_LATENCY_FAILURES = frozenset({
    'box observation is stale or future-dated',
    'LiDAR scan is stale or future-dated',
    'fresh LiDAR scan is required for box witness',
})
# Face-to-base_link distance at which an in-place turn clears the face: the
# collision monitor rotation polygon radius plus margin (pinned by test_t29).
ESCAPE_CLEARANCE_M = 0.565
# Chassis-front band excluded from escape validation: one diagonal costmap
# cell plus capture noise, rounded up to path steps (recomputed by test_t29).
ESCAPE_VALIDATION_EXCLUDE_M = 0.10
# The escape is an intermediate motion judged by the alignment goal checker.
ESCAPE_PATH_CONTRACT = {
    'xy_tolerance_m': MAXIMUM_CONTRACT_VALUES['xy_tolerance_m'],
    'yaw_tolerance_rad': math.radians(MAXIMUM_CONTRACT_VALUES['yaw_tolerance_deg']),
}


class BoxObservationUnavailable(RuntimeError):
    """
    Describe whether changing the camera heading may recover observation.

    latency_limited marks a verdict formed from too few fresh frames after
    an input delay; it may be re-observed in place but never rotates.
    """

    def __init__(self, reason, *, retryable, latency_limited=False):
        super().__init__(f'box observation unavailable: {reason}')
        self.reason = reason
        self.retryable = retryable
        self.latency_limited = latency_limited


class BoxSearchBudget:
    """Issue at most one full turn as fixed, auditable search increments."""

    def __init__(self):
        self.steps_used = 0
        self.cumulative_yaw_rad = 0.0
        self.reposition_attempted = False

    def next_rotation(self):
        """Reserve the next bounded rotation, or return None when exhausted."""
        if self.steps_used >= SEARCH_MAX_STEPS:
            return None
        remaining = SEARCH_MAX_CUMULATIVE_RAD - self.cumulative_yaw_rad
        if remaining + 1e-12 < SEARCH_STEP_RAD:
            return None
        self.steps_used += 1
        self.cumulative_yaw_rad += SEARCH_STEP_RAD
        return SEARCH_STEP_RAD


class BoxServiceRoute(ServiceRoute):
    """Keep every movement on the existing Nav2 and collision-monitor path."""

    def __init__(self, *args, **kwargs):
        self.last_scan = None
        self.box_status_history = deque(maxlen=16)
        self.last_box_face = None
        self.depth_camera_info = None
        self.depth_info_subscription = None
        # The same observer configuration decides how many frames make a
        # stable face; an unreadable value leaves every latency verdict limited.
        try:
            observer = yaml.safe_load((
                Path(get_package_share_directory('jdamr_cube_navigation'))
                / 'config/depth_box_parking.yaml').read_text(encoding='utf-8'))
            stable_frames = observer['jdamr_depth_box_parking']['ros__parameters'][
                'stable_frames']
        except (KeyError, OSError, TypeError, yaml.YAMLError):
            stable_frames = None
        valid = (isinstance(stable_frames, int) and not isinstance(stable_frames, bool)
                 and stable_frames >= 1)
        self.observer_stable_frames = stable_frames if valid else None
        super().__init__(*args, **kwargs)
        self.precision_parameters = AsyncParameterClient(
            self, 'collision_monitor')

    def _scan_callback(self, message):
        super()._scan_callback(message)
        self.last_scan = message

    def _box_callback(self, message):
        super()._box_callback(message)
        # Recent statuses are failure evidence only, never an observation.
        history = getattr(self, 'box_status_history', None)
        if history is None:
            history = self.box_status_history = deque(maxlen=16)
        history.append((time.monotonic(), self.box_status))

    def _depth_info_callback(self, message):
        if getattr(self, 'depth_camera_info', None) is None:
            self.depth_camera_info = message

    def _evidence_emit_failed(self, source, error):
        """Report a diagnostic failure without changing the control flow."""
        try:
            self.emit('evidence_emit_failed', source=source,
                      error_type=type(error).__name__)
        except Exception:
            pass

    def _ensure_depth_info(self):
        """Subscribe to depth camera_info once; release it after one message."""
        try:
            subscription = getattr(self, 'depth_info_subscription', None)
            if getattr(self, 'depth_camera_info', None) is not None:
                if subscription is not None:
                    # Released here, outside the subscription's own callback.
                    self.destroy_subscription(subscription)
                    self.depth_info_subscription = None
            elif subscription is None:
                self.depth_info_subscription = self.create_subscription(
                    CameraInfo, '/camera/depth/camera_info',
                    self._depth_info_callback, qos_profile_sensor_data)
        except Exception as error:
            self._evidence_emit_failed('depth_camera_info', error)

    def _observation_pose_diagnostic(self, robot_pose, camera_mount, region_xy):
        """Record the camera bearing to the table region; never aim or judge."""
        try:
            self._ensure_depth_info()
            info = getattr(self, 'depth_camera_info', None)
            half_fov_rad = None
            if info is not None:
                fx, width = float(info.p[0]), float(info.width)
                if math.isfinite(fx) and fx > 0.0 and width > 0.0:
                    half_fov_rad = math.atan2(width / 2.0, fx)
            fields = {'robot_pose': list(robot_pose), 'region_xy': list(region_xy),
                      'half_fov_rad': half_fov_rad, 'depth_minimum_m': MINIMUM_DISTANCE_M}
            try:
                mount_x, mount_y, _z, _roll, _pitch, mount_yaw = _validated_mount(
                    camera_mount)
            except ValueError as error:
                fields['mount_invalid'] = str(error)
            else:
                x, y, yaw = robot_pose
                camera_xy = (x + mount_x * math.cos(yaw) - mount_y * math.sin(yaw),
                             y + mount_x * math.sin(yaw) + mount_y * math.cos(yaw))
                camera_yaw = yaw + mount_yaw
                bearing = math.atan2(region_xy[1] - camera_xy[1],
                                     region_xy[0] - camera_xy[0])
                bearing_error = math.atan2(math.sin(bearing - camera_yaw),
                                           math.cos(bearing - camera_yaw))
                distance_m = math.dist(camera_xy, region_xy)
                fields.update(
                    camera_xy=list(camera_xy), camera_yaw_rad=camera_yaw,
                    bearing_error_rad=bearing_error, camera_region_distance_m=distance_m,
                    in_fov=(None if half_fov_rad is None
                            else abs(bearing_error) <= half_fov_rad),
                    region_inside_depth_minimum=distance_m < MINIMUM_DISTANCE_M)
            self.emit('box_observation_pose_diagnostic', **self._json_evidence(fields))
        except Exception as error:
            self._evidence_emit_failed('box_observation_pose_diagnostic', error)

    def _emit_observation_unavailable_evidence(self, **fields):
        """Record the inputs behind an unavailable observation, once per window."""
        try:
            scan = getattr(self, 'last_scan', None)
            scan_document = None
            if scan is not None:
                ranges, truncated = self._json_scan_ranges(scan.ranges)
                scan_document = {
                    'frame_id': scan.header.frame_id,
                    'stamp_s': scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9,
                    'angle_min': self._json_scalar(scan.angle_min),
                    'angle_increment': self._json_scalar(scan.angle_increment),
                    'range_min': self._json_scalar(scan.range_min),
                    'range_max': self._json_scalar(scan.range_max),
                    'ranges': ranges, 'ranges_truncated': truncated,
                }
            history = list(getattr(self, 'box_status_history', None) or ())
            self.emit('box_observation_unavailable_evidence', **self._json_evidence({
                **fields, 'status_history': history, 'scan': scan_document}))
        except Exception as error:
            self._evidence_emit_failed('box_observation_unavailable_evidence', error)

    def preflight(self):
        """Also require the transit plan to end at the observation waypoint."""
        self.preflight_end_error_m = None
        if not super().preflight():
            return False
        goal = self.waypoints[-1]
        end_error_m = math.dist(self.preflight_path_end_xy, (goal['x'], goal['y']))
        if not math.isfinite(end_error_m) or end_error_m > TRANSIT_PLAN_END_TOLERANCE_M:
            self.preflight_end_error_m = end_error_m if math.isfinite(end_error_m) else None
            self.get_logger().error(
                'route preflight ends short of the observation waypoint: '
                f'end_error={end_error_m:.3f}m limit={TRANSIT_PLAN_END_TOLERANCE_M:.3f}m')
            return False
        return True

    @staticmethod
    def _json_scan_ranges(ranges, limit=9000):
        """Bound one raw scan and replace non-finite values for JSON evidence."""
        values = list(islice(iter(ranges), limit + 1))
        return [
            (float(value) if not isinstance(value, bool)
             and isinstance(value, (int, float)) and math.isfinite(value)
             else None)
            for value in values[:limit]
        ], len(values) > limit

    @staticmethod
    def _json_scalar(value):
        """Return one finite numeric scalar suitable for strict JSON."""
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)):
            return None
        return float(value)

    @classmethod
    def _json_evidence(cls, value):
        """Normalize nested evidence without changing its measured content."""
        if value is None or isinstance(value, (str, bool, int)):
            return value
        if isinstance(value, float):
            return value if math.isfinite(value) else None
        if isinstance(value, dict):
            return {str(key): cls._json_evidence(item)
                    for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [cls._json_evidence(item) for item in value]
        isoformat = getattr(value, 'isoformat', None)
        if callable(isoformat):
            return isoformat()
        return str(value)

    @staticmethod
    def _polygon(value):
        try:
            polygon = json.loads(value) if isinstance(value, str) else value
        except (TypeError, json.JSONDecodeError) as error:
            raise ValueError('collision polygon is not parseable') from error
        if (not isinstance(polygon, (list, tuple)) or len(polygon) < 3
                or any(not isinstance(point, (list, tuple)) or len(point) != 2
                       or any(isinstance(item, bool)
                              or not isinstance(item, (int, float))
                              or not math.isfinite(item) for item in point)
                       for point in polygon)):
            raise ValueError('collision polygon is invalid')
        return [[float(item) for item in point] for point in polygon]

    def _precision_collision_ready(self, geometry):
        """Match the live safety node to the bounded precision profile."""
        package = Path(get_package_share_directory('jdamr_cube_navigation'))
        baseline = yaml.safe_load((
            package / 'config/new_base_nav2_params.yaml').read_text())
        expected_document = apply_docking_stop_profile(baseline, geometry)
        monitor = expected_document['collision_monitor']['ros__parameters']
        names = [
            'polygons', 'observation_sources', 'source_timeout',
            'StopZone.enabled', 'StopZone.action_type', 'StopZone.min_points',
            'StopZone.velocity_polygons',
            'StopZone.translation_forward.points', 'StopZone.stopped.points',
            'StopZone.translation_backward.points',
            'StopZone.rotation.points', 'StopZone.rotation_clockwise.points',
            'scan.enabled', 'scan.type', 'scan.topic',
            'FootprintApproach.enabled', 'FootprintApproach.type',
            'FootprintApproach.min_points',
            'FootprintApproach.time_before_collision',
            'FootprintApproach.footprint_topic',
        ]

        def nested(name):
            value = monitor
            for component in name.split('.'):
                value = value[component]
            return value

        expected = [nested(name) for name in names]
        client = getattr(self, 'precision_parameters', None)
        if client is None:
            client = AsyncParameterClient(self, 'collision_monitor')
            self.precision_parameters = client
        if not client.wait_for_services(timeout_sec=2.0):
            return False
        try:
            response = self._wait(client.get_parameters(names), 2.0)
        except RuntimeError:
            return False
        if len(response.values) != len(expected):
            return False
        for name, wanted, value in zip(names, expected, response.values):
            actual = parameter_value_to_python(value)
            if name.endswith('.points'):
                try:
                    if self._polygon(actual) != self._polygon(wanted):
                        return False
                except ValueError:
                    return False
            elif isinstance(wanted, float):
                if (isinstance(actual, bool)
                        or not isinstance(actual, (int, float))
                        or not math.isclose(
                            actual, wanted, rel_tol=0.0, abs_tol=1e-9)):
                    return False
            elif type(actual) is not type(wanted) or actual != wanted:
                return False
        return True

    def observe_target(self, camera_mount, front_extent_m, gap_m, region_xy,
                       region_radius_m, timeout_s=12.0):
        """
        Capture a stationary robot and a new stable face, never a cached face.

        One recoverable input gap restarts the window from a new stationary
        capture. Each observer stamp is judged once. A verdict with fewer than
        stable_frames fresh frames after an input delay is latency-limited.
        """
        if getattr(self, 'candidate_trial', False) is not True:
            raise RuntimeError(
                'box observation consumption requires explicit candidate trial')
        call_started_s = time.monotonic()
        self._ensure_depth_info()
        robot_pose, _ = self.capture_stationary_pose()
        self._observation_pose_diagnostic(robot_pose, camera_mount, region_xy)
        revision = self.parking_motion_revision
        cutoff_s = self.get_clock().now().nanoseconds * 1e-9
        # A later cutoff advances by elapsed monotonic time, not another clock read.
        cutoff_monotonic_s = time.monotonic()
        deadline_s = cutoff_monotonic_s + timeout_s
        last_failure = 'no_stable_box'
        last_failure_retryable = False
        rejected_observation_stamps = set()
        input_recovered = False
        latency_reasons = set()
        fresh_since_latency = 0
        evaluated_stamps = set()
        counted_rejection_s = None

        def emit_evidence(reason, retryable):
            stable_frames = getattr(self, 'observer_stable_frames', None)
            latency_limited = bool(latency_reasons) and (
                stable_frames is None or fresh_since_latency < stable_frames)
            self._emit_observation_unavailable_evidence(
                reason=reason, retryable=retryable and not latency_limited,
                latency_limited=latency_limited, latency_reasons=sorted(latency_reasons),
                fresh_since_latency=fresh_since_latency, cutoff_s=cutoff_s,
                robot_pose=list(robot_pose), camera_mount=camera_mount,
                region_xy=list(region_xy))
            return latency_limited

        while not self.stop_requested and time.monotonic() < deadline_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self._navigation_ready(require_fresh_amcl=False):
                reason = self._guard_failure(False)
                if (not input_recovered and not self.stop_requested
                        and self._input_gap_recoverable(reason)
                        and self._wait_for_input_recovery(reason)):
                    # Nothing observed before the gap may form the verdict.
                    input_recovered = True
                    latency_reasons.add('input_gap')
                    fresh_since_latency = 0
                    robot_pose, _ = self.capture_stationary_pose()
                    self._observation_pose_diagnostic(robot_pose, camera_mount, region_xy)
                    revision = self.parking_motion_revision
                    now_monotonic_s = time.monotonic()
                    cutoff_s += now_monotonic_s - cutoff_monotonic_s
                    cutoff_monotonic_s = now_monotonic_s
                    deadline_s = now_monotonic_s + timeout_s
                    last_failure = 'no_stable_box'
                    last_failure_retryable = False
                    continue
                emit_evidence(reason, False)
                raise RuntimeError(reason)
            if self.parking_motion_revision != revision:
                raise RuntimeError('robot moved during stationary box observation')
            observation = self.box_status
            received_s = getattr(self, 'box_status_received_s', None)
            if (isinstance(observation, dict) and received_s is not None
                    and received_s >= call_started_s and received_s != counted_rejection_s
                    and isinstance(observation.get('reason'), str)
                    and observation['reason'] in OBSERVER_LATENCY_REASONS):
                counted_rejection_s = received_s
                latency_reasons.add(observation['reason'])
                fresh_since_latency = 0
            silent_since_s = (call_started_s if received_s is None
                              else max(call_started_s, received_s))
            if time.monotonic() - silent_since_s > OBSERVER_SILENT_S:
                latency_reasons.add('observer_silent')
                fresh_since_latency = 0
            if (not observation or observation.get('frame_id') != 'camera_color_optical_frame'
                    or not isinstance(observation.get('stamp_s'), (int, float))
                    or observation['stamp_s'] <= cutoff_s):
                last_failure_retryable = False
                continue
            # Waiting for the next frame is not latency: judge each stamp once.
            if observation['stamp_s'] in evaluated_stamps:
                continue
            evaluated_stamps.add(observation['stamp_s'])
            try:
                retryable = False
                now_s = self.get_clock().now().nanoseconds * 1e-9
                age_s = now_s - observation['stamp_s']
                if (isinstance(observation['stamp_s'], bool)
                        or not math.isfinite(age_s) or not 0.0 <= age_s <= 0.5):
                    raise ValueError('box observation is stale or future-dated')
                if observation.get('control_ready') is not False:
                    raise ValueError(
                        'observer must remain perception-only; candidate gate is separate')
                retryable = (
                    observation.get('detected') is not True
                    or observation.get('stable') is not True
                    or observation.get('surface_kind') != 'front')
                target = compute_box_docking_target(
                    observation, camera_mount,
                    dict(zip(('x_m', 'y_m', 'yaw_rad'), robot_pose)),
                    requested_gap_m=gap_m, front_extent_m=front_extent_m,
                    now_s=now_s)
                retryable = False
                scan = self.last_scan
                if scan is None:
                    raise ValueError('fresh LiDAR scan is required for box witness')
                scan_stamp_s = (scan.header.stamp.sec
                                + scan.header.stamp.nanosec * 1e-9)
                scan_age_s = now_s - scan_stamp_s
                if (not math.isfinite(scan_stamp_s) or scan_age_s < 0.0
                        or scan_age_s > 0.5):
                    raise ValueError('LiDAR scan is stale or future-dated')
                robot_pose_document = dict(zip(
                    ('x_m', 'y_m', 'yaw_rad'), robot_pose))
                diagnostics = {}
                try:
                    scan_frame = scan.header.frame_id
                    if scan_frame != 'laser_link':
                        raise ValueError(
                            'LiDAR scan frame must be measured laser_link')
                    retryable = True
                    witness = witness_box_face_with_lidar(
                        scan.ranges, angle_min=scan.angle_min,
                        angle_increment=scan.angle_increment,
                        range_min=scan.range_min, range_max=scan.range_max,
                        geometry=self.box_geometry, depth_target=target,
                        robot_pose=robot_pose_document,
                        diagnostics=diagnostics)
                except ValueError as error:
                    last_failure = str(error)
                    last_failure_retryable = (
                        retryable and last_failure in SEARCH_RECOVERABLE_REASONS)
                    fresh_since_latency += 1
                    observation_stamp = observation['stamp_s']
                    if observation_stamp not in rejected_observation_stamps:
                        rejected_observation_stamps.add(observation_stamp)
                        raw_ranges, ranges_truncated = self._json_scan_ranges(
                            scan.ranges)
                        self.emit(
                            'box_lidar_witness_rejected',
                            observation=self._json_evidence(observation),
                            target=self._json_evidence(target),
                            robot_pose=self._json_evidence(
                                robot_pose_document),
                            camera_mount=self._json_evidence(camera_mount),
                            box_geometry=self._json_evidence(
                                self.box_geometry),
                            scan_frame=scan_frame,
                            scan_stamp_s=scan_stamp_s,
                            scan_angle_min=self._json_scalar(scan.angle_min),
                            scan_angle_increment=self._json_scalar(
                                scan.angle_increment),
                            scan_range_min=self._json_scalar(scan.range_min),
                            scan_range_max=self._json_scalar(scan.range_max),
                            scan_ranges=raw_ranges,
                            scan_ranges_truncated=ranges_truncated,
                            diagnostics=self._json_evidence(diagnostics),
                            reason=last_failure)
                    continue
                face = witness['fused_face_center_map_xy_m']
                outward = witness['outward_normal_map_xy']
                center_offset_m = front_extent_m + gap_m
                target.update({
                    'x_m': face[0] + outward[0] * center_offset_m,
                    'y_m': face[1] + outward[1] * center_offset_m,
                    'yaw_rad': math.atan2(-outward[1], -outward[0]),
                    'estimate_gap_m': gap_m,
                    'face_center_map_xy_m': face,
                    'outward_normal_map_xy': outward,
                    'depth_target_provenance': target['provenance'],
                    'lidar_witness': witness,
                    'candidate_trial': True,
                    'physical_accuracy': 'NOT_EXTERNALLY_MEASURED',
                })
                retryable = False
                if self.parking_motion_revision != revision:
                    raise ValueError('robot moved during box sensor witness')
                retryable = True
                if math.dist(target['face_center_map_xy_m'], region_xy) > region_radius_m:
                    raise ValueError('observed_box_outside_selected_table_region')
                self.emit('box_target_observed', observation=observation,
                          target=target, candidate_trial=True,
                          physical_accuracy='NOT_EXTERNALLY_MEASURED')
                return target
            except ValueError as error:
                last_failure = str(error)
                last_failure_retryable = (
                    retryable and last_failure in SEARCH_RECOVERABLE_REASONS)
                if last_failure in BOX_INPUT_LATENCY_FAILURES:
                    latency_reasons.add(last_failure)
                    fresh_since_latency = 0
                else:
                    fresh_since_latency += 1
        latency_limited = emit_evidence(last_failure, last_failure_retryable)
        raise BoxObservationUnavailable(
            last_failure, retryable=last_failure_retryable and not latency_limited,
            latency_limited=latency_limited)

    def _reposition_search(self):
        """Try one planned nearby view after a collision or timeout Spin result."""
        if (self.stop_requested or not self._navigation_ready(require_fresh_amcl=False)
                or getattr(self, 'last_search_error_code', None) not in (
                    Spin.Result.COLLISION_AHEAD, Spin.Result.TIMEOUT)):
            return False
        actual, _ = self.capture_stationary_pose()
        # These are candidate waypoints, not open-loop displacement commands.
        # Nav2 must find a valid path through the current footprint/keepout map.
        for offset_m in (0.2, -0.2):
            candidate = {
                'id': 'search_view', 'priority': 1, 'approach_offset_m': 0.5,
                'x_m': actual[0] + offset_m * math.cos(actual[2]),
                'y_m': actual[1] + offset_m * math.sin(actual[2]),
                'yaw_rad': actual[2],
            }
            # Judged by the alignment goal checker, like face alignment.
            planned = self.plan_pose(
                candidate, single=True,
                end_tolerance_m=MAXIMUM_CONTRACT_VALUES['xy_tolerance_m'])
            if not planned['ok']:
                continue
            self.emit('box_search_reposition', candidate=candidate)
            return self.execute(final_parking=False, alignment=True)
        return False

    def _observe_with_search(self, camera_mount, front_extent_m, gap_m,
                             region_xy, region_radius_m, *, phase,
                             search_enabled, search_budget):
        """Retry a recoverable initial observation after bounded Nav2 rotations."""
        latency_reobserved = False
        while True:
            try:
                return self.observe_target(
                    camera_mount, front_extent_m, gap_m,
                    region_xy, region_radius_m)
            except BoxObservationUnavailable as error:
                latency_limited = getattr(error, 'latency_limited', False) is True
                self.emit(
                    'box_observation_failed', phase=phase,
                    reason=error.reason, retryable=error.retryable,
                    latency_limited=latency_limited,
                    stop_requested=self.stop_requested,
                    search_enabled=search_enabled,
                    search_steps_used=search_budget.steps_used,
                    search_cumulative_yaw_rad=(
                        search_budget.cumulative_yaw_rad))
                # An operator stop or deadline never rotates or observes again.
                if self.stop_requested:
                    raise
                # A late-input verdict is observed once more without moving.
                if latency_limited and not latency_reobserved:
                    latency_reobserved = True
                    self.emit('box_observation_latency_reobserve', phase=phase,
                              reason=error.reason)
                    continue
                # At the close-range final approach, rotation can sweep the
                # chassis into the box.  Only the initial face search rotates.
                if (not search_enabled or phase != 'face_alignment'
                        or not error.retryable):
                    raise
                delta_yaw_rad = search_budget.next_rotation()
                if delta_yaw_rad is None:
                    self.emit(
                        'box_search_exhausted', phase=phase,
                        reason=error.reason,
                        search_steps_used=search_budget.steps_used,
                        search_cumulative_yaw_rad=(
                            search_budget.cumulative_yaw_rad))
                    raise BoxObservationUnavailable(
                        f'search exhausted after {search_budget.steps_used} rotations; '
                        f'last failure: {error.reason}', retryable=False) from error
                self.emit(
                    'box_search_rotation_requested', phase=phase,
                    search_step=search_budget.steps_used,
                    delta_yaw_rad=delta_yaw_rad,
                    search_cumulative_yaw_rad=(
                        search_budget.cumulative_yaw_rad))
                if not self.search_rotation(delta_yaw_rad):
                    if not search_budget.reposition_attempted:
                        search_budget.reposition_attempted = True
                        if self._reposition_search():
                            continue
                    self.emit(
                        'failed', phase='box_search_rotation',
                        observation_phase=phase,
                        reason='nav2_spin_failed',
                        search_step=search_budget.steps_used,
                        search_cumulative_yaw_rad=(
                            search_budget.cumulative_yaw_rad))
                    raise RuntimeError('box search rotation failed')
                self.emit(
                    'box_search_rotation_completed', phase=phase,
                    search_step=search_budget.steps_used,
                    search_cumulative_yaw_rad=(
                        search_budget.cumulative_yaw_rad))

    def visit_observed_box(self, route_path, camera_mount, geometry,
                           table_id, region_xy, region_radius_m, execute=False,
                           candidate_trial=False, resume_at_observation=False,
                           search=False, task_timeout_s=240.0):
        """Separate transit, face alignment and final approach in the result log."""
        if execute and candidate_trial is not True:
            raise RuntimeError(
                'execution requires explicit nominal-camera candidate trial')
        self.candidate_trial = candidate_trial is True
        self.box_geometry = geometry
        route = load_route(route_path)
        front_extent_m = geometry['front_to_wheel_axis']['value']
        width_m = geometry['wheel_outer_width']['value']
        if (isinstance(front_extent_m, bool) or not isinstance(front_extent_m, (int, float))
                or not math.isfinite(front_extent_m) or not 0 < front_extent_m < 0.5):
            raise ValueError('invalid chassis front extent')
        if (isinstance(width_m, bool) or not isinstance(width_m, (int, float))
                or not math.isfinite(width_m) or width_m <= 0):
            raise ValueError('invalid chassis width')
        for key, field in (('map', 'map_yaml'), ('keepout', 'keepout_mask_yaml')):
            verify_identity(self.registry[key], route[field])
        if route.get('frame_id') != 'map':
            raise ValueError('observation route must use map')
        self.table_id = table_id
        self.verify_live_maps()
        if not self.wait_until_ready(timeout=10.0):
            raise RuntimeError('localization or sensor data unavailable')
        if not self._parking_parameters_ready():
            raise RuntimeError('parking controller parameters unavailable')
        if execute and not self._precision_collision_ready(geometry):
            raise RuntimeError('precision collision-monitor profile unavailable')
        # The starting place is an explicit route contract, not a guessed pose.
        start = route.get('start_pose')
        limit_m = route.get('max_route_start_distance_m')
        if (not isinstance(start, dict) or not isinstance(limit_m, (float, int))
                or not math.isfinite(limit_m) or not 0.0 < limit_m <= 0.3):
            raise ValueError('observation route requires a bounded start pose')
        actual, _ = self.capture_stationary_pose()
        reference = route['waypoints'][-1] if resume_at_observation else start
        if math.dist(actual[:2], (reference['x'], reference['y'])) > limit_m:
            if resume_at_observation:
                raise RuntimeError('robot is not at the reached observation region')
            raise RuntimeError('robot is not at the observation route starting place')
        self.config, self.waypoints = route, route['waypoints']
        self.emit('observation_route_selected', waypoints=self.waypoints,
                  candidate_trial=self.candidate_trial,
                  physical_accuracy='NOT_EXTERNALLY_MEASURED')
        if not resume_at_observation and not self.preflight():
            self.emit('failed', phase='preflight',
                      end_error_m=getattr(self, 'preflight_end_error_m', None))
            return False
        if not execute:
            self.emit('transit_planned_only', box_target='requires_observation_on_arrival')
            return True
        # Setup time must not consume the motion/search budget.
        self.run_deadline_s = time.monotonic() + task_timeout_s
        if resume_at_observation:
            self.emit('resume_at_reached_observation', actual_pose=list(actual),
                      transit_skipped=True, final_parking_confirmed=False)
        elif not self.execute(final_parking=False):
            self.emit('failed', phase='table_region_transit')
            return False
        search_budget = BoxSearchBudget()
        # Face alignment happens while depth is still in its usable range.
        for phase, gap_m in (('face_alignment', 0.45), ('final_approach', 0.05)):
            plan_input_recovered = False
            while True:
                target = self._observe_with_search(
                    camera_mount, front_extent_m, gap_m, region_xy,
                    region_radius_m, phase=phase, search_enabled=search,
                    search_budget=search_budget)
                target.update(id=phase, priority=1, approach_offset_m=0.5)
                # The escape before any return starts from the last planned face.
                self.last_box_face = {
                    'face_center_map_xy_m': list(target['face_center_map_xy_m']),
                    'outward_normal_map_xy': list(target['outward_normal_map_xy'])}
                self.emit('box_phase', phase=phase, requested_front_gap_m=gap_m)
                if phase == 'face_alignment':
                    # Judged by the alignment goal checker, not the final contract.
                    planned = self.plan_pose(
                        target, single=True,
                        end_tolerance_m=MAXIMUM_CONTRACT_VALUES['xy_tolerance_m'])
                else:
                    planned = self.plan_pose(target, single=True)
                reason = planned.get('guard_failure')
                if (planned['ok'] or planned.get('reason') != 'navigation_not_ready'
                        or plan_input_recovered or self.stop_requested
                        or not self._input_gap_recoverable(reason)):
                    break
                # A face observed before an input gap is discarded, never planned.
                plan_input_recovered = True
                self.emit('box_target_discarded', phase=phase, reason=reason)
                if not self._wait_for_input_recovery(reason):
                    break
            if not planned['ok']:
                self.emit('failed', phase=phase, planning=planned)
                return False
            # Live map callbacks keep checking the verified identity throughout
            # this visit; do not repeat disk/service configuration audits here.
            if not self.execute(final_parking=phase == 'final_approach',
                                alignment=phase == 'face_alignment'):
                self.emit('failed', phase=phase, reason='nav2_or_stop_confirmation')
                return False
        # Motion is over; the deadline must not stop the final stationary capture.
        self.run_deadline_s = None
        actual, _ = self.capture_stationary_pose()
        outward = target['outward_normal_map_xy']
        face = target['face_center_map_xy_m']
        front = (actual[0] + front_extent_m * math.cos(actual[2]),
                 actual[1] + front_extent_m * math.sin(actual[2]))
        estimated_gap_m = sum((front[i] - face[i]) * outward[i] for i in (0, 1))
        corners = [
            (front[0] - side * width_m / 2 * math.sin(actual[2]),
             front[1] + side * width_m / 2 * math.cos(actual[2]))
            for side in (-1, 1)
        ]
        corner_gaps = [sum((corner[i] - face[i]) * outward[i] for i in (0, 1))
                       for corner in corners]
        yaw_error = abs(math.atan2(
            math.sin(actual[2] - target['yaw_rad']),
            math.cos(actual[2] - target['yaw_rad'])))
        gap_confirmed = (
            all(math.isfinite(gap) and abs(gap - 0.05) <= 0.01
                for gap in (estimated_gap_m, *corner_gaps))
            and yaw_error <= self.parking_contract['yaw_tolerance_rad'])
        if gap_confirmed:
            # A later parked dwell verifies this final target, not a taught pose.
            self.selected_pose = {'id': 'final_approach', 'x_m': target['x_m'],
                                  'y_m': target['y_m'], 'yaw_rad': target['yaw_rad']}
        self.emit('box_approach_finished' if gap_confirmed else 'box_gap_not_confirmed',
                  estimated_front_gap_m=estimated_gap_m,
                  estimated_front_corner_gaps_m=corner_gaps,
                  estimated_face_yaw_error_rad=yaw_error,
                  desired_front_gap_m=0.05, confirmation=self.confirmation,
                  physical_accuracy='NOT_EXTERNALLY_MEASURED',
                  final_depth_measurement=False)
        return gap_confirmed

    def _leave_parked_pose(self):
        """
        Back straight away from the last box face before any home rotation.

        At the final gap the LiDAR cannot see the face, so an in-place turn
        there is unprotected. The band inside the current chassis front is not
        validated because a straight reverse never re-enters it.
        """
        face = getattr(self, 'last_box_face', None)
        if face is None:
            return True
        if not (self._parking_parameters_ready(reverse=True)
                and self._reverse_smoother_ready()):
            self.emit('failed', phase='box_escape', reason='box_escape_unavailable')
            return False
        center = face['face_center_map_xy_m']
        outward = face['outward_normal_map_xy']
        actual, _ = self.capture_stationary_pose()
        distance_m = sum((actual[i] - center[i]) * outward[i] for i in (0, 1))
        # A straight reverse moves away from the face only from a pose that
        # faces it, in front of it and inside the clearance; refuse any other.
        facing = -(math.cos(actual[2]) * outward[0] + math.sin(actual[2]) * outward[1])
        aligned = facing >= math.cos(ESCAPE_PATH_CONTRACT['yaw_tolerance_rad'])
        if not (aligned and 0.0 < distance_m < ESCAPE_CLEARANCE_M):
            self.emit('failed', phase='box_escape', reason='box_escape_unavailable',
                      detail=('heading_not_facing_box_face' if not aligned
                              else 'face_distance_outside_escape_range'),
                      face_distance_m=self._json_scalar(distance_m),
                      face_heading_cos=self._json_scalar(facing))
            return False
        length_m = ESCAPE_CLEARANCE_M - distance_m
        if length_m <= ALREADY_AT_GOAL_DISTANCE_M:
            self.emit('box_escape_skipped', face_distance_m=distance_m)
            return True
        target = (actual[0] - length_m * math.cos(actual[2]),
                  actual[1] - length_m * math.sin(actual[2]), actual[2])
        saved = (self.config, self.waypoints)
        self.config = {'frame_id': 'map', 'waypoints': [
            {'id': 'box_escape', 'x': target[0], 'y': target[1], 'yaw': target[2]}]}
        self.waypoints = self.config['waypoints']
        failure = 'box_escape_unavailable'
        try:
            path = self._make_reverse_path(
                actual, target, path_contract=ESCAPE_PATH_CONTRACT)
            if not self._reverse_path_valid(
                    path, validate_from_m=ESCAPE_VALIDATION_EXCLUDE_M):
                self.emit('failed', phase='box_escape', reason='box_escape_blocked')
                return False
            failure = 'box_escape_failed'
            if not self._execute_reverse_path(
                    path, path_contract=ESCAPE_PATH_CONTRACT,
                    validate_from_m=ESCAPE_VALIDATION_EXCLUDE_M,
                    goal_checker_id='alignment_goal_checker', final=False):
                # The preceding result event carries the Nav2 error code.
                self.emit('failed', phase='box_escape', reason='box_escape_failed')
                return False
        except (ValueError, RuntimeError) as error:
            # Record the failed escape step; the caller still receives the error.
            self.emit('failed', phase='box_escape', reason=failure,
                      error_type=type(error).__name__, detail=str(error))
            raise
        finally:
            self.config, self.waypoints = saved
        after, _ = self.capture_stationary_pose()
        position_error_m, yaw_error_rad = pose_errors(target, after)
        distance_m = sum((after[i] - center[i]) * outward[i] for i in (0, 1))
        if (position_error_m > ESCAPE_PATH_CONTRACT['xy_tolerance_m']
                or yaw_error_rad > ESCAPE_PATH_CONTRACT['yaw_tolerance_rad']
                or distance_m < ESCAPE_CLEARANCE_M - ESCAPE_PATH_CONTRACT['xy_tolerance_m']):
            self.emit('failed', phase='box_escape', reason='box_escape_not_confirmed',
                      face_distance_m=distance_m, position_error_m=position_error_m,
                      yaw_error_rad=yaw_error_rad)
            return False
        self.emit('box_escape_finished', face_distance_m=distance_m,
                  position_error_m=position_error_m, yaw_error_rad=yaw_error_rad)
        self.last_box_face = None
        return True

    def dwell_and_return_home(self, dwell_s, timeout_s):
        """Hold at the verified final target, then return to the dock."""
        if not self.wait_parked(dwell_s):
            self.emit('failed', phase='table_dwell')
            return False
        return self.go_home(execute=True, timeout_s=timeout_s)


def parse_args(argv=None):
    """Require map-bound inputs and explicit execution, defaulting to plan only."""
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('registry', 'approach-route', 'camera-mount', 'geometry', 'log'):
        parser.add_argument('--' + name, required=True, type=Path)
    parser.add_argument('--table-id', choices=('table_01', 'table_02'), required=True)
    parser.add_argument('--region-xy', nargs=2, type=float, required=True)
    parser.add_argument('--region-radius-m', type=float, default=0.6)
    parser.add_argument('--parking-contract', required=True, type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument(
        '--search', action='store_true',
        help='Search for the box with at most twelve 30-degree Nav2 spins')
    parser.add_argument(
        '--resume-at-observation', action='store_true',
        help='Resume from the already reached observation waypoint')
    parser.add_argument(
        '--candidate-trial', action='store_true',
        help='Explicitly allow the nominal-camera candidate parking trial')
    parser.add_argument(
        '--task-timeout-s', type=float, default=240.0,
        help='Motion and search budget from transit start to the final stop')
    parser.add_argument(
        '--return-home', action='store_true',
        help='After a confirmed approach, hold five seconds, escape and dock')
    parser.add_argument(
        '--return-timeout-s', type=float,
        help='Budget for the escape and dock return; required by --return-home')
    args = parser.parse_args(argv)
    if (not all(math.isfinite(v) for v in args.region_xy)
            or not math.isfinite(args.region_radius_m)
            or not 0.0 < args.region_radius_m <= 0.6):
        parser.error('table region must be finite and radius in (0, 0.6] m')
    if args.execute and not args.candidate_trial:
        parser.error('--execute requires --candidate-trial')
    if args.search and not args.execute:
        parser.error('--search requires --execute')
    if args.resume_at_observation and not args.execute:
        parser.error('--resume-at-observation requires --execute')
    if not math.isfinite(args.task_timeout_s) or args.task_timeout_s <= 0.0:
        parser.error('--task-timeout-s must be finite and positive')
    if args.return_home and not args.execute:
        parser.error('--return-home requires --execute')
    if args.return_home and args.return_timeout_s is None:
        parser.error('--return-home requires --return-timeout-s')
    if args.return_timeout_s is not None and (
            not math.isfinite(args.return_timeout_s) or args.return_timeout_s <= 0.0):
        parser.error('--return-timeout-s must be finite and positive')
    return args


def main(argv=None):
    """Run one table attempt without changing taught destinations or home."""
    args = parse_args(argv)
    registry = load_registry(args.registry)
    mount = yaml.safe_load(args.camera_mount.read_text())
    geometry = yaml.safe_load(args.geometry.read_text())
    contract = load_parking_contract(args.parking_contract)
    node = None
    handlers = {}
    with args.log.open('x', encoding='utf-8') as stream:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        try:
            # The dock is accepted with the user's contract, not the box
            # contract; a load failure takes the failure path below.
            home_contract = load_parking_contract(
                Path(get_package_share_directory('jdamr_cube_navigation'))
                / 'config/parking_contract.yaml')
            node = BoxServiceRoute(registry, contract, stream, home_contract=home_contract)
            for signum in (signal.SIGINT, signal.SIGTERM):
                handlers[signum] = signal.signal(signum, lambda *_: node.request_stop())
            ok = node.visit_observed_box(
                args.approach_route, mount, geometry, args.table_id,
                args.region_xy, args.region_radius_m, execute=args.execute,
                candidate_trial=args.candidate_trial,
                resume_at_observation=args.resume_at_observation,
                search=args.search, task_timeout_s=args.task_timeout_s)
            # Exit 0 only when every requested stage, including the return, succeeded.
            if ok and args.return_home:
                ok = node.dwell_and_return_home(5.0, args.return_timeout_s)
            return 0 if ok else 1
        except (ValueError, RuntimeError) as error:
            if node is not None:
                node.emit('failed', reason=str(error))
            print(json.dumps({'failed': str(error)}))
            return 1
        finally:
            try:
                if node is not None and not node.finish_navigation():
                    raise RuntimeError('navigation cancellation unconfirmed')
            finally:
                for signum, handler in handlers.items():
                    signal.signal(signum, handler)
                if node is not None:
                    node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
