"""Approach a mapped table region, then park using a fresh observed box face."""

import argparse
from itertools import islice
import json
import math
from pathlib import Path
import signal
import time

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.box_docking_target import compute_box_docking_target
from jdamr_cube_navigation.box_lidar_witness import witness_box_face_with_lidar
from jdamr_cube_navigation.corridor_route import load_route
from jdamr_cube_navigation.docking_stop_profile import apply_docking_stop_profile
from jdamr_cube_navigation.parking import load_parking_contract
from jdamr_cube_navigation.restaurant_service import ServiceRoute
from jdamr_cube_navigation.service_destinations import load_registry, verify_identity
from nav2_msgs.action import Spin
import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
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


class BoxObservationUnavailable(RuntimeError):
    """Describe whether changing the camera heading may recover observation."""

    def __init__(self, reason, *, retryable):
        super().__init__(f'box observation unavailable: {reason}')
        self.reason = reason
        self.retryable = retryable


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
        super().__init__(*args, **kwargs)
        self.precision_parameters = AsyncParameterClient(
            self, 'collision_monitor')

    def _scan_callback(self, message):
        super()._scan_callback(message)
        self.last_scan = message

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
        """Capture a stationary robot and a new stable face, never a cached face."""
        if getattr(self, 'candidate_trial', False) is not True:
            raise RuntimeError(
                'box observation consumption requires explicit candidate trial')
        robot_pose, _ = self.capture_stationary_pose()
        revision = self.parking_motion_revision
        cutoff_s = self.get_clock().now().nanoseconds * 1e-9
        deadline_s = time.monotonic() + timeout_s
        last_failure = 'no_stable_box'
        last_failure_retryable = False
        rejected_observation_stamps = set()
        while not self.stop_requested and time.monotonic() < deadline_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self._navigation_ready(require_fresh_amcl=False):
                raise RuntimeError(self._guard_failure(False))
            if self.parking_motion_revision != revision:
                raise RuntimeError('robot moved during stationary box observation')
            observation = self.box_status
            if (not observation or observation.get('frame_id') != 'camera_color_optical_frame'
                    or not isinstance(observation.get('stamp_s'), (int, float))
                    or observation['stamp_s'] <= cutoff_s):
                last_failure_retryable = False
                continue
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
        raise BoxObservationUnavailable(
            last_failure, retryable=last_failure_retryable)

    def _reposition_search(self):
        """Try one planned nearby view after a confirmed collision-blocked Spin."""
        if (self.stop_requested or not self._navigation_ready(require_fresh_amcl=False)
                or getattr(self, 'last_search_error_code', None) != Spin.Result.COLLISION_AHEAD):
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
            planned = self.plan_pose(candidate, single=True)
            if not planned['ok']:
                continue
            self.emit('box_search_reposition', candidate=candidate)
            return self.execute(final_parking=False, alignment=True)
        return False

    def _observe_with_search(self, camera_mount, front_extent_m, gap_m,
                             region_xy, region_radius_m, *, phase,
                             search_enabled, search_budget):
        """Retry a recoverable initial observation after bounded Nav2 rotations."""
        while True:
            try:
                return self.observe_target(
                    camera_mount, front_extent_m, gap_m,
                    region_xy, region_radius_m)
            except BoxObservationUnavailable as error:
                self.emit(
                    'box_observation_failed', phase=phase,
                    reason=error.reason, retryable=error.retryable,
                    search_enabled=search_enabled,
                    search_steps_used=search_budget.steps_used,
                    search_cumulative_yaw_rad=(
                        search_budget.cumulative_yaw_rad))
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
                           search=False):
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
            return False
        if not execute:
            self.emit('transit_planned_only', box_target='requires_observation_on_arrival')
            return True
        # Setup time must not consume the motion/search budget.
        self.run_deadline_s = time.monotonic() + 240.0
        if resume_at_observation:
            self.emit('resume_at_reached_observation', actual_pose=list(actual),
                      transit_skipped=True, final_parking_confirmed=False)
        elif not self.execute(final_parking=False):
            self.emit('failed', phase='table_region_transit')
            return False
        search_budget = BoxSearchBudget()
        # Face alignment happens while depth is still in its usable range.
        for phase, gap_m in (('face_alignment', 0.45), ('final_approach', 0.05)):
            target = self._observe_with_search(
                camera_mount, front_extent_m, gap_m, region_xy,
                region_radius_m, phase=phase, search_enabled=search,
                search_budget=search_budget)
            target.update(id=phase, priority=1, approach_offset_m=0.5)
            self.emit('box_phase', phase=phase, requested_front_gap_m=gap_m)
            planned = self.plan_pose(target, single=True)
            if not planned['ok']:
                self.emit('failed', phase=phase, planning=planned)
                return False
            # Live map callbacks keep checking the verified identity throughout
            # this visit; do not repeat disk/service configuration audits here.
            if not self.execute(final_parking=phase == 'final_approach',
                                alignment=phase == 'face_alignment'):
                self.emit('failed', phase=phase, reason='nav2_or_stop_confirmation')
                return False
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
        self.emit('box_approach_finished' if gap_confirmed else 'box_gap_not_confirmed',
                  estimated_front_gap_m=estimated_gap_m,
                  estimated_front_corner_gaps_m=corner_gaps,
                  estimated_face_yaw_error_rad=yaw_error,
                  desired_front_gap_m=0.05, confirmation=self.confirmation,
                  physical_accuracy='NOT_EXTERNALLY_MEASURED',
                  final_depth_measurement=False)
        return gap_confirmed


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
            node = BoxServiceRoute(registry, contract, stream)
            for signum in (signal.SIGINT, signal.SIGTERM):
                handlers[signum] = signal.signal(signum, lambda *_: node.request_stop())
            ok = node.visit_observed_box(
                args.approach_route, mount, geometry, args.table_id,
                args.region_xy, args.region_radius_m, execute=args.execute,
                candidate_trial=args.candidate_trial,
                resume_at_observation=args.resume_at_observation,
                search=args.search)
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
