"""Pure parameter validation for the measured new-base navigation contract."""

import json
import math


# Approved selection policy for the five directional StopZone entries. These
# are software policy limits, not newly measured chassis values.
VELOCITY_POLICY_RANGES = {
    'rotation': (-0.005, 0.005, 0.005, 1.0),
    'rotation_clockwise': (-0.005, 0.005, -1.0, -0.005),
    'translation_forward': (0.005, 0.2, -1.0, 1.0),
    'translation_backward': (-0.2, -0.005, -1.0, 1.0),
    'stopped': (-1.0, 1.0, -1.0, 1.0),
}


def validate_new_base_params(params, geometry):
    """Reject navigation parameters that violate the measured base contract."""
    front = geometry['front_to_wheel_axis']['value']
    rear = front - geometry['frame_length']['value']
    half_width = geometry['wheel_outer_width']['value'] / 2.0

    def polygon_points(value):
        polygon = json.loads(value) if isinstance(value, str) else value
        if not isinstance(polygon, list) or len(polygon) < 3:
            raise RuntimeError('new-base polygon needs at least three points')
        if any(not isinstance(point, (list, tuple)) or len(point) != 2
               or any(type(coordinate) not in (int, float)
                      or not math.isfinite(coordinate)
                      for coordinate in point)
               for point in polygon):
            raise RuntimeError(
                'new-base polygon points must be finite numeric pairs')
        return polygon

    def bounds(points):
        polygon = polygon_points(points)
        if len(polygon) != 4:
            raise RuntimeError('new-base polygon must have four corners')
        xs = sorted({point[0] for point in polygon})
        ys = sorted({point[1] for point in polygon})
        if (len(xs) != 2 or len(ys) != 2 or ys[0] != -ys[1]
                or {tuple(point) for point in polygon} !=
                {(x, y) for x in xs for y in ys}):
            raise RuntimeError(
                'new-base polygon must cover both sides as a rectangle')
        ordered_corners = [
            (xs[1], ys[1]), (xs[1], ys[0]),
            (xs[0], ys[0]), (xs[0], ys[1]),
        ]
        if [tuple(point) for point in polygon] != ordered_corners:
            raise RuntimeError(
                'new-base polygon corners must follow the perimeter')
        return (xs[1], xs[0], ys[1])

    costmaps = [params[key][key]['ros__parameters']
                for key in ('local_costmap', 'global_costmap')]
    footprints = [bounds(costmap['footprint']) for costmap in costmaps]
    if footprints[0] != footprints[1]:
        raise RuntimeError('new-base costmap footprints differ')
    footprint = footprints[0]
    if not (footprint[0] > front and footprint[1] < rear
            and footprint[2] > half_width):
        raise RuntimeError('new-base footprint is smaller than measured base')
    if any(costmap['robot_base_frame'] != 'base_footprint'
           for costmap in costmaps):
        raise RuntimeError('new-base costmaps need base_footprint')
    for costmap in costmaps:
        layer = costmap['obstacle_layer']
        scan = layer['scan']
        sources = layer.get('observation_sources')
        if (not layer['enabled'] or sources != 'scan'
                or scan['topic'] != '/scan' or not scan['marking']):
            raise RuntimeError('new-base costmap scan layer is not active')
    depth_enabled = [
        'depth_obstacle_layer' in costmap for costmap in costmaps]
    if any(depth_enabled) and not all(depth_enabled):
        raise RuntimeError('new-base depth layer must cover both costmaps')
    if all(depth_enabled):
        for costmap in costmaps:
            plugins = costmap.get('plugins', [])
            if ('depth_obstacle_layer' not in plugins
                    or plugins.index('depth_obstacle_layer')
                    != plugins.index('inflation_layer') - 1):
                raise RuntimeError(
                    'new-base depth layer must run before inflation')
            depth_layer = costmap['depth_obstacle_layer']
            expected_layer = {
                'plugin': 'nav2_costmap_2d::VoxelLayer',
                'enabled': True,
                'combination_method': 1,
                'origin_z': 0.0,
                'z_resolution': 0.1,
                'z_voxels': 16,
                'mark_threshold': 0,
                'unknown_threshold': 15,
                'publish_voxel_map': False,
                'min_obstacle_height': -0.05,
                'max_obstacle_height': 1.5,
                'observation_sources': 'depth_marks depth_rays',
            }
            if any(depth_layer.get(key) != value
                   for key, value in expected_layer.items()):
                raise RuntimeError(
                    'new-base depth voxel layer violates approved policy')
            marks = depth_layer.get('depth_marks', {})
            rays = depth_layer.get('depth_rays', {})
            common = (
                marks.get('sensor_frame') == 'camera_color_optical_frame'
                and rays.get('sensor_frame') == 'camera_color_optical_frame'
                and marks.get('data_type') == 'PointCloud2'
                and rays.get('data_type') == 'PointCloud2')
            marking = (
                marks.get('topic') == '/depth_navigation/obstacles'
                and marks.get('marking') is True
                and marks.get('clearing') is False
                and marks.get('min_obstacle_height') == 0.05
                and marks.get('max_obstacle_height') == 1.5
                and marks.get('obstacle_min_range') == 0.4
                and marks.get('obstacle_max_range') == 2.5)
            clearing = (
                rays.get('topic') == '/depth_navigation/rays'
                and rays.get('marking') is False
                and rays.get('clearing') is True
                and rays.get('min_obstacle_height') == -0.05
                and rays.get('max_obstacle_height') == 1.5
                and rays.get('raytrace_min_range') == 0.4
                and rays.get('raytrace_max_range') == 2.5)
            if not (common and marking and clearing):
                raise RuntimeError(
                    'new-base depth costmap sources violate approved policy')

    monitor = params['collision_monitor']['ros__parameters']
    if monitor.get('base_frame_id') != 'base_footprint':
        raise RuntimeError('new-base collision monitor needs base_footprint')
    if not {'StopZone', 'SlowdownZone', 'FootprintApproach'} <= set(
            monitor['polygons']):
        raise RuntimeError('new-base collision polygons are incomplete')
    approved_sources = (['scan'], ['scan', 'depth_obstacles'])
    observation_sources = monitor.get('observation_sources')
    if all(depth_enabled) != (
            observation_sources == ['scan', 'depth_obstacles']):
        raise RuntimeError(
            'new-base depth costmaps and collision source must be enabled together')
    if observation_sources not in approved_sources:
        if (not isinstance(observation_sources, list)
                or 'scan' not in observation_sources):
            raise RuntimeError(
                'new-base scan collision source is not active')
        raise RuntimeError(
            'new-base collision sources must be scan with optional depth')
    for name in ('StopZone', 'SlowdownZone', 'FootprintApproach'):
        effective_sources = monitor[name].get(
            'sources_names', observation_sources)
        if effective_sources != observation_sources:
            raise RuntimeError(
                f'new-base {name} must observe the scan source and all '
                'approved sources')
    stop_zone = monitor['StopZone']
    if (stop_zone['type'] != 'velocity_polygon'
            or stop_zone['action_type'] != 'stop'
            or not stop_zone['enabled']
            or type(stop_zone.get('min_points')) is not int
            or stop_zone['min_points'] != 3
            or stop_zone.get('holonomic') is not False):
        raise RuntimeError('new-base StopZone is not active')
    expected_velocity_polygons = [
        'rotation', 'rotation_clockwise', 'translation_forward',
        'translation_backward', 'stopped']
    if stop_zone.get('velocity_polygons') != expected_velocity_polygons:
        raise RuntimeError(
            'new-base directional StopZone order is incomplete or unsafe')
    for name in expected_velocity_polygons:
        polygon = stop_zone[name]
        for minimum, maximum in (
                ('linear_min', 'linear_max'), ('theta_min', 'theta_max')):
            lower = polygon.get(minimum)
            upper = polygon.get(maximum)
            if (type(lower) not in (int, float)
                    or type(upper) not in (int, float)
                    or not math.isfinite(lower) or not math.isfinite(upper)
                    or lower > upper):
                raise RuntimeError(
                    f'new-base {name} {minimum}/{maximum} range is invalid')
        observed_range = (
            polygon['linear_min'], polygon['linear_max'],
            polygon['theta_min'], polygon['theta_max'])
        if ((name == 'rotation' and polygon['theta_min'] <= 0.0)
                or (name == 'rotation_clockwise'
                    and polygon['theta_max'] >= 0.0)):
            raise RuntimeError(
                'new-base rotation StopZone must exclude zero velocity')
        if observed_range != VELOCITY_POLICY_RANGES[name]:
            raise RuntimeError(
                f'new-base {name} velocity range violates approved policy')
    slow_zone = monitor['SlowdownZone']
    if (slow_zone['type'] != 'polygon'
            or slow_zone['action_type'] != 'slowdown'
            or not slow_zone['enabled']
            or type(slow_zone.get('min_points')) is not int
            or slow_zone['min_points'] != 3):
        raise RuntimeError('new-base SlowdownZone is not active')
    ratio = slow_zone.get('slowdown_ratio')
    if type(ratio) not in (int, float) or ratio != 0.6:
        raise RuntimeError('new-base slowdown ratio violates approved policy')
    approach = monitor['FootprintApproach']
    if (approach.get('type') != 'polygon'
            or approach['action_type'] != 'approach' or not approach['enabled']
            or type(approach.get('min_points')) is not int
            or approach['min_points'] != 3
            or approach.get('footprint_topic') !=
            '/local_costmap/published_footprint'):
        raise RuntimeError('new-base approach monitor is not active')
    for parameter, approved_value in (
            ('time_before_collision', 2.0),
            ('simulation_time_step', 0.1)):
        value = approach.get(parameter)
        if (type(value) not in (int, float) or not math.isfinite(value)
                or value != approved_value):
            raise RuntimeError(
                f'new-base approach {parameter} violates approved policy')
    if ('scan' not in monitor['observation_sources']
            or not monitor['scan']['enabled']
            or monitor['scan']['type'] != 'scan'
            or monitor['scan']['topic'] != '/scan'):
        raise RuntimeError('new-base scan collision source is not active')
    if observation_sources == ['scan', 'depth_obstacles']:
        depth = monitor.get('depth_obstacles', {})
        if (depth.get('type') != 'pointcloud'
                or depth.get('topic') != '/depth_navigation/obstacles'
                or depth.get('enabled') is not True
                or depth.get('source_timeout') != 1.0
                or depth.get('min_height') != 0.05
                or depth.get('max_height') != 1.5):
            raise RuntimeError(
                'new-base depth collision source violates approved policy')

    rotation_points = polygon_points(stop_zone['rotation']['points'])
    clockwise_points = polygon_points(
        stop_zone['rotation_clockwise']['points'])
    counterclockwise = stop_zone['rotation']
    clockwise = stop_zone['rotation_clockwise']
    if (counterclockwise['theta_min'] <= 0.0
            or clockwise['theta_max'] >= 0.0
            or clockwise_points != rotation_points):
        raise RuntimeError(
            'new-base rotation StopZone must exclude zero velocity')
    forward_stop = bounds(stop_zone['translation_forward']['points'])
    backward_stop = bounds(stop_zone['translation_backward']['points'])
    stopped_stop = bounds(stop_zone['stopped']['points'])
    slow = bounds(slow_zone['points'])
    if len(rotation_points) < 12:
        raise RuntimeError('new-base rotation StopZone is too coarse')
    signed_area_twice = sum(
        start[0] * end[1] - start[1] * end[0]
        for start, end in zip(
            rotation_points, rotation_points[1:] + rotation_points[:1]))
    if abs(signed_area_twice) <= 1e-9:
        raise RuntimeError('new-base rotation StopZone has zero area')
    orientation = 1.0 if signed_area_twice > 0.0 else -1.0
    origin_edge_distances = []
    for index, start in enumerate(rotation_points):
        end = rotation_points[(index + 1) % len(rotation_points)]
        edge_x = end[0] - start[0]
        edge_y = end[1] - start[1]
        edge_length = math.hypot(edge_x, edge_y)
        if edge_length <= 0.0:
            raise RuntimeError(
                'new-base rotation StopZone has duplicate points')
        if any(orientation * (
                edge_x * (point[1] - start[1])
                - edge_y * (point[0] - start[0])) < -1e-9
               for point in rotation_points):
            raise RuntimeError('new-base rotation StopZone must be convex')
        origin_edge_distances.append(orientation * (
            edge_y * start[0] - edge_x * start[1]) / edge_length)
    footprint_radius = max(
        math.hypot(x, y)
        for x in (footprint[0], footprint[1])
        for y in (-footprint[2], footprint[2]))
    if min(origin_edge_distances) < footprint_radius - 1e-6:
        raise RuntimeError(
            'new-base rotation StopZone misses the swept corner radius')
    if not (stopped_stop[0] > footprint[0]
            and stopped_stop[1] < footprint[1]
            and stopped_stop[2] > footprint[2]):
        raise RuntimeError('new-base StopZone does not contain footprint')
    if min(stopped_stop[0] - footprint[0],
           footprint[1] - stopped_stop[1],
           stopped_stop[2] - footprint[2]) < 0.05 - 1e-6:
        raise RuntimeError('new-base StopZone margin is below 0.05m')
    if not (forward_stop[0] - footprint[0] >= 0.05 - 1e-6
            and forward_stop[1] <= footprint[1]
            and abs(forward_stop[2] - footprint[2]) <= 1e-6):
        raise RuntimeError('new-base forward StopZone shape is invalid')
    if not (footprint[1] - backward_stop[1] >= 0.05 - 1e-6
            and backward_stop[0] >= footprint[0]
            and abs(backward_stop[2] - footprint[2]) <= 1e-6):
        raise RuntimeError('new-base backward StopZone shape is invalid')
    rotation_front = max(point[0] for point in rotation_points)
    rotation_rear = min(point[0] for point in rotation_points)
    rotation_half_width = max(abs(point[1]) for point in rotation_points)
    if not (slow[0] > max(stopped_stop[0], rotation_front)
            and slow[1] < min(stopped_stop[1], rotation_rear)
            and slow[2] > max(stopped_stop[2], rotation_half_width)):
        raise RuntimeError('new-base SlowdownZone does not contain StopZone')
    if (type(monitor.get('source_timeout')) not in (int, float)
            or monitor['source_timeout'] != 1.0):
        raise RuntimeError('new-base scan source timeout must be 1.0s')
    scan_timeout = monitor['scan'].get('source_timeout', 1.0)
    if (type(scan_timeout) not in (int, float) or scan_timeout != 1.0):
        raise RuntimeError(
            'new-base scan timeout override must be exactly 1.0s')
    if params['amcl']['ros__parameters']['set_initial_pose']:
        raise RuntimeError('new-base AMCL cannot force the old map origin')
    amcl_tf_tolerance_s = params['amcl']['ros__parameters'][
        'transform_tolerance']
    if (type(amcl_tf_tolerance_s) not in (int, float)
            or amcl_tf_tolerance_s != 1.0):
        raise RuntimeError('new-base AMCL transform tolerance must be 1.0s')
    smoother = params['velocity_smoother']['ros__parameters']
    if smoother['max_velocity'][0] > 0.08:
        raise RuntimeError('new-base forward speed exceeds uncalibrated limit')
    controller = params['controller_server']['ros__parameters']
    reverse = controller.get('ParkingReverse')
    minimum_velocity = smoother['min_velocity'][0]
    if (type(minimum_velocity) not in (int, float)
            or not math.isfinite(minimum_velocity)):
        raise RuntimeError('new-base reverse velocity must be finite')
    if reverse is None:
        if minimum_velocity < 0.0:
            raise RuntimeError('new-base autonomous profile cannot reverse')
    elif (not isinstance(reverse, dict)
          or 'ParkingReverse' not in controller.get('controller_plugins', [])
          or reverse.get('allow_reversing') is not True
          or reverse.get('use_rotate_to_heading') is not False
          or reverse.get('use_collision_detection') is not True
          or type(reverse.get('desired_linear_vel')) not in (int, float)
          or not math.isfinite(reverse['desired_linear_vel'])
          or not -0.08 <= minimum_velocity < 0.0
          or not math.isclose(minimum_velocity, -reverse['desired_linear_vel'],
                              abs_tol=1e-9)):
        raise RuntimeError('new-base reverse parking controller/smoother mismatch')
    progress = controller['progress_checker']
    if (progress['plugin'] != 'nav2_controller::PoseProgressChecker'
            or progress.get('required_movement_angle') != 0.10):
        raise RuntimeError('new-base progress checker must count rotation')
    controller_plugins = controller.get('controller_plugins')
    if (not isinstance(controller_plugins, list) or not controller_plugins
            or 'FollowPath' not in controller_plugins):
        raise RuntimeError(
            'new-base controller plugins must include FollowPath')
    supported_controller = (
        'nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController')
    for plugin_name in controller_plugins:
        if plugin_name not in controller:
            raise RuntimeError(
                f'new-base registered controller {plugin_name} is missing')
        plugin = controller[plugin_name]
        if plugin.get('plugin') != supported_controller:
            raise RuntimeError(
                f'new-base controller {plugin_name} plugin is unsupported')
        desired_velocity = plugin.get('desired_linear_vel')
        if (type(desired_velocity) not in (int, float)
                or not math.isfinite(desired_velocity)
                or not 0.0 < desired_velocity <= 0.08):
            raise RuntimeError(
                'new-base controller speed exceeds uncalibrated limit')
        if plugin.get('use_collision_detection') is not True:
            raise RuntimeError(
                f'new-base {plugin_name} collision detection must be active')
    if params['bt_navigator']['ros__parameters'][
            'robot_base_frame'] != 'base_footprint':
        raise RuntimeError('new-base BT needs base_footprint')
    return []
