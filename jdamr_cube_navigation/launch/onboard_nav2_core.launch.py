"""Launch the selected onboard navigation profile in a composed container."""

# Run the corridor Nav2 subset in a composed container and keep graph-loss
# detection separate.  Composition correlated with lower load in earlier
# runs, but it does not prove intra-process delivery: rclcpp defaults that
# option to false and this launch does not override it.  Run-specific evidence
# and remaining causal uncertainty live in evaluation/20260904_HANDOFF.md.

import hashlib
import json
import math
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.keepout_mask import validate_mask
from jdamr_cube_navigation.mobile_manipulator_protection import (
    load_base_obstacle_protection,
    load_mobile_manipulator_protection,
)
from jdamr_cube_navigation.nav2_liveness_guard import DEFAULT_REQUIRED
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
import yaml


REVISIT_REFERENCE = {
    'map': ('autonomous_20260826T161908.yaml',
            '3ddadf69e8f29a2ac4ec17f2d1bfc71f56cda0e805d65c792ddb5f5d46e0d652',
            'autonomous_20260826T161908.pgm',
            'ee9b0911f41a7da31a92eca67d261b2a96c286f6f49d65b3d6c884fb5937fc6e'),
    'mask': ('autonomous_20260826T161908_keepout_multi.yaml',
             '945b904f254864baab2a61f2cbf602cf69a1c8e8a71da1b7f07c0de84b9344f9',
             'autonomous_20260826T161908_keepout_multi.pgm',
             '40a99481046b2ad55fd4c5d3ad3e8b3a6d245fd7c808d32571f2828d4837df28'),
}


def _validate_keepout(context):
    """Reject missing or misaligned physical map inputs before Nav2 starts."""
    map_path = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('map').perform(context))))
    mask_path = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('keepout_mask').perform(context))))
    if not map_path.is_file():
        raise RuntimeError(f'saved map YAML not found: {map_path}')
    if not mask_path.is_file():
        raise RuntimeError(f'keepout mask YAML not found: {mask_path}')
    validate_mask(mask_path, map_path)
    return []


def _reject_legacy_profile_on_new_base(context):
    """Do not let the direct core launch bypass the physical wrapper."""
    if LaunchConfiguration('use_sim_time').perform(context).lower() in (
            '1', 'true', 'yes', 'on'):
        return []
    description_share = Path(get_package_share_directory(
        'jdamr_cube_description'))
    new_base_model = description_share / 'urdf' / 'new_base_real.urdf'
    if (new_base_model.is_file()
            and LaunchConfiguration('navigation_profile').perform(context)
            not in ('new_base_candidate', 'new_base_revisit_candidate')):
        raise RuntimeError('new physical base rejects legacy navigation profiles')
    return []


def _validate_revisit_reference(context):
    """Pin the old map and mask used only to collect a new autonomous scan bag."""
    for label, argument in (('map', 'map'), ('mask', 'keepout_mask')):
        path = Path(os.path.expanduser(
            LaunchConfiguration(argument).perform(context))).resolve()
        yaml_name, expected_yaml_hash, image_name, expected_hash = (
            REVISIT_REFERENCE[label])
        if path.name != yaml_name or not path.is_file():
            raise RuntimeError(f'new-base revisit {label} YAML is not pinned')
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected_yaml_hash:
            raise RuntimeError(f'new-base revisit {label} YAML hash mismatch')
        metadata = yaml.safe_load(path.read_text(encoding='utf-8'))
        image = Path(metadata['image'])
        if image.is_absolute() or image.name != image_name:
            raise RuntimeError(f'new-base revisit {label} image is not pinned')
        image_path = path.parent / image
        if (not image_path.is_file()
                or hashlib.sha256(image_path.read_bytes()).hexdigest()
                != expected_hash):
            raise RuntimeError(f'new-base revisit {label} image hash mismatch')
    return []


def _validate_revisit_isolation(context):
    """Guard the direct physical launch as well as the autorun entrypoint."""
    if LaunchConfiguration('navigation_profile').perform(context) != (
            'new_base_revisit_candidate'):
        return []
    if LaunchConfiguration('use_sim_time').perform(context).lower() in (
            '1', 'true', 'yes', 'on'):
        return []
    if os.environ.get('ROS_DOMAIN_ID') != '12':
        raise RuntimeError('new-base revisit requires physical ROS_DOMAIN_ID=12')
    description_share = Path(get_package_share_directory(
        'jdamr_cube_description'))
    if not (description_share / 'urdf' / 'new_base_real.urdf').is_file():
        raise RuntimeError('new-base revisit model is not installed')
    for service in ('jdamr-cartographer-session.service',
                    'jdamr-webteleop.service'):
        state = subprocess.run(['systemctl', 'is-active', '--quiet', service],
                               check=False, timeout=5)
        if state.returncode == 0:
            raise RuntimeError(f'new-base revisit rejects active {service}')
        if state.returncode not in (3, 4):
            raise RuntimeError(f'new-base revisit cannot verify {service}')
    nodes = subprocess.run(['ros2', 'node', 'list', '--no-daemon',
                            '--spin-time', '2'], check=False,
                           capture_output=True, text=True, timeout=15)
    if nodes.returncode != 0:
        raise RuntimeError('new-base revisit cannot read ROS nodes')
    if any('cartographer' in node or 'web_teleop' in node or node in (
            '/amcl', '/map_server', '/collision_monitor', '/bt_navigator')
           for node in nodes.stdout.splitlines()):
        raise RuntimeError('new-base revisit rejects preexisting control nodes')
    topics = subprocess.run(['ros2', 'topic', 'list', '--no-daemon',
                             '--spin-time', '2'], check=False,
                            capture_output=True, text=True, timeout=15)
    if topics.returncode != 0:
        raise RuntimeError('new-base revisit cannot read ROS topics')
    if '/cmd_vel' in topics.stdout.splitlines():
        command = None
        for _ in range(3):
            try:
                command = subprocess.run(
                    ['ros2', 'topic', 'info', '--no-daemon', '--spin-time',
                     '3', '/cmd_vel'], check=False, capture_output=True,
                    text=True, timeout=20)
            except subprocess.TimeoutExpired:
                continue
            if command.returncode == 0 and 'Publisher count:' in command.stdout:
                break
        if command is None or command.returncode != 0 or (
                'Publisher count: 0' not in command.stdout):
            raise RuntimeError('new-base revisit rejects existing cmd_vel publisher')
    return []


def _shutdown_unless_already_stopping(reason):
    """Fail closed on a required process exit without duplicate shutdowns."""
    def handler(_event, context):
        if context.is_shutdown:
            return []
        return [Shutdown(reason=reason)]

    return handler


def _validate_new_base_params(context, revisit=False):
    """Reject old footprint/StopZone parameters for the new physical base."""
    map_name = Path(LaunchConfiguration('map').perform(context)).name
    mask_name = Path(LaunchConfiguration('keepout_mask').perform(context)).name
    if revisit:
        _validate_revisit_reference(context)
    elif not (map_name.startswith('new_base_')
              and mask_name.startswith('new_base_')):
        raise RuntimeError('new-base profile requires a new-base map and mask')
    params_path = Path(os.path.expanduser(
        LaunchConfiguration('params_file').perform(context)))
    params = yaml.safe_load(params_path.read_text(encoding='utf-8'))
    geometry_path = (Path(get_package_share_directory('jdamr_cube_description'))
                     / 'config' / 'new_base_geometry.yaml')
    geometry = yaml.safe_load(geometry_path.read_text(encoding='utf-8'))
    front = geometry['front_to_wheel_axis']['value']
    rear = front - geometry['frame_length']['value']
    half_width = geometry['wheel_outer_width']['value'] / 2.0

    def polygon_points(value):
        polygon = json.loads(value) if isinstance(value, str) else value
        if not isinstance(polygon, list) or len(polygon) < 3:
            raise RuntimeError('new-base polygon needs at least three points')
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
            raise RuntimeError('new-base polygon must cover both sides as a rectangle')
        ordered_corners = [
            (xs[1], ys[1]), (xs[1], ys[0]),
            (xs[0], ys[0]), (xs[0], ys[1]),
        ]
        if [tuple(point) for point in polygon] != ordered_corners:
            raise RuntimeError('new-base polygon corners must follow the perimeter')
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
        if (not layer['enabled'] or 'scan' not in layer['observation_sources']
                or scan['topic'] != '/scan' or not scan['marking']):
            raise RuntimeError('new-base costmap scan layer is not active')

    monitor = params['collision_monitor']['ros__parameters']
    if not {'StopZone', 'SlowdownZone', 'FootprintApproach'} <= set(
            monitor['polygons']):
        raise RuntimeError('new-base collision polygons are incomplete')
    stop_zone = monitor['StopZone']
    if (stop_zone['type'] != 'velocity_polygon'
            or stop_zone['action_type'] != 'stop'
            or not stop_zone['enabled'] or stop_zone['min_points'] < 3
            or stop_zone.get('holonomic') is not False):
        raise RuntimeError('new-base StopZone is not active')
    expected_velocity_polygons = {
        'rotation', 'rotation_clockwise', 'translation_forward',
        'translation_backward', 'stopped'}
    if (set(stop_zone.get('velocity_polygons', [])) !=
            expected_velocity_polygons):
        raise RuntimeError('new-base directional StopZone is incomplete')
    slow_zone = monitor['SlowdownZone']
    if (slow_zone['type'] != 'polygon'
            or slow_zone['action_type'] != 'slowdown'
            or not slow_zone['enabled'] or slow_zone['min_points'] < 3):
        raise RuntimeError('new-base SlowdownZone is not active')
    if (monitor['FootprintApproach']['action_type'] != 'approach'
            or not monitor['FootprintApproach']['enabled']):
        raise RuntimeError('new-base approach monitor is not active')
    if ('scan' not in monitor['observation_sources']
            or not monitor['scan']['enabled']
            or monitor['scan']['type'] != 'scan'
            or monitor['scan']['topic'] != '/scan'):
        raise RuntimeError('new-base scan collision source is not active')
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
    slow = bounds(monitor['SlowdownZone']['points'])
    if len(rotation_points) < 12:
        raise RuntimeError('new-base rotation StopZone is too coarse')
    edge_distances = []
    turn_sign = None
    for index, start in enumerate(rotation_points):
        end = rotation_points[(index + 1) % len(rotation_points)]
        following = rotation_points[(index + 2) % len(rotation_points)]
        edge_length = math.hypot(end[0] - start[0], end[1] - start[1])
        if edge_length <= 0.0:
            raise RuntimeError(
                'new-base rotation StopZone has duplicate points')
        edge_distances.append(abs(
            start[0] * end[1] - start[1] * end[0]) / edge_length)
        turn = ((end[0] - start[0]) * (following[1] - end[1])
                - (end[1] - start[1]) * (following[0] - end[0]))
        if abs(turn) <= 1e-9:
            continue
        current_sign = 1 if turn > 0.0 else -1
        if turn_sign is None:
            turn_sign = current_sign
        elif turn_sign != current_sign:
            raise RuntimeError('new-base rotation StopZone must be convex')
    footprint_radius = max(
        math.hypot(x, y)
        for x in (footprint[0], footprint[1])
        for y in (-footprint[2], footprint[2]))
    if min(edge_distances) < footprint_radius - 1e-6:
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
    if monitor['source_timeout'] != 1.0:
        raise RuntimeError('new-base scan source timeout must be 1.0s')
    if params['amcl']['ros__parameters']['set_initial_pose']:
        raise RuntimeError('new-base AMCL cannot force the old map origin')
    amcl_tf_tolerance_s = params['amcl']['ros__parameters'][
        'transform_tolerance']
    if (type(amcl_tf_tolerance_s) not in (int, float) or
            amcl_tf_tolerance_s != 1.0):
        raise RuntimeError('new-base AMCL transform tolerance must be 1.0s')
    if params['velocity_smoother']['ros__parameters']['max_velocity'][0] > 0.08:
        raise RuntimeError('new-base forward speed exceeds uncalibrated limit')
    minimum_linear_velocity = params['velocity_smoother'][
        'ros__parameters']['min_velocity'][0]
    if minimum_linear_velocity < 0.0:
        raise RuntimeError('new-base autonomous profile cannot reverse')
    progress = params['controller_server']['ros__parameters'][
        'progress_checker']
    if (progress['plugin'] != 'nav2_controller::PoseProgressChecker'
            or progress.get('required_movement_angle') != 0.10):
        raise RuntimeError('new-base progress checker must count rotation')
    if params['controller_server']['ros__parameters']['FollowPath'][
            'desired_linear_vel'] > 0.08:
        raise RuntimeError('new-base controller speed exceeds uncalibrated limit')
    if params['bt_navigator']['ros__parameters'][
            'robot_base_frame'] != 'base_footprint':
        raise RuntimeError('new-base BT needs base_footprint')
    return []


def _launch_navigation(context):
    """Resolve the profile before selecting its BT and required components."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    profile = LaunchConfiguration('navigation_profile').perform(context)
    behavior_tree = {
        'corridor': 'navigate_to_pose_corridor_fail_fast.xml',
        'obstacle_candidate': 'navigate_to_pose_dynamic_obstacle_eval.xml',
        'obstacle_base_candidate': (
            'navigate_to_pose_dynamic_obstacle_eval.xml'),
        'new_base_candidate': 'navigate_to_pose_dynamic_obstacle_eval.xml',
        'new_base_revisit_candidate': (
            'navigate_to_pose_dynamic_obstacle_eval.xml'),
    }[profile]
    if profile in ('new_base_candidate', 'new_base_revisit_candidate'):
        _validate_new_base_params(
            context, revisit=profile == 'new_base_revisit_candidate')
    map_yaml = LaunchConfiguration('map')
    keepout_mask = LaunchConfiguration('keepout_mask')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    selected_bt = os.path.join(package_share, 'behavior_trees', behavior_tree)
    protection = None
    if profile == 'obstacle_candidate':
        protection = load_mobile_manipulator_protection(Path(
            package_share) / 'config' / 'mobile_manipulator_protection.yaml')
    elif profile == 'obstacle_base_candidate':
        protection = load_base_obstacle_protection(Path(
            package_share) / 'config' / 'base_obstacle_protection.yaml')

    param_rewrites = {
        'default_nav_to_pose_bt_xml': selected_bt,
        'yaml_filename': map_yaml,
        ('local_costmap.local_costmap.ros__parameters.'
         'keepout_filter.enabled'): 'true',
        ('global_costmap.global_costmap.ros__parameters.'
         'keepout_filter.enabled'): 'true',
        'use_sim_time': use_sim_time,
    }
    if profile == 'new_base_revisit_candidate':
        # Resume starts at the observed physical pose, not the old home.
        param_rewrites.update({
            'amcl.ros__parameters.set_initial_pose': 'true',
            'amcl.ros__parameters.initial_pose.x':
                LaunchConfiguration('revisit_initial_x'),
            'amcl.ros__parameters.initial_pose.y':
                LaunchConfiguration('revisit_initial_y'),
            'amcl.ros__parameters.initial_pose.yaw':
                LaunchConfiguration('revisit_initial_yaw'),
        })
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_rewrites,
            convert_types=True,
        ),
        allow_substs=True,
    )
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_bond = {
        # Allow lifecycle service and bond handling to tolerate scheduler
        # jitter.  The executable threshold is covered by launch tests.
        'bond_timeout': 10.0,
        'bond_respawn_max_duration': 20.0,
    }

    def composable(package, plugin, name, parameters, remap=True):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[*parameters, {'use_sim_time': use_sim_time}],
            remappings=remappings if remap else [],
        )

    keepout_components = [
        composable('nav2_map_server', 'nav2_map_server::MapServer',
                   'keepout_filter_mask_server', [{
                       'use_sim_time': use_sim_time,
                       'yaml_filename': keepout_mask,
                       'topic_name': '/keepout_filter_mask',
                       'frame_id': 'map',
                   }], remap=False),
        composable('nav2_map_server',
                   'nav2_map_server::CostmapFilterInfoServer',
                   'keepout_costmap_filter_info_server', [{
                       'use_sim_time': use_sim_time,
                       'type': 0,
                       'filter_info_topic': '/keepout_costmap_filter_info',
                       'mask_topic': '/keepout_filter_mask',
                       'base': 0.0,
                       'multiplier': 1.0,
                   }], remap=False),
    ]
    collision_monitor_parameters = [configured_params]
    if protection is not None:
        collision_monitor_parameters.append(
            protection['collision_monitor_overrides'])
    nav2_components = [
        composable('nav2_map_server', 'nav2_map_server::MapServer',
                   'map_server', [configured_params,
                                  {'yaml_filename': map_yaml}]),
        composable('nav2_amcl', 'nav2_amcl::AmclNode', 'amcl',
                   [configured_params]),
        ComposableNode(
            package='nav2_controller',
            plugin='nav2_controller::ControllerServer',
            name='controller_server',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        composable('nav2_planner', 'nav2_planner::PlannerServer',
                   'planner_server', [configured_params]),
        ComposableNode(
            package='nav2_velocity_smoother',
            plugin='nav2_velocity_smoother::VelocitySmoother',
            name='velocity_smoother',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        composable('nav2_collision_monitor',
                   'nav2_collision_monitor::CollisionMonitor',
                   'collision_monitor', collision_monitor_parameters),
        composable('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator',
                   'bt_navigator', [configured_params]),
    ]
    navigation_nodes = [
        'controller_server', 'planner_server', 'velocity_smoother',
        'collision_monitor', 'bt_navigator',
    ]
    required_nodes = list(DEFAULT_REQUIRED)
    if profile in {'obstacle_candidate', 'obstacle_base_candidate',
                   'new_base_candidate', 'new_base_revisit_candidate'}:
        # The candidate BT calls Wait during bounded recovery.  Load only
        # that plugin; selecting this profile must not enable spin or backup.
        nav2_components.insert(-1, ComposableNode(
            package='nav2_behaviors',
            plugin='behavior_server::BehaviorServer',
            name='behavior_server',
            parameters=[configured_params, {
                'behavior_plugins': ['wait'], 'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ))
        navigation_nodes.insert(-1, 'behavior_server')
        required_nodes.append('behavior_server')

    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        namespace='',
        # Costmaps are child nodes created inside planner/controller, not
        # LoadNode targets.  They inherit process-level ROS arguments, so
        # component parameters alone silently leave them on Nav2 defaults.
        # RewrittenYaml only substitutes existing leaf keys.  The production
        # YAML omits use_sim_time, so pass it explicitly to components and
        # their child nodes instead of relying on that rewrite alone.
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
        composable_node_descriptions=keepout_components + nav2_components,
        output='screen',
    )

    keepout_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'keepout_filter_mask_server',
                'keepout_costmap_filter_info_server',
            ],
            **lifecycle_bond,
        }],
    )
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
            **lifecycle_bond,
        }],
    )
    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': navigation_nodes,
            **lifecycle_bond,
        }],
    )

    # The container process surviving is not evidence that the nodes inside it
    # are alive, so the graph-level guard keeps that failure observable while
    # Nav2 remains composed.
    liveness_guard = Node(
        package='jdamr_cube_navigation',
        executable='nav2_liveness_guard',
        name='nav2_liveness_guard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--required', ','.join(required_nodes)],
    )

    required_processes = [
        container,
        keepout_lifecycle,
        localization_lifecycle,
        navigation_lifecycle,
        liveness_guard,
    ]
    required_exit_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=_shutdown_unless_already_stopping(
                'required Nav2 process exited; stopping navigation')))
        for process in required_processes
    ]

    return [*required_exit_handlers, *required_processes]


def generate_launch_description():
    """Keep the proven corridor profile as the default onboard launch."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    discovery_range = LaunchConfiguration('discovery_range')
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE', discovery_range),
        DeclareLaunchArgument(
            'discovery_range', default_value='LOCALHOST',
            choices=['LOCALHOST', 'SUBNET'],
            description=(
                'The physical wrapper selects SUBNET to consume LOCALHOST '
                'sensor publishers on this host')),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908.yaml')),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908_keepout_multi.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('revisit_initial_x', default_value='0.0'),
        DeclareLaunchArgument('revisit_initial_y', default_value='-0.1'),
        DeclareLaunchArgument('revisit_initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'navigation_profile', default_value='corridor',
            choices=[
                'corridor', 'obstacle_candidate',
                'obstacle_base_candidate', 'new_base_candidate',
                'new_base_revisit_candidate'],
            description='Candidate enables online replanning and Wait recovery'),
        OpaqueFunction(function=_reject_legacy_profile_on_new_base),
        OpaqueFunction(function=_validate_revisit_isolation),
        OpaqueFunction(function=_validate_keepout),
        OpaqueFunction(function=_launch_navigation),
    ])
