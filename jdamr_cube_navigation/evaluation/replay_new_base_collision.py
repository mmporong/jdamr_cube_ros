#!/usr/bin/env python3
"""Offline, read-only replay of new-base Collision Monitor geometry."""

import argparse
from collections import Counter, deque
import csv
import hashlib
import json
import math
from pathlib import Path

import yaml


TOPICS = (
    '/scan', '/tf_static', '/cmd_vel_smoothed', '/cmd_vel',
    '/collision_monitor_state',
)
UPSTREAM_VELOCITY_POLYGON = (
    'https://github.com/ros-navigation/navigation2/blob/1.3.12/'
    'nav2_collision_monitor/src/velocity_polygon.cpp'
)
UPSTREAM_VELOCITY_POLYGON_SHA256 = (
    'bb52c60f8c42c04bccf432c1735bbfb191b2174ec155944145fe03b8feead1e3')


def stamp_ns(stamp):
    """Convert a ROS builtin time to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def sha256_file(path):
    """Hash a file without loading it fully into memory."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def parse_polygon(value):
    """Parse a Nav2 polygon parameter into float point tuples."""
    points = yaml.safe_load(value) if isinstance(value, str) else value
    return [(float(point[0]), float(point[1])) for point in points]


def point_in_polygon(point, polygon):
    """Reproduce Jazzy Polygon::isPointInside ray crossings exactly."""
    x, y = point
    inside = False
    previous_x, previous_y = polygon[-1]
    for current_x, current_y in polygon:
        if (y <= previous_y) == (y > current_y):
            crossing_x = previous_x + (
                (y - previous_y) * (current_x - previous_x)
                / (current_y - previous_y))
            if crossing_x > x:
                inside = not inside
        previous_x, previous_y = current_x, current_y
    return inside


def rectangle_clearance(point, bounds):
    """Signed point-to-rectangle clearance; negative means inside."""
    x, y = point
    xmin, xmax, ymin, ymax = bounds
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    if dx or dy:
        return math.hypot(dx, dy)
    return -min(x - xmin, xmax - x, y - ymin, ymax - y)


def _quat_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quat_normalize(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm == 0.0:
        raise ValueError('zero-length TF quaternion')
    return tuple(value / norm for value in quaternion)


def _rotate(quaternion, point):
    q = _quat_normalize(quaternion)
    vector = (point[0], point[1], point[2], 0.0)
    conjugate = (-q[0], -q[1], -q[2], q[3])
    rotated = _quat_multiply(_quat_multiply(q, vector), conjugate)
    return rotated[:3]


def compose_transform(left, right):
    """Compose target<-middle with middle<-source."""
    lt, lq = left
    rt, rq = right
    rotated = _rotate(lq, rt)
    return (
        tuple(lt[index] + rotated[index] for index in range(3)),
        _quat_normalize(_quat_multiply(lq, rq)),
    )


def inverse_transform(transform):
    """Invert a translation-quaternion rigid transform."""
    translation, quaternion = transform
    q = _quat_normalize(quaternion)
    inverse_q = (-q[0], -q[1], -q[2], q[3])
    inverse_t = _rotate(inverse_q, tuple(-value for value in translation))
    return inverse_t, inverse_q


def apply_transform(transform, point):
    """Apply a translation-quaternion transform to a 3-D point."""
    translation, quaternion = transform
    rotated = _rotate(quaternion, point)
    return tuple(translation[index] + rotated[index] for index in range(3))


def resolve_transform(transforms, source_frame, target_frame):
    """Resolve target<-source from recorded static transforms."""
    graph = {}
    child_parents = {}
    pair_transforms = {}
    for item in transforms:
        parent = item.header.frame_id.lstrip('/')
        child = item.child_frame_id.lstrip('/')
        translation = item.transform.translation
        rotation = item.transform.rotation
        parent_from_child = (
            (float(translation.x), float(translation.y), float(translation.z)),
            (float(rotation.x), float(rotation.y), float(rotation.z),
             float(rotation.w)),
        )
        values = parent_from_child[0] + parent_from_child[1]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f'non-finite static TF for {parent} <- {child}')
        parent_from_child = (
            parent_from_child[0], _quat_normalize(parent_from_child[1]))
        if child in child_parents and child_parents[child] != parent:
            raise ValueError(
                f'conflicting static TF parents for {child}: '
                f'{child_parents[child]} and {parent}')
        child_parents[child] = parent
        key = (parent, child)
        if key in pair_transforms:
            old_translation, old_quaternion = pair_transforms[key]
            new_translation, new_quaternion = parent_from_child
            same_translation = all(
                math.isclose(old, new, abs_tol=1.0e-12)
                for old, new in zip(old_translation, new_translation))
            quaternion_dot = abs(sum(
                old * new for old, new
                in zip(old_quaternion, new_quaternion)))
            if not same_translation or not math.isclose(
                    quaternion_dot, 1.0, abs_tol=1.0e-12):
                raise ValueError(
                    f'conflicting duplicate static TF for {parent} <- {child}')
            continue
        pair_transforms[key] = parent_from_child
        graph.setdefault(child, []).append((parent, parent_from_child))
        graph.setdefault(parent, []).append(
            (child, inverse_transform(parent_from_child)))
    source = source_frame.lstrip('/')
    target = target_frame.lstrip('/')
    identity = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    queue = deque([(source, identity, [source])])
    visited = {source}
    while queue:
        frame, frame_from_source, chain = queue.popleft()
        if frame == target:
            return frame_from_source, chain
        for neighbor, neighbor_from_frame in graph.get(frame, []):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((
                    neighbor,
                    compose_transform(neighbor_from_frame, frame_from_source),
                    chain + [neighbor],
                ))
    raise ValueError(f'no static TF chain from {source!r} to {target!r}')


def select_velocity_polygon(stop_zone, linear_x, angular_z):
    """Reproduce Jazzy's ordered, inclusive velocity range matching."""
    for name in stop_zone['velocity_polygons']:
        candidate = stop_zone[name]
        if (float(candidate['linear_min']) <= linear_x
                <= float(candidate['linear_max'])
                and float(candidate['theta_min']) <= angular_z
                <= float(candidate['theta_max'])):
            return name
    return None


def old_rotation_matching(stop_zone):
    """Return the requested hypothetical prior zero-matching geometry."""
    copied = json.loads(json.dumps(stop_zone))
    copied['rotation']['linear_min'] = -0.005
    copied['rotation']['linear_max'] = 0.005
    copied['rotation']['theta_min'] = -1.0
    copied['rotation']['theta_max'] = 1.0
    return copied


def actual_body_bounds(geometry):
    """Derive the measured lower-base rectangular body envelope."""
    front = float(geometry['front_to_wheel_axis']['value'])
    rear = front - float(geometry['frame_length']['value'])
    half_width = float(geometry['wheel_outer_width']['value']) / 2.0
    return rear, front, -half_width, half_width


def rectangle_bounds(points):
    """Return axis-aligned bounds for rectangle points."""
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), max(xs), min(ys), max(ys)


def scan_points(scan, transform):
    """Filter valid LaserScan ranges and transform them to base points."""
    points = []
    rejected = Counter()
    for index, range_m in enumerate(scan.ranges):
        value = float(range_m)
        if not math.isfinite(value):
            rejected['non_finite'] += 1
            continue
        if value < float(scan.range_min) or value > float(scan.range_max):
            rejected['outside_valid_range'] += 1
            continue
        angle = float(scan.angle_min) + index * float(scan.angle_increment)
        laser_point = (value * math.cos(angle), value * math.sin(angle), 0.0)
        base_point = apply_transform(transform, laser_point)
        if not all(math.isfinite(coordinate) for coordinate in base_point):
            rejected['non_finite_transform'] += 1
            continue
        points.append((base_point[0], base_point[1]))
    return points, rejected


def _open_reader(bag_path, topics):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        metadata.name: metadata.type
        for metadata in reader.get_all_topics_and_types()
    }
    missing = sorted(set(topics) - set(topic_types))
    if missing:
        raise ValueError(f'bag is missing required topics: {missing}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader, topic_types


def _deserialize(data, type_name):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    return deserialize_message(data, get_message(type_name))


def _installed_collision_monitor_version():
    from ament_index_python.packages import get_package_share_directory
    from xml.etree import ElementTree

    package_xml = (
        Path(get_package_share_directory('nav2_collision_monitor'))
        / 'package.xml')
    return ElementTree.parse(package_xml).getroot().findtext('version')


def _load_static_tf(bag_path):
    reader, topic_types = _open_reader(bag_path, ('/tf_static',))
    transforms = []
    while reader.has_next():
        topic, data, _log_time_ns = reader.read_next()
        message = _deserialize(data, topic_types[topic])
        transforms.extend(message.transforms)
    if not transforms:
        raise ValueError('bag has no recorded static transforms')
    return transforms


def replay(bag_path, params_path, geometry_path, output_dir):
    """Replay one bag offline and write per-scan and summary artifacts."""
    bag_path = Path(bag_path).resolve()
    params_path = Path(params_path).resolve()
    geometry_path = Path(geometry_path).resolve()
    output_dir = Path(output_dir).resolve()
    params = yaml.safe_load(params_path.read_text(encoding='utf-8'))
    geometry = yaml.safe_load(geometry_path.read_text(encoding='utf-8'))
    monitor = params['collision_monitor']['ros__parameters']
    stop_zone = monitor['StopZone']
    old_stop_zone = old_rotation_matching(stop_zone)
    padded_points = parse_polygon(
        params['local_costmap']['local_costmap']['ros__parameters'][
            'footprint'])
    actual_bounds = actual_body_bounds(geometry)
    padded_bounds = rectangle_bounds(padded_points)

    transforms = _load_static_tf(bag_path)
    reader, topic_types = _open_reader(
        bag_path,
        ('/scan', '/cmd_vel_smoothed', '/cmd_vel',
         '/collision_monitor_state'),
    )
    transform = None
    tf_chain = None
    scan_frame = None
    latest_input = None
    current_zone_state = None
    old_zone_state = None
    latest_output = None
    latest_state = None
    rows = []
    aggregate = Counter()
    zone_counts = Counter()
    old_zone_counts = Counter()
    state_counts = Counter()
    rejected_ranges = Counter()
    range_contracts = set()
    current_callback_zones = Counter()
    old_callback_zones = Counter()
    max_input_age_s = None
    velocity_timeout_s = float(
        params['velocity_smoother']['ros__parameters']['velocity_timeout'])

    while reader.has_next():
        topic, data, log_time_ns = reader.read_next()
        message = _deserialize(data, topic_types[topic])
        if topic == '/cmd_vel_smoothed':
            latest_input = (
                int(log_time_ns), float(message.linear.x),
                float(message.angular.z))
            matched_zone = select_velocity_polygon(
                stop_zone, latest_input[1], latest_input[2])
            old_matched_zone = select_velocity_polygon(
                old_stop_zone, latest_input[1], latest_input[2])
            if matched_zone is not None:
                current_zone_state = matched_zone
                current_callback_zones[matched_zone] += 1
            else:
                aggregate['current_uncovered_cmd_callbacks'] += 1
            if old_matched_zone is not None:
                old_zone_state = old_matched_zone
                old_callback_zones[old_matched_zone] += 1
            else:
                aggregate['old_uncovered_cmd_callbacks'] += 1
            continue
        if topic == '/cmd_vel':
            latest_output = (
                int(log_time_ns), float(message.linear.x),
                float(message.angular.z))
            continue
        if topic == '/collision_monitor_state':
            latest_state = (
                int(log_time_ns), int(message.action_type),
                str(message.polygon_name))
            state_counts[f'{message.action_type}:{message.polygon_name}'] += 1
            continue

        frame = message.header.frame_id.lstrip('/')
        if transform is None:
            transform, tf_chain = resolve_transform(
                transforms, frame, monitor['base_frame_id'])
            scan_frame = frame
        elif frame != scan_frame:
            raise ValueError(
                f'scan frame changed from {scan_frame!r} to {frame!r}')
        points, rejected = scan_points(message, transform)
        rejected_ranges.update(rejected)
        range_contracts.add(
            (float(message.range_min), float(message.range_max)))
        actual_count = sum(
            point_in_polygon(point, [
                (actual_bounds[1], actual_bounds[3]),
                (actual_bounds[1], actual_bounds[2]),
                (actual_bounds[0], actual_bounds[2]),
                (actual_bounds[0], actual_bounds[3]),
            ]) for point in points)
        padded_count = sum(point_in_polygon(point, padded_points)
                           for point in points)
        actual_clearance = min(
            (rectangle_clearance(point, actual_bounds) for point in points),
            default=None)
        padded_clearance = min(
            (rectangle_clearance(point, padded_bounds) for point in points),
            default=None)

        if latest_input is None:
            input_age_s = None
            selected_name = None
            old_selected_name = None
            selected_count = None
            old_selected_count = None
            aggregate['unknown_input_scans'] += 1
        else:
            input_age_s = (int(log_time_ns) - latest_input[0]) / 1.0e9
            max_input_age_s = max(input_age_s, max_input_age_s or 0.0)
            selected_name = current_zone_state
            old_selected_name = old_zone_state
            selected_polygon = (
                parse_polygon(stop_zone[selected_name]['points'])
                if selected_name else None)
            old_selected_polygon = (
                parse_polygon(old_stop_zone[old_selected_name]['points'])
                if old_selected_name else None)
            selected_count = (sum(point_in_polygon(point, selected_polygon)
                                  for point in points)
                              if selected_polygon else None)
            old_selected_count = (
                sum(point_in_polygon(point, old_selected_polygon)
                    for point in points)
                if old_selected_polygon else None)
            zone_counts[selected_name or 'uncovered'] += 1
            old_zone_counts[old_selected_name or 'uncovered'] += 1
            aggregate['current_stop_scans'] += int(
                selected_count is not None
                and selected_count >= int(stop_zone['min_points']))
            aggregate['old_hypothetical_stop_scans'] += int(
                old_selected_count is not None
                and old_selected_count >= int(stop_zone['min_points']))
            aggregate['geometry_selection_changed_scans'] += int(
                selected_name != old_selected_name)
            aggregate['stop_decision_changed_scans'] += int(
                (selected_count is not None
                 and selected_count >= int(stop_zone['min_points']))
                != (old_selected_count is not None
                    and old_selected_count >= int(stop_zone['min_points'])))
            input_fresh = input_age_s <= velocity_timeout_s
            aggregate['fresh_input_scans'] += int(input_fresh)
            aggregate['fresh_current_stop_scans'] += int(
                input_fresh and selected_count is not None
                and selected_count >= int(stop_zone['min_points']))
            aggregate['fresh_old_hypothetical_stop_scans'] += int(
                input_fresh and old_selected_count is not None
                and old_selected_count >= int(stop_zone['min_points']))

        if latest_input is None:
            input_fresh = None

        output_age_s = ((int(log_time_ns) - latest_output[0]) / 1.0e9
                        if latest_output else None)
        state_age_s = ((int(log_time_ns) - latest_state[0]) / 1.0e9
                       if latest_state else None)
        rows.append({
            'stamp_ns': stamp_ns(message.header.stamp),
            'log_time_ns': int(log_time_ns),
            'input_age_s': input_age_s,
            'input_fresh_within_velocity_smoother_timeout': (
                int(input_fresh) if input_fresh is not None else None),
            'input_linear_x': latest_input[1] if latest_input else None,
            'input_angular_z': latest_input[2] if latest_input else None,
            'selected_zone': selected_name or 'unknown',
            'selected_zone_count': selected_count,
            'selected_zone_stop': (
                int(selected_count >= int(stop_zone['min_points']))
                if selected_count is not None else None),
            'old_hypothetical_zone': old_selected_name or 'unknown',
            'old_hypothetical_count': old_selected_count,
            'valid_scan_points': len(points),
            'actual_body_count': actual_count,
            'padded_footprint_count': padded_count,
            'actual_body_clearance_m': actual_clearance,
            'padded_footprint_clearance_m': padded_clearance,
            'output_age_s_observation_only': output_age_s,
            'output_linear_x_observation_only': (
                latest_output[1] if latest_output else None),
            'output_angular_z_observation_only': (
                latest_output[2] if latest_output else None),
            'recorded_state_age_s': state_age_s,
            'recorded_action_type': latest_state[1] if latest_state else None,
            'recorded_polygon_name': latest_state[2] if latest_state else None,
        })
        aggregate['scan_count'] += 1
        aggregate['valid_scan_points'] += len(points)
        aggregate['actual_body_point_hits'] += actual_count
        aggregate['padded_footprint_point_hits'] += padded_count

    if not rows:
        raise ValueError('bag has no scan messages')

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / 'per_scan.csv'
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    mcap_files = sorted(bag_path.glob('*.mcap'))
    chair = (0.29, -0.30)
    chair_actual = rectangle_clearance(chair, actual_bounds)
    chair_padded = rectangle_clearance(chair, padded_bounds)
    summary = {
        'analysis_kind': 'offline_counterfactual_geometry_replay',
        'ros_publish_performed': False,
        'selection_input': '/cmd_vel_smoothed receive/log time',
        'selection_excludes': '/cmd_vel output (observation only)',
        'actual_recorded_state': {
            'topic': '/collision_monitor_state',
            'event_counts': dict(sorted(state_counts.items())),
            'kept_distinct_from_counterfactual_geometry': True,
        },
        'current_counterfactual': {
            'params_path': str(params_path),
            'params_sha256': sha256_file(params_path),
            'velocity_polygon_order': list(stop_zone['velocity_polygons']),
            'range_semantics': (
                'ordered first match; inclusive min/max; an uncovered command '
                'retains the previously selected polygon'),
            'upstream_source': UPSTREAM_VELOCITY_POLYGON,
            'upstream_source_sha256': UPSTREAM_VELOCITY_POLYGON_SHA256,
            'installed_nav2_collision_monitor_version': (
                _installed_collision_monitor_version()),
            'selection_unit': (
                'per-scan point classification under polygon state retained '
                'from cmd callbacks; not a count of interventions or '
                'callbacks'),
            'cmd_callback_selected_zone_counts': dict(
                sorted(current_callback_zones.items())),
            'selected_zone_scan_counts': dict(sorted(zone_counts.items())),
            'stop_scan_count': aggregate['current_stop_scans'],
            'fresh_stop_scan_count': aggregate['fresh_current_stop_scans'],
        },
        'old_rotation_zero_matching_hypothetical': {
            'hypothetical_not_recorded_actual_state': True,
            'change': {
                'rotation.linear_min': -0.005,
                'rotation.linear_max': 0.005,
                'rotation.theta_min': -1.0,
                'rotation.theta_max': 1.0,
            },
            'selected_zone_scan_counts': dict(sorted(old_zone_counts.items())),
            'cmd_callback_selected_zone_counts': dict(
                sorted(old_callback_zones.items())),
            'stop_scan_count': aggregate['old_hypothetical_stop_scans'],
            'fresh_stop_scan_count': aggregate[
                'fresh_old_hypothetical_stop_scans'],
            'selection_changed_scans': aggregate[
                'geometry_selection_changed_scans'],
            'stop_decision_changed_scans': aggregate[
                'stop_decision_changed_scans'],
        },
        'bag': {
            'path': str(bag_path),
            'metadata_sha256': sha256_file(bag_path / 'metadata.yaml'),
            'mcap_sha256': {
                file.name: sha256_file(file) for file in mcap_files
            },
        },
        'geometry': {
            'source_path': str(geometry_path),
            'source_sha256': sha256_file(geometry_path),
            'actual_body_rectangle_m': {
                'xmin': actual_bounds[0], 'xmax': actual_bounds[1],
                'ymin': actual_bounds[2], 'ymax': actual_bounds[3],
                'derivation': (
                    'front_to_wheel_axis and frame_length; lateral envelope '
                    'uses wheel_outer_width'),
            },
            'padded_footprint_rectangle_m': {
                'xmin': padded_bounds[0], 'xmax': padded_bounds[1],
                'ymin': padded_bounds[2], 'ymax': padded_bounds[3],
            },
            'chair_reference_audit': {
                'point_base_m': list(chair),
                'ahead_of_padded_front_m': chair[0] - padded_bounds[1],
                'outside_padded_side_m': abs(chair[1]) - padded_bounds[3],
                'actual_body_rectangle_clearance_m': chair_actual,
                'padded_footprint_rectangle_clearance_m': chair_padded,
                'interpretation': (
                    'The 10 mm lateral excess alone is not a body collision; '
                    'the point is 205 mm ahead of the padded front boundary.'),
            },
        },
        'static_tf': {
            'scan_frame': scan_frame,
            'base_frame': monitor['base_frame_id'],
            'chain_source_to_target': tf_chain,
            'translation_m': list(transform[0]),
            'quaternion_xyzw': list(transform[1]),
            'recorded_transform_count': len(transforms),
        },
        'scan_ranges': {
            'observed_min_max_m': [
                list(item) for item in sorted(range_contracts)],
            'filter': 'finite and range_min <= range <= range_max',
            'rejected_counts': dict(sorted(rejected_ranges.items())),
            'blind_region_warning': (
                'Ranges below range_min are unobserved/invalid, not proven '
                'free space.'),
        },
        'input_age': {
            'velocity_smoother_timeout_s': velocity_timeout_s,
            'max_known_input_age_s': max_input_age_s,
            'fresh_scan_count': aggregate['fresh_input_scans'],
            'fresh_definition': (
                'input_age_s <= velocity_smoother.velocity_timeout'),
            'interpretation': (
                'Stale scan rows describe retained polygon geometry only; '
                'they are not new Collision Monitor cmd-callback '
                'evaluations.'),
        },
        'point_membership': {
            'policy': 'Jazzy Polygon::isPointInside Shimrat ray crossing',
            'boundary_note': (
                'Boundary behavior follows the upstream strict/equality '
                'comparisons and is not forced inclusive.'),
        },
        'counts': dict(sorted(aggregate.items())),
        'warnings_and_limits': [
            ('Initial scans before the first /cmd_vel_smoothed receive event '
             'are unknown.'),
            ('Alignment uses bag receive/log time; message output is never '
             'used to select a zone.'),
            ('Scan rows reuse the polygon selected by the latest cmd '
             'callback; stale rows are not new interventions.'),
            'Only recorded static TF scan-to-base geometry is replayed.',
            ('Collision Monitor callback-time TF/base_shift_correction '
             'compensation is not reproduced.'),
            'FootprintApproach trajectory projection is not reproduced.',
            'Source timeout and live callback scheduling are not reproduced.',
            ('Geometry results are counterfactual and remain distinct from '
             'recorded actual monitor state.'),
            ('Laser blind/invalid ranges do not establish collision-free '
             'clearance.'),
        ],
        'artifacts': {'per_scan_csv': str(csv_path)},
    }
    summary_path = output_dir / 'summary.json'
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    return summary


def main():
    """Run the command-line offline replay."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--params', required=True, type=Path)
    parser.add_argument('--geometry', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    summary = replay(args.bag, args.params, args.geometry, args.output_dir)
    print(json.dumps({
        'scan_count': summary['counts']['scan_count'],
        'current_stop_scans': summary[
            'current_counterfactual']['stop_scan_count'],
        'old_hypothetical_stop_scans': summary[
            'old_rotation_zero_matching_hypothetical']['stop_scan_count'],
        'output_dir': str(args.output_dir.resolve()),
    }, ensure_ascii=False))


if __name__ == '__main__':
    main()
