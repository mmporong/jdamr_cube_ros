#!/usr/bin/env python3
"""Compare offline 2D SLAM backends replayed from the same recorded bag."""

# The robot has no external ground truth, so this never calls its numbers ATE
# or RPE.  It reports what the recordings actually support: how far each
# backend says the robot travelled, how far its estimate drifts from wheel
# odometry and from the AMCL pose recorded against the published map, and how
# much of the corridor each backend mapped.

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from mcap_ros2.reader import read_ros2_messages


def yaw_of(orientation):
    """Return the planar yaw of a quaternion."""
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))


def compose(parent, child):
    """Compose two planar (x, y, yaw) transforms."""
    px, py, pyaw = parent
    cx, cy, cyaw = child
    return (px + cx * math.cos(pyaw) - cy * math.sin(pyaw),
            py + cx * math.sin(pyaw) + cy * math.cos(pyaw),
            pyaw + cyaw)


# /odom arrives at 50 Hz with centimetre noise.  Summing every consecutive
# pair turned a 23.6 m drive into 304 m of accumulated noise, so the length is
# measured on a trajectory decimated to a minimum step first.
MIN_STEP_M = 0.05


def decimate(points, min_step=MIN_STEP_M):
    """Return points spaced at least *min_step* apart, keeping the ends."""
    if not points:
        return []
    kept = [points[0]]
    for point in points[1:]:
        if math.dist(kept[-1][1:3], point[1:3]) >= min_step:
            kept.append(point)
    if kept[-1] is not points[-1]:
        kept.append(points[-1])
    return kept


def path_length(points):
    """Return the polyline length of a trajectory, noise removed."""
    kept = decimate(points)
    return sum(math.dist(a[1:3], b[1:3]) for a, b in zip(kept, kept[1:]))


def rigid_align(source, target):
    """Return the rotation and translation putting *source* onto *target*."""
    # A SLAM backend starts its map frame at the robot's first pose, while the
    # recorded AMCL pose lives in the pre-built map frame.  Comparing them
    # without alignment measures the frame offset, not the estimate, which is
    # how a 60 s pilot produced a 19.7 m "error".  Planar Kabsch: rotation and
    # translation only, never scale.
    if len(source) < 2:
        return None
    sx = sum(p[0] for p in source) / len(source)
    sy = sum(p[1] for p in source) / len(source)
    tx = sum(p[0] for p in target) / len(target)
    ty = sum(p[1] for p in target) / len(target)
    num = sum((p[0] - sx) * (q[1] - ty) - (p[1] - sy) * (q[0] - tx)
              for p, q in zip(source, target))
    den = sum((p[0] - sx) * (q[0] - tx) + (p[1] - sy) * (q[1] - ty)
              for p, q in zip(source, target))
    if num == 0.0 and den == 0.0:
        return None
    theta = math.atan2(num, den)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return (theta,
            tx - (cos_t * sx - sin_t * sy),
            ty - (sin_t * sx + cos_t * sy))


def apply_rigid(transform, x, y):
    """Apply a (theta, dx, dy) planar transform to a point."""
    theta, dx, dy = transform
    return (math.cos(theta) * x - math.sin(theta) * y + dx,
            math.sin(theta) * x + math.cos(theta) * y + dy)


def read_map_to_odom(result_bag: Path):
    """Return the backend's map->odom estimates as (t, x, y, yaw)."""
    samples = []
    for message in read_ros2_messages(str(result_bag), topics=['/tf']):
        for transform in message.ros_msg.transforms:
            if (transform.header.frame_id.lstrip('/') == 'map'
                    and transform.child_frame_id.lstrip('/') == 'odom'):
                stamp = transform.header.stamp
                samples.append((
                    stamp.sec + stamp.nanosec * 1e-9,
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    yaw_of(transform.transform.rotation)))
    samples.sort()
    return samples


def read_odometry(source_bag: Path):
    """Return recorded wheel odometry as (t, x, y, yaw)."""
    samples = []
    for message in read_ros2_messages(str(source_bag), topics=['/odom']):
        pose = message.ros_msg.pose.pose
        stamp = message.ros_msg.header.stamp
        samples.append((stamp.sec + stamp.nanosec * 1e-9,
                        pose.position.x, pose.position.y,
                        yaw_of(pose.orientation)))
    samples.sort()
    return samples


def read_amcl(source_bag: Path):
    """Return the recorded AMCL pose as (t, x, y, yaw)."""
    samples = []
    for message in read_ros2_messages(str(source_bag), topics=['/amcl_pose']):
        pose = message.ros_msg.pose.pose
        stamp = message.ros_msg.header.stamp
        samples.append((stamp.sec + stamp.nanosec * 1e-9,
                        pose.position.x, pose.position.y,
                        yaw_of(pose.orientation)))
    samples.sort()
    return samples


def nearest(samples, timestamp):
    """Return the sample closest in time, or None when the list is empty."""
    if not samples:
        return None
    best = min(samples, key=lambda s: abs(s[0] - timestamp))
    return best if abs(best[0] - timestamp) < 1.0 else None


def estimated_trajectory(map_to_odom, odometry):
    """Compose the backend correction with wheel odometry."""
    trajectory = []
    for stamp, ox, oy, oyaw in odometry:
        correction = nearest(map_to_odom, stamp)
        if correction is None:
            continue
        x, y, yaw = compose(correction[1:], (ox, oy, oyaw))
        trajectory.append((stamp, x, y, yaw))
    return trajectory


def map_statistics(map_yaml: Path):
    """Return occupied/free cell counts and metric extent of a saved map."""
    import yaml
    document = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    raw = (map_yaml.parent / document['image']).read_bytes()
    width, height = map(int, raw.split(b'\n', 3)[1].split())
    data = raw[-width * height:]
    resolution = float(document['resolution'])
    occupied = sum(1 for value in data if value < 100)
    free = sum(1 for value in data if value > 200)
    return {
        'width_cells': width,
        'height_cells': height,
        'extent_m': [round(width * resolution, 2),
                     round(height * resolution, 2)],
        'occupied_cells': occupied,
        'free_cells': free,
        'unknown_cells': width * height - occupied - free,
        'occupied_length_m': round(occupied * resolution, 1),
    }


def deviation(trajectory, reference):
    """Return residuals after aligning the estimate onto the reference."""
    pairs = []
    for stamp, x, y, _ in decimate(trajectory, 0.10):
        match = nearest(reference, stamp)
        if match is not None:
            pairs.append(((x, y), (match[1], match[2])))
    if len(pairs) < 2:
        return None
    transform = rigid_align([p[0] for p in pairs], [p[1] for p in pairs])
    if transform is None:
        return None
    residuals = [math.dist(apply_rigid(transform, *source), target)
                 for source, target in pairs]
    return {
        'samples': len(residuals),
        'aligned_yaw_deg': round(math.degrees(transform[0]), 2),
        'rms_m': round(
            math.sqrt(sum(r * r for r in residuals) / len(residuals)), 3),
        'max_m': round(max(residuals), 3),
    }


def analyse(result_bag: Path, source_bag: Path, map_yaml: Path | None):
    """Return one comparison record for a replayed backend run."""
    map_to_odom = read_map_to_odom(result_bag)
    odometry = read_odometry(source_bag)
    amcl = read_amcl(source_bag)
    trajectory = estimated_trajectory(map_to_odom, odometry)

    record = {
        'result_bag': result_bag.name,
        'source_bag': source_bag.name,
        'map_to_odom_updates': len(map_to_odom),
        'odometry_samples': len(odometry),
        'trajectory_samples': len(trajectory),
    }
    if trajectory:
        record['estimated_length_m'] = round(path_length(trajectory), 3)
        record['start_xy'] = [round(trajectory[0][1], 3),
                              round(trajectory[0][2], 3)]
        record['end_xy'] = [round(trajectory[-1][1], 3),
                            round(trajectory[-1][2], 3)]
        record['start_to_end_m'] = round(
            math.dist(trajectory[0][1:3], trajectory[-1][1:3]), 3)
    if odometry:
        record['odometry_length_m'] = round(path_length(odometry), 3)
    if amcl:
        record['amcl_length_m'] = round(path_length(amcl), 3)
        record['deviation_from_amcl'] = deviation(trajectory, amcl)
    record['deviation_from_odometry'] = deviation(trajectory, odometry)
    if map_yaml and map_yaml.is_file():
        record['map'] = map_statistics(map_yaml)
    return record


def main(argv=None):
    """Analyse every replayed run in a directory."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--results', type=Path, required=True,
                        help='directory holding *_result bags and maps')
    parser.add_argument('--source-root', type=Path,
                        default=Path.home() / 'jdamr_artifacts')
    parser.add_argument('--output', type=Path, default=None)
    args = parser.parse_args(argv)

    records = []
    for result_dir in sorted(args.results.glob('*_result')):
        mcap = next(iter(sorted(result_dir.glob('*.mcap'))), None)
        if mcap is None:
            continue
        stem = result_dir.name[:-len('_result')]
        source_name, _, _backend = stem.partition('__')
        source_dir = args.source_root / source_name
        source_mcap = next(iter(sorted(source_dir.glob('*.mcap'))), None)
        if source_mcap is None:
            print(f'원본 bag 없음: {source_dir}')
            continue
        map_yaml = args.results / f'{stem}_map.yaml'
        record = analyse(mcap, source_mcap, map_yaml)
        record['backend'] = _backend
        records.append(record)
        print(json.dumps(record, ensure_ascii=False, indent=2))
    if args.output:
        args.output.write_text(
            json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'\n저장: {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
