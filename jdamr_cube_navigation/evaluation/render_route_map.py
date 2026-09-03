#!/usr/bin/env python3
"""Render the saved map, keepout zones, and planned route as a figure."""

# An RViz screenshot cannot be regenerated once the session is gone, and an
# occluded window captures whatever overlaps it.  This renders the same
# information from the files the robot actually used, so the figure can be
# rebuilt from a commit hash alone.

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402


def load_pgm(path: Path):
    """Return a PGM as a numpy array of unsigned bytes."""
    raw = path.read_bytes()
    header = raw.split(b'\n', 3)
    width, height = map(int, header[1].split())
    return np.frombuffer(raw[-width * height:], dtype=np.uint8) \
             .reshape(height, width)


def load_map(map_yaml: Path):
    """Return the occupancy image, resolution and origin for a map YAML."""
    document = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    image = load_pgm(map_yaml.parent / document['image'])
    return image, float(document['resolution']), document['origin']


def extent_of(image, resolution, origin):
    """Return the matplotlib extent in map coordinates."""
    height, width = image.shape
    return [origin[0], origin[0] + width * resolution,
            origin[1], origin[1] + height * resolution]


def render(route_yaml: Path, output: Path, trajectory=None):
    """Draw the corridor map with keepout zones and the planned route."""
    config = yaml.safe_load(route_yaml.read_text(encoding='utf-8'))
    home = str(Path.home())
    map_yaml = Path(config['map_yaml'].replace('$HOME', home))
    mask_yaml = Path(config['keepout_mask_yaml'].replace('$HOME', home))
    occupancy, resolution, origin = load_map(map_yaml)
    mask, _, _ = load_map(mask_yaml)
    extent = extent_of(occupancy, resolution, origin)

    figure, axes = plt.subplots(figsize=(16, 5.5), dpi=160)
    axes.imshow(occupancy, cmap='gray', extent=extent, origin='upper',
                interpolation='nearest', vmin=0, vmax=255)

    keepout = np.ma.masked_where(mask >= 100, np.ones_like(mask))
    axes.imshow(keepout, extent=extent, origin='upper',
                interpolation='nearest', cmap='autumn', alpha=0.55)

    xs = [0.0] + [w['x'] for w in config['waypoints']]
    ys = [0.0] + [w['y'] for w in config['waypoints']]
    axes.plot(xs, ys, '-', color='#1f77b4', linewidth=2.0,
              label='planned route')
    axes.plot(xs[1:], ys[1:], 'o', color='#1f77b4', markersize=4)
    if trajectory:
        axes.plot([p[0] for p in trajectory], [p[1] for p in trajectory],
                  '-', color='#2ca02c', linewidth=1.6, alpha=0.9,
                  label='driven (AMCL)')
    axes.plot([0.0], [0.0], marker='*', color='#111111', markersize=16,
              linestyle='none', label='start / home')
    turn = next(w for w in config['waypoints'] if w['id'] == 'turnaround')
    axes.plot([turn['x']], [turn['y']], marker='s', color='#111111',
              markersize=8, linestyle='none', label='turnaround')

    axes.set_xlabel('x [m]  (map frame)')
    axes.set_ylabel('y [m]')
    axes.set_title(
        f"JD-AMR corridor round trip — {config['id']}\n"
        f"saved map + keepout mask, {len(config['waypoints'])} waypoints, "
        f"{config['planned_length_m']:.2f} m")
    axes.grid(alpha=0.25, linewidth=0.5)
    axes.set_aspect('equal')
    axes.legend(loc='lower right', framealpha=0.9)
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output)
    plt.close(figure)
    return output


def main(argv=None):
    """Render the route figure for the portfolio."""
    package_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--route', type=Path, default=(
        package_root / 'config' / 'corridor_roundtrip.autonomous_20260826.yaml'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trajectory-bag', type=Path, default=None,
                        help='MCAP whose /amcl_pose is drawn over the route')
    args = parser.parse_args(argv)

    trajectory = None
    if args.trajectory_bag:
        from mcap_ros2.reader import read_ros2_messages
        trajectory = [
            (m.ros_msg.pose.pose.position.x, m.ros_msg.pose.pose.position.y)
            for m in read_ros2_messages(str(args.trajectory_bag),
                                        topics=['/amcl_pose'])]
        print(f'trajectory points: {len(trajectory)}')
    print(render(args.route, args.output, trajectory))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
