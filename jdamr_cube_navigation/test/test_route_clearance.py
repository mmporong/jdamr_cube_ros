"""Check waypoint lines against known walls and keepouts, not Nav2 feasibility."""

# On 2026-09-03 the first leg aborted with "collision ahead" because the
# waypoints ran within the robot footprint of mapped walls.  Saved-map unknown
# cells and rotation feasibility still require a Nav2 planning-only preflight.

import math
import json
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROUTE = PACKAGE_ROOT / 'config' / 'corridor_roundtrip.autonomous_20260826.yaml'
NEW_BASE_PARAMS = PACKAGE_ROOT / 'config' / 'new_base_nav2_params.yaml'

_footprint = json.loads(yaml.safe_load(NEW_BASE_PARAMS.read_text())[
    'local_costmap']['local_costmap']['ros__parameters']['footprint'])
INSCRIBED_RADIUS_M = max(abs(point[1]) for point in _footprint)
REQUIRED_CLEARANCE_M = INSCRIBED_RADIUS_M + 0.10
RESOLUTION = 0.05


def _load_pgm(path):
    raw = path.read_bytes()
    width, height = map(int, raw.split(b'\n', 3)[1].split())
    return width, height, raw[-width * height:]


def _map_files():
    config = yaml.safe_load(ROUTE.read_text(encoding='utf-8'))
    map_yaml = Path(config['map_yaml'].replace('$HOME', str(Path.home())))
    mask_yaml = Path(
        config['keepout_mask_yaml'].replace('$HOME', str(Path.home())))
    if not map_yaml.is_file() or not mask_yaml.is_file():
        pytest.skip('saved map is not present on this machine')
    document = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    return (config, map_yaml.parent / document['image'],
            mask_yaml.parent / yaml.safe_load(
                mask_yaml.read_text(encoding='utf-8'))['image'],
            document['origin'])


def _sampler(image_path, origin, occupied):
    width, height, data = _load_pgm(image_path)
    ox, oy = origin[0], origin[1]

    def hit(x, y):
        col = int((x - ox) / RESOLUTION)
        row = height - 1 - int((y - oy) / RESOLUTION)
        if not (0 <= col < width and 0 <= row < height):
            return True
        return occupied(data[row * width + col])
    return hit


def _clearance(hit, x, y, cap):
    if hit(x, y):
        return 0.0
    for step in range(1, int(cap / RESOLUTION)):
        radius = step * RESOLUTION
        for degrees in range(0, 360, 5):
            angle = math.radians(degrees)
            if hit(x + radius * math.cos(angle), y + radius * math.sin(angle)):
                return radius
    return cap


def _segments(config):
    previous = (config['start_pose']['x'], config['start_pose']['y'], 'start')
    for waypoint in config['waypoints']:
        yield previous, (waypoint['x'], waypoint['y'], waypoint['id'])
        previous = (waypoint['x'], waypoint['y'], waypoint['id'])


def _worst_along(hit, start, end, cap):
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(2, int(length / 0.10))
    return min(
        _clearance(hit,
                   start[0] + (end[0] - start[0]) * i / steps,
                   start[1] + (end[1] - start[1]) * i / steps, cap)
        for i in range(steps + 1))


def test_every_segment_clears_the_inscribed_radius():
    """Nominal lines must clear known occupied cells; unknown is checked by Nav2."""
    config, map_image, _, origin = _map_files()
    hit = _sampler(map_image, origin, lambda value: value <= 200)

    for start, end in _segments(config):
        worst = _worst_along(hit, start, end, cap=1.0)
        assert worst >= REQUIRED_CLEARANCE_M, (
            f'{start[2]} -> {end[2]} clears only {worst:.2f}m '
            f'(inscribed radius is {INSCRIBED_RADIUS_M}m)')


def test_no_segment_enters_a_keepout_zone():
    """Nominal lines must not enter the stair keepout mask."""
    config, _, mask_image, origin = _map_files()
    hit = _sampler(mask_image, origin, lambda value: value < 100)

    for start, end in _segments(config):
        worst = _worst_along(hit, start, end, cap=0.6)
        assert worst > 0.0, f'{start[2]} -> {end[2]} passes through a keepout zone'


def test_declared_length_matches_the_waypoints():
    """Keep the documented length from drifting away from the coordinates."""
    config = yaml.safe_load(ROUTE.read_text(encoding='utf-8'))
    previous = (0.0, 0.0)
    total = 0.0
    for waypoint in config['waypoints']:
        total += math.hypot(waypoint['x'] - previous[0],
                            waypoint['y'] - previous[1])
        previous = (waypoint['x'], waypoint['y'])

    assert abs(total - config['planned_length_m']) < 0.5
