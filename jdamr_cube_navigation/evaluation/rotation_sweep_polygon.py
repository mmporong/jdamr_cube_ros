"""
Print the in-place rotation StopZone: the padded footprint swept 0..SWEEP_DEG.

Run from the package root: python3 evaluation/rotation_sweep_polygon.py

The counterclockwise polygon is the union of the footprint rotated by every
STEP_DEG up to SWEEP_DEG, grown by 3 mm and simplified within 2.5 mm, so the
steps between samples and the simplification stay inside; the clockwise
polygon is its mirror about the base X axis.
"""
import json
from pathlib import Path

from shapely import affinity
from shapely.geometry import Polygon
from shapely.ops import unary_union
import yaml

SWEEP_DEG = 6.0
STEP_DEG = 0.25

params = yaml.safe_load(
    (Path(__file__).resolve().parents[1] / 'config/new_base_nav2_params.yaml').read_text())
footprint = Polygon(json.loads(
    params['local_costmap']['local_costmap']['ros__parameters']['footprint']))
steps = int(round(SWEEP_DEG / STEP_DEG))
swept = unary_union([affinity.rotate(footprint, k * STEP_DEG, origin=(0.0, 0.0))
                     for k in range(steps + 1)])
swept = swept.buffer(0.003, join_style='mitre').simplify(0.0025)
points = [[round(x, 4), round(y, 4)] for x, y in list(swept.exterior.coords)[:-1]]
print('rotation:', json.dumps(points))
print('rotation_clockwise:', json.dumps([[x, -y] for x, y in reversed(points)]))
