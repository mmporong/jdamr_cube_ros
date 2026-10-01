"""Offline clearance check for observation waypoints (read-only analysis).

Obstacles = occupied or unknown map cells, plus keepout cells.
Reports distance from each candidate to the nearest obstacle and keepout,
so rotation circles (0.43 m) and footprints can be judged before departure.
"""
import json
import math
import os
import sys

import cv2
import numpy as np
import yaml

ROTATION_RADIUS_M = 0.43       # StopZone in-place rotation radius
FOOTPRINT = [(0.085, 0.29), (0.085, -0.29), (-0.295, -0.29), (-0.295, 0.29)]


def load(yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    img = cv2.imread(os.path.join(os.path.dirname(yaml_path), meta['image']),
                     cv2.IMREAD_UNCHANGED)
    return img, float(meta['resolution']), [float(v) for v in meta['origin'][:2]]


def fields(map_yaml, keepout_yaml):
    img, res, origin = load(map_yaml)
    kimg, kres, korigin = load(keepout_yaml)
    assert img.shape == kimg.shape and res == kres and origin == korigin
    p = (255 - img.astype(float)) / 255.0
    occupied = p > 0.65
    unknown = (p >= 0.196) & (p <= 0.65)
    keepout = kimg < 100
    obstacle = occupied | unknown
    d_obs = cv2.distanceTransform((~obstacle).astype(np.uint8), cv2.DIST_L2, 5) * res
    d_keep = cv2.distanceTransform((~keepout).astype(np.uint8), cv2.DIST_L2, 5) * res
    d_occ = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5) * res
    return {'res': res, 'origin': origin, 'h': img.shape[0], 'occupied': occupied,
            'unknown': unknown, 'keepout': keepout, 'd_obs': d_obs, 'd_keep': d_keep,
            'd_occ': d_occ}


def cell(f, x, y):
    col = int(math.floor((x - f['origin'][0]) / f['res']))
    row = f['h'] - 1 - int(math.floor((y - f['origin'][1]) / f['res']))
    return row, col


def clearance(f, x, y):
    r, c = cell(f, x, y)
    return {'obstacle_m': round(float(f['d_obs'][r, c]), 3),
            'occupied_m': round(float(f['d_occ'][r, c]), 3),
            'keepout_m': round(float(f['d_keep'][r, c]), 3),
            'in_keepout': bool(f['keepout'][r, c]), 'occupied': bool(f['occupied'][r, c])}


def footprint_cells_clear(f, x, y, yaw, step=0.02):
    """Every sample inside the footprint polygon must be free and outside keepout."""
    c, s = math.cos(yaw), math.sin(yaw)
    bad = {'occupied': 0, 'unknown': 0, 'keepout': 0}
    for a in np.arange(-0.295, 0.085 + 1e-9, step):
        for b in np.arange(-0.29, 0.29 + 1e-9, step):
            wx, wy = x + c * a - s * b, y + s * a + c * b
            r, col = cell(f, wx, wy)
            bad['occupied'] += int(f['occupied'][r, col])
            bad['unknown'] += int(f['unknown'][r, col])
            bad['keepout'] += int(f['keepout'][r, col])
    return bad


def segment_min_clearance(f, a, b, step=0.02):
    n = max(2, int(math.dist(a, b) / step))
    return min(clearance(f, a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)['obstacle_m']
               for t in np.linspace(0, 1, n))


if __name__ == '__main__':
    map_yaml, keepout_yaml = sys.argv[1:3]
    f = fields(map_yaml, keepout_yaml)
    points = json.loads(sys.argv[3])
    for name, (x, y) in points.items():
        print(name, (x, y), clearance(f, x, y))
