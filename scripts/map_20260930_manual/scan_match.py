"""Offline scan-to-map localization of a stationary robot (read-only analysis).

Usage: python3 scan_match.py <scan.json> <map.yaml> <out_prefix> [seed_x seed_y seed_yaw ...]
- Median range per beam over all scans (robot stationary).
- Laser pose from /tf_static base_link->laser_link.
- Score = mean truncated distance (m) from beam endpoints to nearest occupied map cell.
- Coarse grid search around each seed, then local refinement.
"""
import json
import math
import sys

import cv2
import numpy as np


def load_map(yaml_path):
    import os
    import yaml
    meta = yaml.safe_load(open(yaml_path))
    origin = [float(v) for v in meta['origin']]
    img = cv2.imread(os.path.join(os.path.dirname(yaml_path), meta['image']), cv2.IMREAD_UNCHANGED)
    res = float(meta['resolution'])
    occ_t, free_t = float(meta['occupied_thresh']), float(meta['free_thresh'])
    assert int(meta.get('negate', 0)) == 0
    p = (255 - img.astype(float)) / 255.0
    occupied = p > occ_t
    free = p < free_t
    return img, occupied, free, res, origin


def world_to_cell(x, y, res, origin, h):
    col = np.floor((x - origin[0]) / res).astype(int)
    row = h - 1 - np.floor((y - origin[1]) / res).astype(int)
    return row, col


def laser_points(scan_json):
    d = json.load(open(scan_json))
    scans = d['scans']
    s0 = scans[0]
    rmin, rmax = s0['range_min'], s0['range_max']
    # beam count varies per revolution: bin by angle (0.5 deg) and take the per-bin median
    nb = 360
    arr = np.full((len(scans), nb), np.nan)
    for i, s in enumerate(scans):
        r = np.array([np.nan if v is None else v for v in s['ranges']], dtype=float)
        a = s['angle_min'] + s['angle_increment'] * np.arange(len(r))
        r[(r < rmin) | (r > rmax)] = np.nan
        b = np.floor(np.mod(a, 2 * math.pi) / (2 * math.pi) * nb).astype(int) % nb
        for bi, rv in zip(b, r):
            if np.isfinite(rv) and (not np.isfinite(arr[i, bi]) or rv < arr[i, bi]):
                arr[i, bi] = rv
    valid_frac = np.mean(np.isfinite(arr), axis=0)
    with np.errstate(all='ignore'):
        med = np.nanmedian(arr, axis=0)
        spread = np.nanmax(arr, axis=0) - np.nanmin(arr, axis=0)
    keep = (valid_frac >= 0.6) & np.isfinite(med) & (spread < 0.10)
    ang = (np.arange(nb) + 0.5) / nb * 2 * math.pi
    tf = next(t for t in d['tf_static'] if t['child'] == s0['frame'])
    qx, qy, qz, qw = tf['q']
    lyaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    lx, ly = tf['t'][0], tf['t'][1]
    # points in base_link
    xl, yl = med * np.cos(ang), med * np.sin(ang)
    xb = lx + math.cos(lyaw) * xl - math.sin(lyaw) * yl
    yb = ly + math.sin(lyaw) * xl + math.cos(lyaw) * yl
    return np.stack([xb[keep], yb[keep]], axis=1), med, ang, keep, (lx, ly, lyaw), d


def make_score(occupied, res, origin, trunc=0.3):
    h, w = occupied.shape
    dist = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5) * res
    dist = np.minimum(dist, trunc)

    def score(pts, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * pts[:, 0] - s * pts[:, 1]
        wy = y + s * pts[:, 0] + c * pts[:, 1]
        r, col = world_to_cell(wx, wy, res, origin, h)
        inside = (r >= 0) & (r < h) & (col >= 0) & (col < w)
        vals = np.full(len(pts), trunc)
        vals[inside] = dist[r[inside], col[inside]]
        return float(np.mean(vals)), vals
    return score, dist


def search(score, pts, seed, xy_half=0.6, yaw_half=math.radians(35)):
    best = (1e9, None)
    for x in np.arange(seed[0] - xy_half, seed[0] + xy_half + 1e-9, 0.05):
        for y in np.arange(seed[1] - xy_half, seed[1] + xy_half + 1e-9, 0.05):
            for yaw in np.arange(seed[2] - yaw_half, seed[2] + yaw_half + 1e-9, math.radians(2)):
                s, _ = score(pts, x, y, yaw)
                if s < best[0]:
                    best = (s, (x, y, yaw))
    # local refinement
    s_best, (x, y, yaw) = best
    for step_xy, step_yaw in ((0.02, math.radians(1)), (0.01, math.radians(0.5)), (0.005, math.radians(0.25))):
        improved = True
        while improved:
            improved = False
            for dx, dy, dyaw in ((step_xy, 0, 0), (-step_xy, 0, 0), (0, step_xy, 0), (0, -step_xy, 0),
                                 (0, 0, step_yaw), (0, 0, -step_yaw)):
                s, _ = score(pts, x + dx, y + dy, yaw + dyaw)
                if s < s_best - 1e-6:
                    s_best, x, y, yaw, improved = s, x + dx, y + dy, yaw + dyaw, True
    return s_best, (x, y, yaw)


def main():
    scan_json, map_yaml, out_prefix = sys.argv[1:4]
    seeds = [float(v) for v in sys.argv[4:]]
    seeds = [tuple(seeds[i:i + 3]) for i in range(0, len(seeds), 3)]
    img, occupied, free, res, origin = load_map(map_yaml)
    pts, med, ang, keep, laser, d = laser_points(scan_json)
    score, dist = make_score(occupied, res, origin)
    print(f'beams kept {len(pts)}/{len(med)}; laser yaw {math.degrees(laser[2]):.1f} deg')
    results = []
    for seed in seeds:
        s, pose = search(score, pts, seed)
        _, vals = score(pts, *pose)
        inl = float(np.mean(vals < 0.05))
        results.append((s, pose, inl, seed))
        print(f'seed {seed} -> pose ({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f} deg) '
              f'mean_dist {s:.4f} m inlier(<5cm) {inl:.3f}')
    results.sort(key=lambda r: r[0])
    s, pose, inl, seed = results[0]
    # second-best distinct minimum check (ambiguity)
    json.dump({'best_pose': pose, 'mean_dist_m': s, 'inlier_5cm': inl,
               'all': [{'seed': r[3], 'pose': r[1], 'mean_dist_m': r[0], 'inlier_5cm': r[2]} for r in results]},
              open(out_prefix + '_match.json', 'w'), indent=1)
    np.save(out_prefix + '_pts_base.npy', pts)


if __name__ == '__main__':
    main()
