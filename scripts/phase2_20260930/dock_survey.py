"""Read-only dock survey: scan-to-map pose and unmapped objects near table markers.

Usage: python3 dock_survey.py <scan.json> <map.yaml> <keepout.yaml> <out_dir>
Outputs <out_dir>/survey.json and overlay PNGs. No ROS, no robot commands.
"""
import json
import math
import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scan_match as m  # noqa: E402

MARKERS = {'table_1': (1.896, 0.303), 'table_2': (3.119, -0.924)}
REGISTRY_HOME = (-0.24715302314139942, -0.8184447988772949, 1.6053823404497964)
MIN_INLIER_5CM = 0.5
UNMAPPED_M = 0.10       # point farther than this from any occupied map cell
NEAR_MARKER_M = 1.2


def global_pose(score, pts, free, res, origin):
    h, _w = free.shape
    rows, cols = np.where(free)
    cands = []
    for r, c in zip(rows, cols):
        if r % 2 or c % 2:
            continue
        cands.append((origin[0] + (c + 0.5) * res, origin[1] + (h - 1 - r + 0.5) * res))
    coarse = []
    for x, y in cands:
        for yaw in np.radians(np.arange(0, 360, 4)):
            coarse.append((score(pts, x, y, yaw)[0], x, y, yaw))
    coarse.sort()
    tops = []
    for s, x, y, yaw in coarse:
        if all(math.hypot(x - t[1], y - t[2]) > 0.4
               or abs((yaw - t[3] + math.pi) % (2 * math.pi) - math.pi) > math.radians(30)
               for t in tops):
            tops.append((s, x, y, yaw))
        if len(tops) >= 5:
            break
    refined = []
    for _s, x, y, yaw in tops:
        s, pose = m.search(score, pts, (x, y, yaw), xy_half=0.15, yaw_half=math.radians(8))
        inl = float(np.mean(score(pts, *pose)[1] < 0.05))
        refined.append({'pose': [pose[0], pose[1], pose[2]], 'mean_dist_m': s, 'inlier_5cm': inl})
    refined.sort(key=lambda r: r['mean_dist_m'])
    return refined


def raw_points_map(scan_json, pose):
    d = json.load(open(scan_json))
    tf = next(t for t in d['tf_static'] if t['child'] == d['scans'][0]['frame'])
    qx, qy, qz, qw = tf['q']
    lyaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    lx, ly = tf['t'][0], tf['t'][1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    out = []
    for sc in d['scans']:
        r = np.array([np.nan if v is None else v for v in sc['ranges']], float)
        a = sc['angle_min'] + sc['angle_increment'] * np.arange(len(r))
        ok = np.isfinite(r) & (r >= sc['range_min']) & (r <= sc['range_max'])
        xl, yl = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        xb = lx + math.cos(lyaw) * xl - math.sin(lyaw) * yl
        yb = ly + math.sin(lyaw) * xl + math.cos(lyaw) * yl
        out.append(np.stack([pose[0] + c * xb - s * yb, pose[1] + s * xb + c * yb], 1))
    return np.concatenate(out), len(d['scans'])


def clusters(points, n_scans, dist, res, origin, h, w):
    """Persistent unmapped returns grouped by 0.05 m grid connectivity."""
    cell = 0.025
    keys = {}
    for x, y in points:
        col = int((x - origin[0]) / res)
        row = h - 1 - int((y - origin[1]) / res)
        if not (0 <= row < h and 0 <= col < w) or dist[row, col] <= UNMAPPED_M:
            continue
        k = (int(math.floor(x / cell)), int(math.floor(y / cell)))
        keys[k] = keys.get(k, 0) + 1
    keep = {k for k, n in keys.items() if n >= max(3, 0.3 * n_scans)}
    seen, groups = set(), []
    for k in keep:
        if k in seen:
            continue
        stack, group = [k], []
        seen.add(k)
        while stack:
            cur = stack.pop()
            group.append(cur)
            for dx in (-2, -1, 0, 1, 2):
                for dy in (-2, -1, 0, 1, 2):
                    nb = (cur[0] + dx, cur[1] + dy)
                    if nb in keep and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        pts = np.array([((a + 0.5) * cell, (b + 0.5) * cell) for a, b in group])
        centroid = pts.mean(0)
        if len(pts) >= 2:
            u, sv, vt = np.linalg.svd(pts - centroid)
            direction = vt[0]
            proj = (pts - centroid) @ direction
            length = float(proj.max() - proj.min())
            thickness = float(np.ptp((pts - centroid) @ vt[1]))
        else:
            direction, length, thickness = np.array([1.0, 0.0]), 0.0, 0.0
        groups.append({'centroid': centroid.round(3).tolist(), 'cells': len(pts),
                       'length_m': round(length, 3), 'thickness_m': round(thickness, 3),
                       'axis_deg': round(math.degrees(math.atan2(direction[1], direction[0])), 1),
                       'bbox': [pts.min(0).round(3).tolist(), pts.max(0).round(3).tolist()]})
    return groups


def render(path, img, kimg, res, origin, points, pose, marks, window, scale=120):
    x0, x1, y0, y1 = window
    h, w = img.shape
    wc, hc = int((x1 - x0) * scale), int((y1 - y0) * scale)
    can = np.full((hc, wc, 3), 255, np.uint8)

    def px(x, y):
        return int((x - x0) * scale), int((y1 - y) * scale)
    for r in range(h):
        for c in range(w):
            x = origin[0] + c * res
            y = origin[1] + (h - 1 - r) * res
            u0, v0 = px(x, y + res)
            u1, v1 = px(x + res, y)
            if u1 < 0 or v1 < 0 or u0 >= wc or v0 >= hc:
                continue
            v = img[r, c]
            col = (0, 0, 0) if v < 100 else ((255, 255, 255) if v > 250 else (190, 190, 190))
            if kimg[r, c] < 100 and v > 250:
                col = (215, 215, 255)
            cv2.rectangle(can, (u0, v0), (u1, v1), col, -1)
    for x, y in points:
        u, v = px(x, y)
        if 0 <= u < wc and 0 <= v < hc:
            can[v, u] = (0, 110, 255)
    for gx in np.arange(math.ceil(x0), x1, 0.5):
        u, _ = px(gx, 0)
        cv2.line(can, (u, 0), (u, hc), (235, 210, 210), 1)
        cv2.putText(can, f'{gx:.1f}', (u + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 0, 0), 1)
    for gy in np.arange(math.ceil(y0), y1, 0.5):
        _, v = px(0, gy)
        cv2.line(can, (0, v), (wc, v), (235, 210, 210), 1)
        cv2.putText(can, f'{gy:.1f}', (2, v - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 0, 0), 1)
    for name, (mx, my, color) in marks.items():
        u, v = px(mx, my)
        cv2.drawMarker(can, (u, v), color, cv2.MARKER_CROSS, 14, 2)
        cv2.putText(can, name, (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    u, v = px(pose[0], pose[1])
    cv2.arrowedLine(can, (u, v), (int(u + 30 * math.cos(pose[2])), int(v - 30 * math.sin(pose[2]))),
                    (0, 150, 0), 2)
    cv2.imwrite(path, can)


def main():
    scan_json, map_yaml, keepout_yaml, out_dir = sys.argv[1:5]
    os.makedirs(out_dir, exist_ok=True)
    img, occ, free, res, origin = m.load_map(map_yaml)
    kmeta = yaml.safe_load(open(keepout_yaml))
    kimg = cv2.imread(os.path.join(os.path.dirname(keepout_yaml), kmeta['image']),
                      cv2.IMREAD_UNCHANGED)
    pts, *_ = m.laser_points(scan_json)
    score, _dist = m.make_score(occ, res, origin)
    ranked = global_pose(score, pts, free, res, origin)
    best = ranked[0]
    pose = best['pose']
    ok = (best['inlier_5cm'] >= MIN_INLIER_5CM and len(ranked) > 1
          and ranked[1]['mean_dist_m'] - best['mean_dist_m'] >= 0.02)
    h, w = occ.shape
    dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res
    raw, n_scans = raw_points_map(scan_json, pose)
    groups = clusters(raw, n_scans, dist, res, origin, h, w)
    near = {}
    for name, (mx, my) in MARKERS.items():
        near[name] = sorted(
            [dict(g, marker_distance_m=round(math.dist(g['centroid'], (mx, my)), 3))
             for g in groups if math.dist(g['centroid'], (mx, my)) <= NEAR_MARKER_M],
            key=lambda g: g['marker_distance_m'])
    result = {
        'scan': os.path.abspath(scan_json), 'map_yaml': os.path.abspath(map_yaml),
        'pose_ok': bool(ok), 'pose': pose, 'pose_yaw_deg': math.degrees(pose[2]),
        'ranked': ranked,
        'registry_home_delta_m': math.dist(pose[:2], REGISTRY_HOME[:2]),
        'registry_home_delta_deg': math.degrees(
            (pose[2] - REGISTRY_HOME[2] + math.pi) % (2 * math.pi) - math.pi),
        'unmapped_near_markers': near, 'n_scans': n_scans,
        'criteria': {'min_inlier_5cm': MIN_INLIER_5CM, 'second_best_margin_m': 0.02,
                     'unmapped_m': UNMAPPED_M, 'near_marker_m': NEAR_MARKER_M},
    }
    json.dump(result, open(os.path.join(out_dir, 'survey.json'), 'w'), indent=1)
    marks = {'T1': (*MARKERS['table_1'], (0, 0, 230)), 'T2': (*MARKERS['table_2'], (0, 0, 230)),
             'home': (REGISTRY_HOME[0], REGISTRY_HOME[1], (150, 0, 150))}
    render(os.path.join(out_dir, 'overview.png'), img, kimg, res, origin, raw, pose, marks,
           (-4.3, 4.6, -3.9, 3.6), 110)
    render(os.path.join(out_dir, 'tables.png'), img, kimg, res, origin, raw, pose, marks,
           (0.8, 4.0, -1.8, 1.3), 220)
    print(json.dumps({k: result[k] for k in ('pose_ok', 'pose', 'pose_yaw_deg',
                                              'registry_home_delta_m',
                                              'registry_home_delta_deg')}))
    print('ranked', [(round(r['mean_dist_m'], 4), round(r['inlier_5cm'], 3),
                      [round(v, 3) for v in r['pose']]) for r in ranked[:3]])
    for name, gs in near.items():
        print(name, [(g['centroid'], g['length_m'], g['marker_distance_m']) for g in gs[:6]])


if __name__ == '__main__':
    main()
