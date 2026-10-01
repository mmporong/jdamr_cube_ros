"""Read-only provisional layout probe on the current Phase 2 map (no ROS, no robot)."""
import math, sys, json
sys.path.insert(0, '/home/lim/jdamr_data/map_update_20260929_4XUrkc/phase2_20260930/tools')
sys.path.insert(0, '/home/lim/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_navigation')
import numpy as np
import geometry_check as g
from pathlib import Path
from jdamr_cube_navigation.reverse_parking import static_corridor_clear

P2 = Path('/home/lim/jdamr_data/map_update_20260929_4XUrkc/phase2_20260930')
MAP, KEEP = P2 / 'map_dockfix.yaml', P2 / 'keepout_dockfix.yaml'
FOOT = [(0.085, 0.29), (0.085, -0.29), (-0.295, -0.29), (-0.295, 0.29)]
R_ROT = 0.414
MARGIN = 0.10
FRONT = 0.065      # new_base_geometry front_to_wheel_axis (box_service front_extent)
CAM_X = 0.065      # lens in chassis front plane
f = g.fields(str(MAP), str(KEEP))

def poses(F, n, d_obs=0.75, d_pre=1.30, align_gap=0.45):
    yaw = math.atan2(-n[1], -n[0])
    P = lambda d: (F[0] + n[0] * d, F[1] + n[1] * d, yaw)
    return {'final': P(FRONT + 0.05), 'align': P(FRONT + align_gap), 'align060': P(FRONT + 0.60),
            'escape': P(0.565), 'observation': P(d_obs), 'pre': P(d_pre)}

def rot_ok(p, use_keep=True):
    c = g.clearance(f, p[0], p[1])
    m = min(c['obstacle_m'], c['keepout_m'] if use_keep else 9)
    return m, m >= R_ROT + MARGIN

def cone(p, box_half_w, d_face, max_depth=2.0, half_fov=math.radians(29)):
    """Occupied cells in the camera cone, not shadowed by the box silhouette."""
    cx, cy = p[0] + CAM_X * math.cos(p[2]), p[1] + CAM_X * math.sin(p[2])
    shadow = math.atan2(box_half_w, d_face)
    rows, cols = np.where(f['occupied'])
    out = []
    for r, c in zip(rows, cols):
        x = f['origin'][0] + (c + 0.5) * f['res']; y = f['origin'][1] + (f['h'] - 1 - r + 0.5) * f['res']
        dx, dy = x - cx, y - cy
        fwd = dx * math.cos(p[2]) + dy * math.sin(p[2])
        lat = -dx * math.sin(p[2]) + dy * math.cos(p[2])
        if fwd <= 0.35 or fwd > max_depth:
            continue
        b = math.atan2(lat, fwd)
        if abs(b) > half_fov:
            continue
        if abs(b) <= shadow and fwd > d_face:
            continue
        out.append((round(fwd, 2), round(math.degrees(b), 1), round(x, 2), round(y, 2)))
    return sorted(out)

def check(name, F, n, W=0.40, D=0.30, d_obs=0.75, d_pre=1.30, extra=()):
    P = poses(F, n, d_obs, d_pre)
    rep = {'F': F, 'n': n}
    for k in ('pre', 'observation', 'escape', 'align', 'align060'):
        m, ok = rot_ok(P[k]); rep[k] = (tuple(round(v, 3) for v in P[k][:2]), round(m, 3), ok)
    seq = [P['pre'], P['observation'], P['align'], P['final']]
    samp = []
    for a, b in zip(seq, seq[1:]):
        for t in np.linspace(0, 1, 12):
            samp.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2]))
    rep['approach_corridor_clear'] = static_corridor_clear(MAP, KEEP, samp, FOOT)
    # box body free in the map (not occupied/unknown) incl. 0.05 margin
    t = (-n[1], n[0])
    bad = 0
    for a in np.arange(-0.05, D + 0.05 + 1e-9, 0.025):
        for b in np.arange(-W / 2 - 0.05, W / 2 + 0.05 + 1e-9, 0.025):
            x, y = F[0] - n[0] * a + t[0] * b, F[1] - n[1] * a + t[1] * b
            r, c = g.cell(f, x, y)
            bad += int(f['occupied'][r, c] or f['unknown'][r, c])
    rep['box_body_map_conflicts'] = bad
    cam_d = d_obs - CAM_X
    cells = cone(P['observation'], W / 2, cam_d)
    rep['cone_occupied_cells'] = len(cells)
    rep['cone_nearest'] = cells[:5]
    rep['cone_within_1_5m'] = sum(1 for c in cells if c[0] <= 1.5)
    print(name, json.dumps(rep))
    return P

if __name__ == '__main__':
    dock = (-0.6082750451916458, -0.8429646082980815, 1.611153748190869)
    stage = (dock[0] + 0.7 * math.cos(dock[2]), dock[1] + 0.7 * math.sin(dock[2]), dock[2])
    for nm, p in (('dock', dock), ('staging', stage), ('home_exit', (-0.255, 0.2, 0))):
        print(nm, [round(v, 3) for v in p], 'rot', rot_ok(p), 'rot_mapOnly', rot_ok(p, False), 'fp', g.footprint_cells_clear(f, *p))
    check('T1_west_face_1.80_0.05', (1.80, 0.05), (-1, 0))
    check('T1_west_face_1.80_-0.10', (1.80, -0.10), (-1, 0))
    for F in ((2.9, -0.6), (2.9, -0.8), (3.0, -0.95), (2.75, -0.9)):
        check(f'T2_west_{F}', F, (-1, 0))
    for F in ((3.0, -1.10), (3.2, -1.10), (2.9, -1.15), (2.6, -1.10)):
        check(f'T2_north_{F}', F, (0, 1), d_pre=1.30)
