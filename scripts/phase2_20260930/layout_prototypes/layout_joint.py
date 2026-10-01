"""Joint two-box probe on the current map, keepout ignored (minimal-keepout assumption)."""
import math, sys, json, io, contextlib
import numpy as np
from pathlib import Path
sys.path.insert(0, '.')
import layout_probe as L
import geometry_check as g
L.KEEP = Path('keepout_allfree_scratch.yaml').resolve()
L.f = g.fields(str(L.MAP), str(L.KEEP))
W, D = 0.40, 0.30
dock = (-0.6082750451916458, -0.8429646082980815, 1.611153748190869)
stage = (dock[0] + 0.7 * math.cos(dock[2]), dock[1] + 0.7 * math.sin(dock[2]))

def cands():
    out = []
    for n in ((-1, 0), (1, 0), (0, 1), (0, -1)):
        for fx in np.arange(0.0, 3.95, 0.1):
            for fy in np.arange(-1.45, 0.75, 0.1):
                F = (round(fx, 2), round(fy, 2)); P = L.poses(F, n)
                try:
                    if not all(L.rot_ok(P[k])[1] for k in ('pre', 'observation', 'escape', 'align')):
                        continue
                except IndexError:
                    continue
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    L.check('x', F, n, W=W, D=D)
                rep = json.loads(buf.getvalue().split(' ', 1)[1])
                if (rep['approach_corridor_clear'] and rep['box_body_map_conflicts'] == 0
                        and rep['cone_within_1_5m'] == 0):
                    out.append((F, n, P, rep['cone_occupied_cells']))
    return out

def box_poly(F, n, pad):
    t = (-n[1], n[0])
    pts = []
    for a, b in ((-pad, -W / 2 - pad), (-pad, W / 2 + pad), (D + pad, W / 2 + pad), (D + pad, -W / 2 - pad)):
        pts.append((F[0] - n[0] * a + t[0] * b, F[1] - n[1] * a + t[1] * b))
    return pts

def inside(p, poly):
    x, y = p; c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            c = not c
    return c

def conflict(boxF, boxn, P):
    """Other box (inflated by rotation radius+margin) vs this table's poses/corridor."""
    poly_rot = box_poly(boxF, boxn, L.R_ROT + L.MARGIN)
    poly_pass = box_poly(boxF, boxn, 0.29 + 0.10)
    for k in ('pre', 'observation', 'escape', 'align'):
        if inside(P[k][:2], poly_rot):
            return True
    seq = [P['pre'], P['observation'], P['align'], P['final']]
    for a, b in zip(seq, seq[1:]):
        for t in np.linspace(0, 1, 15):
            if inside((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t), poly_pass):
                return True
    # straight return escape->staging corridor and dock staging
    for t in np.linspace(0, 1, 30):
        q = (P['escape'][0] + (stage[0] - P['escape'][0]) * t, P['escape'][1] + (stage[1] - P['escape'][1]) * t)
        if inside(q, poly_pass):
            return True
    return inside(stage, poly_rot)

def dock_frame(p):
    dx, dy = p[0] - dock[0], p[1] - dock[1]
    c, s = math.cos(dock[2]), math.sin(dock[2])
    return round(dx * c + dy * s, 2), round(-dx * s + dy * c, 2)

C = cands()
print('single candidates', len(C))
pairs = []
for i, (F1, n1, P1, c1) in enumerate(C):
    for F2, n2, P2, c2 in C[i + 1:]:
        if math.dist(F1, F2) < 1.0:
            continue
        if conflict(F2, n2, P1) or conflict(F1, n1, P2):
            continue
        m1 = min(L.rot_ok(P1[k])[0] for k in ('pre', 'observation', 'escape', 'align'))
        m2 = min(L.rot_ok(P2[k])[0] for k in ('pre', 'observation', 'escape', 'align'))
        pairs.append((c1 + c2, -min(m1, m2), F1, n1, F2, n2))
pairs.sort()
print('pairs', len(pairs))
for p in pairs[:12]:
    c, m, F1, n1, F2, n2 = p
    print(c, -m, F1, n1, dock_frame(F1), F2, n2, dock_frame(F2))
