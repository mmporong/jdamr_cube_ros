"""Read-only layout evaluator on the aligned map: all destinations together."""
import json, math, tempfile, shutil
from pathlib import Path
import numpy as np, cv2, yaml
from PIL import Image
from jdamr_cube_navigation import reverse_parking as rp

A = Path('/home/lim/jdamr_data/map_20260930_manual/aligned')
META = yaml.safe_load(open(A / 'map.yaml')); IMG = np.array(Image.open(A / 'map.pgm'))
RES = META['resolution']; OX, OY = META['origin'][:2]; H, W = IMG.shape
YY, XX = np.mgrid[0:H, 0:W]; CX = OX + (XX + 0.5) * RES; CY = OY + (H - 1 - YY + 0.5) * RES
BASE = IMG < 250
DOCK = json.load(open(A / 'dock_pose.json'))['dock_target']
FP = [(-0.295, -0.29), (0.085, -0.29), (0.085, 0.29), (-0.295, 0.29)]
PRE, OBS, ALIGN, ESC, FINAL = 1.10, 0.60, 0.515, 0.565, 0.115
BW, BD = 0.40, 0.30


def rc(x, y):
    return H - 1 - int(math.floor((y - OY) / RES)), int(math.floor((x - OX) / RES))


def box(face, deg, pad=0.0):
    y = math.radians(deg); n = (math.cos(y), math.sin(y)); t = (-n[1], n[0])
    rx = (CX - face[0]) * n[0] + (CY - face[1]) * n[1]; ry = (CX - face[0]) * t[0] + (CY - face[1]) * t[1]
    return (rx <= pad) & (rx >= -BD - pad) & (np.abs(ry) <= BW / 2 + pad)


def along(d, dist):
    y = math.radians(d['normal_deg']); return (d['face_xy'][0] + dist * math.cos(y), d['face_xy'][1] + dist * math.sin(y))


def evaluate(dest):
    problems, info = [], {}
    boxes = {k: box(d['face_xy'], d['normal_deg']) for k, d in dest.items()}
    allboxes = np.zeros_like(BASE)
    for k, d in dest.items():
        if (BASE & box(d['face_xy'], d['normal_deg'], 0.10)).any():
            problems.append(f'{k}: box area within 0.10 m of mapped cells')
        for j, e in dest.items():
            if j < k and (boxes[j] & box(d['face_xy'], d['normal_deg'], 0.30)).any():
                problems.append(f'{k}/{j}: boxes closer than 0.30 m')
        allboxes |= boxes[k]
    obst = BASE | allboxes
    dist = cv2.distanceTransform((~obst).astype(np.uint8), cv2.DIST_L2, 5) * RES
    hx = (DOCK[0] + 0.70 * math.cos(DOCK[2]), DOCK[1] + 0.70 * math.sin(DOCK[2]))
    pts = {'home_exit': hx}
    for k, d in dest.items():
        for name, s in (('pre', PRE), ('obs', OBS), ('align', ALIGN), ('escape', ESC)):
            pts[f'{k}.{name}'] = along(d, s)
    for name, p in pts.items():
        r, q = rc(*p); c = float(dist[r, q]) if 0 <= r < H and 0 <= q < W else -1.0
        info[name] = round(c, 3)
        if c < 0.514:
            problems.append(f'{name}: rotation clearance {c:.2f} < 0.514')
    tmp = Path(tempfile.mkdtemp()); m2 = IMG.copy(); m2[allboxes] = 0
    Image.fromarray(m2).save(tmp / 'm.pgm'); mm = dict(META); mm['image'] = 'm.pgm'; (tmp / 'm.yaml').write_text(yaml.safe_dump(mm))
    shutil.copy(A / 'keepout.pgm', tmp / 'k.pgm'); kk = dict(META); kk['image'] = 'k.pgm'; (tmp / 'k.yaml').write_text(yaml.safe_dump(kk))
    for k, d in dest.items():
        y = math.radians(d['normal_deg']); hd = math.atan2(-math.sin(y), -math.cos(y))
        poses = [(*along(d, s), hd) for s in np.arange(PRE, FINAL - 0.001, -0.025)]
        if not rp.static_corridor_clear(tmp / 'm.yaml', tmp / 'k.yaml', poses, FP):
            problems.append(f'{k}: straight approach corridor blocked')
        cam = along(d, 0.635); ang = np.arctan2(CY - cam[1], CX - cam[0]) - hd
        ang = np.arctan2(np.sin(ang), np.cos(ang)); rng = np.hypot(CX - cam[0], CY - cam[1])
        cone = (np.abs(ang) <= math.radians(29)) & (rng <= 1.5) & ((IMG < 100) | (allboxes & ~boxes[k]))
        info[f'{k}.cone_cells'] = int(cone.sum()); info[f'{k}.cone_nearest'] = round(float(rng[cone].min()), 2) if cone.any() else None
    stage = (hx[0], hx[1], DOCK[2])
    if not rp.static_corridor_clear(tmp / 'm.yaml', tmp / 'k.yaml', rp.reverse_waypoints(stage, tuple(DOCK), xy_tolerance_m=0.05, yaw_tolerance_rad=math.radians(3)), FP):
        problems.append('dock reverse corridor blocked')
    shutil.rmtree(tmp)
    # transit connectivity: free space eroded by 0.30 m (half chassis width + margin)
    free = (~obst).astype(np.uint8)
    er = cv2.erode(free, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13)))
    n, lab = cv2.connectedComponents(er, connectivity=8)
    comp = lambda p: lab[rc(*p)]
    c0 = comp(hx)
    for name, p in pts.items():
        if name.endswith(('pre', 'obs', 'escape')) and comp(p) != c0:
            problems.append(f'{name}: not connected to home_exit with 0.30 m side clearance')
    return problems, info
