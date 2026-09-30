"""Grid search of box face placements on the current map (read-only, offline)."""
import math, sys, json
import numpy as np, cv2, yaml
from pathlib import Path
sys.path.insert(0, '.')
import layout_probe as L
import geometry_check as g

# map-only variant: all-free keepout copy in scratch (keepout will be redesigned)
src = yaml.safe_load(open(L.KEEP))
img = cv2.imread(str(L.KEEP.parent / src['image']), cv2.IMREAD_UNCHANGED)
free = np.full_like(img, 254)
cv2.imwrite('keepout_allfree_scratch.pgm', free)
yaml.safe_dump(dict(src, image='keepout_allfree_scratch.pgm'), open('keepout_allfree_scratch.yaml', 'w'))
MODE = sys.argv[1] if len(sys.argv) > 1 else 'map_only'
if MODE == 'map_only':
    L.KEEP = Path('keepout_allfree_scratch.yaml').resolve()
    L.f = g.fields(str(L.MAP), str(L.KEEP))
dock = (-0.6082750451916458, -0.8429646082980815, 1.611153748190869)
res = []
import io, contextlib
for nx, ny in ((-1, 0), (1, 0), (0, 1), (0, -1)):
    for fx in np.arange(0.3, 3.95, 0.1):
        for fy in np.arange(-1.45, 0.65, 0.1):
            F = (round(fx, 2), round(fy, 2)); n = (nx, ny)
            P = L.poses(F, n)
            try:
                ok = all(L.rot_ok(P[k])[1] for k in ('pre', 'observation', 'escape', 'align'))
            except IndexError:
                ok = False
            if not ok:
                continue
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                L.check('x', F, n)
            rep = json.loads(buf.getvalue().split(' ', 1)[1])
            if rep['approach_corridor_clear'] and rep['box_body_map_conflicts'] == 0:
                res.append((rep['cone_within_1_5m'], rep['cone_occupied_cells'], F, n,
                            min(rep[k][1] for k in ('pre', 'observation', 'escape', 'align')),
                            round(math.dist(F, dock[:2]), 2)))
res.sort(key=lambda r: (r[0], r[1], -r[4]))
print(MODE, 'candidates', len(res))
for r in res[:25]:
    print(r)
