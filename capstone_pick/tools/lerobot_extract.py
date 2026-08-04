"""LeRobot svla_so101_pickplace 에피소드 → 라디안 궤적 npy 추출.

정규화 좌표(관절 -100~100, 그리퍼 0~100)를 공식 so101_new_calib URDF 한계로
라디안 매핑한다 — 우리 팔 관절 정의가 공식과 소수점까지 일치함을 확인했으므로
(2026-08-04 대조) 이 매핑이 좌표 규약 정합의 근거다.

사용 (conda lerobot 환경): python lerobot_extract.py [에피소드=0]
출력: tools/logs/ep{N}_rad.npy
"""
import os
import sys

import numpy as np
import pyarrow.parquet as pq
from huggingface_hub import hf_hub_download

EP = int(sys.argv[1]) if len(sys.argv) > 1 else 0
LIM = [(-1.91986, 1.91986), (-1.74533, 1.74533), (-1.69, 1.69),
       (-1.65806, 1.65806), (-2.74385, 2.84121), (-0.174533, 1.74533)]

p = hf_hub_download("lerobot/svla_so101_pickplace", "data/chunk-000/file-000.parquet",
                    repo_type="dataset")
t = pq.read_table(p, columns=["episode_index", "frame_index", "action"])
ei = np.array(t["episode_index"])
fi = np.array(t["frame_index"])
act = np.stack([np.array(x.as_py()) for x in t["action"]])
m = ei == EP
if not m.any():
    sys.exit(f"에피소드 {EP} 없음")
A = act[m][np.argsort(fi[m])]
R = np.zeros_like(A)
for j, (lo, hi) in enumerate(LIM):
    R[:, j] = ((A[:, j] + 100) / 200 * (hi - lo) + lo) if j < 5 else (A[:, j] / 100 * (hi - lo) + lo)
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs', f'ep{EP}_rad.npy')
os.makedirs(os.path.dirname(out), exist_ok=True)
np.save(out, R)
print(f"ep{EP}: {R.shape[0]}프레임 → {out}")
for j, n in enumerate(['pan', 'lift', 'elbow', 'wrist', 'roll', 'grip']):
    print(f"  {n}: {R[:, j].min():+.2f} ~ {R[:, j].max():+.2f}")
