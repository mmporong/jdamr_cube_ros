"""정책 궤적 vs 시연 궤적 대조 — 어디서 벗어나는지 특정한다.

평가가 실패했을 때 "정책이 무엇을 잘못했나"를 숫자로 답하기 위한 도구.
시연은 접힘→상공→하강→파지→들기의 정해진 흐름을 따르므로, 정책 궤적이
그 흐름의 어느 단계에서 갈라지는지 보면 원인이 좁혀진다.

사용: python3 traj_diag.py [시행번호=0]
"""
import glob
import sys

import numpy as np

TRIAL = int(sys.argv[1]) if len(sys.argv) > 1 else 0
NAMES = ['pan', 'lift', 'elbow', 'wrist', 'roll', 'grip']

t = np.load(f'logs/eval_traj_{TRIAL}.npy')          # [el, state6, action6]
el, st, ac = t[:, 0], t[:, 1:7], t[:, 7:13]
print(f'정책 궤적: {len(t)}틱, {el[-1]:.1f}초')
print(f'  관절 범위: ' + ' '.join(f'{n} {st[:,i].min():+.2f}~{st[:,i].max():+.2f}' for i, n in enumerate(NAMES)))
print(f'  그리퍼 닫힘 시도: {(ac[:,5] < 0.05).sum()}틱 / 열림 {(ac[:,5] > 0.4).sum()}틱')

demo = np.load(sorted(glob.glob('logs/rule_collect_static/ep*/state.npy'))[0])
print(f'\n시연 궤적(ep000): {len(demo)}틱')
print(f'  관절 범위: ' + ' '.join(f'{n} {demo[:,i].min():+.2f}~{demo[:,i].max():+.2f}' for i, n in enumerate(NAMES)))

print('\n관절별 이탈 (정책이 시연 범위를 벗어난 비율)')
for i, n in enumerate(NAMES):
    lo, hi = demo[:, i].min(), demo[:, i].max()
    out = ((st[:, i] < lo - 0.05) | (st[:, i] > hi + 0.05)).mean()
    print(f'  {n:6s}: {out*100:5.1f}%  (시연 {lo:+.2f}~{hi:+.2f})')
