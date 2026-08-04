"""수집물(logs/collect/ep*) → LeRobot v3 데이터셋 패킹.

성공 에피소드만(기본) 골라 로컬 데이터셋을 만든다 — 학습(lerobot-train)의 입력.
사용 (conda lerobot 환경):
  python lerobot_pack.py [--all] [--limit N] [--out logs/lerobot_ds]
"""
import glob
import json
import os
import sys

import cv2
import numpy as np

TOOLS = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(TOOLS, 'logs',
                   sys.argv[sys.argv.index('--out') + 1] if '--out' in sys.argv else 'lerobot_ds')
LIMIT = int(sys.argv[sys.argv.index('--limit') + 1]) if '--limit' in sys.argv else 10**9
TASK = "Pick up the lego block and put it in the box."
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


def main():
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    eps = []
    for m in sorted(glob.glob(os.path.join(TOOLS, 'logs', 'collect', 'ep*', 'meta.json'))):
        meta = json.load(open(m))
        if '--all' in sys.argv or meta.get('success') or meta.get('lifted'):
            eps.append((os.path.dirname(m), meta))
    eps = eps[:LIMIT]
    if not eps:
        sys.exit('패킹할 에피소드 없음 (성공 0건이면 --all)')
    print(f'패킹 대상 {len(eps)}개: ' + ', '.join(os.path.basename(d) for d, _ in eps))

    if os.path.exists(OUT):
        import shutil
        shutil.rmtree(OUT)
    features = {
        'observation.images.up': {'dtype': 'video', 'shape': (480, 640, 3),
                                  'names': ['height', 'width', 'channels']},
        'observation.images.side': {'dtype': 'video', 'shape': (480, 640, 3),
                                    'names': ['height', 'width', 'channels']},
        'observation.state': {'dtype': 'float32', 'shape': (6,), 'names': JOINTS},
        'action': {'dtype': 'float32', 'shape': (6,), 'names': JOINTS},
    }
    ds = LeRobotDataset.create('local/so101_sim_pickplace', fps=20, features=features,
                               root=OUT, robot_type='so101_sim', use_videos=True)
    for d, meta in eps:
        state = np.load(os.path.join(d, 'state.npy')).astype(np.float32)
        action = np.load(os.path.join(d, 'action.npy')).astype(np.float32)
        ups = sorted(glob.glob(os.path.join(d, 'demo_up', '*.jpg')))
        sides = sorted(glob.glob(os.path.join(d, 'demo_side', '*.jpg')))
        n = min(len(state), len(action), len(ups), len(sides))
        for i in range(n):
            ds.add_frame({
                'observation.images.up': cv2.cvtColor(cv2.imread(ups[i]), cv2.COLOR_BGR2RGB),
                'observation.images.side': cv2.cvtColor(cv2.imread(sides[i]), cv2.COLOR_BGR2RGB),
                'observation.state': state[i],
                'action': action[i],
                'task': TASK,
            })
        ds.save_episode()
        print(f'  {os.path.basename(d)}: {n}프레임 저장 (success={meta.get("success")})')
    print(f'완료 → {OUT}')


if __name__ == '__main__':
    main()
