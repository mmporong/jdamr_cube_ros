#!/bin/bash
# 체크포인트마다 '공식 경로'로 재현 오차를 잰다.
#
# preprocessor → select_action → postprocessor (lerobot_eval.py:288-300).
# 이 두 단계를 빼면 정규화 공간의 숫자를 raw와 비교하게 되어 21배 부풀려진다
# (실측: 같은 5K 체크포인트에서 0.5566 → 0.0262).
T="$(cd "$(dirname "$0")" && pwd)"
source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot
for CK in 010000 015000 020000; do
  D="$T/logs/act_std_gate/checkpoints/$CK/pretrained_model"
  until [ -d "$D" ]; do
    systemctl --user is-active capstone-train >/dev/null 2>&1 || exit 0
    sleep 60
  done
  sleep 15
  python - "$D" "$CK" << 'PY' 2>&1 | grep -vE "WARNING|torchcodec|Loading weights"
import sys, torch, numpy as np, os
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
D, CK = sys.argv[1], sys.argv[2]
T = os.path.expanduser('~/jdamr_cube_ws/src/jdamr_cube_ros/capstone_pick/tools')
p = ACTPolicy.from_pretrained(D); p.to('cuda').eval()
pre, post = make_pre_post_processors(policy_cfg=p.config, pretrained_path=D)
dt = {'action': [i / 20 for i in range(p.config.chunk_size)]}
ds = LeRobotDataset('local/so101_rule_pick', root=f'{T}/logs/std_ds', delta_timestamps=dt)
K = ('observation.images.front', 'observation.images.wrist', 'observation.state')
arm, grip = [], []
for i in (500, 4000, 9000, 14000, 20000, 25000):
    s = ds[i]
    raw = {k: s[k].unsqueeze(0) for k in K}
    p.reset()
    with torch.inference_mode():
        a = post(p.select_action(pre(dict(raw)))).squeeze(0).cpu().numpy()
    t = s['action'].numpy()[0]
    arm.append(np.abs(a[:5] - t[:5]).mean()); grip.append(abs(a[5] - t[5]))
print(f'[eval-trend] {CK}: 공식경로 재현 l1  팔={np.mean(arm):.4f}  그리퍼={np.mean(grip):.4f}')
PY
done
