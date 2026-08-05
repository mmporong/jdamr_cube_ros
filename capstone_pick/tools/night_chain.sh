#!/bin/bash
# 야간 자동 체인: 학습 완료 감지 → 시뮬 기동 → 폐루프 평가 → 결과 기록.
# 사람이 지켜보지 않아도 다음 단계로 넘어가도록 묶는다.
#
# 사용: night_chain.sh <버전태그>   예) night_chain.sh v3
TAG=${1:-v3}
TOOLS="$(cd "$(dirname "$0")" && pwd)"
LOG="$TOOLS/logs/night_chain_${TAG}.log"
CKPT="$TOOLS/logs/act_rule_${TAG}/checkpoints/last/pretrained_model"
TRAINLOG="$TOOLS/logs/act_rule_${TAG}_train.log"
RESULT="$TOOLS/logs/act_eval_${TAG}.json"

say() { echo "[$(date +%H:%M)] $*" | tee -a "$LOG"; }

say "야간 체인 시작 (tag=$TAG)"

# 1) 학습 완료 대기 (최대 4시간)
for i in $(seq 1 480); do
  if grep -q "End of training" "$TRAINLOG" 2>/dev/null; then
    say "학습 완료 감지"
    break
  fi
  if ! systemctl --user is-active capstone-train >/dev/null 2>&1; then
    if grep -q "End of training" "$TRAINLOG" 2>/dev/null; then break; fi
    say "학습 유닛이 종료됨 (완료 표시 없음) — 로그 확인 필요"
    tail -5 "$TRAINLOG" | tr '\r' '\n' | tail -3 >> "$LOG"
    exit 1
  fi
  sleep 30
done

LOSS=$(grep -oE "loss:[0-9.]+" "$TRAINLOG" | tail -1)
say "최종 $LOSS"

# 2) 학습 데이터 재현 검증 (평가 전에 정책이 뭐라도 배웠는지 먼저 본다)
say "재현 검증 시작"
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python - << PYEOF 2>&1 | grep -vE "WARNING|Loading|^$" | tee -a "$LOG"
import torch, numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
ds = LeRobotDataset('local/so101_rule_pick', root='$TOOLS/logs/rule_ds_v2')
p = ACTPolicy.from_pretrained('$CKPT'); p.to('cuda').eval()
e=[]
for i in (200, 3000, 8000, 12000):
    s = ds[i]
    b = {k: s[k].unsqueeze(0).to('cuda') for k in ('observation.images.front','observation.images.wrist','observation.state')}
    p.reset()
    with torch.no_grad(): a = p.select_action(b).squeeze(0).cpu().numpy()
    e.append(abs(a - s['action'].numpy()).mean())
print(f'재현 l1 = {np.mean(e):.4f}  (0.05 이하 정상 / 0.3 이상이면 학습 실패)')
PYEOF

# 3) 시뮬 기동
say "시뮬 기동"
systemctl --user stop capstone-sim 2>/dev/null
systemd-run --user --collect \
  --setenv=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST --setenv=GZ_PARTITION=lim-capstone \
  --unit=capstone-sim bash -c \
  'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && exec ros2 launch jdamr_cube_gazebo gazebo.launch.py gui:=false world:=$HOME/jdamr_cube_ws/install/jdamr_cube_gazebo/share/jdamr_cube_gazebo/worlds/room.world' \
  >/dev/null 2>&1
for i in $(seq 1 40); do
  N=$(ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone bash -c \
      'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; source ~/jdamr_cube_ws/install/setup.bash >/dev/null 2>&1; timeout 10 ros2 control list_controllers 2>/dev/null | grep -c active')
  [ "${N:-0}" = "3" ] && break
  sleep 5
done
say "시뮬 준비 (컨트롤러 ${N:-0}개)"

# 4) 폐루프 평가
say "폐루프 평가 10회 시작"
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone
unset ROS_DOMAIN_ID
timeout 1800 python "$TOOLS/act_eval.py" --ckpt "$CKPT" --trials 10 --out "$RESULT" 2>&1 \
  | grep -E "^\[[0-9]+/|성공률" | tee -a "$LOG"

systemctl --user stop capstone-sim 2>/dev/null
say "야간 체인 종료 (시뮬 정지)"
