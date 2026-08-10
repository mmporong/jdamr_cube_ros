#!/bin/bash
# 관문 체인: 패킹 완료 → 학습(20K) → 폐루프 평가.
#
# 판정 기준은 "실제 파지·운반 성공 1회 이상"(실좌표). 시늉은 통과가 아니다.
# 지난 판에서는 패킹이 실패했는데도 학습·평가로 넘어가 연쇄로 죽었다.
# 그래서 모든 단계는 산출물의 존재를 확인하고서야 다음으로 간다.
set -o pipefail
TOOLS="$(cd "$(dirname "$0")" && pwd)"
LOG="$TOOLS/logs/gate_chain.log"
DS="$TOOLS/logs/std_ds"
OUT="$TOOLS/logs/act_std_gate"
CKPT="$OUT/checkpoints/last/pretrained_model"
say() { echo "[$(date +%H:%M)] $*" | tee -a "$LOG"; }
die() { say "중단: $*"; systemctl --user stop capstone-sim 2>/dev/null; exit 1; }

say "=== 관문 체인 시작 ==="

# 1) 패킹 완료 대기 (최대 40분)
say "패킹 완료 대기"
for i in $(seq 1 80); do
  pgrep -f lerobot_pack.py >/dev/null || break
  sleep 30
done
pgrep -f lerobot_pack.py >/dev/null && die "패킹이 40분 내 끝나지 않음"

NEP=$(python3 -c "import json;print(json.load(open('$DS/meta/info.json'))['total_episodes'])" 2>/dev/null)
NFR=$(python3 -c "import json;print(json.load(open('$DS/meta/info.json'))['total_frames'])" 2>/dev/null)
say "데이터셋: ${NEP:-0}편 ${NFR:-0}프레임"
[ "${NEP:-0}" -lt 20 ] && die "에피소드 부족 (${NEP:-0}편)"

# 2) 학습 — LeRobot 기본 설정 그대로(VAE 포함). 표준을 벗어나지 않는다.
say "학습 시작 (20K steps, ACT 기본)"
source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot
rm -rf "$OUT"
# conda activate를 유닛 '안에서' 해야 한다. PATH만 넘기면 CUDA 런타임
# 라이브러리 경로가 빠져 torch.cuda.is_available()이 False가 되고, 학습이
# 조용히 CPU로 떨어진다(실측: 6.0 s/step = 20K에 34시간).
systemd-run --user --collect --unit=capstone-train bash -c \
  "source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot && cd /tmp && \
   lerobot-train --policy.type=act \
    --dataset.repo_id=local/so101_rule_pick --dataset.root='$DS' \
    --output_dir='$OUT' --steps=20000 --batch_size=8 --save_freq=5000 \
    --policy.device=cuda --wandb.enable=false --policy.push_to_hub=false \
    > '$TOOLS/logs/act_std_gate_train.log' 2>&1" >/dev/null 2>&1

sleep 150
grep -qE "Error|Traceback" "$TOOLS/logs/act_std_gate_train.log" 2>/dev/null && {
  tail -5 "$TOOLS/logs/act_std_gate_train.log" | tr '\r' '\n' >> "$LOG"; die "학습이 시작 직후 실패"; }
# GPU에 실제로 올라갔는지 본다. 로그만 보면 CPU 학습을 놓친다.
VRAM=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)
say "GPU 점유 ${VRAM}MiB"
[ "${VRAM:-0}" -lt 500 ] && { systemctl --user stop capstone-train; die "GPU 미사용 (${VRAM}MiB) — CPU 학습으로 떨어짐"; }
say "학습 진행 확인됨 — 완료 대기 (예상 1.7시간)"

while systemctl --user is-active capstone-train >/dev/null 2>&1; do
  sleep 600
  # 'loss:' 로 뽑으면 같은 줄의 l1_loss/kld_loss까지 걸려 마지막(kld)을 집는다.
  # 실측: kld 0.016을 총 loss로 잘못 보고했다. 앞에 공백을 붙여 총 loss만 뽑는다.
  L=$(grep -oE " loss:[0-9.]+ | l1_loss:[0-9.]+" "$TOOLS/logs/act_std_gate_train.log" 2>/dev/null | tail -2 | tr -d '\n')
  S=$(grep -oE "step:[0-9KM]+" "$TOOLS/logs/act_std_gate_train.log" 2>/dev/null | tail -1)
  say "  $S $L"
done
[ -d "$CKPT" ] || die "체크포인트가 만들어지지 않음"
say "학습 완료 ($(grep -oE ' loss:[0-9.]+ | l1_loss:[0-9.]+' "$TOOLS/logs/act_std_gate_train.log" | tail -2 | tr -d '\n'))"

# 3) 시뮬 기동
say "시뮬 기동"
systemctl --user stop capstone-sim 2>/dev/null; sleep 3
systemd-run --user --collect --setenv=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --setenv=GZ_PARTITION=lim-capstone --unit=capstone-sim bash -c \
  'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && exec ros2 launch jdamr_cube_gazebo gazebo.launch.py gui:=false world:=$HOME/jdamr_cube_ws/install/jdamr_cube_gazebo/share/jdamr_cube_gazebo/worlds/room.world' >/dev/null 2>&1
N=0
for i in $(seq 1 40); do
  N=$(ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone bash -c \
      'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; source ~/jdamr_cube_ws/install/setup.bash >/dev/null 2>&1; timeout 10 ros2 control list_controllers 2>/dev/null | grep -c active')
  [ "${N:-0}" = "3" ] && break
  sleep 5
done
[ "${N:-0}" = "3" ] || die "컨트롤러가 뜨지 않음 (${N:-0}/3)"
say "시뮬 준비 완료"

# 4) 폐루프 평가 — 필터로 에러를 가리지 않는다
say "평가 10회 시작 (기준: 실제 파지 1회 이상)"
source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone
unset ROS_DOMAIN_ID
timeout 2400 python "$TOOLS/act_eval.py" --ckpt "$CKPT" --trials 10 \
  --out "$TOOLS/logs/act_eval_gate.json" 2>&1 | tee -a "$LOG" \
  | grep -E "^\[[0-9]+/|성공|Error|Traceback"

systemctl --user stop capstone-sim 2>/dev/null
say "=== 관문 체인 종료 (시뮬 정지) ==="
