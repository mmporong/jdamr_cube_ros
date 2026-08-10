#!/bin/bash
# GUI 시연: 정책 평가를 화면으로 본다. 시뮬/카메라 창은 이미 떠 있다고 가정.
T="$(cd "$(dirname "$0")" && pwd)"
export DISPLAY="${DISPLAY:-:1}"
source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone
unset ROS_DOMAIN_ID
echo "[$(date +%H:%M)] 정책 평가 5회 시작 — 화면을 보세요"
python "$T/act_eval.py" \
  --ckpt "$T/logs/act_std_gate/checkpoints/last/pretrained_model" \
  --trials 5 --out "$T/logs/act_eval_gui.json" 2>&1 | grep -vE "^  t[0-9]+:"
echo "[$(date +%H:%M)] 시연 종료"
