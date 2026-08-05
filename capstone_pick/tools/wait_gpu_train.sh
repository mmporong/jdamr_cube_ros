#!/bin/bash
# GPU 여유(5GB+)가 생기면 ACT 학습 시작. 다른 사용자 작업을 죽이지 않고 기다린다.
for i in $(seq 1 240); do
  FREE=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits)
  if [ "${FREE:-0}" -gt 5000 ]; then
    echo "$(date +%H:%M) GPU 여유 ${FREE}MiB — 학습 시작"
    systemd-run --user --collect --unit=capstone-train bash -c 'source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot && cd /tmp && exec lerobot-train --policy.type=act --dataset.repo_id=local/so101_rule_pick --dataset.root=/home/lim/jdamr_cube_ws/src/jdamr_cube_ros/capstone_pick/tools/logs/rule_ds_v1 --output_dir=/home/lim/jdamr_cube_ws/src/jdamr_cube_ros/capstone_pick/tools/logs/act_rule_v1 --steps=20000 --batch_size=8 --save_freq=5000 --policy.device=cuda --wandb.enable=false --policy.push_to_hub=false > /home/lim/jdamr_cube_ws/src/jdamr_cube_ros/capstone_pick/tools/logs/act_rule_v1_train.log 2>&1'
    exit 0
  fi
  sleep 30
done
echo "$(date +%H:%M) 2시간 대기 초과 — 수동 확인 필요"
