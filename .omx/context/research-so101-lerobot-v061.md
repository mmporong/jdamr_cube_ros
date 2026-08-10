# Best-Practice Research: SO-101 / LeRobot 픽앤플레이스 기준선

- 조회 기준일: `2026-08-07`
- 공식 기준: LeRobot `v0.6.1`, tag commit `7e241bd630a3719a56157a497ce5d08f244784f1`
- 로컬 기준: `/home/lim/lerobot`, version `0.6.1`, commit `bad0260a461557a09dc3a16a327091dbebd3217d`

## Direct Recommendation

SO-101 데이터/학습/배포의 기준선을 LeRobot v0.6.1에 pin하고, `joint order + units + calibration + Dataset v3 features/FPS/stats + checkpoint-bound pre/postprocessor`를 하나의 불변 contract로 취급한다. LeRobot이 정의하지 않는 모바일 이동 handoff, pick-and-place state machine, 성공 predicate, hardware safety envelope은 프로젝트 ADR로 별도 동결한다.

## Evidence Used

- Official SO-101: <https://huggingface.co/docs/lerobot/en/so101> — 조립, motor ID/baudrate, leader/follower calibration, 6축 구성.
- Official real-world workflow: <https://huggingface.co/docs/lerobot/main/getting_started_real_world_robot> — 동일 robot/teleop ID를 teleop/record/evaluate에서 유지하고 replay로 반복성을 확인.
- Dataset v3: <https://huggingface.co/docs/lerobot/lerobot-dataset-v3> — Parquet low-dimensional signals, MP4 video, feature/FPS/stats/episode/task metadata.
- ACT: <https://huggingface.co/docs/lerobot/en/act> — current joint state와 multi-camera RGB에서 future action chunk 생성.
- Processor contract: <https://huggingface.co/docs/lerobot/en/introduction_processors> — raw observation→preprocessor→policy→postprocessor→robot action.
- Inference: <https://huggingface.co/docs/lerobot/en/inference> — synchronous rollout와 DAgger/intervention 경로.
- Release: <https://github.com/huggingface/lerobot/releases/tag/v0.6.1> — version/date/commit pin.

Upstream v0.6.1 source evidence:

- `src/lerobot/robots/so_follower/so_follower.py:L50-L60` — joint/action/state feature names.
- `so_follower.py:L115-L171` — calibration과 gripper current/overload setup.
- `config_so_follower.py:L31-L53`, `so_follower.py:L219-L230` — optional `max_relative_target` safety clamp.
- `src/lerobot/policies/act/processor_act.py:L29-L50` — normalization/batching/device and action unnormalization.
- `src/lerobot/policies/factory.py:L150-L237` — pretrained processor loading.
- `src/lerobot/datasets/lerobot_dataset.py:L684-L740` — dataset creation and timestamp tolerance context.
- `src/lerobot/datasets/dataset_metadata.py:L641-L681` — default metadata split behavior; project-level held-out seal 필요.

## Version / Date Context

- v0.6.1은 2026-08-03 공개. Dataset v3는 이전 버전에도 있지만 processor/CLI/robot 동작은 버전 민감하므로 `main`을 재현 기준으로 쓰지 않는다.
- 공식 rollout 기본 FPS는 hardware 안전 한계가 아니다.
- SO-101의 공식 최대 안전 속도, payload, collision tolerance, E-stop 수치는 확인되지 않았다.

## Repo-Local Context

- 현재 sim action/state는 rad/sim gripper joint이고 공식 SO-101 follower 기본 표현은 body joint degree와 gripper 0..100이다. 실기 전 typed conversion adapter가 필요하다.
- 현재 evaluator에는 pre/postprocessor 적용이 추가됐지만 run manifest와 observation freshness/sync, gripper lifecycle, fixed success predicate가 없다.
- 현재 Dataset v3 `std_ds`는 37 episodes/27,842 frames가 전부 train이다. 별도 prospective condition/session held-out이 없다.

## Boundaries / Non-goals

- 공식 LeRobot은 이 프로젝트의 base→arm handoff, grasp/place success predicate, target-zone tolerance, retry budget, hardware safety envelope을 정하지 않는다.
- shadow→sim→HIL→real 순서는 안전한 프로젝트 검증 권고이며 LeRobot이 인증한 표준은 아니다.
- 비공식 자료나 임의 수치를 공식 보장처럼 사용하지 않는다.

## Handoff

이 근거는 `.omx/plans/prd-so101-systemic-assurance.md`와 `.omx/specs/test-spec-so101-systemic-assurance.md`에 반영한다. 구현은 Ralplan consensus 후 `$ultragoal` 또는 `$team`에서 수행하며 이 연구 단계는 source를 수정하지 않는다.
