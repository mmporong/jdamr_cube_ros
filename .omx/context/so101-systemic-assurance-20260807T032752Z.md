# SO-101 시스템 신뢰성 감사 컨텍스트

- 생성 UTC: `2026-08-07T03:27:52Z`
- 작업 루트: `/home/lim/jdamr_cube_ws/src/jdamr_cube_ros`
- Git 기준: `9b1f92858ba0a1dac397c3d8b9bb160725135dfe`, dirty worktree
- 계획 모드: `$ralplan` deliberate, 구현 소스 수정 금지

## 작업 진술

반복 테스트 실패, 검증되지 않은 데이터/성공 판정, 국소 패치 누적 때문에 SO-101 모바일 매니퓰레이터의 이동·픽앤플레이스·모방학습 결과를 신뢰하기 어렵다. SO-101/LeRobot 공식 계약과 현재 코드·데이터·로그를 대조하고, 근본 원인을 계층별로 제거하는 구현·테스트·승격 계획을 만든다.

## 원하는 결과

1. 데이터 한 행부터 실기 성공률까지 동일한 버전·단위·시간·조인트·processor·평가 계약으로 추적된다.
2. 규칙 기반 이동/픽앤플레이스는 물리 ground truth로 검증된 기준선 역할을 한다.
3. ACT는 기준선과 분리된 후보 lane에서 offline→replay→sim→HIL→real gate를 순서대로 통과한다.
4. 누락값 대체, 길이 잘라내기, stale 경로, 사후 성공 기준 변경은 fail-closed로 차단된다.
5. 테스트 결과가 실패하면 최초 위반 계약과 증거 digest로 원인이 좁혀진다.

## 현재 확인된 사실

### 저장소와 변경 상태

- 실행 소스는 `capstone_pick/`이며 `/home/lim/gazebo-so101-capstone`은 문서/데모 저장소다.
- 현재 worktree에는 `pick_node.py`, `act_eval.py`, `rule_collect.py`, `lerobot_pack.py`, URDF/world/launch 등 큰 미커밋 변경과 여러 미추적 도구가 있다. 이 계획은 이를 사용자 작업으로 보존한다.
- `capstone_pick`에는 일반 자동 테스트가 사실상 `tools/smoke_test.py` 하나뿐이며 package-level unit/contract test가 없다.
- 이전 감사 `/home/lim/capstone_review2`는 R2-U00~U06과 T00~T12 합의 계획을 완료했다. 이번 계획은 그 증거를 폐기하지 않고 현재 실행 트리의 변경과 전체 이동/픽앤플레이스 범위를 추가한다.

### 현재 코드/데이터의 신뢰성 문제

- `capstone_pick/tools/act_eval.py:105-120,261-263`에는 checkpoint processor 적용이 추가됐다. 과거 processor 누락 finding은 현재 파일 기준으로는 수정 후보 상태다.
- 그러나 `capstone_pick/tools/rule_collect.py:289-293`은 관절/desired reference/gripper command 누락 시 측정 state를 action으로 대체한다. 실제 명령과 대체값이 구분되지 않는다.
- `capstone_pick/tools/rule_collect.py:297-312`는 종료 코드와 XY 이동 50 mm만으로 시연 성공을 정한다. push, lift-drop, 잘못된 place를 구분하지 않는다.
- `capstone_pick/tools/lerobot_pack.py:61-69`는 state/action/카메라 길이가 달라도 최솟값으로 잘라 패킹한다. 0프레임만 막고 부분 손실은 통과시킨다.
- `capstone_pick/tools/act_eval.py:125-132,230-272`는 최신 관측 보유 여부만 보며 camera/joint timestamp age·skew·gap을 검증하지 않는다.
- `capstone_pick/tools/act_eval.py:151-170`은 gripper goal 수명주기(accept/result/cancel/reap)를 끝까지 관리하지 않는다.
- `capstone_pick/tools/act_eval.py:275-305`의 결과에는 code/dataset/checkpoint/processor/split/seed/success-predicate digest가 없다.
- `capstone_pick/tools/traj_diag.py:17-31`은 첫 demo의 관절 min/max만 비교해 최초 divergence, phase/time alignment, command-tracking error를 찾지 못한다.
- `tools/logs/std_ds/meta/info.json`은 37 episodes/27,842 frames를 모두 `train`으로 표시한다. 별도 prospective held-out 계약이 없다.
- 로컬 메타 기준 `rule_std` 37편/27,842프레임, `rule_yaw45` 60편/49,814프레임이 저장돼 있으나 현재 success label이 이동량 기반이므로 진짜 grasp/place 데이터라고 승격할 수 없다.
- `act_eval_gate.json`의 0/10과 `act_eval_gui.json`의 4/5는 success predicate와 코드 lineage를 봉인하지 않아 서로 비교 가능한 성능 증거가 아니다.

### 공식/업스트림 근거

- 기준 버전: LeRobot `v0.6.1`, tag commit `7e241bd630a3719a56157a497ce5d08f244784f1`; 로컬 editable source는 같은 `0.6.1` 계열 commit `bad0260a...`.
- SO-101 공식 문서: <https://huggingface.co/docs/lerobot/en/so101>
- Dataset v3: <https://huggingface.co/docs/lerobot/lerobot-dataset-v3>
- ACT: <https://huggingface.co/docs/lerobot/en/act>
- processor 계약: <https://huggingface.co/docs/lerobot/en/introduction_processors>
- inference: <https://huggingface.co/docs/lerobot/en/inference>
- v0.6.1 릴리스: <https://github.com/huggingface/lerobot/releases/tag/v0.6.1>
- 공식 follower 축은 `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`다. 실물 기본 표현은 body joint degree, gripper 0..100이고 현재 ROS/Gazebo 경계는 rad/시뮬 gripper joint이므로 명시적 adapter가 필요하다.
- 공식 배포 순서는 raw observation→preprocessor→`select_action()`→postprocessor→robot action이다.
- Dataset v3가 생성하는 기본 split은 train 중심이므로 session/condition held-out은 프로젝트 manifest가 별도로 보장해야 한다.
- LeRobot은 프로젝트별 pick-and-place 상태기계, 성공 predicate, 안전 속도/payload/E-stop 수치를 보장하지 않는다. 해당 값은 실측 ADR과 gate로만 확정한다.

## 제약

- 현재 dirty worktree와 기존 로그/데이터/checkpoint를 삭제·덮어쓰지 않는다.
- source/config 수정은 합의 후 별도 실행 lane에서만 한다.
- 새 의존성은 사용자가 명시적으로 승인하지 않는 한 추가하지 않는다.
- 시뮬 ground truth는 label/evaluation evidence로 사용할 수 있지만 policy observation에 섞지 않는다.
- synthetic/mock fixture는 테스트 전용으로 명시하고 실제 성공률 분모/분자에 포함하지 않는다.
- 실기/hardware authorization 기본값은 `false`다.

## 미확정 사항

- 현재 미커밋 변경 각각의 작성 의도와 어떤 실행 로그가 어떤 source digest에서 나왔는지.
- 실제 SO-101 follower calibration/firmware/camera/latency/payload/safety envelope.
- 모바일 base→arm handoff pose와 현장 target-zone product tolerance.
- 실기 승격에 필요한 sample size/minimum success/Wilson lower bound. 데이터 열람 전에 별도 ADR로 사전 동결해야 한다.

## 주요 touchpoint

- 규칙 기반 state machine: `capstone_pick/capstone_pick/pick_node.py:297-2052`
- perception/좌표: `pick_node.py:508-635,1058-1454`
- 이동/운반/place: `pick_node.py:761-1057,1535-2052`
- 수집: `capstone_pick/tools/rule_collect.py:168-377`
- 패킹: `capstone_pick/tools/lerobot_pack.py:24-81`
- 평가: `capstone_pick/tools/act_eval.py:87-311`
- 진단: `capstone_pick/tools/traj_diag.py:1-31`, `capstone_pick/tools/trace_table.py`
- 물리/제어: `jdamr_cube_description/urdf/jdamr_cube.urdf`, `jdamr_cube_description/config/so101_controllers.yaml`, `jdamr_cube_gazebo/worlds/room.world`, `jdamr_cube_gazebo/launch/gazebo.launch.py`

## 계획 가설

문제의 핵심은 단일 모델 hyperparameter가 아니라 서로 다른 truth source, 단위, 시간축, action authority, 성공 predicate, artifact lineage가 한 결과에 섞이는 것이다. 해결책은 규칙 기반 기준선과 학습 후보를 분리하고, 공통 typed contract와 단계별 promotion gate를 먼저 만든 뒤 데이터 재수집/재학습을 허용하는 것이다.

## 중단 조건

Architect→Critic 순서의 승인과 durable handoff가 기록되면 Ralplan을 종료한다. 구현·재수집·재학습·실기 실행은 이 세션에서 하지 않는다.
