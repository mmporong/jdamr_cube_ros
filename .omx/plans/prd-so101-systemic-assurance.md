# SO-101 이동·픽앤플레이스 시스템 신뢰성 개선 PRD

- 상태: Final candidate, Architect fallback iteration 3 승인·Critic 재평가 대기
- 모드: RALPLAN-DR deliberate
- 기준 컨텍스트: `.omx/context/so101-systemic-assurance-20260807T032752Z.md`
- 적용 루트: `/home/lim/jdamr_cube_ws/src/jdamr_cube_ros`
- 구현 권한: 없음; 이 문서는 계획 산출물이다.

## 1. 요구사항 요약

현재 시스템의 이동, 인지, 접근, 파지, 운반, 배치, 데이터 수집, LeRobot 패킹, ACT 학습/평가를 하나의 검증 가능한 계약으로 재구성한다. 국소 패치가 결과 정의를 바꾸거나 stale/fallback 데이터를 통과시키지 못하게 하고, 실패가 발생하면 어느 계층의 계약이 깨졌는지 자동으로 식별한다.

완료 결과는 다음 네 가지다.

1. **검증된 규칙 기반 기준선:** 물리 ground truth와 단계별 trace가 일치하는 이동·pick·place oracle.
2. **신뢰 가능한 데이터 계보:** raw observation, accepted command, controller reference, achieved state, object truth, label, dataset, checkpoint, processor, evaluation이 digest로 연결됨.
3. **격리된 학습 후보:** ACT 결과가 기준선을 덮어쓰지 않고 offline→replay→sim→HIL→real gate를 순서대로 통과함.
4. **독립 검수 가능성:** 새 환경에서 manifest만으로 같은 판정과 보고서를 재생성함.

## 2. 범위와 비목표

### 범위

- `capstone_pick/capstone_pick/pick_node.py`의 이동·인지·grasp·carry·place 상태 전이 및 실패 복구
- ROS 2/Gazebo의 단위, 좌표계, 시계, controller reference, joint/gripper 안전 경계
- `rule_collect.py`→`lerobot_pack.py`→LeRobot Dataset v3→ACT checkpoint/processors→`act_eval.py` 전 계약
- 기존 `rule_std`, `rule_yaw45`, `std_ds`, checkpoint, 평가 JSON의 격리·재분류
- unit/contract/component/sim e2e/HIL/실기 승격 테스트와 observability

### 비목표

- 이 Ralplan 세션에서 source 코드 수정, 기존 데이터 삭제, 재학습, 시뮬/실기 actuator 실행
- 현재 로그의 성공률을 진짜 모델 성능으로 소급 인정
- 공식 근거 없는 SO-101 속도·payload·토크·E-stop 수치 발명
- Nav2/MoveIt 도입 자체를 목표로 삼기
- ACT hyperparameter 튜닝으로 계약 결함을 우회하기

## 3. 증거 등급과 불변 원칙

모든 artifact/row/event는 다음 중 하나의 `evidence_class`를 가진다.

- `measured`: 센서/컨트롤러/실물에서 직접 측정
- `sim_ground_truth`: simulator object/contact/support truth; label/evaluation 전용
- `derived`: 해시된 입력과 결정적 변환에서 생성
- `human_label`: 작성자·시각·근거가 있는 수동 판정
- `synthetic_fixture`: unit/hostile test 전용; 성능 통계 금지
- `unknown`: 승격 금지

원칙:

1. **Evidence before score:** lineage와 predicate가 없는 숫자는 성능이 아니라 관찰값이다.
2. **Fail closed:** missing/stale/skew/shape/unit/digest mismatch를 0, measured state, nearest future, truncation으로 대체하지 않는다.
3. **One semantic contract:** joint order·units·clock·action authority·processor·success predicate를 raw부터 rollout까지 동일하게 유지한다.
4. **Oracle/candidate separation:** 규칙 기반 기준선과 ACT 후보의 결과·artifact·승격 상태를 분리한다.
5. **Staged promotion:** offline을 통과하지 못한 후보는 actuator를 초기화하지 않는다.

## 4. RALPLAN-DR

### Decision Drivers

1. 반복 실패의 최초 원인을 계층별로 특정할 수 있는가.
2. 데이터와 성공률이 조작·누락·사후 기준 변경 없이 재현되는가.
3. 시뮬에서 실기로 갈 때 unit/calibration/safety 차이를 fail-closed로 막는가.

### Viable Options

#### Option A — 현 구조 유지 + 결함별 패치/테스트 추가

- 장점: 변경량과 초기 비용이 작고 기존 스크립트/로그를 즉시 활용한다.
- 단점: 공유 전역 상태, 묵시적 경로, 성공 기준 변동, 데이터 계보 부재가 남는다. 현재의 sandbagging 패턴을 구조적으로 막지 못한다.

#### Option B — 계약 우선 재구성 + 규칙 기반 oracle/ACT 후보 분리 (선택)

- 장점: 오류가 raw/control/data/model/evaluation 중 어디서 생겼는지 분리되고, 현재 T00~T12 감사 증거를 재사용할 수 있다. 실기 안전 gate를 명시할 수 있다.
- 단점: manifest·adapter·validator·fixture 구축과 prospective 재수집 비용이 든다. 기존 점수는 승격 증거에서 제외된다.

#### Option C — 공식 LeRobot runner/robot interface로 전면 교체

- 장점: SO-101 calibration/processor/dataset/inference 관행을 최대한 upstream에 맞춘다.
- 단점: 현재 ROS 2 모바일 base, Gazebo truth, 5축 자세족, 커스텀 perception/place 로직을 그대로 수용하지 않는다. 전면 교체 자체가 새로운 통합 위험이다.

### 선택 및 종합

Option B를 선택하되 Option C의 공식 SO-101/LeRobot contract를 adapter 기준으로 사용한다. 현재 규칙 기반 동작은 그대로 신뢰하지 않고 물리 oracle gate를 통과한 부분만 golden baseline으로 채택한다.

## 5. 목표 아키텍처와 책임 경계

```text
Task contract + immutable scenario
  -> Mobility controller (base only)
  -> manipulation handoff pose gate
  -> PickPlace state machine
       RESET/PRECHECK/OBSERVE/APPROACH/ALIGN/PREGRASP/GRASP
       LIFT_VERIFY/TRANSPORT/PLACE_VERIFY/RELEASE/RETREAT/SUCCESS
  -> truth scorer + trace

Raw capture (observation, command, reference, achieved, truth, timestamps)
  -> fail-closed validator
  -> prospective split seal
  -> one action-contract derivation
  -> LeRobot v3 pack + frozen stats
  -> ACT train + checkpoint-bound processors
  -> contract-matched offline evaluation
  -> guarded replay/sim/HIL/real promotion
```

경계 규칙:

- 모바일 base와 6차원 arm/gripper ACT action을 섞지 않는다. ACT가 base를 제어하지 않으면 `BASE_APPROACH`는 규칙 기반이고 manipulation handoff pose·covariance·freshness를 만족한 뒤 ACT episode가 시작한다.
- 실물 SO-101 degree/gripper 0..100과 ROS/Gazebo rad/sim-gripper는 typed adapter 한 곳에서만 변환한다. 변환 전후 round-trip fixture가 없으면 실기 금지다.
- policy observation에는 sim ground truth를 넣지 않는다. truth는 scorer/label audit에만 사용한다.
- `pick_node.py`의 state machine, perception, actuation, truth/scoring, configuration을 테스트 가능한 모듈 경계로 나누되 기존 동작 regression fixture를 먼저 만든다.

### Dependency DAG

```text
P0 inventory/quarantine
  -> P1 semantic contracts + prospective source/split + formula/selection precommit
  -> P2 regression-only behavior lock
  -> P3 state-machine boundaries
  -> P4a zero-actuation characterization
  -> P4b numeric sync/handoff/safety contract publication
  -> P5 split-seal revalidation + timestamped raw capture
  -> P6 exact/write-once Dataset v3 pack
  -> P7 mutually exclusive action derivation + guarded no-model replay + selection ADR
  -> P8 checkpoint-bound ACT/offline gate
  -> P9 deterministic sim component/e2e gate
  -> P10 sealed held-out -> shadow/HIL/real promotion
  -> P11 independent full-chain verification
```

각 consumer는 모든 direct prerequisite run/result/complete-marker digest를 검증한다. 실패/누락/drift가 있으면 actuator나 downstream output을 초기화하기 전에 non-zero 종료한다.

## 6. 구현 단계와 수용 기준

### P0 — 현재 상태 봉인과 quarantine

대상: Git diff, `capstone_pick/tools/logs`, review1/review2 artifacts, 환경/LeRobot commit.

수용 기준:

- source/config/untracked 파일 inventory와 SHA-256, Git HEAD/status가 write-once manifest에 기록된다.
- source seal은 `.git`, `.omx`, build/install/log output을 제외한 명시적 allowlist만 해시한다. `.omx` planning/evidence layer는 별도 manifest로 해시해 자기 자신을 입력에 포함하지 않는다.
- 기존 data/checkpoint/result는 `legacy_untrusted`, `regression_only`, `candidate` 중 하나로 분류되고 원본은 변경하지 않는다.
- `act_eval_gate.json`과 `act_eval_gui.json`은 lineage/predicate 부재로 성능 비교 금지 상태가 된다.
- 기존 result를 새 manifest에 복사해 정당화하거나 placeholder digest를 넣으면 FAIL.

### P1 — SO-101/ROS 계약과 success ADR 동결

대상: 새 `capstone_pick/contracts/`의 schema/ADR, `jdamr_cube_description/config/so101_controllers.yaml`, URDF/world/launch.

수용 기준:

- LeRobot `0.6.1`/commit, joint order, real/sim units, calibration ID/hash, camera schema, FPS, clock domain, frame tree, controller/interface, limits 출처가 기록된다.
- sync/handoff/action-selection/product-promotion은 숫자가 아니라 formula, characterization producer, data-independent selection rule을 먼저 봉인한다. numeric threshold는 P4a 결과로만 생성한다.
- train/ID/held-out/characterization의 planned source·condition·session IDs, seeds, counts, allowed readers를 어떤 수집/characterization보다 먼저 봉인한다.
- pick/place predicate를 데이터 열람 전에 동결한다: no-contact, push-only, lift-drop, retained-lift, successful-place를 서로 다르게 판정한다.
- unknown safety limit는 unknown으로 남고 hardware authorization은 false다.
- success predicate 버전이 달라지면 같은 run ID/result를 재사용할 수 없다.

### P2 — 회귀 테스트로 현재 규칙 기반 동작 잠금

대상: `capstone_pick/tests/unit/`, `tests/contract/`, `tests/fixtures/`.

수용 기준:

- 좌표변환, joint/unit conversion, command guard, bounded retry, grasp interval, state transition, scoring pure function test가 결정적으로 통과한다.
- synthetic fixtures는 `synthetic_fixture`로 표시되고 실제 성공률 계산 API가 이를 거부한다.
- 기존 `pick_node.py` 분리 전 golden trace를 capture하고 분리 후 단계/명령/predicate가 동일함을 검증한다.
- 이 golden trace는 `regression_only`이며 현재 성공 판정이나 물리 진실을 승인하지 않는다. truth oracle 승격은 P9 scripted/canonical truth gate 이후다.
- capture는 기존 immutable trace 또는 simulator-only 실행으로 제한한다. P2는 hardware parent가 될 수 없고 `authorization=false`를 유지한다.

### P3 — PickPlace state machine와 observability 경계 정리

대상: `pick_node.py:297-2052`의 단계별 모듈화, trace schema, configuration boundary.

수용 기준:

- 모든 stage는 입력 precondition, timeout/retry budget, output event, failure reason, safety stop을 갖는다.
- `GRASP`, `LIFT_VERIFY`, `PLACE_VERIFY`, `SUCCESS`는 장치 상태와 object truth를 분리 기록한다.
- place 성공은 release 후 target zone 유지와 retreat까지 요구한다.
- retry가 성공률 분모를 숨기지 않으며 attempt/episode 두 통계를 모두 낸다.

### P4 — staged characterization과 이동→조작 handoff 계약

대상: base 접근/회전/odometry, camera/TF freshness, arm workspace gate, static zero-actuation 및 base-only characterization.

수용 기준:

- P4a-static은 base/policy/arm/gripper command가 모두 0인 repeated-static run에서 sensor rate·age·skew·gap·TF noise와 controller inventory를 측정한다.
- P4a-mobile은 base command만 허용하고 policy/arm/gripper command는 0인 별도 run에서 odom/TF handoff noise와 settle behavior를 측정한다. 두 subrun은 서로 다른 run ID/evidence class를 가진다.
- P4b는 P1의 formula와 P4a evidence digest만으로 numeric age/skew/handoff/safety threshold를 publish한다. 수동 숫자 override는 새 contract 없이 금지한다.
- base pose, target pose, covariance/age, handoff tolerance, retry/abort를 P4b contract에 동결한다.
- 이동이 포함된 raw episode에서 base action이 label에 없으면 manipulation episode 시작 전 구간을 학습 데이터에서 제외하고 reason을 기록한다.
- TF missing/stale, odom jump, object outside workspace이면 arm command 0으로 중단한다.

### P5 — split-seal 재검증과 timestamped raw capture

대상: `rule_collect.py:168-377`, controller/gripper trace surface, split seal.

수용 기준:

- observation images/state, accepted source goal, arm/gripper desired reference, achieved state, object/contact/support truth, semantic sim timestamp와 arrival timestamp를 기록한다.
- raw sim/arrival timestamp는 reserved LeRobot `timestamp`와 구분된 sidecar 또는 namespaced raw feature로 보존한다. pack 단계가 canonical FPS grid를 생성할 때 source row/time mapping과 resampling error를 기록한다.
- 각 row의 action coverage 100%, unique/monotonic command/goal ID, missing/fallback/future-join/truncation count 0이다.
- train/ID/held-out/characterization source·condition·session 교집합이 0이며 split은 수집 전에 봉인된다.
- P5 시작 시 P1 split seal과 P4 characterization access log를 재검증한다. characterization이 sealed plan 밖 source를 읽었거나 train/ID/held-out에 섞였으면 raw output 생성 전에 FAIL한다.
- 기존 `rule_std`/`rule_yaw45`는 validator를 통과하더라도 prospective held-out으로 재분류하지 않는다.

### P6 — Dataset v3 pack과 데이터 품질 gate

대상: `lerobot_pack.py:24-81`, dataset validator, immutable dataset manifest.

수용 기준:

- 모든 episode의 state/action/각 camera/timestamp 길이가 exact match이고 손실 frame이 있으면 전체 episode FAIL이다.
- output은 content-addressed 새 경로에 write-once로 생성하며 기존 경로를 삭제하거나 덮어쓰지 않는다. 현재 `lerobot_pack.py:52-54`의 recursive delete 동작은 negative fixture로 차단한다.
- image decode 실패, feature shape/order/unit/FPS/stats/task mismatch, NaN/Inf, joint limit violation이 0이다.
- train만 normalization stats/augmentation fit에 사용되고 ID/held-out read count는 0이다.
- raw→derived→packed row ID와 digest가 양방향 추적된다.

### P7 — Action contract 비교와 no-model replay

대상: actual controller reference 후보, timestamped preview 후보, collision/alias-risk 검사, guarded replay.

수용 기준:

- 한 dataset에는 action contract가 정확히 하나만 존재한다.
- actual command는 causal sample-and-hold, preview는 명시적 physical offset으로 별도 dataset을 만든다.
- exact duplicate observation에 상충 target이 있으면 후보 FAIL; near-alias risk는 train-only 사전 공식으로 보고한다.
- 모델 없이 action을 재생했을 때 command-vs-achieved tracking, gripper completion, pick/place truth가 사전 envelope 안에 있어야 학습을 허용한다.
- actual/preview 선택 rule, tie/insufficient-evidence 처리, required playback metrics는 P1에서 결과 열람 전에 봉인한다. 선택 기준 변경은 새 contract와 downstream 무효화를 요구한다.

### P8 — ACT checkpoint/processor/offline gate

대상: frozen training config, checkpoint/processors, `act_eval.py:105-120`, offline evaluator.

수용 기준:

- exact dataset/split/code/env/seed/config/checkpoint/pre/postprocessor digest가 하나의 candidate manifest에 묶인다.
- 공식 순서 `preprocessor -> select_action -> postprocessor -> guard`가 dry-run과 static/runtime test로 증명된다.
- masked chunk loss, first-step raw-unit error, queue/reset semantics를 같은 target contract로 비교한다.
- processor bypass, wrong unit/order/shape, NaN/range violation fixture는 policy rollout 전에 FAIL한다.

### P9 — 결정적 sim component/e2e gate

대상: Gazebo fixed seeds, scripted truth trajectories, full mobility+pick+place scenarios.

수용 기준:

- 규칙 기반 oracle은 각 canonical scenario를 동일 seed로 3/3 통과하며 trace/predicate mismatch 0이다.
- no-contact, push-only, lift-drop, retained-lift, correct-place fixture가 모두 기대 판정과 일치한다.
- ACT 후보는 sealed ID scenarios에서 평가하며 infrastructure-invalid trial을 실패와 분리하되 분모에서 조용히 제외하지 않는다.
- 결과에는 stage별 failure rate, abort, age/skew/gap, command tracking, intervention, run digest가 포함된다.

### P10 — held-out·HIL·실기 승격

대상: 별도 promotion ADR, HIL adapter, hardware checklist.

수용 기준:

- held-out 접근 전 sample size, minimum success, confidence-bound, abort limit, owner/version을 별도 ADR로 봉인한다.
- HIL은 torque/actuation 없는 shadow부터 시작하고, unit/calibration/latency/max-relative-target를 측정한다.
- 실기 success는 sim ground truth와 별도 schema를 사용한다. 사용 센서/영상/human label, reviewer identity, disagreement/audit 규칙과 불확실 label 처리법을 hardware ADR에 동결한다.
- hardware run은 signed `promotion-decision`의 `authorization=true` 없이는 actuator 초기화 전 non-zero 종료한다.
- 실기 결과는 sim 성공률과 합치지 않고 별도 evidence class와 failure taxonomy로 보고한다.

### P11 — 독립 최종 검수와 폐기 정책

대상: full-chain validator, independent verifier report, legacy cleanup proposal.

수용 기준:

- P0~P10 direct prerequisite와 digest DAG가 완전하며 orphan/mutable/placeholder artifact가 0이다.
- fresh environment에서 manifest로 데이터 검사, offline metric, sim deterministic gate를 재생한다.
- code-reviewer/verifier가 source 작성 패스와 분리되어 승인한다.
- legacy file 삭제는 별도 승인 전까지 하지 않고, 삭제 후보와 복구 경로만 제시한다.

### P↔T Mapping

| PRD phase | Test gate |
|---|---|
| P0 inventory/quarantine | T00 |
| P1 semantic contracts/prospective split/precommit | T01 |
| P2 regression-only lock + P3 state-machine boundaries | T02 |
| P4a/P4b characterization/handoff | T03 |
| P5 split revalidation + raw capture | T04 split/access revalidation → T05 raw capture |
| P6 Dataset v3 pack | T06 |
| P7 action derivation/replay/selection | T07 |
| P8 ACT/offline | T08 |
| P9 deterministic sim | T09 |
| P10 held-out + HIL/real | T10 → T11 |
| P11 independent full-chain | T12 |

T04 split-seal complete marker 없이는 T05 capture output 경로를 만들 수 없고, T10 held-out complete marker 없이는 T11 HIL/real promotion decision을 만들 수 없다.

## 7. Acceptance Criteria

| ID | 합격 기준 |
|---|---|
| AC-01 | 모든 성능 결과가 code/data/split/checkpoint/processors/predicate/seed digest를 가진다. |
| AC-02 | raw row action coverage 100%, missing/fallback/future-join/truncation 0이다. |
| AC-03 | train/ID/held-out/characterization source·condition·session overlap 0이다. |
| AC-04 | real degree/gripper 0..100↔sim rad/gripper adapter round-trip이 joint별 tolerance 안이고 단위 없는 값은 거부된다. |
| AC-05 | stale/skew/malformed/NaN/out-of-range fixture에서 policy call/arm publish/gripper goal 모두 0이다. |
| AC-06 | no-contact/push/lift-drop/retained-lift/successful-place 판정 5종이 100% 기대값과 일치한다. |
| AC-07 | 규칙 기반 canonical sim scenario가 seed별 3/3이고 trace-vs-truth mismatch 0이다. |
| AC-08 | ACT offline metric과 rollout이 동일 action/processor/queue 계약을 사용한다. |
| AC-09 | held-out/hardware는 사전 promotion ADR 없이는 시작 전에 차단된다. |
| AC-10 | fresh verifier가 full DAG와 핵심 gate를 재생하고 APPROVE한다. |

## 8. Expanded Test Plan

- **Unit:** unit conversion, transforms, state transitions, scoring predicates, causal join, schema/hash, command limits.
- **Integration:** ROS clock/QoS, controller reference+goal lifecycle, camera/joint sync, packer exact lengths, processor/checkpoint loading.
- **E2E:** base approach→handoff→grasp→lift→transport→place→release→retreat를 Gazebo truth로 검증; ACT는 ID 후 held-out 순서.
- **Observability:** stage event, failure reason, age/skew/gap, command/achieved error, retry, intervention, truth/predicate mismatch와 artifact digest를 구조화한다.
- **HIL/real:** shadow→zero-load motion→single-layout→sealed layouts 순서; promotion authorization 기본 false.

## 9. Pre-mortem

1. **점수만 개선되고 원인이 해결되지 않음:** success predicate나 spawn/source가 바뀌어 0/10이 4/5가 된다. 완화: predicate/source/code digest 없는 결과는 비교 금지, precommit 변경은 새 run ID.
2. **action이 다시 가짜 측정값이 됨:** controller reference 누락을 achieved state로 대체해 데이터가 정상처럼 보인다. 완화: coverage 100%, fallback 0, missing row가 있으면 episode 전체 FAIL.
3. **시뮬 성공 후 실기에서 위험 동작:** rad/degree, gripper scale, calibration, latency 차이가 남는다. 완화: typed adapter round-trip, shadow/HIL gate, signed authorization 전 actuator 초기화 금지.

## 10. ADR

### Decision

규칙 기반 시스템을 물리 truth로 재검증한 oracle lane과 ACT candidate lane으로 분리하고, LeRobot v0.6.1 기반 typed artifact/data/control contract를 양쪽에 공통 적용한다. 기존 데이터와 결과는 삭제하지 않되 재검증 전에는 승격 증거로 사용하지 않는다.

### Drivers

- 반복 실패의 원인을 한 계층에 귀속할 수 있어야 한다.
- 거짓/대체/잘린 데이터와 사후 score 변경을 구조적으로 금지해야 한다.
- 모바일 base+SO-101 sim→real 경계에서 안전과 단위 일치가 우선이다.

### Alternatives considered

- 현 구조 patching: 초기 비용은 낮지만 계약 드리프트를 막지 못해 제외.
- 공식 runner 전면 교체: upstream 일치는 좋지만 현재 ROS/Gazebo/mobile integration을 대체하지 못해 adapter 참고안으로 제한.

### Why chosen

현재 두 차례 감사의 T00~T12 근거를 재사용하면서도 최신 미커밋 수정과 전체 이동/pick/place 범위를 포함할 수 있고, 모델을 다시 학습하기 전에 데이터·제어·평가의 독립성을 증명할 수 있다.

### Consequences

- 기존 score는 regression 참고값만 되고 prospective 재수집/재평가 비용이 발생한다.
- manifest/schema/test code가 늘지만 이후 실패의 원인 범위가 줄어든다.
- product success threshold와 hardware safety 수치는 별도 precommit ADR 없이는 비어 있는 것이 올바른 상태다.

### Follow-ups

- P3~P4 결과로 state-machine 분리와 base→arm handoff 수치를 동결한다.
- P7 near-alias risk가 높으면 sensor-derived phase/history 또는 다른 temporal policy ADR을 연다.
- 실기 전 calibration/latency/safety-envelope ADR을 별도로 승인한다.

## 11. 위험과 중단 규칙

- source/dataset/checkpoint/processor/predicate digest mismatch: 즉시 중단, downstream 무효화.
- 누락/대체/길이 불일치/held-out 조기 접근: 해당 episode/run FAIL, 성공률 미산출.
- dirty worktree drift: 기존 사용자 변경을 보존하고 새 baseline branch/manifest로만 진행.
- hardware authorization false/unknown: actuator 초기화 금지.
- 공식 문서에 없는 수치: 추측으로 채우지 않고 characterization/ADR 전까지 unknown 유지.

## 12. Available-Agent-Types Roster

사용 가능: `explore`, `analyst`, `planner`, `architect`, `critic`, `scholastic`, `dependency-expert`, `researcher`, `executor`, `team-executor`, `test-engineer`, `debugger`, `verifier`, `code-reviewer`, `code-simplifier`, `designer`, `writer`, `git-master`, `vision`.

현재 native hard limit은 leader 포함 4개 동시 슬롯이므로 실행 시 child는 최대 3개다. 논리 상한 10을 요청하더라도 10개가 동시에 실행된다고 보고하지 않는다.

## 13. Follow-up Staffing, Launch Hints, Verification

- 기본 `$ultragoal`: leader가 P0→P11 digest ledger와 stop/resume을 소유한다.
- `$team` 병행 시 3 lanes: (1) `executor` medium—contracts/data pipeline, (2) `executor` medium—state machine/safety/evaluator, (3) `test-engineer` medium—fixtures/e2e/evidence. 공유 `pick_node.py`와 `act_eval.py`는 한 owner만 편집한다.
- 각 stage 종료는 별도 `verifier` high 또는 `code-reviewer` high가 작성 패스와 분리해 승인한다.
- launch 예시:

```bash
omx team 3:executor "Implement .omx/plans/prd-so101-systemic-assurance.md against .omx/specs/test-spec-so101-systemic-assurance.md; Ultragoal owns sequential gates and digests"
```

- Team 종료 전 증명: AC-01~AC-10, hostile zero-actuation, split overlap 0, raw fallback/truncation 0, processor contract, canonical 3/3, full DAG.
- Ultragoal은 Team evidence digest만 checkpoint하고 검증되지 않은 worker 보고를 완료로 기록하지 않는다.

## 14. Goal-Mode Follow-up Suggestions

- **권장:** `$ultragoal` + 필요 시 `$team` — P0~P11을 영속 ledger와 병렬 lane으로 실행.
- `$autoresearch-goal` — SO-101 실기 safety envelope나 다른 정책 비교가 별도 연구 산출물일 때만 사용.
- `$performance-goal` — 정확성 gate 통과 후 FPS/latency/RTF를 최적화할 때만 사용.
- `$ralph` — 사용자가 단일 owner의 지속적 fix/verify를 명시 선택할 때만 fallback.

## 15. 완료/중단 조건

이 계획은 Architect와 Critic이 동일 digest를 순서대로 승인하고 durable consensus handoff가 기록되면 planning complete다. 비대화된 구현, 데이터 수집, 재학습, 시뮬/실기 실행은 별도 승인된 execution lane에서만 수행한다.

## 16. Consensus Improvement Changelog

- Architect iteration 1: dependency DAG, characterization producer, regression/truth 구분, raw/LeRobot timestamp 분리, write-once pack, action selection precommit, real scorer, inventory 자기참조 방지를 추가했다.
- Critic iteration 1: prospective split을 capture보다 앞선 P1/T01 producer로 이동하고 T04에서 재검증하게 했다. static zero-actuation와 base-only mobile characterization을 분리하고 P↔T mapping 및 P2 simulator-only 경계를 추가했다.
- Architect iteration 3: Critic blocker closure와 characterization source seal의 비순환성을 재검토했다.
