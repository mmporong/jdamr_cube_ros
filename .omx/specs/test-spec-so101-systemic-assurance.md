# SO-101 시스템 신뢰성 테스트 명세

- 대응 PRD: `.omx/plans/prd-so101-systemic-assurance.md`
- 상태: Planner iteration 3, Critic iteration-1 feedback 반영
- 원칙: fixture 통과와 실제 성능을 분리하고, invalid infrastructure trial을 숨기지 않는다.

## 1. 공통 테스트 계약

각 테스트 run은 다음을 가진다.

- `run_id`, `stage`, `status`, start/end UTC
- Git/source/config digest와 dirty baseline manifest
- LeRobot/Python/PyTorch/CUDA/ROS/Gazebo version
- input/output/prerequisite SHA-256
- exact argv/env allowlist와 Python/NumPy/Torch/scene seed
- `evidence_class`, checks, counters, units, thresholds, failure reasons
- atomic complete marker; 기존 output overwrite 금지

공통 실패 조건:

- unknown/placeholder/null을 존재하는 artifact처럼 사용
- missing prerequisite를 fallback/skip하고 PASS
- synthetic/mock을 실제 성공률에 포함
- success predicate 또는 split을 run 이후 변경
- incomplete run을 분모에서 조용히 제거

## 2. T00 — Inventory/Provenance Gate

대상: current Git state, untracked files, review1/review2 artifacts, all datasets/checkpoints/results.

PASS:

- source/config/data/checkpoint/result inventory와 hashes가 완전하다.
- 각 legacy artifact가 `legacy_untrusted|regression_only|candidate`로 분류된다.
- `act_eval_gate.json`, `act_eval_gui.json`은 lineage/predicate 부재가 명시된다.
- 재실행해도 동일 bytes/hash이고 원본 mtime/content가 변하지 않는다.
- source inventory allowlist는 `.git/.omx/build/install/log`를 제외하고, planning/evidence manifest는 별도 layer라 자기 digest를 자기 입력에 포함하지 않는다.

FAIL: inventory 누락, symlink target 미봉인, 기존 artifact 수정/삭제, 없는 checkpoint를 available로 표시.

## 3. T01 — Schema/Unit/Clock/Calibration Contract

### Unit tests

- joint order permutation fixture는 거부된다.
- rad↔degree round-trip과 sim-gripper↔0..100 round-trip이 joint별 ADR tolerance 내다.
- 단위 tag 없는 action/state는 거부된다.
- sim time와 wall arrival time이 서로 join key로 사용되지 않는다.
- camera/frame/TF schema mismatch가 FAIL한다.

### Contract tests

- LeRobot `0.6.1`/commit, calibration ID/hash, controller interface, camera config, FPS, feature shape가 모두 manifest에 있어야 한다.
- hardware safety fields가 unknown이면 `authorization=false`다.
- sync/handoff/action selection은 formula와 producer/decision rule만 사전 봉인하며 numeric 값은 T03 characterization output 없이는 존재할 수 없다.
- train/ID/held-out/characterization planned source·condition·session IDs, seeds, counts, allowed readers가 어떤 characterization/capture보다 먼저 봉인되고 class pair overlap이 0이다.

## 4. T02 — PickPlace State Machine Unit/Component Tests

상태: `RESET -> PRECHECK -> OBSERVE -> BASE_APPROACH -> HANDOFF -> ALIGN -> PREGRASP -> GRASP -> LIFT_VERIFY -> TRANSPORT -> PLACE_VERIFY -> RELEASE -> RETREAT -> SUCCESS`.

PASS:

- 각 state의 legal/illegal transition 표가 100% 검증된다.
- timeout/retry budget 초과 시 명시적 reason으로 FAILSAFE에 들어간다.
- no-contact, push-only, lift-drop, retained-lift, successful-place scripted truth가 정확한 label을 받는다.
- release 전 target-zone 진입만으로 success가 되지 않고 release 후 유지+retreat가 필요하다.
- attempt와 episode 통계가 분리된다.
- refactor 전 golden trace는 `regression_only`이고 scripted/canonical physical truth PASS로 승격되지 않는다.
- golden capture는 archived immutable trace 또는 simulator-only이며 T02 marker는 hardware parent가 될 수 없고 `authorization=false`다.

Hostile fixtures는 `policy_call=0`, `arm_publish=0`, `gripper_goal=0` counters를 별도 JSON으로 낸다.

## 5. T03 — Mobility/Handoff Integration Tests

PASS:

- static subrun은 `base_cmd=policy_call=arm_publish=gripper_goal=0`이고 sensor rate/age/skew/gap, TF noise와 controller inventory를 기록한다.
- mobile subrun은 base command만 허용하며 `policy_call=arm_publish=gripper_goal=0`이고 odom/TF handoff noise와 settle behavior를 기록한다.
- static/mobile은 서로 다른 run ID/evidence class를 가진다.
- T01 formula와 characterization digest에서 numeric threshold를 결정적으로 publish하며 unknown/manual override를 거부한다.
- odom/TF/camera/object timestamp age와 skew가 precommitted limit 안일 때만 handoff한다.
- target outside arm workspace, stale TF, odom discontinuity, base not settled에서 arm publish 0이다.
- base motion 구간이 arm-only ACT dataset에 포함되지 않는다.
- 동일 scene seed에서 handoff pose와 truth outcome이 결정적으로 재현된다.

FAIL: base action이 observation에는 영향을 주지만 action label에 없는데 같은 manipulation episode로 pack됨.

## 6. T04 — Split-Seal/Access Revalidation

PASS:

- T01에서 봉인한 train/ID/held-out/characterization source·condition·session 목록, seed, allowed readers digest가 그대로다.
- 모든 class pair의 source/condition/session 교집합 0.
- train/stats/augmentation/model selection 단계의 held-out read count 0.
- T03 static/mobile characterization access log가 T01 seal과 exact-match하고 train/ID/held-out read 0이다.
- characterization data는 training, stats fit, action/model selection에서 제외된다.

FAIL: 기존 train episode를 사후 held-out으로 지정, 프레임 랜덤 분할, 동일 session의 인접 episode를 train/test로 분할, T01 seal 전 characterization/capture, T03 access-log drift.

T04 atomic complete marker가 없으면 T05는 output directory를 만들기 전에 non-zero 종료한다.

## 7. T05 — Raw Capture/Action Authority Tests

필수 row fields:

- source/condition/session/episode/frame IDs
- camera/state semantic stamps + arrival stamps
- accepted source goal ID
- arm/gripper desired reference command ID/stamp/value/unit
- achieved state stamp/value/unit
- object/contact/support truth와 evidence class
- raw sim/arrival time은 reserved LeRobot `timestamp`와 분리된 sidecar/namespaced field이며 source row/time mapping을 가진다.

PASS:

- action coverage 100%; IDs unique/monotonic.
- causal sample-and-hold만 허용; future/nearest join 0.
- missing camera/joint/reference/gripper row 0.
- achieved state/zero/last-value fallback 0.
- stream gap, duplicate, out-of-order, stale/skew count 0 또는 episode FAIL.

현재 `rule_collect.py:289-293` 동작을 재현하는 missing-reference fixture는 반드시 FAIL해야 한다.

## 8. T06 — Dataset v3 Pack/Integrity Tests

PASS:

- episode별 state/action/front/wrist/timestamp count exact match.
- `min(lengths)` truncation 0; 하나라도 다르면 episode FAIL.
- output path는 새 content-addressed 경로여야 하고 기존 output 삭제/덮어쓰기는 시작 전에 FAIL한다.
- images decode/shape/channel/range, feature order/unit/FPS/task exact match.
- NaN/Inf/joint-limit violation/duplicate frame ID 0.
- `meta/info.json`, `meta/stats.json`, episode/task metadata와 raw/derived digests가 일치한다.
- normalization stats는 sealed train만 사용한다.

Negative fixtures: missing last image, corrupt JPEG, swapped cameras, swapped joints, FPS drift, duplicate frame, truncated action.

## 9. T07 — Action Contract/Alias-Risk/No-Model Replay

PASS:

- `actual-controller-reference-v1`과 `preview-state-timestamped-v1`은 별도 derived dataset/manifest다.
- 한 row/manifest에 둘 이상의 action contract가 없다.
- exact observation collision에 conflicting target이 있으면 ineligible.
- near-alias risk 공식/반경은 train-only characterization 전에 사전 봉인된다.
- model 없이 selected action을 guarded replay했을 때 command-achieved tracking, gripper completion, scoring truth가 envelope 안이다.
- actual/preview selection rule, tie와 insufficient-evidence 처리는 T01에서 결과 열람 전에 봉인되고 selection artifact가 그 digest를 직접 참조한다.

FAIL: K row shift를 timestamp 증거 없이 사용, held-out으로 action contract 선택, playback 전에 T01~T06 prerequisite 누락.

## 10. T08 — ACT Processor/Offline Tests

PASS:

- checkpoint config와 pre/postprocessor가 같은 pretrained path/digest에서 로드된다.
- call order가 `pre -> select_action -> post -> guard`다.
- processor bypass/static direct publish 경로 0.
- action shape/order/unit/range, `chunk_size`, `n_action_steps`, queue reset이 manifest와 일치한다.
- training masked chunk loss와 offline masked chunk metric이 같은 target/mask/processor 계약을 쓴다.
- deterministic sample에서 first-step raw-unit error와 command tracking target을 함께 보고한다.

Negative fixtures: wrong stats, wrong camera key, raw state to normalized policy, normalized action to rad controller, queue not reset, non-finite output.

## 11. T09 — Sim Scoring and Canonical Regression

### Scripted truth fixtures

- no-contact → FAIL
- push-only → FAIL
- lift-drop → FAIL
- retained-lift before place → intermediate PASS, episode not complete
- released inside target + retained support + retreat → SUCCESS

### Rule-based oracle

- canonical scenario matrix의 각 seed 3/3.
- trace state/predicate vs Gazebo truth mismatch 0.
- infrastructure-invalid은 별도 count/reason으로 보고하며 재시도도 attempt 통계에 포함.

### ACT ID regression

- sealed ID scenarios에서만 실행.
- success count뿐 아니라 stage failure, abort, age/skew/gap, tracking error, gripper lifecycle, intervention을 보고.
- threshold는 run 전에 봉인; 결과를 본 뒤 변경하면 새 contract/run.

## 12. T10 — Held-out Evidence Gate

PASS prerequisite:

- T00~T09 PASS와 atomic markers.
- sample size/minimum success/exact binomial or Wilson bound/abort limit가 held-out 열람 전에 별도 ADR에 봉인.
- held-out raw access log가 evaluator 외 0.

결과는 descriptive evidence와 promotion decision을 분리한다. ADR이 없으면 결과는 생성할 수 있어도 `authorization=false`이고 hardware parent가 될 수 없다.

## 13. T11 — HIL/Real Safety Gate

순서:

1. processor shadow, action publish 0
2. torque/actuation 없는 interface check
3. 빈 작업공간 zero-load motion
4. 고정 단일 layout
5. sealed layouts/seeds

PASS:

- real calibration/firmware/camera/max-relative-target/latency/safety envelope digest.
- real success scorer는 sim ground truth와 별도 schema이며 센서/영상/human label source, reviewer, disagreement/audit, uncertain-label 처리를 봉인한다.
- immediate power cut operator와 abort path evidence.
- sim/rad와 real/degree 변환 round-trip PASS.
- signed promotion decision `authorization=true`.

FAIL: 공식 문서에 없는 안전 수치를 임의 default로 사용, authorization false/null, calibration drift.

## 14. T12 — Observability and Full-Chain Verification

필수 structured metrics:

- stage enter/exit/duration/retry/failure reason
- camera/state age/skew/gap/drop
- command/reference/achieved tracking and units
- gripper accept/result/timeout/cancel/reap
- truth/predicate mismatch
- dataset/checkpoint/processor/predicate/split/code/run digests
- attempt/episode/infrastructure-invalid counts

Final PASS:

- T00~T11 direct prerequisite DAG complete, digest drift/orphan/mutable/placeholder 0.
- fresh environment에서 schema/unit/scoring/pack/processor/offline/canonical sim tests 재현.
- 작성자가 아닌 `verifier` 또는 `code-reviewer`가 결과를 APPROVE.

## 15. 실행 순서와 명령 형태

구현 후 최소 검증 순서는 다음을 따른다. 실제 파일/target 이름은 구현 PR에서 고정한다.

```bash
python -m pytest -q capstone_pick/tests/unit
python -m pytest -q capstone_pick/tests/contract
python -m pytest -q capstone_pick/tests/integration
colcon test --packages-select capstone_pick jdamr_cube_description jdamr_cube_gazebo
colcon test-result --verbose
python capstone_pick/tools/verify_evidence_chain.py --through T12
```

시뮬 actuator 테스트와 HIL/real 테스트는 위 static/offline prerequisite marker를 검증하기 전에는 실행 명령 자체가 non-zero여야 한다.

## 16. Stop/Resume

- 실패 artifact와 reason은 보존하고 같은 output을 덮어쓰지 않는다.
- 변경된 contract부터 새 child run을 만들고 descendant를 무효화한다.
- source/data/checkpoint drift, leakage, fallback, authorization failure는 즉시 중단한다.
- 실기 승격은 복구 가능한 로컬 작업이 아니므로 별도 authority와 승인 없이는 진행하지 않는다.
