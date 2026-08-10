# Ralplan Architect Review — SO-101 Systemic Assurance, Iteration 1

- Verdict: **ITERATE**
- Review type: **leader fallback Architect review**
- Typed Architect agent: `/root/so101_architect_review`
- Agent result: single 60-second wait timed out; agent interrupted immediately; no verdict/partial artifact returned
- Important: timeout is not approval and this document does not claim typed-agent approval.
- Reviewed PRD SHA-256: `6584d6ea3ee4fe7d344ef21f7fd991eef48d462e3493e8e4e6e8ab40f68b9992`
- Reviewed test-spec SHA-256: `d4793796edaa4432bac72b2c1507bb75e586cff3c9ae21a2ffff169d14495616`

## Steelman antithesis

선택안 B의 가장 강한 반론은 “새 manifest/schema/gate 체계를 만드는 동안 실제 pick/place 결함은 그대로이고, 2,000줄짜리 state machine 위에 또 하나의 복잡한 프로세스 층만 쌓일 수 있다”는 것이다. 현재 목적에 가장 빠른 안전 경로는 ACT와 실기를 모두 동결하고, 규칙 기반 단일 scene의 물리 truth 3/3과 현재 data/evaluator의 최소 contract만 고친 뒤 범위를 확장하는 것이다. 이 반론은 유효하다. 따라서 계획은 모든 장치를 한 번에 만들기보다 각 stage가 바로 다음 위험 하나만 해소하고, downstream 실행을 차단하는 최소 DAG여야 한다.

## Tradeoff tension

- **완전한 추적성 vs 실행 가능성:** 모든 artifact에 full ancestor DAG를 요구하면 신뢰도는 높지만 구현 자체가 새 실패원이 된다.
- **현 동작 보존 vs 거짓 성공 보존:** refactor 전 golden trace는 regression에는 유용하지만 현재 success predicate가 잘못됐으므로 truth oracle이 될 수 없다.
- **사전 threshold 동결 vs 실측 필요:** 숫자를 사전에 정하면 추측이고, 실행 후 정하면 사후 조정이다. formula/producer를 먼저 봉인하고 zero-actuation characterization에서 숫자를 생산해야 한다.

## Required changes

1. 명시적 dependency DAG를 추가한다.
2. zero-actuation characterization producer를 추가하고 numeric threshold의 유일한 출처로 만든다.
3. 현 golden trace는 `regression_only`이지 physical truth 승인이 아님을 명시한다.
4. raw sim/arrival timestamp와 Dataset v3 auto timestamp를 분리한다.
5. 현재 `lerobot_pack.py:52-54`의 recursive output delete를 금지한다.
6. actual/preview action candidate 선택 rule을 결과 열람 전에 봉인한다.
7. sim과 실기의 success evidence/scorer를 분리한다.
8. P0 source inventory에서 `.omx` output/manifest 자기참조를 차단한다.

## Principle check

- Evidence before score: 방향은 충족하나 golden regression/truth 구분 보강 필요.
- Fail closed: 수집/pack/eval에 잘 반영됐으나 output overwrite 금지 명시 필요.
- One semantic contract: raw-vs-LeRobot timestamp와 real-vs-sim scorer가 아직 모호함.
- Oracle/candidate separation: 충족.
- Staged promotion: dependency DAG와 threshold producer가 보강돼야 실제 강제 가능.

## Deliberate-mode check

- Pre-mortem 3개: 존재하고 핵심 실패를 다룬다.
- Unit/integration/e2e/observability/HIL: 모두 존재한다.
- Acceptance criteria: 구조적 기준은 테스트 가능하다. product 성공률은 별도 precommit ADR로 미루면서 `authorization=false`를 유지해 허위 정밀도를 피한다.

## Synthesis

Option B를 유지하되 Option A의 최소주의를 흡수한다. P0~P2는 기존 동작을 변경하지 않는 봉인/순수 테스트 단계로 제한하고, numeric threshold는 no-actuation characterization의 단일 producer에서만 만들며, 각 stage는 다음 stage 한 개의 위험만 해소한다. 위 8개 수정 후 Architect fallback 재검토가 가능하다.
