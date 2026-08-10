# Ralplan Architect Review — SO-101 Systemic Assurance, Iteration 2

- Verdict: **APPROVE (leader fallback)**
- Review type: leader fallback after the typed Architect timed out and was interrupted under the single-wait runtime policy
- This is not represented as typed-agent approval.
- PRD SHA-256: `bfc430702333f879b03b93445db4758ba2beba9e7fdb7db90254217f344e5b3b`
- Test-spec SHA-256: `c5fdddc8795f54b11c99214535fb796742a9cf1a14ce26f0dde712bbe10b76a7`

## Architecture verdict

선택한 contract-first oracle/candidate 분리는 현재 문제의 실제 경계를 반영한다. 현 code/data/log를 즉시 신뢰하지 않고 P0에서 격리한 뒤, semantic formula→zero-actuation characterization→numeric contract→prospective capture 순서를 강제하므로 사후 threshold 조정과 missing-data fallback을 동시에 막는다.

## Steelman antithesis revisited

“process infrastructure가 실제 pick/place보다 커질 수 있다”는 반론은 여전히 유효하다. iteration 2의 explicit DAG와 `P0~P2 regression-only`, direct prerequisite checks, write-once outputs가 이를 완화한다. 실행자는 P0부터 순차로 닫고 downstream scaffold를 미리 만들지 않아야 한다.

## Tradeoff and synthesis

- 추적성 비용을 받아들이되 모든 ancestor를 사람이 반복 해석하지 않고 direct prerequisite run/result/complete digest로 기계 검증한다.
- 기존 behavior trace는 보존하되 `regression_only`로 제한하고, truth oracle은 P9 이후에만 선언한다.
- 수치는 추측하지 않되 formula/producer를 먼저 봉인하고 P4a 실측으로 P4b를 결정한다.

## Iteration-1 closure mapping

1. Dependency DAG: PRD §5에 추가, consumer fail-before-init 명시.
2. Numeric threshold producer: P4a/P4b와 T03에 추가.
3. Regression vs truth: P2/T02에 `regression_only` 추가.
4. Timestamp separation: P5/T04에 raw sidecar/namespaced mapping 추가.
5. Write-once pack: P6/T06에 existing-output fail 추가.
6. Action selection precommit: P1/P7/T01/T07에 추가.
7. Real scorer: P10/T11에 별도 schema와 audit 규칙 추가.
8. Inventory self-reference: P0/T00 allowlist와 separate planning layer 추가.

## Principle violations

없음. 다만 product success threshold와 hardware limit가 의도적으로 unknown이며, 별도 ADR 전 `authorization=false`인 것이 승인 조건이다.

## Test adequacy

- Unit: unit/transform/state/scoring/causal join.
- Integration: clock/QoS/controller/gripper/sync/pack/processors.
- E2E: mobility handoff와 full pick/place truth, ACT ID 후 held-out.
- Observability: state/failure/timing/tracking/provenance/attempt counters.
- HIL/real: shadow부터 signed promotion까지 fail-closed.

Critic에 전달 가능하다. Critic은 특히 계획 규모의 실행 가능성, P0~P11과 T00~T12 mapping, success criterion의 testability, product threshold unknown 처리, write-once/digest claim을 독립 검토해야 한다.
