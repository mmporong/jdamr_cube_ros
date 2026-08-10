# Ralplan Critic Review — SO-101 Systemic Assurance, Iteration 1

- Verdict: **ITERATE**
- Review type: **leader fallback Critic review**
- Typed Critic agent: `/root/so101_critic_review`
- Agent result: single 60-second wait timed out; agent interrupted immediately; no verdict/partial artifact returned
- Important: timeout is not approval and this document does not claim typed-agent approval.
- Reviewed PRD SHA-256: `bfc430702333f879b03b93445db4758ba2beba9e7fdb7db90254217f344e5b3b`
- Reviewed test-spec SHA-256: `c5fdddc8795f54b11c99214535fb796742a9cf1a14ce26f0dde712bbe10b76a7`
- Architect input: `.omx/context/ralplan-architect-review-so101-systemic-assurance-iteration-2.md`

## Blocking findings

1. **Prospective split ordering contradiction.** PRD P5는 split을 수집 전에 봉인한다고 하지만 test-spec은 T04 raw capture 뒤 T05 split이다. 테스트 번호와 문서 순서가 실행 순서를 만들기 때문에 raw capture가 먼저 허용될 수 있다. T04를 split, T05를 capture로 바꾸고 downstream reference를 정합시켜야 한다.
2. **Zero-actuation ambiguity.** P4a/T03은 repeated-static와 mobility characterization을 한 단계로 쓰면서 `policy/arm/gripper=0`만 요구한다. base actuation이 있는 mobility subrun과 모든 actuator가 0인 static sensor subrun을 분리해야 한다.
3. **P↔T mapping 부재.** PRD P0~P11과 test T00~T12는 수가 다르며 묵시적 대응만 있다. full-chain validator가 무엇을 direct prerequisite로 삼는지 explicit mapping이 필요하다.
4. **P2 regression capture의 hardware boundary.** 현 동작 capture가 실기에서 실행될 여지를 닫지 않았다. P2/T02는 archived evidence 또는 simulator-only이며 hardware authorization false임을 명시해야 한다.

## Quality checks

- Principles/options: 선택 Option B는 원칙과 일치하고 A/C alternative는 공정하다.
- Risks/pre-mortem: 3개가 구체적이며 mitigation과 연결된다.
- Tests: unit/integration/e2e/observability/HIL이 존재하고 대부분 testable하다.
- Data integrity: missing/fallback/truncation/leakage/processor/action authority가 fail-closed로 다뤄진다.
- Unknown thresholds: 별도 ADR 전 `authorization=false`는 허위 정밀도보다 타당하다.
- Handoff fields: ADR, roster, reasoning/staffing, launch hint, Team verification, Goal-Mode suggestions가 존재한다.

위 네 blocker를 수정하고 Architect fallback이 수정본을 재검토한 뒤 Critic 재평가를 진행해야 한다.
