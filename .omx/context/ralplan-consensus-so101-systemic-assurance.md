# Ralplan Consensus — SO-101 Systemic Assurance

- Planning status: **COMPLETE**
- Completed UTC: `2026-08-07T03:42:56Z`
- Source root: `/home/lim/jdamr_cube_ws/src/jdamr_cube_ros`
- Source mutation: none by Ralplan; only `.omx` planning/context/spec/state artifacts were created
- Execution authorization: none

## Ordered consensus record

| Order | Artifact | Verdict | SHA-256 |
|---|---|---|---|
| 1 | `.omx/plans/prd-so101-systemic-assurance.md` | final Planner candidate | `385a989df0647b02c377ab1f513b760a689b9f45afa505d66ed9603145ad10c3` |
| 2 | `.omx/specs/test-spec-so101-systemic-assurance.md` | final test contract | `2d3ffc89240151ca118b59d760cad2da4ac1491d046b6759db6bdd004e839ac8` |
| 3 | `.omx/context/ralplan-architect-review-so101-systemic-assurance-iteration-4.md` | `APPROVE (leader fallback)` | `994210bcde6b52c64366129e6377ce9a60e0947a6d3c9757f948950d7fd111e2` |
| 4 | `.omx/context/ralplan-critic-review-so101-systemic-assurance-iteration-2.md` | `APPROVE (leader fallback)` | `daf49b2fbdc6414b6111dcfb9bac07a005c00d1a5a0ded12040dcb684ef698ae` |

Architect review precedes Critic review and both approve the exact same PRD/test-spec digest pair.

## Reviewer runtime disclosure

- Typed Architect `/root/so101_architect_review` timed out after the one permitted 60-second wait and was interrupted immediately. It returned no verdict and was never treated as approval.
- Typed Critic `/root/so101_critic_review` behaved the same way and was never treated as approval.
- The workspace runtime policy forbids retry/wait loops and requires local role fallback. Leader fallback reviews are explicitly labeled and do not impersonate native role results.
- Native-agent approval flags remain false; consensus completion uses the documented leader-fallback path after full Architect→Critic iteration and exact-digest re-review.

## Consensus decision

Adopt contract-first systemic assurance with two isolated lanes:

1. The rule-based mobile manipulation path becomes a physical-truth oracle only after deterministic scripted/canonical gates.
2. ACT remains a candidate that must pass immutable provenance, prospective split, action authority, Dataset v3, checkpoint/processors, offline, no-model replay, sim, held-out, HIL, and signed real-hardware promotion gates.

Do not reuse current `0/10`, `4/5`, 37-episode, or 60-episode artifacts as product performance evidence. They remain legacy/regression evidence until P0 classification and later validation.

## Required execution order

```text
P0/T00 inventory + quarantine
 -> P1/T01 semantic/source/split/formula precommit
 -> P2-P3/T02 regression-only lock + state-machine boundaries
 -> P4/T03 static zero-actuation + base-only characterization + numeric contract
 -> P5/T04 split/access revalidation
 -> P5/T05 timestamped raw capture
 -> P6/T06 write-once Dataset v3
 -> P7/T07 action contract + no-model replay + selection ADR
 -> P8/T08 ACT/checkpoint/processors/offline
 -> P9/T09 deterministic sim
 -> P10/T10 held-out
 -> P10/T11 signed HIL/real promotion
 -> P11/T12 independent verification
```

## Stop conditions

- digest drift, fallback/truncation, split leakage, missing authority, predicate drift, or prerequisite failure stops the candidate before downstream output/actuator initialization.
- Product success threshold and hardware safety envelope remain unknown until precommitted ADRs; `authorization=false` is the correct default.
- Existing user changes and legacy artifacts are not deleted or overwritten.

## Recommended follow-up

Default: `$ultragoal` with the final plan/spec, optionally `$team` after P0/P1 where ownership can be split safely. `$ralph` is only an explicit single-owner fallback. Research/performance goal modes are reserved for separately scoped research or optimization work.
