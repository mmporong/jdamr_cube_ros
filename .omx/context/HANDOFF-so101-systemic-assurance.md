# AI Handoff — SO-101 Systemic Assurance

## Read first

1. `.omx/state/so101-systemic-assurance.json`
2. `.omx/context/ralplan-consensus-so101-systemic-assurance.md`
3. `.omx/plans/prd-so101-systemic-assurance.md`
4. `.omx/specs/test-spec-so101-systemic-assurance.md`
5. `.omx/context/so101-systemic-assurance-20260807T032752Z.md`
6. `.omx/context/research-so101-lerobot-v061.md`

## Current state

- Ralplan planning is complete through ordered Architect→Critic leader-fallback review.
- No implementation source, data, checkpoint, simulator, or hardware was changed/run by this workflow.
- The Git worktree already contained substantial user modifications and untracked tools before planning; preserve them.
- Current scores and datasets are not promotion evidence because lineage/predicate/split/fallback contracts are incomplete.

## Exact next action

Start a separate execution workflow at **P0/T00 only**. Seal an allowlisted current source/config inventory, classify existing data/checkpoints/results without modifying them, and prove the T00 write-once validator. Do not start P1 or edit behavior code until T00 complete evidence exists.

Recommended execution: `$ultragoal` as durable ledger owner. Add `$team` only after P0/P1 and respect the current native hard limit of leader plus at most three child agents.

## Prohibitions

- Do not delete/overwrite `capstone_pick/tools/logs` or current untracked files.
- Do not relabel existing train episodes as held-out.
- Do not treat missing controller references as measured state/zero/last-value action.
- Do not use `min(lengths)` to hide partial episodes.
- Do not initialize hardware while authorization is false/null.
