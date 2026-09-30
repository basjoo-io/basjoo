# Audit Protocol

Three audit stages (arch_audit, impl_audit, code_audit) share the same finding-fix-reaudit loop.

**Roster difference from `reviewing-code`:** Audit stages and code review use different agent rosters because they serve different purposes. `reviewing-code` reviews code diffs (always-on: correctness, testing, maintainability, project-standards, agent-native, learnings). Audit stages review documents and plan-vs-code alignment — agents like `maintainability-reviewer` and `learnings-researcher` are not included because audit focus is architecture/executability/correctness, not code-level maintainability.

## Audit Stages and Their Agents

### Stage 3 — Architecture Audit (`arch_audit`)

Core agents (parallel): `architecture-strategist`, `performance-oracle`, `code-simplicity-reviewer`, `pattern-recognition-specialist`

Focus: "Is the design right?" — SOLID compliance, module boundaries, unnecessary copies, simpler alternatives.

### Stage 5 — Implementation Plan Audit (`impl_audit`)

Core agents (parallel): `spec-flow-analyzer`, `security-sentinel`, `performance-oracle`, `agent-native-reviewer`

Conditional: `data-integrity-guardian`, `data-migration-expert`, `kieran-python-reviewer`, `kieran-typescript-reviewer`

Focus: "Is the plan agent-executable?" — all paths covered, edge cases, code completeness.

**Gate C artifact:** After Stage 5 audit loop completes, compute `agent_native_review_passed` from `agent-native-reviewer` output: `true` if no blocking findings (p1) related to agent-executability remain. Write to `stages.impl_audit.gate_c.artifacts.agent_native_review_passed`.

### Dispatch rules — Stages 3 & 5 (mandatory)

- **Default:** The orchestrator dispatches **one Task subagent per core agent** listed for that stage, **in parallel**. A single merged reviewer (e.g. one `generalPurpose` pass) **must not** be used as the default substitute for the full core set.
- **Quick audit (explicit opt-in only):** If the human explicitly chooses reduced coverage (e.g. timeboxed session), use **AskQuestion** with a clear label such as **「Quick audit — 单合并审查，接受视角缩水」**. Before presenting Gate B or Gate C, record in task YAML, for example:
  - `stages.arch_audit.review_mode: quick` or `stages.impl_audit.review_mode: quick`
  - Optionally `stages.<stage>.review_agents_actual: [...]` so metadata matches what ran.
- **Do not** approve Gate B/C on a quick audit without the above fields — otherwise later readers assume full core coverage that did not occur.

### Stage 8 — Code Audit (`code_audit`)

Core agents (parallel): `architecture-strategist`, `code-simplicity-reviewer`, `correctness-reviewer`, `testing-reviewer`, `pattern-recognition-specialist`, `performance-oracle`, `security-sentinel`, `agent-native-reviewer`

Conditional: `design-implementation-reviewer`, `kieran-python-reviewer`, `kieran-typescript-reviewer`, `julik-frontend-races-reviewer`, `data-integrity-guardian`, `data-migration-expert`, `schema-drift-detector`

Focus: "Does what was built match what was planned?" — drift detection, code quality, correctness, test coverage, agent parity.

**Alternative: `reviewing-code` for Stage 8.** When running Stage 8 via `reviewing-code` (e.g., `mode:headless`), agents produce JSON per `findings-schema.json` instead of the numbered-list format. The orchestrator must use the matching merge pipeline (Stage 5 merge for JSON, audit numbered-list merge for audit format). See agents README "Output Contracts" section for format details.

**Scope: "task-related changed files"** — the union of files listed in the implementation plan's File Map AND files appearing in `git diff` from the task branch. Findings outside this set are `out_of_scope` unless the user requests a broader audit.

**Drift detection algorithm:**

1. Read `paths.implementation` (implementation plan) — extract the File Map (exact paths, actions, responsibilities)
2. Determine base branch: `git merge-base HEAD main 2>/dev/null || git merge-base HEAD master 2>/dev/null`
3. Run `git diff <base>..HEAD --stat` — list all changed files
4. For each planned file, check: implemented as described? Missing? Structurally different?
5. For each changed file NOT in the File Map: is it an unplanned addition?
6. A **drift point** is: a file built differently than planned, a planned file missing, or an unplanned file added
7. Record `stages.code_audit.drift_points` as the total count
8. Each drift point must be dispositioned (accepted as intentional change, or fixed) before Gate E

**Subagent access exception:** `design-implementation-reviewer` receives `paths.plan` AND `paths.implementation` as inputs despite the orchestrator's own Per-Stage Read Policy — subagents dispatched via Task may read any files relevant to their review.

## Agent Dispatch Context

When dispatching review agents, pass explicit context per audit stage:

| Stage          | Document inputs                                            | Review focus to communicate                                                     |
| -------------- | ---------------------------------------------------------- | ------------------------------------------------------------------------------- |
| 3 `arch_audit` | `paths.plan` (architecture plan)                           | Architecture soundness: SOLID, module boundaries, simpler alternatives          |
| 5 `impl_audit` | `paths.implementation` (implementation plan)               | Agent-executability: complete code, valid paths, resolvable targets, edge cases |
| 8 `code_audit` | `paths.plan` + `git diff <base>..HEAD` + changed file list | Plan-vs-code drift, correctness, test coverage, code quality                    |

For each agent Task prompt, include:

1. The document(s) from the table above (pass as file paths for the agent to read)
2. The review focus sentence
3. The output format instruction below

## Agent Output Requirements

When dispatching review agents, include in the Task prompt:

> Return ALL findings in your FINAL message as a single numbered list. Do NOT split findings across multiple messages. Format: `N. [pX] Title — one-line description and fix.`

This ensures the Task tool's last-message return captures the complete findings list.

## Error Recovery

If a review agent crashes, times out, or returns malformed output:

1. Log the failure in task YAML: `stages.<stage>.agent_failures: [{agent, error, iteration}]`
2. Proceed with findings from successful agents
3. If >50% of agents fail, escalate to user via AskQuestion: "Most review agents failed. Retry / Proceed with partial findings / Abort"

## Finding Presentation Format

Present findings to human as a prioritized list:

    ## Audit Findings

    ### p1 (blocking) — must fix before gate (covers both critical and high-impact issues; maps to code review P0+P1)
    1. [architecture-strategist] Title of finding
       Description and recommended fix.

    ### p2 (should fix)
    2. [performance-oracle] Title of finding
       Description and recommended fix.

    ### p3 (nice to have)
    3. [code-simplicity-reviewer] Title of finding
       Description and recommended fix.

**Counting rule for gate artifacts:** `findings_p1` is the count of findings where `priority == p1 AND disposition NOT IN (rejected, wontfix)`. Rejected and wontfix findings do not block the gate.

Then apply `audit_triage_policy` (from stage's YAML field, default `manual`):

- **manual**: AskQuestion "Which findings should I address?" with options per finding
- **auto_safe**: Auto-fix all p1 findings, auto-accept p2 (wontfix), present only p3 via AskQuestion for human decision. If no p1 findings, skip AskQuestion entirely.
- **auto_all**: Auto-fix p1+p2, advisory-only for p3 (log but don't ask). Entire triage is non-blocking.

## Audit Loop

1. Dispatch review agents in parallel (one Task per agent)
2. Collect and merge findings into the format above
3. **Orchestrator triage** — For each p1 finding, independently verify the claim: read the cited file/line, cross-check against project conventions (AGENTS.md, `.cursor/rules/`, existing patterns). If factually incorrect → `rejected` with reason. Do NOT present false positives to user as p1.
   Scope guard: by default, only accept findings within the current task's changed-file whitelist. Mark other findings as `out_of_scope` unless the user requested a broader audit.
4. Apply `audit_triage_policy` to present/route verified findings (see Finding Presentation Format above)
5. Fix approved findings
6. Increment `iteration` in task YAML
7. Re-audit policy (governed by `max_iterations` in task YAML — single source of truth):
   - Round 1 is mandatory
   - Subsequent rounds allowed only if the previous round has unresolved p1 findings
   - Stop after `max_iterations` rounds; escalate unresolved p1 via AskQuestion (see Termination)
8. Compute gate artifacts → present gate

Audit stages stay in the same `status` until the gate passes. The `findings` list in task YAML preserves per-finding disposition (`reported`, `approved`, `fixed`, `wontfix`) across iterations and sessions.

## Termination

`max_iterations` is defined per audit stage in task YAML (single source of truth; default `2`). After reaching max iterations with `findings_p1 > 0`, present to human via AskQuestion:

- "Force proceed (accept remaining p1 risk)"
- "Request manual intervention"
- "Revert to previous stage for revision" — triggers the audit loop revert (see below)
- "Abort task"

Record decision in task YAML: `stages.<stage>.termination_override: force | manual | revert | abort`.

## Audit Loop Revert

The declared revert paths (`arch_audit → arch_plan`, `impl_audit → impl_plan`, `code_audit → execute`) trigger when:

1. The user selects "Revert to previous stage" at audit termination, OR
2. The user selects "Needs revision" at a gate and the revision requires changes to the previous stage's artifact

On revert:

- Set `status` back to the target stage (e.g., `arch_plan`)
- Preserve the current `findings[]` list (do not clear)
- Reset `iteration: 0` for the audit stage
- The reverted-to stage re-runs its skill, producing an updated artifact
- On re-entry to the audit stage, findings from the previous audit are available for comparison

**Escalation revert (Stage 8 only):** If code audit reveals design-level issues beyond execution fixes, offer an additional option: "Revert to arch_plan for architecture revision." This skips back further, setting `status: arch_plan` and preserving `code_audit.findings` for reference.

## deepen-plan Integration (Stage 5 only)

When `agent-native-reviewer` finds "plan insufficiency" (vague code, missing paths, incomplete commands), the fix action is the `deepen-plan` skill on the specific task rather than a direct code edit. The insufficiency type is:

- Paths don't exist or are not valid new-file locations
- Build targets are not resolvable
- Code is fragments or pseudocode, not complete
- Task dependency chain is cyclic
- Verify criteria are not machine-evaluable

## Conditional Agent Activation

Read `compound-engineering.local.md` for per-path agent overrides. Check `stages.<stage>.conditional_agents` in task YAML for task-level overrides. Activation is based on file extension overlap between the plan's changed files and the agent's scope (e.g. `.py` files → `kieran-python-reviewer`).

If `compound-engineering.local.md` does not exist: use core agents only (no conditional agents from local config). Conditional agents listed in `stages.<stage>.conditional_agents` in task YAML still apply regardless.
