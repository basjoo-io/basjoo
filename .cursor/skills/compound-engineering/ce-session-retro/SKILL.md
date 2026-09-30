---
name: ce-session-retro
description: >
  CE workflow retrospective with six-dimension analysis. Use when reviewing
  a completed or partially completed /run-ce-workflow task for dispatch
  accuracy, workflow friction, and skill/agent effectiveness. Triggers:
  /ce-session-retro, session retro, CE 复盘, 工作流复盘, workflow retro.
disable-model-invocation: true
argument-hint: "[task-id]"
---

# CE Session Retrospective

**Announce at start:** "I'm using the ce-session-retro skill to review this CE workflow session."

## Overview

Structured retrospective for `/run-ce-workflow` sessions. Analyzes six dimensions: (1) workflow execution quality, (2) missed dispatches, (3) spurious dispatches, (4) workflow process optimization, (5) individual skill/agent effectiveness, and (6) other observations. Produces actionable improvement items that feed back into skill/agent/workflow updates.

## When to Use

- After completing (or partially completing) a task via `/run-ce-workflow`
- When the user wants to review a CE workflow session for improvements
- As part of a periodic CE workflow health check

**When NOT to use:** General session retrospectives unrelated to the CE workflow — use `/session-retro` instead.

## Parameters

- `task-id` (optional): Reads `docs/tasks/<task-id>.yaml` for stage history. If omitted, infer from the most recent task YAML in `docs/tasks/`.

## Inputs

| Source                                             | Purpose                                               |
| -------------------------------------------------- | ----------------------------------------------------- |
| `docs/tasks/<task-id>.yaml`                        | Stage history, gate decisions, agent failures, timing |
| Current session transcript                         | Actual skill/agent invocations, friction points       |
| `agent-transcripts/` directory                     | Prior session transcripts for multi-session tasks     |
| `.cursor/skills/compound-engineering/`         | Skill inventory for dispatch analysis                 |
| `compound-engineering.local.md`                | Configured conditional agents                         |
| `run-ce-workflow/references/audit-protocol.md` | Expected agent dispatch per stage                     |
| `docs/retros/`                                     | Prior retro reports for pattern detection             |

Create TodoWrite todos when invoked:

```
TodoWrite todos:
  - id: "rt-1", content: "Phase 1: Gather evidence (task YAML + transcript + skill inventory + prior retros)", status: "pending"
  - id: "rt-2", content: "Phase 2: Analyze workflow execution quality", status: "pending"
  - id: "rt-3", content: "Phase 3: Audit skill/agent dispatch accuracy", status: "pending"
  - id: "rt-4", content: "Phase 4: Evaluate workflow process optimization", status: "pending"
  - id: "rt-5", content: "Phase 5: Review individual skill/agent effectiveness", status: "pending"
  - id: "rt-6", content: "Phase 6: Synthesize findings + propose actions", status: "pending"
```

## Phase 1: Gather Evidence

### 1.1 Load task state

Read `docs/tasks/<task-id>.yaml`. Extract:

- Stages completed (`status` history, `sessions[]`)
- Gate decisions and dates
- Agent failures (`agent_failures[]`)
- Audit iterations and findings counts
- Timing signals (session count, stages per session)

If task YAML is missing or incomplete, ask the user to describe the session.

### 1.2 Reconstruct dispatch history

Gather transcript evidence from all available sources:

1. **Current session**: The conversation context contains what happened this session — scan for skill announcements ("I'm using the * skill"), Task tool dispatches, AskQuestion interactions, error recovery, and manual interventions.
2. **Prior sessions**: If `sessions[]` in task YAML lists earlier sessions, check `agent-transcripts/` for matching `.jsonl` files. Read relevant transcripts to reconstruct dispatch history across the full task lifecycle.
3. **Task YAML signals**: `stages.<stage>.agent_failures`, `stages.<stage>.iteration`, and `stages.<stage>.conditional_agents` record dispatch facts even without transcripts.

Build a dispatch log: `{stage, skill_loaded, agents_dispatched[], gates_presented[], errors[]}` per stage.

If all three sources are empty (new session, no prior transcripts, no task YAML history), ask the user to describe what happened in the workflow session.

### 1.3 Load reference inventories

Read (on demand, not bulk):

- `run-ce-workflow/SKILL.md` — Stage Dispatch table, Supporting Skills
- `run-ce-workflow/references/audit-protocol.md` — expected agents per audit stage
- `compound-engineering.local.md` — configured conditional agents
- README.md — full skill and agent inventory

### 1.4 Review prior retros

Scan `docs/retros/` for previous retro reports. If prior retros exist:

- Extract recurring findings (same issue appearing in 2+ retros)
- Note previously proposed actions that were or weren't implemented
- Flag systemic patterns for escalation in Phase 6

If no prior retros exist, skip this step.

## Phase 2: Workflow Execution Quality

Evaluate each completed stage against its expected behavior:

| Check                 | What to look for                                                      |
| --------------------- | --------------------------------------------------------------------- |
| Stage ordering        | Did stages execute in the correct sequence? Any unexpected skips?     |
| Gate compliance       | Were all required gates presented? Were artifacts computed correctly? |
| Session boundaries    | Were context health assessments performed? Were handoffs generated?   |
| TodoWrite discipline  | Were stage-prefixed todos created and completed?                      |
| Per-stage read policy | Did the orchestrator avoid reading documents outside its read policy? |
| Error recovery        | Were failures handled per protocol (agent crashes, build failures)?   |

For each issue found, classify:

- **Protocol violation** — skill/workflow says X, but Y happened
- **Missed opportunity** — a better path was available but not taken
- **Friction** — correct behavior, but unnecessarily slow or confusing

## Phase 3: Skill/Agent Dispatch Audit

The core analysis. Two sub-dimensions:

### 3.1 Missed dispatches (should have been called, but weren't)

Determine expected dispatches by reading the **source of truth** dynamically — do NOT rely on memorized agent lists:

1. **Expected skills per stage**: Read `run-ce-workflow/SKILL.md` Stage Dispatch table
2. **Expected agents per audit stage (3/5/8)**: Read `run-ce-workflow/references/audit-protocol.md`
3. **Expected conditional agents**: Read `compound-engineering.local.md` and match globs against task diff
4. **Expected research agents (Stage 1)**: Read `brainstorming/SKILL.md` Phase 2 table
5. **Expected document reviewers (Stages 1/2/4)**: Read each stage's skill for reviewer dispatch

Compare expected vs actual (from Phase 1.2). Also check **supporting skills** that activate on triggers:

- `systematic-debugging` — were there any test/build failures during execution?
- `deepen-plan` — were there "plan insufficiency" findings in Stage 5?
- `reviewing-code` — was it used in Stage 8 when appropriate?
- GitNexus `impact`/`context` — were they called before modifying code?

For each miss: state which skill/agent, which stage, why it should have been called, and estimated impact (high/medium/low).

### 3.2 Spurious dispatches (called, but shouldn't have been)

Identify agents or skills that were dispatched unnecessarily:

- Agent dispatched for a stage where it's not in the core or conditional list
- Skill loaded when its trigger conditions weren't met
- Redundant dispatches (same agent called twice for the same review)
- Research agents dispatched when the information was already available
- Conditional agents activated despite no matching file globs in the diff

For each spurious dispatch: state which skill/agent, which stage, why it was unnecessary, and token/time cost (high/medium/low).

## Phase 4: Workflow Process Evaluation

Evaluate the workflow structure itself for optimization:

### 4.1 Stage transition friction

- Were any stage transitions unnecessarily slow?
- Did the orchestrator get stuck in loops (audit re-iterations)?
- Were gates too strict or too lenient for the task's actual risk?
- Did session management (handoff generation) interrupt flow?

### 4.2 Information flow

- Did later stages lack context that earlier stages had?
- Were documents re-read unnecessarily (violating read policy)?
- Did subagent outputs provide sufficient information to the orchestrator?
- Were handoff prompts complete enough for session resumption?

### 4.3 Process gaps

- Were there decision points where the workflow had no guidance?
- Did any stage's completion detection fail or trigger prematurely?
- Were there manual workarounds that should be automated?
- Did the `gate.policy` settings match the task's actual needs?

### 4.4 Overhead assessment

- Which stages added the most value vs. overhead for this task?
- Could any stages have been skipped or combined for this task size?
- Were audit iterations proportionate to the finding severity?

## Phase 5: Individual Skill/Agent Effectiveness

For each skill and agent that was actually invoked, evaluate:

| Dimension           | Question                                                                         |
| ------------------- | -------------------------------------------------------------------------------- |
| Output quality      | Did the skill/agent produce useful, actionable output?                           |
| Instruction clarity | Were skill instructions clear enough for the orchestrator to follow?             |
| False positives     | Did agents report findings that were factually wrong?                            |
| Missed findings     | Were there real issues the agents should have caught but didn't?                 |
| Token efficiency    | Did the skill/agent use context efficiently, or waste tokens on irrelevant work? |
| Interaction design  | Were AskQuestion prompts well-designed? Too many? Too few?                       |

For each issue, classify as:

- **Skill bug** — skill instructions are incorrect or contradictory
- **Agent prompt gap** — agent definition missing important guidance
- **Skill enhancement** — working correctly but could be improved
- **Documentation gap** — behavior is correct but poorly documented

## Phase 6: Synthesis and Actions

### 6.1 Present findings table

Print a structured summary. **Omit dimensions with no findings** — do not print empty tables.

```
## CE Retrospective: <task-id>

### Dimension 1: Workflow Execution Quality
| # | Finding | Category | Impact | Action |
|---|---------|----------|--------|--------|
| 1 | ... | protocol_violation | high | ... |

### Dimension 2: Missed Dispatches
| # | Skill/Agent | Stage | Reason | Impact |
|---|-------------|-------|--------|--------|
| 1 | security-sentinel | 5 impl_audit | Core agent per audit-protocol but not dispatched | high |

### Dimension 3: Spurious Dispatches
| # | Skill/Agent | Stage | Reason | Waste |
|---|-------------|-------|--------|-------|

### Dimension 4: Process Optimization
| # | Finding | Category | Benefit | Action |
|---|---------|----------|---------|--------|

### Dimension 5: Skill/Agent Effectiveness
| # | Skill/Agent | Issue | Category | Action |
|---|-------------|-------|----------|--------|

### Dimension 6: Other Observations
| # | Finding | Context | Action |
|---|---------|---------|--------|
```

**Dimension 6** captures anything that doesn't fit dimensions 1–5: task YAML schema gaps, skill interdependency issues, tooling limitations, or patterns from prior retros (Phase 1.4).

**Recurring patterns**: If Phase 1.4 found the same issue in prior retros, tag it `[recurring]` and escalate to a structural fix rather than a point fix.

### 6.2 Prioritize with user

Use AskQuestion to let the user select which findings to act on:

- "Which improvements should we implement now?" (multi-select)
- For each selected: propose the minimal diff (one-sentence addition or fix)

### 6.3 Apply approved changes

For each approved action, make the edit. Typical targets:

| Action type               | Target file                                                |
| ------------------------- | ---------------------------------------------------------- |
| Fix skill instructions    | `.cursor/skills/compound-engineering/<skill>/SKILL.md` |
| Fix agent prompt          | `.cursor/agents/<agent>.md`                                |
| Update workflow dispatch  | `run-ce-workflow/SKILL.md`                             |
| Update audit protocol     | `run-ce-workflow/references/audit-protocol.md`         |
| Update session management | `run-ce-workflow/references/session-management.md`     |
| Update conditional agents | `compound-engineering.local.md`                        |
| Update README             | `README.md`                                                |

### 6.4 Handoff summary

End with a one-line handoff:

> 如果把这个复盘交给下一个 agent，最重要的一件事是：\_\_\_

## Output Rules

- **NEVER** create new files beyond the retro report itself
- Retro report written to `docs/retros/YYYY-MM-DD-<task-id>-retro.md`
- Edits to existing skills/agents/workflow files are the primary output
- Keep findings specific and actionable — no vague "improve X"

## Quick Reference

| Item                 | Value                                                                                                                             |
| -------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| **Dimensions**       | 6: execution quality, missed dispatches, spurious dispatches, process optimization, skill/agent effectiveness, other observations |
| **Input**            | Task YAML + transcripts (current + prior) + skill inventory + prior retros                                                        |
| **Output**           | Retro report + approved edits to skills/agents/workflow                                                                           |
| **TodoWrite prefix** | `rt-`                                                                                                                             |
