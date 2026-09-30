---
name: run-ce-workflow
description: >
  Structured engineering workflow orchestration. Use when starting or resuming
  a development task via the stage-based workflow. Triggers:
  /run-ce-workflow, task YAML, 开始任务, 恢复任务.
disable-model-invocation: true
argument-hint: "<task-id> [--stage <stage>] [--to <stage>] [--status]"
---

# Engineering Workflow

## Overview

Orchestrate the multi-stage development cycle with human gates. Each gate is a natural session split point — after approval, assess context health and recommend a fresh chat only when 2+ exhaustion signals fire (see [session-management.md](references/session-management.md)). Execution is driven by `docs/tasks/<task-id>.yaml` and the stage dispatch table in this skill.

## When to Use

- Starting a new development task with structured workflow stages
- Resuming an existing task via `docs/tasks/<task-id>.yaml`
- Checking task status with `--status`

**When NOT to use:** Ad-hoc fixes, quick questions, or tasks that do not need staged delivery.

## Parameters

- `task-id` (required): Matches `docs/tasks/<task-id>.yaml`. If missing, create from [task-state.yaml](templates/task-state.yaml).
- `--stage <stage>` (optional): Start from this stage instead of resuming from `status`. Validate that the target stage's prerequisite transition conditions (from gate-protocol) are met. If not met, warn user via AskQuestion and ask whether to proceed anyway.
- `--to <stage>` (optional): Execute through this stage (inclusive). If the stage has a gate, execute through the gate. After completing, save state to YAML and present a summary: stages completed, current status, next stage if continued.
- `--status` (optional): Display current task status without executing. Shows: current stage, gate statuses, document paths, blockers. Read-only — does not advance state.

## Startup Protocol

1. If `--status`: read task YAML, display status summary, stop (read-only)
2. **Auto-Setup:** Check if `compound-engineering.local.md` exists in the project root. If absent, read and follow `setup/SKILL.md` with `--auto` flag (non-interactive: detect stack → auto-configure → write config → one-line summary). If present, skip silently.
3. Read `docs/tasks/<task-id>.yaml` (create from template if new)
4. Read `status` field → determine current stage
5. Read the **required inputs** for the current stage exactly as defined in [session-management.md](references/session-management.md) (some stages require both a document and git diff context).
6. Execute the stage below

⚠️ If task YAML does not exist and `--stage` is set to a non-brainstorm stage, warn the user — starting mid-workflow without prior stages is risky.

## TodoWrite Policy (Workflow-wide)

- Always update todos with `merge: true`
- Never reset workflow todos with `merge: false` during stage transitions
- Use stage-scoped id prefixes to avoid collisions (`bs-`, `ap-`, `aa-`, `ip-`, `ia-`, `td-`, `ex-`, `ca-`, `sh-`, `vf-`, `cp-`)
- At each stage end: mark that stage's todos `completed` and keep history visible

## Stage Dispatch

"Load skill" means: read `skills/<skill-name>/SKILL.md` via the Read tool and follow its process. The sub-skill reads `docs/tasks/<task-id>.yaml` autonomously. Ensure task YAML `status` matches the expected stage before loading.

| #   | Status       | Action                                                                                                                                                                                                                                                                                                                                                                               | Gate  |
| --- | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----- |
| 1   | `brainstorm` | Read and follow `brainstorming/SKILL.md` → produces `paths.design`                                                                                                                                                                                                                                                                                                               | **A** |
| 2   | `arch_plan`  | Read and follow `writing-arch-plans/SKILL.md` → produces `paths.plan`                                                                                                                                                                                                                                                                                                            | —     |
| 3   | `arch_audit` | Dispatch review agents per [audit-protocol.md](references/audit-protocol.md) Stage 3 → fix → re-audit                                                                                                                                                                                                                                                                                | **B** |
| 4   | `impl_plan`  | Read and follow `writing-impl-plans/SKILL.md` → produces `paths.implementation`                                                                                                                                                                                                                                                                                                  | —     |
| 5   | `impl_audit` | Dispatch review agents per [audit-protocol.md](references/audit-protocol.md) Stage 5 → fix → re-audit                                                                                                                                                                                                                                                                                | **C** |
| 6   | `tdd`        | Branch Setup (see below) → read and follow `test-driven-development/SKILL.md` (RED phase only) → failing tests                                                                                                                                                                                                                                                                   | **D** |
| 7   | `execute`    | **Execution Strategy AskQuestion** (single-select) then load chosen skill → GREEN code. Options: ① _Subagent-Driven (recommended)_ → load `subagent-driven-development`; ② _Inline Execution_ → load `executing-plans`; ③ _(optional) Save only_ → stop, user runs execution in a later session. Record choice in `stages.execute.strategy: subagent \| inline \| deferred`. | —     |
| 8   | `code_audit` | Dispatch review agents per [audit-protocol.md](references/audit-protocol.md) Stage 8 → fix → re-audit                                                                                                                                                                                                                                                                                | **E** |
| 9   | `ship`       | Read and follow `finishing-a-development-branch/SKILL.md` → PR or merge. If user chooses "Keep as-is": set `stages.ship.pr: skipped`, `stages.ship.skip_reason`, advance.                                                                                                                                                                                                        | —     |
| 10  | `verify`     | Read and follow `verification/SKILL.md` → select method, execute checks, collect evidence                                                                                                                                                                                                                                                                                        | **F** |
| 11  | `compound`   | Skip detection first (see below). If `compound-docs/SKILL.md` exists: read and follow it → write solutions. If absent: skip this stage. Then if `agents-memory-updater` agent available: dispatch as Task; if absent: skip.                                                                                                                                                      | —     |
| —   | `done`       | Task complete. Present summary: all gate dates, PR link, solution docs. AskQuestion: "Start new task" / "View artifacts" / "Reopen task (returns to verify)" / "Close".                                                                                                                                                                                                              | —     |

**Stages 3 & 5 — audit dispatch:** Default is **one parallel Task per core agent** per [audit-protocol.md](references/audit-protocol.md) «Dispatch rules — Stages 3 & 5». A single merged reviewer is allowed **only** after explicit **Quick audit** opt-in and YAML `review_mode: quick` (same section). Do not treat ad-hoc shortcuts as equivalent to full Stage 3 / 5 coverage.

**Execution Strategy details (Stage 7):** The AskQuestion is described inline in the Stage Dispatch table row for `execute`. Record the user's choice in `stages.execute.strategy` (`subagent`, `inline`, or `deferred`). If the user chose `deferred`, write handoff and stop — do not load an execution skill. If resuming and `stages.execute.strategy` is already set, skip the AskQuestion and reload the previously chosen skill.

**Stage 8 scope guard:** Review only task-related changed files by default (union of implementation plan File Map + `git diff` from task branch). Treat findings outside this set as `out_of_scope` unless user explicitly requests a broader audit.

**Stage 11 skip detection:** Run `git diff --stat` against the task branch base. If total changed lines < 20 AND only 1 file changed, OR if all changed files are `.md`, offer skip via AskQuestion. On skip: set `retro_written: true`, `knowledge_synced: true`. After compound-docs completes: set `retro_written: true`. After agents-memory-updater completes or is skipped: `knowledge_synced` remains as set by compound-docs.

## Branch Setup (before Stage 6)

When transitioning from `impl_audit` (Gate C) to `tdd`, check branch state:

1. If already on a feature branch → skip, record `stages.tdd.branch`
2. If on main/master → AskQuestion:
   - "Create branch `feat/<task-id>` (current directory)" → `git checkout -b feat/<task-id>`
   - "Create worktree (isolated workspace)" → use `git worktree add` to set up isolated workspace
   - "Stay on main (I'll handle branching)" → record choice
3. Write to task YAML: `stages.tdd.branch`, `stages.tdd.worktree_path` (if worktree)

## Status Transitions

    brainstorm → arch_plan → arch_audit → impl_plan → impl_audit → tdd → execute → code_audit → ship → verify → compound → done

Audit revert loops (triggered by user at audit termination or gate rejection — see [audit-protocol.md](references/audit-protocol.md)):

- `arch_audit → arch_plan` — revise architecture plan
- `impl_audit → impl_plan` — revise implementation plan
- `code_audit → execute` — fix implementation
- `code_audit → arch_plan` — escalation revert for design-level issues (Stage 8 only)

## Skill Completion Detection

| Stage          | Completion signal                                                                                                            |
| -------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| 1 `brainstorm` | `paths.design` written + `stages.brainstorm.gate_a.artifacts.document_review_passed == true` + Gate A answered/auto-approved |
| 2 `arch_plan`  | `paths.plan` written + `stages.arch_plan.document_review_passed == true`                                                     |
| 3 `arch_audit` | Audit loop terminated + Gate B answered/auto-approved                                                                        |
| 4 `impl_plan`  | `paths.implementation` written + `stages.impl_plan.document_review_passed == true`                                           |
| 5 `impl_audit` | Audit loop terminated + Gate C answered/auto-approved                                                                        |
| 6 `tdd`        | `stages.tdd.test_count > 0` + `stages.tdd.all_tests_red == true` + Gate D answered/auto-approved                             |
| 7 `execute`    | `stages.execute.all_checkpoints_done == true` + `stages.execute.all_tests_green == true`                                     |
| 8 `code_audit` | Audit loop terminated + Gate E answered/auto-approved                                                                        |
| 9 `ship`       | `stages.ship.pr` is set (URL, `skipped`, `local_merge`, or `discarded`)                                                      |
| 10 `verify`    | Gate F answered/auto-approved                                                                                                |
| 11 `compound`  | `stages.compound.retro_written == true` + `stages.compound.knowledge_synced == true`                                         |

On detection, update task YAML status and proceed to the next stage.

## Workflow Detection Convention

A sub-skill is "within the workflow" when `docs/tasks/<task-id>.yaml` exists AND the task YAML `status` matches the skill's expected stage. Sub-skills should check this to decide whether to skip their own handoff menus (the orchestrator manages transitions).

## Gate Behavior

At each gate, compute artifacts, apply gate policy, advance or loop. Full protocol: [gate-protocol.md](references/gate-protocol.md).

**Gate policy precedence:** The `gate.policy` field in task YAML takes precedence over any per-skill AskQuestion hardcoding. If `gate.policy == auto_if_pass` and all artifacts are `true`, auto-approve without presenting AskQuestion. Sub-skills must check `gate.policy` before presenting gate UI.

## Document Review vs Audit Agents

- **document-reviewer** (runs in Stages 1, 2, 4): structural/quality gate — formatting, completeness, section coverage, internal consistency. Sets `document_review_passed`.
- **Audit agents** (run in Stages 3, 5, 8): content/correctness review — architecture soundness, security, performance, agent-executability, drift. Produces `findings[]`.

## Stage Transition (MANDATORY after every stage completion)

After completing a stage and updating task YAML, **you MUST execute this checklist in order**:

### 1. Evaluate context health (print the table)

Count each signal and **print a brief inline assessment** (do not skip this):

| Signal                           | This session's value | Verdict            |
| -------------------------------- | -------------------- | ------------------ |
| Stages completed this session    | _fill in_            | sufficient / tired |
| Subagent dispatches this session | _fill in_            | sufficient / tired |
| Documents read (>200 lines)      | _fill in_            | sufficient / tired |
| Audit loops this session         | _fill in_            | sufficient / tired |
| Conversation turns               | _fill in_            | sufficient / tired |

**Decision rule:** If **2+ signals = tired** → in the follow-up **AskQuestion**, recommend **新开聊天继续** in one line above the widget. **Do not** skip the handoff block or the question menu solely because signals are fresh.

### 2. Handoff prompt: file + chat (MANDATORY)

Per [session-management.md](references/session-management.md) §2:

1. Write `docs/tasks/<task-id>-handoff.md` (overwrite).
2. In chat, print heading **`### 可复制续跑提示`** and a **single fenced code block** with the **same verbatim content** as the file. The chat copy is the **primary** UX; do not instruct the user to open the file first.
3. The block MUST be self-contained: `/run-ce-workflow <task-id>`, current `status` / gate, `paths.*`, key decisions, and the exact next action for the next agent.

### 3. Next step: AskQuestion (MANDATORY)

After §1 table + §2 handoff block, use **AskQuestion** — **never** make free-form typing (e.g. "回一句执行 Task 1") the only way to continue.

**Default options** (localize labels if needed; keep meanings):

1. **继续本会话** — load the next stage skill (or current stage if resuming) and proceed immediately after the user submits this answer.
2. **新开聊天继续** — stop; user pastes the §2 block into a new chat.
3. **暂停** — stop without starting the next stage.

**When `status` transitions to `tdd` and `paths.implementation` is set:** Read the implementation plan; add a **first option** or dedicated question that names **Task 1** (or the first unchecked task), e.g. _「本会话：从实现计划 Task 1 — &lt;title&gt; 开始 TDD RED」_ vs _「本会话：我自己选稍后开始的 Task」_ — still use **AskQuestion**, not typed instructions.

**When stage 7 `execute` is next:** Use the **Execution Strategy AskQuestion** from the Stage Dispatch table (row 7) as the "next step" question — it replaces the generic "继续本会话 / 新开聊天 / 暂停" menu for this boundary. The **可复制续跑提示** block still appears **before** the execution-strategy question.

### 4. Boundary reminder

Present at most **one** stage-boundary status line for this stage completion (in addition to §2–§3). Do not interrupt mid-stage. Do not ask duplicate **AskQuestion** widgets for the same gate boundary.

Full protocol details: [session-management.md](references/session-management.md).

## Resume

Re-invoking with the same `task-id` reads YAML and continues from `status`. Completed stages are not re-executed.

## Conditional Agents

Read `compound-engineering.local.md` (auto-generated by Startup Protocol step 2 via `setup --auto`; can also be manually regenerated with `/setup`) for per-path agent overrides. This file is per-developer configuration — ensure it is listed in `.gitignore`.

Expected format of `compound-engineering.local.md`:

```yaml
# Per-path agent overrides — merge with core agents for matching stages
overrides:
  - glob: "**/*.py"
    add_agents: [kieran-python-reviewer]
    stages: [impl_audit, code_audit]
  - glob: "**/*.ts"
    add_agents: [kieran-typescript-reviewer]
    stages: [impl_audit, code_audit]
  - glob: "**/migrations/**"
    add_agents: [data-integrity-guardian, data-migration-expert]
    stages: [code_audit]
```

Merge semantics: `add_agents` are appended to the core agent list for matching stages. Core agents are never removed by local config.

## External Dependencies

| Resource                | Type   | Location                                                | Required       | Fallback                                                                                                            |
| ----------------------- | ------ | ------------------------------------------------------- | -------------- | ------------------------------------------------------------------------------------------------------------------- |
| `knowledge`         | plugin | `~/.cursor/plugins/local/knowledge/`                | Yes (Stage 11) | If missing: AskQuestion "Install knowledge" / "Skip compound docs" / "Write to local `docs/solutions/` instead" |
| `agents-memory-updater` | agent  | `continual-learning` plugin or Cursor built-in subagent | No (Stage 11)  | Skip memory update                                                                                                  |

**Build system:** Skills use CMake commands (`ctest`, `clang-format`, `ruff`). Detect build system from project root files (`CMakeLists.txt`, `pyproject.toml`).

## Supporting Skills (available at any stage)

| Skill                                | When to load                                                                  |
| ------------------------------------ | ----------------------------------------------------------------------------- |
| `systematic-debugging`           | Any test failure, build failure, or unexpected behavior during execution      |
| `using-git-worktrees` (global skill) | Branch Setup (before Stage 6), or when isolated workspace needed at any stage |

## References

- [session-management.md](references/session-management.md) — Session boundaries, per-stage read policy, subagent offloading
- [gate-protocol.md](references/gate-protocol.md) — Gate artifacts, transition validation, decision packet format
- [audit-protocol.md](references/audit-protocol.md) — Audit finding format, loop semantics, deepen-plan integration
- [task-state.yaml](templates/task-state.yaml) — Task state file template
