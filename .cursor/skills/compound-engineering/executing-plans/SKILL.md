---
name: executing-plans
description: >
  Use when an implementation plan exists, task status is execute, or the user
  says execute plan, run tasks, or 执行计划. Triggers: execute plan, run tasks,
  执行计划, 开始实现.
argument-hint: "[task id or path to docs/tasks/<task-id>.yaml]"
---

# Executing Plans

**Announce at start:** "I'm using the executing-plans skill to implement this plan."

## When to Use

- An implementation plan (`implementation.md`) exists and is ready for execution
- Task file status is `execute`
- The user says "execute the plan", "run the tasks", or "start implementing"

**When NOT to use:** When no plan exists yet (use **writing-impl-plans** first) or when the task needs brainstorming.

## Overview

Load plan, review critically, execute tasks in batches, report for review between batches.

When the task file is already loaded (you have `docs/tasks/<task-id>.yaml` and `paths.implementation`), treat **implementation.md** as the plan document and **skip bare-prompt triage** (no Phase-0-style discovery routing). Proceed directly to Startup below.

## Execution Mode Routing

This skill implements **Inline Execution** — batch execution with per-task checkpoints in the current session. The alternative is **Subagent-Driven** (`subagent-driven-development`) — fresh subagent per task with two-stage review.

**Within `/run-ce-workflow`:** The workflow orchestrator already asked the Execution Strategy AskQuestion at Stage 7 entry and routed here. **Skip this section** — proceed directly to Startup.

**Standalone invocation** (no task YAML, or invoked directly by user): Present AskQuestion before starting:

- title: "Execution Mode"
- prompt: "How do you want to execute this plan?"
- options:
  - "Inline Execution (this session) — batch execution with per-task checkpoints" → proceed to Startup
  - "Subagent-Driven (recommended for many tasks) — fresh subagent per task + two-stage review" → load `subagent-driven-development` instead; **stop** this skill
  - "Review plan first — I want to read before deciding" → show plan, then re-ask

## Startup

1. Read `docs/tasks/<task-id>.yaml` — get `paths.implementation`
2. Read implementation.md
3. Review critically — identify concerns. Treat the plan as a decision artifact: use `Implementation Units`, verification, scope boundaries, and deferred items if present. If scope is large or ambiguous and there is no solid plan yet, suggest **brainstorming** or **writing-impl-plans** before execution; honor the user's choice.
4. If concerns → AskQuestion: "Proceed anyway" / "Address concerns first" / "Abort"
5. Parse Task list → build dependency DAG
6. Group into Batches by dependency level
7. Create TodoWrite todos (one per task)

## Execution

### Per Batch

**Dispatch Matrix (Incremental)**

- **Serial subagents**: 3+ tasks with strict dependencies.
- **Parallel subagents**: independent tasks touching non-overlapping files.
- **Default: single-threaded execution.** Override away from single-threaded only when parallel or serial dispatch above clearly applies; when unclear, or for high-coupling refactors where ordering risk is high, or when file overlap/dependency is ambiguous, stay single-threaded.

1. For independent tasks within a batch: dispatch via `subagent-driven-development` or `best-of-n-runner` (parallel)
2. For each task internally:
   - Implement code as specified
   - **Test discovery** — Before changing a file, find existing tests (test/spec files that import, reference, or share naming with it). Prefer plan-listed tests, then widen search. Update tests for new/changed/removed behavior.
   - `ctest` target → GREEN
   - **System-wide test check** — **Default: perform this check.** Override: skip only for trivial leaf changes. What fires (callbacks, middleware, observers)? Do tests exercise a real chain vs all-mocked isolation? Can failures leave orphaned state (DB/cache/file before external calls)? Other entry points (mixins, alternate APIs)? Align error handling across layers (no double-retry conflicts). If nothing fires and change is additive-only, a quick mental pass suffices.
   - `clang-format -i <files>`
   - Lint check
   - GitNexus `detect_changes` — verify only expected files changed
   - **Incremental commit** — After a logical unit passes tests and format: if you can describe a complete change in one conventional message, commit scoped files (`git add` only that unit); avoid WIP/partial messages. Use **publishing-pr** (or project workflow) for push/PR when appropriate.
   - Mark task completed in TodoWrite
   - **Per-Task handoff** — If this unit maps to a numbered Task in `implementation.md` and `checkpoint_policy` is `manual`, follow **Per-Task 衔接** before starting the next Task (unless the user already chose **后续全部执行** this session).

### Per-Task 衔接（`checkpoint_policy: manual`）

当 `implementation.md` 里存在 **Task 1…N** 时，以 **每个 Task 完成** 为默认停损点（不必等到整批 Batch 结束）。

在每个 Task **全部完成**（测试绿、format、lint、`detect_changes`、TodoWrite、可选增量 commit）之后：

1. 一行摘要：改动要点 + 关键验证命令/结果。
2. **必须调用 `AskQuestion`** 再开下一 Task — **除非** 本会话内用户已选 **后续全部执行**（见下表）。

**`AskQuestion` 选项（中英文标签可择一，语义一致即可）：**

| 选项                                     | 行为                                                                                                               |
| ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| **继续下一项** / Continue next task      | 只执行 **下一个** Task；到下一边界再停。                                                                           |
| **后续全部执行** / Approve all remaining | **本会话内** 连续跑完剩余 Task；**不再**在每个 Task 边界 `AskQuestion`；遇 blocker、测试失败、指令歧义仍必须停下。 |
| **有反馈** / I have feedback             | 用户改计划或补充约束后再继续。                                                                                     |
| **停止** / Stop                          | 结束执行；更新 task YAML 的 `stages.execute.checkpoint_current`（及 handoff 若存在）。                             |

**会话内记忆：** 用户一旦选 **后续全部执行**，后续 Task 边界跳过步骤 2，直至计划完成或停止条件；**新会话**默认恢复「每 Task 一问」。

### Checkpoint（Batch 与 Task 对齐）

读 task YAML：`stages.execute.checkpoint_policy`。

- **`manual`**（默认）：
  - 有 **implementation.md 编号 Task**：以 **Per-Task 衔接** 为主；Batch 末尾可再发摘要，但**不替代**每 Task 的 `AskQuestion`（除非已 **后续全部执行**）。
  - **仅有 Batch、无逐 Task 清单**：每 Batch 结束后 `AskQuestion`，摘要「Completed tasks X–Y」+ 验证；选项与上表对齐（**继续下一批** ↔ **继续下一项**）。
- **`auto`**：测试绿且 `detect_changes` 符合预期则自动推进；打日志，不在边界 `AskQuestion`。

更新 task 文件：`stages.execute.checkpoint_current: N`，`stages.execute.checkpoint_total: M`。

### Per-Task Quality Gate

**Default: run a two-stage review** after implementation for non-trivial tasks. **Override:** skip for simple tasks (single-file change, < 20 lines) — the next Task handoff or batch checkpoint covers it.

1. **Spec compliance** — Does the code match what the plan specified? Missing requirements? Extra code not requested?
2. **Code quality** — Style, naming, error handling, test coverage.

Spec compliance runs first. Code quality only starts after spec is satisfied. If either stage finds issues, fix and re-verify before marking the task complete.

**Tier hint (mirrors upstream depth):** Use inline self-review only when change is purely additive, single-concern, pattern-following, and plan-faithful. Otherwise run the **local review workflow** (e.g. `workflows:review` or project reviewer agents) before marking the batch or feature done.

### Blockers

When hitting a blocker (regardless of checkpoint_policy):

- AskQuestion: describe blocker + options: "Help resolve" / "Skip task" / "Abort"
- Record in task file: `stages.execute.blockers`

## When to Stop

Stop executing immediately when:

- Hit a blocker mid-batch (missing dependency, test fails, instruction unclear)
- Plan has critical gaps preventing starting
- An instruction is ambiguous and guessing could cause damage
- Verification fails repeatedly (3+ attempts)
- `detect_changes` shows unexpected files changed and the cause is unclear

Ask for clarification rather than guessing.

## When to Revisit Earlier Steps

Return to Startup (re-read plan) when:

- User updates the plan based on feedback
- Fundamental approach needs rethinking after blocker analysis

Return to previous batch when:

- A completed task turns out to have a defect discovered in a later batch

Don't force through blockers — stop and ask.

## Completion

After all tasks:

1. Run `ctest --preset linux-gcc-x64` (full test suite)
2. Run `clang-format` on all changed files
3. GitNexus `detect_changes` — final scope verification
4. For UI/design changes: capture and upload screenshots; include URLs in final PR/summary
5. If standalone (not within `/run-ce-workflow`): load `finishing-a-development-branch` skill for branch integration options
6. If within workflow: set `stages.execute.all_checkpoints_done: true`, `stages.execute.all_tests_green: true`, advance `status: code_audit`

## Post-Deploy Monitoring & Validation

Before final PR or handoff, add a **Post-Deploy Monitoring & Validation** block to the PR description (or session summary):

- Log queries / search terms to verify behavior
- Metrics or dashboards to watch; expected healthy signals
- Failure signals and rollback or mitigation triggers
- Validation window and owner (if applicable)

If there is no production or runtime impact, state **No additional operational monitoring required** with a one-line reason.

## Key Principles

**Local workflow**

- **Never start on main/master** without explicit consent — prefer **using-git-worktrees** for isolated feature work when appropriate
- **Default: ask before acting** when instructions are unclear — do not guess
- **Stop on blocker** — don't force through
- **Checkpoint = trust boundary** — human reviews between batches (`manual` vs `auto` per task file)
- **Spec before quality** — verify correctness first, polish second (per-task quality gate)
- **Scope discipline** — GitNexus `detect_changes` after every task

**Execution themes**

- **Finish the feature** — ship complete work, not 80% done
- **Plan as guide** — implementation.md is a decision artifact; follow references and patterns in-repo
- **Test as you go** — `ctest` after meaningful changes, not only at the end
- **Quality built in** — format, lint, tests, and review tiers (inline vs local review workflow)

## Quality Checklist

Before PR or branch completion:

- [ ] Clarifying questions resolved at startup (or deferred items tracked)
- [ ] All batch tasks completed; task file / TodoWrite aligned
- [ ] `ctest --preset <preset-name>` passes
- [ ] `clang-format` on changed files; lint clean
- [ ] GitNexus `detect_changes` shows only expected scope
- [ ] Per-task quality gate satisfied (spec then code quality) where non-trivial
- [ ] UI changes: screenshots captured and linked where required
- [ ] PR description includes **Post-Deploy Monitoring & Validation** (or explicit no-impact line)
- [ ] Review done: inline self-review **or** local review workflow (`workflows:review` / reviewer agents)
- [ ] If publishing: use **publishing-pr** (or equivalent); conventional commits when incrementally committing

## Common Pitfalls

- **Analysis paralysis** — read implementation.md and execute; escalate scope via brainstorming / writing-impl-plans instead of stalling
- **Skipping clarifying questions** — ask at startup, not after building the wrong thing
- **Ignoring plan references** — load cited files and match conventions
- **Testing only at the end** — run `ctest` continuously per task
- **Forgetting checkpoints** — respect `checkpoint_policy` and task file progress
- **Skipping review** — every change gets review; depth varies (inline vs local review workflow)
- **Scope creep** — `detect_changes` catches unexpected files; stop if unexplained
- **80% done** — finish tasks and completion steps before switching context

## Upstream Borrowed Notes (Superpowers)

- If subagents are available, prefer subagent-driven execution for large plans.
- If subagents are unavailable, continue with this skill in single-agent mode and keep checkpoints strict.
