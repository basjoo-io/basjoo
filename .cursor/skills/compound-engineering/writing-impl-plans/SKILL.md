---
name: writing-impl-plans
description: >
  Use when the user says "plan the implementation", "write impl plan", "break
  this down into tasks", or when an architecture plan is ready for implementation
  planning. Triggers: plan the implementation, write impl plan, break this down,
  实现计划, 拆分任务, impl_plan.
argument-hint: "[architecture plan path, task id, or feature description]"
---

# Writing Implementation Plans

**Note:** Use the current date from session context when dating plans and searching for recent documentation.

## When to Use

- An architecture plan exists and is ready for implementation planning
- The user says "plan the implementation", "write impl plan", or "break this down into tasks"
- Task file status is `impl_plan`

**When NOT to use:** When no architecture plan exists yet (use **writing-arch-plans** first) or when the user needs brainstorming.

## Overview

Write implementation plans for an executor who has zero context. This skill handles **Stage 4 (impl_plan)** only — agent-executable implementation steps with TDD and complete code.

**Workflow bridge:** `brainstorming` defines **what** to build. **writing-arch-plans** defines **how** (architecture). **writing-impl-plans** defines agent-executable implementation steps. **executing-plans** carries out the implementation tasks. This skill writes `paths.implementation` and hands off to execution.

**Philosophy:** Upstream ce:plan stresses "decisions, not code." This skill intentionally diverges because executors may have **zero context**: the implementation artifact MUST remain **agent-executable** with TDD steps, **full code**, build commands, and commit steps.

## Interaction Method

Prefer **AskQuestion** when clarifying choices (blocking; use a concise single-select when natural). On other platforms, use the blocking question tool if available (`AskUserQuestion` in Claude Code, `request_user_input` in Codex, `ask_user` in Gemini); otherwise present numbered options in chat and wait for the user's reply before continuing.

Ask **one question at a time**.

**Announce at start:** "I'm using the writing-impl-plans skill to create the implementation plan."

Create TodoWrite todos:

```
TodoWrite todos:
  - id: "ip-1", content: "Read architecture plan + validate input", status: "pending"
  - id: "ip-2", content: "Research Exact Code (skip if arch plan sufficient)", status: "pending"
  - id: "ip-3", content: "Resolve Deferred Questions", status: "pending"
  - id: "ip-4", content: "Write implementation plan (all tasks)", status: "pending"
  - id: "ip-5", content: "Self-check + dispatch document-reviewer agent", status: "pending"
  - id: "ip-6", content: "Execution handoff", status: "pending"

Note: These `ip-` prefixes match the orchestrator's TodoWrite policy for Stage 4.
```

## Phase Detection

Read `docs/tasks/<task-id>.yaml`:

- `status == impl_plan` → proceed with this skill

**Fallback (no task file):** If no task file exists (standalone invocation), infer:

- User provides an existing architecture plan → proceed
- User provides a design doc that already contains architecture-level detail (module design, interface definitions, scope boundaries) → proceed (skip architecture)
- User provides a raw design doc with no architecture → redirect to **writing-arch-plans**

When falling back, announce: "No task file found. Inferred: Implementation Phase because [reason]."

## Resume, Source, and Scope

If the user references an **existing plan** under `docs/plans/`:

- Read it.
- Confirm **update in place** versus **create a new file** before overwriting.
- If updating in place, preserve completed progress markers and edit only sections that are still relevant.

Task-driven inputs remain authoritative: `paths.plan` (architecture plan) and `paths.design` (rare, only if needed).

## Validate Input

Read `paths.plan` from task file (or user-provided architecture plan).

**Scope Check:** If the architecture plan covers multiple independent subsystems, suggest splitting into separate implementation plans — one per subsystem. Each plan should produce working, testable software on its own.

If the architecture plan is incomplete or ambiguous → **AskQuestion**:

- title: "Architecture Clarification Needed"
- prompt: list specific gaps, ask user to clarify before proceeding
- options: "Here's the clarification", "Use your best judgment", "Let me update the plan first"

**Inherit Plan Depth** from arch plan's YAML frontmatter `depth` field. Announce: "Inherited Plan Depth: [Lightweight/Standard/Deep]."

## Research

Focus: **Exact Code** — file paths, API signatures, version constraints.
Architecture-level research (module boundaries, blast radius) was completed in the architecture plan; do not repeat it.

Prepare a short **planning context summary** from the architecture plan (and design doc only if needed).

Dispatch in **parallel**:

- `repo-research-analyst` — scope: patterns, concrete file targets, sequencing cues; include the summary
- `learnings-researcher` — institutional learnings for the areas being changed
- `framework-docs-researcher` — exact API signatures, version constraints (only if architecture plan introduces new deps)
- GitNexus `context` — 360° symbol dependency view for implementation targets

**Depth bump — external contract surfaces:** If tasks touch exported APIs, CI/config consumed outside the repo, env vars, or shared downstream contracts, ensure steps explicitly cover verification, parity, and rollback where relevant; escalate thoroughness accordingly.

**Flow analysis (conditional):** For Standard/Deep-scale implementation scope, or unclear control flow and handoffs across layers, run `spec-flow-analyzer` with the summary and findings before finalizing tasks.

Skip research entirely if the architecture plan already provides sufficient code-level detail.

## Resolve Deferred Questions

Between Research and Write Plan:

1. Read arch plan's "Open Questions > Deferred to Implementation"
2. Check if implementation research answers them
3. For unresolved questions: present to user via AskQuestion
4. Record resolutions inline within relevant tasks

## Output Structure (implementation.md)

Each task must be complete enough for any agent to execute with zero context.
Each step is one action (2–5 minutes). Show complete code — not vague instructions.
For modifications, show only changed lines with enough context to locate them.

**Before defining tasks**, map out all files that will be created or modified. This is where decomposition decisions get locked in. Design units with clear boundaries — each file should have one clear responsibility.

YAML frontmatter:

```yaml
---
title: "<Feature Name>"
type: feat | fix | refactor
status: draft
date: YYYY-MM-DD
origin: <path to architecture plan>
depth: <inherited from arch plan>
---
```

````markdown
# [Feature] Implementation Plan

> **For AI Agent:** REQUIRED — Use the `executing-plans` skill (superpowers:executing-plans) to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One sentence.
**Architecture:** 2-3 sentences about approach.

---

## File Map

| File                         | Action | Responsibility       |
| ---------------------------- | ------ | -------------------- |
| `exact/path/to/new_file.h`   | Create | Brief responsibility |
| `exact/path/to/existing.cc`  | Modify | What changes and why |
| `exact/path/to/test_file.cc` | Create | What it tests        |

---

## Task 1: [Component Name]

**Files:**

- Create: `exact/path/to/file.ext`
- Modify: `exact/path/to/existing.ext:123-145`
- Test: `exact/path/to/test_file.ext`

- [ ] **Step 1: Write the failing test**

```language
// complete test code
```

- [ ] **Step 2: Run test — verify FAIL**

Run: `ctest --preset linux-gcc-x64
Expected: FAIL with "symbol not defined"

- [ ] **Step 3: Write minimal implementation**

```language
// complete implementation code
```

- [ ] **Step 4: Run test — verify PASS**

Run: `ctest --preset linux-gcc-x64
Expected: PASS

- [ ] **Step 5: Format and commit**

```bash
clang-format -i path/to/files
git add path/to/files
git commit -m "feat: add component"
```

## Task 2: ...
````

### Optional Plan Completeness Sections

- `## Alternative Approaches Considered` — why alternatives were rejected
- `## Success Metrics` — measurable outcomes for validation

### Task Ordering Strategy

- **New feature (greenfield)** → bottom-up: data layer → logic → interface → integration
- **Refactoring** → inside-out: extract → migrate callers → delete old code
- **Bugfix** → reproduce → fix → regression test

### Task Requirements

- **Exact file paths** — verify with Glob before including
- **Complete code** — not vague instructions like "add validation"
- **Nested code blocks** — When file content contains markdown fences, use a higher backtick count for the outer fence (e.g., ```for outer when inner uses`). Never use zero-width spaces as escape.
- **Large data files** — If a task creates a file >100 lines of structured data (YAML, JSON, config), write the full content as a separate file in `docs/plans/assets/` and reference it from the task: "Write content from `assets/<name>.yaml`".
- **Build commands** — cmake, ctest, clang-format
- **Dependencies** — ordered by dependency (prerequisite tasks first)
- **Verification** — machine-evaluable criteria

## Self-Check

**Verify** before saving — all must be true:

- Every file path verified with Glob
- No task depends on a later task
- Each task has test step (Step 1–2) and verification step (Step 4)
- Commit messages follow conventional commits
- Requirements or architecture bullets trace to tasks/modules where applicable (no orphan scope)
- Product or scope blockers are not conflated with technical deferrals (and vice versa)
- Test scenarios per task cover applicable categories (happy path, edge cases, error/failure paths, integration across layers) at proportionate depth

**Placeholder scan** — these are plan failures; search and eliminate all instances:

- "TBD", "TODO", "implement later", "fill in details"
- "Add appropriate error handling" / "add validation" / "handle edge cases"
- "Write tests for the above" (without actual test code)
- "Similar to Task N" (repeat the code — the executor may read tasks out of order)
- Steps that describe what to do without showing how (code steps must have code blocks)
- References to types, functions, or methods not defined in any task

**Type/name consistency** — verify across all tasks:

- Types, method signatures, and property names used in later tasks match definitions in earlier tasks
- A function called `ClearLayers()` in Task 3 but `ClearFullLayers()` in Task 7 is a bug — fix inline

## Save

1. Write to `docs/plans/YYYY-MM-DD-NNN-<type>-<name>-implementation.md` (same `NNN` sequencing rules as architecture plans)
2. Update task file: `paths.implementation`
3. Dispatch `document-reviewer` agent as Task subagent on implementation.md (pass absolute path + "implementation plan") — agent returns structured verdict only. Optionally shape the Task with [plan-document-reviewer-prompt.md](./plan-document-reviewer-prompt.md).
4. If document-reviewer returns FAIL: present findings, do NOT advance. Wait for user to approve revisions or accept.
5. Update `stages.impl_plan.document_review_passed: true`
6. If passed: advance `status: impl_audit`

## Execution Handoff

After saving the implementation plan, present execution options via AskQuestion:

- title: "Plan Complete"
- prompt: "Plan saved to `docs/plans/{filename}.md`. How would you like to proceed?"
- options:
  - "Subagent-Driven (recommended) — dispatch a fresh subagent per task, review between tasks"
  - "Inline Execution — execute tasks in this session using executing-plans"
  - "Save only — execute in a separate session"
  - "Revise the plan"

If **Subagent-Driven** chosen: load `subagent-driven-development` skill — fresh subagent per task + two-stage review.
If **Inline Execution** chosen: load `executing-plans` skill — batch execution with checkpoints.

If within `/run-ce-workflow`: skip handoff — the workflow manages state transitions.

## Key Principles

- DRY, YAGNI, TDD
- Every file path verified with Glob
- No task depends on a later task
- Each task has test and verification steps
- Implementation = agent-executable code, not decisions
