---
name: writing-arch-plans
description: >
  Use when the user says "plan this", "create a plan", "write a tech plan",
  "how should we build", or when a brainstorm or design doc is ready for
  technical planning. Triggers: plan this, create a plan, write a tech plan,
  架构规划, arch plan, 技术方案.
argument-hint: "[design doc path, task id, or feature description]"
---

# Writing Architecture Plans

## Overview

Write architecture plans for an executor who has zero context. Handles **Stage 2 (arch_plan)** only — architecture-level decisions, not code.

**Note:** Use the current date from session context when dating plans and searching for recent documentation.

## When to Use

| Trigger                                                     | Action                                 |
| ----------------------------------------------------------- | -------------------------------------- |
| User says "plan this", "create a plan", "write a tech plan" | Proceed                                |
| Brainstorm or design doc ready for technical planning       | Proceed                                |
| Task status is `arch_plan`                                  | Proceed                                |
| User provides an existing architecture plan                 | Redirect to **writing-impl-plans** |

**Workflow bridge:** `brainstorming` → **writing-arch-plans** → `writing-impl-plans` → `executing-plans`

## Interaction Method

Prefer **AskQuestion** when clarifying choices (blocking; use a concise single-select when natural). On other platforms, use the blocking question tool if available (`AskUserQuestion` in Claude Code, `request_user_input` in Codex, `ask_user` in Gemini); otherwise present numbered options in chat and wait for the user's reply before continuing.

Ask **one question at a time**.

**Announce at start:** "I'm using the writing-arch-plans skill to create the architecture plan."

Create TodoWrite todos:

```
TodoWrite todos:
  - id: "ap-1", content: "Read design doc + validate input", status: "pending"
  - id: "ap-2", content: "Classify Plan Depth", status: "pending"
  - id: "ap-3", content: "Research How/Where (skip if design doc sufficient)", status: "pending"
  - id: "ap-4", content: "Resolve Planning Questions", status: "pending"
  - id: "ap-5", content: "Write architecture plan", status: "pending"
  - id: "ap-6", content: "Confidence Check", status: "pending"
  - id: "ap-7", content: "Self-check + dispatch document-reviewer agent", status: "pending"

Note: These `ap-` prefixes match the orchestrator's TodoWrite policy for Stage 2.
```

## Phase Detection

Read `docs/tasks/<task-id>.yaml`:

- `status == arch_plan` → proceed with this skill

**Fallback (no task file):** If no task file exists (standalone invocation), infer:

- User provides a design/brainstorm doc with no existing architecture plan under `docs/plans/` → proceed
- User provides an existing architecture plan → redirect to **writing-impl-plans**

When falling back, announce: "No task file found. Inferred: Architecture Phase because [reason]."

## Resume, Source, and Scope

If the user references an **existing plan** under `docs/plans/`:

- Read it.
- Confirm **update in place** versus **create a new file** before overwriting.
- If updating in place, preserve completed progress markers and edit only sections that are still relevant.

Task-driven inputs remain authoritative: `paths.design` from task file, plus any brainstorm or requirements document those paths imply.

## Validate Input

Read `paths.design` from task file (or user-provided design doc).

**Scope Check:** If the design document covers multiple independent subsystems that could each produce working, testable software on their own, suggest breaking into separate plans — one per subsystem. Flag via **AskQuestion**:

- title: "Scope Too Broad"
- prompt: "This design covers N independent subsystems: [list]. Recommend splitting into separate plans."
- options: "Split into separate plans", "Keep as one plan (I understand the risk)", "Let me update the design first"

If the design document is incomplete or ambiguous → **AskQuestion**:

- title: "Design Clarification Needed"
- prompt: list specific gaps, ask user to clarify before proceeding
- options: "Here's the clarification", "Use your best judgment", "Let me update the design first"

## Plan Depth Classification

Classify after Validate Input based on scope signals from the design doc:

| Depth           | Criteria                                           | Research Agents                                  | Confidence Check          |
| --------------- | -------------------------------------------------- | ------------------------------------------------ | ------------------------- |
| **Lightweight** | ≤5 files changed, single module, low risk          | 1–2 (`repo-research-analyst`, GitNexus `impact`) | Score-only (no deepening) |
| **Standard**    | Cross-module, moderate risk, 6–20 files            | 3–4 + conditional `spec-flow-analyzer`           | Run                       |
| **Deep**        | Cross-cutting, high risk, >20 files, architectural | All agents + `spec-flow-analyzer`                | Run                       |

Signal sources: file count from design doc, module count, risk assessment, external contract surfaces.

Announce: "Plan Depth: [Lightweight/Standard/Deep] because [reason]."

## Research

Focus: **How & Where** — module boundaries, dependencies, blast radius.
Design-level research (What & Why) was completed during brainstorming; do not repeat it.

Prepare a short **planning context summary** (one or two paragraphs) from the design doc: problem frame, requirements, key decisions. Pass it to research subagents as input.

Dispatch in **parallel** (count tied to Plan Depth):

- `repo-research-analyst` — scope: technology, architecture, patterns; include the planning context summary
- `learnings-researcher` — institutional learnings (`docs/solutions/`, prior art) using the same summary
- `framework-docs-researcher` — dependency library API reference, version constraints (only if design introduces new deps; Standard/Deep only)
- GitNexus `impact` — blast radius of planned changes
- GitNexus `context` — 360° symbol dependency view for modified modules (Standard/Deep only)

**Leverage repo-research-analyst output** for sharper decisions: pass exact framework versions to `framework-docs-researcher` when needed; skip redundant external research when local patterns are strong and low-risk.

**External Research judgment (R2.10):** If the work was treated as small/low-risk but research shows it touches: environment variables consumed by external systems, CI, or other repos; exported public APIs, CLI flags, or command contracts; CI/CD or deployment config; shared types/interfaces for downstream consumers; documentation referenced externally — bump to **Standard** depth. Announce: "Treating as Standard depth — touches [exported APIs / CI / shared contracts]."

**Flow and edge-case analysis (conditional):** For **Standard**- or **Deep**-scale work, or when user-flow completeness is still unclear after research, dispatch `spec-flow-analyzer` with the planning context summary and research findings. Use its output to tighten edge cases, state transitions, and requirements trace.

Skip research entirely if the design document already contains sufficient detail for all sections below.

## Resolve Planning Questions

Between Research and Write Plan:

1. Collect open questions from design doc's "Outstanding Questions > Deferred to planning"
2. Check if research results already answer them
3. For unresolved questions: present to user via AskQuestion (one at a time)
4. Record resolved answers → will go into plan's "Open Questions > Resolved During Planning"
5. Record still-unresolved items → will go into plan's "Open Questions > Deferred to Implementation"

## Output Structure (plan.md)

YAML frontmatter:

```yaml
---
title: "<Feature Name>"
type: feat | fix | refactor
status: draft
date: YYYY-MM-DD
origin: <path to design doc>
depth: Lightweight | Standard | Deep
deepened: true | false
---
```

Sections (depth-dependent):

| Section                                  | Lightweight | Standard   | Deep     |
| ---------------------------------------- | ----------- | ---------- | -------- | -------- | -------- | -------- |
| Goal                                     | Required    | Required   | Required |
| Problem Frame                            | —           | Required   | Required |
| Module Design                            | Required    | Required   | Required |
| Data Flow                                | —           | Required   | Required |
| Interface Definitions                    | Required    | Required   | Required |
| Dependency Analysis                      | —           | Required   | Required |
| Risk Points (`                           | Risk        | Mitigation | ` table) | Required | Required | Required |
| Requirements Trace (stable IDs: R1, R2…) | Required    | Required   | Required |
| Open Questions (Resolved / Deferred)     | —           | Required   | Required |
| System-Wide Impact                       | —           | —          | Required |
| High-Level Technical Design              | —           | Optional   | Required |
| Sources & References                     | —           | Optional   | Required |

**Section content guidance:**

- **System-Wide Impact** must include: interaction graph, error propagation paths, state lifecycle changes, API parity implications, integration coverage requirements, unchanged invariants.
- **High-Level Technical Design** must include media selection guidance: DSL/pseudo-code for algorithms, Mermaid diagrams for multi-component interactions, state diagrams for state-heavy flows, sequence diagrams for request/response chains.

## Confidence Check

After writing the plan, run a scoring pass on each section.

**Scoring:** For each section, assess a "gap score" (0–3):

- 0 = Thorough, no improvement needed
- 1 = Minor gaps, not worth deepening
- 2 = Moderate gaps, would benefit from targeted research
- 3 = Major gaps, critical for plan quality

**Threshold:** gap score ≥ 2 triggers deepening for that section.

**Lightweight exception:** Scoring pass always runs (for observability), but deepening is never triggered. Record `deepened: false` in YAML frontmatter.

**Deepening agent dispatch (Standard/Deep only):**

| Plan Section          | Agent to Dispatch                                               |
| --------------------- | --------------------------------------------------------------- |
| Module Design         | `repo-research-analyst` — check patterns, suggest restructuring |
| Data Flow             | `spec-flow-analyzer` — trace flow completeness                  |
| Interface Definitions | `framework-docs-researcher` — verify API compatibility          |
| Dependency Analysis   | GitNexus `impact` rerun with refined scope                      |
| Risk Points           | `adversarial-document-reviewer` — stress-test mitigations   |
| System-Wide Impact    | `architecture-strategist` — verify cross-system implications    |
| Open Questions        | `learnings-researcher` — search past solutions                  |

**Merge:** Agent output is merged into the section it targets. Set YAML frontmatter `deepened: true` if any section was deepened.

## Self-Check

**Verify** before saving — all must be true:

- Every module described with clear responsibility
- Data flow covers all input/output paths
- Interface definitions are concrete (not vague)
- Dependency analysis includes GitNexus blast radius
- Risk points have mitigations (table format)
- Requirements trace maps each design requirement to concrete modules/interfaces — not vague buckets
- Product or scope blockers are not conflated with technical deferrals (and vice versa)
- Where test scenarios or verification hooks are described, they cover applicable categories (happy path, edge cases, errors, integration) at proportionate depth

## Save

1. Write to `docs/plans/YYYY-MM-DD-NNN-<type>-<name>-plan.md`
   - Create `docs/plans/` if it does not exist.
   - For today's date, list existing `docs/plans/` files and assign the next **three-digit sequence** (`001`, `002`, …), zero-padded.
   - Keep `<name>` concise (3–5 words), kebab-case; `<type>` is `feat`, `fix`, or `refactor`.
2. Update task file: `paths.plan`
3. Dispatch `document-reviewer` agent as Task subagent on plan.md (pass absolute path + "architecture plan") — agent returns structured verdict only. Default: use [plan-document-reviewer-prompt.md](./plan-document-reviewer-prompt.md) as the Task prompt. Override only if the plan requires domain-specific review criteria not covered by the default prompt.
4. If document-reviewer returns FAIL: present findings, do NOT advance. Wait for user to approve revisions or accept.
5. Update `stages.arch_plan.document_review_passed: true`
6. If passed: advance `status: arch_audit`

Do NOT present execution handoff — the next step is arch_audit, not execution.

## Quick Reference

| Phase             | Key Action                         | Output                    |
| ----------------- | ---------------------------------- | ------------------------- |
| Validate Input    | Read design doc, scope check       | Confirmed input           |
| Plan Depth        | Classify Lightweight/Standard/Deep | Depth + research scope    |
| Research          | Parallel subagent dispatch         | Context for plan sections |
| Resolve Questions | Surface deferred questions         | Resolved/deferred list    |
| Write Plan        | Depth-dependent sections           | `plan.md`                 |
| Confidence Check  | Gap scoring (0–3 per section)      | Deepening if gap ≥ 2      |
| Self-Check + Save | Verify completeness, write file    | `docs/plans/` file        |

## Key Principles

- DRY, YAGNI
- Every file path verified with Glob
- Architecture = decisions, not code
- Confidence Check auto-strengthens before audit
