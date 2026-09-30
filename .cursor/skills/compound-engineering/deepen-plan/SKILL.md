---
name: deepen-plan
description: >-
  Enhance an existing plan with parallel research and review agents to add
  depth, best practices, and implementation details. Use when a plan needs
  more detail, when audit finds plan insufficiency, or when the user says
  "deepen plan", "enhance plan", 深化计划, 增强计划, 补全计划.
argument-hint: "[path to plan file]"
---

# Deepen Plan

**Announce at start:** "I'm using the deepen-plan skill to enhance this plan with research."

**Note: The current year is 2026.** Use this when searching for recent documentation and best practices.

## Overview

**Core principle:** Research first, original content preserved always.

This skill takes an existing plan and enriches each section by dispatching parallel research and review agents. The result is a deeply grounded, production-ready plan with concrete implementation details, best practices, and edge case coverage.

**NEVER write implementation code.** This skill only researches and enhances plan documents.

## When to Use

- A plan exists but lacks depth, best practices, or implementation details
- Audit stage (Stage 5 in `run-ce-workflow`) finds "plan insufficiency"
- Your human partner says "deepen plan", "enhance plan", "补全计划"

**When NOT to use:**

- No plan file exists yet — use `writing-arch-plans` or `writing-impl-plans` first
- The plan is already execution-ready and the user wants to implement — use `executing-plans`

## The Process

Create TodoWrite todos from this workflow:

```
TodoWrite todos:
  - id: "dp-1", content: "Phase 1: Locate and parse plan file", status: "in_progress"
  - id: "dp-2", content: "Phase 2: Dispatch research and review agents", status: "pending"
  - id: "dp-3", content: "Phase 3: Synthesize findings", status: "pending"
  - id: "dp-4", content: "Phase 4: Enhance plan and write file", status: "pending"
```

### Phase 1: Locate and Parse Plan

#### 1.1 Resolve plan file path

If `argument-hint` was provided, use it. Otherwise:

1. Check for recent plans: look in `docs/plans/` and `docs/brainstorms/`
2. Ask: "Which plan would you like to deepen? Provide the path."

<HARD-GATE>
Do NOT proceed to Phase 2 until you have a valid, readable plan file.
If the file does not exist, STOP and ask the user for a correct path.
</HARD-GATE>

#### 1.2 Parse plan structure

Read the plan file. Identify its major sections and create a section manifest:

```
Section 1: [Title] - [What to research for this section]
Section 2: [Title] - [What to research for this section]
...
```

Present the manifest to the user for confirmation before dispatching agents.

### Phase 2: Dispatch Research and Review Agents

Dispatch ALL of the following agent categories in PARALLEL. Use the Task tool for each.

#### 2.1 Per-section research agents

For each section in the manifest, dispatch an `explore` subagent:

```
Task explore: "Research best practices, patterns, and real-world examples for: [section topic].
Return concrete, actionable recommendations with references."
```

Use WebSearch for current best practices (2024-2026).

#### 2.2 Skill-matched agents

Discover available skills from plugin and global paths. For each skill whose description matches the plan content, dispatch a `generalPurpose` subagent to apply that skill's lens to the relevant plan section.

#### 2.3 Learnings and solutions

Check `docs/solutions/` for documented past learnings. For each relevant learning, dispatch a subagent to extract applicable insights.

#### 2.4 Review agents

Discover review agents from `.cursor/agents/` (all agents are flat in this directory). Dispatch each as a Task with the appropriate `subagent_type`.

**MUST skip workflow agents** (`bug-reproduction-validator`, `pr-comment-resolver`, `spec-flow-analyzer`, `lint`) and **design agents** (`design-iterator`, `figma-design-sync`) — workflow agents are orchestrators not reviewers, and design agents operate on screenshots/visual comparisons not document text.

### Phase 3: Synthesize Findings

Collect outputs from all dispatched agents, then:

1. **Deduplicate** — Merge similar recommendations from multiple agents
2. **Prioritize** — Rank by impact (high-value improvements first)
3. **Flag conflicts** — Surface contradictory advice for human review
4. **Group** — Organize findings by plan section

### Phase 4: Enhance Plan and Write

#### 4.1 Enhance each section

For each plan section, append research insights using the format from [enhancement-template.md](references/enhancement-template.md). MUST preserve all original content unchanged.

#### 4.2 Add enhancement summary

Add an Enhancement Summary block at the top of the plan. See [enhancement-template.md](references/enhancement-template.md) for the exact format.

#### 4.3 Quality checks

Before writing the file, verify all items in the quality checklist from [enhancement-template.md](references/enhancement-template.md).

#### 4.4 Write the enhanced plan

Write the enhanced plan to the original file. If the user prefers a separate file, add `-deepened` suffix to the filename.

## Post-Enhancement Options

After writing the enhanced plan, present via `AskQuestion`:

| Option                      | Description                                        |
| --------------------------- | -------------------------------------------------- |
| **View diff** (recommended) | Show what was added/changed                        |
| **Run audit**               | Dispatch review agents on the enhanced plan        |
| **Execute plan**            | Load `executing-plans` to begin implementation |
| **Deepen further**          | Run another round on specific sections             |
| **Revert**                  | Restore original plan from git                     |

## Stage 5 Integration (run-ce-workflow)

When dispatched from `run-ce-workflow` Stage 5 audit:

- The `agent-native-reviewer` identifies "plan insufficiency" (vague code, missing paths, incomplete commands, pseudocode, cyclic dependencies, non-machine-evaluable verify criteria)
- This skill is the fix action for those findings — enhance the specific insufficient sections rather than making direct code edits
- After enhancement, return control to the audit loop

## Reference Files

- [enhancement-template.md](references/enhancement-template.md) — Output format templates and quality checklist
