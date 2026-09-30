---
name: setup
description: >-
  Use when starting the compound engineering workflow for the first time or
  reconfiguring review agent selection. Triggers: first setup, reconfigure
  agents, compound-engineering.local.md.
disable-model-invocation: true # manual-only; prevents accidental model-triggered setup
argument-hint: "[--auto]"
---

# Compound Engineering Setup

## Overview

compound engineering setup selects which conditional review agents run per sub-module path during `/run-ce-workflow` audit stages and writes `compound-engineering.local.md` in the project root.

**Auto-invocation:** `/run-ce-workflow` automatically calls this skill with `--auto` when `compound-engineering.local.md` does not exist. Users no longer need to run `/setup` manually before starting a workflow.

## Parameters

- `--auto` (optional): Non-interactive mode. Skips all AskQuestion prompts — auto-detects stack, auto-configures defaults, writes config silently. Used by `run-ce-workflow` Startup Protocol. If config already exists, exits immediately (no-op).

## When to Use

- Auto-invoked by `/run-ce-workflow` Startup Protocol (no user action needed)
- Manual `/setup` to reconfigure review agent selection
- Manual `/setup` to regenerate `compound-engineering.local.md`

Configure which conditional review agents activate per sub-module path during /run-ce-workflow audit stages. Produces `compound-engineering.local.md`.

Create TodoWrite todos from this workflow:

```
TodoWrite todos:
  - id: "su-1", content: "Check existing config", status: "pending"
  - id: "su-2", content: "Detect project stack", status: "pending"
  - id: "su-3", content: "Build per-path agent map", status: "pending"
  - id: "su-4", content: "Write config + confirm", status: "pending"
```

⚠️ This file is per-developer configuration. Ensure it is listed in `.gitignore` before running setup.

## Step 1: Check Existing Config

Read `compound-engineering.local.md` in the project root.

**`--auto` mode:** If config exists → exit immediately (no-op, no output). If config does not exist → proceed to Step 2.

**Interactive mode (default):** If config exists, display current settings and use AskQuestion:

- "Reconfigure" — run setup again
- "View current" — show file, then stop
- "Cancel" — keep current settings

## Step 2: Detect Project Stack

Scan the repository for language indicators:

```bash
test -f CMakeLists.txt && echo "cmake-project"
find src/ -name "CMakeLists.txt" -maxdepth 2 2>/dev/null | head -1 && echo "cpp-modules"
find src/ -name "*.py" -maxdepth 2 2>/dev/null | head -1 && echo "python-services"
find . -name "*.proto" 2>/dev/null | head -1 && echo "protobuf"
```

## Step 3: Build Glob-to-Agent Overrides

Map detected stacks to `overrides` entries. Each entry uses a file glob (matched against the plan's changed files), a list of agents to add, and which audit stages they apply to.

Core agents per stage are hardcoded in [audit-protocol.md](../run-ce-workflow/references/audit-protocol.md) — do NOT repeat them here. This config only controls **conditional** agents.

| Detected Stack  | Glob                | Agents                                             | Stages                 |
| --------------- | ------------------- | -------------------------------------------------- | ---------------------- |
| python-services | `**/*.py`           | `kieran-python-reviewer`                           | impl_audit, code_audit |
| ml-python       | (merged into above) |                                                    |                        |
| typescript-web  | `**/*.ts`           | `kieran-typescript-reviewer`                       | impl_audit, code_audit |
| typescript-web  | `**/*.tsx`          | `kieran-typescript-reviewer`                       | impl_audit, code_audit |
| typescript-web  | `**/*.tsx`          | `julik-frontend-races-reviewer`                    | code_audit             |
| python-services | `**/migrations/**`  | `data-integrity-guardian`, `data-migration-expert` | impl_audit, code_audit |
| protobuf        | `**/*.proto`        | `schema-drift-detector`                            | code_audit             |
| any (always)    | `src/**`            | `design-implementation-reviewer`                   | code_audit             |

Merge semantics: `add_agents` are appended to the stage's core agent list. Core agents are never removed by local config.

## Step 4: Choose configuration path (AskQuestion)

**`--auto` mode:** Skip this step — always use auto-configure defaults, proceed directly to Step 5.

**Interactive mode (default):** Use `AskQuestion` before writing config. The default path is auto-configuration; customization is optional.

- "Auto-configure (Recommended)" — use detected defaults, skip to Step 5
- "Customize" — optional: choose which conditional agents to enable/disable per path group

If the user chooses Customize: present each path group with toggle options.

## Step 5: Write Config

Write `compound-engineering.local.md` using the `overrides` format expected by `run-ce-workflow`:

```yaml
# Per-path conditional agent overrides for /run-ce-workflow audit stages.
# Generated by /setup. Re-run to regenerate.
#
# Core agents per audit stage are hardcoded in audit-protocol.md.
# This file only adds conditional agents based on file-glob matching.
# Merge semantics: add_agents are appended; core agents are never removed.
overrides:
  - glob: "**/*.py"
    add_agents: [kieran-python-reviewer]
    stages: [impl_audit, code_audit]
  - glob: "**/*.ts"
    add_agents: [kieran-typescript-reviewer]
    stages: [impl_audit, code_audit]
  - glob: "**/*.tsx"
    add_agents: [kieran-typescript-reviewer]
    stages: [impl_audit, code_audit]
  - glob: "**/*.tsx"
    add_agents: [julik-frontend-races-reviewer]
    stages: [code_audit]
  - glob: "**/migrations/**"
    add_agents: [data-integrity-guardian, data-migration-expert]
    stages: [impl_audit, code_audit]
  - glob: "**/*.proto"
    add_agents: [schema-drift-detector]
    stages: [code_audit]
  - glob: "src/**"
    add_agents: [design-implementation-reviewer]
    stages: [code_audit]
```

Only include entries whose detected stack matched in Step 2. For example, if `protobuf` was not detected, omit the `**/*.proto` entry.

## Step 6: Confirm

**`--auto` mode:** Print one-line summary (e.g., "Auto-configured 6 agent overrides for 4 detected stacks.") and return to caller.

**Interactive mode (default):** Display summary: detected stacks, agent count per path, total agents configured.

## Quick Reference

| Item                   | Value                                                                                                                                                                                                  |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Output file**        | `compound-engineering.local.md` (project root, `.gitignore`d)                                                                                                                                      |
| **Format**             | `overrides` array with `glob` + `add_agents` + `stages` per entry (see `run-ce-workflow` Conditional Agents section)                                                                               |
| **Core agents**        | Hardcoded per stage in `audit-protocol.md` — NOT configured here                                                                                                                                       |
| **Conditional agents** | `kieran-python-reviewer`, `kieran-typescript-reviewer`, `julik-frontend-races-reviewer`, `data-integrity-guardian`, `data-migration-expert`, `schema-drift-detector`, `design-implementation-reviewer` |
| **Re-run**             | Invoke `/setup` → choose "Reconfigure"                                                                                                                                                             |
