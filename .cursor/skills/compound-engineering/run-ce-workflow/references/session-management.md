# Session Management

The workflow is too large for one Cursor session. Context window grows with document reads, agent outputs, and code diffs. The task YAML bridges sessions.

## Gate = Session Boundary

Each gate is a natural session split point. After gate approval, assess context health (see below) — recommend a fresh chat only when 2+ exhaustion signals fire.

| Session | Stages                     | Context in                                                                                                     | Output                       |
| ------- | -------------------------- | -------------------------------------------------------------------------------------------------------------- | ---------------------------- |
| A       | 1 Brainstorm               | User requirement                                                                                               | `design.md` → Gate A         |
| B       | 2 + 3 Arch plan + audit    | `design.md`                                                                                                    | `plan.md` → Gate B           |
| C       | 4 + 5 Impl plan + audit    | `plan.md`                                                                                                      | `implementation.md` → Gate C |
| D       | 6 TDD                      | `implementation.md`                                                                                            | Failing tests → Gate D       |
| E       | 7 + 8 Execute + code audit | Stage 7: `implementation.md` + tests; Stage 8: `paths.plan` + `paths.implementation` + `git diff <base>..HEAD` | Code → Gate E                |
| F1      | 9 + 10 Ship + verify       | `git diff` + task YAML                                                                                         | PR + Gate F                  |
| F2      | 11 Compound                | Session transcript + task YAML                                                                                 | Retro + knowledge            |

## New Session Startup

Every new session reads:

1. `docs/tasks/<task-id>.yaml` — current stage, document paths, gate status
2. The stage-required inputs from the table below (some stages require more than one input, such as plan + git diff)

## Per-Stage Read Policy

**Scope:** This policy applies to the **orchestrator agent** only. Subagents dispatched via Task tool may read any files relevant to their task (source code, test files, config files).

| Stage        | Must read                                                                       | Do NOT read            |
| ------------ | ------------------------------------------------------------------------------- | ---------------------- |
| 1 Brainstorm | — (start fresh)                                                                 | —                      |
| 2 Arch Plan  | `paths.design`                                                                  | —                      |
| 3 Arch Audit | `paths.plan`                                                                    | `design.md`            |
| 4 Impl Plan  | `paths.plan`                                                                    | `design.md`            |
| 5 Impl Audit | `paths.implementation`                                                          | `design.md`, `plan.md` |
| 6 TDD        | `paths.implementation` (task list section)                                      | `design.md`, `plan.md` |
| 7 Execute    | `paths.implementation` (current task only)                                      | Everything else        |
| 8 Code Audit | `paths.plan` + `paths.implementation` (File Map only) + `git diff <base>..HEAD` | `design.md`            |
| 9 Ship       | `git diff <base>..HEAD`                                                         | All `.md`              |
| 10 Verify    | `paths.implementation` (acceptance criteria only)                               | `design.md`, `plan.md` |
| 11 Compound  | Session transcript                                                              | All `.md`              |

**Why "Do NOT read"?** Each document layer compresses the previous. Reading back creates redundancy and wastes context tokens.

**Subagent exception:** This policy applies to the orchestrator only. Subagents dispatched via Task tool may read any files relevant to their task (e.g., `design-implementation-reviewer` in Stage 8 receives both `paths.plan` and `paths.implementation`).

**Base branch:** Throughout this document, `<base>` means the result of `git merge-base HEAD main 2>/dev/null || git merge-base HEAD master 2>/dev/null`. Never hardcode `main`.

## Stage Transition Protocol

At every stage completion boundary (after updating task YAML), the orchestrator should evaluate session health, but should not over-interrupt execution.

### 1. Assess Context Health

Evaluate whether the current session can handle the next stage. Heuristics:

| Signal                           | Likely exhausted           | Likely sufficient |
| -------------------------------- | -------------------------- | ----------------- |
| Stages completed this session    | 2+ stages with audits      | 0–1 stage         |
| Subagent dispatches this session | 4+ round-trips             | 0–2 round-trips   |
| Documents read this session      | 2+ large docs (>200 lines) | 1 doc or none     |
| Audit loops this session         | Any re-audit iteration     | No audits yet     |
| Conversation turns               | 20+ user/assistant turns   | <10 turns         |

If **2+ signals** point to "likely exhausted", recommend switching session.

### 2. Generate Handoff

Always generate a handoff prompt, regardless of assessment. Write it to **both** a file **and the chat**.

**Primary UX:** The user must be able to **copy the continuation prompt from the chat** without opening `docs/tasks/<task-id>-handoff.md`. The file is for crash recovery and diffs; **never** tell the user that the handoff file is the only or preferred copy source.

**The handoff is a complete, self-contained prompt** that the user can paste verbatim into a new chat. It must include enough context for the next agent to start without reading this session.

1. Write to `docs/tasks/<task-id>-handoff.md` (overwrite each time):

```markdown
/run-ce-workflow <task-id>

## 上下文

Stage `<completed-stage>` 已通过 Gate <X>，继续 `<next-stage>`。

关键文档：

- 设计文档: `<paths.design>`
- 架构方案: `<paths.plan>`（如已产出）
- 实现计划: `<paths.implementation>`（如已产出）

本轮关键决策：

- <decision 1>
- <decision 2>

## 未决问题（下一阶段需关注）

- <any Outstanding Questions tagged "Resolve before planning" or open items>

## 下一步

请按 `/run-ce-workflow` 的 stage dispatch 表执行 `<next-stage>` 阶段。
```

2. **Present in chat (MANDATORY every stage completion):**

- Add a short heading, e.g. `### 可复制续跑提示`
- Immediately below, one **fenced code block** containing **exactly** the same markdown as written to `<task-id>-handoff.md` (verbatim), so the user can copy in one gesture. Prefer an outer fence with the `text` info string so inner markdown headers do not break rendering.
- Do **not** nest ambiguous ` ``` ` fences; if the inner content is markdown, still wrap the **whole** handoff as one copy-paste unit inside a single outer fence.

**Removed:** Older guidance that suppressed the chat block when auto-continuing in the same session. Session fatigue is handled by **AskQuestion** (see §3), not by hiding the prompt.

3. Append to `sessions[]` in task YAML:

```yaml
- started: <ISO timestamp when this session began>
  stages_covered: [<stages completed this session>]
  decisions: ["<key decision 1>", "<key decision 2>"]
```

### 3. User Prompt Policy

Default policy (recommended):

- At each stage completion, assess context budget first (MUST print the signal table — see main SKILL.md "Stage Transition").
- **Always** show the **可复制续跑提示** block in chat (§2). Then use **AskQuestion** for the next move — do **not** rely on the user typing free-form text (e.g. "执行 Task 1") as the only continuation path.
- **AskQuestion** menu (adjust labels to the real `<next_stage>`):
  1. **继续本会话** — orchestrator loads the next stage skill and proceeds now.
  2. **新开聊天继续** — user copies the already-shown block into a new chat; orchestrator **stops** after confirming.
  3. **暂停** — only update YAML if needed; no automatic stage start.

- If **2+ tired signals**: preface the AskQuestion with one line recommending option 2, but still offer option 1.
- Never prompt more than once for the same stage completion (same gate boundary).
- Never interrupt mid-stage for session-boundary decisions.

If user chooses **新开聊天继续**: the handoff block is already in the chat — confirm and **stop**. Do not reprint the block unless the user asks.

## Subagent Offloading

Delegate heavy work to subagents (Task tool) to keep the orchestrator context lean:

| Work type                    | Subagent strategy                                                                                    | Parent receives           |
| ---------------------------- | ---------------------------------------------------------------------------------------------------- | ------------------------- |
| **Research** (Stage 1, 2, 4) | One Task per research agent                                                                          | Summary paragraph         |
| **Review** (Stage 3, 5, 8)   | One Task per review agent                                                                            | Numbered findings list    |
| **Execution** (Stage 7)      | One Task per implementation task (`subagent-driven-development`; `best-of-n-runner` for retries) | Pass/fail + files changed |

The parent agent holds only: **task YAML + current document + subagent summaries**.

## Subagent Output Verification

Subagent outputs are advisory, not authoritative. The orchestrator MUST independently verify before acting:

| Check                          | How                                          |
| ------------------------------ | -------------------------------------------- |
| Factual claims                 | Cross-check against source files (Read/Grep) |
| Version numbers, counts, paths | Verify against actual filesystem state       |
| "This is wrong / missing"      | Read the cited location before accepting     |

If a subagent finding contradicts what you can verify, mark it `rejected` with reason. Never fix what isn't broken.
