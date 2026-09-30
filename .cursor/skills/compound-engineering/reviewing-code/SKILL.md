---
name: reviewing-code
description: "Use when reviewing code changes before creating a PR. Triggers: code review, 代码审查, review my code, review this branch, review this PR."
argument-hint: "[blank to review current branch, or provide PR link]"
---

# Code Review

Reviews code changes using dynamically selected reviewer personas. Spawns parallel sub-agents that return structured JSON, then merges and deduplicates findings into a single report.

## Overview

This skill orchestrates 15 reviewer personas across three tiers (always-on, cross-cutting conditional, stack-specific conditional) plus project-specific agents. Each reviewer runs as a Task subagent, returns structured JSON findings, and the orchestrator merges/deduplicates into one report with actionable routing.

## When to Use

- Before creating a PR
- After completing a task during iterative implementation
- When feedback is needed on any code changes
- As a read-only or autofix review step inside larger workflows (e.g., `run-ce-workflow` Stage 8)

## Argument Parsing

Parse `$ARGUMENTS` for the following optional tokens. Strip each recognized token before interpreting the remainder as the PR number, GitHub URL, or branch name.

| Token               | Example                                       | Effect                                           |
| ------------------- | --------------------------------------------- | ------------------------------------------------ |
| `mode:autofix`      | `mode:autofix`                                | Select autofix mode                              |
| `mode:report-only`  | `mode:report-only`                            | Select report-only mode                          |
| `mode:headless`     | `mode:headless`                               | Select headless mode for programmatic callers    |
| `base:<sha-or-ref>` | `base:abc1234` or `base:origin/main`          | Skip scope detection — use as diff base directly |
| `plan:<path>`       | `plan:docs/plans/YYYY-MM-DD-feat-foo-plan.md` | Load this plan for requirements verification     |

All tokens are optional. **Conflicting mode flags:** If multiple mode tokens appear, stop and report the conflict without dispatching agents.

## Mode Detection

| Mode                      | When               | Behavior                                                                   |
| ------------------------- | ------------------ | -------------------------------------------------------------------------- |
| **Interactive** (default) | No mode token      | Review, apply safe_auto fixes, present findings, ask for policy decisions  |
| **Autofix**               | `mode:autofix`     | No user interaction. Apply only safe_auto fixes, write run artifact        |
| **Report-only**           | `mode:report-only` | Read-only. Report only, no edits, no artifacts                             |
| **Headless**              | `mode:headless`    | Programmatic. Apply safe_auto (single pass), return structured text output |

### Autofix mode rules

- Skip all user questions. Apply only `safe_auto` fixes (orchestrator inline).
- Never commit, push, or create a PR from autofix mode. Parent workflows own those decisions.

### Report-only mode rules

- Skip all user questions. Never edit files.
- Safe for parallel read-only verification.
- Do not switch the shared checkout.

### Headless mode rules

- Skip all user questions. If no diff scope can be determined, emit error and stop.
- Apply only `safe_auto` fixes in a single pass (no re-review rounds).
- Return all non-auto findings as structured text output.
- End with "Review complete" as the terminal signal.
- Never commit, push, or create a PR.

## Severity Scale

All reviewers use P0-P3:

| Level  | Meaning                                                            | Action                 |
| ------ | ------------------------------------------------------------------ | ---------------------- |
| **P0** | Critical breakage, exploitable vulnerability, data loss/corruption | Must fix before merge  |
| **P1** | High-impact defect likely hit in normal usage, breaking contract   | Should fix             |
| **P2** | Moderate issue with meaningful downside                            | Fix if straightforward |
| **P3** | Low-impact, narrow scope, minor improvement                        | User's discretion      |

## Action Routing

Severity answers **urgency**. Routing answers **who acts next**.

| `autofix_class` | Default owner                         | Meaning                                                              |
| --------------- | ------------------------------------- | -------------------------------------------------------------------- |
| `safe_auto`     | orchestrator (inline fix)             | Local, deterministic fix applied by the orchestrator itself          |
| `gated_auto`    | orchestrator (with approval) or human | Concrete fix exists, but changes behavior/contracts — needs approval |
| `manual`        | human                                 | Requires design decisions or cross-cutting changes                   |
| `advisory`      | human or release                      | Report-only: learnings, rollout notes, residual risk                 |

Routing rules:

- Synthesis owns the final route. Persona-provided routing is input, not the last word.
- Choose the more conservative route on disagreement.
- Only `safe_auto` findings enter the fixer queue automatically (orchestrator applies fixes inline).

## Reviewers

15 reviewer personas in layered conditionals, plus project-specific agents. See the persona catalog for the full list: [persona-catalog.md](./references/persona-catalog.md)

**Always-on (every review):**

| subagent_type                | Focus                                                     |
| ---------------------------- | --------------------------------------------------------- |
| `correctness-reviewer`       | Logic errors, edge cases, state bugs, error propagation   |
| `testing-reviewer`           | Coverage gaps, weak assertions, brittle tests             |
| `maintainability-reviewer`   | Coupling, complexity, naming, dead code, abstraction debt |
| `project-standards-reviewer` | AGENTS.md and `.cursor/rules/` compliance                 |
| `agent-native-reviewer`      | Verify new features are agent-accessible                  |
| `learnings-researcher`       | Search docs/solutions/ for past issues related to this PR |

**Cross-cutting conditional (selected per diff):**

| subagent_type                  | Select when diff touches...                                   |
| ------------------------------ | ------------------------------------------------------------- |
| `security-reviewer`            | Auth, public endpoints, user input, permissions               |
| `performance-reviewer`         | DB queries, hot loops, data transforms, caching, async        |
| `api-contract-reviewer`        | Routes, proto services, type signatures, versioning           |
| `data-migrations-reviewer`     | Migrations, schema changes, backfills                         |
| `reliability-reviewer`         | Error handling, retries, timeouts, background jobs            |
| `adversarial-reviewer`         | Diff >=50 changed lines, or auth/data mutations/external APIs |
| `cli-agent-readiness-reviewer` | CLI command definitions, argument parsing                     |
| `previous-comments-reviewer`   | Reviewing a PR with existing review comments                  |

**Stack-specific conditional:**

| subagent_type                   | Select when diff touches...                        |
| ------------------------------- | -------------------------------------------------- |
| `kieran-python-reviewer`        | Python modules, endpoints, scripts, services       |
| `kieran-typescript-reviewer`    | TypeScript components, services, hooks, utilities  |
| `julik-frontend-races-reviewer` | Stimulus/Turbo, DOM events, timers, async UI flows |

**conditional (migration-specific):**

| subagent_type                   | Select when diff includes...    |
| ------------------------------- | ------------------------------- |
| `schema-drift-detector`         | Proto schema changes            |
| `deployment-verification-agent` | Deployment-sensitive components |

## Protected Artifacts

These paths must never be flagged for deletion by any reviewer:

- `docs/brainstorms/`
- `docs/plans/`
- `docs/solutions/`

## How to Run

### Stage 1: Determine scope

Compute the diff range, file list, and diff.

**If `base:` argument is provided (fast path):**

```bash
BASE_ARG="{base_arg}"
BASE=$(git merge-base HEAD "$BASE_ARG" 2>/dev/null) || BASE="$BASE_ARG"
echo "BASE:$BASE" && echo "FILES:" && git diff --name-only $BASE && echo "DIFF:" && git diff -U10 $BASE && echo "UNTRACKED:" && git ls-files --others --exclude-standard
```

**If a PR number or GitHub URL is provided:**

Verify worktree is clean. For `mode:report-only` or `mode:headless`, do not switch the checkout — emit an error instead. Otherwise check out the PR branch:

```bash
git status --porcelain
gh pr checkout <number-or-url>
gh pr view <number-or-url> --json title,body,baseRefName,headRefName,url
```

Then compute a local diff against the PR's base branch (substituting values from PR metadata). If the base ref cannot be resolved, stop — a PR review without the base branch is incomplete.

**If a branch name is provided:**

Check out the branch, then detect the review base and compute the merge-base using `references/resolve-base.sh`:

```bash
RESOLVE_OUT=$(bash references/resolve-base.sh) || { echo "ERROR: resolve-base.sh failed"; exit 1; }
BASE=$(echo "$RESOLVE_OUT" | sed 's/^BASE://')
echo "BASE:$BASE" && echo "FILES:" && git diff --name-only $BASE && echo "DIFF:" && git diff -U10 $BASE && echo "UNTRACKED:" && git ls-files --others --exclude-standard
```

**If no argument (standalone on current branch):**

Same as branch mode but skip the checkout step. Use [resolve-base.sh](./references/resolve-base.sh) to find the base.

**Untracked file handling:** Inspect the `UNTRACKED:` list. Untracked files are outside review scope. If non-empty, inform the user which files are excluded. In headless/autofix mode, proceed with tracked changes only and note exclusions in Coverage.

### Stage 2: Intent discovery

Understand what the change is trying to accomplish.

- **PR mode:** Use PR title, body, and linked issues from `gh pr view`.
- **Branch/standalone:** Run `git log --oneline ${BASE}..HEAD`.

Write a 2-3 line intent summary:

```
Intent: Simplify tax calculation by replacing the multi-tier rate lookup
with a flat-rate computation. Must not regress edge cases in tax-exempt handling.
```

Pass this to every reviewer in their spawn prompt.

**When intent is ambiguous:**

- **Interactive mode:** Ask one question via AskQuestion: "What is the primary goal of these changes?"
- **Other modes:** Infer intent conservatively. Note uncertainty in Coverage.

### Stage 2b: Plan discovery (requirements verification)

Locate the plan document for Stage 6 requirements verification. Check in priority order:

1. **`plan:` argument.** Use directly.
2. **PR body.** Scan for `docs/plans/*.md` paths.
3. **Auto-discover.** Extract keywords from branch name, glob `docs/plans/*`, match.

Record `plan_source: explicit` (high confidence) or `plan_source: inferred` (lower).

### Stage 3: Select reviewers

Read the diff and file list from Stage 1. The 4 always-on personas and 2 always-on agents are automatic. For each conditional persona in [persona-catalog.md](./references/persona-catalog.md), decide whether the diff warrants it.

`previous-comments` is **PR-only** — skip for standalone branch reviews.

Announce the team before spawning:

```
Review team:
- correctness (always)
- testing (always)
- maintainability (always)
- project-standards (always)
- agent-native-reviewer (always)
- learnings-researcher (always)
- security -- new endpoint accepts user-provided redirect URL
- kieran-python -- Python service refactored in perception/
```

### Stage 3b: Discover project standards paths

Before spawning, find paths of all `**/AGENTS.md` files and Cursor rules directories (`.cursor/rules/`) relevant to the changed files. Filter to those whose directory is an ancestor of at least one changed file. Pass the path list to the `project-standards` persona in a `<standards-paths>` block.

### Stage 4: Spawn sub-agents

#### Model tiering

Use `model: "fast"` for all persona sub-agents. The orchestrator stays on the default model for intent discovery, reviewer selection, and synthesis.

#### Spawning

Spawn each selected persona reviewer as a **parallel Task** using the subagent template from [subagent-template.md](./references/subagent-template.md). Each persona sub-agent receives:

1. Their persona file content (from `.cursor/agents/<name>.md`)
2. Shared diff-scope rules from [diff-scope.md](./references/diff-scope.md)
3. The JSON output contract from [findings-schema.json](./references/findings-schema.json)
4. PR metadata (title, body, URL) when reviewing a PR
5. Review context: intent summary, file list, diff
6. **For `project-standards` only:** standards paths from Stage 3b

Persona sub-agents are **read-only**: they review and return structured JSON. They do not edit files.

**always-on agents** (`agent-native-reviewer`, `learnings-researcher`) are dispatched as Task calls in parallel with persona agents. Give them the same review context. Their output is unstructured and synthesized separately in Stage 6.

**conditional agents** (`schema-drift-detector`, `deployment-verification-agent`) are dispatched as Task calls when applicable. Pass the same review context plus the applicability reason.

**Error handling:** If an agent fails or times out, proceed with findings from agents that completed. Note the failed agent in Coverage.

### Stage 5: Merge findings

Convert multiple reviewer JSON payloads into one deduplicated, confidence-gated finding set.

1. **Validate.** Check each output against the schema. Drop malformed findings. Record drop count.
2. **Confidence gate.** Suppress findings below 0.60 confidence. Exception: P0 at 0.50+ survive.
3. **Deduplicate.** Fingerprint: `normalize(file) + line_bucket(line, ±3) + normalize(title)`. Merge: keep highest severity, strongest evidence, union evidence, note which reviewers flagged it.
4. **Cross-reviewer agreement.** When 2+ reviewers flag the same issue, boost confidence by 0.10 (capped at 1.0).
5. **Separate pre-existing.** Pull out findings with `pre_existing: true`.
6. **Resolve disagreements.** Record severity/routing disagreements in evidence.
7. **Normalize routing.** Set final `autofix_class`, `owner`, `requires_verification`. Conservative on disagreement.
8. **Partition the work:**
   - Fixer queue: only `safe_auto` (orchestrator applies inline)
   - Residual actionable queue: unresolved `gated_auto` or `manual`
   - Report-only queue: `advisory` findings
9. **Sort.** By severity (P0 first) → confidence (desc) → file path → line number.
10. **Collect coverage.** Union residual_risks and testing_gaps across reviewers.
11. **Preserve project agent artifacts.** Keep learnings, agent-native, schema-drift, deployment-verification outputs alongside merged findings.

### Stage 6: Synthesize and present

Assemble the final report using **pipe-delimited markdown tables** from [review-output-template.md](./references/review-output-template.md).

1. **Header.** Scope, intent, mode, reviewer team with per-conditional justifications.
2. **Findings.** Tables grouped by severity (`### P0 -- Critical`, etc.). Omit empty levels.
3. **Requirements Completeness.** Include only when a plan was found. For `plan_source: explicit`, flag unaddressed requirements as P1/manual. For `plan_source: inferred`, flag as P3/advisory.
4. **Applied Fixes.** Include only if a fix phase ran.
5. **Residual Actionable Work.** Include when unresolved actionable findings exist.
6. **Pre-existing.** Separate section, does not count toward verdict.
7. **Learnings & Past Solutions.** Surface learnings-researcher results.
8. **Agent-Native Gaps.** Surface agent-native-reviewer results. Omit if no gaps.
9. **Schema Drift Check.** If schema-drift-detector ran, summarize.
10. **Deployment Notes.** If deployment-verification-agent ran, surface key items.
11. **Coverage.** Suppressed count, residual risks, testing gaps, failed reviewers.
12. **Verdict.** Ready to merge / Ready with fixes / Not ready. Fix order if applicable.

**Format verification:** Before delivering, verify findings use pipe-delimited table rows. Never freeform text blocks.

## Quality Gates

Before delivering the review:

1. **Every finding is actionable.** No "consider" or "might want to" without a concrete fix.
2. **No false positives from skimming.** Verify surrounding code was actually read.
3. **Severity is calibrated.** Style nit is never P0. SQL injection is never P3.
4. **Line numbers are accurate.** Verify each cited line against file content.
5. **Protected artifacts respected.** Discard findings targeting docs/brainstorms/, docs/plans/, docs/solutions/.
6. **No linter overlap.** Don't flag things the project's linter/formatter would catch.

## After Review

### Interactive mode post-review

- Apply `safe_auto` findings automatically (orchestrator applies fixes inline — no separate fixer agent needed).
- Ask a policy question via AskQuestion only when `gated_auto` or `manual` findings remain:
  - When `gated_auto` present: "Review and approve specific gated fixes" / "Leave as residual work" / "Report only"
  - When only `manual`: "Leave as residual work" / "Report only"
- If no gated/manual remain after safe fixes, skip the question.
- Re-review only changed scope after fixes. Bound to `max_rounds: 2`.
- Offer next steps based on entry mode (Create PR / Push fixes / Exit).

### Autofix mode post-review

- Apply only `safe_auto` fixes (orchestrator inline).
- Never commit, push, or create a PR.

### Report-only and headless mode post-review

- Stop after the report. No edits, no commits.

## Fallback

If the platform doesn't support parallel sub-agents, run reviewers sequentially. Everything else stays the same.

## Quick Reference

| Phase         | Action                           | Output                       |
| ------------- | -------------------------------- | ---------------------------- |
| 1: Scope      | Compute diff range and file list | BASE, FILES, DIFF            |
| 2: Intent     | Understand change purpose        | Intent summary               |
| 2b: Plan      | Find related plan document       | Requirements list (optional) |
| 3: Select     | Choose reviewer personas         | Reviewer team                |
| 3b: Standards | Find AGENTS.md/rules paths       | Standards path list          |
| 4: Spawn      | Parallel Task dispatch           | Raw reviewer JSON            |
| 5: Merge      | Dedup, gate, normalize routing   | Merged finding set           |
| 6: Present    | Synthesize report                | Final verdict                |

**Always-on:** correctness-reviewer, testing-reviewer, maintainability-reviewer, project-standards-reviewer, agent-native-reviewer, learnings-researcher

**Conditional:** security-reviewer, performance-reviewer, api-contract-reviewer, data-migrations-reviewer, reliability-reviewer, adversarial-reviewer, cli-agent-readiness-reviewer, previous-comments-reviewer, kieran-python-reviewer, kieran-typescript-reviewer, julik-frontend-races-reviewer, schema-drift-detector, deployment-verification-agent

---

## Included References

- [persona-catalog.md](./references/persona-catalog.md)
- [subagent-template.md](./references/subagent-template.md)
- [diff-scope.md](./references/diff-scope.md)
- [findings-schema.json](./references/findings-schema.json)
- [review-output-template.md](./references/review-output-template.md)
- [resolve-base.sh](./references/resolve-base.sh)
