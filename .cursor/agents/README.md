# Custom Subagents

Project-level subagents for Cursor. Files in this directory are auto-discovered
and available to the main agent via the Task tool.

> **Format**: Each `.md` file uses YAML frontmatter (`name`, `description`, `model`, etc.)
> followed by the agent's system prompt. See [Cursor docs](https://cursor.com/docs/subagents).

## Output Contracts

Agents produce different output formats depending on their consumer:

| Consumer                        | Output format                                                     | Used by                                      |
| ------------------------------- | ----------------------------------------------------------------- | -------------------------------------------- |
| `reviewing-code` (Stage 5)     | JSON (`{reviewer, findings[], residual_risks[], testing_gaps[]}`) | Code Review agents (always-on + conditional) |
| `audit-protocol` (Stage 3/5/8) | Numbered list: `N. [pX] Title — description`                      | Code Review (Specialized) agents             |
| `brainstorming` / planning     | Markdown with `Research value: {high\|moderate\|low}` header      | Research agents                              |
| `document-review`              | Structured verdict (varies per reviewer)                          | Document Review agents                       |

## Code Review (Always-on)

These agents run on every code review regardless of diff content.

| Agent                        | Description                                                                                |
| ---------------------------- | ------------------------------------------------------------------------------------------ |
| `correctness-reviewer`       | Logic errors, edge cases, state bugs, error propagation, intent-vs-implementation mismatch |
| `maintainability-reviewer`   | Premature abstraction, dead code, coupling, naming clarity                                 |
| `testing-reviewer`           | Test coverage gaps, weak assertions, brittle tests, missing edge cases                     |
| `project-standards-reviewer` | AGENTS.md and CLAUDE.md compliance — naming, cross-platform portability                    |
| `agent-native-reviewer`      | Agent-native parity — any user action should be agent-accessible                           |

## Code Review (Conditional)

Selected by the main agent based on diff content and risk signals.

| Agent                           | Trigger condition                                                            |
| ------------------------------- | ---------------------------------------------------------------------------- |
| `adversarial-reviewer`          | Large diff (>=50 lines) or high-risk domain (auth, payments, data mutations) |
| `api-contract-reviewer`         | API routes, request/response types, serialization, versioning                |
| `cli-agent-readiness-reviewer`  | CLI source code — agent readiness and usability                              |
| `data-integrity-guardian`       | Database migrations, data models, persistent data, privacy compliance        |
| `data-migration-expert`         | Data migrations, backfills, column renames, schema changes                   |
| `data-migrations-reviewer`      | Migration files, schema changes, data transformations, backfill scripts      |
| `kieran-python-reviewer`        | Python code — Pythonic clarity, type hints, maintainability                  |
| `performance-reviewer`          | Loop-heavy transforms, caching, I/O paths                                   |
| `previous-comments-reviewer`    | PRs with existing review threads — checks if feedback was addressed          |
| `reliability-reviewer`          | Error handling, retries, timeouts, health checks                             |
| `schema-drift-detector`         | Proto schema changes — cross-references against PR's stated scope            |
| `security-reviewer`             | User input handling, permission checks                                       |

## Code Review (Specialized)

Deep-dive agents for specific review concerns, typically used in workflow audit stages.

| Agent                            | Focus                                                                     |
| -------------------------------- | ------------------------------------------------------------------------- |
| `architecture-strategist`        | Architectural patterns, design integrity, structural refactors            |
| `code-simplicity-reviewer`       | YAGNI violations, unnecessary complexity (final review pass)              |
| `design-implementation-reviewer` | Compare plan design intent against actual implementation to detect drift  |
| `pattern-recognition-specialist` | Design patterns, anti-patterns, naming conventions, duplication           |
| `performance-oracle`             | Deep performance: algorithmic complexity, memory, scalability             |
| `security-sentinel`              | Deep security: input validation, hardcoded secrets                        |

## Document Review

Review planning documents, specs, and design docs.

| Agent                               | Focus                                                                      |
| ----------------------------------- | -------------------------------------------------------------------------- |
| `document-reviewer`             | Clarity, completeness, specificity, YAGNI (lightweight gate)               |
| `adversarial-document-reviewer` | Challenges premises, surfaces unstated assumptions, stress-tests decisions |
| `coherence-reviewer`                | Internal consistency, contradictions, terminology drift, ambiguity         |
| `design-lens-reviewer`              | Missing design decisions — interaction states, user flows                  |
| `feasibility-reviewer`              | Architecture conflicts, dependency gaps, implementability                  |
| `product-lens-reviewer`             | Strategic consequences, goal-work misalignment, adoption impact            |
| `scope-guardian-reviewer`           | Scope creep, unjustified complexity, premature frameworks                  |
| `security-lens-reviewer`            | Plan-level security gaps — auth assumptions, data exposure                 |

## Research

Gather context and intelligence before implementation.

| Agent                        | Use when                                                                     |
| ---------------------------- | ---------------------------------------------------------------------------- |
| `best-practices-researcher`  | Need industry standards, community conventions, or implementation guidance   |
| `framework-docs-researcher`  | Need official docs, version-specific constraints, or implementation patterns |
| `git-history-analyzer`       | Trace code evolution, identify contributors, understand historical patterns  |
| `issue-intelligence-analyst` | Analyze GitHub issues for recurring themes, pain patterns, severity trends   |
| `learnings-researcher`       | Search past solutions for institutional knowledge and prevent repeated mistakes |
| `repo-research-analyst`      | Onboard to codebase — structure, conventions, implementation patterns        |

## Workflow

Task-specific automation agents.

| Agent                           | Use when                                                          |
| ------------------------------- | ----------------------------------------------------------------- |
| `bug-reproduction-validator`    | Validate bug reports — confirm reported behavior is an actual bug |
| `deployment-verification-agent` | Go/No-Go deployment checklists — build, rollback, monitoring      |
| `pr-comment-resolver`           | Resolve a single PR review thread — assess, fix, draft reply      |
| `spec-flow-analyzer`            | Analyze specs for user flow completeness and gap identification   |
