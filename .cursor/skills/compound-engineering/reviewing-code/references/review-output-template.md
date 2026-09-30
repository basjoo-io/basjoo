# Code Review Output Template

Use this **exact format** when presenting synthesized review findings. Findings are grouped by severity, not by reviewer.

**IMPORTANT:** Use pipe-delimited markdown tables (`| col | col |`). Do NOT use ASCII box-drawing characters.

## Example

```markdown
## Code Review Results

**Scope:** merge-base with the review base branch -> working tree (14 files, 342 lines)
**Intent:** Add order export endpoint with CSV and JSON format support
**Mode:** interactive

**Reviewers:** correctness, testing, maintainability, security, api-contract

- security -- new public endpoint accepts user-provided format parameter
- api-contract -- new /api/orders/export route with response schema

### P0 -- Critical

| #   | File                      | Issue                                                      | Reviewer | Confidence | Route                               |
| --- | ------------------------- | ---------------------------------------------------------- | -------- | ---------- | ----------------------------------- |
| 1   | `orders_controller.cc:42` | User-supplied ID in account lookup without ownership check | security | 0.92       | `gated_auto -> downstream-resolver` |

### P1 -- High

| #   | File                   | Issue                                                          | Reviewer                  | Confidence | Route                           |
| --- | ---------------------- | -------------------------------------------------------------- | ------------------------- | ---------- | ------------------------------- |
| 2   | `export_service.cc:87` | Loads all orders into memory -- unbounded for large accounts   | performance               | 0.85       | `safe_auto -> review-fixer`     |
| 3   | `export_service.cc:91` | No pagination -- response size grows linearly with order count | api-contract, performance | 0.80       | `manual -> downstream-resolver` |

### P2 -- Moderate

| #   | File                   | Issue                                            | Reviewer    | Confidence | Route                       |
| --- | ---------------------- | ------------------------------------------------ | ----------- | ---------- | --------------------------- |
| 4   | `export_service.cc:45` | Missing error handling for serialization failure | correctness | 0.75       | `safe_auto -> review-fixer` |

### P3 -- Low

| #   | File                  | Issue                                                                 | Reviewer        | Confidence | Route               |
| --- | --------------------- | --------------------------------------------------------------------- | --------------- | ---------- | ------------------- |
| 5   | `export_helper.py:12` | Format detection could use early return instead of nested conditional | maintainability | 0.70       | `advisory -> human` |

### Applied Fixes

- `safe_auto`: Added bounded export pagination guard and serialization failure test coverage in this run

### Residual Actionable Work

| #   | File                      | Issue                                            | Route                               | Next Step                                         |
| --- | ------------------------- | ------------------------------------------------ | ----------------------------------- | ------------------------------------------------- |
| 1   | `orders_controller.cc:42` | Ownership check missing on export lookup         | `gated_auto -> downstream-resolver` | Requires explicit approval before behavior change |
| 2   | `export_service.cc:91`    | Pagination contract needs a broader API decision | `manual -> downstream-resolver`     | Contract and client impact need resolution        |

### Pre-existing Issues

| #   | File                      | Issue                                       | Reviewer    |
| --- | ------------------------- | ------------------------------------------- | ----------- |
| 1   | `orders_controller.cc:12` | Broad catch masking failed permission check | correctness |

### Learnings & Past Solutions

- [Known Pattern] `docs/solutions/export-pagination.md` -- previous export pagination fix applies

### Agent-Native Gaps

- New export endpoint has no CLI/agent equivalent -- agent users cannot trigger exports

### Schema Drift Check

- Clean: proto schema changes match the scope

### Deployment Notes

- Pre-deploy: capture baseline counts before enabling the backfill
- Rollback: keep the old path available until validated

### Coverage

- Suppressed: 2 findings below 0.60 confidence
- Residual risks: No rate limiting on export endpoint
- Testing gaps: No test for concurrent export requests

---

> **Verdict:** Ready with fixes
>
> **Reasoning:** 1 critical auth bypass must be fixed. The memory/pagination issues (P1) should be addressed for production safety.
>
> **Fix order:** P0 auth bypass -> P1 memory/pagination -> P2 error handling if straightforward
```

## Formatting Rules

- **Pipe-delimited markdown tables** for findings -- never ASCII box-drawing characters
- **Severity-grouped sections** -- `### P0 -- Critical`, `### P1 -- High`, `### P2 -- Moderate`, `### P3 -- Low`. Omit empty levels.
- **Always include file:line location** for code review issues
- **Reviewer column** shows which persona(s) flagged the issue. Multiple = cross-reviewer agreement.
- **Confidence column** shows the finding's confidence score
- **Route column** shows the synthesized handling decision as `autofix_class -> owner`
- **Header includes** scope, intent, and reviewer team with per-conditional justifications
- **Mode line** -- include `interactive`, `autofix`, `report-only`, or `headless`
- **Applied Fixes section** -- include only when a fix phase ran
- **Residual Actionable Work section** -- include only when unresolved actionable findings exist
- **Pre-existing section** -- separate table, no confidence column
- **Summary uses blockquotes** for verdict, reasoning, and fix order
- **Horizontal rule** (`---`) separates findings from verdict
- Omit any section with zero items

## Headless Mode Format

In `mode:headless`, replace the interactive table report with a structured text envelope. Key differences:

- No pipe-delimited tables. Findings use `[severity][autofix_class -> owner] File: <file:line> -- <title>` format with indented evidence lines.
- Findings grouped by autofix_class instead of severity.
- Verdict in header (top of output) for programmatic callers.
- `[needs-verification]` marker on findings where `requires_verification: true`.
- Completion signal: "Review complete" as the final line.
