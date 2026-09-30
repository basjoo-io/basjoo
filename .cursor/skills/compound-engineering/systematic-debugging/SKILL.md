---
name: systematic-debugging
description: >-
  Use when encountering any bug, test failure, build failure, or unexpected
  behavior. Triggers: error, crash, test failure, build failure, unexpected output.
---

# Systematic Debugging

**Core principle:** ALWAYS find root cause before attempting fixes.

## Overview

This skill structures debugging into four phases so you establish root cause before changing code, then verify fixes with tests and tooling.

## When to Use

- Any bug, error, or crash during development or testing
- Test failures (unit, integration, simulation)
- Build failures with unclear cause
- Unexpected behavior that doesn't match specification

## When NOT to Use

- Known fix for a well-understood issue (apply directly)
- Configuration typos caught by linter

## References

- [root-cause-tracing.md](references/root-cause-tracing.md) — Detailed root cause tracing techniques
- [defense-in-depth.md](references/defense-in-depth.md) — Defense-in-depth debugging strategy
- [condition-based-waiting.md](references/condition-based-waiting.md) — Condition-based waiting patterns
- [condition-based-waiting-example.ts](references/condition-based-waiting-example.ts) — Example implementation
- [find-polluter.sh](references/find-polluter.sh) — Script to find test polluters

## Workflow

Create TodoWrite todos from this checklist at session start:

```
TodoWrite todos:
  - id: "debug-1", content: "Phase 1: Root Cause Investigation complete", status: "pending"
  - id: "debug-2", content: "Phase 2: Pattern Analysis complete", status: "pending"
  - id: "debug-3", content: "Phase 3: Hypothesis tested", status: "pending"
  - id: "debug-4", content: "Phase 4: Fix implemented + verified", status: "pending"
```

Cannot skip phases. Cannot attempt fixes before Phase 1 is complete.

## The Iron Law

```
NO FIXES WITHOUT ROOT CAUSE INVESTIGATION FIRST
```

If Phase 1 is not complete, fixes are forbidden.

## The Four Phases

Complete each phase before proceeding to the next.

### Phase 1: Root Cause Investigation

1. **Read** error messages, stack traces, and warnings completely. Note line numbers, file paths, error codes.

2. **Reproduce** the issue consistently. If not reproducible → gather more data, don't guess.

3. **Check** recent changes: `git diff`, recent commits, new dependencies, config changes, environmental differences.

4. **Gather evidence** at each component boundary: log data entering/exiting, verify environment/config propagation, check state at each layer. **Run** GitNexus `context` and `impact` on the affected symbol.

5. **Trace** data flow upstream: Where does the bad value originate? What called this with the bad value? Keep tracing until you find the source. Fix at source, not at symptom.

### Phase 2: Pattern Analysis

1. **Find working examples** — Locate similar working code in the codebase
2. **Compare against references** — Read reference implementation completely, don't skim
3. **Identify differences** — List every difference, however small
4. **Understand dependencies** — What other components, settings, environment does this need?

### Phase 3: Hypothesis and Testing

1. **Form single hypothesis** — "I think X is the root cause because Y"
2. **Test minimally** — Smallest possible change to test the hypothesis, one variable at a time
3. **Verify before continuing** — Worked → Phase 4. Didn't work → new hypothesis, don't pile fixes
4. **When you don't know** — Say so. Research more. Use the `AskQuestion` tool with concrete options, for example: continue investigating, narrow scope to a specific subsystem, or pause for human input.

### Phase 4: Implementation

1. **Create failing test case**
   - Use `test-driven-development` skill for proper failing test
   - `ctest --preset linux-gcc-x64 must fail for the expected reason

2. **Implement single fix**
   - Address the root cause identified
   - ONE change at a time
   - No "while I'm here" improvements

3. **Verify fix**
   - Test passes now?
   - No other tests broken? (`ctest --preset linux-gcc-x64`)
   - `clang-format` on changed files
   - GitNexus `detect_changes` — only expected files changed?

4. **If fix doesn't work**
   - Count: how many fixes have you tried?
   - If < 3: return to Phase 1, re-analyze with new information
   - **If >= 3: STOP — question the architecture (see below)** — use `AskQuestion` with options such as: continue investigating from Phase 1, narrow scope to a smaller repro, or pause for human input on architecture.

5. **If 3+ fixes failed: Question Architecture**

   Pattern indicating architectural problem:
   - Each fix reveals new coupling in a different place
   - Fixes require massive refactoring
   - Each fix creates new symptoms elsewhere

   STOP before more fixes. Use `AskQuestion` with concrete options (e.g. continue with a design discussion, narrow to one coupling point, or pause for human input), then discuss fundamentals with the user.

## Red Flags — STOP and Return to Phase 1

If you catch yourself thinking:

- "Quick fix for now, investigate later"
- "Just try changing X and see"
- "I don't fully understand but this might work"
- "Here are the main problems: [lists fixes without investigation]"
- Proposing solutions before tracing data flow
- "One more fix attempt" (when already tried 2+)

## Quick Reference

| Phase                 | Key Activities                                    | Success Criteria            |
| --------------------- | ------------------------------------------------- | --------------------------- |
| **1. Root Cause**     | Read errors, reproduce, check changes, trace flow | Understand WHAT and WHY     |
| **2. Pattern**        | Find working examples, compare                    | Identify differences        |
| **3. Hypothesis**     | Form theory, test minimally                       | Confirmed or new hypothesis |
| **4. Implementation** | Create test, fix, verify                          | Bug resolved, tests pass    |
