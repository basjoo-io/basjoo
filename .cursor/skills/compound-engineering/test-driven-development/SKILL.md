---
name: test-driven-development
description: >-
  Use when implementing any feature, fixing any bug, or when the task file
  status is tdd. Triggers: new feature, bug fix, TDD stage, test-first.
---

# Test-Driven Development (TDD)

**Core principle:** If you didn't watch the test fail, you don't know if it tests the right thing.

## When to Use

- Implementing any new function, method, or behavior
- Fixing any bug (write a failing test reproducing it first)
- Task file `status: tdd` (the TDD stage of `/run-ce-workflow`)

## When NOT to Use

- Config-only changes with no testable logic
- Documentation-only changes

## References

- [testing-anti-patterns.md](references/testing-anti-patterns.md) — Common TDD anti-patterns and fixes

## Workflow

Create TodoWrite todos from this checklist at session start:

```
TodoWrite todos:
  - id: "td-1", content: "Every new function/method has a test", status: "pending"
  - id: "td-2", content: "Watched each test fail before implementing", status: "pending"
  - id: "td-3", content: "Each test failed for expected reason", status: "pending"
  - id: "td-4", content: "Wrote minimal code to pass each test", status: "pending"
  - id: "td-5", content: "All tests pass", status: "pending"
  - id: "td-6", content: "Output pristine (no errors, warnings)", status: "pending"
  - id: "td-7", content: "Tests use real code (mocks only if unavoidable)", status: "pending"
  - id: "td-8", content: "Edge cases and errors covered", status: "pending"

Note: These `td-` prefixes match the orchestrator's TodoWrite policy for Stage 6.
```

Cannot mark work complete until all items are checked off.

For multi-test sessions, add per-test cycle todos:

- `"RED: {test name} — write failing test"`
- `"GREEN: {test name} — minimal code to pass"`
- `"REFACTOR: {test name} — clean up"`

## Gate Check: Before Writing Any Production Code

```
BEFORE writing or modifying production code:
  Ask: "Does a failing test exist for this change?"

  IF no failing test exists:
    STOP — Write the test first

  IF production code already exists without a test:
    DELETE the untested code
    START OVER with a failing test

  No exceptions. No "keeping as reference." Delete means delete.
```

Use AskQuestion if the user requests an exception (throwaway prototype, generated code, config file).

## The Cycle: RED → GREEN → REFACTOR

### RED — Write One Failing Test

Write a minimal test for one behavior:

- One behavior per test
- Clear name describing expected behavior
- Real code, no mocks unless unavoidable

Run the test:

```bash
ctest --preset linux-gcc-x64
```

Replace with the actual ctest target name from the task or plan.

Confirm the test fails correctly:

- Fails (not errors/crashes)
- Failure message matches expectation
- Fails because the feature is missing, not due to typos

If test passes immediately: testing existing behavior — fix the test.
If test errors/crashes: fix the error, re-run until it fails correctly.

### GREEN — Write Minimal Code to Pass

Write the simplest code that makes the test pass. Nothing more.

Run tests again:

```bash
ctest --preset linux-gcc-x64
```

Replace with the actual ctest target name from the task or plan.

Confirm:

- The new test passes
- All existing tests still pass
- Output is clean (no errors, no warnings)

If test still fails: fix the code, not the test.
If other tests broke: fix them now before continuing.

### REFACTOR — Clean Up (Tests Must Stay Green)

After green only:

- Remove duplication
- Improve names
- Extract helpers

Run tests after each refactoring change. Stay green.

Format:

```bash
clang-format -i path/to/files
```

### Repeat

Write the next failing test for the next behavior. Return to RED.

## Task File Integration

When invoked by `/run-ce-workflow` for the TDD stage (task file status: `tdd`):

1. Read `paths.implementation` from task file
2. Extract test specifications from each task
3. Write all tests (RED phase only — all tests should fail)
4. Update task file: `stages.tdd.test_count`, `stages.tdd.test_files`
5. Verify: all tests compile and fail as expected
6. Update task file: `stages.tdd.all_tests_red: true`
7. Present Gate D artifacts via AskQuestion

The GREEN phase happens in the implementation-execution stage (`executing-plans`), not here.

## When Mocking Is Acceptable

- **External service (network, DB, filesystem)** → Mock the boundary
- **Slow operation (>1s) blocking test feedback** → Mock the slow part only
- **Everything else** → Use real code

## When Stuck

| Problem                | Action                                                    |
| ---------------------- | --------------------------------------------------------- |
| Don't know how to test | Write the wished-for API first, then the assertion        |
| Test too complicated   | Design too complicated — simplify the interface           |
| Must mock everything   | Code too coupled — refactor with dependency injection     |
| Test setup too large   | Extract test helpers; still complex → simplify the design |

## Bug Found During Development

Write a failing test reproducing the bug. Follow the RED-GREEN-REFACTOR cycle. The test proves the fix works and prevents regression.

Never fix bugs without a failing test first.

⚠️ Production code without a prior failing test is not TDD. No exceptions without user's explicit permission.
