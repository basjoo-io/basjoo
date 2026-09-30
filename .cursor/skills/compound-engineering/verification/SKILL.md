---
name: verification
description: >-
  Use when task status is verify (Stage 10) or when explicit verification
  is needed after code changes. Triggers: post-ship, verify stage, evidence collection.
---

# Verification

**Announce at start:** "I'm using the verification skill to verify the shipped changes."

## Overview

This skill provides a structured verification pass after shipped code changes: choose a method, collect evidence, evaluate results, and obtain Gate F approval. It ties task YAML (`stages.verify`) to integration tests, manual checks, or simulation so correctness is documented before advancing to compound status.

## Parameters

- `evidence:<path>` (optional): Path to a YAML file with pre-collected evidence. When provided, loads entries and presents a single confirmation AskQuestion instead of per-criterion interactive checks.

  Expected format:

  ```yaml
  evidence:
    - type: test_output
      description: "All unit tests pass"
      result: pass
      artifact: "out/build/<preset>/Testing/"
  ```

## When to Use

- Task file `status: verify` (Stage 10 of /run-ce-workflow)
- After shipping code changes that need structured evidence of correctness
- When the user explicitly requests verification of recent changes

Create TodoWrite todos from this workflow:

```
TodoWrite todos:
  - id: "vf-1", content: "Determine verification method", status: "pending"
  - id: "vf-2", content: "Execute verification checks", status: "pending"
  - id: "vf-3", content: "Evaluate evidence", status: "pending"
  - id: "vf-4", content: "Gate F approval", status: "pending"
```

## Workflow

## Step 1: Determine Verification Method

Read `stages.verify.verification_method` from task YAML.

If empty: use AskQuestion (default: **Integration test** if `paths.implementation` contains test targets; otherwise **Manual test**):

- title: "Verification Method"
- prompt: "How should we verify the shipped changes?"
- options:
  - "Integration test (recommended) — run ctest on integration targets"
  - "Manual test — I'll verify manually with a checklist"
  - "Simulation — run a simulation scenario"

Update task YAML: `stages.verify.verification_method: <choice>`

## Step 2: Execute Verification

### Pre-Collected Evidence (when `evidence:<path>` provided)

1. Read the YAML file at `<path>`
2. Validate each entry has `type`, `description`, `result`, `artifact`
3. Write all entries to `stages.verify.evidence[]`
4. Present summary via AskQuestion:
   - "Accept all N evidence entries (M pass, K fail)?" / "Review individually" / "Reject and re-collect"
5. If accepted: proceed to Step 3
6. If "Review individually": fall through to the method-specific flow below

### Manual Test

1. Read `paths.implementation` to extract acceptance criteria from each task
2. Present checklist via AskQuestion (one item at a time):
   - title: "Manual Verification"
   - prompt: "Verify: \<criterion\>"
   - options: "Pass" / "Fail (describe issue)" / "Skip (not applicable)"
3. For each response, append to `stages.verify.evidence[]`:
   ```yaml
   - type: manual_check
     description: <criterion>
     result: pass | fail
     artifact: <user's note if fail>
   ```

### Integration Test

1. Read `paths.implementation` to extract test targets
2. If no targets specified: AskQuestion for targets
3. Run: `ctest --preset <preset-name>`
4. Append to `stages.verify.evidence[]`:
   ```yaml
   - type: test_output
     description: "ctest --preset <preset-name>"
     result: pass | fail
     artifact: <test output summary>
   ```

### Simulation

**Note:** Simulation verification requires human execution — there is no fully automated protocol. The agent facilitates but the user runs the simulation.

1. Read `stages.verify.scenarios` from task YAML
2. If empty: AskQuestion for scenario name/path
3. Present simulation execution guidance with specific commands if available (e.g., `ctest --preset <preset-name> -R <scenario>`)
4. AskQuestion: "Please run the simulation and report the result" — options: "Pass" / "Fail (describe)" / "Cannot run (explain)"
5. Append to `stages.verify.evidence[]`:
   ```yaml
   - type: simulation
     description: <scenario name>
     result: pass | fail
     artifact: <log path or summary>
   ```

## Step 3: Evaluate Evidence

After all verification steps:

1. Check: all entries in `stages.verify.evidence[]` have `result: pass`?
2. If all pass: proceed to Gate F
3. If any fail: present failures, use AskQuestion:
   - "Fix and re-verify"
   - "Accept with known issues (document reason)"
   - "Abort"

## Step 4: Gate F

Compute gate artifacts:

- `verification_method_set`: `stages.verify.verification_method` is not empty
- `evidence_collected`: `stages.verify.evidence` has at least one entry
- `all_evidence_passing`: every entry has `result: pass`

Present via AskQuestion:

- Gate F results (artifact check results)
- Options: "Approve" / "Reject (explain why)" / "Needs revision"

On approve: advance `status: compound`

## Quick Reference

| Item                | Value                                                                                     |
| ------------------- | ----------------------------------------------------------------------------------------- |
| **Gate**            | Gate F — requires `verification_method_set`, `evidence_collected`, `all_evidence_passing` |
| **Default method**  | Integration test (if targets available) → Manual test (fallback)                          |
| **Evidence format** | `type`, `description`, `result` (pass/fail), `artifact`                                   |
| **On approve**      | Advance task to `status: compound`                                                        |

## Extensibility

Future verification methods can be added by:

1. Adding a new option to Step 1's AskQuestion
2. Adding a new subsection in Step 2
3. Evidence format stays the same (`type`, `description`, `result`, `artifact`)
