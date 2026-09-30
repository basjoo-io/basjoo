---
name: finishing-a-development-branch
description: >
  Use when implementation is complete and all tests pass.
  Triggers: "ship it", "发布", "merge branch", "open PR", "discard branch", Stage 9.
---

# Finishing a Development Branch

## When to Use

- Implementation is complete and all tests pass
- Stage 9 (ship) of `run-ce-workflow`
- Branch work is done and an integration decision is needed

Verify tests → Present options → Execute choice → Clean up.

Create TodoWrite todos from this workflow:

```
TodoWrite todos:
  - id: "sh-1", content: "Verify all tests pass", status: "pending"
  - id: "sh-2", content: "Determine base branch", status: "pending"
  - id: "sh-3", content: "Present integration options", status: "pending"
  - id: "sh-4", content: "Execute chosen option", status: "pending"

Note: These `sh-` prefixes match the orchestrator's TodoWrite policy for Stage 9.
```

## Step 1: Verify Tests

```bash
ctest --preset linux-gcc-x64
clang-format -i $(git diff --name-only HEAD)
```

If tests fail: stop. Fix before proceeding.
If tests pass: continue to Step 2.

## Step 2: Determine Base Branch

```bash
git merge-base HEAD main 2>/dev/null || git merge-base HEAD master 2>/dev/null
```

Or ask: "This branch split from main — is that correct?"

## Step 3: Present Options

> **When in Stage 9 (ship):** Option 2 (PR) is the expected default path.

Use AskQuestion with exactly these 4 options:

- title: "Implementation Complete"
- prompt: "All tests pass. What would you like to do?"
- options:
  - "Merge back to {base-branch} locally"
  - "Push and create a Pull Request"
  - "Keep as-is / Skip PR (I'll handle it later)"
  - "Discard this work"

### Option 1: Merge Locally

```bash
git checkout <base-branch>
git pull
git merge <feature-branch>
ctest --preset linux-gcc-x64
git branch -d <feature-branch>
```

Then run worktree cleanup (see below).

### Option 2: Push and Create PR

```bash
git push -u origin <feature-branch>
gh pr create --title "<title>" --body "$(cat <<'EOF'
## Summary
<2-3 bullets>

## Test Plan
- [ ] <verification steps>
EOF
)"
```

Then run worktree cleanup (see below).

### Option 3: Keep As-Is / Skip PR

Report: "Keeping branch `<name>`. Ready when you are."

### Option 4: Discard

Confirm first via AskQuestion:

- title: "Confirm Discard"
- prompt: "This will permanently delete branch {name} and all commits. Are you sure?"
- options: "Yes, permanently discard" / "No, go back to options"

Only proceed if confirmed:

```bash
git checkout <base-branch>
git branch -D <feature-branch>
```

## Task File Integration

When invoked within `run-ce-workflow` (Stage 9: ship):

- **Option 2 (PR)** — expected path. After PR creation: set `stages.ship.pr: <url>`, wait for CI, set `stages.ship.ci: passed|failed`, advance `status: verify`.
- **Option 3 (Keep as-is / Skip PR)** — set `stages.ship.pr: skipped`, `stages.ship.skip_reason: <user reason>`, advance `status: verify`.
- **Option 1 (Merge locally)** — set `stages.ship.pr: local_merge`, advance `status: verify`.
- **Option 4 (Discard)** — set `stages.ship.pr: discarded`, do NOT advance.

When invoked standalone (from `executing-plans`):

- All 4 options available
- No task file update needed

## Worktree Cleanup

If `stages.tdd.worktree_path` is set in task YAML (or the current directory is inside a worktree):

```bash
# Return to main worktree first
cd "$(git worktree list --porcelain | head -2 | tail -1 | sed 's/worktree //')"
# Remove the feature worktree
git worktree remove <worktree-path>
```

Skip if no worktree was used (branch was created in the main working directory).

## Quick Reference

| Gate            | Condition                          |
| --------------- | ---------------------------------- |
| Tests           | Must pass before proceeding        |
| Discard         | Requires explicit confirmation     |
| Worktree        | Auto-cleanup if worktree was used  |
| Stage 9 default | Option 2 (PR) is the expected path |

## Never

- Proceed with failing tests
- Merge without verifying tests on the result
- Delete work without confirmation
- Force-push without explicit request

## Upstream Borrowed Checks (Superpowers)

- Before presenting integration options, tests must pass; otherwise stop and ask for fix direction.
- After merge/PR/keep/discard selection, always run explicit cleanup confirmation for worktree/branch state.
