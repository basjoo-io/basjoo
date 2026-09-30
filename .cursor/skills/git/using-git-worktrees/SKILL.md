---
name: using-git-worktrees
description: >-
  Create isolated git worktrees for parallel development.
  Use when starting feature work, splitting PRs, working on a hotfix,
  创建工作树, 隔离工作区, git worktree, isolated workspace,
  or before executing implementation plans.
---

# Using Git Worktrees

## Overview

Git worktrees create isolated workspaces sharing the same repository, allowing work on multiple branches simultaneously without switching.

**Core principle:** Systematic directory selection + safety verification = reliable isolation.

**Announce at start:** "I'm using the using-git-worktrees skill to set up an isolated workspace."

**Create TodoWrite todos from this workflow:**

| id   | content                   | status  |
| ---- | ------------------------- | ------- |
| wt-1 | Select worktree directory | pending |
| wt-2 | Safety verification       | pending |
| wt-3 | Create worktree + branch  | pending |
| wt-4 | Verify baseline           | pending |

## When to Use

- Starting feature work that benefits from branch isolation
- Executing implementation plans with independent tasks
- Splitting a large PR into smaller, focused PRs
- Working on a hotfix while another feature is in progress

## When NOT to Use

- Single-file changes that don't need isolation
- Quick fixes on the current branch
- Changes that depend on uncommitted work in the current worktree

## Directory Selection Process

Follow this priority order:

### 1. Check Existing Directories

```bash
# Check in priority order
ls -d .worktrees 2>/dev/null     # Preferred (hidden)
ls -d worktrees 2>/dev/null      # Alternative
```

**If found:** Use that directory. If both exist, `.worktrees` wins.

### 2. Check AGENTS.md

```bash
grep -i "worktree.*director" AGENTS.md 2>/dev/null
```

**If a preference is specified:** Use it directly without asking.

### 3. Ask Your Human Partner

Use the `AskQuestion` tool:

- **title:** "Worktree Directory"
- **question prompt:** "No worktree directory found. Where should I create worktrees?"
- **options:**
  - ".worktrees/ (project-local, hidden)"
  - "~/.cursor/worktrees/ (global, shared across projects)"
  - "Custom path (I'll specify)"

## Safety Verification

### For Project-Local Directories (.worktrees or worktrees)

**MUST verify directory is ignored before creating worktree:**

```bash
# Check if the CHOSEN directory is ignored
# Replace <chosen_dir> with the actual directory name (.worktrees or worktrees)
git check-ignore -q <chosen_dir> 2>/dev/null
```

**If NOT ignored:**

Fix immediately:

1. Add to local git exclude (avoids committing a .gitignore change):
   ```bash
   GIT_DIR="$(git rev-parse --git-dir)"
   mkdir -p "${GIT_DIR}/info"
   grep -qxF '<chosen_dir>/' "${GIT_DIR}/info/exclude" 2>/dev/null \
     || echo '<chosen_dir>/' >> "${GIT_DIR}/info/exclude"
   ```
2. Verify: `git check-ignore -q <chosen_dir>` should return 0
3. Proceed with worktree creation

**Why critical:** Prevents accidentally committing worktree contents to repository.

### For Global Directory (~/.cursor/worktrees)

No gitignore verification needed - outside project entirely.

## Creation Steps

### 1. Detect Project Name

```bash
# Use git-common-dir for stability — git rev-parse --show-toplevel
# returns the worktree path when run inside a worktree, not the main repo.
project=$(basename "$(dirname "$(git rev-parse --git-common-dir)")")
```

### 2. Determine Branch Name

**Read** [branch-naming-convention.md](../branch-naming-convention.md) for the full specification.

Quick ref: `<your_name>_<describe_the_pr_change>` — e.g., `alice_refactor_localization`, `bob_feat_knn_library`.

### 3. Create Worktree

Set `LOCATION` to the directory chosen in the Directory Selection step above (e.g., `.worktrees`, `worktrees`, or `$HOME/.cursor/worktrees`):

```bash
LOCATION=".worktrees"  # result from Directory Selection

# Determine full path
case $LOCATION in
  .worktrees|worktrees)
    path="$LOCATION/$BRANCH_NAME"
    ;;
  "$HOME/.cursor/worktrees")
    path="$HOME/.cursor/worktrees/$project/$BRANCH_NAME"
    ;;
esac

# Create worktree with new branch
git worktree add "$path" -b "$BRANCH_NAME"
cd "$path"
```

### 4. Run Project Setup

Initialize the development environment in the new worktree:

```bash
# No additional tool setup needed for CMake projects
direnv allow                       # Activate environment
```

After creating the worktree, configure the build with `cmake --preset <preset-name>`.

### 5. Verify Clean Baseline

Run tests to ensure worktree starts clean. **Choose scope based on change type:**

- **Code changes:** Run relevant tests for the changed package:
  ```bash
  ctest --preset linux-gcc-x64    # Test the affected package
  ```
- **Documentation / config / skill files only:** Verify `git status` is clean — skip full test suite:
  ```bash
  git status
  ```

**If tests fail:** Report failures, then use the `AskQuestion` tool:

- **title:** "Baseline Tests Failing"
- **question prompt:** "{N} tests failing in the worktree baseline.\n\n{show failure summary}"
- **options:**
  - "Proceed anyway (I know about these failures)"
  - "Stop and investigate failures first"

**If tests pass (or skipped for non-code changes):** Report ready.

### 6. Report Location

```
Worktree ready at <full-path>
Branch: <branch-name>
Ready to implement <feature-name>
```

## Quick Reference

| Situation                    | Action                                          |
| ---------------------------- | ----------------------------------------------- |
| `.worktrees/` exists         | Use it (verify ignored)                         |
| `worktrees/` exists          | Use it (verify ignored)                         |
| Both exist                   | Use `.worktrees/`                               |
| Neither exists               | Check AGENTS.md → AskQuestion                   |
| Directory not ignored        | Add to local git exclude                        |
| Tests fail during baseline   | Report failures + ask                           |
| Non-code changes only        | Skip full test suite, verify `git status` clean |
| Setup after worktree created | `cmake --preset <preset-name>`        |

## Common Mistakes

### Skipping ignore verification

- **Problem:** Worktree contents get tracked, pollute git status
- **Fix:** Always use `git check-ignore` before creating project-local worktree

### Assuming directory location

- **Problem:** Creates inconsistency, violates project conventions
- **Fix:** Follow priority: existing > AGENTS.md > ask

### Proceeding with failing tests

- **Problem:** Can't distinguish new bugs from pre-existing issues
- **Fix:** Report failures, get explicit permission to proceed

### Skipping build configuration

- **Problem:** Tools not available in worktree, direnv fails
- **Fix:** Always run `cmake --preset <preset-name>` after creating worktree

### Running full test suite for non-code changes

- **Problem:** Wastes time in large monorepos when only docs/config changed
- **Fix:** Match test scope to change scope

## Example: Creating a Worktree for `bob_feat_git_skills`

1. Announce: "I'm using the using-git-worktrees skill to set up an isolated workspace."
2. Directory selection: `.worktrees/` not found → AGENTS.md has no preference → AskQuestion → `.worktrees/`
3. Safety: `git check-ignore -q .worktrees` fails → add to local exclude → verify
4. Create: `git worktree add .worktrees/bob_feat_git_skills -b bob_feat_git_skills origin/master`
5. Setup: `cmake --preset <preset-name>`
6. Verify: `git status` clean
7. Report: "Worktree ready at `<project-root>/.worktrees/bob_feat_git_skills`, branch `bob_feat_git_skills`"

## Red Flags

**Never:**

- Create worktree without verifying it's ignored (project-local)
- Skip baseline verification
- Proceed with failing tests without asking
- Assume directory location when ambiguous
- Skip AGENTS.md check

**Always:**

- Follow directory priority: existing > AGENTS.md > ask
- Verify directory is ignored for project-local
- Auto-detect and run project setup
- Match test scope to change type

## Cleanup

Handled by **finishing-a-development-branch** after merge/PR:

```bash
git worktree remove <path>
```

## Integration

**Called by:**

- **brainstorming** (Phase 4) - when design is approved and implementation follows
- **subagent-driven-development** - before executing any tasks
- **executing-plans** - before executing any tasks
- Any skill needing isolated workspace

**Pairs with:**

- **finishing-a-development-branch** - REQUIRED for cleanup after work complete
