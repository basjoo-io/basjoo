---
name: fixing-rebase-conflict
description: >-
  Resolve git rebase conflicts through deep analysis, commit history tracing,
  and merge recommendations. Use when your human partner says "解决冲突", "rebase 冲突",
  "fix conflict", or explicitly references @fixing-rebase-conflict or
  /fixing-rebase-conflict.
---

# PR Rebase Conflict Resolution

**Core principle:** Understand intent before resolving — trace commit history to preserve both sides' goals.

**Announce at start:** "I'm using the fixing-rebase-conflict skill to resolve rebase conflicts."

**Activation:** When your human partner asks to resolve rebase conflicts (e.g., "解决冲突", "rebase 冲突", "fix conflict"), explicitly references `@fixing-rebase-conflict` or `/fixing-rebase-conflict`, or is called by another skill. Do NOT auto-trigger on bare "rebase" without conflict context.

⚠️ **Before any rebase continue or conflict resolution**: present summary → ask your human partner for confirmation → execute only after approval.

## When to Use

- Rebase in progress with unmerged files
- Dirty working tree with conflict markers
- Your human partner explicitly says "解决冲突", "fix conflict", or references this skill

## When NOT to Use

- Bare "rebase" without conflicts — let git handle it
- Merge conflicts (not rebase) — different workflow
- Conflicts already resolved, just need `git rebase --continue`

## Workflow

**Create TodoWrite todos from these steps:**

0. Pre-check working tree state
1. Gather conflict information
2. Analyze each conflict
3. Present summary + details
4. Mode select (auto / interactive)
5. Collect decisions or auto-resolve
6. Execute resolutions
7. Continue rebase
8. Cleanup + final summary

---

## Step 0: Pre-check Working Tree State

**Run** `git status` and branch to the matching scenario:

| Condition                              | Action                                    |
| -------------------------------------- | ----------------------------------------- |
| Rebase in progress + unmerged files    | → Step 1                                  |
| Rebase in progress + no unmerged files | → `GIT_EDITOR=true git rebase --continue` |
| Dirty tree + conflict markers found    | → Step 1                                  |
| Dirty tree + no conflict markers       | → Ask your human partner to commit/stash first |
| Clean tree                             | → Confirm with your human partner, then start rebase |

### Scenario A: Rebase in Progress

If `git status` shows "rebase in progress" with unmerged files, proceed to **Step 1**.

### Scenario B: Dirty Working Tree (No Rebase)

**Run** `git grep -l "<<<<<<< " 2>/dev/null || echo "No conflict markers found"`

- **Markers found** → Proceed to **Step 1**
- **No markers** → Tell your human partner to commit or stash changes, then re-run rebase

### Scenario C: Clean Working Tree

**Use AskQuestion** to confirm before starting rebase:

```json
AskQuestion({
  "title": "Ready to Start Rebase",
  "questions": [{
    "id": "rebase_confirm",
    "prompt": "Branch: <branch_name>\nTarget: origin/master\nCommits ahead: <N>\n\nThis will run: git fetch origin master && git rebase origin/master",
    "options": [
      {"id": "yes", "label": "Yes, start rebase"},
      {"id": "no", "label": "No, cancel"}
    ]
  }]
})
```

**Run** `git fetch origin master && git rebase origin/master` after approval.

- Succeeds → Report success with `git log --oneline -5`
- Conflicts → Proceed to **Step 1**

## Step 1: Gather Conflict Information

**Run** these commands in parallel:

```bash
git status
git log -1 --format="%h %s" REBASE_HEAD
git diff --name-only --diff-filter=U
git diff
```

## Step 2: Analyze Each Conflict

For each conflicting file:

1. **Run** `git diff <file>` to see conflicting sections
2. **Run** `git log --oneline -5 -- <file>` for our branch's changes
3. **Run** `git log --oneline origin/master -5 -- <file>` for master's changes
4. **Read** the conflicting functions and **trace** call relationships if needed
5. **Classify** the conflict type — see [conflict-types.md](conflict-types.md)
6. **CMakeLists.txt conflict check: When resolving conflicts in CMakeLists.txt files, verify that no duplicate target definitions exist.

## Step 3: Present Conflicts

**Report** summary first (summary table), then per-file details (our intent, their intent, root cause, recommended resolution). Use templates from [templates.md](templates.md).

## Step 3.5: Mode Selection

**Use AskQuestion**:

```json
AskQuestion({
  "title": "Execution Mode",
  "questions": [{
    "id": "exec_mode",
    "prompt": "How would you like to proceed?",
    "options": [
      {"id": "auto", "label": "Auto mode - resolve all automatically, show diff when done"},
      {"id": "interactive", "label": "Interactive - confirm each resolution individually"},
      {"id": "abort", "label": "Abort rebase"}
    ]
  }]
})
```

**Auto mode** behavior:

1. Apply learned strategies for known conflict types
2. Default to "accept HEAD" for ambiguous cases
3. Fix dependent code for API changes
4. Clean up orphaned files
5. Show complete diff summary when done

**Interactive mode** → proceed to Step 4.

### Permission Escalation

| Your human partner says    | Meaning                                               |
| -------------------------- | ----------------------------------------------------- |
| "lgtm" / "approve all"     | Approve current batch, still pause at next checkpoint |
| "auto" / "full delegation" | Switch to auto mode for all remaining conflicts       |
| "continue"                 | Proceed to next commit, may still ask for decisions   |

## Step 4: Batch Decision (Interactive Mode)

**Use AskQuestion** after displaying recommendations:

```json
AskQuestion({
  "title": "Decision Required",
  "questions": [{
    "id": "batch_decision",
    "prompt": "Review the conflict recommendations above.",
    "options": [
      {"id": "approve_all", "label": "Approve all recommendations"},
      {"id": "modify", "label": "Approve with modifications"},
      {"id": "discuss", "label": "Discuss further"},
      {"id": "abort", "label": "Abort rebase"}
    ]
  }]
})
```

**Track** your human partner's decisions per conflict type — when they decide on a type, apply the same strategy to subsequent conflicts of that type automatically. See [conflict-types.md](conflict-types.md) for classification and learning mechanism.

## Step 5: Execute Resolutions

⚠️ **Confirm before executing** — display resolution table, then **use AskQuestion**:

```json
AskQuestion({
  "title": "Apply Resolutions",
  "questions": [{
    "id": "apply_confirm",
    "prompt": "Apply the resolutions listed above?",
    "options": [
      {"id": "yes", "label": "Yes, apply"},
      {"id": "no", "label": "Cancel"},
      {"id": "modify", "label": "Modify some"}
    ]
  }]
})
```

After approval:

1. **Edit** each file with the approved resolution
2. **Remove** all conflict markers (`<<<<<<<`, `=======`, `>>>>>>>`)
3. **Verify** no conflict markers remain using the **Grep tool** (not shell `rg` or `grep`)
4. **Run** `git add <file>` for each resolved file

## Step 6: Post-Resolution Continue

⚠️ **Always use `GIT_EDITOR=true git rebase --continue`** to prevent vim/editor blocking.

**Use AskQuestion**:

```json
AskQuestion({
  "title": "Conflicts Resolved",
  "questions": [{
    "id": "post_resolution",
    "prompt": "All conflicts for commit <hash> resolved and staged.",
    "options": [
      {"id": "continue", "label": "Continue rebase"},
      {"id": "abort", "label": "Abort rebase"},
      {"id": "review", "label": "Review diff first (git diff --staged)"}
    ]
  }]
})
```

If more conflicts appear after continuing, **repeat from Step 1**.

## Step 7: Cleanup and Summary

**Run** `git diff --name-status HEAD~1 | grep "^A" | grep -E "(vendor|nanoflann|third_party)"` to check for orphaned vendor files. If found and not referenced in BUILD, **remove** and amend.

**Present** final summary using the Rebase Complete template from [templates.md](templates.md).

---

## References

- [conflict-types.md](conflict-types.md) — Classification, default strategies, and learning mechanism
- [commands-reference.md](commands-reference.md) — Quick reference git commands for rebase operations
- [templates.md](templates.md) — Presentation templates for conflict summaries and resolution reports

## Integration

**Called by:**

- Any workflow encountering rebase conflicts

**Pairs with:**

- **publishing-pr** — after conflicts are resolved, publish the rebased branch
