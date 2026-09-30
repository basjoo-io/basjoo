---
name: splitting-pr
description: >-
  Split large PRs into smaller, focused PRs and publish each independently.
  Use when your human partner says "拆分PR", "split PR", "PR太大", or explicitly references
  @splitting-pr or /splitting-pr.
---

# PR Split and Publish

**Core principle:** Each split must be independently reviewable — no file appears in two PRs, and dependencies merge first.

**Announce at start:** "I'm using the splitting-pr skill to split this PR into smaller, focused PRs."

**Activation:** When your human partner asks to split a PR (e.g., "拆分PR", "split PR", "PR太大"), explicitly references `@splitting-pr` or `/splitting-pr`, or is called by another skill. Do NOT auto-trigger on bare "PR" without split intent.

**Create TodoWrite todos from this workflow:**

1. Analyze changes (`git diff $DIFF_BASE --stat`)
2. Present split plan table → wait for your human partner's approval
3. Initialize worktree environment (Phase 2.5)
4. For each split PR: create worktree → prepare → confirm → invoke `/publishing-pr`
5. Clean up worktrees and report final summary with all PR URLs

## When to Use

- PR has changes spanning multiple unrelated areas (e.g., infra + feature + tests)
- PR is too large for a single reviewer to digest
- Your human partner explicitly says "拆分PR", "split PR", or references this skill

## When NOT to Use

- PR is already focused on a single concern
- Changes are tightly coupled and cannot be split without breaking compilation
- Small PRs (< 200 lines) that don't benefit from splitting

---

## Phase 1: Analyze

### 0. Compute diff base

Use `git merge-base` to find the common ancestor between the current branch and `origin/master`. This ensures we only analyze the user's actual changes, even if the branch has not been rebased onto the latest master.

```bash
git fetch origin
DIFF_BASE=$(git merge-base origin/master HEAD)
echo "Diff base (merge-base): $DIFF_BASE"
```

### 1. Understand the change scope

**Run** these commands:

```bash
git diff $DIFF_BASE --stat
git diff $DIFF_BASE --stat | grep -E "^\s" | awk -F'/' '{print $1"/"$2}' | sort | uniq -c | sort -rn
```

**Present** a split plan table to your human partner:

| PR # | Branch Name | Files | Type | Estimated Lines |
| ---- | ----------- | ----- | ---- | --------------- |
| 1    | ...         | ...   | ...  | ...             |

Branch names MUST follow [branch-naming-convention.md](../branch-naming-convention.md). Quick ref: `<your_name>_<describe_the_pr_change>` (e.g., `alice_add_knn_library`).

**Classify** each file group by category and assign split priority:

| Category                              | Priority                          |
| ------------------------------------- | --------------------------------- |
| Infrastructure (BUILD, deps, configs) | Split first — often blocks others |
| Refactoring (no behavior change)      | Split early                       |
| Core logic / algorithms               | Split by feature                  |
| Interface (protos, APIs, types)       | Split separately                  |
| Tests                                 | Keep with related implementation  |

**Order** splits so that dependencies merge first:

1. Base infrastructure → 2. Shared utilities → 3. Core features → 4. Dependent features → 5. Integration/cleanup

### Compilability Gate (MANDATORY)

For EACH proposed split PR, verify it won't break compilation when merged alone:

1. **Identify header files with API changes** (renamed types/functions, changed signatures, removed fields):
   ```bash
   git diff $DIFF_BASE -- <PR_files> | grep -E '^\-.*struct |^\-.*class |^\-.*enum |^\-.*using ' | head -20
   ```
2. **For each changed header**, find ALL consumers in the repo:
   ```bash
   # Find BUILD targets depending on the changed library
   grep -r ':<target_name>"' --include=BUILD <repo_path>
   # Find source files including the header
   grep -r '#include.*<header>' --include='*.h' --include='*.cc' <repo_path>
   ```
3. **Rule**: If a header's public API changes (type renames, field removals, signature changes), ALL consumers that reference the old API **MUST be in the same PR**. Moving consumers across PRs is acceptable — violating compilation is not.
4. **Validate** by dry-building in the worktree before committing:
   ```bash
   cmake --build --preset linux-gcc-x64 2>&1 | tail -20
   ```

⚠️ Directory-based grouping is a starting heuristic, NOT the final split. The compilability gate overrides directory boundaries.

---

## Phase 2: Choose Strategy

### Decision: File distribution approach

- **Default (recommended)** — Each file appears in exactly ONE split PR
  - Eliminates merge conflicts between split PRs
  - PRs merge in dependency order (each "Update branch" before merge)
  - Only the final PR needs to compile
- **If independent builds are required** — Each PR must build and pass tests independently
  - May require duplicating BUILD files or shared configs across PRs

### Decision: BUILD file handling

- **Run** `git show origin/master:path/to/BUILD 2>/dev/null` to check if BUILD exists on master
- **If BUILD exists on master** → Scenario A: KEEP all existing targets, ADD new ones. **Read** [build-file-handling.md](build-file-handling.md) §Scenario A.
- **If no BUILD on master** → Scenario B: Create separate BUILD per split PR with only relevant targets. **Read** [build-file-handling.md](build-file-handling.md) §Scenario B.

⚠️ **Never replace an existing BUILD file with only your new targets** — this deletes all existing targets. See [troubleshooting.md](troubleshooting.md) §BUILD Targets Missing.

**Wait** for your human partner's approval of the split plan before proceeding.

---

## Phase 2.5: Initialize Worktree Environment

After your human partner approves the split plan, set up an isolated worktree directory so the **main workspace never switches branches**.

### 0. Sync remote and record commits

```bash
git fetch origin
```

Record the base commit for worktree creation AND the diff base for patch generation:

```bash
BASE_COMMIT=$(git rev-parse origin/master)
DIFF_BASE=$(git merge-base origin/master HEAD)
echo "Worktree base (origin/master): $BASE_COMMIT"
echo "Diff base (merge-base):        $DIFF_BASE"
```

- `BASE_COMMIT` — used to create worktrees (latest `origin/master`)
- `DIFF_BASE` — used to generate patches (only the user's actual changes, not master drift)

### 1. Directory selection

```bash
ls -d .worktrees 2>/dev/null || ls -d worktrees 2>/dev/null
```

- If `.worktrees/` exists → use it. If both exist → `.worktrees/` wins.
- If neither exists → use `AskQuestion` (title: "Worktree Directory", options: ".worktrees/ (project-local, hidden)" / "Custom path").

### 2. Verify directory is ignored

```bash
git check-ignore -q .worktrees 2>/dev/null
```

If **NOT** ignored → add to **local exclude** (no commit needed):

```bash
GIT_DIR="$(git rev-parse --git-dir)"
mkdir -p "${GIT_DIR}/info"
grep -qxF '.worktrees/' "${GIT_DIR}/info/exclude" 2>/dev/null \
  || echo '.worktrees/' >> "${GIT_DIR}/info/exclude"
```

Verify: `git check-ignore -q .worktrees` should return 0.

See [troubleshooting.md](troubleshooting.md) §Worktree Directory Not Ignored.

### 3. Record base path

```bash
WORKTREE_BASE=".worktrees"   # or your human partner's choice
ORIGINAL_DIR="$(pwd)"
```

---

## Phase 3: Execute Each Split

For each split PR in the approved plan:

### Step 1: Prepare the branch (via worktree)

**Stay on original branch.** Do NOT checkout — create an isolated worktree instead.

```bash
# Generate patch from original branch (already on it — no checkout needed)
git diff $DIFF_BASE -- <paths> > /tmp/split_N.patch

# Create worktree with new branch from the recorded base commit
git worktree add "${WORKTREE_BASE}/<new_branch_name>" -b <new_branch_name> $BASE_COMMIT

# Apply patch INSIDE the worktree
cd "${WORKTREE_BASE}/<new_branch_name>"
git apply /tmp/split_N.patch
git add .
```

⚠️ If `git apply` fails, **prefer direct file copy** over `--3way` (which can leave partial state):

```bash
# Preferred: copy files directly from the feature branch
for f in <file_list>; do
  git show <feature_branch>:"$f" > "$f"
done
```

Only fall back to `git apply --3way` for large diffs where manual copy is impractical. See [troubleshooting.md](troubleshooting.md) §Patch Apply Fails.
⚠️ If `git worktree add` fails with "already checked out", see [troubleshooting.md](troubleshooting.md) §Worktree Already Exists.

**Handle BUILD file** inside the worktree according to the decision in Phase 2. **Read** [build-file-handling.md](build-file-handling.md) for code templates.

### Step 2: Confirm with your human partner

**Display** in your message:

- Branch name
- File list
- Proposed commit message
- PR type, scope, and summary

**Use AskQuestion** tool:

```
title: "Ready to Commit & Publish Split PR #N"
prompt: "Branch: {branch}\nCommit: {type}({scope}): {description}\nPR type: {type} | Scope: {scope} | Summary: {summary}\n\nProceed with commit and publish?"
options:
  - id: "yes"       label: "Yes, commit & publish"
  - id: "yes_all"   label: "Yes to all (skip remaining confirmations)"
  - id: "no"        label: "No, cancel"
  - id: "modify"    label: "Modify details"
```

| Response                  | Action                                              |
| ------------------------- | --------------------------------------------------- |
| **Yes, commit & publish** | Proceed to Step 3                                   |
| **Yes to all**            | Auto-approve all remaining split PRs                |
| **No, cancel**            | Skip this PR, ask what to change                    |
| **Modify details**        | Ask your human partner for changes, then re-confirm |

### Step 3: Commit & Publish (inside worktree)

**Ensure cwd is inside the worktree** before running publishing-pr:

```bash
cd "${WORKTREE_BASE}/<new_branch_name>"
```

**Read and invoke** the `/publishing-pr` skill.

**Skip Steps 2.5 and 4.5** in publishing-pr (confirmation steps) — your human partner already approved in Step 2 above.

Execute publishing-pr Steps 3–7:

- Step 3: Commit with HEREDOC
- Step 4: Push to remote
- Step 5: Create or locate PR with `gh pr create --draft --title "<type>(<scope>): <summary>" --base master`
- Step 6: Update PR description (⚠️ **Read existing description first, merge new info — never replace**)
- Step 7: Trigger CI for new PRs

### Step 4: Return & Loop

**Return to main workspace** after each split PR is published:

```bash
cd "${ORIGINAL_DIR}"
```

**Update** TodoWrite progress, then go back to Step 1 for the next split PR.

---

## Phase 4: Post-Split

After all split PRs are published:

1. **Report** final summary with all PR URLs and merge order

2. **Do NOT auto-clean worktrees** — your human partner may need them to address review comments locally. List remaining worktrees for reference:

   ```bash
   cd "${ORIGINAL_DIR}"
   git worktree list | grep "${WORKTREE_BASE}"
   ```

3. **Original branch** remains intact — it was never switched away from

4. **Merge order**: Your human partner merges split PRs in dependency order, clicking "Update branch" between each

5. **Manual cleanup** (your human partner runs when done with all reviews):
   ```bash
   for wt in ${WORKTREE_BASE}/*/; do
     git worktree remove "$wt" --force 2>/dev/null
   done
   git worktree prune
   ```

---

## References

- [build-file-handling.md](build-file-handling.md) — BUILD file scenarios (existing vs new directory) with code templates
- [examples.md](examples.md) — Complete worked examples (KNN Library Split, Evaluator Module Split)
- [troubleshooting.md](troubleshooting.md) — Common issues and fixes

## Integration

**Called by:**

- Any workflow where a PR is too large and needs to be split

**Depends on:**

- **using-git-worktrees** — Phase 2.5 follows its directory selection and safety verification protocol
- **publishing-pr** — Step 3 of each split invokes this skill for commit and publish
