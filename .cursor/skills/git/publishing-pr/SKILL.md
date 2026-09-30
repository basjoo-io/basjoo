---
name: publishing-pr
description: >-
  Commit local changes and publish as a GitHub PR with automated interaction handling.
  Use when your human partner says "发PR", "提交并发布", "publish PR", or explicitly references
  @publishing-pr or /publishing-pr.
---

# PR Commit and Publish

**Core principle:** Commit with clarity, publish with confidence — every PR tells a complete story covering all its commits.

**Announce at start:** "I'm using the publishing-pr skill to commit and publish a PR."

**Create TodoWrite todos from this workflow:**

| Step | Action                   | Description                                     |
| ---- | ------------------------ | ----------------------------------------------- |
| 1    | Check working tree       | `git status --porcelain` — skip commit if clean |
| 2    | Analyze changes          | `git diff $DIFF_BASE` — understand scope        |
| 3    | Commit (if needed)       | Stage, generate message, commit with HEREDOC    |
| 4    | Format & push            | diff format → commit format → push              |
| 5    | Create or locate PR      | `gh pr create --draft` if no PR exists          |
| 6    | Update PR description    | Read existing → merge new content → `gh api`    |
| 7    | Trigger CI (new PR only) | Post CI trigger comment (if project uses one)    |

**Activation:** When your human partner asks to publish a PR (e.g., "发PR", "提交并发布", "publish PR"), explicitly references `@publishing-pr` or `/publishing-pr`, or is called by another skill. Do NOT auto-trigger on bare keywords like "commit" or "push" without PR intent.

## When to Use

- Your human partner says "发PR", "提交并发布", "publish PR"
- Explicitly referenced via `@publishing-pr` or `/publishing-pr`
- Called by other skills (e.g., `splitting-pr`) to publish a split PR

## When NOT to Use

- Bare "commit" or "push" without intent to create/update a PR
- Draft commits during development — just use `git commit` directly
- Already published PR that only needs description updates

---

## Step 1: Pre-publish Checks

### 1a) Check working tree

```bash
git status --porcelain
git log origin/master..HEAD --oneline
```

**Decision:**

- `git status --porcelain` returns **empty** → working tree clean, **skip to Step 4**
- `git status --porcelain` returns **output** → has changes, **proceed to Step 2**
- `git log` returns **empty** AND working tree clean → nothing to publish, **stop and report**

## Step 2: Analyze Changes

Use `merge-base` to only show the user's actual changes (not master drift):

**Run**:

```bash
DIFF_BASE=$(git merge-base origin/master HEAD)
git status
git diff --cached --stat
git diff --stat
git diff $DIFF_BASE --stat
git diff $DIFF_BASE
```

## Step 3: Commit

**Skip** if working tree is clean.

### Generate Commit Message

Follow conventional commits: `<type>(<scope>): <description>`

- **Type**: See [PR Type Options](reference.md#pr-type-options) — use the same type for commit and publish
- **Scope**: Extract from primary directory (e.g., `src/localization/` → `localization`)
- **Description**: Imperative mood, concise, focus on "why"

### Stage and Commit

⚠️ **Before `git add .`**: **Check** `git status` output for risky files. If any of the following are detected, **stop and escalate to your human partner** via AskQuestion:

- Secrets or credentials (`.env`, `*token*`, `*secret*`, `*password*`)
- Generated artifacts (`*.pb.go`, `*_pb2.py`, `compile_commands.json`)
- Large binaries (`*.bin`, `*.so`, `*.tar.gz`)
- Files outside the expected change scope

If clean, proceed:

```bash
git add .
git commit -m "$(cat <<'EOF'
<type>(<scope>): <description>

<optional body>
EOF
)"
```

If commit fails (pre-commit hook, merge conflict, etc.), **stop and report the error to your human partner** — do not retry silently.

## Step 4: Format & Push

### 4a) Run diff format

Format all changed files (diff against master) as a safety net:

```bash
# Ensure cmake and tooling are on PATH

BASE_COMMIT=$(git merge-base HEAD origin/master)
DIFF_FILES=$(git diff --name-only --diff-filter=ACMR $BASE_COMMIT -- . ':!experimental')

if [ -n "$DIFF_FILES" ]; then
  format_diff $DIFF_FILES
fi
```

For C++ files, also run line-level formatting on the diff:

```bash
git clang-format --force --extensions=h,hpp,cc,cpp,cu --style=file $BASE_COMMIT -- . ':!experimental' 2>/dev/null || true
```

### 4b) Commit format changes (if any)

```bash
if [ -n "$(git status --short)" ]; then
  git add .
  git commit -m "[script] diff format"
fi
```

### 4c) Push

```bash
git push --force-with-lease -u origin HEAD
```

⚠️ `--force-with-lease` allows push after rebase while protecting against overwriting unexpected remote commits. The `-u` flag sets upstream tracking for new branches.

## Step 5: Create or Locate PR

### 5a) Check if PR already exists

```bash
PR_NUMBER=$(gh pr view --json number -q .number 2>/dev/null || echo "")
```

### 5b) Decision

- **`PR_NUMBER` is non-empty** → PR already exists. Record `PR_NUMBER`, set `IS_NEW_PR=false`, skip to Step 6.
- **`PR_NUMBER` is empty** → No PR yet. Proceed to Step 5c.

### 5c) Create new PR

Reuse the same `<type>`, `<scope>`, and `<summary>` from the commit message in Step 3 for the PR title:

```bash
gh pr create --draft --title "<type>(<scope>): <summary>" --base master --body ""
PR_NUMBER=$(gh pr view --json number -q .number)
IS_NEW_PR=true
```

Type/scope table: See [reference.md — PR Type Options](reference.md#pr-type-options).

⚠️ The PR is created as **draft**. The body is intentionally empty — Step 6 writes the full description.

## Step 6: Update PR Description

⚠️ **NEVER blindly replace the PR description.** Always read existing → merge new content. The description must cover ALL commits in the PR, not just the latest.

### 6a. Read Existing

```bash
PR_NUMBER=$(gh pr view --json number -q .number)
gh pr view --json body -q .body
```

### 6b. Decision

- **Body empty** (new PR) → **Write** from scratch covering ALL commits (`git log origin/master..HEAD`)
- **Body present** (existing PR) → **Merge** new changes into existing description. Do NOT discard earlier content.

### 6c. Write

⚠️ **Use `gh api` instead of `gh pr edit`** — `gh pr edit` fails with "Projects (classic) deprecated" error.

```bash
OWNER_REPO=$(gh repo view --json nameWithOwner -q .nameWithOwner)
gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER" -X PATCH -f body="$(cat <<'EOF'
## Summary
<Brief description covering ALL changes in the PR>

## Changes
- <Change 1>
- <Change 2>

## Test Plan
- [ ] Unit tests pass
- [ ] Manual testing completed
EOF
)" --jq '.html_url'
```

## Step 7: Trigger CI (New PR Only)

**Decision**: Check `IS_NEW_PR` from Step 5:

- `IS_NEW_PR=true` → **NEW PR**, proceed
- `IS_NEW_PR=false` → **SKIP** this step (CI re-triggers automatically on push)

**Run** (example — adapt to project CI triggers):

```bash
gh pr comment $PR_NUMBER --body "bugbot run"
```

## References

- [reference.md](reference.md) — Type table, scope rules, complete examples
- [troubleshooting.md](troubleshooting.md) — Common error fixes

## Integration

**Called by:**

- **splitting-pr** — invokes this skill for each split PR's commit and publish
- Any workflow that needs to publish changes as a PR

**Pairs with:**

- **fixing-rebase-conflict** — resolve conflicts before publishing
