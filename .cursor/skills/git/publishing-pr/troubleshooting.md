# PR Commit and Publish — Troubleshooting

Back to [SKILL.md](SKILL.md).

## "Nothing to commit"

**Run** `git log origin/master..HEAD` to check if changes are already committed. If so, skip to Step 4 (Push).

## "Branch has no upstream"

**Run** `git push --force-with-lease -u origin HEAD` — the `-u` flag sets upstream tracking.

## Push rejected after rebase

`--force-with-lease` handles this. If it still fails (e.g., remote has unexpected commits from another author), check with your human partner before force pushing.

## "Projects (classic) is being deprecated" — gh pr edit fails

`gh pr edit` triggers a GraphQL query on Projects Classic which GitHub is sunsetting. Use `gh api` REST endpoint instead:

```bash
OWNER_REPO=$(gh repo view --json nameWithOwner -q .nameWithOwner)
gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER" -X PATCH -f body="..." --jq '.html_url'
```

## "gh: command not found"

Ensure `gh` CLI is installed and available on PATH.

## Pre-commit hook failures

If `git commit` fails due to pre-commit hooks:

1. **Read** the hook error output
2. **Fix** the reported issues (typically formatting or linting)
3. **Run** `git add .` to stage fixes
4. **Retry** the failed step (commit or publish)

## Commit Fails in Worktree (HEREDOC)

**Symptom**: `git commit -m "$(cat <<'EOF' ...)"` in a worktree silently produces no commit (exit 0 but HEAD unchanged).

**Fix**: Use `-C` flag to target the worktree explicitly, or use multi-line `-m`:

```bash
# Option A: explicit worktree path
git -C /path/to/worktree commit -m "message"

# Option B: multiple -m flags instead of HEREDOC
git commit -m "type(scope): summary" -m "- detail 1" -m "- detail 2"
```

**Verify** commit actually happened:

```bash
git log -1 --oneline  # Should show your new commit, not the base
```
