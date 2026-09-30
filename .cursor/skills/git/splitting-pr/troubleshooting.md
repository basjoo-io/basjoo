# Troubleshooting

Common issues when splitting PRs and their fixes.

---

## Uncommitted Changes

**Symptom**: "Current branch has uncommitted changes"

**Fix**: Commit or stash before splitting:

```bash
git stash  # or git commit
```

---

## Rebase Conflicts

**Symptom**: "Conflicts occurred during sync"

**Fix**:

```bash
git add .
git rebase --continue
# Then retry the split
```

---

## No Changes Found

**Symptom**: "No changes found in specified directories"

**Fix**: Verify the path has changes vs the merge-base:

```bash
DIFF_BASE=$(git merge-base origin/master HEAD)
git diff $DIFF_BASE -- path/to/dir/
```

Check spelling and ensure paths are relative to repo root.

---

## No Commits Ahead

**Symptom**: "No commits ahead of remote branch"

**Fix**: Ensure changes are committed:

```bash
git log origin/master..HEAD  # Should show commits
```

---

## Patch Apply Fails

**Symptom**: `git apply` exits with error or conflicts.

**Fix**: Try 3-way merge:

```bash
git apply --3way /tmp/diff.patch
```

If still failing, inspect the patch for conflicting hunks:

```bash
cat /tmp/diff.patch
```

---

## BUILD Targets Missing

**Symptom**: After splitting, `cmake --build` fails with missing targets for existing targets.

**Cause**: When adding files to an EXISTING directory, the entire BUILD file was replaced with only new targets, deleting all existing ones.

**Fix**:

```bash
git show origin/master:path/to/BUILD > /tmp/existing.txt
# Append your new targets to /tmp/existing.txt
cp /tmp/existing.txt path/to/BUILD
git add path/to/BUILD
git commit --amend  # or new commit
git push --force-with-lease
```

**Prevention**: Always check first:

```bash
git show origin/master:path/to/BUILD 2>/dev/null && echo "EXISTS - keep existing targets!"
```

See [build-file-handling.md](build-file-handling.md) §Scenario A for the correct approach.

---

## Worktree Already Exists

**Symptom**: `git worktree add` fails with "already checked out" or "already exists"

**Fix**: Remove stale worktree and retry:

```bash
git worktree remove .worktrees/<branch_name> --force
git worktree prune
git worktree add .worktrees/<branch_name> -b <branch_name> origin/master
```

---

## Worktree Directory Not Ignored

**Symptom**: `git status` shows worktree contents as untracked files

**Fix**: Add to local exclude (no commit needed):

```bash
GIT_DIR="$(git rev-parse --git-dir)"
mkdir -p "${GIT_DIR}/info"
echo '.worktrees/' >> "${GIT_DIR}/info/exclude"
```

Verify: `git check-ignore -q .worktrees` should return 0.
