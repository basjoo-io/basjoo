# Quick Reference Commands

Git commands for rebase conflict resolution.

## Status & Navigation

```bash
git status                              # Current conflict status
git log -1 REBASE_HEAD                  # Commit being rebased
git diff <file>                         # Conflict details for a file
git diff --name-only --diff-filter=U    # List all conflicting files
```

## Resolution

```bash
git checkout --theirs <file>            # Accept branch being rebased (in rebase context)
git checkout --ours <file>              # Accept target branch / master (in rebase context)
git add <file>                          # Stage resolved file
```

## Rebase Control

```bash
GIT_EDITOR=true git rebase --continue   # Continue (prevents editor blocking)
git rebase --abort                      # Abort entirely
git rebase --skip                       # Skip current commit
```

## Cleanup

```bash
rm -f .git/.COMMIT_EDITMSG.swp                                              # Remove stale swap files
git diff --name-status HEAD~1 | grep "^A" | grep -E "(vendor|third_party)"  # Check orphaned vendor files
```
