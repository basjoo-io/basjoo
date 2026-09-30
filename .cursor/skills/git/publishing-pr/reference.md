# PR Commit and Publish — Reference

Back to [SKILL.md](SKILL.md).

## PR Type Options

Used by both commit messages and PR titles (conventional commits format):

| Type     | When to Use                                             |
| -------- | ------------------------------------------------------- |
| feat     | A new feature                                           |
| fix      | A bug fix                                               |
| docs     | Documentation only changes                              |
| style    | Changes that do not affect the meaning of the code      |
| refactor | Code change that neither fixes a bug nor adds a feature |
| perf     | Code change that improves performance                   |
| test     | Adding missing tests or correcting existing tests       |
| build    | Changes to build system or external dependencies        |
| ci       | Changes to CI configuration files and scripts           |
| chore    | Other changes that don't modify src or test files       |
| revert   | Reverts a previous commit                               |
| config   | Changes to config that affect system behavior           |

## Scope Extraction Rules

Extract scope from the primary directory being modified:

| Directory Pattern          | Scope          |
| -------------------------- | -------------- |
| `src/localization/` | `localization` |
| `src/math/`           | `math`         |
| `src/proto/`     | `proto`        |
| `src/perception/`       | `perception`   |
| `config/integration/`      | `config`       |
| `experimental/`            | `experimental` |

General rule: use the most specific meaningful directory name.

## Complete Examples

### Example 1: With Uncommitted Changes

```bash
# Step 1: Check working tree
git status --porcelain  # Returns output → has changes

# Step 2: Analyze
DIFF_BASE=$(git merge-base origin/master HEAD)
git status
git diff $DIFF_BASE --stat

# Step 3: Commit (check for risky files before git add)
git add .
git commit -m "$(cat <<'EOF'
feat(localization): add interpolator name for offline debug

Add name tracking to Interpolator class to help identify which
interpolator is being used during offline debugging sessions.
EOF
)"

# Step 4: Format & push
# Ensure cmake and tooling are on PATH
BASE_COMMIT=$(git merge-base HEAD origin/master)
DIFF_FILES=$(git diff --name-only --diff-filter=ACMR $BASE_COMMIT -- . ':!experimental')
[ -n "$DIFF_FILES" ] && format_diff $DIFF_FILES
git clang-format --force --extensions=h,hpp,cc,cpp,cu --style=file $BASE_COMMIT -- . ':!experimental' 2>/dev/null || true
# Commit format changes if any
[ -n "$(git status --short)" ] && git add . && git commit -m "[script] diff format"
git push --force-with-lease -u origin HEAD

# Step 5: Create or locate PR
PR_NUMBER=$(gh pr view --json number -q .number 2>/dev/null || echo "")
# No existing PR → create one
gh pr create --draft --title "feat(localization): add interpolator name for offline debug" --base master --body ""
PR_NUMBER=$(gh pr view --json number -q .number)
IS_NEW_PR=true

# Step 6: Update PR description (read existing → merge)
OWNER_REPO=$(gh repo view --json nameWithOwner -q .nameWithOwner)
gh pr view --json body -q .body
gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER" -X PATCH -f body="$(cat <<'EOF'
## Summary
Add name tracking to the Interpolator class for easier offline debugging.

## Changes
- Added `name_` member to Interpolator base class
- Added `GetName()` method to retrieve interpolator name
- Updated all Interpolator subclasses to set names

## Test Plan
- [ ] Unit tests pass
- [ ] Verified interpolator names appear in debug output
EOF
)" --jq '.html_url'

# Step 7: Trigger CI (new PR)
gh pr comment $PR_NUMBER --body "bugbot run"
```

### Example 2: Working Tree Already Clean

```bash
# Step 1: Working tree clean
git status --porcelain  # Returns empty
git log origin/master..HEAD --oneline  # Shows commits to publish

# Steps 2-3: SKIP (nothing to commit)

# Step 4: Format & push (format still runs as safety net)
# Ensure cmake and tooling are on PATH
BASE_COMMIT=$(git merge-base HEAD origin/master)
DIFF_FILES=$(git diff --name-only --diff-filter=ACMR $BASE_COMMIT -- . ':!experimental')
[ -n "$DIFF_FILES" ] && format_diff $DIFF_FILES
git clang-format --force --extensions=h,hpp,cc,cpp,cu --style=file $BASE_COMMIT -- . ':!experimental' 2>/dev/null || true
[ -n "$(git status --short)" ] && git add . && git commit -m "[script] diff format"
git push --force-with-lease -u origin HEAD

# Step 5: Create or locate PR
PR_NUMBER=$(gh pr view --json number -q .number 2>/dev/null || echo "")
# PR already exists → skip creation
IS_NEW_PR=false

# Step 6: Update PR description (read existing → merge, NEVER replace)
OWNER_REPO=$(gh repo view --json nameWithOwner -q .nameWithOwner)
gh pr view --json body -q .body
gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER" -X PATCH -f body="$(cat <<'EOF'
## Summary
Add name tracking to the Interpolator class for easier offline debugging.

## Changes
- Added `name_` member to Interpolator base class
- Added `GetName()` method to retrieve interpolator name

## Test Plan
- [ ] Unit tests pass
EOF
)" --jq '.html_url'

# Step 7: SKIP (existing PR — CI re-triggers on push)
```
