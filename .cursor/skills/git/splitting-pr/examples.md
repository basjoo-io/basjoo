# Complete Split Examples

Worked examples demonstrating the full split workflow end-to-end.

> **Note:** These examples are illustrative — adapt file paths, branch names, and commands to match your actual PR. Do NOT copy them literally.

---

## Example 1: KNN Library Split (5,400 lines → 4 PRs)

### Original PR

```
src/math/knn/CMakeLists.txt                              |   94 +
src/math/knn/brute_force_knn.h                     |  199 ++
src/math/knn/knn.h                                 |  186 ++
src/math/knn/knn_interface.h                        |   80 +
src/math/knn/knn_test.cc                            |  647 +++++
src/math/knn/nanoflann/CMakeLists.txt                        |    8 +
src/math/knn/nanoflann/nanoflann.hpp                | 2552 ++++++++++++++
src/math/knn/spherical_euclidean_knn.h              |  226 ++
src/math/knn/spherical_euclidean_knn_benchmark.cc   |  533 ++++
src/math/knn/spherical_euclidean_knn_test.cc        |  495 ++++
src/math/knn/spherical_euclidean_knn_utils.h        |  189 ++
src/math/knn/spherical_euclidean_knn_utils_test.cc  |  193 ++
12 files changed, 5402 insertions(+)
```

### Split Plan

| PR # | Branch                          | Files                                                          | Type  | Lines  |
| ---- | ------------------------------- | -------------------------------------------------------------- | ----- | ------ |
| 1    | `alice_knn_nanoflann_vendor`    | `nanoflann/CMakeLists.txt`, `nanoflann.hpp`                             | build | ~2,560 |
| 2    | `alice_knn_core_interface`      | `knn_interface.h`, `knn.h`, `brute_force_knn.h`, `knn_test.cc` | feat  | ~1,112 |
| 3    | `alice_knn_spherical_euclidean` | `spherical_euclidean_knn.h`, `*_utils.h`, `*_test.cc`          | feat  | ~1,103 |
| 4    | `alice_knn_build_and_benchmark` | `CMakeLists.txt`, `benchmark.cc`                                        | build | ~627   |

### Execution

```bash
# Setup: initialize worktree directory (Phase 2.5)
WORKTREE_BASE=".worktrees"
ORIGINAL_DIR="$(pwd)"
DIFF_BASE=$(git merge-base origin/master HEAD)

# PR #1: Vendor dependency (split first — blocks others)
git diff $DIFF_BASE -- src/math/knn/nanoflann/ > /tmp/nanoflann.patch
git worktree add ${WORKTREE_BASE}/alice_knn_nanoflann_vendor -b alice_knn_nanoflann_vendor origin/master
cd ${WORKTREE_BASE}/alice_knn_nanoflann_vendor
git apply /tmp/nanoflann.patch && git add .
git commit -m "Add nanoflann header-only KNN library"
git push -u origin alice_knn_nanoflann_vendor
gh pr create --draft --title "build(knn): add nanoflann header-only KNN library as vendor dependency" --base master --body ""
cd "${ORIGINAL_DIR}"

# PR #2: Core interface
git diff $DIFF_BASE -- src/math/knn/knn.h src/math/knn/knn_interface.h \
    src/math/knn/brute_force_knn.h src/math/knn/knn_test.cc > /tmp/core.patch
git worktree add ${WORKTREE_BASE}/alice_knn_core_interface -b alice_knn_core_interface origin/master
cd ${WORKTREE_BASE}/alice_knn_core_interface
git apply /tmp/core.patch && git add .
git commit -m "Add core KNN interface and implementations"
git push -u origin alice_knn_core_interface
gh pr create --draft --title "feat(knn): add core KNN interface and implementations" --base master --body ""
cd "${ORIGINAL_DIR}"

# PR #3: Spherical euclidean
git diff $DIFF_BASE -- src/math/knn/spherical_euclidean_knn.h \
    src/math/knn/spherical_euclidean_knn_utils.h \
    src/math/knn/spherical_euclidean_knn_test.cc \
    src/math/knn/spherical_euclidean_knn_utils_test.cc > /tmp/spherical.patch
git worktree add ${WORKTREE_BASE}/alice_knn_spherical_euclidean -b alice_knn_spherical_euclidean origin/master
cd ${WORKTREE_BASE}/alice_knn_spherical_euclidean
git apply /tmp/spherical.patch && git add .
git commit -m "Add spherical euclidean KNN implementation"
git push -u origin alice_knn_spherical_euclidean
gh pr create --draft --title "feat(knn): add spherical euclidean KNN implementation" --base master --body ""
cd "${ORIGINAL_DIR}"

# PR #4: CMakeLists.txt and benchmark (last — CMake targets reference all others)
git diff $DIFF_BASE -- src/math/knn/CMakeLists.txt \
    src/math/knn/spherical_euclidean_knn_benchmark.cc > /tmp/build.patch
git worktree add ${WORKTREE_BASE}/alice_knn_build_and_benchmark -b alice_knn_build_and_benchmark origin/master
cd ${WORKTREE_BASE}/alice_knn_build_and_benchmark
git apply /tmp/build.patch && git add .
git commit -m "Add KNN CMakeLists.txt and benchmark"
git push -u origin alice_knn_build_and_benchmark
gh pr create --draft --title "build(knn): add CMakeLists.txt and benchmark for KNN library" --base master --body ""
cd "${ORIGINAL_DIR}"

# Cleanup all worktrees
for wt in ${WORKTREE_BASE}/*/; do
  git worktree remove "$wt" --force 2>/dev/null
done
git worktree prune
```

### Merge Order

1. Merge `alice_knn_nanoflann_vendor`
2. Update branch → merge `alice_knn_core_interface`
3. Update branch → merge `alice_knn_spherical_euclidean`
4. Update branch → merge `alice_knn_build_and_benchmark` (now builds successfully)

---

## Example 2: Evaluator Module Split (CMakeLists.txt Handling)

### Original PR

```
src/evaluator/CMakeLists.txt                         |  50 +
src/evaluator/statistics_aggregator.h       |  64 +
src/evaluator/statistics_aggregator.cc      | 148 ++
src/evaluator/statistics_aggregator_test.cc |  85 +
src/evaluator/evaluator_flags.h             |  27 +
src/evaluator/evaluator_flags.cc            |  33 +
src/evaluator/evaluator.h                   |  59 +
src/evaluator/evaluator.cc                  | 135 ++
src/evaluator/evaluation_main.cc            | 116 ++
9 files changed, 717 insertions(+)
```

### Split Plan

| PR # | Branch                       | Source Files                                             | CMake targets                                         |
| ---- | ---------------------------- | -------------------------------------------------------- | ----------------------------------------------------- |
| 1    | `alice_evaluator_statistics` | `statistics_aggregator.*`                                | `statistics_aggregator`, `statistics_aggregator_test` |
| 2    | `alice_evaluator_main`       | `evaluator_flags.*`, `evaluator.*`, `evaluation_main.cc` | `evaluator_flags`, `evaluator`, `evaluation_main`     |

### Execution

```bash
# Setup: initialize worktree directory (Phase 2.5)
WORKTREE_BASE=".worktrees"
ORIGINAL_DIR="$(pwd)"
DIFF_BASE=$(git merge-base origin/master HEAD)

# ========== PR #1: Statistics Aggregator ==========

git diff $DIFF_BASE -- \
  src/evaluator/statistics_aggregator.h \
  src/evaluator/statistics_aggregator.cc \
  src/evaluator/statistics_aggregator_test.cc \
  > /tmp/pr1_source.patch

# Create CMakeLists.txt with ONLY statistics_aggregator targets (Scenario B — new directory)
# See build-file-handling.md §Scenario B for the CMakeLists.txt content

git worktree add ${WORKTREE_BASE}/alice_evaluator_statistics -b alice_evaluator_statistics origin/master
cd ${WORKTREE_BASE}/alice_evaluator_statistics
git apply /tmp/pr1_source.patch
mkdir -p src/evaluator
# Write PR-specific CMakeLists.txt here
git add .

git commit -m "$(cat <<'EOF'
feat(evaluator): add statistics aggregator for evaluation results

- Add StatisticsAggregator class to aggregate PVA diff results
- Support per-dimension statistics for position, velocity, attitude
EOF
)"

git push -u origin alice_evaluator_statistics
gh pr create --draft --title "feat(evaluator): add statistics aggregator for evaluation results" --base master --body ""

# Post-publish: update description and trigger CI
PR_NUMBER=$(gh pr view --json number -q .number)
gh pr edit $PR_NUMBER --body "$(cat <<'EOF'
## Summary
Add StatisticsAggregator for aggregating localization evaluation results.
## Changes
- `statistics_aggregator.h/cc`: Main aggregator class
- `statistics_aggregator_test.cc`: Unit tests
## Dependencies
- Part 1 of evaluator module split
- Merge order: this PR → PR #2 (alice_evaluator_main)
EOF
)"
gh pr comment $PR_NUMBER --body "bugbot run"
cd "${ORIGINAL_DIR}"

# ========== PR #2: Evaluator Main ==========

git diff $DIFF_BASE -- \
  src/evaluator/evaluator_flags.h \
  src/evaluator/evaluator_flags.cc \
  src/evaluator/evaluator.h \
  src/evaluator/evaluator.cc \
  src/evaluator/evaluation_main.cc \
  > /tmp/pr2_source.patch

git worktree add ${WORKTREE_BASE}/alice_evaluator_main -b alice_evaluator_main origin/master
cd ${WORKTREE_BASE}/alice_evaluator_main
git apply /tmp/pr2_source.patch
mkdir -p src/evaluator
# Write PR-specific CMakeLists.txt here (see build-file-handling.md)
git add .

git commit -m "$(cat <<'EOF'
feat(evaluator): add evaluator and main entry point

- Add Evaluator class to run evaluation on records
- Add evaluation_main binary as CLI entry point
- Add evaluator_flags for command line arguments
EOF
)"

git push -u origin alice_evaluator_main
gh pr create --draft --title "feat(evaluator): add evaluator and main entry point" --base master --body ""

PR_NUMBER=$(gh pr view --json number -q .number)
gh pr edit $PR_NUMBER --body "$(cat <<'EOF'
## Summary
Add Evaluator class and CLI entry point for localization evaluation.
## Changes
- `evaluator_flags.h/cc`: Command line flags
- `evaluator.h/cc`: Main evaluator logic
- `evaluation_main.cc`: CLI entry point
## Dependencies
- Part 2 of evaluator module split
- **Merge after**: PR #1 (alice_evaluator_statistics)
EOF
)"
gh pr comment $PR_NUMBER --body "bugbot run"
cd "${ORIGINAL_DIR}"

# Cleanup all worktrees
for wt in ${WORKTREE_BASE}/*/; do
  git worktree remove "$wt" --force 2>/dev/null
done
git worktree prune
```

### Merge Order

1. Merge `alice_evaluator_statistics`
2. Update branch → merge `alice_evaluator_main` (CMakeLists.txt now has all targets)

---

## Key Lessons

1. **Vendor/third-party first** — external dependencies block others, split them first
2. **Tests with implementation** — keep test files with their corresponding source files
3. **No file duplication** — each file appears in exactly one PR to avoid conflicts
4. **Missing deps are OK** — `CMakeLists.txt` can reference targets that don't exist yet
5. **Existing vs new directory** — KEEP all existing CMake targets (Scenario A) vs create per-PR `CMakeLists.txt` (Scenario B)
6. **Post-publish** — update PR description (read existing first!) and trigger CI for each new PR
