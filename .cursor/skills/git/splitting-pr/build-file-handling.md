# CMakeLists.txt Handling

How to handle `CMakeLists.txt` files when splitting PRs. The strategy depends on whether the target directory already has a `CMakeLists.txt` on master.

**Quick check:**

```bash
git show origin/master:path/to/CMakeLists.txt 2>/dev/null && echo "EXISTS" || echo "NEW"
```

---

## Scenario A: Adding to EXISTING Directory

The directory already has a `CMakeLists.txt` on master. **KEEP all existing targets and ADD yours.**

### Steps

```bash
git show origin/master:path/to/CMakeLists.txt > /tmp/existing_cmake.txt

# Append new targets to end of file
cat >> /tmp/existing_cmake.txt << 'EOF'

add_library(your_new_target STATIC
    your_new_target.cc
)
target_link_libraries(your_new_target PRIVATE ...)
EOF

cp /tmp/existing_cmake.txt path/to/CMakeLists.txt
```

### Correct vs Wrong

```cmake
# CORRECT: All existing targets preserved + new one added at end
# CMake: see cmake/utils.cmake for test helpers

# === EXISTING TARGETS (from master) - DO NOT DELETE ===
add_library(pva_diff_calculator ...)
add_library(statistics_aggregator ...)
add_library(evaluator ...)

# === NEW TARGET (added by this PR) ===
add_library(version_comparator STATIC
    version_comparator.cc
)
target_link_libraries(version_comparator PRIVATE ...)
```

```cmake
# WRONG: Only new target — deletes all existing targets!
add_library(version_comparator STATIC version_comparator.cc)
```

---

## Scenario B: Creating NEW Directory

No `CMakeLists.txt` exists on master. **Create a separate `CMakeLists.txt` per split PR** with only the targets relevant to that PR's files.

### Rules

1. Each PR gets its own `CMakeLists.txt` with only its targets
2. `cmake_minimum_required` / project are usually in the root; subdirectory lists add `add_library` / `add_executable` / `add_test`
3. Dependencies can reference targets that don't exist yet — OK if the tree doesn't configure until later merges
4. Goal is file-target consistency, not a green configure on every intermediate branch

### Steps

```bash
# Write CMakeLists.txt content with only this PR's targets
cat > /tmp/prN_cmake.txt << 'CMAKEEOF'
# CMake: see cmake/utils.cmake for test helpers

add_library(target_for_this_pr STATIC
    target_for_this_pr.cc
)
target_link_libraries(target_for_this_pr PRIVATE
    other_target  # OK - doesn't exist yet
)
CMAKEEOF

mkdir -p path/to/new_dir/
cp /tmp/prN_cmake.txt path/to/new_dir/CMakeLists.txt
```

### Example: Splitting new evaluator into 2 PRs

Original `CMakeLists.txt` has: `statistics_aggregator`, `statistics_aggregator_test`, `evaluator_flags`, `evaluator`, `evaluation_main`

**PR 1** `CMakeLists.txt` — only statistics_aggregator targets:

```cmake
# CMake: see cmake/utils.cmake for test helpers

add_library(statistics_aggregator STATIC
    statistics_aggregator.cc
)
target_link_libraries(statistics_aggregator PRIVATE
    pva_diff_calculator  # OK - doesn't exist yet
    basjoo::math_statistics
)

add_executable(statistics_aggregator_test statistics_aggregator_test.cc)
target_link_libraries(statistics_aggregator_test PRIVATE statistics_aggregator)
add_test(NAME statistics_aggregator_test COMMAND statistics_aggregator_test)
```

**PR 2** `CMakeLists.txt` — evaluator targets (merged after PR 1):

```cmake
add_library(evaluator_flags STATIC
    evaluator_flags.cc
)
target_link_libraries(evaluator_flags PRIVATE absl::flags)

add_library(evaluator STATIC
    evaluator.cc
)
target_link_libraries(evaluator PRIVATE
    pva_diff_calculator
    statistics_aggregator  # Will exist after PR 1 merges
)

add_executable(evaluation_main evaluation_main.cc)
target_link_libraries(evaluation_main PRIVATE evaluator evaluator_flags)
```
