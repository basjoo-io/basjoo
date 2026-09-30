# basjoo — GitHub Copilot Instructions

This file orients coding agents and contributors to the **basjoo** codebase. Deeper, file-type-specific guidance lives under [`.github/instructions/`](instructions/).

## Project overview

basjoo is a **high-performance C++23** mathematical library (with **GNU extensions**) for autonomous driving and robotics: dense/sparse linear algebra, piecewise curves and functions, convex optimization, and motion-planning-oriented bicycle dynamics models. The license is **BSD 3-Clause**. **Python** APIs and tooling are **planned** (uv, ruff, pyright, pytest).

## Technical stack

| Area        | Choice                                               |
| ----------- | ---------------------------------------------------- |
| C++         | C++23, GCC 14+ / Clang 20+ / MSVC (Windows preset)   |
| Build       | CMake 3.31+ (Ninja), optional **xmake**              |
| C++ deps    | **CPM**, **vcpkg** (vendored tree)                   |
| Python deps | **uv** + `pyproject.toml` / `uv.lock` (when enabled) |
| Format      | clang-format 20+ (Microsoft-based), ruff (Python)    |
| Lint        | clang-tidy 20+, ruff, pyright                        |
| Test        | doctest 2.4.12 (C++), pytest (Python)                |

## Module architecture

```
src/basjoo/
├── common/    # Macros, logging, allocators, shared utilities
├── math/      # dense, sparse, curves, functions, mdfunctions
├── cvxopm/    # Convex optimization problems & solvers
└── bicycle/   # Motion models
```

Dependency order: **`common → math → cvxopm → bicycle`** (acyclic). Tests mirror this under `tests/basjoo/`.

## Namespaces

- `basjoo::common`, `basjoo::math`, `basjoo::math::pmr`, `basjoo::math::detail`, `basjoo::cvxopm`, `basjoo::bicycle`
- Never `using namespace` in headers; close namespaces with `} // namespace basjoo::...`

## Build commands

**CMake** (preferred):

```bash
cmake --preset linux-gcc-x64
cmake --build --preset linux-gcc-x64
ctest --preset linux-gcc-x64
ctest --preset linux-gcc-x64 -R <regex>
```

Presets: `linux-gcc-x64`, `linux-clang-x64`, `darwin-gcc-arm64`, `darwin-clang-arm64`, `windows-msvc-x64`. Build dir: `out/build/<preset>`.

**xmake**:

```bash
xmake build
```

**Python** (when present):

```bash
uv venv --system-site-packages   # ROS pytest plugins need /usr/lib/python3/dist-packages
uv sync --no-install-project
uv run pytest
```

## Build options (CMake)

| CMake option                | Purpose                                       |
| --------------------------- | --------------------------------------------- |
| `BASJOO_CHECK_PARAMS`        | Extra validation (`BASJOO_CHECK_PARAMS` macro) |
| `BASJOO_BUILD_TESTING`       | Tests                                         |
| `BASJOO_ENABLE_INSTALL`      | Install rules                                 |
| `BASJOO_USE_BLAS_LAPACK`     | BLAS/LAPACK backend                           |
| `BASJOO_USE_BOOST_UNORDERED` | Boost unordered containers                    |

## Dependencies (indicative)

| Library             | Role                     |
| ------------------- | ------------------------ |
| Boost               | Unordered                |
| zpp_bits            | Serialization            |
| doctest             | Unit tests               |
| spdlog              | Logging                  |
| OSQP / qdldl        | QP                       |
| OpenBLAS            | BLAS/LAPACK              |
| cxxopts / Matplot++ | Test utilities           |

Pins are in CMake/xmake dependency files—not duplicated here.

## Naming conventions

| Element           | Style                                                        |
| ----------------- | ------------------------------------------------------------ |
| Types             | `PascalCase`                                                 |
| Functions         | `camelCase`                                                  |
| Private fields    | `m_snake_case`                                               |
| Constants         | `kPascalCase`                                                |
| Macros            | `BASJOO_SNAKE_CASE`                                           |
| Type-alias suffix | `s` float, `d` double, `c` complex float, `z` complex double |

## Coding rules (summary)

1. **Trailing return type** on new functions: `auto foo() -> Bar;`
2. **`#pragma once`**; include order: std → third party → `basjoo/...`
3. `final`, `explicit`, `override` per guidelines in [instructions](instructions/cpp-coding-standards.instructions.md)
4. **Exceptions** for errors; **`BASJOO_CHECK_PARAMS`** for optional defensive checks—not alternate status types
5. **zpp_bits** roundtrip tests for serializable types
6. **GNU attributes** on hot paths only (`[[using gnu: ...]]`)

## Error handling pattern

```cpp
constexpr auto coeff(size_type i) const noexcept(!BASJOO_CHECK_PARAMS) -> value_type {
#if BASJOO_CHECK_PARAMS == 1
  if (i >= kSize) [[unlikely]] {
    throw std::out_of_range("index out of range");
  }
#endif
  return m_data[i];
}
```

## Performance

- Stack + `alignas`; `std::span`; PMR where provided
- `constexpr` / concepts; avoid virtual/`std::function` in hot loops
- BLAS/LAPACK via configured backend; SIMD gated by compiler feature macros (`__AVX512F__`, etc.) per TU

## Macros (`basjoo/common/macros.hpp`)

`ENABLE_COPY` / `DISABLE_COPY`, `ENABLE_MOVE` / `DISABLE_MOVE`, `ENABLE_COPY_AND_MOVE` / `DISABLE_COPY_AND_MOVE`, `MAKE_SINGLETON`, etc.—use these instead of ad-hoc `= delete` when matching project style.

## CMake targets

Helper functions in `cmake/utils.cmake`:

```cmake
basjoo_cxx_library(NAME <target> HDRS "..." [SRCS "..."] DEPS ...)
basjoo_cxx_test(NAME test_<target> SRCS "test_<source>.cpp" DEPS ...)
basjoo_cxx_module(NAME <target> IXXS "..." DEPS ...)  # when modules are used
```

## Testing (doctest)

- `TEST_CASE_TEMPLATE` for multi-scalar coverage
- `#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN` **once** before `#include "doctest/doctest.h"`
- `doctest::Approx` with explicit epsilon for floats
- Header-under-test included first

## Security

- No hardcoded credentials; no secrets in tracked config

## TODO format

```
// TODO(github_id): description [#issue]
```

## Instruction files

| File                                                                                            | Scope            |
| ----------------------------------------------------------------------------------------------- | ---------------- |
| [workflow.instructions.md](instructions/workflow.instructions.md)                               | General workflow |
| [cpp-coding-standards.instructions.md](instructions/cpp-coding-standards.instructions.md)       | `*.hpp`, `*.cpp` |
| [cpp-testing.instructions.md](instructions/cpp-testing.instructions.md)                         | `*_test.cpp`     |
| [cmake.instructions.md](instructions/cmake.instructions.md)                                     | CMake            |
| [copyright.instructions.md](instructions/copyright.instructions.md)                             | Headers          |
| [general.instructions.md](instructions/general.instructions.md)                                 | All files        |
| [security.instructions.md](instructions/security.instructions.md)                               | Secrets          |
| [todo-format.instructions.md](instructions/todo-format.instructions.md)                         | TODOs            |
| [python-coding-standards.instructions.md](instructions/python-coding-standards.instructions.md) | Python           |
| [python-testing.instructions.md](instructions/python-testing.instructions.md)                   | Python tests     |

---

# andrej-karpathy-skills

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:

- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:

- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:

- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:

- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:

```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.
