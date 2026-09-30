# AGENTS.md — basjoo Project Guide for AI Agents

## Project Overview

**basjoo** is a high-performance **C++23** mathematical library for autonomous driving and robotics. It provides dense and sparse linear algebra, piecewise curves and functions, convex optimization solvers, and motion-planning oriented bicycle dynamics models. The project is licensed under the **BSD 3-Clause** license. Python bindings live alongside the C++ sources under `src/basjoo/` and are built via **nanobind**, managed with **uv**.

The codebase uses **GNU extensions** where beneficial and targets multiple host platforms via CMake presets and an optional **xmake** workflow.

## Build System

- **Primary**: **CMake 3.31+** with **Ninja** (see `CMakePresets.json`).
- **Alternatives**: **xmake** (see root `xmake.lua` and `xmake/`).
- **C++ dependencies**: **CPM** (CMake Package Manager) and **vcpkg** (vendored under `vcpkg/` as applicable).
- **Python**: **uv** with `pyproject.toml` / `uv.lock`. The `_core` nanobind extension is built via CMake (or xmake) using the Python interpreter in `.venv/`; `uv venv --system-site-packages` followed by `uv sync --no-install-project` provisions that venv without triggering the (slow, full-C++-dependency-tree) scikit-build-core wheel build — the flag is required so the devcontainer's ROS pytest plugins can see their system-installed dependencies. `pyproject.toml` uses **scikit-build-core**/CMake as the build backend by default; `pyproject.xmake.toml` is an alternate config using the **xmake-python** backend instead (swap in with `cp pyproject.xmake.toml pyproject.toml`), see `.claude/rules/09-python-conventions.md`.

### Key Commands

| Command                              | Purpose                                               |
| ------------------------------------ | ----------------------------------------------------- |
| `cmake --preset <preset>`            | Configure (e.g. `linux-gcc-x64`, `linux-clang-x64`)   |
| `cmake --build --preset <preset>`    | Build all targets                                     |
| `ctest --preset <preset>`            | Run all tests for that preset                         |
| `ctest --preset <preset> -R <regex>` | Run tests matching a name                             |
| `clang-format -i <files>`            | Format C/C++ (Microsoft-based style, see repo config) |
| `clang-tidy <args>`                  | C++ static analysis                                   |
| `xmake` / `xmake build`              | Configure and build via xmake                         |
| `uv venv --system-site-packages`     | Create `.venv` with system site-packages (ROS pytest plugins) |
| `uv sync --no-install-project`       | Provision `.venv` (nanobind, pytest, dev tools) without building the wheel |
| `uv run pytest`                      | Python tests                                          |
| `ruff check` / `ruff format`         | Python lint and format                                |

### CMake Presets

| Preset               | Host                  |
| -------------------- | --------------------- |
| `linux-gcc-x64`      | Linux, GCC, x86_64    |
| `linux-clang-x64`    | Linux, Clang, x86_64  |
| `darwin-gcc-arm64`   | macOS, GCC, arm64     |
| `darwin-clang-arm64` | macOS, Clang, arm64   |
| `windows-msvc-x64`   | Windows, MSVC, x86_64 |

Configure and build output directories follow `CMakePresets.json` (typically `out/build/<preset>` and `out/install/<preset>`).

## Languages & Toolchains

| Language   | Version / Toolchain                                    | Primary Location                                     |
| ---------- | ------------------------------------------------------ | ---------------------------------------------------- |
| **C++**    | C++23, GCC 14+ or Clang 20+, MSVC where preset applies | `src/basjoo/`                                         |
| **CMake**  | 3.31+                                                  | `CMakeLists.txt`, `cmake/`                           |
| **xmake**  | xmake 2.9+ (see `set_xmakever`)                        | `xmake.lua`, `xmake/`                                |
| **Python** | 3.14+ via **uv**, bindings via **nanobind**             | `src/basjoo/`                                          |

## Code Formatting

- **C++**: **clang-format 20+**, **Microsoft** base style (see `.clang-format`). Format changed files before completing work.
- **Python**: **ruff** for lint + format; **pyright** for type checking.
- **CMake**: follow project conventions and keep lists readable; no separate clang-format requirement unless configured.

## Repository Structure

```
basjoo/
├── src/basjoo/           # Library sources (headers + occasional .cpp) + Python package/nanobind `_core`
│   ├── common/          # Utilities, allocators, logging, macros
│   ├── math/            # dense, sparse, curves, functions, mdfunctions
│   ├── cvxopm/          # Convex optimization (problems, solvers)
│   └── bicycle/         # Motion planning models
├── tests/basjoo/         # doctest-based tests (mirror of src layout)
├── cmake/               # Toolchains, utils.cmake (basjoo_cxx_* helpers), third_party hooks
├── xmake/               # xmake rules and third-party recipes
├── deps/                # CPM/xrepo third-party package glue (incl. nanobind)
├── out/                 # Build trees (CMake/xmake; typically gitignored)
├── CMakePresets.json
├── CMakeLists.txt
├── xmake.lua
├── pyproject.toml       # Python project metadata (uv, scikit-build-core default)
├── pyproject.xmake.toml # Alternate: same package via the xmake-python backend
└── vcpkg/               # Vendored vcpkg (large)
```

## Module Architecture

Dependency direction (no cycles):

```
common → math → cvxopm → bicycle
```

- **common**: FSM helpers, allocators, logging (e.g. spdlog), shared macros.
- **math**: Vectors, matrices, sparse formats, curves, piecewise functions, multi-dimensional functions.
- **cvxopm**: QP and related convex optimization (OSQP, L-BFGS, etc., per `cmake/` / CPM).
- **bicycle**: Bicycle-model based motion planning models.

Library code is mostly header-only with fine-grained CMake (and xmake) targets per component.

## Serialization

Types opt into binary (de)serialization via **zpp_bits** (source-introduced via CPM/xmake, not vcpkg; see `deps/zpp_bits/`), not Boost.Serialization. Prefer the declarative form — `friend zpp::bits::access; using serialize = zpp::bits::members<N>;` — for private members; public-only members can skip the friend declaration entirely. For asymmetric cases (computed/derived fields reconstructed via a factory on deserialize) or deleted move/copy constructors, write an explicit `serialize(auto& archive, ...)` hook instead: a **free function** (found via ADL, defined in the same namespace as the type) when every member is public, otherwise a `static constexpr auto serialize(auto& archive, auto& self) -> zpp::bits::errc { ... }` member. Always cover serializable types with roundtrip tests (serialize → deserialize → equal, or approximate equality for floats).

For `pro::proxy<Facade>` type-erasure wrappers, which can't be modified directly: enable `pro::skills::rtti` on the facade (`pro::skills::rtti<typename pro::facade_builder::...>::build` — note the required `typename`); access `proxy_typeid`/`proxy_cast` only via `*self` (`pro::proxy` holds its accessor by composition, not inheritance); `proxy_cast<T>(...)` returns `T*`, so pass the candidate type itself, not a pointer to it. Maintain a closed, hand-written list of known concrete types directly in the proxy's own header, tagged with a `PascalCase` `enum class ...Tag : std::uint8_t` (`UPPER_SNAKE_CASE` enumerators, no `k` prefix; declaration order is the wire format — append-only, never reorder). Write two ADL `serialize` overloads — `const&` (output only) and `&` (both directions via `if constexpr` on `archive_type::kind()`) — duplicating the output logic inline in both rather than sharing a helper. Round-trip tests should assert the reconstructed concrete type matches (`proxy_typeid`) in addition to value equality. See `src/basjoo/math/curves/curve_proxy.hpp` for the reference shape.

When a type holds a container whose element type comes from a trait keyed on its own template parameter (e.g. `DenseGenerateTraitT<T>` / `DenseDegenerateTraitT<T>`), `#include` the full definition of every trait specialization reachable under whatever concept constrains `T` — trait headers typically only forward-declare their targets, and relying on some other translation unit to happen to provide the missing header compiles by accident and breaks the moment the type is used from a context that doesn't.

## Namespace Structure

| Namespace             | Role                                |
| --------------------- | ----------------------------------- |
| `basjoo`               | Root                                |
| `basjoo::common`       | Shared utilities                    |
| `basjoo::math`         | Core math types and algorithms      |
| `basjoo::math::pmr`    | Polymorphic allocator variants      |
| `basjoo::math::detail` | Internal implementation details     |
| `basjoo::cvxopm`       | Optimization interfaces and solvers |
| `basjoo::bicycle`      | Kinematic / planning models         |

**Never** use `using namespace` at namespace scope in headers. Close namespaces with a trailing comment (e.g. `} // namespace basjoo::math`).

## Development Workflow

1. **Implement** functionality in the appropriate module under `src/basjoo/` (C++ sources and Python bindings live together).
2. **Add or update tests** under `tests/basjoo/` (doctest for C++; pytest for Python).
3. **Build** with `cmake --preset ...` and `cmake --build --preset ...`, or xmake. Run `uv venv --system-site-packages` and then `uv sync --no-install-project` first so `.venv` has nanobind for the CMake/xmake Python discovery.
4. **Run tests**: `ctest --preset ...` (or xmake test targets when using xmake); `uv run pytest` for Python.
5. **Format**: `clang-format -i` on touched C++ files; `ruff format` on touched Python files.
6. **Lint**: `clang-tidy` on C++; `ruff check` + `pyright` on Python.
7. **Do not** add READMEs, demos, or extra docs unless explicitly requested.

## Copyright Header

C++ headers and sources use a **Doxygen file block** at the top:

```cpp
/**
 * @file filename.hpp
 * @brief One-line description.
 * @copyright Copyright (c) YYYY basjoo development team
 */
```

Python:

```python
# Copyright (c) YYYY basjoo development team. All rights reserved.
```

CMake scripts and `.mdc` rule files do not use this header where YAML frontmatter or CMake policy must be first line.

## TODO Format

```
// TODO(github_username): description [optional issue or PR reference]
```

Use `//` in C++ and `#` in Python. Include a real GitHub username in parentheses.

## File Creation Policy

- **Do not** create demo programs, sample apps, unsolicited README files, or documentation files unless the user explicitly asks.
- **Do not** add dependencies without using the project’s workflow (**CPM** / **vcpkg** for C++; **uv** for Python).
- Keep changes focused on the requested task.

## Security

- **Never** hardcode API keys, tokens, or passwords in source or committed config.
- **Never** commit `.env` files with real credentials.
- Prefer environment variables or secret managers for sensitive values.

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
