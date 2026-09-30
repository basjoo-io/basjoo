---
paths:
  - "src/**/*.hpp"
  - "src/**/*.cpp"
  - "src/**/*.cuh"
  - "src/**/*.cu"
  - "tests/**/*.cpp"
---

# C++ Conventions

## Naming

| Element                     | Style                | Example                  |
| --------------------------- | -------------------- | ------------------------ |
| Namespace                   | `snake_case`         | `basjoo::detail`            |
| Class / struct / enum class | `PascalCase`         | `AlignedAllocator`       |
| Function / method           | `camelCase`          | `newtonRaphson`          |
| Public member               | `snake_case`         | `settings`               |
| Private member              | `m_` + `snake_case`  | `m_workspace`            |
| Constant                    | `kPascalCase`        | `kEpsilon`               |
| Enumerator                  | `UPPER_SNAKE_CASE`   | `ROW_MAJOR`              |
| Template parameter          | `PascalCase`         | `Scalar`, `Index`        |
| Local type alias            | `snake_case`         | `archive_type`           |
| Macro                       | `BASJOO_UPPER_SNAKE`    | `BASJOO_LOG_LEVEL`          |

## Declarations

- **Trailing return types** on all new functions: `auto norm(const Vec& v) noexcept -> double;`
- **Requires clauses** go on their own line directly after the template parameter list, never after the declarator. A trailing `requires` is only for code without a template head (non-template members of class templates, abbreviated function templates). Never add a dummy `template <typename = void>` to move a clause; append extra conditions to an existing head clause with `&&`.
- Prefer an abbreviated constrained parameter (`std::sentinel_for<It> auto last`) when the parameter is named once and constrained by one existing concept; put further conditions in a trailing `requires` via `decltype`. Don't invent a concept just for this.
- `final` unless designed for inheritance; `explicit` single-argument constructors unless conversion is intended; `override` on every override; `struct` for passive data, `class` for invariants.
- `noexcept` when failure is not modeled; exceptions signal misuse or numerical failure.
- `constexpr` / `if constexpr` and concepts over runtime checks and SFINAE.
- `std::span` / `std::string_view` for non-owning parameters; don't store them in objects unless the lifetime is explicit.

## Headers and namespaces

- `#pragma once`. Include order: the header under implementation/test → standard library → third-party → `"basjoo/..."` (always the full project path).
- Never `using namespace` at namespace scope in a header. Close namespaces with a comment: `} // namespace basjoo::detail`.
- When a type's members depend on a trait specialization (e.g. a container whose element type comes from a trait on `T`), include the **full definition** of every specialization reachable under `T`'s constraint. Trait headers usually only forward-declare; relying on another TU's include compiles by accident.

## Comments

- New files start with the Doxygen file header shown in `AGENTS.md`.
- Line comments only for non-obvious rationale, units, or tolerances.
- `// TODO(github_username): description [optional #issue]` — real username, removed once resolved.

## Tooling

- `clang-format -i` (Microsoft base style, 100 columns, repository `.clang-format`) on every touched file.
- `clang-tidy -p out/build/<preset> <file>` for static analysis.
