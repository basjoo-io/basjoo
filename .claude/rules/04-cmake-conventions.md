# CMake Conventions

## Helper functions

Defined in `cmake/utils.cmake`:

### `basjoo_cxx_library`

- **NAME**: CMake target name (also `basjoo::NAME` alias).
- **HDRS** / **SRCS**: Headers always listed; sources optional (interface library if only headers).
- **DEPS**: `PUBLIC` link libraries.
- Optional: `COPTS`, `DEFINES`, `LINKOPTS`, `PUBLIC`, `TESTONLY`, `DISABLE_INSTALL`.

### `basjoo_cxx_module`

- For C++ module interfaces: **IXXS** file set, plus **SRCS**, **DEPS**, etc.

### `basjoo_cxx_test`

- Builds a test executable when `BASJOO_BUILD_TESTING` is enabled, links doctest (and project-provided test deps), and registers `add_test`.

### Keeping `DEPS` in sync with `#include`

Whenever a header gains a new `#include` of another module's header (e.g. adding a transitive completeness `#include` or a new third-party usage), add the matching library to **both** `DEPS`/`add_packages` in `CMakeLists.txt` **and** `add_deps`/`add_packages` in the corresponding `xmake.lua`. A stale `DEPS` list still compiles locally via already-resolved transitive includes and only surfaces on a clean reconfigure or in a different consumer — don't rely on the build "still passing" as proof the dependency list is correct.

## CPM

- Third-party packages are fetched via **CPM** with pinned tags/commits.
- Keep dependency logic in dedicated `.cmake` fragments—do not scatter `CPMAddPackage` calls across leaf `CMakeLists.txt` unless the project already does so for that module.

## Presets

- Developers should use **`cmake --preset`** workflows from `CMakePresets.json`.
- Toolchain files live in `cmake/toolchains/`.

## Install

- Guard packaging rules with **`BASJOO_ENABLE_INSTALL`** to support CI and local dev without install.
