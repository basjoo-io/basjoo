---
paths:
  - "**/CMakeLists.txt"
  - "**/*.cmake"
  - "**/xmake.lua"
  - "xmake/**"
  - "deps/**"
  - "CMakePresets.json"
  - ".github/workflows/**"
  - ".devcontainer/**"
---

# Build System (CMake + xmake)

CMake (with presets) is primary, xmake is a full alternative: **every target, option, and flag exists in both, and changes land in both together.**

## Targets

| CMake helper (`cmake/utils.cmake`) | Use                                                                           |
| ---------------------------------- | ----------------------------------------------------------------------------- |
| `basjoo_cxx_library`                  | `NAME`, `HDRS`, optional `SRCS` (interface library if none), `DEPS` (public), `COPTS`, `DEFINES`, `LINKOPTS`; flags `PUBLIC`, `TESTONLY`, `DISABLE_INSTALL`. Creates the `basjoo::NAME` alias. |
| `basjoo_cxx_module`                   | C++ module interfaces (`IXXS`) plus `SRCS`, `DEPS`.                           |
| `basjoo_cxx_test`                     | doctest executable linked with doctest/cxxopts/Matplot++ and `DEPS`; registered with CTest when `BASJOO_BUILD_TESTING` is on. |

- xmake mirrors each target with the same name; tests use the `basjoo.cxx.test` rule (`xmake/rules.lua`).
- Each xmake target adds its own include root relative to its `xmake.lua` (e.g. `add_includedirs("../..")`); there is no project-wide `add_includedirs`.
- xmake options are the lowercase twins of the CMake ones (`BASJOO_FOO` ↔ `basjoo_foo`).

## Dependencies

- Third-party C++ packages live in one fragment per package under `deps/<package>/`: `CMakeLists.txt` (`CPMAddPackage` with a pinned URL + hash) and `xmake.lua` (xrepo `package(...)` / `add_requires`). Never call `CPMAddPackage` from leaf `CMakeLists.txt`. Sources cache in `third_party/` (`CPM_SOURCE_CACHE`).
- **Keep `DEPS` in sync with `#include`.** When a header starts including another target's header or a new third-party library, add it to `DEPS`/`add_packages` in `CMakeLists.txt` **and** `add_deps`/`add_packages` in `xmake.lua`. A stale list often still builds through transitive includes and only breaks on a clean configure or in another consumer.

## Toolchains and presets

- Presets: `linux-gcc-x64`, `linux-clang-x64`, `darwin-gcc-arm64`, `darwin-clang-arm64`, `windows-msvc-x64`; build trees in `out/build/<preset>`, xmake uses `build/`.
- Compiler flags live only in `cmake/toolchains/<preset>.cmake` and `xmake/toolchains/<preset>.lua`, kept identical (same flags, same order). Release modes build with `-O2` in both (xmake pins `set_optimize("faster")`).
- Debug / RelWithDebInfo (xmake: debug / releasedbg) enable ASan + UBSan on GCC/Clang; MSVC has no sanitizer coverage by design.
- Install rules are guarded by `BASJOO_ENABLE_INSTALL`.

## CI and container

- `.github/workflows/{ubuntu-x64,darwin-arm64,windows-x64}.yml` each build and test with **both** CMake and xmake. Matrix dimensions mirror the project's CMake options; when adding an option, add it to the matrix, the job name, the ccache key, and both configure steps.
- `clang-format-check.yml` enforces formatting.
- `.devcontainer/` pins compiler, CMake, and xmake versions through `devcontainer.json` build args; the image tag encodes them, so bump the tag when they change.
