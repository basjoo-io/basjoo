# Build and Test

## CMake presets

| Preset | Platform / toolchain |
| ------ | -------------------- |
| `linux-gcc-x64` | Linux, GCC, x86_64 |
| `linux-clang-x64` | Linux, Clang, x86_64 |
| `darwin-gcc-arm64` | macOS, GCC, arm64 |
| `darwin-clang-arm64` | macOS, Clang, arm64 |
| `windows-msvc-x64` | Windows, MSVC, x86_64 |

## Common commands (CMake)

```bash
cmake --preset <preset>
cmake --build --preset <preset>
ctest --preset <preset>
ctest --preset <preset> -R <regex>
clang-format -i <files>
clang-tidy -p out/build/<preset> <file>
```

`CMakePresets.json` sets:

- **Generator**: Ninja
- **Binary directory**: `out/build/${presetName}`
- **Install directory**: `out/install/${presetName}`
- **`CMAKE_EXPORT_COMPILE_COMMANDS`**: `ON` for IDE / clang-tidy integration

## xmake

```bash
xmake config -m release
xmake build
```

xmake uses `out/build` as configured in `xmake.lua` (see `set_config("builddir", ...)`).

## Python (when enabled)

```bash
# --system-site-packages: the devcontainer's ROS pytest plugins (autoloaded via
# PYTHONPATH) import packages from /usr/lib/python3/dist-packages, invisible to
# a plain uv-created venv.
uv venv --system-site-packages
uv sync --no-install-project
uv run pytest
ruff check .
ruff format .
pyright
```

## Key CMake cache options

| Option | Role |
| ------ | ---- |
| `BASJOO_CHECK_PARAMS` | Extra parameter validation |
| `BASJOO_BUILD_TESTING` | Build and register tests |
| `BASJOO_ENABLE_INSTALL` | Export install rules |
| `BASJOO_USE_BLAS_LAPACK` | Select BLAS/LAPACK provider |
| `BASJOO_USE_BOOST_UNORDERED` | Boost unordered containers |

## CMake helpers (`cmake/utils.cmake`)

| Function | Purpose |
| -------- | ------- |
| `basjoo_cxx_library` | Headers + optional sources → static/interface library; defines `basjoo::NAME` |
| `basjoo_cxx_module` | C++ modules (`.ixx`) when used |
| `basjoo_cxx_test` | doctest executable + `add_test` registration |

CPM dependency blocks should remain centralized—follow existing includes in the root CMake flow.

Workflow steps (implement → test → format → lint) live in `08-workflow.md`.
