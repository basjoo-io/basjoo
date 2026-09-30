# Python Conventions

## Style

- **PEP 8** baseline; **ruff** enforces project-specific rules via `pyproject.toml`.
- **Type hints** on all public functions and methods.
- Imports grouped: stdlib → third party → local.

## Tools

| Tool | Role |
| ---- | ---- |
| `uv` | Environment and lockfile |
| `ruff` | Lint + format |
| `pyright` | Static typing |
| `ty` | Static typing (secondary checker) |
| `pytest` | Tests (`uv run pytest`) |
| `nanobind` | C++/Python bindings for the `_core` extension |

## Modules

- `src/basjoo/` holds both the C++ headers and the Python package: `__init__.py`, pure-Python modules, and the nanobind `_core` extension (`_core.cpp` + generated `_core.pyi`/`py.typed`). `xmake-python`'s wheel builder only recognizes `<name>/` or `src/<name>/` package layouts, which is why C++ and Python live in the same directory rather than split.
- Avoid empty `__init__.py` unless packaging requires it; prefer explicit imports.

## Build backend

- **Default**: `pyproject.toml` uses **scikit-build-core** driving the CMake presets (`cmake.args = ["--preset=..."]` per platform in `[[tool.scikit-build.overrides]]`).
- **Alternate**: `pyproject.xmake.toml` configures the same package via the **xmake-python** PEP 517 backend instead. Swap it in with `cp pyproject.xmake.toml pyproject.toml && uv venv --system-site-packages && uv sync --no-install-project`; the shared `[project]`/`[tool.ruff]`/`[tool.pyright]`/etc. sections are kept in sync by hand between the two files.
- xmake toolchain selection lives in `xmake.lua` itself (`is_host(...)` branches per platform, overridable with `xmake f --toolchain=...`), not in `pyproject.xmake.toml`.
- **Known limitation**: xmake-python's packaging step always runs a bare `xmake` build (no way to scope it to just the `_core` target), so it also builds every other default-enabled C++ target in the project (`math`, `cvxopm`, `bicycle`, `common`). This is unrelated to the Python bindings but means the xmake-python path only succeeds on a toolchain that can build the whole project (matches the devcontainer's GCC 16 / Clang 22, not necessarily an arbitrary host GCC).

## Sanitizers (Debug / RelWithDebInfo / releasedbg)

- On Linux/macOS (GCC/Clang), `_core` and its bundled `nanobind`/`nanobind-static` object files are always compiled with an explicit `-fno-sanitize=address,undefined` (CMake `target_compile_options`/`target_link_options`, guarded by `$<NOT:$<CXX_COMPILER_ID:MSVC>>`; xmake `add_cxflags`/`add_shflags` inside `on_config`, guarded by `not target:has_tool("cxx", "cl")`, both with `{force = true}`), overriding whatever `-fsanitize=` the active Debug/RelWithDebInfo/releasedbg toolchain flags would otherwise apply project-wide. `_core` therefore never needs a sanitizer runtime preloaded, and `uv run pytest` / `import _core` work the same way in every build type with no extra setup. Everything else (doctest C++ test binaries, the CUDA/C++ libraries themselves) still gets ASan/UBSan normally — only the Python extension module is exempted.
- **Windows/MSVC has no ASan/UBSan coverage at all, by design**, in any build type — the toolchain files never set `/fsanitize=address`. This is a deliberate project decision (not just a `_core` exemption): MSVC's sanitizer support was never actually validated against a real Windows environment in this project's history and was a persistent source of friction (nanobind skips its preload logic entirely on Windows; the DLL/PATH story differs fundamentally from Linux's `LD_PRELOAD`), so the `-fno-sanitize=` guards above exist only to keep `_core`'s CMakeLists.txt/xmake.lua from passing GCC/Clang-only flag syntax to `cl.exe` should MSVC ever gain sanitizer flags again — they're not currently exempting anything, since there's nothing enabled to exempt from.

## Tests

- **Arrange / Act / Assert** structure
- **`@pytest.fixture`** for reusable setup
- **`@pytest.mark.parametrize`** for data-driven cases

## Adding dependencies

```bash
uv add package_name
uv lock
```

See `.github/instructions/python-coding-standards.instructions.md` for more detail.
