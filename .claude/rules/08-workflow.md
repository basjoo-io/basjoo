# Development Workflow

## Steps

1. **Implement** — Work in the correct module (`common`, `math`, `cvxopm`, `bicycle`) respecting dependency order.
2. **Test** — Add doctest coverage under `tests/basjoo/`; for future Python, use `uv run pytest`.
3. **Format** — `clang-format -i` on touched C++ files; `ruff format` / `ruff check --fix` on Python.
4. **Lint** — `clang-tidy` with compile commands from the active preset; `pyright` on Python.
5. **Done** — All relevant tests pass locally.

## CMake quick path

```bash
cmake --preset linux-gcc-x64
cmake --build --preset linux-gcc-x64
ctest --preset linux-gcc-x64
```

## xmake quick path

```bash
xmake build
```

## File creation rules

- Do **not** add demos, examples, or unsolicited documentation files.
- Do **not** add dependencies without going through **CPM**/**vcpkg** (C++) or **uv** (Python).

## Documentation policy

- No new markdown guides unless explicitly requested by the user or maintainers.

## Python workflow (future)

Create the venv with `--system-site-packages` first: the devcontainer's ROS pytest plugins import packages from `/usr/lib/python3/dist-packages`.

```bash
uv venv --system-site-packages
uv sync --no-install-project
uv run pytest
uv run ruff check .
uv run pyright
```
