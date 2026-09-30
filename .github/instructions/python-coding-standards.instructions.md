---
applyTo: "**/*.py,**/*.pyi"
---
# Python coding standards

## Tools

- **ruff** — lint and format (see `pyproject.toml`).
- **pyright** — static typing.
- **uv** — environments and dependency locking.

## Style

- PEP 8 baseline; let ruff enforce project rules.
- Type hints on all public functions and methods.
- Imports: standard library, blank line, third party, blank line, local packages.

## Package layout

- Avoid gratuitous `__init__.py`; prefer explicit imports from submodules.

## Docstrings

Add only when behavior is not obvious from names and types.

## Dependencies

Use `uv add` and commit lockfile updates when the repo tracks `uv.lock`.
