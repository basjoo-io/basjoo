# Project Overview

basjoo is a **high-performance C++23** mathematical library for autonomous driving and robotics, distributed under the **BSD 3-Clause** license. It emphasizes dense and sparse linear algebra, piecewise curves and scalar/vector functions, convex optimization solvers, and bicycle dynamics models.

Python bindings live under `python/basjoo/`, built via **nanobind** and packaged/tested with **uv** (`pyproject.toml`, `uv.lock`; lint/format via **ruff**, tests via **pytest**). The `_core` extension module is compiled from CMake or xmake against the interpreter in `.venv/`; run `uv venv --system-site-packages` then `uv sync --no-install-project` before a plain CMake/xmake build so nanobind is present.

## Module architecture

Dependency flow (strictly acyclic):

```
common → math → cvxopm → bicycle
```

- **common** — Shared utilities: finite-state helpers, allocators, logging (e.g. spdlog), macros (`BASJOO_*`), and low-level helpers.
- **math** — `dense`, `sparse`, `curves`, `functions`, `mdfunctions`, and related CMake/xmake targets.
- **cvxopm** — Convex optimization: problem formulations and solvers (OSQP, L-BFGS, etc., per dependencies).
- **bicycle** — Bicycle-model based motion planning models.

Implementation is predominantly **header-first** with fine-grained targets per class or small component.

## Dependencies

C++ dependencies are managed with **CPM** and **vcpkg** (vendored). Typical libraries include Boost (unordered), **zpp_bits** (serialization, source-introduced via CPM/xmake — see `deps/zpp_bits/`), doctest, spdlog, OSQP/qdldl, OpenBLAS, and test helpers (cxxopts, Matplot++). Exact pins live under `cmake/` and `xmake/third_party/`.

## Library design principles

- **Namespaces** partition concepts: `basjoo::math`, `basjoo::cvxopm`, `basjoo::bicycle`, `basjoo::common`, with `detail` and `pmr` sub-namespaces where appropriate.
- **No circular** link dependencies between modules.
- **Exceptions** communicate misuse or numerical failure; optional **`BASJOO_CHECK_PARAMS`** adds inexpensive checks in debug-oriented builds.
- **Serialization**: **zpp_bits** where types opt in (declarative `using serialize = zpp::bits::members<N>;`, or an explicit `static constexpr auto serialize(auto& archive, auto& self) -> zpp::bits::errc` for asymmetric cases)—always cover with roundtrip tests.

For command-line workflows and directory layout, see `02-build-and-test.md` and repository `AGENTS.md`.
