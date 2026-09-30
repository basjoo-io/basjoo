<h1 align="center">basjoo</h1>

<p align="center">
  <b>The fundamental math basis for autonomous driving vehicles and robotics.</b>
</p>

<p align="center">
  English | <a href="README.zh-CN.md">简体中文</a>
</p>

<p align="center">
  <a href="https://github.com/basjoo-io/basjoo/actions/workflows/ubuntu-x64.yml"><img src="https://github.com/basjoo-io/basjoo/actions/workflows/ubuntu-x64.yml/badge.svg" alt="ubuntu-x64"></a>
  <a href="https://github.com/basjoo-io/basjoo/actions/workflows/darwin-arm64.yml"><img src="https://github.com/basjoo-io/basjoo/actions/workflows/darwin-arm64.yml/badge.svg" alt="darwin-arm64"></a>
  <a href="https://github.com/basjoo-io/basjoo/actions/workflows/windows-x64.yml"><img src="https://github.com/basjoo-io/basjoo/actions/workflows/windows-x64.yml/badge.svg" alt="windows-x64"></a>
  <img src="https://img.shields.io/badge/C%2B%2B-23-blue.svg" alt="C++23">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-BSD--3--Clause-blue.svg" alt="BSD-3-Clause"></a>
</p>

---

basjoo is a header-mostly **C++23** library with the math that planning and control stacks are built on: splines for paths, convex optimization for trajectory smoothing, bicycle models for motion planning, and the linear algebra under all of it. Every component is a fine-grained CMake/xmake target (`basjoo::math_vec2`, `basjoo::cvxopm_osqp_solver`, …), so you link only what you use.

## Modules

| Module    | Namespace         | What's inside                                                                                          |
| --------- | ----------------- | ------------------------------------------------------------------------------------------------------ |
| `common`  | `basjoo::common`  | FSM, spdlog-based logging, aligned allocators and PMR resources, timing and RAII helpers               |
| `math`    | `basjoo::math`    | Dense `Vec2`/`Vec3`/`Vector`/`Matrix` with LU/QR (optional BLAS/LAPACK, AVX-512); sparse DOK/LIL/COO/CSR/CSC with QDLDL; linear/cubic/quintic curves and functions; multivariate functions; FFT, Chebyshev, fixed-point, polygons |
| `cvxopm`  | `basjoo::cvxopm`  | Sparse QP via OSQP; L-BFGS, BFGS, Nelder–Mead, and line-search solvers for dense problems              |
| `bicycle` | `basjoo::bicycle` | Route lines, paths, motions, borders, fences; QP-based acceleration/offset/kinematics planners; kinematics/dynamics models; PID, feed-forward, MPC, and flow controllers |

Dependencies flow strictly `common → math → cvxopm → bicycle`. Curves, functions, and sparse matrices are type-erased through [Microsoft Proxy](https://github.com/microsoft/proxy) and serializable with [zpp_bits](https://github.com/eyalz800/zpp_bits).

## Getting started

The easiest environment is the devcontainer (`.devcontainer/`: GCC 16, Clang 22, CMake, xmake, uv). On your own machine you need CMake ≥ 3.31, Ninja, a C++23 compiler with `<mdspan>` (GCC ≥ 16, Clang ≥ 18 with libc++, or MSVC ≥ 19.39), Python ≥ 3.14 with [uv](https://docs.astral.sh/uv/), and optionally ccache. Every C++ dependency is fetched by [CPM](https://github.com/cpm-cmake/CPM.cmake).

```bash
git clone --recursive https://github.com/basjoo-io/basjoo.git && cd basjoo

# Python environment first: CMake/xmake locate nanobind in .venv
uv venv --system-site-packages
uv sync --no-install-project

# Build and test with a preset (linux-gcc-x64, linux-clang-x64, darwin-{gcc,clang}-arm64, windows-msvc-x64)
cmake --preset linux-gcc-x64
cmake --build --preset linux-gcc-x64
ctest --preset linux-gcc-x64

# ...or with xmake
xmake f -m release && xmake build && xmake test
```

`--system-site-packages` is required in the devcontainer, where ROS 2 pytest plugins import system-installed packages.

### Options

| CMake                       | xmake                        | Default    | Description                                       |
| --------------------------- | ---------------------------- | ---------- | ------------------------------------------------- |
| `BASJOO_BUILD_TESTING`       | `basjoo_build_testing`        | `ON`       | Build unit tests                                  |
| `BASJOO_CHECK_PARAMS`        | `basjoo_check_params`         | `OFF`      | Runtime parameter validation                      |
| `BASJOO_USE_BLAS_LAPACK`     | `basjoo_use_blas_lapack`      | `OpenBLAS` | `OpenBLAS`, `Netlib`, `MKL`, or `OFF`             |
| `BASJOO_USE_BOOST_UNORDERED` | `basjoo_use_boost_unordered`  | `ON`       | Use Boost.Unordered containers                    |
| `BASJOO_ENABLE_INSTALL`      | `basjoo_enable_install`       | `ON`       | Install and export rules                          |
| `BUILD_SHARED_LIBS`         | `--kind=shared`              | `OFF`      | Build shared libraries                            |

### Use from CMake

```bash
cmake --install out/build/linux-gcc-x64 --prefix /your/prefix
```

```cmake
find_package(basjoo REQUIRED)
target_link_libraries(your_target PRIVATE basjoo::math_piecewise_cubic_curve basjoo::cvxopm_osqp_solver)
```

## Examples

### Cubic spline curve

```cpp
#include "basjoo/math/curves/piecewise_cubic_curve.hpp"
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/functions/boundary_mode.hpp"

using namespace basjoo::math;

// 21 anchor points on a semicircle of radius 2
std::vector<Vec2d> anchors(21);
for (std::size_t i = 0; i < anchors.size(); ++i) {
    const double t = M_PI * i / 20.0;
    anchors[i] = Vec2d{2.0 * std::cos(t), 2.0 * std::sin(t)};
}

// Clamped boundary conditions (first derivative at both ends)
const BoundaryMode<Vec2d> b0{.order = 1, .derivative = Vec2d{0.0, 1.0}};
const BoundaryMode<Vec2d> bf{.order = 1, .derivative = Vec2d{0.0, -1.0}};
const PiecewiseCubicCurve<Vec2d> curve{anchors, b0, bf};

const Vec2d tangent = curve.tangent(1.0);
const double kappa  = curve.curvature(1.0);         // ≈ 0.5
const auto [s, l]   = curve.inverse(Vec2d{1.0, 1.0}); // Cartesian → Frenet
```

### Quadratic program (OSQP)

```cpp
#include "basjoo/cvxopm/problems/osqp_problem.hpp"
#include "basjoo/cvxopm/solvers/osqp_solver.hpp"

using namespace basjoo::cvxopm;

// min 2x₀² + x₁² + x₀x₁ + x₀ + x₁   s.t.  x₀ + x₁ = 1,  0 ≤ x₀, x₁ ≤ 0.7
OsqpProblem<double, int> problem(2, 3);
problem.updateQuadCostTerm(0, 0, 2.0);
problem.updateQuadCostTerm(1, 1, 1.0);
problem.updateQuadCostTerm(0, 1, 1.0);
problem.updateLinCostTerm(0, 1.0);
problem.updateLinCostTerm(1, 1.0);
problem.updateConstraintTerm(0, {{0, 1.0}, {1, 1.0}}, 1.0, 1.0);
problem.updateConstraintTerm(1, {{0, 1.0}}, 0.0, 0.7);
problem.updateConstraintTerm(2, {{1, 1.0}}, 0.0, 0.7);

const OsqpSolver<double, int> solver{Settings<double, int>{.polishing = true}};
const auto [result, info] = solver.solve(problem); // result.prim_vars ≈ {0.3, 0.7}
```

### Unconstrained minimization (L-BFGS)

```cpp
#include "basjoo/cvxopm/problems/dense_problem.hpp"
#include "basjoo/cvxopm/solvers/lbfgs_solver.hpp"
#include "basjoo/math/mdfunctions/rosenbrock_function.hpp"

using namespace basjoo;

const cvxopm::DenseProblem<double> problem(
    math::makeMdFunctionProxy(math::RosenbrockFunction<math::pmr::Vector<double>>(1.0, 100.0))
);
const cvxopm::LbfgsSolver<double> solver{cvxopm::Settings<double>{.eps_abs{1E-6}}};
const auto [result, info] = solver.solve(problem, math::pmr::Vector<double>({-1.0, 1.0}));
// result.prim_vars ≈ {1.0, 1.0}
```

More usage lives in the tests under [`tests/basjoo/`](tests/basjoo), which mirror the source tree.

## Python

`src/basjoo/` is also a Python package with a nanobind extension (`basjoo._core`), buildable as a wheel with scikit-build-core (`pyproject.toml`) or xmake-python (`pyproject.xmake.toml`). The bindings are scaffolding for now and do not expose an API yet.

## Contributing

Conventions for humans and coding agents are in [`AGENTS.md`](AGENTS.md) and [`.claude/rules/`](.claude/rules). Every change keeps CMake and xmake in sync, adds tests, and is formatted with `clang-format` / `ruff format`.

## License

[BSD 3-Clause](LICENSE).

basjoo builds on
[Boost](https://www.boost.org/),
[cxxopts](https://github.com/jarro2783/cxxopts),
[doctest](https://github.com/doctest/doctest),
[Matplot++](https://github.com/alandefreitas/matplotplusplus),
[Microsoft Proxy](https://github.com/microsoft/proxy),
[nanobind](https://github.com/wjakob/nanobind),
[OpenBLAS](https://github.com/OpenMathLib/OpenBLAS),
[OSQP](https://osqp.org/),
[pocketfft](https://github.com/mreineck/pocketfft),
[qdldl](https://github.com/osqp/qdldl),
[spdlog](https://github.com/gabime/spdlog),
[stdexec](https://github.com/NVIDIA/stdexec),
[Taskflow](https://github.com/taskflow/taskflow), and
[zpp_bits](https://github.com/eyalz800/zpp_bits) — thank you to their authors.
