<h1 align="center">basjoo</h1>

<p align="center">
  <b>面向自动驾驶与机器人的基础数学库。</b>
</p>

<p align="center">
  <a href="README.md">English</a> | 简体中文
</p>

<p align="center">
  <a href="https://github.com/basjoo-io/basjoo/actions/workflows/ubuntu-x64.yml"><img src="https://github.com/basjoo-io/basjoo/actions/workflows/ubuntu-x64.yml/badge.svg" alt="ubuntu-x64"></a>
  <a href="https://github.com/basjoo-io/basjoo/actions/workflows/darwin-arm64.yml"><img src="https://github.com/basjoo-io/basjoo/actions/workflows/darwin-arm64.yml/badge.svg" alt="darwin-arm64"></a>
  <a href="https://github.com/basjoo-io/basjoo/actions/workflows/windows-x64.yml"><img src="https://github.com/basjoo-io/basjoo/actions/workflows/windows-x64.yml/badge.svg" alt="windows-x64"></a>
  <img src="https://img.shields.io/badge/C%2B%2B-23-blue.svg" alt="C++23">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-BSD--3--Clause-blue.svg" alt="BSD-3-Clause"></a>
</p>

---

basjoo 是一个以头文件为主的 **C++23** 数学库，提供规划与控制系统所依赖的数学基础：用于路径表示的样条、用于轨迹平滑的凸优化、用于运动规划的自行车模型，以及支撑这一切的线性代数。每个组件都是细粒度的 CMake/xmake 目标（`basjoo::math_vec2`、`basjoo::cvxopm_osqp_solver` 等），按需链接即可。

## 模块

| 模块      | 命名空间          | 内容                                                                                                   |
| --------- | ----------------- | ------------------------------------------------------------------------------------------------------ |
| `common`  | `basjoo::common`  | 有限状态机、基于 spdlog 的日志、对齐分配器与 PMR 内存资源、计时与 RAII 工具                            |
| `math`    | `basjoo::math`    | 稠密 `Vec2`/`Vec3`/`Vector`/`Matrix` 及 LU/QR 分解（可选 BLAS/LAPACK、AVX-512）；稀疏 DOK/LIL/COO/CSR/CSC 及 QDLDL；线性/三次/五次曲线与函数；多元函数；FFT、Chebyshev、定点数、多边形 |
| `cvxopm`  | `basjoo::cvxopm`  | 基于 OSQP 的稀疏二次规划；面向稠密问题的 L-BFGS、BFGS、Nelder–Mead 与线搜索求解器                      |
| `bicycle` | `basjoo::bicycle` | 参考线、路径、运动、边界、围栏；基于 QP 的加速度/横向偏移/运动学规划器；运动学/动力学模型；PID、前馈、MPC 与流控制器 |

模块依赖严格遵循 `common → math → cvxopm → bicycle`。曲线、函数与稀疏矩阵通过 [Microsoft Proxy](https://github.com/microsoft/proxy) 实现类型擦除，并可通过 [zpp_bits](https://github.com/eyalz800/zpp_bits) 序列化。

## 快速开始

最省事的环境是 devcontainer（`.devcontainer/`：GCC 16、Clang 22、CMake、xmake、uv）。在本机构建需要 CMake ≥ 3.31、Ninja、支持 `<mdspan>` 的 C++23 编译器（GCC ≥ 16、配合 libc++ 的 Clang ≥ 18，或 MSVC ≥ 19.39）、Python ≥ 3.14 与 [uv](https://docs.astral.sh/uv/)，ccache 可选。所有 C++ 依赖均由 [CPM](https://github.com/cpm-cmake/CPM.cmake) 自动获取。

```bash
git clone --recursive https://github.com/basjoo-io/basjoo.git && cd basjoo

# 先准备 Python 环境：CMake/xmake 需要从 .venv 中找到 nanobind
uv venv --system-site-packages
uv sync --no-install-project

# 使用预设构建与测试（linux-gcc-x64、linux-clang-x64、darwin-{gcc,clang}-arm64、windows-msvc-x64）
cmake --preset linux-gcc-x64
cmake --build --preset linux-gcc-x64
ctest --preset linux-gcc-x64

# ……或使用 xmake
xmake f -m release && xmake build && xmake test
```

devcontainer 中必须使用 `--system-site-packages`：ROS 2 的 pytest 插件会导入系统安装的包。

### 构建选项

| CMake                       | xmake                        | 默认值     | 说明                                    |
| --------------------------- | ---------------------------- | ---------- | --------------------------------------- |
| `BASJOO_BUILD_TESTING`       | `basjoo_build_testing`        | `ON`       | 构建单元测试                            |
| `BASJOO_CHECK_PARAMS`        | `basjoo_check_params`         | `OFF`      | 运行时参数校验                          |
| `BASJOO_USE_BLAS_LAPACK`     | `basjoo_use_blas_lapack`      | `OpenBLAS` | `OpenBLAS`、`Netlib`、`MKL` 或 `OFF`    |
| `BASJOO_USE_BOOST_UNORDERED` | `basjoo_use_boost_unordered`  | `ON`       | 使用 Boost.Unordered 容器               |
| `BASJOO_ENABLE_INSTALL`      | `basjoo_enable_install`       | `ON`       | 安装与导出规则                          |
| `BUILD_SHARED_LIBS`         | `--kind=shared`              | `OFF`      | 构建动态库                              |

### 在 CMake 项目中使用

```bash
cmake --install out/build/linux-gcc-x64 --prefix /your/prefix
```

```cmake
find_package(basjoo REQUIRED)
target_link_libraries(your_target PRIVATE basjoo::math_piecewise_cubic_curve basjoo::cvxopm_osqp_solver)
```

## 示例

### 三次样条曲线

```cpp
#include "basjoo/math/curves/piecewise_cubic_curve.hpp"
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/functions/boundary_mode.hpp"

using namespace basjoo::math;

// 半径为 2 的半圆上的 21 个锚点
std::vector<Vec2d> anchors(21);
for (std::size_t i = 0; i < anchors.size(); ++i) {
    const double t = M_PI * i / 20.0;
    anchors[i] = Vec2d{2.0 * std::cos(t), 2.0 * std::sin(t)};
}

// 两端一阶导数约束（clamped）
const BoundaryMode<Vec2d> b0{.order = 1, .derivative = Vec2d{0.0, 1.0}};
const BoundaryMode<Vec2d> bf{.order = 1, .derivative = Vec2d{0.0, -1.0}};
const PiecewiseCubicCurve<Vec2d> curve{anchors, b0, bf};

const Vec2d tangent = curve.tangent(1.0);
const double kappa  = curve.curvature(1.0);         // ≈ 0.5
const auto [s, l]   = curve.inverse(Vec2d{1.0, 1.0}); // 笛卡尔坐标 → Frenet 坐标
```

### 二次规划（OSQP）

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

### 无约束最小化（L-BFGS）

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

更多用法见 [`tests/basjoo/`](tests/basjoo) 下的测试，其目录结构与源码一一对应。

## Python

`src/basjoo/` 同时也是一个 Python 包，包含 nanobind 扩展（`basjoo._core`），可通过 scikit-build-core（`pyproject.toml`）或 xmake-python（`pyproject.xmake.toml`）构建 wheel。目前绑定仅为脚手架，尚未导出 API。

## 参与贡献

面向开发者与编码代理的约定见 [`AGENTS.md`](AGENTS.md) 与 [`.claude/rules/`](.claude/rules)。每次改动都需同步 CMake 与 xmake、补充测试，并使用 `clang-format` / `ruff format` 格式化。

## 许可证

[BSD 3-Clause](LICENSE)。

basjoo 构建于
[Boost](https://www.boost.org/)、
[cxxopts](https://github.com/jarro2783/cxxopts)、
[doctest](https://github.com/doctest/doctest)、
[Matplot++](https://github.com/alandefreitas/matplotplusplus)、
[Microsoft Proxy](https://github.com/microsoft/proxy)、
[nanobind](https://github.com/wjakob/nanobind)、
[OpenBLAS](https://github.com/OpenMathLib/OpenBLAS)、
[OSQP](https://osqp.org/)、
[pocketfft](https://github.com/mreineck/pocketfft)、
[qdldl](https://github.com/osqp/qdldl)、
[spdlog](https://github.com/gabime/spdlog)、
[stdexec](https://github.com/NVIDIA/stdexec)、
[Taskflow](https://github.com/taskflow/taskflow) 与
[zpp_bits](https://github.com/eyalz800/zpp_bits) 之上，感谢这些项目的作者。
