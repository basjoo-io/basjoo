<h1 align="center">basjoo</h1>

<p align="center">
  <b>面向自动驾驶与机器人的基础数学库。</b>
</p>

<p align="center">
  <a href="README.md">English</a> •
  <a href="LICENSE">BSD-3-Clause 许可证</a> •
  <a href="https://github.com/basjoo-io/basjoo">GitHub</a>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/C%2B%2B-23-blue.svg" alt="C++ Standard">
  <img src="https://img.shields.io/badge/CMake-≥3.31-064F8C.svg?logo=cmake&logoColor=white" alt="CMake">
  <img src="https://img.shields.io/badge/license-BSD--3--Clause-blue.svg" alt="License">
  <img src="https://img.shields.io/badge/Platform-Linux%20|%20macOS%20|%20Windows-lightgrey.svg" alt="Platform">
</p>

---

## 简介

basjoo 是一个高性能 C++23 数学库，专为**自动驾驶**和**机器人**应用场景设计。在这些领域中，工程师经常需要：

- **基于样条的路径表示** — 将航点平滑插值为可行驶轨迹，并支持曲率、切线、弧长查询。
- **凸优化** — 实时求解二次规划问题，用于轨迹平滑、避障和控制分配。
- **运动规划模型** — 结合参考线、时间-弧长映射以及硬/软约束（边界、围栏），生成可行且舒适的运动轨迹。
- **高效线性代数** — 快速的向量/矩阵运算（可选 BLAS/LAPACK 和 AVX-512 SIMD 加速），作为上述所有功能的计算基础。

basjoo 以统一的、以头文件为主的库形式提供上述全部功能，采用现代 C++23 Concepts 和模板。每个组件都是细粒度的 CMake 目标（如 `basjoo::math_vec2`、`basjoo::cvxopm_osqp_solver`），可以按需链接。

## 模块概览

```
src/basjoo/
├── common/                         # basjoo::common — 工具库
│   ├── fsm.hpp
│   ├── aligned_allocator.hpp
│   ├── aligned_memory_resource.hpp
│   ├── chrono_inspector.hpp
│   ├── exec_on_exit.hpp
│   ├── in_in_in_out_result.hpp
│   ├── logging.hpp
│   └── macros.hpp
├── math/                           # basjoo::math — 数学基础
│   ├── concepts.hpp
│   ├── type_traits.hpp
│   ├── utils.hpp
│   ├── cubic_interpolation.hpp
│   ├── quintic_interpolation.hpp
│   ├── fft.hpp
│   ├── chebyshev.hpp
│   ├── fixed.hpp
│   ├── dense/                      #   稠密线性代数
│   │   ├── vec2.hpp
│   │   ├── vec3.hpp
│   │   ├── vector.hpp
│   │   ├── vectorx.hpp
│   │   ├── vector_view.hpp
│   │   ├── matrix.hpp
│   │   ├── matrixx.hpp
│   │   ├── matrix_view.hpp
│   │   ├── lu_dcmp.hpp
│   │   ├── qr_dcmp.hpp
│   │   ├── partial_pivot.hpp
│   │   ├── dense_traits.hpp
│   │   └── detail/
│   ├── sparse/                     #   稀疏矩阵
│   │   ├── dok_matrix.hpp
│   │   ├── lil_matrix.hpp
│   │   ├── coo_matrix.hpp
│   │   ├── csr_matrix.hpp
│   │   ├── csc_matrix.hpp
│   │   ├── sparse_matrix_proxy.hpp
│   │   ├── index_pair.hpp
│   │   ├── sparse_traits.hpp
│   │   └── qdldl_dcmp.hpp
│   ├── curves/                     #   分段曲线
│   │   ├── piecewise_linear_curve.hpp
│   │   ├── piecewise_cubic_curve.hpp
│   │   ├── piecewise_quintic_curve.hpp
│   │   ├── curve_proxy.hpp
│   │   ├── sl.hpp
│   │   └── slv.hpp
│   ├── functions/                  #   分段函数
│   │   ├── piecewise_linear_function.hpp
│   │   ├── piecewise_cubic_function.hpp
│   │   ├── piecewise_quintic_function.hpp
│   │   ├── function_proxy.hpp
│   │   └── boundary_mode.hpp
│   └── mdfunctions/                #   多维函数
│       ├── linear_mdfunction.hpp
│       ├── quadratic_mdfunction.hpp
│       ├── rosenbrock_function.hpp
│       └── mdfunction_proxy.hpp
├── cvxopm/                         # basjoo::cvxopm — 凸优化
│   ├── settings.hpp
│   ├── result.hpp
│   ├── info.hpp
│   ├── problems/
│   │   ├── osqp_problem.hpp
│   │   └── dense_problem.hpp
│   └── solvers/
│       ├── osqp_solver.hpp / .cpp
│       ├── lbfgs_solver.hpp
│       ├── bfgs_solver.hpp
│       ├── amoeba_solver.hpp
│       ├── lnsrch_solver.hpp
│       └── detail/
└── bicycle/                       # basjoo::bicycle — 运动规划
    └── planners/
        ├── route_line.hpp
        ├── path.hpp
        ├── motion.hpp
        ├── fence.hpp
        ├── border.hpp
        ├── dualism.hpp
        ├── route_line_cubic_acc_planner.hpp
        ├── route_line_quintic_acc_planner.hpp
        ├── route_line_cubic_offset_planner.hpp
        └── route_line_quintic_offset_planner.hpp
```

### `basjoo::math` — 数学基础

#### 稠密线性代数（`math/dense/`）

| 文件              | 目标                      | 说明                                                                                                                    |
| ----------------- | ------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| `vec2.hpp`        | `basjoo::math_vec2`        | 固定尺寸 2D 向量 `Vec2<T>`，提供 `euclidean()`、`radian()`、`rotate()`、`crossProj()`、`normalized()`，可选 AVX-512 SIMD |
| `vec3.hpp`        | `basjoo::math_vec3`        | 固定尺寸 3D 向量 `Vec3<T>`，提供 `cross()`、`crossProj()`，可选 AVX-512 SIMD                                            |
| `vector.hpp`      | `basjoo::math_vector`      | 固定尺寸稠密向量 `Vector<T, N>`，提供 `dot()`、`euclidean()`、`conjugated()`，可选 BLAS/LAPACK                          |
| `vectorx.hpp`     | `basjoo::math_vectorx`     | 动态尺寸稠密向量 `VectorX<T>`（堆分配，支持 PMR）                                                                       |
| `vector_view.hpp` | `basjoo::math_vector_view` | 非拥有视图 `VectorView<T>`，支持步长访问连续内存                                                                        |
| `matrix.hpp`      | `basjoo::math_matrix`      | 固定尺寸稠密矩阵 `Matrix<T, NRows, NCols>`，支持行优先/列优先布局，可选 BLAS/LAPACK                                     |
| `matrixx.hpp`     | `basjoo::math_matrixx`     | 动态尺寸稠密矩阵 `MatrixX<T>`（堆分配，支持 PMR）                                                                       |
| `matrix_view.hpp` | `basjoo::math_matrix_view` | 非拥有矩阵视图 `MatrixView<T>`                                                                                          |
| `lu_dcmp.hpp`     | `basjoo::math_lu_dcmp`     | LU 分解 `LuDcmp<T>`，带部分主元选取，用于求解线性方程组                                                                 |
| `qr_dcmp.hpp`     | `basjoo::math_qr_dcmp`     | QR 分解 `QrDcmp<T>`，用于最小二乘和线性方程组求解                                                                       |

#### 稀疏矩阵（`math/sparse/`）

| 文件                      | 目标                              | 说明                                                                                            |
| ------------------------- | --------------------------------- | ----------------------------------------------------------------------------------------------- |
| `dok_matrix.hpp`          | `basjoo::math_dok_matrix`          | 键值字典格式 `DokMatrix<T, I>`，适用于增量构造                                                  |
| `lil_matrix.hpp`          | `basjoo::math_lil_matrix`          | 行链表格式 `LilMatrix<T, I>`，适用于逐行构造                                                    |
| `coo_matrix.hpp`          | `basjoo::math_coo_matrix`          | 坐标格式 `CooMatrix<T, I>`（行/列/值三元组）                                                    |
| `csr_matrix.hpp`          | `basjoo::math_csr_matrix`          | 压缩稀疏行 `CsrMatrix<T, I>`，高效行访问                                                        |
| `csc_matrix.hpp`          | `basjoo::math_csc_matrix`          | 压缩稀疏列 `CscMatrix<T, I>`，高效列访问                                                        |
| `sparse_matrix_proxy.hpp` | `basjoo::math_sparse_matrix_proxy` | 稀疏矩阵的类型擦除代理（基于 Microsoft Proxy）                                                  |
| `qdldl_dcmp.hpp`          | `basjoo::math_qdldl_dcmp`          | 稀疏 LDL^T 分解 `QdldlDcmp<T>`（基于 qdldl），用于求解以上三角 CSC 格式存储的对称准定线性方程组 |

#### 分段曲线（`math/curves/`）

| 文件                          | 目标                                  | 说明                                                                                     |
| ----------------------------- | ------------------------------------- | ---------------------------------------------------------------------------------------- |
| `piecewise_linear_curve.hpp`  | `basjoo::math_piecewise_linear_curve`  | 通过 `Vec2`/`Vec3` 锚点的线性插值曲线                                                    |
| `piecewise_cubic_curve.hpp`   | `basjoo::math_piecewise_cubic_curve`   | 三次样条曲线，提供 `tangent()`、`normal()`、`curvature()`、`inverse()`，可自定义边界条件 |
| `piecewise_quintic_curve.hpp` | `basjoo::math_piecewise_quintic_curve` | 五次样条曲线，具有更高阶平滑性                                                           |
| `curve_proxy.hpp`             | `basjoo::math_curve_proxy`             | 曲线类型的类型擦除代理                                                                   |
| `sl.hpp` / `slv.hpp`          | （包含在曲线目标中）                  | 弧长-横向偏移坐标类型 `SlDuplet`、`SlvTriplet`                                           |

#### 分段函数（`math/functions/`）

| 文件                             | 目标                                     | 说明                                                              |
| -------------------------------- | ---------------------------------------- | ----------------------------------------------------------------- |
| `piecewise_linear_function.hpp`  | `basjoo::math_piecewise_linear_function`  | 分段线性标量/向量函数                                             |
| `piecewise_cubic_function.hpp`   | `basjoo::math_piecewise_cubic_function`   | 三次样条标量函数，提供 `derivative()`、`integral()`，支持边界条件 |
| `piecewise_quintic_function.hpp` | `basjoo::math_piecewise_quintic_function` | 五次样条标量函数，具有更高阶平滑性                                |
| `function_proxy.hpp`             | `basjoo::math_function_proxy`             | 函数类型的类型擦除代理                                            |
| `boundary_mode.hpp`              | （包含在三次/五次目标中）                | 边界条件定义 `BoundaryMode<T>`                                    |

#### 多维函数（`math/mdfunctions/`）

| 文件                       | 目标                               | 说明                                                  |
| -------------------------- | ---------------------------------- | ----------------------------------------------------- |
| `linear_mdfunction.hpp`    | `basjoo::math_linear_mdfunction`    | 线性多维函数 `LinearMdFunction<T>`                    |
| `quadratic_mdfunction.hpp` | `basjoo::math_quadratic_mdfunction` | 二次多维函数 `QuadraticMdFunction<T>`                 |
| `rosenbrock_function.hpp`  | `basjoo::math_rosenbrock_function`  | Rosenbrock 基准函数 `RosenbrockFunction<T>`           |
| `mdfunction_proxy.hpp`     | `basjoo::math_mdfunction_proxy`     | 多维函数的类型擦除代理，提供 `eval()` 和 `gradient()` |

#### 其他数学工具

| 文件                        | 目标                                | 说明                                                                                                                                                                                                                                          |
| --------------------------- | ----------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `concepts.hpp`              | `basjoo::math_concepts`              | C++23 Concepts：`ScalarArithmetic`、`VecArithmetic`、`MatArithmetic`、`Allocatory` 等                                                                                                                                                         |
| `type_traits.hpp`           | `basjoo::math_type_traits`           | 用于模板元编程的类型萃取                                                                                                                                                                                                                      |
| `utils.hpp`                 | `basjoo::math_utils`                 | 工具函数：`lerp()`、`linspace()`、`hasDuplicates()`、`nearestUpperElement()`                                                                                                                                                                  |
| `cubic_interpolation.hpp`   | `basjoo::math_cubic_interpolation`   | 三次插值算法                                                                                                                                                                                                                                  |
| `quintic_interpolation.hpp` | `basjoo::math_quintic_interpolation` | 五次插值算法                                                                                                                                                                                                                                  |
| `fft.hpp`                   | `basjoo::math_fft`                   | 基于 pocketfft 的 FFT（前向、后向、正交归一化）                                                                                                                                                                                               |
| `chebyshev.hpp`             | `basjoo::math_chebyshev`             | Chebyshev 多项式逼近 `Chebyshev<T>`                                                                                                                                                                                                           |
| `fixed.hpp`                 | `basjoo::math_fixed`                 | 定点数类型 `Fixed<ValueType, IntermediateType, FractionBits>`，提供完整算术运算符、数学函数（`sqrt`、`sin`、`cos`、`exp`、`log` 等）、`std::numeric_limits` 特化及 `std::formatter` 支持。预定义别名：`Fixed32f8`、`Fixed32f16`、`Fixed32f24` |

---

### `basjoo::cvxopm` — 凸优化

#### 问题表述（`cvxopm/problems/`）

| 文件                | 目标                          | 说明                                                                  |
| ------------------- | ----------------------------- | --------------------------------------------------------------------- |
| `osqp_problem.hpp`  | `basjoo::cvxopm_osqp_problem`  | 稀疏 QP 问题 `OsqpProblem<T, I>`：最小化 ½xᵀPx + qᵀx，约束 l ≤ Ax ≤ u |
| `dense_problem.hpp` | `basjoo::cvxopm_dense_problem` | 无约束稠密问题 `DenseProblem<T>`，封装 `MdFunctionProxy` 目标函数     |

#### 求解器（`cvxopm/solvers/`）

| 文件                | 目标                          | 说明                                                            |
| ------------------- | ----------------------------- | --------------------------------------------------------------- |
| `osqp_solver.hpp`   | `basjoo::cvxopm_osqp_solver`   | QP 求解器 `OsqpSolver<T, I>`，基于 OSQP 库，支持热启动和打磨    |
| `lbfgs_solver.hpp`  | `basjoo::cvxopm_lbfgs_solver`  | 有限内存 BFGS 求解器 `LbfgsSolver<T>`，用于无约束优化           |
| `bfgs_solver.hpp`   | `basjoo::cvxopm_bfgs_solver`   | BFGS 求解器 `BfgsSolver<T>`，用于无约束优化                     |
| `amoeba_solver.hpp` | `basjoo::cvxopm_amoeba_solver` | Nelder-Mead（单纯形法）求解器 `AmoebaSolver<T>`，用于无导数优化 |
| `lnsrch_solver.hpp` | `basjoo::cvxopm_lnsrch_solver` | 线搜索求解器 `LnsrchSolver<T>`                                  |

#### 辅助类型

| 文件           | 目标                     | 说明                                      |
| -------------- | ------------------------ | ----------------------------------------- |
| `settings.hpp` | `basjoo::cvxopm_settings` | 求解器设置 `Settings<T, I>`               |
| `result.hpp`   | `basjoo::cvxopm_result`   | 求解结果 `Result<T>`（原始/对偶变量）     |
| `info.hpp`     | `basjoo::cvxopm_info`     | 求解器信息 `Info<T, I>`（迭代次数、状态） |

---

### `basjoo::bicycle` — 运动规划

| 文件             | 目标                         | 说明                                                                        |
| ---------------- | ---------------------------- | --------------------------------------------------------------------------- |
| `route_line.hpp` | `basjoo::bicycle_route_line` | 参考线 `RouteLine<T>`，基于分段五次曲线，用于自动驾驶                       |
| `path.hpp`       | `basjoo::bicycle_path`       | 2D 路径 `Path<T>`，提供 `tangent()`、`normal()`、`curvature()`、`inverse()` |
| `motion.hpp`     | `basjoo::bicycle_motion`     | 时间-弧长映射 `Motion<T>`，提供 `velocity()`、`accel()`、`jerk()`           |
| `fence.hpp`      | `basjoo::bicycle_fence`      | 时间-空间约束 `HardFence<T>` / `SoftFence<T>`，用于纵向规划                 |
| `border.hpp`     | `basjoo::bicycle_border`     | 空间约束 `HardBorder<T>` / `SoftBorder<T>`，带手性，用于横向规划            |
| `dualism.hpp`    | `basjoo::bicycle_dualism`    | 枚举：`Chirality`（LEFT/RIGHT）、`Actio`（BLOCKING/YIELDING）               |

#### 运动规划器（`bicycle/planners/`）

| 文件                                     | 目标                                                | 说明                                                |
| ----------------------------------------- | ---------------------------------------------------- | --------------------------------------------------- |
| `route_line_cubic_acc_planner.hpp`      | `basjoo::bicycle_route_line_cubic_acc_planner`      | 三次加速度规划器 — 通过 QP 优化生成纵向运动           |
| `route_line_quintic_acc_planner.hpp`    | `basjoo::bicycle_route_line_quintic_acc_planner`    | 五次加速度规划器 — 通过 QP 优化生成更平滑的纵向运动   |
| `route_line_cubic_offset_planner.hpp`   | `basjoo::bicycle_route_line_cubic_offset_planner`   | 三次偏移规划器 — 通过 QP 优化生成横向路径偏移         |
| `route_line_quintic_offset_planner.hpp` | `basjoo::bicycle_route_line_quintic_offset_planner` | 五次偏移规划器 — 通过 QP 优化生成更平滑的横向路径偏移 |

---

### `basjoo::common` — 工具库

| 文件                          | 目标                                    | 说明                                                            |
| ----------------------------- | --------------------------------------- | --------------------------------------------------------------- |
| `fsm.hpp`                     | `basjoo::common_fsm`                     | 有限状态机，支持事件分发、状态栈和 `FSM_INITIAL_STATE` 宏       |
| `logging.hpp`                 | `basjoo::common_logging`                 | 基于 spdlog 的日志宏（`BASJOO_LOG_TRACE`、`BASJOO_LOG_DEBUG` 等） |
| `macros.hpp`                  | `basjoo::common_macros`                  | 拷贝/移动/单例模式的便捷宏                                      |
| `aligned_allocator.hpp`       | `basjoo::common_aligned_allocator`       | 32 字节对齐分配器 `AlignedAllocator<T>`                         |
| `aligned_memory_resource.hpp` | `basjoo::common_aligned_memory_resource` | PMR 对齐内存资源                                                |
| `exec_on_exit.hpp`            | `basjoo::common_exec_on_exit`            | RAII 清理处理器 `ExecOnExit`                                    |
| `chrono_inspector.hpp`        | `basjoo::common_chrono_inspector`        | 执行时间测量                                                    |

---

## 前置依赖

以下工具需要预先安装在系统中：

| 工具                                | 用途                       |
| ----------------------------------- | -------------------------- |
| **CMake**（≥ 3.31）                 | 构建系统生成器             |
| **Ninja**                           | 构建后端（所有预设均使用） |
| **GCC**（≥ 14）或 **Clang**（≥ 20） | 支持 C++23 的编译器        |
| **clangd**                          | 语言服务器，用于 IDE 集成  |
| **clang-tidy**                      | 静态分析                   |
| **clang-format**                    | 代码格式化                 |
| **ccache**                          | 编译缓存，加速重复构建     |

其余所有依赖（Boost、OSQP、spdlog、doctest 等）均由 [CPM.cmake](https://github.com/cpm-cmake/CPM.cmake) 自动管理，无需手动安装。

## 构建

```bash
# 配置（Linux + GCC）
cmake --preset linux-gcc-x64

# 编译
cmake --build --preset linux-gcc-x64

# 运行测试
ctest --preset linux-gcc-x64
```

使用 Clang：

```bash
cmake --preset linux-clang-x64
cmake --build --preset linux-clang-x64
ctest --preset linux-clang-x64
```

### CMake 选项

| 选项                        | 默认值     | 说明                                                   |
| --------------------------- | ---------- | ------------------------------------------------------ |
| `BASJOO_CHECK_PARAMS`        | `OFF`      | 启用运行时参数校验                                     |
| `BASJOO_BUILD_TESTING`       | `ON`       | 构建单元测试                                           |
| `BASJOO_ENABLE_INSTALL`      | `ON`       | 启用安装目标                                           |
| `BASJOO_USE_BLAS_LAPACK`     | `OpenBLAS` | BLAS/LAPACK 后端（`OpenBLAS`、`Netlib`、`MKL`、`OFF`） |
| `BASJOO_USE_BOOST_UNORDERED` | `ON`       | 使用 Boost.Unordered 容器                              |
| `BUILD_SHARED_LIBS`         | `OFF`      | 构建动态库                                             |

### 安装

```bash
cmake --install out/build/linux-gcc-x64 --prefix /your/install/path
```

安装后，在你自己的 CMake 项目中按需链接细粒度目标即可使用 basjoo：

```cmake
find_package(basjoo REQUIRED)
target_link_libraries(your_target
  PRIVATE
    basjoo::math_vec2
    basjoo::math_piecewise_cubic_curve
    basjoo::cvxopm_osqp_solver
    basjoo::bicycle_route_line
)
```

### Python 绑定

Python 包（nanobind `_core` 扩展）基于 `.venv/` 中的解释器构建。同步依赖前必须**以启用系统 site packages 的方式**创建 venv：devcontainer 中的 ROS pytest 插件（经 `PYTHONPATH` 自动加载）会导入 ROS deb 安装在 `/usr/lib/python3/dist-packages` 下的包，普通 venv 看不到它们。

```bash
uv venv --system-site-packages
uv sync --no-install-project
```

## 使用示例

### 2D 向量运算

```cpp
#include "basjoo/math/dense/vec2.hpp"

using namespace basjoo::math;

// 构造 2D 向量
Vec2d a(1.0, std::numbers::sqrt3);

// 基本查询
double len   = a.euclidean();             // 2.0
double radian = a.radian();               // π/3
Vec2d  unit  = a.normalized();            // (0.5, √3/2)

// 旋转
Vec2d rot = a.rotate(M_PI / 6.0);        // (0, 2)

// 叉积投影与点积
double cross = a.crossProj(Vec2d{0.0, 1.0});
double dot   = a.dot(Vec2d{1.0, 0.0});   // 1.0
```

### 固定尺寸与动态向量

```cpp
#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/dense/vectorx.hpp"

using namespace basjoo::math;

// 固定尺寸向量
Vector<double, 4> v;
v[0] = 1.0; v[1] = 2.0; v[2] = 3.0; v[3] = 4.0;

double norm = v.euclidean();
double d    = v.dot(v);
bool   same = v.identicalTo(v * 1.000000001);  // true（在容差范围内）
auto   conj = v.conjugated();

// 动态尺寸向量（支持 PMR）
VectorX<double> dyn(100);
```

### 三次样条曲线

```cpp
#include "basjoo/math/curves/piecewise_cubic_curve.hpp"
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/functions/boundary_mode.hpp"

using namespace basjoo::math;

// 在半圆上创建锚点（半径 = 2）
std::vector<Vec2d> anchors(21);
for (std::size_t i = 0; i < 21; ++i) {
    double theta = M_PI * i / 20.0;
    anchors[i] = Vec2d{2.0 * std::cos(theta), 2.0 * std::sin(theta)};
}

// 设置固定边界条件（一阶导数）
BoundaryMode<Vec2d> b0{.order = 1, .derivative = Vec2d{0.0, 1.0}};
BoundaryMode<Vec2d> bf{.order = 1, .derivative = Vec2d{0.0, -1.0}};

// 构造三次样条曲线
PiecewiseCubicCurve<Vec2d> curve{anchors, b0, bf};

// 在弧长 s = 1.0 处查询曲线
Vec2d  point   = curve(1.0);
Vec2d  tangent = curve.tangent(1.0);
Vec2d  normal  = curve.normal(1.0);
double kappa   = curve.curvature(1.0);    // ≈ 0.5（1/半径）

// 笛卡尔 → Frenet (s, l) 坐标转换
SlDupletd sl        = curve.inverse(Vec2d{1.0, 1.0});
Vec2d     recovered = curve(sl);           // (s, l) → 笛卡尔
```

### 三次样条函数

```cpp
#include "basjoo/math/functions/piecewise_cubic_function.hpp"
#include "basjoo/math/functions/boundary_mode.hpp"
#include "basjoo/math/utils.hpp"

using namespace basjoo::math;

// 为多项式生成采样点
std::vector<double> ts = linspace(-2.0, 2.0, 41);
std::vector<double> ys;
for (double t : ts) {
    ys.push_back(0.45 + 5.3*t - 1.3*t*t + 0.65*t*t*t);
}

// 边界条件：自然（二阶导数）
BoundaryMode<double> b0{.order = 2, .derivative = -2.6};
BoundaryMode<double> bf{.order = 2, .derivative =  1.3};

PiecewiseCubicFunction<double> func{ts, ys, b0, bf};

double val  = func(0.5);                  // f(0.5)
double dval = func.derivative(0.5);       // f'(0.5)
double area = func.integral(-1.0, 1.0);   // ∫₋₁¹ f(t) dt
```

### 二次规划（OSQP）

```cpp
#include "basjoo/cvxopm/problems/osqp_problem.hpp"
#include "basjoo/cvxopm/solvers/osqp_solver.hpp"
#include "basjoo/cvxopm/settings.hpp"

using namespace basjoo::cvxopm;

// 最小化：2x₀² + x₁² + x₀x₁ + x₀ + x₁
// 约束条件：x₀ + x₁ = 1，0 ≤ x₀ ≤ 0.7，0 ≤ x₁ ≤ 0.7
OsqpProblem<double, int> osqp_problem(2, 3);   // 2 个变量，3 个约束
osqp_problem.updateQuadCostTerm(0, 0, 2.0);
osqp_problem.updateQuadCostTerm(1, 1, 1.0);
osqp_problem.updateQuadCostTerm(0, 1, 1.0);
osqp_problem.updateLinCostTerm(0, 1.0);
osqp_problem.updateLinCostTerm(1, 1.0);
osqp_problem.updateConstraintTerm(0, {{0, 1.0}, {1, 1.0}}, 1.0, 1.0);
osqp_problem.updateConstraintTerm(1, {{0, 1.0}}, 0.0, 0.7);
osqp_problem.updateConstraintTerm(2, {{1, 1.0}}, 0.0, 0.7);

OsqpSolver<double, int> solver{Settings<double, int>{.polishing = true}};
auto [result, info] = solver.solve(osqp_problem);
// result.prim_vars[0] ≈ 0.3，result.prim_vars[1] ≈ 0.7

// 使用已知初始点热启动
std::vector<double> prim_vars_0{0.3, 0.7};
std::vector<double> dual_vars_0{-2.9, 0.0, 0.2};
auto [result2, info2] = solver.solve(osqp_problem, prim_vars_0, dual_vars_0);
```

### 无约束优化（L-BFGS）

```cpp
#include "basjoo/cvxopm/solvers/lbfgs_solver.hpp"
#include "basjoo/cvxopm/problems/dense_problem.hpp"
#include "basjoo/math/mdfunctions/rosenbrock_function.hpp"
#include "basjoo/math/dense/vectorx.hpp"

using namespace basjoo::cvxopm;
using namespace basjoo::math;

// 最小化 Rosenbrock 函数：f(x,y) = (a-x)² + b(y-x²)²
DenseProblem<double> problem(
    makeMdFunctionProxy(RosenbrockFunction<pmr::VectorX<double>>(1.0, 100.0))
);

LbfgsSolver<double> solver{.settings{.eps_abs{1E-6}}};
pmr::VectorX<double> x0({-1.0, 1.0});
auto [result, info] = solver.solve(problem, std::move(x0));
// result.prim_vars ≈ (1.0, 1.0)，约 40 次迭代内收敛
```

### 稀疏矩阵

```cpp
#include "basjoo/math/sparse/dok_matrix.hpp"
#include "basjoo/math/sparse/csr_matrix.hpp"

using namespace basjoo::math;

// 使用 DOK 格式增量构建稀疏矩阵
DokMatrix<double, int> dok(3, 3);
dok.updateCoeff(0, 0, 1.0);
dok.updateCoeff(1, 1, 2.0);
dok.updateCoeff(2, 2, 3.0);
dok.updateCoeff(0, 2, 0.5);

// 转换为 CSR 格式以实现高效行访问
CsrMatrix<double, int> csr(dok);
```

## 命名空间

| 命名空间              | 说明                                                  |
| --------------------- | ----------------------------------------------------- |
| `basjoo::math`         | 数学基础：向量、矩阵、曲线、函数、FFT、Chebyshev      |
| `basjoo::math::detail` | 内部实现细节                                          |
| `basjoo::cvxopm`       | 凸优化：QP 问题、OSQP / L-BFGS / BFGS / Amoeba 求解器 |
| `basjoo::bicycle`     | 运动规划：路径、参考线、运动、边界、围栏、规划模型    |
| `basjoo::common`       | 工具：FSM、日志、对齐内存、宏                         |

## 许可证

本项目使用 [BSD 3-Clause 许可证](LICENSE) 授权。

## 致谢

basjoo 基于以下优秀的开源项目构建，在此对它们的开发者和社区致以诚挚的感谢：

| 项目                                                          | 版本     | 说明                            | 主页                                             |
| ------------------------------------------------------------- | -------- | ------------------------------- | ------------------------------------------------ |
| [Boost](https://www.boost.org/)                               | 1.91.0   | 序列化、无序容器、哈希          | https://www.boost.org/                           |
| [CPM.cmake](https://github.com/cpm-cmake/CPM.cmake)           | 0.42.1   | CMake 包管理器                  | https://github.com/cpm-cmake/CPM.cmake           |
| [cxxopts](https://github.com/jarro2783/cxxopts)               | 3.3.1    | 命令行选项解析                  | https://github.com/jarro2783/cxxopts             |
| [doctest](https://github.com/doctest/doctest)                 | 2.5.2    | 快速 C++ 测试框架               | https://github.com/doctest/doctest               |
| [Matplot++](https://github.com/alandefreitas/matplotplusplus) | 1.2.2    | C++ 数据可视化图形库            | https://github.com/alandefreitas/matplotplusplus |
| [Microsoft Proxy](https://github.com/microsoft/proxy)         | 4.0.2    | 类型擦除多态库                  | https://github.com/microsoft/proxy               |
| [OpenBLAS](https://github.com/OpenMathLib/OpenBLAS)           | 0.3.33   | 优化的 BLAS/LAPACK 实现（可选） | https://github.com/OpenMathLib/OpenBLAS          |
| [OSQP](https://osqp.org/)                                     | 1.0.0    | 二次规划求解器                  | https://osqp.org/                                |
| [pocketfft](https://github.com/mreineck/pocketfft)            | cpp 分支 | 仅头文件的 FFT 库               | https://github.com/mreineck/pocketfft            |
| [qdldl](https://github.com/osqp/qdldl)                        | 0.1.9    | 稀疏 LDL 分解（OSQP 依赖）      | https://github.com/osqp/qdldl                    |
| [spdlog](https://github.com/gabime/spdlog)                    | 1.17.0   | 快速 C++ 日志库                 | https://github.com/gabime/spdlog                 |
