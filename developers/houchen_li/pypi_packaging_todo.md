# basjoo PyPI 发布待办

日期：2026-09-30

## 背景

占位 wheel `basjoo-0.0.1a1-py3-none-any.whl` 已通过 scikit-build-core 后端构建并验证
（`uv build --wheel --config-setting 'wheel.cmake=false'`，跳过 CMake，纯元数据包），
先上传 TestPyPI 试水，再发布正式 PyPI。`basjoo` 在正式 PyPI 上尚未被占用。

以下为正式发布（含 `_core` 二进制扩展的版本）之前需要解决的问题。

## 待办

### 1. 多平台 wheel 构建

TODO(houchen_li): 正式版含 `_core` 二进制扩展，单个 `py3-none-any` wheel 不再适用，
需要 cibuildwheel 或 GitHub Actions 矩阵产出各平台 wheel：

- manylinux（x86_64，视需要加 aarch64 / musl）
- macOS（arm64 + x86_64）
- Windows（MSVC x64）

注意 Python 版本下限为 3.14；GitHub 仓库路径为 `basjoo-io/basjoo`。

### 2. sdist 精简

TODO(houchen_li): 当前 `uv build --sdist` 会把整个仓库打进 tar.gz（约 14k 个文件、
5.6 MB，包含 `vcpkg/` 子模块、`.claude/`、`.cursor/`、`developers/` 等）。正式发布前
配置 `[tool.scikit-build] sdist.exclude`（或最小化 include 列表），只保留从源码构建
wheel 所需的文件；xmake-python 后端的 sdist 内容同样需要检查。

### 3. `_core` 精准进入 wheel（CMake 组件机制）

TODO(houchen_li): `pyproject.cmake.toml` 已设置 `install.components = ["python"]`，
当前用于把 C++ install 树（`include/`、`lib/`、`share/`，约 6600 个文件，含 OpenBLAS
静态库）挡在 wheel 外。正式版需要把 `src/basjoo/CMakeLists.txt` 中的
`install(TARGETS _core ...)` 与 `install(FILES ...)` 加上 `COMPONENT python`，使
`_core.so` + `_core.pyi` + `py.typed` + `__init__.py` 精准进入 wheel，其余 install
内容继续被组件过滤隔离。完成后去掉占位构建所用的 `wheel.cmake=false`。

### 4. xmake-python 后端与 scikit-build-core 的产出对齐

TODO(houchen_li): xmake-python 后端无法跳过 C++ 构建（无 `wheel.cmake=false` 等价
开关；纯 wheel 模式要求所有 target 为 phony），产出必然是平台专属 tag 的 wheel。两个
后端已知差异，正式发布前需要决策/处理：

- xmake-python 打包时 `xmake install` 会安装**所有** target 的 `add_headerfiles`
  头文件（85 个，落 `.data/headers/`）与 static 产物（`libcvxopm_osqp_solver.a`，
  落 `.data/data/lib/`），占位 wheel 因此从 11 KB 膨胀到 947 KB。这些
  `add_headerfiles` 均无条件登记，如需排除要在库 target 的 xmake.lua 里挂
  `basjoo_enable_install` 条件或寻找全局机制。
- `[tool.xmake.xmaker] tempname = "out/build/xmake-python"` 在 xmake-python 0.1.7
  中**不生效**（`WheelBuilder.__init__` 无条件将 tempname 覆盖为 ephemeral 临时目录），
  该配置的注释已过时，每次打包都是全新目录、无增量缓存。

## 实验记录

2026-09-30，两个后端各构建占位 wheel 一次，均通过 `twine check` 并在隔离环境完成
安装验证：

- scikit-build-core：`uv build --wheel --config-setting 'wheel.cmake=false'` →
  `basjoo-0.0.1a1-py3-none-any.whl`（11 KB，纯元数据占位包，推荐上传 TestPyPI 的版本）。
- xmake-python：`cp pyproject.xmake.toml pyproject.toml && uv build --wheel` →
  `basjoo-0.0.1a1-cp314-cp314-linux_x86_64.whl`（947 KB，`basjoo/` 内含
  `__init__.py`、`_core.*.so`、`_core.pyi`、`py.typed`，`import basjoo._core` 可用；
  其余为 `.data/` 下的 C++ 头文件与静态库）。
- 实验过程中修复了两处 xmake 侧问题（已入库）：
  1. 全部 26 个 `xmake.lua` 的 `add_includedirs("$(projectdir)/src")` 改为
     scriptdir 相对路径——xmake-python 通过临时工程根 `includes()` 本项目时，
     `$(projectdir)` 指向临时目录而非本仓库，导致所有 include basjoo 头的编译失败。
  2. `src/basjoo/xmake.lua` 的 `_core` target 增加
     `set_prefixdir("$(prefixdir)/$(pythondir)", { libdir = "basjoo" })`（按
     xmake-python 官方 pybind 示例的姿势），使 `.so` 落入 wheel 的 `basjoo/`
     platlib 而非 `.data/data/lib/`。
