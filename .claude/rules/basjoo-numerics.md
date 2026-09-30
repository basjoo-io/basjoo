---
paths:
  - "src/**/*.hpp"
  - "src/**/*.cpp"
---

# Numerics and Performance

## Type alias suffixes

Vector/matrix aliases encode the scalar with a suffix, declared with `using` (never macros):

| Suffix | Scalar                 | Example           |
| ------ | ---------------------- | ----------------- |
| `s`    | `float`                | `Vec2s`, `Mat33s` |
| `d`    | `double`               | `Vec2d`, `Mat33d` |
| `c`    | `std::complex<float>`  | `Vec2c`           |
| `z`    | `std::complex<double>` | `Vec2z`           |

## Memory

- Stack storage for small fixed-size objects; document SIMD alignment with `alignas`.
- PMR variants (`basjoo::math::pmr`) for localized allocation arenas in hot code.

## Hot paths

- No virtual dispatch or `std::function` in tight numerical kernels unless a benchmark justifies it; prefer templates or function pointers.
- `[[using gnu: always_inline, leaf, hot]]` only after profiling.
- Validation and sizing at compile time (`constexpr`, `if constexpr`, concepts); cheap runtime parameter checks go through `BASJOO_CHECK_PARAMS`, not ad-hoc bounds tests.

## SIMD and BLAS/LAPACK

- Guard SIMD with feature macros (`__SSE2__`/`_M_X64`, `__SSE3__`, `__SSE4_1__`, `__AVX__`, `__AVX2__`, `__AVX512F__`, `__AVX512VL__`, `__AVX512DQ__`, `__ARM_NEON`, …) and always keep a scalar fallback. MSVC defines no SSE3/SSE4.1 macros, so pair them with `__AVX__` (`__SSE3__ || __AVX__`). Put intrinsics under `if !consteval`: constant evaluation runs the scalar code. Declare the pointers and registers of a branch inside it, so that a build without that instruction set has no unused variables; declarations that several branches share stay before them, marked `[[maybe_unused]]`, so that a target with no SIMD branch at all (riscv64, for one) builds warning-free.
- OpenBLAS is always built and `blas_lapack` linked in both build systems; `BASJOO_USE_BLAS_LAPACK` only names the backend (`Netlib` and `MKL` fail at configure time until implemented). Whether code calls BLAS or LAPACK is the `bool Blas = false` template parameter of `MatrixBase` and `VectorBase` (`euclidean`, `euclideanSqr`, `dot`, `orthogonalTo`) and the `bool Lapack = false` one of `LuDcmp` and `QrDcmp`, tested with `if constexpr`. The backend runs only outside constant evaluation and only for `float`, `double` and their complex types; everything else falls through to plain loops, written once after the backend branch, with no SIMD. `Matrix` and `Vector` derive the `false` bases: switching BLAS on means editing their class heads.
- `Vec2`, `Vec3`, `Mat22`, `Mat33` and `Quaternion` never call BLAS: their norms, inner products and products are SIMD intrinsics written inline in each member, overriding the base members of the same name.
- `math/dense/dense_lanes.hpp` holds element-wise kernels only (`DenseArithmeticBase`, conjugation, equality); reductions and products do not belong there. The kernels take any alignment; heap storage defaults to `basjoo::common::kSimdAlignment`, so pass an alignment only where no default exists.
- Low-level math code (the dense kernels and their SIMD branches) uses no lambdas and no single-use helper functions: expand the code at each use, however long it gets. A function that outgrows `.clang-tidy`'s size or cognitive-complexity thresholds is wrapped in `NOLINTBEGIN`/`NOLINTEND` for exactly the checks it exceeds.
