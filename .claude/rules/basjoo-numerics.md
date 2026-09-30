---
paths:
  - "src/**/*.hpp"
  - "src/**/*.cpp"
---

# Numerics and Performance

## Type alias suffixes

Vector/matrix aliases encode the scalar with a suffix, declared with `using` (never macros):

| Suffix | Scalar                 | Example          |
| ------ | ---------------------- | ---------------- |
| `s`    | `float`                | `Vec2s`, `Mat3s` |
| `d`    | `double`               | `Vec2d`, `Mat3d` |
| `c`    | `std::complex<float>`  | `Vec2c`          |
| `z`    | `std::complex<double>` | `Vec2z`          |

## Memory

- Stack storage for small fixed-size objects; document SIMD alignment with `alignas`.
- PMR variants (`basjoo::math::pmr`) for localized allocation arenas in hot code.

## Hot paths

- No virtual dispatch or `std::function` in tight numerical kernels unless a benchmark justifies it; prefer templates or function pointers.
- `[[using gnu: always_inline, leaf, hot]]` only after profiling.
- Validation and sizing at compile time (`constexpr`, `if constexpr`, concepts); cheap runtime parameter checks go through `BASJOO_CHECK_PARAMS`, not ad-hoc bounds tests.

## SIMD and BLAS/LAPACK

- Guard SIMD with feature macros (`__AVX512F__`, `__AVX512VL__`, `__AVX512DQ__`, `__AVX512BW__`, `__AVX512CD__`) and always keep a scalar fallback.
- Production linear algebra calls the BLAS/LAPACK backend selected by the `BASJOO_USE_BLAS_LAPACK` option: guard backend calls with `#ifdef BASJOO_USE_BLAS_LAPACK` and keep the native implementation in the `#else` branch (the option may be `OFF`). Link `blas_lapack` in both build systems.
