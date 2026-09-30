# Performance

## Memory

- Prefer **stack** storage for small fixed-size objects; document alignment requirements with **`alignas`** when SIMD is involved.
- Use **`std::span`** instead of pointer + length; avoid storing **`span`** or **`string_view`** in objects unless lifetimes are explicit.
- Use **PMR** (`basjoo::math::pmr`) for localized allocation arenas in hot code when the project provides them.

## Compile-time work

- Move validation and sizing to **`constexpr`** / **`if constexpr`** where possible.
- Use **concepts** to fail fast at compile time instead of runtime checks in templated APIs.

## Hot paths

- Avoid **virtual** dispatch in tight numerical kernels unless benchmarks justify it.
- Avoid **`std::function`** in hot loops; use templates, function pointers, or lightweight functors.
- Mark functions **`noexcept`** when failure is not modeled.

## SIMD

- Guard SIMD code with compiler feature macros (e.g. `__AVX512F__`, `__AVX512VL__`, `__AVX512DQ__`, `__AVX512BW__`, `__AVX512CD__`); provide scalar fallbacks.

## BLAS / LAPACK

- Call the configured **BLAS/LAPACK** implementation; do not replace with naive triple loops for production paths.

## Macros

- Prefer **`BASJOO_CHECK_PARAMS`** for inexpensive defensive checks over duplicating manual bounds tests everywhere.
