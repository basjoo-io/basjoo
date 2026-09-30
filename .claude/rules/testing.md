---
paths:
  - "tests/**"
---

# Testing Conventions

Tests mirror the `src/basjoo/` layout under `tests/basjoo/`.

## C++ (doctest)

- Exactly one TU per test binary defines `DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN` (or `DOCTEST_CONFIG_IMPLEMENT` with a custom `main` that parses extra CLI flags) **before** including `doctest/doctest.h`.
- Include order: header under test → standard library → third-party → project headers → doctest last.
- Use `TEST_CASE_TEMPLATE` to cover `float`, `double` (and complex types) when the algorithm is generic; `SUBCASE` for variants of one scenario.
- Compare floating point with `doctest::Approx(x).epsilon(tol)` and an explicit tolerance.
- No order dependencies or mutable global state between tests.
- Register every binary in both build systems: `basjoo_cxx_test(...)` in `CMakeLists.txt` and a target using the `basjoo.cxx.test` rule in `xmake.lua`.
- Serializable types get a round-trip test (serialize → deserialize → equal / approximately equal).

## Python (pytest)

- Files `tests/basjoo/test_*.py`, functions `test_*`; Arrange / Act / Assert.
- `@pytest.fixture` for shared setup, `@pytest.mark.parametrize` for data-driven cases.

## Running

```bash
ctest --preset <preset>                 # all C++ tests
ctest --preset <preset> -R <regex>      # subset
xmake test                              # xmake equivalent
uv run pytest                           # Python
```
