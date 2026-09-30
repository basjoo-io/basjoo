---
applyTo: "**/test_*.cpp"
---
# C++ testing (doctest)

## Setup

Exactly one translation unit per test binary should contain:

```cpp
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
```

Place the `#define` **before** including doctest.

## Structure

1. Include the header under test first (when applicable).
2. Include other `basjoo/...` headers.
3. Include `doctest/doctest.h`.

Place tests in the same `namespace` as production code when practical.

## Templates

Use `TEST_CASE_TEMPLATE` to run the same assertions on `float`, `double`, and complex types as needed.

## Floating point

Use `doctest::Approx` with explicit tolerances:

```cpp
REQUIRE(value == doctest::Approx(expected).epsilon(1e-12));
```

## Serialization tests

For **zpp_bits**-serializable types, verify equality after a roundtrip through a stream or archive. For `pro::proxy<Facade>` type-erasure results, also assert the reconstructed concrete type matches via `proxy_typeid`, not just value equality.

## Naming

`TEST_CASE` names should read like specifications, not vague labels.
