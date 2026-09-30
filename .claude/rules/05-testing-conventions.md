# Testing Conventions (C++)

## Framework

- **doctest** 2.4.12

## Main macro

```cpp
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
```

Place the `#define` **before** the doctest include. **Exactly one** TU per test binary should provide this.

## Templates

Use **`TEST_CASE_TEMPLATE`** to cover `float`, `double`, and complex scalar types where algorithms are generic.

## Includes

1. Header under test (when applicable)
2. Project headers
3. doctest

## Assertions

- **`doctest::Approx`** for floating comparisons with explicit tolerances:

```cpp
REQUIRE(x == doctest::Approx(y).epsilon(1e-12));
```

## Serialization

For **zpp_bits**-serializable types, verify **serialize → deserialize → equal** to the original object (or approximate equality for floats). For `pro::proxy<Facade>` type-erasure results, also assert the reconstructed concrete type matches via `proxy_typeid`, not just value equality.

## Naming

- Test executables and `TEST_CASE` strings should read like specifications (`"LU decomposition recovers identity on well-conditioned matrix"`).

## Organization

- Mirror `src/basjoo/` layout under `tests/basjoo/`.
- Avoid global state and order dependencies between tests.
