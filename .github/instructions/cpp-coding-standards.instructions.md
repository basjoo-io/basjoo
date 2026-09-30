---
applyTo: "**/*.cpp,**/*.hpp"
---
# C++ coding standards

## Naming

- **Namespaces**: `snake_case` (`basjoo::math`).
- **Classes / structs**: `PascalCase`.
- **Functions / methods**: `camelCase`.
- **Private members**: `m_` prefix + `snake_case`.
- **Constants**: `kPascalCase`.
- **Macros**: `BASJOO_UPPER_SNAKE`.
- **Enum classes**: `PascalCase`; enumerators `UPPER_SNAKE_CASE` (no `k` prefix).
- **Local type aliases** (inside a function body): `snake_case` (e.g. `archive_type`).

## Trailing return types

Use for new and updated functions:

```cpp
auto frobnicate(Vec3d x) noexcept -> double;
```

## Headers

- `#pragma once`.
- Include order: C++ standard library, third party, then `basjoo/...` headers.
- No `using namespace` in headers.

## Class design

- `final` for non-polymorphic types; `explicit` constructors unless implicit conversion is intended; `override` on overrides; virtual destructor when base is polymorphic.
- Use project macros (`ENABLE_COPY`, `DISABLE_MOVE`, …) from `basjoo/common/macros.hpp` when appropriate.

## GNU attributes

Apply on proven hot paths:

```cpp
[[using gnu: always_inline, leaf, hot]]
```

## Modern C++

- Concepts for template constraints; `if constexpr` where it simplifies generics.
- `std::span` / `std::string_view` for non-owning data.
- Exceptions for unrecoverable misuse; `BASJOO_CHECK_PARAMS` for optional validation.

## Serialization

Use **zpp_bits** patterns already present in the module — declarative `using serialize = zpp::bits::members<N>;` by default. For asymmetric cases (computed fields, deleted move/copy), write an explicit `serialize(auto& archive, ...)` hook: a free function (ADL, same namespace) if every member is public, otherwise a `static constexpr auto serialize(auto& archive, auto& self) -> zpp::bits::errc` member; add roundtrip tests.

For `pro::proxy<Facade>` type erasure: enable `pro::skills::rtti` on the facade; reach `proxy_typeid`/`proxy_cast` only via `*self`; `proxy_cast<T>` returns `T*` (pass the candidate type, not a pointer); tag known concrete types with a `PascalCase enum class ...Tag : std::uint8_t` (`UPPER_SNAKE_CASE` enumerators, append-only); write two ADL `serialize` overloads (`const&` output-only, `&` both directions) with the output branch duplicated rather than shared. See `src/basjoo/math/curves/curve_proxy.hpp`.

If a type stores a container whose element type is computed from a trait keyed on its own template parameter (`DenseGenerateTraitT<T>`, `DenseDegenerateTraitT<T>`, …), `#include` the full definition of every trait target reachable under the constraining concept — trait headers usually only forward-declare them.
