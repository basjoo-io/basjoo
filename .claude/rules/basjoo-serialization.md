---
paths:
  - "src/**/*.hpp"
  - "tests/**/*.cpp"
---

# Serialization (zpp_bits)

Types opt into binary (de)serialization via **zpp_bits** (source-introduced through `deps/zpp_bits/`, not vcpkg).

- **Default — declarative:** `friend zpp::bits::access; using serialize = zpp::bits::members<N>;`. Types whose members are all public can drop the `friend`.
- **Asymmetric cases** (derived fields rebuilt by a factory on load, or deleted copy/move constructors): write an explicit hook.
  - All members public → a **free** `serialize(auto& archive, ...)` in the type's namespace (found via ADL).
  - Otherwise → `static constexpr auto serialize(auto& archive, auto& self) -> zpp::bits::errc { ... }`.
- Every serializable type gets a round-trip test (serialize → deserialize → equal, or `doctest::Approx` for floats).

## `pro::proxy<Facade>` wrappers

Reference shape: `src/basjoo/math/curves/curve_proxy.hpp`.

- Enable RTTI on the facade: `pro::skills::rtti<typename pro::facade_builder::...>::build` (the `typename` is required).
- Reach `proxy_typeid` / `proxy_cast` only through `*self` (the proxy holds its accessor by composition). `proxy_cast<T>(...)` returns `T*`, so pass `T`, not `T*`.
- Keep a closed, hand-written list of concrete types in the proxy's own header, tagged by `enum class ...Tag : std::uint8_t` (`UPPER_SNAKE_CASE` enumerators, no `k` prefix). **Declaration order is the wire format: append only, never reorder.**
- Write two ADL `serialize` overloads — `const&` (output only) and `&` (both directions via `if constexpr` on `archive_type::kind()`) — duplicating the output logic inline rather than sharing a helper.
- Round-trip tests also assert the reconstructed concrete type via `proxy_typeid`.
