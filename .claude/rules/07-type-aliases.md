# Type Alias Suffixes

Vector, matrix, and scalar typedefs use a **suffix letter** to encode the underlying scalar field:

| Suffix | Scalar type | Example |
| ------ | ------------- | ------- |
| `s` | `float` | `Vec2s`, `Mat3s` |
| `d` | `double` | `Vec2d`, `Mat3d` |
| `c` | `std::complex<float>` | `Vec2c` |
| `z` | `std::complex<double>` | `Vec2z` |

When adding new aliases, keep naming **consistent** with existing types in the same module. Prefer **`using`** declarations over macros.
