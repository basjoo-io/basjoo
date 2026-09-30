add_subdirectory(curves)
add_subdirectory(functions)
add_subdirectory(mdfunctions)
add_subdirectory(dense)
add_subdirectory(sparse)

basjoo_cxx_library(
  NAME
    math_type_traits
  HDRS
    "type_traits.hpp"
)

basjoo_cxx_library(
  NAME
    math_concepts
  HDRS
    "concepts.hpp"
)

basjoo_cxx_library(
  NAME
    math_fft
  HDRS
    "fft.hpp"
  DEPS
    pocketfft
)

basjoo_cxx_library(
  NAME
    math_chebyshev
  HDRS
    "chebyshev.hpp"
  DEPS
    zpp_bits
)

basjoo_cxx_library(
  NAME
    math_utils
  HDRS
    "utils.hpp"
  DEPS
    math_concepts
)

basjoo_cxx_library(
  NAME
    math_cubic_interpolation
  HDRS
    "cubic_interpolation.hpp"
  DEPS
    math_concepts
    math_utils
)

basjoo_cxx_library(
  NAME
    math_quintic_interpolation
  HDRS
    "quintic_interpolation.hpp"
  DEPS
    math_concepts
    math_cubic_interpolation
)

basjoo_cxx_library(
  NAME
    math_fixed
  HDRS
    "fixed.hpp"
  DEPS
    zpp_bits
    "$<$<CXX_COMPILER_ID:MSVC>:Boost::multiprecision>"
)

basjoo_cxx_library(
  NAME
    math_polygon
  HDRS
    "polygon.hpp"
  DEPS
    zpp_bits
    common_aligned_allocator
    math_concepts
    math_utils
    math_piecewise_linear_curve
    math_vec2
)
