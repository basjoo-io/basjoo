add_subdirectory(curves)
add_subdirectory(functions)
add_subdirectory(mdfunctions)
add_subdirectory(dense)
add_subdirectory(sparse)

basjoo_cxx_test(
  NAME
    test_math_fft
  SRCS
    "test_fft.cpp"
  DEPS
    math_fft
)

basjoo_cxx_test(
  NAME
    test_math_chebyshev
  SRCS
    "test_chebyshev.cpp"
  DEPS
    math_chebyshev
    math_utils
)

basjoo_cxx_test(
  NAME
    test_math_cubic_interpolation
  SRCS
    "test_cubic_interpolation.cpp"
  DEPS
    math_cubic_interpolation
)

basjoo_cxx_test(
  NAME
    test_math_quintic_interpolation
  SRCS
    "test_quintic_interpolation.cpp"
  DEPS
    math_quintic_interpolation
)

basjoo_cxx_test(
  NAME
    test_math_fixed
  SRCS
    "test_fixed.cpp"
  DEPS
    math_fixed
)

basjoo_cxx_test(
  NAME
    test_math_polygon
  SRCS
    "test_polygon.cpp"
  DEPS
    math_polygon
)
