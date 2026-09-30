basjoo_cxx_test(
  NAME
    test_math_curve_proxy
  SRCS
    "test_curve_proxy.cpp"
  DEPS
    math_curve_proxy
    math_piecewise_linear_curve
    math_piecewise_cubic_curve
    math_piecewise_quintic_curve
)

basjoo_cxx_test(
  NAME
    test_math_piecewise_linear_curve2
  SRCS
    "test_piecewise_linear_curve2.cpp"
  DEPS
    math_piecewise_linear_curve
)

basjoo_cxx_test(
  NAME
    test_math_piecewise_cubic_curve2
  SRCS
    "test_piecewise_cubic_curve2.cpp"
  DEPS
    math_piecewise_cubic_curve
)

basjoo_cxx_test(
  NAME
    test_math_piecewise_quintic_curve2
  SRCS
    "test_piecewise_quintic_curve2.cpp"
  DEPS
    math_piecewise_quintic_curve
)

basjoo_cxx_test(
  NAME
    test_math_piecewise_linear_curve3
  SRCS
    "test_piecewise_linear_curve3.cpp"
  DEPS
    math_piecewise_linear_curve
)

basjoo_cxx_test(
  NAME
    test_math_piecewise_cubic_curve3
  SRCS
    "test_piecewise_cubic_curve3.cpp"
  DEPS
    math_piecewise_cubic_curve
)

basjoo_cxx_test(
  NAME
    test_math_piecewise_quintic_curve3
  SRCS
    "test_piecewise_quintic_curve3.cpp"
  DEPS
    math_piecewise_quintic_curve
)
