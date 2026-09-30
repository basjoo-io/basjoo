basjoo_cxx_library(
  NAME
    bicycle_kinematics_model
  HDRS
    "kinematics_model.hpp"
  DEPS
    zpp_bits
    math_dense_arithmetic_base
    math_integrate_mode
)

basjoo_cxx_library(
  NAME
    bicycle_dynamics_model
  HDRS
    "dynamics_model.hpp"
)
