basjoo_cxx_library(
  NAME
    bicycle_feedforward_controller
  HDRS
    "feedforward_controller.hpp"
)

basjoo_cxx_library(
  NAME
    bicycle_pid_controller
  HDRS
    "pid_controller.hpp"
)

basjoo_cxx_library(
  NAME
    bicycle_model_predictive_controller
  HDRS
    "model_predictive_controller.hpp"
)

basjoo_cxx_library(
  NAME
    bicycle_kinematics_flow_controller
  HDRS
    "kinematics_flow_controller.hpp"
  DEPS
    zpp_bits
    bicycle_kinematics_model
    math_dense_arithmetic_base
    math_integrate_mode
)

basjoo_cxx_library(
  NAME
    bicycle_kinematics_adaptive_flow_controller
  HDRS
    "kinematics_adaptive_flow_controller.hpp"
  DEPS
    zpp_bits
    bicycle_kinematics_model
    math_dense_arithmetic_base
    math_integrate_mode
)
