basjoo_cxx_library(
  NAME
    bicycle_state
  HDRS
    "state.hpp"
  DEPS
    zpp_bits
)

basjoo_cxx_library(
  NAME
    bicycle_kinematics_model
  HDRS
    "kinematics_model.hpp"
  DEPS
    zpp_bits
    bicycle_state
)

basjoo_cxx_library(
  NAME
    bicycle_dynamics_model
  HDRS
    "dynamics_model.hpp"
  DEPS
    bicycle_state
)
