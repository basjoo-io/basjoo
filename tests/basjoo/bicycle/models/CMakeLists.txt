basjoo_cxx_test(
  NAME
    test_bicycle_state
  SRCS
    "test_state.cpp"
  DEPS
    bicycle_state
)

basjoo_cxx_test(
  NAME
    test_bicycle_kinematics_model
  SRCS
    "test_kinematics_model.cpp"
  DEPS
    bicycle_kinematics_model
    math_vec2
)

basjoo_cxx_test(
  NAME
    test_bicycle_dynamics_model
  SRCS
    "test_dynamics_model.cpp"
  DEPS
    bicycle_dynamics_model
)
