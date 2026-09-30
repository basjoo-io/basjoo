basjoo_cxx_test(
  NAME
    test_bicycle_feedforward_controller
  SRCS
    "test_feedforward_controller.cpp"
  DEPS
    bicycle_feedforward_controller
)

basjoo_cxx_test(
  NAME
    test_bicycle_pid_controller
  SRCS
    "test_pid_controller.cpp"
  DEPS
    bicycle_pid_controller
)

basjoo_cxx_test(
  NAME
    test_bicycle_model_predictive_controller
  SRCS
    "test_model_predictive_controller.cpp"
  DEPS
    bicycle_model_predictive_controller
)

basjoo_cxx_test(
  NAME
    test_bicycle_kinematics_flow_controller
  SRCS
    "test_kinematics_flow_controller.cpp"
  DEPS
    bicycle_kinematics_flow_controller
)
