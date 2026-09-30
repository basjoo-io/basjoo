basjoo_cxx_test(
  NAME
    test_bicycle_route_line_cubic_acc_planner
  SRCS
    "test_route_line_cubic_acc_planner.cpp"
  DEPS
    bicycle_route_line_cubic_acc_planner
)

basjoo_cxx_test(
  NAME
    test_bicycle_route_line_quintic_acc_planner
  SRCS
    "test_route_line_quintic_acc_planner.cpp"
  DEPS
    bicycle_route_line_quintic_acc_planner
)

basjoo_cxx_test(
  NAME
    test_bicycle_route_line_cubic_offset_planner
  SRCS
    "test_route_line_cubic_offset_planner.cpp"
  DEPS
    bicycle_route_line_cubic_offset_planner
)

basjoo_cxx_test(
  NAME
    test_bicycle_route_line_quintic_offset_planner
  SRCS
    "test_route_line_quintic_offset_planner.cpp"
  DEPS
    bicycle_route_line_quintic_offset_planner
)

basjoo_cxx_test(
  NAME
    test_bicycle_route_line_kinematics_planner
  SRCS
    "test_route_line_kinematics_planner.cpp"
  DEPS
    bicycle_route_line_kinematics_planner
)
