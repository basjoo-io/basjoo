add_subdirectory(problems)
add_subdirectory(solvers)

basjoo_cxx_library(
  NAME
    cvxopm_settings
  HDRS
    "settings.hpp"
  DEPS
    zpp_bits
    common_aligned_memory_resource
)

basjoo_cxx_library(
  NAME
    cvxopm_result
  HDRS
    "result.hpp"
  DEPS
    zpp_bits
)

basjoo_cxx_library(
  NAME
    cvxopm_info
  HDRS
    "info.hpp"
  DEPS
    zpp_bits
)
