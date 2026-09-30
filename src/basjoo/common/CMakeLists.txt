basjoo_cxx_library(
  NAME
    common_fsm
  HDRS
    "fsm.hpp"
)

basjoo_cxx_library(
  NAME
    common_macros
  HDRS
    "macros.hpp"
)

basjoo_cxx_library(
  NAME
    common_logging
  HDRS
    "logging.hpp"
  DEPS
    spdlog::spdlog_header_only
)

basjoo_cxx_library(
  NAME
    common_exec_on_exit
  HDRS
    "exec_on_exit.hpp"
)

basjoo_cxx_library(
  NAME
    common_chrono_inspector
  HDRS
    "chrono_inspector.hpp"
  DEPS
    common_logging
)

basjoo_cxx_library(
  NAME
    common_aligned_allocator
  HDRS
    "aligned_allocator.hpp"
)

basjoo_cxx_library(
  NAME
    common_aligned_memory_resource
  HDRS
    "aligned_memory_resource.hpp"
  DEPS
    zpp_bits
)

basjoo_cxx_library(
  NAME
    common_in_in_in_out_result
  HDRS
    "in_in_in_out_result.hpp"
)
