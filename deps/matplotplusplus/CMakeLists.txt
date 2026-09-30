CPMAddPackage(
  NAME matplotplusplus
  VERSION 1.2.2
  URL https://github.com/alandefreitas/matplotplusplus/archive/refs/tags/v1.2.2.tar.gz
  URL_HASH SHA256=c7434b4fea0d0cc3508fd7104fafbb2fa7c824b1d2ccc51c52eaee26fc55a9a0
  PATCHES
    ${CMAKE_CURRENT_SOURCE_DIR}/disable_warning_as_error.patch
    ${CMAKE_CURRENT_SOURCE_DIR}/deprecate_is_trivial.patch
    ${CMAKE_CURRENT_SOURCE_DIR}/lapack_fallback_to_blas.patch
  OPTIONS
    "CMAKE_COMPILE_WARNING_AS_ERROR OFF"
    "MATPLOTPP_BUILD_WITH_PEDANTIC_WARNINGS OFF"
    "MATPLOTPP_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS}"
    "MATPLOTPP_BUILD_TESTS OFF"
    "MATPLOTPP_BUILD_EXAMPLES OFF"
)
