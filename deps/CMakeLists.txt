include(ExternalProject)

find_package(Threads REQUIRED MODULE GLOBAL)
find_package(OpenMP REQUIRED MODULE GLOBAL)

include(python/CMakeLists.txt)
include(nanobind/CMakeLists.txt)

add_subdirectory(cpm)

add_subdirectory(cxxopts)
add_subdirectory(doctest)
add_subdirectory(msft_proxy4)
add_subdirectory(pocketfft)
add_subdirectory(spdlog)
add_subdirectory(zpp_bits)

add_subdirectory(qdldl)
add_subdirectory(osqp)

add_subdirectory(boost)
add_subdirectory(taskflow)
add_subdirectory(stdexec)

add_library(blas_lapack INTERFACE)
if(BASJOO_USE_BLAS_LAPACK STREQUAL "OpenBLAS")
  add_subdirectory(openblas)
endif()
add_subdirectory(matplotplusplus)

if(BASJOO_ENABLE_INSTALL)
  install(
    TARGETS blas_lapack
    EXPORT ${CMAKE_PROJECT_NAME}Targets
  )
endif()
