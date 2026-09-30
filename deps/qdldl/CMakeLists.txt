# On MSVC, qdldlstatic has OUTPUT_NAME "qdldl", so its .lib and the shared
# import library would both be named qdldl.lib — ninja rejects duplicate rules.
# Mirror BUILD_SHARED_LIBS to build exactly one variant.
if(BUILD_SHARED_LIBS)
  set(QDLDL_BUILD_STATIC_LIB OFF CACHE BOOL "" FORCE)
  set(QDLDL_BUILD_SHARED_LIB ON  CACHE BOOL "" FORCE)
else()
  set(QDLDL_BUILD_STATIC_LIB ON  CACHE BOOL "" FORCE)
  set(QDLDL_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
endif()

CPMAddPackage(
  NAME qdldl
  VERSION 0.1.9
  URL https://github.com/osqp/qdldl/archive/refs/tags/v0.1.9.tar.gz
  URL_HASH SHA256=7d1285b2db15cf2730dc83b3d16ed28412f558591108cca4f28d4438bf72ceed
  OPTIONS
    "CMAKE_POLICY_DEFAULT_CMP0069 NEW"
    "CMAKE_COMPILE_WARNING_AS_ERROR OFF"
    "QDLDL_LONG OFF"
    "QDLDL_FLOAT OFF"
    "QDLDL_BUILD_DEMO_EXE OFF"
    "QDLDL_UNITTESTS OFF"
)
