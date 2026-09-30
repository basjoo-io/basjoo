# osqp defaults to building both osqpstatic and osqp (shared) regardless of
# BUILD_SHARED_LIBS. Mirror it to build exactly one variant.
if(BUILD_SHARED_LIBS)
  set(OSQP_BUILD_STATIC_LIB OFF CACHE BOOL "" FORCE)
  set(OSQP_BUILD_SHARED_LIB ON  CACHE BOOL "" FORCE)
else()
  set(OSQP_BUILD_STATIC_LIB ON  CACHE BOOL "" FORCE)
  set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
endif()

CPMAddPackage(
  NAME osqp
  VERSION 1.0.0
  URL https://github.com/osqp/osqp/releases/download/v1.0.0/osqp-v1.0.0-src.tar.gz
  URL_HASH SHA256=ec0bb8fd34625d0ea44274ab3e991aa56e3e360ba30935ae62476557b101c646
  OPTIONS
    "CMAKE_POLICY_VERSION_MINIMUM 3.10"
    "CMAKE_COMPILE_WARNING_AS_ERROR OFF"
    "OSQP_USE_LONG OFF"
    "OSQP_USE_FLOAT OFF"
    "OSQP_BUILD_DEMO_EXE OFF"
    "OSQP_ENABLE_PRINTING OFF"
    "OSQP_ENABLE_PROFILING OFF"
    "OSQP_ENABLE_INTERRUPT OFF"
)
