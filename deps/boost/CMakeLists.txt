CPMAddPackage(
  NAME Boost
  VERSION 1.92.0
  URL https://github.com/boostorg/boost/releases/download/boost-1.92.0/boost-1.92.0-cmake.tar.xz
  URL_HASH SHA256=9bed76128d4e46755dbe818487788c6fceb6f72b378f4daa49b7e1e600d9088d
  OPTIONS
    "CMAKE_COMPILE_WARNING_AS_ERROR OFF"
    "CMAKE_UNITY_BUILD OFF"
    "BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS}"
    "BOOST_RUNTIME_LINK $<IF:$<BOOL:${BUILD_SHARED_LIBS}>,shared,static>"
    "BOOST_ENABLE_CMAKE ON"
    "BOOST_SKIP_INSTALL_RULES OFF"
    "BOOST_INCLUDE_LIBRARIES headers\\\;unordered\\\;asio\\\;multiprecision\\\;system"
)
