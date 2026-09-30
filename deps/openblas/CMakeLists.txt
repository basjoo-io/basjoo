CPMAddPackage(
  NAME OpenBLAS
  VERSION 0.3.34
  URL https://github.com/OpenMathLib/OpenBLAS/releases/download/v0.3.34/OpenBLAS-0.3.34.tar.gz
  URL_HASH SHA256=cd7e129868320cc2d033afa920e31202dfe0b8066a5b66661900ccc0f197dfed
  DOWNLOAD_ONLY True
)
if (WIN32)
  include(${CMAKE_CURRENT_SOURCE_DIR}/windows.cmake)
elseif(APPLE)
  include(${CMAKE_CURRENT_SOURCE_DIR}/darwin.cmake)
else()
  include(${CMAKE_CURRENT_SOURCE_DIR}/linux.cmake)
endif()
