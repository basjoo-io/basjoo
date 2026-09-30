CPMAddPackage(
  NAME zpp_bits
  VERSION 4.7.6
  URL https://github.com/eyalz800/zpp_bits/archive/refs/tags/v4.7.6.tar.gz
  URL_HASH SHA256=536b40cf921e2a88739668785181f09de1fab4be1b3ca9172f2702fc3700abf9
  DOWNLOAD_ONLY True
  PATCHES
    ${CMAKE_CURRENT_SOURCE_DIR}/exclude_complex_from_member_autodetect.patch
)

add_library(zpp_bits INTERFACE)
target_sources(zpp_bits
  INTERFACE
    FILE_SET HEADERS
      TYPE HEADERS
      BASE_DIRS ${zpp_bits_SOURCE_DIR}
      FILES ${zpp_bits_SOURCE_DIR}/zpp_bits.h
)

target_include_directories(zpp_bits INTERFACE
  "$<BUILD_INTERFACE:${zpp_bits_SOURCE_DIR}>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

if(BASJOO_ENABLE_INSTALL)
  install(
    TARGETS
      zpp_bits
    EXPORT ${PROJECT_NAME}Targets
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
endif()
