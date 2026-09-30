CPMAddPackage(
  NAME zpp_bits
  VERSION 4.7.7
  URL https://github.com/eyalz800/zpp_bits/archive/refs/tags/v4.7.7.tar.gz
  URL_HASH SHA256=cb813308e51ad1922c6578759202927c3efa4588227fd9f182934bf1f38ecd4c
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
