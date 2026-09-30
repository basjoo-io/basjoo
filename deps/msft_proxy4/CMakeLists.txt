CPMAddPackage(
  NAME msft_proxy4
  VERSION 4.1.0
  URL https://github.com/ngcpp/proxy/releases/download/4.1.0/proxy-4.1.0.tgz
  URL_HASH SHA256=3b96e426df094fdc1da9b2bfb997c04574fe2ab2d022df834f2be2ab8339ddb5
  DOWNLOAD_ONLY True
)

file(GLOB_RECURSE _PROXY_HEADERS CONFIGURE_DEPENDS
  "${msft_proxy4_SOURCE_DIR}/proxy/*.h"
  "${msft_proxy4_SOURCE_DIR}/proxy/*.ixx"
)

add_library(msft_proxy4 INTERFACE)
target_sources(msft_proxy4
  INTERFACE
    FILE_SET HEADERS
      TYPE HEADERS
      BASE_DIRS ${msft_proxy4_SOURCE_DIR}
      FILES ${_PROXY_HEADERS}
)
target_include_directories(msft_proxy4 INTERFACE
  "$<BUILD_INTERFACE:${msft_proxy4_SOURCE_DIR}>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

if(BASJOO_ENABLE_INSTALL)
  install(
    TARGETS
      msft_proxy4
    EXPORT ${PROJECT_NAME}Targets
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
endif()
