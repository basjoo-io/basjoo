CPMAddPackage(
  NAME spdlog
  VERSION 1.17.0
  URL https://github.com/gabime/spdlog/archive/refs/tags/v1.17.0.tar.gz
  URL_HASH SHA256=d8862955c6d74e5846b3f580b1605d2428b11d97a410d86e2fb13e857cd3a744
  OPTIONS
    "SPDLOG_BUILD_SHARED ${BUILD_SHARED_LIBS}"
    "SPDLOG_INSTALL ON"
    "SPDLOG_USE_STD_FORMAT ON"
)
target_compile_definitions(spdlog INTERFACE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
target_compile_definitions(spdlog_header_only INTERFACE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
