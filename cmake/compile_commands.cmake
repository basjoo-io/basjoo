file(COPY_FILE
  "${CMAKE_BINARY_DIR}/compile_commands.json"
  "${CMAKE_SOURCE_DIR}/compile_commands.json"
  RESULT RES
  ONLY_IF_DIFFERENT
  INPUT_MAY_BE_RECENT
)
