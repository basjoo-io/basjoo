add_subdirectory(bicycle)
add_subdirectory(common)
add_subdirectory(cvxopm)
add_subdirectory(math)

nanobind_add_module(_core
  STABLE_ABI
  NB_STATIC
  _core.cpp
)
set_property(TARGET _core PROPERTY FOLDER ${BASJOO_IDE_FOLDER}/python)
set_target_properties(_core PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_compile_options(_core PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-fno-sanitize=address,undefined>)
target_link_options(_core PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-fno-sanitize=address,undefined>)
target_compile_options(nanobind-static PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-fno-sanitize=address,undefined>)
target_link_options(nanobind-static PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-fno-sanitize=address,undefined>)

nanobind_add_stub(_core_stub
  MODULE _core
  OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/_core.pyi"
  PYTHON_PATH "${CMAKE_CURRENT_SOURCE_DIR}"
  DEPENDS _core
)

file(TOUCH "${CMAKE_CURRENT_SOURCE_DIR}/py.typed")

if(BASJOO_ENABLE_INSTALL)
  install(TARGETS _core LIBRARY DESTINATION basjoo)
  install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/_core.pyi"
    "${CMAKE_CURRENT_SOURCE_DIR}/py.typed"
    "${CMAKE_CURRENT_SOURCE_DIR}/__init__.py"
    DESTINATION basjoo
  )
endif()
