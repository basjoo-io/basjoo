execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_VARIABLE nanobind_ROOT
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
file(TO_CMAKE_PATH "${nanobind_ROOT}" nanobind_ROOT)

find_package(nanobind CONFIG REQUIRED)
