if(WIN32)
  set(Python_EXECUTABLE ${CMAKE_SOURCE_DIR}/.venv/Scripts/python.exe CACHE FILEPATH "Python executable")
else()
  set(Python_EXECUTABLE ${CMAKE_SOURCE_DIR}/.venv/bin/python CACHE FILEPATH "Python executable")
endif()

if(NOT Python_INCLUDE_DIRS)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_path('include'))"
    OUTPUT_VARIABLE Python_INCLUDE_DIRS
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()
set(Python_INCLUDE_DIRS "${Python_INCLUDE_DIRS}" CACHE INTERNAL "Python include directories")

find_package(Python 3.14 REQUIRED COMPONENTS Interpreter Development.Module GLOBAL)
