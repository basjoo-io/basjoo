include(${CMAKE_CURRENT_LIST_DIR}/../../vcpkg/scripts/buildsystems/vcpkg.cmake)

# set(CMAKE_SYSTEM_NAME Darwin CACHE STRING "System name is macOS")
# set(CMAKE_SYSTEM_PROCESSOR arm64 CACHE STRING "System processor is arm64")

set(CMAKE_AR gcc-ar CACHE FILEPATH "GNU AR")
set(CMAKE_RANLIB gcc-ranlib CACHE FILEPATH "GNU RANLIB")

set(CMAKE_C_COMPILER_LAUNCHER ccache CACHE FILEPATH "Enable ccache")
set(CMAKE_CXX_COMPILER_LAUNCHER ccache CACHE FILEPATH "Enable ccache")
set(CMAKE_Fortran_COMPILER_LAUNCHER ccache CACHE FILEPATH "Enable ccache")

set(CMAKE_C_COMPILER gcc CACHE FILEPATH "GNU C compiler")
set(CMAKE_CXX_COMPILER g++ CACHE FILEPATH "GNU C++ compiler")
set(CMAKE_Fortran_COMPILER gfortran CACHE FILEPATH "GNU Fortran compiler")

set(CMAKE_C_COMPILE_OPTIONS_IPO "-flto=auto;-fno-fat-lto-objects" CACHE STRING "C compiler IPO flags")
set(CMAKE_C_LINK_OPTIONS_IPO "-flto=auto;-flto-partition=none" CACHE STRING "C linker IPO flags")
set(CMAKE_CXX_COMPILE_OPTIONS_IPO "-flto=auto;-fno-fat-lto-objects" CACHE STRING "C++ compiler IPO flags")
set(CMAKE_CXX_LINK_OPTIONS_IPO "-flto=auto;-flto-partition=none" CACHE STRING "C++ linker IPO flags")
set(CMAKE_Fortran_COMPILE_OPTIONS_IPO "-flto=auto;-fno-fat-lto-objects" CACHE STRING "Fortran compiler IPO flags")
set(CMAKE_Fortran_LINK_OPTIONS_IPO "-flto=auto;-flto-partition=none" CACHE STRING "Fortran linker IPO flags")

set(CMAKE_C_FLAGS "-march=armv8-a -mtune=generic -pipe -fPIC -fexceptions -fno-omit-frame-pointer -mno-omit-leaf-frame-pointer -Wall -Wextra -Wpedantic -Wno-maybe-uninitialized -Wno-psabi" CACHE STRING "C compiler flags")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wp,-D_GLIBCXX_ASSERTIONS" CACHE STRING "C++ compiler flags")
set(CMAKE_Fortran_FLAGS "${CMAKE_C_FLAGS} -frecursive" CACHE STRING "Fortran compiler flags")

set(CMAKE_EXE_LINKER_FLAGS "-Wno-psabi" CACHE STRING "Executable linker flags")
set(CMAKE_SHARED_LINKER_FLAGS "-Wno-psabi" CACHE STRING "Shared linker flags")
set(CMAKE_MODULE_LINKER_FLAGS "-Wno-psabi" CACHE STRING "Module linker flags")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g" CACHE STRING "C compiler flags for debug mode")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}" CACHE STRING "C++ compiler flags for debug mode")
set(CMAKE_Fortran_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}" CACHE STRING "Fortran compiler flags for debug mode")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_DEBUG False CACHE BOOL "Disable interprocedural optimization for debug mode")

set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG -ftree-vectorize" CACHE STRING "C compiler flags for release mode")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}" CACHE STRING "C++ compiler flags for release mode")
set(CMAKE_Fortran_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}" CACHE STRING "Fortran compiler flags for release mode")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE True CACHE BOOL "Enable interprocedural optimization for release mode")

set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG -ftree-vectorize" CACHE STRING "C compiler flags for release with debug info mode")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}" CACHE STRING "C++ compiler flags for release with debug info mode")
set(CMAKE_Fortran_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}" CACHE STRING "Fortran compiler flags for release mode with debug info")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO False CACHE BOOL "Enable interprocedural optimization for release with debug info mode")

set(CMAKE_C_FLAGS_MINSIZEREL "-Os -DNDEBUG -ftree-vectorize" CACHE STRING "C compiler flags for minimum size release mode")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL}" CACHE STRING "C++ compiler flags for minimum size release mode")
set(CMAKE_Fortran_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL}" CACHE STRING "Fortran compiler flags for minimum size release mode")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL True CACHE BOOL "Enable interprocedural optimization for minimum size release mode")

set(CMAKE_CXX_SCAN_FOR_MODULES OFF CACHE BOOL "Disable scanning for modules")
