include(${CMAKE_CURRENT_LIST_DIR}/../../vcpkg/scripts/buildsystems/vcpkg.cmake)

# set(CMAKE_SYSTEM_NAME Linux CACHE STRING "System name is Linux")
# set(CMAKE_SYSTEM_PROCESSOR x86_64 CACHE STRING "System processor is x86_64")

set(CMAKE_AR llvm-ar CACHE FILEPATH "LLVM AR")
set(CMAKE_RANLIB llvm-ranlib CACHE FILEPATH "LLVM RANLIB")

set(CMAKE_C_COMPILER_LAUNCHER ccache CACHE FILEPATH "Enable ccache")
set(CMAKE_CXX_COMPILER_LAUNCHER ccache CACHE FILEPATH "Enable ccache")
set(CMAKE_Fortran_COMPILER_LAUNCHER ccache CACHE FILEPATH "Enable ccache")

set(CMAKE_C_COMPILER clang CACHE FILEPATH "LLVM C compiler")
set(CMAKE_CXX_COMPILER clang++ CACHE FILEPATH "LLVM C++ compiler")
set(CMAKE_Fortran_COMPILER flang CACHE FILEPATH "LLVM Fortran compiler")

set(CMAKE_Fortran_COMPILER_WORKS True CACHE BOOL "Fortran compiler works")

set(CMAKE_LINKER_TYPE LLD CACHE STRING "LLVM Linker")

set(CMAKE_C_FLAGS "-march=x86-64-v3 -mtune=generic -pipe -fPIC -fexceptions -fno-omit-frame-pointer -mno-omit-leaf-frame-pointer -fno-plt -Wall -Wextra -Wpedantic -Wno-psabi" CACHE STRING "C compiler flags")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -stdlib=libc++" CACHE STRING "C++ compiler flags")
set(CMAKE_Fortran_FLAGS "-march=x86-64-v3 -mtune=generic -fPIC -fno-omit-frame-pointer -pedantic" CACHE STRING "Fortran compiler flags")

set(CMAKE_EXE_LINKER_FLAGS "-stdlib=libc++ -rtlib=compiler-rt --unwindlib=libunwind -lc++abi -Wno-psabi" CACHE STRING "Executable linker flags")
set(CMAKE_SHARED_LINKER_FLAGS "-stdlib=libc++ -rtlib=compiler-rt --unwindlib=libunwind -lc++abi -Wno-psabi" CACHE STRING "Shared linker flags")
set(CMAKE_MODULE_LINKER_FLAGS "-stdlib=libc++ -rtlib=compiler-rt --unwindlib=libunwind -lc++abi -Wno-psabi" CACHE STRING "Module linker flags")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g -fsanitize=address,undefined" CACHE STRING "C compiler flags for debug mode")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}" CACHE STRING "C++ compiler flags for debug mode")
set(CMAKE_Fortran_FLAGS_DEBUG "-O0 -g" CACHE STRING "Fortran compiler flags for debug mode")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_DEBUG False CACHE BOOL "Disable interprocedural optimization for debug mode")

set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG -Wp,-D_FORTIFY_SOURCE=3 -ftree-vectorize" CACHE STRING "C compiler flags for release mode")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}" CACHE STRING "C++ compiler flags for release mode")
set(CMAKE_Fortran_FLAGS_RELEASE "-O2 -DNDEBUG -ftree-vectorize" CACHE STRING "Fortran compiler flags for release mode")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE True CACHE BOOL "Enable interprocedural optimization for release mode")

set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG -Wp,-D_FORTIFY_SOURCE=3 -fsanitize=address,undefined -ftree-vectorize" CACHE STRING "C compiler flags for release with debug info mode")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}" CACHE STRING "C++ compiler flags for release with debug info mode")
set(CMAKE_Fortran_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG -ftree-vectorize" CACHE STRING "Fortran compiler flags for release mode with debug info")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO True CACHE BOOL "Enable interprocedural optimization for release with debug info mode")

set(CMAKE_C_FLAGS_MINSIZEREL "-Os -DNDEBUG -Wp,-D_FORTIFY_SOURCE=3 -ftree-vectorize" CACHE STRING "C compiler flags for minimum size release mode")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL}" CACHE STRING "C++ compiler flags for minimum size release mode")
set(CMAKE_Fortran_FLAGS_MINSIZEREL "-Os -DNDEBUG -ftree-vectorize" CACHE STRING "Fortran compiler flags for minimum size release mode")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL True CACHE BOOL "Enable interprocedural optimization for minimum size release mode")
